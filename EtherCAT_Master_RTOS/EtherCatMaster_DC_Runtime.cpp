#include "EtherCatMaster.h"
#include "EtherCatMaster_DC_Internal.h"
#include "EtherCatMaster_DC_Topology.h"
#include "EtherCatMaster_DC_Tuning.h"
#include "EtherCatPdoRuntimeInvalidCorrelation.h"
#include "EtherCatPdoSafetyStopDebounce.h"
#include "EtherCatRxForensics.h"
#include "EtherCatP64DeferredDiagnostics.h"
#include "AlarmManager.h"
#include <windows.h> 
#include <rtapi.h> 
#include <rtssapi.h> 
#include <array>
#include <atomic>
#include <cstring>
#include <stdio.h>
#include "GlobalConfig.h"
#include "PLCManager.h" // 🌟 1. 記得 include PLCManager 標頭檔
#define MAX_MBX_SIZE 1024

#if defined(_MSC_VER)
#define OSCARMAX_P64_DIAG_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define OSCARMAX_P64_DIAG_NOINLINE __attribute__((noinline))
#else
#define OSCARMAX_P64_DIAG_NOINLINE
#endif

// ============================================================================
// EtherCatMaster_DC_Runtime.cpp
// EtherCAT DC 即時循環正式版候選 RC1.8（冷／溫機 Drift 自動捕獲版）
//
// 本檔責任：
//   1. 執行 4 kHz／250 us PDO 即時循環。
//   2. 用 LRW + FRMW 在同一個 Ethernet frame 交換 PDO，並擷取所選 Reference DC 時間。
//   3. 維護 QPC <-> DC Reference 對映、漂移觀測器、Real FF 與 Phase-P 控制器。
//   4. LRW Process Data 有效時更新 PLC/Motion；DC sample 另由獨立品質閘門控制。
//   5. 只把統計結果發布到 snapshot，實際文字輸出交給 Priority 50 主執行緒。
//
// 執行緒與資料所有權：
//   - GlobalTimerHandler_PDO 由 Priority 64 的 RTX64 timer callback 執行。
//   - 此 Handler 是實體 EtherCAT IO Map 的唯一擁有者。
//   - 主執行緒只能透過 sequence snapshot 讀取診斷資料，不可直接改寫中間狀態。
//   - sequence 採 seqlock 慣例：奇數表示寫入中，偶數表示完整快照可讀。
//
// 目前真正會影響 PDO 時序的功能：
//   - PDO One-Shot Scheduler：啟用。
//   - Real FF V0：啟用，會更新排程週期使用的 drift ppb。
//   - Phase-P V0：啟用，會把相位 offset 套到下一次 one-shot final target。
//
// 目前只觀測、不直接控制硬體的功能：
//   - Robust Drift、Trusted Drift、Live-FF shadow、Residual V1/V1A、Frequency FF V2。
//   - QPC Coarse Re-Anchor dry-run 與 Fine Scheduler timing foundation。
//
// 目前明確停用：
//   - Fine Wait（避免 Priority 64 內 busy wait）。
//   - RTX64 HAL Frequency Burst Actuator（不修改 HAL period counts）。
//   - 額外逐站讀取 0x092C/0x0928 的 DC 診斷 frame。
//
// 重要基準：
//   - PDO 週期：250000 ns（4 kHz）。
//   - Sync0 目標相位：125000 ns。
//   - AUTO 啟動校正前 Bootstrap：-9500 ppb；校正後採本次量測 Baseline。
//   - One-Shot coarse guard：100000 ns；Fine Wait 仍為 OFF。
//   - AUTO 開機捕獲與 Real FF 安全範圍：-16000..-5000 ppb。
//   - Real FF 每個觀測窗最多變更 10 ppb。
//   - Phase-P：P=1/8、deadband=500 ns、每次最多 250 ns、總 offset ±120000 ns。
//
// 即時路徑禁止事項：
//   - 不新增動態配置、檔案 I/O、鎖等待、Sleep 或無界迴圈。
//   - 不在正常 4 kHz 路徑新增 RtPrintf；若需文字診斷，先發布 snapshot。
//   - 不可任意更動「FlushOutputs -> EtherCAT -> FetchInputs -> DC -> Motion」順序。
//   - 修改控制參數時一次只改一項，並保留原值與長時間測試 LOG。
//
// 可調參數索引（依風險分級）：
//
// [A：可先調，但一次只改一個]
//   PHASE_P_ACT_DIVISOR = 8
//     - P-only 強度；數字變小＝修正變強，數字變大＝修正變慢。
//     - 建議只用 8 -> 10 或 8 -> 6 的小步測試，不要直接減半。
//   PHASE_P_ACT_DEADBAND_NS = 500 ns
//     - |WrappedErr| 小於此值不修正；調大可減少抖動，調小可讓誤差更貼近 0。
//     - 若 Step 正負頻繁切換，先試 750 或 1000 ns；若長期停在 500~1000 ns，
//       才考慮降到 250 ns。
//   PHASE_P_ACT_MAX_STEP_NS = 250 ns／觀測窗
//     - 單次 offset 最大改變量；調大收斂較快但相位跳動較大，調小則較平滑。
//     - 建議每次以 50 ns 為一級調整。
//   REAL_FF_V0_MAX_STEP_PPB = 10 ppb／觀測窗
//     - Real FF 接近建議頻率的速度；調大追蹤較快，也更容易追到短期雜訊。
//     - 長期 ActualErr 單方向緩慢漂移、且 PhaseGood 穩定時才考慮 10 -> 15/20。
//
// [B：安全邊界，只在 A 類無法解決時調]
//   PHASE_P_ACT_MAX_OFFSET_NS = 120000 ns
//     - Phase-P 可累積的總 offset；若 OffsetSat:YES 才有理由檢討此值。
//     - 不能超過半個 250 us 週期的 125000 ns；目前 120000 ns 已接近上限，
//       正式機不建議再增大。若希望更保守可降到 100000 或 50000 ns。
//   REAL_FF_V0_MIN_PPB / MAX_PPB = -16000 / -5000 ppb
//     - Real FF 絕對限幅；ClampActive:YES 表示 observer 建議超出此安全範圍。
//     - 不要為了消除 ClampActive 就直接放寬，應先檢查 V1A slope/MAD 與 RX timeout。
//   *_ARM_WINDOWS、*_HOLD_RECOVERY_WINDOWS、*_HOLD_BAD_LIMIT
//     - 只改變進入 ACTIVE／恢復／Trip 的速度，不改變 P 增益。
//     - 增大＝較保守但恢復慢；減小＝反應快但較容易受單窗雜訊影響。
//
// [C：排程安全參數，正式版先不要動]
//   PDO_ONESHOT_COARSE_GUARD_NS = 100000 ns
//     - callback 相對 final target 提前醒來的時間；Fine Wait OFF 時也會改變送出相位。
//   PDO_MIN_REARM_LEAD_NS = 50000 ns
//     - re-arm 的最低安全 lead；過小可能來不及，過大可能增加 Recovery/Skip。
//   PDO_RUNTIME_RECOVERY_TARGET_LEAD_NS = 200000 ns
//     - scheduler 落後時跳過週期後要恢復的目標 lead。
//   PDO_BOOTSTRAP_DELAY_NS = 250000 ns
//     - one-shot fallback 週期，必須和 4 kHz PDO 週期一致。
//   PHASE_P_ACT_CYCLE_NS = 250000 ns、QPC_LIVE_FF_SYNC0_PHASE_NS = 125000 ns
//     - 系統週期與 Sync0 相位基準；除非 ENI／整個同步設計一起變更，否則不要調。
//
// 外部但重要的測試設定：
//   RTX64 HAL = 25 us；NAL Interrupt priority = 70；TX complete priority = 70。
//   這三項會影響 wake／RX 最大延遲，正式測試期間應固定，不要和程式參數同時改。
// ============================================================================

// ============================================================================
// Stage 12B.1B - Owner-Safe RT Process Image Shadow Snapshot
// ============================================================================
//
// Purpose:
//     Capture one coherent Logical Process Image snapshot for the Windows
//     commissioning/diagnosis path without letting a lower-priority thread
//     touch EtherCatMaster::m_IoMap directly.
//
// Ownership:
//     Writer  : Priority-64 PDO owner thread only.
//     Reader  : Priority-50 supervisory publisher through the bounded
//               OSCARMAX_ECAT_DiagRtShadow_Read() copy API below.
//
// Cadence:
//     One capture attempt every 40 PDO cycles.
//     At 250 us/cycle this is 10 ms (100 Hz).
//
// Real-time rules:
//     - fixed-size static storage only;
//     - no allocation / lock / sleep / file I/O / printf;
//     - one bounded memcpy of the active Process Image only;
//     - invalid PDO cycles do NOT overwrite the last known-good image;
//     - odd/even Sequence protects the lower-priority reader from torn data.
//
// Stage boundary:
//     This stage does NOT write OSCARMAX_ECAT_DIAG Shared Memory.
//     Stage 12B.1C will copy this RT shadow into that Shared Memory from
//     Priority 50, keeping the hard real-time PDO owner free of UI/SHM work.
// ============================================================================

namespace
{
    constexpr uint32_t OSCARMAX_ECAT_DIAG_RT_PROCESS_IMAGE_CAPACITY = 4096u;
    constexpr uint64_t OSCARMAX_ECAT_DIAG_RT_CAPTURE_DIVISOR = 40ULL;

    struct OSCARMAX_ECAT_DiagRtShadow
    {
        // First field intentionally naturally aligned for InterlockedIncrement.
        volatile LONG Sequence = 0;

        uint32_t ProcessImageBytes = 0;
        uint32_t ProcessImageValid = 0;

        int32_t ActualLrwWkc = 0;
        int32_t DcWkc = 0;

        uint64_t SourcePdoTick = 0;
        uint64_t LastValidPdoTick = 0;

        uint64_t CaptureAttempts = 0;
        uint64_t ValidSnapshots = 0;
        uint64_t InvalidSkips = 0;

        uint64_t LastCostNs = 0;
        uint64_t MaxCostNs = 0;
        uint64_t TotalCostNs = 0;

        uint8_t ProcessImage[OSCARMAX_ECAT_DIAG_RT_PROCESS_IMAGE_CAPACITY] = {};
    };

    OSCARMAX_ECAT_DiagRtShadow g_ecatDiagRtShadow;

    void CaptureEtherCatDiagRtShadow(
        EtherCatMaster* pMaster,
        int currentLrwWkc,
        int currentDcWkc,
        bool processDataValid)
    {
        if (pMaster == nullptr)
        {
            return;
        }

        // 250 us * 40 = 10 ms. This branch is false for 97.5% of PDO cycles.
        if ((pMaster->tickCount_PDO % OSCARMAX_ECAT_DIAG_RT_CAPTURE_DIVISOR) != 0ULL)
        {
            return;
        }

        const uint64_t costStartNs =
            pMaster->GetCurrentMasterTimeNs();

        uint32_t imageBytes = 0u;

        if (pMaster->m_IoMapSize > 0)
        {
            imageBytes =
                static_cast<uint32_t>(pMaster->m_IoMapSize);

            if (imageBytes > OSCARMAX_ECAT_DIAG_RT_PROCESS_IMAGE_CAPACITY)
            {
                imageBytes = OSCARMAX_ECAT_DIAG_RT_PROCESS_IMAGE_CAPACITY;
            }
        }

        // Begin seqlock write. Odd Sequence means writer active.
        InterlockedIncrement(&g_ecatDiagRtShadow.Sequence);
        MemoryBarrier();

        g_ecatDiagRtShadow.ProcessImageBytes = imageBytes;
        g_ecatDiagRtShadow.ProcessImageValid = processDataValid ? 1u : 0u;
        g_ecatDiagRtShadow.ActualLrwWkc = static_cast<int32_t>(currentLrwWkc);
        g_ecatDiagRtShadow.DcWkc = static_cast<int32_t>(currentDcWkc);
        g_ecatDiagRtShadow.SourcePdoTick = pMaster->tickCount_PDO;
        g_ecatDiagRtShadow.CaptureAttempts++;

        if (processDataValid &&
            pMaster->m_IoMap != nullptr &&
            imageBytes > 0u)
        {
            std::memcpy(
                g_ecatDiagRtShadow.ProcessImage,
                pMaster->m_IoMap,
                imageBytes);

            g_ecatDiagRtShadow.LastValidPdoTick = pMaster->tickCount_PDO;
            g_ecatDiagRtShadow.ValidSnapshots++;
        }
        else
        {
            // Keep the previous known-good bytes. The validity flag lets the
            // Windows side distinguish "last good image" from current health.
            g_ecatDiagRtShadow.InvalidSkips++;
        }

        const uint64_t costEndNs =
            pMaster->GetCurrentMasterTimeNs();

        uint64_t costNs = 0u;

        if (costEndNs >= costStartNs)
        {
            costNs = costEndNs - costStartNs;
        }

        g_ecatDiagRtShadow.LastCostNs = costNs;

        if (costNs > g_ecatDiagRtShadow.MaxCostNs)
        {
            g_ecatDiagRtShadow.MaxCostNs = costNs;
        }

        // Saturating accumulation keeps long-running diagnostic statistics
        // well-defined even if a machine remains online for a very long time.
        if (0xFFFFFFFFFFFFFFFFULL - g_ecatDiagRtShadow.TotalCostNs >= costNs)
        {
            g_ecatDiagRtShadow.TotalCostNs += costNs;
        }
        else
        {
            g_ecatDiagRtShadow.TotalCostNs = 0xFFFFFFFFFFFFFFFFULL;
        }

        MemoryBarrier();
        InterlockedIncrement(&g_ecatDiagRtShadow.Sequence);
        // Even Sequence means one complete snapshot is available.
    }
}

// ============================================================================
// DC-DIAG.2 - Priority-64 numeric diagnostic handoff
// ============================================================================
//
// The fixed PDO callback never formats text. Its single writer publishes one
// small latest-event snapshot; Priority 50 notices EventSerial changes and
// performs the optional RtPrintf work. Multiple events before one P50 read may
// collapse to the newest event, and the serial gap makes that condition visible.
// ============================================================================
namespace
{
    struct EtherCatP64DeferredDiagSlot
    {
        volatile LONG Sequence = 0;
        EtherCatP64DeferredDiagSnapshot Snapshot{};
    };

    static_assert(
        sizeof(EtherCatP64DeferredDiagSnapshot) <= 96u,
        "Priority-64 deferred diagnostic snapshot must remain small.");

    EtherCatP64DeferredDiagSlot g_p64DeferredDiagSlot{};
    uint64_t g_p64DeferredDiagNextSerial = 0ULL;

    OSCARMAX_P64_DIAG_NOINLINE void PublishP64DeferredDiagnostic(
        EtherCatP64DeferredDiagKind kind,
        uint64_t pdoTick,
        int64_t value0,
        int64_t value1,
        int64_t value2,
        int64_t value3,
        int64_t value4,
        int64_t value5,
        int64_t value6,
        int64_t value7) noexcept
    {
        EtherCatP64DeferredDiagSnapshot& snapshot =
            g_p64DeferredDiagSlot.Snapshot;

        InterlockedIncrement(&g_p64DeferredDiagSlot.Sequence);
        MemoryBarrier();

        snapshot.EventSerial = ++g_p64DeferredDiagNextSerial;
        snapshot.PdoTick = pdoTick;
        snapshot.Kind = static_cast<uint32_t>(kind);
        snapshot.Reserved = 0u;
        snapshot.Value0 = value0;
        snapshot.Value1 = value1;
        snapshot.Value2 = value2;
        snapshot.Value3 = value3;
        snapshot.Value4 = value4;
        snapshot.Value5 = value5;
        snapshot.Value6 = value6;
        snapshot.Value7 = value7;

        MemoryBarrier();
        InterlockedIncrement(&g_p64DeferredDiagSlot.Sequence);
    }
}

OSCARMAX_P64_DIAG_NOINLINE bool TryReadEtherCatP64DeferredDiagnostic(
    EtherCatP64DeferredDiagSnapshot& snapshot) noexcept
{
    const LONG sequenceBefore =
        g_p64DeferredDiagSlot.Sequence;

    if (sequenceBefore == 0L ||
        (sequenceBefore & 1L) != 0L)
    {
        return false;
    }

    MemoryBarrier();
    snapshot = g_p64DeferredDiagSlot.Snapshot;
    MemoryBarrier();

    const LONG sequenceAfter =
        g_p64DeferredDiagSlot.Sequence;

    return
        sequenceBefore == sequenceAfter &&
        (sequenceAfter & 1L) == 0L;
}

// ============================================================================
// NC-0.2K.7.2.1 - PDO Runtime Invalid Source Correlation Diagnostic
// ============================================================================
//
// This publisher is deliberately independent from the Motion settle bank.
// The Priority-64 owner observes the already-computed PDO validity contract;
// Priority-50 can only read the published atomic words.  The data is never
// consumed by EtherCAT, Motion, RESET, Registry, or read-ahead decisions.
//
// Healthy cycles execute the pure tracker plus one predictable false branch.
// Atomic publication happens only for an invalid edge/checkpoint, recovery,
// true runtime tick discontinuity, or contract event/checkpoint.
// ============================================================================

namespace
{
    using PdoInvalidCorrelationSnapshot =
        EtherCatPdoRuntimeInvalidCorrelationSnapshot;

    constexpr std::size_t PDO_INVALID_CORRELATION_WORD_COUNT =
        (sizeof(PdoInvalidCorrelationSnapshot) + sizeof(std::uint64_t) - 1U) /
        sizeof(std::uint64_t);

    static_assert(
        PDO_INVALID_CORRELATION_WORD_COUNT <= 32U,
        "PDO invalid diagnostic publication must remain small and bounded.");
    static_assert(
        ATOMIC_LLONG_LOCK_FREE == 2,
        "Priority-64 diagnostic publication requires lock-free 64-bit atomics.");

    struct PdoInvalidCorrelationAtomicBank
    {
        std::atomic<std::uint64_t> sequence{ 0ULL };
        std::array<
            std::atomic<std::uint64_t>,
            PDO_INVALID_CORRELATION_WORD_COUNT> words{};

        PdoInvalidCorrelationAtomicBank() noexcept
        {
            for (auto& word : words)
            {
                word.store(0ULL, std::memory_order_relaxed);
            }
        }
    };

    PdoInvalidCorrelationAtomicBank g_pdoInvalidCorrelationBank;
    EtherCatPdoRuntimeInvalidCorrelationTracker
        g_pdoInvalidCorrelationTracker;

    using PdoSafetyStopCauseSnapshot =
        EtherCatPdoSafetyStopCauseSnapshot;

    constexpr std::size_t PDO_SAFETY_STOP_CAUSE_WORD_COUNT =
        (sizeof(PdoSafetyStopCauseSnapshot) + sizeof(std::uint64_t) - 1U) /
        sizeof(std::uint64_t);

    static_assert(
        PDO_SAFETY_STOP_CAUSE_WORD_COUNT <= 16U,
        "PDO safety stop cause publication must remain small and bounded.");

    struct PdoSafetyStopCauseAtomicBank
    {
        std::atomic<std::uint64_t> sequence{ 0ULL };
        std::array<
            std::atomic<std::uint64_t>,
            PDO_SAFETY_STOP_CAUSE_WORD_COUNT> words{};

        PdoSafetyStopCauseAtomicBank() noexcept
        {
            for (auto& word : words)
            {
                word.store(0ULL, std::memory_order_relaxed);
            }
        }
    };

    PdoSafetyStopCauseAtomicBank g_pdoSafetyStopCauseBank;
    EtherCatPdoSafetyStopAlarmBridgeTracker
        g_pdoSafetyStopAlarmBridgeTracker;

    void PublishPdoInvalidCorrelationSnapshot(
        const PdoInvalidCorrelationSnapshot& snapshot) noexcept
    {
        std::array<
            std::uint64_t,
            PDO_INVALID_CORRELATION_WORD_COUNT> packed{};
        std::memcpy(packed.data(), &snapshot, sizeof(snapshot));

        const std::uint64_t writingSequence =
            g_pdoInvalidCorrelationBank.sequence.fetch_add(
                1ULL,
                std::memory_order_acq_rel) + 1ULL;

        for (std::size_t index = 0U;
            index < PDO_INVALID_CORRELATION_WORD_COUNT;
            ++index)
        {
            g_pdoInvalidCorrelationBank.words[index].store(
                packed[index],
                std::memory_order_relaxed);
        }

        g_pdoInvalidCorrelationBank.sequence.store(
            writingSequence + 1ULL,
            std::memory_order_release);
    }

    void PublishPdoSafetyStopCauseSnapshot(
        const PdoSafetyStopCauseSnapshot& snapshot) noexcept
    {
        std::array<
            std::uint64_t,
            PDO_SAFETY_STOP_CAUSE_WORD_COUNT> packed{};
        std::memcpy(packed.data(), &snapshot, sizeof(snapshot));

        const std::uint64_t writingSequence =
            g_pdoSafetyStopCauseBank.sequence.fetch_add(
                1ULL,
                std::memory_order_acq_rel) + 1ULL;

        for (std::size_t index = 0U;
            index < PDO_SAFETY_STOP_CAUSE_WORD_COUNT;
            ++index)
        {
            g_pdoSafetyStopCauseBank.words[index].store(
                packed[index],
                std::memory_order_relaxed);
        }

        g_pdoSafetyStopCauseBank.sequence.store(
            writingSequence + 1ULL,
            std::memory_order_release);
    }

    void ObservePdoRuntimeInvalidCorrelation(
        std::uint64_t runtimeCycleTick,
        bool processDataValid,
        std::int32_t actualLrwWkc,
        std::int32_t expectedLrwWkc,
        std::int32_t dcWkc,
        bool dcReferenceRequired,
        std::uint64_t combinedPathNs,
        std::uint64_t runtimePdoNegativeTotal,
        std::uint64_t runtimePdoWkcErrorTotal,
        std::uint32_t subTick) noexcept
    {
        if (!g_pdoInvalidCorrelationTracker.Observe(
            runtimeCycleTick,
            processDataValid,
            actualLrwWkc,
            expectedLrwWkc,
            dcWkc,
            dcReferenceRequired,
            combinedPathNs,
            runtimePdoNegativeTotal,
            runtimePdoWkcErrorTotal,
            subTick))
        {
            return;
        }

        PublishPdoInvalidCorrelationSnapshot(
            g_pdoInvalidCorrelationTracker.Snapshot());
    }
}

bool TryReadEtherCatPdoRuntimeInvalidCorrelation(
    EtherCatPdoRuntimeInvalidCorrelationSnapshot& snapshot) noexcept
{
    constexpr std::uint32_t MAX_READ_ATTEMPTS = 4U;

    for (std::uint32_t attempt = 0U;
        attempt < MAX_READ_ATTEMPTS;
        ++attempt)
    {
        const std::uint64_t sequenceBefore =
            g_pdoInvalidCorrelationBank.sequence.load(
                std::memory_order_acquire);
        if ((sequenceBefore & 1ULL) != 0ULL)
        {
            continue;
        }

        std::array<
            std::uint64_t,
            PDO_INVALID_CORRELATION_WORD_COUNT> packed{};
        for (std::size_t index = 0U;
            index < PDO_INVALID_CORRELATION_WORD_COUNT;
            ++index)
        {
            packed[index] =
                g_pdoInvalidCorrelationBank.words[index].load(
                    std::memory_order_relaxed);
        }

        std::atomic_thread_fence(std::memory_order_acquire);
        const std::uint64_t sequenceAfter =
            g_pdoInvalidCorrelationBank.sequence.load(
                std::memory_order_acquire);
        if (sequenceBefore == sequenceAfter &&
            (sequenceAfter & 1ULL) == 0ULL)
        {
            std::memcpy(&snapshot, packed.data(), sizeof(snapshot));
            return true;
        }
    }

    return false;
}

bool TryReadEtherCatPdoSafetyStopCause(
    EtherCatPdoSafetyStopCauseSnapshot& snapshot) noexcept
{
    constexpr std::uint32_t MAX_READ_ATTEMPTS = 4U;

    for (std::uint32_t attempt = 0U;
        attempt < MAX_READ_ATTEMPTS;
        ++attempt)
    {
        const std::uint64_t sequenceBefore =
            g_pdoSafetyStopCauseBank.sequence.load(
                std::memory_order_acquire);
        if ((sequenceBefore & 1ULL) != 0ULL)
        {
            continue;
        }

        std::array<
            std::uint64_t,
            PDO_SAFETY_STOP_CAUSE_WORD_COUNT> packed{};
        for (std::size_t index = 0U;
            index < PDO_SAFETY_STOP_CAUSE_WORD_COUNT;
            ++index)
        {
            packed[index] =
                g_pdoSafetyStopCauseBank.words[index].load(
                    std::memory_order_relaxed);
        }

        std::atomic_thread_fence(std::memory_order_acquire);
        const std::uint64_t sequenceAfter =
            g_pdoSafetyStopCauseBank.sequence.load(
                std::memory_order_acquire);
        if (sequenceBefore == sequenceAfter &&
            (sequenceAfter & 1ULL) == 0ULL)
        {
            std::memcpy(&snapshot, packed.data(), sizeof(snapshot));
            return true;
        }
    }

    return false;
}

// ---------------------------------------------------------------------------
// Stage 12B.1B reader seam for Priority-50 publisher.
//
// No retry loop is used here. If the 100 Hz writer happens to overlap this
// copy, the reader returns false and simply tries again on its next 10 ms
// supervisory iteration. This keeps behavior bounded and lock-free.
// ---------------------------------------------------------------------------
extern "C" bool OSCARMAX_ECAT_DiagRtShadow_Read(
    uint8_t * destination,
    uint32_t destinationCapacity,
    uint32_t * processImageBytes,
    uint32_t * processImageValid,
    int32_t * actualLrwWkc,
    int32_t * dcWkc,
    uint64_t * sourcePdoTick,
    uint64_t * lastValidPdoTick,
    uint64_t * captureAttempts,
    uint64_t * validSnapshots,
    uint64_t * invalidSkips,
    uint64_t * lastCostNs,
    uint64_t * maxCostNs,
    uint64_t * averageCostNs)
{
    if (destination == nullptr ||
        processImageBytes == nullptr ||
        processImageValid == nullptr ||
        actualLrwWkc == nullptr ||
        dcWkc == nullptr ||
        sourcePdoTick == nullptr ||
        lastValidPdoTick == nullptr ||
        captureAttempts == nullptr ||
        validSnapshots == nullptr ||
        invalidSkips == nullptr ||
        lastCostNs == nullptr ||
        maxCostNs == nullptr ||
        averageCostNs == nullptr)
    {
        return false;
    }

    const LONG sequenceBegin =
        g_ecatDiagRtShadow.Sequence;

    if ((sequenceBegin & 1L) != 0L)
    {
        return false;
    }

    MemoryBarrier();

    const uint32_t bytes =
        g_ecatDiagRtShadow.ProcessImageBytes;

    if (bytes > destinationCapacity ||
        bytes > OSCARMAX_ECAT_DIAG_RT_PROCESS_IMAGE_CAPACITY)
    {
        return false;
    }

    const uint32_t valid =
        g_ecatDiagRtShadow.ProcessImageValid;

    const int32_t lrw =
        g_ecatDiagRtShadow.ActualLrwWkc;

    const int32_t dc =
        g_ecatDiagRtShadow.DcWkc;

    const uint64_t tick =
        g_ecatDiagRtShadow.SourcePdoTick;

    const uint64_t validTick =
        g_ecatDiagRtShadow.LastValidPdoTick;

    const uint64_t attempts =
        g_ecatDiagRtShadow.CaptureAttempts;

    const uint64_t snapshots =
        g_ecatDiagRtShadow.ValidSnapshots;

    const uint64_t skips =
        g_ecatDiagRtShadow.InvalidSkips;

    const uint64_t lastNs =
        g_ecatDiagRtShadow.LastCostNs;

    const uint64_t maxNs =
        g_ecatDiagRtShadow.MaxCostNs;

    const uint64_t totalNs =
        g_ecatDiagRtShadow.TotalCostNs;

    if (bytes > 0u)
    {
        std::memcpy(
            destination,
            g_ecatDiagRtShadow.ProcessImage,
            bytes);
    }

    MemoryBarrier();

    const LONG sequenceEnd =
        g_ecatDiagRtShadow.Sequence;

    if (sequenceBegin != sequenceEnd ||
        (sequenceEnd & 1L) != 0L)
    {
        return false;
    }

    *processImageBytes = bytes;
    *processImageValid = valid;
    *actualLrwWkc = lrw;
    *dcWkc = dc;
    *sourcePdoTick = tick;
    *lastValidPdoTick = validTick;
    *captureAttempts = attempts;
    *validSnapshots = snapshots;
    *invalidSkips = skips;
    *lastCostNs = lastNs;
    *maxCostNs = maxNs;
    *averageCostNs =
        (attempts > 0u)
        ? (totalNs / attempts)
        : 0u;

    return true;
}


// ============================================================================
// Stage 12F.3E.1 - Runtime-Safe Non-Blocking CoE / SDO Mailbox State Machine
// Stage 12F.3E.5 - Mailbox counter sequence fix: numbered requests use 1..7 only.
// Stage 12F.3E.11 - Fixed-address mailbox transport + response-match correction
//                    + explicit SM0-consumed / SM1-response timeout classifier.
// ============================================================================
//
// Why this exists:
//     The legacy ecx_SDOread()/ecx_SDOwrite() helpers are synchronous mailbox
//     routines.  They poll repeatedly and call RtSleepFt(), which is acceptable
//     during startup/configuration but is NOT acceptable inside the 250 us
//     Priority-64 PDO owner thread.
//
// Runtime policy:
//     - The PDO owner remains the ONLY thread that touches the EtherCAT NIC.
//     - Online SDO is split across many PDO cycles as a small state machine.
//     - One mailbox transport step is attempted at most once per 1 ms
//       (subTick == 2 in the existing async-command seam).
//     - Each FPWR/FPRD transport step is one frame only, has NO Sleep/retry
//       loop, and is capped by a 55 us hard deadline.
//     - If the PDO cycle is unhealthy, or the handler has already consumed too
//       much of its 250 us budget, the SDO step is simply DEFERRED.
//     - Mailbox processing latency is therefore allowed to span milliseconds
//       while cyclic PDO continues every 250 us.
//
// Supported online service scope for this stage:
//     - CoE expedited SDO Read/Write only (1, 2, or 4 byte scalar).
//     - Complete Access and segmented transfers remain unsupported online.
//     - Startup/configuration code may continue using the legacy blocking SDO
//       helpers because that path runs before the 4 kHz PDO runtime.
//
// Safety objective:
//     No RtSleepFt(), no 20x poll loop, no 8x retry loop and no synchronous
//     ecx_SDOread()/ecx_SDOwrite() call remains in the live PDO async path.
// ============================================================================
namespace
{
    // Source fingerprint for field verification.
    constexpr char OSCARMAX_ECAT_SDO_RT_BUILD_TAG[] =
        "OSCARMAX_SDO_RT_12F3E11_20260826";
    static_assert(
        OSCARMAX_ECAT_SDO_RT_BUILD_TAG[0] == 'O',
        "SDO RT source fingerprint missing.");

    constexpr uint64_t OSCARMAX_ECAT_SDO_RT_STEP_DEADLINE_NS = 55000ULL;

    // Stage 12F.3E.4 - mailbox-ready check before destructive fetch.
    constexpr uint64_t OSCARMAX_ECAT_SDO_RT_FETCH_DEADLINE_NS = 120000ULL;
    constexpr uint64_t OSCARMAX_ECAT_SDO_RT_FETCH_HANDLER_GATE_NS = 70000ULL;
    constexpr uint16_t OSCARMAX_ECAT_SDO_RT_MBX_IN_SM_STATUS_REG = 0x080Du; // SM1 status

    // Stage 12F.3E.6/11 - mailbox-out EMPTY gate before FPWR.
    // SM0 status bit 3: 0 = mailbox empty / writable, 1 = mailbox full.
    // Keep this as a separate bounded RT step so the 250 us PDO cycle never
    // performs both the status FPRD and the mailbox FPWR in one callback.
    constexpr uint16_t OSCARMAX_ECAT_SDO_RT_MBX_OUT_SM_STATUS_REG = 0x0805u; // SM0 status fallback

    constexpr uint64_t OSCARMAX_ECAT_SDO_RT_HANDLER_BUDGET_GATE_NS = 120000ULL;
    constexpr uint64_t OSCARMAX_ECAT_SDO_RT_NEXT_POLL_CYCLES = 4ULL;      // 1 ms
    constexpr uint64_t OSCARMAX_ECAT_SDO_RT_OPERATION_TIMEOUT_CYCLES = 8000ULL; // 2 s
    constexpr uint16_t OSCARMAX_ECAT_SDO_RT_MAX_MAILBOX_BYTES = 1024u;

    enum class RuntimeSafeSdoPhase : uint8_t
    {
        Idle = 0,
        SendRequest = 1,
        PollResponse = 2
    };

    enum class RuntimeSafeSdoFailureReason : uint32_t
    {
        None = 0u,
        InvalidStart = 1u,
        SlotMismatch = 2u,
        OperationTimeout = 3u,
        SendDeadline = 4u,
        SendWkcZero = 5u,
        SendTransportFailure = 6u,
        SdoAbort = 7u,
        InvalidResponse = 8u,
        InvalidPhase = 9u,
        MailboxOutNotConsumed = 10u,
        MailboxInNoResponse = 11u
    };

    struct RuntimeSafeSdoState
    {
        bool active = false;
        RuntimeSafeSdoPhase phase = RuntimeSafeSdoPhase::Idle;

        int commandType = (int)EcatCmdType::CMD_NONE;
        uint16_t slave = 0u;
        uint16_t index = 0u;
        uint8_t subIndex = 0u;
        int dataSize = 0;
        uint32_t dataValue = 0u;
        uint8_t mailboxCounter = 0u;

        // Runtime Mailbox schema is authoritative when available.  These are
        // status-byte addresses (SM base + 5), with conventional SM0/SM1
        // values retained only as the legacy fallback.
        uint16_t mailboxOutStatusReg = OSCARMAX_ECAT_SDO_RT_MBX_OUT_SM_STATUS_REG;
        uint16_t mailboxInStatusReg = OSCARMAX_ECAT_SDO_RT_MBX_IN_SM_STATUS_REG;

        // Stage 12F.3E.6:
        // Latched only after the non-destructive SM0 status check reports
        // mailbox EMPTY.  The actual FPWR is intentionally deferred to the
        // next PDO callback so one RT step still contains only one frame.
        bool mailboxOutReadyLatched = false;

        // Stage 12F.3E.11 handshake classifier.
        // requestWriteIssued is set after an acknowledged FPWR or an
        // acknowledgement-deadline miss (delivery uncertain, so never resend).
        // requestConsumed becomes true only after SM0 is observed EMPTY again,
        // proving that the slave-side PDI consumed the mailbox request.
        bool requestWriteIssued = false;
        bool requestConsumed = false;

        // Latched only after the non-destructive SM1 status check reports
        // mailbox FULL.  A later bounded step then fetches mailbox data.
        bool mailboxInReadyLatched = false;

        uint64_t startTick = 0ULL;
        uint64_t nextStepTick = 0ULL;
        uint64_t deadlineTick = 0ULL;
        uint64_t operationStartNs = 0ULL;

        // Last terminal/transport details are intentionally NOT cleared by
        // ResetRuntimeSafeSdoState().  The Priority-50 engineering publisher
        // can therefore explain why the most recent request failed.
        uint32_t lastAbortCode = 0u;
        RuntimeSafeSdoFailureReason lastFailureReason =
            RuntimeSafeSdoFailureReason::None;
        int32_t lastTransportResult = 0;
        int32_t lastTerminalResult = 0; // 0=None, 1=Success, -1=Error

        int lastCommandType = (int)EcatCmdType::CMD_NONE;
        uint16_t lastSlave = 0u;
        uint16_t lastIndex = 0u;
        uint8_t lastSubIndex = 0u;
        uint8_t lastDataSize = 0u;

        uint64_t lastOperationDurationUs = 0ULL;
        uint64_t maxOperationDurationUs = 0ULL;

        // Runtime diagnostics.  These counters are observational only and do
        // not alter mailbox scheduling or PDO ownership.
        uint64_t operationsStarted = 0ULL;
        uint64_t operationsCompleted = 0ULL;
        uint64_t operationsFailed = 0ULL;
        uint64_t operationTimeouts = 0ULL;
        uint64_t invalidStartCount = 0ULL;
        uint64_t slotMismatchCount = 0ULL;

        uint64_t sendAttempts = 0ULL;
        uint64_t sendSuccess = 0ULL;
        uint64_t sendWkcZero = 0ULL;
        uint64_t sendDeadlineMiss = 0ULL;
        uint64_t sendTransportFailure = 0ULL;

        uint64_t pollAttempts = 0ULL;
        uint64_t pollWkcPositive = 0ULL;
        uint64_t pollWkcZero = 0ULL;
        uint64_t pollDeadlineMiss = 0ULL;
        uint64_t pollTransportFailure = 0ULL;
        uint64_t pollNotReady = 0ULL;
        uint64_t responseMismatchCount = 0ULL;
        uint64_t invalidResponseCount = 0ULL;

        uint64_t stepsExecuted = 0ULL;
        uint64_t stepsDeferredPdo = 0ULL;
        uint64_t stepsDeferredBudget = 0ULL;
        uint64_t transportDeadlineMisses = 0ULL;
        uint64_t abortCount = 0ULL;

        uint64_t stepCostLastNs = 0ULL;
        uint64_t stepCostMaxNs = 0ULL;
        uint64_t stepCostTotalNs = 0ULL;
    };

    struct OSCARMAX_ECAT_SdoRtDiagSnapshot
    {
        volatile LONG Sequence = 0;

        uint32_t StructSize = 0u;
        uint32_t Version = 1u;

        uint32_t Active = 0u;
        uint32_t Phase = 0u;

        int32_t LastTerminalResult = 0;
        uint32_t LastFailureReason = 0u;
        int32_t LastTransportResult = 0;
        uint32_t LastAbortCode = 0u;

        int32_t LastCommandType = 0;
        uint32_t LastSlave = 0u;
        uint32_t LastIndex = 0u;
        uint32_t LastSubIndex = 0u;
        uint32_t LastDataSize = 0u;

        uint64_t LastOperationDurationUs = 0ULL;
        uint64_t MaxOperationDurationUs = 0ULL;

        uint64_t OperationsStarted = 0ULL;
        uint64_t OperationsCompleted = 0ULL;
        uint64_t OperationsFailed = 0ULL;
        uint64_t OperationTimeouts = 0ULL;
        uint64_t InvalidStartCount = 0ULL;
        uint64_t SlotMismatchCount = 0ULL;

        uint64_t SendAttempts = 0ULL;
        uint64_t SendSuccess = 0ULL;
        uint64_t SendWkcZero = 0ULL;
        uint64_t SendDeadlineMiss = 0ULL;
        uint64_t SendTransportFailure = 0ULL;

        uint64_t PollAttempts = 0ULL;
        uint64_t PollWkcPositive = 0ULL;
        uint64_t PollWkcZero = 0ULL;
        uint64_t PollDeadlineMiss = 0ULL;
        uint64_t PollTransportFailure = 0ULL;
        uint64_t PollNotReady = 0ULL;
        uint64_t ResponseMismatchCount = 0ULL;
        uint64_t InvalidResponseCount = 0ULL;

        uint64_t StepsExecuted = 0ULL;
        uint64_t StepsDeferredPdo = 0ULL;
        uint64_t StepsDeferredBudget = 0ULL;
        uint64_t TransportDeadlineMisses = 0ULL;
        uint64_t AbortCount = 0ULL;

        uint64_t StepCostLastNs = 0ULL;
        uint64_t StepCostMaxNs = 0ULL;
        uint64_t StepCostAverageNs = 0ULL;
    };

    static_assert(
        sizeof(OSCARMAX_ECAT_SdoRtDiagSnapshot) == 288,
        "OSCARMAX_ECAT_SdoRtDiagSnapshot ABI changed.");

    RuntimeSafeSdoState g_runtimeSafeSdo;
    OSCARMAX_ECAT_SdoRtDiagSnapshot g_runtimeSafeSdoDiag;

    void PublishRuntimeSafeSdoDiagSnapshot()
    {
        InterlockedIncrement(&g_runtimeSafeSdoDiag.Sequence);
        MemoryBarrier();

        g_runtimeSafeSdoDiag.StructSize =
            static_cast<uint32_t>(sizeof(OSCARMAX_ECAT_SdoRtDiagSnapshot));
        g_runtimeSafeSdoDiag.Version = 1u;

        g_runtimeSafeSdoDiag.Active =
            g_runtimeSafeSdo.active ? 1u : 0u;
        g_runtimeSafeSdoDiag.Phase =
            static_cast<uint32_t>(g_runtimeSafeSdo.phase);

        g_runtimeSafeSdoDiag.LastTerminalResult =
            g_runtimeSafeSdo.lastTerminalResult;
        g_runtimeSafeSdoDiag.LastFailureReason =
            static_cast<uint32_t>(g_runtimeSafeSdo.lastFailureReason);
        g_runtimeSafeSdoDiag.LastTransportResult =
            g_runtimeSafeSdo.lastTransportResult;
        g_runtimeSafeSdoDiag.LastAbortCode =
            g_runtimeSafeSdo.lastAbortCode;

        g_runtimeSafeSdoDiag.LastCommandType =
            g_runtimeSafeSdo.lastCommandType;
        g_runtimeSafeSdoDiag.LastSlave =
            static_cast<uint32_t>(g_runtimeSafeSdo.lastSlave);
        g_runtimeSafeSdoDiag.LastIndex =
            static_cast<uint32_t>(g_runtimeSafeSdo.lastIndex);
        g_runtimeSafeSdoDiag.LastSubIndex =
            static_cast<uint32_t>(g_runtimeSafeSdo.lastSubIndex);
        g_runtimeSafeSdoDiag.LastDataSize =
            static_cast<uint32_t>(g_runtimeSafeSdo.lastDataSize);

        g_runtimeSafeSdoDiag.LastOperationDurationUs =
            g_runtimeSafeSdo.lastOperationDurationUs;
        g_runtimeSafeSdoDiag.MaxOperationDurationUs =
            g_runtimeSafeSdo.maxOperationDurationUs;

        g_runtimeSafeSdoDiag.OperationsStarted =
            g_runtimeSafeSdo.operationsStarted;
        g_runtimeSafeSdoDiag.OperationsCompleted =
            g_runtimeSafeSdo.operationsCompleted;
        g_runtimeSafeSdoDiag.OperationsFailed =
            g_runtimeSafeSdo.operationsFailed;
        g_runtimeSafeSdoDiag.OperationTimeouts =
            g_runtimeSafeSdo.operationTimeouts;
        g_runtimeSafeSdoDiag.InvalidStartCount =
            g_runtimeSafeSdo.invalidStartCount;
        g_runtimeSafeSdoDiag.SlotMismatchCount =
            g_runtimeSafeSdo.slotMismatchCount;

        g_runtimeSafeSdoDiag.SendAttempts =
            g_runtimeSafeSdo.sendAttempts;
        g_runtimeSafeSdoDiag.SendSuccess =
            g_runtimeSafeSdo.sendSuccess;
        g_runtimeSafeSdoDiag.SendWkcZero =
            g_runtimeSafeSdo.sendWkcZero;
        g_runtimeSafeSdoDiag.SendDeadlineMiss =
            g_runtimeSafeSdo.sendDeadlineMiss;
        g_runtimeSafeSdoDiag.SendTransportFailure =
            g_runtimeSafeSdo.sendTransportFailure;

        g_runtimeSafeSdoDiag.PollAttempts =
            g_runtimeSafeSdo.pollAttempts;
        g_runtimeSafeSdoDiag.PollWkcPositive =
            g_runtimeSafeSdo.pollWkcPositive;
        g_runtimeSafeSdoDiag.PollWkcZero =
            g_runtimeSafeSdo.pollWkcZero;
        g_runtimeSafeSdoDiag.PollDeadlineMiss =
            g_runtimeSafeSdo.pollDeadlineMiss;
        g_runtimeSafeSdoDiag.PollTransportFailure =
            g_runtimeSafeSdo.pollTransportFailure;
        g_runtimeSafeSdoDiag.PollNotReady =
            g_runtimeSafeSdo.pollNotReady;
        g_runtimeSafeSdoDiag.ResponseMismatchCount =
            g_runtimeSafeSdo.responseMismatchCount;
        g_runtimeSafeSdoDiag.InvalidResponseCount =
            g_runtimeSafeSdo.invalidResponseCount;

        g_runtimeSafeSdoDiag.StepsExecuted =
            g_runtimeSafeSdo.stepsExecuted;
        g_runtimeSafeSdoDiag.StepsDeferredPdo =
            g_runtimeSafeSdo.stepsDeferredPdo;
        g_runtimeSafeSdoDiag.StepsDeferredBudget =
            g_runtimeSafeSdo.stepsDeferredBudget;
        g_runtimeSafeSdoDiag.TransportDeadlineMisses =
            g_runtimeSafeSdo.transportDeadlineMisses;
        g_runtimeSafeSdoDiag.AbortCount =
            g_runtimeSafeSdo.abortCount;

        g_runtimeSafeSdoDiag.StepCostLastNs =
            g_runtimeSafeSdo.stepCostLastNs;
        g_runtimeSafeSdoDiag.StepCostMaxNs =
            g_runtimeSafeSdo.stepCostMaxNs;

        if (g_runtimeSafeSdo.stepsExecuted > 0ULL)
        {
            g_runtimeSafeSdoDiag.StepCostAverageNs =
                g_runtimeSafeSdo.stepCostTotalNs /
                g_runtimeSafeSdo.stepsExecuted;
        }
        else
        {
            g_runtimeSafeSdoDiag.StepCostAverageNs = 0ULL;
        }

        MemoryBarrier();
        InterlockedIncrement(&g_runtimeSafeSdoDiag.Sequence);
    }

    // ------------------------------------------------------------------------
    // One bounded Fixed-Position Physical transaction.
    //
    // Runtime mailbox traffic uses the slave Configured Station Address
    // (FPRD/FPWR).  This intentionally does NOT call the generic blocking
    // helpers because they contain timeout retry loops + RtSleepFt().
    // This helper sends one frame, polls ReceivePacket() without sleeping,
    // and returns at the hard deadline.  It is used only by the online SDO
    // runtime state machine.
    //
    // Return:
    //     >0  EtherCAT WKC
    //      0  WKC 0 / invalid response
    //     -1  hard deadline expired
    //     -2  TX submission failed / invalid argument
    // ------------------------------------------------------------------------
    int RuntimeSafeSdoFpTransaction(
        EtherCatMaster* pMaster,
        uint8_t command,
        uint16_t adp,
        uint16_t ado,
        uint16_t length,
        const uint8_t* writeData,
        uint8_t* readData,
        uint64_t deadlineNs)
    {
        if (pMaster == nullptr ||
            pMaster->m_pNic == nullptr ||
            length == 0u ||
            length > OSCARMAX_ECAT_SDO_RT_MAX_MAILBOX_BYTES ||
            (28u + static_cast<uint32_t>(length)) > 1514u)
        {
            return -2;
        }

        static uint8_t txFrame[1514];
        static uint8_t rxFrame[1514];

        const int totalFrameBytes =
            28 + static_cast<int>(length);

        std::memset(txFrame, 0, static_cast<size_t>(totalFrameBytes));

        // Ethernet header.
        for (int i = 0; i < 6; ++i)
        {
            txFrame[i] = 0xFFu;
        }

        uint8_t sourceMac[6] = {};
        pMaster->m_pNic->GetMacAddress(sourceMac);
        std::memcpy(&txFrame[6], sourceMac, 6u);

        txFrame[12] = 0x88u;
        txFrame[13] = 0xA4u;

        // EtherCAT header: one datagram, Type 1.
        const uint16_t ecatPayloadLength =
            static_cast<uint16_t>(10u + length + 2u);

        const uint16_t ecatHeader =
            static_cast<uint16_t>((ecatPayloadLength & 0x07FFu) | 0x1000u);

        txFrame[14] = static_cast<uint8_t>(ecatHeader & 0xFFu);
        txFrame[15] = static_cast<uint8_t>((ecatHeader >> 8) & 0xFFu);

        const uint8_t datagramIndex =
            pMaster->m_idx++;

        txFrame[16] = command;
        txFrame[17] = datagramIndex;
        txFrame[18] = static_cast<uint8_t>(adp & 0xFFu);
        txFrame[19] = static_cast<uint8_t>((adp >> 8) & 0xFFu);
        txFrame[20] = static_cast<uint8_t>(ado & 0xFFu);
        txFrame[21] = static_cast<uint8_t>((ado >> 8) & 0xFFu);

        const uint16_t lengthInfo =
            static_cast<uint16_t>(length & 0x07FFu);

        txFrame[22] = static_cast<uint8_t>(lengthInfo & 0xFFu);
        txFrame[23] = static_cast<uint8_t>((lengthInfo >> 8) & 0xFFu);
        txFrame[24] = 0x00u;
        txFrame[25] = 0x00u;

        if (command == 0x05u && writeData != nullptr) // FPWR
        {
            std::memcpy(&txFrame[26], writeData, length);
        }
        else
        {
            std::memset(&txFrame[26], 0, length);
        }

        txFrame[26 + length] = 0x00u;
        txFrame[27 + length] = 0x00u;

        const uint64_t startNs =
            pMaster->GetCurrentMasterTimeNs();

        if (!pMaster->m_pNic->SendPacket(
            txFrame,
            static_cast<unsigned int>(totalFrameBytes)))
        {
            return -2;
        }

        int fallbackAttempts = 64;

        while (true)
        {
            const uint64_t beforeRxNs =
                pMaster->GetCurrentMasterTimeNs();

            if (startNs != 0u &&
                beforeRxNs >= startNs &&
                (beforeRxNs - startNs) >= deadlineNs)
            {
                return -1;
            }

            if (startNs == 0u && fallbackAttempts-- <= 0)
            {
                return -1;
            }

            const int rxLength =
                static_cast<int>(pMaster->m_pNic->ReceivePacket(rxFrame));

            const uint64_t afterRxNs =
                pMaster->GetCurrentMasterTimeNs();

            if (startNs != 0u &&
                afterRxNs >= startNs &&
                (afterRxNs - startNs) >= deadlineNs)
            {
                // A frame arriving after the bounded mailbox step deadline is
                // deliberately rejected.  Cyclic PDO owns the timing budget.
                return -1;
            }

            if (rxLength <= 0)
            {
                continue;
            }

            if (rxLength < totalFrameBytes ||
                rxFrame[12] != 0x88u ||
                rxFrame[13] != 0xA4u ||
                rxFrame[16] != command ||
                rxFrame[17] != datagramIndex)
            {
                // Late/unrelated frame.  Consume and continue until deadline.
                continue;
            }

            const int wkcOffset =
                26 + static_cast<int>(length);

            const uint16_t wkc =
                static_cast<uint16_t>(rxFrame[wkcOffset]) |
                static_cast<uint16_t>(
                    static_cast<uint16_t>(rxFrame[wkcOffset + 1]) << 8);

            if (wkc > 0u &&
                command == 0x04u &&
                readData != nullptr) // FPRD
            {
                std::memcpy(readData, &rxFrame[26], length);
            }

            return static_cast<int>(wkc);
        }
    }

    uint16_t ResolveRuntimeSafeSdoMailboxStatusRegister(
        EtherCatMaster* pMaster,
        uint16_t slave,
        bool mailboxOut)
    {
        const uint16_t fallbackRegister =
            mailboxOut
            ? OSCARMAX_ECAT_SDO_RT_MBX_OUT_SM_STATUS_REG
            : OSCARMAX_ECAT_SDO_RT_MBX_IN_SM_STATUS_REG;

        if (pMaster == nullptr || pMaster->m_pEni == nullptr)
        {
            return fallbackRegister;
        }

        const auto& runtimeSlaves =
            pMaster->m_pEni->GetSlaves();

        if (static_cast<size_t>(slave) >= runtimeSlaves.size())
        {
            return fallbackRegister;
        }

        const EtherCatSlave& runtimeSlave =
            runtimeSlaves[static_cast<size_t>(slave)];

        if (!runtimeSlave.runtimeMailbox.present)
        {
            return fallbackRegister;
        }

        const EtherCatRuntimeMailboxDirectionConfig& direction =
            mailboxOut
            ? runtimeSlave.runtimeMailbox.out
            : runtimeSlave.runtimeMailbox.in;

        if (!direction.present ||
            direction.smIndex < 0 ||
            direction.smIndex > 15)
        {
            return fallbackRegister;
        }

        return static_cast<uint16_t>(
            0x0805u +
            static_cast<uint16_t>(direction.smIndex * 8));
    }

    bool RuntimeSafeSdoCommandMatchesSlot(
        EtherCatMaster* pMaster)
    {
        if (pMaster == nullptr || !g_runtimeSafeSdo.active)
        {
            return false;
        }

        return
            g_runtimeSafeSdo.commandType == pMaster->m_asyncCmd.type &&
            g_runtimeSafeSdo.slave == pMaster->m_asyncCmd.slaveAddr &&
            g_runtimeSafeSdo.index == pMaster->m_asyncCmd.index &&
            g_runtimeSafeSdo.subIndex == pMaster->m_asyncCmd.subIndex;
    }

    void ResetRuntimeSafeSdoState()
    {
        g_runtimeSafeSdo.active = false;
        g_runtimeSafeSdo.phase = RuntimeSafeSdoPhase::Idle;
        g_runtimeSafeSdo.commandType = (int)EcatCmdType::CMD_NONE;
        g_runtimeSafeSdo.slave = 0u;
        g_runtimeSafeSdo.index = 0u;
        g_runtimeSafeSdo.subIndex = 0u;
        g_runtimeSafeSdo.dataSize = 0;
        g_runtimeSafeSdo.dataValue = 0u;
        g_runtimeSafeSdo.mailboxCounter = 0u;
        g_runtimeSafeSdo.mailboxOutStatusReg =
            OSCARMAX_ECAT_SDO_RT_MBX_OUT_SM_STATUS_REG;
        g_runtimeSafeSdo.mailboxInStatusReg =
            OSCARMAX_ECAT_SDO_RT_MBX_IN_SM_STATUS_REG;
        g_runtimeSafeSdo.mailboxOutReadyLatched = false;
        g_runtimeSafeSdo.requestWriteIssued = false;
        g_runtimeSafeSdo.requestConsumed = false;
        g_runtimeSafeSdo.mailboxInReadyLatched = false;
        g_runtimeSafeSdo.startTick = 0ULL;
        g_runtimeSafeSdo.nextStepTick = 0ULL;
        g_runtimeSafeSdo.deadlineTick = 0ULL;
        g_runtimeSafeSdo.operationStartNs = 0ULL;
    }

    bool BeginRuntimeSafeSdoCommand(
        EtherCatMaster* pMaster)
    {
        if (pMaster == nullptr)
        {
            return false;
        }

        const int type =
            pMaster->m_asyncCmd.type;

        if (type != (int)EcatCmdType::CMD_SDO_READ &&
            type != (int)EcatCmdType::CMD_SDO_WRITE)
        {
            return false;
        }

        const int size =
            pMaster->m_asyncCmd.dataSize;

        if (size != 1 && size != 2 && size != 4)
        {
            pMaster->m_asyncCmd.resultWKC = 0;
            g_runtimeSafeSdo.invalidStartCount++;
            g_runtimeSafeSdo.lastFailureReason =
                RuntimeSafeSdoFailureReason::InvalidStart;
            g_runtimeSafeSdo.lastTerminalResult = -1;
            PublishRuntimeSafeSdoDiagSnapshot();
            return false;
        }

        const uint16_t slave =
            pMaster->m_asyncCmd.slaveAddr;

        if (slave >= 128u ||
            m_slaveInfo[slave].configAddr == 0u ||
            m_slaveInfo[slave].mbxOutAddr == 0u ||
            m_slaveInfo[slave].mbxInAddr == 0u ||
            m_slaveInfo[slave].mbxOutLength < 16u ||
            m_slaveInfo[slave].mbxInLength < 16u ||
            m_slaveInfo[slave].mbxOutLength > OSCARMAX_ECAT_SDO_RT_MAX_MAILBOX_BYTES ||
            m_slaveInfo[slave].mbxInLength > OSCARMAX_ECAT_SDO_RT_MAX_MAILBOX_BYTES)
        {
            pMaster->m_asyncCmd.resultWKC = 0;
            g_runtimeSafeSdo.invalidStartCount++;
            g_runtimeSafeSdo.lastFailureReason =
                RuntimeSafeSdoFailureReason::InvalidStart;
            g_runtimeSafeSdo.lastTerminalResult = -1;
            g_runtimeSafeSdo.lastSlave = slave;
            g_runtimeSafeSdo.lastIndex = pMaster->m_asyncCmd.index;
            g_runtimeSafeSdo.lastSubIndex = pMaster->m_asyncCmd.subIndex;
            PublishRuntimeSafeSdoDiagSnapshot();
            return false;
        }

        g_runtimeSafeSdo.active = true;
        g_runtimeSafeSdo.phase = RuntimeSafeSdoPhase::SendRequest;
        g_runtimeSafeSdo.commandType = type;
        g_runtimeSafeSdo.slave = slave;
        g_runtimeSafeSdo.index = pMaster->m_asyncCmd.index;
        g_runtimeSafeSdo.subIndex = pMaster->m_asyncCmd.subIndex;
        g_runtimeSafeSdo.dataSize = size;
        g_runtimeSafeSdo.dataValue = pMaster->m_asyncCmd.dataValue;
        g_runtimeSafeSdo.mailboxOutStatusReg =
            ResolveRuntimeSafeSdoMailboxStatusRegister(
                pMaster,
                slave,
                true);
        g_runtimeSafeSdo.mailboxInStatusReg =
            ResolveRuntimeSafeSdoMailboxStatusRegister(
                pMaster,
                slave,
                false);
        g_runtimeSafeSdo.mailboxOutReadyLatched = false;
        g_runtimeSafeSdo.requestWriteIssued = false;
        g_runtimeSafeSdo.requestConsumed = false;
        g_runtimeSafeSdo.mailboxInReadyLatched = false;
        g_runtimeSafeSdo.startTick = pMaster->tickCount_PDO;
        g_runtimeSafeSdo.nextStepTick = pMaster->tickCount_PDO;
        g_runtimeSafeSdo.deadlineTick =
            pMaster->tickCount_PDO + OSCARMAX_ECAT_SDO_RT_OPERATION_TIMEOUT_CYCLES;
        g_runtimeSafeSdo.operationStartNs =
            pMaster->GetCurrentMasterTimeNs();

        g_runtimeSafeSdo.lastCommandType = type;
        g_runtimeSafeSdo.lastSlave = slave;
        g_runtimeSafeSdo.lastIndex = pMaster->m_asyncCmd.index;
        g_runtimeSafeSdo.lastSubIndex = pMaster->m_asyncCmd.subIndex;
        g_runtimeSafeSdo.lastDataSize =
            static_cast<uint8_t>(size);
        g_runtimeSafeSdo.lastAbortCode = 0u;
        g_runtimeSafeSdo.lastTransportResult = 0;
        g_runtimeSafeSdo.lastTerminalResult = 0;
        g_runtimeSafeSdo.operationsStarted++;

        PublishRuntimeSafeSdoDiagSnapshot();
        return true;
    }

    void FinalizeRuntimeSafeSdoOperation(
        EtherCatMaster* pMaster,
        bool success,
        RuntimeSafeSdoFailureReason failureReason)
    {
        uint64_t durationUs = 0ULL;

        if (pMaster != nullptr &&
            g_runtimeSafeSdo.operationStartNs != 0ULL)
        {
            const uint64_t endNs =
                pMaster->GetCurrentMasterTimeNs();

            if (endNs >= g_runtimeSafeSdo.operationStartNs)
            {
                durationUs =
                    (endNs - g_runtimeSafeSdo.operationStartNs + 500ULL) /
                    1000ULL;
            }
        }

        g_runtimeSafeSdo.lastOperationDurationUs = durationUs;

        if (durationUs > g_runtimeSafeSdo.maxOperationDurationUs)
        {
            g_runtimeSafeSdo.maxOperationDurationUs = durationUs;
        }

        g_runtimeSafeSdo.lastTerminalResult =
            success ? 1 : -1;

        g_runtimeSafeSdo.lastFailureReason =
            success ?
            RuntimeSafeSdoFailureReason::None :
            failureReason;
    }

    void BuildRuntimeSafeSdoRequest(
        EtherCatMaster* pMaster,
        uint8_t* request,
        uint16_t requestCapacity)
    {
        if (pMaster == nullptr ||
            request == nullptr ||
            requestCapacity == 0u)
        {
            return;
        }

        std::memset(request, 0, requestCapacity);

        // Expedited upload/download request uses 10 bytes of Mailbox Service
        // Data after the 6-byte EtherCAT mailbox header.
        request[0] = 0x0Au;
        request[1] = 0x00u;
        request[2] = 0x00u;
        request[3] = 0x00u;
        request[4] = 0x00u;

        // Stage 12F.3E.5 - EtherCAT mailbox counter must cycle 1..7.
        //
        // Counter value 0 is not used for normal numbered mailbox sessions.
        // The previous implementation used `++counter & 0x07`, which emitted
        // 0 once every eight requests.  Some slaves (including the tested
        // Delta drive) can then ignore the request and never publish an SM1
        // response, producing an OPERATION_TIMEOUT even though cyclic PDO is
        // completely healthy.
        //
        // Keep the existing master-owned counter, but wrap 7 -> 1 explicitly.
        uint8_t nextCounter =
            static_cast<uint8_t>(pMaster->m_mboxCnt & 0x07u);

        nextCounter++;

        if (nextCounter == 0u || nextCounter > 7u)
        {
            nextCounter = 1u;
        }

        pMaster->m_mboxCnt = nextCounter;
        g_runtimeSafeSdo.mailboxCounter = nextCounter;

        request[5] =
            static_cast<uint8_t>(0x03u | (g_runtimeSafeSdo.mailboxCounter << 4));

        request[6] = 0x00u;
        request[7] = 0x20u; // CoE SDO request (0x2000 LE)

        if (g_runtimeSafeSdo.commandType == (int)EcatCmdType::CMD_SDO_READ)
        {
            request[8] = 0x40u; // Upload request
        }
        else
        {
            const int emptyBytes =
                4 - g_runtimeSafeSdo.dataSize;

            request[8] =
                static_cast<uint8_t>(0x23u | ((emptyBytes & 0x03) << 2));
        }

        request[9] =
            static_cast<uint8_t>(g_runtimeSafeSdo.index & 0xFFu);
        request[10] =
            static_cast<uint8_t>((g_runtimeSafeSdo.index >> 8) & 0xFFu);
        request[11] =
            g_runtimeSafeSdo.subIndex;

        if (g_runtimeSafeSdo.commandType == (int)EcatCmdType::CMD_SDO_WRITE)
        {
            std::memcpy(
                &request[12],
                &g_runtimeSafeSdo.dataValue,
                static_cast<size_t>(g_runtimeSafeSdo.dataSize));
        }
    }

    enum class RuntimeSafeSdoStepDisposition : uint8_t
    {
        InProgress = 0,
        Done = 1,
        Error = 2
    };

    RuntimeSafeSdoStepDisposition ProcessRuntimeSafeSdoStep(
        EtherCatMaster* pMaster,
        uint64_t pdoCycleStartMasterNs,
        bool processDataValid,
        int* terminalWkc)
    {
        if (terminalWkc != nullptr)
        {
            *terminalWkc = 0;
        }

        if (pMaster == nullptr)
        {
            return RuntimeSafeSdoStepDisposition::Error;
        }

        if (!g_runtimeSafeSdo.active)
        {
            if (!BeginRuntimeSafeSdoCommand(pMaster))
            {
                return RuntimeSafeSdoStepDisposition::Error;
            }
        }
        else if (!RuntimeSafeSdoCommandMatchesSlot(pMaster))
        {
            g_runtimeSafeSdo.operationsFailed++;
            g_runtimeSafeSdo.slotMismatchCount++;
            FinalizeRuntimeSafeSdoOperation(
                pMaster,
                false,
                RuntimeSafeSdoFailureReason::SlotMismatch);
            ResetRuntimeSafeSdoState();
            PublishRuntimeSafeSdoDiagSnapshot();
            return RuntimeSafeSdoStepDisposition::Error;
        }

        if (pMaster->tickCount_PDO >= g_runtimeSafeSdo.deadlineTick)
        {
            g_runtimeSafeSdo.operationsFailed++;
            g_runtimeSafeSdo.operationTimeouts++;

            RuntimeSafeSdoFailureReason timeoutReason =
                RuntimeSafeSdoFailureReason::OperationTimeout;

            if (g_runtimeSafeSdo.requestWriteIssued &&
                !g_runtimeSafeSdo.requestConsumed)
            {
                timeoutReason =
                    RuntimeSafeSdoFailureReason::MailboxOutNotConsumed;
            }
            else if (g_runtimeSafeSdo.requestConsumed)
            {
                timeoutReason =
                    RuntimeSafeSdoFailureReason::MailboxInNoResponse;
            }

            FinalizeRuntimeSafeSdoOperation(
                pMaster,
                false,
                timeoutReason);
            ResetRuntimeSafeSdoState();
            PublishRuntimeSafeSdoDiagSnapshot();
            return RuntimeSafeSdoStepDisposition::Error;
        }

        if (!processDataValid)
        {
            g_runtimeSafeSdo.stepsDeferredPdo++;
            PublishRuntimeSafeSdoDiagSnapshot();
            return RuntimeSafeSdoStepDisposition::InProgress;
        }

        if (pMaster->tickCount_PDO < g_runtimeSafeSdo.nextStepTick)
        {
            return RuntimeSafeSdoStepDisposition::InProgress;
        }

        const uint64_t nowNs =
            pMaster->GetCurrentMasterTimeNs();

        if (pdoCycleStartMasterNs != 0u &&
            nowNs >= pdoCycleStartMasterNs &&
            (nowNs - pdoCycleStartMasterNs) >=
            OSCARMAX_ECAT_SDO_RT_HANDLER_BUDGET_GATE_NS)
        {
            g_runtimeSafeSdo.stepsDeferredBudget++;
            g_runtimeSafeSdo.nextStepTick =
                pMaster->tickCount_PDO + OSCARMAX_ECAT_SDO_RT_NEXT_POLL_CYCLES;
            PublishRuntimeSafeSdoDiagSnapshot();
            return RuntimeSafeSdoStepDisposition::InProgress;
        }

        const uint64_t stepStartNs =
            pMaster->GetCurrentMasterTimeNs();

        RuntimeSafeSdoStepDisposition disposition =
            RuntimeSafeSdoStepDisposition::InProgress;

        if (g_runtimeSafeSdo.phase == RuntimeSafeSdoPhase::SendRequest)
        {
            // =============================================================
            // Stage 12F.3E.6 - Mailbox Link Layer TX availability gate.
            //
            // Before writing a CoE request into the master->slave mailbox,
            // first read the configured Mailbox-Out SM Status byte and require
            // mailbox bit 3 = 0 (SM0/0x0805 on the standard layout).
            //
            // Why this matters:
            // - FPWR WKC>0 only proves that the EtherCAT write transaction
            //   reached the ESC; it does not replace mailbox ownership rules.
            // - Writing while SM0 is still FULL can overwrite / fail to create
            //   the mailbox hand-off edge expected by the slave application.
            // - The observed failure pattern was exactly: SEND transport alive,
            //   SM1 never became FULL, then the 2 s operation timeout expired.
            //
            // RT rule:
            // The SM0 status FPRD and the real mailbox FPWR are deliberately
            // split across two PDO callbacks.  This preserves the one-frame,
            // bounded-step rule and does not expand the Priority-64 critical
            // section.
            // =============================================================
            if (!g_runtimeSafeSdo.mailboxOutReadyLatched)
            {
                uint8_t sm0Status = 0u;

                const int wkc =
                    RuntimeSafeSdoFpTransaction(
                        pMaster,
                        0x04u, // FPRD - Mailbox-Out SM status, non-destructive
                        m_slaveInfo[g_runtimeSafeSdo.slave].configAddr,
                        g_runtimeSafeSdo.mailboxOutStatusReg,
                        1u,
                        nullptr,
                        &sm0Status,
                        OSCARMAX_ECAT_SDO_RT_STEP_DEADLINE_NS);

                g_runtimeSafeSdo.lastTransportResult = wkc;
                g_runtimeSafeSdo.stepsExecuted++;

                if (wkc > 0)
                {
                    const bool mailboxFull =
                        (sm0Status & 0x08u) != 0u;

                    if (!mailboxFull)
                    {
                        g_runtimeSafeSdo.mailboxOutReadyLatched = true;
                        g_runtimeSafeSdo.nextStepTick =
                            pMaster->tickCount_PDO + 1ULL;
                    }
                    else
                    {
                        // Normal mailbox back-pressure.  Do not write and do
                        // not fail the request; retry the status check later.
                        g_runtimeSafeSdo.nextStepTick =
                            pMaster->tickCount_PDO +
                            OSCARMAX_ECAT_SDO_RT_NEXT_POLL_CYCLES;
                    }
                }
                else
                {
                    // This is a SEND-path availability check, so reuse the
                    // existing compact SEND transport counters.  The operation
                    // itself remains pending until its normal 2 s deadline.
                    if (wkc == -1)
                    {
                        g_runtimeSafeSdo.transportDeadlineMisses++;
                        g_runtimeSafeSdo.sendDeadlineMiss++;
                    }
                    else if (wkc == 0)
                    {
                        g_runtimeSafeSdo.sendWkcZero++;
                    }
                    else
                    {
                        g_runtimeSafeSdo.sendTransportFailure++;
                    }

                    g_runtimeSafeSdo.nextStepTick =
                        pMaster->tickCount_PDO +
                        OSCARMAX_ECAT_SDO_RT_NEXT_POLL_CYCLES;
                }
            }
            else
            {
                static uint8_t request[OSCARMAX_ECAT_SDO_RT_MAX_MAILBOX_BYTES];

                const uint16_t mailboxLength =
                    m_slaveInfo[g_runtimeSafeSdo.slave].mbxOutLength;

                BuildRuntimeSafeSdoRequest(
                    pMaster,
                    request,
                    mailboxLength);

                // The EMPTY observation belongs only to this one FPWR attempt.
                // Never carry it across a failed/late send or into another
                // engineering request.
                g_runtimeSafeSdo.mailboxOutReadyLatched = false;

                g_runtimeSafeSdo.sendAttempts++;

                const int wkc =
                    RuntimeSafeSdoFpTransaction(
                        pMaster,
                        0x05u, // FPWR
                        m_slaveInfo[g_runtimeSafeSdo.slave].configAddr,
                        m_slaveInfo[g_runtimeSafeSdo.slave].mbxOutAddr,
                        mailboxLength,
                        request,
                        nullptr,
                        OSCARMAX_ECAT_SDO_RT_STEP_DEADLINE_NS);

                g_runtimeSafeSdo.lastTransportResult = wkc;
                g_runtimeSafeSdo.stepsExecuted++;

                if (wkc > 0)
                {
                    g_runtimeSafeSdo.sendSuccess++;
                    g_runtimeSafeSdo.requestWriteIssued = true;
                    g_runtimeSafeSdo.requestConsumed = false;
                    g_runtimeSafeSdo.phase = RuntimeSafeSdoPhase::PollResponse;
                    g_runtimeSafeSdo.nextStepTick =
                        pMaster->tickCount_PDO + OSCARMAX_ECAT_SDO_RT_NEXT_POLL_CYCLES;
                }
                else
                {
                    RuntimeSafeSdoFailureReason failureReason =
                        RuntimeSafeSdoFailureReason::SendTransportFailure;

                    if (wkc == -1)
                    {
                        // -----------------------------------------------------
                        // Stage 12F.3E.3 - Uncertain SEND acknowledgement recovery
                        // -----------------------------------------------------
                        // A hard-deadline miss after SendPacket() does NOT prove
                        // that the FPWR request failed to reach the slave.  The
                        // frame may already be on the wire / accepted by the ESC,
                        // while only the returning EtherCAT acknowledgement arrived
                        // later than our bounded RT step window.
                        //
                        // DO NOT resend: a duplicate SDO download may apply twice.
                        // Instead, treat the SEND result as "delivery uncertain"
                        // and move to the normal response-poll state.
                        // -----------------------------------------------------
                        g_runtimeSafeSdo.transportDeadlineMisses++;
                        g_runtimeSafeSdo.sendDeadlineMiss++;
                        g_runtimeSafeSdo.requestWriteIssued = true;
                        g_runtimeSafeSdo.requestConsumed = false;

                        g_runtimeSafeSdo.phase =
                            RuntimeSafeSdoPhase::PollResponse;

                        g_runtimeSafeSdo.nextStepTick =
                            pMaster->tickCount_PDO +
                            OSCARMAX_ECAT_SDO_RT_NEXT_POLL_CYCLES;

                        disposition =
                            RuntimeSafeSdoStepDisposition::InProgress;
                    }
                    else
                    {
                        if (wkc == 0)
                        {
                            // WKC=0 is a completed FPWR frame for which no slave
                            // accepted the write.  Unlike a deadline miss, this is
                            // not an ambiguous acknowledgement and is therefore a
                            // terminal transport failure in this stage.
                            g_runtimeSafeSdo.sendWkcZero++;
                            failureReason =
                                RuntimeSafeSdoFailureReason::SendWkcZero;
                        }
                        else
                        {
                            g_runtimeSafeSdo.sendTransportFailure++;
                        }

                        g_runtimeSafeSdo.operationsFailed++;
                        g_runtimeSafeSdo.lastFailureReason = failureReason;
                        disposition = RuntimeSafeSdoStepDisposition::Error;
                    }
                }
            }
        }
        else if (g_runtimeSafeSdo.phase == RuntimeSafeSdoPhase::PollResponse)
        {
            // =============================================================
            // Stage 12F.3E.11 - Mailbox handshake classifier.
            //
            // First prove that the just-written master->slave mailbox was
            // consumed by the slave PDI: after FPWR, SM0 bit 3 must return
            // to EMPTY.  Only then start waiting for the slave->master SM1
            // response.  This cleanly separates:
            //   REASON 10 = request was issued but SM0 was not consumed.
            //   REASON 11 = SM0 was consumed but SM1 never produced response.
            // =============================================================

            g_runtimeSafeSdo.pollAttempts++;

            if (g_runtimeSafeSdo.requestWriteIssued &&
                !g_runtimeSafeSdo.requestConsumed)
            {
                uint8_t sm0Status = 0u;

                const int wkc =
                    RuntimeSafeSdoFpTransaction(
                        pMaster,
                        0x04u, // FPRD - SM0 status, non-destructive
                        m_slaveInfo[g_runtimeSafeSdo.slave].configAddr,
                        g_runtimeSafeSdo.mailboxOutStatusReg,
                        1u,
                        nullptr,
                        &sm0Status,
                        OSCARMAX_ECAT_SDO_RT_STEP_DEADLINE_NS);

                g_runtimeSafeSdo.lastTransportResult = wkc;
                g_runtimeSafeSdo.stepsExecuted++;

                if (wkc <= 0)
                {
                    if (wkc == -1)
                    {
                        g_runtimeSafeSdo.transportDeadlineMisses++;
                        g_runtimeSafeSdo.pollDeadlineMiss++;
                    }
                    else if (wkc == 0)
                    {
                        g_runtimeSafeSdo.pollWkcZero++;
                    }
                    else
                    {
                        g_runtimeSafeSdo.pollTransportFailure++;
                    }

                    g_runtimeSafeSdo.pollNotReady++;
                    g_runtimeSafeSdo.nextStepTick =
                        pMaster->tickCount_PDO +
                        OSCARMAX_ECAT_SDO_RT_NEXT_POLL_CYCLES;
                }
                else
                {
                    g_runtimeSafeSdo.pollWkcPositive++;

                    const bool requestStillFull =
                        (sm0Status & 0x08u) != 0u;

                    if (requestStillFull)
                    {
                        g_runtimeSafeSdo.pollNotReady++;
                        g_runtimeSafeSdo.nextStepTick =
                            pMaster->tickCount_PDO +
                            OSCARMAX_ECAT_SDO_RT_NEXT_POLL_CYCLES;
                    }
                    else
                    {
                        g_runtimeSafeSdo.requestConsumed = true;
                        g_runtimeSafeSdo.nextStepTick =
                            pMaster->tickCount_PDO + 1ULL;
                    }
                }
            }
            else if (!g_runtimeSafeSdo.mailboxInReadyLatched)
            {
                // =========================================================
                // Stage 12F.3E.4 - Check mailbox FULL before fetching data.
                // =========================================================
                uint8_t sm1Status = 0u;

                const int wkc =
                    RuntimeSafeSdoFpTransaction(
                        pMaster,
                        0x04u, // FPRD - SM1 status, non-destructive
                        m_slaveInfo[g_runtimeSafeSdo.slave].configAddr,
                        g_runtimeSafeSdo.mailboxInStatusReg,
                        1u,
                        nullptr,
                        &sm1Status,
                        OSCARMAX_ECAT_SDO_RT_STEP_DEADLINE_NS);

                g_runtimeSafeSdo.lastTransportResult = wkc;
                g_runtimeSafeSdo.stepsExecuted++;

                if (wkc <= 0)
                {
                    if (wkc == -1)
                    {
                        g_runtimeSafeSdo.transportDeadlineMisses++;
                        g_runtimeSafeSdo.pollDeadlineMiss++;
                    }
                    else if (wkc == 0)
                    {
                        g_runtimeSafeSdo.pollWkcZero++;
                    }
                    else
                    {
                        g_runtimeSafeSdo.pollTransportFailure++;
                    }

                    g_runtimeSafeSdo.pollNotReady++;
                    g_runtimeSafeSdo.nextStepTick =
                        pMaster->tickCount_PDO +
                        OSCARMAX_ECAT_SDO_RT_NEXT_POLL_CYCLES;
                }
                else
                {
                    g_runtimeSafeSdo.pollWkcPositive++;

                    const bool mailboxFull =
                        (sm1Status & 0x08u) != 0u;

                    if (!mailboxFull)
                    {
                        g_runtimeSafeSdo.pollNotReady++;
                        g_runtimeSafeSdo.nextStepTick =
                            pMaster->tickCount_PDO +
                            OSCARMAX_ECAT_SDO_RT_NEXT_POLL_CYCLES;
                    }
                    else
                    {
                        g_runtimeSafeSdo.mailboxInReadyLatched = true;
                        g_runtimeSafeSdo.nextStepTick =
                            pMaster->tickCount_PDO + 1ULL;
                    }
                }
            }
            else
            {
                const uint64_t fetchGateNowNs =
                    pMaster->GetCurrentMasterTimeNs();

                if (pdoCycleStartMasterNs != 0u &&
                    fetchGateNowNs >= pdoCycleStartMasterNs &&
                    (fetchGateNowNs - pdoCycleStartMasterNs) >=
                    OSCARMAX_ECAT_SDO_RT_FETCH_HANDLER_GATE_NS)
                {
                    g_runtimeSafeSdo.stepsDeferredBudget++;
                    g_runtimeSafeSdo.nextStepTick =
                        pMaster->tickCount_PDO + 1ULL;
                }
                else
                {
                    static uint8_t response[OSCARMAX_ECAT_SDO_RT_MAX_MAILBOX_BYTES];
                    std::memset(response, 0, sizeof(response));

                    const uint16_t mailboxLength =
                        m_slaveInfo[g_runtimeSafeSdo.slave].mbxInLength;

                    const int wkc =
                        RuntimeSafeSdoFpTransaction(
                            pMaster,
                            0x04u, // FPRD - mailbox data fetch
                            m_slaveInfo[g_runtimeSafeSdo.slave].configAddr,
                            m_slaveInfo[g_runtimeSafeSdo.slave].mbxInAddr,
                            mailboxLength,
                            nullptr,
                            response,
                            OSCARMAX_ECAT_SDO_RT_FETCH_DEADLINE_NS);

                    g_runtimeSafeSdo.lastTransportResult = wkc;
                    g_runtimeSafeSdo.stepsExecuted++;
                    g_runtimeSafeSdo.mailboxInReadyLatched = false;

                    if (wkc <= 0)
                    {
                        if (wkc == -1)
                        {
                            g_runtimeSafeSdo.transportDeadlineMisses++;
                            g_runtimeSafeSdo.pollDeadlineMiss++;
                        }
                        else if (wkc == 0)
                        {
                            g_runtimeSafeSdo.pollWkcZero++;
                        }
                        else
                        {
                            g_runtimeSafeSdo.pollTransportFailure++;
                        }

                        g_runtimeSafeSdo.pollNotReady++;
                        g_runtimeSafeSdo.nextStepTick =
                            pMaster->tickCount_PDO +
                            OSCARMAX_ECAT_SDO_RT_NEXT_POLL_CYCLES;
                    }
                    else
                    {
                        g_runtimeSafeSdo.pollWkcPositive++;

                        const uint8_t mailboxType =
                            static_cast<uint8_t>(response[5] & 0x0Fu);

                        // The mailbox counter is sender-local sequencing; a
                        // slave response is NOT required to echo the master's
                        // request counter.  Match the response by protocol +
                        // CoE SDO service + object identity instead.
                        const uint16_t coeHeader =
                            static_cast<uint16_t>(response[6]) |
                            static_cast<uint16_t>(
                                static_cast<uint16_t>(response[7]) << 8);

                        const uint8_t coeService =
                            static_cast<uint8_t>((coeHeader >> 12) & 0x0Fu);

                        const uint8_t command = response[8];
                        const uint16_t responseIndex =
                            static_cast<uint16_t>(response[9]) |
                            static_cast<uint16_t>(
                                static_cast<uint16_t>(response[10]) << 8);
                        const uint8_t responseSubIndex = response[11];

                        const bool belongsToCurrentRequest =
                            mailboxType == 0x03u &&
                            coeService == 0x03u &&
                            responseIndex == g_runtimeSafeSdo.index &&
                            responseSubIndex == g_runtimeSafeSdo.subIndex;

                        if (!belongsToCurrentRequest)
                        {
                            g_runtimeSafeSdo.responseMismatchCount++;
                            g_runtimeSafeSdo.pollNotReady++;
                            g_runtimeSafeSdo.nextStepTick =
                                pMaster->tickCount_PDO +
                                OSCARMAX_ECAT_SDO_RT_NEXT_POLL_CYCLES;
                        }
                        else if (command == 0x80u)
                        {
                            uint32_t abortCode = 0u;
                            std::memcpy(&abortCode, &response[12], sizeof(abortCode));
                            g_runtimeSafeSdo.lastAbortCode = abortCode;
                            g_runtimeSafeSdo.abortCount++;
                            g_runtimeSafeSdo.operationsFailed++;
                            g_runtimeSafeSdo.lastFailureReason =
                                RuntimeSafeSdoFailureReason::SdoAbort;
                            disposition = RuntimeSafeSdoStepDisposition::Error;
                        }
                        else if (g_runtimeSafeSdo.commandType == (int)EcatCmdType::CMD_SDO_WRITE)
                        {
                            if (command == 0x60u)
                            {
                                if (terminalWkc != nullptr)
                                {
                                    *terminalWkc = wkc;
                                }

                                pMaster->m_asyncCmd.dataValue =
                                    g_runtimeSafeSdo.dataValue;
                                pMaster->m_asyncCmd.dataSize =
                                    g_runtimeSafeSdo.dataSize;

                                g_runtimeSafeSdo.operationsCompleted++;
                                disposition = RuntimeSafeSdoStepDisposition::Done;
                            }
                            else
                            {
                                g_runtimeSafeSdo.operationsFailed++;
                                g_runtimeSafeSdo.invalidResponseCount++;
                                g_runtimeSafeSdo.lastFailureReason =
                                    RuntimeSafeSdoFailureReason::InvalidResponse;
                                disposition = RuntimeSafeSdoStepDisposition::Error;
                            }
                        }
                        else // CMD_SDO_READ
                        {
                            const bool uploadResponse =
                                (command & 0xE0u) == 0x40u;

                            const bool expedited =
                                (command & 0x02u) != 0u;

                            if (!uploadResponse || !expedited)
                            {
                                g_runtimeSafeSdo.operationsFailed++;
                                g_runtimeSafeSdo.invalidResponseCount++;
                                g_runtimeSafeSdo.lastFailureReason =
                                    RuntimeSafeSdoFailureReason::InvalidResponse;
                                disposition = RuntimeSafeSdoStepDisposition::Error;
                            }
                            else
                            {
                                int validBytes = 4;

                                if ((command & 0x01u) != 0u)
                                {
                                    const int emptyBytes =
                                        static_cast<int>((command >> 2) & 0x03u);
                                    validBytes = 4 - emptyBytes;
                                }

                                if (validBytes != 1 &&
                                    validBytes != 2 &&
                                    validBytes != 4)
                                {
                                    g_runtimeSafeSdo.operationsFailed++;
                                    g_runtimeSafeSdo.invalidResponseCount++;
                                    g_runtimeSafeSdo.lastFailureReason =
                                        RuntimeSafeSdoFailureReason::InvalidResponse;
                                    disposition = RuntimeSafeSdoStepDisposition::Error;
                                }
                                else
                                {
                                    uint32_t value = 0u;
                                    std::memcpy(
                                        &value,
                                        &response[12],
                                        static_cast<size_t>(validBytes));

                                    pMaster->m_asyncCmd.dataValue = value;
                                    pMaster->m_asyncCmd.dataSize = validBytes;

                                    if (terminalWkc != nullptr)
                                    {
                                        *terminalWkc = wkc;
                                    }

                                    g_runtimeSafeSdo.operationsCompleted++;
                                    disposition = RuntimeSafeSdoStepDisposition::Done;
                                }
                            }
                        }
                    }
                }
            }
        }
        else
        {
            g_runtimeSafeSdo.operationsFailed++;
            g_runtimeSafeSdo.invalidResponseCount++;
            g_runtimeSafeSdo.lastFailureReason =
                RuntimeSafeSdoFailureReason::InvalidPhase;
            disposition = RuntimeSafeSdoStepDisposition::Error;
        }

        const uint64_t stepEndNs =
            pMaster->GetCurrentMasterTimeNs();

        uint64_t stepCostNs = 0ULL;

        if (stepEndNs >= stepStartNs)
        {
            stepCostNs = stepEndNs - stepStartNs;
        }

        g_runtimeSafeSdo.stepCostLastNs = stepCostNs;

        if (stepCostNs > g_runtimeSafeSdo.stepCostMaxNs)
        {
            g_runtimeSafeSdo.stepCostMaxNs = stepCostNs;
        }

        if (0xFFFFFFFFFFFFFFFFULL - g_runtimeSafeSdo.stepCostTotalNs >= stepCostNs)
        {
            g_runtimeSafeSdo.stepCostTotalNs += stepCostNs;
        }
        else
        {
            g_runtimeSafeSdo.stepCostTotalNs = 0xFFFFFFFFFFFFFFFFULL;
        }

        if (disposition == RuntimeSafeSdoStepDisposition::Done)
        {
            FinalizeRuntimeSafeSdoOperation(
                pMaster,
                true,
                RuntimeSafeSdoFailureReason::None);
            ResetRuntimeSafeSdoState();
            PublishRuntimeSafeSdoDiagSnapshot();
        }
        else if (disposition == RuntimeSafeSdoStepDisposition::Error)
        {
            RuntimeSafeSdoFailureReason reason =
                g_runtimeSafeSdo.lastFailureReason;

            if (reason == RuntimeSafeSdoFailureReason::None)
            {
                reason = RuntimeSafeSdoFailureReason::InvalidResponse;
            }

            FinalizeRuntimeSafeSdoOperation(
                pMaster,
                false,
                reason);
            ResetRuntimeSafeSdoState();
            PublishRuntimeSafeSdoDiagSnapshot();
        }
        else
        {
            PublishRuntimeSafeSdoDiagSnapshot();
        }

        return disposition;
    }
}

// ============================================================================
// Stage 12F.3E.3 - Priority-50 readable SDO RT diagnostics snapshot
// ============================================================================
//
// This is an in-process C ABI only; it does not touch Shared Memory and does
// not change the live SDO state machine.  A lower-priority publisher can copy
// the most recent bounded-mailbox statistics without reading P64-owned state
// directly.  Read attempts are bounded to three seqlock checks and never wait.
// ============================================================================
extern "C" bool OSCARMAX_ECAT_SdoRtDiag_Read(
    void* output,
    uint32_t outputBytes)
{
    if (output == nullptr ||
        outputBytes < sizeof(OSCARMAX_ECAT_SdoRtDiagSnapshot))
    {
        return false;
    }

    for (int attempt = 0; attempt < 3; ++attempt)
    {
        const LONG sequenceBefore =
            g_runtimeSafeSdoDiag.Sequence;

        if ((sequenceBefore & 1L) != 0L)
        {
            continue;
        }

        MemoryBarrier();

        OSCARMAX_ECAT_SdoRtDiagSnapshot local = {};
        std::memcpy(
            &local,
            &g_runtimeSafeSdoDiag,
            sizeof(local));

        MemoryBarrier();

        const LONG sequenceAfter =
            g_runtimeSafeSdoDiag.Sequence;

        if (sequenceBefore == sequenceAfter &&
            (sequenceAfter & 1L) == 0L)
        {
            std::memcpy(
                output,
                &local,
                sizeof(local));
            return true;
        }
    }

    return false;
}

// ============================================================================
// Stage 12E.3A - Owner-Safe Per-Slave ESC Live Diagnostic RT Shadow
// Stage 12E.4A - Owner-Safe SyncManager Live Register Snapshot
// Stage 12E.5A - Owner-Safe FMMU Live Register Snapshot
// ============================================================================
//
// Purpose:
//     Complete the data foundation behind the ENI Tool Expert Diagnostics
//     without allowing the Priority-50 publisher or Windows UI to touch the
//     EtherCAT NIC directly.
//
// Runtime policy:
//     - Priority 64 remains the ONLY live EtherCAT NIC owner.
//     - One bounded FPRD probe is attempted at most once every 80 PDO cycles
//       (20 ms at 250 us/cycle).
//     - The probe runs only on subTick 0, while async SDO/state commands use
//       subTick 2, so a callback never stacks an SDO frame and an ESC probe.
//     - SDO active/pending always has priority; ESC diagnostics simply defer.
//     - No Sleep, allocation, file I/O, lock or retry loop is added.
//     - The existing 55 us bounded single-frame transport is reused.
//
// Probe set per slave:
//     1) 0x0130..0x0135 : AL Status + AL Status Code
//     2) 0x0110..0x0111 : ESC DL Status / physical + communication links
//     3) 0x0300..0x0313 : RX error counters + Lost Link counters
//     4) One 8-byte FPRD for EACH configured Runtime SyncManager:
//            0x0800 + SM * 8
//        returning Start Address, Length, Control, Status, Activate and
//        PDI-Control exactly as the ESC currently exposes them.
//     5) One 16-byte FPRD for EACH configured Runtime FMMU:
//            0x0600 + FMMU * 16
//        returning Logical Start/Length/Bits, Physical Start/Bit, Type and
//        Activate exactly as the ESC currently exposes them.
//     6) One bounded 67-byte FPRD per slave beginning at 0x0400:
//            0x0400:0x0401  Watchdog Divider
//            0x0420:0x0421  Watchdog Time Process Data
//            0x0440:0x0441  Watchdog Status Process Data
//            0x0442         Watchdog Counter Process Data
//        The read is diagnostic-only and never acknowledges/clears a counter.
//
// Scheduling note:
//     SM, FMMU and Watchdog probes extend the SAME low-rate diagnostic sweep;
//     they do not create another diagnostic timer or add a second diagnostic
//     frame to the same callback. One additional Watchdog probe is added per
//     slave, so the commissioning refresh remains low-rate and bounded.
//
// Existing Stage 12E.3A / 12E.4A / 12E.5A reader ABIs remain unchanged.
// Stage 12E.7A adds OSCARMAX_ECAT_WatchdogDiagRt_Read() for Priority-50 only.
// No Shared Memory ABI is changed in this file.
//
// Source fingerprints:
//     OSCARMAX_ESC_DIAG_RT_12E3A_20260826
//     OSCARMAX_SM_DIAG_RT_12E4A_20260826
//     OSCARMAX_FMMU_DIAG_RT_12E5A_20260826
//     OSCARMAX_WATCHDOG_DIAG_RT_12E7A_20260827
// ============================================================================

// DC-RX.4A monotonic correlation serial. It changes only when a low-rate ESC
// port/error-counter sample observes a raw counter change.
__declspec(align(8)) volatile LONGLONG
g_ecatRx4aEscPortChangeSerial = 0;

namespace
{
    constexpr uint32_t OSCARMAX_ECAT_ESC_DIAG_MAX_SLAVES = 128u;

    static_assert(
        ETHERCAT_RX4A_ESC_MAX_SLAVES ==
        OSCARMAX_ECAT_ESC_DIAG_MAX_SLAVES,
        "DC-RX.4A slave-capacity mismatch");
    constexpr uint32_t OSCARMAX_ECAT_ESC_DIAG_MAX_SYNC_MANAGERS = 16u;
    constexpr uint32_t OSCARMAX_ECAT_ESC_DIAG_MAX_FMMUS = 16u;
    constexpr uint64_t OSCARMAX_ECAT_ESC_DIAG_PROBE_INTERVAL_CYCLES = 80ULL;
    constexpr uint64_t OSCARMAX_ECAT_ESC_DIAG_HANDLER_BUDGET_GATE_NS = 120000ULL;
    constexpr uint64_t OSCARMAX_ECAT_ESC_DIAG_STEP_DEADLINE_NS = 55000ULL;

    constexpr uint16_t OSCARMAX_ECAT_ESC_REG_DL_STATUS = 0x0110u;
    constexpr uint16_t OSCARMAX_ECAT_ESC_REG_AL_STATUS = 0x0130u;
    constexpr uint16_t OSCARMAX_ECAT_ESC_REG_ERROR_COUNTERS = 0x0300u;
    constexpr uint16_t OSCARMAX_ECAT_ESC_REG_WATCHDOG_BASE = 0x0400u;
    constexpr uint16_t OSCARMAX_ECAT_ESC_REG_FMMU_BASE = 0x0600u;
    constexpr uint16_t OSCARMAX_ECAT_ESC_REG_SM_BASE = 0x0800u;

    constexpr uint32_t OSCARMAX_ECAT_ESC_DIAG_VALID_AL = 0x00000001u;
    constexpr uint32_t OSCARMAX_ECAT_ESC_DIAG_VALID_DL = 0x00000002u;
    constexpr uint32_t OSCARMAX_ECAT_ESC_DIAG_VALID_ERRORS = 0x00000004u;
    constexpr uint32_t OSCARMAX_ECAT_ESC_DIAG_VALID_WATCHDOG = 0x00000008u;

    struct OSCARMAX_ECAT_SmDiagRtEntry
    {
        uint32_t Present = 0u;
        uint32_t Valid = 0u;
        uint16_t RegisterAddress = 0u;
        uint16_t StartAddress = 0u;
        uint16_t Length = 0u;
        uint8_t ControlByte = 0u;
        uint8_t StatusByte = 0u;
        uint8_t ActivateByte = 0u;
        uint8_t PdiControlByte = 0u;
        int32_t LastTransportResult = 0;
        uint64_t LastUpdateTick = 0ULL;
        uint64_t ProbeSuccess = 0ULL;
        uint64_t ProbeFailure = 0ULL;
    };

    struct OSCARMAX_ECAT_FmmuDiagRtEntry
    {
        uint32_t Present = 0u;
        uint32_t Valid = 0u;
        uint16_t RegisterAddress = 0u;

        uint32_t LogicalStartAddress = 0u;
        uint16_t LogicalLength = 0u;
        uint8_t LogicalStartBit = 0u;
        uint8_t LogicalEndBit = 0u;

        uint16_t PhysicalStartAddress = 0u;
        uint8_t PhysicalStartBit = 0u;
        uint8_t Type = 0u;
        uint8_t Activate = 0u;

        int32_t LastTransportResult = 0;
        uint64_t LastUpdateTick = 0ULL;
        uint64_t ProbeSuccess = 0ULL;
        uint64_t ProbeFailure = 0ULL;
    };

    struct OSCARMAX_ECAT_EscDiagRtSlaveShadow
    {
        uint32_t ValidMask = 0u;
        uint16_t AlState = 0u;
        uint16_t AlStatusCode = 0u;
        uint16_t DlStatus = 0u;
        uint8_t PhysicalLinkMask = 0u;
        uint8_t CommunicationMask = 0u;
        uint32_t RxErrorCount = 0u;
        uint32_t LostLinkCount = 0u;

        // DC-RX.4A raw per-port counters, baseline/delta and saturation state.
        EtherCatEscPortErrorSnapshot Rx4aPortErrors{};

        // Stage 12E.7A - raw ESC Process Data Watchdog registers.
        uint16_t WatchdogDivider = 0u;           // 0x0400
        uint16_t WatchdogTimeProcessData = 0u;   // 0x0420
        uint16_t WatchdogStatusProcessData = 0u; // 0x0440
        uint8_t WatchdogCounterProcessData = 0u; // 0x0442
        uint8_t WatchdogReserved = 0u;

        int32_t LastTransportResult = 0;
        uint64_t LastUpdateTick = 0ULL;
        uint64_t ProbeSuccess = 0ULL;
        uint64_t ProbeFailure = 0ULL;

        uint32_t SyncManagerPresentMask = 0u;
        uint32_t SyncManagerValidMask = 0u;
        OSCARMAX_ECAT_SmDiagRtEntry
            SyncManagers[OSCARMAX_ECAT_ESC_DIAG_MAX_SYNC_MANAGERS] = {};

        uint32_t FmmuPresentMask = 0u;
        uint32_t FmmuValidMask = 0u;
        OSCARMAX_ECAT_FmmuDiagRtEntry
            Fmmus[OSCARMAX_ECAT_ESC_DIAG_MAX_FMMUS] = {};
    };

    struct OSCARMAX_ECAT_EscDiagRtShadow
    {
        volatile LONG Sequence = 0;
        uint32_t SlaveCount = 0u;
        uint32_t CurrentSlave = 0u;
        uint32_t CurrentField = 0u;
        uint64_t SweepCount = 0ULL;
        uint64_t ProbeAttempts = 0ULL;
        uint64_t ProbeSuccess = 0ULL;
        uint64_t ProbeFailure = 0ULL;
        uint64_t DeferredSdo = 0ULL;
        uint64_t DeferredPdo = 0ULL;
        uint64_t DeferredBudget = 0ULL;
        OSCARMAX_ECAT_EscDiagRtSlaveShadow Slaves[OSCARMAX_ECAT_ESC_DIAG_MAX_SLAVES] = {};
    };

    OSCARMAX_ECAT_EscDiagRtShadow g_ecatEscDiagRtShadow;

    bool Rx4aPortArrayChanged(
        const uint8_t* before,
        const uint8_t* after)
    {
        for (uint32_t port = 0u;
            port < ETHERCAT_RX4A_ESC_PORT_COUNT;
            ++port)
        {
            if (before[port] != after[port])
                return true;
        }

        return false;
    }

    bool Rx4aPortArrayDecreased(
        const uint8_t* before,
        const uint8_t* after)
    {
        for (uint32_t port = 0u;
            port < ETHERCAT_RX4A_ESC_PORT_COUNT;
            ++port)
        {
            if (after[port] < before[port])
                return true;
        }

        return false;
    }

    void Rx4aCopyPortArray(
        uint8_t* target,
        const uint8_t* source)
    {
        for (uint32_t port = 0u;
            port < ETHERCAT_RX4A_ESC_PORT_COUNT;
            ++port)
        {
            target[port] = source[port];
        }
    }

    uint32_t Rx4aBuildSaturationMask(
        const uint8_t* values)
    {
        uint32_t mask = 0u;

        for (uint32_t port = 0u;
            port < ETHERCAT_RX4A_ESC_PORT_COUNT;
            ++port)
        {
            if (values[port] == 0xFFu)
            {
                mask |= (1u << port);
            }
        }

        return mask;
    }

    void Rx4aUpdateEscPortErrorSnapshot(
        EtherCatEscPortErrorSnapshot& snapshot,
        uint32_t slavePosition,
        uint16_t configuredAddress,
        uint64_t sampleTick,
        const uint8_t* readBuffer)
    {
        uint8_t invalidFrame[ETHERCAT_RX4A_ESC_PORT_COUNT] = {};
        uint8_t physicalRx[ETHERCAT_RX4A_ESC_PORT_COUNT] = {};
        uint8_t forwardedRx[ETHERCAT_RX4A_ESC_PORT_COUNT] = {};
        uint8_t lostLink[ETHERCAT_RX4A_ESC_PORT_COUNT] = {};

        for (uint32_t port = 0u;
            port < ETHERCAT_RX4A_ESC_PORT_COUNT;
            ++port)
        {
            invalidFrame[port] = readBuffer[port * 2u];
            physicalRx[port] = readBuffer[port * 2u + 1u];
            forwardedRx[port] = readBuffer[8u + port];
            lostLink[port] = readBuffer[16u + port];
        }

        const uint8_t ecatProcessingUnitError = readBuffer[12u];
        const uint8_t pdiError = readBuffer[13u];

        const bool firstSample = snapshot.Valid == 0u;
        const bool rawChanged =
            firstSample ||
            Rx4aPortArrayChanged(snapshot.InvalidFrame, invalidFrame) ||
            Rx4aPortArrayChanged(snapshot.PhysicalRxError, physicalRx) ||
            Rx4aPortArrayChanged(snapshot.ForwardedRxError, forwardedRx) ||
            Rx4aPortArrayChanged(snapshot.LostLink, lostLink) ||
            snapshot.EcatProcessingUnitError != ecatProcessingUnitError ||
            snapshot.PdiError != pdiError;

        const bool rxGroupReset =
            !firstSample &&
            (Rx4aPortArrayDecreased(snapshot.InvalidFrame, invalidFrame) ||
                Rx4aPortArrayDecreased(snapshot.PhysicalRxError, physicalRx) ||
                Rx4aPortArrayDecreased(snapshot.ForwardedRxError, forwardedRx));

        const bool ecatProcessingUnitReset =
            !firstSample &&
            ecatProcessingUnitError < snapshot.EcatProcessingUnitError;

        const bool pdiReset =
            !firstSample &&
            pdiError < snapshot.PdiError;

        const bool lostLinkGroupReset =
            !firstSample &&
            Rx4aPortArrayDecreased(snapshot.LostLink, lostLink);

        snapshot.Valid = 1u;
        snapshot.SlavePosition = slavePosition;
        snapshot.ConfiguredAddress = configuredAddress;
        snapshot.SampleTick = sampleTick;
        snapshot.SampleCount++;

        if (firstSample)
        {
            snapshot.BaselineValid = 1u;
            snapshot.InitialBaselineTick = sampleTick;
            snapshot.RxBaselineTick = sampleTick;
            snapshot.EcatProcessingUnitBaselineTick = sampleTick;
            snapshot.PdiBaselineTick = sampleTick;
            snapshot.LostLinkBaselineTick = sampleTick;
            Rx4aCopyPortArray(snapshot.BaselineInvalidFrame, invalidFrame);
            Rx4aCopyPortArray(snapshot.BaselinePhysicalRxError, physicalRx);
            Rx4aCopyPortArray(snapshot.BaselineForwardedRxError, forwardedRx);
            Rx4aCopyPortArray(snapshot.BaselineLostLink, lostLink);
            snapshot.BaselineEcatProcessingUnitError =
                ecatProcessingUnitError;
            snapshot.BaselinePdiError = pdiError;
        }
        else
        {
            if (rxGroupReset)
            {
                snapshot.RxGroupResetCount++;
                snapshot.RxBaselineTick = sampleTick;
                Rx4aCopyPortArray(snapshot.BaselineInvalidFrame, invalidFrame);
                Rx4aCopyPortArray(snapshot.BaselinePhysicalRxError, physicalRx);
                Rx4aCopyPortArray(snapshot.BaselineForwardedRxError, forwardedRx);
            }

            if (ecatProcessingUnitReset)
            {
                snapshot.EcatProcessingUnitResetCount++;
                snapshot.EcatProcessingUnitBaselineTick = sampleTick;
                snapshot.BaselineEcatProcessingUnitError =
                    ecatProcessingUnitError;
            }

            if (pdiReset)
            {
                snapshot.PdiResetCount++;
                snapshot.PdiBaselineTick = sampleTick;
                snapshot.BaselinePdiError = pdiError;
            }

            if (lostLinkGroupReset)
            {
                snapshot.LostLinkGroupResetCount++;
                snapshot.LostLinkBaselineTick = sampleTick;
                Rx4aCopyPortArray(snapshot.BaselineLostLink, lostLink);
            }
        }

        Rx4aCopyPortArray(snapshot.InvalidFrame, invalidFrame);
        Rx4aCopyPortArray(snapshot.PhysicalRxError, physicalRx);
        Rx4aCopyPortArray(snapshot.ForwardedRxError, forwardedRx);
        Rx4aCopyPortArray(snapshot.LostLink, lostLink);
        snapshot.EcatProcessingUnitError = ecatProcessingUnitError;
        snapshot.PdiError = pdiError;

        for (uint32_t port = 0u;
            port < ETHERCAT_RX4A_ESC_PORT_COUNT;
            ++port)
        {
            snapshot.DeltaInvalidFrame[port] =
                static_cast<uint32_t>(invalidFrame[port]) -
                static_cast<uint32_t>(snapshot.BaselineInvalidFrame[port]);
            snapshot.DeltaPhysicalRxError[port] =
                static_cast<uint32_t>(physicalRx[port]) -
                static_cast<uint32_t>(snapshot.BaselinePhysicalRxError[port]);
            snapshot.DeltaForwardedRxError[port] =
                static_cast<uint32_t>(forwardedRx[port]) -
                static_cast<uint32_t>(snapshot.BaselineForwardedRxError[port]);
            snapshot.DeltaLostLink[port] =
                static_cast<uint32_t>(lostLink[port]) -
                static_cast<uint32_t>(snapshot.BaselineLostLink[port]);
        }

        snapshot.DeltaEcatProcessingUnitError =
            static_cast<uint32_t>(ecatProcessingUnitError) -
            static_cast<uint32_t>(
                snapshot.BaselineEcatProcessingUnitError);
        snapshot.DeltaPdiError =
            static_cast<uint32_t>(pdiError) -
            static_cast<uint32_t>(snapshot.BaselinePdiError);

        snapshot.SaturatedInvalidFrameMask =
            Rx4aBuildSaturationMask(invalidFrame);
        snapshot.SaturatedPhysicalRxErrorMask =
            Rx4aBuildSaturationMask(physicalRx);
        snapshot.SaturatedForwardedRxErrorMask =
            Rx4aBuildSaturationMask(forwardedRx);
        snapshot.SaturatedLostLinkMask =
            Rx4aBuildSaturationMask(lostLink);
        snapshot.SaturatedMiscMask =
            (ecatProcessingUnitError == 0xFFu ? 0x01u : 0u) |
            (pdiError == 0xFFu ? 0x02u : 0u);

        if (rawChanged)
        {
            snapshot.LastChangeTick = sampleTick;
            snapshot.ChangeSerial = static_cast<uint64_t>(
                InterlockedIncrement64(
                    &g_ecatRx4aEscPortChangeSerial));
        }
    }

    uint8_t DecodeEscCommunicationMask(uint16_t dlStatus)
    {
        uint8_t mask = 0u;

        if ((dlStatus & (1u << 9)) != 0u)  mask |= 0x01u;
        if ((dlStatus & (1u << 11)) != 0u) mask |= 0x02u;
        if ((dlStatus & (1u << 13)) != 0u) mask |= 0x04u;
        if ((dlStatus & (1u << 15)) != 0u) mask |= 0x08u;

        return mask;
    }

    uint32_t CountRuntimeEscDiagSyncManagers(
        const EtherCatSlave& slave)
    {
        uint32_t count = 0u;

        for (const auto& sm : slave.runtimeSyncManagers)
        {
            if (sm.index >= 0 &&
                sm.index < static_cast<int>(OSCARMAX_ECAT_ESC_DIAG_MAX_SYNC_MANAGERS))
            {
                count++;
            }
        }

        return count;
    }

    const EtherCatRuntimeSyncManagerConfig* FindRuntimeEscDiagSyncManagerByOrdinal(
        const EtherCatSlave& slave,
        uint32_t ordinal)
    {
        uint32_t current = 0u;

        for (const auto& sm : slave.runtimeSyncManagers)
        {
            if (sm.index < 0 ||
                sm.index >= static_cast<int>(OSCARMAX_ECAT_ESC_DIAG_MAX_SYNC_MANAGERS))
            {
                continue;
            }

            if (current == ordinal)
            {
                return &sm;
            }

            current++;
        }

        return nullptr;
    }

    uint32_t BuildRuntimeEscDiagSyncManagerPresentMask(
        const EtherCatSlave& slave)
    {
        uint32_t mask = 0u;

        for (const auto& sm : slave.runtimeSyncManagers)
        {
            if (sm.index >= 0 &&
                sm.index < static_cast<int>(OSCARMAX_ECAT_ESC_DIAG_MAX_SYNC_MANAGERS))
            {
                mask |= (1u << static_cast<uint32_t>(sm.index));
            }
        }

        return mask;
    }

    uint32_t CountRuntimeEscDiagFmmus(
        const EtherCatSlave& slave)
    {
        uint32_t count = 0u;

        for (const auto& fmmu : slave.runtimeFmmus)
        {
            if (fmmu.index >= 0 &&
                fmmu.index < static_cast<int>(OSCARMAX_ECAT_ESC_DIAG_MAX_FMMUS))
            {
                count++;
            }
        }

        return count;
    }

    const EtherCatRuntimeFmmuConfig* FindRuntimeEscDiagFmmuByOrdinal(
        const EtherCatSlave& slave,
        uint32_t ordinal)
    {
        uint32_t current = 0u;

        for (const auto& fmmu : slave.runtimeFmmus)
        {
            if (fmmu.index < 0 ||
                fmmu.index >= static_cast<int>(OSCARMAX_ECAT_ESC_DIAG_MAX_FMMUS))
            {
                continue;
            }

            if (current == ordinal)
            {
                return &fmmu;
            }

            current++;
        }

        return nullptr;
    }

    uint32_t BuildRuntimeEscDiagFmmuPresentMask(
        const EtherCatSlave& slave)
    {
        uint32_t mask = 0u;

        for (const auto& fmmu : slave.runtimeFmmus)
        {
            if (fmmu.index >= 0 &&
                fmmu.index < static_cast<int>(OSCARMAX_ECAT_ESC_DIAG_MAX_FMMUS))
            {
                mask |= (1u << static_cast<uint32_t>(fmmu.index));
            }
        }

        return mask;
    }

    void ProcessRuntimeEscDiagProbe(
        EtherCatMaster* pMaster,
        uint64_t pdoCycleStartMasterNs,
        bool processDataValid)
    {
        static uint32_t nextSlave = 0u;
        static uint32_t nextField = 0u;
        static uint64_t nextProbeTick = 0ULL;

        if (pMaster == nullptr || pMaster->m_pEni == nullptr)
        {
            return;
        }

        // Async commands execute on subTick 2. ESC diagnostic probes are
        // restricted to subTick 0 so the same callback never performs both.
        if ((pMaster->tickCount_PDO % 4ULL) != 0ULL)
        {
            return;
        }

        if (pMaster->tickCount_PDO < nextProbeTick)
        {
            return;
        }

        if (!processDataValid)
        {
            g_ecatEscDiagRtShadow.DeferredPdo++;
            nextProbeTick =
                pMaster->tickCount_PDO + OSCARMAX_ECAT_ESC_DIAG_PROBE_INTERVAL_CYCLES;
            return;
        }

        if (g_runtimeSafeSdo.active ||
            pMaster->m_asyncCmd.status == (int)EcatCmdStatus::ECAT_STATUS_PENDING)
        {
            g_ecatEscDiagRtShadow.DeferredSdo++;
            nextProbeTick =
                pMaster->tickCount_PDO + OSCARMAX_ECAT_ESC_DIAG_PROBE_INTERVAL_CYCLES;
            return;
        }

        const uint64_t nowNs = pMaster->GetCurrentMasterTimeNs();

        if (pdoCycleStartMasterNs != 0ULL &&
            nowNs >= pdoCycleStartMasterNs &&
            (nowNs - pdoCycleStartMasterNs) >=
            OSCARMAX_ECAT_ESC_DIAG_HANDLER_BUDGET_GATE_NS)
        {
            g_ecatEscDiagRtShadow.DeferredBudget++;
            nextProbeTick =
                pMaster->tickCount_PDO + OSCARMAX_ECAT_ESC_DIAG_PROBE_INTERVAL_CYCLES;
            return;
        }

        const auto& runtimeSlaves = pMaster->m_pEni->GetSlaves();
        uint32_t slaveCount = static_cast<uint32_t>(runtimeSlaves.size());

        if (slaveCount > OSCARMAX_ECAT_ESC_DIAG_MAX_SLAVES)
        {
            slaveCount = OSCARMAX_ECAT_ESC_DIAG_MAX_SLAVES;
        }

        if (slaveCount == 0u)
        {
            return;
        }

        if (nextSlave >= slaveCount)
        {
            nextSlave = 0u;
            nextField = 0u;
        }

        const EtherCatSlave& runtimeSlave =
            runtimeSlaves[static_cast<size_t>(nextSlave)];

        const uint32_t syncManagerProbeCount =
            CountRuntimeEscDiagSyncManagers(runtimeSlave);

        const uint32_t fmmuProbeCount =
            CountRuntimeEscDiagFmmus(runtimeSlave);

        const uint32_t fieldCount =
            4u +
            syncManagerProbeCount +
            fmmuProbeCount;

        if (nextField >= fieldCount)
        {
            nextField = 0u;
            nextSlave++;

            if (nextSlave >= slaveCount)
            {
                nextSlave = 0u;
                g_ecatEscDiagRtShadow.SweepCount++;
            }

            nextProbeTick =
                pMaster->tickCount_PDO + OSCARMAX_ECAT_ESC_DIAG_PROBE_INTERVAL_CYCLES;
            return;
        }

        const uint16_t configuredAddress =
            m_slaveInfo[nextSlave].configAddr;

        if (configuredAddress == 0u)
        {
            nextField++;
            if (nextField >= fieldCount)
            {
                nextField = 0u;
                nextSlave++;
                if (nextSlave >= slaveCount)
                {
                    nextSlave = 0u;
                    g_ecatEscDiagRtShadow.SweepCount++;
                }
            }

            nextProbeTick =
                pMaster->tickCount_PDO + OSCARMAX_ECAT_ESC_DIAG_PROBE_INTERVAL_CYCLES;
            return;
        }

        // Largest diagnostic request is the Stage 12E.7A Watchdog block:
        // 0x0400..0x0442 inclusive = 67 bytes.
        uint8_t readBuffer[68] = {};
        uint16_t registerAddress = 0u;
        uint16_t readLength = 0u;
        int selectedSmIndex = -1;
        int selectedFmmuIndex = -1;

        if (nextField == 0u)
        {
            registerAddress = OSCARMAX_ECAT_ESC_REG_AL_STATUS;
            readLength = 6u;
        }
        else if (nextField == 1u)
        {
            registerAddress = OSCARMAX_ECAT_ESC_REG_DL_STATUS;
            readLength = 2u;
        }
        else if (nextField == 2u)
        {
            registerAddress = OSCARMAX_ECAT_ESC_REG_ERROR_COUNTERS;
            readLength = 20u;
        }
        else if (nextField == 3u)
        {
            // One read captures all Process Data Watchdog registers needed
            // for diagnosis without a second frame in this callback:
            //   offset  0 = 0x0400 Watchdog Divider
            //   offset 32 = 0x0420 Watchdog Time Process Data
            //   offset 64 = 0x0440 Watchdog Status Process Data
            //   offset 66 = 0x0442 Watchdog Counter Process Data
            registerAddress = OSCARMAX_ECAT_ESC_REG_WATCHDOG_BASE;
            readLength = 67u;
        }
        else
        {
            const uint32_t configuredOrdinal =
                nextField - 4u;

            if (configuredOrdinal < syncManagerProbeCount)
            {
                const EtherCatRuntimeSyncManagerConfig* sm =
                    FindRuntimeEscDiagSyncManagerByOrdinal(
                        runtimeSlave,
                        configuredOrdinal);

                if (sm == nullptr ||
                    sm->index < 0 ||
                    sm->index >= static_cast<int>(OSCARMAX_ECAT_ESC_DIAG_MAX_SYNC_MANAGERS))
                {
                    nextField++;
                    nextProbeTick =
                        pMaster->tickCount_PDO + OSCARMAX_ECAT_ESC_DIAG_PROBE_INTERVAL_CYCLES;
                    return;
                }

                selectedSmIndex = sm->index;
                registerAddress = static_cast<uint16_t>(
                    OSCARMAX_ECAT_ESC_REG_SM_BASE +
                    static_cast<uint16_t>(selectedSmIndex * 8));
                readLength = 8u;
            }
            else
            {
                const uint32_t fmmuOrdinal =
                    configuredOrdinal -
                    syncManagerProbeCount;

                const EtherCatRuntimeFmmuConfig* fmmu =
                    FindRuntimeEscDiagFmmuByOrdinal(
                        runtimeSlave,
                        fmmuOrdinal);

                if (fmmu == nullptr ||
                    fmmu->index < 0 ||
                    fmmu->index >= static_cast<int>(OSCARMAX_ECAT_ESC_DIAG_MAX_FMMUS))
                {
                    nextField++;
                    nextProbeTick =
                        pMaster->tickCount_PDO + OSCARMAX_ECAT_ESC_DIAG_PROBE_INTERVAL_CYCLES;
                    return;
                }

                selectedFmmuIndex = fmmu->index;
                registerAddress = static_cast<uint16_t>(
                    OSCARMAX_ECAT_ESC_REG_FMMU_BASE +
                    static_cast<uint16_t>(selectedFmmuIndex * 16));
                readLength = 16u;
            }
        }

        const int transportResult =
            RuntimeSafeSdoFpTransaction(
                pMaster,
                0x04u, // FPRD
                configuredAddress,
                registerAddress,
                readLength,
                nullptr,
                readBuffer,
                OSCARMAX_ECAT_ESC_DIAG_STEP_DEADLINE_NS);

        InterlockedIncrement(&g_ecatEscDiagRtShadow.Sequence);
        MemoryBarrier();

        g_ecatEscDiagRtShadow.SlaveCount = slaveCount;
        g_ecatEscDiagRtShadow.CurrentSlave = nextSlave;
        g_ecatEscDiagRtShadow.CurrentField = nextField;
        g_ecatEscDiagRtShadow.ProbeAttempts++;

        OSCARMAX_ECAT_EscDiagRtSlaveShadow& target =
            g_ecatEscDiagRtShadow.Slaves[nextSlave];

        target.SyncManagerPresentMask =
            BuildRuntimeEscDiagSyncManagerPresentMask(runtimeSlave);

        target.FmmuPresentMask =
            BuildRuntimeEscDiagFmmuPresentMask(runtimeSlave);

        // If the Runtime schema is ever refreshed, do not retain validity
        // for an SM/FMMU that is no longer present in the configuration.
        target.SyncManagerValidMask &= target.SyncManagerPresentMask;
        target.FmmuValidMask &= target.FmmuPresentMask;

        target.LastTransportResult = transportResult;
        target.LastUpdateTick = pMaster->tickCount_PDO;

        if (selectedSmIndex >= 0)
        {
            OSCARMAX_ECAT_SmDiagRtEntry& smTarget =
                target.SyncManagers[static_cast<uint32_t>(selectedSmIndex)];

            smTarget.Present = 1u;
            smTarget.RegisterAddress = registerAddress;
            smTarget.LastTransportResult = transportResult;
            smTarget.LastUpdateTick = pMaster->tickCount_PDO;

            if (transportResult > 0)
            {
                smTarget.StartAddress =
                    static_cast<uint16_t>(readBuffer[0]) |
                    static_cast<uint16_t>(
                        static_cast<uint16_t>(readBuffer[1]) << 8);

                smTarget.Length =
                    static_cast<uint16_t>(readBuffer[2]) |
                    static_cast<uint16_t>(
                        static_cast<uint16_t>(readBuffer[3]) << 8);

                smTarget.ControlByte = readBuffer[4];
                smTarget.StatusByte = readBuffer[5];
                smTarget.ActivateByte = readBuffer[6];
                smTarget.PdiControlByte = readBuffer[7];
                smTarget.Valid = 1u;
                smTarget.ProbeSuccess++;

                target.SyncManagerValidMask |=
                    (1u << static_cast<uint32_t>(selectedSmIndex));
            }
            else
            {
                smTarget.Valid = 0u;
                smTarget.ProbeFailure++;
                target.SyncManagerValidMask &=
                    ~(1u << static_cast<uint32_t>(selectedSmIndex));
            }
        }
        else if (selectedFmmuIndex >= 0)
        {
            OSCARMAX_ECAT_FmmuDiagRtEntry& fmmuTarget =
                target.Fmmus[static_cast<uint32_t>(selectedFmmuIndex)];

            fmmuTarget.Present = 1u;
            fmmuTarget.RegisterAddress = registerAddress;
            fmmuTarget.LastTransportResult = transportResult;
            fmmuTarget.LastUpdateTick = pMaster->tickCount_PDO;

            if (transportResult > 0)
            {
                fmmuTarget.LogicalStartAddress =
                    static_cast<uint32_t>(readBuffer[0]) |
                    (static_cast<uint32_t>(readBuffer[1]) << 8) |
                    (static_cast<uint32_t>(readBuffer[2]) << 16) |
                    (static_cast<uint32_t>(readBuffer[3]) << 24);

                fmmuTarget.LogicalLength =
                    static_cast<uint16_t>(readBuffer[4]) |
                    static_cast<uint16_t>(
                        static_cast<uint16_t>(readBuffer[5]) << 8);

                fmmuTarget.LogicalStartBit = readBuffer[6];
                fmmuTarget.LogicalEndBit = readBuffer[7];

                fmmuTarget.PhysicalStartAddress =
                    static_cast<uint16_t>(readBuffer[8]) |
                    static_cast<uint16_t>(
                        static_cast<uint16_t>(readBuffer[9]) << 8);

                fmmuTarget.PhysicalStartBit = readBuffer[10];
                fmmuTarget.Type = readBuffer[11];
                fmmuTarget.Activate = readBuffer[12];

                fmmuTarget.Valid = 1u;
                fmmuTarget.ProbeSuccess++;

                target.FmmuValidMask |=
                    (1u << static_cast<uint32_t>(selectedFmmuIndex));
            }
            else
            {
                fmmuTarget.Valid = 0u;
                fmmuTarget.ProbeFailure++;

                target.FmmuValidMask &=
                    ~(1u << static_cast<uint32_t>(selectedFmmuIndex));
            }
        }

        if (transportResult > 0)
        {
            target.ProbeSuccess++;
            g_ecatEscDiagRtShadow.ProbeSuccess++;

            if (nextField == 0u)
            {
                target.AlState =
                    static_cast<uint16_t>(readBuffer[0]) |
                    static_cast<uint16_t>(
                        static_cast<uint16_t>(readBuffer[1]) << 8);

                target.AlStatusCode =
                    static_cast<uint16_t>(readBuffer[4]) |
                    static_cast<uint16_t>(
                        static_cast<uint16_t>(readBuffer[5]) << 8);

                target.ValidMask |= OSCARMAX_ECAT_ESC_DIAG_VALID_AL;
            }
            else if (nextField == 1u)
            {
                target.DlStatus =
                    static_cast<uint16_t>(readBuffer[0]) |
                    static_cast<uint16_t>(
                        static_cast<uint16_t>(readBuffer[1]) << 8);

                target.PhysicalLinkMask =
                    static_cast<uint8_t>((target.DlStatus >> 4) & 0x0Fu);

                target.CommunicationMask =
                    DecodeEscCommunicationMask(target.DlStatus);

                target.ValidMask |= OSCARMAX_ECAT_ESC_DIAG_VALID_DL;
            }
            else if (nextField == 2u)
            {
                uint32_t rxErrorTotal = 0u;
                uint32_t lostLinkTotal = 0u;

                // 0x0300..0x0307 contain the per-port CRC/RX error bytes.
                for (uint32_t i = 0u; i < 8u; ++i)
                {
                    rxErrorTotal += static_cast<uint32_t>(readBuffer[i]);
                }

                // 0x0310..0x0313 are one-byte Lost Link counters per port.
                for (uint32_t i = 16u; i < 20u; ++i)
                {
                    lostLinkTotal += static_cast<uint32_t>(readBuffer[i]);
                }

                target.RxErrorCount = rxErrorTotal;
                target.LostLinkCount = lostLinkTotal;

                Rx4aUpdateEscPortErrorSnapshot(
                    target.Rx4aPortErrors,
                    nextSlave,
                    configuredAddress,
                    pMaster->tickCount_PDO,
                    readBuffer);

                target.ValidMask |= OSCARMAX_ECAT_ESC_DIAG_VALID_ERRORS;
            }
            else if (nextField == 3u)
            {
                target.WatchdogDivider =
                    static_cast<uint16_t>(readBuffer[0]) |
                    static_cast<uint16_t>(
                        static_cast<uint16_t>(readBuffer[1]) << 8);

                target.WatchdogTimeProcessData =
                    static_cast<uint16_t>(readBuffer[32]) |
                    static_cast<uint16_t>(
                        static_cast<uint16_t>(readBuffer[33]) << 8);

                target.WatchdogStatusProcessData =
                    static_cast<uint16_t>(readBuffer[64]) |
                    static_cast<uint16_t>(
                        static_cast<uint16_t>(readBuffer[65]) << 8);

                target.WatchdogCounterProcessData =
                    readBuffer[66];

                target.ValidMask |=
                    OSCARMAX_ECAT_ESC_DIAG_VALID_WATCHDOG;
            }
        }
        else
        {
            target.ProbeFailure++;
            g_ecatEscDiagRtShadow.ProbeFailure++;
        }

        nextField++;
        if (nextField >= fieldCount)
        {
            nextField = 0u;
            nextSlave++;

            if (nextSlave >= slaveCount)
            {
                nextSlave = 0u;
                g_ecatEscDiagRtShadow.SweepCount++;
            }
        }

        g_ecatEscDiagRtShadow.CurrentSlave = nextSlave;
        g_ecatEscDiagRtShadow.CurrentField = nextField;

        MemoryBarrier();
        InterlockedIncrement(&g_ecatEscDiagRtShadow.Sequence);

        nextProbeTick =
            pMaster->tickCount_PDO + OSCARMAX_ECAT_ESC_DIAG_PROBE_INTERVAL_CYCLES;
    }
}

// ---------------------------------------------------------------------------
// DC-RX.4A Priority-50 readers. These copy the existing low-rate owner-safe
// shadow only; they never issue an EtherCAT frame and never clear ESC counters.
// ---------------------------------------------------------------------------
uint32_t EtherCatRx4aEscPortSlaveCount()
{
    for (int attempt = 0; attempt < 3; ++attempt)
    {
        const LONG sequenceBefore =
            g_ecatEscDiagRtShadow.Sequence;

        if ((sequenceBefore & 1L) != 0L)
            continue;

        MemoryBarrier();
        uint32_t count = g_ecatEscDiagRtShadow.SlaveCount;
        MemoryBarrier();

        const LONG sequenceAfter =
            g_ecatEscDiagRtShadow.Sequence;

        if (sequenceBefore == sequenceAfter &&
            (sequenceAfter & 1L) == 0L)
        {
            if (count > ETHERCAT_RX4A_ESC_MAX_SLAVES)
                count = ETHERCAT_RX4A_ESC_MAX_SLAVES;
            return count;
        }
    }

    return 0u;
}

bool EtherCatRx4aEscPortRead(
    uint16_t slavePosition,
    EtherCatEscPortErrorSnapshot* snapshot)
{
    if (snapshot == nullptr ||
        slavePosition >= ETHERCAT_RX4A_ESC_MAX_SLAVES)
    {
        return false;
    }

    for (int attempt = 0; attempt < 3; ++attempt)
    {
        const LONG sequenceBefore =
            g_ecatEscDiagRtShadow.Sequence;

        if ((sequenceBefore & 1L) != 0L)
            continue;

        MemoryBarrier();

        if (static_cast<uint32_t>(slavePosition) >=
            g_ecatEscDiagRtShadow.SlaveCount)
        {
            return false;
        }

        const EtherCatEscPortErrorSnapshot local =
            g_ecatEscDiagRtShadow.Slaves[slavePosition].Rx4aPortErrors;

        MemoryBarrier();

        const LONG sequenceAfter =
            g_ecatEscDiagRtShadow.Sequence;

        if (sequenceBefore == sequenceAfter &&
            (sequenceAfter & 1L) == 0L)
        {
            *snapshot = local;
            return local.Valid != 0u;
        }
    }

    return false;
}

// ---------------------------------------------------------------------------
// Priority-50 reader seam for one slave's most recent ESC diagnostic snapshot.
// Existing Stage 12E.3A ABI is intentionally unchanged.
// ---------------------------------------------------------------------------
extern "C" __declspec(dllexport) bool OSCARMAX_ECAT_EscDiagRt_Read(
    uint16_t slavePosition,
    uint32_t * validMask,
    uint16_t * alState,
    uint16_t * alStatusCode,
    uint16_t * dlStatus,
    uint8_t * physicalLinkMask,
    uint8_t * communicationMask,
    uint32_t * rxErrorCount,
    uint32_t * lostLinkCount,
    int32_t * lastTransportResult,
    uint64_t * lastUpdateTick,
    uint64_t * probeSuccess,
    uint64_t * probeFailure)
{
    if (validMask == nullptr ||
        alState == nullptr ||
        alStatusCode == nullptr ||
        dlStatus == nullptr ||
        physicalLinkMask == nullptr ||
        communicationMask == nullptr ||
        rxErrorCount == nullptr ||
        lostLinkCount == nullptr ||
        lastTransportResult == nullptr ||
        lastUpdateTick == nullptr ||
        probeSuccess == nullptr ||
        probeFailure == nullptr ||
        slavePosition >= OSCARMAX_ECAT_ESC_DIAG_MAX_SLAVES)
    {
        return false;
    }

    const LONG sequenceBegin = g_ecatEscDiagRtShadow.Sequence;

    if ((sequenceBegin & 1L) != 0L)
    {
        return false;
    }

    MemoryBarrier();

    if (static_cast<uint32_t>(slavePosition) >=
        g_ecatEscDiagRtShadow.SlaveCount)
    {
        return false;
    }

    const OSCARMAX_ECAT_EscDiagRtSlaveShadow local =
        g_ecatEscDiagRtShadow.Slaves[slavePosition];

    MemoryBarrier();

    const LONG sequenceEnd = g_ecatEscDiagRtShadow.Sequence;

    if (sequenceBegin != sequenceEnd ||
        (sequenceEnd & 1L) != 0L)
    {
        return false;
    }

    *validMask = local.ValidMask;
    *alState = local.AlState;
    *alStatusCode = local.AlStatusCode;
    *dlStatus = local.DlStatus;
    *physicalLinkMask = local.PhysicalLinkMask;
    *communicationMask = local.CommunicationMask;
    *rxErrorCount = local.RxErrorCount;
    *lostLinkCount = local.LostLinkCount;
    *lastTransportResult = local.LastTransportResult;
    *lastUpdateTick = local.LastUpdateTick;
    *probeSuccess = local.ProbeSuccess;
    *probeFailure = local.ProbeFailure;

    return true;
}

// ---------------------------------------------------------------------------
// Stage 12E.7A - Priority-50 reader seam for ESC Process Data Watchdog.
//
// Raw live values:
//   0x0400:0x0401  Watchdog Divider
//   0x0420:0x0421  Watchdog Time Process Data
//   0x0440:0x0441  Watchdog Status Process Data
//   0x0442         Watchdog Counter Process Data
//
// enabledSmMask is derived from the most recent live SM Control bytes.
// Bit N = SyncManager N has ControlByte[6] Watchdog Trigger Enable set.
//
// No register is written or acknowledged by this path.
// ---------------------------------------------------------------------------
extern "C" __declspec(dllexport) bool OSCARMAX_ECAT_WatchdogDiagRt_Read(
    uint16_t slavePosition,
    uint32_t * valid,
    uint16_t * watchdogDivider,
    uint16_t * watchdogTimeProcessData,
    uint16_t * watchdogStatusProcessData,
    uint8_t * watchdogCounterProcessData,
    uint16_t * enabledSmMask,
    int32_t * lastTransportResult,
    uint64_t * lastUpdateTick)
{
    if (valid == nullptr ||
        watchdogDivider == nullptr ||
        watchdogTimeProcessData == nullptr ||
        watchdogStatusProcessData == nullptr ||
        watchdogCounterProcessData == nullptr ||
        enabledSmMask == nullptr ||
        lastTransportResult == nullptr ||
        lastUpdateTick == nullptr ||
        slavePosition >= OSCARMAX_ECAT_ESC_DIAG_MAX_SLAVES)
    {
        return false;
    }

    const LONG sequenceBegin =
        g_ecatEscDiagRtShadow.Sequence;

    if ((sequenceBegin & 1L) != 0L)
    {
        return false;
    }

    MemoryBarrier();

    if (static_cast<uint32_t>(slavePosition) >=
        g_ecatEscDiagRtShadow.SlaveCount)
    {
        return false;
    }

    const OSCARMAX_ECAT_EscDiagRtSlaveShadow local =
        g_ecatEscDiagRtShadow.Slaves[slavePosition];

    MemoryBarrier();

    const LONG sequenceEnd =
        g_ecatEscDiagRtShadow.Sequence;

    if (sequenceBegin != sequenceEnd ||
        (sequenceEnd & 1L) != 0L)
    {
        return false;
    }

    uint16_t localEnabledSmMask = 0u;

    for (uint32_t smIndex = 0u;
        smIndex < OSCARMAX_ECAT_ESC_DIAG_MAX_SYNC_MANAGERS;
        ++smIndex)
    {
        const OSCARMAX_ECAT_SmDiagRtEntry& sm =
            local.SyncManagers[smIndex];

        if (sm.Present != 0u &&
            sm.Valid != 0u &&
            (sm.ControlByte & 0x40u) != 0u)
        {
            localEnabledSmMask |=
                static_cast<uint16_t>(1u << smIndex);
        }
    }

    *valid =
        ((local.ValidMask &
            OSCARMAX_ECAT_ESC_DIAG_VALID_WATCHDOG) != 0u)
        ? 1u
        : 0u;

    *watchdogDivider =
        local.WatchdogDivider;

    *watchdogTimeProcessData =
        local.WatchdogTimeProcessData;

    *watchdogStatusProcessData =
        local.WatchdogStatusProcessData;

    *watchdogCounterProcessData =
        local.WatchdogCounterProcessData;

    *enabledSmMask =
        localEnabledSmMask;

    *lastTransportResult =
        local.LastTransportResult;

    *lastUpdateTick =
        local.LastUpdateTick;

    return true;
}


// ---------------------------------------------------------------------------
// Stage 12E.4A - Priority-50 reader seam for one live SyncManager register.
//
// smIndex is the physical ESC SyncManager number (0..15), not an ordinal in
// the Runtime XML vector. The call is lock-free and wait-free; read collision
// simply returns false and the 10 ms publisher retries on its next pass.
// ---------------------------------------------------------------------------
extern "C" __declspec(dllexport) bool OSCARMAX_ECAT_SmDiagRt_Read(
    uint16_t slavePosition,
    uint8_t smIndex,
    uint32_t * present,
    uint32_t * valid,
    uint16_t * registerAddress,
    uint16_t * startAddress,
    uint16_t * length,
    uint8_t * controlByte,
    uint8_t * statusByte,
    uint8_t * activateByte,
    uint8_t * pdiControlByte,
    int32_t * lastTransportResult,
    uint64_t * lastUpdateTick,
    uint64_t * probeSuccess,
    uint64_t * probeFailure)
{
    if (present == nullptr ||
        valid == nullptr ||
        registerAddress == nullptr ||
        startAddress == nullptr ||
        length == nullptr ||
        controlByte == nullptr ||
        statusByte == nullptr ||
        activateByte == nullptr ||
        pdiControlByte == nullptr ||
        lastTransportResult == nullptr ||
        lastUpdateTick == nullptr ||
        probeSuccess == nullptr ||
        probeFailure == nullptr ||
        slavePosition >= OSCARMAX_ECAT_ESC_DIAG_MAX_SLAVES ||
        smIndex >= OSCARMAX_ECAT_ESC_DIAG_MAX_SYNC_MANAGERS)
    {
        return false;
    }

    const LONG sequenceBegin = g_ecatEscDiagRtShadow.Sequence;

    if ((sequenceBegin & 1L) != 0L)
    {
        return false;
    }

    MemoryBarrier();

    if (static_cast<uint32_t>(slavePosition) >=
        g_ecatEscDiagRtShadow.SlaveCount)
    {
        return false;
    }

    const OSCARMAX_ECAT_EscDiagRtSlaveShadow& slave =
        g_ecatEscDiagRtShadow.Slaves[slavePosition];

    const OSCARMAX_ECAT_SmDiagRtEntry local =
        slave.SyncManagers[smIndex];

    const uint32_t presentMaskBit =
        (slave.SyncManagerPresentMask >> smIndex) & 0x01u;

    const uint32_t validMaskBit =
        (slave.SyncManagerValidMask >> smIndex) & 0x01u;

    MemoryBarrier();

    const LONG sequenceEnd = g_ecatEscDiagRtShadow.Sequence;

    if (sequenceBegin != sequenceEnd ||
        (sequenceEnd & 1L) != 0L)
    {
        return false;
    }

    *present = presentMaskBit != 0u ? 1u : 0u;
    *valid = (validMaskBit != 0u && local.Valid != 0u) ? 1u : 0u;
    *registerAddress = local.RegisterAddress;
    *startAddress = local.StartAddress;
    *length = local.Length;
    *controlByte = local.ControlByte;
    *statusByte = local.StatusByte;
    *activateByte = local.ActivateByte;
    *pdiControlByte = local.PdiControlByte;
    *lastTransportResult = local.LastTransportResult;
    *lastUpdateTick = local.LastUpdateTick;
    *probeSuccess = local.ProbeSuccess;
    *probeFailure = local.ProbeFailure;

    return true;
}

// ---------------------------------------------------------------------------
// Stage 12E.5A - Priority-50 reader seam for one live FMMU register.
//
// fmmuIndex is the physical ESC FMMU number (0..15).  This accessor is
// lock-free and wait-free and follows the same RT shadow seqlock as AL/DL/SM.
// ---------------------------------------------------------------------------
extern "C" __declspec(dllexport) bool OSCARMAX_ECAT_FmmuDiagRt_Read(
    uint16_t slavePosition,
    uint8_t fmmuIndex,
    uint32_t * present,
    uint32_t * valid,
    uint16_t * registerAddress,
    uint32_t * logicalStartAddress,
    uint16_t * logicalLength,
    uint8_t * logicalStartBit,
    uint8_t * logicalEndBit,
    uint16_t * physicalStartAddress,
    uint8_t * physicalStartBit,
    uint8_t * type,
    uint8_t * activate,
    int32_t * lastTransportResult,
    uint64_t * lastUpdateTick,
    uint64_t * probeSuccess,
    uint64_t * probeFailure)
{
    if (present == nullptr ||
        valid == nullptr ||
        registerAddress == nullptr ||
        logicalStartAddress == nullptr ||
        logicalLength == nullptr ||
        logicalStartBit == nullptr ||
        logicalEndBit == nullptr ||
        physicalStartAddress == nullptr ||
        physicalStartBit == nullptr ||
        type == nullptr ||
        activate == nullptr ||
        lastTransportResult == nullptr ||
        lastUpdateTick == nullptr ||
        probeSuccess == nullptr ||
        probeFailure == nullptr ||
        slavePosition >= OSCARMAX_ECAT_ESC_DIAG_MAX_SLAVES ||
        fmmuIndex >= OSCARMAX_ECAT_ESC_DIAG_MAX_FMMUS)
    {
        return false;
    }

    const LONG sequenceBegin =
        g_ecatEscDiagRtShadow.Sequence;

    if ((sequenceBegin & 1L) != 0L)
    {
        return false;
    }

    MemoryBarrier();

    if (static_cast<uint32_t>(slavePosition) >=
        g_ecatEscDiagRtShadow.SlaveCount)
    {
        return false;
    }

    const OSCARMAX_ECAT_EscDiagRtSlaveShadow& slave =
        g_ecatEscDiagRtShadow.Slaves[slavePosition];

    const OSCARMAX_ECAT_FmmuDiagRtEntry local =
        slave.Fmmus[fmmuIndex];

    const uint32_t presentMaskBit =
        (slave.FmmuPresentMask >> fmmuIndex) & 0x01u;

    const uint32_t validMaskBit =
        (slave.FmmuValidMask >> fmmuIndex) & 0x01u;

    MemoryBarrier();

    const LONG sequenceEnd =
        g_ecatEscDiagRtShadow.Sequence;

    if (sequenceBegin != sequenceEnd ||
        (sequenceEnd & 1L) != 0L)
    {
        return false;
    }

    *present =
        presentMaskBit != 0u ? 1u : 0u;

    *valid =
        (validMaskBit != 0u &&
            local.Valid != 0u)
        ? 1u
        : 0u;

    *registerAddress = local.RegisterAddress;
    *logicalStartAddress = local.LogicalStartAddress;
    *logicalLength = local.LogicalLength;
    *logicalStartBit = local.LogicalStartBit;
    *logicalEndBit = local.LogicalEndBit;
    *physicalStartAddress = local.PhysicalStartAddress;
    *physicalStartBit = local.PhysicalStartBit;
    *type = local.Type;
    *activate = local.Activate;
    *lastTransportResult = local.LastTransportResult;
    *lastUpdateTick = local.LastUpdateTick;
    *probeSuccess = local.ProbeSuccess;
    *probeFailure = local.ProbeFailure;

    return true;
}

// =============================================================
// 啟動 Drift 設定與校正快照
//
// Startup 只在正式 PDO timer 建立前寫 Config 欄位。Runtime 以 Robust Drift
// 的連續五個合格視窗鎖定 AUTO Baseline；鎖定後本次執行不再重校。
// =============================================================

volatile LONG g_dcDriftConfigReady = 0;
volatile LONG g_dcDriftConfiguredMode = 0;
volatile LONGLONG g_dcDriftConfiguredFixedPpb =
EtherCatDcTuning::SchedulerBootstrapDriftPpb;

volatile LONG g_dcDriftCalibrationDiagSequence = 0;
volatile LONG g_dcDriftCalibrationMode = 0;
volatile LONG g_dcDriftCalibrationState = 0;
volatile LONG g_dcDriftCalibrationCandidateGood = 0;
volatile LONG g_dcDriftCalibrationGoodWindows = 0;
volatile LONG g_dcDriftCalibrationRequiredWindows =
(LONG)EtherCatDcTuning::DriftCalibrationGoodWindows;
volatile LONG g_dcDriftCalibrationRobustSequence = 0;
volatile LONGLONG g_dcDriftCalibrationRawPpb = 0;
volatile LONGLONG g_dcDriftCalibrationMedianPpb = 0;
volatile LONGLONG g_dcDriftCalibrationMadPpb = 0;
volatile LONGLONG g_dcDriftCalibrationRawMedianDeviationPpb = 0;
volatile LONGLONG g_dcDriftCalibrationBaselinePpb =
EtherCatDcTuning::SchedulerBootstrapDriftPpb;
volatile LONG g_dcDriftCalibrationLockCount = 0;

// =============================================================
// DC-RX.3F test-only immutable startup config
// =============================================================
volatile LONG g_dcRx3fConfigReady = 0;
volatile LONG g_dcRx3fConfiguredScenario = 0;
volatile LONG g_dcRx3fConfiguredCycles = 0;
volatile LONG g_dcRx3fConfiguredStartDelayCycles = 0;
volatile LONGLONG g_dcRx3fConfiguredValueNs = 0;
volatile LONG g_dcRx3fConfiguredRequireServoOff = 1;
volatile LONG g_dcRx3fConfiguredAllowSafetyStop = 0;

// =============================================================
// PDO 即時診斷快照
//
// Writer：Priority 64 PDO Handler
// Reader：Priority 50 Main Thread
//
// Timer 欄位描述 callback 間隔；Combined 描述 LRW+FRMW round trip；
// Exec 描述整個 Handler 執行時間；WKC 保存本窗最後一次通訊結果。
// Handler 只寫數字，不在此處輸出文字。
// =============================================================

volatile LONG g_pdoRtDiagSequence = 0;


// Callback 間隔統計；Short < 200 us、Normal 200..300 us、Long > 300 us。
volatile LONGLONG g_pdoRtTimerAvgNs = 0;
volatile LONGLONG g_pdoRtTimerMinNs = 0;
volatile LONGLONG g_pdoRtTimerMaxNs = 0;

volatile LONG g_pdoRtTimerShortCount = 0;
volatile LONG g_pdoRtTimerNormalCount = 0;
volatile LONG g_pdoRtTimerLongCount = 0;


// LRW + FRMW 合併 frame 的通訊耗時統計。
volatile LONGLONG g_pdoRtCombinedAvgNs = 0;
volatile LONGLONG g_pdoRtCombinedMinNs = 0;
volatile LONGLONG g_pdoRtCombinedMaxNs = 0;


// 完整 PDO Handler 執行時間與超過 250/300/400 us 的次數。
volatile LONGLONG g_pdoRtExecAvgNs = 0;
volatile LONGLONG g_pdoRtExecMinNs = 0;
volatile LONGLONG g_pdoRtExecMaxNs = 0;

volatile LONG g_pdoRtExecOver250Count = 0;
volatile LONG g_pdoRtExecOver300Count = 0;
volatile LONG g_pdoRtExecOver400Count = 0;


// LRW PDO WKC 與 FRMW DC WKC。DC-RX.3A 起，Process Data 與 DC sample
// 使用獨立有效性；DC-only 抖動不再把正常 LRW/PDO 判成 invalid。
volatile LONG g_pdoRtLrwWkc = 0;
volatile LONG g_pdoRtDcWkc = 0;
volatile LONG g_pdoRtDcReferencePresent = 0;
volatile LONG g_pdoRtProcessDataValid = 0;
volatile LONG g_pdoRtDcTransportValid = 0;
volatile LONG g_pdoRtProcessInvalidStreak = 0;
volatile LONG g_pdoRtDcTransportInvalidStreak = 0;
volatile LONG g_pdoRtDcTransportInvalidMaxStreak = 0;
volatile LONGLONG g_pdoRtDcWkcInvalidTotal = 0;
volatile LONGLONG g_pdoRtDcOnlyInvalidTotal = 0;
volatile LONGLONG g_pdoRtDcTransportRecoveryTotal = 0;

// DC-RX.3B - bounded DC-only holdover / graded requalification.
// These are internal diagnostic snapshots only; SHM/API layout is unchanged.
volatile LONG g_pdoRtDcOnlyInvalidStreak = 0;
volatile LONG g_pdoRtDcOnlyInvalidMaxStreak = 0;
volatile LONG g_pdoRtDcHoldoverTier = 0;
volatile LONG g_pdoRtDcHoldoverCurrentCycles = 0;
volatile LONG g_pdoRtDcHoldoverMaxCycles = 0;
volatile LONG g_pdoRtDcGlitchDebt = 0;
volatile LONG g_pdoRtDcGlitchDebtMax = 0;
volatile LONG g_pdoRtDcGlitchDebtLimit = 16;
volatile LONGLONG g_pdoRtDcGraceAcceptedCyclesTotal = 0;
volatile LONGLONG g_pdoRtDcHoldoverEpisodeTotal = 0;
volatile LONGLONG g_pdoRtDcHoldoverEntryTotal = 0;
volatile LONGLONG g_pdoRtDcDegradedEntryTotal = 0;
volatile LONGLONG g_pdoRtDcRelockEntryTotal = 0;

// DC-RX.3C - DC sample freshness / age / phase jump guards.
// Internal diagnostic snapshot only; SHM/API layout remains unchanged.
volatile LONG g_pdoRtDcSampleGuardState = 0;
volatile LONG g_pdoRtDcSampleQualified = 0;
volatile LONG g_pdoRtDcSampleGuardReasonMask = 0;
volatile LONG g_pdoRtDcSampleRejectStreak = 0;
volatile LONG g_pdoRtDcSampleRejectMaxStreak = 0;
volatile LONGLONG g_pdoRtDcSampleApproxAgeNs = 0;
volatile LONGLONG g_pdoRtDcSampleApproxAgeMaxNs = 0;
volatile LONGLONG g_pdoRtDcSampleQpcDeltaNs = 0;
volatile LONGLONG g_pdoRtDcSampleDcDeltaNs = 0;
volatile LONGLONG g_pdoRtDcSampleDeltaErrorNs = 0;
volatile LONGLONG g_pdoRtDcSampleDeltaToleranceNs = 0;
volatile LONGLONG g_pdoRtDcSampleAcceptedTotal = 0;
volatile LONGLONG g_pdoRtDcSampleRejectedTotal = 0;
volatile LONGLONG g_pdoRtDcSampleAnchorTotal = 0;
volatile LONGLONG g_pdoRtDcSampleReanchorTotal = 0;
volatile LONGLONG g_pdoRtDcSampleAgeRejectTotal = 0;
volatile LONGLONG g_pdoRtDcSampleOrderRejectTotal = 0;
volatile LONGLONG g_pdoRtDcSampleDeltaRejectTotal = 0;
volatile LONG g_pdoRtDcPhaseJumpGuardActive = 0;
volatile LONG g_pdoRtDcPhaseJumpGuardGoodWindows = 3;
volatile LONG g_pdoRtDcPhaseJumpGuardRequiredWindows = 3;
volatile LONGLONG g_pdoRtDcPhaseJumpLastNs = 0;
volatile LONGLONG g_pdoRtDcPhaseJumpMaxAbsNs = 0;
volatile LONGLONG g_pdoRtDcPhaseJumpArmTotal = 0;
volatile LONGLONG g_pdoRtDcPhaseJumpPassTotal = 0;
volatile LONGLONG g_pdoRtDcPhaseJumpRejectTotal = 0;
volatile LONG g_pdoRtDcPhaseMapSequence = 0;
volatile LONG g_pdoRtDcPhaseMapNew = 0;
volatile LONG g_pdoRtDcPhaseMapAgeGood = 0;
volatile LONGLONG g_pdoRtDcPhaseMapAgeNs = 0;
volatile LONGLONG g_pdoRtDcPhaseMapStaleTotal = 0;

// DC-RX.3D - exact TX/RX software timing anchor diagnostics.
// Internal snapshot only; SHM/API layout remains unchanged.
volatile LONG g_pdoRtDcTimingSource = 0;
volatile LONG g_pdoRtDcExactTimingLocked = 0;
volatile LONG g_pdoRtDcExactTimingValid = 0;
volatile LONGLONG g_pdoRtDcTimingSelectedRttNs = 0;
volatile LONGLONG g_pdoRtDcTimingExactRttNs = 0;
volatile LONGLONG g_pdoRtDcTimingExactRttMaxNs = 0;
volatile LONGLONG g_pdoRtDcTimingCallRttNs = 0;
volatile LONGLONG g_pdoRtDcTimingExcludedOverheadNs = 0;
volatile LONGLONG g_pdoRtDcTimingMidpointShiftNs = 0;
volatile LONGLONG g_pdoRtDcTimingMidpointShiftMaxAbsNs = 0;
volatile LONGLONG g_pdoRtDcTimingExactUseTotal = 0;
volatile LONGLONG g_pdoRtDcTimingFallbackUseTotal = 0;
volatile LONGLONG g_pdoRtDcTimingMissingAfterLockTotal = 0;
volatile LONGLONG g_pdoRtDcTimingSourceSwitchTotal = 0;
volatile LONGLONG g_pdoRtDcTimingRejectTotal = 0;

// DC-RX.3E - adaptive exact-RTT envelope / late-sample quarantine.
// Internal snapshot only; SHM/API layout remains unchanged.
volatile LONG g_pdoRtDcRttGuardState = 0;
volatile LONG g_pdoRtDcRttGuardAccepted = 0;
volatile LONG g_pdoRtDcRttGuardWarmupSamples = 0;
volatile LONG g_pdoRtDcRttGuardWarmupRequired = 128;
volatile LONGLONG g_pdoRtDcRttGuardCurrentNs = 0;
volatile LONGLONG g_pdoRtDcRttGuardBaselineNs = 0;
volatile LONGLONG g_pdoRtDcRttGuardDeviationNs = 0;
volatile LONGLONG g_pdoRtDcRttGuardLimitNs = 0;
volatile LONGLONG g_pdoRtDcRttGuardExcessNs = 0;
volatile LONG g_pdoRtDcRttGuardOutlierStreak = 0;
volatile LONG g_pdoRtDcRttGuardOutlierMaxStreak = 0;
volatile LONG g_pdoRtDcRttGuardRebaseCandidateSamples = 0;
volatile LONGLONG g_pdoRtDcRttGuardAcceptedTotal = 0;
volatile LONGLONG g_pdoRtDcRttGuardRejectedTotal = 0;
volatile LONGLONG g_pdoRtDcRttGuardRebaseTotal = 0;
volatile LONGLONG g_pdoRtDcSampleRttRejectTotal = 0;

// DC-RX.3F deterministic test-only fault-injection snapshot.
// Published inside the existing PDO diagnostic seqlock.
volatile LONG g_pdoRtDcFaultState = 0;
volatile LONG g_pdoRtDcFaultScenario = 0;
volatile LONG g_pdoRtDcFaultConfiguredCycles = 0;
volatile LONG g_pdoRtDcFaultAppliedCycles = 0;
volatile LONG g_pdoRtDcFaultStartDelayRemaining = 0;
volatile LONG g_pdoRtDcFaultRecoveryCycles = 0;
volatile LONG g_pdoRtDcFaultTargetWaitCycles = 0;
volatile LONG g_pdoRtDcFaultActiveThisCycle = 0;
volatile LONG g_pdoRtDcFaultGateBlockMask = 0;
volatile LONG g_pdoRtDcFaultEvidenceMask = 0;
volatile LONG g_pdoRtDcFaultFailureMask = 0;
volatile LONG g_pdoRtDcFaultRequireServoOff = 1;
volatile LONG g_pdoRtDcFaultAllowSafetyStop = 0;
volatile LONGLONG g_pdoRtDcFaultValueNs = 0;
volatile LONGLONG g_pdoRtDcFaultBaselineAppliedPpb = 0;
volatile LONGLONG g_pdoRtDcFaultCurrentAppliedPpb = 0;
volatile LONGLONG g_pdoRtDcFaultMaxAppliedDeltaPpb = 0;
volatile LONG g_pdoRtDcFaultMaxPdoInvalidStreak = 0;
volatile LONGLONG g_pdoRtDcFaultStartTick = 0;
volatile LONGLONG g_pdoRtDcFaultLastAppliedTick = 0;
volatile LONGLONG g_pdoRtDcFaultEndTick = 0;


// =============================================================
// QPC <-> EtherCAT DC Reference 頻率估測快照（僅診斷）
//
// 用 EtherCAT 呼叫前後 QPC 中點對應 DC Reference time，降低固定通訊延遲的影響。
// Delta = DC elapsed - QPC elapsed；DriftPpb 是長時間斜率，不是單次相位誤差。
// RTT 過大或時間倒退的樣本會列入 rejected，不會餵入觀測器。
// =============================================================

volatile LONG
g_qpcDcDiagSequence =
0;

volatile LONGLONG
g_qpcDcQpcElapsedNs =
0;

volatile LONGLONG
g_qpcDcDcElapsedNs =
0;

volatile LONGLONG
g_qpcDcDeltaNs =
0;

volatile LONGLONG
g_qpcDcDriftPpb =
0;

volatile LONGLONG
g_qpcDcRttAvgNs =
0;

volatile LONGLONG
g_qpcDcRttMinNs =
0;

volatile LONGLONG
g_qpcDcRttMaxNs =
0;

volatile LONG
g_qpcDcValidSamples =
0;

volatile LONG
g_qpcDcRejectedSamples =
0;

volatile LONG
g_qpcDcValid =
0;
volatile LONG g_pdoRecoveryForensicSeq = 0;
volatile LONG g_pdoRecoveryForensicCaptured = 0;
volatile LONGLONG g_pdoRecoveryForensicWakeIntervalNs = 0;
volatile LONGLONG g_pdoRecoveryForensicLeadAtWakeNs = 0;
volatile LONGLONG g_pdoRecoveryForensicWakeToArmNs = 0;
volatile LONGLONG g_pdoRecoveryForensicLeadAtArmNs = 0;
volatile LONGLONG g_pdoRecoveryForensicLeadAfterNs = 0;
volatile LONG g_pdoRecoveryForensicSkipCycles = 0;

// =============================================================
// PDO Fine Scheduler Timing Foundation V1 快照（僅量測，Fine Wait 未啟用）
//
// Interval：QPC callback 間隔；WakeToEcat：callback 到 EtherCAT 呼叫前的延遲；
// QpcRead：連續 QPC 讀取成本。這些欄位供 Main Thread 判斷排程抖動來源。
// =============================================================

volatile LONG
g_pdoFineDiagSequence =
0;

volatile LONGLONG
g_pdoFineQpcFrequency =
0;

volatile LONGLONG
g_pdoFineIntervalAvgNs =
0;

volatile LONGLONG
g_pdoFineIntervalMinNs =
0;

volatile LONGLONG
g_pdoFineIntervalMaxNs =
0;

volatile LONGLONG
g_pdoFineWakeToEcatAvgNs =
0;

volatile LONGLONG
g_pdoFineWakeToEcatMinNs =
0;

volatile LONGLONG
g_pdoFineWakeToEcatMaxNs =
0;

volatile LONGLONG
g_pdoFineQpcReadAvgNs =
0;

volatile LONGLONG
g_pdoFineQpcReadMaxNs =
0;

volatile LONG
g_pdoFineQpcValid =
0;


// =============================================================
// QPC Scheduler Re-Anchor + Guard V1B 快照
//
// 此快照同時記錄排程模型誤差；真正的 timer re-arm 在後方 One-Shot 區塊執行。
// 本區本身不 busy wait、不修改 HAL，也不直接送 EtherCAT frame。
// =============================================================

volatile LONG
g_qpcSchedulerDiagSequence =
0;

volatile LONGLONG
g_qpcSchedulerTargetQpc =
0;

volatile LONGLONG
g_qpcSchedulerActualQpc =
0;

volatile LONGLONG
g_qpcSchedulerErrorCounts =
0;

volatile LONGLONG
g_qpcSchedulerErrorNs =
0;

volatile LONGLONG
g_qpcSchedulerWindowStartErrorNs =
0;

volatile LONGLONG
g_qpcSchedulerWindowEndErrorNs =
0;

volatile LONGLONG
g_qpcSchedulerWindowDeltaErrorNs =
0;

volatile LONGLONG
g_qpcSchedulerWindowMinErrorNs =
0;

volatile LONGLONG
g_qpcSchedulerWindowMaxErrorNs =
0;

volatile LONGLONG
g_qpcSchedulerPeriodWholeCounts =
0;

volatile LONGLONG
g_qpcSchedulerPeriodFractionScaled =
0;

volatile LONGLONG
g_qpcSchedulerAssumedDriftPpb =
0;

volatile LONG
g_qpcSchedulerWindowSamples =
0;

volatile LONG
g_qpcSchedulerValid =
0;

// ============================================================================
// Real FF V0 與 Phase-P V0 正式控制快照
//
// Real FF 狀態：0=WAIT、1=ARM、2=ACTIVE、3=HOLD、4=TRIP/FALLBACK。
//   - ACTIVE 時把合格的 Frequency FF V2 建議值，限幅與限速後套入 QPC period。
//   - TripMask 0x01 表示相位觀測器進入 FALLBACK；0x04/0x08 才是 hard trip。
//   - 單次 PDO/DC 無效與 Scheduler runtime recovery 不再寫入 TripMask；它們只會
//     進入可恢復 HOLD，並保持最後一個合格 Applied FF，避免跳回 Startup Baseline。
//   - 週期控制只使用上一個 PDO callback 的即時品質，不再使用約一秒才發布一次的
//     RX 診斷快照，避免已恢復的 RX 被過時值持續判定為故障。
//
// Phase-P 狀態：0=WAIT、1=ARM、2=ACTIVE、3=HOLD、4=TRIP。
//   - 只有 Real FF ACTIVE、phase gate 合格、TripMask=0 時才修正 offset。
//   - transient HOLD 保留既有 offset；觀測器重新合格後再由 HOLD 平順恢復。
//   - ActualErr/Offset/Step 都是 ns；offset 會真正加到 One-Shot final target。
//
// DC-RX.1 fingerprint:
//   OSCARMAX_DC_RX1_TRANSIENT_HOLD_LAST_GOOD_FF_20260903
// ============================================================================
volatile LONG g_qpcRealFfV0Seq = 0;
volatile LONG g_qpcRealFfV0State = 0;
volatile LONG g_qpcRealFfV0PhaseGood = 0;
volatile LONG g_qpcRealFfV0ArmGood = 0;
volatile LONG g_qpcRealFfV0HoldMask = 0;
volatile LONG g_qpcRealFfV0HistoryMask = 0;
volatile LONG g_qpcRealFfV0CleanCycles = 0;
volatile LONG g_qpcRealFfV0CleanCyclesRequired = 32;
volatile LONG g_qpcRealFfV0TripMask = 0;
volatile LONG g_qpcRealFfV0TripCount = 0;
volatile LONGLONG g_qpcRealFfV0RecommendedPpb = EtherCatDcTuning::SchedulerBootstrapDriftPpb;
volatile LONGLONG g_qpcRealFfV0DesiredPpb = EtherCatDcTuning::SchedulerBootstrapDriftPpb;
volatile LONGLONG g_qpcRealFfV0AppliedPpb = EtherCatDcTuning::SchedulerBootstrapDriftPpb;
volatile LONGLONG g_qpcRealFfV0LastStepPpb = 0;
volatile LONGLONG g_qpcRealFfV0TargetVsFixedNs = 0;
volatile LONGLONG g_qpcRealFfV0DcErrEstNs = 0;
volatile LONG g_qpcRealFfV0PhaseSeq = 0;
volatile LONG g_qpcRealFfV0PhaseRejectMask = 0;
volatile LONG g_qpcRealFfV0LastPhaseRejectMask = 0;
volatile LONG g_qpcRealFfV0HoldGood = 0;
volatile LONG g_qpcRealFfV0HoldBad = 0;
volatile LONG g_qpcRealFfV0HoldEntries = 0;
volatile LONG g_qpcRealFfV0ClampActive = 0;
volatile LONG g_qpcRealFfV0RecoveryProfile = 0;
volatile LONG g_qpcRealFfV0HoldRecoveryWindowsRequired = 3;

volatile LONG g_qpcRealFfClampSelfTestSeq = 0;
volatile LONG g_qpcRealFfClampSelfTestPass = 0;
volatile LONG g_qpcRealFfClampSelfTestCases = 0;
volatile LONG g_qpcRealFfClampSelfTestFail = 0;
volatile LONGLONG g_qpcRealFfClampSelfTestRec0 = 0;
volatile LONGLONG g_qpcRealFfClampSelfTestDesired0 = 0;
volatile LONG g_qpcRealFfClampSelfTestClamp0 = 0;
volatile LONGLONG g_qpcRealFfClampSelfTestRec1 = 0;
volatile LONGLONG g_qpcRealFfClampSelfTestDesired1 = 0;
volatile LONG g_qpcRealFfClampSelfTestClamp1 = 0;
volatile LONGLONG g_qpcRealFfClampSelfTestRec2 = 0;
volatile LONGLONG g_qpcRealFfClampSelfTestDesired2 = 0;
volatile LONG g_qpcRealFfClampSelfTestClamp2 = 0;
volatile LONGLONG g_qpcRealFfClampSelfTestRec3 = 0;
volatile LONGLONG g_qpcRealFfClampSelfTestDesired3 = 0;
volatile LONG g_qpcRealFfClampSelfTestClamp3 = 0;
volatile LONGLONG g_qpcRealFfClampSelfTestRec4 = 0;
volatile LONGLONG g_qpcRealFfClampSelfTestDesired4 = 0;
volatile LONG g_qpcRealFfClampSelfTestClamp4 = 0;

volatile LONG g_qpcPhasePActV0Seq = 0;
volatile LONG g_qpcPhasePActV0State = 0;
volatile LONG g_qpcPhasePActV0GateGood = 0;
volatile LONG g_qpcPhasePActV0ArmGood = 0;
volatile LONGLONG g_qpcPhasePActV0BaseErrNs = 0;
volatile LONGLONG g_qpcPhasePActV0ActualErrNs = 0;
volatile LONGLONG g_qpcPhasePActV0WrappedErrNs = 0;
volatile LONGLONG g_qpcPhasePActV0RawCorrectionNs = 0;
volatile LONGLONG g_qpcPhasePActV0CommandNs = 0;
volatile LONGLONG g_qpcPhasePActV0StepNs = 0;
volatile LONGLONG g_qpcPhasePActV0OffsetNs = 0;
volatile LONGLONG g_qpcPhasePActV0PredictedErrNs = 0;
volatile LONG g_qpcPhasePActV0CommandSat = 0;
volatile LONG g_qpcPhasePActV0OffsetSat = 0;
volatile LONG g_qpcPhasePActV0Improve = 0;
volatile LONG g_qpcPhasePActV0HoldGood = 0;
volatile LONG g_qpcPhasePActV0HoldEntries = 0;
volatile LONG g_qpcPhasePActV0TripCount = 0;
volatile LONG g_qpcPhasePActV0RealFfState = 0;


// V1B 排程狀態補充：
//   0=WAIT_STABLE，尚未取得穩定 QPC/DC 基準。
//   1=EARLY，距目標仍超過 10 us。
//   2=NEAR，位於目標前 0..10 us。
//   3=LATE，callback 已晚於目標。

volatile LONG
g_qpcSchedulerState =
0;

volatile LONGLONG
g_qpcSchedulerGuardNs =
0;

volatile LONGLONG
g_qpcSchedulerRemainingToTargetNs =
0;

volatile LONGLONG
g_qpcSchedulerLateByNs =
0;

volatile LONGLONG
g_qpcSchedulerAnchorQpc =
0;

volatile LONG
g_qpcSchedulerAnchorDcDiagSequence =
0;

volatile LONG
g_qpcSchedulerStableWaitCycles =
0;

volatile LONG
g_qpcSchedulerEarlyCount =
0;

volatile LONG
g_qpcSchedulerNearCount =
0;

volatile LONG
g_qpcSchedulerLateCount =
0;


// =============================================================
// QPC <-> DC Reference Robust Drift V1 快照（僅觀測）
//
// 對最近 9 個約一秒的 drift 樣本計算 median 與 MAD；CurrentAccepted 表示
// 本窗通過 RTT／幅度門檻，Locked 表示樣本數與 MAD 已達可信條件。
// 此觀測器不改 QPC period、PDO timer、Fine Wait 或 HAL。
// =============================================================

volatile LONG
g_qpcDcRobustDiagSequence =
0;

volatile LONGLONG
g_qpcDcRobustRawDriftPpb =
0;

volatile LONGLONG
g_qpcDcRobustMedianDriftPpb =
0;

volatile LONGLONG
g_qpcDcRobustMadPpb =
0;

volatile LONGLONG
g_qpcDcRobustPeriodFfPs =
0;

volatile LONG
g_qpcDcRobustBufferCount =
0;

volatile LONG
g_qpcDcRobustCurrentAccepted =
0;

volatile LONG
g_qpcDcRobustLocked =
0;

volatile LONG
g_qpcDcRobustAcceptedTotal =
0;

volatile LONG
g_qpcDcRobustRejectedTotal =
0;


// =============================================================
// QPC <-> DC Reference Trusted Drift V1A 快照（僅觀測）
//
// 狀態：0=WARMUP、1=TRACK、2=HOLD、3=UNTRUSTED。
// WARMUP 需連續合格窗；TRACK 以受限 slew 更新；HOLD 保留最後可信值；
// UNTRUSTED 回到啟動 Baseline 並等待重新鎖定。本觀測器不直接修改 timer/HAL。
// =============================================================

volatile LONG
g_qpcDcTrustedDiagSequence =
0;

volatile LONGLONG
g_qpcDcTrustedRawDriftPpb =
0;

volatile LONGLONG
g_qpcDcTrustedRobustMedianPpb =
0;

volatile LONGLONG
g_qpcDcTrustedRobustMadPpb =
0;

volatile LONGLONG
g_qpcDcTrustedDriftPpb =
EtherCatDcTuning::SchedulerBootstrapDriftPpb;

volatile LONGLONG
g_qpcDcTrustedCandidateDeviationPpb =
0;

volatile LONGLONG
g_qpcDcTrustedRawMedianDeviationPpb =
0;

volatile LONGLONG
g_qpcDcTrustedSlewAppliedPpb =
0;

volatile LONGLONG
g_qpcDcTrustedPeriodFfPs =
0;

volatile LONG
g_qpcDcTrustedCandidateGood =
0;

volatile LONG
g_qpcDcTrustedValid =
0;

volatile LONG
g_qpcDcTrustedState =
0;

volatile LONG
g_qpcDcTrustedWarmupGoodCount =
0;

volatile LONG
g_qpcDcTrustedBadCount =
0;

volatile LONG
g_qpcDcTrustedRecoveryGoodCount =
0;

volatile LONG
g_qpcDcTrustedUpdateTotal =
0;

volatile LONG
g_qpcDcTrustedHoldTotal =
0;

volatile LONG
g_qpcDcTrustedUnlockTotal =
0;

volatile LONG
g_qpcDcTrustedRelockTotal =
0;


// =============================================================
// Trusted Drift V1A 拒絕原因與狀態轉移快照（僅診斷）
//
// RejectMask：0x01=本窗未接受、0x02=Robust 未鎖定、0x04=Buffer 未滿、
// 0x08=MAD 過大、0x10=候選值偏離安全基準、0x20=Raw/Median 差異過大。
// Current 是本窗原因；Last 保存最近一次失敗原因；Total 欄位是累積計數。
// =============================================================

volatile LONG
g_qpcDcTrustedReasonDiagSequence =
0;

volatile LONG
g_qpcDcTrustedRejectRobustAcceptTotal =
0;

volatile LONG
g_qpcDcTrustedRejectRobustLockTotal =
0;

volatile LONG
g_qpcDcTrustedRejectBufferNotFullTotal =
0;

volatile LONG
g_qpcDcTrustedRejectMadTotal =
0;

volatile LONG
g_qpcDcTrustedRejectCandidateDeviationTotal =
0;

volatile LONG
g_qpcDcTrustedRejectRawMedianDeviationTotal =
0;

volatile LONG
g_qpcDcTrustedCurrentRejectMask =
0;

volatile LONG
g_qpcDcTrustedLastRejectMask =
0;

volatile LONG
g_qpcDcTrustedMaxBadStreak =
0;

volatile LONG
g_qpcDcTrustedWarmupToTrackTotal =
0;

volatile LONG
g_qpcDcTrustedTrackToHoldTotal =
0;

volatile LONG
g_qpcDcTrustedHoldToTrackTotal =
0;

volatile LONG
g_qpcDcTrustedHoldToUntrustedTotal =
0;

volatile LONG
g_qpcDcTrustedUntrustedToTrackTotal =
0;

volatile LONG
g_qpcDcTrustedLastTransitionFrom =
-1;

volatile LONG
g_qpcDcTrustedLastTransitionTo =
-1;

volatile LONGLONG
g_qpcDcTrustedLastTransitionRawPpb =
0;

volatile LONGLONG
g_qpcDcTrustedLastTransitionMedianPpb =
0;

volatile LONGLONG
g_qpcDcTrustedLastTransitionMadPpb =
0;

volatile LONGLONG
g_qpcDcTrustedLastTransitionTrustedPpb =
EtherCatDcTuning::SchedulerBootstrapDriftPpb;


// =============================================================
// Trusted Drift -> QPC Live Feed-Forward V1 Shadow 快照
//
// 使用獨立 target accumulator，比較 Trusted 與本次啟動 Baseline 的長期差異。
// Mode 0=FIXED_FALLBACK，1=TRUSTED。此 timeline 不寫入真正 scheduler target，
// 不重算既有 anchor，也不改 timer re-arm 或 PDO 送出時間。
// =============================================================

volatile LONG
g_qpcLiveFfDiagSequence =
0;

volatile LONG
g_qpcLiveFfInitialized =
0;

volatile LONG
g_qpcLiveFfMode =
0;

volatile LONG
g_qpcLiveFfTrustedSnapshotValid =
0;

volatile LONG
g_qpcLiveFfTrustedValid =
0;

volatile LONG
g_qpcLiveFfTrustedState =
0;

volatile LONGLONG
g_qpcLiveFfTrustedDriftPpb =
EtherCatDcTuning::SchedulerBootstrapDriftPpb;

volatile LONGLONG
g_qpcLiveFfAppliedDriftPpb =
EtherCatDcTuning::SchedulerBootstrapDriftPpb;

volatile LONGLONG
g_qpcLiveFfAppliedPeriodFfPs =
EtherCatDcTuning::SchedulerBootstrapPeriodFfPs;

volatile LONGLONG
g_qpcLiveFfFixedErrorNs =
0;

volatile LONGLONG
g_qpcLiveFfShadowErrorNs =
0;

volatile LONGLONG
g_qpcLiveFfShadowVsFixedTargetNs =
0;

volatile LONGLONG
g_qpcLiveFfShadowWindowStartErrorNs =
0;

volatile LONGLONG
g_qpcLiveFfShadowWindowEndErrorNs =
0;

volatile LONGLONG
g_qpcLiveFfShadowWindowDeltaErrorNs =
0;

volatile LONGLONG
g_qpcLiveFfShadowWindowMinErrorNs =
0;

volatile LONGLONG
g_qpcLiveFfShadowWindowMaxErrorNs =
0;

volatile LONGLONG
g_qpcLiveFfTargetDeltaWindowStartNs =
0;

volatile LONGLONG
g_qpcLiveFfTargetDeltaWindowEndNs =
0;

volatile LONGLONG
g_qpcLiveFfTargetDeltaWindowDeltaNs =
0;

volatile LONG
g_qpcLiveFfTrustedCyclesWindow =
0;

volatile LONG
g_qpcLiveFfFallbackCyclesWindow =
0;

volatile LONG
g_qpcLiveFfModeSwitchesWindow =
0;

volatile LONGLONG
g_qpcLiveFfTotalTrustedCycles =
0;

volatile LONGLONG
g_qpcLiveFfTotalFallbackCycles =
0;

volatile LONG
g_qpcLiveFfTotalModeSwitches =
0;

volatile LONG
g_qpcLiveFfInitCount =
0;

volatile LONG
g_qpcLiveFfSamples =
0;


// =============================================================
// Trusted Live-FF DC Phase Predictor V1 快照（僅診斷）
//
// 把 Fixed target 與 Trusted shadow target 都投影到 DC Reference 時域，比較 wrapped／
// unwrapped phase、Sync0 margin、平均絕對誤差與優劣次數。對映使用當前有效的
// QPC midpoint / DC_reference_time pair；結果不直接驅動 timer。
// =============================================================

volatile LONG
g_qpcLiveFfDcPhaseDiagSequence =
0;

// QPC midpoint of the newest sample included in this coherent phase snapshot.
// DC-RX.3C uses it only to reject stale Phase-P publications.
volatile LONGLONG
g_qpcLiveFfDcPhaseLastSampleQpc =
0;

volatile LONG
g_qpcLiveFfDcPhaseInitialized =
0;

volatile LONG
g_qpcLiveFfDcPhaseBoundInitCount =
0;

volatile LONGLONG
g_qpcLiveFfDcPhaseBaselinePhaseNs =
0;

volatile LONGLONG
g_qpcLiveFfDcPhaseBaselineSync0MarginNs =
0;

volatile LONGLONG
g_qpcLiveFfDcPhaseFixedPhaseNs =
0;

volatile LONGLONG
g_qpcLiveFfDcPhaseShadowPhaseNs =
0;

volatile LONGLONG
g_qpcLiveFfDcPhaseFixedSync0MarginNs =
0;

volatile LONGLONG
g_qpcLiveFfDcPhaseShadowSync0MarginNs =
0;

volatile LONGLONG
g_qpcLiveFfDcPhaseFixedWrappedErrorNs =
0;

volatile LONGLONG
g_qpcLiveFfDcPhaseShadowWrappedErrorNs =
0;

volatile LONGLONG
g_qpcLiveFfDcPhaseFixedUnwrappedErrorNs =
0;

volatile LONGLONG
g_qpcLiveFfDcPhaseShadowUnwrappedErrorNs =
0;

volatile LONGLONG
g_qpcLiveFfDcPhaseShadowVsFixedNs =
0;

volatile LONGLONG
g_qpcLiveFfDcPhaseFixedWindowStartNs =
0;

volatile LONGLONG
g_qpcLiveFfDcPhaseFixedWindowEndNs =
0;

volatile LONGLONG
g_qpcLiveFfDcPhaseFixedWindowDeltaNs =
0;

volatile LONGLONG
g_qpcLiveFfDcPhaseFixedWindowMinNs =
0;

volatile LONGLONG
g_qpcLiveFfDcPhaseFixedWindowMaxNs =
0;

volatile LONGLONG
g_qpcLiveFfDcPhaseShadowWindowStartNs =
0;

volatile LONGLONG
g_qpcLiveFfDcPhaseShadowWindowEndNs =
0;

volatile LONGLONG
g_qpcLiveFfDcPhaseShadowWindowDeltaNs =
0;

volatile LONGLONG
g_qpcLiveFfDcPhaseShadowWindowMinNs =
0;

volatile LONGLONG
g_qpcLiveFfDcPhaseShadowWindowMaxNs =
0;

volatile LONGLONG
g_qpcLiveFfDcPhaseFixedAbsAvgNs =
0;

volatile LONGLONG
g_qpcLiveFfDcPhaseShadowAbsAvgNs =
0;

volatile LONG
g_qpcLiveFfDcPhaseShadowBetterCount =
0;

volatile LONG
g_qpcLiveFfDcPhaseShadowWorseCount =
0;

volatile LONG
g_qpcLiveFfDcPhaseEqualCount =
0;

volatile LONGLONG
g_qpcLiveFfDcPhaseMapRttAvgNs =
0;

volatile LONGLONG
g_qpcLiveFfDcPhaseMapRttMinNs =
0;

volatile LONGLONG
g_qpcLiveFfDcPhaseMapRttMaxNs =
0;

volatile LONG
g_qpcLiveFfDcPhaseSamples =
0;


// =============================================================
// DC Reference Phase Residual Drift Observer V1 快照（僅診斷）
//
// 以本次啟動 Baseline timeline 的相位在約一秒內的變化估算 residual drift；
// RobustResidualPpb 是通過門檻樣本的 rolling median；建議排程值為
// Baseline - RobustResidualPpb。此 V1 結果不直接控制 scheduler。
// =============================================================

volatile LONG
g_qpcDcPhaseResidualDiagSequence =
0;

volatile LONG
g_qpcDcPhaseResidualInitialized =
0;

volatile LONG
g_qpcDcPhaseResidualBoundInitCount =
0;

volatile LONGLONG
g_qpcDcPhaseResidualRawPpb =
0;

volatile LONGLONG
g_qpcDcPhaseResidualMedianPpb =
0;

volatile LONGLONG
g_qpcDcPhaseResidualMadPpb =
0;

volatile LONGLONG
g_qpcDcPhaseResidualRecommendedSchedulerPpb =
EtherCatDcTuning::SchedulerBootstrapDriftPpb;

volatile LONGLONG
g_qpcDcPhaseResidualTrustedDriftPpb =
EtherCatDcTuning::SchedulerBootstrapDriftPpb;

volatile LONGLONG
g_qpcDcPhaseResidualTrustedMinusRecommendedPpb =
0;

volatile LONGLONG
g_qpcDcPhaseResidualFixedWindowDeltaNs =
0;

volatile LONGLONG
g_qpcDcPhaseResidualWindowElapsedNs =
0;

volatile LONGLONG
g_qpcDcPhaseResidualWindowRttMaxNs =
0;

volatile LONG
g_qpcDcPhaseResidualCurrentAccepted =
0;

volatile LONG
g_qpcDcPhaseResidualBufferCount =
0;

volatile LONG
g_qpcDcPhaseResidualLocked =
0;

volatile LONG
g_qpcDcPhaseResidualAcceptedTotal =
0;

volatile LONG
g_qpcDcPhaseResidualRejectedTotal =
0;

volatile LONG
g_qpcDcPhaseResidualRejectElapsedTotal =
0;

volatile LONG
g_qpcDcPhaseResidualRejectRttTotal =
0;

volatile LONG
g_qpcDcPhaseResidualRejectMagnitudeTotal =
0;

volatile LONGLONG
g_qpcDcPhaseResidualRingMinPpb =
0;

volatile LONGLONG
g_qpcDcPhaseResidualRingMaxPpb =
0;


// =============================================================
// DC Reference Phase Residual Drift Observer V1A 快照（僅診斷）
//
// 每個約一秒／4000 sample 視窗先產生一個平均相位點，再以最近 16 個合格點
// 的所有 pair slope（最多 120 組）計算 Theil-Sen median 與 slope MAD。
// Locked 需同時滿足點數、時間跨度與 MAD 門檻；結果供 Frequency FF V2 判斷。
// =============================================================

volatile LONG
g_qpcDcPhaseResidualV1ADiagSequence =
0;

volatile LONG
g_qpcDcPhaseResidualV1AInitialized =
0;

volatile LONG
g_qpcDcPhaseResidualV1ABoundInitCount =
0;

volatile LONGLONG
g_qpcDcPhaseResidualV1AMeanPhaseNs =
0;

volatile LONGLONG
g_qpcDcPhaseResidualV1APhaseSpanNs =
0;

volatile LONGLONG
g_qpcDcPhaseResidualV1APointTimeNs =
0;

volatile LONG
g_qpcDcPhaseResidualV1APointAccepted =
0;

volatile LONG
g_qpcDcPhaseResidualV1AMeanSamples =
0;

volatile LONG
g_qpcDcPhaseResidualV1APointBufferCount =
0;

volatile LONG
g_qpcDcPhaseResidualV1APairSlopeCount =
0;

volatile LONGLONG
g_qpcDcPhaseResidualV1ATheilSenPpb =
0;

volatile LONGLONG
g_qpcDcPhaseResidualV1ASlopeMadPpb =
0;

volatile LONGLONG
g_qpcDcPhaseResidualV1ATimeSpanNs =
0;

volatile LONG
g_qpcDcPhaseResidualV1ALocked =
0;

volatile LONGLONG
g_qpcDcPhaseResidualV1ARecommendedSchedulerPpb =
EtherCatDcTuning::SchedulerBootstrapDriftPpb;

volatile LONGLONG
g_qpcDcPhaseResidualV1ATrustedDriftPpb =
EtherCatDcTuning::SchedulerBootstrapDriftPpb;

volatile LONGLONG
g_qpcDcPhaseResidualV1ATrustedMinusRecommendedPpb =
0;

volatile LONGLONG
g_qpcDcPhaseResidualV1AWindowElapsedNs =
0;

volatile LONGLONG
g_qpcDcPhaseResidualV1AWindowRttMaxNs =
0;

volatile LONG
g_qpcDcPhaseResidualV1AAcceptedPointTotal =
0;

volatile LONG
g_qpcDcPhaseResidualV1ARejectedPointTotal =
0;

volatile LONG
g_qpcDcPhaseResidualV1ARejectSamplesTotal =
0;

volatile LONG
g_qpcDcPhaseResidualV1ARejectElapsedTotal =
0;

volatile LONG
g_qpcDcPhaseResidualV1ARejectRttTotal =
0;

volatile LONG
g_qpcDcPhaseResidualV1ARejectSpanTotal =
0;

volatile LONGLONG
g_qpcDcPhaseResidualV1ASlopeMinPpb =
0;

volatile LONGLONG
g_qpcDcPhaseResidualV1ASlopeMaxPpb =
0;

volatile LONG g_qpcV1aCostSeq = 0;
volatile LONGLONG g_qpcV1aCostLastNs = 0;
volatile LONGLONG g_qpcV1aCostAvgNs = 0;
volatile LONGLONG g_qpcV1aCostMaxNs = 0;
volatile LONG g_qpcV1aCostEvents = 0;
volatile LONG g_qpcV1aCostOver20us = 0;
volatile LONG g_qpcV1aCostOver40us = 0;
volatile LONG g_qpcV1aCostOver80us = 0;
volatile LONG g_qpcV1aCostQpcFail = 0;


// =============================================================
// Frequency FF V2 觀測器快照
//
// 狀態：0=WARMUP、1=TRACK、2=HOLD、3=FALLBACK。
// 以 V1A phase slope 做嚴格資格判斷，再以慢速 slew 產生建議值。
// V2 的 target 本身是 shadow；但其合格 snapshot 會被 Real FF V0 讀取，
// 經 Real FF 的額外 gate、限幅與限速後才可能影響真正 scheduler。
// =============================================================

volatile LONG
g_qpcPhaseFfV2DiagSequence =
0;

volatile LONG
g_qpcPhaseFfV2Initialized =
0;

volatile LONG
g_qpcPhaseFfV2State =
0;

volatile LONG
g_qpcPhaseFfV2CandidateGood =
0;

volatile LONG
g_qpcPhaseFfV2WarmupGoodCount =
0;

volatile LONG
g_qpcPhaseFfV2BadCount =
0;

volatile LONG
g_qpcPhaseFfV2RecoveryGoodCount =
0;

volatile LONG
g_qpcPhaseFfV2ObserverSequence =
0;

volatile LONGLONG
g_qpcPhaseFfV2RecommendedPpb =
EtherCatDcTuning::SchedulerBootstrapDriftPpb;

volatile LONGLONG
g_qpcPhaseFfV2DesiredPpb =
EtherCatDcTuning::SchedulerBootstrapDriftPpb;

volatile LONGLONG
g_qpcPhaseFfV2AppliedPpb =
EtherCatDcTuning::SchedulerBootstrapDriftPpb;

volatile LONGLONG
g_qpcPhaseFfV2SlewAppliedPpb =
0;

volatile LONGLONG
g_qpcPhaseFfV2SlopeMadPpb =
0;

volatile LONG
g_qpcPhaseFfV2Points =
0;

volatile LONG
g_qpcPhaseFfV2Pairs =
0;

volatile LONG
g_qpcPhaseFfV2ObserverLocked =
0;

volatile LONG
g_qpcPhaseFfV2PointAccepted =
0;

volatile LONGLONG
g_qpcPhaseFfV2TargetVsFixedNs =
0;

volatile LONGLONG
g_qpcPhaseFfV2WakeErrorNs =
0;

volatile LONGLONG
g_qpcPhaseFfV2TargetDeltaWindowStartNs =
0;

volatile LONGLONG
g_qpcPhaseFfV2TargetDeltaWindowEndNs =
0;

volatile LONGLONG
g_qpcPhaseFfV2TargetDeltaWindowDeltaNs =
0;

volatile LONG
g_qpcPhaseFfV2TrackCyclesWindow =
0;

volatile LONG
g_qpcPhaseFfV2HoldCyclesWindow =
0;

volatile LONG
g_qpcPhaseFfV2FallbackCyclesWindow =
0;

volatile LONG
g_qpcPhaseFfV2StateSwitchesWindow =
0;

volatile LONG
g_qpcPhaseFfV2TotalStateSwitches =
0;

volatile LONG
g_qpcPhaseFfV2WarmupToTrackTotal =
0;

volatile LONG
g_qpcPhaseFfV2TrackToHoldTotal =
0;

volatile LONG
g_qpcPhaseFfV2HoldToTrackTotal =
0;

volatile LONG
g_qpcPhaseFfV2HoldToFallbackTotal =
0;

volatile LONG
g_qpcPhaseFfV2FallbackToTrackTotal =
0;

volatile LONG
g_qpcPhaseFfV2Samples =
0;


// =============================================================
// Frequency FF V2 與 DC Reference 相位比較快照（V2 shadow 效果驗證）
// =============================================================

volatile LONG
g_qpcPhaseFfV2DcDiagSequence =
0;

volatile LONG
g_qpcPhaseFfV2DcInitialized =
0;

volatile LONGLONG
g_qpcPhaseFfV2DcPhaseNs =
0;

volatile LONGLONG
g_qpcPhaseFfV2DcSync0MarginNs =
0;

volatile LONGLONG
g_qpcPhaseFfV2DcUnwrappedErrorNs =
0;

volatile LONGLONG
g_qpcPhaseFfV2DcVsFixedNs =
0;

volatile LONGLONG
g_qpcPhaseFfV2DcVsTrustedNs =
0;

volatile LONGLONG
g_qpcPhaseFfV2DcWindowStartNs =
0;

volatile LONGLONG
g_qpcPhaseFfV2DcWindowEndNs =
0;

volatile LONGLONG
g_qpcPhaseFfV2DcWindowDeltaNs =
0;

volatile LONGLONG
g_qpcPhaseFfV2DcWindowMinNs =
0;

volatile LONGLONG
g_qpcPhaseFfV2DcWindowMaxNs =
0;

volatile LONGLONG
g_qpcPhaseFfV2DcAbsAvgNs =
0;

volatile LONG
g_qpcPhaseFfV2DcBetterThanFixed =
0;

volatile LONG
g_qpcPhaseFfV2DcBetterThanTrusted =
0;

volatile LONG
g_qpcPhaseFfV2DcSamples =
0;


// =============================================================
// QPC Coarse Re-Anchor V1 虛擬模型快照（僅診斷）
//
// 初始虛擬誤差 -100 us；margin < 50 us 時，以 50 us 為修正量子。
// 只統計觸發密度與修正量，不改真實 PDO/timer/send/HAL。
// =============================================================

volatile LONG g_qpcCoarseReanchorDiagSequence = 0;
volatile LONG g_qpcCoarseReanchorInitialized = 0;
volatile LONGLONG g_qpcCoarseReanchorRawErrorNs = 0;
volatile LONGLONG g_qpcCoarseReanchorVirtualErrorNs = 0;
volatile LONGLONG g_qpcCoarseReanchorVirtualMarginNs = 0;
volatile LONGLONG g_qpcCoarseReanchorVirtualOffsetNs = 0;
volatile LONGLONG g_qpcCoarseReanchorWindowStartErrorNs = 0;
volatile LONGLONG g_qpcCoarseReanchorWindowEndErrorNs = 0;
volatile LONGLONG g_qpcCoarseReanchorWindowDeltaErrorNs = 0;
volatile LONGLONG g_qpcCoarseReanchorWindowMinErrorNs = 0;
volatile LONGLONG g_qpcCoarseReanchorWindowMaxErrorNs = 0;
volatile LONG g_qpcCoarseReanchorCorrectionEventsWindow = 0;
volatile LONG g_qpcCoarseReanchorCorrectionTicksWindow = 0;
volatile LONG g_qpcCoarseReanchorMaxTicksPerEventWindow = 0;
volatile LONG g_qpcCoarseReanchorMultiTickEventsWindow = 0;
volatile LONGLONG g_qpcCoarseReanchorTotalCorrectionEvents = 0;
volatile LONGLONG g_qpcCoarseReanchorTotalCorrectionTicks = 0;
volatile LONG g_qpcCoarseReanchorLastEventSpacingCycles = 0;
volatile LONG g_qpcCoarseReanchorMinEventSpacingCycles = 0;
volatile LONG g_qpcCoarseReanchorMaxEventSpacingCycles = 0;
volatile LONGLONG g_qpcCoarseReanchorRobustMedianDriftPpb = 0;
volatile LONGLONG g_qpcCoarseReanchorRobustMadPpb = 0;
volatile LONG g_qpcCoarseReanchorRobustLocked = 0;
volatile LONG g_qpcCoarseReanchorWindowSamples = 0;
volatile LONG g_qpcCoarseReanchorOverflow = 0;


// =============================================================
// PDO One-Shot Scheduler V1B - 真實 Timer Bridge
//
// g_pdoOneShotTimerHandle 由 RunRealTimeCycle... 建立後寫入。
// StartupMode：0=尚未就緒、1=同一 timer 的 warm-up callback、
//              2=正式 one-shot 排程。只有 mode 2 能進入 EtherCAT 循環。
// =============================================================

HANDLE
g_pdoOneShotTimerHandle =
NULL;

volatile LONG
g_pdoOneShotStartupMode =
0;

volatile LONG
g_pdoOneShotWarmupCallbackDone =
0;


// V1B 真實 re-arm 統計；Fail/Bootstrap/Recovery 應搭配 RX timeout 一起判讀。
volatile LONG
g_pdoOneShotInfraRearmOkCount =
0;

volatile LONG
g_pdoOneShotInfraRearmFailCount =
0;

volatile LONG
g_pdoOneShotInfraBootstrapCount =
0;

volatile LONG
g_pdoOneShotInfraActiveCount =
0;

volatile LONGLONG
g_pdoOneShotInfraCoarseErrorAvgNs =
0;

volatile LONGLONG
g_pdoOneShotInfraCoarseErrorMinNs =
0;

volatile LONGLONG
g_pdoOneShotInfraCoarseErrorMaxNs =
0;

volatile LONGLONG
g_pdoOneShotInfraFinalMarginAvgNs =
0;

volatile LONGLONG
g_pdoOneShotInfraFinalMarginMinNs =
0;

volatile LONGLONG
g_pdoOneShotInfraFinalMarginMaxNs =
0;

volatile LONGLONG
g_pdoOneShotInfraRearmCostAvgNs =
0;

volatile LONGLONG
g_pdoOneShotInfraRearmCostMinNs =
0;

volatile LONGLONG
g_pdoOneShotInfraRearmCostMaxNs =
0;


// =============================================================
// PDO One-Shot Bootstrap / Runtime Recovery 快照
//
// Bootstrap reason：1=前置條件未就緒、2=final target 不足 coarse guard、
// 3=arm 前 QPC 讀取失敗、4=coarse target lead 小於最低值、5=其他原因。
// Bootstrap 會以 250 us 安全延遲重新啟動；Runtime Recovery 會跳過落後週期，
// 讓下一個 target 恢復到安全 lead。這些計數是排程恢復事件，不等同 RX timeout。
// =============================================================

volatile LONG g_pdoBootstrapDiagSequence = 0;

volatile LONG g_pdoBootstrapWindowBootstrap = 0;
volatile LONG g_pdoBootstrapCurrentConsecutive = 0;
volatile LONG g_pdoBootstrapMaxConsecutive = 0;
volatile LONGLONG g_pdoBootstrapTotal = 0;
volatile LONG g_pdoBootstrapRecoveryAfterBootstrap = 0;

volatile LONG g_pdoBootstrapReasonNotReady = 0;
volatile LONG g_pdoBootstrapReasonFinalUnderGuard = 0;
volatile LONG g_pdoBootstrapReasonQpcBeforeArmFail = 0;
volatile LONG g_pdoBootstrapReasonLeadTooShort = 0;
volatile LONG g_pdoBootstrapReasonOther = 0;

volatile LONG g_pdoBootstrapSchedulerInitialized = 0;

volatile LONG g_pdoBootstrapPredictedLeadValid = 0;
volatile LONGLONG g_pdoBootstrapPredictedLeadAvgNs = 0;
volatile LONGLONG g_pdoBootstrapPredictedLeadMinNs = 0;
volatile LONGLONG g_pdoBootstrapPredictedLeadMaxNs = 0;

volatile LONGLONG g_pdoBootstrapSchedulerErrorNs = 0;
volatile LONGLONG g_pdoBootstrapSchedulerLateByNs = 0;

volatile LONG g_pdoRuntimeRecoveryWindowEvents = 0;
volatile LONG g_pdoRuntimeRecoveryWindowSkippedCycles = 0;
volatile LONG g_pdoRuntimeRecoveryMaxSkipCycles = 0;
volatile LONGLONG g_pdoRuntimeRecoveryTotalEvents = 0;
volatile LONGLONG g_pdoRuntimeRecoveryTotalSkippedCycles = 0;
volatile LONGLONG g_pdoRuntimeRecoveryLastLeadBeforeNs = 0;
volatile LONGLONG g_pdoRuntimeRecoveryLastLeadAfterNs = 0;

volatile LONG g_pdoRecoverySelfTestSeq = 0;
volatile LONG g_pdoRecoverySelfTestPass = 0;
volatile LONG g_pdoRecoverySelfTestCases = 0;
volatile LONG g_pdoRecoverySelfTestFail = 0;
volatile LONGLONG g_pdoRecoverySelfTestBefore0 = 0;
volatile LONGLONG g_pdoRecoverySelfTestAfter0 = 0;
volatile LONG g_pdoRecoverySelfTestSkip0 = 0;
volatile LONGLONG g_pdoRecoverySelfTestBefore1 = 0;
volatile LONGLONG g_pdoRecoverySelfTestAfter1 = 0;
volatile LONG g_pdoRecoverySelfTestSkip1 = 0;
volatile LONGLONG g_pdoRecoverySelfTestBefore2 = 0;
volatile LONGLONG g_pdoRecoverySelfTestAfter2 = 0;
volatile LONG g_pdoRecoverySelfTestSkip2 = 0;
volatile LONGLONG g_pdoRecoverySelfTestBefore3 = 0;
volatile LONGLONG g_pdoRecoverySelfTestAfter3 = 0;
volatile LONG g_pdoRecoverySelfTestSkip3 = 0;
volatile LONG g_pdoRecoverySelfTestShadowPreserve = 0;


// =============================================================
// PDO One-Shot Scheduler Infrastructure 快照
//
// 注意：名稱沿用早期 V1A，但目前 V1B 已啟用真實 RtSetTimerRelative re-arm。
// 本快照只發布狀態；真正 re-arm 在 Handler 前半段執行。Fine Wait 與 HAL 仍停用。
// 常用狀態：0=WAIT_TARGET、1=READY_BEFORE_COARSE、4=COARSE_ACTIVE；
// final target 已逾期時會走 bootstrap 或 runtime recovery，並留下原因與 lead。
// =============================================================

volatile LONG
g_pdoOneShotInfraSequence =
0;

volatile LONG
g_pdoOneShotInfraState =
0;

volatile LONG
g_pdoOneShotInfraValid =
0;

volatile LONG
g_pdoOneShotInfraControlEnabled =
0;

volatile LONG
g_pdoOneShotInfraFineWaitEnabled =
0;

volatile LONG
g_pdoOneShotInfraWarmupRequired =
1;

volatile LONG
g_pdoOneShotInfraWarmupComplete =
0;

volatile LONGLONG
g_pdoOneShotInfraQpcFrequency =
0;

volatile LONGLONG
g_pdoOneShotInfraFinalTargetQpc =
0;

volatile LONGLONG
g_pdoOneShotInfraCoarseTargetQpc =
0;

volatile LONGLONG
g_pdoOneShotInfraActualWakeQpc =
0;

volatile LONGLONG
g_pdoOneShotInfraGuardNs =
0;

volatile LONGLONG
g_pdoOneShotInfraToCoarseNs =
0;

volatile LONGLONG
g_pdoOneShotInfraToFinalNs =
0;

volatile LONGLONG
g_pdoOneShotInfraFinalLateByNs =
0;

volatile LONG
g_pdoOneShotInfraDcDiagSequence =
0;

volatile LONG
g_pdoOneShotInfraWindowSamples =
0;

volatile LONG
g_pdoOneShotInfraReadyCount =
0;

volatile LONG
g_pdoOneShotInfraCoarseWindowCount =
0;

volatile LONG
g_pdoOneShotInfraFinalLateCount =
0;


// ============================================================================
// Priority 64 PDO Timer Callback
//
// 正常週期順序：
//   1. 驗證 one-shot startup gate，讀取 QPC wake time。
//   2. 更新 QPC scheduler／Real FF／Phase-P，先 re-arm 下一次 callback。
//   3. Flush PLC outputs，送出 LRW+FRMW，分別驗證 Process Data/DC sample。
//   4. 更新 QPC<->DC Reference 與各種 shadow observer snapshot。
//   5. LRW 有效則 Fetch PLC inputs；DC sample 有效才更新 DC estimator/controller。
//   6. 處理低頻 async command，更新 Motion，發布執行時間快照。
//
// 安全原則：下一次 timer 必須在 EtherCAT 通訊前 re-arm；LRW 失敗時不採用 Input，
// 連續八個無效週期才觸發各軸 EmergencyStop，避免單次雜訊造成不必要停機。
// ============================================================================
void RTAPI GlobalTimerHandler_PDO(void* nContext)
{
    EtherCatMaster* pMaster =
        (EtherCatMaster*)nContext;


    if (pMaster == nullptr)
    {
        return;
    }


    // =========================================================
    // PDO One-Shot V1B 啟動閘門
    //
    // Mode 1 是同一 timer 的 warm-up callback：只回報完成，不碰 EtherCAT、不 re-arm。
    // Mode 2 才是正常 live one-shot；其他值直接返回，防止尚未初始化就進入 IO。
    // =========================================================

    LONG oneShotStartupMode =
        g_pdoOneShotStartupMode;


    if (oneShotStartupMode == 1)
    {
        MemoryBarrier();

        InterlockedExchange(
            &g_pdoOneShotWarmupCallbackDone,
            1);

        return;
    }


    if (oneShotStartupMode != 2)
    {
        return;
    }


    uint16_t state = 0;


    // =========================================================
    // PDO Handler Start Time
    // =========================================================

    uint64_t pdoCycleStartMasterNs =
        pMaster->GetCurrentMasterTimeNs();

    // =============================================================
// Fine Scheduler V1
// QPC / TSC Timing Foundation
//
// ★ Diagnostic only
// ★ NO WAIT
// ★ NO TIMER CHANGE
// =============================================================

    static bool qpcInitAttempted =
        false;

    static bool qpcValid =
        false;

    static uint64_t qpcFrequency =
        0;


    // -------------------------------------------------------------
    // QPC Frequency 只取得一次
    // -------------------------------------------------------------

    if (!qpcInitAttempted)
    {
        LARGE_INTEGER frequency;

        if (RtQueryPerformanceFrequency(
            &frequency) &&
            frequency.QuadPart > 0)
        {
            qpcFrequency =
                (uint64_t)
                frequency.QuadPart;

            qpcValid =
                true;
        }

        qpcInitAttempted =
            true;
    }


    // -------------------------------------------------------------
    // Handler Wake QPC
    //
    // 用兩次 back-to-back read，順便量 QPC read cost。
    // -------------------------------------------------------------

    LARGE_INTEGER qpcReadA = {};
    LARGE_INTEGER qpcWake = {};

    bool qpcWakeValid =
        false;

    uint64_t qpcReadCostNs =
        0;


    if (qpcValid)
    {
        BOOL readAOk =
            RtQueryPerformanceCounter(
                &qpcReadA);

        BOOL wakeOk =
            RtQueryPerformanceCounter(
                &qpcWake);

        if (readAOk &&
            wakeOk &&
            qpcWake.QuadPart >=
            qpcReadA.QuadPart)
        {
            uint64_t deltaCounts =
                (uint64_t)
                (
                    qpcWake.QuadPart -
                    qpcReadA.QuadPart
                    );

            qpcReadCostNs =
                (
                    deltaCounts *
                    1000000000ULL
                    )
                /
                qpcFrequency;

            qpcWakeValid =
                true;
        }
    }

    static uint64_t recoveryForensicPreviousWakeQpc = 0;
    int64_t recoveryForensicWakeIntervalNs = 0;

    if (qpcWakeValid && qpcFrequency > 0)
    {
        uint64_t currentWakeQpc = (uint64_t)qpcWake.QuadPart;

        if (recoveryForensicPreviousWakeQpc != 0 &&
            currentWakeQpc >= recoveryForensicPreviousWakeQpc)
        {
            recoveryForensicWakeIntervalNs =
                (int64_t)(((currentWakeQpc - recoveryForensicPreviousWakeQpc) *
                    1000000000ULL) / qpcFrequency);
        }

        recoveryForensicPreviousWakeQpc = currentWakeQpc;
    }


    // =============================================================
    // QPC Scheduler Re-Anchor + Guard Dry Run V1B
    //
    // Purpose:
    // 1. Do NOT anchor during startup transients.
    // 2. Wait until QPC<->DC diagnostics have produced at least
    //    two complete windows.
    // 3. Wait another 1000 valid PDO wakes (~250 ms).
    // 4. Anchor target 50 us AFTER the current coarse wake.
    // 5. Continue the DC-equivalent QPC target sequence and watch
    //    the coarse wake consume that 50 us guard at ~8.3 us/s.
    //
    // State:
    //     0 = WAIT_STABLE
    //     1 = EARLY  (> 10 us before target)
    //     2 = NEAR   (0..10 us before target)
    //     3 = LATE   (coarse wake already after target)
    //
    // IMPORTANT:
    // - Diagnostic only.
    // - NO busy wait.
    // - NO timer re-arm.
    // - NO HAL actuator.
    // - NO EtherCAT send timing change.
    // =============================================================

    static bool qpcSchedulerInitialized =
        false;

    static uint64_t qpcSchedulerTargetQpc =
        0;

    static uint64_t qpcSchedulerAnchorQpc =
        0;

    static LONG qpcSchedulerAnchorDcDiagSequence =
        0;

    static uint64_t qpcSchedulerFractionRemainder =
        0;


    static uint32_t qpcSchedulerStableWakeCycles =
        0;

    static uint32_t qpcSchedulerWindowSamples =
        0;


    static int64_t qpcSchedulerWindowStartErrorNs =
        0;

    static int64_t qpcSchedulerWindowEndErrorNs =
        0;

    static int64_t qpcSchedulerWindowMinErrorNs =
        0;

    static int64_t qpcSchedulerWindowMaxErrorNs =
        0;


    static uint32_t qpcSchedulerEarlyCount =
        0;

    static uint32_t qpcSchedulerNearCount =
        0;

    static uint32_t qpcSchedulerLateCount =
        0;

    static bool qpcFixedRefInitialized = false;
    static uint64_t qpcFixedRefTargetQpc = 0;
    static uint64_t qpcFixedRefFractionRemainder = 0;

    static LONG realFfV0State = 0;
    static uint32_t realFfV0ArmGood = 0;
    static LONG realFfV0LastPhaseSeq = 0;
    static int64_t realFfV0DesiredPpb = EtherCatDcTuning::SchedulerBootstrapDriftPpb;
    static int64_t realFfV0AppliedPpb = EtherCatDcTuning::SchedulerBootstrapDriftPpb;
    static int64_t realFfV0LastStepPpb = 0;
    static LONG realFfV0TripMask = 0;
    static uint32_t realFfV0TripCount = 0;
    static bool realFfV0OneShotHealthy = false;
    static LONG realFfV0PhaseRejectMask = 0;
    static LONG realFfV0LastPhaseRejectMask = 0;
    static uint32_t realFfV0HoldGood = 0;
    static uint32_t realFfV0HoldBad = 0;
    static uint32_t realFfV0HoldEntries = 0;

    // Stage 11F.2A-R1:
    // Only TripMask == 0x01 may accumulate this recovery counter.
    static uint32_t realFfV0SoftRearmGood = 0;

    // DC-RX.1：由上一個 4 kHz callback 的實際 PDO/DC WKC 與 one-shot 結果
    // 建立控制品質，不再使用每約一秒才發布一次的 RX 診斷欄位。
    static bool realFfV0CycleQualityKnown = false;
    static bool realFfV0PreviousCycleClean = true;
    static LONG realFfV0PreviousCycleReasonMask = 0;
    static uint32_t realFfV0CleanCycleStreak = 0;
    static LONG realFfV0TransientHoldReasonMask = 0;
    static LONG realFfV0HistoryMask = 0;
    static LONG realFfV0RecoveryObserverSeqFloor = 0;
    static bool realFfV0FreshObserverRequired = false;
    static bool realFfV0PreviousDcTransportValid = true;
    static bool realFfV0PreviousDcSampleQualified = true;

    // DC-RX.3C sample freshness state.
    // State: 0=UNBOUND, 1=VERIFY, 2=TRACK.
    static LONG dcSampleGuardState = 0;
    static LONG dcSampleGuardLastReasonMask = 0;
    static uint64_t dcSampleGuardLastAcceptedDcNs = 0;
    static uint64_t dcSampleGuardLastAcceptedQpcMidCount = 0;
    static uint32_t dcSampleGuardConsecutiveRejects = 0;
    static uint32_t dcSampleGuardMaximumConsecutiveRejects = 0;
    static uint64_t dcSampleGuardAcceptedTotal = 0;
    static uint64_t dcSampleGuardRejectedTotal = 0;
    static uint64_t dcSampleGuardAnchorTotal = 0;
    static uint64_t dcSampleGuardReanchorTotal = 0;
    static uint64_t dcSampleGuardAgeRejectTotal = 0;
    static uint64_t dcSampleGuardOrderRejectTotal = 0;
    static uint64_t dcSampleGuardDeltaRejectTotal = 0;
    static uint64_t dcSampleGuardLastApproxAgeNs = 0;
    static uint64_t dcSampleGuardMaximumApproxAgeNs = 0;
    static uint64_t dcSampleGuardLastQpcDeltaNs = 0;
    static uint64_t dcSampleGuardLastDcDeltaNs = 0;
    static int64_t dcSampleGuardLastDeltaErrorNs = 0;
    static uint64_t dcSampleGuardLastDeltaToleranceNs = 0;
    static uint64_t dcSampleGuardTimingRejectTotal = 0;
    static bool dcPhaseMapRebindPending = false;

    // DC-RX.3D exact TX/RX timing-source policy.
    // Once exact timing has appeared, the session never silently falls back to
    // the wider whole-call midpoint; a missing exact timestamp is treated as a
    // recoverable DC-only sample miss so fixed timing bias cannot flap.
    static LONG dcSampleTimingSource = 0;
    static bool dcExactTimingLocked = false;
    static uint64_t dcTimingExactUseTotal = 0;
    static uint64_t dcTimingFallbackUseTotal = 0;
    static uint64_t dcTimingMissingAfterLockTotal = 0;
    static uint64_t dcTimingSourceSwitchTotal = 0;
    static uint64_t dcTimingExactRttMaximumNs = 0;
    static int64_t dcTimingMidpointShiftLastNs = 0;
    static uint64_t dcTimingMidpointShiftMaximumAbsNs = 0;

    // DC-RX.3E adaptive exact-RTT envelope.
    // State: 0=UNBOUND, 1=WARMUP, 2=TRACK, 3=SHIFT_CHECK.
    static LONG dcRttGuardState = 0;
    static uint32_t dcRttGuardWarmupSamples = 0;
    static uint64_t dcRttGuardBaselineNs = 0;
    static uint64_t dcRttGuardDeviationNs = 0;
    static uint64_t dcRttGuardLimitNs = 0;
    static uint64_t dcRttGuardLastRttNs = 0;
    static uint64_t dcRttGuardLastExcessNs = 0;
    static uint32_t dcRttGuardOutlierStreak = 0;
    static uint32_t dcRttGuardOutlierMaximumStreak = 0;
    static uint32_t dcRttGuardRebaseCandidateSamples = 0;
    static uint64_t dcRttGuardRebaseCandidateMeanNs = 0;
    static uint64_t dcRttGuardRebaseCandidateMinNs = 0;
    static uint64_t dcRttGuardRebaseCandidateMaxNs = 0;
    static uint64_t dcRttGuardAcceptedTotal = 0;
    static uint64_t dcRttGuardRejectedTotal = 0;
    static uint64_t dcRttGuardRebaseTotal = 0;
    static uint64_t dcSampleGuardRttRejectTotal = 0;

    // DC-RX.3F deterministic test-only fault injection.
    // All state is owned by this Priority-64 callback. Startup publishes an
    // immutable, token-gated configuration before the timer is created.
    static bool dcRx3fRuntimeInitialized = false;
    static DcRx3fFaultScenario dcRx3fScenario =
        DcRx3fFaultScenario::Off;
    static DcRx3fFaultState dcRx3fState =
        DcRx3fFaultState::Off;
    static uint32_t dcRx3fConfiguredCycles = 0;
    static uint32_t dcRx3fConfiguredStartDelayCycles = 0;
    static uint32_t dcRx3fStartDelayRemaining = 0;
    static uint32_t dcRx3fAppliedCycles = 0;
    static uint32_t dcRx3fTargetWaitCycles = 0;
    static uint32_t dcRx3fRecoveryCycles = 0;
    static uint32_t dcRx3fRecoveryStableCycles = 0;
    static uint32_t dcRx3fMaximumPdoInvalidStreak = 0;
    static uint64_t dcRx3fConfiguredValueNs = 0;
    static uint64_t dcRx3fStartTick = 0;
    static uint64_t dcRx3fLastAppliedTick = 0;
    static uint64_t dcRx3fEndTick = 0;
    static uint64_t dcRx3fBaselineAcceptedDcNs = 0;
    static int64_t dcRx3fBaselineAppliedPpb = 0;
    static int64_t dcRx3fMaximumAppliedDeltaPpb = 0;
    static LONG dcRx3fGateBlockMask = 0;
    static LONG dcRx3fEvidenceMask = 0;
    static LONG dcRx3fFailureMask = 0;
    static bool dcRx3fRequireServoOff = true;
    static bool dcRx3fAllowSafetyStop = false;

    // DC-RX.3C Phase-P recovery jump guard.
    static bool phasePRecoveryJumpGuardActive = false;
    static bool phasePRecoveryJumpReferenceValid = false;
    static int64_t phasePRecoveryJumpReferenceWrappedNs = 0;
    static uint32_t phasePRecoveryJumpGoodWindows = 3U;
    static uint64_t phasePRecoveryJumpArmTotal = 0;
    static uint64_t phasePRecoveryJumpPassTotal = 0;
    static uint64_t phasePRecoveryJumpRejectTotal = 0;
    static int64_t phasePRecoveryJumpLastNs = 0;
    static int64_t phasePRecoveryJumpMaximumAbsNs = 0;
    static bool phasePActV0LastObservedWrappedValid = false;
    static int64_t phasePActV0LastObservedWrappedNs = 0;
    static LONG phasePActV0LastSeenPhaseMapSeq = 0;
    static LONG phasePActV0LastAcceptedPhaseMapSeq = 0;
    static bool phasePActV0AcceptedPhaseMapValid = false;
    static int64_t phasePActV0AcceptedFixedUnwrappedErrorNs = 0;
    static uint64_t phasePActV0AcceptedPhaseMapQpc = 0;
    static bool phasePActV0PhaseMapStaleEpisodeActive = false;
    static uint64_t phasePActV0PhaseMapStaleTotal = 0;
    static LONG phasePActV0DiagPhaseMapSequence = 0;
    static bool phasePActV0DiagPhaseMapNew = false;
    static bool phasePActV0DiagPhaseMapAgeGood = false;
    static uint64_t phasePActV0DiagPhaseMapAgeNs = 0;

    // DC-RX.3B bounded holdover policy state.
    // RecoveryProfile: 0=FAST, 1=CONSERVATIVE, 2=FULL_RELOCK.
    static LONG realFfV0RecoveryProfile = 0;
    static uint32_t realFfV0CleanCyclesRequired = 32U;
    static uint32_t realFfV0HoldRecoveryWindowsRequired = 3U;

    static uint32_t dcOnlyInvalidConsecutiveCycles = 0;
    static uint32_t dcOnlyInvalidMaximumConsecutiveCycles = 0;
    static uint32_t dcOnlyGlitchDebt = 0;
    static uint32_t dcOnlyGlitchDebtMaximum = 0;
    static bool dcHoldoverEpisodeActive = false;
    static uint32_t dcHoldoverUnqualifiedCycles = 0;
    static uint32_t dcHoldoverMaximumUnqualifiedCycles = 0;
    static uint64_t dcGraceAcceptedCyclesTotal = 0;
    static uint64_t dcHoldoverEpisodeTotal = 0;
    static uint64_t dcHoldoverEntryTotal = 0;
    static uint64_t dcDegradedEntryTotal = 0;
    static uint64_t dcRelockEntryTotal = 0;
    static LONG dcHoldoverDiagTier = 0;

    static bool realFfV0ClampActive = false;
    static bool realFfClampSelfTestDone = false;

    static LONG phasePActV0State = 0;
    static uint32_t phasePActV0ArmGood = 0;
    static uint32_t phasePActV0HoldGood = 0;
    static uint32_t phasePActV0HoldEntries = 0;
    static uint32_t phasePActV0TripCount = 0;
    static int64_t phasePActV0OffsetNs = 0;
    static int64_t phasePActV0LastStepNs = 0;

    // 啟動校正只在本次 process 執行一次。AUTO 以五個連續合格 Robust
    // median 的平均值鎖定；FIXED 在第一個有效 callback 直接鎖定指定值。
    static bool driftCalibrationInitialized = false;
    static bool driftCalibrationLocked = false;
    static LONG driftCalibrationMode = 0;
    static LONG driftCalibrationLastRobustSequence = 0;
    static uint32_t driftCalibrationGoodWindows = 0;
    static int64_t driftCalibrationMedianSumPpb = 0;
    static int64_t driftBaselinePpb =
        EtherCatDcTuning::SchedulerBootstrapDriftPpb;
    static uint32_t driftCalibrationLockCount = 0;


    // =============================================================
    // QPC Coarse Re-Anchor Dry Run V1 state
    // =============================================================

    static bool qpcCoarseReanchorInitialized = false;
    static int64_t qpcCoarseReanchorVirtualOffsetNs = 0;

    static uint32_t qpcCoarseReanchorWindowSamples = 0;
    static int64_t qpcCoarseReanchorWindowStartErrorNs = 0;
    static int64_t qpcCoarseReanchorWindowEndErrorNs = 0;
    static int64_t qpcCoarseReanchorWindowMinErrorNs = 0;
    static int64_t qpcCoarseReanchorWindowMaxErrorNs = 0;

    static uint32_t qpcCoarseReanchorCorrectionEventsWindow = 0;
    static uint32_t qpcCoarseReanchorCorrectionTicksWindow = 0;
    static uint32_t qpcCoarseReanchorMaxTicksPerEventWindow = 0;
    static uint32_t qpcCoarseReanchorMultiTickEventsWindow = 0;

    static uint64_t qpcCoarseReanchorTotalCorrectionEvents = 0;
    static uint64_t qpcCoarseReanchorTotalCorrectionTicks = 0;

    static uint32_t qpcCoarseReanchorCyclesSinceLastEvent = 0;
    static uint32_t qpcCoarseReanchorLastEventSpacingCycles = 0;
    static uint32_t qpcCoarseReanchorMinEventSpacingCycles = 0;
    static uint32_t qpcCoarseReanchorMaxEventSpacingCycles = 0;
    static bool qpcCoarseReanchorHasPreviousEvent = false;
    static bool qpcCoarseReanchorOverflow = false;


    // =============================================================
    // Trusted Drift -> Live Feed-Forward Dry Run V1 state
    //
    // IMPORTANT:
    // This target is NEVER used for the real one-shot scheduler.
    // =============================================================

    static bool qpcLiveFfInitialized =
        false;

    static uint64_t qpcLiveFfTargetQpc =
        0;

    static uint64_t qpcLiveFfFractionRemainder =
        0;

    static bool qpcLiveFfHasPreviousMode =
        false;

    static bool qpcLiveFfPreviousTrustedMode =
        false;


    static uint32_t qpcLiveFfWindowSamples =
        0;

    static int64_t qpcLiveFfShadowWindowStartErrorNs =
        0;

    static int64_t qpcLiveFfShadowWindowEndErrorNs =
        0;

    static int64_t qpcLiveFfShadowWindowMinErrorNs =
        0;

    static int64_t qpcLiveFfShadowWindowMaxErrorNs =
        0;

    static int64_t qpcLiveFfTargetDeltaWindowStartNs =
        0;

    static int64_t qpcLiveFfTargetDeltaWindowEndNs =
        0;


    static uint32_t qpcLiveFfTrustedCyclesWindow =
        0;

    static uint32_t qpcLiveFfFallbackCyclesWindow =
        0;

    static uint32_t qpcLiveFfModeSwitchesWindow =
        0;


    static uint64_t qpcLiveFfTotalTrustedCycles =
        0;

    static uint64_t qpcLiveFfTotalFallbackCycles =
        0;

    static uint32_t qpcLiveFfTotalModeSwitches =
        0;

    static uint32_t qpcLiveFfInitCount =
        0;


    // =============================================================
    // Frequency FF Dry Run V2 - Phase-derived shadow state
    // =============================================================

    static bool
        qpcPhaseFfV2Initialized =
        false;

    static uint64_t
        qpcPhaseFfV2TargetQpc =
        0;

    static uint64_t
        qpcPhaseFfV2FractionRemainder =
        0;

    static LONG
        qpcPhaseFfV2State =
        0; // WARMUP

    static int64_t
        qpcPhaseFfV2DesiredPpb =
        EtherCatDcTuning::SchedulerBootstrapDriftPpb;

    static int64_t
        qpcPhaseFfV2AppliedPpb =
        EtherCatDcTuning::SchedulerBootstrapDriftPpb;

    static int64_t
        qpcPhaseFfV2LastAppliedStepPpb =
        0;

    static LONG
        qpcPhaseFfV2LastObserverSequence =
        0;

    static uint32_t
        qpcPhaseFfV2WarmupGoodCount =
        0;

    static uint32_t
        qpcPhaseFfV2BadCount =
        0;

    static uint32_t
        qpcPhaseFfV2RecoveryGoodCount =
        0;

    static uint32_t
        qpcPhaseFfV2StateSwitchesWindow =
        0;

    static uint32_t
        qpcPhaseFfV2TotalStateSwitches =
        0;

    static uint32_t
        qpcPhaseFfV2WarmupToTrackTotal =
        0;

    static uint32_t
        qpcPhaseFfV2TrackToHoldTotal =
        0;

    static uint32_t
        qpcPhaseFfV2HoldToTrackTotal =
        0;

    static uint32_t
        qpcPhaseFfV2HoldToFallbackTotal =
        0;

    static uint32_t
        qpcPhaseFfV2FallbackToTrackTotal =
        0;


    static uint32_t
        qpcPhaseFfV2WindowSamples =
        0;

    static int64_t
        qpcPhaseFfV2TargetDeltaWindowStartNs =
        0;

    static int64_t
        qpcPhaseFfV2TargetDeltaWindowEndNs =
        0;

    static uint32_t
        qpcPhaseFfV2TrackCyclesWindow =
        0;

    static uint32_t
        qpcPhaseFfV2HoldCyclesWindow =
        0;

    static uint32_t
        qpcPhaseFfV2FallbackCyclesWindow =
        0;


    const uint32_t
        QPC_PHASE_FF_V2_WARMUP_GOOD_WINDOWS =
        3U;

    const uint32_t
        QPC_PHASE_FF_V2_HOLD_MAX_BAD_WINDOWS =
        5U;

    const uint32_t
        QPC_PHASE_FF_V2_HOLD_RECOVERY_GOOD_WINDOWS =
        3U;

    const uint32_t
        QPC_PHASE_FF_V2_FALLBACK_RECOVERY_GOOD_WINDOWS =
        5U;

    const int64_t
        QPC_PHASE_FF_V2_MAX_SLOPE_MAD_PPB =
        150LL;

    const int64_t
        QPC_PHASE_FF_V2_MAX_RECOMMENDED_DEVIATION_PPB =
        1800LL;

    const int64_t
        QPC_PHASE_FF_V2_MAX_SLEW_PPB_PER_OBSERVER_WINDOW =
        25LL;


    // ---------------------------------------------------------------------
    // DC-RX.3B bounded holdover contract
    //
    // 0x02/0x10/0x20/0x80 are recoverable HOLD reasons:
    //   0x02 = LRW Process Data invalid (the existing 8-cycle PDO safety
    //          debounce remains authoritative).
    //   0x10 = one-shot scheduler continuity interruption.
    //   0x20 = LRW valid / FRMW DC WKC invalid; PDO/PLC/Motion continue.
    //   0x80 = FRMW WKC valid, but DC sample age/order/delta is untrusted.
    //          PDO/PLC/Motion still continue from the valid LRW image.
    //
    // 0x40 is a recoverable FULL_RELOCK marker. It is entered only when the
    // DC-only path cannot regain a continuous clean qualification interval for
    // about one second. Unlike 0x04/0x08, it preserves Last-Known-Good FF and
    // may automatically re-arm after conservative clean/fresh observer gates.
    // ---------------------------------------------------------------------
    const uint32_t REAL_FF_V0_DEFAULT_CLEAN_CYCLES = 32U;       // 8 ms @ 4 kHz
    const uint32_t REAL_FF_V0_CONSERVATIVE_CLEAN_CYCLES = 128U; // 32 ms @ 4 kHz
    const uint32_t REAL_FF_V0_DEFAULT_HOLD_RECOVERY_WINDOWS = 3U;
    const uint32_t REAL_FF_V0_CONSERVATIVE_HOLD_RECOVERY_WINDOWS = 5U;

    const uint32_t DC_RX3B_GLITCH_GRACE_CYCLES = 4U;
    const uint32_t DC_RX3B_GLITCH_DEBT_ADD = 4U;
    const uint32_t DC_RX3B_GLITCH_DEBT_DECAY = 1U;
    const uint32_t DC_RX3B_GLITCH_DEBT_LIMIT = 16U;
    const uint32_t DC_RX3B_GLITCH_DEBT_SATURATION = 64U;
    const uint32_t DC_RX3B_DEGRADED_HOLDOVER_CYCLES = 400U;  // 100 ms
    const uint32_t DC_RX3B_FULL_RELOCK_CYCLES = 4000U;       // 1 s

    // DC-RX.3C sample freshness / stale-frame rejection.
    // The combined call includes frame build and RX processing. The 300 us
    // ceiling is an observer-qualification guard only: it does not change the
    // existing 205/210 us RX deadlines or the 250 us PDO cycle. A transaction
    // beyond this ceiling is never allowed to steer a DC observer.
    const uint64_t DC_RX3C_MAX_SAMPLE_TRANSACTION_NS = 300000ULL;
    const uint64_t DC_RX3C_DELTA_BASE_TOLERANCE_NS = 150000ULL;
    const uint64_t DC_RX3C_DELTA_DRIFT_DIVISOR = 20000ULL; // 50 ppm
    const uint64_t DC_RX3C_DELTA_MAX_TOLERANCE_NS = 750000ULL;
    const uint32_t DC_RX3C_REANCHOR_REJECT_LIMIT = 4U;

    const LONG DC_RX3C_SAMPLE_STATE_UNBOUND = 0;
    const LONG DC_RX3C_SAMPLE_STATE_VERIFY = 1;
    const LONG DC_RX3C_SAMPLE_STATE_TRACK = 2;

    const LONG DC_RX3C_SAMPLE_REJECT_AGE = 0x01;
    const LONG DC_RX3C_SAMPLE_REJECT_ANCHOR = 0x02;
    const LONG DC_RX3C_SAMPLE_REJECT_DC_ORDER = 0x04;
    const LONG DC_RX3C_SAMPLE_REJECT_QPC_ORDER = 0x08;
    const LONG DC_RX3C_SAMPLE_REJECT_DELTA = 0x10;
    const LONG DC_RX3C_SAMPLE_REJECT_CONVERSION = 0x20;
    const LONG DC_RX3D_SAMPLE_REJECT_TIMING_SOURCE = 0x40;
    const LONG DC_RX3E_SAMPLE_REJECT_RTT_ENVELOPE = 0x80;

    const LONG DC_RX3D_TIMING_SOURCE_NONE = 0;
    const LONG DC_RX3D_TIMING_SOURCE_EXACT_TXRX = 1;
    const LONG DC_RX3D_TIMING_SOURCE_CALL_FALLBACK = 2;

    // DC-RX.3E exact software RTT quality contract.
    // A frame may still beat the 210 us hard deadline yet arrive much later
    // than the stable path. Such a sample carries receive-queue/IST latency in
    // its software midpoint and must not steer the DC observer.
    const LONG DC_RX3E_RTT_STATE_UNBOUND = 0;
    const LONG DC_RX3E_RTT_STATE_WARMUP = 1;
    const LONG DC_RX3E_RTT_STATE_TRACK = 2;
    const LONG DC_RX3E_RTT_STATE_SHIFT_CHECK = 3;
    const uint32_t DC_RX3E_RTT_WARMUP_SAMPLES = 128U; // 32 ms @ 4 kHz
    const uint64_t DC_RX3E_RTT_INITIAL_DEVIATION_NS = 5000ULL;
    const uint64_t DC_RX3E_RTT_MIN_HEADROOM_NS = 50000ULL;
    const uint64_t DC_RX3E_RTT_MAX_HEADROOM_NS = 100000ULL;
    const uint64_t DC_RX3E_RTT_JITTER_BIAS_NS = 10000ULL;
    const uint64_t DC_RX3E_RTT_ABSOLUTE_LIMIT_NS = 200000ULL;
    const uint32_t DC_RX3E_RTT_REBASE_SAMPLES = 64U; // 16 ms @ 4 kHz
    const uint64_t DC_RX3E_RTT_REBASE_CONSISTENCY_NS = 20000ULL;
    const uint64_t DC_RX3E_RTT_REBASE_RANGE_NS = 40000ULL;

    const uint32_t DC_RX3C_PHASE_JUMP_GOOD_WINDOWS = 3U;
    const int64_t DC_RX3C_PHASE_JUMP_MAX_NS = 50000LL;
    const uint64_t DC_RX3C_PHASE_MAP_MAX_AGE_NS = 1500000000ULL;

    const LONG DC_RX3B_TIER_NONE = 0;
    const LONG DC_RX3B_TIER_GRACE = 1;
    const LONG DC_RX3B_TIER_HOLDOVER = 2;
    const LONG DC_RX3B_TIER_DEGRADED = 3;
    const LONG DC_RX3B_TIER_RELOCK = 4;
    const LONG DC_RX3B_TIER_REQUALIFY = 5;

    const LONG REAL_FF_V0_TRANSIENT_RX_REASON = 0x02;
    const LONG REAL_FF_V0_TRANSIENT_SCHEDULER_REASON = 0x10;
    const LONG REAL_FF_V0_TRANSIENT_DC_SAMPLE_REASON = 0x20;
    const LONG REAL_FF_V0_TRANSIENT_DC_FRESHNESS_REASON = 0x80;
    const LONG REAL_FF_V0_SOFT_DC_RELOCK_REASON = 0x40;
    const LONG REAL_FF_V0_SOFT_RECOVERABLE_TRIP_MASK =
        0x01 | REAL_FF_V0_SOFT_DC_RELOCK_REASON;
    const LONG REAL_FF_V0_HARD_TRIP_MASK = 0x04 | 0x08;

    auto IsRealFfV0CleanRecoveryReady = [&]() -> bool
    {
        return
            !realFfV0CycleQualityKnown ||
            realFfV0CleanCycleStreak >= realFfV0CleanCyclesRequired;
    };

    auto QpcCountsToNsSafe =
        [&](uint64_t counts, uint64_t* resultNs) -> bool
    {
        if (resultNs == nullptr || qpcFrequency == 0)
        {
            return false;
        }

        // Quotient/remainder conversion avoids counts * 1e9 overflow after a
        // prolonged communication interruption. The practical RTX64 QPC
        // frequency is far below the guarded multiplication limit.
        const uint64_t wholeSeconds = counts / qpcFrequency;
        const uint64_t remainderCounts = counts % qpcFrequency;

        if (wholeSeconds > 0xFFFFFFFFFFFFFFFFULL / 1000000000ULL ||
            remainderCounts > 0xFFFFFFFFFFFFFFFFULL / 1000000000ULL)
        {
            return false;
        }

        const uint64_t wholeNs = wholeSeconds * 1000000000ULL;
        const uint64_t remainderNs =
            (remainderCounts * 1000000000ULL) / qpcFrequency;

        if (wholeNs > 0xFFFFFFFFFFFFFFFFULL - remainderNs)
        {
            return false;
        }

        *resultNs = wholeNs + remainderNs;
        return true;
    };

    auto ArmPhasePRecoveryJumpGuard =
        [&](bool preservePreviousReference)
    {
        if (!phasePRecoveryJumpGuardActive)
        {
            phasePRecoveryJumpArmTotal++;
            phasePRecoveryJumpGuardActive = true;
            phasePRecoveryJumpGoodWindows = 0;

            if (preservePreviousReference &&
                phasePActV0LastObservedWrappedValid)
            {
                phasePRecoveryJumpReferenceWrappedNs =
                    phasePActV0LastObservedWrappedNs;
                phasePRecoveryJumpReferenceValid = true;
            }
            else
            {
                phasePRecoveryJumpReferenceValid = false;
            }
        }
        else if (!preservePreviousReference)
        {
            // A DC clock chronology re-anchor changes the coordinate binding.
            // Do not compare the new phase against the pre-reanchor reference.
            phasePRecoveryJumpReferenceValid = false;
            phasePRecoveryJumpGoodWindows = 0;
        }
    };

    // These locals describe this callback's scheduler result and remain in
    // scope until the LRW+FRMW result is known later in the same callback.
    bool currentSchedulerEvaluated = false;
    bool currentSchedulerRearmOk = false;
    bool currentSchedulerUsedBootstrap = false;
    bool currentSchedulerRuntimeRecovery = false;
    bool dcObserversResetThisCycle = false;

    auto ResetDcObserversAfterTransientInterruption = [&]()
    {
        // Preserve the already-qualified timelines and the real actuator's
        // last-known-good frequency. Only incomplete windows that span the
        // interruption are discarded later in this callback.
        qpcPhaseFfV2DesiredPpb = qpcPhaseFfV2AppliedPpb;
        qpcPhaseFfV2LastAppliedStepPpb = 0;
        qpcPhaseFfV2LastObserverSequence =
            g_qpcDcPhaseResidualV1ADiagSequence;
        qpcPhaseFfV2WarmupGoodCount = 0;
        qpcPhaseFfV2BadCount = 0;
        qpcPhaseFfV2RecoveryGoodCount = 0;
        qpcPhaseFfV2WindowSamples = 0;
        qpcPhaseFfV2TargetDeltaWindowStartNs = 0;
        qpcPhaseFfV2TargetDeltaWindowEndNs = 0;
        qpcPhaseFfV2TrackCyclesWindow = 0;
        qpcPhaseFfV2HoldCyclesWindow = 0;
        qpcPhaseFfV2FallbackCyclesWindow = 0;
        qpcPhaseFfV2StateSwitchesWindow = 0;

        qpcLiveFfWindowSamples = 0;
        qpcLiveFfShadowWindowStartErrorNs = 0;
        qpcLiveFfShadowWindowEndErrorNs = 0;
        qpcLiveFfShadowWindowMinErrorNs = 0;
        qpcLiveFfShadowWindowMaxErrorNs = 0;
        qpcLiveFfTargetDeltaWindowStartNs = 0;
        qpcLiveFfTargetDeltaWindowEndNs = 0;
        qpcLiveFfTrustedCyclesWindow = 0;
        qpcLiveFfFallbackCyclesWindow = 0;
        qpcLiveFfModeSwitchesWindow = 0;

        // Real FF may resume only after V2 carries a V1A publication newer
        // than the interruption. This prevents an old one-second snapshot from
        // releasing HOLD while preserving the completed 16-point history.
        realFfV0RecoveryObserverSeqFloor =
            g_qpcDcPhaseResidualV1ADiagSequence;
        realFfV0FreshObserverRequired = true;

        // Do not consume the pre-interruption V2 publication again.
        realFfV0LastPhaseSeq =
            g_qpcPhaseFfV2DiagSequence;

        dcObserversResetThisCycle = true;
    };

    auto EnterRealFfV0TransientHold =
        [&](LONG reasonMask, bool resetObservers)
    {
        realFfV0CleanCycleStreak = 0;

        // A hard trip remains latched. Transient activity must not downgrade it.
        if ((realFfV0TripMask & REAL_FF_V0_HARD_TRIP_MASK) != 0)
        {
            return;
        }

        realFfV0TransientHoldReasonMask |= reasonMask;
        realFfV0HistoryMask |= reasonMask;
        realFfV0LastStepPpb = 0;
        realFfV0SoftRearmGood = 0;

        if (realFfV0State == 2 || realFfV0State == 3)
        {
            if (realFfV0State != 3)
            {
                realFfV0HoldEntries++;
            }

            realFfV0State = 3;
            realFfV0DesiredPpb = realFfV0AppliedPpb;
            realFfV0HoldGood = 0;
            realFfV0HoldBad = 0;
        }
        else if (realFfV0State == 0 || realFfV0State == 1)
        {
            realFfV0State = 0;
            realFfV0ArmGood = 0;
            realFfV0DesiredPpb = realFfV0AppliedPpb;
            realFfV0HoldGood = 0;
            realFfV0HoldBad = 0;
        }
        else if (realFfV0State == 4)
        {
            // Observer-only state 4 also keeps the last-known-good frequency.
            realFfV0DesiredPpb = realFfV0AppliedPpb;
        }

        if (resetObservers)
        {
            ArmPhasePRecoveryJumpGuard(true);

            if (!dcObserversResetThisCycle)
            {
                ResetDcObserversAfterTransientInterruption();
            }
        }
    };

    auto ResetRealFfV0RecoveryProfileAfterActive = [&]()
    {
        realFfV0RecoveryProfile = 0;
        realFfV0CleanCyclesRequired = REAL_FF_V0_DEFAULT_CLEAN_CYCLES;
        realFfV0HoldRecoveryWindowsRequired =
            REAL_FF_V0_DEFAULT_HOLD_RECOVERY_WINDOWS;
        dcHoldoverEpisodeActive = false;
        dcHoldoverUnqualifiedCycles = 0;
        dcHoldoverDiagTier = DC_RX3B_TIER_NONE;
    };

    auto ApplyRealFfV0ConservativeRecoveryProfile = [&]()
    {
        if (realFfV0RecoveryProfile < 1)
        {
            realFfV0RecoveryProfile = 1;
        }

        realFfV0CleanCyclesRequired =
            REAL_FF_V0_CONSERVATIVE_CLEAN_CYCLES;
        realFfV0HoldRecoveryWindowsRequired =
            REAL_FF_V0_CONSERVATIVE_HOLD_RECOVERY_WINDOWS;
    };

    auto EscalateRealFfV0ToSoftDcRelock = [&]()
    {
        // Timer/bootstrap integrity failures remain the only hard-latched
        // reasons. A prolonged DC-only outage keeps Last-Known-Good FF and
        // requests a full, automatically recoverable requalification.
        if ((realFfV0TripMask & REAL_FF_V0_HARD_TRIP_MASK) != 0)
        {
            return;
        }

        ApplyRealFfV0ConservativeRecoveryProfile();
        realFfV0RecoveryProfile = 2;
        realFfV0State = 4;
        realFfV0DesiredPpb = realFfV0AppliedPpb;
        realFfV0LastStepPpb = 0;
        realFfV0TripMask |= REAL_FF_V0_SOFT_DC_RELOCK_REASON;
        realFfV0HistoryMask |= REAL_FF_V0_SOFT_DC_RELOCK_REASON;
        realFfV0TransientHoldReasonMask |=
            REAL_FF_V0_TRANSIENT_DC_SAMPLE_REASON;
        realFfV0ArmGood = 0;
        realFfV0HoldGood = 0;
        realFfV0HoldBad = 0;
        realFfV0SoftRearmGood = 0;

        if (!dcObserversResetThisCycle)
        {
            ResetDcObserversAfterTransientInterruption();
        }
    };


    // ---------------------------------------------------------------------
    // DC-RX.3F deterministic test-only fault injection controller
    //
    // This controller only decides when a logical result/sample will be
    // overridden later in this callback. It never skips the real EtherCAT
    // exchange and never performs file I/O, printing, sleeping, or allocation.
    // A scenario runs once per RTOS process and remains PASS/FAIL afterwards.
    // ---------------------------------------------------------------------
    const LONG DC_RX3F_GATE_REAL_FF = 0x0001;
    const LONG DC_RX3F_GATE_PHASE_P = 0x0002;
    const LONG DC_RX3F_GATE_HOLD_TRIP = 0x0004;
    const LONG DC_RX3F_GATE_CLEAN = 0x0008;
    const LONG DC_RX3F_GATE_SAMPLE = 0x0010;
    const LONG DC_RX3F_GATE_EXACT = 0x0020;
    const LONG DC_RX3F_GATE_RTT = 0x0040;
    const LONG DC_RX3F_GATE_PHASE_GUARD = 0x0080;
    const LONG DC_RX3F_GATE_SCHEDULER = 0x0100;
    const LONG DC_RX3F_GATE_SERVO_ON = 0x0200;
    const LONG DC_RX3F_GATE_DC_REFERENCE = 0x0400;
    const LONG DC_RX3F_GATE_PHASE_MAP = 0x0800;

    const LONG DC_RX3F_EVIDENCE_APPLIED = 0x0001;
    const LONG DC_RX3F_EVIDENCE_DOWNSTREAM = 0x0002;
    const LONG DC_RX3F_EVIDENCE_PDO_PRESERVED = 0x0004;
    const LONG DC_RX3F_EVIDENCE_HOLD_OR_GRACE = 0x0008;
    const LONG DC_RX3F_EVIDENCE_SAFETY_STOP = 0x0010;
    const LONG DC_RX3F_EVIDENCE_RECOVERED = 0x0020;
    const LONG DC_RX3F_EVIDENCE_FF_PRESERVED = 0x0040;

    const LONG DC_RX3F_FAIL_TARGET_TIMEOUT = 0x0001;
    const LONG DC_RX3F_FAIL_RECOVERY_TIMEOUT = 0x0002;
    const LONG DC_RX3F_FAIL_HARD_TRIP = 0x0004;
    const LONG DC_RX3F_FAIL_FF_DISCONTINUITY = 0x0008;
    const LONG DC_RX3F_FAIL_UNEXPECTED_PDO_INVALID = 0x0010;
    const LONG DC_RX3F_FAIL_UNEXPECTED_SAFETY_STOP = 0x0020;
    const LONG DC_RX3F_FAIL_MISSING_EVIDENCE = 0x0040;

    const uint32_t DC_RX3F_TARGET_WAIT_LIMIT_CYCLES = 40000U; // 10 s
    const uint32_t DC_RX3F_RECOVERY_LIMIT_CYCLES = 180000U;   // 45 s
    const uint32_t DC_RX3F_RECOVERY_STABLE_CYCLES = 128U;     // 32 ms
    const int64_t DC_RX3F_MAX_APPLIED_FF_DELTA_PPB = 1000LL;

    bool dcRx3fInjectionRequestedThisCycle = false;
    bool dcRx3fInjectionAppliedThisCycle = false;

    if (!dcRx3fRuntimeInitialized &&
        g_dcRx3fConfigReady != 0)
    {
        MemoryBarrier();

        const LONG configuredScenario =
            g_dcRx3fConfiguredScenario;

        if (configuredScenario >=
            (LONG)DcRx3fFaultScenario::DcWkcDrop &&
            configuredScenario <=
            (LONG)DcRx3fFaultScenario::LrwTimeout &&
            g_dcRx3fConfiguredCycles > 0)
        {
            dcRx3fScenario =
                (DcRx3fFaultScenario)configuredScenario;
            dcRx3fConfiguredCycles =
                (uint32_t)g_dcRx3fConfiguredCycles;
            dcRx3fConfiguredStartDelayCycles =
                g_dcRx3fConfiguredStartDelayCycles > 0
                ? (uint32_t)g_dcRx3fConfiguredStartDelayCycles
                : 0U;
            dcRx3fStartDelayRemaining =
                dcRx3fConfiguredStartDelayCycles;
            dcRx3fConfiguredValueNs =
                g_dcRx3fConfiguredValueNs > 0
                ? (uint64_t)g_dcRx3fConfiguredValueNs
                : 0ULL;
            dcRx3fRequireServoOff =
                g_dcRx3fConfiguredRequireServoOff != 0;
            dcRx3fAllowSafetyStop =
                g_dcRx3fConfiguredAllowSafetyStop != 0;
            dcRx3fState = DcRx3fFaultState::WaitGate;
        }
        else
        {
            dcRx3fScenario = DcRx3fFaultScenario::Off;
            dcRx3fState = DcRx3fFaultState::Off;
        }

        dcRx3fRuntimeInitialized = true;
    }

    bool dcRx3fAllExistingAxesServoOff = true;
    if (dcRx3fRuntimeInitialized &&
        dcRx3fScenario != DcRx3fFaultScenario::Off &&
        dcRx3fRequireServoOff)
    {
        for (const AxisContext& axis : pMaster->m_Axes)
        {
            if (axis.isExist && axis.isServoOn)
            {
                dcRx3fAllExistingAxesServoOff = false;
                break;
            }
        }
    }

    dcRx3fGateBlockMask = 0;
    if (dcRx3fRuntimeInitialized &&
        dcRx3fScenario != DcRx3fFaultScenario::Off)
    {
        if (realFfV0State != 2)
            dcRx3fGateBlockMask |= DC_RX3F_GATE_REAL_FF;
        if (phasePActV0State != 2)
            dcRx3fGateBlockMask |= DC_RX3F_GATE_PHASE_P;
        if (realFfV0TransientHoldReasonMask != 0 ||
            realFfV0TripMask != 0)
        {
            dcRx3fGateBlockMask |= DC_RX3F_GATE_HOLD_TRIP;
        }
        if (!realFfV0CycleQualityKnown ||
            !realFfV0PreviousCycleClean ||
            realFfV0CleanCycleStreak < realFfV0CleanCyclesRequired)
        {
            dcRx3fGateBlockMask |= DC_RX3F_GATE_CLEAN;
        }
        if (dcSampleGuardState != DC_RX3C_SAMPLE_STATE_TRACK ||
            dcSampleGuardLastAcceptedDcNs == 0)
        {
            dcRx3fGateBlockMask |= DC_RX3F_GATE_SAMPLE;
        }
        if (!dcExactTimingLocked)
            dcRx3fGateBlockMask |= DC_RX3F_GATE_EXACT;
        if (dcRttGuardState != DC_RX3E_RTT_STATE_TRACK)
            dcRx3fGateBlockMask |= DC_RX3F_GATE_RTT;
        if (phasePRecoveryJumpGuardActive)
            dcRx3fGateBlockMask |= DC_RX3F_GATE_PHASE_GUARD;
        if (!qpcSchedulerInitialized)
            dcRx3fGateBlockMask |= DC_RX3F_GATE_SCHEDULER;
        if (!dcRx3fAllExistingAxesServoOff)
            dcRx3fGateBlockMask |= DC_RX3F_GATE_SERVO_ON;
        if (GetDcReferenceSlaveIndex() < 0)
            dcRx3fGateBlockMask |= DC_RX3F_GATE_DC_REFERENCE;
        if (!phasePActV0AcceptedPhaseMapValid ||
            !phasePActV0DiagPhaseMapAgeGood)
        {
            dcRx3fGateBlockMask |= DC_RX3F_GATE_PHASE_MAP;
        }
    }

    auto BeginDcRx3fInjection = [&]()
    {
        dcRx3fState = DcRx3fFaultState::Inject;
        dcRx3fAppliedCycles = 0;
        dcRx3fTargetWaitCycles = 0;
        dcRx3fRecoveryCycles = 0;
        dcRx3fRecoveryStableCycles = 0;
        dcRx3fEvidenceMask = 0;
        dcRx3fFailureMask = 0;
        dcRx3fMaximumPdoInvalidStreak = 0;
        dcRx3fBaselineAcceptedDcNs =
            dcSampleGuardLastAcceptedDcNs;
        dcRx3fBaselineAppliedPpb = realFfV0AppliedPpb;
        dcRx3fMaximumAppliedDeltaPpb = 0;
        dcRx3fStartTick = pMaster->tickCount_PDO;
        dcRx3fLastAppliedTick = 0;
        dcRx3fEndTick = 0;
    };

    if (dcRx3fState == DcRx3fFaultState::WaitGate)
    {
        dcRx3fStartDelayRemaining =
            dcRx3fConfiguredStartDelayCycles;

        if (dcRx3fGateBlockMask == 0)
        {
            if (dcRx3fStartDelayRemaining == 0)
            {
                BeginDcRx3fInjection();
            }
            else
            {
                dcRx3fState = DcRx3fFaultState::Delay;
            }
        }
    }
    else if (dcRx3fState == DcRx3fFaultState::Delay)
    {
        if (dcRx3fGateBlockMask != 0)
        {
            dcRx3fState = DcRx3fFaultState::WaitGate;
            dcRx3fStartDelayRemaining =
                dcRx3fConfiguredStartDelayCycles;
        }
        else
        {
            if (dcRx3fStartDelayRemaining > 0)
            {
                dcRx3fStartDelayRemaining--;
            }

            if (dcRx3fStartDelayRemaining == 0)
            {
                BeginDcRx3fInjection();
            }
        }
    }

    dcRx3fInjectionRequestedThisCycle =
        dcRx3fState == DcRx3fFaultState::Inject &&
        dcRx3fAppliedCycles < dcRx3fConfiguredCycles;

    // ---------------------------------------------------------------------
    // 啟動 Drift 校正
    //
    // AUTO：Robust snapshot 必須已鎖定、Buffer=9、MAD 與 Raw/Median 差值
    // 都通過門檻，且前一段 4 kHz PDO/DC 控制品質已連續穩定。冷機與溫機不需要
    // 接近 Bootstrap，只要在絕對捕獲範圍內連續五窗穩定，就取 median 平均鎖定。
    // FIXED：第一個 callback 直接採用 Startup 已驗證的設定值。
    // 校正只改「後續週期的頻率」，不重算既有 target，因此沒有相位突跳。
    // ---------------------------------------------------------------------
    bool driftCalibrationPublish = false;
    bool driftCalibrationLockedThisCycle = false;
    bool driftCalibrationCandidateGood = false;
    LONG driftCalibrationRobustSequence = 0;
    int64_t driftCalibrationRawPpb = 0;
    int64_t driftCalibrationMedianPpb = 0;
    int64_t driftCalibrationMadPpb = 0;
    int64_t driftCalibrationRawMedianDeviationPpb = 0;

    if (!driftCalibrationInitialized)
    {
        driftCalibrationMode =
            g_dcDriftConfigReady != 0
            ? g_dcDriftConfiguredMode
            : 0;

        driftBaselinePpb =
            EtherCatDcTuning::SchedulerBootstrapDriftPpb;

        if (driftCalibrationMode == 1)
        {
            int64_t fixedPpb =
                (int64_t)g_dcDriftConfiguredFixedPpb;

            if (fixedPpb >= EtherCatDcTuning::RealFfMinimumDriftPpb &&
                fixedPpb <= EtherCatDcTuning::RealFfMaximumDriftPpb)
            {
                driftBaselinePpb = fixedPpb;
                driftCalibrationLocked = true;
                driftCalibrationLockCount = 1;
                driftCalibrationCandidateGood = true;
                driftCalibrationLockedThisCycle = true;
                driftCalibrationPublish = true;
            }
            else
            {
                driftCalibrationMode = 0;
            }
        }

        driftCalibrationInitialized = true;
    }

    if (driftCalibrationMode == 0 &&
        !driftCalibrationLocked)
    {
        LONG robustSequenceBefore =
            g_qpcDcRobustDiagSequence;

        if (robustSequenceBefore != 0 &&
            (robustSequenceBefore & 1) == 0 &&
            robustSequenceBefore != driftCalibrationLastRobustSequence)
        {
            MemoryBarrier();

            driftCalibrationRawPpb =
                (int64_t)g_qpcDcRobustRawDriftPpb;

            driftCalibrationMedianPpb =
                (int64_t)g_qpcDcRobustMedianDriftPpb;

            driftCalibrationMadPpb =
                (int64_t)g_qpcDcRobustMadPpb;

            LONG robustBufferCount =
                g_qpcDcRobustBufferCount;

            LONG robustCurrentAccepted =
                g_qpcDcRobustCurrentAccepted;

            LONG robustLocked =
                g_qpcDcRobustLocked;

            MemoryBarrier();

            LONG robustSequenceAfter =
                g_qpcDcRobustDiagSequence;

            if (robustSequenceBefore == robustSequenceAfter &&
                (robustSequenceAfter & 1) == 0)
            {
                driftCalibrationLastRobustSequence =
                    robustSequenceAfter;

                driftCalibrationRobustSequence =
                    robustSequenceAfter;

                driftCalibrationRawMedianDeviationPpb =
                    driftCalibrationRawPpb -
                    driftCalibrationMedianPpb;

                int64_t rawMedianAbsPpb =
                    driftCalibrationRawMedianDeviationPpb >= 0
                    ? driftCalibrationRawMedianDeviationPpb
                    : -driftCalibrationRawMedianDeviationPpb;

                driftCalibrationCandidateGood =
                    robustCurrentAccepted != 0 &&
                    robustLocked != 0 &&
                    robustBufferCount >= 9 &&
                    driftCalibrationMedianPpb >=
                    EtherCatDcTuning::DriftCalibrationMinimumPpb &&
                    driftCalibrationMedianPpb <=
                    EtherCatDcTuning::DriftCalibrationMaximumPpb &&
                    driftCalibrationMadPpb <=
                    EtherCatDcTuning::DriftCalibrationMaximumMadPpb &&
                    rawMedianAbsPpb <=
                    EtherCatDcTuning::DriftCalibrationMaximumRawMedianDeviationPpb &&
                    IsRealFfV0CleanRecoveryReady();

                if (driftCalibrationCandidateGood)
                {
                    driftCalibrationMedianSumPpb +=
                        driftCalibrationMedianPpb;

                    driftCalibrationGoodWindows++;

                    if (driftCalibrationGoodWindows >=
                        EtherCatDcTuning::DriftCalibrationGoodWindows)
                    {
                        const int64_t calibrationDivisor =
                            (int64_t)EtherCatDcTuning::DriftCalibrationGoodWindows;

                        driftBaselinePpb =
                            driftCalibrationMedianSumPpb >= 0
                            ? (driftCalibrationMedianSumPpb +
                                calibrationDivisor / 2LL) /
                            calibrationDivisor
                            : (driftCalibrationMedianSumPpb -
                                calibrationDivisor / 2LL) /
                            calibrationDivisor;

                        if (driftBaselinePpb <
                            EtherCatDcTuning::RealFfMinimumDriftPpb)
                        {
                            driftBaselinePpb =
                                EtherCatDcTuning::RealFfMinimumDriftPpb;
                        }

                        if (driftBaselinePpb >
                            EtherCatDcTuning::RealFfMaximumDriftPpb)
                        {
                            driftBaselinePpb =
                                EtherCatDcTuning::RealFfMaximumDriftPpb;
                        }

                        driftCalibrationLocked = true;
                        driftCalibrationLockCount++;
                        driftCalibrationLockedThisCycle = true;
                    }
                }
                else
                {
                    driftCalibrationGoodWindows = 0;
                    driftCalibrationMedianSumPpb = 0;
                }

                driftCalibrationPublish = true;
            }
        }
    }

    if (driftCalibrationLockedThisCycle)
    {
        realFfV0State = 0;
        realFfV0ArmGood = 0;
        realFfV0LastPhaseSeq = 0;
        realFfV0DesiredPpb = driftBaselinePpb;
        realFfV0AppliedPpb = driftBaselinePpb;
        realFfV0LastStepPpb = 0;
        realFfV0TripMask = 0;
        realFfV0TransientHoldReasonMask = 0;
        realFfV0HistoryMask = 0;
        realFfV0RecoveryObserverSeqFloor = 0;
        realFfV0FreshObserverRequired = false;
        realFfV0HoldGood = 0;
        realFfV0HoldBad = 0;
        realFfV0SoftRearmGood = 0;

        phasePActV0State = 0;
        phasePActV0ArmGood = 0;
        phasePActV0HoldGood = 0;
        phasePActV0OffsetNs = 0;
        phasePActV0LastStepNs = 0;
        phasePRecoveryJumpGuardActive = false;
        phasePRecoveryJumpReferenceValid = false;
        phasePRecoveryJumpGoodWindows =
            DC_RX3C_PHASE_JUMP_GOOD_WINDOWS;
        phasePActV0LastObservedWrappedValid = false;
        phasePActV0LastObservedWrappedNs = 0;
        phasePActV0LastSeenPhaseMapSeq = 0;
        phasePActV0LastAcceptedPhaseMapSeq = 0;
        phasePActV0AcceptedPhaseMapValid = false;
        phasePActV0AcceptedFixedUnwrappedErrorNs = 0;
        phasePActV0AcceptedPhaseMapQpc = 0;
        phasePActV0PhaseMapStaleEpisodeActive = false;
        phasePActV0DiagPhaseMapSequence = 0;
        phasePActV0DiagPhaseMapNew = false;
        phasePActV0DiagPhaseMapAgeGood = false;
        phasePActV0DiagPhaseMapAgeNs = 0;

        qpcPhaseFfV2Initialized = false;
        qpcPhaseFfV2State = 0;
        qpcPhaseFfV2DesiredPpb = driftBaselinePpb;
        qpcPhaseFfV2AppliedPpb = driftBaselinePpb;
        qpcPhaseFfV2LastAppliedStepPpb = 0;
        qpcPhaseFfV2LastObserverSequence = 0;
        qpcPhaseFfV2WarmupGoodCount = 0;
        qpcPhaseFfV2BadCount = 0;
        qpcPhaseFfV2RecoveryGoodCount = 0;
        qpcPhaseFfV2WindowSamples = 0;
        qpcPhaseFfV2TargetDeltaWindowStartNs = 0;
        qpcPhaseFfV2TargetDeltaWindowEndNs = 0;
        qpcPhaseFfV2TrackCyclesWindow = 0;
        qpcPhaseFfV2HoldCyclesWindow = 0;
        qpcPhaseFfV2FallbackCyclesWindow = 0;
        qpcPhaseFfV2StateSwitchesWindow = 0;

        // 讓所有 residual observer 經由 Live-FF init generation 自動重新綁定。
        qpcLiveFfInitialized = false;
        qpcLiveFfHasPreviousMode = false;
        qpcLiveFfWindowSamples = 0;
        qpcLiveFfShadowWindowStartErrorNs = 0;
        qpcLiveFfShadowWindowEndErrorNs = 0;
        qpcLiveFfShadowWindowMinErrorNs = 0;
        qpcLiveFfShadowWindowMaxErrorNs = 0;
        qpcLiveFfTargetDeltaWindowStartNs = 0;
        qpcLiveFfTargetDeltaWindowEndNs = 0;
        qpcLiveFfTrustedCyclesWindow = 0;
        qpcLiveFfFallbackCyclesWindow = 0;
        qpcLiveFfModeSwitchesWindow = 0;
    }

    if (driftCalibrationPublish)
    {
        InterlockedIncrement(
            &g_dcDriftCalibrationDiagSequence);

        g_dcDriftCalibrationMode =
            driftCalibrationMode;

        g_dcDriftCalibrationState =
            driftCalibrationMode == 1
            ? 2L
            : driftCalibrationLocked ? 1L : 0L;

        g_dcDriftCalibrationCandidateGood =
            driftCalibrationCandidateGood ? 1L : 0L;

        g_dcDriftCalibrationGoodWindows =
            (LONG)driftCalibrationGoodWindows;

        g_dcDriftCalibrationRequiredWindows =
            (LONG)EtherCatDcTuning::DriftCalibrationGoodWindows;

        g_dcDriftCalibrationRobustSequence =
            driftCalibrationRobustSequence;

        g_dcDriftCalibrationRawPpb =
            (LONGLONG)driftCalibrationRawPpb;

        g_dcDriftCalibrationMedianPpb =
            (LONGLONG)driftCalibrationMedianPpb;

        g_dcDriftCalibrationMadPpb =
            (LONGLONG)driftCalibrationMadPpb;

        g_dcDriftCalibrationRawMedianDeviationPpb =
            (LONGLONG)driftCalibrationRawMedianDeviationPpb;

        g_dcDriftCalibrationBaselinePpb =
            (LONGLONG)driftBaselinePpb;

        g_dcDriftCalibrationLockCount =
            (LONG)driftCalibrationLockCount;

        MemoryBarrier();

        InterlockedIncrement(
            &g_dcDriftCalibrationDiagSequence);
    }

    // ---------------------------------------------------------------------
    // 正式控制參數集中區
    //
    // QPC_SCHEDULER_ASSUMED_DRIFT_PPB 是啟動、Trip 與觀測不可信時的安全基準。
    // Real FF 必須連續 3 個合格觀測窗才能 ACTIVE；ACTIVE 每窗最多走 10 ppb，
    // 並限制在 -16000..-5000 ppb，避免單一估測異常直接改變週期。
    // Phase-P 每次使用 wrapped phase error 的 1/8；小於 500 ns 不動作；
    // command、step、累積 offset 各有獨立飽和，防止相位迴路突跳。
    // ---------------------------------------------------------------------
    const int64_t QPC_SCHEDULER_ASSUMED_DRIFT_PPB =
        driftBaselinePpb;

    const int64_t REAL_FF_V0_MIN_PPB =
        EtherCatDcTuning::RealFfMinimumDriftPpb;

    const int64_t REAL_FF_V0_MAX_PPB =
        EtherCatDcTuning::RealFfMaximumDriftPpb;
    const int64_t REAL_FF_V0_MAX_STEP_PPB = 10LL; // 每個約一秒觀測窗最大頻率變更。
    const uint32_t REAL_FF_V0_ARM_WINDOWS = 3U;   // 連續合格 3 窗才進入 ACTIVE。
    const uint32_t REAL_FF_V0_HOLD_BAD_LIMIT = 5U; // HOLD 連續失敗 5 窗即 soft trip。

    // V2 自己從 FALLBACK 回 TRACK 後，再額外要求 5 個完整合格窗，
    // 才允許 Real FF 的 soft observer trip (0x01 only) 重新進 ARM。
    const uint32_t REAL_FF_V0_SOFT_REARM_WINDOWS = 5U;

    const int64_t PHASE_P_ACT_CYCLE_NS =
        EtherCatDcTuning::PdoCycleNs;
    const int64_t PHASE_P_ACT_DIVISOR = 8LL;       // P=1/8；值越小修正越強。
    const int64_t PHASE_P_ACT_MAX_COMMAND_NS = 5000LL; // 原始 P command 絕對限幅。
    const int64_t PHASE_P_ACT_MAX_STEP_NS = 250LL; // 每觀測窗實際 offset 最大變更量。
    const int64_t PHASE_P_ACT_MAX_OFFSET_NS = 120000LL; // 累積 offset 限幅，需小於 125 us。
    const int64_t PHASE_P_ACT_DEADBAND_NS = 500LL; // 誤差在 ±500 ns 內不修正。
    const uint32_t PHASE_P_ACT_ARM_WINDOWS = 3U;   // Gate 連續合格 3 窗才 ACTIVE。
    const uint32_t PHASE_P_ACT_HOLD_RECOVERY_WINDOWS = 3U; // HOLD 恢復條件。

    if (!realFfClampSelfTestDone)
    {
        const int64_t rec[5] =
        {
            EtherCatDcTuning::SchedulerBootstrapDriftPpb,
            EtherCatDcTuning::RealFfMinimumDriftPpb - 44LL,
            EtherCatDcTuning::RealFfMaximumDriftPpb + 200LL,
            EtherCatDcTuning::RealFfMinimumDriftPpb,
            EtherCatDcTuning::RealFfMaximumDriftPpb
        };

        const int64_t expectedDesired[5] =
        {
            EtherCatDcTuning::SchedulerBootstrapDriftPpb,
            EtherCatDcTuning::RealFfMinimumDriftPpb,
            EtherCatDcTuning::RealFfMaximumDriftPpb,
            EtherCatDcTuning::RealFfMinimumDriftPpb,
            EtherCatDcTuning::RealFfMaximumDriftPpb
        };

        const bool expectedClamp[5] =
        { false, true, true, false, false };

        int64_t desired[5] = {};
        bool clamp[5] = {};
        LONG fail = 0;

        for (int i = 0; i < 5; i++)
        {
            clamp[i] =
                rec[i] < REAL_FF_V0_MIN_PPB ||
                rec[i] > REAL_FF_V0_MAX_PPB;

            desired[i] = rec[i];

            if (desired[i] < REAL_FF_V0_MIN_PPB)
                desired[i] = REAL_FF_V0_MIN_PPB;

            if (desired[i] > REAL_FF_V0_MAX_PPB)
                desired[i] = REAL_FF_V0_MAX_PPB;

            if (desired[i] != expectedDesired[i] ||
                clamp[i] != expectedClamp[i])
            {
                fail++;
            }
        }

        InterlockedIncrement(&g_qpcRealFfClampSelfTestSeq);
        g_qpcRealFfClampSelfTestPass = fail == 0 ? 1L : 0L;
        g_qpcRealFfClampSelfTestCases = 5;
        g_qpcRealFfClampSelfTestFail = fail;

        g_qpcRealFfClampSelfTestRec0 = rec[0];
        g_qpcRealFfClampSelfTestDesired0 = desired[0];
        g_qpcRealFfClampSelfTestClamp0 = clamp[0] ? 1L : 0L;

        g_qpcRealFfClampSelfTestRec1 = rec[1];
        g_qpcRealFfClampSelfTestDesired1 = desired[1];
        g_qpcRealFfClampSelfTestClamp1 = clamp[1] ? 1L : 0L;

        g_qpcRealFfClampSelfTestRec2 = rec[2];
        g_qpcRealFfClampSelfTestDesired2 = desired[2];
        g_qpcRealFfClampSelfTestClamp2 = clamp[2] ? 1L : 0L;

        g_qpcRealFfClampSelfTestRec3 = rec[3];
        g_qpcRealFfClampSelfTestDesired3 = desired[3];
        g_qpcRealFfClampSelfTestClamp3 = clamp[3] ? 1L : 0L;

        g_qpcRealFfClampSelfTestRec4 = rec[4];
        g_qpcRealFfClampSelfTestDesired4 = desired[4];
        g_qpcRealFfClampSelfTestClamp4 = clamp[4] ? 1L : 0L;

        MemoryBarrier();
        InterlockedIncrement(&g_qpcRealFfClampSelfTestSeq);

        realFfClampSelfTestDone = true;
    }

    const uint64_t QPC_SCHEDULER_SCALE =
        1000000000ULL;

    const uint64_t QPC_SCHEDULER_GUARD_NS =
        50000ULL;          // 50 us

    const int64_t QPC_SCHEDULER_NEAR_NS =
        10000LL;           // 10 us

    const uint32_t QPC_SCHEDULER_EXTRA_STABLE_CYCLES =
        1000U;             // ~250 ms


    const int64_t QPC_COARSE_REANCHOR_TARGET_MARGIN_NS =
        100000LL;          // 100 us

    const int64_t QPC_COARSE_REANCHOR_TRIGGER_MARGIN_NS =
        50000LL;           // 50 us

    const int64_t QPC_COARSE_REANCHOR_TICK_NS =
        50000LL;           // one 50 us HAL tick

    const uint32_t QPC_COARSE_REANCHOR_MAX_TICKS_PER_EVENT =
        8U;


    if (qpcWakeValid &&
        qpcFrequency > 0)
    {
        // -----------------------------------------------------------------
        // 讀取 Frequency FF V2 snapshot，建立 Real FF 的二次資格閘門。
        //
        // phaseReject bit：
        //   0x01=snapshot 無效或 V2 非 TRACK
        //   0x02=candidate 不合格
        //   0x04=observer 未鎖定
        //   0x08=本相位點未接受
        //   0x10=點數不是 16
        //   0x20=pair 數不是 120
        //   0x40=slope MAD > 150 ppb
        //   0x80=啟動 Drift 尚未鎖定
        //   0x100=transient 後尚未看到新的 V1A observer publication
        // 只有 mask=0 才能累積 ARM 或在 ACTIVE 中更新 drift。
        // -----------------------------------------------------------------
        LONG realPhaseSeq1 = g_qpcPhaseFfV2DiagSequence;
        LONG realPhaseState = 0;
        LONG realPhaseCandidate = 0;
        LONG realPhaseLock = 0;
        LONG realPhasePoint = 0;
        LONG realPhasePoints = 0;
        LONG realPhasePairs = 0;
        LONG realPhaseObserverSeq = 0;
        LONGLONG realPhaseMad = 0;
        LONGLONG realPhaseRecommended = QPC_SCHEDULER_ASSUMED_DRIFT_PPB;
        bool realPhaseSnapshot = false;

        if (realPhaseSeq1 != 0 && !(realPhaseSeq1 & 1))
        {
            MemoryBarrier();
            realPhaseState = g_qpcPhaseFfV2State;
            realPhaseCandidate = g_qpcPhaseFfV2CandidateGood;
            realPhaseLock = g_qpcPhaseFfV2ObserverLocked;
            realPhasePoint = g_qpcPhaseFfV2PointAccepted;
            realPhasePoints = g_qpcPhaseFfV2Points;
            realPhasePairs = g_qpcPhaseFfV2Pairs;
            realPhaseObserverSeq = g_qpcPhaseFfV2ObserverSequence;
            realPhaseMad = g_qpcPhaseFfV2SlopeMadPpb;
            realPhaseRecommended = g_qpcPhaseFfV2RecommendedPpb;
            MemoryBarrier();

            LONG realPhaseSeq2 = g_qpcPhaseFfV2DiagSequence;
            realPhaseSnapshot =
                realPhaseSeq1 == realPhaseSeq2 && !(realPhaseSeq2 & 1);
        }

        realFfV0ClampActive =
            realPhaseRecommended < REAL_FF_V0_MIN_PPB ||
            realPhaseRecommended > REAL_FF_V0_MAX_PPB;

        LONG phaseReject = 0;
        if (!realPhaseSnapshot)
        {
            phaseReject = 0x01;
        }
        else
        {
            if (realPhaseState != 1) phaseReject |= 0x01;
            if (realPhaseCandidate == 0) phaseReject |= 0x02;
            if (realPhaseLock == 0) phaseReject |= 0x04;
            if (realPhasePoint == 0) phaseReject |= 0x08;
            if (realPhasePoints != 16) phaseReject |= 0x10;
            if (realPhasePairs != 120) phaseReject |= 0x20;
            if (realPhaseMad > 150) phaseReject |= 0x40;
            if (!driftCalibrationLocked) phaseReject |= 0x80;

            if (realFfV0FreshObserverRequired)
            {
                const bool freshObserverPublication =
                    realPhaseObserverSeq != 0 &&
                    (realPhaseObserverSeq & 1) == 0 &&
                    realPhaseObserverSeq > realFfV0RecoveryObserverSeqFloor;

                if (!freshObserverPublication)
                {
                    phaseReject |= 0x100;
                }
                else
                {
                    realFfV0FreshObserverRequired = false;
                }
            }
        }

        realFfV0PhaseRejectMask = phaseReject;
        bool realPhaseGood = phaseReject == 0;

        // DC-RX.1：PDO/DC 通訊品質在本 callback 的 combined transaction 完成後
        // 直接寫入 transient HOLD；這裡不再讀取約一秒才發布一次的 RX 診斷快照。

        if (realPhaseSnapshot &&
            realPhaseSeq1 != realFfV0LastPhaseSeq)
        {
            // 狀態機只在 V2 發布「新的完整觀測窗」時走一次；不能在 4 kHz
            // 每個 callback 重複累積 ARM/HOLD 計數。
            realFfV0LastPhaseSeq = realPhaseSeq1;

            if (realFfV0State == 4)
            {
                const bool hardSafetyTrip =
                    (realFfV0TripMask & REAL_FF_V0_HARD_TRIP_MASK) != 0;

                if (hardSafetyTrip)
                {
                    // Timer/bootstrap integrity failures remain latched on the
                    // startup baseline until the RTOS process restarts.
                    realFfV0AppliedPpb = QPC_SCHEDULER_ASSUMED_DRIFT_PPB;
                    realFfV0DesiredPpb = QPC_SCHEDULER_ASSUMED_DRIFT_PPB;
                }
                else
                {
                    // Observer-only trip: freeze, do not throw away the last
                    // frequency that was already proven on the real actuator.
                    realFfV0DesiredPpb = realFfV0AppliedPpb;
                }

                realFfV0LastStepPpb = 0;

                // Observer fallback (0x01) and prolonged DC relock (0x40)
                // are automatically recoverable. Any 0x04/0x08 hard-safety bit
                // remains latched until the RTOS process restarts.
                const bool softRecoverableTripOnly =
                    realFfV0TripMask != 0 &&
                    (realFfV0TripMask & REAL_FF_V0_HARD_TRIP_MASK) == 0 &&
                    (realFfV0TripMask &
                        ~REAL_FF_V0_SOFT_RECOVERABLE_TRIP_MASK) == 0;

                const bool softRecoveryGood =
                    softRecoverableTripOnly &&
                    realPhaseGood &&
                    realPhaseState == 1 &&
                    driftCalibrationLocked &&
                    realFfV0OneShotHealthy &&
                    qpcSchedulerInitialized &&
                    IsRealFfV0CleanRecoveryReady();

                if (softRecoveryGood)
                {
                    // A transient reason is released only after both the
                    // dynamic clean gate and a new good V2 window pass.
                    realFfV0TransientHoldReasonMask = 0;
                    realFfV0SoftRearmGood++;

                    if (realFfV0SoftRearmGood >=
                        REAL_FF_V0_SOFT_REARM_WINDOWS)
                    {
                        // Never jump from TRIP directly to ACTIVE.
                        // Re-enter normal ARM qualification without a frequency jump.
                        realFfV0TripMask &=
                            ~REAL_FF_V0_SOFT_RECOVERABLE_TRIP_MASK;
                        realFfV0State = 1;
                        realFfV0ArmGood = 0;
                        realFfV0HoldGood = 0;
                        realFfV0HoldBad = 0;
                        realFfV0SoftRearmGood = 0;
                        realFfV0TransientHoldReasonMask = 0;
                        realFfV0DesiredPpb = realFfV0AppliedPpb;
                        realFfV0LastStepPpb = 0;
                    }
                }
                else
                {
                    realFfV0SoftRearmGood = 0;
                }
            }
            else if (realFfV0State == 0 || realFfV0State == 1)
            {
                if (realPhaseGood &&
                    driftCalibrationLocked &&
                    realFfV0OneShotHealthy &&
                    qpcSchedulerInitialized &&
                    IsRealFfV0CleanRecoveryReady())
                {
                    realFfV0TransientHoldReasonMask = 0;
                    realFfV0State = 1;
                    realFfV0ArmGood++;

                    if (realFfV0ArmGood >= REAL_FF_V0_ARM_WINDOWS)
                    {
                        realFfV0State = 2;
                        realFfV0TransientHoldReasonMask = 0;
                        ResetRealFfV0RecoveryProfileAfterActive();
                    }
                }
                else
                {
                    realFfV0State = 0;
                    realFfV0ArmGood = 0;
                }
            }
            else if (realFfV0State == 2)
            {
                if (!realPhaseGood)
                {
                    realFfV0LastPhaseRejectMask = phaseReject;
                    realFfV0LastStepPpb = 0;

                    if (realPhaseState == 3)
                    {
                        realFfV0State = 4;
                        realFfV0DesiredPpb = realFfV0AppliedPpb;
                        realFfV0TripMask |= 0x01;
                        realFfV0HistoryMask |= 0x01;
                        realFfV0TransientHoldReasonMask = 0;
                        realFfV0TripCount++;
                        realFfV0SoftRearmGood = 0;
                    }
                    else
                    {
                        realFfV0State = 3;
                        realFfV0DesiredPpb = realFfV0AppliedPpb;
                        realFfV0HoldGood = 0;
                        realFfV0HoldBad = 1;
                        realFfV0HoldEntries++;
                    }
                }
                else
                {
                    // 合格建議值先做絕對限幅，再以每觀測窗 ±10 ppb 慢速靠近。
                    int64_t desired = (int64_t)realPhaseRecommended;
                    if (desired < REAL_FF_V0_MIN_PPB) desired = REAL_FF_V0_MIN_PPB;
                    if (desired > REAL_FF_V0_MAX_PPB) desired = REAL_FF_V0_MAX_PPB;
                    realFfV0DesiredPpb = desired;

                    int64_t step = realFfV0DesiredPpb - realFfV0AppliedPpb;
                    if (step > REAL_FF_V0_MAX_STEP_PPB) step = REAL_FF_V0_MAX_STEP_PPB;
                    if (step < -REAL_FF_V0_MAX_STEP_PPB) step = -REAL_FF_V0_MAX_STEP_PPB;

                    realFfV0AppliedPpb += step;
                    realFfV0LastStepPpb = step;
                }
            }
            else if (realFfV0State == 3)
            {
                realFfV0LastStepPpb = 0;
                realFfV0DesiredPpb = realFfV0AppliedPpb;

                const bool holdInfrastructureGood =
                    driftCalibrationLocked &&
                    realFfV0OneShotHealthy &&
                    qpcSchedulerInitialized &&
                    IsRealFfV0CleanRecoveryReady();

                // While a PDO/scheduler interruption is rebuilding its observer
                // windows, stay in HOLD without counting those expected WARMUP
                // publications as observer failures.
                if (realFfV0TransientHoldReasonMask != 0)
                {
                    realFfV0HoldGood = 0;
                    realFfV0HoldBad = 0;

                    if (holdInfrastructureGood && realPhaseGood)
                    {
                        realFfV0TransientHoldReasonMask = 0;
                    }
                }

                if (realFfV0TransientHoldReasonMask != 0 ||
                    !holdInfrastructureGood)
                {
                    realFfV0HoldGood = 0;
                    realFfV0HoldBad = 0;
                }
                else if (realPhaseGood)
                {
                    realFfV0HoldGood++;
                    realFfV0HoldBad = 0;

                    if (realFfV0HoldGood >=
                        realFfV0HoldRecoveryWindowsRequired)
                    {
                        realFfV0State = 2;
                        realFfV0HoldGood = 0;
                        realFfV0TransientHoldReasonMask = 0;
                        realFfV0DesiredPpb = realFfV0AppliedPpb;
                        ResetRealFfV0RecoveryProfileAfterActive();
                    }
                }
                else
                {
                    realFfV0LastPhaseRejectMask = phaseReject;
                    realFfV0HoldGood = 0;
                    realFfV0HoldBad++;

                    if (realPhaseState == 3 ||
                        realFfV0HoldBad >= REAL_FF_V0_HOLD_BAD_LIMIT)
                    {
                        realFfV0State = 4;
                        realFfV0DesiredPpb = realFfV0AppliedPpb;
                        realFfV0TripMask |= 0x01;
                        realFfV0HistoryMask |= 0x01;
                        realFfV0TransientHoldReasonMask = 0;
                        realFfV0TripCount++;
                        realFfV0SoftRearmGood = 0;
                    }
                }
            }
        }

        // ---------------------------------------------------------
        // 250 us nominal QPC period.
        //
        // At 3 GHz:
        //     750000 counts
        //
        // 實際修正由本次啟動 Baseline 或 Real FF Applied 值決定。
        // ---------------------------------------------------------

        uint64_t nominalPeriodCounts =
            qpcFrequency /
            4000ULL;


        int64_t periodScaledCounts =
            (int64_t)
            nominalPeriodCounts *
            (
                1000000000LL +
                realFfV0AppliedPpb
                );

        int64_t fixedRefScaledCounts =
            (int64_t)
            nominalPeriodCounts *
            (
                1000000000LL +
                QPC_SCHEDULER_ASSUMED_DRIFT_PPB
                );


        uint64_t periodWholeCounts =
            0;

        uint64_t periodFractionScaled =
            0;

        uint64_t fixedRefWholeCounts = 0;
        uint64_t fixedRefFractionScaled = 0;


        if (periodScaledCounts > 0)
        {
            periodWholeCounts =
                (uint64_t)
                (
                    periodScaledCounts /
                    1000000000LL
                    );


            periodFractionScaled =
                (uint64_t)
                (
                    periodScaledCounts %
                    1000000000LL
                    );
        }

        if (fixedRefScaledCounts > 0)
        {
            fixedRefWholeCounts =
                (uint64_t)(fixedRefScaledCounts / 1000000000LL);
            fixedRefFractionScaled =
                (uint64_t)(fixedRefScaledCounts % 1000000000LL);
        }


        uint64_t actualWakeQpc =
            (uint64_t)
            qpcWake.QuadPart;


        // ---------------------------------------------------------
        // Stability gate.
        //
        // QPC<->DC producer increments its sequence by 2 for each
        // complete ~1 second snapshot.
        //
        // >= 4 therefore means at least two complete windows exist.
        // ---------------------------------------------------------

        LONG qpcDcSequenceForAnchor =
            g_qpcDcDiagSequence;


        bool qpcDcStableForAnchor =
            (
                g_qpcDcValid != 0 &&
                qpcDcSequenceForAnchor >= 4 &&
                (qpcDcSequenceForAnchor & 1) == 0
                );


        if (!qpcSchedulerInitialized)
        {
            phasePActV0State = 0;
            phasePActV0ArmGood = 0;
            phasePActV0HoldGood = 0;
            phasePActV0OffsetNs = 0;
            phasePActV0LastStepNs = 0;

            qpcFixedRefInitialized = false;

            // -----------------------------------------------------
            // Shadow Live-FF timeline follows real scheduler
            // lifecycle only for diagnostic alignment.
            //
            // No real scheduler state is changed here.
            // -----------------------------------------------------

            qpcLiveFfInitialized =
                false;

            qpcLiveFfHasPreviousMode =
                false;

            qpcLiveFfWindowSamples =
                0;

            qpcLiveFfTrustedCyclesWindow =
                0;

            qpcLiveFfFallbackCyclesWindow =
                0;

            qpcLiveFfModeSwitchesWindow =
                0;


            // Phase-FF V2 shadow follows the same lifecycle.
            qpcPhaseFfV2Initialized =
                false;

            qpcPhaseFfV2State =
                0;

            qpcPhaseFfV2DesiredPpb =
                QPC_SCHEDULER_ASSUMED_DRIFT_PPB;

            qpcPhaseFfV2AppliedPpb =
                QPC_SCHEDULER_ASSUMED_DRIFT_PPB;

            qpcPhaseFfV2LastObserverSequence =
                0;

            qpcPhaseFfV2WarmupGoodCount =
                0;

            qpcPhaseFfV2BadCount =
                0;

            qpcPhaseFfV2RecoveryGoodCount =
                0;

            qpcPhaseFfV2WindowSamples =
                0;

            qpcPhaseFfV2TrackCyclesWindow =
                0;

            qpcPhaseFfV2HoldCyclesWindow =
                0;

            qpcPhaseFfV2FallbackCyclesWindow =
                0;

            qpcPhaseFfV2StateSwitchesWindow =
                0;


            g_qpcSchedulerState =
                0; // WAIT_STABLE


            if (qpcDcStableForAnchor)
            {
                if (qpcSchedulerStableWakeCycles <
                    QPC_SCHEDULER_EXTRA_STABLE_CYCLES)
                {
                    qpcSchedulerStableWakeCycles++;
                }
            }
            else
            {
                qpcSchedulerStableWakeCycles =
                    0;
            }


            g_qpcSchedulerStableWaitCycles =
                (LONG)
                qpcSchedulerStableWakeCycles;


            if (qpcDcStableForAnchor &&
                qpcSchedulerStableWakeCycles >=
                QPC_SCHEDULER_EXTRA_STABLE_CYCLES)
            {
                // -------------------------------------------------
                // Re-anchor with a deliberate 50 us fine-wait guard.
                //
                // Target is in the FUTURE relative to current wake:
                //
                //     Error = Actual - Target ~= -50000 ns
                //
                // From here the free-running coarse wake is expected
                // to gain about +8.3 us/s on the target.
                // -------------------------------------------------

                uint64_t guardCounts =
                    (
                        qpcFrequency *
                        QPC_SCHEDULER_GUARD_NS
                        )
                    /
                    1000000000ULL;


                qpcSchedulerAnchorQpc =
                    actualWakeQpc;


                qpcSchedulerAnchorDcDiagSequence =
                    qpcDcSequenceForAnchor;


                qpcSchedulerTargetQpc =
                    actualWakeQpc +
                    guardCounts;


                qpcSchedulerFractionRemainder =
                    0;

                qpcFixedRefTargetQpc = qpcSchedulerTargetQpc;
                qpcFixedRefFractionRemainder = 0;
                qpcFixedRefInitialized = true;


                qpcSchedulerWindowSamples =
                    0;

                qpcSchedulerWindowStartErrorNs =
                    0;

                qpcSchedulerWindowEndErrorNs =
                    0;

                qpcSchedulerWindowMinErrorNs =
                    0;

                qpcSchedulerWindowMaxErrorNs =
                    0;


                qpcSchedulerEarlyCount =
                    0;

                qpcSchedulerNearCount =
                    0;

                qpcSchedulerLateCount =
                    0;


                qpcSchedulerInitialized =
                    true;


                // Publish anchor metadata immediately into globals.
                // Sequence is not advanced here; the normal 4000-cycle
                // snapshot publisher below remains the visible window.
                g_qpcSchedulerAnchorQpc =
                    (LONGLONG)
                    qpcSchedulerAnchorQpc;

                g_qpcSchedulerAnchorDcDiagSequence =
                    qpcSchedulerAnchorDcDiagSequence;

                g_qpcSchedulerGuardNs =
                    (LONGLONG)
                    QPC_SCHEDULER_GUARD_NS;

                g_qpcSchedulerStableWaitCycles =
                    (LONG)
                    qpcSchedulerStableWakeCycles;
            }
        }
        else if (periodWholeCounts > 0)
        {
            // -----------------------------------------------------
            // Advance the target by one DC-equivalent QPC cycle.
            // -----------------------------------------------------

            qpcSchedulerTargetQpc +=
                periodWholeCounts;


            qpcSchedulerFractionRemainder +=
                periodFractionScaled;


            if (qpcSchedulerFractionRemainder >=
                QPC_SCHEDULER_SCALE)
            {
                qpcSchedulerTargetQpc++;

                qpcSchedulerFractionRemainder -=
                    QPC_SCHEDULER_SCALE;
            }

            if (qpcFixedRefInitialized && fixedRefWholeCounts > 0)
            {
                qpcFixedRefTargetQpc += fixedRefWholeCounts;
                qpcFixedRefFractionRemainder += fixedRefFractionScaled;

                if (qpcFixedRefFractionRemainder >= QPC_SCHEDULER_SCALE)
                {
                    qpcFixedRefTargetQpc++;
                    qpcFixedRefFractionRemainder -= QPC_SCHEDULER_SCALE;
                }
            }


            // -----------------------------------------------------
            // Error:
            //
            // negative = coarse wake is EARLY -> fine wait possible
            // zero     = exactly at target
            // positive = coarse wake is LATE -> fine wait cannot fix
            // -----------------------------------------------------

            int64_t errorCounts =
                (int64_t)
                actualWakeQpc -
                (int64_t)
                qpcSchedulerTargetQpc;


            int64_t errorNs =
                (
                    errorCounts *
                    1000000000LL
                    )
                /
                (int64_t)
                qpcFrequency;


            // =============================================================
            // Trusted Drift -> Live Feed-Forward Dry Run V1
            //
            // SHADOW target only.
            //
            // Critical design rule:
            //
            //     NEVER recalculate all history from an old anchor
            //     using a newly changed drift value.
            //
            // Instead:
            //
            //     ShadowTarget[n+1]
            //       = ShadowTarget[n]
            //       + Period(CurrentAppliedDrift)
            //
            // Therefore a Trusted Drift change affects only future
            // frequency. It cannot create a retroactive phase jump.
            // =============================================================

            LONG liveTrustedSequenceBefore =
                g_qpcDcTrustedDiagSequence;


            bool liveTrustedSnapshotValid =
                false;

            bool liveTrustedValid =
                false;

            LONG liveTrustedState =
                0;

            int64_t liveTrustedDriftPpb =
                QPC_SCHEDULER_ASSUMED_DRIFT_PPB;


            if (liveTrustedSequenceBefore != 0 &&
                (liveTrustedSequenceBefore & 1) == 0)
            {
                MemoryBarrier();


                LONG trustedValidSnapshot =
                    g_qpcDcTrustedValid;

                LONG trustedStateSnapshot =
                    g_qpcDcTrustedState;

                LONGLONG trustedDriftSnapshot =
                    g_qpcDcTrustedDriftPpb;


                MemoryBarrier();


                LONG liveTrustedSequenceAfter =
                    g_qpcDcTrustedDiagSequence;


                if (liveTrustedSequenceBefore ==
                    liveTrustedSequenceAfter &&
                    (liveTrustedSequenceAfter & 1) == 0)
                {
                    liveTrustedSnapshotValid =
                        true;

                    liveTrustedValid =
                        trustedValidSnapshot != 0;

                    liveTrustedState =
                        trustedStateSnapshot;

                    liveTrustedDriftPpb =
                        (int64_t)
                        trustedDriftSnapshot;
                }
            }


            bool liveUseTrusted =
                (
                    liveTrustedSnapshotValid &&
                    liveTrustedValid
                    );


            int64_t liveAppliedDriftPpb =
                liveUseTrusted
                ? liveTrustedDriftPpb
                : QPC_SCHEDULER_ASSUMED_DRIFT_PPB;


            // Defensive range only for the SHADOW diagnostic.
            // A corrupt/unexpected Trusted value must not overflow
            // the local period arithmetic.
            if (liveAppliedDriftPpb < -1000000LL ||
                liveAppliedDriftPpb > 1000000LL)
            {
                liveAppliedDriftPpb =
                    QPC_SCHEDULER_ASSUMED_DRIFT_PPB;

                liveUseTrusted =
                    false;
            }


            int64_t livePeriodScaledCounts =
                (int64_t)
                nominalPeriodCounts *
                (
                    1000000000LL +
                    liveAppliedDriftPpb
                    );


            uint64_t livePeriodWholeCounts =
                0;

            uint64_t livePeriodFractionScaled =
                0;


            if (livePeriodScaledCounts > 0)
            {
                livePeriodWholeCounts =
                    (uint64_t)
                    (
                        livePeriodScaledCounts /
                        1000000000LL
                        );

                livePeriodFractionScaled =
                    (uint64_t)
                    (
                        livePeriodScaledCounts %
                        1000000000LL
                        );
            }


            if (!qpcLiveFfInitialized)
            {
                // -------------------------------------------------
                // Re-anchor SHADOW to the CURRENT real target only.
                //
                // This gives:
                //
                //     ShadowTarget == FixedTarget
                //
                // at initialization, so there is NO target jump.
                // -------------------------------------------------

                qpcLiveFfTargetQpc =
                    qpcSchedulerTargetQpc;

                qpcLiveFfFractionRemainder =
                    qpcSchedulerFractionRemainder;

                qpcLiveFfInitialized =
                    true;

                qpcLiveFfInitCount++;

                qpcLiveFfHasPreviousMode =
                    true;

                qpcLiveFfPreviousTrustedMode =
                    liveUseTrusted;
            }
            else if (livePeriodWholeCounts > 0)
            {
                // -------------------------------------------------
                // Incremental shadow feed-forward update.
                // -------------------------------------------------

                qpcLiveFfTargetQpc +=
                    livePeriodWholeCounts;

                qpcLiveFfFractionRemainder +=
                    livePeriodFractionScaled;


                if (qpcLiveFfFractionRemainder >=
                    QPC_SCHEDULER_SCALE)
                {
                    qpcLiveFfTargetQpc++;

                    qpcLiveFfFractionRemainder -=
                        QPC_SCHEDULER_SCALE;
                }


                if (qpcLiveFfHasPreviousMode &&
                    qpcLiveFfPreviousTrustedMode !=
                    liveUseTrusted)
                {
                    qpcLiveFfModeSwitchesWindow++;
                    qpcLiveFfTotalModeSwitches++;
                }


                qpcLiveFfPreviousTrustedMode =
                    liveUseTrusted;

                qpcLiveFfHasPreviousMode =
                    true;
            }


            int64_t liveShadowErrorCounts =
                (int64_t)
                actualWakeQpc -
                (int64_t)
                qpcLiveFfTargetQpc;


            int64_t liveShadowErrorNs =
                (
                    liveShadowErrorCounts *
                    1000000000LL
                    )
                /
                (int64_t)
                qpcFrequency;


            int64_t liveTargetDeltaCounts =
                (int64_t)
                qpcLiveFfTargetQpc -
                (int64_t)
                qpcSchedulerTargetQpc;


            int64_t liveTargetDeltaNs =
                (
                    liveTargetDeltaCounts *
                    1000000000LL
                    )
                /
                (int64_t)
                qpcFrequency;


            int64_t liveAppliedPeriodFfPs =
                (
                    250000LL *
                    liveAppliedDriftPpb
                    )
                /
                1000000LL;


            if (liveUseTrusted)
            {
                qpcLiveFfTrustedCyclesWindow++;
                qpcLiveFfTotalTrustedCycles++;
            }
            else
            {
                qpcLiveFfFallbackCyclesWindow++;
                qpcLiveFfTotalFallbackCycles++;
            }


            if (qpcLiveFfWindowSamples == 0)
            {
                qpcLiveFfShadowWindowStartErrorNs =
                    liveShadowErrorNs;

                qpcLiveFfShadowWindowMinErrorNs =
                    liveShadowErrorNs;

                qpcLiveFfShadowWindowMaxErrorNs =
                    liveShadowErrorNs;

                qpcLiveFfTargetDeltaWindowStartNs =
                    liveTargetDeltaNs;
            }


            qpcLiveFfShadowWindowEndErrorNs =
                liveShadowErrorNs;

            qpcLiveFfTargetDeltaWindowEndNs =
                liveTargetDeltaNs;


            if (liveShadowErrorNs <
                qpcLiveFfShadowWindowMinErrorNs)
            {
                qpcLiveFfShadowWindowMinErrorNs =
                    liveShadowErrorNs;
            }


            if (liveShadowErrorNs >
                qpcLiveFfShadowWindowMaxErrorNs)
            {
                qpcLiveFfShadowWindowMaxErrorNs =
                    liveShadowErrorNs;
            }


            qpcLiveFfWindowSamples++;


            if (qpcLiveFfWindowSamples >= 4000U)
            {
                int64_t liveShadowWindowDeltaErrorNs =
                    qpcLiveFfShadowWindowEndErrorNs -
                    qpcLiveFfShadowWindowStartErrorNs;


                int64_t liveTargetDeltaWindowDeltaNs =
                    qpcLiveFfTargetDeltaWindowEndNs -
                    qpcLiveFfTargetDeltaWindowStartNs;


                InterlockedIncrement(
                    &g_qpcLiveFfDiagSequence);


                g_qpcLiveFfInitialized =
                    qpcLiveFfInitialized
                    ? 1L
                    : 0L;

                g_qpcLiveFfMode =
                    liveUseTrusted
                    ? 1L
                    : 0L;

                g_qpcLiveFfTrustedSnapshotValid =
                    liveTrustedSnapshotValid
                    ? 1L
                    : 0L;

                g_qpcLiveFfTrustedValid =
                    liveTrustedValid
                    ? 1L
                    : 0L;

                g_qpcLiveFfTrustedState =
                    liveTrustedState;

                g_qpcLiveFfTrustedDriftPpb =
                    (LONGLONG)
                    liveTrustedDriftPpb;

                g_qpcLiveFfAppliedDriftPpb =
                    (LONGLONG)
                    liveAppliedDriftPpb;

                g_qpcLiveFfAppliedPeriodFfPs =
                    (LONGLONG)
                    liveAppliedPeriodFfPs;


                g_qpcLiveFfFixedErrorNs =
                    (LONGLONG)
                    errorNs;

                g_qpcLiveFfShadowErrorNs =
                    (LONGLONG)
                    liveShadowErrorNs;

                g_qpcLiveFfShadowVsFixedTargetNs =
                    (LONGLONG)
                    liveTargetDeltaNs;


                g_qpcLiveFfShadowWindowStartErrorNs =
                    (LONGLONG)
                    qpcLiveFfShadowWindowStartErrorNs;

                g_qpcLiveFfShadowWindowEndErrorNs =
                    (LONGLONG)
                    qpcLiveFfShadowWindowEndErrorNs;

                g_qpcLiveFfShadowWindowDeltaErrorNs =
                    (LONGLONG)
                    liveShadowWindowDeltaErrorNs;

                g_qpcLiveFfShadowWindowMinErrorNs =
                    (LONGLONG)
                    qpcLiveFfShadowWindowMinErrorNs;

                g_qpcLiveFfShadowWindowMaxErrorNs =
                    (LONGLONG)
                    qpcLiveFfShadowWindowMaxErrorNs;


                g_qpcLiveFfTargetDeltaWindowStartNs =
                    (LONGLONG)
                    qpcLiveFfTargetDeltaWindowStartNs;

                g_qpcLiveFfTargetDeltaWindowEndNs =
                    (LONGLONG)
                    qpcLiveFfTargetDeltaWindowEndNs;

                g_qpcLiveFfTargetDeltaWindowDeltaNs =
                    (LONGLONG)
                    liveTargetDeltaWindowDeltaNs;


                g_qpcLiveFfTrustedCyclesWindow =
                    (LONG)
                    qpcLiveFfTrustedCyclesWindow;

                g_qpcLiveFfFallbackCyclesWindow =
                    (LONG)
                    qpcLiveFfFallbackCyclesWindow;

                g_qpcLiveFfModeSwitchesWindow =
                    (LONG)
                    qpcLiveFfModeSwitchesWindow;


                g_qpcLiveFfTotalTrustedCycles =
                    (LONGLONG)
                    qpcLiveFfTotalTrustedCycles;

                g_qpcLiveFfTotalFallbackCycles =
                    (LONGLONG)
                    qpcLiveFfTotalFallbackCycles;

                g_qpcLiveFfTotalModeSwitches =
                    (LONG)
                    qpcLiveFfTotalModeSwitches;

                g_qpcLiveFfInitCount =
                    (LONG)
                    qpcLiveFfInitCount;

                g_qpcLiveFfSamples =
                    (LONG)
                    qpcLiveFfWindowSamples;


                MemoryBarrier();


                InterlockedIncrement(
                    &g_qpcLiveFfDiagSequence);


                qpcLiveFfWindowSamples =
                    0;

                qpcLiveFfShadowWindowStartErrorNs =
                    0;

                qpcLiveFfShadowWindowEndErrorNs =
                    0;

                qpcLiveFfShadowWindowMinErrorNs =
                    0;

                qpcLiveFfShadowWindowMaxErrorNs =
                    0;

                qpcLiveFfTargetDeltaWindowStartNs =
                    0;

                qpcLiveFfTargetDeltaWindowEndNs =
                    0;

                qpcLiveFfTrustedCyclesWindow =
                    0;

                qpcLiveFfFallbackCyclesWindow =
                    0;

                qpcLiveFfModeSwitchesWindow =
                    0;
            }


            // =============================================================
            // Frequency FF Dry Run V2
            // Phase-derived strict qualification + slow slew
            //
            // This is a THIRD independent shadow timeline.
            //
            // It never writes:
            //   qpcSchedulerTargetQpc
            //   qpcLiveFfTargetQpc
            //   requestedRelative100ns
            //   real one-shot timer
            // =============================================================

            LONG phaseFfObserverSeqBefore =
                g_qpcDcPhaseResidualV1ADiagSequence;


            bool phaseFfObserverSnapshotValid =
                false;

            LONG phaseFfObserverLocked =
                0;

            LONG phaseFfPointAccepted =
                0;

            LONG phaseFfPoints =
                0;

            LONG phaseFfPairs =
                0;

            int64_t phaseFfRecommendedPpb =
                QPC_SCHEDULER_ASSUMED_DRIFT_PPB;

            int64_t phaseFfSlopeMadPpb =
                0;


            if (phaseFfObserverSeqBefore != 0 &&
                (phaseFfObserverSeqBefore & 1) == 0)
            {
                MemoryBarrier();


                phaseFfObserverLocked =
                    g_qpcDcPhaseResidualV1ALocked;

                phaseFfPointAccepted =
                    g_qpcDcPhaseResidualV1APointAccepted;

                phaseFfPoints =
                    g_qpcDcPhaseResidualV1APointBufferCount;

                phaseFfPairs =
                    g_qpcDcPhaseResidualV1APairSlopeCount;

                phaseFfRecommendedPpb =
                    (int64_t)
                    g_qpcDcPhaseResidualV1ARecommendedSchedulerPpb;

                phaseFfSlopeMadPpb =
                    (int64_t)
                    g_qpcDcPhaseResidualV1ASlopeMadPpb;


                MemoryBarrier();


                LONG phaseFfObserverSeqAfter =
                    g_qpcDcPhaseResidualV1ADiagSequence;


                if (phaseFfObserverSeqBefore ==
                    phaseFfObserverSeqAfter &&
                    (phaseFfObserverSeqAfter & 1) == 0)
                {
                    phaseFfObserverSnapshotValid =
                        true;
                }
            }


            int64_t phaseFfRecommendedDeviationPpb =
                phaseFfRecommendedPpb -
                QPC_SCHEDULER_ASSUMED_DRIFT_PPB;


            int64_t phaseFfRecommendedAbsDeviationPpb =
                phaseFfRecommendedDeviationPpb >= 0
                ? phaseFfRecommendedDeviationPpb
                : -phaseFfRecommendedDeviationPpb;


            bool phaseFfCandidateGood =
                (
                    driftCalibrationLocked &&
                    phaseFfObserverSnapshotValid &&
                    phaseFfObserverLocked != 0 &&
                    phaseFfPointAccepted != 0 &&
                    phaseFfPoints == 16 &&
                    phaseFfPairs == 120 &&
                    phaseFfSlopeMadPpb <=
                    QPC_PHASE_FF_V2_MAX_SLOPE_MAD_PPB &&
                    phaseFfRecommendedAbsDeviationPpb <=
                    QPC_PHASE_FF_V2_MAX_RECOMMENDED_DEVIATION_PPB
                    );


            int64_t phaseFfSlewAppliedPpb =
                0;


            // Process the qualification/state machine only once per
            // new V1A observer publication (~1 second).
            if (phaseFfObserverSnapshotValid &&
                phaseFfObserverSeqBefore !=
                qpcPhaseFfV2LastObserverSequence)
            {
                qpcPhaseFfV2LastObserverSequence =
                    phaseFfObserverSeqBefore;


                LONG phaseFfStateBefore =
                    qpcPhaseFfV2State;

                int64_t phaseFfAppliedBeforePpb =
                    qpcPhaseFfV2AppliedPpb;


                if (qpcPhaseFfV2State == 0)
                {
                    // WARMUP
                    if (phaseFfCandidateGood)
                    {
                        qpcPhaseFfV2WarmupGoodCount++;


                        if (qpcPhaseFfV2WarmupGoodCount >=
                            QPC_PHASE_FF_V2_WARMUP_GOOD_WINDOWS)
                        {
                            qpcPhaseFfV2State =
                                1; // TRACK

                            qpcPhaseFfV2DesiredPpb =
                                phaseFfRecommendedPpb;

                            qpcPhaseFfV2BadCount =
                                0;

                            qpcPhaseFfV2RecoveryGoodCount =
                                0;
                        }
                    }
                    else
                    {
                        qpcPhaseFfV2WarmupGoodCount =
                            0;
                    }
                }
                else if (qpcPhaseFfV2State == 1)
                {
                    // TRACK
                    if (phaseFfCandidateGood)
                    {
                        qpcPhaseFfV2DesiredPpb =
                            phaseFfRecommendedPpb;

                        qpcPhaseFfV2BadCount =
                            0;

                        qpcPhaseFfV2RecoveryGoodCount =
                            0;
                    }
                    else
                    {
                        qpcPhaseFfV2State =
                            2; // HOLD

                        qpcPhaseFfV2BadCount =
                            1;

                        qpcPhaseFfV2RecoveryGoodCount =
                            0;

                        // Freeze at current applied frequency.
                        qpcPhaseFfV2DesiredPpb =
                            qpcPhaseFfV2AppliedPpb;
                    }
                }
                else if (qpcPhaseFfV2State == 2)
                {
                    // HOLD
                    if (phaseFfCandidateGood)
                    {
                        qpcPhaseFfV2RecoveryGoodCount++;


                        if (qpcPhaseFfV2RecoveryGoodCount >=
                            QPC_PHASE_FF_V2_HOLD_RECOVERY_GOOD_WINDOWS)
                        {
                            qpcPhaseFfV2State =
                                1; // TRACK

                            qpcPhaseFfV2DesiredPpb =
                                phaseFfRecommendedPpb;

                            qpcPhaseFfV2BadCount =
                                0;

                            qpcPhaseFfV2RecoveryGoodCount =
                                0;
                        }
                    }
                    else
                    {
                        qpcPhaseFfV2RecoveryGoodCount =
                            0;

                        qpcPhaseFfV2BadCount++;


                        if (qpcPhaseFfV2BadCount >=
                            QPC_PHASE_FF_V2_HOLD_MAX_BAD_WINDOWS)
                        {
                            qpcPhaseFfV2State =
                                3; // FALLBACK

                            qpcPhaseFfV2DesiredPpb =
                                QPC_SCHEDULER_ASSUMED_DRIFT_PPB;

                            qpcPhaseFfV2AppliedPpb =
                                QPC_SCHEDULER_ASSUMED_DRIFT_PPB;

                            qpcPhaseFfV2RecoveryGoodCount =
                                0;
                        }
                    }
                }
                else
                {
                    // FALLBACK
                    qpcPhaseFfV2DesiredPpb =
                        QPC_SCHEDULER_ASSUMED_DRIFT_PPB;

                    qpcPhaseFfV2AppliedPpb =
                        QPC_SCHEDULER_ASSUMED_DRIFT_PPB;


                    if (phaseFfCandidateGood)
                    {
                        qpcPhaseFfV2RecoveryGoodCount++;


                        if (qpcPhaseFfV2RecoveryGoodCount >=
                            QPC_PHASE_FF_V2_FALLBACK_RECOVERY_GOOD_WINDOWS)
                        {
                            qpcPhaseFfV2State =
                                1; // TRACK

                            qpcPhaseFfV2DesiredPpb =
                                phaseFfRecommendedPpb;

                            qpcPhaseFfV2BadCount =
                                0;

                            qpcPhaseFfV2RecoveryGoodCount =
                                0;
                        }
                    }
                    else
                    {
                        qpcPhaseFfV2RecoveryGoodCount =
                            0;
                    }
                }


                // Slow frequency slew only while TRACK.
                if (qpcPhaseFfV2State == 1)
                {
                    int64_t phaseFfDeltaPpb =
                        qpcPhaseFfV2DesiredPpb -
                        qpcPhaseFfV2AppliedPpb;


                    if (phaseFfDeltaPpb >
                        QPC_PHASE_FF_V2_MAX_SLEW_PPB_PER_OBSERVER_WINDOW)
                    {
                        phaseFfDeltaPpb =
                            QPC_PHASE_FF_V2_MAX_SLEW_PPB_PER_OBSERVER_WINDOW;
                    }
                    else if (phaseFfDeltaPpb <
                        -QPC_PHASE_FF_V2_MAX_SLEW_PPB_PER_OBSERVER_WINDOW)
                    {
                        phaseFfDeltaPpb =
                            -QPC_PHASE_FF_V2_MAX_SLEW_PPB_PER_OBSERVER_WINDOW;
                    }


                    qpcPhaseFfV2AppliedPpb +=
                        phaseFfDeltaPpb;

                    phaseFfSlewAppliedPpb =
                        phaseFfDeltaPpb;
                }


                qpcPhaseFfV2LastAppliedStepPpb =
                    qpcPhaseFfV2AppliedPpb - phaseFfAppliedBeforePpb;


                if (phaseFfStateBefore !=
                    qpcPhaseFfV2State)
                {
                    qpcPhaseFfV2StateSwitchesWindow++;
                    qpcPhaseFfV2TotalStateSwitches++;


                    if (phaseFfStateBefore == 0 &&
                        qpcPhaseFfV2State == 1)
                    {
                        qpcPhaseFfV2WarmupToTrackTotal++;
                    }
                    else if (phaseFfStateBefore == 1 &&
                        qpcPhaseFfV2State == 2)
                    {
                        qpcPhaseFfV2TrackToHoldTotal++;
                    }
                    else if (phaseFfStateBefore == 2 &&
                        qpcPhaseFfV2State == 1)
                    {
                        qpcPhaseFfV2HoldToTrackTotal++;
                    }
                    else if (phaseFfStateBefore == 2 &&
                        qpcPhaseFfV2State == 3)
                    {
                        qpcPhaseFfV2HoldToFallbackTotal++;
                    }
                    else if (phaseFfStateBefore == 3 &&
                        qpcPhaseFfV2State == 1)
                    {
                        qpcPhaseFfV2FallbackToTrackTotal++;
                    }
                }
            }


            // ---------------------------------------------------------
            // Incremental Phase-FF V2 target.
            // ---------------------------------------------------------

            int64_t phaseFfPeriodScaledCounts =
                (int64_t)
                nominalPeriodCounts *
                (
                    1000000000LL +
                    qpcPhaseFfV2AppliedPpb
                    );


            uint64_t phaseFfPeriodWholeCounts =
                0;

            uint64_t phaseFfPeriodFractionScaled =
                0;


            if (phaseFfPeriodScaledCounts > 0)
            {
                phaseFfPeriodWholeCounts =
                    (uint64_t)
                    (
                        phaseFfPeriodScaledCounts /
                        1000000000LL
                        );

                phaseFfPeriodFractionScaled =
                    (uint64_t)
                    (
                        phaseFfPeriodScaledCounts %
                        1000000000LL
                        );
            }


            if (!qpcPhaseFfV2Initialized)
            {
                qpcPhaseFfV2TargetQpc =
                    qpcSchedulerTargetQpc;

                qpcPhaseFfV2FractionRemainder =
                    qpcSchedulerFractionRemainder;

                qpcPhaseFfV2Initialized =
                    true;
            }
            else if (phaseFfPeriodWholeCounts > 0)
            {
                qpcPhaseFfV2TargetQpc +=
                    phaseFfPeriodWholeCounts;

                qpcPhaseFfV2FractionRemainder +=
                    phaseFfPeriodFractionScaled;


                if (qpcPhaseFfV2FractionRemainder >=
                    QPC_SCHEDULER_SCALE)
                {
                    qpcPhaseFfV2TargetQpc++;

                    qpcPhaseFfV2FractionRemainder -=
                        QPC_SCHEDULER_SCALE;
                }
            }


            int64_t phaseFfWakeErrorCounts =
                (int64_t)
                actualWakeQpc -
                (int64_t)
                qpcPhaseFfV2TargetQpc;


            int64_t phaseFfWakeErrorNs =
                (
                    phaseFfWakeErrorCounts *
                    1000000000LL
                    )
                /
                (int64_t)
                qpcFrequency;


            int64_t phaseFfTargetVsFixedCounts =
                (int64_t)
                qpcPhaseFfV2TargetQpc -
                (int64_t)
                qpcSchedulerTargetQpc;


            int64_t phaseFfTargetVsFixedNs =
                (
                    phaseFfTargetVsFixedCounts *
                    1000000000LL
                    )
                /
                (int64_t)
                qpcFrequency;


            if (qpcPhaseFfV2WindowSamples == 0)
            {
                qpcPhaseFfV2TargetDeltaWindowStartNs =
                    phaseFfTargetVsFixedNs;
            }


            qpcPhaseFfV2TargetDeltaWindowEndNs =
                phaseFfTargetVsFixedNs;


            if (qpcPhaseFfV2State == 1)
            {
                qpcPhaseFfV2TrackCyclesWindow++;
            }
            else if (qpcPhaseFfV2State == 2)
            {
                qpcPhaseFfV2HoldCyclesWindow++;
            }
            else
            {
                qpcPhaseFfV2FallbackCyclesWindow++;
            }


            qpcPhaseFfV2WindowSamples++;


            if (qpcPhaseFfV2WindowSamples >= 4000U)
            {
                int64_t phaseFfTargetDeltaWindowDeltaNs =
                    qpcPhaseFfV2TargetDeltaWindowEndNs -
                    qpcPhaseFfV2TargetDeltaWindowStartNs;


                InterlockedIncrement(
                    &g_qpcPhaseFfV2DiagSequence);


                g_qpcPhaseFfV2Initialized =
                    qpcPhaseFfV2Initialized
                    ? 1L
                    : 0L;

                g_qpcPhaseFfV2State =
                    qpcPhaseFfV2State;

                g_qpcPhaseFfV2CandidateGood =
                    phaseFfCandidateGood
                    ? 1L
                    : 0L;

                g_qpcPhaseFfV2WarmupGoodCount =
                    (LONG)
                    qpcPhaseFfV2WarmupGoodCount;

                g_qpcPhaseFfV2BadCount =
                    (LONG)
                    qpcPhaseFfV2BadCount;

                g_qpcPhaseFfV2RecoveryGoodCount =
                    (LONG)
                    qpcPhaseFfV2RecoveryGoodCount;

                g_qpcPhaseFfV2ObserverSequence =
                    phaseFfObserverSeqBefore;

                g_qpcPhaseFfV2RecommendedPpb =
                    (LONGLONG)
                    phaseFfRecommendedPpb;

                g_qpcPhaseFfV2DesiredPpb =
                    (LONGLONG)
                    qpcPhaseFfV2DesiredPpb;

                g_qpcPhaseFfV2AppliedPpb =
                    (LONGLONG)
                    qpcPhaseFfV2AppliedPpb;

                g_qpcPhaseFfV2SlewAppliedPpb =
                    (LONGLONG)
                    qpcPhaseFfV2LastAppliedStepPpb;

                g_qpcPhaseFfV2SlopeMadPpb =
                    (LONGLONG)
                    phaseFfSlopeMadPpb;

                g_qpcPhaseFfV2Points =
                    phaseFfPoints;

                g_qpcPhaseFfV2Pairs =
                    phaseFfPairs;

                g_qpcPhaseFfV2ObserverLocked =
                    phaseFfObserverLocked;

                g_qpcPhaseFfV2PointAccepted =
                    phaseFfPointAccepted;

                g_qpcPhaseFfV2TargetVsFixedNs =
                    (LONGLONG)
                    phaseFfTargetVsFixedNs;

                g_qpcPhaseFfV2WakeErrorNs =
                    (LONGLONG)
                    phaseFfWakeErrorNs;

                g_qpcPhaseFfV2TargetDeltaWindowStartNs =
                    (LONGLONG)
                    qpcPhaseFfV2TargetDeltaWindowStartNs;

                g_qpcPhaseFfV2TargetDeltaWindowEndNs =
                    (LONGLONG)
                    qpcPhaseFfV2TargetDeltaWindowEndNs;

                g_qpcPhaseFfV2TargetDeltaWindowDeltaNs =
                    (LONGLONG)
                    phaseFfTargetDeltaWindowDeltaNs;

                g_qpcPhaseFfV2TrackCyclesWindow =
                    (LONG)
                    qpcPhaseFfV2TrackCyclesWindow;

                g_qpcPhaseFfV2HoldCyclesWindow =
                    (LONG)
                    qpcPhaseFfV2HoldCyclesWindow;

                g_qpcPhaseFfV2FallbackCyclesWindow =
                    (LONG)
                    qpcPhaseFfV2FallbackCyclesWindow;

                g_qpcPhaseFfV2StateSwitchesWindow =
                    (LONG)
                    qpcPhaseFfV2StateSwitchesWindow;

                g_qpcPhaseFfV2TotalStateSwitches =
                    (LONG)
                    qpcPhaseFfV2TotalStateSwitches;

                g_qpcPhaseFfV2WarmupToTrackTotal =
                    (LONG)
                    qpcPhaseFfV2WarmupToTrackTotal;

                g_qpcPhaseFfV2TrackToHoldTotal =
                    (LONG)
                    qpcPhaseFfV2TrackToHoldTotal;

                g_qpcPhaseFfV2HoldToTrackTotal =
                    (LONG)
                    qpcPhaseFfV2HoldToTrackTotal;

                g_qpcPhaseFfV2HoldToFallbackTotal =
                    (LONG)
                    qpcPhaseFfV2HoldToFallbackTotal;

                g_qpcPhaseFfV2FallbackToTrackTotal =
                    (LONG)
                    qpcPhaseFfV2FallbackToTrackTotal;

                g_qpcPhaseFfV2Samples =
                    (LONG)
                    qpcPhaseFfV2WindowSamples;


                MemoryBarrier();


                InterlockedIncrement(
                    &g_qpcPhaseFfV2DiagSequence);


                qpcPhaseFfV2WindowSamples =
                    0;

                qpcPhaseFfV2TrackCyclesWindow =
                    0;

                qpcPhaseFfV2HoldCyclesWindow =
                    0;

                qpcPhaseFfV2FallbackCyclesWindow =
                    0;

                qpcPhaseFfV2StateSwitchesWindow =
                    0;
            }


            // =============================================================
            // QPC Coarse Re-Anchor Dry Run V1
            //
            // Existing raw error:
            //     errorNs = actual wake - DC-equivalent target
            //
            // The current calibrated baseline target is intentionally untouched.
            // This model only asks whether periodic 50 us coarse phase
            // advances could keep the virtual final margin bounded.
            // =============================================================

            LONG robustSequenceForCoarseDry =
                g_qpcDcRobustDiagSequence;

            bool robustReadyForCoarseDry =
                (
                    robustSequenceForCoarseDry != 0 &&
                    (robustSequenceForCoarseDry & 1) == 0 &&
                    g_qpcDcRobustLocked != 0 &&
                    g_qpcDcRobustBufferCount >= 9
                    );

            int64_t coarseDryRobustMedianPpb =
                (int64_t)g_qpcDcRobustMedianDriftPpb;

            int64_t coarseDryRobustMadPpb =
                (int64_t)g_qpcDcRobustMadPpb;


            if (!qpcCoarseReanchorInitialized &&
                robustReadyForCoarseDry)
            {
                // Virtual re-anchor:
                // virtualError = errorNs - offset = -100 us.
                qpcCoarseReanchorVirtualOffsetNs =
                    errorNs +
                    QPC_COARSE_REANCHOR_TARGET_MARGIN_NS;

                qpcCoarseReanchorWindowSamples = 0;
                qpcCoarseReanchorCorrectionEventsWindow = 0;
                qpcCoarseReanchorCorrectionTicksWindow = 0;
                qpcCoarseReanchorMaxTicksPerEventWindow = 0;
                qpcCoarseReanchorMultiTickEventsWindow = 0;
                qpcCoarseReanchorCyclesSinceLastEvent = 0;
                qpcCoarseReanchorLastEventSpacingCycles = 0;
                qpcCoarseReanchorMinEventSpacingCycles = 0;
                qpcCoarseReanchorMaxEventSpacingCycles = 0;
                qpcCoarseReanchorHasPreviousEvent = false;
                qpcCoarseReanchorOverflow = false;

                qpcCoarseReanchorInitialized = true;
            }


            if (qpcCoarseReanchorInitialized)
            {
                qpcCoarseReanchorCyclesSinceLastEvent++;


                int64_t virtualErrorNs =
                    errorNs -
                    qpcCoarseReanchorVirtualOffsetNs;

                uint32_t correctionTicksThisEvent =
                    0;


                while (
                    virtualErrorNs >
                    -QPC_COARSE_REANCHOR_TRIGGER_MARGIN_NS &&
                    correctionTicksThisEvent <
                    QPC_COARSE_REANCHOR_MAX_TICKS_PER_EVENT)
                {
                    qpcCoarseReanchorVirtualOffsetNs +=
                        QPC_COARSE_REANCHOR_TICK_NS;

                    virtualErrorNs -=
                        QPC_COARSE_REANCHOR_TICK_NS;

                    correctionTicksThisEvent++;
                }


                if (virtualErrorNs >
                    -QPC_COARSE_REANCHOR_TRIGGER_MARGIN_NS)
                {
                    qpcCoarseReanchorOverflow = true;
                }


                if (correctionTicksThisEvent > 0)
                {
                    qpcCoarseReanchorCorrectionEventsWindow++;
                    qpcCoarseReanchorCorrectionTicksWindow +=
                        correctionTicksThisEvent;

                    qpcCoarseReanchorTotalCorrectionEvents++;
                    qpcCoarseReanchorTotalCorrectionTicks +=
                        correctionTicksThisEvent;


                    if (correctionTicksThisEvent >
                        qpcCoarseReanchorMaxTicksPerEventWindow)
                    {
                        qpcCoarseReanchorMaxTicksPerEventWindow =
                            correctionTicksThisEvent;
                    }


                    if (correctionTicksThisEvent > 1)
                    {
                        qpcCoarseReanchorMultiTickEventsWindow++;
                    }


                    if (qpcCoarseReanchorHasPreviousEvent)
                    {
                        qpcCoarseReanchorLastEventSpacingCycles =
                            qpcCoarseReanchorCyclesSinceLastEvent;


                        if (qpcCoarseReanchorMinEventSpacingCycles == 0 ||
                            qpcCoarseReanchorLastEventSpacingCycles <
                            qpcCoarseReanchorMinEventSpacingCycles)
                        {
                            qpcCoarseReanchorMinEventSpacingCycles =
                                qpcCoarseReanchorLastEventSpacingCycles;
                        }


                        if (qpcCoarseReanchorLastEventSpacingCycles >
                            qpcCoarseReanchorMaxEventSpacingCycles)
                        {
                            qpcCoarseReanchorMaxEventSpacingCycles =
                                qpcCoarseReanchorLastEventSpacingCycles;
                        }
                    }


                    qpcCoarseReanchorCyclesSinceLastEvent = 0;
                    qpcCoarseReanchorHasPreviousEvent = true;
                }


                int64_t virtualMarginNs =
                    virtualErrorNs < 0
                    ? -virtualErrorNs
                    : 0;


                if (qpcCoarseReanchorWindowSamples == 0)
                {
                    qpcCoarseReanchorWindowStartErrorNs =
                        virtualErrorNs;

                    qpcCoarseReanchorWindowMinErrorNs =
                        virtualErrorNs;

                    qpcCoarseReanchorWindowMaxErrorNs =
                        virtualErrorNs;
                }

                qpcCoarseReanchorWindowEndErrorNs =
                    virtualErrorNs;

                if (virtualErrorNs <
                    qpcCoarseReanchorWindowMinErrorNs)
                {
                    qpcCoarseReanchorWindowMinErrorNs =
                        virtualErrorNs;
                }

                if (virtualErrorNs >
                    qpcCoarseReanchorWindowMaxErrorNs)
                {
                    qpcCoarseReanchorWindowMaxErrorNs =
                        virtualErrorNs;
                }

                qpcCoarseReanchorWindowSamples++;


                if (qpcCoarseReanchorWindowSamples >= 4000U)
                {
                    int64_t virtualWindowDeltaNs =
                        qpcCoarseReanchorWindowEndErrorNs -
                        qpcCoarseReanchorWindowStartErrorNs;


                    InterlockedIncrement(
                        &g_qpcCoarseReanchorDiagSequence);

                    g_qpcCoarseReanchorInitialized = 1L;
                    g_qpcCoarseReanchorRawErrorNs =
                        (LONGLONG)errorNs;
                    g_qpcCoarseReanchorVirtualErrorNs =
                        (LONGLONG)virtualErrorNs;
                    g_qpcCoarseReanchorVirtualMarginNs =
                        (LONGLONG)virtualMarginNs;
                    g_qpcCoarseReanchorVirtualOffsetNs =
                        (LONGLONG)qpcCoarseReanchorVirtualOffsetNs;

                    g_qpcCoarseReanchorWindowStartErrorNs =
                        (LONGLONG)qpcCoarseReanchorWindowStartErrorNs;
                    g_qpcCoarseReanchorWindowEndErrorNs =
                        (LONGLONG)qpcCoarseReanchorWindowEndErrorNs;
                    g_qpcCoarseReanchorWindowDeltaErrorNs =
                        (LONGLONG)virtualWindowDeltaNs;
                    g_qpcCoarseReanchorWindowMinErrorNs =
                        (LONGLONG)qpcCoarseReanchorWindowMinErrorNs;
                    g_qpcCoarseReanchorWindowMaxErrorNs =
                        (LONGLONG)qpcCoarseReanchorWindowMaxErrorNs;

                    g_qpcCoarseReanchorCorrectionEventsWindow =
                        (LONG)qpcCoarseReanchorCorrectionEventsWindow;
                    g_qpcCoarseReanchorCorrectionTicksWindow =
                        (LONG)qpcCoarseReanchorCorrectionTicksWindow;
                    g_qpcCoarseReanchorMaxTicksPerEventWindow =
                        (LONG)qpcCoarseReanchorMaxTicksPerEventWindow;
                    g_qpcCoarseReanchorMultiTickEventsWindow =
                        (LONG)qpcCoarseReanchorMultiTickEventsWindow;

                    g_qpcCoarseReanchorTotalCorrectionEvents =
                        (LONGLONG)qpcCoarseReanchorTotalCorrectionEvents;
                    g_qpcCoarseReanchorTotalCorrectionTicks =
                        (LONGLONG)qpcCoarseReanchorTotalCorrectionTicks;

                    g_qpcCoarseReanchorLastEventSpacingCycles =
                        (LONG)qpcCoarseReanchorLastEventSpacingCycles;
                    g_qpcCoarseReanchorMinEventSpacingCycles =
                        (LONG)qpcCoarseReanchorMinEventSpacingCycles;
                    g_qpcCoarseReanchorMaxEventSpacingCycles =
                        (LONG)qpcCoarseReanchorMaxEventSpacingCycles;

                    g_qpcCoarseReanchorRobustMedianDriftPpb =
                        (LONGLONG)coarseDryRobustMedianPpb;
                    g_qpcCoarseReanchorRobustMadPpb =
                        (LONGLONG)coarseDryRobustMadPpb;
                    g_qpcCoarseReanchorRobustLocked =
                        robustReadyForCoarseDry ? 1L : 0L;

                    g_qpcCoarseReanchorWindowSamples =
                        (LONG)qpcCoarseReanchorWindowSamples;
                    g_qpcCoarseReanchorOverflow =
                        qpcCoarseReanchorOverflow ? 1L : 0L;

                    MemoryBarrier();

                    InterlockedIncrement(
                        &g_qpcCoarseReanchorDiagSequence);


                    qpcCoarseReanchorWindowSamples = 0;
                    qpcCoarseReanchorWindowStartErrorNs = 0;
                    qpcCoarseReanchorWindowEndErrorNs = 0;
                    qpcCoarseReanchorWindowMinErrorNs = 0;
                    qpcCoarseReanchorWindowMaxErrorNs = 0;
                    qpcCoarseReanchorCorrectionEventsWindow = 0;
                    qpcCoarseReanchorCorrectionTicksWindow = 0;
                    qpcCoarseReanchorMaxTicksPerEventWindow = 0;
                    qpcCoarseReanchorMultiTickEventsWindow = 0;
                    qpcCoarseReanchorOverflow = false;
                }
            }


            int64_t remainingToTargetNs =
                0;

            int64_t lateByNs =
                0;

            LONG schedulerState =
                0;


            if (errorNs > 0)
            {
                schedulerState =
                    3; // LATE

                lateByNs =
                    errorNs;

                qpcSchedulerLateCount++;
            }
            else
            {
                remainingToTargetNs =
                    -errorNs;


                if (remainingToTargetNs <=
                    QPC_SCHEDULER_NEAR_NS)
                {
                    schedulerState =
                        2; // NEAR

                    qpcSchedulerNearCount++;
                }
                else
                {
                    schedulerState =
                        1; // EARLY

                    qpcSchedulerEarlyCount++;
                }
            }


            if (qpcSchedulerWindowSamples == 0)
            {
                qpcSchedulerWindowStartErrorNs =
                    errorNs;

                qpcSchedulerWindowMinErrorNs =
                    errorNs;

                qpcSchedulerWindowMaxErrorNs =
                    errorNs;
            }


            qpcSchedulerWindowEndErrorNs =
                errorNs;


            if (errorNs <
                qpcSchedulerWindowMinErrorNs)
            {
                qpcSchedulerWindowMinErrorNs =
                    errorNs;
            }


            if (errorNs >
                qpcSchedulerWindowMaxErrorNs)
            {
                qpcSchedulerWindowMaxErrorNs =
                    errorNs;
            }


            qpcSchedulerWindowSamples++;


            // -----------------------------------------------------
            // Publish about once per second.
            // -----------------------------------------------------

            if (qpcSchedulerWindowSamples >=
                4000U)
            {
                int64_t windowDeltaErrorNs =
                    qpcSchedulerWindowEndErrorNs -
                    qpcSchedulerWindowStartErrorNs;


                InterlockedIncrement(
                    &g_qpcSchedulerDiagSequence);


                g_qpcSchedulerTargetQpc =
                    (LONGLONG)
                    qpcSchedulerTargetQpc;

                g_qpcSchedulerActualQpc =
                    (LONGLONG)
                    actualWakeQpc;


                g_qpcSchedulerErrorCounts =
                    (LONGLONG)
                    errorCounts;

                g_qpcSchedulerErrorNs =
                    (LONGLONG)
                    errorNs;


                g_qpcSchedulerWindowStartErrorNs =
                    (LONGLONG)
                    qpcSchedulerWindowStartErrorNs;

                g_qpcSchedulerWindowEndErrorNs =
                    (LONGLONG)
                    qpcSchedulerWindowEndErrorNs;

                g_qpcSchedulerWindowDeltaErrorNs =
                    (LONGLONG)
                    windowDeltaErrorNs;

                g_qpcSchedulerWindowMinErrorNs =
                    (LONGLONG)
                    qpcSchedulerWindowMinErrorNs;

                g_qpcSchedulerWindowMaxErrorNs =
                    (LONGLONG)
                    qpcSchedulerWindowMaxErrorNs;


                g_qpcSchedulerPeriodWholeCounts =
                    (LONGLONG)
                    periodWholeCounts;

                g_qpcSchedulerPeriodFractionScaled =
                    (LONGLONG)
                    periodFractionScaled;


                g_qpcSchedulerAssumedDriftPpb =
                    (LONGLONG)
                    realFfV0AppliedPpb;


                g_qpcSchedulerWindowSamples =
                    (LONG)
                    qpcSchedulerWindowSamples;


                g_qpcSchedulerState =
                    schedulerState;

                g_qpcSchedulerGuardNs =
                    (LONGLONG)
                    QPC_SCHEDULER_GUARD_NS;

                g_qpcSchedulerRemainingToTargetNs =
                    (LONGLONG)
                    remainingToTargetNs;

                g_qpcSchedulerLateByNs =
                    (LONGLONG)
                    lateByNs;


                g_qpcSchedulerAnchorQpc =
                    (LONGLONG)
                    qpcSchedulerAnchorQpc;

                g_qpcSchedulerAnchorDcDiagSequence =
                    qpcSchedulerAnchorDcDiagSequence;

                g_qpcSchedulerStableWaitCycles =
                    (LONG)
                    qpcSchedulerStableWakeCycles;


                g_qpcSchedulerEarlyCount =
                    (LONG)
                    qpcSchedulerEarlyCount;

                g_qpcSchedulerNearCount =
                    (LONG)
                    qpcSchedulerNearCount;

                g_qpcSchedulerLateCount =
                    (LONG)
                    qpcSchedulerLateCount;


                g_qpcSchedulerValid =
                    1L;

                int64_t realVsFixedNs = 0;
                if (qpcFixedRefInitialized)
                {
                    int64_t c =
                        (int64_t)qpcSchedulerTargetQpc -
                        (int64_t)qpcFixedRefTargetQpc;
                    realVsFixedNs =
                        (c * 1000000000LL) / (int64_t)qpcFrequency;
                }

                InterlockedIncrement(&g_qpcRealFfV0Seq);
                g_qpcRealFfV0State = realFfV0State;
                g_qpcRealFfV0PhaseGood = realPhaseGood ? 1L : 0L;
                g_qpcRealFfV0ArmGood = (LONG)realFfV0ArmGood;
                g_qpcRealFfV0HoldMask = realFfV0TransientHoldReasonMask;
                g_qpcRealFfV0HistoryMask = realFfV0HistoryMask;
                g_qpcRealFfV0CleanCycles = (LONG)realFfV0CleanCycleStreak;
                g_qpcRealFfV0CleanCyclesRequired =
                    (LONG)realFfV0CleanCyclesRequired;
                g_qpcRealFfV0RecoveryProfile = realFfV0RecoveryProfile;
                g_qpcRealFfV0HoldRecoveryWindowsRequired =
                    (LONG)realFfV0HoldRecoveryWindowsRequired;
                g_qpcRealFfV0TripMask = realFfV0TripMask;
                g_qpcRealFfV0TripCount = (LONG)realFfV0TripCount;
                g_qpcRealFfV0RecommendedPpb = realPhaseRecommended;
                g_qpcRealFfV0DesiredPpb = (LONGLONG)realFfV0DesiredPpb;
                g_qpcRealFfV0AppliedPpb = (LONGLONG)realFfV0AppliedPpb;
                g_qpcRealFfV0LastStepPpb = (LONGLONG)realFfV0LastStepPpb;
                g_qpcRealFfV0TargetVsFixedNs = (LONGLONG)realVsFixedNs;
                g_qpcRealFfV0DcErrEstNs =
                    phasePActV0AcceptedPhaseMapValid
                    ? (LONGLONG)(phasePActV0AcceptedFixedUnwrappedErrorNs +
                        realVsFixedNs)
                    : 0;
                g_qpcRealFfV0PhaseSeq = realPhaseSeq1;
                g_qpcRealFfV0PhaseRejectMask = realFfV0PhaseRejectMask;
                g_qpcRealFfV0LastPhaseRejectMask = realFfV0LastPhaseRejectMask;
                g_qpcRealFfV0HoldGood = (LONG)realFfV0HoldGood;
                g_qpcRealFfV0HoldBad = (LONG)realFfV0HoldBad;
                g_qpcRealFfV0HoldEntries = (LONG)realFfV0HoldEntries;
                g_qpcRealFfV0ClampActive = realFfV0ClampActive ? 1L : 0L;
                MemoryBarrier();
                InterlockedIncrement(&g_qpcRealFfV0Seq);

                // ---------------------------------------------------------
                // DC-RX.3C coherent Phase-P publication freshness guard.
                //
                // A scheduler window is allowed to consume a phase map only
                // once. A coherent map older than 1.5 s is considered stale;
                // a discontinuous new map is quarantined until three mutually
                // consistent publications arrive. Existing Phase-P offset is
                // preserved during every quarantine/HOLD transition.
                // ---------------------------------------------------------
                LONG pPhaseMapSeq1 =
                    g_qpcLiveFfDcPhaseDiagSequence;
                LONG pPhaseMapInitialized = 0;
                LONG pPhaseMapSamples = 0;
                LONGLONG pPhaseMapFixedUnwrappedErrorNs = 0;
                LONGLONG pPhaseMapLastSampleQpc = 0;
                bool pPhaseMapSnapshotCoherent = false;

                if (pPhaseMapSeq1 != 0 &&
                    (pPhaseMapSeq1 & 1) == 0)
                {
                    MemoryBarrier();
                    pPhaseMapInitialized =
                        g_qpcLiveFfDcPhaseInitialized;
                    pPhaseMapSamples =
                        g_qpcLiveFfDcPhaseSamples;
                    pPhaseMapFixedUnwrappedErrorNs =
                        g_qpcLiveFfDcPhaseFixedUnwrappedErrorNs;
                    pPhaseMapLastSampleQpc =
                        g_qpcLiveFfDcPhaseLastSampleQpc;
                    MemoryBarrier();

                    const LONG pPhaseMapSeq2 =
                        g_qpcLiveFfDcPhaseDiagSequence;
                    pPhaseMapSnapshotCoherent =
                        pPhaseMapSeq1 == pPhaseMapSeq2 &&
                        (pPhaseMapSeq2 & 1) == 0;
                }

                const bool pPhaseMapNew =
                    pPhaseMapSnapshotCoherent &&
                    pPhaseMapSeq1 != phasePActV0LastSeenPhaseMapSeq;

                if (pPhaseMapSnapshotCoherent)
                {
                    phasePActV0LastSeenPhaseMapSeq = pPhaseMapSeq1;
                }

                uint64_t pPhaseMapAgeNs = 0;
                bool pPhaseMapAgeConvertible = false;

                if (pPhaseMapSnapshotCoherent &&
                    pPhaseMapLastSampleQpc > 0 &&
                    actualWakeQpc >= (uint64_t)pPhaseMapLastSampleQpc)
                {
                    pPhaseMapAgeConvertible =
                        QpcCountsToNsSafe(
                            actualWakeQpc -
                            (uint64_t)pPhaseMapLastSampleQpc,
                            &pPhaseMapAgeNs);
                }

                const bool pPhaseMapCurrentUsable =
                    pPhaseMapSnapshotCoherent &&
                    pPhaseMapInitialized != 0 &&
                    pPhaseMapSamples > 0 &&
                    pPhaseMapAgeConvertible &&
                    pPhaseMapAgeNs <= DC_RX3C_PHASE_MAP_MAX_AGE_NS;

                const bool pPhaseObservationFresh =
                    !realFfV0CycleQualityKnown ||
                    (realFfV0PreviousDcTransportValid &&
                        realFfV0PreviousDcSampleQualified);

                const bool pPhaseObservationEligible =
                    pPhaseMapNew &&
                    pPhaseMapCurrentUsable &&
                    qpcSchedulerInitialized &&
                    pPhaseObservationFresh &&
                    (realFfV0TripMask & REAL_FF_V0_HARD_TRIP_MASK) == 0;

                // DC-RX.3F PHASE_MAP_JUMP alters only this local consumer copy.
                // The producer snapshot remains untouched, so the next genuine
                // publication proves that quarantine/requalification can recover.
                if (dcRx3fInjectionRequestedThisCycle &&
                    dcRx3fScenario == DcRx3fFaultScenario::PhaseMapJump &&
                    pPhaseObservationEligible &&
                    !phasePRecoveryJumpGuardActive &&
                    dcRx3fConfiguredValueNs > 0)
                {
                    pPhaseMapFixedUnwrappedErrorNs +=
                        (LONGLONG)dcRx3fConfiguredValueNs;
                    dcRx3fInjectionAppliedThisCycle = true;
                }

                bool pPhaseMapAcceptedThisWindow = false;

                auto WrapPhaseValueNs =
                    [&](int64_t valueNs) -> int64_t
                {
                    int64_t wrappedNs =
                        valueNs % PHASE_P_ACT_CYCLE_NS;

                    if (wrappedNs > PHASE_P_ACT_CYCLE_NS / 2LL)
                        wrappedNs -= PHASE_P_ACT_CYCLE_NS;

                    if (wrappedNs < -PHASE_P_ACT_CYCLE_NS / 2LL)
                        wrappedNs += PHASE_P_ACT_CYCLE_NS;

                    return wrappedNs;
                };

                auto WrapPhaseJumpNs =
                    [&](int64_t currentNs, int64_t previousNs) -> int64_t
                {
                    return WrapPhaseValueNs(currentNs - previousNs);
                };

                if (pPhaseObservationEligible)
                {
                    const int64_t pCandidateBaseWrappedNs =
                        WrapPhaseValueNs(
                            (int64_t)pPhaseMapFixedUnwrappedErrorNs +
                            realVsFixedNs);

                    bool acceptCurrentMap = false;

                    if (!phasePActV0AcceptedPhaseMapValid &&
                        !phasePRecoveryJumpGuardActive)
                    {
                        // Initial bind is safe because no previous phase command
                        // exists to jump away from.
                        acceptCurrentMap = true;
                    }
                    else if (phasePRecoveryJumpGuardActive)
                    {
                        if (!phasePRecoveryJumpReferenceValid)
                        {
                            phasePRecoveryJumpReferenceWrappedNs =
                                pCandidateBaseWrappedNs;
                            phasePRecoveryJumpReferenceValid = true;
                            phasePRecoveryJumpGoodWindows = 1;
                        }
                        else
                        {
                            const int64_t jumpNs =
                                WrapPhaseJumpNs(
                                    pCandidateBaseWrappedNs,
                                    phasePRecoveryJumpReferenceWrappedNs);
                            const int64_t jumpAbsNs =
                                jumpNs >= 0 ? jumpNs : -jumpNs;

                            phasePRecoveryJumpLastNs = jumpNs;
                            if (jumpAbsNs > phasePRecoveryJumpMaximumAbsNs)
                            {
                                phasePRecoveryJumpMaximumAbsNs = jumpAbsNs;
                            }

                            phasePRecoveryJumpReferenceWrappedNs =
                                pCandidateBaseWrappedNs;

                            if (jumpAbsNs <= DC_RX3C_PHASE_JUMP_MAX_NS)
                            {
                                if (phasePRecoveryJumpGoodWindows <
                                    DC_RX3C_PHASE_JUMP_GOOD_WINDOWS)
                                {
                                    phasePRecoveryJumpGoodWindows++;
                                }

                                if (phasePRecoveryJumpGoodWindows >=
                                    DC_RX3C_PHASE_JUMP_GOOD_WINDOWS)
                                {
                                    phasePRecoveryJumpGuardActive = false;
                                    phasePRecoveryJumpReferenceValid = false;
                                    phasePRecoveryJumpGoodWindows =
                                        DC_RX3C_PHASE_JUMP_GOOD_WINDOWS;
                                    phasePRecoveryJumpPassTotal++;
                                    acceptCurrentMap = true;
                                }
                            }
                            else
                            {
                                // The new publication becomes the next
                                // candidate, but cannot control Phase-P yet.
                                phasePRecoveryJumpGoodWindows = 1;
                                phasePRecoveryJumpRejectTotal++;
                            }
                        }
                    }
                    else
                    {
                        const int64_t jumpNs =
                            WrapPhaseJumpNs(
                                pCandidateBaseWrappedNs,
                                phasePActV0LastObservedWrappedNs);
                        const int64_t jumpAbsNs =
                            jumpNs >= 0 ? jumpNs : -jumpNs;

                        phasePRecoveryJumpLastNs = jumpNs;
                        if (jumpAbsNs > phasePRecoveryJumpMaximumAbsNs)
                        {
                            phasePRecoveryJumpMaximumAbsNs = jumpAbsNs;
                        }

                        if (phasePActV0LastObservedWrappedValid &&
                            jumpAbsNs > DC_RX3C_PHASE_JUMP_MAX_NS)
                        {
                            ArmPhasePRecoveryJumpGuard(false);
                            phasePRecoveryJumpReferenceWrappedNs =
                                pCandidateBaseWrappedNs;
                            phasePRecoveryJumpReferenceValid = true;
                            phasePRecoveryJumpGoodWindows = 1;
                            phasePRecoveryJumpRejectTotal++;
                        }
                        else
                        {
                            acceptCurrentMap = true;
                        }
                    }

                    if (acceptCurrentMap)
                    {
                        phasePActV0AcceptedPhaseMapValid = true;
                        phasePActV0AcceptedFixedUnwrappedErrorNs =
                            (int64_t)pPhaseMapFixedUnwrappedErrorNs;
                        phasePActV0AcceptedPhaseMapQpc =
                            (uint64_t)pPhaseMapLastSampleQpc;
                        phasePActV0LastAcceptedPhaseMapSeq =
                            pPhaseMapSeq1;
                        phasePActV0LastObservedWrappedNs =
                            pCandidateBaseWrappedNs;
                        phasePActV0LastObservedWrappedValid = true;
                        pPhaseMapAcceptedThisWindow = true;
                    }
                }

                uint64_t pAcceptedPhaseMapAgeNs = 0;
                bool pAcceptedPhaseMapAgeGood = false;

                if (phasePActV0AcceptedPhaseMapValid &&
                    actualWakeQpc >= phasePActV0AcceptedPhaseMapQpc &&
                    QpcCountsToNsSafe(
                        actualWakeQpc - phasePActV0AcceptedPhaseMapQpc,
                        &pAcceptedPhaseMapAgeNs))
                {
                    pAcceptedPhaseMapAgeGood =
                        pAcceptedPhaseMapAgeNs <=
                        DC_RX3C_PHASE_MAP_MAX_AGE_NS;
                }

                if (!pAcceptedPhaseMapAgeGood)
                {
                    if (!phasePActV0PhaseMapStaleEpisodeActive &&
                        phasePActV0AcceptedPhaseMapValid)
                    {
                        phasePActV0PhaseMapStaleEpisodeActive = true;
                        if (phasePActV0PhaseMapStaleTotal !=
                            0xFFFFFFFFFFFFFFFFULL)
                        {
                            phasePActV0PhaseMapStaleTotal++;
                        }
                    }

                    if (phasePActV0State == 2)
                    {
                        ArmPhasePRecoveryJumpGuard(true);
                    }
                }
                else
                {
                    phasePActV0PhaseMapStaleEpisodeActive = false;
                }

                phasePActV0DiagPhaseMapSequence =
                    phasePActV0LastAcceptedPhaseMapSeq;
                phasePActV0DiagPhaseMapNew =
                    pPhaseMapAcceptedThisWindow;
                phasePActV0DiagPhaseMapAgeGood =
                    pAcceptedPhaseMapAgeGood;
                phasePActV0DiagPhaseMapAgeNs =
                    pAcceptedPhaseMapAgeNs;

                const int64_t pBaseErrNs =
                    phasePActV0AcceptedPhaseMapValid
                    ? phasePActV0AcceptedFixedUnwrappedErrorNs +
                    realVsFixedNs
                    : 0;

                const int64_t pActualErrNs =
                    pBaseErrNs + phasePActV0OffsetNs;

                const int64_t pWrappedErrNs =
                    WrapPhaseValueNs(pActualErrNs);

                const bool pGateGood =
                    realFfV0State == 2 &&
                    realPhaseGood &&
                    realFfV0TripMask == 0 &&
                    qpcSchedulerInitialized &&
                    phasePActV0AcceptedPhaseMapValid &&
                    pAcceptedPhaseMapAgeGood &&
                    !phasePRecoveryJumpGuardActive;

                // ---------------------------------------------------------
                // Phase-P 正式控制器
                //
                // BaseErr 是 FF timeline 對 DC Reference 的未包絡誤差；ActualErr 再加上
                // 已套用 offset。WrappedErr 映射到一個 250 us 週期的 ±125 us，
                // 避免跨週期時把等價相位誤認為巨幅誤差。
                // Gate 連續合格 3 窗後 ACTIVE；暫時不合格進 HOLD；Real FF Trip
                // 則 Phase-P 也 Trip。HOLD/Trip 不新增 step，保留最後 offset。
                // ---------------------------------------------------------
                if (phasePActV0State == 0 ||
                    phasePActV0State == 1)
                {
                    if (pGateGood)
                    {
                        phasePActV0State = 1;

                        if (pPhaseMapAcceptedThisWindow)
                        {
                            phasePActV0ArmGood++;
                        }

                        if (phasePActV0ArmGood >= PHASE_P_ACT_ARM_WINDOWS)
                            phasePActV0State = 2;
                    }
                    else
                    {
                        phasePActV0State = 0;
                        phasePActV0ArmGood = 0;
                    }
                }
                else if (phasePActV0State == 2 && !pGateGood)
                {
                    if (realFfV0State == 4 || realFfV0TripMask != 0)
                    {
                        phasePActV0State = 4;
                        phasePActV0TripCount++;
                    }
                    else
                    {
                        phasePActV0State = 3;
                        phasePActV0HoldGood = 0;
                        phasePActV0HoldEntries++;
                    }
                }
                else if (phasePActV0State == 3)
                {
                    if (realFfV0State == 4 || realFfV0TripMask != 0)
                    {
                        phasePActV0State = 4;
                        phasePActV0TripCount++;
                    }
                    else if (pGateGood)
                    {
                        if (pPhaseMapAcceptedThisWindow)
                        {
                            phasePActV0HoldGood++;
                        }

                        if (phasePActV0HoldGood >=
                            PHASE_P_ACT_HOLD_RECOVERY_WINDOWS)
                        {
                            phasePActV0State = 2;
                            phasePActV0HoldGood = 0;
                        }
                    }
                    else
                    {
                        phasePActV0HoldGood = 0;
                    }
                }
                else if (phasePActV0State == 4)
                {
                    // Parent hard trips never leave Real FF state 4.
                    // This release is reachable only after an observer-only
                    // soft trip passed the conservative re-arm gate.
                    //
                    // Preserve phasePActV0OffsetNs to avoid a phase jump.
                    if (realFfV0State != 4 &&
                        realFfV0TripMask == 0)
                    {
                        phasePActV0State = 0;
                        phasePActV0ArmGood = 0;
                        phasePActV0HoldGood = 0;
                        phasePActV0LastStepNs = 0;
                    }
                }

                int64_t pRawCorrectionNs = 0;
                int64_t pCommandNs = 0;
                int64_t pStepNs = 0;
                bool pCommandSat = false;
                bool pOffsetSat = false;

                int64_t pAbsErrNs =
                    pWrappedErrNs >= 0
                    ? pWrappedErrNs : -pWrappedErrNs;

                const bool pCorrectionSampleFresh =
                    pPhaseMapAcceptedThisWindow &&
                    (!realFfV0CycleQualityKnown ||
                        realFfV0PreviousDcSampleQualified) &&
                    !phasePRecoveryJumpGuardActive;

                if (phasePActV0State == 2 &&
                    pGateGood &&
                    pCorrectionSampleFresh &&
                    pAbsErrNs > PHASE_P_ACT_DEADBAND_NS)
                {
                    // 負回授：誤差為正就減少 offset，誤差為負就增加 offset。
                    // 依序套用 P command 限制、單步 slew 限制與總 offset 限制。
                    pRawCorrectionNs =
                        -(pWrappedErrNs / PHASE_P_ACT_DIVISOR);

                    pCommandNs = pRawCorrectionNs;

                    if (pCommandNs > PHASE_P_ACT_MAX_COMMAND_NS)
                    {
                        pCommandNs = PHASE_P_ACT_MAX_COMMAND_NS;
                        pCommandSat = true;
                    }

                    if (pCommandNs < -PHASE_P_ACT_MAX_COMMAND_NS)
                    {
                        pCommandNs = -PHASE_P_ACT_MAX_COMMAND_NS;
                        pCommandSat = true;
                    }

                    pStepNs = pCommandNs;

                    if (pStepNs > PHASE_P_ACT_MAX_STEP_NS)
                        pStepNs = PHASE_P_ACT_MAX_STEP_NS;

                    if (pStepNs < -PHASE_P_ACT_MAX_STEP_NS)
                        pStepNs = -PHASE_P_ACT_MAX_STEP_NS;

                    int64_t oldOffsetNs = phasePActV0OffsetNs;
                    int64_t newOffsetNs = oldOffsetNs + pStepNs;

                    if (newOffsetNs > PHASE_P_ACT_MAX_OFFSET_NS)
                    {
                        newOffsetNs = PHASE_P_ACT_MAX_OFFSET_NS;
                        pOffsetSat = true;
                    }

                    if (newOffsetNs < -PHASE_P_ACT_MAX_OFFSET_NS)
                    {
                        newOffsetNs = -PHASE_P_ACT_MAX_OFFSET_NS;
                        pOffsetSat = true;
                    }

                    phasePActV0OffsetNs = newOffsetNs;
                    pStepNs = newOffsetNs - oldOffsetNs;
                }

                phasePActV0LastStepNs = pStepNs;

                int64_t pPredictedErrNs =
                    pWrappedErrNs + pStepNs;

                int64_t pAbsPredictedNs =
                    pPredictedErrNs >= 0
                    ? pPredictedErrNs : -pPredictedErrNs;

                bool pImprove =
                    phasePActV0State == 2 &&
                    pGateGood &&
                    pCorrectionSampleFresh &&
                    (
                        pAbsErrNs <= PHASE_P_ACT_DEADBAND_NS ||
                        pAbsPredictedNs < pAbsErrNs
                        );

                InterlockedIncrement(&g_qpcPhasePActV0Seq);
                g_qpcPhasePActV0State = phasePActV0State;
                g_qpcPhasePActV0GateGood = pGateGood ? 1L : 0L;
                g_qpcPhasePActV0ArmGood = (LONG)phasePActV0ArmGood;
                g_qpcPhasePActV0BaseErrNs = pBaseErrNs;
                g_qpcPhasePActV0ActualErrNs = pActualErrNs;
                g_qpcPhasePActV0WrappedErrNs = pWrappedErrNs;
                g_qpcPhasePActV0RawCorrectionNs = pRawCorrectionNs;
                g_qpcPhasePActV0CommandNs = pCommandNs;
                g_qpcPhasePActV0StepNs = pStepNs;
                g_qpcPhasePActV0OffsetNs = phasePActV0OffsetNs;
                g_qpcPhasePActV0PredictedErrNs = pPredictedErrNs;
                g_qpcPhasePActV0CommandSat = pCommandSat ? 1L : 0L;
                g_qpcPhasePActV0OffsetSat = pOffsetSat ? 1L : 0L;
                g_qpcPhasePActV0Improve = pImprove ? 1L : 0L;
                g_qpcPhasePActV0HoldGood = (LONG)phasePActV0HoldGood;
                g_qpcPhasePActV0HoldEntries = (LONG)phasePActV0HoldEntries;
                g_qpcPhasePActV0TripCount = (LONG)phasePActV0TripCount;
                g_qpcPhasePActV0RealFfState = realFfV0State;
                MemoryBarrier();
                InterlockedIncrement(&g_qpcPhasePActV0Seq);


                MemoryBarrier();


                InterlockedIncrement(
                    &g_qpcSchedulerDiagSequence);


                // -------------------------------------------------
                // Reset only diagnostic-window values.
                //
                // Target / anchor / fraction continue.
                // -------------------------------------------------

                qpcSchedulerWindowSamples =
                    0;

                qpcSchedulerWindowStartErrorNs =
                    0;

                qpcSchedulerWindowEndErrorNs =
                    0;

                qpcSchedulerWindowMinErrorNs =
                    0;

                qpcSchedulerWindowMaxErrorNs =
                    0;


                qpcSchedulerEarlyCount =
                    0;

                qpcSchedulerNearCount =
                    0;

                qpcSchedulerLateCount =
                    0;
            }
        }
    }

    // =============================================================
    // PDO One-Shot Scheduler V1B - Coarse Wake 正式控制
    //
    // 真正 PDO timer 行為：
    //
    // Bootstrap:
    //     expired one-shot
    //       -> RtSetTimerRelative(250 us, NULL)
    //
    // After QPC/DC scheduler stability gate:
    //     final target = existing DC-equivalent QPC target
    //     coarse target = final target - 100 us
    //
    // Every callback predicts NEXT final target and immediately
    // re-arms this expired timer toward NEXT coarse target.
    //
    // Fine Wait 維持 OFF，因此 EtherCAT 通訊在 coarse callback 醒來後立即開始；
    // Phase-P offset 會加在 final target，再由 final target 減 100 us 得 coarse target。
    //
    // Safety:
    // - NO HAL change.
    // - NO QPC spin.
    // - re-arm happens before EtherCAT communication.
    // - if active target is unusable, fall back to a 250 us
    //   bootstrap one-shot and force scheduler re-anchor.
    // =============================================================

    {
        const bool PDO_ONESHOT_SCHEDULER_CONTROL =
            true;

        const bool PDO_FINE_WAIT_CONTROL =
            false;


        const uint64_t PDO_ONESHOT_COARSE_GUARD_NS =
            100000ULL;              // final target 前 100 us 喚醒；正式版先不要調。

        const uint64_t PDO_BOOTSTRAP_DELAY_NS =
            250000ULL;              // fallback 250 us，必須等於 PDO cycle。

        const uint64_t PDO_MIN_REARM_LEAD_NS =
            50000ULL;               // re-arm 至少保留 50 us lead。

        const uint64_t PDO_RUNTIME_RECOVERY_TARGET_LEAD_NS =
            200000ULL;              // Recovery 後把 target 拉回至少 200 us 之後。

        const uint32_t PDO_ONESHOT_DIAG_WINDOW =
            4000U;                  // 4 kHz 下約一秒，只影響統計發布頻率。


        static bool coarseActive =
            false;


        static uint32_t windowSamples =
            0;

        static uint32_t windowRearmOk =
            0;

        static uint32_t windowRearmFail =
            0;

        static uint32_t windowBootstrap =
            0;

        static uint32_t windowActive =
            0;


        // =============================================================
        // Bootstrap Fallback Diagnostic V1 state
        // =============================================================

        static uint32_t bootstrapCurrentConsecutive =
            0;

        static uint32_t bootstrapMaxConsecutiveWindow =
            0;

        static uint64_t bootstrapTotal =
            0;

        static uint32_t bootstrapRecoveryWindow =
            0;

        static uint32_t bootstrapReasonNotReadyWindow =
            0;

        static uint32_t bootstrapReasonFinalUnderGuardWindow =
            0;

        static uint32_t bootstrapReasonQpcBeforeArmFailWindow =
            0;

        static uint32_t bootstrapReasonLeadTooShortWindow =
            0;

        static uint32_t bootstrapReasonOtherWindow =
            0;

        static uint32_t runtimeRecoveryWindowEvents = 0;
        static uint32_t runtimeRecoveryWindowSkippedCycles = 0;
        static uint32_t runtimeRecoveryMaxSkipCycles = 0;
        static uint64_t runtimeRecoveryTotalEvents = 0;
        static uint64_t runtimeRecoveryTotalSkippedCycles = 0;
        static int64_t runtimeRecoveryLastLeadBeforeNs = 0;
        static int64_t runtimeRecoveryLastLeadAfterNs = 0;

        static bool runtimeRecoverySelfTestDone = false;


        static int64_t bootstrapPredictedLeadSumNs =
            0;

        static int64_t bootstrapPredictedLeadMinNs =
            0;

        static int64_t bootstrapPredictedLeadMaxNs =
            0;

        static uint32_t bootstrapPredictedLeadValidWindow =
            0;


        static int64_t coarseErrorSumNs =
            0;

        static int64_t coarseErrorMinNs =
            0;

        static int64_t coarseErrorMaxNs =
            0;


        static int64_t finalMarginSumNs =
            0;

        static int64_t finalMarginMinNs =
            0;

        static int64_t finalMarginMaxNs =
            0;


        static uint64_t rearmCostSumNs =
            0;

        static uint64_t rearmCostMinNs =
            0;

        static uint64_t rearmCostMaxNs =
            0;

        static uint32_t rearmCostSamples =
            0;


        LONG infraState =
            0;                      // WAIT_TARGET

        int64_t toCoarseNs =
            0;

        int64_t toFinalNs =
            0;

        int64_t finalLateByNs =
            0;

        int64_t coarseErrorNs =
            0;

        int64_t finalMarginNs =
            0;


        uint64_t finalTargetQpc =
            0;

        uint64_t coarseTargetQpc =
            0;

        uint64_t actualWakeQpc =
            0;


        bool activeMeasurementValid =
            false;

        bool rearmOk =
            false;

        bool usedBootstrap =
            false;

        bool runtimeRecoveryThisCycle =
            false;

        DWORD rearmError =
            ERROR_SUCCESS;


        // ---------------------------------------------------------
        // 以 Real FF 實際 applied ppb 建立下一個 DC-equivalent QPC period。
        // 這是正式控制路徑，不是 shadow；ACTIVE_ASSUMED_DRIFT_PPB 改變後會改變
        // qpcSchedulerTargetQpc 每個週期的增量。
        // ---------------------------------------------------------

        const int64_t ACTIVE_ASSUMED_DRIFT_PPB =
            realFfV0AppliedPpb;

        const uint64_t ACTIVE_SCALE =
            1000000000ULL;


        uint64_t activePeriodWholeCounts =
            0;

        uint64_t activePeriodFractionScaled =
            0;


        if (qpcWakeValid &&
            qpcFrequency > 0)
        {
            uint64_t nominalPeriodCounts =
                qpcFrequency /
                4000ULL;


            int64_t activePeriodScaledCounts =
                (int64_t)
                nominalPeriodCounts *
                (
                    1000000000LL +
                    ACTIVE_ASSUMED_DRIFT_PPB
                    );


            if (activePeriodScaledCounts > 0)
            {
                activePeriodWholeCounts =
                    (uint64_t)
                    (
                        activePeriodScaledCounts /
                        1000000000LL
                        );


                activePeriodFractionScaled =
                    (uint64_t)
                    (
                        activePeriodScaledCounts %
                        1000000000LL
                        );
            }


            actualWakeQpc =
                (uint64_t)
                qpcWake.QuadPart;
        }

        int64_t phasePActOffsetCounts = 0;

        if (qpcFrequency > 0)
        {
            phasePActOffsetCounts =
                (phasePActV0OffsetNs * (int64_t)qpcFrequency) /
                1000000000LL;
        }


        // ---------------------------------------------------------
        // If active, this callback should be around:
        //
        //     current final target - 100 us
        //
        // qpcSchedulerTargetQpc has already been advanced by the
        // existing scheduler block above for this callback.
        // ---------------------------------------------------------

        if (coarseActive &&
            qpcWakeValid &&
            qpcFrequency > 0 &&
            qpcSchedulerInitialized &&
            qpcSchedulerTargetQpc > 0)
        {
            // 正常 active callback：計算目前 coarse 誤差與距 final target 的 margin。
            // actualWake 已晚於 final target 時不可直接 re-arm 過去時間，後方會以
            // Runtime Recovery 跳過必要週期，或退回 250 us bootstrap。
            uint64_t guardCounts =
                (
                    qpcFrequency *
                    PDO_ONESHOT_COARSE_GUARD_NS
                    )
                /
                1000000000ULL;


            int64_t actuatedFinalSigned =
                (int64_t)qpcSchedulerTargetQpc +
                phasePActOffsetCounts;

            if (guardCounts > 0 &&
                actuatedFinalSigned > (int64_t)guardCounts)
            {
                finalTargetQpc =
                    (uint64_t)actuatedFinalSigned;

                coarseTargetQpc =
                    finalTargetQpc -
                    guardCounts;


                int64_t coarseErrorCounts =
                    (int64_t)
                    actualWakeQpc -
                    (int64_t)
                    coarseTargetQpc;


                coarseErrorNs =
                    (
                        coarseErrorCounts *
                        1000000000LL
                        )
                    /
                    (int64_t)
                    qpcFrequency;


                if (actualWakeQpc <
                    finalTargetQpc)
                {
                    uint64_t marginCounts =
                        finalTargetQpc -
                        actualWakeQpc;


                    finalMarginNs =
                        (int64_t)
                        (
                            (
                                marginCounts *
                                1000000000ULL
                                )
                            /
                            qpcFrequency
                            );


                    toFinalNs =
                        finalMarginNs;
                }
                else
                {
                    uint64_t lateCounts =
                        actualWakeQpc -
                        finalTargetQpc;


                    finalLateByNs =
                        (int64_t)
                        (
                            (
                                lateCounts *
                                1000000000ULL
                                )
                            /
                            qpcFrequency
                            );
                }


                if (actualWakeQpc <
                    coarseTargetQpc)
                {
                    infraState =
                        1;          // READY_BEFORE_COARSE


                    uint64_t deltaCounts =
                        coarseTargetQpc -
                        actualWakeQpc;


                    toCoarseNs =
                        (int64_t)
                        (
                            (
                                deltaCounts *
                                1000000000ULL
                                )
                            /
                            qpcFrequency
                            );
                }
                else if (actualWakeQpc <
                    finalTargetQpc)
                {
                    infraState =
                        4;          // COARSE_ACTIVE
                }
                else
                {
                    infraState =
                        3;          // FINAL_TARGET_LATE
                }


                activeMeasurementValid =
                    true;


                if (windowActive == 0)
                {
                    coarseErrorMinNs =
                        coarseErrorNs;

                    coarseErrorMaxNs =
                        coarseErrorNs;

                    finalMarginMinNs =
                        finalMarginNs;

                    finalMarginMaxNs =
                        finalMarginNs;
                }
                else
                {
                    if (coarseErrorNs <
                        coarseErrorMinNs)
                    {
                        coarseErrorMinNs =
                            coarseErrorNs;
                    }


                    if (coarseErrorNs >
                        coarseErrorMaxNs)
                    {
                        coarseErrorMaxNs =
                            coarseErrorNs;
                    }


                    if (finalMarginNs <
                        finalMarginMinNs)
                    {
                        finalMarginMinNs =
                            finalMarginNs;
                    }


                    if (finalMarginNs >
                        finalMarginMaxNs)
                    {
                        finalMarginMaxNs =
                            finalMarginNs;
                    }
                }


                coarseErrorSumNs +=
                    coarseErrorNs;

                finalMarginSumNs +=
                    finalMarginNs;

                windowActive++;
            }
        }


        // ---------------------------------------------------------
        // Decide NEXT one-shot.
        //
        // Before scheduler lock:
        //     bootstrap 250 us relative one-shot.
        //
        // On transition / active:
        //     predict next final target from current QPC scheduler
        //     target + one DC-equivalent cycle.
        // ---------------------------------------------------------

        uint64_t requestedRelative100ns =
            0;


        // =============================================================
        // Bootstrap Fallback Diagnostic V1
        //
        // bootstrapReason:
        //   0 = active target produced successfully
        //   1 = scheduler prerequisites not ready
        //   2 = predicted final target <= guard
        //   3 = QPC read before arm failed
        //   4 = predicted lead <= minimum required lead
        //   5 = unexpected / other fallback
        //
        // The control decisions below remain identical to the
        // pre-diagnostic implementation.
        // =============================================================

        LONG bootstrapReason =
            0;

        bool predictedLeadValid =
            false;

        int64_t predictedLeadNs =
            0;


        const bool schedulerReadyForActive =
            (
                qpcWakeValid &&
                qpcFrequency > 0 &&
                qpcSchedulerInitialized &&
                qpcSchedulerTargetQpc > 0 &&
                activePeriodWholeCounts > 0
                );


        if (schedulerReadyForActive && !runtimeRecoverySelfTestDone)
        {
            const int64_t testBeforeNs[4] =
            { -300000LL, -600000LL, 40000LL, 100000LL };

            int64_t testAfterNs[4] = {};
            uint32_t testSkip[4] = {};
            uint32_t testFail = 0;
            bool shadowPreserve = true;

            uint64_t testGuard =
                (qpcFrequency * PDO_ONESHOT_COARSE_GUARD_NS) /
                1000000000ULL;

            uint64_t testMinLead =
                (qpcFrequency * PDO_MIN_REARM_LEAD_NS) /
                1000000000ULL;

            uint64_t testTargetLead =
                (qpcFrequency * PDO_RUNTIME_RECOVERY_TARGET_LEAD_NS) /
                1000000000ULL;

            const uint64_t testNow = 1000000000000000ULL;

            for (int i = 0; i < 4; i++)
            {
                uint64_t leadCounts =
                    ((uint64_t)(testBeforeNs[i] >= 0
                        ? testBeforeNs[i] : -testBeforeNs[i]) *
                        qpcFrequency) / 1000000000ULL;

                uint64_t desiredCoarse =
                    testBeforeNs[i] >= 0
                    ? testNow + leadCounts
                    : testNow - leadCounts;

                uint64_t frac0 = qpcSchedulerFractionRemainder;
                uint64_t predFrac0 =
                    frac0 + activePeriodFractionScaled;

                uint64_t carry0 =
                    predFrac0 >= ACTIVE_SCALE ? 1ULL : 0ULL;

                uint64_t fakeTarget =
                    desiredCoarse + testGuard -
                    activePeriodWholeCounts - carry0;

                uint64_t fakeTrusted = fakeTarget + 12345ULL;
                uint64_t fakePhase = fakeTarget - 54321ULL;

                uint64_t predFinal =
                    fakeTarget + activePeriodWholeCounts + carry0;

                uint64_t predCoarse =
                    predFinal - testGuard;

                bool stale =
                    predCoarse <= testNow + testMinLead;

                if (stale)
                {
                    uint64_t required =
                        testNow + testTargetLead;

                    uint64_t deficit =
                        required > predCoarse
                        ? required - predCoarse : 0ULL;

                    uint64_t skip =
                        deficit == 0
                        ? 1ULL
                        : (deficit + activePeriodWholeCounts - 1ULL) /
                        activePeriodWholeCounts;

                    uint64_t oldTarget = fakeTarget;
                    uint64_t frac =
                        frac0 + skip * activePeriodFractionScaled;

                    fakeTarget +=
                        skip * activePeriodWholeCounts +
                        frac / ACTIVE_SCALE;

                    frac0 = frac % ACTIVE_SCALE;

                    uint64_t shift = fakeTarget - oldTarget;
                    fakeTrusted += shift;
                    fakePhase += shift;

                    uint64_t predFrac =
                        frac0 + activePeriodFractionScaled;

                    predFinal =
                        fakeTarget + activePeriodWholeCounts;

                    if (predFrac >= ACTIVE_SCALE)
                        predFinal++;

                    predCoarse =
                        predFinal - testGuard;

                    testSkip[i] = (uint32_t)skip;

                    if ((fakeTrusted - fakeTarget) != 12345ULL ||
                        (fakeTarget - fakePhase) != 54321ULL)
                        shadowPreserve = false;
                }

                testAfterNs[i] =
                    predCoarse >= testNow
                    ? (int64_t)(((predCoarse - testNow) *
                        1000000000ULL) / qpcFrequency)
                    : -(int64_t)(((testNow - predCoarse) *
                        1000000000ULL) / qpcFrequency);

                if (i < 3)
                {
                    if (!stale || testSkip[i] == 0 ||
                        testAfterNs[i] < (int64_t)PDO_RUNTIME_RECOVERY_TARGET_LEAD_NS)
                        testFail++;
                }
                else
                {
                    if (stale || testSkip[i] != 0 ||
                        testAfterNs[i] < (int64_t)PDO_MIN_REARM_LEAD_NS)
                        testFail++;
                }
            }

            if (!shadowPreserve)
                testFail++;

            InterlockedIncrement(&g_pdoRecoverySelfTestSeq);
            g_pdoRecoverySelfTestPass = testFail == 0 ? 1L : 0L;
            g_pdoRecoverySelfTestCases = 4;
            g_pdoRecoverySelfTestFail = (LONG)testFail;
            g_pdoRecoverySelfTestBefore0 = testBeforeNs[0];
            g_pdoRecoverySelfTestAfter0 = testAfterNs[0];
            g_pdoRecoverySelfTestSkip0 = (LONG)testSkip[0];
            g_pdoRecoverySelfTestBefore1 = testBeforeNs[1];
            g_pdoRecoverySelfTestAfter1 = testAfterNs[1];
            g_pdoRecoverySelfTestSkip1 = (LONG)testSkip[1];
            g_pdoRecoverySelfTestBefore2 = testBeforeNs[2];
            g_pdoRecoverySelfTestAfter2 = testAfterNs[2];
            g_pdoRecoverySelfTestSkip2 = (LONG)testSkip[2];
            g_pdoRecoverySelfTestBefore3 = testBeforeNs[3];
            g_pdoRecoverySelfTestAfter3 = testAfterNs[3];
            g_pdoRecoverySelfTestSkip3 = (LONG)testSkip[3];
            g_pdoRecoverySelfTestShadowPreserve =
                shadowPreserve ? 1L : 0L;
            MemoryBarrier();
            InterlockedIncrement(&g_pdoRecoverySelfTestSeq);

            runtimeRecoverySelfTestDone = true;
        }


        if (!schedulerReadyForActive)
        {
            bootstrapReason =
                1;
        }
        else
        {
            uint64_t predictedBaseFinalQpc =
                qpcSchedulerTargetQpc +
                activePeriodWholeCounts;


            uint64_t predictedFraction =
                qpcSchedulerFractionRemainder +
                activePeriodFractionScaled;


            if (predictedFraction >=
                ACTIVE_SCALE)
            {
                predictedBaseFinalQpc++;
            }

            int64_t predictedActuatedSigned =
                (int64_t)predictedBaseFinalQpc +
                phasePActOffsetCounts;

            uint64_t predictedNextFinalQpc =
                predictedActuatedSigned > 0
                ? (uint64_t)predictedActuatedSigned
                : 0ULL;


            uint64_t guardCounts =
                (
                    qpcFrequency *
                    PDO_ONESHOT_COARSE_GUARD_NS
                    )
                /
                1000000000ULL;


            if (predictedNextFinalQpc <=
                guardCounts)
            {
                bootstrapReason =
                    2;
            }
            else
            {
                uint64_t predictedNextCoarseQpc =
                    predictedNextFinalQpc -
                    guardCounts;


                LARGE_INTEGER qpcBeforeArmLi =
                {};


                if (!RtQueryPerformanceCounter(
                    &qpcBeforeArmLi))
                {
                    bootstrapReason =
                        3;
                }
                else
                {
                    uint64_t qpcBeforeArm =
                        (uint64_t)
                        qpcBeforeArmLi.QuadPart;


                    uint64_t minLeadCounts =
                        (
                            qpcFrequency *
                            PDO_MIN_REARM_LEAD_NS
                            )
                        /
                        1000000000ULL;


                    // -------------------------------------------------
                    // Diagnostic lead measurement.
                    //
                    // Signed so a target already behind qpcBeforeArm
                    // is visible as a negative lead.
                    // -------------------------------------------------

                    if (predictedNextCoarseQpc >=
                        qpcBeforeArm)
                    {
                        uint64_t leadCounts =
                            predictedNextCoarseQpc -
                            qpcBeforeArm;

                        predictedLeadNs =
                            (int64_t)
                            (
                                (
                                    leadCounts *
                                    1000000000ULL
                                    )
                                /
                                qpcFrequency
                                );
                    }
                    else
                    {
                        uint64_t lateCounts =
                            qpcBeforeArm -
                            predictedNextCoarseQpc;

                        predictedLeadNs =
                            -
                            (int64_t)
                            (
                                (
                                    lateCounts *
                                    1000000000ULL
                                    )
                                /
                                qpcFrequency
                                );
                    }


                    predictedLeadValid =
                        true;


                    bootstrapPredictedLeadSumNs +=
                        predictedLeadNs;

                    bootstrapPredictedLeadValidWindow++;


                    if (bootstrapPredictedLeadValidWindow ==
                        1)
                    {
                        bootstrapPredictedLeadMinNs =
                            predictedLeadNs;

                        bootstrapPredictedLeadMaxNs =
                            predictedLeadNs;
                    }
                    else
                    {
                        if (predictedLeadNs <
                            bootstrapPredictedLeadMinNs)
                        {
                            bootstrapPredictedLeadMinNs =
                                predictedLeadNs;
                        }


                        if (predictedLeadNs >
                            bootstrapPredictedLeadMaxNs)
                        {
                            bootstrapPredictedLeadMaxNs =
                                predictedLeadNs;
                        }
                    }


                    // During first transition the predicted next
                    // coarse wake should be ~200 us ahead.
                    // During steady state it should be ~250 us ahead.
                    if (predictedNextCoarseQpc <=
                        qpcBeforeArm + minLeadCounts)
                    {
                        uint64_t forensicOriginalNextCoarseQpc =
                            predictedNextCoarseQpc;

                        uint64_t targetLeadCounts =
                            (qpcFrequency *
                                PDO_RUNTIME_RECOVERY_TARGET_LEAD_NS) /
                            1000000000ULL;

                        uint64_t requiredCoarse =
                            qpcBeforeArm + targetLeadCounts;

                        uint64_t deficit =
                            requiredCoarse > predictedNextCoarseQpc
                            ? requiredCoarse - predictedNextCoarseQpc
                            : 0ULL;

                        uint64_t skipCycles =
                            deficit == 0
                            ? 1ULL
                            : (deficit + activePeriodWholeCounts - 1ULL) /
                            activePeriodWholeCounts;

                        uint64_t oldTarget = qpcSchedulerTargetQpc;
                        uint64_t frac =
                            qpcSchedulerFractionRemainder +
                            skipCycles * activePeriodFractionScaled;

                        qpcSchedulerTargetQpc +=
                            skipCycles * activePeriodWholeCounts +
                            frac / ACTIVE_SCALE;

                        qpcSchedulerFractionRemainder =
                            frac % ACTIVE_SCALE;

                        uint64_t shift =
                            qpcSchedulerTargetQpc - oldTarget;

                        if (qpcLiveFfInitialized)
                            qpcLiveFfTargetQpc += shift;

                        if (qpcPhaseFfV2Initialized)
                            qpcPhaseFfV2TargetQpc += shift;

                        if (qpcFixedRefInitialized)
                            qpcFixedRefTargetQpc += shift;

                        predictedBaseFinalQpc =
                            qpcSchedulerTargetQpc +
                            activePeriodWholeCounts;

                        predictedFraction =
                            qpcSchedulerFractionRemainder +
                            activePeriodFractionScaled;

                        if (predictedFraction >= ACTIVE_SCALE)
                            predictedBaseFinalQpc++;

                        predictedActuatedSigned =
                            (int64_t)predictedBaseFinalQpc +
                            phasePActOffsetCounts;

                        predictedNextFinalQpc =
                            predictedActuatedSigned > 0
                            ? (uint64_t)predictedActuatedSigned
                            : 0ULL;

                        predictedNextCoarseQpc =
                            predictedNextFinalQpc - guardCounts;

                        runtimeRecoveryThisCycle = true;
                        runtimeRecoveryWindowEvents++;
                        runtimeRecoveryTotalEvents++;
                        runtimeRecoveryWindowSkippedCycles +=
                            (uint32_t)skipCycles;
                        runtimeRecoveryTotalSkippedCycles += skipCycles;

                        if (skipCycles > runtimeRecoveryMaxSkipCycles)
                            runtimeRecoveryMaxSkipCycles =
                            (uint32_t)skipCycles;

                        runtimeRecoveryLastLeadBeforeNs =
                            predictedLeadNs;

                        runtimeRecoveryLastLeadAfterNs =
                            predictedNextCoarseQpc >= qpcBeforeArm
                            ? (int64_t)(((predictedNextCoarseQpc - qpcBeforeArm) *
                                1000000000ULL) / qpcFrequency)
                            : -(int64_t)(((qpcBeforeArm - predictedNextCoarseQpc) *
                                1000000000ULL) / qpcFrequency);

                        if (g_pdoRecoveryForensicCaptured == 0 &&
                            (realFfV0State == 2 ||
                                realFfV0State == 3) &&
                            qpcWakeValid &&
                            qpcFrequency > 0)
                        {
                            uint64_t wakeQpc = (uint64_t)qpcWake.QuadPart;

                            int64_t leadAtWakeNs =
                                forensicOriginalNextCoarseQpc >= wakeQpc
                                ? (int64_t)(((forensicOriginalNextCoarseQpc - wakeQpc) *
                                    1000000000ULL) / qpcFrequency)
                                : -(int64_t)(((wakeQpc - forensicOriginalNextCoarseQpc) *
                                    1000000000ULL) / qpcFrequency);

                            int64_t wakeToArmNs =
                                qpcBeforeArm >= wakeQpc
                                ? (int64_t)(((qpcBeforeArm - wakeQpc) *
                                    1000000000ULL) / qpcFrequency)
                                : 0;

                            InterlockedIncrement(&g_pdoRecoveryForensicSeq);
                            g_pdoRecoveryForensicWakeIntervalNs =
                                (LONGLONG)recoveryForensicWakeIntervalNs;
                            g_pdoRecoveryForensicLeadAtWakeNs =
                                (LONGLONG)leadAtWakeNs;
                            g_pdoRecoveryForensicWakeToArmNs =
                                (LONGLONG)wakeToArmNs;
                            g_pdoRecoveryForensicLeadAtArmNs =
                                (LONGLONG)predictedLeadNs;
                            g_pdoRecoveryForensicLeadAfterNs =
                                (LONGLONG)runtimeRecoveryLastLeadAfterNs;
                            g_pdoRecoveryForensicSkipCycles =
                                (LONG)skipCycles;
                            g_pdoRecoveryForensicCaptured = 1;
                            MemoryBarrier();
                            InterlockedIncrement(&g_pdoRecoveryForensicSeq);
                        }
                    }

                    if (predictedNextCoarseQpc >
                        qpcBeforeArm + minLeadCounts)
                    {
                        uint64_t remainingCounts =
                            predictedNextCoarseQpc -
                            qpcBeforeArm;

                        requestedRelative100ns =
                            (remainingCounts * 10000000ULL +
                                qpcFrequency - 1ULL) /
                            qpcFrequency;

                        if (requestedRelative100ns == 0)
                            requestedRelative100ns = 1;

                        coarseActive = true;
                    }
                    else
                    {
                        bootstrapReason = 4;
                    }
                }
            }
        }


        // ---------------------------------------------------------
        // If active scheduling could not produce a safe next
        // coarse target, use the SAME 250 us bootstrap one-shot.
        //
        // Diagnostic counters are updated only; scheduler state is
        // intentionally NOT invalidated in this revision.
        // ---------------------------------------------------------

        if (requestedRelative100ns == 0)
        {
            if (bootstrapReason == 0)
            {
                bootstrapReason =
                    5;
            }


            switch (bootstrapReason)
            {
            case 1:
                bootstrapReasonNotReadyWindow++;
                break;

            case 2:
                bootstrapReasonFinalUnderGuardWindow++;
                break;

            case 3:
                bootstrapReasonQpcBeforeArmFailWindow++;
                break;

            case 4:
                bootstrapReasonLeadTooShortWindow++;
                break;

            default:
                bootstrapReasonOtherWindow++;
                break;
            }


            bootstrapCurrentConsecutive++;
            bootstrapTotal++;


            if (bootstrapCurrentConsecutive >
                bootstrapMaxConsecutiveWindow)
            {
                bootstrapMaxConsecutiveWindow =
                    bootstrapCurrentConsecutive;
            }


            requestedRelative100ns =
                (
                    PDO_BOOTSTRAP_DELAY_NS +
                    99ULL
                    )
                /
                100ULL;


            usedBootstrap =
                true;

            coarseActive =
                false;

            windowBootstrap++;
        }
        else
        {
            if (bootstrapCurrentConsecutive >
                0)
            {
                bootstrapRecoveryWindow++;
                bootstrapCurrentConsecutive =
                    0;
            }
        }


        LARGE_INTEGER nextExpiration = {};

        nextExpiration.QuadPart =
            (LONGLONG)
            requestedRelative100ns;


        LARGE_INTEGER rearmQpcBefore = {};
        LARGE_INTEGER rearmQpcAfter = {};

        bool rearmQpcBeforeValid =
            false;

        bool rearmQpcAfterValid =
            false;


        if (qpcFrequency > 0)
        {
            if (RtQueryPerformanceCounter(
                &rearmQpcBefore))
            {
                rearmQpcBeforeValid =
                    true;
            }
        }


        if (g_pdoOneShotTimerHandle != NULL)
        {
            rearmOk =
                RtSetTimerRelative(
                    g_pdoOneShotTimerHandle,
                    &nextExpiration,
                    NULL);


            if (!rearmOk)
            {
                rearmError =
                    GetLastError();
            }
        }
        else
        {
            rearmError =
                ERROR_INVALID_HANDLE;
        }


        if (rearmQpcBeforeValid)
        {
            if (RtQueryPerformanceCounter(
                &rearmQpcAfter))
            {
                if (rearmQpcAfter.QuadPart >=
                    rearmQpcBefore.QuadPart)
                {
                    rearmQpcAfterValid =
                        true;
                }
            }
        }


        if (rearmQpcBeforeValid &&
            rearmQpcAfterValid &&
            qpcFrequency > 0)
        {
            uint64_t costCounts =
                (uint64_t)
                (
                    rearmQpcAfter.QuadPart -
                    rearmQpcBefore.QuadPart
                    );


            uint64_t rearmCostNs =
                (
                    costCounts *
                    1000000000ULL
                    )
                /
                qpcFrequency;


            if (rearmCostSamples == 0)
            {
                rearmCostMinNs =
                    rearmCostNs;

                rearmCostMaxNs =
                    rearmCostNs;
            }
            else
            {
                if (rearmCostNs <
                    rearmCostMinNs)
                {
                    rearmCostMinNs =
                        rearmCostNs;
                }


                if (rearmCostNs >
                    rearmCostMaxNs)
                {
                    rearmCostMaxNs =
                        rearmCostNs;
                }
            }


            rearmCostSumNs +=
                rearmCostNs;

            rearmCostSamples++;
        }


        if (rearmOk)
        {
            windowRearmOk++;
        }
        else
        {
            windowRearmFail++;


            // -----------------------------------------------------
            // Emergency recovery:
            // retry once with a simple 250 us one-shot.
            // -----------------------------------------------------

            LARGE_INTEGER emergencyExpiration = {};

            emergencyExpiration.QuadPart =
                2500;              // 250 us


            BOOL emergencyOk =
                FALSE;


            if (g_pdoOneShotTimerHandle != NULL)
            {
                emergencyOk =
                    RtSetTimerRelative(
                        g_pdoOneShotTimerHandle,
                        &emergencyExpiration,
                        NULL);
            }


            coarseActive =
                false;


            qpcSchedulerInitialized =
                false;

            qpcSchedulerStableWakeCycles =
                0;


            if (!emergencyOk)
            {
                // No print at Priority 64.
                // Diagnostics will expose the failure if Main can
                // still run; cyclic PDO stops if both attempts fail.
            }
        }


        // DC-RX.3F SCHEDULER_RECOVERY is a logical recovery-path test.
        // The real timer was already re-armed successfully; only the controller
        // result is overridden so no artificial timer miss is created.
        if (dcRx3fInjectionRequestedThisCycle &&
            dcRx3fScenario == DcRx3fFaultScenario::SchedulerRecovery &&
            rearmOk &&
            !usedBootstrap &&
            !runtimeRecoveryThisCycle)
        {
            runtimeRecoveryThisCycle = true;
            dcRx3fInjectionAppliedThisCycle = true;
        }

        currentSchedulerEvaluated = true;
        currentSchedulerRearmOk = rearmOk;
        currentSchedulerUsedBootstrap = usedBootstrap;
        currentSchedulerRuntimeRecovery = runtimeRecoveryThisCycle;

        LONG hardTripMask = 0;
        if (usedBootstrap) hardTripMask |= 0x04;
        if (!rearmOk) hardTripMask |= 0x08;

        const bool realFfCanLatchHardTrip =
            realFfV0State == 2 ||
            realFfV0State == 3 ||
            (realFfV0State == 4 &&
                (realFfV0TripMask & REAL_FF_V0_HARD_TRIP_MASK) == 0);

        if (hardTripMask != 0 && realFfCanLatchHardTrip)
        {
            realFfV0State = 4;
            realFfV0AppliedPpb = QPC_SCHEDULER_ASSUMED_DRIFT_PPB;
            realFfV0DesiredPpb = QPC_SCHEDULER_ASSUMED_DRIFT_PPB;
            realFfV0LastStepPpb = 0;
            realFfV0TripMask |= hardTripMask;
            realFfV0HistoryMask |= hardTripMask;
            realFfV0TransientHoldReasonMask = 0;
            realFfV0TripCount++;
            realFfV0SoftRearmGood = 0;
        }
        else if (hardTripMask != 0 &&
            (realFfV0State == 0 || realFfV0State == 1))
        {
            // Startup/ARM has not yet earned ACTIVE authority. Return to WAIT,
            // but do not create a latched trip merely for bootstrap qualification.
            realFfV0State = 0;
            realFfV0ArmGood = 0;
            realFfV0DesiredPpb = realFfV0AppliedPpb;
            realFfV0LastStepPpb = 0;
        }
        else if (hardTripMask == 0)
        {
            LONG transientReasonMask = 0;

            if (runtimeRecoveryThisCycle)
            {
                transientReasonMask |= REAL_FF_V0_TRANSIENT_SCHEDULER_REASON;
            }

            if (realFfV0CycleQualityKnown &&
                !realFfV0PreviousCycleClean)
            {
                transientReasonMask |=
                    realFfV0PreviousCycleReasonMask != 0
                    ? realFfV0PreviousCycleReasonMask
                    : REAL_FF_V0_TRANSIENT_SCHEDULER_REASON;
            }

            if (transientReasonMask != 0)
            {
                // A scheduler target skip changes the timeline immediately, so
                // always reopen observers. A previous PDO failure was already
                // reopened at the end of the callback that detected it.
                EnterRealFfV0TransientHold(
                    transientReasonMask,
                    runtimeRecoveryThisCycle);
            }
        }

        bool realFfCycleSafe =
            rearmOk &&
            !usedBootstrap &&
            !runtimeRecoveryThisCycle &&
            (!realFfV0CycleQualityKnown ||
                realFfV0PreviousCycleClean);

        realFfV0OneShotHealthy = realFfCycleSafe;


        windowSamples++;


        // ---------------------------------------------------------
        // Publish one snapshot per ~4000 PDO callbacks.
        // ---------------------------------------------------------

        if (windowSamples >=
            PDO_ONESHOT_DIAG_WINDOW)
        {
            int64_t coarseErrorAvgNs =
                0;

            int64_t finalMarginAvgNs =
                0;

            uint64_t rearmCostAvgNs =
                0;


            if (windowActive > 0)
            {
                coarseErrorAvgNs =
                    coarseErrorSumNs /
                    (int64_t)
                    windowActive;


                finalMarginAvgNs =
                    finalMarginSumNs /
                    (int64_t)
                    windowActive;
            }


            if (rearmCostSamples > 0)
            {
                rearmCostAvgNs =
                    rearmCostSumNs /
                    rearmCostSamples;
            }


            // =============================================================
            // Publish Bootstrap Fallback Diagnostic V1 snapshot.
            // =============================================================

            int64_t bootstrapPredictedLeadAvgNs =
                0;


            if (bootstrapPredictedLeadValidWindow >
                0)
            {
                bootstrapPredictedLeadAvgNs =
                    bootstrapPredictedLeadSumNs /
                    (int64_t)
                    bootstrapPredictedLeadValidWindow;
            }


            InterlockedIncrement(
                &g_pdoBootstrapDiagSequence);


            g_pdoBootstrapWindowBootstrap =
                (LONG)
                windowBootstrap;

            g_pdoBootstrapCurrentConsecutive =
                (LONG)
                bootstrapCurrentConsecutive;

            g_pdoBootstrapMaxConsecutive =
                (LONG)
                bootstrapMaxConsecutiveWindow;

            g_pdoBootstrapTotal =
                (LONGLONG)
                bootstrapTotal;

            g_pdoBootstrapRecoveryAfterBootstrap =
                (LONG)
                bootstrapRecoveryWindow;


            g_pdoBootstrapReasonNotReady =
                (LONG)
                bootstrapReasonNotReadyWindow;

            g_pdoBootstrapReasonFinalUnderGuard =
                (LONG)
                bootstrapReasonFinalUnderGuardWindow;

            g_pdoBootstrapReasonQpcBeforeArmFail =
                (LONG)
                bootstrapReasonQpcBeforeArmFailWindow;

            g_pdoBootstrapReasonLeadTooShort =
                (LONG)
                bootstrapReasonLeadTooShortWindow;

            g_pdoBootstrapReasonOther =
                (LONG)
                bootstrapReasonOtherWindow;


            g_pdoBootstrapSchedulerInitialized =
                qpcSchedulerInitialized
                ? 1L
                : 0L;


            g_pdoBootstrapPredictedLeadValid =
                (LONG)
                bootstrapPredictedLeadValidWindow;

            g_pdoBootstrapPredictedLeadAvgNs =
                (LONGLONG)
                bootstrapPredictedLeadAvgNs;

            g_pdoBootstrapPredictedLeadMinNs =
                (LONGLONG)
                (
                    bootstrapPredictedLeadValidWindow > 0
                    ? bootstrapPredictedLeadMinNs
                    : 0
                    );

            g_pdoBootstrapPredictedLeadMaxNs =
                (LONGLONG)
                (
                    bootstrapPredictedLeadValidWindow > 0
                    ? bootstrapPredictedLeadMaxNs
                    : 0
                    );

            g_pdoBootstrapSchedulerErrorNs =
                g_qpcSchedulerErrorNs;

            g_pdoBootstrapSchedulerLateByNs =
                g_qpcSchedulerLateByNs;

            g_pdoRuntimeRecoveryWindowEvents =
                (LONG)runtimeRecoveryWindowEvents;
            g_pdoRuntimeRecoveryWindowSkippedCycles =
                (LONG)runtimeRecoveryWindowSkippedCycles;
            g_pdoRuntimeRecoveryMaxSkipCycles =
                (LONG)runtimeRecoveryMaxSkipCycles;
            g_pdoRuntimeRecoveryTotalEvents =
                (LONGLONG)runtimeRecoveryTotalEvents;
            g_pdoRuntimeRecoveryTotalSkippedCycles =
                (LONGLONG)runtimeRecoveryTotalSkippedCycles;
            g_pdoRuntimeRecoveryLastLeadBeforeNs =
                (LONGLONG)runtimeRecoveryLastLeadBeforeNs;
            g_pdoRuntimeRecoveryLastLeadAfterNs =
                (LONGLONG)runtimeRecoveryLastLeadAfterNs;


            MemoryBarrier();


            InterlockedIncrement(
                &g_pdoBootstrapDiagSequence);


            InterlockedIncrement(
                &g_pdoOneShotInfraSequence);


            g_pdoOneShotInfraState =
                infraState;

            g_pdoOneShotInfraValid =
                1L;


            g_pdoOneShotInfraControlEnabled =
                PDO_ONESHOT_SCHEDULER_CONTROL
                ? 1L
                : 0L;

            g_pdoOneShotInfraFineWaitEnabled =
                PDO_FINE_WAIT_CONTROL
                ? 1L
                : 0L;


            g_pdoOneShotInfraWarmupRequired =
                1L;

            g_pdoOneShotInfraWarmupComplete =
                (
                    g_pdoOneShotWarmupCallbackDone != 0
                    )
                ? 1L
                : 0L;


            g_pdoOneShotInfraQpcFrequency =
                (LONGLONG)
                qpcFrequency;

            g_pdoOneShotInfraFinalTargetQpc =
                (LONGLONG)
                finalTargetQpc;

            g_pdoOneShotInfraCoarseTargetQpc =
                (LONGLONG)
                coarseTargetQpc;

            g_pdoOneShotInfraActualWakeQpc =
                (LONGLONG)
                actualWakeQpc;


            g_pdoOneShotInfraGuardNs =
                (LONGLONG)
                PDO_ONESHOT_COARSE_GUARD_NS;

            g_pdoOneShotInfraToCoarseNs =
                (LONGLONG)
                toCoarseNs;

            g_pdoOneShotInfraToFinalNs =
                (LONGLONG)
                toFinalNs;

            g_pdoOneShotInfraFinalLateByNs =
                (LONGLONG)
                finalLateByNs;


            g_pdoOneShotInfraDcDiagSequence =
                g_qpcDcDiagSequence;

            g_pdoOneShotInfraWindowSamples =
                (LONG)
                windowSamples;


            g_pdoOneShotInfraReadyCount =
                activeMeasurementValid
                ? 0L
                : (LONG)
                windowBootstrap;

            g_pdoOneShotInfraCoarseWindowCount =
                (LONG)
                windowActive;

            g_pdoOneShotInfraFinalLateCount =
                (LONG)
                (
                    finalLateByNs > 0
                    ? 1
                    : 0
                    );


            g_pdoOneShotInfraRearmOkCount =
                (LONG)
                windowRearmOk;

            g_pdoOneShotInfraRearmFailCount =
                (LONG)
                windowRearmFail;

            g_pdoOneShotInfraBootstrapCount =
                (LONG)
                windowBootstrap;

            g_pdoOneShotInfraActiveCount =
                (LONG)
                windowActive;


            g_pdoOneShotInfraCoarseErrorAvgNs =
                (LONGLONG)
                coarseErrorAvgNs;

            g_pdoOneShotInfraCoarseErrorMinNs =
                (LONGLONG)
                coarseErrorMinNs;

            g_pdoOneShotInfraCoarseErrorMaxNs =
                (LONGLONG)
                coarseErrorMaxNs;


            g_pdoOneShotInfraFinalMarginAvgNs =
                (LONGLONG)
                finalMarginAvgNs;

            g_pdoOneShotInfraFinalMarginMinNs =
                (LONGLONG)
                finalMarginMinNs;

            g_pdoOneShotInfraFinalMarginMaxNs =
                (LONGLONG)
                finalMarginMaxNs;


            g_pdoOneShotInfraRearmCostAvgNs =
                (LONGLONG)
                rearmCostAvgNs;

            g_pdoOneShotInfraRearmCostMinNs =
                (LONGLONG)
                rearmCostMinNs;

            g_pdoOneShotInfraRearmCostMaxNs =
                (LONGLONG)
                rearmCostMaxNs;


            MemoryBarrier();


            InterlockedIncrement(
                &g_pdoOneShotInfraSequence);


            windowSamples =
                0;

            windowRearmOk =
                0;

            windowRearmFail =
                0;

            windowBootstrap =
                0;

            windowActive =
                0;


            bootstrapMaxConsecutiveWindow =
                0;

            bootstrapRecoveryWindow =
                0;

            bootstrapReasonNotReadyWindow =
                0;

            bootstrapReasonFinalUnderGuardWindow =
                0;

            bootstrapReasonQpcBeforeArmFailWindow =
                0;

            bootstrapReasonLeadTooShortWindow =
                0;

            bootstrapReasonOtherWindow =
                0;

            runtimeRecoveryWindowEvents = 0;
            runtimeRecoveryWindowSkippedCycles = 0;
            runtimeRecoveryMaxSkipCycles = 0;

            bootstrapPredictedLeadSumNs =
                0;

            bootstrapPredictedLeadMinNs =
                0;

            bootstrapPredictedLeadMaxNs =
                0;

            bootstrapPredictedLeadValidWindow =
                0;


            coarseErrorSumNs =
                0;

            coarseErrorMinNs =
                0;

            coarseErrorMaxNs =
                0;


            finalMarginSumNs =
                0;

            finalMarginMinNs =
                0;

            finalMarginMaxNs =
                0;


            rearmCostSumNs =
                0;

            rearmCostMinNs =
                0;

            rearmCostMaxNs =
                0;

            rearmCostSamples =
                0;
        }
    }


    // =========================================================
    // RTX64 HAL Frequency Burst Actuator V2（保留程式，正式版目前停用）
    //
    // 不再：
    //
    //     每 37~38 PDO Cycle
    //     3000 -> 2999 -> 3000
    //
    // 因為上一版每秒約呼叫：
    //
    //     RtSetHalTimerPeriodCounts()
    //
    // 兩百多次，實測造成 CLOCK_2 Drift 異常。
    //
    //
    // 現在改成：
    //
    // 4000 PDO Cycle ≈ 1 second
    //
    // Example:
    //
    // FrequencyCommand = +8740 ppb
    //
    // Base = 3000
    // Alt  = 2999
    //
    // Superframe:
    //
    //     約 105 Cycle -> 2999
    //     約3895 Cycle -> 3000
    //
    // 每秒正常只 Set 約兩次：
    //
    //     3000 -> 2999
    //     2999 -> 3000
    //
    // =========================================================


    static ULONG lastAppliedHalCounts =
        0;


    // ---------------------------------------------------------
    // 4000 PDO Cycle Superframe
    // ---------------------------------------------------------

    static uint32_t burstCycleIndex =
        0;


    static uint32_t burstAltCycles =
        0;


    // ---------------------------------------------------------
    // Fractional remainder
    //
    // 例如：
    //
    // 104.88 Alt Cycle
    //
    // 自然產生：
    //
    // 104
    // 105
    // 105
    // ...
    // ---------------------------------------------------------

    static uint64_t burstFractionRemainder =
        0;


    // ---------------------------------------------------------
    // Diagnostic
    // ---------------------------------------------------------

    static uint64_t burstBaseCycleCount =
        0;


    static uint64_t burstAlternateCycleCount =
        0;


    static uint64_t halSetSuccessCount =
        0;


    static uint64_t halSetFailureCount =
        0;


    static bool halActuatorFault =
        false;


    // =========================================================
    // HAL Burst Actuator Enable Gate
    //
    // 固定為 false：目前 HAL 25 us 與 NAL priority 由 RTX64 設定管理，
    // Runtime 不呼叫 RtSetHalTimerPeriodCounts 改頻。未經獨立測試不可改成 true。
    // =========================================================
    const bool enableRuntimeHalActuator =
        false;
    if (enableRuntimeHalActuator &&
        pMaster->m_dcHalDitherEnabled &&
        !halActuatorFault &&
        pMaster->m_dcHalDitherStep > 0 &&
        pMaster->m_dcHalDitherThreshold > 0 &&
        pMaster->m_dcHalBaseCounts > 0 &&
        pMaster->m_dcHalAlternateCounts > 0)
    {
        // =====================================================
        // First Activation
        // =====================================================

        if (lastAppliedHalCounts == 0)
        {
            ULONG currentCounts =
                0;


            ULONG baseCounts =
                0;


            if (RtGetHalTimerPeriodCounts(
                &currentCounts,
                &baseCounts))
            {
                lastAppliedHalCounts =
                    currentCounts;


                burstCycleIndex =
                    0;


                burstFractionRemainder =
                    0;


                burstBaseCycleCount =
                    0;


                burstAlternateCycleCount =
                    0;


                halSetSuccessCount =
                    0;


                halSetFailureCount =
                    0;


                PublishP64DeferredDiagnostic(
                    EtherCatP64DeferredDiagKind::HalBurstStart,
                    static_cast<uint64_t>(pMaster->tickCount_PDO),
                    static_cast<int64_t>(currentCounts),
                    static_cast<int64_t>(baseCounts),
                    static_cast<int64_t>(
                        pMaster->m_dcHalFrequencyCommandPpb),
                    0LL,
                    0LL,
                    0LL,
                    0LL,
                    0LL);
            }
            else
            {
                const DWORD errorCode =
                    GetLastError();

                PublishP64DeferredDiagnostic(
                    EtherCatP64DeferredDiagKind::HalBurstGetCountsFailed,
                    static_cast<uint64_t>(pMaster->tickCount_PDO),
                    static_cast<int64_t>(errorCode),
                    0LL,
                    0LL,
                    0LL,
                    0LL,
                    0LL,
                    0LL,
                    0LL);


                halActuatorFault =
                    true;
            }
        }


        // =====================================================
        // Actuator Running
        // =====================================================

        if (!halActuatorFault)
        {
            // =================================================
            // Start of new 4000-Cycle Superframe
            // =================================================

            if (burstCycleIndex == 0)
            {
                // ---------------------------------------------
                // Example:
                //
                // Step:
                //
                //     26,220,000
                //
                // × 4000 Cycle
                //
                // = 104,880,000,000
                //
                // / 1,000,000,000
                //
                // = 104.88 Alt Cycle
                // ---------------------------------------------

                uint64_t superframeDemand =
                    burstFractionRemainder +
                    (
                        pMaster->
                        m_dcHalDitherStep *
                        4000ULL
                        );


                burstAltCycles =
                    (uint32_t)(
                        superframeDemand /
                        pMaster->
                        m_dcHalDitherThreshold);


                burstFractionRemainder =
                    superframeDemand %
                    pMaster->
                    m_dcHalDitherThreshold;


                // =============================================
                // Safety
                // =============================================

                if (burstAltCycles >
                    4000U)
                {
                    PublishP64DeferredDiagnostic(
                        EtherCatP64DeferredDiagKind::HalBurstAltCyclesInvalid,
                        static_cast<uint64_t>(pMaster->tickCount_PDO),
                        static_cast<int64_t>(burstAltCycles),
                        0LL,
                        0LL,
                        0LL,
                        0LL,
                        0LL,
                        0LL,
                        0LL);


                    halActuatorFault =
                        true;
                }
                else
                {
                    PublishP64DeferredDiagnostic(
                        EtherCatP64DeferredDiagKind::HalBurstPlan,
                        static_cast<uint64_t>(pMaster->tickCount_PDO),
                        static_cast<int64_t>(burstAltCycles),
                        static_cast<int64_t>(4000U - burstAltCycles),
                        static_cast<int64_t>(burstFractionRemainder),
                        static_cast<int64_t>(pMaster->m_dcHalDitherStep),
                        0LL,
                        0LL,
                        0LL,
                        0LL);
                }
            }


            // =================================================
            // Current Superframe Mode
            //
            // ALT first:
            //
            // 2999 2999 ... 2999
            //
            // then BASE:
            //
            // 3000 3000 ... 3000
            // =================================================

            if (!halActuatorFault)
            {
                bool useAlternate =
                    (
                        burstCycleIndex <
                        burstAltCycles
                        );


                ULONG desiredCounts =
                    useAlternate
                    ?
                    (ULONG)
                    pMaster->
                    m_dcHalAlternateCounts
                    :
                    (ULONG)
                    pMaster->
                    m_dcHalBaseCounts;


                if (useAlternate)
                {
                    burstAlternateCycleCount++;
                }
                else
                {
                    burstBaseCycleCount++;
                }


                // =============================================
                // Only Set when Count changes
                //
                // 正常一秒大約只有兩次：
                //
                // 3000 -> 2999
                // 2999 -> 3000
                // =============================================

                if (desiredCounts !=
                    lastAppliedHalCounts)
                {
                    bool setOk =
                        RtSetHalTimerPeriodCounts(
                            desiredCounts);


                    if (setOk)
                    {
                        lastAppliedHalCounts =
                            desiredCounts;


                        halSetSuccessCount++;
                    }
                    else
                    {
                        halSetFailureCount++;


                        DWORD errorCode =
                            GetLastError();


                        PublishP64DeferredDiagnostic(
                            EtherCatP64DeferredDiagKind::HalBurstSetFailed,
                            static_cast<uint64_t>(pMaster->tickCount_PDO),
                            static_cast<int64_t>(desiredCounts),
                            static_cast<int64_t>(errorCode),
                            0LL,
                            0LL,
                            0LL,
                            0LL,
                            0LL,
                            0LL);


                        // =====================================
                        // Safety Restore Base
                        // =====================================

                        bool restoreOk =
                            RtSetHalTimerPeriodCounts(
                                (ULONG)
                                pMaster->
                                m_dcHalBaseCounts);


                        if (restoreOk)
                        {
                            lastAppliedHalCounts =
                                (ULONG)
                                pMaster->
                                m_dcHalBaseCounts;


                            PublishP64DeferredDiagnostic(
                                EtherCatP64DeferredDiagKind::HalBurstBaseRestored,
                                static_cast<uint64_t>(pMaster->tickCount_PDO),
                                static_cast<int64_t>(
                                    pMaster->m_dcHalBaseCounts),
                                0LL,
                                0LL,
                                0LL,
                                0LL,
                                0LL,
                                0LL,
                                0LL);
                        }
                        else
                        {
                            const DWORD restoreErrorCode =
                                GetLastError();

                            PublishP64DeferredDiagnostic(
                                EtherCatP64DeferredDiagKind::HalBurstBaseRestoreFailed,
                                static_cast<uint64_t>(pMaster->tickCount_PDO),
                                static_cast<int64_t>(restoreErrorCode),
                                0LL,
                                0LL,
                                0LL,
                                0LL,
                                0LL,
                                0LL,
                                0LL);
                        }


                        halActuatorFault =
                            true;
                    }
                }


                // =============================================
                // Advance Superframe
                // =============================================

                burstCycleIndex++;


                // =============================================
                // End of 4000 PDO Cycle Superframe
                // =============================================

                if (burstCycleIndex >=
                    4000U)
                {
                    uint64_t totalCycles =
                        burstBaseCycleCount +
                        burstAlternateCycleCount;


                    uint64_t altDutyPercentX1000 =
                        0;


                    if (totalCycles > 0)
                    {
                        altDutyPercentX1000 =
                            (
                                burstAlternateCycleCount *
                                100000ULL
                                )
                            /
                            totalCycles;
                    }


                    PublishP64DeferredDiagnostic(
                        EtherCatP64DeferredDiagKind::HalBurstSummary,
                        static_cast<uint64_t>(pMaster->tickCount_PDO),
                        static_cast<int64_t>(burstBaseCycleCount),
                        static_cast<int64_t>(burstAlternateCycleCount),
                        static_cast<int64_t>(altDutyPercentX1000),
                        static_cast<int64_t>(lastAppliedHalCounts),
                        static_cast<int64_t>(halSetSuccessCount),
                        static_cast<int64_t>(halSetFailureCount),
                        static_cast<int64_t>(burstFractionRemainder),
                        0LL);


                    // =========================================
                    // Reset Superframe Statistics
                    //
                    // Fraction remainder 不 Reset。
                    // =========================================

                    burstCycleIndex =
                        0;


                    burstBaseCycleCount =
                        0;


                    burstAlternateCycleCount =
                        0;


                    halSetSuccessCount =
                        0;


                    halSetFailureCount =
                        0;
                }
            }
        }
    }
    else
    {
        // =====================================================
        // Safety:
        //
        // 如果 Actuator 曾經運作，
        // 但 Controller 後來 Disable，
        // 確保 HAL 回到 Base Count。
        // =====================================================

        if (!halActuatorFault &&
            lastAppliedHalCounts != 0 &&
            pMaster->m_dcHalBaseCounts > 0 &&
            lastAppliedHalCounts !=
            (ULONG)
            pMaster->m_dcHalBaseCounts)
        {
            bool restoreOk =
                RtSetHalTimerPeriodCounts(
                    (ULONG)
                    pMaster->
                    m_dcHalBaseCounts);


            if (restoreOk)
            {
                lastAppliedHalCounts =
                    (ULONG)
                    pMaster->
                    m_dcHalBaseCounts;


                PublishP64DeferredDiagnostic(
                    EtherCatP64DeferredDiagKind::HalBurstDisabledBaseRestored,
                    static_cast<uint64_t>(pMaster->tickCount_PDO),
                    static_cast<int64_t>(pMaster->m_dcHalBaseCounts),
                    0LL,
                    0LL,
                    0LL,
                    0LL,
                    0LL,
                    0LL,
                    0LL);
            }
        }
    }


    // =========================================================
    // PDO Handler Interval Diagnostic（CLOCK_2，只量測、不修正）
    //
    // CLOCK_2 only.
    //
    // Expected:
    //
    // 250000 ns
    // =========================================================

    static uint64_t previousPdoStartNs =
        0;


    static uint64_t pdoIntervalSumNs =
        0;


    static uint64_t pdoIntervalMinNs =
        0;


    static uint64_t pdoIntervalMaxNs =
        0;


    static uint32_t pdoIntervalSamples =
        0;


    static uint32_t pdoShortCount =
        0;


    static uint32_t pdoNormalCount =
        0;


    static uint32_t pdoLongCount =
        0;
    // ---------------------------------------------------------
// 保存最近完成的 Timer 視窗；等 Combined 與 Exec 視窗也完成後一次發布。
// ---------------------------------------------------------

    static uint64_t latestPdoTimerAvgNs =
        0;

    static uint64_t latestPdoTimerMinNs =
        0;

    static uint64_t latestPdoTimerMaxNs =
        0;

    static uint32_t latestPdoTimerShortCount =
        0;

    static uint32_t latestPdoTimerNormalCount =
        0;

    static uint32_t latestPdoTimerLongCount =
        0;

    static bool latestPdoTimerValid =
        false;

    if (previousPdoStartNs != 0 &&
        pdoCycleStartMasterNs >=
        previousPdoStartNs)
    {
        uint64_t intervalNs =
            pdoCycleStartMasterNs -
            previousPdoStartNs;


        if (pdoIntervalSamples == 0)
        {
            pdoIntervalMinNs =
                intervalNs;


            pdoIntervalMaxNs =
                intervalNs;
        }


        if (intervalNs <
            pdoIntervalMinNs)
        {
            pdoIntervalMinNs =
                intervalNs;
        }


        if (intervalNs >
            pdoIntervalMaxNs)
        {
            pdoIntervalMaxNs =
                intervalNs;
        }


        pdoIntervalSumNs +=
            intervalNs;


        pdoIntervalSamples++;


        if (intervalNs <
            200000ULL)
        {
            pdoShortCount++;
        }
        else if (intervalNs >
            300000ULL)
        {
            pdoLongCount++;
        }
        else
        {
            pdoNormalCount++;
        }


        if (pdoIntervalSamples >=
            4000U)
        {
            uint64_t averageNs =
                pdoIntervalSumNs /
                (uint64_t)
                pdoIntervalSamples;


            latestPdoTimerAvgNs =
                averageNs;

            latestPdoTimerMinNs =
                pdoIntervalMinNs;

            latestPdoTimerMaxNs =
                pdoIntervalMaxNs;

            latestPdoTimerShortCount =
                pdoShortCount;

            latestPdoTimerNormalCount =
                pdoNormalCount;

            latestPdoTimerLongCount =
                pdoLongCount;

            latestPdoTimerValid =
                true;


            // =====================================================
            // Reset PDO Timer Window
            // =====================================================

            pdoIntervalSumNs =
                0;

            pdoIntervalMinNs =
                0;

            pdoIntervalMaxNs =
                0;

            pdoIntervalSamples =
                0;

            pdoShortCount =
                0;

            pdoNormalCount =
                0;

            pdoLongCount =
                0;
        }
    }


    previousPdoStartNs =
        pdoCycleStartMasterNs;


    // =========================================================
    // PLC OUTPUT：Shadow Output -> EtherCAT IO Map
    //
    // Shadow Output -> EtherCAT IO Map
    // =========================================================

    static uint32_t pdoConsecutiveInvalidCycles =
        0;

    // DC-RX.3A：DC sample transport 與 LRW Process Data 各自維護品質。
    // DC-only invalid 不會增加 pdoConsecutiveInvalidCycles，也不會阻止 PLC/Motion；
    // 它只讓 DC controller 進入 last-known-good frequency holdover。
    static uint32_t dcTransportConsecutiveInvalidCycles = 0;
    static uint32_t dcTransportMaximumInvalidCycles = 0;
    static uint64_t dcWkcInvalidCyclesTotal = 0;
    static uint64_t dcOnlyInvalidCyclesTotal = 0;
    static uint64_t dcTransportRecoveryTotal = 0;
    static bool dcTransportQualityKnown = false;
    static bool previousDcTransportValid = true;

    // 上一週期 Process Data 有效才把新的 PLC output 刷入實體 IO Map；若 LRW
    // 已失效，保留最後送出資料並等待安全處置，避免錯誤期間注入變化命令。
    if (pdoConsecutiveInvalidCycles == 0)
    {
        pMaster->m_Plc.FlushOutputs();
    }


    // =========================================================
    // EtherCAT Combined Frame：本週期唯一主要 round trip
    //
    // Datagram #1 是 LRW Process Data；Datagram #2 是 FRMW DC Reference 0x0910。
    // 合併成一個 Ethernet frame／一次 round trip，降低額外 frame 對 250 us 的干擾。
    // =========================================================

    uint64_t combinedCommStartNs =
        pMaster->GetCurrentMasterTimeNs();


    int wkc =
        0;


    int dcWkc =
        0;

    // DC-RX.3D receives the exact software TX/RX window only when a fully
    // correlated LRW+FRMW response is accepted by ecx_LRW_FRMW().
    EtherCatDcCycleTiming dcCycleTiming = {};

    // =============================================================
// EtherCAT combined call 前的 QPC 時間戳
//
// 注意：
// 這裡目前量的是
//
// Handler Wake
//      ->
// ecx_LRW_FRMW() Call
//
// 還不是實際 NIC SendPacket。
// 下一階段會再量 Frame Build -> SendPacket。
// =============================================================

    LARGE_INTEGER qpcBeforeEcat = {};

    bool qpcBeforeEcatValid =
        false;


    if (qpcWakeValid)
    {
        if (RtQueryPerformanceCounter(
            &qpcBeforeEcat))
        {
            if (qpcBeforeEcat.QuadPart >=
                qpcWake.QuadPart)
            {
                qpcBeforeEcatValid =
                    true;
            }
        }
    }
    // =============================================================
// QPC <-> DC Reference estimator 的 call 前時間戳；call 後再取一次，兩者中點
// 近似 DC_reference_time 被讀回的主站時刻，RTT 則用於樣本品質門檻。
// =============================================================

    LARGE_INTEGER qpcDcBefore = {};

    bool qpcDcBeforeValid =
        false;


    if (qpcValid)
    {
        if (RtQueryPerformanceCounter(
            &qpcDcBefore))
        {
            qpcDcBeforeValid =
                true;
        }
    }
    // DC Reference 已在 StartDcPdoRuntime() 啟動階段選定；即時路徑只讀一次索引。
    const int dcReferenceSlaveIndex =
        GetDcReferenceSlaveIndex();

    if (dcReferenceSlaveIndex >= 0)
    {
        wkc =
            pMaster->ecx_LRW_FRMW(
                0x00000000,
                (uint16_t)
                pMaster->m_IoMapSize,
                pMaster->m_IoMap,
                m_slaveInfo[
                    dcReferenceSlaveIndex
                ].configAddr,
                &pMaster->
                        DC_reference_time,
                        &dcWkc,
                        50,
                        &dcCycleTiming);
    }
    else
    {
        // =====================================================
        // Fallback:
        //
        // 沒有 DC Reference Slave 時，
        // 至少維持普通 LRW。
        // =====================================================

        wkc =
            pMaster->ecx_LRW(
                0x00000000,
                (uint16_t)
                pMaster->m_IoMapSize,
                pMaster->m_IoMap,
                50);
    }

    LARGE_INTEGER qpcDcAfter = {};

    bool qpcDcAfterValid =
        false;


    if (qpcDcBeforeValid)
    {
        if (RtQueryPerformanceCounter(
            &qpcDcAfter))
        {
            if (qpcDcAfter.QuadPart >=
                qpcDcBefore.QuadPart)
            {
                qpcDcAfterValid =
                    true;
            }
        }
    }

    // ---------------------------------------------------------
    // DC-RX.3F logical transport/sample overrides
    //
    // The real LRW+FRMW transaction always completes first. A test replaces
    // only the result consumed by the downstream quality state machines, so
    // fault containment is reproducible without intentionally disturbing the
    // physical EtherCAT wire or NAL queue.
    // ---------------------------------------------------------
    const bool dcRx3fActualLrwValid =
        wkc == pMaster->EXPECTED_WKC_PDO;
    const bool dcRx3fActualDcValid =
        dcReferenceSlaveIndex < 0 || dcWkc > 0;
    const bool dcRx3fActualCombinedValid =
        dcRx3fActualLrwValid && dcRx3fActualDcValid;
    const bool dcRx3fActualExactTimingValid =
        dcCycleTiming.valid != 0U &&
        dcCycleTiming.source ==
        (uint32_t)EtherCatDcCycleTimingSource::ExactSoftwareTxRx &&
        dcCycleTiming.softwareRoundTripCounts > 0 &&
        dcCycleTiming.sampleMidpointQpc > 0;

    if (dcRx3fInjectionRequestedThisCycle &&
        !dcRx3fInjectionAppliedThisCycle)
    {
        switch (dcRx3fScenario)
        {
        case DcRx3fFaultScenario::DcWkcDrop:
            if (dcReferenceSlaveIndex >= 0 &&
                dcRx3fActualCombinedValid)
            {
                dcWkc = 0;
                dcRx3fInjectionAppliedThisCycle = true;
            }
            break;

        case DcRx3fFaultScenario::LrwWkcDrop:
            if (dcRx3fActualCombinedValid &&
                pMaster->EXPECTED_WKC_PDO > 0)
            {
                wkc = pMaster->EXPECTED_WKC_PDO - 1;
                dcRx3fInjectionAppliedThisCycle = true;
            }
            break;

        case DcRx3fFaultScenario::LrwTimeout:
            if (dcRx3fActualCombinedValid)
            {
                wkc = -1;
                dcRx3fInjectionAppliedThisCycle = true;
            }
            break;

        case DcRx3fFaultScenario::ExactTimingMissing:
            if (dcRx3fActualCombinedValid &&
                dcRx3fActualExactTimingValid)
            {
                dcCycleTiming.valid = 0U;
                dcRx3fInjectionAppliedThisCycle = true;
            }
            break;

        case DcRx3fFaultScenario::LateRtt:
            if (dcRx3fActualCombinedValid &&
                dcRx3fActualExactTimingValid &&
                qpcFrequency > 0 &&
                dcRx3fConfiguredValueNs > 0 &&
                dcRx3fConfiguredValueNs <=
                0xFFFFFFFFFFFFFFFFULL / qpcFrequency)
            {
                const uint64_t forcedRttProduct =
                    dcRx3fConfiguredValueNs * qpcFrequency;

                uint64_t forcedRttCounts =
                    forcedRttProduct / 1000000000ULL;

                if ((forcedRttProduct % 1000000000ULL) != 0)
                {
                    forcedRttCounts++;
                }

                if (forcedRttCounts == 0)
                {
                    forcedRttCounts = 1;
                }

                dcCycleTiming.softwareRoundTripCounts =
                    forcedRttCounts;
                dcRx3fInjectionAppliedThisCycle = true;
            }
            break;

        case DcRx3fFaultScenario::DcTimestampRepeat:
            if (dcRx3fActualCombinedValid &&
                dcRx3fBaselineAcceptedDcNs > 0)
            {
                pMaster->DC_reference_time =
                    dcRx3fBaselineAcceptedDcNs;
                dcRx3fInjectionAppliedThisCycle = true;
            }
            break;

        case DcRx3fFaultScenario::DcTimestampBackward:
            if (dcRx3fActualCombinedValid &&
                dcRx3fConfiguredValueNs > 0 &&
                dcRx3fBaselineAcceptedDcNs >
                dcRx3fConfiguredValueNs)
            {
                pMaster->DC_reference_time =
                    dcRx3fBaselineAcceptedDcNs -
                    dcRx3fConfiguredValueNs;
                dcRx3fInjectionAppliedThisCycle = true;
            }
            break;

        default:
            break;
        }
    }

    const bool pdoWkcValid =
        wkc == pMaster->EXPECTED_WKC_PDO;

    // DC-RX.3A/3B quality split:
    //   Process Data validity is decided by LRW WKC only.
    //   DC transport validity additionally requires a successful FRMW DC WKC.
    // A DC-only miss therefore freezes/requalifies DC observers while PDO/PLC/Motion
    // continue from a coherent LRW process image.
    const bool processDataValid =
        pdoWkcValid;

    const bool dcReferencePresent =
        dcReferenceSlaveIndex >= 0;

    const bool dcWkcValid =
        !dcReferencePresent ||
        dcWkc > 0;

    const bool dcTransportValid =
        processDataValid &&
        dcWkcValid;

    const bool dcOnlyInvalid =
        dcReferencePresent &&
        processDataValid &&
        !dcWkcValid;

    // ---------------------------------------------------------
    // DC-RX.3C/3D sample freshness + exact timing-source guard
    //
    // WKC proves that a matching FRMW datagram returned, but it does not prove
    // that the captured DC time is chronologically fresh. DC-RX.3D additionally
    // removes variable frame-build/post-RX software time from the QPC pairing by
    // preferring the exact SendPacket()/matching ReceivePacket() window returned
    // by ecx_LRW_FRMW().
    //
    // Policy:
    //   - Exact TX/RX timing is preferred immediately when available.
    //   - Whole-call midpoint remains a startup compatibility fallback only.
    //   - After exact timing has appeared once, a missing exact timestamp is a
    //     recoverable DC-only sample miss; the session never flaps back to the
    //     wider call midpoint and therefore never injects a fixed phase bias.
    // ---------------------------------------------------------
    uint64_t qpcDcRttNs = 0;
    uint64_t qpcDcMidCount = 0;
    uint64_t qpcDcApproxAgeNs = 0;
    uint64_t qpcDcExactRttNs = 0;
    uint64_t qpcDcCallRttNs = 0;
    uint64_t qpcDcExcludedOverheadNs = 0;
    int64_t qpcDcMidpointShiftNs = 0;
    bool qpcDcCallTimingAvailable = false;
    bool qpcDcExactTimingAvailable = false;
    const bool dcReferenceTimeValueValid =
        pMaster->DC_reference_time > 0;
    bool dcSampleTimingCandidate = false;
    bool dcSampleFreshnessAccepted = false;
    bool dcSampleGuardWarmupThisCycle = false;
    bool dcSampleFreshnessReanchorThisCycle = false;
    bool dcSampleTimingSourceChangedThisCycle = false;
    bool dcRttGuardEvaluatedThisCycle = false;
    bool dcRttGuardAcceptedThisCycle = false;
    bool dcRttGuardRebaseThisCycle = false;
    LONG dcSampleTimingSourceThisCycle = DC_RX3D_TIMING_SOURCE_NONE;
    LONG dcSampleGuardReasonMask = 0;

    uint64_t qpcDcCallMidCount = 0;
    if (dcTransportValid &&
        dcReferencePresent &&
        qpcDcBeforeValid &&
        qpcDcAfterValid &&
        qpcFrequency > 0)
    {
        const uint64_t qpcDcCallRttCounts =
            (uint64_t)(qpcDcAfter.QuadPart - qpcDcBefore.QuadPart);

        if (qpcDcCallRttCounts > 0 &&
            QpcCountsToNsSafe(qpcDcCallRttCounts, &qpcDcCallRttNs))
        {
            qpcDcCallMidCount =
                (uint64_t)qpcDcBefore.QuadPart +
                qpcDcCallRttCounts / 2ULL;
            qpcDcCallTimingAvailable = true;
        }
    }

    if (dcTransportValid &&
        dcReferencePresent &&
        dcCycleTiming.valid != 0U &&
        dcCycleTiming.source ==
        (uint32_t)EtherCatDcCycleTimingSource::ExactSoftwareTxRx &&
        dcCycleTiming.softwareRoundTripCounts > 0 &&
        dcCycleTiming.sampleMidpointQpc > 0 &&
        dcCycleTiming.rxAfterMatchQpc >= dcCycleTiming.txAfterSendQpc &&
        dcCycleTiming.txAfterSendQpc >= dcCycleTiming.txBeforeSendQpc &&
        QpcCountsToNsSafe(
            dcCycleTiming.softwareRoundTripCounts,
            &qpcDcExactRttNs))
    {
        qpcDcExactTimingAvailable = true;
    }

    // -----------------------------------------------------------------
    // DC-RX.3E: adaptive exact-RTT envelope / late-sample quarantine.
    //
    // The 210 us RX hard deadline decides whether Process Data arrived in
    // time.  DC phase/frequency control needs a stricter contract: a frame
    // can still beat that deadline yet carry an unusually large receive-IST,
    // queue, or scheduler delay in its software midpoint.  Such a frame is
    // valid Process Data, but it must not steer the DC observer.
    //
    // The guard learns the low-latency exact RTT floor for 32 ms, then accepts
    // Base + max(50 us, 3 x deviation + 10 us), capped at 200 us.  A stable
    // new latency regime may rebase after 64 coherent samples (16 ms).  An
    // isolated late sample is rejected only from DC; PDO/PLC/Motion remain on
    // the Process Data quality path introduced by DC-RX.3A.
    // -----------------------------------------------------------------
    auto ComputeDcRttGuardLimit = [&]() -> uint64_t
    {
        if (dcRttGuardBaselineNs == 0)
        {
            return DC_RX3E_RTT_ABSOLUTE_LIMIT_NS;
        }

        uint64_t scaledDeviationNs = dcRttGuardDeviationNs;
        if (scaledDeviationNs >
            (0xFFFFFFFFFFFFFFFFULL - DC_RX3E_RTT_JITTER_BIAS_NS) /
            3ULL)
        {
            scaledDeviationNs = DC_RX3E_RTT_MAX_HEADROOM_NS;
        }
        else
        {
            scaledDeviationNs =
                scaledDeviationNs * 3ULL +
                DC_RX3E_RTT_JITTER_BIAS_NS;
        }

        uint64_t headroomNs = scaledDeviationNs;
        if (headroomNs < DC_RX3E_RTT_MIN_HEADROOM_NS)
        {
            headroomNs = DC_RX3E_RTT_MIN_HEADROOM_NS;
        }
        if (headroomNs > DC_RX3E_RTT_MAX_HEADROOM_NS)
        {
            headroomNs = DC_RX3E_RTT_MAX_HEADROOM_NS;
        }

        uint64_t limitNs =
            dcRttGuardBaselineNs >
            0xFFFFFFFFFFFFFFFFULL - headroomNs
            ? 0xFFFFFFFFFFFFFFFFULL
            : dcRttGuardBaselineNs + headroomNs;

        if (limitNs > DC_RX3E_RTT_ABSOLUTE_LIMIT_NS)
        {
            limitNs = DC_RX3E_RTT_ABSOLUTE_LIMIT_NS;
        }

        return limitNs;
    };

    auto UpdateDcRttGuardDeviation = [&](uint64_t residualNs)
    {
        // A late spike must not inflate the envelope enough to admit the next
        // spike.  The tracked deviation itself is therefore bounded.
        if (residualNs > DC_RX3E_RTT_MAX_HEADROOM_NS)
        {
            residualNs = DC_RX3E_RTT_MAX_HEADROOM_NS;
        }

        if (residualNs > dcRttGuardDeviationNs)
        {
            // Rise moderately (1/32) so sustained jitter is learned without
            // reacting to one sample.
            dcRttGuardDeviationNs +=
                (residualNs - dcRttGuardDeviationNs + 31ULL) /
                32ULL;
        }
        else if (dcRttGuardDeviationNs > residualNs)
        {
            // Decay more slowly (1/64), avoiding a chattering threshold.
            dcRttGuardDeviationNs -=
                (dcRttGuardDeviationNs - residualNs + 63ULL) /
                64ULL;
        }
    };

    auto ResetDcRttGuardRebaseCandidate = [&]()
    {
        dcRttGuardRebaseCandidateSamples = 0;
        dcRttGuardRebaseCandidateMeanNs = 0;
        dcRttGuardRebaseCandidateMinNs = 0;
        dcRttGuardRebaseCandidateMaxNs = 0;
    };

    if (qpcDcExactTimingAvailable)
    {
        dcRttGuardEvaluatedThisCycle = true;
        dcRttGuardLastRttNs = qpcDcExactRttNs;

        if (qpcDcExactRttNs > DC_RX3E_RTT_ABSOLUTE_LIMIT_NS)
        {
            // Never learn from or rebase to a sample inside the final 10 us
            // before the 210 us Process Data hard deadline.
            if (dcRttGuardState == DC_RX3E_RTT_STATE_TRACK ||
                dcRttGuardState == DC_RX3E_RTT_STATE_SHIFT_CHECK)
            {
                dcRttGuardState = DC_RX3E_RTT_STATE_SHIFT_CHECK;
            }

            dcRttGuardLastExcessNs =
                qpcDcExactRttNs - DC_RX3E_RTT_ABSOLUTE_LIMIT_NS;

            if (dcRttGuardOutlierStreak != 0xFFFFFFFFU)
            {
                dcRttGuardOutlierStreak++;
            }
            if (dcRttGuardOutlierStreak >
                dcRttGuardOutlierMaximumStreak)
            {
                dcRttGuardOutlierMaximumStreak =
                    dcRttGuardOutlierStreak;
            }

            ResetDcRttGuardRebaseCandidate();
        }
        else if (dcRttGuardState == DC_RX3E_RTT_STATE_UNBOUND)
        {
            dcRttGuardBaselineNs = qpcDcExactRttNs;
            dcRttGuardDeviationNs =
                DC_RX3E_RTT_INITIAL_DEVIATION_NS;
            dcRttGuardWarmupSamples = 1U;
            dcRttGuardState = DC_RX3E_RTT_STATE_WARMUP;
            dcRttGuardOutlierStreak = 0;
            dcRttGuardLastExcessNs = 0;
            ResetDcRttGuardRebaseCandidate();
            dcRttGuardAcceptedThisCycle = true;
        }
        else if (dcRttGuardState == DC_RX3E_RTT_STATE_WARMUP)
        {
            // Learn the low-latency floor.  Startup DC controllers are already
            // qualification-gated, so these bounded samples may establish the
            // envelope without changing the normal Phase-P gain.
            if (qpcDcExactRttNs < dcRttGuardBaselineNs)
            {
                dcRttGuardBaselineNs = qpcDcExactRttNs;
            }

            const uint64_t residualNs =
                qpcDcExactRttNs >= dcRttGuardBaselineNs
                ? qpcDcExactRttNs - dcRttGuardBaselineNs
                : dcRttGuardBaselineNs - qpcDcExactRttNs;

            UpdateDcRttGuardDeviation(residualNs);

            if (dcRttGuardWarmupSamples <
                DC_RX3E_RTT_WARMUP_SAMPLES)
            {
                dcRttGuardWarmupSamples++;
            }
            if (dcRttGuardWarmupSamples >=
                DC_RX3E_RTT_WARMUP_SAMPLES)
            {
                dcRttGuardState = DC_RX3E_RTT_STATE_TRACK;
            }

            dcRttGuardOutlierStreak = 0;
            dcRttGuardLastExcessNs = 0;
            ResetDcRttGuardRebaseCandidate();
            dcRttGuardAcceptedThisCycle = true;
        }
        else
        {
            dcRttGuardLimitNs = ComputeDcRttGuardLimit();

            if (qpcDcExactRttNs <= dcRttGuardLimitNs)
            {
                dcRttGuardState = DC_RX3E_RTT_STATE_TRACK;
                dcRttGuardOutlierStreak = 0;
                dcRttGuardLastExcessNs = 0;
                ResetDcRttGuardRebaseCandidate();

                // Follow a lower latency floor quickly but a higher one very
                // slowly.  This prevents ordinary late samples from dragging
                // the accepted envelope upward.
                if (qpcDcExactRttNs < dcRttGuardBaselineNs)
                {
                    dcRttGuardBaselineNs -=
                        (dcRttGuardBaselineNs - qpcDcExactRttNs + 3ULL) /
                        4ULL;
                }
                else if (qpcDcExactRttNs > dcRttGuardBaselineNs)
                {
                    dcRttGuardBaselineNs +=
                        (qpcDcExactRttNs - dcRttGuardBaselineNs + 1023ULL) /
                        1024ULL;
                }

                const uint64_t residualNs =
                    qpcDcExactRttNs >= dcRttGuardBaselineNs
                    ? qpcDcExactRttNs - dcRttGuardBaselineNs
                    : dcRttGuardBaselineNs - qpcDcExactRttNs;

                UpdateDcRttGuardDeviation(residualNs);
                dcRttGuardAcceptedThisCycle = true;
            }
            else
            {
                dcRttGuardState = DC_RX3E_RTT_STATE_SHIFT_CHECK;
                dcRttGuardLastExcessNs =
                    qpcDcExactRttNs - dcRttGuardLimitNs;

                if (dcRttGuardOutlierStreak != 0xFFFFFFFFU)
                {
                    dcRttGuardOutlierStreak++;
                }
                if (dcRttGuardOutlierStreak >
                    dcRttGuardOutlierMaximumStreak)
                {
                    dcRttGuardOutlierMaximumStreak =
                        dcRttGuardOutlierStreak;
                }

                bool rebaseCandidateConsistent = false;

                if (dcRttGuardRebaseCandidateSamples == 0U)
                {
                    rebaseCandidateConsistent = true;
                    dcRttGuardRebaseCandidateSamples = 1U;
                    dcRttGuardRebaseCandidateMeanNs =
                        qpcDcExactRttNs;
                    dcRttGuardRebaseCandidateMinNs =
                        qpcDcExactRttNs;
                    dcRttGuardRebaseCandidateMaxNs =
                        qpcDcExactRttNs;
                }
                else
                {
                    const uint64_t candidateDifferenceNs =
                        qpcDcExactRttNs >=
                        dcRttGuardRebaseCandidateMeanNs
                        ? qpcDcExactRttNs -
                        dcRttGuardRebaseCandidateMeanNs
                        : dcRttGuardRebaseCandidateMeanNs -
                        qpcDcExactRttNs;

                    const uint64_t candidateMinimumNs =
                        qpcDcExactRttNs <
                        dcRttGuardRebaseCandidateMinNs
                        ? qpcDcExactRttNs
                        : dcRttGuardRebaseCandidateMinNs;

                    const uint64_t candidateMaximumNs =
                        qpcDcExactRttNs >
                        dcRttGuardRebaseCandidateMaxNs
                        ? qpcDcExactRttNs
                        : dcRttGuardRebaseCandidateMaxNs;

                    rebaseCandidateConsistent =
                        candidateDifferenceNs <=
                        DC_RX3E_RTT_REBASE_CONSISTENCY_NS &&
                        candidateMaximumNs - candidateMinimumNs <=
                        DC_RX3E_RTT_REBASE_RANGE_NS;

                    if (rebaseCandidateConsistent)
                    {
                        if (dcRttGuardRebaseCandidateSamples !=
                            0xFFFFFFFFU)
                        {
                            dcRttGuardRebaseCandidateSamples++;
                        }

                        const uint64_t divisor =
                            (uint64_t)dcRttGuardRebaseCandidateSamples;

                        if (qpcDcExactRttNs >=
                            dcRttGuardRebaseCandidateMeanNs)
                        {
                            dcRttGuardRebaseCandidateMeanNs +=
                                (qpcDcExactRttNs -
                                    dcRttGuardRebaseCandidateMeanNs) /
                                divisor;
                        }
                        else
                        {
                            dcRttGuardRebaseCandidateMeanNs -=
                                (dcRttGuardRebaseCandidateMeanNs -
                                    qpcDcExactRttNs) /
                                divisor;
                        }

                        dcRttGuardRebaseCandidateMinNs =
                            candidateMinimumNs;
                        dcRttGuardRebaseCandidateMaxNs =
                            candidateMaximumNs;
                    }
                    else
                    {
                        // Start a new proof sequence.  This sample itself is
                        // still rejected; a regime change must remain coherent
                        // for the full 64-sample qualification interval.
                        dcRttGuardRebaseCandidateSamples = 1U;
                        dcRttGuardRebaseCandidateMeanNs =
                            qpcDcExactRttNs;
                        dcRttGuardRebaseCandidateMinNs =
                            qpcDcExactRttNs;
                        dcRttGuardRebaseCandidateMaxNs =
                            qpcDcExactRttNs;
                    }
                }

                if (rebaseCandidateConsistent &&
                    dcRttGuardRebaseCandidateSamples >=
                    DC_RX3E_RTT_REBASE_SAMPLES &&
                    dcRttGuardRebaseCandidateMeanNs <=
                    DC_RX3E_RTT_ABSOLUTE_LIMIT_NS)
                {
                    const uint64_t candidateRangeNs =
                        dcRttGuardRebaseCandidateMaxNs -
                        dcRttGuardRebaseCandidateMinNs;

                    dcRttGuardBaselineNs =
                        dcRttGuardRebaseCandidateMeanNs;
                    dcRttGuardDeviationNs =
                        candidateRangeNs / 2ULL;

                    if (dcRttGuardDeviationNs <
                        DC_RX3E_RTT_INITIAL_DEVIATION_NS)
                    {
                        dcRttGuardDeviationNs =
                            DC_RX3E_RTT_INITIAL_DEVIATION_NS;
                    }

                    dcRttGuardWarmupSamples =
                        DC_RX3E_RTT_WARMUP_SAMPLES;
                    dcRttGuardState = DC_RX3E_RTT_STATE_TRACK;
                    dcRttGuardOutlierStreak = 0;
                    dcRttGuardLastExcessNs = 0;
                    dcRttGuardRebaseThisCycle = true;
                    dcRttGuardAcceptedThisCycle = true;

                    if (dcRttGuardRebaseTotal !=
                        0xFFFFFFFFFFFFFFFFULL)
                    {
                        dcRttGuardRebaseTotal++;
                    }

                    ResetDcRttGuardRebaseCandidate();
                }
            }
        }

        dcRttGuardLimitNs = ComputeDcRttGuardLimit();

        if (dcRttGuardAcceptedThisCycle)
        {
            if (dcRttGuardAcceptedTotal != 0xFFFFFFFFFFFFFFFFULL)
            {
                dcRttGuardAcceptedTotal++;
            }
        }
        else if (dcRttGuardRejectedTotal != 0xFFFFFFFFFFFFFFFFULL)
        {
            dcRttGuardRejectedTotal++;
        }
    }

    if (qpcDcExactTimingAvailable)
    {
        dcExactTimingLocked = true;
        dcSampleTimingSourceThisCycle =
            DC_RX3D_TIMING_SOURCE_EXACT_TXRX;
        qpcDcRttNs = qpcDcExactRttNs;
        qpcDcMidCount = dcCycleTiming.sampleMidpointQpc;

        if (dcTimingExactUseTotal != 0xFFFFFFFFFFFFFFFFULL)
        {
            dcTimingExactUseTotal++;
        }

        if (qpcDcExactRttNs > dcTimingExactRttMaximumNs)
        {
            dcTimingExactRttMaximumNs = qpcDcExactRttNs;
        }
    }
    else if (!dcExactTimingLocked && qpcDcCallTimingAvailable)
    {
        dcSampleTimingSourceThisCycle =
            DC_RX3D_TIMING_SOURCE_CALL_FALLBACK;
        qpcDcRttNs = qpcDcCallRttNs;
        qpcDcMidCount = qpcDcCallMidCount;

        if (dcTimingFallbackUseTotal != 0xFFFFFFFFFFFFFFFFULL)
        {
            dcTimingFallbackUseTotal++;
        }
    }
    else if (dcTransportValid && dcReferencePresent)
    {
        if (dcExactTimingLocked)
        {
            dcSampleGuardReasonMask |=
                DC_RX3D_SAMPLE_REJECT_TIMING_SOURCE;

            if (dcTimingMissingAfterLockTotal != 0xFFFFFFFFFFFFFFFFULL)
            {
                dcTimingMissingAfterLockTotal++;
            }
        }
        else
        {
            dcSampleGuardReasonMask |=
                DC_RX3C_SAMPLE_REJECT_AGE;
        }
    }

    if (dcSampleTimingSourceThisCycle != DC_RX3D_TIMING_SOURCE_NONE)
    {
        dcSampleTimingSourceChangedThisCycle =
            dcSampleTimingSource != DC_RX3D_TIMING_SOURCE_NONE &&
            dcSampleTimingSource != dcSampleTimingSourceThisCycle;

        if (dcSampleTimingSourceChangedThisCycle &&
            dcTimingSourceSwitchTotal != 0xFFFFFFFFFFFFFFFFULL)
        {
            dcTimingSourceSwitchTotal++;
        }

        dcSampleTimingSource = dcSampleTimingSourceThisCycle;
    }

    if (qpcDcExactTimingAvailable && qpcDcCallTimingAvailable)
    {
        qpcDcExcludedOverheadNs =
            qpcDcCallRttNs > qpcDcExactRttNs
            ? qpcDcCallRttNs - qpcDcExactRttNs
            : 0ULL;

        const bool exactMidAfterCallMid =
            dcCycleTiming.sampleMidpointQpc >= qpcDcCallMidCount;
        const uint64_t midpointShiftCounts =
            exactMidAfterCallMid
            ? dcCycleTiming.sampleMidpointQpc - qpcDcCallMidCount
            : qpcDcCallMidCount - dcCycleTiming.sampleMidpointQpc;
        uint64_t midpointShiftAbsNs = 0;

        if (QpcCountsToNsSafe(
            midpointShiftCounts,
            &midpointShiftAbsNs))
        {
            if (midpointShiftAbsNs > 0x7FFFFFFFFFFFFFFFULL)
            {
                qpcDcMidpointShiftNs =
                    exactMidAfterCallMid
                    ? 0x7FFFFFFFFFFFFFFFLL
                    : (-0x7FFFFFFFFFFFFFFFLL - 1LL);
            }
            else
            {
                qpcDcMidpointShiftNs =
                    exactMidAfterCallMid
                    ? (int64_t)midpointShiftAbsNs
                    : -(int64_t)midpointShiftAbsNs;
            }

            dcTimingMidpointShiftLastNs = qpcDcMidpointShiftNs;
            if (midpointShiftAbsNs >
                dcTimingMidpointShiftMaximumAbsNs)
            {
                dcTimingMidpointShiftMaximumAbsNs =
                    midpointShiftAbsNs;
            }
        }
    }

    if (dcSampleTimingSourceThisCycle != DC_RX3D_TIMING_SOURCE_NONE)
    {
        const bool sampleTransactionWithinBounds =
            dcReferenceTimeValueValid &&
            qpcDcRttNs > 0 &&
            qpcDcRttNs <= DC_RX3C_MAX_SAMPLE_TRANSACTION_NS;

        if (!sampleTransactionWithinBounds)
        {
            dcSampleGuardReasonMask |=
                DC_RX3C_SAMPLE_REJECT_AGE;
        }
        else if (dcSampleTimingSourceThisCycle ==
            DC_RX3D_TIMING_SOURCE_EXACT_TXRX &&
            dcRttGuardEvaluatedThisCycle &&
            !dcRttGuardAcceptedThisCycle)
        {
            dcSampleGuardReasonMask |=
                DC_RX3E_SAMPLE_REJECT_RTT_ENVELOPE;
        }
        else
        {
            qpcDcApproxAgeNs = qpcDcRttNs / 2ULL;
            dcSampleTimingCandidate = true;
        }
    }

    dcSampleGuardLastApproxAgeNs = qpcDcApproxAgeNs;
    if (qpcDcApproxAgeNs > dcSampleGuardMaximumApproxAgeNs)
    {
        dcSampleGuardMaximumApproxAgeNs = qpcDcApproxAgeNs;
    }

    auto BindDcSampleGuardAnchor = [&]()
    {
        dcSampleGuardLastAcceptedDcNs =
            (uint64_t)pMaster->DC_reference_time;
        dcSampleGuardLastAcceptedQpcMidCount = qpcDcMidCount;
        dcSampleGuardState = DC_RX3C_SAMPLE_STATE_VERIFY;
        dcSampleGuardConsecutiveRejects = 0;
        dcSampleGuardAnchorTotal++;
    };

    if (dcSampleTimingCandidate)
    {
        if (dcSampleGuardState == DC_RX3C_SAMPLE_STATE_UNBOUND)
        {
            // The first sample establishes chronology only. It is deliberately
            // not fed into a phase/frequency observer.
            BindDcSampleGuardAnchor();
            dcSampleGuardWarmupThisCycle = true;
            dcSampleGuardReasonMask = DC_RX3C_SAMPLE_REJECT_ANCHOR;
        }
        else if (dcSampleTimingSourceChangedThisCycle ||
            dcRttGuardRebaseThisCycle)
        {
            // A timing-source switch or a proven persistent RTT-regime shift
            // changes fixed software capture bias. Never bridge an observer
            // window across that boundary: bind a new anchor, force recoverable
            // HOLD, and quarantine Phase-P until fresh maps prove continuity.
            BindDcSampleGuardAnchor();
            dcSampleGuardReasonMask =
                DC_RX3C_SAMPLE_REJECT_ANCHOR;

            if (dcSampleTimingSourceChangedThisCycle)
            {
                dcSampleGuardReasonMask |=
                    DC_RX3D_SAMPLE_REJECT_TIMING_SOURCE;
            }
            if (dcRttGuardRebaseThisCycle)
            {
                dcSampleGuardReasonMask |=
                    DC_RX3E_SAMPLE_REJECT_RTT_ENVELOPE;
            }

            dcSampleGuardReanchorTotal++;
            dcSampleFreshnessReanchorThisCycle = true;
            dcPhaseMapRebindPending = true;
            ArmPhasePRecoveryJumpGuard(false);
        }
        else
        {
            const uint64_t currentDcNs =
                (uint64_t)pMaster->DC_reference_time;

            const bool qpcMonotonic =
                qpcDcMidCount > dcSampleGuardLastAcceptedQpcMidCount;
            const bool dcMonotonic =
                currentDcNs > dcSampleGuardLastAcceptedDcNs;

            uint64_t qpcDeltaNs = 0;
            uint64_t dcDeltaNs = 0;
            uint64_t deltaToleranceNs =
                DC_RX3C_DELTA_BASE_TOLERANCE_NS;
            int64_t deltaErrorNs = 0;
            bool deltaConversionValid = false;
            bool deltaPlausible = false;

            if (!qpcMonotonic)
            {
                dcSampleGuardReasonMask |=
                    DC_RX3C_SAMPLE_REJECT_QPC_ORDER;
            }

            if (!dcMonotonic)
            {
                dcSampleGuardReasonMask |=
                    DC_RX3C_SAMPLE_REJECT_DC_ORDER;
            }

            if (qpcMonotonic && dcMonotonic)
            {
                const uint64_t qpcDeltaCounts =
                    qpcDcMidCount -
                    dcSampleGuardLastAcceptedQpcMidCount;

                dcDeltaNs =
                    currentDcNs - dcSampleGuardLastAcceptedDcNs;

                deltaConversionValid =
                    QpcCountsToNsSafe(qpcDeltaCounts, &qpcDeltaNs);

                if (deltaConversionValid)
                {
                    uint64_t driftToleranceNs =
                        qpcDeltaNs / DC_RX3C_DELTA_DRIFT_DIVISOR;

                    if (driftToleranceNs >
                        DC_RX3C_DELTA_MAX_TOLERANCE_NS -
                        DC_RX3C_DELTA_BASE_TOLERANCE_NS)
                    {
                        deltaToleranceNs =
                            DC_RX3C_DELTA_MAX_TOLERANCE_NS;
                    }
                    else
                    {
                        deltaToleranceNs =
                            DC_RX3C_DELTA_BASE_TOLERANCE_NS +
                            driftToleranceNs;
                    }

                    const bool dcAhead = dcDeltaNs >= qpcDeltaNs;
                    const uint64_t absoluteDeltaErrorNs =
                        dcAhead
                        ? dcDeltaNs - qpcDeltaNs
                        : qpcDeltaNs - dcDeltaNs;

                    if (absoluteDeltaErrorNs > 0x7FFFFFFFFFFFFFFFULL)
                    {
                        deltaErrorNs =
                            dcAhead
                            ? 0x7FFFFFFFFFFFFFFFLL
                            : (-0x7FFFFFFFFFFFFFFFLL - 1LL);
                    }
                    else
                    {
                        deltaErrorNs =
                            dcAhead
                            ? (int64_t)absoluteDeltaErrorNs
                            : -(int64_t)absoluteDeltaErrorNs;
                    }

                    deltaPlausible =
                        absoluteDeltaErrorNs <= deltaToleranceNs;

                    if (!deltaPlausible)
                    {
                        dcSampleGuardReasonMask |=
                            DC_RX3C_SAMPLE_REJECT_DELTA;
                    }
                }
                else
                {
                    dcSampleGuardReasonMask |=
                        DC_RX3C_SAMPLE_REJECT_CONVERSION;
                }
            }

            dcSampleGuardLastQpcDeltaNs = qpcDeltaNs;
            dcSampleGuardLastDcDeltaNs = dcDeltaNs;
            dcSampleGuardLastDeltaErrorNs = deltaErrorNs;
            dcSampleGuardLastDeltaToleranceNs = deltaToleranceNs;

            const bool samplePlausible =
                qpcMonotonic &&
                dcMonotonic &&
                deltaConversionValid &&
                deltaPlausible;

            if (samplePlausible)
            {
                dcSampleFreshnessAccepted = true;
                dcSampleGuardState = DC_RX3C_SAMPLE_STATE_TRACK;
                dcSampleGuardLastAcceptedDcNs = currentDcNs;
                dcSampleGuardLastAcceptedQpcMidCount = qpcDcMidCount;
                dcSampleGuardConsecutiveRejects = 0;
                dcSampleGuardAcceptedTotal++;
                dcSampleGuardReasonMask = 0;
            }
            else
            {
                dcSampleGuardRejectedTotal++;

                if ((dcSampleGuardReasonMask &
                    DC_RX3C_SAMPLE_REJECT_AGE) != 0)
                {
                    dcSampleGuardAgeRejectTotal++;
                }

                if ((dcSampleGuardReasonMask &
                    (DC_RX3C_SAMPLE_REJECT_DC_ORDER |
                        DC_RX3C_SAMPLE_REJECT_QPC_ORDER)) != 0)
                {
                    dcSampleGuardOrderRejectTotal++;
                }

                if ((dcSampleGuardReasonMask &
                    (DC_RX3C_SAMPLE_REJECT_DELTA |
                        DC_RX3C_SAMPLE_REJECT_CONVERSION)) != 0)
                {
                    dcSampleGuardDeltaRejectTotal++;
                }

                if (dcSampleGuardConsecutiveRejects < 0xFFFFFFFFU)
                {
                    dcSampleGuardConsecutiveRejects++;
                }

                if (dcSampleGuardConsecutiveRejects >
                    dcSampleGuardMaximumConsecutiveRejects)
                {
                    dcSampleGuardMaximumConsecutiveRejects =
                        dcSampleGuardConsecutiveRejects;
                }

                if (dcSampleGuardState == DC_RX3C_SAMPLE_STATE_VERIFY)
                {
                    // Replace an unproven anchor. This lets one stale first
                    // frame recover on the following clean pair.
                    BindDcSampleGuardAnchor();
                }
                else if (dcSampleGuardConsecutiveRejects >=
                    DC_RX3C_REANCHOR_REJECT_LIMIT)
                {
                    // Four chronology/plausibility failures indicate that the
                    // reference coordinate may have discontinuously changed.
                    // Rebind phase mapping and force a real HOLD/requalification.
                    BindDcSampleGuardAnchor();
                    dcSampleGuardReanchorTotal++;
                    dcSampleFreshnessReanchorThisCycle = true;
                    dcPhaseMapRebindPending = true;
                    ArmPhasePRecoveryJumpGuard(false);
                }
            }
        }
    }
    else if (dcTransportValid && dcReferencePresent)
    {
        dcSampleGuardRejectedTotal++;

        if ((dcSampleGuardReasonMask &
            DC_RX3C_SAMPLE_REJECT_AGE) != 0)
        {
            dcSampleGuardAgeRejectTotal++;
        }

        if ((dcSampleGuardReasonMask &
            DC_RX3D_SAMPLE_REJECT_TIMING_SOURCE) != 0)
        {
            dcSampleGuardTimingRejectTotal++;
        }

        if ((dcSampleGuardReasonMask &
            DC_RX3E_SAMPLE_REJECT_RTT_ENVELOPE) != 0)
        {
            dcSampleGuardRttRejectTotal++;
        }

        if (dcSampleGuardConsecutiveRejects < 0xFFFFFFFFU)
        {
            dcSampleGuardConsecutiveRejects++;
        }

        if (dcSampleGuardConsecutiveRejects >
            dcSampleGuardMaximumConsecutiveRejects)
        {
            dcSampleGuardMaximumConsecutiveRejects =
                dcSampleGuardConsecutiveRejects;
        }
    }

    dcSampleGuardLastReasonMask = dcSampleGuardReasonMask;

    const bool dcSampleFreshnessInvalid =
        dcReferencePresent &&
        dcTransportValid &&
        !dcSampleFreshnessAccepted &&
        !dcSampleGuardWarmupThisCycle;

    const bool dcControlSampleValid =
        !dcReferencePresent
        ? processDataValid
        : dcSampleFreshnessAccepted;

    const bool dcControlSampleInvalid =
        dcReferencePresent &&
        processDataValid &&
        !dcControlSampleValid &&
        !dcSampleGuardWarmupThisCycle;

    if (dcReferencePresent)
    {
        if (!dcWkcValid &&
            dcWkcInvalidCyclesTotal != 0xFFFFFFFFFFFFFFFFULL)
        {
            dcWkcInvalidCyclesTotal++;
        }

        if (dcOnlyInvalid &&
            dcOnlyInvalidCyclesTotal != 0xFFFFFFFFFFFFFFFFULL)
        {
            dcOnlyInvalidCyclesTotal++;
        }

        if (!dcTransportValid)
        {
            if (dcTransportConsecutiveInvalidCycles < 0xFFFFFFFFU)
            {
                dcTransportConsecutiveInvalidCycles++;
            }

            if (dcTransportConsecutiveInvalidCycles >
                dcTransportMaximumInvalidCycles)
            {
                dcTransportMaximumInvalidCycles =
                    dcTransportConsecutiveInvalidCycles;
            }
        }
        else
        {
            if (dcTransportQualityKnown &&
                !previousDcTransportValid &&
                dcTransportRecoveryTotal != 0xFFFFFFFFFFFFFFFFULL)
            {
                dcTransportRecoveryTotal++;
            }

            dcTransportConsecutiveInvalidCycles = 0;
        }

        previousDcTransportValid = dcTransportValid;
        dcTransportQualityKnown = true;
    }
    else
    {
        // No DC reference means ordinary LRW mode. Keep the DC-only diagnostic
        // neutral while Process Data keeps its normal safety contract.
        dcTransportConsecutiveInvalidCycles = 0;
        previousDcTransportValid = true;
        dcTransportQualityKnown = false;
    }

    const bool currentSchedulerClean =
        currentSchedulerEvaluated &&
        currentSchedulerRearmOk &&
        !currentSchedulerUsedBootstrap &&
        !currentSchedulerRuntimeRecovery;

    // ---------------------------------------------------------
    // DC-RX.3B micro-glitch budget
    //
    // A stable ACTIVE controller may bridge at most four consecutive DC-only
    // misses (1 ms at 4 kHz), provided the leaky debt budget is not exhausted.
    // The missing DC samples are still rejected from every observer. Only the
    // expensive HOLD/reset/requalification transition is filtered.
    // ---------------------------------------------------------
    bool dcOnlyGraceAcceptedThisCycle = false;
    bool dcOnlyRequiresHoldThisCycle = false;

    if (dcControlSampleInvalid)
    {
        if (dcOnlyInvalidConsecutiveCycles < 0xFFFFFFFFU)
        {
            dcOnlyInvalidConsecutiveCycles++;
        }

        if (dcOnlyInvalidConsecutiveCycles >
            dcOnlyInvalidMaximumConsecutiveCycles)
        {
            dcOnlyInvalidMaximumConsecutiveCycles =
                dcOnlyInvalidConsecutiveCycles;
        }

        // Count every raw DC-only burst, including those fully absorbed by the
        // grace budget. HoldoverEntry counts only bursts promoted to HOLD.
        if (dcOnlyInvalidConsecutiveCycles == 1U &&
            dcHoldoverEpisodeTotal != 0xFFFFFFFFFFFFFFFFULL)
        {
            dcHoldoverEpisodeTotal++;
        }

        if (dcOnlyGlitchDebt <=
            DC_RX3B_GLITCH_DEBT_SATURATION - DC_RX3B_GLITCH_DEBT_ADD)
        {
            dcOnlyGlitchDebt += DC_RX3B_GLITCH_DEBT_ADD;
        }
        else
        {
            dcOnlyGlitchDebt = DC_RX3B_GLITCH_DEBT_SATURATION;
        }

        if (dcOnlyGlitchDebt > dcOnlyGlitchDebtMaximum)
        {
            dcOnlyGlitchDebtMaximum = dcOnlyGlitchDebt;
        }

        const bool graceEligible =
            realFfV0State == 2 &&
            realFfV0TripMask == 0 &&
            realFfV0TransientHoldReasonMask == 0 &&
            realFfV0RecoveryProfile == 0 &&
            currentSchedulerClean;

        dcOnlyGraceAcceptedThisCycle =
            graceEligible &&
            !dcSampleFreshnessReanchorThisCycle &&
            dcOnlyInvalidConsecutiveCycles <=
            DC_RX3B_GLITCH_GRACE_CYCLES &&
            dcOnlyGlitchDebt <= DC_RX3B_GLITCH_DEBT_LIMIT;

        if (dcOnlyGraceAcceptedThisCycle)
        {
            // Preserve forensic history without turning the micro-glitch into
            // an active HOLD reason. History never blocks STABLE by itself.
            realFfV0HistoryMask |=
                dcSampleFreshnessInvalid
                ? REAL_FF_V0_TRANSIENT_DC_FRESHNESS_REASON
                : REAL_FF_V0_TRANSIENT_DC_SAMPLE_REASON;

            if (dcGraceAcceptedCyclesTotal != 0xFFFFFFFFFFFFFFFFULL)
            {
                dcGraceAcceptedCyclesTotal++;
            }
        }
        else
        {
            dcOnlyRequiresHoldThisCycle = true;
        }
    }
    else if (dcControlSampleValid)
    {
        dcOnlyInvalidConsecutiveCycles = 0;

        if (dcOnlyGlitchDebt > DC_RX3B_GLITCH_DEBT_DECAY)
        {
            dcOnlyGlitchDebt -= DC_RX3B_GLITCH_DEBT_DECAY;
        }
        else
        {
            dcOnlyGlitchDebt = 0;
        }
    }
    else
    {
        // LRW invalid is governed by the existing PDO safety debounce. Do not
        // disguise it as a DC-only grace event, and do not erase recent debt.
        dcOnlyInvalidConsecutiveCycles = 0;
    }

    const bool currentControlCycleActuallyClean =
        dcControlSampleValid && currentSchedulerClean;

    const bool currentControlCyclePolicySafe =
        currentControlCycleActuallyClean ||
        dcOnlyGraceAcceptedThisCycle ||
        dcSampleGuardWarmupThisCycle;

    LONG currentControlCycleReasonMask = 0;
    if (!processDataValid)
    {
        currentControlCycleReasonMask |=
            REAL_FF_V0_TRANSIENT_RX_REASON;
    }
    else if (!dcWkcValid && !dcOnlyGraceAcceptedThisCycle)
    {
        currentControlCycleReasonMask |=
            REAL_FF_V0_TRANSIENT_DC_SAMPLE_REASON;
    }
    else if (dcSampleFreshnessInvalid && !dcOnlyGraceAcceptedThisCycle)
    {
        currentControlCycleReasonMask |=
            REAL_FF_V0_TRANSIENT_DC_FRESHNESS_REASON;
    }

    if (!currentSchedulerEvaluated ||
        currentSchedulerRuntimeRecovery)
    {
        currentControlCycleReasonMask |=
            REAL_FF_V0_TRANSIENT_SCHEDULER_REASON;
    }

    if (currentControlCycleActuallyClean)
    {
        // Saturating at the current qualification threshold keeps diagnostics
        // bounded. A conservative episode may raise this target from 32 to 128.
        if (realFfV0CleanCycleStreak < realFfV0CleanCyclesRequired)
        {
            realFfV0CleanCycleStreak++;
        }
    }
    else if (dcOnlyGraceAcceptedThisCycle)
    {
        // Do not claim a new clean sample, but preserve previously earned
        // qualification. Frequency and Phase-P commands remain unchanged.
    }
    else
    {
        const bool startsNewControlInterruption =
            !realFfV0CycleQualityKnown ||
            realFfV0PreviousCycleClean;

        realFfV0CleanCycleStreak = 0;

        if (dcOnlyRequiresHoldThisCycle && !dcHoldoverEpisodeActive)
        {
            dcHoldoverEpisodeActive = true;
            dcHoldoverUnqualifiedCycles = 0;

            if (dcHoldoverEntryTotal != 0xFFFFFFFFFFFFFFFFULL)
            {
                dcHoldoverEntryTotal++;
            }
        }

        if (currentControlCycleReasonMask != 0)
        {
            // The first non-grace DC-only miss reopens partial observers.
            // Subsequent callbacks preserve Last-Known-Good FF in HOLD.
            EnterRealFfV0TransientHold(
                currentControlCycleReasonMask,
                startsNewControlInterruption);
        }
    }

    // ---------------------------------------------------------
    // Bounded holdover escalation
    //
    // UnqualifiedCycles measures how long a forced DC-only episode has failed
    // to earn its current continuous clean gate. It stops accumulating once
    // that gate is met, so normal multi-window observer requalification is not
    // mistaken for continuing transport loss.
    // ---------------------------------------------------------
    if (dcHoldoverEpisodeActive &&
        !IsRealFfV0CleanRecoveryReady() &&
        processDataValid)
    {
        if (dcHoldoverUnqualifiedCycles < 0xFFFFFFFFU)
        {
            dcHoldoverUnqualifiedCycles++;
        }

        if (dcHoldoverUnqualifiedCycles >
            dcHoldoverMaximumUnqualifiedCycles)
        {
            dcHoldoverMaximumUnqualifiedCycles =
                dcHoldoverUnqualifiedCycles;
        }

        if (dcHoldoverUnqualifiedCycles >=
            DC_RX3B_DEGRADED_HOLDOVER_CYCLES &&
            realFfV0RecoveryProfile < 1)
        {
            ApplyRealFfV0ConservativeRecoveryProfile();

            if (dcDegradedEntryTotal != 0xFFFFFFFFFFFFFFFFULL)
            {
                dcDegradedEntryTotal++;
            }

            // Start the conservative qualification from a post-escalation
            // observer publication, not from a partially unstable window.
            if (!dcObserversResetThisCycle)
            {
                ResetDcObserversAfterTransientInterruption();
            }
        }

        if (dcHoldoverUnqualifiedCycles >=
            DC_RX3B_FULL_RELOCK_CYCLES &&
            realFfV0RecoveryProfile < 2 &&
            (realFfV0TripMask & REAL_FF_V0_HARD_TRIP_MASK) == 0)
        {
            if (dcRelockEntryTotal != 0xFFFFFFFFFFFFFFFFULL)
            {
                dcRelockEntryTotal++;
            }

            realFfV0TripCount++;
            EscalateRealFfV0ToSoftDcRelock();
        }
    }

    if ((realFfV0TripMask & REAL_FF_V0_SOFT_DC_RELOCK_REASON) != 0)
    {
        dcHoldoverDiagTier = DC_RX3B_TIER_RELOCK;
    }
    else if (dcHoldoverEpisodeActive)
    {
        dcHoldoverDiagTier =
            IsRealFfV0CleanRecoveryReady()
            ? DC_RX3B_TIER_REQUALIFY
            : (realFfV0RecoveryProfile >= 1
                ? DC_RX3B_TIER_DEGRADED
                : DC_RX3B_TIER_HOLDOVER);
    }
    else if (dcOnlyGraceAcceptedThisCycle)
    {
        dcHoldoverDiagTier = DC_RX3B_TIER_GRACE;
    }
    else
    {
        dcHoldoverDiagTier = DC_RX3B_TIER_NONE;
    }

    realFfV0PreviousCycleClean =
        currentControlCyclePolicySafe;
    realFfV0PreviousCycleReasonMask =
        currentControlCycleReasonMask;
    realFfV0PreviousDcTransportValid =
        dcTransportValid;
    realFfV0PreviousDcSampleQualified =
        dcControlSampleValid;
    realFfV0CycleQualityKnown = true;

    // Process Data safety 只看 LRW WKC。DC-only invalid 不得增加 PDO invalid
    // debounce，也不得阻止 FetchInputs / Motion。沒有 DC reference 時維持普通 LRW。
    if (processDataValid)
    {
        pdoConsecutiveInvalidCycles =
            0;
    }
    else if (pdoConsecutiveInvalidCycles <
        0xFFFFFFFFU)
    {
        pdoConsecutiveInvalidCycles++;
    }

    // =============================================================
// QPC <-> DC Reference Frequency Estimator V1（約一秒視窗）
//
// 固定 capture bias 會在 elapsed 差分中抵消，因此此處只比較長時間 slope/frequency。
//
// Window:
//     4000 valid samples ≈ 1 second
// =============================================================

    static uint64_t
        qpcDcWindowStartMidCount =
        0;

    static uint64_t
        qpcDcWindowEndMidCount =
        0;

    static uint64_t
        qpcDcWindowStartDcNs =
        0;

    static uint64_t
        qpcDcWindowEndDcNs =
        0;


    static uint64_t
        qpcDcRttSumNs =
        0;

    static uint64_t
        qpcDcRttMinNs =
        0;

    static uint64_t
        qpcDcRttMaxNs =
        0;


    static uint32_t
        qpcDcWindowValidSamples =
        0;

    static uint32_t
        qpcDcWindowRejectedSamples =
        0;


    // =============================================================
    // Trusted Live-FF DC Phase Predictor V1 內部狀態（shadow）
    //
    // wrapped 值限制在單一 250 us 週期；unwrapped 值跨週期累積，適合觀察
    // 長期斜率。BoundInitCount 用來辨識 scheduler re-anchor，避免跨基準混算。
    // =============================================================

    static bool
        qpcLiveFfDcPhaseInitialized =
        false;

    static uint32_t
        qpcLiveFfDcPhaseBoundInitCount =
        0;

    static int64_t
        qpcLiveFfDcPhaseBaselinePhaseNs =
        0;

    static int64_t
        qpcLiveFfDcPhaseBaselineSync0MarginNs =
        0;


    static bool
        qpcLiveFfDcPhaseFixedUnwrapValid =
        false;

    static bool
        qpcLiveFfDcPhaseShadowUnwrapValid =
        false;

    static int64_t
        qpcLiveFfDcPhasePrevFixedWrappedNs =
        0;

    static int64_t
        qpcLiveFfDcPhasePrevShadowWrappedNs =
        0;

    static int64_t
        qpcLiveFfDcPhaseFixedUnwrappedNs =
        0;

    static int64_t
        qpcLiveFfDcPhaseShadowUnwrappedNs =
        0;


    static uint32_t
        qpcLiveFfDcPhaseWindowSamples =
        0;

    static int64_t
        qpcLiveFfDcPhaseFixedWindowStartNs =
        0;

    static int64_t
        qpcLiveFfDcPhaseFixedWindowEndNs =
        0;

    static int64_t
        qpcLiveFfDcPhaseFixedWindowMinNs =
        0;

    static int64_t
        qpcLiveFfDcPhaseFixedWindowMaxNs =
        0;

    static int64_t
        qpcLiveFfDcPhaseShadowWindowStartNs =
        0;

    static int64_t
        qpcLiveFfDcPhaseShadowWindowEndNs =
        0;

    static int64_t
        qpcLiveFfDcPhaseShadowWindowMinNs =
        0;

    static int64_t
        qpcLiveFfDcPhaseShadowWindowMaxNs =
        0;


    static uint64_t
        qpcLiveFfDcPhaseFixedAbsSumNs =
        0;

    static uint64_t
        qpcLiveFfDcPhaseShadowAbsSumNs =
        0;


    static uint32_t
        qpcLiveFfDcPhaseShadowBetterCount =
        0;

    static uint32_t
        qpcLiveFfDcPhaseShadowWorseCount =
        0;

    static uint32_t
        qpcLiveFfDcPhaseEqualCount =
        0;


    static uint64_t
        qpcLiveFfDcPhaseMapRttSumNs =
        0;

    static uint64_t
        qpcLiveFfDcPhaseMapRttMinNs =
        0;

    static uint64_t
        qpcLiveFfDcPhaseMapRttMaxNs =
        0;


    // =============================================================
    // Frequency FF V2 - DC Reference phase 比較內部狀態（shadow）
    // =============================================================

    static bool
        qpcPhaseFfV2DcInitialized =
        false;

    static bool
        qpcPhaseFfV2DcUnwrapValid =
        false;

    static int64_t
        qpcPhaseFfV2DcPrevWrappedNs =
        0;

    static int64_t
        qpcPhaseFfV2DcUnwrappedNs =
        0;

    static uint32_t
        qpcPhaseFfV2DcWindowSamples =
        0;

    static int64_t
        qpcPhaseFfV2DcWindowStartNs =
        0;

    static int64_t
        qpcPhaseFfV2DcWindowEndNs =
        0;

    static int64_t
        qpcPhaseFfV2DcWindowMinNs =
        0;

    static int64_t
        qpcPhaseFfV2DcWindowMaxNs =
        0;

    static uint64_t
        qpcPhaseFfV2DcAbsSumNs =
        0;

    static uint32_t
        qpcPhaseFfV2DcBetterThanFixed =
        0;

    static uint32_t
        qpcPhaseFfV2DcBetterThanTrusted =
        0;


    // Actual DC Reference elapsed span for each 4000-sample phase window.
    static uint64_t
        qpcLiveFfDcPhaseWindowStartDcNs =
        0;

    static uint64_t
        qpcLiveFfDcPhaseWindowEndDcNs =
        0;


    // =============================================================
    // DC Reference Phase Residual Drift Observer V1（僅診斷）
    //
    // Ring=16；單窗需 0.5..1.5 s、RTT <=250 us、|residual|<=5000 ppb。
    // 至少 8 筆且 MAD<=1000 ppb 才 Locked。拒絕樣本只累加原因，不入 ring。
    // =============================================================

    static const uint32_t
        QPC_DC_PHASE_RESIDUAL_RING_SIZE =
        16U;

    static int64_t
        qpcDcPhaseResidualRing[
            QPC_DC_PHASE_RESIDUAL_RING_SIZE] =
            {};

            static uint32_t
                qpcDcPhaseResidualRingCount =
                0;

            static uint32_t
                qpcDcPhaseResidualRingWriteIndex =
                0;

            static uint32_t
                qpcDcPhaseResidualBoundInitCount =
                0;

            static bool
                qpcDcPhaseResidualInitialized =
                false;

            static uint32_t
                qpcDcPhaseResidualAcceptedTotal =
                0;

            static uint32_t
                qpcDcPhaseResidualRejectedTotal =
                0;

            static uint32_t
                qpcDcPhaseResidualRejectElapsedTotal =
                0;

            static uint32_t
                qpcDcPhaseResidualRejectRttTotal =
                0;

            static uint32_t
                qpcDcPhaseResidualRejectMagnitudeTotal =
                0;


            const uint64_t
                QPC_DC_PHASE_RESIDUAL_MIN_ELAPSED_NS =
                500000000ULL;

            const uint64_t
                QPC_DC_PHASE_RESIDUAL_MAX_ELAPSED_NS =
                1500000000ULL;

            const uint64_t
                QPC_DC_PHASE_RESIDUAL_MAX_RTT_NS =
                250000ULL;

            const int64_t
                QPC_DC_PHASE_RESIDUAL_MAX_ABS_PPB =
                5000LL;

            const uint32_t
                QPC_DC_PHASE_RESIDUAL_LOCK_MIN_SAMPLES =
                8U;

            const int64_t
                QPC_DC_PHASE_RESIDUAL_LOCK_MAX_MAD_PPB =
                1000LL;


            // =============================================================
            // DC Reference Phase Residual Drift Observer V1A（僅診斷）
            //
            // 每窗至少 3900 個有效 phase sample 才形成 point；保存 16 points，
            // 以最多 120 個 pair slopes 求 Theil-Sen。鎖定另要求至少 7 s 跨度、
            // slope MAD <=750 ppb，避免短時間雜訊被誤當成長期 frequency drift。
            // =============================================================

            static int64_t
                qpcDcPhaseResidualV1AMeanSumNs =
                0;

            static uint32_t
                qpcDcPhaseResidualV1AMeanSampleCount =
                0;


            static const uint32_t
                QPC_DC_PHASE_RESIDUAL_V1A_POINT_COUNT =
                16U;

            static const uint32_t
                QPC_DC_PHASE_RESIDUAL_V1A_MAX_PAIR_COUNT =
                120U;


            static int64_t
                qpcDcPhaseResidualV1APhasePointsNs[
                    QPC_DC_PHASE_RESIDUAL_V1A_POINT_COUNT] =
                    {};

                    static uint64_t
                        qpcDcPhaseResidualV1ATimePointsNs[
                            QPC_DC_PHASE_RESIDUAL_V1A_POINT_COUNT] =
                            {};

                            static uint32_t
                                qpcDcPhaseResidualV1APointCount =
                                0;

                            static uint32_t
                                qpcDcPhaseResidualV1AWriteIndex =
                                0;

                            static bool
                                qpcDcPhaseResidualV1AInitialized =
                                false;

                            static uint32_t
                                qpcDcPhaseResidualV1ABoundInitCount =
                                0;


                            static uint32_t
                                qpcDcPhaseResidualV1AAcceptedPointTotal =
                                0;

                            static uint32_t
                                qpcDcPhaseResidualV1ARejectedPointTotal =
                                0;

                            static uint32_t
                                qpcDcPhaseResidualV1ARejectSamplesTotal =
                                0;

                            static uint32_t
                                qpcDcPhaseResidualV1ARejectElapsedTotal =
                                0;

                            static uint32_t
                                qpcDcPhaseResidualV1ARejectRttTotal =
                                0;

                            static uint32_t
                                qpcDcPhaseResidualV1ARejectSpanTotal =
                                0;

                            static uint64_t qpcV1aCostSumNs = 0;
                            static uint64_t qpcV1aCostMaxNs = 0;
                            static uint32_t qpcV1aCostEvents = 0;
                            static uint32_t qpcV1aCostOver20us = 0;
                            static uint32_t qpcV1aCostOver40us = 0;
                            static uint32_t qpcV1aCostOver80us = 0;
                            static uint32_t qpcV1aCostQpcFail = 0;


                            const uint32_t
                                QPC_DC_PHASE_RESIDUAL_V1A_MIN_MEAN_SAMPLES =
                                3900U;

                            const uint64_t
                                QPC_DC_PHASE_RESIDUAL_V1A_MIN_ELAPSED_NS =
                                500000000ULL;

                            const uint64_t
                                QPC_DC_PHASE_RESIDUAL_V1A_MAX_ELAPSED_NS =
                                1500000000ULL;

                            const uint64_t
                                QPC_DC_PHASE_RESIDUAL_V1A_MAX_RTT_NS =
                                250000ULL;

                            const int64_t
                                QPC_DC_PHASE_RESIDUAL_V1A_MAX_PHASE_SPAN_NS =
                                100000LL;

                            const uint32_t
                                QPC_DC_PHASE_RESIDUAL_V1A_LOCK_MIN_POINTS =
                                8U;

                            const uint64_t
                                QPC_DC_PHASE_RESIDUAL_V1A_LOCK_MIN_SPAN_NS =
                                7000000000ULL;

                            const int64_t
                                QPC_DC_PHASE_RESIDUAL_V1A_LOCK_MAX_SLOPE_MAD_PPB =
                                750LL;


                            const int64_t
                                QPC_LIVE_FF_DC_CYCLE_NS =
                                250000LL;

                            const int64_t
                                QPC_LIVE_FF_SYNC0_PHASE_NS =
                                125000LL;


                            const uint64_t
                                QPC_DC_MAX_RTT_NS =
                                500000ULL;       // 500 us


                            // =============================================================
                            // Robust Drift V1（僅診斷）
                            //
                            // 每個約一秒 raw QPC/DC 視窗只執行一次。固定 9 筆陣列與
                            // insertion sort，不做動態配置；成本相對一秒更新週期可控。
                            // =============================================================

                            static int64_t
                                qpcDcRobustRing[9] =
                            {};

                            static uint32_t
                                qpcDcRobustRingCount =
                                0;

                            static uint32_t
                                qpcDcRobustRingWriteIndex =
                                0;

                            static uint32_t
                                qpcDcRobustAcceptedTotal =
                                0;

                            static uint32_t
                                qpcDcRobustRejectedTotal =
                                0;


                            const uint32_t
                                QPC_DC_ROBUST_RING_SIZE =
                                9U;

                            const uint32_t
                                QPC_DC_ROBUST_MIN_LOCK_SAMPLES =
                                5U;

                            const int64_t
                                QPC_DC_ROBUST_MAX_ABS_DRIFT_PPB =
                                30000LL;

                            const uint64_t
                                QPC_DC_ROBUST_MAX_RTT_AVG_NS =
                                100000ULL;       // 100 us

                            const uint64_t
                                QPC_DC_ROBUST_MAX_RTT_MAX_NS =
                                250000ULL;       // 250 us

                            const int64_t
                                QPC_DC_ROBUST_MAX_MAD_PPB =
                                1500LL;


                            // =============================================================
                            // Trusted Drift V1 - Dry Run
                            //
                            // The calibrated scheduler baseline is used ONLY as the
                            // bootstrap safety reference. Trusted Drift still has no control
                            // authority in this revision.
                            // =============================================================

                            static int64_t
                                qpcDcTrustedDriftPpb =
                                EtherCatDcTuning::SchedulerBootstrapDriftPpb;

                            static bool
                                qpcDcTrustedCalibrationBound =
                                false;

                            static bool
                                qpcDcTrustedValid =
                                false;

                            static LONG
                                qpcDcTrustedState =
                                0;                  // WARMUP

                            static uint32_t
                                qpcDcTrustedWarmupGoodCount =
                                0;

                            static uint32_t
                                qpcDcTrustedBadCount =
                                0;

                            static uint32_t
                                qpcDcTrustedRecoveryGoodCount =
                                0;

                            static uint32_t
                                qpcDcTrustedUpdateTotal =
                                0;

                            static uint32_t
                                qpcDcTrustedHoldTotal =
                                0;

                            static uint32_t
                                qpcDcTrustedUnlockTotal =
                                0;

                            static uint32_t
                                qpcDcTrustedRelockTotal =
                                0;


                            // =============================================================
                            // Trusted Drift V1A - Reject / Transition Reason Diagnostic
                            //
                            // Bookkeeping only. These counters do not participate in any
                            // Trusted Drift decision.
                            // =============================================================

                            static uint32_t
                                qpcDcTrustedRejectRobustAcceptTotal =
                                0;

                            static uint32_t
                                qpcDcTrustedRejectRobustLockTotal =
                                0;

                            static uint32_t
                                qpcDcTrustedRejectBufferNotFullTotal =
                                0;

                            static uint32_t
                                qpcDcTrustedRejectMadTotal =
                                0;

                            static uint32_t
                                qpcDcTrustedRejectCandidateDeviationTotal =
                                0;

                            static uint32_t
                                qpcDcTrustedRejectRawMedianDeviationTotal =
                                0;

                            static LONG
                                qpcDcTrustedLastRejectMask =
                                0;

                            static uint32_t
                                qpcDcTrustedMaxBadStreak =
                                0;

                            static uint32_t
                                qpcDcTrustedWarmupToTrackTotal =
                                0;

                            static uint32_t
                                qpcDcTrustedTrackToHoldTotal =
                                0;

                            static uint32_t
                                qpcDcTrustedHoldToTrackTotal =
                                0;

                            static uint32_t
                                qpcDcTrustedHoldToUntrustedTotal =
                                0;

                            static uint32_t
                                qpcDcTrustedUntrustedToTrackTotal =
                                0;

                            static LONG
                                qpcDcTrustedLastTransitionFrom =
                                -1;

                            static LONG
                                qpcDcTrustedLastTransitionTo =
                                -1;

                            static int64_t
                                qpcDcTrustedLastTransitionRawPpb =
                                0;

                            static int64_t
                                qpcDcTrustedLastTransitionMedianPpb =
                                0;

                            static int64_t
                                qpcDcTrustedLastTransitionMadPpb =
                                0;

                            static int64_t
                                qpcDcTrustedLastTransitionTrustedPpb =
                                EtherCatDcTuning::SchedulerBootstrapDriftPpb;

                            const uint32_t
                                QPC_DC_TRUSTED_WARMUP_GOOD_WINDOWS =
                                8U;

                            const uint32_t
                                QPC_DC_TRUSTED_HOLD_MAX_BAD_WINDOWS =
                                10U;

                            const uint32_t
                                QPC_DC_TRUSTED_HOLD_RECOVERY_WINDOWS =
                                3U;

                            const uint32_t
                                QPC_DC_TRUSTED_RELOCK_GOOD_WINDOWS =
                                5U;

                            const int64_t
                                QPC_DC_TRUSTED_MAX_BASELINE_DEVIATION_PPB =
                                1200LL;

                            const int64_t
                                QPC_DC_TRUSTED_MAX_RAW_MEDIAN_DEVIATION_PPB =
                                2000LL;

                            const int64_t
                                QPC_DC_TRUSTED_MAX_MAD_PPB =
                                800LL;

                            const int64_t
                                QPC_DC_TRUSTED_MAX_SLEW_PPB_PER_WINDOW =
                                50LL;



                            // DC-RX.1: discard only partial windows that cross
                            // a PDO/DC or scheduler interruption. Completed
                            // robust/V1A history and phase unwrap continuity are
                            // retained, so recovery does not require rebuilding
                            // the entire 16-point observer from zero.
                            if (dcObserversResetThisCycle)
                            {
                                qpcDcWindowStartMidCount = 0;
                                qpcDcWindowEndMidCount = 0;
                                qpcDcWindowStartDcNs = 0;
                                qpcDcWindowEndDcNs = 0;
                                qpcDcRttSumNs = 0;
                                qpcDcRttMinNs = 0;
                                qpcDcRttMaxNs = 0;
                                qpcDcWindowValidSamples = 0;
                                qpcDcWindowRejectedSamples = 0;

                                qpcLiveFfDcPhaseWindowSamples = 0;
                                qpcLiveFfDcPhaseFixedWindowStartNs = 0;
                                qpcLiveFfDcPhaseFixedWindowEndNs = 0;
                                qpcLiveFfDcPhaseFixedWindowMinNs = 0;
                                qpcLiveFfDcPhaseFixedWindowMaxNs = 0;
                                qpcLiveFfDcPhaseShadowWindowStartNs = 0;
                                qpcLiveFfDcPhaseShadowWindowEndNs = 0;
                                qpcLiveFfDcPhaseShadowWindowMinNs = 0;
                                qpcLiveFfDcPhaseShadowWindowMaxNs = 0;
                                qpcLiveFfDcPhaseFixedAbsSumNs = 0;
                                qpcLiveFfDcPhaseShadowAbsSumNs = 0;
                                qpcLiveFfDcPhaseShadowBetterCount = 0;
                                qpcLiveFfDcPhaseShadowWorseCount = 0;
                                qpcLiveFfDcPhaseEqualCount = 0;
                                qpcLiveFfDcPhaseMapRttSumNs = 0;
                                qpcLiveFfDcPhaseMapRttMinNs = 0;
                                qpcLiveFfDcPhaseMapRttMaxNs = 0;
                                qpcLiveFfDcPhaseWindowStartDcNs = 0;
                                qpcLiveFfDcPhaseWindowEndDcNs = 0;

                                qpcPhaseFfV2DcWindowSamples = 0;
                                qpcPhaseFfV2DcWindowStartNs = 0;
                                qpcPhaseFfV2DcWindowEndNs = 0;
                                qpcPhaseFfV2DcWindowMinNs = 0;
                                qpcPhaseFfV2DcWindowMaxNs = 0;
                                qpcPhaseFfV2DcAbsSumNs = 0;
                                qpcPhaseFfV2DcBetterThanFixed = 0;
                                qpcPhaseFfV2DcBetterThanTrusted = 0;

                                qpcDcPhaseResidualV1AMeanSumNs = 0;
                                qpcDcPhaseResidualV1AMeanSampleCount = 0;
                            }

                            // DC-RX.3C: only a chronologically qualified sample
                            // may enter any DC observer. Transport WKC alone is not enough.
                            const bool qpcDcSampleValid =
                                dcReferenceSlaveIndex >= 0 &&
                                dcSampleFreshnessAccepted &&
                                qpcDcRttNs > 0 &&
                                qpcDcRttNs <= QPC_DC_MAX_RTT_NS;


                            if (dcPhaseMapRebindPending &&
                                qpcDcSampleValid)
                            {
                                qpcLiveFfDcPhaseInitialized = false;
                                qpcLiveFfDcPhaseFixedUnwrapValid = false;
                                qpcLiveFfDcPhaseShadowUnwrapValid = false;
                                qpcLiveFfDcPhaseFixedUnwrappedNs = 0;
                                qpcLiveFfDcPhaseShadowUnwrappedNs = 0;
                                qpcLiveFfDcPhaseWindowSamples = 0;
                                qpcLiveFfDcPhaseFixedAbsSumNs = 0;
                                qpcLiveFfDcPhaseShadowAbsSumNs = 0;
                                qpcLiveFfDcPhaseShadowBetterCount = 0;
                                qpcLiveFfDcPhaseShadowWorseCount = 0;
                                qpcLiveFfDcPhaseEqualCount = 0;
                                qpcLiveFfDcPhaseMapRttSumNs = 0;
                                qpcLiveFfDcPhaseMapRttMinNs = 0;
                                qpcLiveFfDcPhaseMapRttMaxNs = 0;
                                qpcDcPhaseResidualV1AMeanSumNs = 0;
                                qpcDcPhaseResidualV1AMeanSampleCount = 0;
                                qpcPhaseFfV2DcWindowSamples = 0;
                                qpcPhaseFfV2DcAbsSumNs = 0;
                                dcPhaseMapRebindPending = false;
                            }

                            // =============================================================
                            // Accumulate valid sample
                            // =============================================================

                            if (qpcDcSampleValid)
                            {
                                // =========================================================
                                // Trusted Live-FF DC Phase Predictor Dry Run V1
                                //
                                // Map BOTH scheduler targets into the SAME current DC Reference
                                // coordinate system.
                                //
                                // TargetDc ~= DC_reference_time
                                //            + (TargetQpc - QpcMidpoint) converted to ns
                                //
                                // The target is only ~hundreds of us away from this sample,
                                // so QPC-vs-DC frequency error contributes sub-ns mapping
                                // error over this local interval. The dominant fixed capture
                                // bias is common-mode and is removed by baseline phase.
                                // =========================================================

                                bool qpcLiveFfDcPhaseMapValid =
                                    (
                                        qpcSchedulerInitialized &&
                                        qpcLiveFfInitialized &&
                                        qpcLiveFfInitCount > 0 &&
                                        qpcFrequency > 0 &&
                                        qpcFixedRefInitialized &&
                                        qpcFixedRefTargetQpc > 0 &&
                                        qpcLiveFfTargetQpc > 0
                                        );


                                // If the shadow scheduler has been re-initialized, bind the
                                // phase predictor to the new shadow timeline.
                                if (qpcLiveFfDcPhaseInitialized &&
                                    qpcLiveFfDcPhaseBoundInitCount !=
                                    qpcLiveFfInitCount)
                                {
                                    qpcLiveFfDcPhaseInitialized =
                                        false;

                                    qpcLiveFfDcPhaseFixedUnwrapValid =
                                        false;

                                    qpcLiveFfDcPhaseShadowUnwrapValid =
                                        false;

                                    qpcLiveFfDcPhaseWindowSamples =
                                        0;

                                    qpcLiveFfDcPhaseFixedAbsSumNs =
                                        0;

                                    qpcLiveFfDcPhaseShadowAbsSumNs =
                                        0;

                                    qpcLiveFfDcPhaseShadowBetterCount =
                                        0;

                                    qpcLiveFfDcPhaseShadowWorseCount =
                                        0;

                                    qpcLiveFfDcPhaseEqualCount =
                                        0;

                                    qpcLiveFfDcPhaseMapRttSumNs =
                                        0;

                                    qpcLiveFfDcPhaseMapRttMinNs =
                                        0;

                                    qpcLiveFfDcPhaseMapRttMaxNs =
                                        0;
                                }


                                if (qpcLiveFfDcPhaseMapValid)
                                {
                                    int64_t fixedTargetDeltaCounts =
                                        (int64_t)
                                        qpcFixedRefTargetQpc -
                                        (int64_t)
                                        qpcDcMidCount;


                                    int64_t shadowTargetDeltaCounts =
                                        (int64_t)
                                        qpcLiveFfTargetQpc -
                                        (int64_t)
                                        qpcDcMidCount;


                                    int64_t fixedTargetDeltaNs =
                                        0;

                                    int64_t shadowTargetDeltaNs =
                                        0;


                                    if (fixedTargetDeltaCounts >= 0)
                                    {
                                        fixedTargetDeltaNs =
                                            (int64_t)
                                            (
                                                (
                                                    (uint64_t)
                                                    fixedTargetDeltaCounts *
                                                    1000000000ULL
                                                    )
                                                /
                                                qpcFrequency
                                                );
                                    }
                                    else
                                    {
                                        uint64_t fixedMagnitudeCounts =
                                            (uint64_t)
                                            (
                                                -fixedTargetDeltaCounts
                                                );


                                        fixedTargetDeltaNs =
                                            -
                                            (int64_t)
                                            (
                                                (
                                                    fixedMagnitudeCounts *
                                                    1000000000ULL
                                                    )
                                                /
                                                qpcFrequency
                                                );
                                    }


                                    if (shadowTargetDeltaCounts >= 0)
                                    {
                                        shadowTargetDeltaNs =
                                            (int64_t)
                                            (
                                                (
                                                    (uint64_t)
                                                    shadowTargetDeltaCounts *
                                                    1000000000ULL
                                                    )
                                                /
                                                qpcFrequency
                                                );
                                    }
                                    else
                                    {
                                        uint64_t shadowMagnitudeCounts =
                                            (uint64_t)
                                            (
                                                -shadowTargetDeltaCounts
                                                );


                                        shadowTargetDeltaNs =
                                            -
                                            (int64_t)
                                            (
                                                (
                                                    shadowMagnitudeCounts *
                                                    1000000000ULL
                                                    )
                                                /
                                                qpcFrequency
                                                );
                                    }


                                    int64_t referenceDcNs =
                                        (int64_t)
                                        pMaster->DC_reference_time;


                                    int64_t fixedTargetDcNs =
                                        referenceDcNs +
                                        fixedTargetDeltaNs;


                                    int64_t shadowTargetDcNs =
                                        referenceDcNs +
                                        shadowTargetDeltaNs;


                                    int64_t fixedTargetPhaseNs =
                                        fixedTargetDcNs %
                                        QPC_LIVE_FF_DC_CYCLE_NS;


                                    int64_t shadowTargetPhaseNs =
                                        shadowTargetDcNs %
                                        QPC_LIVE_FF_DC_CYCLE_NS;


                                    if (fixedTargetPhaseNs < 0)
                                    {
                                        fixedTargetPhaseNs +=
                                            QPC_LIVE_FF_DC_CYCLE_NS;
                                    }


                                    if (shadowTargetPhaseNs < 0)
                                    {
                                        shadowTargetPhaseNs +=
                                            QPC_LIVE_FF_DC_CYCLE_NS;
                                    }


                                    int64_t fixedSync0MarginNs =
                                        QPC_LIVE_FF_SYNC0_PHASE_NS -
                                        fixedTargetPhaseNs;


                                    fixedSync0MarginNs %=
                                        QPC_LIVE_FF_DC_CYCLE_NS;


                                    if (fixedSync0MarginNs < 0)
                                    {
                                        fixedSync0MarginNs +=
                                            QPC_LIVE_FF_DC_CYCLE_NS;
                                    }


                                    int64_t shadowSync0MarginNs =
                                        QPC_LIVE_FF_SYNC0_PHASE_NS -
                                        shadowTargetPhaseNs;


                                    shadowSync0MarginNs %=
                                        QPC_LIVE_FF_DC_CYCLE_NS;


                                    if (shadowSync0MarginNs < 0)
                                    {
                                        shadowSync0MarginNs +=
                                            QPC_LIVE_FF_DC_CYCLE_NS;
                                    }


                                    if (!qpcLiveFfDcPhaseInitialized)
                                    {
                                        // ---------------------------------------------
                                        // Baseline is the CURRENT fixed scheduler DC
                                        // phase. This removes deterministic capture bias.
                                        // ---------------------------------------------

                                        qpcLiveFfDcPhaseBaselinePhaseNs =
                                            fixedTargetPhaseNs;

                                        qpcLiveFfDcPhaseBaselineSync0MarginNs =
                                            fixedSync0MarginNs;

                                        qpcLiveFfDcPhaseBoundInitCount =
                                            qpcLiveFfInitCount;

                                        qpcLiveFfDcPhaseFixedUnwrapValid =
                                            false;

                                        qpcLiveFfDcPhaseShadowUnwrapValid =
                                            false;

                                        qpcLiveFfDcPhaseFixedUnwrappedNs =
                                            0;

                                        qpcLiveFfDcPhaseShadowUnwrappedNs =
                                            0;

                                        qpcLiveFfDcPhaseWindowSamples =
                                            0;

                                        qpcLiveFfDcPhaseFixedAbsSumNs =
                                            0;

                                        qpcLiveFfDcPhaseShadowAbsSumNs =
                                            0;

                                        qpcLiveFfDcPhaseShadowBetterCount =
                                            0;

                                        qpcLiveFfDcPhaseShadowWorseCount =
                                            0;

                                        qpcLiveFfDcPhaseEqualCount =
                                            0;

                                        qpcLiveFfDcPhaseMapRttSumNs =
                                            0;

                                        qpcLiveFfDcPhaseMapRttMinNs =
                                            0;

                                        qpcLiveFfDcPhaseMapRttMaxNs =
                                            0;

                                        qpcLiveFfDcPhaseInitialized =
                                            true;
                                    }


                                    int64_t fixedWrappedErrorNs =
                                        fixedTargetPhaseNs -
                                        qpcLiveFfDcPhaseBaselinePhaseNs;


                                    int64_t shadowWrappedErrorNs =
                                        shadowTargetPhaseNs -
                                        qpcLiveFfDcPhaseBaselinePhaseNs;


                                    // Wrap both to [-125 us, +125 us).
                                    while (fixedWrappedErrorNs >=
                                        QPC_LIVE_FF_DC_CYCLE_NS / 2LL)
                                    {
                                        fixedWrappedErrorNs -=
                                            QPC_LIVE_FF_DC_CYCLE_NS;
                                    }


                                    while (fixedWrappedErrorNs <
                                        -QPC_LIVE_FF_DC_CYCLE_NS / 2LL)
                                    {
                                        fixedWrappedErrorNs +=
                                            QPC_LIVE_FF_DC_CYCLE_NS;
                                    }


                                    while (shadowWrappedErrorNs >=
                                        QPC_LIVE_FF_DC_CYCLE_NS / 2LL)
                                    {
                                        shadowWrappedErrorNs -=
                                            QPC_LIVE_FF_DC_CYCLE_NS;
                                    }


                                    while (shadowWrappedErrorNs <
                                        -QPC_LIVE_FF_DC_CYCLE_NS / 2LL)
                                    {
                                        shadowWrappedErrorNs +=
                                            QPC_LIVE_FF_DC_CYCLE_NS;
                                    }


                                    // -------------------------------------------------
                                    // Unwrap Fixed error.
                                    // -------------------------------------------------

                                    if (!qpcLiveFfDcPhaseFixedUnwrapValid)
                                    {
                                        qpcLiveFfDcPhaseFixedUnwrappedNs =
                                            fixedWrappedErrorNs;

                                        qpcLiveFfDcPhaseFixedUnwrapValid =
                                            true;
                                    }
                                    else
                                    {
                                        int64_t fixedStepNs =
                                            fixedWrappedErrorNs -
                                            qpcLiveFfDcPhasePrevFixedWrappedNs;


                                        if (fixedStepNs >=
                                            QPC_LIVE_FF_DC_CYCLE_NS / 2LL)
                                        {
                                            fixedStepNs -=
                                                QPC_LIVE_FF_DC_CYCLE_NS;
                                        }
                                        else if (fixedStepNs <
                                            -QPC_LIVE_FF_DC_CYCLE_NS / 2LL)
                                        {
                                            fixedStepNs +=
                                                QPC_LIVE_FF_DC_CYCLE_NS;
                                        }


                                        qpcLiveFfDcPhaseFixedUnwrappedNs +=
                                            fixedStepNs;
                                    }


                                    // -------------------------------------------------
                                    // Unwrap Shadow error.
                                    // -------------------------------------------------

                                    if (!qpcLiveFfDcPhaseShadowUnwrapValid)
                                    {
                                        qpcLiveFfDcPhaseShadowUnwrappedNs =
                                            shadowWrappedErrorNs;

                                        qpcLiveFfDcPhaseShadowUnwrapValid =
                                            true;
                                    }
                                    else
                                    {
                                        int64_t shadowStepNs =
                                            shadowWrappedErrorNs -
                                            qpcLiveFfDcPhasePrevShadowWrappedNs;


                                        if (shadowStepNs >=
                                            QPC_LIVE_FF_DC_CYCLE_NS / 2LL)
                                        {
                                            shadowStepNs -=
                                                QPC_LIVE_FF_DC_CYCLE_NS;
                                        }
                                        else if (shadowStepNs <
                                            -QPC_LIVE_FF_DC_CYCLE_NS / 2LL)
                                        {
                                            shadowStepNs +=
                                                QPC_LIVE_FF_DC_CYCLE_NS;
                                        }


                                        qpcLiveFfDcPhaseShadowUnwrappedNs +=
                                            shadowStepNs;
                                    }


                                    qpcLiveFfDcPhasePrevFixedWrappedNs =
                                        fixedWrappedErrorNs;

                                    qpcLiveFfDcPhasePrevShadowWrappedNs =
                                        shadowWrappedErrorNs;


                                    int64_t shadowVsFixedPhaseNs =
                                        qpcLiveFfDcPhaseShadowUnwrappedNs -
                                        qpcLiveFfDcPhaseFixedUnwrappedNs;


                                    uint64_t fixedAbsErrorNs =
                                        (uint64_t)
                                        (
                                            qpcLiveFfDcPhaseFixedUnwrappedNs >= 0
                                            ? qpcLiveFfDcPhaseFixedUnwrappedNs
                                            : -qpcLiveFfDcPhaseFixedUnwrappedNs
                                            );


                                    uint64_t shadowAbsErrorNs =
                                        (uint64_t)
                                        (
                                            qpcLiveFfDcPhaseShadowUnwrappedNs >= 0
                                            ? qpcLiveFfDcPhaseShadowUnwrappedNs
                                            : -qpcLiveFfDcPhaseShadowUnwrappedNs
                                            );


                                    if (qpcLiveFfDcPhaseWindowSamples == 0)
                                    {
                                        // New ~1 second phase window for V1A mean point.
                                        qpcDcPhaseResidualV1AMeanSumNs =
                                            0;

                                        qpcDcPhaseResidualV1AMeanSampleCount =
                                            0;


                                        qpcLiveFfDcPhaseWindowStartDcNs =
                                            (uint64_t)
                                            pMaster->DC_reference_time;

                                        qpcLiveFfDcPhaseWindowEndDcNs =
                                            qpcLiveFfDcPhaseWindowStartDcNs;


                                        qpcLiveFfDcPhaseFixedWindowStartNs =
                                            qpcLiveFfDcPhaseFixedUnwrappedNs;

                                        qpcLiveFfDcPhaseFixedWindowMinNs =
                                            qpcLiveFfDcPhaseFixedUnwrappedNs;

                                        qpcLiveFfDcPhaseFixedWindowMaxNs =
                                            qpcLiveFfDcPhaseFixedUnwrappedNs;


                                        qpcLiveFfDcPhaseShadowWindowStartNs =
                                            qpcLiveFfDcPhaseShadowUnwrappedNs;

                                        qpcLiveFfDcPhaseShadowWindowMinNs =
                                            qpcLiveFfDcPhaseShadowUnwrappedNs;

                                        qpcLiveFfDcPhaseShadowWindowMaxNs =
                                            qpcLiveFfDcPhaseShadowUnwrappedNs;


                                        qpcLiveFfDcPhaseMapRttMinNs =
                                            qpcDcRttNs;

                                        qpcLiveFfDcPhaseMapRttMaxNs =
                                            qpcDcRttNs;
                                    }


                                    qpcLiveFfDcPhaseFixedWindowEndNs =
                                        qpcLiveFfDcPhaseFixedUnwrappedNs;

                                    qpcLiveFfDcPhaseShadowWindowEndNs =
                                        qpcLiveFfDcPhaseShadowUnwrappedNs;


                                    qpcLiveFfDcPhaseWindowEndDcNs =
                                        (uint64_t)
                                        pMaster->DC_reference_time;


                                    if (qpcLiveFfDcPhaseFixedUnwrappedNs <
                                        qpcLiveFfDcPhaseFixedWindowMinNs)
                                    {
                                        qpcLiveFfDcPhaseFixedWindowMinNs =
                                            qpcLiveFfDcPhaseFixedUnwrappedNs;
                                    }


                                    if (qpcLiveFfDcPhaseFixedUnwrappedNs >
                                        qpcLiveFfDcPhaseFixedWindowMaxNs)
                                    {
                                        qpcLiveFfDcPhaseFixedWindowMaxNs =
                                            qpcLiveFfDcPhaseFixedUnwrappedNs;
                                    }


                                    if (qpcLiveFfDcPhaseShadowUnwrappedNs <
                                        qpcLiveFfDcPhaseShadowWindowMinNs)
                                    {
                                        qpcLiveFfDcPhaseShadowWindowMinNs =
                                            qpcLiveFfDcPhaseShadowUnwrappedNs;
                                    }


                                    if (qpcLiveFfDcPhaseShadowUnwrappedNs >
                                        qpcLiveFfDcPhaseShadowWindowMaxNs)
                                    {
                                        qpcLiveFfDcPhaseShadowWindowMaxNs =
                                            qpcLiveFfDcPhaseShadowUnwrappedNs;
                                    }


                                    if (qpcDcRttNs <
                                        qpcLiveFfDcPhaseMapRttMinNs)
                                    {
                                        qpcLiveFfDcPhaseMapRttMinNs =
                                            qpcDcRttNs;
                                    }


                                    if (qpcDcRttNs >
                                        qpcLiveFfDcPhaseMapRttMaxNs)
                                    {
                                        qpcLiveFfDcPhaseMapRttMaxNs =
                                            qpcDcRttNs;
                                    }


                                    qpcLiveFfDcPhaseFixedAbsSumNs +=
                                        fixedAbsErrorNs;

                                    qpcLiveFfDcPhaseShadowAbsSumNs +=
                                        shadowAbsErrorNs;

                                    qpcLiveFfDcPhaseMapRttSumNs +=
                                        qpcDcRttNs;


                                    if (shadowAbsErrorNs <
                                        fixedAbsErrorNs)
                                    {
                                        qpcLiveFfDcPhaseShadowBetterCount++;
                                    }
                                    else if (shadowAbsErrorNs >
                                        fixedAbsErrorNs)
                                    {
                                        qpcLiveFfDcPhaseShadowWorseCount++;
                                    }
                                    else
                                    {
                                        qpcLiveFfDcPhaseEqualCount++;
                                    }


                                    // -------------------------------------------------
                                    // V1A: use the mean of all phase samples in this
                                    // window instead of noisy first/last endpoints.
                                    // -------------------------------------------------

                                    qpcDcPhaseResidualV1AMeanSumNs +=
                                        qpcLiveFfDcPhaseFixedUnwrappedNs;

                                    qpcDcPhaseResidualV1AMeanSampleCount++;


                                    // =====================================================
                                    // Frequency FF Dry Run V2 - third target DC projection
                                    // =====================================================

                                    if (qpcPhaseFfV2Initialized &&
                                        qpcPhaseFfV2TargetQpc > 0 &&
                                        qpcFrequency > 0)
                                    {
                                        int64_t phaseFfDcDeltaCounts =
                                            (int64_t)
                                            qpcPhaseFfV2TargetQpc -
                                            (int64_t)
                                            qpcDcMidCount;


                                        int64_t phaseFfDcDeltaNs =
                                            0;


                                        if (phaseFfDcDeltaCounts >= 0)
                                        {
                                            phaseFfDcDeltaNs =
                                                (int64_t)
                                                (
                                                    (
                                                        (uint64_t)
                                                        phaseFfDcDeltaCounts *
                                                        1000000000ULL
                                                        )
                                                    /
                                                    qpcFrequency
                                                    );
                                        }
                                        else
                                        {
                                            uint64_t magnitudeCounts =
                                                (uint64_t)
                                                (
                                                    -phaseFfDcDeltaCounts
                                                    );


                                            phaseFfDcDeltaNs =
                                                -
                                                (int64_t)
                                                (
                                                    (
                                                        magnitudeCounts *
                                                        1000000000ULL
                                                        )
                                                    /
                                                    qpcFrequency
                                                    );
                                        }


                                        int64_t phaseFfTargetDcNs =
                                            referenceDcNs +
                                            phaseFfDcDeltaNs;


                                        int64_t phaseFfPhaseNs =
                                            phaseFfTargetDcNs %
                                            QPC_LIVE_FF_DC_CYCLE_NS;


                                        if (phaseFfPhaseNs < 0)
                                        {
                                            phaseFfPhaseNs +=
                                                QPC_LIVE_FF_DC_CYCLE_NS;
                                        }


                                        int64_t phaseFfSync0MarginNs =
                                            QPC_LIVE_FF_SYNC0_PHASE_NS -
                                            phaseFfPhaseNs;


                                        phaseFfSync0MarginNs %=
                                            QPC_LIVE_FF_DC_CYCLE_NS;


                                        if (phaseFfSync0MarginNs < 0)
                                        {
                                            phaseFfSync0MarginNs +=
                                                QPC_LIVE_FF_DC_CYCLE_NS;
                                        }


                                        int64_t phaseFfWrappedErrorNs =
                                            phaseFfPhaseNs -
                                            qpcLiveFfDcPhaseBaselinePhaseNs;


                                        while (phaseFfWrappedErrorNs >=
                                            QPC_LIVE_FF_DC_CYCLE_NS / 2LL)
                                        {
                                            phaseFfWrappedErrorNs -=
                                                QPC_LIVE_FF_DC_CYCLE_NS;
                                        }


                                        while (phaseFfWrappedErrorNs <
                                            -QPC_LIVE_FF_DC_CYCLE_NS / 2LL)
                                        {
                                            phaseFfWrappedErrorNs +=
                                                QPC_LIVE_FF_DC_CYCLE_NS;
                                        }


                                        if (!qpcPhaseFfV2DcInitialized)
                                        {
                                            qpcPhaseFfV2DcInitialized =
                                                true;

                                            qpcPhaseFfV2DcUnwrapValid =
                                                false;

                                            qpcPhaseFfV2DcUnwrappedNs =
                                                0;

                                            qpcPhaseFfV2DcWindowSamples =
                                                0;

                                            qpcPhaseFfV2DcAbsSumNs =
                                                0;

                                            qpcPhaseFfV2DcBetterThanFixed =
                                                0;

                                            qpcPhaseFfV2DcBetterThanTrusted =
                                                0;
                                        }


                                        if (!qpcPhaseFfV2DcUnwrapValid)
                                        {
                                            qpcPhaseFfV2DcUnwrappedNs =
                                                phaseFfWrappedErrorNs;

                                            qpcPhaseFfV2DcUnwrapValid =
                                                true;
                                        }
                                        else
                                        {
                                            int64_t phaseFfStepNs =
                                                phaseFfWrappedErrorNs -
                                                qpcPhaseFfV2DcPrevWrappedNs;


                                            if (phaseFfStepNs >=
                                                QPC_LIVE_FF_DC_CYCLE_NS / 2LL)
                                            {
                                                phaseFfStepNs -=
                                                    QPC_LIVE_FF_DC_CYCLE_NS;
                                            }
                                            else if (phaseFfStepNs <
                                                -QPC_LIVE_FF_DC_CYCLE_NS / 2LL)
                                            {
                                                phaseFfStepNs +=
                                                    QPC_LIVE_FF_DC_CYCLE_NS;
                                            }


                                            qpcPhaseFfV2DcUnwrappedNs +=
                                                phaseFfStepNs;
                                        }


                                        qpcPhaseFfV2DcPrevWrappedNs =
                                            phaseFfWrappedErrorNs;


                                        int64_t phaseFfDcVsFixedNs =
                                            qpcPhaseFfV2DcUnwrappedNs -
                                            qpcLiveFfDcPhaseFixedUnwrappedNs;


                                        int64_t phaseFfDcVsTrustedNs =
                                            qpcPhaseFfV2DcUnwrappedNs -
                                            qpcLiveFfDcPhaseShadowUnwrappedNs;


                                        uint64_t phaseFfAbsNs =
                                            (uint64_t)
                                            (
                                                qpcPhaseFfV2DcUnwrappedNs >= 0
                                                ? qpcPhaseFfV2DcUnwrappedNs
                                                : -qpcPhaseFfV2DcUnwrappedNs
                                                );


                                        uint64_t fixedAbsNsForCompare =
                                            (uint64_t)
                                            (
                                                qpcLiveFfDcPhaseFixedUnwrappedNs >= 0
                                                ? qpcLiveFfDcPhaseFixedUnwrappedNs
                                                : -qpcLiveFfDcPhaseFixedUnwrappedNs
                                                );


                                        uint64_t trustedAbsNsForCompare =
                                            (uint64_t)
                                            (
                                                qpcLiveFfDcPhaseShadowUnwrappedNs >= 0
                                                ? qpcLiveFfDcPhaseShadowUnwrappedNs
                                                : -qpcLiveFfDcPhaseShadowUnwrappedNs
                                                );


                                        if (qpcPhaseFfV2DcWindowSamples == 0)
                                        {
                                            qpcPhaseFfV2DcWindowStartNs =
                                                qpcPhaseFfV2DcUnwrappedNs;

                                            qpcPhaseFfV2DcWindowMinNs =
                                                qpcPhaseFfV2DcUnwrappedNs;

                                            qpcPhaseFfV2DcWindowMaxNs =
                                                qpcPhaseFfV2DcUnwrappedNs;
                                        }


                                        qpcPhaseFfV2DcWindowEndNs =
                                            qpcPhaseFfV2DcUnwrappedNs;


                                        if (qpcPhaseFfV2DcUnwrappedNs <
                                            qpcPhaseFfV2DcWindowMinNs)
                                        {
                                            qpcPhaseFfV2DcWindowMinNs =
                                                qpcPhaseFfV2DcUnwrappedNs;
                                        }


                                        if (qpcPhaseFfV2DcUnwrappedNs >
                                            qpcPhaseFfV2DcWindowMaxNs)
                                        {
                                            qpcPhaseFfV2DcWindowMaxNs =
                                                qpcPhaseFfV2DcUnwrappedNs;
                                        }


                                        qpcPhaseFfV2DcAbsSumNs +=
                                            phaseFfAbsNs;


                                        if (phaseFfAbsNs <
                                            fixedAbsNsForCompare)
                                        {
                                            qpcPhaseFfV2DcBetterThanFixed++;
                                        }


                                        if (phaseFfAbsNs <
                                            trustedAbsNsForCompare)
                                        {
                                            qpcPhaseFfV2DcBetterThanTrusted++;
                                        }


                                        qpcPhaseFfV2DcWindowSamples++;


                                        if (qpcPhaseFfV2DcWindowSamples >=
                                            4000U)
                                        {
                                            int64_t phaseFfDcWindowDeltaNs =
                                                qpcPhaseFfV2DcWindowEndNs -
                                                qpcPhaseFfV2DcWindowStartNs;


                                            uint64_t phaseFfDcAbsAvgNs =
                                                qpcPhaseFfV2DcAbsSumNs /
                                                qpcPhaseFfV2DcWindowSamples;


                                            InterlockedIncrement(
                                                &g_qpcPhaseFfV2DcDiagSequence);


                                            g_qpcPhaseFfV2DcInitialized =
                                                qpcPhaseFfV2DcInitialized
                                                ? 1L
                                                : 0L;

                                            g_qpcPhaseFfV2DcPhaseNs =
                                                (LONGLONG)
                                                phaseFfPhaseNs;

                                            g_qpcPhaseFfV2DcSync0MarginNs =
                                                (LONGLONG)
                                                phaseFfSync0MarginNs;

                                            g_qpcPhaseFfV2DcUnwrappedErrorNs =
                                                (LONGLONG)
                                                qpcPhaseFfV2DcUnwrappedNs;

                                            g_qpcPhaseFfV2DcVsFixedNs =
                                                (LONGLONG)
                                                phaseFfDcVsFixedNs;

                                            g_qpcPhaseFfV2DcVsTrustedNs =
                                                (LONGLONG)
                                                phaseFfDcVsTrustedNs;

                                            g_qpcPhaseFfV2DcWindowStartNs =
                                                (LONGLONG)
                                                qpcPhaseFfV2DcWindowStartNs;

                                            g_qpcPhaseFfV2DcWindowEndNs =
                                                (LONGLONG)
                                                qpcPhaseFfV2DcWindowEndNs;

                                            g_qpcPhaseFfV2DcWindowDeltaNs =
                                                (LONGLONG)
                                                phaseFfDcWindowDeltaNs;

                                            g_qpcPhaseFfV2DcWindowMinNs =
                                                (LONGLONG)
                                                qpcPhaseFfV2DcWindowMinNs;

                                            g_qpcPhaseFfV2DcWindowMaxNs =
                                                (LONGLONG)
                                                qpcPhaseFfV2DcWindowMaxNs;

                                            g_qpcPhaseFfV2DcAbsAvgNs =
                                                (LONGLONG)
                                                phaseFfDcAbsAvgNs;

                                            g_qpcPhaseFfV2DcBetterThanFixed =
                                                (LONG)
                                                qpcPhaseFfV2DcBetterThanFixed;

                                            g_qpcPhaseFfV2DcBetterThanTrusted =
                                                (LONG)
                                                qpcPhaseFfV2DcBetterThanTrusted;

                                            g_qpcPhaseFfV2DcSamples =
                                                (LONG)
                                                qpcPhaseFfV2DcWindowSamples;


                                            MemoryBarrier();


                                            InterlockedIncrement(
                                                &g_qpcPhaseFfV2DcDiagSequence);


                                            qpcPhaseFfV2DcWindowSamples =
                                                0;

                                            qpcPhaseFfV2DcAbsSumNs =
                                                0;

                                            qpcPhaseFfV2DcBetterThanFixed =
                                                0;

                                            qpcPhaseFfV2DcBetterThanTrusted =
                                                0;
                                        }
                                    }


                                    qpcLiveFfDcPhaseWindowSamples++;


                                    if (qpcLiveFfDcPhaseWindowSamples >=
                                        4000U)
                                    {
                                        int64_t fixedWindowDeltaNs =
                                            qpcLiveFfDcPhaseFixedWindowEndNs -
                                            qpcLiveFfDcPhaseFixedWindowStartNs;


                                        int64_t shadowWindowDeltaNs =
                                            qpcLiveFfDcPhaseShadowWindowEndNs -
                                            qpcLiveFfDcPhaseShadowWindowStartNs;


                                        uint64_t fixedAbsAvgNs =
                                            qpcLiveFfDcPhaseFixedAbsSumNs /
                                            qpcLiveFfDcPhaseWindowSamples;


                                        uint64_t shadowAbsAvgNs =
                                            qpcLiveFfDcPhaseShadowAbsSumNs /
                                            qpcLiveFfDcPhaseWindowSamples;


                                        uint64_t mapRttAvgNs =
                                            qpcLiveFfDcPhaseMapRttSumNs /
                                            qpcLiveFfDcPhaseWindowSamples;


                                        // =====================================================
                                        // DC Reference Phase Residual Drift Observer V1
                                        //
                                        // Frequency error is inferred from the slope of the
                                        // FIXED target's unwrapped DC Reference phase.
                                        //
                                        // Use the actual DC Reference elapsed time rather than assuming
                                        // the 4000-sample window is exactly one second.
                                        // =====================================================

                                        uint64_t residualWindowElapsedNs =
                                            0;


                                        if (qpcLiveFfDcPhaseWindowEndDcNs >=
                                            qpcLiveFfDcPhaseWindowStartDcNs)
                                        {
                                            residualWindowElapsedNs =
                                                qpcLiveFfDcPhaseWindowEndDcNs -
                                                qpcLiveFfDcPhaseWindowStartDcNs;
                                        }


                                        int64_t residualRawPpb =
                                            0;


                                        bool residualElapsedValid =
                                            (
                                                residualWindowElapsedNs >=
                                                QPC_DC_PHASE_RESIDUAL_MIN_ELAPSED_NS &&
                                                residualWindowElapsedNs <=
                                                QPC_DC_PHASE_RESIDUAL_MAX_ELAPSED_NS
                                                );


                                        if (residualElapsedValid)
                                        {
                                            residualRawPpb =
                                                (int64_t)
                                                (
                                                    (
                                                        fixedWindowDeltaNs *
                                                        1000000000LL
                                                        )
                                                    /
                                                    (int64_t)
                                                    residualWindowElapsedNs
                                                    );
                                        }


                                        int64_t residualAbsRawPpb =
                                            residualRawPpb >= 0
                                            ? residualRawPpb
                                            : -residualRawPpb;


                                        bool residualRttValid =
                                            qpcLiveFfDcPhaseMapRttMaxNs <=
                                            QPC_DC_PHASE_RESIDUAL_MAX_RTT_NS;


                                        bool residualMagnitudeValid =
                                            residualAbsRawPpb <=
                                            QPC_DC_PHASE_RESIDUAL_MAX_ABS_PPB;


                                        bool residualCurrentAccepted =
                                            (
                                                residualElapsedValid &&
                                                residualRttValid &&
                                                residualMagnitudeValid
                                                );


                                        // If the phase predictor was rebound to a newly
                                        // initialized shadow timeline, restart this observer.
                                        if (!qpcDcPhaseResidualInitialized ||
                                            qpcDcPhaseResidualBoundInitCount !=
                                            qpcLiveFfDcPhaseBoundInitCount)
                                        {
                                            for (uint32_t i = 0;
                                                i <
                                                QPC_DC_PHASE_RESIDUAL_RING_SIZE;
                                                i++)
                                            {
                                                qpcDcPhaseResidualRing[i] =
                                                    0;
                                            }


                                            qpcDcPhaseResidualRingCount =
                                                0;

                                            qpcDcPhaseResidualRingWriteIndex =
                                                0;

                                            qpcDcPhaseResidualBoundInitCount =
                                                qpcLiveFfDcPhaseBoundInitCount;

                                            qpcDcPhaseResidualInitialized =
                                                true;
                                        }


                                        if (residualCurrentAccepted)
                                        {
                                            qpcDcPhaseResidualRing[
                                                qpcDcPhaseResidualRingWriteIndex] =
                                                residualRawPpb;


                                                qpcDcPhaseResidualRingWriteIndex++;


                                                if (qpcDcPhaseResidualRingWriteIndex >=
                                                    QPC_DC_PHASE_RESIDUAL_RING_SIZE)
                                                {
                                                    qpcDcPhaseResidualRingWriteIndex =
                                                        0;
                                                }


                                                if (qpcDcPhaseResidualRingCount <
                                                    QPC_DC_PHASE_RESIDUAL_RING_SIZE)
                                                {
                                                    qpcDcPhaseResidualRingCount++;
                                                }


                                                qpcDcPhaseResidualAcceptedTotal++;
                                        }
                                        else
                                        {
                                            qpcDcPhaseResidualRejectedTotal++;


                                            if (!residualElapsedValid)
                                            {
                                                qpcDcPhaseResidualRejectElapsedTotal++;
                                            }


                                            if (!residualRttValid)
                                            {
                                                qpcDcPhaseResidualRejectRttTotal++;
                                            }


                                            if (!residualMagnitudeValid)
                                            {
                                                qpcDcPhaseResidualRejectMagnitudeTotal++;
                                            }
                                        }


                                        int64_t residualSorted[
                                            QPC_DC_PHASE_RESIDUAL_RING_SIZE] =
                                            {};


                                            for (uint32_t i = 0;
                                                i <
                                                qpcDcPhaseResidualRingCount;
                                                i++)
                                            {
                                                residualSorted[i] =
                                                    qpcDcPhaseResidualRing[i];
                                            }


                                            // Small fixed-size insertion sort.
                                            for (uint32_t i = 1;
                                                i <
                                                qpcDcPhaseResidualRingCount;
                                                i++)
                                            {
                                                int64_t key =
                                                    residualSorted[i];

                                                int32_t j =
                                                    (int32_t)i - 1;


                                                while (j >= 0 &&
                                                    residualSorted[j] > key)
                                                {
                                                    residualSorted[j + 1] =
                                                        residualSorted[j];

                                                    j--;
                                                }


                                                residualSorted[j + 1] =
                                                    key;
                                            }


                                            int64_t residualMedianPpb =
                                                0;


                                            if (qpcDcPhaseResidualRingCount > 0)
                                            {
                                                if ((qpcDcPhaseResidualRingCount & 1U) != 0)
                                                {
                                                    residualMedianPpb =
                                                        residualSorted[
                                                            qpcDcPhaseResidualRingCount /
                                                                2U];
                                                }
                                                else
                                                {
                                                    uint32_t hi =
                                                        qpcDcPhaseResidualRingCount /
                                                        2U;

                                                    uint32_t lo =
                                                        hi - 1U;


                                                    residualMedianPpb =
                                                        (
                                                            residualSorted[lo] +
                                                            residualSorted[hi]
                                                            )
                                                        /
                                                        2LL;
                                                }
                                            }


                                            int64_t residualDeviationSorted[
                                                QPC_DC_PHASE_RESIDUAL_RING_SIZE] =
                                                {};


                                                for (uint32_t i = 0;
                                                    i <
                                                    qpcDcPhaseResidualRingCount;
                                                    i++)
                                                {
                                                    int64_t deviation =
                                                        qpcDcPhaseResidualRing[i] -
                                                        residualMedianPpb;


                                                    residualDeviationSorted[i] =
                                                        deviation >= 0
                                                        ? deviation
                                                        : -deviation;
                                                }


                                                for (uint32_t i = 1;
                                                    i <
                                                    qpcDcPhaseResidualRingCount;
                                                    i++)
                                                {
                                                    int64_t key =
                                                        residualDeviationSorted[i];

                                                    int32_t j =
                                                        (int32_t)i - 1;


                                                    while (j >= 0 &&
                                                        residualDeviationSorted[j] > key)
                                                    {
                                                        residualDeviationSorted[j + 1] =
                                                            residualDeviationSorted[j];

                                                        j--;
                                                    }


                                                    residualDeviationSorted[j + 1] =
                                                        key;
                                                }


                                                int64_t residualMadPpb =
                                                    0;


                                                if (qpcDcPhaseResidualRingCount > 0)
                                                {
                                                    if ((qpcDcPhaseResidualRingCount & 1U) != 0)
                                                    {
                                                        residualMadPpb =
                                                            residualDeviationSorted[
                                                                qpcDcPhaseResidualRingCount /
                                                                    2U];
                                                    }
                                                    else
                                                    {
                                                        uint32_t hi =
                                                            qpcDcPhaseResidualRingCount /
                                                            2U;

                                                        uint32_t lo =
                                                            hi - 1U;


                                                        residualMadPpb =
                                                            (
                                                                residualDeviationSorted[lo] +
                                                                residualDeviationSorted[hi]
                                                                )
                                                            /
                                                            2LL;
                                                    }
                                                }


                                                int64_t residualRingMinPpb =
                                                    0;

                                                int64_t residualRingMaxPpb =
                                                    0;


                                                if (qpcDcPhaseResidualRingCount > 0)
                                                {
                                                    residualRingMinPpb =
                                                        residualSorted[0];

                                                    residualRingMaxPpb =
                                                        residualSorted[
                                                            qpcDcPhaseResidualRingCount -
                                                                1U];
                                                }


                                                bool residualLocked =
                                                    (
                                                        qpcDcPhaseResidualRingCount >=
                                                        QPC_DC_PHASE_RESIDUAL_LOCK_MIN_SAMPLES &&
                                                        residualMadPpb <=
                                                        QPC_DC_PHASE_RESIDUAL_LOCK_MAX_MAD_PPB
                                                        );


                                                const int64_t
                                                    RESIDUAL_FIXED_SCHEDULER_PPB =
                                                    driftBaselinePpb;


                                                int64_t residualRecommendedSchedulerPpb =
                                                    RESIDUAL_FIXED_SCHEDULER_PPB -
                                                    residualMedianPpb;


                                                // Read Trusted Drift with its existing seqlock.
                                                int64_t residualTrustedDriftPpb =
                                                    RESIDUAL_FIXED_SCHEDULER_PPB;


                                                LONG residualTrustedSeqBefore =
                                                    g_qpcDcTrustedDiagSequence;


                                                if (residualTrustedSeqBefore != 0 &&
                                                    (residualTrustedSeqBefore & 1) == 0)
                                                {
                                                    MemoryBarrier();


                                                    LONGLONG trustedDriftSnapshot =
                                                        g_qpcDcTrustedDriftPpb;


                                                    MemoryBarrier();


                                                    LONG residualTrustedSeqAfter =
                                                        g_qpcDcTrustedDiagSequence;


                                                    if (residualTrustedSeqBefore ==
                                                        residualTrustedSeqAfter &&
                                                        (residualTrustedSeqAfter & 1) == 0)
                                                    {
                                                        residualTrustedDriftPpb =
                                                            (int64_t)
                                                            trustedDriftSnapshot;
                                                    }
                                                }


                                                int64_t residualTrustedMinusRecommendedPpb =
                                                    residualTrustedDriftPpb -
                                                    residualRecommendedSchedulerPpb;


                                                // =====================================================
                                                // Publish residual observer snapshot.
                                                // =====================================================

                                                InterlockedIncrement(
                                                    &g_qpcDcPhaseResidualDiagSequence);


                                                g_qpcDcPhaseResidualInitialized =
                                                    qpcDcPhaseResidualInitialized
                                                    ? 1L
                                                    : 0L;

                                                g_qpcDcPhaseResidualBoundInitCount =
                                                    (LONG)
                                                    qpcDcPhaseResidualBoundInitCount;

                                                g_qpcDcPhaseResidualRawPpb =
                                                    (LONGLONG)
                                                    residualRawPpb;

                                                g_qpcDcPhaseResidualMedianPpb =
                                                    (LONGLONG)
                                                    residualMedianPpb;

                                                g_qpcDcPhaseResidualMadPpb =
                                                    (LONGLONG)
                                                    residualMadPpb;

                                                g_qpcDcPhaseResidualRecommendedSchedulerPpb =
                                                    (LONGLONG)
                                                    residualRecommendedSchedulerPpb;

                                                g_qpcDcPhaseResidualTrustedDriftPpb =
                                                    (LONGLONG)
                                                    residualTrustedDriftPpb;

                                                g_qpcDcPhaseResidualTrustedMinusRecommendedPpb =
                                                    (LONGLONG)
                                                    residualTrustedMinusRecommendedPpb;

                                                g_qpcDcPhaseResidualFixedWindowDeltaNs =
                                                    (LONGLONG)
                                                    fixedWindowDeltaNs;

                                                g_qpcDcPhaseResidualWindowElapsedNs =
                                                    (LONGLONG)
                                                    residualWindowElapsedNs;

                                                g_qpcDcPhaseResidualWindowRttMaxNs =
                                                    (LONGLONG)
                                                    qpcLiveFfDcPhaseMapRttMaxNs;

                                                g_qpcDcPhaseResidualCurrentAccepted =
                                                    residualCurrentAccepted
                                                    ? 1L
                                                    : 0L;

                                                g_qpcDcPhaseResidualBufferCount =
                                                    (LONG)
                                                    qpcDcPhaseResidualRingCount;

                                                g_qpcDcPhaseResidualLocked =
                                                    residualLocked
                                                    ? 1L
                                                    : 0L;

                                                g_qpcDcPhaseResidualAcceptedTotal =
                                                    (LONG)
                                                    qpcDcPhaseResidualAcceptedTotal;

                                                g_qpcDcPhaseResidualRejectedTotal =
                                                    (LONG)
                                                    qpcDcPhaseResidualRejectedTotal;

                                                g_qpcDcPhaseResidualRejectElapsedTotal =
                                                    (LONG)
                                                    qpcDcPhaseResidualRejectElapsedTotal;

                                                g_qpcDcPhaseResidualRejectRttTotal =
                                                    (LONG)
                                                    qpcDcPhaseResidualRejectRttTotal;

                                                g_qpcDcPhaseResidualRejectMagnitudeTotal =
                                                    (LONG)
                                                    qpcDcPhaseResidualRejectMagnitudeTotal;

                                                g_qpcDcPhaseResidualRingMinPpb =
                                                    (LONGLONG)
                                                    residualRingMinPpb;

                                                g_qpcDcPhaseResidualRingMaxPpb =
                                                    (LONGLONG)
                                                    residualRingMaxPpb;


                                                MemoryBarrier();


                                                InterlockedIncrement(
                                                    &g_qpcDcPhaseResidualDiagSequence);


                                                // =====================================================
                                                // DC Reference Phase Residual Drift Observer V1A
                                                // Mean Phase + Theil-Sen Robust Long-Slope
                                                //
                                                // V1 remains above for comparison only.
                                                // V1A is the candidate replacement.
                                                // =====================================================

                                                LARGE_INTEGER v1aCostStart = {};
                                                bool v1aCostStartOk =
                                                    RtQueryPerformanceCounter(&v1aCostStart) ? true : false;

                                                int64_t residualV1AMeanPhaseNs =
                                                    0;


                                                if (qpcDcPhaseResidualV1AMeanSampleCount > 0)
                                                {
                                                    residualV1AMeanPhaseNs =
                                                        qpcDcPhaseResidualV1AMeanSumNs /
                                                        (int64_t)
                                                        qpcDcPhaseResidualV1AMeanSampleCount;
                                                }


                                                int64_t residualV1APhaseSpanNs =
                                                    qpcLiveFfDcPhaseFixedWindowMaxNs -
                                                    qpcLiveFfDcPhaseFixedWindowMinNs;


                                                uint64_t residualV1AWindowElapsedNs =
                                                    residualWindowElapsedNs;


                                                uint64_t residualV1APointTimeNs =
                                                    qpcLiveFfDcPhaseWindowStartDcNs;


                                                if (qpcLiveFfDcPhaseWindowEndDcNs >=
                                                    qpcLiveFfDcPhaseWindowStartDcNs)
                                                {
                                                    residualV1APointTimeNs +=
                                                        (
                                                            qpcLiveFfDcPhaseWindowEndDcNs -
                                                            qpcLiveFfDcPhaseWindowStartDcNs
                                                            )
                                                        /
                                                        2ULL;
                                                }


                                                bool residualV1ASamplesValid =
                                                    qpcDcPhaseResidualV1AMeanSampleCount >=
                                                    QPC_DC_PHASE_RESIDUAL_V1A_MIN_MEAN_SAMPLES;


                                                bool residualV1AElapsedValid =
                                                    (
                                                        residualV1AWindowElapsedNs >=
                                                        QPC_DC_PHASE_RESIDUAL_V1A_MIN_ELAPSED_NS &&
                                                        residualV1AWindowElapsedNs <=
                                                        QPC_DC_PHASE_RESIDUAL_V1A_MAX_ELAPSED_NS
                                                        );


                                                bool residualV1ARttValid =
                                                    qpcLiveFfDcPhaseMapRttMaxNs <=
                                                    QPC_DC_PHASE_RESIDUAL_V1A_MAX_RTT_NS;


                                                bool residualV1ASpanValid =
                                                    (
                                                        residualV1APhaseSpanNs >= 0 &&
                                                        residualV1APhaseSpanNs <=
                                                        QPC_DC_PHASE_RESIDUAL_V1A_MAX_PHASE_SPAN_NS
                                                        );


                                                bool residualV1APointAccepted =
                                                    (
                                                        residualV1ASamplesValid &&
                                                        residualV1AElapsedValid &&
                                                        residualV1ARttValid &&
                                                        residualV1ASpanValid
                                                        );


                                                // Rebind/reset only when shadow/DC phase predictor
                                                // itself receives a new initialization.
                                                if (!qpcDcPhaseResidualV1AInitialized ||
                                                    qpcDcPhaseResidualV1ABoundInitCount !=
                                                    qpcLiveFfDcPhaseBoundInitCount)
                                                {
                                                    for (uint32_t i = 0;
                                                        i <
                                                        QPC_DC_PHASE_RESIDUAL_V1A_POINT_COUNT;
                                                        i++)
                                                    {
                                                        qpcDcPhaseResidualV1APhasePointsNs[i] =
                                                            0;

                                                        qpcDcPhaseResidualV1ATimePointsNs[i] =
                                                            0;
                                                    }


                                                    qpcDcPhaseResidualV1APointCount =
                                                        0;

                                                    qpcDcPhaseResidualV1AWriteIndex =
                                                        0;

                                                    qpcDcPhaseResidualV1ABoundInitCount =
                                                        qpcLiveFfDcPhaseBoundInitCount;

                                                    qpcDcPhaseResidualV1AInitialized =
                                                        true;
                                                }


                                                if (residualV1APointAccepted)
                                                {
                                                    qpcDcPhaseResidualV1APhasePointsNs[
                                                        qpcDcPhaseResidualV1AWriteIndex] =
                                                        residualV1AMeanPhaseNs;

                                                        qpcDcPhaseResidualV1ATimePointsNs[
                                                            qpcDcPhaseResidualV1AWriteIndex] =
                                                            residualV1APointTimeNs;


                                                            qpcDcPhaseResidualV1AWriteIndex++;


                                                            if (qpcDcPhaseResidualV1AWriteIndex >=
                                                                QPC_DC_PHASE_RESIDUAL_V1A_POINT_COUNT)
                                                            {
                                                                qpcDcPhaseResidualV1AWriteIndex =
                                                                    0;
                                                            }


                                                            if (qpcDcPhaseResidualV1APointCount <
                                                                QPC_DC_PHASE_RESIDUAL_V1A_POINT_COUNT)
                                                            {
                                                                qpcDcPhaseResidualV1APointCount++;
                                                            }


                                                            qpcDcPhaseResidualV1AAcceptedPointTotal++;
                                                }
                                                else
                                                {
                                                    qpcDcPhaseResidualV1ARejectedPointTotal++;


                                                    if (!residualV1ASamplesValid)
                                                    {
                                                        qpcDcPhaseResidualV1ARejectSamplesTotal++;
                                                    }


                                                    if (!residualV1AElapsedValid)
                                                    {
                                                        qpcDcPhaseResidualV1ARejectElapsedTotal++;
                                                    }


                                                    if (!residualV1ARttValid)
                                                    {
                                                        qpcDcPhaseResidualV1ARejectRttTotal++;
                                                    }


                                                    if (!residualV1ASpanValid)
                                                    {
                                                        qpcDcPhaseResidualV1ARejectSpanTotal++;
                                                    }
                                                }


                                                // -------------------------------------------------
                                                // Build all pair slopes.
                                                //
                                                // Ring storage order does not matter because each
                                                // point carries its own absolute DC Reference timestamp.
                                                // -------------------------------------------------

                                                int64_t residualV1ASlopes[
                                                    QPC_DC_PHASE_RESIDUAL_V1A_MAX_PAIR_COUNT] =
                                                    {};


                                                    uint32_t residualV1APairCount =
                                                        0;


                                                    uint64_t residualV1AMinTimeNs =
                                                        0;

                                                    uint64_t residualV1AMaxTimeNs =
                                                        0;


                                                    if (qpcDcPhaseResidualV1APointCount > 0)
                                                    {
                                                        residualV1AMinTimeNs =
                                                            qpcDcPhaseResidualV1ATimePointsNs[0];

                                                        residualV1AMaxTimeNs =
                                                            qpcDcPhaseResidualV1ATimePointsNs[0];
                                                    }


                                                    for (uint32_t i = 0;
                                                        i <
                                                        qpcDcPhaseResidualV1APointCount;
                                                        i++)
                                                    {
                                                        uint64_t timeI =
                                                            qpcDcPhaseResidualV1ATimePointsNs[i];


                                                        if (timeI <
                                                            residualV1AMinTimeNs)
                                                        {
                                                            residualV1AMinTimeNs =
                                                                timeI;
                                                        }


                                                        if (timeI >
                                                            residualV1AMaxTimeNs)
                                                        {
                                                            residualV1AMaxTimeNs =
                                                                timeI;
                                                        }


                                                        for (uint32_t j = i + 1;
                                                            j <
                                                            qpcDcPhaseResidualV1APointCount;
                                                            j++)
                                                        {
                                                            uint64_t timeJ =
                                                                qpcDcPhaseResidualV1ATimePointsNs[j];


                                                            if (timeI == timeJ)
                                                            {
                                                                continue;
                                                            }


                                                            int64_t phaseI =
                                                                qpcDcPhaseResidualV1APhasePointsNs[i];

                                                            int64_t phaseJ =
                                                                qpcDcPhaseResidualV1APhasePointsNs[j];


                                                            int64_t deltaPhaseNs =
                                                                phaseJ -
                                                                phaseI;


                                                            uint64_t deltaTimeNs =
                                                                0;


                                                            if (timeJ > timeI)
                                                            {
                                                                deltaTimeNs =
                                                                    timeJ -
                                                                    timeI;
                                                            }
                                                            else
                                                            {
                                                                deltaTimeNs =
                                                                    timeI -
                                                                    timeJ;

                                                                deltaPhaseNs =
                                                                    -deltaPhaseNs;
                                                            }


                                                            if (deltaTimeNs == 0)
                                                            {
                                                                continue;
                                                            }


                                                            if (residualV1APairCount <
                                                                QPC_DC_PHASE_RESIDUAL_V1A_MAX_PAIR_COUNT)
                                                            {
                                                                residualV1ASlopes[
                                                                    residualV1APairCount] =
                                                                    (
                                                                        deltaPhaseNs *
                                                                        1000000000LL
                                                                        )
                                                                        /
                                                                        (int64_t)
                                                                        deltaTimeNs;


                                                                    residualV1APairCount++;
                                                            }
                                                        }
                                                    }


                                                    // Sort pair slopes.
                                                    for (uint32_t i = 1;
                                                        i <
                                                        residualV1APairCount;
                                                        i++)
                                                    {
                                                        int64_t key =
                                                            residualV1ASlopes[i];

                                                        int32_t j =
                                                            (int32_t)i - 1;


                                                        while (j >= 0 &&
                                                            residualV1ASlopes[j] > key)
                                                        {
                                                            residualV1ASlopes[j + 1] =
                                                                residualV1ASlopes[j];

                                                            j--;
                                                        }


                                                        residualV1ASlopes[j + 1] =
                                                            key;
                                                    }


                                                    int64_t residualV1ATheilSenPpb =
                                                        0;


                                                    if (residualV1APairCount > 0)
                                                    {
                                                        if ((residualV1APairCount & 1U) != 0)
                                                        {
                                                            residualV1ATheilSenPpb =
                                                                residualV1ASlopes[
                                                                    residualV1APairCount /
                                                                        2U];
                                                        }
                                                        else
                                                        {
                                                            uint32_t hi =
                                                                residualV1APairCount /
                                                                2U;

                                                            uint32_t lo =
                                                                hi - 1U;


                                                            residualV1ATheilSenPpb =
                                                                (
                                                                    residualV1ASlopes[lo] +
                                                                    residualV1ASlopes[hi]
                                                                    )
                                                                /
                                                                2LL;
                                                        }
                                                    }


                                                    int64_t residualV1ASlopeDeviation[
                                                        QPC_DC_PHASE_RESIDUAL_V1A_MAX_PAIR_COUNT] =
                                                        {};


                                                        for (uint32_t i = 0;
                                                            i <
                                                            residualV1APairCount;
                                                            i++)
                                                        {
                                                            int64_t deviation =
                                                                residualV1ASlopes[i] -
                                                                residualV1ATheilSenPpb;


                                                            residualV1ASlopeDeviation[i] =
                                                                deviation >= 0
                                                                ? deviation
                                                                : -deviation;
                                                        }


                                                        for (uint32_t i = 1;
                                                            i <
                                                            residualV1APairCount;
                                                            i++)
                                                        {
                                                            int64_t key =
                                                                residualV1ASlopeDeviation[i];

                                                            int32_t j =
                                                                (int32_t)i - 1;


                                                            while (j >= 0 &&
                                                                residualV1ASlopeDeviation[j] > key)
                                                            {
                                                                residualV1ASlopeDeviation[j + 1] =
                                                                    residualV1ASlopeDeviation[j];

                                                                j--;
                                                            }


                                                            residualV1ASlopeDeviation[j + 1] =
                                                                key;
                                                        }


                                                        int64_t residualV1ASlopeMadPpb =
                                                            0;


                                                        if (residualV1APairCount > 0)
                                                        {
                                                            if ((residualV1APairCount & 1U) != 0)
                                                            {
                                                                residualV1ASlopeMadPpb =
                                                                    residualV1ASlopeDeviation[
                                                                        residualV1APairCount /
                                                                            2U];
                                                            }
                                                            else
                                                            {
                                                                uint32_t hi =
                                                                    residualV1APairCount /
                                                                    2U;

                                                                uint32_t lo =
                                                                    hi - 1U;


                                                                residualV1ASlopeMadPpb =
                                                                    (
                                                                        residualV1ASlopeDeviation[lo] +
                                                                        residualV1ASlopeDeviation[hi]
                                                                        )
                                                                    /
                                                                    2LL;
                                                            }
                                                        }


                                                        int64_t residualV1ASlopeMinPpb =
                                                            0;

                                                        int64_t residualV1ASlopeMaxPpb =
                                                            0;


                                                        if (residualV1APairCount > 0)
                                                        {
                                                            residualV1ASlopeMinPpb =
                                                                residualV1ASlopes[0];

                                                            residualV1ASlopeMaxPpb =
                                                                residualV1ASlopes[
                                                                    residualV1APairCount -
                                                                        1U];
                                                        }


                                                        uint64_t residualV1ATimeSpanNs =
                                                            0;


                                                        if (residualV1AMaxTimeNs >=
                                                            residualV1AMinTimeNs)
                                                        {
                                                            residualV1ATimeSpanNs =
                                                                residualV1AMaxTimeNs -
                                                                residualV1AMinTimeNs;
                                                        }


                                                        bool residualV1ALocked =
                                                            (
                                                                qpcDcPhaseResidualV1APointCount >=
                                                                QPC_DC_PHASE_RESIDUAL_V1A_LOCK_MIN_POINTS &&
                                                                residualV1ATimeSpanNs >=
                                                                QPC_DC_PHASE_RESIDUAL_V1A_LOCK_MIN_SPAN_NS &&
                                                                residualV1APairCount > 0 &&
                                                                residualV1ASlopeMadPpb <=
                                                                QPC_DC_PHASE_RESIDUAL_V1A_LOCK_MAX_SLOPE_MAD_PPB
                                                                );


                                                        const int64_t
                                                            RESIDUAL_V1A_FIXED_SCHEDULER_PPB =
                                                            driftBaselinePpb;


                                                        int64_t residualV1ARecommendedSchedulerPpb =
                                                            RESIDUAL_V1A_FIXED_SCHEDULER_PPB -
                                                            residualV1ATheilSenPpb;


                                                        // Read current Trusted Drift only for comparison.
                                                        int64_t residualV1ATrustedDriftPpb =
                                                            RESIDUAL_V1A_FIXED_SCHEDULER_PPB;


                                                        LONG residualV1ATrustedSeqBefore =
                                                            g_qpcDcTrustedDiagSequence;


                                                        if (residualV1ATrustedSeqBefore != 0 &&
                                                            (residualV1ATrustedSeqBefore & 1) == 0)
                                                        {
                                                            MemoryBarrier();


                                                            LONGLONG trustedDriftSnapshot =
                                                                g_qpcDcTrustedDriftPpb;


                                                            MemoryBarrier();


                                                            LONG residualV1ATrustedSeqAfter =
                                                                g_qpcDcTrustedDiagSequence;


                                                            if (residualV1ATrustedSeqBefore ==
                                                                residualV1ATrustedSeqAfter &&
                                                                (residualV1ATrustedSeqAfter & 1) == 0)
                                                            {
                                                                residualV1ATrustedDriftPpb =
                                                                    (int64_t)
                                                                    trustedDriftSnapshot;
                                                            }
                                                        }


                                                        int64_t residualV1ATrustedMinusRecommendedPpb =
                                                            residualV1ATrustedDriftPpb -
                                                            residualV1ARecommendedSchedulerPpb;


                                                        InterlockedIncrement(
                                                            &g_qpcDcPhaseResidualV1ADiagSequence);


                                                        g_qpcDcPhaseResidualV1AInitialized =
                                                            qpcDcPhaseResidualV1AInitialized
                                                            ? 1L
                                                            : 0L;

                                                        g_qpcDcPhaseResidualV1ABoundInitCount =
                                                            (LONG)
                                                            qpcDcPhaseResidualV1ABoundInitCount;

                                                        g_qpcDcPhaseResidualV1AMeanPhaseNs =
                                                            (LONGLONG)
                                                            residualV1AMeanPhaseNs;

                                                        g_qpcDcPhaseResidualV1APhaseSpanNs =
                                                            (LONGLONG)
                                                            residualV1APhaseSpanNs;

                                                        g_qpcDcPhaseResidualV1APointTimeNs =
                                                            (LONGLONG)
                                                            residualV1APointTimeNs;

                                                        g_qpcDcPhaseResidualV1APointAccepted =
                                                            residualV1APointAccepted
                                                            ? 1L
                                                            : 0L;

                                                        g_qpcDcPhaseResidualV1AMeanSamples =
                                                            (LONG)
                                                            qpcDcPhaseResidualV1AMeanSampleCount;

                                                        g_qpcDcPhaseResidualV1APointBufferCount =
                                                            (LONG)
                                                            qpcDcPhaseResidualV1APointCount;

                                                        g_qpcDcPhaseResidualV1APairSlopeCount =
                                                            (LONG)
                                                            residualV1APairCount;
                                                        g_qpcDcPhaseResidualV1ATheilSenPpb =
                                                            (LONGLONG)
                                                            residualV1ATheilSenPpb;

                                                        g_qpcDcPhaseResidualV1ASlopeMadPpb =
                                                            (LONGLONG)
                                                            residualV1ASlopeMadPpb;

                                                        g_qpcDcPhaseResidualV1ATimeSpanNs =
                                                            (LONGLONG)
                                                            residualV1ATimeSpanNs;

                                                        g_qpcDcPhaseResidualV1ALocked =
                                                            residualV1ALocked
                                                            ? 1L
                                                            : 0L;

                                                        g_qpcDcPhaseResidualV1ARecommendedSchedulerPpb =
                                                            (LONGLONG)
                                                            residualV1ARecommendedSchedulerPpb;

                                                        g_qpcDcPhaseResidualV1ATrustedDriftPpb =
                                                            (LONGLONG)
                                                            residualV1ATrustedDriftPpb;

                                                        g_qpcDcPhaseResidualV1ATrustedMinusRecommendedPpb =
                                                            (LONGLONG)
                                                            residualV1ATrustedMinusRecommendedPpb;

                                                        g_qpcDcPhaseResidualV1AWindowElapsedNs =
                                                            (LONGLONG)
                                                            residualV1AWindowElapsedNs;

                                                        g_qpcDcPhaseResidualV1AWindowRttMaxNs =
                                                            (LONGLONG)
                                                            qpcLiveFfDcPhaseMapRttMaxNs;

                                                        g_qpcDcPhaseResidualV1AAcceptedPointTotal =
                                                            (LONG)
                                                            qpcDcPhaseResidualV1AAcceptedPointTotal;

                                                        g_qpcDcPhaseResidualV1ARejectedPointTotal =
                                                            (LONG)
                                                            qpcDcPhaseResidualV1ARejectedPointTotal;

                                                        g_qpcDcPhaseResidualV1ARejectSamplesTotal =
                                                            (LONG)
                                                            qpcDcPhaseResidualV1ARejectSamplesTotal;

                                                        g_qpcDcPhaseResidualV1ARejectElapsedTotal =
                                                            (LONG)
                                                            qpcDcPhaseResidualV1ARejectElapsedTotal;

                                                        g_qpcDcPhaseResidualV1ARejectRttTotal =
                                                            (LONG)
                                                            qpcDcPhaseResidualV1ARejectRttTotal;

                                                        g_qpcDcPhaseResidualV1ARejectSpanTotal =
                                                            (LONG)
                                                            qpcDcPhaseResidualV1ARejectSpanTotal;

                                                        g_qpcDcPhaseResidualV1ASlopeMinPpb =
                                                            (LONGLONG)
                                                            residualV1ASlopeMinPpb;

                                                        g_qpcDcPhaseResidualV1ASlopeMaxPpb =
                                                            (LONGLONG)
                                                            residualV1ASlopeMaxPpb;


                                                        MemoryBarrier();


                                                        InterlockedIncrement(
                                                            &g_qpcDcPhaseResidualV1ADiagSequence);

                                                        LARGE_INTEGER v1aCostEnd = {};
                                                        if (v1aCostStartOk &&
                                                            RtQueryPerformanceCounter(&v1aCostEnd) &&
                                                            v1aCostEnd.QuadPart >= v1aCostStart.QuadPart &&
                                                            qpcFrequency > 0)
                                                        {
                                                            uint64_t ns =
                                                                ((uint64_t)(v1aCostEnd.QuadPart - v1aCostStart.QuadPart)
                                                                    * 1000000000ULL) / qpcFrequency;

                                                            qpcV1aCostEvents++;
                                                            qpcV1aCostSumNs += ns;
                                                            if (ns > qpcV1aCostMaxNs) qpcV1aCostMaxNs = ns;
                                                            if (ns > 20000ULL) qpcV1aCostOver20us++;
                                                            if (ns > 40000ULL) qpcV1aCostOver40us++;
                                                            if (ns > 80000ULL) qpcV1aCostOver80us++;

                                                            InterlockedIncrement(&g_qpcV1aCostSeq);
                                                            g_qpcV1aCostLastNs = (LONGLONG)ns;
                                                            g_qpcV1aCostAvgNs =
                                                                (LONGLONG)(qpcV1aCostSumNs / qpcV1aCostEvents);
                                                            g_qpcV1aCostMaxNs = (LONGLONG)qpcV1aCostMaxNs;
                                                            g_qpcV1aCostEvents = (LONG)qpcV1aCostEvents;
                                                            g_qpcV1aCostOver20us = (LONG)qpcV1aCostOver20us;
                                                            g_qpcV1aCostOver40us = (LONG)qpcV1aCostOver40us;
                                                            g_qpcV1aCostOver80us = (LONG)qpcV1aCostOver80us;
                                                            g_qpcV1aCostQpcFail = (LONG)qpcV1aCostQpcFail;
                                                            MemoryBarrier();
                                                            InterlockedIncrement(&g_qpcV1aCostSeq);
                                                        }
                                                        else
                                                        {
                                                            qpcV1aCostQpcFail++;
                                                        }


                                                        // Reset only the per-window mean accumulator.
                                                        // The 16-point Theil-Sen history remains.
                                                        qpcDcPhaseResidualV1AMeanSumNs =
                                                            0;

                                                        qpcDcPhaseResidualV1AMeanSampleCount =
                                                            0;


                                                        InterlockedIncrement(
                                                            &g_qpcLiveFfDcPhaseDiagSequence);


                                                        g_qpcLiveFfDcPhaseInitialized =
                                                            qpcLiveFfDcPhaseInitialized
                                                            ? 1L
                                                            : 0L;

                                                        g_qpcLiveFfDcPhaseLastSampleQpc =
                                                            (LONGLONG)qpcDcMidCount;

                                                        g_qpcLiveFfDcPhaseBoundInitCount =
                                                            (LONG)
                                                            qpcLiveFfDcPhaseBoundInitCount;


                                                        g_qpcLiveFfDcPhaseBaselinePhaseNs =
                                                            (LONGLONG)
                                                            qpcLiveFfDcPhaseBaselinePhaseNs;

                                                        g_qpcLiveFfDcPhaseBaselineSync0MarginNs =
                                                            (LONGLONG)
                                                            qpcLiveFfDcPhaseBaselineSync0MarginNs;


                                                        g_qpcLiveFfDcPhaseFixedPhaseNs =
                                                            (LONGLONG)
                                                            fixedTargetPhaseNs;

                                                        g_qpcLiveFfDcPhaseShadowPhaseNs =
                                                            (LONGLONG)
                                                            shadowTargetPhaseNs;

                                                        g_qpcLiveFfDcPhaseFixedSync0MarginNs =
                                                            (LONGLONG)
                                                            fixedSync0MarginNs;

                                                        g_qpcLiveFfDcPhaseShadowSync0MarginNs =
                                                            (LONGLONG)
                                                            shadowSync0MarginNs;


                                                        g_qpcLiveFfDcPhaseFixedWrappedErrorNs =
                                                            (LONGLONG)
                                                            fixedWrappedErrorNs;

                                                        g_qpcLiveFfDcPhaseShadowWrappedErrorNs =
                                                            (LONGLONG)
                                                            shadowWrappedErrorNs;

                                                        g_qpcLiveFfDcPhaseFixedUnwrappedErrorNs =
                                                            (LONGLONG)
                                                            qpcLiveFfDcPhaseFixedUnwrappedNs;

                                                        g_qpcLiveFfDcPhaseShadowUnwrappedErrorNs =
                                                            (LONGLONG)
                                                            qpcLiveFfDcPhaseShadowUnwrappedNs;

                                                        g_qpcLiveFfDcPhaseShadowVsFixedNs =
                                                            (LONGLONG)
                                                            shadowVsFixedPhaseNs;


                                                        g_qpcLiveFfDcPhaseFixedWindowStartNs =
                                                            (LONGLONG)
                                                            qpcLiveFfDcPhaseFixedWindowStartNs;

                                                        g_qpcLiveFfDcPhaseFixedWindowEndNs =
                                                            (LONGLONG)
                                                            qpcLiveFfDcPhaseFixedWindowEndNs;

                                                        g_qpcLiveFfDcPhaseFixedWindowDeltaNs =
                                                            (LONGLONG)
                                                            fixedWindowDeltaNs;

                                                        g_qpcLiveFfDcPhaseFixedWindowMinNs =
                                                            (LONGLONG)
                                                            qpcLiveFfDcPhaseFixedWindowMinNs;

                                                        g_qpcLiveFfDcPhaseFixedWindowMaxNs =
                                                            (LONGLONG)
                                                            qpcLiveFfDcPhaseFixedWindowMaxNs;


                                                        g_qpcLiveFfDcPhaseShadowWindowStartNs =
                                                            (LONGLONG)
                                                            qpcLiveFfDcPhaseShadowWindowStartNs;

                                                        g_qpcLiveFfDcPhaseShadowWindowEndNs =
                                                            (LONGLONG)
                                                            qpcLiveFfDcPhaseShadowWindowEndNs;

                                                        g_qpcLiveFfDcPhaseShadowWindowDeltaNs =
                                                            (LONGLONG)
                                                            shadowWindowDeltaNs;

                                                        g_qpcLiveFfDcPhaseShadowWindowMinNs =
                                                            (LONGLONG)
                                                            qpcLiveFfDcPhaseShadowWindowMinNs;

                                                        g_qpcLiveFfDcPhaseShadowWindowMaxNs =
                                                            (LONGLONG)
                                                            qpcLiveFfDcPhaseShadowWindowMaxNs;


                                                        g_qpcLiveFfDcPhaseFixedAbsAvgNs =
                                                            (LONGLONG)
                                                            fixedAbsAvgNs;

                                                        g_qpcLiveFfDcPhaseShadowAbsAvgNs =
                                                            (LONGLONG)
                                                            shadowAbsAvgNs;


                                                        g_qpcLiveFfDcPhaseShadowBetterCount =
                                                            (LONG)
                                                            qpcLiveFfDcPhaseShadowBetterCount;

                                                        g_qpcLiveFfDcPhaseShadowWorseCount =
                                                            (LONG)
                                                            qpcLiveFfDcPhaseShadowWorseCount;

                                                        g_qpcLiveFfDcPhaseEqualCount =
                                                            (LONG)
                                                            qpcLiveFfDcPhaseEqualCount;


                                                        g_qpcLiveFfDcPhaseMapRttAvgNs =
                                                            (LONGLONG)
                                                            mapRttAvgNs;

                                                        g_qpcLiveFfDcPhaseMapRttMinNs =
                                                            (LONGLONG)
                                                            qpcLiveFfDcPhaseMapRttMinNs;

                                                        g_qpcLiveFfDcPhaseMapRttMaxNs =
                                                            (LONGLONG)
                                                            qpcLiveFfDcPhaseMapRttMaxNs;

                                                        g_qpcLiveFfDcPhaseSamples =
                                                            (LONG)
                                                            qpcLiveFfDcPhaseWindowSamples;


                                                        MemoryBarrier();


                                                        InterlockedIncrement(
                                                            &g_qpcLiveFfDcPhaseDiagSequence);


                                                        qpcLiveFfDcPhaseWindowSamples =
                                                            0;

                                                        qpcLiveFfDcPhaseFixedAbsSumNs =
                                                            0;

                                                        qpcLiveFfDcPhaseShadowAbsSumNs =
                                                            0;

                                                        qpcLiveFfDcPhaseShadowBetterCount =
                                                            0;

                                                        qpcLiveFfDcPhaseShadowWorseCount =
                                                            0;

                                                        qpcLiveFfDcPhaseEqualCount =
                                                            0;

                                                        qpcLiveFfDcPhaseMapRttSumNs =
                                                            0;

                                                        qpcLiveFfDcPhaseMapRttMinNs =
                                                            0;

                                                        qpcLiveFfDcPhaseMapRttMaxNs =
                                                            0;
                                    }
                                }


                                if (qpcDcWindowValidSamples == 0)
                                {
                                    qpcDcWindowStartMidCount =
                                        qpcDcMidCount;

                                    qpcDcWindowStartDcNs =
                                        pMaster->
                                        DC_reference_time;


                                    qpcDcRttMinNs =
                                        qpcDcRttNs;

                                    qpcDcRttMaxNs =
                                        qpcDcRttNs;
                                }


                                qpcDcWindowEndMidCount =
                                    qpcDcMidCount;

                                qpcDcWindowEndDcNs =
                                    pMaster->
                                    DC_reference_time;


                                if (qpcDcRttNs <
                                    qpcDcRttMinNs)
                                {
                                    qpcDcRttMinNs =
                                        qpcDcRttNs;
                                }


                                if (qpcDcRttNs >
                                    qpcDcRttMaxNs)
                                {
                                    qpcDcRttMaxNs =
                                        qpcDcRttNs;
                                }


                                qpcDcRttSumNs +=
                                    qpcDcRttNs;


                                qpcDcWindowValidSamples++;


                                // =========================================================
                                // Complete approximately one-second window
                                // =========================================================

                                if (qpcDcWindowValidSamples >=
                                    4000U)
                                {
                                    bool windowValid =
                                        false;


                                    uint64_t qpcElapsedNs =
                                        0;

                                    uint64_t dcElapsedNs =
                                        0;

                                    int64_t deltaNs =
                                        0;

                                    int64_t driftPpb =
                                        0;


                                    if (qpcDcWindowEndMidCount >
                                        qpcDcWindowStartMidCount &&
                                        qpcDcWindowEndDcNs >
                                        qpcDcWindowStartDcNs)
                                    {
                                        uint64_t deltaQpcCounts =
                                            qpcDcWindowEndMidCount -
                                            qpcDcWindowStartMidCount;


                                        // -------------------------------------------------
                                        // 約 1 秒 Window：
                                        //
                                        // 3GHz × 1sec × 1e9
                                        //
                                        // 仍在 uint64_t 範圍內。
                                        // -------------------------------------------------

                                        qpcElapsedNs =
                                            (
                                                deltaQpcCounts *
                                                1000000000ULL
                                                )
                                            /
                                            qpcFrequency;


                                        dcElapsedNs =
                                            qpcDcWindowEndDcNs -
                                            qpcDcWindowStartDcNs;


                                        if (qpcElapsedNs > 0 &&
                                            dcElapsedNs > 0)
                                        {
                                            deltaNs =
                                                (int64_t)
                                                qpcElapsedNs -
                                                (int64_t)
                                                dcElapsedNs;


                                            driftPpb =
                                                (
                                                    deltaNs *
                                                    1000000000LL
                                                    )
                                                /
                                                (int64_t)
                                                dcElapsedNs;


                                            windowValid =
                                                true;
                                        }
                                    }


                                    // =====================================================
                                    // Robust Drift Dry-Run V1
                                    //
                                    // Input:
                                    //     one raw ~1 second drift window.
                                    //
                                    // Acceptance gates intentionally reject heavily disturbed
                                    // windows before they enter the 9-window median buffer.
                                    //
                                    // IMPORTANT:
                                    //     The result is diagnostic only.
                                    //     The real scheduler uses the startup baseline.
                                    // =====================================================

                                    if (windowValid)
                                    {
                                        uint64_t currentRttAvgNs =
                                            qpcDcWindowValidSamples > 0
                                            ?
                                            (
                                                qpcDcRttSumNs /
                                                qpcDcWindowValidSamples
                                                )
                                            :
                                            0ULL;


                                        int64_t absRawDriftPpb =
                                            driftPpb >= 0
                                            ? driftPpb
                                            : -driftPpb;


                                        bool robustCurrentAccepted =
                                            (
                                                absRawDriftPpb <=
                                                QPC_DC_ROBUST_MAX_ABS_DRIFT_PPB &&
                                                currentRttAvgNs > 0 &&
                                                currentRttAvgNs <=
                                                QPC_DC_ROBUST_MAX_RTT_AVG_NS &&
                                                qpcDcRttMaxNs > 0 &&
                                                qpcDcRttMaxNs <=
                                                QPC_DC_ROBUST_MAX_RTT_MAX_NS
                                                );


                                        if (robustCurrentAccepted)
                                        {
                                            qpcDcRobustRing[
                                                qpcDcRobustRingWriteIndex
                                            ] =
                                                driftPpb;


                                                qpcDcRobustRingWriteIndex++;

                                                if (qpcDcRobustRingWriteIndex >=
                                                    QPC_DC_ROBUST_RING_SIZE)
                                                {
                                                    qpcDcRobustRingWriteIndex =
                                                        0;
                                                }


                                                if (qpcDcRobustRingCount <
                                                    QPC_DC_ROBUST_RING_SIZE)
                                                {
                                                    qpcDcRobustRingCount++;
                                                }


                                                qpcDcRobustAcceptedTotal++;
                                        }
                                        else
                                        {
                                            qpcDcRobustRejectedTotal++;
                                        }


                                        int64_t robustMedianPpb =
                                            0;

                                        int64_t robustMadPpb =
                                            0;


                                        if (qpcDcRobustRingCount > 0)
                                        {
                                            int64_t sortedDrift[9] =
                                            {};


                                            for (uint32_t i = 0;
                                                i < qpcDcRobustRingCount;
                                                ++i)
                                            {
                                                sortedDrift[i] =
                                                    qpcDcRobustRing[i];
                                            }


                                            // Insertion sort: max 9 values.
                                            for (uint32_t i = 1;
                                                i < qpcDcRobustRingCount;
                                                ++i)
                                            {
                                                int64_t key =
                                                    sortedDrift[i];

                                                int32_t j =
                                                    (int32_t)i -
                                                    1;


                                                while (j >= 0 &&
                                                    sortedDrift[j] > key)
                                                {
                                                    sortedDrift[j + 1] =
                                                        sortedDrift[j];

                                                    --j;
                                                }


                                                sortedDrift[j + 1] =
                                                    key;
                                            }


                                            if ((qpcDcRobustRingCount & 1U) != 0)
                                            {
                                                robustMedianPpb =
                                                    sortedDrift[
                                                        qpcDcRobustRingCount /
                                                            2U
                                                    ];
                                            }
                                            else
                                            {
                                                uint32_t upper =
                                                    qpcDcRobustRingCount /
                                                    2U;

                                                uint32_t lower =
                                                    upper -
                                                    1U;


                                                robustMedianPpb =
                                                    (
                                                        sortedDrift[lower] +
                                                        sortedDrift[upper]
                                                        )
                                                    /
                                                    2LL;
                                            }


                                            int64_t sortedDeviation[9] =
                                            {};


                                            for (uint32_t i = 0;
                                                i < qpcDcRobustRingCount;
                                                ++i)
                                            {
                                                int64_t deviation =
                                                    qpcDcRobustRing[i] -
                                                    robustMedianPpb;


                                                if (deviation < 0)
                                                {
                                                    deviation =
                                                        -deviation;
                                                }


                                                sortedDeviation[i] =
                                                    deviation;
                                            }


                                            // Insertion sort: max 9 values.
                                            for (uint32_t i = 1;
                                                i < qpcDcRobustRingCount;
                                                ++i)
                                            {
                                                int64_t key =
                                                    sortedDeviation[i];

                                                int32_t j =
                                                    (int32_t)i -
                                                    1;


                                                while (j >= 0 &&
                                                    sortedDeviation[j] > key)
                                                {
                                                    sortedDeviation[j + 1] =
                                                        sortedDeviation[j];

                                                    --j;
                                                }


                                                sortedDeviation[j + 1] =
                                                    key;
                                            }


                                            if ((qpcDcRobustRingCount & 1U) != 0)
                                            {
                                                robustMadPpb =
                                                    sortedDeviation[
                                                        qpcDcRobustRingCount /
                                                            2U
                                                    ];
                                            }
                                            else
                                            {
                                                uint32_t upper =
                                                    qpcDcRobustRingCount /
                                                    2U;

                                                uint32_t lower =
                                                    upper -
                                                    1U;


                                                robustMadPpb =
                                                    (
                                                        sortedDeviation[lower] +
                                                        sortedDeviation[upper]
                                                        )
                                                    /
                                                    2LL;
                                            }
                                        }


                                        bool robustLocked =
                                            (
                                                qpcDcRobustRingCount >=
                                                QPC_DC_ROBUST_MIN_LOCK_SAMPLES &&
                                                robustMadPpb <=
                                                QPC_DC_ROBUST_MAX_MAD_PPB
                                                );


                                        // 250 us cycle:
                                        //
                                        // drift(ppb) * 250000(ns) / 1e6
                                        //     = period correction in ps/cycle.
                                        //
                                        // Diagnostic only.
                                        int64_t robustPeriodFfPs =
                                            (
                                                250000LL *
                                                robustMedianPpb
                                                )
                                            /
                                            1000000LL;


                                        // Calibration 鎖定後只綁定一次新 Baseline；先前以
                                        // Bootstrap 建立的 WARMUP/HOLD 狀態不得沿用。
                                        if (driftCalibrationLocked &&
                                            !qpcDcTrustedCalibrationBound)
                                        {
                                            qpcDcTrustedDriftPpb =
                                                driftBaselinePpb;

                                            qpcDcTrustedValid = false;
                                            qpcDcTrustedState = 0;
                                            qpcDcTrustedWarmupGoodCount = 0;
                                            qpcDcTrustedBadCount = 0;
                                            qpcDcTrustedRecoveryGoodCount = 0;
                                            qpcDcTrustedLastTransitionTrustedPpb =
                                                driftBaselinePpb;
                                            qpcDcTrustedCalibrationBound = true;
                                        }

                                        // =====================================================
                                        // Trusted Drift V1 - DRY RUN STATE MACHINE
                                        //
                                        // This layer intentionally treats the existing robust
                                        // median as a CANDIDATE, not automatically as truth.
                                        //
                                        // The real scheduler uses the startup baseline.
                                        // =====================================================

                                        int64_t trustedCandidateDeviationPpb =
                                            robustMedianPpb -
                                            qpcDcTrustedDriftPpb;


                                        int64_t trustedAbsCandidateDeviationPpb =
                                            trustedCandidateDeviationPpb >= 0
                                            ? trustedCandidateDeviationPpb
                                            : -trustedCandidateDeviationPpb;


                                        int64_t trustedRawMedianDeviationPpb =
                                            driftPpb -
                                            robustMedianPpb;


                                        int64_t trustedAbsRawMedianDeviationPpb =
                                            trustedRawMedianDeviationPpb >= 0
                                            ? trustedRawMedianDeviationPpb
                                            : -trustedRawMedianDeviationPpb;


                                        bool trustedCandidateGood =
                                            (
                                                driftCalibrationLocked &&
                                                robustCurrentAccepted &&
                                                robustLocked &&
                                                qpcDcRobustRingCount >=
                                                QPC_DC_ROBUST_RING_SIZE &&
                                                robustMadPpb <=
                                                QPC_DC_TRUSTED_MAX_MAD_PPB &&
                                                trustedAbsCandidateDeviationPpb <=
                                                QPC_DC_TRUSTED_MAX_BASELINE_DEVIATION_PPB &&
                                                trustedAbsRawMedianDeviationPpb <=
                                                QPC_DC_TRUSTED_MAX_RAW_MEDIAN_DEVIATION_PPB
                                                );


                                        // =====================================================
                                        // Trusted Drift V1A reject-reason classification
                                        //
                                        // Multiple bits may be set in the same window.
                                        // This does NOT alter trustedCandidateGood.
                                        // =====================================================

                                        const LONG TRUST_REJECT_ROBUST_ACCEPT =
                                            0x01L;

                                        const LONG TRUST_REJECT_ROBUST_LOCK =
                                            0x02L;

                                        const LONG TRUST_REJECT_BUFFER_NOT_FULL =
                                            0x04L;

                                        const LONG TRUST_REJECT_MAD =
                                            0x08L;

                                        const LONG TRUST_REJECT_CANDIDATE_DEVIATION =
                                            0x10L;

                                        const LONG TRUST_REJECT_RAW_MEDIAN_DEVIATION =
                                            0x20L;


                                        LONG trustedCurrentRejectMask =
                                            0;


                                        if (!robustCurrentAccepted)
                                        {
                                            trustedCurrentRejectMask |=
                                                TRUST_REJECT_ROBUST_ACCEPT;

                                            qpcDcTrustedRejectRobustAcceptTotal++;
                                        }


                                        if (!robustLocked)
                                        {
                                            trustedCurrentRejectMask |=
                                                TRUST_REJECT_ROBUST_LOCK;

                                            qpcDcTrustedRejectRobustLockTotal++;
                                        }


                                        if (qpcDcRobustRingCount <
                                            QPC_DC_ROBUST_RING_SIZE)
                                        {
                                            trustedCurrentRejectMask |=
                                                TRUST_REJECT_BUFFER_NOT_FULL;

                                            qpcDcTrustedRejectBufferNotFullTotal++;
                                        }


                                        if (robustMadPpb >
                                            QPC_DC_TRUSTED_MAX_MAD_PPB)
                                        {
                                            trustedCurrentRejectMask |=
                                                TRUST_REJECT_MAD;

                                            qpcDcTrustedRejectMadTotal++;
                                        }


                                        if (trustedAbsCandidateDeviationPpb >
                                            QPC_DC_TRUSTED_MAX_BASELINE_DEVIATION_PPB)
                                        {
                                            trustedCurrentRejectMask |=
                                                TRUST_REJECT_CANDIDATE_DEVIATION;

                                            qpcDcTrustedRejectCandidateDeviationTotal++;
                                        }


                                        if (trustedAbsRawMedianDeviationPpb >
                                            QPC_DC_TRUSTED_MAX_RAW_MEDIAN_DEVIATION_PPB)
                                        {
                                            trustedCurrentRejectMask |=
                                                TRUST_REJECT_RAW_MEDIAN_DEVIATION;

                                            qpcDcTrustedRejectRawMedianDeviationTotal++;
                                        }


                                        if (!trustedCandidateGood)
                                        {
                                            qpcDcTrustedLastRejectMask =
                                                trustedCurrentRejectMask;
                                        }


                                        LONG trustedStateBefore =
                                            qpcDcTrustedState;


                                        int64_t trustedSlewAppliedPpb =
                                            0;


                                        switch (qpcDcTrustedState)
                                        {
                                            // -------------------------------------------------
                                            // 0 = WARMUP
                                            //
                                            // Require a full robust buffer plus 8 consecutive
                                            // clean windows near the locked startup baseline.
                                            // A startup local cluster around -5.x ppm therefore
                                            // cannot become trusted merely because MAD is small.
                                            // -------------------------------------------------
                                        case 0:
                                            qpcDcTrustedValid =
                                                false;

                                            qpcDcTrustedBadCount =
                                                0;

                                            qpcDcTrustedRecoveryGoodCount =
                                                0;


                                            if (trustedCandidateGood)
                                            {
                                                qpcDcTrustedWarmupGoodCount++;


                                                if (qpcDcTrustedWarmupGoodCount >=
                                                    QPC_DC_TRUSTED_WARMUP_GOOD_WINDOWS)
                                                {
                                                    qpcDcTrustedState =
                                                        1;      // TRACK

                                                    qpcDcTrustedValid =
                                                        true;

                                                    qpcDcTrustedWarmupGoodCount =
                                                        QPC_DC_TRUSTED_WARMUP_GOOD_WINDOWS;
                                                }
                                            }
                                            else
                                            {
                                                qpcDcTrustedWarmupGoodCount =
                                                    0;
                                            }

                                            break;


                                            // -------------------------------------------------
                                            // 1 = TRACK
                                            //
                                            // Trusted value follows the candidate only through
                                            // a very small slew limit: max 50 ppb per ~1 sec.
                                            // -------------------------------------------------
                                        case 1:
                                            if (trustedCandidateGood)
                                            {
                                                qpcDcTrustedBadCount =
                                                    0;

                                                qpcDcTrustedRecoveryGoodCount =
                                                    0;


                                                int64_t desiredStepPpb =
                                                    robustMedianPpb -
                                                    qpcDcTrustedDriftPpb;


                                                if (desiredStepPpb >
                                                    QPC_DC_TRUSTED_MAX_SLEW_PPB_PER_WINDOW)
                                                {
                                                    desiredStepPpb =
                                                        QPC_DC_TRUSTED_MAX_SLEW_PPB_PER_WINDOW;
                                                }
                                                else if (desiredStepPpb <
                                                    -QPC_DC_TRUSTED_MAX_SLEW_PPB_PER_WINDOW)
                                                {
                                                    desiredStepPpb =
                                                        -QPC_DC_TRUSTED_MAX_SLEW_PPB_PER_WINDOW;
                                                }


                                                qpcDcTrustedDriftPpb +=
                                                    desiredStepPpb;

                                                trustedSlewAppliedPpb =
                                                    desiredStepPpb;

                                                qpcDcTrustedUpdateTotal++;
                                            }
                                            else
                                            {
                                                qpcDcTrustedState =
                                                    2;          // HOLD

                                                qpcDcTrustedBadCount =
                                                    1;

                                                qpcDcTrustedRecoveryGoodCount =
                                                    0;

                                                qpcDcTrustedHoldTotal++;
                                            }

                                            break;


                                            // -------------------------------------------------
                                            // 2 = HOLD
                                            //
                                            // Keep the last trusted value. Do not chase a new
                                            // local cluster. Up to 10 bad windows are tolerated.
                                            // Three consecutive good windows return to TRACK.
                                            // -------------------------------------------------
                                        case 2:
                                            if (trustedCandidateGood)
                                            {
                                                qpcDcTrustedRecoveryGoodCount++;


                                                if (qpcDcTrustedRecoveryGoodCount >=
                                                    QPC_DC_TRUSTED_HOLD_RECOVERY_WINDOWS)
                                                {
                                                    qpcDcTrustedState =
                                                        1;      // TRACK

                                                    qpcDcTrustedBadCount =
                                                        0;

                                                    qpcDcTrustedRecoveryGoodCount =
                                                        0;
                                                }
                                            }
                                            else
                                            {
                                                qpcDcTrustedRecoveryGoodCount =
                                                    0;

                                                qpcDcTrustedBadCount++;


                                                if (qpcDcTrustedBadCount >=
                                                    QPC_DC_TRUSTED_HOLD_MAX_BAD_WINDOWS)
                                                {
                                                    qpcDcTrustedState =
                                                        3;      // UNTRUSTED

                                                    qpcDcTrustedValid =
                                                        false;

                                                    qpcDcTrustedUnlockTotal++;

                                                    qpcDcTrustedRecoveryGoodCount =
                                                        0;
                                                }
                                            }

                                            break;


                                            // -------------------------------------------------
                                            // 3 = UNTRUSTED
                                            //
                                            // Keep the last known-safe numeric value, but mark
                                            // it invalid for future live control. Five clean
                                            // windows near the held baseline are required to
                                            // relock.
                                            // -------------------------------------------------
                                        default:
                                            qpcDcTrustedValid =
                                                false;


                                            if (trustedCandidateGood)
                                            {
                                                qpcDcTrustedRecoveryGoodCount++;


                                                if (qpcDcTrustedRecoveryGoodCount >=
                                                    QPC_DC_TRUSTED_RELOCK_GOOD_WINDOWS)
                                                {
                                                    qpcDcTrustedState =
                                                        1;      // TRACK

                                                    qpcDcTrustedValid =
                                                        true;

                                                    qpcDcTrustedBadCount =
                                                        0;

                                                    qpcDcTrustedRecoveryGoodCount =
                                                        0;

                                                    qpcDcTrustedRelockTotal++;
                                                }
                                            }
                                            else
                                            {
                                                qpcDcTrustedRecoveryGoodCount =
                                                    0;
                                            }

                                            break;
                                        }


                                        // =====================================================
                                        // Trusted Drift V1A transition diagnostic
                                        // =====================================================

                                        if (qpcDcTrustedBadCount >
                                            qpcDcTrustedMaxBadStreak)
                                        {
                                            qpcDcTrustedMaxBadStreak =
                                                qpcDcTrustedBadCount;
                                        }


                                        if (trustedStateBefore !=
                                            qpcDcTrustedState)
                                        {
                                            if (trustedStateBefore == 0 &&
                                                qpcDcTrustedState == 1)
                                            {
                                                qpcDcTrustedWarmupToTrackTotal++;
                                            }
                                            else if (trustedStateBefore == 1 &&
                                                qpcDcTrustedState == 2)
                                            {
                                                qpcDcTrustedTrackToHoldTotal++;
                                            }
                                            else if (trustedStateBefore == 2 &&
                                                qpcDcTrustedState == 1)
                                            {
                                                qpcDcTrustedHoldToTrackTotal++;
                                            }
                                            else if (trustedStateBefore == 2 &&
                                                qpcDcTrustedState == 3)
                                            {
                                                qpcDcTrustedHoldToUntrustedTotal++;
                                            }
                                            else if (trustedStateBefore == 3 &&
                                                qpcDcTrustedState == 1)
                                            {
                                                qpcDcTrustedUntrustedToTrackTotal++;
                                            }


                                            qpcDcTrustedLastTransitionFrom =
                                                trustedStateBefore;

                                            qpcDcTrustedLastTransitionTo =
                                                qpcDcTrustedState;

                                            qpcDcTrustedLastTransitionRawPpb =
                                                driftPpb;

                                            qpcDcTrustedLastTransitionMedianPpb =
                                                robustMedianPpb;

                                            qpcDcTrustedLastTransitionMadPpb =
                                                robustMadPpb;

                                            qpcDcTrustedLastTransitionTrustedPpb =
                                                qpcDcTrustedDriftPpb;
                                        }


                                        int64_t trustedPeriodFfPs =
                                            (
                                                250000LL *
                                                qpcDcTrustedDriftPpb
                                                )
                                            /
                                            1000000LL;


                                        // Recalculate deviation AFTER any allowed slew update.
                                        trustedCandidateDeviationPpb =
                                            robustMedianPpb -
                                            qpcDcTrustedDriftPpb;


                                        // =====================================================
                                        // Publish Trusted Drift V1A reason snapshot.
                                        // =====================================================

                                        InterlockedIncrement(
                                            &g_qpcDcTrustedReasonDiagSequence);


                                        g_qpcDcTrustedRejectRobustAcceptTotal =
                                            (LONG)
                                            qpcDcTrustedRejectRobustAcceptTotal;

                                        g_qpcDcTrustedRejectRobustLockTotal =
                                            (LONG)
                                            qpcDcTrustedRejectRobustLockTotal;

                                        g_qpcDcTrustedRejectBufferNotFullTotal =
                                            (LONG)
                                            qpcDcTrustedRejectBufferNotFullTotal;

                                        g_qpcDcTrustedRejectMadTotal =
                                            (LONG)
                                            qpcDcTrustedRejectMadTotal;

                                        g_qpcDcTrustedRejectCandidateDeviationTotal =
                                            (LONG)
                                            qpcDcTrustedRejectCandidateDeviationTotal;

                                        g_qpcDcTrustedRejectRawMedianDeviationTotal =
                                            (LONG)
                                            qpcDcTrustedRejectRawMedianDeviationTotal;


                                        g_qpcDcTrustedCurrentRejectMask =
                                            trustedCurrentRejectMask;

                                        g_qpcDcTrustedLastRejectMask =
                                            qpcDcTrustedLastRejectMask;

                                        g_qpcDcTrustedMaxBadStreak =
                                            (LONG)
                                            qpcDcTrustedMaxBadStreak;


                                        g_qpcDcTrustedWarmupToTrackTotal =
                                            (LONG)
                                            qpcDcTrustedWarmupToTrackTotal;

                                        g_qpcDcTrustedTrackToHoldTotal =
                                            (LONG)
                                            qpcDcTrustedTrackToHoldTotal;

                                        g_qpcDcTrustedHoldToTrackTotal =
                                            (LONG)
                                            qpcDcTrustedHoldToTrackTotal;

                                        g_qpcDcTrustedHoldToUntrustedTotal =
                                            (LONG)
                                            qpcDcTrustedHoldToUntrustedTotal;

                                        g_qpcDcTrustedUntrustedToTrackTotal =
                                            (LONG)
                                            qpcDcTrustedUntrustedToTrackTotal;


                                        g_qpcDcTrustedLastTransitionFrom =
                                            qpcDcTrustedLastTransitionFrom;

                                        g_qpcDcTrustedLastTransitionTo =
                                            qpcDcTrustedLastTransitionTo;

                                        g_qpcDcTrustedLastTransitionRawPpb =
                                            (LONGLONG)
                                            qpcDcTrustedLastTransitionRawPpb;

                                        g_qpcDcTrustedLastTransitionMedianPpb =
                                            (LONGLONG)
                                            qpcDcTrustedLastTransitionMedianPpb;

                                        g_qpcDcTrustedLastTransitionMadPpb =
                                            (LONGLONG)
                                            qpcDcTrustedLastTransitionMadPpb;

                                        g_qpcDcTrustedLastTransitionTrustedPpb =
                                            (LONGLONG)
                                            qpcDcTrustedLastTransitionTrustedPpb;


                                        MemoryBarrier();


                                        InterlockedIncrement(
                                            &g_qpcDcTrustedReasonDiagSequence);


                                        // =====================================================
                                        // Publish Trusted Drift V1 snapshot.
                                        // =====================================================

                                        InterlockedIncrement(
                                            &g_qpcDcTrustedDiagSequence);


                                        g_qpcDcTrustedRawDriftPpb =
                                            (LONGLONG)
                                            driftPpb;

                                        g_qpcDcTrustedRobustMedianPpb =
                                            (LONGLONG)
                                            robustMedianPpb;

                                        g_qpcDcTrustedRobustMadPpb =
                                            (LONGLONG)
                                            robustMadPpb;

                                        g_qpcDcTrustedDriftPpb =
                                            (LONGLONG)
                                            qpcDcTrustedDriftPpb;

                                        g_qpcDcTrustedCandidateDeviationPpb =
                                            (LONGLONG)
                                            trustedCandidateDeviationPpb;

                                        g_qpcDcTrustedRawMedianDeviationPpb =
                                            (LONGLONG)
                                            trustedRawMedianDeviationPpb;

                                        g_qpcDcTrustedSlewAppliedPpb =
                                            (LONGLONG)
                                            trustedSlewAppliedPpb;

                                        g_qpcDcTrustedPeriodFfPs =
                                            (LONGLONG)
                                            trustedPeriodFfPs;

                                        g_qpcDcTrustedCandidateGood =
                                            trustedCandidateGood
                                            ? 1L
                                            : 0L;

                                        g_qpcDcTrustedValid =
                                            qpcDcTrustedValid
                                            ? 1L
                                            : 0L;

                                        g_qpcDcTrustedState =
                                            qpcDcTrustedState;

                                        g_qpcDcTrustedWarmupGoodCount =
                                            (LONG)
                                            qpcDcTrustedWarmupGoodCount;

                                        g_qpcDcTrustedBadCount =
                                            (LONG)
                                            qpcDcTrustedBadCount;

                                        g_qpcDcTrustedRecoveryGoodCount =
                                            (LONG)
                                            qpcDcTrustedRecoveryGoodCount;

                                        g_qpcDcTrustedUpdateTotal =
                                            (LONG)
                                            qpcDcTrustedUpdateTotal;

                                        g_qpcDcTrustedHoldTotal =
                                            (LONG)
                                            qpcDcTrustedHoldTotal;

                                        g_qpcDcTrustedUnlockTotal =
                                            (LONG)
                                            qpcDcTrustedUnlockTotal;

                                        g_qpcDcTrustedRelockTotal =
                                            (LONG)
                                            qpcDcTrustedRelockTotal;


                                        MemoryBarrier();


                                        InterlockedIncrement(
                                            &g_qpcDcTrustedDiagSequence);


                                        InterlockedIncrement(
                                            &g_qpcDcRobustDiagSequence);


                                        g_qpcDcRobustRawDriftPpb =
                                            (LONGLONG)
                                            driftPpb;

                                        g_qpcDcRobustMedianDriftPpb =
                                            (LONGLONG)
                                            robustMedianPpb;

                                        g_qpcDcRobustMadPpb =
                                            (LONGLONG)
                                            robustMadPpb;

                                        g_qpcDcRobustPeriodFfPs =
                                            (LONGLONG)
                                            robustPeriodFfPs;

                                        g_qpcDcRobustBufferCount =
                                            (LONG)
                                            qpcDcRobustRingCount;

                                        g_qpcDcRobustCurrentAccepted =
                                            robustCurrentAccepted
                                            ? 1L
                                            : 0L;

                                        g_qpcDcRobustLocked =
                                            robustLocked
                                            ? 1L
                                            : 0L;

                                        g_qpcDcRobustAcceptedTotal =
                                            (LONG)
                                            qpcDcRobustAcceptedTotal;

                                        g_qpcDcRobustRejectedTotal =
                                            (LONG)
                                            qpcDcRobustRejectedTotal;


                                        MemoryBarrier();


                                        InterlockedIncrement(
                                            &g_qpcDcRobustDiagSequence);
                                    }


                                    // =====================================================
                                    // Publish Raw QPC<->DC Snapshot
                                    // =====================================================

                                    if (windowValid)
                                    {
                                        InterlockedIncrement(
                                            &g_qpcDcDiagSequence);


                                        g_qpcDcQpcElapsedNs =
                                            (LONGLONG)
                                            qpcElapsedNs;


                                        g_qpcDcDcElapsedNs =
                                            (LONGLONG)
                                            dcElapsedNs;


                                        g_qpcDcDeltaNs =
                                            (LONGLONG)
                                            deltaNs;


                                        g_qpcDcDriftPpb =
                                            (LONGLONG)
                                            driftPpb;


                                        g_qpcDcRttAvgNs =
                                            (LONGLONG)
                                            (
                                                qpcDcRttSumNs /
                                                qpcDcWindowValidSamples
                                                );


                                        g_qpcDcRttMinNs =
                                            (LONGLONG)
                                            qpcDcRttMinNs;


                                        g_qpcDcRttMaxNs =
                                            (LONGLONG)
                                            qpcDcRttMaxNs;


                                        g_qpcDcValidSamples =
                                            (LONG)
                                            qpcDcWindowValidSamples;


                                        g_qpcDcRejectedSamples =
                                            (LONG)
                                            qpcDcWindowRejectedSamples;


                                        g_qpcDcValid =
                                            1L;


                                        MemoryBarrier();


                                        InterlockedIncrement(
                                            &g_qpcDcDiagSequence);
                                    }


                                    // =====================================================
                                    // Reset Window
                                    // =====================================================

                                    qpcDcWindowStartMidCount =
                                        0;

                                    qpcDcWindowEndMidCount =
                                        0;

                                    qpcDcWindowStartDcNs =
                                        0;

                                    qpcDcWindowEndDcNs =
                                        0;


                                    qpcDcRttSumNs =
                                        0;

                                    qpcDcRttMinNs =
                                        0;

                                    qpcDcRttMaxNs =
                                        0;


                                    qpcDcWindowValidSamples =
                                        0;

                                    qpcDcWindowRejectedSamples =
                                        0;
                                }
                            }
                            else
                            {
                                qpcDcWindowRejectedSamples++;
                            }


                            uint64_t combinedCommEndNs =
                                pMaster->GetCurrentMasterTimeNs();

                            // =============================================================
                        // PDO Fine Scheduler Timing Statistics V1
                        // =============================================================

                            static uint64_t previousWakeQpc =
                                0;


                            // QPC Handler Interval
                            static uint64_t qpcIntervalSumNs =
                                0;

                            static uint64_t qpcIntervalMinNs =
                                0;

                            static uint64_t qpcIntervalMaxNs =
                                0;


                            // Wake -> EtherCAT Call
                            static uint64_t wakeToEcatSumNs =
                                0;

                            static uint64_t wakeToEcatMinNs =
                                0;

                            static uint64_t wakeToEcatMaxNs =
                                0;


                            // QPC Read Cost
                            static uint64_t qpcReadSumNs =
                                0;

                            static uint64_t qpcReadMaxNs =
                                0;


                            static uint32_t qpcFineSamples =
                                0;


                            // -------------------------------------------------------------
                            // Only accept complete sample
                            // -------------------------------------------------------------

                            if (qpcWakeValid &&
                                qpcBeforeEcatValid &&
                                qpcFrequency > 0)
                            {
                                uint64_t wakeToEcatCounts =
                                    (uint64_t)
                                    (
                                        qpcBeforeEcat.QuadPart -
                                        qpcWake.QuadPart
                                        );


                                uint64_t wakeToEcatNs =
                                    (
                                        wakeToEcatCounts *
                                        1000000000ULL
                                        )
                                    /
                                    qpcFrequency;


                                uint64_t qpcIntervalNs =
                                    0;

                                bool intervalValid =
                                    false;


                                if (previousWakeQpc != 0 &&
                                    (uint64_t)
                                    qpcWake.QuadPart >=
                                    previousWakeQpc)
                                {
                                    uint64_t intervalCounts =
                                        (uint64_t)
                                        qpcWake.QuadPart -
                                        previousWakeQpc;


                                    qpcIntervalNs =
                                        (
                                            intervalCounts *
                                            1000000000ULL
                                            )
                                        /
                                        qpcFrequency;


                                    intervalValid =
                                        true;
                                }


                                previousWakeQpc =
                                    (uint64_t)
                                    qpcWake.QuadPart;


                                if (intervalValid)
                                {
                                    if (qpcFineSamples == 0)
                                    {
                                        qpcIntervalMinNs =
                                            qpcIntervalNs;

                                        qpcIntervalMaxNs =
                                            qpcIntervalNs;

                                        wakeToEcatMinNs =
                                            wakeToEcatNs;

                                        wakeToEcatMaxNs =
                                            wakeToEcatNs;
                                    }


                                    if (qpcIntervalNs <
                                        qpcIntervalMinNs)
                                    {
                                        qpcIntervalMinNs =
                                            qpcIntervalNs;
                                    }


                                    if (qpcIntervalNs >
                                        qpcIntervalMaxNs)
                                    {
                                        qpcIntervalMaxNs =
                                            qpcIntervalNs;
                                    }


                                    if (wakeToEcatNs <
                                        wakeToEcatMinNs)
                                    {
                                        wakeToEcatMinNs =
                                            wakeToEcatNs;
                                    }


                                    if (wakeToEcatNs >
                                        wakeToEcatMaxNs)
                                    {
                                        wakeToEcatMaxNs =
                                            wakeToEcatNs;
                                    }


                                    if (qpcReadCostNs >
                                        qpcReadMaxNs)
                                    {
                                        qpcReadMaxNs =
                                            qpcReadCostNs;
                                    }


                                    qpcIntervalSumNs +=
                                        qpcIntervalNs;

                                    wakeToEcatSumNs +=
                                        wakeToEcatNs;

                                    qpcReadSumNs +=
                                        qpcReadCostNs;


                                    qpcFineSamples++;


                                    // =====================================================
                                    // 4000 cycles ≈ 1 second
                                    // =====================================================

                                    if (qpcFineSamples >=
                                        4000U)
                                    {
                                        uint64_t intervalAvgNs =
                                            qpcIntervalSumNs /
                                            qpcFineSamples;


                                        uint64_t wakeToEcatAvgNs =
                                            wakeToEcatSumNs /
                                            qpcFineSamples;


                                        uint64_t qpcReadAvgNs =
                                            qpcReadSumNs /
                                            qpcFineSamples;


                                        // -------------------------------------------------
                                        // Seqlock-style publish
                                        // -------------------------------------------------

                                        InterlockedIncrement(
                                            &g_pdoFineDiagSequence);


                                        g_pdoFineQpcFrequency =
                                            (LONGLONG)
                                            qpcFrequency;


                                        g_pdoFineIntervalAvgNs =
                                            (LONGLONG)
                                            intervalAvgNs;

                                        g_pdoFineIntervalMinNs =
                                            (LONGLONG)
                                            qpcIntervalMinNs;

                                        g_pdoFineIntervalMaxNs =
                                            (LONGLONG)
                                            qpcIntervalMaxNs;


                                        g_pdoFineWakeToEcatAvgNs =
                                            (LONGLONG)
                                            wakeToEcatAvgNs;

                                        g_pdoFineWakeToEcatMinNs =
                                            (LONGLONG)
                                            wakeToEcatMinNs;

                                        g_pdoFineWakeToEcatMaxNs =
                                            (LONGLONG)
                                            wakeToEcatMaxNs;


                                        g_pdoFineQpcReadAvgNs =
                                            (LONGLONG)
                                            qpcReadAvgNs;

                                        g_pdoFineQpcReadMaxNs =
                                            (LONGLONG)
                                            qpcReadMaxNs;


                                        g_pdoFineQpcValid =
                                            1L;


                                        MemoryBarrier();


                                        InterlockedIncrement(
                                            &g_pdoFineDiagSequence);


                                        // -------------------------------------------------
                                        // Reset window
                                        // -------------------------------------------------

                                        qpcIntervalSumNs =
                                            0;

                                        qpcIntervalMinNs =
                                            0;

                                        qpcIntervalMaxNs =
                                            0;


                                        wakeToEcatSumNs =
                                            0;

                                        wakeToEcatMinNs =
                                            0;

                                        wakeToEcatMaxNs =
                                            0;


                                        qpcReadSumNs =
                                            0;

                                        qpcReadMaxNs =
                                            0;


                                        qpcFineSamples =
                                            0;
                                    }
                                }
                            }
                            uint64_t combinedCommNs =
                                0;


                            if (combinedCommEndNs >=
                                combinedCommStartNs)
                            {
                                combinedCommNs =
                                    combinedCommEndNs -
                                    combinedCommStartNs;
                            }


                            pMaster->wkc_PDO =
                                wkc;


                            // =========================================================
                            // Stage 12B.1B - Owner-safe RT Process Image snapshot
                            //
                            // This is the only place where Online Diagnosis copies the
                            // live m_IoMap. It executes in the PDO owner thread and only
                            // once per 40 cycles (10 ms at 250 us). No Shared Memory,
                            // formatting or UI work is performed here.
                            // =========================================================

                            CaptureEtherCatDiagRtShadow(
                                pMaster,
                                wkc,
                                dcWkc,
                                processDataValid);


                            // =========================================================
                            // PLC INPUT：EtherCAT IO Map -> Shadow Input
                            //
                            // 只有 LRW Process Data 有效才更新 shadow input，避免應用層讀到
                            // timeout／WKC 錯誤週期的半成品。PDO Handler 是 IO Map 唯一 Owner。
                            // =========================================================

                            if (processDataValid)
                            {
                                pMaster->m_Plc.FetchInputs();
                            }


                            // =========================================================
                            // EtherCAT DC Estimator / Controller（目前啟用）
                            // =========================================================

                            if (dcReferenceSlaveIndex >= 0)
                            {
                                pMaster->wk_read =
                                    dcWkc;


                                // =========================================================
                                // DC Software Estimator / Phase Controller
                                //
                                // 目前 enableDcSoftwareDiagnostics=true，因此有效 DC transport
                                // 週期會更新 Master/DC estimator 與 PDO phase controller。
                                // 兩個函式必須維持無高頻 RtPrintf 的即時安全版本。
                                // 即使日後關閉此 gate，LRW+FRMW 與從站 DC/Sync0 仍會運作；
                                // 關閉的只會是主站軟體估測與 phase controller 更新。
                                // =========================================================

                                const bool enableDcSoftwareDiagnostics =
                                    true;


                                if (dcTransportValid &&
                                    enableDcSoftwareDiagnostics)
                                {
                                    pMaster->
                                        UpdateDCMasterClockEstimator(
                                            combinedCommStartNs,
                                            combinedCommEndNs,
                                            pMaster->DC_reference_time,
                                            dcWkc);


                                    pMaster->
                                        UpdateDCPdoPhaseController(
                                            pdoCycleStartMasterNs);
                                }



                            }


                            // =========================================================
                            // 額外逐站 DC register 診斷（目前固定 Disable）
                            //
                            // 0x092C/0x0928 讀取會增加額外 EtherCAT frame，因此正式
                            // 250 us 路徑保持 false。需要量測時應在停機診斷版本另行開啟。
                            // =========================================================

                            static size_t dcDiagServoIndex =
                                0;


                            if (false &&
                                (pMaster->tickCount_PDO %
                                    4000ULL) == 0 &&
                                !pMaster->
                                m_ServoList.empty())
                            {
                                if (dcDiagServoIndex >=
                                    pMaster->
                                    m_ServoList.size())
                                {
                                    dcDiagServoIndex =
                                        0;
                                }


                                ENI_ServoDrive& servo =
                                    pMaster->
                                    m_ServoList[
                                        dcDiagServoIndex
                                    ];


                                int slaveIndex =
                                    servo.slaveIndex;


                                uint32_t rawDcDifference =
                                    0;


                                int dcDiagWkc =
                                    pMaster->ecx_FPRD(
                                        m_slaveInfo[
                                            slaveIndex
                                        ].configAddr,
                                        0x092C,
                                                &rawDcDifference,
                                                4,
                                                1);


                                if (dcDiagWkc > 0)
                                {
                                    uint32_t magnitude =
                                        rawDcDifference &
                                        0x7FFFFFFF;


                                    int32_t dcDifferenceNs =
                                        0;


                                    if ((rawDcDifference &
                                        0x80000000) != 0)
                                    {
                                        dcDifferenceNs =
                                            -(int32_t)
                                            magnitude;
                                    }
                                    else
                                    {
                                        dcDifferenceNs =
                                            (int32_t)
                                            magnitude;
                                    }


                                    uint32_t dcPropagationDelay =
                                        0;


                                    int delayWkc =
                                        pMaster->ecx_FPRD(
                                            m_slaveInfo[
                                                slaveIndex
                                            ].configAddr,
                                            0x0928,
                                                    &dcPropagationDelay,
                                                    4,
                                                    1);


                                    PublishP64DeferredDiagnostic(
                                        EtherCatP64DeferredDiagKind::DcSlaveSample,
                                        static_cast<uint64_t>(pMaster->tickCount_PDO),
                                        static_cast<int64_t>(dcDiagServoIndex),
                                        static_cast<int64_t>(slaveIndex),
                                        static_cast<int64_t>(dcDifferenceNs),
                                        static_cast<int64_t>(dcPropagationDelay),
                                        static_cast<int64_t>(delayWkc),
                                        static_cast<int64_t>(rawDcDifference),
                                        0LL,
                                        0LL);
                                }
                                else
                                {
                                    PublishP64DeferredDiagnostic(
                                        EtherCatP64DeferredDiagKind::DcSlaveReadFailed,
                                        static_cast<uint64_t>(pMaster->tickCount_PDO),
                                        static_cast<int64_t>(dcDiagServoIndex),
                                        static_cast<int64_t>(slaveIndex),
                                        0LL,
                                        0LL,
                                        0LL,
                                        0LL,
                                        0LL,
                                        0LL);
                                }


                                dcDiagServoIndex++;
                            }


                            // =========================================================
                            // Async EtherCAT Command
                            //
                            // 每 4 個 PDO cycle 的 subTick 2 最多處理一筆 pending 命令，
                            // 避免每個 250 us 週期都加入非循環存取。完成後寫 resultWKC/DONE。
                            // SDO timeout 值是既有行為；若要調整，必須另測 Handler deadline。
                            // =========================================================

                            int subTick =
                                (int)(
                                    pMaster->
                                    tickCount_PDO %
                                    4ULL);


                            // =========================================================
                            // Stage 11G.6 - State Transition Command Decoupling
                            //
                            // CMD_SET_STATE 必須能在 PDO WKC 尚未完整有效時執行。
                            // 典型情境是 SAFE-OP -> OP：部分從站的 Output SM 在
                            // OP 前可能尚未貢獻完整 LRW WKC。若把進 OP 指令本身
                            // 綁在 processDataValid，會形成死結：
                            //
                            //   PDO WKC 未完整 -> 不送 OP -> 永遠無法進 OP
                            //
                            // 安全策略：
                            // - CMD_SET_STATE：允許在 processDataValid == false 時執行。
                            // - 其他 Async Command（例如 SDO）：仍要求 PDO cycle valid。
                            // - 仍只在 subTick == 2 處理，維持既有 NIC 單一執行路徑。
                            // =========================================================

                            const bool asyncCmdPending =
                                pMaster->m_asyncCmd.status ==
                                (int)EcatCmdStatus::ECAT_STATUS_PENDING;

                            const bool stateTransitionPending =
                                asyncCmdPending &&
                                pMaster->m_asyncCmd.type ==
                                (int)EcatCmdType::CMD_SET_STATE;

                            const bool asyncCmdAllowed =
                                processDataValid ||
                                stateTransitionPending;

                            if (subTick == 2 &&
                                asyncCmdPending &&
                                asyncCmdAllowed)
                            {
                                int cmdWKC = 0;
                                bool commandTerminal = true;
                                bool commandError = false;

                                if (!processDataValid &&
                                    stateTransitionPending)
                                {
                                    PublishP64DeferredDiagnostic(
                                        EtherCatP64DeferredDiagKind::AsyncStateWithoutFullPdoWkc,
                                        static_cast<uint64_t>(pMaster->tickCount_PDO),
                                        static_cast<int64_t>(
                                            static_cast<uint16_t>(
                                                pMaster->m_asyncCmd.dataValue)),
                                        static_cast<int64_t>(wkc),
                                        static_cast<int64_t>(
                                            pMaster->EXPECTED_WKC_PDO),
                                        static_cast<int64_t>(dcWkc),
                                        0LL,
                                        0LL,
                                        0LL,
                                        0LL);
                                }

                                switch (pMaster->m_asyncCmd.type)
                                {
                                case (int)EcatCmdType::CMD_SET_STATE:
                                {
                                    state =
                                        (uint16_t)pMaster->m_asyncCmd.dataValue;

                                    cmdWKC =
                                        pMaster->ecx_BWR(
                                            0x0000,
                                            0x0120,
                                            2,
                                            &state,
                                            20);

                                    break;
                                }

                                // =================================================
                                // Stage 12F.3E.1 - Runtime-Safe Online SDO
                                //
                                // IMPORTANT:
                                //     Do NOT call ecx_SDOread/ecx_SDOwrite here.
                                //     Those legacy helpers are intentionally blocking
                                //     and remain startup/configuration-only.
                                //
                                // The state machine executes at most one bounded
                                // FPWR/FPRD step on this 1 ms async slot, then returns
                                // immediately to the 250 us cyclic runtime.
                                // =================================================
                                case (int)EcatCmdType::CMD_SDO_READ:
                                case (int)EcatCmdType::CMD_SDO_WRITE:
                                {
                                    int terminalSdoWkc = 0;

                                    const RuntimeSafeSdoStepDisposition disposition =
                                        ProcessRuntimeSafeSdoStep(
                                            pMaster,
                                            pdoCycleStartMasterNs,
                                            processDataValid,
                                            &terminalSdoWkc);

                                    if (disposition ==
                                        RuntimeSafeSdoStepDisposition::InProgress)
                                    {
                                        commandTerminal = false;
                                    }
                                    else if (disposition ==
                                        RuntimeSafeSdoStepDisposition::Done)
                                    {
                                        cmdWKC = terminalSdoWkc;
                                    }
                                    else
                                    {
                                        cmdWKC = 0;
                                        commandError = true;
                                    }

                                    break;
                                }

                                default:
                                {
                                    cmdWKC = 0;
                                    commandError = true;
                                    break;
                                }
                                }

                                if (commandTerminal)
                                {
                                    pMaster->m_asyncCmd.resultWKC = cmdWKC;

                                    MemoryBarrier();

                                    pMaster->m_asyncCmd.status =
                                        commandError
                                        ? (int)EcatCmdStatus::ECAT_STATUS_ERROR
                                        : (int)EcatCmdStatus::ECAT_STATUS_DONE;
                                }
                            }


                            // =========================================================
                            // Stage 12E.3A - owner-safe ESC live diagnostic probe
                            //
                            // One bounded FPRD at most every 20 ms, only on subTick 0.
                            // SDO/state commands keep priority and run on subTick 2.
                            // =========================================================
                            ProcessRuntimeEscDiagProbe(
                                pMaster,
                                pdoCycleStartMasterNs,
                                processDataValid);

                            // =========================================================
                            // PDO 通訊結果分類：wkc<0 算 timeout；frame 有回覆但 LRW
                            // WKC 不符合才算 PDO wkc_error。FRMW DC WKC 由 DC-RX.3A
                            // 獨立計數，不再把 DC-only 抖動偽裝成 PDO instability。
                            // =========================================================

                            if (wkc < 0)
                            {
                                pMaster->
                                    timeout_count_PDO++;
                            }
                            else if (!pdoWkcValid)
                            {
                                pMaster->
                                    wkc_error_count_PDO++;
                            }


                            // =========================================================
                            // PDO Tick：不論本週期有效與否都遞增，供 slot 與統計視窗使用。
                            // =========================================================

                            pMaster->tickCount_PDO++;

                            // NC-0.2J.5 / K.6.2: publish every PDO-cycle
                            // validity edge to the Motion settle producer
                            // before any interpolation work.  The invalid
                            // first-cycle path intentionally skips Motion, so
                            // ObserveNCSettleRuntimeCycle() also revokes stale
                            // drain proof immediately for that case.
                            pMaster->
                                m_Motion.
                                ObserveNCSettleRuntimeCycle(
                                    pMaster->tickCount_PDO,
                                    processDataValid);


                            // =========================================================
                            // Motion
                            //
                            // 有效週期才更新插補與軸控制。無效週期 1..7 不採用 Input、
                            // 不更新插補／Motion；偵測到第一次無效後，下一 callback 起
                            // 也不再 Flush 新 PLC output。連續第 8 個無效週期才執行
                            // EmergencyStopAllAxes 並由下方發布 AL1003。
                            // =========================================================

                            uint64_t stageMotionStartNs =
                                pMaster->
                                GetCurrentMasterTimeNs();

                            bool pdoSafetyStopAppliedThisCycle =
                                false;


                            if (processDataValid)
                            {
                                pMaster->
                                    m_Motion.
                                    UpdateInterpolation();


                                pMaster->
                                    m_Motion.
                                    UpdateAllMotion();
                            }
                            else if (
                                EtherCatPdoSafetyStopDebounce::
                                IsContainmentRequired(
                                    pdoConsecutiveInvalidCycles))
                            {
                                pMaster->
                                    m_Motion.
                                    EmergencyStopAllAxes();

                                pdoSafetyStopAppliedThisCycle =
                                    true;


                                pMaster->
                                    m_Motion.
                                    UpdateAllMotion();
                            }


                            uint64_t stageMotionEndNs =
                                pMaster->
                                GetCurrentMasterTimeNs();


                            uint64_t motionNs =
                                0;


                            if (stageMotionEndNs >=
                                stageMotionStartNs)
                            {
                                motionNs =
                                    stageMotionEndNs -
                                    stageMotionStartNs;
                            }

                            // NC-0.2K.7.2.1: latch exact source values only
                            // after the existing Motion validity observer and
                            // this cycle's interpolation / Emergency Stop work
                            // have completed.  The diagnostic therefore cannot
                            // delay a safety action and is excluded from the
                            // existing motionNs measurement above.
                            ObservePdoRuntimeInvalidCorrelation(
                                pMaster->tickCount_PDO,
                                processDataValid,
                                static_cast<std::int32_t>(wkc),
                                static_cast<std::int32_t>(
                                    pMaster->EXPECTED_WKC_PDO),
                                static_cast<std::int32_t>(dcWkc),
                                dcReferenceSlaveIndex >= 0,
                                combinedCommNs,
                                static_cast<std::uint64_t>(
                                    pMaster->timeout_count_PDO),
                                static_cast<std::uint64_t>(
                                    pMaster->wkc_error_count_PDO),
                                static_cast<std::uint32_t>(subTick));

                            // NC-0.2K.7.2.2: the existing safety action above
                            // remains first.  Only after ESTOP and K.7.2.1
                            // source capture do we publish the small cause
                            // latch and bridge it into AlarmManager.  RESET
                            // clearing Alarm during a sustained PDO fault is
                            // detected by HasAlarm() and immediately re-latched.
                            if (pdoSafetyStopAppliedThisCycle)
                            {
                                AlarmManager& alarmManager =
                                    AlarmManager::GetInstance();
                                const std::int32_t alarmCode =
                                    static_cast<std::int32_t>(
                                        AlarmManager::
                                        ETHERCAT_PDO_SAFETY_STOP);

                                if (g_pdoSafetyStopAlarmBridgeTracker.
                                    ObserveSafetyContainment(
                                        alarmManager.HasAlarm(),
                                        alarmCode,
                                        g_pdoInvalidCorrelationTracker.
                                        Snapshot(),
                                        pdoConsecutiveInvalidCycles,
                                        dcReferenceSlaveIndex >= 0,
                                        static_cast<std::uint64_t>(
                                            pMaster->timeout_count_PDO),
                                        static_cast<std::uint64_t>(
                                            pMaster->wkc_error_count_PDO),
                                        static_cast<std::uint32_t>(subTick)))
                                {
                                    // Cause publication is intentionally
                                    // visible before the Alarm image.
                                    PublishPdoSafetyStopCauseSnapshot(
                                        g_pdoSafetyStopAlarmBridgeTracker.
                                        Snapshot());
                                    alarmManager.Trigger(
                                        alarmCode,
                                        0,
                                        -1);
                                }
                            }


                            // =========================================================
                            // DC-RX.3F deterministic fault-injection acceptance engine
                            //
                            // This block runs after the existing Motion/AL1003 safety
                            // boundary. It never changes a production decision; it only
                            // records whether the already-existing DC/PDO containment and
                            // recovery paths reacted exactly as designed.
                            // =========================================================

                            if (dcRx3fScenario != DcRx3fFaultScenario::Off &&
                                dcRx3fState != DcRx3fFaultState::Off)
                            {
                                const bool dcRx3fLrwScenario =
                                    dcRx3fScenario ==
                                    DcRx3fFaultScenario::LrwWkcDrop ||
                                    dcRx3fScenario ==
                                    DcRx3fFaultScenario::LrwTimeout;

                                const bool dcRx3fDestructiveLrwScenario =
                                    dcRx3fLrwScenario &&
                                    dcRx3fConfiguredCycles >= 8U &&
                                    dcRx3fAllowSafetyStop;

                                if (pdoConsecutiveInvalidCycles >
                                    dcRx3fMaximumPdoInvalidStreak)
                                {
                                    dcRx3fMaximumPdoInvalidStreak =
                                        pdoConsecutiveInvalidCycles;
                                }

                                int64_t dcRx3fAppliedDeltaPpb =
                                    realFfV0AppliedPpb -
                                    dcRx3fBaselineAppliedPpb;

                                if (dcRx3fAppliedDeltaPpb < 0)
                                {
                                    dcRx3fAppliedDeltaPpb =
                                        -dcRx3fAppliedDeltaPpb;
                                }

                                if (dcRx3fAppliedDeltaPpb >
                                    dcRx3fMaximumAppliedDeltaPpb)
                                {
                                    dcRx3fMaximumAppliedDeltaPpb =
                                        dcRx3fAppliedDeltaPpb;
                                }

                                const bool dcRx3fTestActive =
                                    dcRx3fState == DcRx3fFaultState::Inject ||
                                    dcRx3fState == DcRx3fFaultState::Recovery;

                                if (dcRx3fInjectionRequestedThisCycle)
                                {
                                    if (dcRx3fInjectionAppliedThisCycle)
                                    {
                                        if (dcRx3fAppliedCycles < 0xFFFFFFFFU)
                                        {
                                            dcRx3fAppliedCycles++;
                                        }

                                        dcRx3fLastAppliedTick =
                                            pMaster->tickCount_PDO;
                                        dcRx3fEvidenceMask |=
                                            DC_RX3F_EVIDENCE_APPLIED;

                                        switch (dcRx3fScenario)
                                        {
                                        case DcRx3fFaultScenario::DcWkcDrop:
                                            if (!dcWkcValid && dcOnlyInvalid)
                                            {
                                                dcRx3fEvidenceMask |=
                                                    DC_RX3F_EVIDENCE_DOWNSTREAM;
                                            }
                                            break;

                                        case DcRx3fFaultScenario::LrwWkcDrop:
                                            if (!processDataValid && wkc >= 0)
                                            {
                                                dcRx3fEvidenceMask |=
                                                    DC_RX3F_EVIDENCE_DOWNSTREAM;
                                            }
                                            break;

                                        case DcRx3fFaultScenario::LrwTimeout:
                                            if (!processDataValid && wkc < 0)
                                            {
                                                dcRx3fEvidenceMask |=
                                                    DC_RX3F_EVIDENCE_DOWNSTREAM;
                                            }
                                            break;

                                        case DcRx3fFaultScenario::ExactTimingMissing:
                                            if (!dcControlSampleValid &&
                                                (dcSampleGuardReasonMask &
                                                    DC_RX3D_SAMPLE_REJECT_TIMING_SOURCE) != 0)
                                            {
                                                dcRx3fEvidenceMask |=
                                                    DC_RX3F_EVIDENCE_DOWNSTREAM;
                                            }
                                            break;

                                        case DcRx3fFaultScenario::LateRtt:
                                            if (dcRttGuardEvaluatedThisCycle &&
                                                !dcRttGuardAcceptedThisCycle &&
                                                !dcControlSampleValid &&
                                                (dcSampleGuardReasonMask &
                                                    DC_RX3E_SAMPLE_REJECT_RTT_ENVELOPE) != 0)
                                            {
                                                dcRx3fEvidenceMask |=
                                                    DC_RX3F_EVIDENCE_DOWNSTREAM;
                                            }
                                            break;

                                        case DcRx3fFaultScenario::DcTimestampRepeat:
                                        case DcRx3fFaultScenario::DcTimestampBackward:
                                            if (!dcControlSampleValid &&
                                                (dcSampleGuardReasonMask &
                                                    DC_RX3C_SAMPLE_REJECT_DC_ORDER) != 0)
                                            {
                                                dcRx3fEvidenceMask |=
                                                    DC_RX3F_EVIDENCE_DOWNSTREAM;
                                            }
                                            break;

                                        case DcRx3fFaultScenario::PhaseMapJump:
                                            if (phasePRecoveryJumpGuardActive)
                                            {
                                                dcRx3fEvidenceMask |=
                                                    DC_RX3F_EVIDENCE_DOWNSTREAM;
                                            }
                                            break;

                                        case DcRx3fFaultScenario::SchedulerRecovery:
                                            if (currentSchedulerRuntimeRecovery)
                                            {
                                                dcRx3fEvidenceMask |=
                                                    DC_RX3F_EVIDENCE_DOWNSTREAM;
                                            }
                                            break;

                                        default:
                                            break;
                                        }

                                        if (!dcRx3fLrwScenario &&
                                            processDataValid)
                                        {
                                            dcRx3fEvidenceMask |=
                                                DC_RX3F_EVIDENCE_PDO_PRESERVED;
                                        }

                                        if (!dcRx3fLrwScenario &&
                                            (dcOnlyGraceAcceptedThisCycle ||
                                                realFfV0State == 3 ||
                                                phasePActV0State == 3 ||
                                                realFfV0TransientHoldReasonMask != 0 ||
                                                phasePRecoveryJumpGuardActive ||
                                                currentSchedulerRuntimeRecovery))
                                        {
                                            dcRx3fEvidenceMask |=
                                                DC_RX3F_EVIDENCE_HOLD_OR_GRACE;
                                        }

                                        if (dcRx3fAppliedCycles >=
                                            dcRx3fConfiguredCycles)
                                        {
                                            dcRx3fState =
                                                DcRx3fFaultState::Recovery;
                                            dcRx3fRecoveryCycles = 0;
                                            dcRx3fRecoveryStableCycles = 0;
                                        }
                                    }
                                    else
                                    {
                                        if (dcRx3fTargetWaitCycles < 0xFFFFFFFFU)
                                        {
                                            dcRx3fTargetWaitCycles++;
                                        }

                                        if (dcRx3fTargetWaitCycles >=
                                            DC_RX3F_TARGET_WAIT_LIMIT_CYCLES)
                                        {
                                            dcRx3fFailureMask |=
                                                DC_RX3F_FAIL_TARGET_TIMEOUT;
                                        }
                                    }
                                }

                                if (dcRx3fTestActive ||
                                    dcRx3fState == DcRx3fFaultState::Recovery)
                                {
                                    if ((realFfV0TripMask &
                                        REAL_FF_V0_HARD_TRIP_MASK) != 0)
                                    {
                                        dcRx3fFailureMask |=
                                            DC_RX3F_FAIL_HARD_TRIP;
                                    }

                                    if (!dcRx3fDestructiveLrwScenario &&
                                        dcRx3fMaximumAppliedDeltaPpb >
                                        DC_RX3F_MAX_APPLIED_FF_DELTA_PPB)
                                    {
                                        dcRx3fFailureMask |=
                                            DC_RX3F_FAIL_FF_DISCONTINUITY;
                                    }

                                    if (!dcRx3fLrwScenario &&
                                        pdoConsecutiveInvalidCycles != 0)
                                    {
                                        dcRx3fFailureMask |=
                                            DC_RX3F_FAIL_UNEXPECTED_PDO_INVALID;
                                    }

                                    if (pdoSafetyStopAppliedThisCycle &&
                                        !dcRx3fDestructiveLrwScenario)
                                    {
                                        dcRx3fFailureMask |=
                                            DC_RX3F_FAIL_UNEXPECTED_SAFETY_STOP;
                                    }
                                }

                                if (dcRx3fDestructiveLrwScenario &&
                                    dcRx3fState == DcRx3fFaultState::Recovery)
                                {
                                    if (pdoSafetyStopAppliedThisCycle)
                                    {
                                        dcRx3fEvidenceMask |=
                                            DC_RX3F_EVIDENCE_DOWNSTREAM |
                                            DC_RX3F_EVIDENCE_SAFETY_STOP;
                                        dcRx3fState = DcRx3fFaultState::Pass;
                                        dcRx3fEndTick = pMaster->tickCount_PDO;
                                    }
                                    else
                                    {
                                        dcRx3fFailureMask |=
                                            DC_RX3F_FAIL_MISSING_EVIDENCE;
                                    }
                                }
                                else if (dcRx3fState ==
                                    DcRx3fFaultState::Recovery)
                                {
                                    if (dcRx3fRecoveryCycles < 0xFFFFFFFFU)
                                    {
                                        dcRx3fRecoveryCycles++;
                                    }

                                    const bool dcRx3fFullyRecovered =
                                        realFfV0State == 2 &&
                                        phasePActV0State == 2 &&
                                        realFfV0TransientHoldReasonMask == 0 &&
                                        realFfV0TripMask == 0 &&
                                        realFfV0CycleQualityKnown &&
                                        realFfV0PreviousCycleClean &&
                                        realFfV0CleanCycleStreak >=
                                        realFfV0CleanCyclesRequired &&
                                        dcSampleGuardState ==
                                        DC_RX3C_SAMPLE_STATE_TRACK &&
                                        dcControlSampleValid &&
                                        dcExactTimingLocked &&
                                        dcRttGuardState ==
                                        DC_RX3E_RTT_STATE_TRACK &&
                                        !phasePRecoveryJumpGuardActive &&
                                        phasePActV0AcceptedPhaseMapValid &&
                                        phasePActV0DiagPhaseMapAgeGood &&
                                        currentSchedulerClean &&
                                        processDataValid &&
                                        pdoConsecutiveInvalidCycles == 0;

                                    if (dcRx3fFullyRecovered)
                                    {
                                        if (dcRx3fRecoveryStableCycles <
                                            DC_RX3F_RECOVERY_STABLE_CYCLES)
                                        {
                                            dcRx3fRecoveryStableCycles++;
                                        }
                                    }
                                    else
                                    {
                                        dcRx3fRecoveryStableCycles = 0;
                                    }

                                    if (dcRx3fRecoveryStableCycles >=
                                        DC_RX3F_RECOVERY_STABLE_CYCLES)
                                    {
                                        dcRx3fEvidenceMask |=
                                            DC_RX3F_EVIDENCE_RECOVERED;

                                        if (dcRx3fMaximumAppliedDeltaPpb <=
                                            DC_RX3F_MAX_APPLIED_FF_DELTA_PPB)
                                        {
                                            dcRx3fEvidenceMask |=
                                                DC_RX3F_EVIDENCE_FF_PRESERVED;
                                        }

                                        LONG dcRx3fRequiredEvidence =
                                            DC_RX3F_EVIDENCE_APPLIED |
                                            DC_RX3F_EVIDENCE_DOWNSTREAM |
                                            DC_RX3F_EVIDENCE_RECOVERED |
                                            DC_RX3F_EVIDENCE_FF_PRESERVED;

                                        if (!dcRx3fLrwScenario)
                                        {
                                            dcRx3fRequiredEvidence |=
                                                DC_RX3F_EVIDENCE_PDO_PRESERVED |
                                                DC_RX3F_EVIDENCE_HOLD_OR_GRACE;
                                        }

                                        const bool dcRx3fLrwStreakProved =
                                            !dcRx3fLrwScenario ||
                                            dcRx3fMaximumPdoInvalidStreak >=
                                            dcRx3fConfiguredCycles;

                                        if ((dcRx3fEvidenceMask &
                                            dcRx3fRequiredEvidence) ==
                                            dcRx3fRequiredEvidence &&
                                            dcRx3fLrwStreakProved)
                                        {
                                            dcRx3fState =
                                                DcRx3fFaultState::Pass;
                                            dcRx3fEndTick =
                                                pMaster->tickCount_PDO;
                                        }
                                        else
                                        {
                                            dcRx3fFailureMask |=
                                                DC_RX3F_FAIL_MISSING_EVIDENCE;
                                        }
                                    }
                                    else if (dcRx3fRecoveryCycles >=
                                        DC_RX3F_RECOVERY_LIMIT_CYCLES)
                                    {
                                        dcRx3fFailureMask |=
                                            DC_RX3F_FAIL_RECOVERY_TIMEOUT;
                                    }
                                }

                                if (dcRx3fFailureMask != 0 &&
                                    dcRx3fState != DcRx3fFaultState::Pass &&
                                    dcRx3fState != DcRx3fFaultState::Fail)
                                {
                                    dcRx3fState = DcRx3fFaultState::Fail;
                                    dcRx3fEndTick = pMaster->tickCount_PDO;
                                }
                            }

                            // =========================================================
                            // LRW+FRMW round-trip 統計；4000 samples 約一秒形成快照。
                            // =========================================================

                            static uint64_t combinedSumNs =
                                0;


                            static uint64_t combinedMinNs =
                                0;


                            static uint64_t combinedMaxNs =
                                0;


                            static uint32_t combinedSamples =
                                0;
                            static uint64_t latestCombinedAvgNs =
                                0;

                            static uint64_t latestCombinedMinNs =
                                0;

                            static uint64_t latestCombinedMaxNs =
                                0;

                            static bool latestCombinedValid =
                                false;

                            if (combinedSamples == 0)
                            {
                                combinedMinNs =
                                    combinedCommNs;


                                combinedMaxNs =
                                    combinedCommNs;
                            }


                            if (combinedCommNs <
                                combinedMinNs)
                            {
                                combinedMinNs =
                                    combinedCommNs;
                            }


                            if (combinedCommNs >
                                combinedMaxNs)
                            {
                                combinedMaxNs =
                                    combinedCommNs;
                            }


                            combinedSumNs +=
                                combinedCommNs;


                            combinedSamples++;


                            if (combinedSamples >=
                                4000U)
                            {
                                uint64_t combinedAverageNs =
                                    combinedSumNs /
                                    (uint64_t)
                                    combinedSamples;


                                latestCombinedAvgNs =
                                    combinedAverageNs;

                                latestCombinedMinNs =
                                    combinedMinNs;

                                latestCombinedMaxNs =
                                    combinedMaxNs;

                                latestCombinedValid =
                                    true;


                                combinedSumNs =
                                    0;


                                combinedMinNs =
                                    0;


                                combinedMaxNs =
                                    0;


                                combinedSamples =
                                    0;
                            }


                            // =========================================================
                            // PDO Handler 總執行時間診斷
                            //
                            // Handler Start:
                            //
                            //     pdoCycleStartMasterNs
                            //
                            // Handler End:
                            //
                            //     現在
                            //
                            // =========================================================

                            uint64_t pdoHandlerEndNs =
                                pMaster->
                                GetCurrentMasterTimeNs();


                            if (pdoHandlerEndNs >=
                                pdoCycleStartMasterNs)
                            {
                                uint64_t executionNs =
                                    pdoHandlerEndNs -
                                    pdoCycleStartMasterNs;


                                static uint64_t execSumNs =
                                    0;


                                static uint64_t execMinNs =
                                    0;


                                static uint64_t execMaxNs =
                                    0;


                                static uint32_t execSamples =
                                    0;


                                static uint32_t execOver250Count =
                                    0;


                                static uint32_t execOver300Count =
                                    0;


                                static uint32_t execOver400Count =
                                    0;


                                // =====================================================
                                // Latest finished PDO Execution window
                                // =====================================================

                                static uint64_t latestExecAvgNs =
                                    0;

                                static uint64_t latestExecMinNs =
                                    0;

                                static uint64_t latestExecMaxNs =
                                    0;

                                static uint32_t latestExecOver250Count =
                                    0;

                                static uint32_t latestExecOver300Count =
                                    0;

                                static uint32_t latestExecOver400Count =
                                    0;

                                static bool latestExecValid =
                                    false;


                                // =====================================================
                                // Initialize Min / Max
                                // =====================================================

                                if (execSamples == 0)
                                {
                                    execMinNs =
                                        executionNs;


                                    execMaxNs =
                                        executionNs;
                                }


                                if (executionNs <
                                    execMinNs)
                                {
                                    execMinNs =
                                        executionNs;
                                }


                                if (executionNs >
                                    execMaxNs)
                                {
                                    execMaxNs =
                                        executionNs;
                                }


                                execSumNs +=
                                    executionNs;


                                execSamples++;


                                // =====================================================
                                // Deadline 統計：分別計算超過 250/300/400 us 的週期數。
                                // =====================================================

                                if (executionNs >
                                    250000ULL)
                                {
                                    execOver250Count++;
                                }


                                if (executionNs >
                                    300000ULL)
                                {
                                    execOver300Count++;
                                }


                                if (executionNs >
                                    400000ULL)
                                {
                                    execOver400Count++;
                                }


                                // =====================================================
                                // 每 4000 cycle 完成一個約一秒的 Exec 視窗；此處只保存數值。
                                // =====================================================

                                if (execSamples >=
                                    4000U)
                                {
                                    uint64_t execAvgNs =
                                        execSumNs /
                                        (uint64_t)
                                        execSamples;


                                    // =====================================================
                                    // Save completed EXEC window
                                    // =====================================================

                                    latestExecAvgNs =
                                        execAvgNs;

                                    latestExecMinNs =
                                        execMinNs;

                                    latestExecMaxNs =
                                        execMaxNs;

                                    latestExecOver250Count =
                                        execOver250Count;

                                    latestExecOver300Count =
                                        execOver300Count;

                                    latestExecOver400Count =
                                        execOver400Count;

                                    latestExecValid =
                                        true;


                                    // =====================================================
                                    // Reset EXEC Window
                                    // =====================================================

                                    execSumNs =
                                        0;

                                    execMinNs =
                                        0;

                                    execMaxNs =
                                        0;

                                    execSamples =
                                        0;

                                    execOver250Count =
                                        0;

                                    execOver300Count =
                                        0;

                                    execOver400Count =
                                        0;
                                }


                                // =========================================================
                                // PDO RT 診斷快照發布器
                                //
                                // Timer、Combined、Exec 三個視窗各自完成後再一起發布，
                                // 不要求同一個 cycle 同時結束。sequence 先變奇數，資料全部
                                // 寫完並 MemoryBarrier 後再變偶數，Main Thread 才可採用。
                                // =========================================================

                                if (latestPdoTimerValid &&
                                    latestCombinedValid &&
                                    latestExecValid)
                                {
                                    // -----------------------------------------------------
                                    // Odd = Writer updating
                                    // -----------------------------------------------------

                                    InterlockedIncrement(
                                        &g_pdoRtDiagSequence);


                                    // =====================================================
                                    // PDO Timer
                                    // =====================================================

                                    g_pdoRtTimerAvgNs =
                                        (LONGLONG)
                                        latestPdoTimerAvgNs;

                                    g_pdoRtTimerMinNs =
                                        (LONGLONG)
                                        latestPdoTimerMinNs;

                                    g_pdoRtTimerMaxNs =
                                        (LONGLONG)
                                        latestPdoTimerMaxNs;

                                    g_pdoRtTimerShortCount =
                                        (LONG)
                                        latestPdoTimerShortCount;

                                    g_pdoRtTimerNormalCount =
                                        (LONG)
                                        latestPdoTimerNormalCount;

                                    g_pdoRtTimerLongCount =
                                        (LONG)
                                        latestPdoTimerLongCount;


                                    // =====================================================
                                    // EtherCAT Combined
                                    // =====================================================

                                    g_pdoRtCombinedAvgNs =
                                        (LONGLONG)
                                        latestCombinedAvgNs;

                                    g_pdoRtCombinedMinNs =
                                        (LONGLONG)
                                        latestCombinedMinNs;

                                    g_pdoRtCombinedMaxNs =
                                        (LONGLONG)
                                        latestCombinedMaxNs;


                                    // =====================================================
                                    // PDO Execution
                                    // =====================================================

                                    g_pdoRtExecAvgNs =
                                        (LONGLONG)
                                        latestExecAvgNs;

                                    g_pdoRtExecMinNs =
                                        (LONGLONG)
                                        latestExecMinNs;

                                    g_pdoRtExecMaxNs =
                                        (LONGLONG)
                                        latestExecMaxNs;

                                    g_pdoRtExecOver250Count =
                                        (LONG)
                                        latestExecOver250Count;

                                    g_pdoRtExecOver300Count =
                                        (LONG)
                                        latestExecOver300Count;

                                    g_pdoRtExecOver400Count =
                                        (LONG)
                                        latestExecOver400Count;


                                    // =====================================================
                                    // WKC
                                    // =====================================================

                                    g_pdoRtLrwWkc =
                                        (LONG)
                                        wkc;

                                    g_pdoRtDcWkc =
                                        (LONG)
                                        dcWkc;

                                    g_pdoRtDcReferencePresent =
                                        dcReferencePresent ? 1L : 0L;

                                    g_pdoRtProcessDataValid =
                                        processDataValid ? 1L : 0L;

                                    g_pdoRtDcTransportValid =
                                        dcTransportValid ? 1L : 0L;

                                    g_pdoRtProcessInvalidStreak =
                                        (LONG)(
                                            pdoConsecutiveInvalidCycles > 0x7FFFFFFFU
                                            ? 0x7FFFFFFFU
                                            : pdoConsecutiveInvalidCycles);

                                    g_pdoRtDcTransportInvalidStreak =
                                        (LONG)(
                                            dcTransportConsecutiveInvalidCycles > 0x7FFFFFFFU
                                            ? 0x7FFFFFFFU
                                            : dcTransportConsecutiveInvalidCycles);

                                    g_pdoRtDcTransportInvalidMaxStreak =
                                        (LONG)(
                                            dcTransportMaximumInvalidCycles > 0x7FFFFFFFU
                                            ? 0x7FFFFFFFU
                                            : dcTransportMaximumInvalidCycles);

                                    g_pdoRtDcWkcInvalidTotal =
                                        (LONGLONG)dcWkcInvalidCyclesTotal;

                                    g_pdoRtDcOnlyInvalidTotal =
                                        (LONGLONG)dcOnlyInvalidCyclesTotal;

                                    g_pdoRtDcTransportRecoveryTotal =
                                        (LONGLONG)dcTransportRecoveryTotal;

                                    g_pdoRtDcOnlyInvalidStreak =
                                        (LONG)(
                                            dcOnlyInvalidConsecutiveCycles > 0x7FFFFFFFU
                                            ? 0x7FFFFFFFU
                                            : dcOnlyInvalidConsecutiveCycles);

                                    g_pdoRtDcOnlyInvalidMaxStreak =
                                        (LONG)(
                                            dcOnlyInvalidMaximumConsecutiveCycles > 0x7FFFFFFFU
                                            ? 0x7FFFFFFFU
                                            : dcOnlyInvalidMaximumConsecutiveCycles);

                                    g_pdoRtDcHoldoverTier =
                                        dcHoldoverDiagTier;

                                    const uint32_t dcHoldoverCurrentCycles =
                                        dcHoldoverEpisodeActive
                                        ? dcHoldoverUnqualifiedCycles
                                        : (dcOnlyGraceAcceptedThisCycle
                                            ? dcOnlyInvalidConsecutiveCycles
                                            : 0U);

                                    g_pdoRtDcHoldoverCurrentCycles =
                                        (LONG)(
                                            dcHoldoverCurrentCycles > 0x7FFFFFFFU
                                            ? 0x7FFFFFFFU
                                            : dcHoldoverCurrentCycles);

                                    g_pdoRtDcHoldoverMaxCycles =
                                        (LONG)(
                                            dcHoldoverMaximumUnqualifiedCycles > 0x7FFFFFFFU
                                            ? 0x7FFFFFFFU
                                            : dcHoldoverMaximumUnqualifiedCycles);

                                    g_pdoRtDcGlitchDebt =
                                        (LONG)dcOnlyGlitchDebt;

                                    g_pdoRtDcGlitchDebtMax =
                                        (LONG)dcOnlyGlitchDebtMaximum;

                                    g_pdoRtDcGlitchDebtLimit =
                                        (LONG)DC_RX3B_GLITCH_DEBT_LIMIT;

                                    g_pdoRtDcGraceAcceptedCyclesTotal =
                                        (LONGLONG)dcGraceAcceptedCyclesTotal;

                                    g_pdoRtDcHoldoverEpisodeTotal =
                                        (LONGLONG)dcHoldoverEpisodeTotal;

                                    g_pdoRtDcHoldoverEntryTotal =
                                        (LONGLONG)dcHoldoverEntryTotal;

                                    g_pdoRtDcDegradedEntryTotal =
                                        (LONGLONG)dcDegradedEntryTotal;

                                    g_pdoRtDcRelockEntryTotal =
                                        (LONGLONG)dcRelockEntryTotal;

                                    g_pdoRtDcSampleGuardState =
                                        dcSampleGuardState;

                                    g_pdoRtDcSampleQualified =
                                        dcControlSampleValid ? 1L : 0L;

                                    g_pdoRtDcSampleGuardReasonMask =
                                        dcSampleGuardLastReasonMask;

                                    g_pdoRtDcSampleRejectStreak =
                                        (LONG)(
                                            dcSampleGuardConsecutiveRejects > 0x7FFFFFFFU
                                            ? 0x7FFFFFFFU
                                            : dcSampleGuardConsecutiveRejects);

                                    g_pdoRtDcSampleRejectMaxStreak =
                                        (LONG)(
                                            dcSampleGuardMaximumConsecutiveRejects > 0x7FFFFFFFU
                                            ? 0x7FFFFFFFU
                                            : dcSampleGuardMaximumConsecutiveRejects);

                                    g_pdoRtDcSampleApproxAgeNs =
                                        (LONGLONG)dcSampleGuardLastApproxAgeNs;

                                    g_pdoRtDcSampleApproxAgeMaxNs =
                                        (LONGLONG)dcSampleGuardMaximumApproxAgeNs;

                                    g_pdoRtDcSampleQpcDeltaNs =
                                        (LONGLONG)dcSampleGuardLastQpcDeltaNs;

                                    g_pdoRtDcSampleDcDeltaNs =
                                        (LONGLONG)dcSampleGuardLastDcDeltaNs;

                                    g_pdoRtDcSampleDeltaErrorNs =
                                        (LONGLONG)dcSampleGuardLastDeltaErrorNs;

                                    g_pdoRtDcSampleDeltaToleranceNs =
                                        (LONGLONG)dcSampleGuardLastDeltaToleranceNs;

                                    g_pdoRtDcSampleAcceptedTotal =
                                        (LONGLONG)dcSampleGuardAcceptedTotal;

                                    g_pdoRtDcSampleRejectedTotal =
                                        (LONGLONG)dcSampleGuardRejectedTotal;

                                    g_pdoRtDcSampleAnchorTotal =
                                        (LONGLONG)dcSampleGuardAnchorTotal;

                                    g_pdoRtDcSampleReanchorTotal =
                                        (LONGLONG)dcSampleGuardReanchorTotal;

                                    g_pdoRtDcSampleAgeRejectTotal =
                                        (LONGLONG)dcSampleGuardAgeRejectTotal;

                                    g_pdoRtDcSampleOrderRejectTotal =
                                        (LONGLONG)dcSampleGuardOrderRejectTotal;

                                    g_pdoRtDcSampleDeltaRejectTotal =
                                        (LONGLONG)dcSampleGuardDeltaRejectTotal;

                                    g_pdoRtDcPhaseJumpGuardActive =
                                        phasePRecoveryJumpGuardActive ? 1L : 0L;

                                    g_pdoRtDcPhaseJumpGuardGoodWindows =
                                        (LONG)phasePRecoveryJumpGoodWindows;

                                    g_pdoRtDcPhaseJumpGuardRequiredWindows =
                                        (LONG)DC_RX3C_PHASE_JUMP_GOOD_WINDOWS;

                                    g_pdoRtDcPhaseJumpLastNs =
                                        (LONGLONG)phasePRecoveryJumpLastNs;

                                    g_pdoRtDcPhaseJumpMaxAbsNs =
                                        (LONGLONG)phasePRecoveryJumpMaximumAbsNs;

                                    g_pdoRtDcPhaseJumpArmTotal =
                                        (LONGLONG)phasePRecoveryJumpArmTotal;

                                    g_pdoRtDcPhaseJumpPassTotal =
                                        (LONGLONG)phasePRecoveryJumpPassTotal;

                                    g_pdoRtDcPhaseJumpRejectTotal =
                                        (LONGLONG)phasePRecoveryJumpRejectTotal;

                                    g_pdoRtDcPhaseMapSequence =
                                        phasePActV0DiagPhaseMapSequence;

                                    g_pdoRtDcPhaseMapNew =
                                        phasePActV0DiagPhaseMapNew ? 1L : 0L;

                                    g_pdoRtDcPhaseMapAgeGood =
                                        phasePActV0DiagPhaseMapAgeGood ? 1L : 0L;

                                    g_pdoRtDcPhaseMapAgeNs =
                                        (LONGLONG)phasePActV0DiagPhaseMapAgeNs;

                                    g_pdoRtDcPhaseMapStaleTotal =
                                        (LONGLONG)phasePActV0PhaseMapStaleTotal;

                                    g_pdoRtDcTimingSource =
                                        dcSampleTimingSource;

                                    g_pdoRtDcExactTimingLocked =
                                        dcExactTimingLocked ? 1L : 0L;

                                    g_pdoRtDcExactTimingValid =
                                        qpcDcExactTimingAvailable ? 1L : 0L;

                                    g_pdoRtDcTimingSelectedRttNs =
                                        (LONGLONG)qpcDcRttNs;

                                    g_pdoRtDcTimingExactRttNs =
                                        (LONGLONG)qpcDcExactRttNs;

                                    g_pdoRtDcTimingExactRttMaxNs =
                                        (LONGLONG)dcTimingExactRttMaximumNs;

                                    g_pdoRtDcTimingCallRttNs =
                                        (LONGLONG)qpcDcCallRttNs;

                                    g_pdoRtDcTimingExcludedOverheadNs =
                                        (LONGLONG)qpcDcExcludedOverheadNs;

                                    g_pdoRtDcTimingMidpointShiftNs =
                                        (LONGLONG)dcTimingMidpointShiftLastNs;

                                    g_pdoRtDcTimingMidpointShiftMaxAbsNs =
                                        (LONGLONG)
                                        dcTimingMidpointShiftMaximumAbsNs;

                                    g_pdoRtDcTimingExactUseTotal =
                                        (LONGLONG)dcTimingExactUseTotal;

                                    g_pdoRtDcTimingFallbackUseTotal =
                                        (LONGLONG)dcTimingFallbackUseTotal;

                                    g_pdoRtDcTimingMissingAfterLockTotal =
                                        (LONGLONG)
                                        dcTimingMissingAfterLockTotal;

                                    g_pdoRtDcTimingSourceSwitchTotal =
                                        (LONGLONG)dcTimingSourceSwitchTotal;

                                    g_pdoRtDcTimingRejectTotal =
                                        (LONGLONG)
                                        dcSampleGuardTimingRejectTotal;

                                    g_pdoRtDcRttGuardState =
                                        dcRttGuardState;

                                    g_pdoRtDcRttGuardAccepted =
                                        (dcRttGuardEvaluatedThisCycle &&
                                            dcRttGuardAcceptedThisCycle)
                                        ? 1L : 0L;

                                    g_pdoRtDcRttGuardWarmupSamples =
                                        (LONG)dcRttGuardWarmupSamples;

                                    g_pdoRtDcRttGuardWarmupRequired =
                                        (LONG)DC_RX3E_RTT_WARMUP_SAMPLES;

                                    g_pdoRtDcRttGuardCurrentNs =
                                        (LONGLONG)dcRttGuardLastRttNs;

                                    g_pdoRtDcRttGuardBaselineNs =
                                        (LONGLONG)dcRttGuardBaselineNs;

                                    g_pdoRtDcRttGuardDeviationNs =
                                        (LONGLONG)dcRttGuardDeviationNs;

                                    g_pdoRtDcRttGuardLimitNs =
                                        (LONGLONG)dcRttGuardLimitNs;

                                    g_pdoRtDcRttGuardExcessNs =
                                        (LONGLONG)dcRttGuardLastExcessNs;

                                    g_pdoRtDcRttGuardOutlierStreak =
                                        (LONG)(
                                            dcRttGuardOutlierStreak > 0x7FFFFFFFU
                                            ? 0x7FFFFFFFU
                                            : dcRttGuardOutlierStreak);

                                    g_pdoRtDcRttGuardOutlierMaxStreak =
                                        (LONG)(
                                            dcRttGuardOutlierMaximumStreak > 0x7FFFFFFFU
                                            ? 0x7FFFFFFFU
                                            : dcRttGuardOutlierMaximumStreak);

                                    g_pdoRtDcRttGuardRebaseCandidateSamples =
                                        (LONG)(
                                            dcRttGuardRebaseCandidateSamples > 0x7FFFFFFFU
                                            ? 0x7FFFFFFFU
                                            : dcRttGuardRebaseCandidateSamples);

                                    g_pdoRtDcRttGuardAcceptedTotal =
                                        (LONGLONG)dcRttGuardAcceptedTotal;

                                    g_pdoRtDcRttGuardRejectedTotal =
                                        (LONGLONG)dcRttGuardRejectedTotal;

                                    g_pdoRtDcRttGuardRebaseTotal =
                                        (LONGLONG)dcRttGuardRebaseTotal;

                                    g_pdoRtDcSampleRttRejectTotal =
                                        (LONGLONG)dcSampleGuardRttRejectTotal;

                                    // =====================================================
                                    // DC-RX.3F deterministic fault-injection snapshot
                                    // =====================================================

                                    g_pdoRtDcFaultState =
                                        (LONG)dcRx3fState;

                                    g_pdoRtDcFaultScenario =
                                        (LONG)dcRx3fScenario;

                                    g_pdoRtDcFaultConfiguredCycles =
                                        (LONG)(
                                            dcRx3fConfiguredCycles > 0x7FFFFFFFU
                                            ? 0x7FFFFFFFU
                                            : dcRx3fConfiguredCycles);

                                    g_pdoRtDcFaultAppliedCycles =
                                        (LONG)(
                                            dcRx3fAppliedCycles > 0x7FFFFFFFU
                                            ? 0x7FFFFFFFU
                                            : dcRx3fAppliedCycles);

                                    g_pdoRtDcFaultStartDelayRemaining =
                                        (LONG)(
                                            dcRx3fStartDelayRemaining > 0x7FFFFFFFU
                                            ? 0x7FFFFFFFU
                                            : dcRx3fStartDelayRemaining);

                                    g_pdoRtDcFaultRecoveryCycles =
                                        (LONG)(
                                            dcRx3fRecoveryCycles > 0x7FFFFFFFU
                                            ? 0x7FFFFFFFU
                                            : dcRx3fRecoveryCycles);

                                    g_pdoRtDcFaultTargetWaitCycles =
                                        (LONG)(
                                            dcRx3fTargetWaitCycles > 0x7FFFFFFFU
                                            ? 0x7FFFFFFFU
                                            : dcRx3fTargetWaitCycles);

                                    g_pdoRtDcFaultActiveThisCycle =
                                        dcRx3fInjectionAppliedThisCycle ? 1L : 0L;

                                    g_pdoRtDcFaultGateBlockMask =
                                        dcRx3fGateBlockMask;

                                    g_pdoRtDcFaultEvidenceMask =
                                        dcRx3fEvidenceMask;

                                    g_pdoRtDcFaultFailureMask =
                                        dcRx3fFailureMask;

                                    g_pdoRtDcFaultRequireServoOff =
                                        dcRx3fRequireServoOff ? 1L : 0L;

                                    g_pdoRtDcFaultAllowSafetyStop =
                                        dcRx3fAllowSafetyStop ? 1L : 0L;

                                    g_pdoRtDcFaultValueNs =
                                        (LONGLONG)dcRx3fConfiguredValueNs;

                                    g_pdoRtDcFaultBaselineAppliedPpb =
                                        (LONGLONG)dcRx3fBaselineAppliedPpb;

                                    g_pdoRtDcFaultCurrentAppliedPpb =
                                        (LONGLONG)realFfV0AppliedPpb;

                                    g_pdoRtDcFaultMaxAppliedDeltaPpb =
                                        (LONGLONG)dcRx3fMaximumAppliedDeltaPpb;

                                    g_pdoRtDcFaultMaxPdoInvalidStreak =
                                        (LONG)(
                                            dcRx3fMaximumPdoInvalidStreak > 0x7FFFFFFFU
                                            ? 0x7FFFFFFFU
                                            : dcRx3fMaximumPdoInvalidStreak);

                                    g_pdoRtDcFaultStartTick =
                                        (LONGLONG)dcRx3fStartTick;

                                    g_pdoRtDcFaultLastAppliedTick =
                                        (LONGLONG)dcRx3fLastAppliedTick;

                                    g_pdoRtDcFaultEndTick =
                                        (LONGLONG)dcRx3fEndTick;


                                    // 確保資料先完成，再發布 even sequence。
                                    MemoryBarrier();


                                    // -----------------------------------------------------
                                    // Even = Snapshot Ready
                                    // -----------------------------------------------------

                                    InterlockedIncrement(
                                        &g_pdoRtDiagSequence);


                                    // -----------------------------------------------------
                                    // Consume all three completed windows
                                    // -----------------------------------------------------

                                    latestPdoTimerValid =
                                        false;

                                    latestCombinedValid =
                                        false;

                                    latestExecValid =
                                        false;
                                }
                            }
}
