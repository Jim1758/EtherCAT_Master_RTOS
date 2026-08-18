#include "EtherCatMaster.h"
#include "EtherCatMaster_DC_Internal.h"
#include "GlobalConfig.h"
#include <windows.h>
#include <rtapi.h>
#include <rtssapi.h>
#include <stdio.h>

// ============================================================================
// EtherCatMaster_DC_Startup.cpp
// EtherCAT DC 啟動與 PDO One-Shot 建立流程 RC1.4（Startup Probe 開關版）
//
// 本檔責任：
//   1. 啟動時確認 RTX64 HAL period counts 回到系統 base 值。
//   2. 用隔離的 disposable timer 驗證 coarse one-shot + QPC fine wait 能力。
//   3. 建立真正 Priority 80 PDO timer，並先做同一 timer 的一次 warm-up。
//   4. 從 DC Reference（目前 S4／Motor_Start_Index）讀取 0x0910 System Time。
//   5. 以最低 RTT 樣本估算 CLOCK_2 Master time 與 EtherCAT DC time 的 offset。
//   6. 找出未來的 DC phase 0 目標，換算成 CLOCK_2 absolute expiration 後啟動 PDO。
//   7. 若 DC 量測或 absolute arm 失敗，以 250 us relative one-shot 安全啟動。
//
// 啟動完成後的責任分工：
//   - 本檔只負責建立／warm-up／第一次 arm。
//   - 後續每個 4 kHz callback 由 GlobalTimerHandler_PDO() 執行並自行 re-arm。
//   - Runtime 的 Fine Wait 為 OFF；本檔 coarse/fine busy-wait 只存在隔離探針中。
//
// 目前重要設定：
//   - PDO cycle：250000 ns（4 kHz）。
//   - Servo Sync0 shift：125000 ns；PDO 啟動目標 phase：0 ns。
//   - Probe/PDO coarse guard：100000 ns。
//   - DC alignment：8 個 0x0910 樣本，接受的最佳 RTT 必須 <= 1000000 ns。
//   - DC absolute start guard：至少在估測 DC now 之後 20 ms。
//   - RtSetTimer 前的最低 CLOCK_2 lead：5 ms。
//
// 回傳值：
//   0  = 正式 DC-aligned timer 或 relative fallback 已成功啟動。
//   -1 = 真正 PDO timer 建立、warm-up，或最後 fallback arm 失敗。
//
// 即時／安全限制：
//   - Probe callback 不做 RtPrintf、EtherCAT I/O、Sleep 或動態配置。
//   - Probe timer 用完必須刪除，之後才建立真正 PDO timer。
//   - 真正 PDO timer 的 warm-up callback 不得碰 EtherCAT process image。
//   - 不可把 Probe 的 QPC busy-wait 複製到正式 4 kHz Runtime。
//   - 修改週期、phase、guard 或 priority 時，一次只改一項並重新長時間測試。
//
// 啟動參數調整指南：
//   - ALIGN_SAMPLE_COUNT=8：只影響啟動取樣數；增加可提高找到低 RTT 樣本的機會，
//     但會增加啟動時間與額外 register traffic。
//   - MAX_ALIGN_RTT_NS=1000000：只決定樣本是否可用；放寬不會改善同步品質，
//     只會允許較差的 Master/DC 時間映射。
//   - START_GUARD_NS=20000000：第一個 DC target 至少排在估測 DC now 之後 20 ms。
//   - MIN_TIMER_LEAD_NS=5000000：真正 RtSetTimer 前至少保留 5 ms；太近時程式會
//     自動向後推完整 250 us cycle，所以不會改變 target phase。
//   - WARMUP/PROBE timeout：只是啟動等待上限，不是 EtherCAT PDO 接收 timeout。
//   - PDO_DC_CYCLE_NS、PDO_TARGET_PHASE_NS、Sync0、Timer Priority 與 coarse guard
//     是整體時序設計；沒有重新量測前不要單獨修改。
//   - Runtime RX HardTimeout、Recent_Timeout 或 ActualErr 不在本檔調整；啟動參數
//     不能拿來掩蓋執行中的網卡延遲、WKC 或排程抖動。
//
// 啟動 DEBUG 快速判讀：
//   - [DC-HAL-STARTUP]：Current 最後應等於 Base；恢復失敗會記錄但仍繼續。
//   - [COARSE-FINE-QPC-PROBE]：理想為 PASS、Callbacks 4000/4000、
//     RearmOK 3999/3999，RearmFail/FineMiss/NextCoarsePast 全為 0。
//   - [PDO-ONESHOT-WARMUP]：必須 PASS；FAIL 會在進入正式 PDO 前回傳 -1。
//   - [PDO-DC-SAMPLE]：有效樣本 WKC>0；RTT 越小越適合估算 Master/DC offset。
//   - [PDO-DC-BEST-SAMPLE]：Direction/Offset 是兩個 absolute clock epoch 的映射，
//     不是主從站瞬間同步誤差，Offset 很大本身不代表 DC 同步失敗。
//   - [PDO-DC-ALIGN-V2.1]：正式路徑應顯示 SUCCESS、TargetPhase=0、
//     Sync0=125000 ns；Remaining 是 arm 前保留時間，應至少約 5 ms。
//   - [PDO-DC-ALIGN-FALLBACK]：代表 absolute DC phase 啟動失敗而改用 relative；
//     PDO 仍可運作，但第一個 callback 不保證從設計的 DC phase 0 開始。
// ============================================================================

// ============================================================================
// Startup Probe 總開關
//
// true ：執行約一秒的 Coarse + Fine QPC 排程診斷，並輸出
//        [COARSE-FINE-QPC-PROBE] 統計。
// false：完全跳過 disposable Probe Timer，不建立 Priority 62 Probe、不做 busy-wait。
//
// 此開關只控制啟動診斷，不會關閉：
//   - 正式 Priority 80 PDO Timer。
//   - [PDO-ONESHOT-WARMUP] 正式 timer warm-up。
//   - S4 DC 取樣、absolute DC Alignment 與 relative fallback。
//
// 建議：調機／正式版驗收設為 true；正式量產且已完成驗收後可設為 false。
// ============================================================================
static const bool
ENABLE_COARSE_FINE_QPC_STARTUP_PROBE =
true;

// =============================================================
// RTX64 Warmed Coarse + Fine QPC Scheduler Probe V6（隔離啟動自我測試）
//
// 在碰觸真正 EtherCAT PDO timer 前驗證完整排程機制：
//   1. 先讓 disposable one-shot timer 執行一次 warm-up callback。
//   2. 建立固定 250 us 的 QPC final-target timeline。
//   3. 把 CLOCK_2 relative one-shot arm 在 finalTarget - 100 us。
//   4. coarse callback 醒來後，先 arm 下一次 one-shot。
//   5. 再以 QPC busy-wait 到本次 final target，量測 release error。
//
// Probe 共執行 4000 callback（約一秒），結束後刪除 timer；成功或失敗都不會
// 直接改變 EtherCAT、HAL 或正式 PDO 時序。Callback 只更新固定大小統計欄位。
// =============================================================

static HANDLE
g_coarseFineProbeTimer =
NULL;


// Probe mode：0=等待 warm-up callback；1=coarse+fine 排程測試中。
static volatile LONG
g_coarseFineProbeMode =
0;

static volatile LONG
g_coarseFineProbeWarmupDone =
0;

static volatile LONG
g_coarseFineProbeDone =
0;

static volatile LONG
g_coarseFineProbeLastError =
0;


static volatile LONG
g_coarseFineProbeCallbacks =
0;

static volatile LONG
g_coarseFineProbeRearmOk =
0;

static volatile LONG
g_coarseFineProbeRearmFail =
0;

static volatile LONG
g_coarseFineProbeFineMiss =
0;

static volatile LONG
g_coarseFineProbeNextCoarsePast =
0;


// QPC timeline：以 whole counts + remainder 保存 4 kHz 非整數週期，避免累積截斷誤差。
static uint64_t
g_coarseFineProbeQpcFrequency =
0;

static uint64_t
g_coarseFineProbeCurrentFinalTargetQpc =
0;

static uint64_t
g_coarseFineProbePeriodWholeCounts =
0;

static uint64_t
g_coarseFineProbePeriodRemainder =
0;

static uint64_t
g_coarseFineProbeFractionAccumulator =
0;

static uint64_t
g_coarseFineProbeGuardCounts =
0;


// coarse callback 間隔：確認 one-shot callback 平均是否接近 250000 ns。
static uint64_t
g_coarseFineProbePreviousWakeQpc =
0;

static uint64_t
g_coarseFineProbeIntervalSumNs =
0;

static uint64_t
g_coarseFineProbeIntervalMinNs =
0;

static uint64_t
g_coarseFineProbeIntervalMaxNs =
0;

static uint32_t
g_coarseFineProbeIntervalSamples =
0;


// coarse wake 相對 coarse target 的誤差：正值=晚到，負值=提早。
static int64_t
g_coarseFineProbeCoarseErrorSumNs =
0;

static int64_t
g_coarseFineProbeCoarseErrorMinNs =
0;

static int64_t
g_coarseFineProbeCoarseErrorMaxNs =
0;

static uint32_t
g_coarseFineProbeCoarseErrorSamples =
0;


// coarse wake 到 final target 的剩餘時間，近似本次 QPC busy-wait 長度。
static uint64_t
g_coarseFineProbeFineWaitSumNs =
0;

static uint64_t
g_coarseFineProbeFineWaitMinNs =
0;

static uint64_t
g_coarseFineProbeFineWaitMaxNs =
0;

static uint32_t
g_coarseFineProbeFineWaitSamples =
0;


// QPC busy-wait 結束後的 release error；迴圈在 QPC>=target 才離開，正常應 >=0。
static int64_t
g_coarseFineProbeReleaseErrorSumNs =
0;

static int64_t
g_coarseFineProbeReleaseErrorMinNs =
0;

static int64_t
g_coarseFineProbeReleaseErrorMaxNs =
0;

static uint32_t
g_coarseFineProbeReleaseErrorSamples =
0;


// RtSetTimerRelative() re-arm 呼叫成本；只量 API 執行時間，不含等待到 callback。
static uint64_t
g_coarseFineProbeRearmCostSumNs =
0;

static uint64_t
g_coarseFineProbeRearmCostMinNs =
0;

static uint64_t
g_coarseFineProbeRearmCostMaxNs =
0;

static uint32_t
g_coarseFineProbeRearmCostSamples =
0;


static void RTAPI
WarmedCoarseFineQpcSchedulerProbeHandler(
    void* context)
{
    // 此 callback 屬於 disposable probe timer（Priority 62），不是正式 PDO callback。
    // 所有狀態均為本檔 static，啟動執行緒只在 callback 完成後讀取結果。
    UNREFERENCED_PARAMETER(
        context);


    const LONG TARGET_CALLBACKS =
        4000;

    const uint64_t TARGET_DENOMINATOR =
        4000ULL;


    // =========================================================
    // Warm-up callback：第一次 arm 只用來支付 RTX64 timer 的一次性初始化成本。
    // 不統計、不 re-arm、不 busy-wait；StartDcPdoRuntime 看到 WarmupDone 後才進 Phase B。
    // =========================================================

    if (g_coarseFineProbeMode == 0)
    {
        InterlockedExchange(
            &g_coarseFineProbeWarmupDone,
            1);

        return;
    }


    if (g_coarseFineProbeQpcFrequency == 0 ||
        g_coarseFineProbeCurrentFinalTargetQpc == 0 ||
        g_coarseFineProbePeriodWholeCounts == 0 ||
        g_coarseFineProbeGuardCounts == 0)
    {
        // 任何 QPC／period／guard 前置值為 0 都代表 probe 初始化不完整。
        g_coarseFineProbeLastError =
            ERROR_INVALID_DATA;

        InterlockedExchange(
            &g_coarseFineProbeDone,
            1);

        return;
    }


    // =========================================================
    // 取得本次 coarse callback 實際醒來 QPC；失敗就終止 probe，不使用無效時間戳。
    // =========================================================

    LARGE_INTEGER wakeQpcLi = {};


    if (!RtQueryPerformanceCounter(
        &wakeQpcLi))
    {
        g_coarseFineProbeLastError =
            (LONG)
            GetLastError();

        InterlockedExchange(
            &g_coarseFineProbeDone,
            1);

        return;
    }


    uint64_t wakeQpc =
        (uint64_t)
        wakeQpcLi.QuadPart;


    uint64_t currentFinalTargetQpc =
        g_coarseFineProbeCurrentFinalTargetQpc;


    uint64_t currentCoarseTargetQpc =
        currentFinalTargetQpc -
        g_coarseFineProbeGuardCounts;


    // =========================================================
    // 從第二次 callback 起統計相鄰 coarse wake 間隔；第一筆沒有 previous wake。
    // =========================================================

    if (g_coarseFineProbePreviousWakeQpc > 0 &&
        wakeQpc >=
        g_coarseFineProbePreviousWakeQpc)
    {
        uint64_t deltaCounts =
            wakeQpc -
            g_coarseFineProbePreviousWakeQpc;


        uint64_t intervalNs =
            (
                deltaCounts *
                1000000000ULL
                )
            /
            g_coarseFineProbeQpcFrequency;


        if (g_coarseFineProbeIntervalSamples == 0)
        {
            g_coarseFineProbeIntervalMinNs =
                intervalNs;

            g_coarseFineProbeIntervalMaxNs =
                intervalNs;
        }
        else
        {
            if (intervalNs <
                g_coarseFineProbeIntervalMinNs)
            {
                g_coarseFineProbeIntervalMinNs =
                    intervalNs;
            }


            if (intervalNs >
                g_coarseFineProbeIntervalMaxNs)
            {
                g_coarseFineProbeIntervalMaxNs =
                    intervalNs;
            }
        }


        g_coarseFineProbeIntervalSumNs +=
            intervalNs;

        g_coarseFineProbeIntervalSamples++;
    }


    g_coarseFineProbePreviousWakeQpc =
        wakeQpc;


    // =========================================================
    // coarseError = actual wake - requested coarse target。
    // 正值表示 timer callback 晚到；負值表示比 target 早醒。
    // =========================================================

    int64_t coarseErrorCounts =
        (int64_t)
        wakeQpc -
        (int64_t)
        currentCoarseTargetQpc;


    int64_t coarseErrorNs =
        (
            coarseErrorCounts *
            1000000000LL
            )
        /
        (int64_t)
        g_coarseFineProbeQpcFrequency;


    if (g_coarseFineProbeCoarseErrorSamples == 0)
    {
        g_coarseFineProbeCoarseErrorMinNs =
            coarseErrorNs;

        g_coarseFineProbeCoarseErrorMaxNs =
            coarseErrorNs;
    }
    else
    {
        if (coarseErrorNs <
            g_coarseFineProbeCoarseErrorMinNs)
        {
            g_coarseFineProbeCoarseErrorMinNs =
                coarseErrorNs;
        }


        if (coarseErrorNs >
            g_coarseFineProbeCoarseErrorMaxNs)
        {
            g_coarseFineProbeCoarseErrorMaxNs =
                coarseErrorNs;
        }
    }


    g_coarseFineProbeCoarseErrorSumNs +=
        coarseErrorNs;

    g_coarseFineProbeCoarseErrorSamples++;


    // =========================================================
    // 若 coarse wake 已經超過 final target，fine wait 無法把時間倒轉回去；
    // 立刻記 FineMiss/ERROR_TIMEOUT 並終止，避免把失敗樣本當成成功 release。
    // =========================================================

    if (wakeQpc >=
        currentFinalTargetQpc)
    {
        InterlockedIncrement(
            &g_coarseFineProbeFineMiss);


        g_coarseFineProbeLastError =
            ERROR_TIMEOUT;


        InterlockedExchange(
            &g_coarseFineProbeDone,
            1);

        return;
    }


    // =========================================================
    // 記錄本次還剩多少時間到 final target；理想值接近 100 us coarse guard，
    // 實際會扣掉 timer 晚到與 callback 前段計算成本。
    // =========================================================

    uint64_t fineWaitCounts =
        currentFinalTargetQpc -
        wakeQpc;


    uint64_t fineWaitNs =
        (
            fineWaitCounts *
            1000000000ULL
            )
        /
        g_coarseFineProbeQpcFrequency;


    if (g_coarseFineProbeFineWaitSamples == 0)
    {
        g_coarseFineProbeFineWaitMinNs =
            fineWaitNs;

        g_coarseFineProbeFineWaitMaxNs =
            fineWaitNs;
    }
    else
    {
        if (fineWaitNs <
            g_coarseFineProbeFineWaitMinNs)
        {
            g_coarseFineProbeFineWaitMinNs =
                fineWaitNs;
        }


        if (fineWaitNs >
            g_coarseFineProbeFineWaitMaxNs)
        {
            g_coarseFineProbeFineWaitMaxNs =
                fineWaitNs;
        }
    }


    g_coarseFineProbeFineWaitSumNs +=
        fineWaitNs;

    g_coarseFineProbeFineWaitSamples++;


    LONG callbackCount =
        InterlockedIncrement(
            &g_coarseFineProbeCallbacks);


    // =========================================================
    // 先 arm 下一次 coarse wake，再 busy-wait 本次 final target。
    // nextFinal=currentFinal+250 us；nextCoarse=nextFinal-100 us guard。
    // 這個順序避免本次約 100 us busy-wait 吃掉下一次 timer 的 re-arm lead。
    // =========================================================

    if (callbackCount < TARGET_CALLBACKS)
    {
        uint64_t nextFinalTargetQpc =
            currentFinalTargetQpc +
            g_coarseFineProbePeriodWholeCounts;


        g_coarseFineProbeFractionAccumulator +=
            g_coarseFineProbePeriodRemainder;


        if (g_coarseFineProbeFractionAccumulator >=
            TARGET_DENOMINATOR)
        {
            // QPC frequency 通常不能被 4000 整除；remainder 累積滿 4000 時補 1 count，
            // 讓長時間平均週期保持精確，不因整數除法持續漂移。
            nextFinalTargetQpc++;

            g_coarseFineProbeFractionAccumulator -=
                TARGET_DENOMINATOR;
        }


        uint64_t nextCoarseTargetQpc =
            nextFinalTargetQpc -
            g_coarseFineProbeGuardCounts;


        LARGE_INTEGER beforeRearmQpcLi = {};


        if (!RtQueryPerformanceCounter(
            &beforeRearmQpcLi))
        {
            g_coarseFineProbeLastError =
                (LONG)
                GetLastError();

            InterlockedExchange(
                &g_coarseFineProbeDone,
                1);

            return;
        }


        uint64_t beforeRearmQpc =
            (uint64_t)
            beforeRearmQpcLi.QuadPart;


        if (nextCoarseTargetQpc <=
            beforeRearmQpc)
        {
            // 下一個 coarse target 已在過去，表示 callback/re-arm 成本超出週期預算。
            InterlockedIncrement(
                &g_coarseFineProbeNextCoarsePast);


            g_coarseFineProbeLastError =
                ERROR_TIMEOUT;


            InterlockedExchange(
                &g_coarseFineProbeDone,
                1);

            return;
        }


        uint64_t remainingCounts =
            nextCoarseTargetQpc -
            beforeRearmQpc;


        // QPC counts -> CLOCK_2 relative 100 ns 單位，採 ceiling 避免截斷後過早到期。
        uint64_t numerator =
            remainingCounts *
            10000000ULL;


        uint64_t relative100ns =
            (
                numerator +
                g_coarseFineProbeQpcFrequency -
                1ULL
                )
            /
            g_coarseFineProbeQpcFrequency;


        if (relative100ns == 0)
        {
            relative100ns =
                1;
        }


        LARGE_INTEGER relativeExpiration = {};

        relativeExpiration.QuadPart =
            (LONGLONG)
            relative100ns;


        LARGE_INTEGER rearmStartQpcLi = {};
        LARGE_INTEGER rearmEndQpcLi = {};

        bool rearmStartValid =
            false;

        bool rearmEndValid =
            false;


        if (RtQueryPerformanceCounter(
            &rearmStartQpcLi))
        {
            rearmStartValid =
                true;
        }


        BOOL rearmOk =
            RtSetTimerRelative(
                g_coarseFineProbeTimer,
                &relativeExpiration,
                NULL);


        DWORD rearmError =
            ERROR_SUCCESS;


        if (!rearmOk)
        {
            rearmError =
                GetLastError();
        }


        if (rearmStartValid)
        {
            if (RtQueryPerformanceCounter(
                &rearmEndQpcLi))
            {
                if (rearmEndQpcLi.QuadPart >=
                    rearmStartQpcLi.QuadPart)
                {
                    rearmEndValid =
                        true;
                }
            }
        }


        if (rearmStartValid &&
            rearmEndValid)
        {
            uint64_t rearmCounts =
                (uint64_t)
                (
                    rearmEndQpcLi.QuadPart -
                    rearmStartQpcLi.QuadPart
                    );


            uint64_t rearmCostNs =
                (
                    rearmCounts *
                    1000000000ULL
                    )
                /
                g_coarseFineProbeQpcFrequency;


            if (g_coarseFineProbeRearmCostSamples == 0)
            {
                g_coarseFineProbeRearmCostMinNs =
                    rearmCostNs;

                g_coarseFineProbeRearmCostMaxNs =
                    rearmCostNs;
            }
            else
            {
                if (rearmCostNs <
                    g_coarseFineProbeRearmCostMinNs)
                {
                    g_coarseFineProbeRearmCostMinNs =
                        rearmCostNs;
                }


                if (rearmCostNs >
                    g_coarseFineProbeRearmCostMaxNs)
                {
                    g_coarseFineProbeRearmCostMaxNs =
                        rearmCostNs;
                }
            }


            g_coarseFineProbeRearmCostSumNs +=
                rearmCostNs;

            g_coarseFineProbeRearmCostSamples++;
        }


        if (!rearmOk)
        {
            g_coarseFineProbeLastError =
                (LONG)
                rearmError;


            InterlockedIncrement(
                &g_coarseFineProbeRearmFail);


            InterlockedExchange(
                &g_coarseFineProbeDone,
                1);

            return;
        }


        g_coarseFineProbeCurrentFinalTargetQpc =
            nextFinalTargetQpc;


        InterlockedIncrement(
            &g_coarseFineProbeRearmOk);
    }


    // =========================================================
    // Fine QPC busy-wait 到「本次」final target。這是 probe 唯一 busy-wait；
    // 下一次 timer 已先 arm，因此只用來量測可達的最終 release 精度。
    // =========================================================

    LARGE_INTEGER spinQpcLi = {};


    for (;;)
    {
        if (!RtQueryPerformanceCounter(
            &spinQpcLi))
        {
            g_coarseFineProbeLastError =
                (LONG)
                GetLastError();

            InterlockedExchange(
                &g_coarseFineProbeDone,
                1);

            return;
        }


        if ((uint64_t)
            spinQpcLi.QuadPart >=
            currentFinalTargetQpc)
        {
            break;
        }
    }


    uint64_t releaseQpc =
        (uint64_t)
        spinQpcLi.QuadPart;


    int64_t releaseErrorCounts =
        (int64_t)
        releaseQpc -
        (int64_t)
        currentFinalTargetQpc;


    int64_t releaseErrorNs =
        (
            releaseErrorCounts *
            1000000000LL
            )
        /
        (int64_t)
        g_coarseFineProbeQpcFrequency;


    if (g_coarseFineProbeReleaseErrorSamples == 0)
    {
        g_coarseFineProbeReleaseErrorMinNs =
            releaseErrorNs;

        g_coarseFineProbeReleaseErrorMaxNs =
            releaseErrorNs;
    }
    else
    {
        if (releaseErrorNs <
            g_coarseFineProbeReleaseErrorMinNs)
        {
            g_coarseFineProbeReleaseErrorMinNs =
                releaseErrorNs;
        }


        if (releaseErrorNs >
            g_coarseFineProbeReleaseErrorMaxNs)
        {
            g_coarseFineProbeReleaseErrorMaxNs =
                releaseErrorNs;
        }
    }


    g_coarseFineProbeReleaseErrorSumNs +=
        releaseErrorNs;

    g_coarseFineProbeReleaseErrorSamples++;


    // =========================================================
    // 第 4000 個 callback 完成後發布 Done；StartDcPdoRuntime 隨後計算平均值並刪 timer。
    // =========================================================

    if (callbackCount >=
        TARGET_CALLBACKS)
    {
        MemoryBarrier();

        InterlockedExchange(
            &g_coarseFineProbeDone,
            1);
    }
}



int EtherCatMaster::StartDcPdoRuntime()
{
    // ========================================================================
    // DC/PDO 啟動總入口
    //
    // 執行順序：HAL base recovery -> disposable QPC probe -> 建立真正 PDO timer
    // -> 同 timer warm-up -> S4 DC 對齊 -> absolute arm；失敗則 relative fallback。
    // 本函式由非即時啟動執行緒呼叫，因此允許 RtPrintf 與 bounded RtSleep 等待。
    // ========================================================================
    HANDLE hTimer_PDO = NULL;
    LARGE_INTEGER liPeriod_PDO;
    liPeriod_PDO.QuadPart = 2500; // CLOCK_2 以 100 ns 為單位：2500 = 250 us。


    // ========================================================================
    // Step 1：恢復 RTX64 HAL base count
    //
    // 若前一次程式或測試曾改變 HAL period counts，啟動時先恢復 Control Panel
    // 定義的 base。此步不把 HAL 改成特定數字，只回到 RTX64 回報的 base 值。
    // 讀取／恢復失敗目前只記錄 LOG，不中止 PDO 啟動。
    // ========================================================================
    {
        ULONG currentHalCounts =
            0;


        ULONG baseHalCounts =
            0;


        if (RtGetHalTimerPeriodCounts(
            &currentHalCounts,
            &baseHalCounts))
        {
            RtPrintf(
                "[DC-HAL-STARTUP] "
                "Current:%lu | Base:%lu\n",

                (unsigned long)
                currentHalCounts,

                (unsigned long)
                baseHalCounts);


            if (currentHalCounts !=
                baseHalCounts)
            {
                RtPrintf(
                    "[DC-HAL-STARTUP] "
                    "Recover HAL Count: %lu -> %lu\n",

                    (unsigned long)
                    currentHalCounts,

                    (unsigned long)
                    baseHalCounts);


                if (RtSetHalTimerPeriodCounts(
                    baseHalCounts))
                {
                    RtPrintf(
                        "[DC-HAL-STARTUP] "
                        "HAL Base Count restored successfully.\n");
                }
                else
                {
                    RtPrintf(
                        "[DC-HAL-STARTUP] "
                        "HAL Base Count restore FAILED | "
                        "Error:%lu\n",

                        (unsigned long)
                        GetLastError());
                }
            }
            else
            {
                RtPrintf(
                    "[DC-HAL-STARTUP] "
                    "HAL already at Base Count.\n");
            }
        }
        else
        {
            RtPrintf(
                "[DC-HAL-STARTUP] "
                "RtGetHalTimerPeriodCounts FAILED | "
                "Error:%lu\n",

                (unsigned long)
                GetLastError());
        }
    }

    // ============================================================
    // Step 2：RTX64 Warmed Coarse + Fine QPC Scheduler Probe V6
    //
    // 先 warm-up，再以每 250 us final target、提前 100 us coarse wake、
    // QPC busy-wait 到 final target 的方式跑 4000 callback。
    // Probe FAIL 只代表啟動環境量測不理想；設計上仍繼續建立正式 PDO timer。
    // ============================================================

    if (ENABLE_COARSE_FINE_QPC_STARTUP_PROBE)
    {
        const LONG TARGET_CALLBACKS =
            4000;             // 4 kHz 下約一秒。

        const LONG EXPECTED_REARMS =
            TARGET_CALLBACKS -
            1;

        const uint64_t TARGET_DENOMINATOR =
            4000ULL;

        const uint64_t WARMUP_DELAY_NS =
            5000000ULL;       // disposable timer 第一次 arm 延遲 5 ms。

        const uint64_t START_DELAY_NS =
            5000000ULL;       // Phase B 第一個 final target 放在 5 ms 後。

        const uint64_t COARSE_GUARD_NS =
            100000ULL;       // coarse wake 比 final target 提前 100 us。

        const int WARMUP_TIMEOUT_MS =
            500;              // 啟動執行緒等待 warm-up 的上限。

        const int TARGET_TIMEOUT_MS =
            1500;             // 約一秒 probe 另留 500 ms 容錯。


        // --------------------------------------------------------
        // 重設所有 probe 靜態狀態，確保同一 process 若重新呼叫不沿用上次統計。
        // --------------------------------------------------------

        g_coarseFineProbeTimer =
            NULL;

        g_coarseFineProbeMode =
            0;

        g_coarseFineProbeWarmupDone =
            0;

        g_coarseFineProbeDone =
            0;

        g_coarseFineProbeLastError =
            0;


        g_coarseFineProbeCallbacks =
            0;

        g_coarseFineProbeRearmOk =
            0;

        g_coarseFineProbeRearmFail =
            0;

        g_coarseFineProbeFineMiss =
            0;

        g_coarseFineProbeNextCoarsePast =
            0;


        g_coarseFineProbeQpcFrequency =
            0;

        g_coarseFineProbeCurrentFinalTargetQpc =
            0;

        g_coarseFineProbePeriodWholeCounts =
            0;

        g_coarseFineProbePeriodRemainder =
            0;

        g_coarseFineProbeFractionAccumulator =
            0;

        g_coarseFineProbeGuardCounts =
            0;


        g_coarseFineProbePreviousWakeQpc =
            0;

        g_coarseFineProbeIntervalSumNs =
            0;

        g_coarseFineProbeIntervalMinNs =
            0;

        g_coarseFineProbeIntervalMaxNs =
            0;

        g_coarseFineProbeIntervalSamples =
            0;


        g_coarseFineProbeCoarseErrorSumNs =
            0;

        g_coarseFineProbeCoarseErrorMinNs =
            0;

        g_coarseFineProbeCoarseErrorMaxNs =
            0;

        g_coarseFineProbeCoarseErrorSamples =
            0;


        g_coarseFineProbeFineWaitSumNs =
            0;

        g_coarseFineProbeFineWaitMinNs =
            0;

        g_coarseFineProbeFineWaitMaxNs =
            0;

        g_coarseFineProbeFineWaitSamples =
            0;


        g_coarseFineProbeReleaseErrorSumNs =
            0;

        g_coarseFineProbeReleaseErrorMinNs =
            0;

        g_coarseFineProbeReleaseErrorMaxNs =
            0;

        g_coarseFineProbeReleaseErrorSamples =
            0;


        g_coarseFineProbeRearmCostSumNs =
            0;

        g_coarseFineProbeRearmCostMinNs =
            0;

        g_coarseFineProbeRearmCostMaxNs =
            0;

        g_coarseFineProbeRearmCostSamples =
            0;


        LARGE_INTEGER qpcFrequencyLi = {};


        if (RtQueryPerformanceFrequency(
            &qpcFrequencyLi) &&
            qpcFrequencyLi.QuadPart > 0)
        {
            g_coarseFineProbeQpcFrequency =
                (uint64_t)
                qpcFrequencyLi.QuadPart;
        }


        if (g_coarseFineProbeQpcFrequency == 0)
        {
            // QPC 不可用時無法執行 probe；不把診斷能力缺失當作 EtherCAT 啟動失敗。
            RtPrintf(
                "[COARSE-FINE-QPC-PROBE] "
                "QPC frequency unavailable. "
                "Continuing normal PDO startup.\n");
        }
        else
        {
            g_coarseFineProbePeriodWholeCounts =
                g_coarseFineProbeQpcFrequency /
                TARGET_DENOMINATOR;


            g_coarseFineProbePeriodRemainder =
                g_coarseFineProbeQpcFrequency %
                TARGET_DENOMINATOR;


            g_coarseFineProbeGuardCounts =
                (
                    g_coarseFineProbeQpcFrequency *
                    COARSE_GUARD_NS
                    )
                /
                1000000000ULL;


            g_coarseFineProbeTimer =
                RtCreateTimer(
                    NULL,
                    0,
                    WarmedCoarseFineQpcSchedulerProbeHandler,
                    NULL,
                    62,
                    CLOCK_2);

            // Probe Priority 62 低於真正 PDO timer Priority 80；兩者不會同時存在。


            if (g_coarseFineProbeTimer == NULL)
            {
                RtPrintf(
                    "[COARSE-FINE-QPC-PROBE] "
                    "CREATE FAILED | "
                    "Error:%lu | "
                    "Continuing normal PDO startup.\n",

                    (unsigned long)
                    GetLastError());
            }
            else
            {
                // =================================================
                // Phase A：disposable timer 第一次 relative arm，只驗證 warm-up callback。
                // =================================================

                LARGE_INTEGER warmupExpiration = {};

                warmupExpiration.QuadPart =
                    (LONGLONG)
                    (
                        (
                            WARMUP_DELAY_NS +
                            99ULL
                            )
                        /
                        100ULL
                        );


                LARGE_INTEGER warmupArmStartQpc = {};
                LARGE_INTEGER warmupArmEndQpc = {};


                RtQueryPerformanceCounter(
                    &warmupArmStartQpc);


                BOOL warmupArmOk =
                    RtSetTimerRelative(
                        g_coarseFineProbeTimer,
                        &warmupExpiration,
                        NULL);


                DWORD warmupArmError =
                    ERROR_SUCCESS;


                if (!warmupArmOk)
                {
                    warmupArmError =
                        GetLastError();
                }


                RtQueryPerformanceCounter(
                    &warmupArmEndQpc);


                uint64_t warmupArmWindowNs =
                    0;


                if (warmupArmEndQpc.QuadPart >=
                    warmupArmStartQpc.QuadPart)
                {
                    uint64_t warmupCounts =
                        (uint64_t)
                        (
                            warmupArmEndQpc.QuadPart -
                            warmupArmStartQpc.QuadPart
                            );


                    warmupArmWindowNs =
                        (
                            warmupCounts *
                            1000000000ULL
                            )
                        /
                        g_coarseFineProbeQpcFrequency;
                }


                int warmupWaitedMs =
                    0;


                if (warmupArmOk)
                {
                    // 啟動執行緒用 1 ms 粒度 bounded wait；probe callback 本身不 Sleep。
                    while (g_coarseFineProbeWarmupDone == 0 &&
                        warmupWaitedMs <
                        WARMUP_TIMEOUT_MS)
                    {
                        RtSleep(
                            1);

                        warmupWaitedMs++;
                    }
                }


                MemoryBarrier();


                // =================================================
                // Phase B：以目前 QPC+5 ms 建立第一個 final target，再減 100 us
                // 得第一個 coarse target；後續由 callback 自行維持 250 us timeline。
                // =================================================

                BOOL firstCoarseArmOk =
                    FALSE;

                DWORD firstCoarseArmError =
                    ERROR_SUCCESS;

                uint64_t firstCoarseArmCostNs =
                    0;


                if (warmupArmOk &&
                    g_coarseFineProbeWarmupDone != 0)
                {
                    LARGE_INTEGER qpcNowLi = {};


                    if (RtQueryPerformanceCounter(
                        &qpcNowLi))
                    {
                        uint64_t startDelayCounts =
                            (
                                g_coarseFineProbeQpcFrequency *
                                START_DELAY_NS
                                )
                            /
                            1000000000ULL;


                        g_coarseFineProbeCurrentFinalTargetQpc =
                            (uint64_t)
                            qpcNowLi.QuadPart +
                            startDelayCounts;


                        g_coarseFineProbeFractionAccumulator =
                            0;


                        uint64_t firstCoarseTargetQpc =
                            g_coarseFineProbeCurrentFinalTargetQpc -
                            g_coarseFineProbeGuardCounts;


                        LARGE_INTEGER qpcBeforeArmLi = {};


                        RtQueryPerformanceCounter(
                            &qpcBeforeArmLi);


                        uint64_t qpcBeforeArm =
                            (uint64_t)
                            qpcBeforeArmLi.QuadPart;


                        if (firstCoarseTargetQpc >
                            qpcBeforeArm)
                        {
                            uint64_t remainingCounts =
                                firstCoarseTargetQpc -
                                qpcBeforeArm;


                            uint64_t numerator =
                                remainingCounts *
                                10000000ULL;


                            uint64_t relative100ns =
                                (
                                    numerator +
                                    g_coarseFineProbeQpcFrequency -
                                    1ULL
                                    )
                                /
                                g_coarseFineProbeQpcFrequency;


                            if (relative100ns == 0)
                            {
                                relative100ns =
                                    1;
                            }


                            LARGE_INTEGER firstCoarseExpiration = {};

                            firstCoarseExpiration.QuadPart =
                                (LONGLONG)
                                relative100ns;


                            LARGE_INTEGER armStartQpc = {};
                            LARGE_INTEGER armEndQpc = {};


                            RtQueryPerformanceCounter(
                                &armStartQpc);


                            firstCoarseArmOk =
                                RtSetTimerRelative(
                                    g_coarseFineProbeTimer,
                                    &firstCoarseExpiration,
                                    NULL);


                            if (!firstCoarseArmOk)
                            {
                                firstCoarseArmError =
                                    GetLastError();
                            }


                            RtQueryPerformanceCounter(
                                &armEndQpc);


                            if (armEndQpc.QuadPart >=
                                armStartQpc.QuadPart)
                            {
                                uint64_t armCounts =
                                    (uint64_t)
                                    (
                                        armEndQpc.QuadPart -
                                        armStartQpc.QuadPart
                                        );


                                firstCoarseArmCostNs =
                                    (
                                        armCounts *
                                        1000000000ULL
                                        )
                                    /
                                    g_coarseFineProbeQpcFrequency;
                            }


                            if (firstCoarseArmOk)
                            {
                                g_coarseFineProbeMode =
                                    1;
                            }
                        }
                        else
                        {
                            firstCoarseArmError =
                                ERROR_TIMEOUT;
                        }
                    }
                    else
                    {
                        firstCoarseArmError =
                            GetLastError();
                    }
                }


                int targetWaitedMs =
                    0;


                if (firstCoarseArmOk)
                {
                    while (g_coarseFineProbeDone == 0 &&
                        targetWaitedMs <
                        TARGET_TIMEOUT_MS)
                    {
                        RtSleep(
                            1);

                        targetWaitedMs++;
                    }
                }


                MemoryBarrier();


                uint64_t intervalAvgNs =
                    0;


                if (g_coarseFineProbeIntervalSamples > 0)
                {
                    intervalAvgNs =
                        g_coarseFineProbeIntervalSumNs /
                        g_coarseFineProbeIntervalSamples;
                }


                int64_t coarseErrorAvgNs =
                    0;


                if (g_coarseFineProbeCoarseErrorSamples > 0)
                {
                    coarseErrorAvgNs =
                        g_coarseFineProbeCoarseErrorSumNs /
                        (int64_t)
                        g_coarseFineProbeCoarseErrorSamples;
                }


                uint64_t fineWaitAvgNs =
                    0;


                if (g_coarseFineProbeFineWaitSamples > 0)
                {
                    fineWaitAvgNs =
                        g_coarseFineProbeFineWaitSumNs /
                        g_coarseFineProbeFineWaitSamples;
                }


                int64_t releaseErrorAvgNs =
                    0;


                if (g_coarseFineProbeReleaseErrorSamples > 0)
                {
                    releaseErrorAvgNs =
                        g_coarseFineProbeReleaseErrorSumNs /
                        (int64_t)
                        g_coarseFineProbeReleaseErrorSamples;
                }


                uint64_t rearmCostAvgNs =
                    0;


                if (g_coarseFineProbeRearmCostSamples > 0)
                {
                    rearmCostAvgNs =
                        g_coarseFineProbeRearmCostSumNs /
                        g_coarseFineProbeRearmCostSamples;
                }


                bool probePassed =
                    (
                        warmupArmOk &&
                        g_coarseFineProbeWarmupDone != 0 &&
                        firstCoarseArmOk &&
                        g_coarseFineProbeDone != 0 &&
                        g_coarseFineProbeCallbacks ==
                        TARGET_CALLBACKS &&
                        g_coarseFineProbeRearmOk ==
                        EXPECTED_REARMS &&
                        g_coarseFineProbeRearmFail == 0 &&
                        g_coarseFineProbeFineMiss == 0 &&
                        g_coarseFineProbeNextCoarsePast == 0
                        );


                // PASS 必須同時滿足：warm-up／first arm 成功、4000 callbacks 完成、
                // 3999 次 re-arm 全成功，而且沒有 FineMiss 或 next target 已過期。
                // Interval/CoarseErr/FineWait/ReleaseErr/RearmCost 是效能量測，不直接
                // 參與 PASS 門檻；查看 LOG 時用它們定位 jitter 與 timer API 成本。
                RtPrintf(
                    "[COARSE-FINE-QPC-PROBE] "
                    "Result:%s | "
                    "Warmup:%s Err:%lu ArmWindow:%llu ns | "
                    "FirstCoarse:%s Err:%lu Cost:%llu ns | "
                    "Callbacks:%ld/%ld | "
                    "RearmOK:%ld/%ld | "
                    "RearmFail:%ld | "
                    "FineMiss:%ld | "
                    "NextCoarsePast:%ld | "
                    "Error:%ld | "
                    "Wait:%dms | "
                    "Guard:%llu ns | "
                    "Interval Avg:%llu Min:%llu Max:%llu ns | "
                    "CoarseErr Avg:%+lld Min:%+lld Max:%+lld ns | "
                    "FineWait Avg:%llu Min:%llu Max:%llu ns | "
                    "ReleaseErr Avg:%+lld Min:%+lld Max:%+lld ns | "
                    "RearmCost Avg:%llu Min:%llu Max:%llu ns\n",

                    probePassed
                    ? "PASS"
                    : "FAIL",

                    warmupArmOk
                    ? "OK"
                    : "FAIL",

                    (unsigned long)
                    warmupArmError,

                    (unsigned long long)
                    warmupArmWindowNs,

                    firstCoarseArmOk
                    ? "OK"
                    : "FAIL",

                    (unsigned long)
                    firstCoarseArmError,

                    (unsigned long long)
                    firstCoarseArmCostNs,

                    (long)
                    g_coarseFineProbeCallbacks,

                    (long)
                    TARGET_CALLBACKS,

                    (long)
                    g_coarseFineProbeRearmOk,

                    (long)
                    EXPECTED_REARMS,

                    (long)
                    g_coarseFineProbeRearmFail,

                    (long)
                    g_coarseFineProbeFineMiss,

                    (long)
                    g_coarseFineProbeNextCoarsePast,

                    (long)
                    g_coarseFineProbeLastError,

                    targetWaitedMs,

                    (unsigned long long)
                    COARSE_GUARD_NS,

                    (unsigned long long)
                    intervalAvgNs,

                    (unsigned long long)
                    g_coarseFineProbeIntervalMinNs,

                    (unsigned long long)
                    g_coarseFineProbeIntervalMaxNs,

                    (long long)
                    coarseErrorAvgNs,

                    (long long)
                    g_coarseFineProbeCoarseErrorMinNs,

                    (long long)
                    g_coarseFineProbeCoarseErrorMaxNs,

                    (unsigned long long)
                    fineWaitAvgNs,

                    (unsigned long long)
                    g_coarseFineProbeFineWaitMinNs,

                    (unsigned long long)
                    g_coarseFineProbeFineWaitMaxNs,

                    (long long)
                    releaseErrorAvgNs,

                    (long long)
                    g_coarseFineProbeReleaseErrorMinNs,

                    (long long)
                    g_coarseFineProbeReleaseErrorMaxNs,

                    (unsigned long long)
                    rearmCostAvgNs,

                    (unsigned long long)
                    g_coarseFineProbeRearmCostMinNs,

                    (unsigned long long)
                    g_coarseFineProbeRearmCostMaxNs);


                if (!RtDeleteTimer(
                    g_coarseFineProbeTimer))
                {
                    RtPrintf(
                        "[COARSE-FINE-QPC-PROBE] "
                        "DELETE FAILED | "
                        "Error:%lu\n",

                        (unsigned long)
                        GetLastError());
                }


                // 無論 PASS/FAIL 都不再使用 probe timer；正式 PDO timer 在後面另建。
                g_coarseFineProbeTimer =
                    NULL;
            }
        }
    }
    else
    {
        // 關閉時不建立 Probe Timer，也不執行 4000 次 callback 或 QPC busy-wait。
        // 正式 PDO Timer 建立、warm-up 與 DC Alignment 仍會照常執行。
        RtPrintf(
            "[COARSE-FINE-QPC-PROBE] "
            "DISABLED | Continuing normal PDO startup.\n");
    }


    // ============================================================
 // Step 3：建立真正 PDO Timer
 // Priority 80 高於 Probe 62 與 Main Thread 50；callback 使用 CLOCK_2。
 // ============================================================

    hTimer_PDO =
        RtCreateTimer(
            NULL,
            0,
            GlobalTimerHandler_PDO,
            this,
            80,
            CLOCK_2);


    if (hTimer_PDO == NULL)
    {
        DEBUG_PRINT(
            "GlobalTimerHandler_PDO Error>>%d\n",
            GetLastError());

        return -1;
    }


    // ============================================================
    // Step 4：PDO One-Shot Scheduler V1B，同一個正式 timer 的 warm-up
    //
    // 新建 timer 的第一次 relative arm 偶爾有一次性較大成本，因此真正 PDO timer
    // 也要 warm-up 一次。StartupMode=1 時 Runtime callback 只設 WarmupDone 後返回，
    // 不做 EtherCAT I/O、不 re-arm；PASS 後才切換 StartupMode=2 LIVE。
    // ============================================================

    g_pdoOneShotTimerHandle =
        hTimer_PDO;

    g_pdoOneShotWarmupCallbackDone =
        0;

    g_pdoOneShotStartupMode =
        1;                      // WARM-UP


    LARGE_INTEGER pdoWarmupExpiration = {};

    pdoWarmupExpiration.QuadPart =
        50000;                  // 5 ms, units = 100 ns


    uint64_t pdoWarmupBeforeNs =
        GetCurrentMasterTimeNs();


    BOOL pdoWarmupArmOk =
        RtSetTimerRelative(
            hTimer_PDO,
            &pdoWarmupExpiration,
            NULL);


    DWORD pdoWarmupArmError =
        ERROR_SUCCESS;


    if (!pdoWarmupArmOk)
    {
        pdoWarmupArmError =
            GetLastError();
    }


    uint64_t pdoWarmupAfterNs =
        GetCurrentMasterTimeNs();


    uint64_t pdoWarmupArmElapsedNs =
        0;


    if (pdoWarmupAfterNs >=
        pdoWarmupBeforeNs)
    {
        pdoWarmupArmElapsedNs =
            pdoWarmupAfterNs -
            pdoWarmupBeforeNs;
    }


    int pdoWarmupWaitedMs =
        0;


    if (pdoWarmupArmOk)
    {
        // 這是非即時啟動執行緒的 bounded wait；最多等待 500 ms。
        while (g_pdoOneShotWarmupCallbackDone == 0 &&
            pdoWarmupWaitedMs < 500)
        {
            RtSleep(
                1);

            pdoWarmupWaitedMs++;
        }
    }


    MemoryBarrier();


    bool pdoWarmupPassed =
        (
            pdoWarmupArmOk &&
            g_pdoOneShotWarmupCallbackDone != 0
            );


    RtPrintf(
        "[PDO-ONESHOT-WARMUP] "
        "Result:%s | "
        "ArmElapsed:%llu ns | "
        "Wait:%d ms | "
        "Error:%lu\n",

        pdoWarmupPassed
        ? "PASS"
        : "FAIL",

        (unsigned long long)
        pdoWarmupArmElapsedNs,

        pdoWarmupWaitedMs,

        (unsigned long)
        pdoWarmupArmError);


    if (!pdoWarmupPassed)
    {
        RtPrintf(
            "[PDO-ONESHOT-WARMUP] "
            "Real PDO timer warm-up failed. "
            "V1B startup aborted before OP.\n");

        g_pdoOneShotStartupMode =
            0;

        return -1;
    }


    // warm-up one-shot 已到期且 inactive；下一次 arm 就是正式 DC-aligned one-shot。
    g_pdoOneShotStartupMode =
        2;                      // LIVE


    // ============================================================
    // Step 5：PDO Timer DC Alignment
    //
    // 優先使用 DC-aligned absolute timer；量測、換算或 RtSetTimer 任何一步失敗，
    // 最後改用 relative 250 us one-shot。Fallback 可維持 PDO 通訊，但第一個
    // callback 不保證落在設計的 EtherCAT DC phase 0。
    // ============================================================
    bool pdoTimerStarted =
        false;


    // ============================================================
    // PDO <-> EtherCAT DC Startup Alignment V2
    //
    // 1. 對 DC Reference slave 的 0x0910 連續取 8 個 64-bit System Time 樣本。
    // 2. 用 MasterBefore/MasterAfter 中點近似該次 DC read 對應的 CLOCK_2 時刻。
    // 3. 選 RTT 最小的有效樣本，降低通訊延遲不對稱造成的 offset 誤差。
    // 4. 最佳 RTT 超過 1 ms 就拒絕 absolute alignment，避免用啟動異常樣本。
    // 5. DC absolute time 可能超過 INT64_MAX；offset 使用 uint64 magnitude+direction，
    //    避免直接做 signed subtraction 造成溢位。
    // ============================================================

    if (Motor_Start_Index >= 0)
    {
        const int ALIGN_SAMPLE_COUNT =
            8;                 // 增加樣本可提高選到低 RTT 的機會，但會延長啟動。


        const uint64_t MAX_ALIGN_RTT_NS =
            1000000ULL;        // 最佳樣本仍超過 1 ms 就不採用。


        uint64_t bestRttNs =
            ~0ULL;


        uint64_t bestMasterBeforeNs =
            0;


        uint64_t bestMasterAfterNs =
            0;


        uint64_t bestMasterMidNs =
            0;


        uint64_t bestDcReferenceNs =
            0;


        int bestDcWkc =
            0;


        // ========================================================
        // 1. 多次讀取 DC Reference 0x0910；timeout=20 是既有 register read 參數。
        // ========================================================

        for (int sample = 0;
            sample < ALIGN_SAMPLE_COUNT;
            sample++)
        {
            uint64_t masterBeforeNs =
                GetCurrentMasterTimeNs();


            uint64_t dcReferenceNs =
                0;


            int dcWkc =
                ecx_FPRD(
                    m_slaveInfo[
                        Motor_Start_Index
                    ].configAddr,
                    0x0910,
                            &dcReferenceNs,
                            8,
                            20);


            uint64_t masterAfterNs =
                GetCurrentMasterTimeNs();


            uint64_t rttNs =
                0;


            if (masterAfterNs >=
                masterBeforeNs)
            {
                rttNs =
                    masterAfterNs -
                    masterBeforeNs;
            }


            RtPrintf(
                "[PDO-DC-SAMPLE] "
                "%d/%d | "
                "WKC:%d | "
                "RTT:%llu ns | "
                "DC:%llu\n",

                sample + 1,

                ALIGN_SAMPLE_COUNT,

                dcWkc,

                (unsigned long long)
                rttNs,

                (unsigned long long)
                dcReferenceNs);


            // ====================================================
            // 只接受 WKC>0、DC time>0、Master time 單調且 RTT>0 的樣本；
            // 每次找到更小 RTT 就更新 best sample。
            // ====================================================

            if (dcWkc > 0 &&
                dcReferenceNs > 0 &&
                masterAfterNs >=
                masterBeforeNs &&
                rttNs > 0)
            {
                if (rttNs <
                    bestRttNs)
                {
                    bestRttNs =
                        rttNs;


                    bestMasterBeforeNs =
                        masterBeforeNs;


                    bestMasterAfterNs =
                        masterAfterNs;


                    bestMasterMidNs =
                        masterBeforeNs +
                        (
                            rttNs /
                            2ULL
                            );


                    bestDcReferenceNs =
                        dcReferenceNs;


                    bestDcWkc =
                        dcWkc;
                }
            }
        }


        // ========================================================
        // 2. 驗證最佳樣本；沒有有效 WKC 或 RTT 超過門檻就走 fallback。
        // ========================================================

        if (bestDcWkc > 0 &&
            bestRttNs != ~0ULL &&
            bestRttNs <=
            MAX_ALIGN_RTT_NS)
        {
            // ====================================================
            // 計算 Master/DC offset，但不直接把兩個 64-bit absolute ns 轉成 int64。
            // masterAheadOfDc 保存方向，masterDcOffsetMagnitudeNs 保存絕對差值。
            // ====================================================
            bool masterAheadOfDc =
                (
                    bestMasterMidNs >=
                    bestDcReferenceNs
                    );


            uint64_t masterDcOffsetMagnitudeNs =
                0;


            if (masterAheadOfDc)
            {
                masterDcOffsetMagnitudeNs =
                    bestMasterMidNs -
                    bestDcReferenceNs;
            }
            else
            {
                masterDcOffsetMagnitudeNs =
                    bestDcReferenceNs -
                    bestMasterMidNs;
            }


            RtPrintf(
                "\n"
                "============================================================\n"
                "[PDO-DC-BEST-SAMPLE]\n"
                "WKC        :%d\n"
                "RTT        :%llu ns\n"
                "MasterMid  :%llu ns\n"
                "DCReference:%llu ns\n"
                "Direction  :%s\n"
                "Offset     :%llu ns\n"
                "============================================================\n",

                bestDcWkc,

                (unsigned long long)
                bestRttNs,

                (unsigned long long)
                bestMasterMidNs,

                (unsigned long long)
                bestDcReferenceNs,

                masterAheadOfDc
                ? "MASTER_AHEAD"
                : "MASTER_BEHIND",

                (unsigned long long)
                masterDcOffsetMagnitudeNs);


            // ====================================================
            // 3. 用最佳樣本的固定 offset，把目前 CLOCK_2 Master time 投影成 DC now。
            // ====================================================

            uint64_t masterNowNs =
                GetCurrentMasterTimeNs();


            uint64_t estimatedDcNowNs =
                0;


            bool dcNowValid =
                true;


            if (masterAheadOfDc)
            {
                // Master = DC + Offset
                //
                // DC = Master - Offset

                if (masterNowNs >=
                    masterDcOffsetMagnitudeNs)
                {
                    estimatedDcNowNs =
                        masterNowNs -
                        masterDcOffsetMagnitudeNs;
                }
                else
                {
                    dcNowValid =
                        false;
                }
            }
            else
            {
                // DC = Master + Offset

                estimatedDcNowNs =
                    masterNowNs +
                    masterDcOffsetMagnitudeNs;
            }


            if (dcNowValid)
            {
                // ================================================
                // DC cycle configuration：PDO 4 kHz，第一個 PDO 目標落在 DC phase 0。
                // ================================================

                const uint64_t PDO_DC_CYCLE_NS =
                    250000ULL;


                // Servo Sync0 在 ENI 設為 125 us；PDO 選 phase 0，讓 frame 在 Sync0
                // 前約半個 cycle 執行。這兩個相位是整體設計，不可單獨任意更改。
                const uint64_t PDO_TARGET_PHASE_NS =
                    0ULL;


                // 先把候選目標放到 DC now 至少 20 ms 後，給啟動 LOG/計算足夠餘裕。
                const uint64_t START_GUARD_NS =
                    20000000ULL;


                uint64_t earliestDcNs =
                    estimatedDcNowNs +
                    START_GUARD_NS;


                // ================================================
                // 找第一個同時滿足 target % 250000 = phase 且不早於 earliestDcNs 的 DC 時刻。
                // ================================================

                uint64_t cycleBaseNs =
                    (
                        earliestDcNs /
                        PDO_DC_CYCLE_NS
                        )
                    *
                    PDO_DC_CYCLE_NS;


                uint64_t targetDcNs =
                    cycleBaseNs +
                    PDO_TARGET_PHASE_NS;


                if (targetDcNs <
                    earliestDcNs)
                {
                    targetDcNs +=
                        PDO_DC_CYCLE_NS;
                }


                // ================================================
                // 4. 依 offset 方向把 DC target 轉回 CLOCK_2 absolute target；
                // 全程使用 unsigned magnitude，並檢查 subtraction 是否會 underflow。
                // ================================================

                uint64_t targetMasterNs =
                    0;


                bool targetMasterValid =
                    true;


                if (masterAheadOfDc)
                {
                    // Master 領先 DC：Master = DC + Offset。

                    targetMasterNs =
                        targetDcNs +
                        masterDcOffsetMagnitudeNs;
                }
                else
                {
                    // Master 落後 DC：Master = DC - Offset；先檢查 targetDc >= offset。

                    if (targetDcNs >=
                        masterDcOffsetMagnitudeNs)
                    {
                        targetMasterNs =
                            targetDcNs -
                            masterDcOffsetMagnitudeNs;
                    }
                    else
                    {
                        targetMasterValid =
                            false;
                    }
                }


                if (targetMasterValid)
                {
                    // ============================================================
                    // PDO DC Startup Alignment V2.1
                    //
                    // 1. 在 RtSetTimer() 前先取得最新 CLOCK_2，確認 target 尚有足夠 lead。
                    // 2. 若 target 太近，只能整數增加 N 個 250 us cycle，保持 DC phase 不變。
                    // 3. 用 absolute RtSetTimer() arm 真正 PDO timer，之後才輸出詳細 LOG。
                    // 4. HAL 目前 25 us；PDO 250 us。HAL 是 RTX64 時基解析度／排程因素，
                    //    不是這裡的 PDO phase，兩者不可混成同一參數。
                    // ============================================================

                    const uint64_t MIN_TIMER_LEAD_NS =
                        5000000ULL;       // RtSetTimer 前至少保留 5 ms。


                    // ------------------------------------------------------------
                    // 在真正 arm 前重新讀 CLOCK_2，避免前面計算與 LOG 已消耗 start guard。
                    // ------------------------------------------------------------

                    uint64_t preSetNowNs =
                        GetCurrentMasterTimeNs();


                    // ------------------------------------------------------------
                    // 若 target 距現在 <=5 ms，就同步把 targetMasterNs 與 targetDcNs
                    // 向後推 N 個完整 250 us cycle。兩者加相同 advanceNs，因此
                    // targetDcNs % 250000 仍等於 PDO_TARGET_PHASE_NS，不破壞 DC phase。
                    // ------------------------------------------------------------

                    uint64_t minimumSafeTargetNs =
                        preSetNowNs +
                        MIN_TIMER_LEAD_NS;


                    uint64_t advancedCycles =
                        0;


                    if (targetMasterNs <=
                        minimumSafeTargetNs)
                    {
                        uint64_t deltaNs =
                            minimumSafeTargetNs -
                            targetMasterNs;


                        advancedCycles =
                            (
                                deltaNs /
                                PDO_DC_CYCLE_NS
                                )
                            +
                            1ULL;


                        uint64_t advanceNs =
                            advancedCycles *
                            PDO_DC_CYCLE_NS;


                        targetMasterNs +=
                            advanceNs;


                        targetDcNs +=
                            advanceNs;
                    }


                    // ------------------------------------------------------------
                    // remainingNs 只用於啟動 LOG，表示 arm 前尚有多少 CLOCK_2 lead。
                    // ------------------------------------------------------------

                    uint64_t remainingNs =
                        targetMasterNs -
                        preSetNowNs;


                    // ------------------------------------------------------------
                    // absolute CLOCK_2 ns -> 100 ns timer units，採 ceiling：
                    // (targetMasterNs + 99) / 100，避免 truncation 讓 timer 提早到期。
                    // ------------------------------------------------------------

                    uint64_t expiration100ns =
                        (
                            targetMasterNs +
                            99ULL
                            )
                        /
                        100ULL;


                    const uint64_t
                        MAX_LARGE_INTEGER_POSITIVE =
                        0x7FFFFFFFFFFFFFFFULL;


                    if (expiration100ns <=
                        MAX_LARGE_INTEGER_POSITIVE)
                    {
                        LARGE_INTEGER
                            firstExpiration_PDO;


                        firstExpiration_PDO.QuadPart =
                            (LONGLONG)
                            expiration100ns;


                        // ========================================================
                        // 關鍵順序：先 RtSetTimer()，再做 RtPrintf。
                        // arm 前不增加 EtherCAT I/O、Sleep 或不必要輸出，避免 target
                        // 在真正提交 timer 前被啟動診斷工作吃掉 lead。
                        //
                        // ========================================================

                        BOOL timerSetOk =
                            RtSetTimer(
                                hTimer_PDO,
                                &firstExpiration_PDO,
                                NULL);


                        // 只有 RtSetTimer 失敗才讀 GetLastError，避免成功時留下舊錯誤碼。
                        DWORD timerSetError =
                            ERROR_SUCCESS;


                        if (!timerSetOk)
                        {
                            timerSetError =
                                GetLastError();
                        }


                        uint64_t postSetNowNs =
                            GetCurrentMasterTimeNs();


                        uint64_t armCallElapsedNs =
                            0;


                        if (postSetNowNs >=
                            preSetNowNs)
                        {
                            armCallElapsedNs =
                                postSetNowNs -
                                preSetNowNs;
                        }


                        // ========================================================
                        // timer 已完成 arm 後才輸出完整對齊資料；此時 callback 尚在未來。
                        // ========================================================

                        RtPrintf(
                            "\n"
                            "============================================================\n"
                            "[PDO-DC-ALIGN-V2.1]\n"
                            "EstimatedDCNow:%llu ns\n"
                            "TargetDC      :%llu ns\n"
                            "TargetPhase   :%llu ns\n"
                            "TargetMaster  :%llu ns\n"
                            "PreSetNow     :%llu ns\n"
                            "Remaining     :%llu ns\n"
                            "AdvanceCycles :%llu\n"
                            "Expire100ns   :%llu\n"
                            "ArmElapsed    :%llu ns\n"
                            "Result        :%s\n"
                            "Error         :%lu\n"
                            "============================================================\n",

                            (unsigned long long)
                            estimatedDcNowNs,

                            (unsigned long long)
                            targetDcNs,

                            (unsigned long long)
                            (
                                targetDcNs %
                                PDO_DC_CYCLE_NS
                                ),

                            (unsigned long long)
                            targetMasterNs,

                            (unsigned long long)
                            preSetNowNs,

                            (unsigned long long)
                            remainingNs,

                            (unsigned long long)
                            advancedCycles,

                            (unsigned long long)
                            expiration100ns,

                            (unsigned long long)
                            armCallElapsedNs,

                            timerSetOk
                            ? "SUCCESS"
                            : "FAILED",

                            (unsigned long)
                            timerSetError);


                        if (timerSetOk)
                        {
                            pdoTimerStarted =
                                true;


                            RtPrintf(
                                "[PDO-DC-ALIGN-V2.1] "
                                "SUCCESS | "
                                "TargetPhase:%llu ns | "
                                "Sync0:125000 ns\n",

                                (unsigned long long)
                                PDO_TARGET_PHASE_NS);
                        }
                        else
                        {
                            RtPrintf(
                                "[PDO-DC-ALIGN-V2.1] "
                                "RtSetTimer FAILED | "
                                "Error:%lu\n",

                                (unsigned long)
                                timerSetError);
                        }
                    }
                    else
                    {
                        RtPrintf(
                            "[PDO-DC-ALIGN-V2.1] "
                            "Expiration exceeds "
                            "LARGE_INTEGER.\n");
                    }
                }
                else
                {
                    RtPrintf(
                        "[PDO-DC-ALIGN-V2] "
                        "Target master conversion invalid.\n");
                }
            }
            else
            {
                RtPrintf(
                    "[PDO-DC-ALIGN-V2] "
                    "DC now calculation invalid.\n");
            }
        }
        else
        {
            RtPrintf(
                "\n"
                "============================================================\n"
                "[PDO-DC-ALIGN-V2] Measurement rejected\n"
                "BestRTT:%llu ns\n"
                "Limit  :%llu ns\n"
                "============================================================\n",

                (unsigned long long)
                bestRttNs,

                (unsigned long long)
                MAX_ALIGN_RTT_NS);
        }
    }
    else
    {
        RtPrintf(
            "[PDO-DC-ALIGN-V2] "
            "Motor_Start_Index invalid:%d\n",

            Motor_Start_Index);
    }


    if (!pdoTimerStarted)
    {
        RtPrintf(
            "\n"
            "============================================================\n"
            "[PDO-DC-ALIGN-FALLBACK]\n"
            "DC aligned timer was not started.\n"
            "Fallback to one-shot RtSetTimerRelative 250us.\n"
            "============================================================\n");


        if (!RtSetTimerRelative(
            hTimer_PDO,
            &liPeriod_PDO,
            NULL))
        {
            RtPrintf(
                "[PDO-DC-ALIGN-FALLBACK] "
                "RtSetTimerRelative FAILED | "
                "Error:%lu\n",

                (unsigned long)
                GetLastError());


            // Absolute DC alignment 已失敗，relative fallback 也無法 arm；
            // 此時沒有可運行的 PDO timer，必須讓上層中止 EtherCAT 啟動。
            return -1;
        }


        // Relative fallback 成功只表示 PDO timer 已啟動；它不等同於
        // 第一個 PDO callback 已按 DC phase 0 對齊。後續仍需用 Runtime/DC LOG 驗證。
        pdoTimerStarted =
            true;


        RtPrintf(
            "[PDO-DC-ALIGN-FALLBACK] "
            "Relative one-shot PDO Timer started successfully.\n");
    }

    return 0;
}
