#include "EtherCatMaster.h"
#include "EtherCatMaster_DC_Internal.h"
#include "NicDriver.h"
#include <windows.h>
#include <rtapi.h>
#include <rtssapi.h>
#include <stdio.h>

/*
 * 檔案：EtherCatMaster_DC_Diagnostics.cpp
 * 版本：EtherCAT DC Release Candidate RC1.2（調參與 Debug 判讀註解版）
 *
 * 功能：
 * - 由 Priority 50 的 1000 ms 工作讀取 Priority 64 發布的診斷快照。
 * - 輸出 DC、PDO Scheduler、RX Deadline、TX ownership、WKC 與健康總結。
 * - 診斷端只讀取 snapshot，不直接修改 DC、Motion、NC 或 PDO 控制狀態。
 *
 * Snapshot 規則：
 * - Sequence 為 odd：Priority 64 正在寫入，不可使用。
 * - Sequence 為 even 且前後一致：本次快照完整，可輸出。
 * - MemoryBarrier() 防止編譯器或 CPU 重新排序 volatile 欄位讀取。
 *
 * Timeout 顯示規則：
 * - Total：系統啟動後 Hard Timeout 累積值，永不自動清除。
 * - Recent_Timeout：最近五分鐘內的 Hard Timeout 數量。
 * - Quiet：距離最後一次 Hard Timeout 的安靜秒數。
 * - SINGLE：五分鐘內只有一次，暫時視為孤立事件。
 * - REPEATED：五分鐘內出現兩次以上，需要進一步檢查。
 * - CLEAR：最近五分鐘沒有新事件，Recent_Timeout 已歸零。
 *
 * 注意：只清除顯示用 Recent_Timeout，不會清除底層 TotalHardTimeout。
 *
 * Debug 快速判讀順序：
 *
 * 1. 先看 [DC-HEALTH-SUMMARY]
 *    正常目標：ACTIVE、PhaseGood:YES、ActualErr 接近 0、OffsetSat:NO、
 *    TripMask:0x00、Recover/Skip=0、Recent_Timeout=0、WKC=13/7、RESULT:STABLE。
 *    ActualErr 的 NEAR_ZERO 門檻是 ±1000 ns；這是顯示門檻，不是 Phase-P 的
 *    500 ns deadband。Snapshot:BUSY 偶爾一行可忽略，連續出現才要查 snapshot writer。
 *
 * 2. 若 Timeout 增加，看 [ECAT-RX-DEADLINE-MAIN]
 *    FirstRx 接近 Calls 表示大多第一次 ReceivePacket 就收到；EmptyRx 只是輪詢時
 *    暫時無 frame，不等同 HardTimeout。HardTimeout 是本視窗新事件，TotalHardTimeout
 *    是啟動後永久累積；CurrentConsecutive 回到 0 表示後續已恢復。
 *    RxElapsed Max 接近／超過 Hard deadline，才是 deadline 壓力的直接證據。
 *
 * 3. 再看 [ECAT-RX-STAGE-MAIN] 判斷卡在哪一階段
 *    PreDeadline：進入 receive 前已無時間；PostReceive：ReceivePacket 返回後才超時；
 *    SleepAtTimeout 0/1/2：timeout 發生前走過幾次 coarse sleep；
 *    ReceiveCallMax：單次 NIC receive 呼叫最久時間；TimeoutCallMax：timeout 當次最久值。
 *    若 ReceiveCallMax 突增，優先查 NAL interrupt／CPU priority；若 PreDeadline 增加，
 *    優先查 TX、Motion、NC 或前段 Handler 執行時間。
 *
 * 4. 看 [ECAT-TX-ROOT-RC1-MAIN] 與 [ECAT-WKC-MAIN]
 *    TxFrameBusy、TxNotOwner、TxSubmitFail、TxSubmitted0 正常都應維持 0。
 *    四者都為 0 但 RX timeout 增加，問題較可能在 RX interrupt／receive／排程。
 *    目前 3 軸拓撲 WKC 正常值是 LRW=13、DC=7；拓撲或 PDO mapping 改變後要重算，
 *    不能把 13/7 當成所有機台的固定標準。
 *
 * 5. 看正式 DC 控制
 *    [QPC-REAL-FF-V0C-MAIN]：State=ACTIVE、PhaseGood=YES、TripMask=0、Reject=0。
 *    Applied 是真正使用的 drift；Step 應小且緩慢。LATCHED 或 TripMask 非 0 是正式警報。
 *    [QPC-PHASE-P-ACT-V0-MAIN]：State=ACTIVE、Gate=YES、ActualErr 接近 0、
 *    OffsetSat=NO、Improve=YES。Step 正負頻繁切換代表 deadband／P 強度可能過敏；
 *    Offset 長期往單方向累積代表 Real FF 尚有 residual frequency error。
 *
 * 6. 看 One-Shot 排程
 *    [PDO-ONESHOT-INFRA-MAIN]：Control=ON、FineWait=OFF、RearmFail=0。
 *    [PDO-BOOTSTRAP-REASON-MAIN]：啟動初期少量 NotReady 可接受；穩定運轉後
 *    Bootstrap、RuntimeRecover、Skip 持續增加，表示 callback 醒來或 re-arm 太晚。
 *    Recover/Skip 是 scheduler 自救事件，不等同 EtherCAT RX HardTimeout。
 *
 * 7. 看即時負載
 *    [PDO-TIMER-MAIN] Avg 應接近 250000 ns；Short/Long 代表 callback 抖動分布。
 *    [PDO-COMBINED-MAIN] 是 LRW+FRMW round trip；[PDO-EXEC-MAIN] 是整個 Handler。
 *    PDO-EXEC Over250 理想為 0；若增加，先縮短非循環工作，不要先放寬 RX deadline。
 *
 * 主要訊息層級：
 * - 正式健康：DC-HEALTH、ECAT-RX、ECAT-TX、ECAT-WKC、REAL-FF、PHASE-P、ONESHOT。
 * - 效能定位：PDO-TIMER、PDO-COMBINED、PDO-EXEC、PDO-FINE、ECAT-SEND。
 * - Observer／Dry Run：名稱含 DRY、ROBUST、TRUSTED、RESIDUAL、V1A 的訊息；
 *   這些用來解釋來源，不應單獨判定機台失步。
 */

namespace
{
    // 純顯示參數，不會改變 RX deadline 或 DC 控制：
    // PrintDcHealthSummary() 每 1000 ms 呼叫一次，因此 300 次代表約五分鐘。
    // 調大：孤立 Timeout 需要更久才由 Recent_Timeout 清除；調小：較快恢復 STABLE，
    // 但可能把低頻重複事件過早視為雜訊。正式長跑建議維持 300 秒。
    const LONG TIMEOUT_RECENT_RESET_SECONDS = 300;

    // 以下狀態只由 Priority 50 的診斷執行緒存取，不參與即時控制。
    bool timeoutWindowInitialized = false;
    LONGLONG timeoutWindowLastTotal = 0;
    LONGLONG timeoutWindowRecent = 0;
    LONG timeoutWindowQuietSeconds = TIMEOUT_RECENT_RESET_SECONDS;

    void UpdateTimeoutWindow(
        LONGLONG totalTimeout,
        LONGLONG& recentTimeout,
        LONG& quietSeconds,
        const char*& stateText)
    {
        // 第一次執行或偵測到累積計數重新開始時，重新建立視窗基準。
        if (!timeoutWindowInitialized ||
            totalTimeout < timeoutWindowLastTotal)
        {
            timeoutWindowInitialized = true;
            timeoutWindowLastTotal = totalTimeout;
            timeoutWindowRecent = totalTimeout;
            timeoutWindowQuietSeconds =
                totalTimeout > 0 ? 0 : TIMEOUT_RECENT_RESET_SECONDS;
        }
        else
        {
            // Total 的差值就是自上一次 1000 ms 總結後新增的 Hard Timeout。
            const LONGLONG newTimeout =
                totalTimeout - timeoutWindowLastTotal;

            timeoutWindowLastTotal = totalTimeout;

            if (newTimeout > 0)
            {
                // 有新事件：累加 Recent_Timeout，並重新開始五分鐘安靜計時。
                timeoutWindowRecent += newTimeout;
                timeoutWindowQuietSeconds = 0;
            }
            else if (timeoutWindowRecent > 0)
            {
                // 沒有新事件：每次 1000 ms 診斷增加一秒。
                if (timeoutWindowQuietSeconds <
                    TIMEOUT_RECENT_RESET_SECONDS)
                {
                    ++timeoutWindowQuietSeconds;
                }

                if (timeoutWindowQuietSeconds >=
                    TIMEOUT_RECENT_RESET_SECONDS)
                {
                    // 安靜滿五分鐘，只清除短期視窗；Total 保持不變。
                    timeoutWindowRecent = 0;
                    timeoutWindowQuietSeconds =
                        TIMEOUT_RECENT_RESET_SECONDS;
                }
            }
        }

        recentTimeout = timeoutWindowRecent;
        quietSeconds = timeoutWindowQuietSeconds;
        // CLEAR/SINGLE/REPEATED 是人員查看 LOG 時的快速判讀狀態。
        stateText =
            recentTimeout == 0 ? "CLEAR" :
            recentTimeout == 1 ? "SINGLE" : "REPEATED";
    }

    void PrintDcHealthSummary()
    {
        // 最多重試三次，避免剛好撞上 Priority 64 的 snapshot publish 時段。
        for (int attempt = 0; attempt < 3; ++attempt)
        {
            LONG realSeqBefore = g_qpcRealFfV0Seq;
            LONG phaseSeqBefore = g_qpcPhasePActV0Seq;
            LONG recoverySeqBefore = g_pdoBootstrapDiagSequence;
            LONG timeoutSeqBefore = g_ecatRxDiagSequence;
            LONG wkcSeqBefore = g_pdoRtDiagSequence;

            if (realSeqBefore == 0 ||
                phaseSeqBefore == 0 ||
                recoverySeqBefore == 0 ||
                timeoutSeqBefore == 0 ||
                wkcSeqBefore == 0 ||
                (realSeqBefore & 1) != 0 ||
                (phaseSeqBefore & 1) != 0 ||
                (recoverySeqBefore & 1) != 0 ||
                (timeoutSeqBefore & 1) != 0 ||
                (wkcSeqBefore & 1) != 0)
            {
                continue;
            }

            MemoryBarrier();

            LONG realState = g_qpcRealFfV0State;
            LONG phaseGood = g_qpcRealFfV0PhaseGood;
            LONG tripMask = g_qpcRealFfV0TripMask;
            LONG phasePState = g_qpcPhasePActV0State;
            LONG phasePGate = g_qpcPhasePActV0GateGood;
            LONG phasePOffsetSat = g_qpcPhasePActV0OffsetSat;
            LONGLONG actualErrNs = g_qpcPhasePActV0ActualErrNs;
            LONGLONG totalRecover = g_pdoRuntimeRecoveryTotalEvents;
            LONGLONG totalSkip = g_pdoRuntimeRecoveryTotalSkippedCycles;
            LONGLONG totalSoftLate = g_ecatRxDiagTotalSoftLateAccepted;
            LONGLONG totalTimeout = g_ecatRxDiagTotalHardTimeout;
            LONG lrwWkc = g_pdoRtLrwWkc;
            LONG dcWkc = g_pdoRtDcWkc;

            MemoryBarrier();

            LONG realSeqAfter = g_qpcRealFfV0Seq;
            LONG phaseSeqAfter = g_qpcPhasePActV0Seq;
            LONG recoverySeqAfter = g_pdoBootstrapDiagSequence;
            LONG timeoutSeqAfter = g_ecatRxDiagSequence;
            LONG wkcSeqAfter = g_pdoRtDiagSequence;

            if (realSeqBefore != realSeqAfter ||
                phaseSeqBefore != phaseSeqAfter ||
                recoverySeqBefore != recoverySeqAfter ||
                timeoutSeqBefore != timeoutSeqAfter ||
                wkcSeqBefore != wkcSeqAfter ||
                (realSeqAfter & 1) != 0 ||
                (phaseSeqAfter & 1) != 0 ||
                (recoverySeqAfter & 1) != 0 ||
                (timeoutSeqAfter & 1) != 0 ||
                (wkcSeqAfter & 1) != 0)
            {
                continue;
            }

            const char* realStateText =
                realState == 2 ? "ACTIVE" :
                realState == 1 ? "ARMING" :
                realState == 3 ? "HOLD" :
                realState == 4 ? "LATCHED" : "WAIT";

            // 正式候選版將 ActualErr 絕對值 <= 1000 ns 視為接近零。
            bool nearZero =
                actualErrNs >= -1000 &&
                actualErrNs <= 1000;

            LONGLONG recentTimeout = 0;
            LONG timeoutQuietSeconds = 0;
            const char* timeoutStateText = "CLEAR";

            // 將永久累積 Total 轉換成五分鐘 Recent_Timeout 狀態。
            UpdateTimeoutWindow(
                totalTimeout,
                recentTimeout,
                timeoutQuietSeconds,
                timeoutStateText);

            // STABLE 使用 Recent_Timeout，而不是永久累積 Total。
            // 因此單次孤立事件安靜滿五分鐘後可恢復 STABLE；
            // Total 仍保留在同一行，方便日後統計與追查。
            bool stable =
                realState == 2 &&
                phaseGood != 0 &&
                phasePState == 2 &&
                phasePGate != 0 &&
                phasePOffsetSat == 0 &&
                nearZero &&
                tripMask == 0 &&
                totalRecover == 0 &&
                totalSkip == 0 &&
                totalSoftLate == 0 &&
                recentTimeout == 0 &&
                lrwWkc == 13 &&
                dcWkc == 7;

            // 一行總結的優先判讀：
            // 先看 RESULT，再依序看 TripMask、WKC、Recent_Timeout、Recover/Skip，
            // 最後才看 ActualErr。Total 是歷史證據，單獨不會阻止五分鐘後恢復 STABLE。
            RtPrintf(
                "[DC-HEALTH-SUMMARY] "
                "%s + PhaseGood:%s + "
                "ActualErr:%+lldns(NEAR_ZERO:%s) + "
                "OffsetSat:%s + "
                "TripMask:0x%02lX + "
                "Recover:%lld/Skip:%lld + "
                "SoftLate:%lld + "
                "Timeout Total:%lld Recent_Timeout:%lld "
                "Quiet:%ld/%lds State:%s + "
                "WKC:%ld/%ld => RESULT:%s\n",
                realStateText,
                phaseGood ? "YES" : "NO",
                (long long)actualErrNs,
                nearZero ? "YES" : "NO",
                phasePOffsetSat ? "YES" : "NO",
                (long)tripMask,
                (long long)totalRecover,
                (long long)totalSkip,
                (long long)totalSoftLate,
                (long long)totalTimeout,
                (long long)recentTimeout,
                (long)timeoutQuietSeconds,
                (long)TIMEOUT_RECENT_RESET_SECONDS,
                timeoutStateText,
                (long)lrwWkc,
                (long)dcWkc,
                stable ? "STABLE" : "CHECK");

            return;
        }

        RtPrintf(
            "[DC-HEALTH-SUMMARY] "
            "Snapshot:BUSY => RESULT:CHECK\n");
    }
}

void EtherCatMaster::PrintDcRuntimeDiagnostics()
{
    // 本函式只能在非 PDO 即時路徑呼叫。
    // 所有 RtPrintf 都集中於此，避免 Priority 64 因格式化輸出被阻塞。
    // =============================================================
    // QPC <-> S4 Trusted Drift V1A
    // Reject / Transition Reason Diagnostic Reader
    //
    // Reject mask:
    //   01 RobustAccept
    //   02 RobustLock
    //   04 BufferNotFull
    //   08 MAD
    //   10 CandidateDeviation
    //   20 RawMedianDeviation
    //
    // Diagnostic only. Control remains OFF.
    // =============================================================

    static LONG
        lastQpcDcTrustedReasonPrintedSequence =
        0;


    LONG qpcDcTrustedReasonSequenceBefore =
        g_qpcDcTrustedReasonDiagSequence;


    if (qpcDcTrustedReasonSequenceBefore != 0 &&
        (qpcDcTrustedReasonSequenceBefore & 1) == 0 &&
        qpcDcTrustedReasonSequenceBefore !=
        lastQpcDcTrustedReasonPrintedSequence)
    {
        MemoryBarrier();


        LONG rejectRobustAccept =
            g_qpcDcTrustedRejectRobustAcceptTotal;

        LONG rejectRobustLock =
            g_qpcDcTrustedRejectRobustLockTotal;

        LONG rejectBufferNotFull =
            g_qpcDcTrustedRejectBufferNotFullTotal;

        LONG rejectMad =
            g_qpcDcTrustedRejectMadTotal;

        LONG rejectCandidateDeviation =
            g_qpcDcTrustedRejectCandidateDeviationTotal;

        LONG rejectRawMedianDeviation =
            g_qpcDcTrustedRejectRawMedianDeviationTotal;


        LONG currentRejectMask =
            g_qpcDcTrustedCurrentRejectMask;

        LONG lastRejectMask =
            g_qpcDcTrustedLastRejectMask;

        LONG currentBadStreak =
            g_qpcDcTrustedBadCount;

        LONG maxBadStreak =
            g_qpcDcTrustedMaxBadStreak;


        LONG warmupToTrack =
            g_qpcDcTrustedWarmupToTrackTotal;

        LONG trackToHold =
            g_qpcDcTrustedTrackToHoldTotal;

        LONG holdToTrack =
            g_qpcDcTrustedHoldToTrackTotal;

        LONG holdToUntrusted =
            g_qpcDcTrustedHoldToUntrustedTotal;

        LONG untrustedToTrack =
            g_qpcDcTrustedUntrustedToTrackTotal;


        LONG lastTransitionFrom =
            g_qpcDcTrustedLastTransitionFrom;

        LONG lastTransitionTo =
            g_qpcDcTrustedLastTransitionTo;

        LONGLONG lastTransitionRaw =
            g_qpcDcTrustedLastTransitionRawPpb;

        LONGLONG lastTransitionMedian =
            g_qpcDcTrustedLastTransitionMedianPpb;

        LONGLONG lastTransitionMad =
            g_qpcDcTrustedLastTransitionMadPpb;

        LONGLONG lastTransitionTrusted =
            g_qpcDcTrustedLastTransitionTrustedPpb;


        MemoryBarrier();


        LONG qpcDcTrustedReasonSequenceAfter =
            g_qpcDcTrustedReasonDiagSequence;


        if (qpcDcTrustedReasonSequenceBefore ==
            qpcDcTrustedReasonSequenceAfter &&
            (qpcDcTrustedReasonSequenceAfter & 1) == 0)
        {
            lastQpcDcTrustedReasonPrintedSequence =
                qpcDcTrustedReasonSequenceAfter;


            const char* currentRejectText =
                "NONE";

            const char* lastRejectText =
                "NONE";


            if (currentRejectMask != 0)
            {
                if (currentRejectMask == 0x01)
                    currentRejectText = "ROBUST_ACCEPT";
                else if (currentRejectMask == 0x02)
                    currentRejectText = "ROBUST_LOCK";
                else if (currentRejectMask == 0x04)
                    currentRejectText = "BUFFER";
                else if (currentRejectMask == 0x08)
                    currentRejectText = "MAD";
                else if (currentRejectMask == 0x10)
                    currentRejectText = "CANDIDATE_DEV";
                else if (currentRejectMask == 0x20)
                    currentRejectText = "RAW_MEDIAN";
                else
                    currentRejectText = "MULTI";
            }


            if (lastRejectMask != 0)
            {
                if (lastRejectMask == 0x01)
                    lastRejectText = "ROBUST_ACCEPT";
                else if (lastRejectMask == 0x02)
                    lastRejectText = "ROBUST_LOCK";
                else if (lastRejectMask == 0x04)
                    lastRejectText = "BUFFER";
                else if (lastRejectMask == 0x08)
                    lastRejectText = "MAD";
                else if (lastRejectMask == 0x10)
                    lastRejectText = "CANDIDATE_DEV";
                else if (lastRejectMask == 0x20)
                    lastRejectText = "RAW_MEDIAN";
                else
                    lastRejectText = "MULTI";
            }


            const char* fromText =
                "NONE";

            const char* toText =
                "NONE";


            if (lastTransitionFrom == 0)
                fromText = "WARMUP";
            else if (lastTransitionFrom == 1)
                fromText = "TRACK";
            else if (lastTransitionFrom == 2)
                fromText = "HOLD";
            else if (lastTransitionFrom == 3)
                fromText = "UNTRUSTED";


            if (lastTransitionTo == 0)
                toText = "WARMUP";
            else if (lastTransitionTo == 1)
                toText = "TRACK";
            else if (lastTransitionTo == 2)
                toText = "HOLD";
            else if (lastTransitionTo == 3)
                toText = "UNTRUSTED";


            RtPrintf(
                "[QPC-DRIFT-TRUST-REASON-MAIN] "
                "RejectTotal "
                "RobustAccept:%ld "
                "RobustLock:%ld "
                "Buffer:%ld "
                "MAD:%ld "
                "CandidateDev:%ld "
                "RawMedianDev:%ld | "
                "CurrentReject:%s(0x%02lX) "
                "LastReject:%s(0x%02lX) | "
                "BadNow:%ld MaxBad:%ld | "
                "Transitions "
                "W2T:%ld T2H:%ld H2T:%ld "
                "H2U:%ld U2T:%ld | "
                "LastTransition:%s->%s "
                "Raw:%+lld Median:%+lld "
                "MAD:%lld Trusted:%+lld ppb | "
                "Control:OFF | "
                "Snap:%ld\n",

                (long)
                rejectRobustAccept,

                (long)
                rejectRobustLock,

                (long)
                rejectBufferNotFull,

                (long)
                rejectMad,

                (long)
                rejectCandidateDeviation,

                (long)
                rejectRawMedianDeviation,

                currentRejectText,

                (long)
                currentRejectMask,

                lastRejectText,

                (long)
                lastRejectMask,

                (long)
                currentBadStreak,

                (long)
                maxBadStreak,

                (long)
                warmupToTrack,

                (long)
                trackToHold,

                (long)
                holdToTrack,

                (long)
                holdToUntrusted,

                (long)
                untrustedToTrack,

                fromText,

                toText,

                (long long)
                lastTransitionRaw,

                (long long)
                lastTransitionMedian,

                (long long)
                lastTransitionMad,

                (long long)
                lastTransitionTrusted,

                (long)
                qpcDcTrustedReasonSequenceAfter);
        }
    }


    // =============================================================
    // QPC <-> S4 Trusted Drift V1A Dry-Run Reader
    //
    // This is the control-grade candidate layer, but still:
    //
    //     Control:OFF
    //     SchedulerUsed:-8300 ppb
    //
    // State:
    //     WARMUP / TRACK / HOLD / UNTRUSTED
    // =============================================================

    static LONG
        lastQpcDcTrustedPrintedSequence =
        0;


    LONG qpcDcTrustedSequenceBefore =
        g_qpcDcTrustedDiagSequence;


    if (qpcDcTrustedSequenceBefore != 0 &&
        (qpcDcTrustedSequenceBefore & 1) == 0 &&
        qpcDcTrustedSequenceBefore !=
        lastQpcDcTrustedPrintedSequence)
    {
        MemoryBarrier();


        LONGLONG trustedRawPpb =
            g_qpcDcTrustedRawDriftPpb;

        LONGLONG trustedMedianPpb =
            g_qpcDcTrustedRobustMedianPpb;

        LONGLONG trustedMadPpb =
            g_qpcDcTrustedRobustMadPpb;

        LONGLONG trustedDriftPpb =
            g_qpcDcTrustedDriftPpb;

        LONGLONG trustedCandidateDeviationPpb =
            g_qpcDcTrustedCandidateDeviationPpb;

        LONGLONG trustedRawMedianDeviationPpb =
            g_qpcDcTrustedRawMedianDeviationPpb;

        LONGLONG trustedSlewAppliedPpb =
            g_qpcDcTrustedSlewAppliedPpb;

        LONGLONG trustedPeriodFfPs =
            g_qpcDcTrustedPeriodFfPs;


        LONG trustedCandidateGood =
            g_qpcDcTrustedCandidateGood;

        LONG trustedValid =
            g_qpcDcTrustedValid;

        LONG trustedState =
            g_qpcDcTrustedState;

        LONG trustedWarmupGood =
            g_qpcDcTrustedWarmupGoodCount;

        LONG trustedBadCount =
            g_qpcDcTrustedBadCount;

        LONG trustedRecoveryGood =
            g_qpcDcTrustedRecoveryGoodCount;

        LONG trustedUpdateTotal =
            g_qpcDcTrustedUpdateTotal;

        LONG trustedHoldTotal =
            g_qpcDcTrustedHoldTotal;

        LONG trustedUnlockTotal =
            g_qpcDcTrustedUnlockTotal;

        LONG trustedRelockTotal =
            g_qpcDcTrustedRelockTotal;


        MemoryBarrier();


        LONG qpcDcTrustedSequenceAfter =
            g_qpcDcTrustedDiagSequence;


        if (qpcDcTrustedSequenceBefore ==
            qpcDcTrustedSequenceAfter &&
            (qpcDcTrustedSequenceAfter & 1) == 0)
        {
            lastQpcDcTrustedPrintedSequence =
                qpcDcTrustedSequenceAfter;


            const char* trustedStateText =
                "UNTRUSTED";


            if (trustedState == 0)
            {
                trustedStateText =
                    "WARMUP";
            }
            else if (trustedState == 1)
            {
                trustedStateText =
                    "TRACK";
            }
            else if (trustedState == 2)
            {
                trustedStateText =
                    "HOLD";
            }


            RtPrintf(
                "[QPC-DRIFT-TRUSTED-MAIN] "
                "Raw:%+lld ppb | "
                "Median:%+lld ppb | "
                "MAD:%lld ppb | "
                "Candidate:%s | "
                "CandidateDev:%+lld ppb | "
                "RawMedianDev:%+lld ppb | "
                "Trusted:%+lld ppb | "
                "TrustedValid:%s | "
                "State:%s | "
                "Warmup:%ld/8 | "
                "Bad:%ld/10 | "
                "Recovery:%ld | "
                "Slew:%+lld ppb | "
                "TrustedFF:%+lld ps/cycle | "
                "Updates:%ld Holds:%ld "
                "Unlocks:%ld Relocks:%ld | "
                "GateDev:1200 RawMedian:2000 "
                "MAD:800 SlewMax:50 ppb/window | "
                "SchedulerUsed:-8300 ppb | "
                "Control:OFF | "
                "Snap:%ld\n",

                (long long)
                trustedRawPpb,

                (long long)
                trustedMedianPpb,

                (long long)
                trustedMadPpb,

                trustedCandidateGood != 0
                ? "GOOD"
                : "REJECT",

                (long long)
                trustedCandidateDeviationPpb,

                (long long)
                trustedRawMedianDeviationPpb,

                (long long)
                trustedDriftPpb,

                trustedValid != 0
                ? "YES"
                : "NO",

                trustedStateText,

                (long)
                trustedWarmupGood,

                (long)
                trustedBadCount,

                (long)
                trustedRecoveryGood,

                (long long)
                trustedSlewAppliedPpb,

                (long long)
                trustedPeriodFfPs,

                (long)
                trustedUpdateTotal,

                (long)
                trustedHoldTotal,

                (long)
                trustedUnlockTotal,

                (long)
                trustedRelockTotal,

                (long)
                qpcDcTrustedSequenceAfter);
        }
    }


    // =============================================================
    // Trusted Drift -> QPC Live Feed-Forward Dry Run V1 Reader
    //
    // SHADOW ONLY.
    //
    // FixedErr:
    //   Actual real wake - real fixed -8300 target.
    //
    // ShadowErr:
    //   Same actual wake - incremental Trusted/Fallback shadow target.
    //
    // ShadowVsFixedTarget:
    //   ShadowTarget - FixedTarget.
    //
    // No real scheduler/timer behavior is changed.
    // =============================================================

    static LONG
        lastQpcLiveFfPrintedSequence =
        0;


    LONG qpcLiveFfSequenceBefore =
        g_qpcLiveFfDiagSequence;


    if (qpcLiveFfSequenceBefore != 0 &&
        (qpcLiveFfSequenceBefore & 1) == 0 &&
        qpcLiveFfSequenceBefore !=
        lastQpcLiveFfPrintedSequence)
    {
        MemoryBarrier();


        LONG liveInitialized =
            g_qpcLiveFfInitialized;

        LONG liveMode =
            g_qpcLiveFfMode;

        LONG liveTrustedSnapshotValid =
            g_qpcLiveFfTrustedSnapshotValid;

        LONG liveTrustedValid =
            g_qpcLiveFfTrustedValid;

        LONG liveTrustedState =
            g_qpcLiveFfTrustedState;


        LONGLONG liveTrustedDriftPpb =
            g_qpcLiveFfTrustedDriftPpb;

        LONGLONG liveAppliedDriftPpb =
            g_qpcLiveFfAppliedDriftPpb;

        LONGLONG liveAppliedPeriodFfPs =
            g_qpcLiveFfAppliedPeriodFfPs;


        LONGLONG liveFixedErrorNs =
            g_qpcLiveFfFixedErrorNs;

        LONGLONG liveShadowErrorNs =
            g_qpcLiveFfShadowErrorNs;

        LONGLONG liveShadowVsFixedTargetNs =
            g_qpcLiveFfShadowVsFixedTargetNs;


        LONGLONG liveWindowStartErrorNs =
            g_qpcLiveFfShadowWindowStartErrorNs;

        LONGLONG liveWindowEndErrorNs =
            g_qpcLiveFfShadowWindowEndErrorNs;

        LONGLONG liveWindowDeltaErrorNs =
            g_qpcLiveFfShadowWindowDeltaErrorNs;

        LONGLONG liveWindowMinErrorNs =
            g_qpcLiveFfShadowWindowMinErrorNs;

        LONGLONG liveWindowMaxErrorNs =
            g_qpcLiveFfShadowWindowMaxErrorNs;


        LONGLONG liveTargetDeltaStartNs =
            g_qpcLiveFfTargetDeltaWindowStartNs;

        LONGLONG liveTargetDeltaEndNs =
            g_qpcLiveFfTargetDeltaWindowEndNs;

        LONGLONG liveTargetDeltaDeltaNs =
            g_qpcLiveFfTargetDeltaWindowDeltaNs;


        LONG liveTrustedCycles =
            g_qpcLiveFfTrustedCyclesWindow;

        LONG liveFallbackCycles =
            g_qpcLiveFfFallbackCyclesWindow;

        LONG liveModeSwitches =
            g_qpcLiveFfModeSwitchesWindow;


        LONGLONG liveTotalTrustedCycles =
            g_qpcLiveFfTotalTrustedCycles;

        LONGLONG liveTotalFallbackCycles =
            g_qpcLiveFfTotalFallbackCycles;

        LONG liveTotalModeSwitches =
            g_qpcLiveFfTotalModeSwitches;

        LONG liveInitCount =
            g_qpcLiveFfInitCount;

        LONG liveSamples =
            g_qpcLiveFfSamples;


        MemoryBarrier();


        LONG qpcLiveFfSequenceAfter =
            g_qpcLiveFfDiagSequence;


        if (qpcLiveFfSequenceBefore ==
            qpcLiveFfSequenceAfter &&
            (qpcLiveFfSequenceAfter & 1) == 0)
        {
            lastQpcLiveFfPrintedSequence =
                qpcLiveFfSequenceAfter;


            const char* liveModeText =
                liveMode != 0
                ? "TRUSTED"
                : "FIXED_FALLBACK";


            const char* liveTrustedStateText =
                "UNTRUSTED";


            if (liveTrustedState == 0)
                liveTrustedStateText = "WARMUP";
            else if (liveTrustedState == 1)
                liveTrustedStateText = "TRACK";
            else if (liveTrustedState == 2)
                liveTrustedStateText = "HOLD";


            RtPrintf(
                "[QPC-LIVE-FF-DRY-MAIN] "
                "Init:%s InitCount:%ld | "
                "Mode:%s | "
                "TrustedSnapshot:%s "
                "TrustedValid:%s "
                "TrustedState:%s | "
                "FixedDrift:-8300 "
                "TrustedDrift:%+lld "
                "AppliedDrift:%+lld ppb | "
                "AppliedFF:%+lld ps/cycle | "
                "FixedErr:%+lld ns "
                "ShadowErr:%+lld ns | "
                "ShadowVsFixedTarget:%+lld ns | "
                "ShadowWin Start:%+lld End:%+lld "
                "Delta:%+lld Min:%+lld Max:%+lld ns | "
                "TargetDeltaWin Start:%+lld End:%+lld "
                "Delta:%+lld ns | "
                "Cycles Trusted:%ld Fallback:%ld "
                "ModeSwitch:%ld | "
                "TotalCycles Trusted:%lld Fallback:%lld "
                "TotalModeSwitch:%ld | "
                "Samples:%ld | "
                "Incremental:YES Retroactive:NO | "
                "SchedulerUsed:-8300 ppb | "
                "Control:OFF | "
                "Snap:%ld\n",

                liveInitialized != 0
                ? "YES"
                : "NO",

                (long)
                liveInitCount,

                liveModeText,

                liveTrustedSnapshotValid != 0
                ? "YES"
                : "NO",

                liveTrustedValid != 0
                ? "YES"
                : "NO",

                liveTrustedStateText,

                (long long)
                liveTrustedDriftPpb,

                (long long)
                liveAppliedDriftPpb,

                (long long)
                liveAppliedPeriodFfPs,

                (long long)
                liveFixedErrorNs,

                (long long)
                liveShadowErrorNs,

                (long long)
                liveShadowVsFixedTargetNs,

                (long long)
                liveWindowStartErrorNs,

                (long long)
                liveWindowEndErrorNs,

                (long long)
                liveWindowDeltaErrorNs,

                (long long)
                liveWindowMinErrorNs,

                (long long)
                liveWindowMaxErrorNs,

                (long long)
                liveTargetDeltaStartNs,

                (long long)
                liveTargetDeltaEndNs,

                (long long)
                liveTargetDeltaDeltaNs,

                (long)
                liveTrustedCycles,

                (long)
                liveFallbackCycles,

                (long)
                liveModeSwitches,

                (long long)
                liveTotalTrustedCycles,

                (long long)
                liveTotalFallbackCycles,

                (long)
                liveTotalModeSwitches,

                (long)
                liveSamples,

                (long)
                qpcLiveFfSequenceAfter);
        }
    }


    // =============================================================
    // Trusted Live-FF DC Phase Predictor Dry Run V1 Reader
    //
    // Fixed/Shadow errors are relative to the SAME baseline
    // S4 DC phase captured when this predictor binds to the
    // current Live-FF shadow initialization.
    //
    // Sync0 phase assumption:
    //     DC cycle = 250000 ns
    //     Sync0    = 125000 ns
    //
    // Target-domain diagnostic only; this is NOT yet the actual
    // EtherCAT send-point phase.
    // =============================================================

    static LONG
        lastQpcLiveFfDcPhasePrintedSequence =
        0;


    LONG qpcLiveFfDcPhaseSequenceBefore =
        g_qpcLiveFfDcPhaseDiagSequence;


    if (qpcLiveFfDcPhaseSequenceBefore != 0 &&
        (qpcLiveFfDcPhaseSequenceBefore & 1) == 0 &&
        qpcLiveFfDcPhaseSequenceBefore !=
        lastQpcLiveFfDcPhasePrintedSequence)
    {
        MemoryBarrier();


        LONG dcPhaseInitialized =
            g_qpcLiveFfDcPhaseInitialized;

        LONG dcPhaseBoundInitCount =
            g_qpcLiveFfDcPhaseBoundInitCount;


        LONGLONG baselinePhaseNs =
            g_qpcLiveFfDcPhaseBaselinePhaseNs;

        LONGLONG baselineMarginNs =
            g_qpcLiveFfDcPhaseBaselineSync0MarginNs;


        LONGLONG fixedPhaseNs =
            g_qpcLiveFfDcPhaseFixedPhaseNs;

        LONGLONG shadowPhaseNs =
            g_qpcLiveFfDcPhaseShadowPhaseNs;

        LONGLONG fixedMarginNs =
            g_qpcLiveFfDcPhaseFixedSync0MarginNs;

        LONGLONG shadowMarginNs =
            g_qpcLiveFfDcPhaseShadowSync0MarginNs;


        LONGLONG fixedWrappedErrorNs =
            g_qpcLiveFfDcPhaseFixedWrappedErrorNs;

        LONGLONG shadowWrappedErrorNs =
            g_qpcLiveFfDcPhaseShadowWrappedErrorNs;

        LONGLONG fixedUnwrappedErrorNs =
            g_qpcLiveFfDcPhaseFixedUnwrappedErrorNs;

        LONGLONG shadowUnwrappedErrorNs =
            g_qpcLiveFfDcPhaseShadowUnwrappedErrorNs;

        LONGLONG shadowVsFixedNs =
            g_qpcLiveFfDcPhaseShadowVsFixedNs;


        LONGLONG fixedWindowStartNs =
            g_qpcLiveFfDcPhaseFixedWindowStartNs;

        LONGLONG fixedWindowEndNs =
            g_qpcLiveFfDcPhaseFixedWindowEndNs;

        LONGLONG fixedWindowDeltaNs =
            g_qpcLiveFfDcPhaseFixedWindowDeltaNs;

        LONGLONG fixedWindowMinNs =
            g_qpcLiveFfDcPhaseFixedWindowMinNs;

        LONGLONG fixedWindowMaxNs =
            g_qpcLiveFfDcPhaseFixedWindowMaxNs;


        LONGLONG shadowWindowStartNs =
            g_qpcLiveFfDcPhaseShadowWindowStartNs;

        LONGLONG shadowWindowEndNs =
            g_qpcLiveFfDcPhaseShadowWindowEndNs;

        LONGLONG shadowWindowDeltaNs =
            g_qpcLiveFfDcPhaseShadowWindowDeltaNs;

        LONGLONG shadowWindowMinNs =
            g_qpcLiveFfDcPhaseShadowWindowMinNs;

        LONGLONG shadowWindowMaxNs =
            g_qpcLiveFfDcPhaseShadowWindowMaxNs;


        LONGLONG fixedAbsAvgNs =
            g_qpcLiveFfDcPhaseFixedAbsAvgNs;

        LONGLONG shadowAbsAvgNs =
            g_qpcLiveFfDcPhaseShadowAbsAvgNs;


        LONG shadowBetterCount =
            g_qpcLiveFfDcPhaseShadowBetterCount;

        LONG shadowWorseCount =
            g_qpcLiveFfDcPhaseShadowWorseCount;

        LONG equalCount =
            g_qpcLiveFfDcPhaseEqualCount;


        LONGLONG mapRttAvgNs =
            g_qpcLiveFfDcPhaseMapRttAvgNs;

        LONGLONG mapRttMinNs =
            g_qpcLiveFfDcPhaseMapRttMinNs;

        LONGLONG mapRttMaxNs =
            g_qpcLiveFfDcPhaseMapRttMaxNs;

        LONG dcPhaseSamples =
            g_qpcLiveFfDcPhaseSamples;


        MemoryBarrier();


        LONG qpcLiveFfDcPhaseSequenceAfter =
            g_qpcLiveFfDcPhaseDiagSequence;


        if (qpcLiveFfDcPhaseSequenceBefore ==
            qpcLiveFfDcPhaseSequenceAfter &&
            (qpcLiveFfDcPhaseSequenceAfter & 1) == 0)
        {
            lastQpcLiveFfDcPhasePrintedSequence =
                qpcLiveFfDcPhaseSequenceAfter;


            RtPrintf(
                "[QPC-LIVE-FF-DC-PHASE-MAIN] "
                "Init:%s BoundInit:%ld | "
                "BaselinePhase:%lld ns "
                "BaselineSync0Margin:%lld ns | "
                "TargetPhase Fixed:%lld Shadow:%lld ns | "
                "Sync0Margin Fixed:%lld Shadow:%lld ns | "
                "WrappedErr Fixed:%+lld Shadow:%+lld ns | "
                "UnwrappedErr Fixed:%+lld Shadow:%+lld ns | "
                "ShadowVsFixed:%+lld ns | "
                "FixedWin Start:%+lld End:%+lld Delta:%+lld "
                "Min:%+lld Max:%+lld ns | "
                "ShadowWin Start:%+lld End:%+lld Delta:%+lld "
                "Min:%+lld Max:%+lld ns | "
                "AbsAvg Fixed:%lld Shadow:%lld ns | "
                "Compare Better:%ld Worse:%ld Equal:%ld | "
                "MapRTT Avg:%lld Min:%lld Max:%lld ns | "
                "Cycle:250000 Sync0Phase:125000 | "
                "Samples:%ld | "
                "TargetDomain:YES SendPoint:NO | "
                "Control:OFF | "
                "Snap:%ld\n",

                dcPhaseInitialized != 0
                ? "YES"
                : "NO",

                (long)
                dcPhaseBoundInitCount,

                (long long)
                baselinePhaseNs,

                (long long)
                baselineMarginNs,

                (long long)
                fixedPhaseNs,

                (long long)
                shadowPhaseNs,

                (long long)
                fixedMarginNs,

                (long long)
                shadowMarginNs,

                (long long)
                fixedWrappedErrorNs,

                (long long)
                shadowWrappedErrorNs,

                (long long)
                fixedUnwrappedErrorNs,

                (long long)
                shadowUnwrappedErrorNs,

                (long long)
                shadowVsFixedNs,

                (long long)
                fixedWindowStartNs,

                (long long)
                fixedWindowEndNs,

                (long long)
                fixedWindowDeltaNs,

                (long long)
                fixedWindowMinNs,

                (long long)
                fixedWindowMaxNs,

                (long long)
                shadowWindowStartNs,

                (long long)
                shadowWindowEndNs,

                (long long)
                shadowWindowDeltaNs,

                (long long)
                shadowWindowMinNs,

                (long long)
                shadowWindowMaxNs,

                (long long)
                fixedAbsAvgNs,

                (long long)
                shadowAbsAvgNs,

                (long)
                shadowBetterCount,

                (long)
                shadowWorseCount,

                (long)
                equalCount,

                (long long)
                mapRttAvgNs,

                (long long)
                mapRttMinNs,

                (long long)
                mapRttMaxNs,

                (long)
                dcPhaseSamples,

                (long)
                qpcLiveFfDcPhaseSequenceAfter);
        }
    }


    // =============================================================
    // S4 DC Phase Residual Drift Observer V1 Dry-Run Reader
    //
    // RawResidual:
    //   one ~1 second fixed-target phase slope in ppb.
    //
    // Median/MAD:
    //   rolling accepted residual slopes, ring = 16.
    //
    // Recommended:
    //   -8300 - MedianResidual
    //
    // This is frequency observation only.
    // No scheduler control.
    // =============================================================

    static LONG
        lastQpcDcPhaseResidualPrintedSequence =
        0;


    LONG qpcDcPhaseResidualSequenceBefore =
        g_qpcDcPhaseResidualDiagSequence;


    if (qpcDcPhaseResidualSequenceBefore != 0 &&
        (qpcDcPhaseResidualSequenceBefore & 1) == 0 &&
        qpcDcPhaseResidualSequenceBefore !=
        lastQpcDcPhaseResidualPrintedSequence)
    {
        MemoryBarrier();


        LONG residualInitialized =
            g_qpcDcPhaseResidualInitialized;

        LONG residualBoundInit =
            g_qpcDcPhaseResidualBoundInitCount;


        LONGLONG residualRawPpb =
            g_qpcDcPhaseResidualRawPpb;

        LONGLONG residualMedianPpb =
            g_qpcDcPhaseResidualMedianPpb;

        LONGLONG residualMadPpb =
            g_qpcDcPhaseResidualMadPpb;

        LONGLONG residualRecommendedPpb =
            g_qpcDcPhaseResidualRecommendedSchedulerPpb;

        LONGLONG residualTrustedPpb =
            g_qpcDcPhaseResidualTrustedDriftPpb;

        LONGLONG residualTrustedMinusRecommendedPpb =
            g_qpcDcPhaseResidualTrustedMinusRecommendedPpb;


        LONGLONG residualPhaseDeltaNs =
            g_qpcDcPhaseResidualFixedWindowDeltaNs;

        LONGLONG residualElapsedNs =
            g_qpcDcPhaseResidualWindowElapsedNs;

        LONGLONG residualRttMaxNs =
            g_qpcDcPhaseResidualWindowRttMaxNs;


        LONG residualAccepted =
            g_qpcDcPhaseResidualCurrentAccepted;

        LONG residualBufferCount =
            g_qpcDcPhaseResidualBufferCount;

        LONG residualLocked =
            g_qpcDcPhaseResidualLocked;


        LONG residualAcceptedTotal =
            g_qpcDcPhaseResidualAcceptedTotal;

        LONG residualRejectedTotal =
            g_qpcDcPhaseResidualRejectedTotal;

        LONG residualRejectElapsed =
            g_qpcDcPhaseResidualRejectElapsedTotal;

        LONG residualRejectRtt =
            g_qpcDcPhaseResidualRejectRttTotal;

        LONG residualRejectMagnitude =
            g_qpcDcPhaseResidualRejectMagnitudeTotal;


        LONGLONG residualRingMinPpb =
            g_qpcDcPhaseResidualRingMinPpb;

        LONGLONG residualRingMaxPpb =
            g_qpcDcPhaseResidualRingMaxPpb;


        MemoryBarrier();


        LONG qpcDcPhaseResidualSequenceAfter =
            g_qpcDcPhaseResidualDiagSequence;


        if (qpcDcPhaseResidualSequenceBefore ==
            qpcDcPhaseResidualSequenceAfter &&
            (qpcDcPhaseResidualSequenceAfter & 1) == 0)
        {
            lastQpcDcPhaseResidualPrintedSequence =
                qpcDcPhaseResidualSequenceAfter;


            RtPrintf(
                "[QPC-DC-PHASE-DRIFT-MAIN] "
                "Init:%s BoundInit:%ld | "
                "PhaseDelta:%+lld ns "
                "Elapsed:%lld ns | "
                "RawResidual:%+lld ppb "
                "Accept:%s | "
                "MedianResidual:%+lld ppb "
                "MAD:%lld ppb | "
                "Buffer:%ld/16 "
                "Lock:%s | "
                "RingMin:%+lld RingMax:%+lld ppb | "
                "FixedScheduler:-8300 ppb | "
                "Recommended:%+lld ppb | "
                "Trusted:%+lld ppb "
                "TrustedMinusRecommended:%+lld ppb | "
                "Accepted:%ld Rejected:%ld "
                "RejectElapsed:%ld "
                "RejectRTT:%ld "
                "RejectMagnitude:%ld | "
                "RTTMax:%lld ns "
                "GateRTT:250000 ns "
                "GateAbs:5000 ppb "
                "LockMAD:1000 ppb | "
                "Window:~1s Ring:16 | "
                "Control:OFF | "
                "Snap:%ld\n",

                residualInitialized != 0
                ? "YES"
                : "NO",

                (long)
                residualBoundInit,

                (long long)
                residualPhaseDeltaNs,

                (long long)
                residualElapsedNs,

                (long long)
                residualRawPpb,

                residualAccepted != 0
                ? "YES"
                : "NO",

                (long long)
                residualMedianPpb,

                (long long)
                residualMadPpb,

                (long)
                residualBufferCount,

                residualLocked != 0
                ? "YES"
                : "NO",

                (long long)
                residualRingMinPpb,

                (long long)
                residualRingMaxPpb,

                (long long)
                residualRecommendedPpb,

                (long long)
                residualTrustedPpb,

                (long long)
                residualTrustedMinusRecommendedPpb,

                (long)
                residualAcceptedTotal,

                (long)
                residualRejectedTotal,

                (long)
                residualRejectElapsed,

                (long)
                residualRejectRtt,

                (long)
                residualRejectMagnitude,

                (long long)
                residualRttMaxNs,

                (long)
                qpcDcPhaseResidualSequenceAfter);
        }
    }


    // =============================================================
    // S4 DC Phase Residual Drift Observer V1A Reader
    // Mean Phase + 16-point Theil-Sen Slope
    //
    // This is the candidate replacement for the noisy V1
    // first/last endpoint slope.
    // =============================================================

    static LONG
        lastQpcDcPhaseResidualV1APrintedSequence =
        0;


    LONG qpcDcPhaseResidualV1ASequenceBefore =
        g_qpcDcPhaseResidualV1ADiagSequence;


    if (qpcDcPhaseResidualV1ASequenceBefore != 0 &&
        (qpcDcPhaseResidualV1ASequenceBefore & 1) == 0 &&
        qpcDcPhaseResidualV1ASequenceBefore !=
        lastQpcDcPhaseResidualV1APrintedSequence)
    {
        MemoryBarrier();


        LONG v1aInitialized =
            g_qpcDcPhaseResidualV1AInitialized;

        LONG v1aBoundInit =
            g_qpcDcPhaseResidualV1ABoundInitCount;

        LONGLONG v1aMeanPhaseNs =
            g_qpcDcPhaseResidualV1AMeanPhaseNs;

        LONGLONG v1aPhaseSpanNs =
            g_qpcDcPhaseResidualV1APhaseSpanNs;

        LONGLONG v1aPointTimeNs =
            g_qpcDcPhaseResidualV1APointTimeNs;

        LONG v1aPointAccepted =
            g_qpcDcPhaseResidualV1APointAccepted;

        LONG v1aMeanSamples =
            g_qpcDcPhaseResidualV1AMeanSamples;

        LONG v1aPointCount =
            g_qpcDcPhaseResidualV1APointBufferCount;

        LONG v1aPairCount =
            g_qpcDcPhaseResidualV1APairSlopeCount;

        LONGLONG v1aTheilSenPpb =
            g_qpcDcPhaseResidualV1ATheilSenPpb;

        LONGLONG v1aSlopeMadPpb =
            g_qpcDcPhaseResidualV1ASlopeMadPpb;

        LONGLONG v1aTimeSpanNs =
            g_qpcDcPhaseResidualV1ATimeSpanNs;

        LONG v1aLocked =
            g_qpcDcPhaseResidualV1ALocked;

        LONGLONG v1aRecommendedPpb =
            g_qpcDcPhaseResidualV1ARecommendedSchedulerPpb;

        LONGLONG v1aTrustedPpb =
            g_qpcDcPhaseResidualV1ATrustedDriftPpb;

        LONGLONG v1aTrustedMinusRecommendedPpb =
            g_qpcDcPhaseResidualV1ATrustedMinusRecommendedPpb;

        LONGLONG v1aWindowElapsedNs =
            g_qpcDcPhaseResidualV1AWindowElapsedNs;

        LONGLONG v1aWindowRttMaxNs =
            g_qpcDcPhaseResidualV1AWindowRttMaxNs;

        LONG v1aAcceptedTotal =
            g_qpcDcPhaseResidualV1AAcceptedPointTotal;

        LONG v1aRejectedTotal =
            g_qpcDcPhaseResidualV1ARejectedPointTotal;

        LONG v1aRejectSamples =
            g_qpcDcPhaseResidualV1ARejectSamplesTotal;

        LONG v1aRejectElapsed =
            g_qpcDcPhaseResidualV1ARejectElapsedTotal;

        LONG v1aRejectRtt =
            g_qpcDcPhaseResidualV1ARejectRttTotal;

        LONG v1aRejectSpan =
            g_qpcDcPhaseResidualV1ARejectSpanTotal;

        LONGLONG v1aSlopeMinPpb =
            g_qpcDcPhaseResidualV1ASlopeMinPpb;

        LONGLONG v1aSlopeMaxPpb =
            g_qpcDcPhaseResidualV1ASlopeMaxPpb;


        MemoryBarrier();


        LONG qpcDcPhaseResidualV1ASequenceAfter =
            g_qpcDcPhaseResidualV1ADiagSequence;


        if (qpcDcPhaseResidualV1ASequenceBefore ==
            qpcDcPhaseResidualV1ASequenceAfter &&
            (qpcDcPhaseResidualV1ASequenceAfter & 1) == 0)
        {
            lastQpcDcPhaseResidualV1APrintedSequence =
                qpcDcPhaseResidualV1ASequenceAfter;


            RtPrintf(
                "[QPC-DC-PHASE-DRIFT-V1A-MAIN] "
                "Init:%s BoundInit:%ld | "
                "MeanPhase:%+lld ns "
                "PhaseSpan:%lld ns "
                "MeanSamples:%ld | "
                "PointTimeU:%llu "
                "WindowElapsed:%lld ns "
                "RTTMax:%lld ns | "
                "PointAccept:%s "
                "Points:%ld/16 "
                "Pairs:%ld/120 | "
                "TheilSenResidual:%+lld ppb "
                "SlopeMAD:%lld ppb | "
                "SlopeMin:%+lld SlopeMax:%+lld ppb | "
                "TimeSpan:%lld ns | "
                "Lock:%s | "
                "FixedScheduler:-8300 ppb | "
                "Recommended:%+lld ppb | "
                "Trusted:%+lld ppb "
                "TrustedMinusRecommended:%+lld ppb | "
                "AcceptedPoints:%ld RejectedPoints:%ld "
                "RejectSamples:%ld RejectElapsed:%ld "
                "RejectRTT:%ld RejectSpan:%ld | "
                "GateSamples:3900 "
                "GateRTT:250000 ns "
                "GatePhaseSpan:100000 ns "
                "LockPoints:8 "
                "LockTimeSpan:7000000000 ns "
                "LockSlopeMAD:750 ppb | "
                "MeanWindow:~1s TheilSenPoints:16 | "
                "Control:OFF | "
                "Snap:%ld\n",

                v1aInitialized != 0
                ? "YES"
                : "NO",

                (long)
                v1aBoundInit,

                (long long)
                v1aMeanPhaseNs,

                (long long)
                v1aPhaseSpanNs,

                (long)
                v1aMeanSamples,

                (unsigned long long)
                (uint64_t)
                v1aPointTimeNs,

                (long long)
                v1aWindowElapsedNs,

                (long long)
                v1aWindowRttMaxNs,

                v1aPointAccepted != 0
                ? "YES"
                : "NO",

                (long)
                v1aPointCount,

                (long)
                v1aPairCount,

                (long long)
                v1aTheilSenPpb,

                (long long)
                v1aSlopeMadPpb,

                (long long)
                v1aSlopeMinPpb,

                (long long)
                v1aSlopeMaxPpb,

                (long long)
                v1aTimeSpanNs,

                v1aLocked != 0
                ? "YES"
                : "NO",

                (long long)
                v1aRecommendedPpb,

                (long long)
                v1aTrustedPpb,

                (long long)
                v1aTrustedMinusRecommendedPpb,

                (long)
                v1aAcceptedTotal,

                (long)
                v1aRejectedTotal,

                (long)
                v1aRejectSamples,

                (long)
                v1aRejectElapsed,

                (long)
                v1aRejectRtt,

                (long)
                v1aRejectSpan,

                (long)
                qpcDcPhaseResidualV1ASequenceAfter);
        }
    }


    // =============================================================
    // Frequency FF Dry Run V2 Reader
    // =============================================================

    static LONG
        lastQpcPhaseFfV2PrintedSequence =
        0;


    LONG qpcPhaseFfV2SequenceBefore =
        g_qpcPhaseFfV2DiagSequence;


    if (qpcPhaseFfV2SequenceBefore != 0 &&
        (qpcPhaseFfV2SequenceBefore & 1) == 0 &&
        qpcPhaseFfV2SequenceBefore !=
        lastQpcPhaseFfV2PrintedSequence)
    {
        MemoryBarrier();


        LONG phaseFfInit =
            g_qpcPhaseFfV2Initialized;

        LONG phaseFfState =
            g_qpcPhaseFfV2State;

        LONG phaseFfCandidate =
            g_qpcPhaseFfV2CandidateGood;

        LONG phaseFfWarmup =
            g_qpcPhaseFfV2WarmupGoodCount;

        LONG phaseFfBad =
            g_qpcPhaseFfV2BadCount;

        LONG phaseFfRecovery =
            g_qpcPhaseFfV2RecoveryGoodCount;

        LONG phaseFfObserverSeq =
            g_qpcPhaseFfV2ObserverSequence;

        LONGLONG phaseFfRecommended =
            g_qpcPhaseFfV2RecommendedPpb;

        LONGLONG phaseFfDesired =
            g_qpcPhaseFfV2DesiredPpb;

        LONGLONG phaseFfApplied =
            g_qpcPhaseFfV2AppliedPpb;

        LONGLONG phaseFfSlew =
            g_qpcPhaseFfV2SlewAppliedPpb;

        LONGLONG phaseFfMad =
            g_qpcPhaseFfV2SlopeMadPpb;

        LONG phaseFfPoints =
            g_qpcPhaseFfV2Points;

        LONG phaseFfPairs =
            g_qpcPhaseFfV2Pairs;

        LONG phaseFfLock =
            g_qpcPhaseFfV2ObserverLocked;

        LONG phaseFfPointAccept =
            g_qpcPhaseFfV2PointAccepted;

        LONGLONG phaseFfTargetVsFixed =
            g_qpcPhaseFfV2TargetVsFixedNs;

        LONGLONG phaseFfWakeError =
            g_qpcPhaseFfV2WakeErrorNs;

        LONGLONG phaseFfDeltaStart =
            g_qpcPhaseFfV2TargetDeltaWindowStartNs;

        LONGLONG phaseFfDeltaEnd =
            g_qpcPhaseFfV2TargetDeltaWindowEndNs;

        LONGLONG phaseFfDeltaDelta =
            g_qpcPhaseFfV2TargetDeltaWindowDeltaNs;

        LONG phaseFfTrackCycles =
            g_qpcPhaseFfV2TrackCyclesWindow;

        LONG phaseFfHoldCycles =
            g_qpcPhaseFfV2HoldCyclesWindow;

        LONG phaseFfFallbackCycles =
            g_qpcPhaseFfV2FallbackCyclesWindow;

        LONG phaseFfSwitches =
            g_qpcPhaseFfV2StateSwitchesWindow;

        LONG phaseFfTotalSwitches =
            g_qpcPhaseFfV2TotalStateSwitches;

        LONG phaseFfW2T =
            g_qpcPhaseFfV2WarmupToTrackTotal;

        LONG phaseFfT2H =
            g_qpcPhaseFfV2TrackToHoldTotal;

        LONG phaseFfH2T =
            g_qpcPhaseFfV2HoldToTrackTotal;

        LONG phaseFfH2F =
            g_qpcPhaseFfV2HoldToFallbackTotal;

        LONG phaseFfF2T =
            g_qpcPhaseFfV2FallbackToTrackTotal;

        LONG phaseFfSamples =
            g_qpcPhaseFfV2Samples;


        MemoryBarrier();


        LONG qpcPhaseFfV2SequenceAfter =
            g_qpcPhaseFfV2DiagSequence;


        if (qpcPhaseFfV2SequenceBefore ==
            qpcPhaseFfV2SequenceAfter &&
            (qpcPhaseFfV2SequenceAfter & 1) == 0)
        {
            lastQpcPhaseFfV2PrintedSequence =
                qpcPhaseFfV2SequenceAfter;


            const char* phaseFfStateText =
                "WARMUP";


            if (phaseFfState == 1)
                phaseFfStateText = "TRACK";
            else if (phaseFfState == 2)
                phaseFfStateText = "HOLD";
            else if (phaseFfState == 3)
                phaseFfStateText = "FALLBACK";


            RtPrintf(
                "[QPC-PHASE-FF-V2-DRY-MAIN] "
                "Init:%s State:%s | "
                "Candidate:%s Lock:%s PointAccept:%s "
                "Points:%ld/16 Pairs:%ld/120 "
                "SlopeMAD:%lld ppb | "
                "Recommended:%+lld Desired:%+lld "
                "Applied:%+lld ppb LastAppliedStep:%+lld ppb | "
                "Warmup:%ld/3 Bad:%ld/5 Recovery:%ld | "
                "TargetVsFixed:%+lld ns "
                "WakeErr:%+lld ns | "
                "TargetDeltaWin Start:%+lld End:%+lld "
                "Delta:%+lld ns | "
                "Cycles Track:%ld Hold:%ld Fallback:%ld | "
                "Transitions W2T:%ld T2H:%ld H2T:%ld "
                "H2F:%ld F2T:%ld "
                "WindowSwitch:%ld TotalSwitch:%ld | "
                "ObserverSeq:%ld Samples:%ld | "
                "GateMAD:150 ppb GateDev:1000 ppb "
                "SlewMax:25 ppb/window | "
                "Incremental:YES Retroactive:NO | "
                "SchedulerUsed:-8300 ppb | "
                "Control:OFF | "
                "Snap:%ld\n",

                phaseFfInit != 0
                ? "YES"
                : "NO",

                phaseFfStateText,

                phaseFfCandidate != 0
                ? "GOOD"
                : "REJECT",

                phaseFfLock != 0
                ? "YES"
                : "NO",

                phaseFfPointAccept != 0
                ? "YES"
                : "NO",

                (long)
                phaseFfPoints,

                (long)
                phaseFfPairs,

                (long long)
                phaseFfMad,

                (long long)
                phaseFfRecommended,

                (long long)
                phaseFfDesired,

                (long long)
                phaseFfApplied,

                (long long)
                phaseFfSlew,

                (long)
                phaseFfWarmup,

                (long)
                phaseFfBad,

                (long)
                phaseFfRecovery,

                (long long)
                phaseFfTargetVsFixed,

                (long long)
                phaseFfWakeError,

                (long long)
                phaseFfDeltaStart,

                (long long)
                phaseFfDeltaEnd,

                (long long)
                phaseFfDeltaDelta,

                (long)
                phaseFfTrackCycles,

                (long)
                phaseFfHoldCycles,

                (long)
                phaseFfFallbackCycles,

                (long)
                phaseFfW2T,

                (long)
                phaseFfT2H,

                (long)
                phaseFfH2T,

                (long)
                phaseFfH2F,

                (long)
                phaseFfF2T,

                (long)
                phaseFfSwitches,

                (long)
                phaseFfTotalSwitches,

                (long)
                phaseFfObserverSeq,

                (long)
                phaseFfSamples,

                (long)
                qpcPhaseFfV2SequenceAfter);
        }
    }


    // =============================================================
    // Frequency FF Dry Run V2 - S4 DC Comparison Reader
    // =============================================================

    static LONG
        lastQpcPhaseFfV2DcPrintedSequence =
        0;


    LONG qpcPhaseFfV2DcSequenceBefore =
        g_qpcPhaseFfV2DcDiagSequence;


    if (qpcPhaseFfV2DcSequenceBefore != 0 &&
        (qpcPhaseFfV2DcSequenceBefore & 1) == 0 &&
        qpcPhaseFfV2DcSequenceBefore !=
        lastQpcPhaseFfV2DcPrintedSequence)
    {
        MemoryBarrier();


        LONG phaseFfDcInit =
            g_qpcPhaseFfV2DcInitialized;

        LONGLONG phaseFfDcPhase =
            g_qpcPhaseFfV2DcPhaseNs;

        LONGLONG phaseFfDcMargin =
            g_qpcPhaseFfV2DcSync0MarginNs;

        LONGLONG phaseFfDcError =
            g_qpcPhaseFfV2DcUnwrappedErrorNs;

        LONGLONG phaseFfDcVsFixed =
            g_qpcPhaseFfV2DcVsFixedNs;

        LONGLONG phaseFfDcVsTrusted =
            g_qpcPhaseFfV2DcVsTrustedNs;

        LONGLONG phaseFfDcStart =
            g_qpcPhaseFfV2DcWindowStartNs;

        LONGLONG phaseFfDcEnd =
            g_qpcPhaseFfV2DcWindowEndNs;

        LONGLONG phaseFfDcDelta =
            g_qpcPhaseFfV2DcWindowDeltaNs;

        LONGLONG phaseFfDcMin =
            g_qpcPhaseFfV2DcWindowMinNs;

        LONGLONG phaseFfDcMax =
            g_qpcPhaseFfV2DcWindowMaxNs;

        LONGLONG phaseFfDcAbsAvg =
            g_qpcPhaseFfV2DcAbsAvgNs;

        LONG phaseFfDcBetterFixed =
            g_qpcPhaseFfV2DcBetterThanFixed;

        LONG phaseFfDcBetterTrusted =
            g_qpcPhaseFfV2DcBetterThanTrusted;

        LONG phaseFfDcSamples =
            g_qpcPhaseFfV2DcSamples;

        MemoryBarrier();


        LONG qpcPhaseFfV2DcSequenceAfter =
            g_qpcPhaseFfV2DcDiagSequence;


        if (qpcPhaseFfV2DcSequenceBefore ==
            qpcPhaseFfV2DcSequenceAfter &&
            (qpcPhaseFfV2DcSequenceAfter & 1) == 0)
        {
            lastQpcPhaseFfV2DcPrintedSequence =
                qpcPhaseFfV2DcSequenceAfter;


            RtPrintf(
                "[QPC-PHASE-FF-V2-DC-MAIN] "
                "Init:%s | "
                "Phase:%lld ns Sync0Margin:%lld ns | "
                "UnwrappedErr:%+lld ns | "
                "VsFixed:%+lld ns VsTrusted:%+lld ns | "
                "Win Start:%+lld End:%+lld Delta:%+lld "
                "Min:%+lld Max:%+lld ns | "
                "AbsAvg:%lld ns | "
                "BetterThanFixed:%ld/4000 "
                "BetterThanTrusted:%ld/4000 | "
                "Samples:%ld | "
                "TargetDomain:YES SendPoint:NO | "
                "SchedulerUsed:-8300 ppb | "
                "Control:OFF | "
                "Snap:%ld\n",

                phaseFfDcInit != 0
                ? "YES"
                : "NO",

                (long long)
                phaseFfDcPhase,

                (long long)
                phaseFfDcMargin,

                (long long)
                phaseFfDcError,

                (long long)
                phaseFfDcVsFixed,

                (long long)
                phaseFfDcVsTrusted,

                (long long)
                phaseFfDcStart,

                (long long)
                phaseFfDcEnd,

                (long long)
                phaseFfDcDelta,

                (long long)
                phaseFfDcMin,

                (long long)
                phaseFfDcMax,

                (long long)
                phaseFfDcAbsAvg,

                (long)
                phaseFfDcBetterFixed,

                (long)
                phaseFfDcBetterTrusted,

                (long)
                phaseFfDcSamples,

                (long)
                qpcPhaseFfV2DcSequenceAfter);
        }
    }


    static LONG lastV1aCostSeq = 0;
    LONG v1aSeq1 = g_qpcV1aCostSeq;
    if (v1aSeq1 != 0 && !(v1aSeq1 & 1) && v1aSeq1 != lastV1aCostSeq)
    {
        MemoryBarrier();
        LONGLONG lastNs = g_qpcV1aCostLastNs;
        LONGLONG avgNs = g_qpcV1aCostAvgNs;
        LONGLONG maxNs = g_qpcV1aCostMaxNs;
        LONG events = g_qpcV1aCostEvents;
        LONG over20 = g_qpcV1aCostOver20us;
        LONG over40 = g_qpcV1aCostOver40us;
        LONG over80 = g_qpcV1aCostOver80us;
        LONG qpcFail = g_qpcV1aCostQpcFail;
        MemoryBarrier();
        LONG v1aSeq2 = g_qpcV1aCostSeq;

        if (v1aSeq1 == v1aSeq2 && !(v1aSeq2 & 1))
        {
            lastV1aCostSeq = v1aSeq2;
            RtPrintf(
                "[QPC-V1A-OBSERVER-COST-MAIN] "
                "Last:%lld Avg:%lld Max:%lld ns | "
                "Events:%ld Over20us:%ld Over40us:%ld Over80us:%ld "
                "QpcFail:%ld | Control:OFF | Snap:%ld\n",
                (long long)lastNs, (long long)avgNs,
                (long long)maxNs, (long)events,
                (long)over20, (long)over40, (long)over80,
                (long)qpcFail, (long)v1aSeq2);
        }
    }


    // =============================================================
    // QPC Coarse Re-Anchor Dry Run V1 Reader
    //
    // VIRTUAL ONLY. No real timer/scheduler control.
    // =============================================================

    static LONG
        lastQpcCoarseReanchorPrintedSequence =
        0;


    LONG qpcCoarseSequenceBefore =
        g_qpcCoarseReanchorDiagSequence;


    if (qpcCoarseSequenceBefore != 0 &&
        (qpcCoarseSequenceBefore & 1) == 0 &&
        qpcCoarseSequenceBefore !=
        lastQpcCoarseReanchorPrintedSequence)
    {
        MemoryBarrier();


        LONG coarseInitialized =
            g_qpcCoarseReanchorInitialized;

        LONGLONG coarseRawErrorNs =
            g_qpcCoarseReanchorRawErrorNs;

        LONGLONG coarseVirtualErrorNs =
            g_qpcCoarseReanchorVirtualErrorNs;

        LONGLONG coarseVirtualMarginNs =
            g_qpcCoarseReanchorVirtualMarginNs;

        LONGLONG coarseVirtualOffsetNs =
            g_qpcCoarseReanchorVirtualOffsetNs;

        LONGLONG coarseWinStartNs =
            g_qpcCoarseReanchorWindowStartErrorNs;

        LONGLONG coarseWinEndNs =
            g_qpcCoarseReanchorWindowEndErrorNs;

        LONGLONG coarseWinDeltaNs =
            g_qpcCoarseReanchorWindowDeltaErrorNs;

        LONGLONG coarseWinMinNs =
            g_qpcCoarseReanchorWindowMinErrorNs;

        LONGLONG coarseWinMaxNs =
            g_qpcCoarseReanchorWindowMaxErrorNs;

        LONG coarseEvents =
            g_qpcCoarseReanchorCorrectionEventsWindow;

        LONG coarseTicks =
            g_qpcCoarseReanchorCorrectionTicksWindow;

        LONG coarseMaxTicksPerEvent =
            g_qpcCoarseReanchorMaxTicksPerEventWindow;

        LONG coarseMultiTickEvents =
            g_qpcCoarseReanchorMultiTickEventsWindow;

        LONGLONG coarseTotalEvents =
            g_qpcCoarseReanchorTotalCorrectionEvents;

        LONGLONG coarseTotalTicks =
            g_qpcCoarseReanchorTotalCorrectionTicks;

        LONG coarseLastSpacing =
            g_qpcCoarseReanchorLastEventSpacingCycles;

        LONG coarseMinSpacing =
            g_qpcCoarseReanchorMinEventSpacingCycles;

        LONG coarseMaxSpacing =
            g_qpcCoarseReanchorMaxEventSpacingCycles;

        LONGLONG coarseRobustMedianPpb =
            g_qpcCoarseReanchorRobustMedianDriftPpb;

        LONGLONG coarseRobustMadPpb =
            g_qpcCoarseReanchorRobustMadPpb;

        LONG coarseRobustLocked =
            g_qpcCoarseReanchorRobustLocked;

        LONG coarseSamples =
            g_qpcCoarseReanchorWindowSamples;

        LONG coarseOverflow =
            g_qpcCoarseReanchorOverflow;


        MemoryBarrier();


        LONG qpcCoarseSequenceAfter =
            g_qpcCoarseReanchorDiagSequence;


        if (qpcCoarseSequenceBefore ==
            qpcCoarseSequenceAfter &&
            (qpcCoarseSequenceAfter & 1) == 0)
        {
            lastQpcCoarseReanchorPrintedSequence =
                qpcCoarseSequenceAfter;


            RtPrintf(
                "[QPC-COARSE-REANCHOR-DRY-MAIN] "
                "Init:%s | "
                "RawErr:%+lld ns | "
                "VirtualErr:%+lld ns | "
                "VirtualMargin:%lld ns | "
                "VirtualOffset:%+lld ns | "
                "WinStart:%+lld WinEnd:%+lld "
                "WinDelta:%+lld ns | "
                "WinMin:%+lld WinMax:%+lld ns | "
                "Events:%ld Ticks:%ld "
                "MaxTicks/Event:%ld MultiTickEvents:%ld | "
                "TotalEvents:%lld TotalTicks:%lld | "
                "Spacing Last:%ld Min:%ld Max:%ld cycles | "
                "Robust:%+lld ppb MAD:%lld Lock:%s | "
                "Overflow:%s | "
                "TargetMargin:100000 "
                "TriggerMargin:50000 Tick:50000 ns | "
                "Samples:%ld | "
                "Control:OFF | "
                "Snap:%ld\n",

                coarseInitialized != 0 ? "YES" : "NO",
                (long long)coarseRawErrorNs,
                (long long)coarseVirtualErrorNs,
                (long long)coarseVirtualMarginNs,
                (long long)coarseVirtualOffsetNs,
                (long long)coarseWinStartNs,
                (long long)coarseWinEndNs,
                (long long)coarseWinDeltaNs,
                (long long)coarseWinMinNs,
                (long long)coarseWinMaxNs,
                (long)coarseEvents,
                (long)coarseTicks,
                (long)coarseMaxTicksPerEvent,
                (long)coarseMultiTickEvents,
                (long long)coarseTotalEvents,
                (long long)coarseTotalTicks,
                (long)coarseLastSpacing,
                (long)coarseMinSpacing,
                (long)coarseMaxSpacing,
                (long long)coarseRobustMedianPpb,
                (long long)coarseRobustMadPpb,
                coarseRobustLocked != 0 ? "YES" : "NO",
                coarseOverflow != 0 ? "YES" : "NO",
                (long)coarseSamples,
                (long)qpcCoarseSequenceAfter);
        }
    }


    // =============================================================
// QPC Scheduler Dry Run V1 Snapshot Reader
//
// Priority 64:
//     Target generator + diagnostic publish
//
// Priority 50:
//     Print only
// =============================================================

    static LONG
        lastQpcSchedulerPrintedSequence =
        0;


    LONG qpcSchedulerSequenceBefore =
        g_qpcSchedulerDiagSequence;


    if (qpcSchedulerSequenceBefore != 0 &&
        (qpcSchedulerSequenceBefore & 1) == 0 &&
        qpcSchedulerSequenceBefore !=
        lastQpcSchedulerPrintedSequence)
    {
        MemoryBarrier();


        // =========================================================
        // Copy Snapshot
        // =========================================================

        LONGLONG targetQpc =
            g_qpcSchedulerTargetQpc;

        LONGLONG actualQpc =
            g_qpcSchedulerActualQpc;


        LONGLONG errorCounts =
            g_qpcSchedulerErrorCounts;

        LONGLONG errorNs =
            g_qpcSchedulerErrorNs;


        LONGLONG windowStartErrorNs =
            g_qpcSchedulerWindowStartErrorNs;

        LONGLONG windowEndErrorNs =
            g_qpcSchedulerWindowEndErrorNs;

        LONGLONG windowDeltaErrorNs =
            g_qpcSchedulerWindowDeltaErrorNs;

        LONGLONG windowMinErrorNs =
            g_qpcSchedulerWindowMinErrorNs;

        LONGLONG windowMaxErrorNs =
            g_qpcSchedulerWindowMaxErrorNs;


        LONGLONG periodWholeCounts =
            g_qpcSchedulerPeriodWholeCounts;

        LONGLONG periodFractionScaled =
            g_qpcSchedulerPeriodFractionScaled;

        LONGLONG assumedDriftPpb =
            g_qpcSchedulerAssumedDriftPpb;


        LONG windowSamples =
            g_qpcSchedulerWindowSamples;

        LONG schedulerValid =
            g_qpcSchedulerValid;

        LONG schedulerState =
            g_qpcSchedulerState;

        LONGLONG schedulerGuardNs =
            g_qpcSchedulerGuardNs;

        LONGLONG remainingToTargetNs =
            g_qpcSchedulerRemainingToTargetNs;

        LONGLONG lateByNs =
            g_qpcSchedulerLateByNs;

        LONGLONG anchorQpc =
            g_qpcSchedulerAnchorQpc;

        LONG anchorDcDiagSequence =
            g_qpcSchedulerAnchorDcDiagSequence;

        LONG stableWaitCycles =
            g_qpcSchedulerStableWaitCycles;

        LONG earlyCount =
            g_qpcSchedulerEarlyCount;

        LONG nearCount =
            g_qpcSchedulerNearCount;

        LONG lateCount =
            g_qpcSchedulerLateCount;
        MemoryBarrier();


        LONG qpcSchedulerSequenceAfter =
            g_qpcSchedulerDiagSequence;


        // =========================================================
        // Seqlock validation
        // =========================================================

        if (qpcSchedulerSequenceBefore ==
            qpcSchedulerSequenceAfter &&
            (qpcSchedulerSequenceAfter & 1) == 0)
        {
            lastQpcSchedulerPrintedSequence =
                qpcSchedulerSequenceAfter;

            const char* schedulerStateText =
                "UNKNOWN";


            switch (schedulerState)
            {
            case 0:
                schedulerStateText =
                    "WAIT_STABLE";
                break;

            case 1:
                schedulerStateText =
                    "EARLY";
                break;

            case 2:
                schedulerStateText =
                    "NEAR";
                break;

            case 3:
                schedulerStateText =
                    "LATE";
                break;

            default:
                schedulerStateText =
                    "UNKNOWN";
                break;
            }

            RtPrintf(
                "[QPC-SCHED-MAIN] "
                "State:%s | "
                "Target:%lld | "
                "Actual:%lld | "
                "Err:%+lld ns | "
                "Remain:%lld ns | "
                "LateBy:%lld ns | "
                "Guard:%lld ns | "
                "WinStart:%+lld | "
                "WinEnd:%+lld | "
                "WinDelta:%+lld ns | "
                "WinMin:%+lld | "
                "WinMax:%+lld ns | "
                "Early:%ld Near:%ld Late:%ld | "
                "Anchor:%lld | "
                "AnchorDCSeq:%ld | "
                "StableWait:%ld | "
                "Period:%lld + %lld/1e9 | "
                "Drift:%+lld ppb | "
                "Samples:%ld | "
                "Valid:%s | "
                "Snap:%ld\n",

                schedulerStateText,

                (long long)
                targetQpc,

                (long long)
                actualQpc,

                (long long)
                errorNs,

                (long long)
                remainingToTargetNs,

                (long long)
                lateByNs,

                (long long)
                schedulerGuardNs,

                (long long)
                windowStartErrorNs,

                (long long)
                windowEndErrorNs,

                (long long)
                windowDeltaErrorNs,

                (long long)
                windowMinErrorNs,

                (long long)
                windowMaxErrorNs,

                (long)
                earlyCount,

                (long)
                nearCount,

                (long)
                lateCount,

                (long long)
                anchorQpc,

                (long)
                anchorDcDiagSequence,

                (long)
                stableWaitCycles,

                (long long)
                periodWholeCounts,

                (long long)
                periodFractionScaled,

                (long long)
                assumedDriftPpb,

                (long)
                windowSamples,

                schedulerValid != 0
                ? "YES"
                : "NO",

                (long)
                qpcSchedulerSequenceAfter);
        }
    }
    // =============================================================
// QPC <-> EtherCAT S4 DC Estimator Snapshot Reader
//
// Priority 64:
//     Measure + publish only
//
// Priority 50:
//     Print here
// =============================================================

    static LONG
        lastQpcDcDiagPrintedSequence =
        0;


    LONG qpcDcSequenceBefore =
        g_qpcDcDiagSequence;


    if (qpcDcSequenceBefore != 0 &&
        (qpcDcSequenceBefore & 1) == 0 &&
        qpcDcSequenceBefore !=
        lastQpcDcDiagPrintedSequence)
    {
        MemoryBarrier();


        // =========================================================
        // Copy Snapshot
        // =========================================================

        LONGLONG qpcElapsedNs =
            g_qpcDcQpcElapsedNs;

        LONGLONG dcElapsedNs =
            g_qpcDcDcElapsedNs;

        LONGLONG deltaNs =
            g_qpcDcDeltaNs;

        LONGLONG driftPpb =
            g_qpcDcDriftPpb;


        LONGLONG rttAvgNs =
            g_qpcDcRttAvgNs;

        LONGLONG rttMinNs =
            g_qpcDcRttMinNs;

        LONGLONG rttMaxNs =
            g_qpcDcRttMaxNs;


        LONG validSamples =
            g_qpcDcValidSamples;

        LONG rejectedSamples =
            g_qpcDcRejectedSamples;

        LONG valid =
            g_qpcDcValid;


        MemoryBarrier();


        LONG qpcDcSequenceAfter =
            g_qpcDcDiagSequence;


        // =========================================================
        // Seqlock Validation
        // =========================================================

        if (qpcDcSequenceBefore ==
            qpcDcSequenceAfter &&
            (qpcDcSequenceAfter & 1) == 0)
        {
            lastQpcDcDiagPrintedSequence =
                qpcDcSequenceAfter;


            RtPrintf(
                "[QPC-DC-MAIN] "
                "QPCElapsed:%lld ns | "
                "DCElapsed:%lld ns | "
                "Delta:%+lld ns | "
                "Drift:%+lld ppb | "
                "RTT Avg:%lld Min:%lld Max:%lld ns | "
                "Valid:%ld Reject:%ld | "
                "State:%s | "
                "Snap:%ld\n",

                (long long)
                qpcElapsedNs,

                (long long)
                dcElapsedNs,

                (long long)
                deltaNs,

                (long long)
                driftPpb,

                (long long)
                rttAvgNs,

                (long long)
                rttMinNs,

                (long long)
                rttMaxNs,

                (long)
                validSamples,

                (long)
                rejectedSamples,

                valid != 0
                ? "VALID"
                : "INVALID",

                (long)
                qpcDcSequenceAfter);
        }
    }


    // =============================================================
    // QPC <-> S4 Robust Drift Dry-Run V1 Reader
    //
    // Priority 64:
    //     one update per completed raw ~1 second window.
    //
    // Priority 50:
    //     print here.
    //
    // DRY-RUN ONLY:
    //     scheduler still uses fixed -8300 ppb.
    // =============================================================

    static LONG
        lastQpcDcRobustPrintedSequence =
        0;


    LONG qpcDcRobustSequenceBefore =
        g_qpcDcRobustDiagSequence;


    if (qpcDcRobustSequenceBefore != 0 &&
        (qpcDcRobustSequenceBefore & 1) == 0 &&
        qpcDcRobustSequenceBefore !=
        lastQpcDcRobustPrintedSequence)
    {
        MemoryBarrier();


        LONGLONG robustRawDriftPpb =
            g_qpcDcRobustRawDriftPpb;

        LONGLONG robustMedianDriftPpb =
            g_qpcDcRobustMedianDriftPpb;

        LONGLONG robustMadPpb =
            g_qpcDcRobustMadPpb;

        LONGLONG robustPeriodFfPs =
            g_qpcDcRobustPeriodFfPs;


        LONG robustBufferCount =
            g_qpcDcRobustBufferCount;

        LONG robustCurrentAccepted =
            g_qpcDcRobustCurrentAccepted;

        LONG robustLocked =
            g_qpcDcRobustLocked;

        LONG robustAcceptedTotal =
            g_qpcDcRobustAcceptedTotal;

        LONG robustRejectedTotal =
            g_qpcDcRobustRejectedTotal;


        MemoryBarrier();


        LONG qpcDcRobustSequenceAfter =
            g_qpcDcRobustDiagSequence;


        if (qpcDcRobustSequenceBefore ==
            qpcDcRobustSequenceAfter &&
            (qpcDcRobustSequenceAfter & 1) == 0)
        {
            lastQpcDcRobustPrintedSequence =
                qpcDcRobustSequenceAfter;


            RtPrintf(
                "[QPC-DRIFT-ROBUST-MAIN] "
                "Raw:%+lld ppb | "
                "Accept:%s | "
                "Median:%+lld ppb | "
                "MAD:%lld ppb | "
                "Buffer:%ld/9 | "
                "Lock:%s | "
                "FF:%+lld ps/cycle | "
                "Accepted:%ld Rejected:%ld | "
                "SchedulerUsed:-8300 ppb | "
                "Control:OFF | "
                "Snap:%ld\n",

                (long long)
                robustRawDriftPpb,

                robustCurrentAccepted != 0
                ? "YES"
                : "NO",

                (long long)
                robustMedianDriftPpb,

                (long long)
                robustMadPpb,

                (long)
                robustBufferCount,

                robustLocked != 0
                ? "YES"
                : "NO",

                (long long)
                robustPeriodFfPs,

                (long)
                robustAcceptedTotal,

                (long)
                robustRejectedTotal,

                (long)
                qpcDcRobustSequenceAfter);
        }
    }


    // =============================================================
    // EtherCAT RX Hard Deadline V1C Diagnostic Reader
    //
    // Priority 64: counters + seqlock snapshot only.
    // Priority 50: print here.
    // No control action.
    // =============================================================

    static LONG
        lastEcatRxDiagPrintedSequence =
        0;


    LONG rxDiagSequenceBefore =
        g_ecatRxDiagSequence;


    if (rxDiagSequenceBefore != 0 &&
        (rxDiagSequenceBefore & 1) == 0 &&
        rxDiagSequenceBefore !=
        lastEcatRxDiagPrintedSequence)
    {
        MemoryBarrier();


        LONG rxDiagCalls =
            g_ecatRxDiagCalls;

        LONG rxDiagFirstRxSuccess =
            g_ecatRxDiagFirstRxSuccess;

        LONG rxDiagEmptyRx =
            g_ecatRxDiagEmptyRx;

        LONG rxDiagInvalidFrame =
            g_ecatRxDiagInvalidFrame;

        LONG rxDiagSoftLateAccepted =
            g_ecatRxDiagSoftLateAccepted;

        LONGLONG rxDiagTotalSoftLateAccepted =
            g_ecatRxDiagTotalSoftLateAccepted;

        LONGLONG rxDiagSoftLateElapsedMaxNs =
            g_ecatRxDiagSoftLateElapsedMaxNs;

        LONG rxDiagHardTimeout =
            g_ecatRxDiagHardTimeout;

        LONG rxDiagPostReceiveLate =
            g_ecatRxDiagPostReceiveLate;

        LONG rxDiagCurrentConsecutiveTimeout =
            g_ecatRxDiagCurrentConsecutiveTimeout;

        LONG rxDiagMaxConsecutiveTimeout =
            g_ecatRxDiagMaxConsecutiveTimeout;

        LONGLONG rxDiagTotalHardTimeout =
            g_ecatRxDiagTotalHardTimeout;

        LONG rxDiagRecoveryAfterTimeout =
            g_ecatRxDiagRecoveryAfterTimeout;

        LONG rxDiagQpcFail =
            g_ecatRxDiagQpcFail;

        LONG rxDiagSleepCount =
            g_ecatRxDiagSleepCount;

        LONG rxDiagElapsedValid =
            g_ecatRxDiagElapsedValid;

        LONGLONG rxDiagElapsedAvgNs =
            g_ecatRxDiagElapsedAvgNs;

        LONGLONG rxDiagElapsedMaxNs =
            g_ecatRxDiagElapsedMaxNs;

        LONG rxDiagTimeoutPreReceive =
            g_ecatRxDiagTimeoutPreReceive;

        LONG rxDiagTimeoutSleep0 =
            g_ecatRxDiagTimeoutSleep0;

        LONG rxDiagTimeoutSleep1 =
            g_ecatRxDiagTimeoutSleep1;

        LONG rxDiagTimeoutSleep2 =
            g_ecatRxDiagTimeoutSleep2;

        LONG rxDiagTimeoutAttemptAvg =
            g_ecatRxDiagTimeoutAttemptAvg;

        LONG rxDiagTimeoutAttemptMax =
            g_ecatRxDiagTimeoutAttemptMax;

        LONGLONG rxDiagReceiveCallMaxNs =
            g_ecatRxDiagReceiveCallMaxNs;

        LONGLONG rxDiagTimeoutReceiveCallMaxNs =
            g_ecatRxDiagTimeoutReceiveCallMaxNs;

        LONG rxDiagSoftDeadlineNs =
            g_ecatRxDiagSoftDeadlineNs;

        LONG rxDiagHardDeadlineNs =
            g_ecatRxDiagHardDeadlineNs;


        MemoryBarrier();


        LONG rxDiagSequenceAfter =
            g_ecatRxDiagSequence;


        if (rxDiagSequenceBefore ==
            rxDiagSequenceAfter &&
            (rxDiagSequenceAfter & 1) == 0)
        {
            lastEcatRxDiagPrintedSequence =
                rxDiagSequenceAfter;


            // RX 主訊息判讀：
            // - Calls/FirstRx：本窗呼叫數與第一次就收到的數量。
            // - EmptyRx/Invalid：輪詢空回覆與錯誤 frame；只有後者代表不可用資料。
            // - SoftLateAccepted：205..210 us 之間仍採用的 frame，是 deadline 預警。
            // - HardTimeout/TotalHardTimeout：本窗／全程真正逾越 210 us 的事件。
            // - CurrentConsecutive：下一週期成功會回 0；持續非 0 才是連續失聯。
            // - PostReceiveLate：ReceivePacket 返回後才超時，通常指向 driver/NAL 延遲。
            RtPrintf(
                "[ECAT-RX-DEADLINE-MAIN] "
                "Calls:%ld | "
                "FirstRx:%ld | "
                "EmptyRx:%ld | "
                "Invalid:%ld | "
                "SoftLateAccepted:%ld | "
                "TotalSoftLate:%lld | "
                "SoftLateElapsedMax:%lld ns | "
                "HardTimeout:%ld | "
                "PostReceiveLate:%ld | "
                "CurrentConsecutive:%ld | "
                "MaxConsecutive:%ld | "
                "TotalHardTimeout:%lld | "
                "RecoveryAfterTimeout:%ld | "
                "QpcFail:%ld | "
                "Sleep:%ld | "
                "RxElapsed Avg:%lld Max:%lld ns | "
                "ElapsedValid:%ld | "
                "Soft:%ld Hard:%ld ns | "
                "Snap:%ld\n",

                (long)rxDiagCalls,
                (long)rxDiagFirstRxSuccess,
                (long)rxDiagEmptyRx,
                (long)rxDiagInvalidFrame,
                (long)rxDiagSoftLateAccepted,
                (long long)rxDiagTotalSoftLateAccepted,
                (long long)rxDiagSoftLateElapsedMaxNs,
                (long)rxDiagHardTimeout,
                (long)rxDiagPostReceiveLate,
                (long)rxDiagCurrentConsecutiveTimeout,
                (long)rxDiagMaxConsecutiveTimeout,
                (long long)rxDiagTotalHardTimeout,
                (long)rxDiagRecoveryAfterTimeout,
                (long)rxDiagQpcFail,
                (long)rxDiagSleepCount,
                (long long)rxDiagElapsedAvgNs,
                (long long)rxDiagElapsedMaxNs,
                (long)rxDiagElapsedValid,
                (long)rxDiagSoftDeadlineNs,
                (long)rxDiagHardDeadlineNs,
                (long)rxDiagSequenceAfter);


            // Stage 訊息只定位 timeout 發生位置，不重複計算事件次數。
            // PreDeadline 高：TX 或 Handler 前段已耗掉 RX 預算。
            // SleepAtTimeout=2 高：兩次 coarse wait 後仍未收包。
            // ReceiveCallMax 高：ReceivePacket 本身耗時突增，優先檢查 NIC interrupt。
            RtPrintf(
                "[ECAT-RX-STAGE-MAIN] "
                "PreDeadline:%ld | "
                "PostReceive:%ld | "
                "SleepAtTimeout 0:%ld 1:%ld 2:%ld | "
                "Attempts Avg:%ld Max:%ld | "
                "ReceiveCallMax:%lld ns | "
                "TimeoutCallMax:%lld ns | "
                "Snap:%ld\n",

                (long)rxDiagTimeoutPreReceive,
                (long)rxDiagPostReceiveLate,
                (long)rxDiagTimeoutSleep0,
                (long)rxDiagTimeoutSleep1,
                (long)rxDiagTimeoutSleep2,
                (long)rxDiagTimeoutAttemptAvg,
                (long)rxDiagTimeoutAttemptMax,
                (long long)rxDiagReceiveCallMaxNs,
                (long long)rxDiagTimeoutReceiveCallMaxNs,
                (long)rxDiagSequenceAfter);
        }
    }


    // =============================================================
// EtherCAT Send Point Diagnostic Snapshot Reader
//
// Priority 64:
//     ecx_LRW_FRMW() measurement + publish
//
// Priority 50:
//     Print here
// =============================================================

    static LONG
        lastEcatSendDiagPrintedSequence =
        0;


    LONG sendSequenceBefore =
        g_ecatSendDiagSequence;


    if (sendSequenceBefore != 0 &&
        (sendSequenceBefore & 1) == 0 &&
        sendSequenceBefore !=
        lastEcatSendDiagPrintedSequence)
    {
        MemoryBarrier();


        // =========================================================
        // Copy volatile snapshot
        // =========================================================

        LONGLONG buildAvgNs =
            g_ecatSendBuildAvgNs;

        LONGLONG buildMinNs =
            g_ecatSendBuildMinNs;

        LONGLONG buildMaxNs =
            g_ecatSendBuildMaxNs;


        LONGLONG sendCallAvgNs =
            g_ecatSendCallAvgNs;

        LONGLONG sendCallMinNs =
            g_ecatSendCallMinNs;

        LONGLONG sendCallMaxNs =
            g_ecatSendCallMaxNs;


        LONG sendQpcValid =
            g_ecatSendQpcValid;


        MemoryBarrier();


        LONG sendSequenceAfter =
            g_ecatSendDiagSequence;


        // =========================================================
        // Seqlock validation
        // =========================================================

        if (sendSequenceBefore ==
            sendSequenceAfter &&
            (sendSequenceAfter & 1) == 0)
        {
            lastEcatSendDiagPrintedSequence =
                sendSequenceAfter;


            RtPrintf(
                "[ECAT-SEND-MAIN] "
                "Build Avg:%lld Min:%lld Max:%lld ns | "
                "SendCall Avg:%lld Min:%lld Max:%lld ns | "
                "Valid:%ld | "
                "Snap:%ld\n",

                (long long)
                buildAvgNs,

                (long long)
                buildMinNs,

                (long long)
                buildMaxNs,

                (long long)
                sendCallAvgNs,

                (long long)
                sendCallMinNs,

                (long long)
                sendCallMaxNs,

                (long)
                sendQpcValid,

                (long)
                sendSequenceAfter);

            // TX Root Cause 欄位：
            // Calls/Success     ：累積呼叫與成功提交數。
            // TxFrameBusy       ：送出前 ownership 尚未歸還。
            // TxNotOwner        ：提交 API 回報 ERROR_NOT_OWNER。
            // TxSubmitFail      ：任何提交失敗或數量不符。
            // TxSubmitted0      ：NAL 接受數量為 0。
            // LastError         ：最後一次 TX 異常代碼。
            RtPrintf(
                "[ECAT-TX-ROOT-RC1-MAIN] "
                "Calls:%lld | "
                "Success:%lld | "
                "TxFrameBusy:%lld | "
                "TxNotOwner:%lld | "
                "TxSubmitFail:%lld | "
                "TxSubmitted0:%lld | "
                "LastError:0x%08lX\n",

                (long long)g_nicTxCalls,
                (long long)g_nicTxSuccess,
                (long long)g_nicTxFrameBusy,
                (long long)g_nicTxNotOwner,
                (long long)g_nicTxSubmitFail,
                (long long)g_nicTxSubmitted0,
                (unsigned long)g_nicTxLastError);
        }
    }
    // =========================================================
// RT Diagnostic Heartbeat
//
// Priority 50 Main Thread only.
//
// 目的：
// 1. 確認 Main Thread 的 1000ms task 有在跑
// 2. 確認 PDO Handler 有在跑
// 3. 確認 Snapshot Sequence 有沒有更新
// =========================================================

    RtPrintf(
        "[RT-DIAG-HB] "
        "PDO-Tick:%llu | "
        "Sequence:%ld | "
        "LRW:%d | "
        "DC:%d\n",

        (unsigned long long)
        tickCount_PDO,

        (long)
        g_pdoRtDiagSequence,

        wkc_PDO,

        wk_read);

    // =========================================================
// PDO RT Diagnostic Snapshot Reader
//
// Priority 64 PDO Handler:
//     只寫資料
//
// Priority 50 Main Thread:
//     在這裡才 RtPrintf
//
// Sequence：
//
// odd  = PDO 正在更新
// even = snapshot 完整
//
// =========================================================

    static LONG lastPrintedSequence =
        0;


    LONG sequenceBefore =
        g_pdoRtDiagSequence;


    // ---------------------------------------------------------
    // Sequence 必須：
    //
    // 1. 非 0
    // 2. even
    // 3. 跟上一次不同
    //
    // ---------------------------------------------------------

    if (sequenceBefore != 0 &&
        (sequenceBefore & 1) == 0 &&
        sequenceBefore !=
        lastPrintedSequence)
    {
        // -----------------------------------------------------
        // Copy snapshot to local variables
        //
        // Print 時不直接反覆讀 volatile。
        // -----------------------------------------------------

        LONGLONG timerAvgNs =
            g_pdoRtTimerAvgNs;

        LONGLONG timerMinNs =
            g_pdoRtTimerMinNs;

        LONGLONG timerMaxNs =
            g_pdoRtTimerMaxNs;


        LONG timerShort =
            g_pdoRtTimerShortCount;

        LONG timerNormal =
            g_pdoRtTimerNormalCount;

        LONG timerLong =
            g_pdoRtTimerLongCount;


        LONGLONG combinedAvgNs =
            g_pdoRtCombinedAvgNs;

        LONGLONG combinedMinNs =
            g_pdoRtCombinedMinNs;

        LONGLONG combinedMaxNs =
            g_pdoRtCombinedMaxNs;


        LONGLONG execAvgNs =
            g_pdoRtExecAvgNs;

        LONGLONG execMinNs =
            g_pdoRtExecMinNs;

        LONGLONG execMaxNs =
            g_pdoRtExecMaxNs;


        LONG execOver250 =
            g_pdoRtExecOver250Count;

        LONG execOver300 =
            g_pdoRtExecOver300Count;

        LONG execOver400 =
            g_pdoRtExecOver400Count;


        LONG lrwWkc =
            g_pdoRtLrwWkc;

        LONG dcWkc =
            g_pdoRtDcWkc;


        // -----------------------------------------------------
        // 確保上面所有讀取完成後，
        // 再檢查 Writer 有沒有在中途更新。
        // -----------------------------------------------------

        MemoryBarrier();


        LONG sequenceAfter =
            g_pdoRtDiagSequence;


        if (sequenceBefore ==
            sequenceAfter &&
            (sequenceAfter & 1) == 0)
        {
            lastPrintedSequence =
                sequenceAfter;


            RtPrintf(
                "[PDO-TIMER-MAIN] "
                "Avg:%lld ns | "
                "Min:%lld ns | "
                "Max:%lld ns | "
                "Short:%ld Normal:%ld Long:%ld\n",

                (long long)
                timerAvgNs,

                (long long)
                timerMinNs,

                (long long)
                timerMaxNs,

                (long)
                timerShort,

                (long)
                timerNormal,

                (long)
                timerLong);


            RtPrintf(
                "[PDO-COMBINED-MAIN] "
                "Avg:%lld ns | "
                "Min:%lld ns | "
                "Max:%lld ns\n",

                (long long)
                combinedAvgNs,

                (long long)
                combinedMinNs,

                (long long)
                combinedMaxNs);


            RtPrintf(
                "[PDO-EXEC-MAIN] "
                "Avg:%lld ns | "
                "Min:%lld ns | "
                "Max:%lld ns | "
                "Over250:%ld | "
                "Over300:%ld | "
                "Over400:%ld\n",

                (long long)
                execAvgNs,

                (long long)
                execMinNs,

                (long long)
                execMaxNs,

                (long)
                execOver250,

                (long)
                execOver300,

                (long)
                execOver400);


            RtPrintf(
                "[ECAT-WKC-MAIN] "
                "LRW:%ld | "
                "DC:%ld | "
                "Snapshot:%ld\n",

                (long)
                lrwWkc,

                (long)
                dcWkc,

                (long)
                sequenceAfter);


        }
    }

    // =============================================================
// DC PLL Diagnostic Snapshot Reader
//
// Priority 64:
//     Calculate + Publish only
//
// Priority 50:
//     Print here
// =============================================================

    static LONG
        lastDcDiagPrintedSequence =
        0;


    LONG dcSequenceBefore =
        g_dcPllDiagSequence;


    // -------------------------------------------------------------
    // 0   = 尚未有 DC snapshot
    // odd = writer 正在更新
    // -------------------------------------------------------------

    if (dcSequenceBefore != 0 &&
        (dcSequenceBefore & 1) == 0 &&
        dcSequenceBefore !=
        lastDcDiagPrintedSequence)
    {
        MemoryBarrier();


        // =========================================================
        // Copy volatile snapshot -> local variables
        // =========================================================

        LONGLONG estimatorSequence =
            g_dcPllDiagEstimatorSequence;

        LONGLONG estimatorOffsetNs =
            g_dcPllDiagEstimatorOffsetNs;

        LONGLONG driftPpb =
            g_dcPllDiagDriftPpb;

        LONGLONG pdoPhaseNs =
            g_dcPllDiagPdoPhaseNs;

        LONGLONG targetPhaseNs =
            g_dcPllDiagTargetPhaseNs;

        LONGLONG phaseStepNs =
            g_dcPllDiagPhaseStepNs;

        LONGLONG wrappedErrorNs =
            g_dcPllDiagWrappedErrorNs;

        LONGLONG unwrappedErrorNs =
            g_dcPllDiagUnwrappedErrorNs;

        LONGLONG sync0MarginNs =
            g_dcPllDiagSync0MarginNs;
        LONGLONG pCommandNs =
            g_dcPllDiagPCommandNs;

        LONGLONG periodCorrectionPs =
            g_dcPllDiagPeriodCorrectionPs;

        LONG targetCaptured =
            g_dcPllDiagTargetCaptured;

        LONG stableWindows =
            g_dcPllDiagStableWindows;


        MemoryBarrier();


        LONG dcSequenceAfter =
            g_dcPllDiagSequence;


        // =========================================================
        // Snapshot 必須前後 Sequence 完全一致
        // =========================================================

        if (dcSequenceBefore ==
            dcSequenceAfter &&
            (dcSequenceAfter & 1) == 0)
        {
            lastDcDiagPrintedSequence =
                dcSequenceAfter;


            if (targetCaptured != 0)
            {
                RtPrintf(
                    "[DC-PLL-MAIN] "
                    "Seq:%lld | "
                    "Offset:%lld ns | "
                    "Drift:%lld ppb | "
                    "Phase:%lld ns | "
                    "Target:%lld ns | "
                    "Step:%+lld ns | "
                    "ErrWrap:%+lld ns | "
                    "ErrUnwrap:%+lld ns | "
                    "Sync0Margin:%lld ns | "
                    "PCmd:%+lld ns | "
                    "PeriodFF:%+lld ps/cycle | "
                    "Control:OFF | "
                    "Lock:YES | "
                    "Snap:%ld\n",

                    (long long)
                    estimatorSequence,

                    (long long)
                    estimatorOffsetNs,

                    (long long)
                    driftPpb,

                    (long long)
                    pdoPhaseNs,

                    (long long)
                    targetPhaseNs,

                    (long long)
                    phaseStepNs,

                    (long long)
                    wrappedErrorNs,

                    (long long)
                    unwrappedErrorNs,

                    (long long)
                    sync0MarginNs,
                    (long long)
                    pCommandNs,

                    (long long)
                    periodCorrectionPs,
                    (long)
                    dcSequenceAfter);
            }
            else
            {
                RtPrintf(
                    "[DC-PLL-MAIN-WAIT] "
                    "Seq:%lld | "
                    "Offset:%lld ns | "
                    "Drift:%lld ppb | "
                    "Phase:%lld ns | "
                    "Stable:%ld/2 | "
                    "Lock:NO | "
                    "Snap:%ld\n",

                    (long long)
                    estimatorSequence,

                    (long long)
                    estimatorOffsetNs,

                    (long long)
                    driftPpb,

                    (long long)
                    pdoPhaseNs,

                    (long)
                    stableWindows,

                    (long)
                    dcSequenceAfter);
            }
        }
    }

    static LONG lastRealFfClampSelfTestSeq = 0;
    LONG cs1 = g_qpcRealFfClampSelfTestSeq;

    if (cs1 != 0 &&
        !(cs1 & 1) &&
        cs1 != lastRealFfClampSelfTestSeq)
    {
        MemoryBarrier();

        LONG pass = g_qpcRealFfClampSelfTestPass;
        LONG cases = g_qpcRealFfClampSelfTestCases;
        LONG fail = g_qpcRealFfClampSelfTestFail;

        LONGLONG r0 = g_qpcRealFfClampSelfTestRec0;
        LONGLONG d0 = g_qpcRealFfClampSelfTestDesired0;
        LONG c0 = g_qpcRealFfClampSelfTestClamp0;

        LONGLONG r1 = g_qpcRealFfClampSelfTestRec1;
        LONGLONG d1 = g_qpcRealFfClampSelfTestDesired1;
        LONG c1 = g_qpcRealFfClampSelfTestClamp1;

        LONGLONG r2 = g_qpcRealFfClampSelfTestRec2;
        LONGLONG d2 = g_qpcRealFfClampSelfTestDesired2;
        LONG c2 = g_qpcRealFfClampSelfTestClamp2;

        LONGLONG r3 = g_qpcRealFfClampSelfTestRec3;
        LONGLONG d3 = g_qpcRealFfClampSelfTestDesired3;
        LONG c3 = g_qpcRealFfClampSelfTestClamp3;

        LONGLONG r4 = g_qpcRealFfClampSelfTestRec4;
        LONGLONG d4 = g_qpcRealFfClampSelfTestDesired4;
        LONG c4 = g_qpcRealFfClampSelfTestClamp4;

        MemoryBarrier();
        LONG cs2 = g_qpcRealFfClampSelfTestSeq;

        if (cs1 == cs2 && !(cs2 & 1))
        {
            lastRealFfClampSelfTestSeq = cs2;

            RtPrintf(
                "[QPC-REAL-FF-CLAMP-SELFTEST] "
                "Pass:%s Cases:%ld Fail:%ld | "
                "C0:%+lld->%+lld Clamp:%s | "
                "C1:%+lld->%+lld Clamp:%s | "
                "C2:%+lld->%+lld Clamp:%s | "
                "C3:%+lld->%+lld Clamp:%s | "
                "C4:%+lld->%+lld Clamp:%s | "
                "Control:OFF Snap:%ld\n",
                pass ? "YES" : "NO",
                (long)cases,
                (long)fail,
                (long long)r0, (long long)d0, c0 ? "YES" : "NO",
                (long long)r1, (long long)d1, c1 ? "YES" : "NO",
                (long long)r2, (long long)d2, c2 ? "YES" : "NO",
                (long long)r3, (long long)d3, c3 ? "YES" : "NO",
                (long long)r4, (long long)d4, c4 ? "YES" : "NO",
                (long)cs2);
        }
    }


    static LONG lastRealFfV0Seq = 0;
    LONG rf1 = g_qpcRealFfV0Seq;
    if (rf1 != 0 && !(rf1 & 1) && rf1 != lastRealFfV0Seq)
    {
        MemoryBarrier();
        LONG st = g_qpcRealFfV0State;
        LONG good = g_qpcRealFfV0PhaseGood;
        LONG arm = g_qpcRealFfV0ArmGood;
        LONG trip = g_qpcRealFfV0TripMask;
        LONG trips = g_qpcRealFfV0TripCount;
        LONGLONG rec = g_qpcRealFfV0RecommendedPpb;
        LONGLONG des = g_qpcRealFfV0DesiredPpb;
        LONGLONG app = g_qpcRealFfV0AppliedPpb;
        LONGLONG step = g_qpcRealFfV0LastStepPpb;
        LONGLONG delta = g_qpcRealFfV0TargetVsFixedNs;
        LONGLONG dcEst = g_qpcRealFfV0DcErrEstNs;
        LONG pseq = g_qpcRealFfV0PhaseSeq;
        LONG reject = g_qpcRealFfV0PhaseRejectMask;
        LONG lastReject = g_qpcRealFfV0LastPhaseRejectMask;
        LONG holdGood = g_qpcRealFfV0HoldGood;
        LONG holdBad = g_qpcRealFfV0HoldBad;
        LONG holdEntries = g_qpcRealFfV0HoldEntries;
        LONG clampActive = g_qpcRealFfV0ClampActive;
        MemoryBarrier();
        LONG rf2 = g_qpcRealFfV0Seq;

        if (rf1 == rf2 && !(rf2 & 1))
        {
            lastRealFfV0Seq = rf2;
            const char* stateText =
                st == 2 ? "ACTIVE" :
                st == 1 ? "ARMING" :
                st == 3 ? "HOLD" :
                st == 4 ? "LATCHED" : "WAIT";

            // 正式 Real FF 判讀：ACTIVE + PhaseGood:YES + Reject:0 + TripMask:0。
            // Rec 是 observer 建議；Desired 是限幅後目標；Applied 才是真正排程使用值；
            // Step 是本觀測窗實際變化。ClampActive:YES 時不要直接放寬上下限。
            RtPrintf(
                "[QPC-REAL-FF-V0C-MAIN] "
                "State:%s PhaseGood:%s Arm:%ld/3 | "
                "Rec:%+lld Desired:%+lld Applied:%+lld Step:%+lld ppb "
                "ClampActive:%s | "
                "Reject:0x%02lX LastReject:0x%02lX "
                "HoldGood:%ld/3 HoldBad:%ld/5 HoldEntries:%ld | "
                "TargetVsFixed:%+lld ns RealDcEst:%+lld ns | "
                "TripMask:0x%02lX Trips:%ld PhaseSeq:%ld | "
                "Control:ON Snap:%ld\n",
                stateText, good ? "YES" : "NO", (long)arm,
                (long long)rec, (long long)des,
                (long long)app, (long long)step,
                clampActive ? "YES" : "NO",
                (long)reject, (long)lastReject,
                (long)holdGood, (long)holdBad, (long)holdEntries,
                (long long)delta, (long long)dcEst,
                (long)trip, (long)trips, (long)pseq, (long)rf2);
        }
    }


    static LONG lastPhasePActV0Seq = 0;
    LONG pp1 = g_qpcPhasePActV0Seq;

    if (pp1 != 0 &&
        !(pp1 & 1) &&
        pp1 != lastPhasePActV0Seq)
    {
        MemoryBarrier();

        LONG state = g_qpcPhasePActV0State;
        LONG gate = g_qpcPhasePActV0GateGood;
        LONG arm = g_qpcPhasePActV0ArmGood;
        LONGLONG baseErr = g_qpcPhasePActV0BaseErrNs;
        LONGLONG actualErr = g_qpcPhasePActV0ActualErrNs;
        LONGLONG wrapped = g_qpcPhasePActV0WrappedErrNs;
        LONGLONG rawCorr = g_qpcPhasePActV0RawCorrectionNs;
        LONGLONG cmd = g_qpcPhasePActV0CommandNs;
        LONGLONG step = g_qpcPhasePActV0StepNs;
        LONGLONG offset = g_qpcPhasePActV0OffsetNs;
        LONGLONG predicted = g_qpcPhasePActV0PredictedErrNs;
        LONG cmdSat = g_qpcPhasePActV0CommandSat;
        LONG offsetSat = g_qpcPhasePActV0OffsetSat;
        LONG improve = g_qpcPhasePActV0Improve;
        LONG holdGood = g_qpcPhasePActV0HoldGood;
        LONG holdEntries = g_qpcPhasePActV0HoldEntries;
        LONG trips = g_qpcPhasePActV0TripCount;
        LONG ffState = g_qpcPhasePActV0RealFfState;

        MemoryBarrier();
        LONG pp2 = g_qpcPhasePActV0Seq;

        if (pp1 == pp2 && !(pp2 & 1))
        {
            lastPhasePActV0Seq = pp2;

            const char* stateText =
                state == 2 ? "ACTIVE" :
                state == 1 ? "ARMING" :
                state == 3 ? "HOLD" :
                state == 4 ? "LATCHED" : "WAIT";

            const char* direction =
                step < 0 ? "EARLIER" :
                step > 0 ? "LATER" : "NONE";

            // 正式 Phase-P 判讀：ACTIVE + Gate:YES + OffsetSat:NO + Improve:YES。
            // ActualErr 是目前控制後誤差；Wrap 是 P 控制使用的 ±125 us 等價誤差；
            // Step<0 代表提早、Step>0 代表延後。Offset 持續單向走表示仍有頻率殘差。
            RtPrintf(
                "[QPC-PHASE-P-ACT-V0-MAIN] "
                "State:%s Gate:%s Arm:%ld/3 RealFF:%ld | "
                "BaseErr:%+lld ActualErr:%+lld Wrap:%+lld ns | "
                "RawCorr:%+lld Cmd:%+lld Step:%+lld ns Direction:%s | "
                "Offset:%+lld ns CmdSat:%s OffsetSat:%s "
                "PredErr:%+lld Improve:%s | "
                "HoldGood:%ld/3 HoldEntries:%ld Trips:%ld | "
                "PDiv:8 Slew:250ns/win MaxOffset:120000ns Deadband:500ns | "
                "Control:ON Snap:%ld\n",
                stateText, gate ? "YES" : "NO",
                (long)arm, (long)ffState,
                (long long)baseErr, (long long)actualErr,
                (long long)wrapped, (long long)rawCorr,
                (long long)cmd, (long long)step, direction,
                (long long)offset,
                cmdSat ? "YES" : "NO",
                offsetSat ? "YES" : "NO",
                (long long)predicted,
                improve ? "YES" : "NO",
                (long)holdGood, (long)holdEntries,
                (long)trips, (long)pp2);
        }
    }


    static LONG lastRecoverySelfTestSeq = 0;
    LONG rst1 = g_pdoRecoverySelfTestSeq;
    if (rst1 != 0 && !(rst1 & 1) && rst1 != lastRecoverySelfTestSeq)
    {
        MemoryBarrier();
        LONG pass = g_pdoRecoverySelfTestPass;
        LONG cases = g_pdoRecoverySelfTestCases;
        LONG fail = g_pdoRecoverySelfTestFail;
        LONGLONG b0 = g_pdoRecoverySelfTestBefore0;
        LONGLONG a0 = g_pdoRecoverySelfTestAfter0;
        LONG k0 = g_pdoRecoverySelfTestSkip0;
        LONGLONG b1 = g_pdoRecoverySelfTestBefore1;
        LONGLONG a1 = g_pdoRecoverySelfTestAfter1;
        LONG k1 = g_pdoRecoverySelfTestSkip1;
        LONGLONG b2 = g_pdoRecoverySelfTestBefore2;
        LONGLONG a2 = g_pdoRecoverySelfTestAfter2;
        LONG k2 = g_pdoRecoverySelfTestSkip2;
        LONGLONG b3 = g_pdoRecoverySelfTestBefore3;
        LONGLONG a3 = g_pdoRecoverySelfTestAfter3;
        LONG k3 = g_pdoRecoverySelfTestSkip3;
        LONG shadow = g_pdoRecoverySelfTestShadowPreserve;
        MemoryBarrier();
        LONG rst2 = g_pdoRecoverySelfTestSeq;

        if (rst1 == rst2 && !(rst2 & 1))
        {
            lastRecoverySelfTestSeq = rst2;
            RtPrintf(
                "[PDO-RUNTIME-RECOVERY-SELFTEST] "
                "Pass:%s Cases:%ld Fail:%ld | "
                "C0:%+lld->%+lld Skip:%ld | "
                "C1:%+lld->%+lld Skip:%ld | "
                "C2:%+lld->%+lld Skip:%ld | "
                "C3:%+lld->%+lld Skip:%ld | "
                "ShadowPreserve:%s Control:OFF Snap:%ld\n",
                pass ? "YES" : "NO", (long)cases, (long)fail,
                (long long)b0, (long long)a0, (long)k0,
                (long long)b1, (long long)a1, (long)k1,
                (long long)b2, (long long)a2, (long)k2,
                (long long)b3, (long long)a3, (long)k3,
                shadow ? "YES" : "NO", (long)rst2);
        }
    }

    static LONG lastRecoveryForensicSeq = 0;
    LONG forensicSeqBefore = g_pdoRecoveryForensicSeq;

    if (g_pdoRecoveryForensicCaptured != 0 &&
        forensicSeqBefore != 0 &&
        (forensicSeqBefore & 1) == 0 &&
        forensicSeqBefore != lastRecoveryForensicSeq)
    {
        MemoryBarrier();

        LONGLONG wakeIntervalNs =
            g_pdoRecoveryForensicWakeIntervalNs;
        LONGLONG leadAtWakeNs =
            g_pdoRecoveryForensicLeadAtWakeNs;
        LONGLONG wakeToArmNs =
            g_pdoRecoveryForensicWakeToArmNs;
        LONGLONG leadAtArmNs =
            g_pdoRecoveryForensicLeadAtArmNs;
        LONGLONG leadAfterNs =
            g_pdoRecoveryForensicLeadAfterNs;
        LONG skipCycles =
            g_pdoRecoveryForensicSkipCycles;

        MemoryBarrier();

        LONG forensicSeqAfter =
            g_pdoRecoveryForensicSeq;

        if (forensicSeqBefore == forensicSeqAfter &&
            (forensicSeqAfter & 1) == 0)
        {
            lastRecoveryForensicSeq = forensicSeqAfter;

            RtPrintf(
                "[PDO-RECOVERY-FORENSIC] "
                "WakeInterval:%lld ns | "
                "LeadAtWake:%+lld ns | "
                "WakeToArm:%lld ns | "
                "LeadAtArm:%+lld ns | "
                "LeadAfter:%+lld ns | "
                "Skip:%ld | Snap:%ld\n",
                (long long)wakeIntervalNs,
                (long long)leadAtWakeNs,
                (long long)wakeToArmNs,
                (long long)leadAtArmNs,
                (long long)leadAfterNs,
                (long)skipCycles,
                (long)forensicSeqAfter);
        }
    }


    // =============================================================
    // PDO One-Shot Bootstrap Fallback Diagnostic V1 Reader
    //
    // Priority 64 publishes counters only.
    // Priority 50 prints them here.
    // Control remains unchanged.
    // =============================================================

    static LONG
        lastPdoBootstrapDiagPrintedSequence =
        0;


    LONG pdoBootstrapSequenceBefore =
        g_pdoBootstrapDiagSequence;


    if (pdoBootstrapSequenceBefore != 0 &&
        (pdoBootstrapSequenceBefore & 1) == 0 &&
        pdoBootstrapSequenceBefore !=
        lastPdoBootstrapDiagPrintedSequence)
    {
        MemoryBarrier();


        LONG bootstrapWindow =
            g_pdoBootstrapWindowBootstrap;

        LONG bootstrapCurrentConsecutive =
            g_pdoBootstrapCurrentConsecutive;

        LONG bootstrapMaxConsecutive =
            g_pdoBootstrapMaxConsecutive;

        LONGLONG bootstrapTotal =
            g_pdoBootstrapTotal;

        LONG bootstrapRecovery =
            g_pdoBootstrapRecoveryAfterBootstrap;


        LONG reasonNotReady =
            g_pdoBootstrapReasonNotReady;

        LONG reasonFinalUnderGuard =
            g_pdoBootstrapReasonFinalUnderGuard;

        LONG reasonQpcBeforeArmFail =
            g_pdoBootstrapReasonQpcBeforeArmFail;

        LONG reasonLeadTooShort =
            g_pdoBootstrapReasonLeadTooShort;

        LONG reasonOther =
            g_pdoBootstrapReasonOther;


        LONG schedulerInitialized =
            g_pdoBootstrapSchedulerInitialized;


        LONG predictedLeadValid =
            g_pdoBootstrapPredictedLeadValid;

        LONGLONG predictedLeadAvgNs =
            g_pdoBootstrapPredictedLeadAvgNs;

        LONGLONG predictedLeadMinNs =
            g_pdoBootstrapPredictedLeadMinNs;

        LONGLONG predictedLeadMaxNs =
            g_pdoBootstrapPredictedLeadMaxNs;


        LONGLONG schedulerErrorNs =
            g_pdoBootstrapSchedulerErrorNs;

        LONGLONG schedulerLateByNs =
            g_pdoBootstrapSchedulerLateByNs;

        LONG runtimeRecoveryEvents =
            g_pdoRuntimeRecoveryWindowEvents;
        LONG runtimeRecoverySkipped =
            g_pdoRuntimeRecoveryWindowSkippedCycles;
        LONG runtimeRecoveryMaxSkip =
            g_pdoRuntimeRecoveryMaxSkipCycles;
        LONGLONG runtimeRecoveryTotal =
            g_pdoRuntimeRecoveryTotalEvents;
        LONGLONG runtimeRecoveryTotalSkipped =
            g_pdoRuntimeRecoveryTotalSkippedCycles;
        LONGLONG runtimeRecoveryLeadBefore =
            g_pdoRuntimeRecoveryLastLeadBeforeNs;
        LONGLONG runtimeRecoveryLeadAfter =
            g_pdoRuntimeRecoveryLastLeadAfterNs;


        MemoryBarrier();


        LONG pdoBootstrapSequenceAfter =
            g_pdoBootstrapDiagSequence;


        if (pdoBootstrapSequenceBefore ==
            pdoBootstrapSequenceAfter &&
            (pdoBootstrapSequenceAfter & 1) == 0)
        {
            lastPdoBootstrapDiagPrintedSequence =
                pdoBootstrapSequenceAfter;


            RtPrintf(
                "[PDO-BOOTSTRAP-REASON-MAIN] "
                "Bootstrap:%ld | "
                "CurrentConsecutive:%ld | "
                "MaxConsecutive:%ld | "
                "Total:%lld | "
                "Recovery:%ld | "
                "Reason NotReady:%ld "
                "FinalUnderGuard:%ld "
                "QpcBeforeArmFail:%ld "
                "LeadTooShort:%ld "
                "Other:%ld | "
                "SchedulerInit:%s | "
                "PredictedLead Avg:%+lld Min:%+lld Max:%+lld ns "
                "Valid:%ld | "
                "RequiredMinLead:50000 ns | "
                "SchedulerErr:%+lld ns | "
                "SchedulerLateBy:%lld ns | "
                "RuntimeRecover:%ld Skip:%ld MaxSkip:%ld "
                "TotalRecover:%lld TotalSkip:%lld "
                "Lead:%+lld->%+lld ns | "
                "Control:ON | "
                "Snap:%ld\n",

                (long)
                bootstrapWindow,

                (long)
                bootstrapCurrentConsecutive,

                (long)
                bootstrapMaxConsecutive,

                (long long)
                bootstrapTotal,

                (long)
                bootstrapRecovery,

                (long)
                reasonNotReady,

                (long)
                reasonFinalUnderGuard,

                (long)
                reasonQpcBeforeArmFail,

                (long)
                reasonLeadTooShort,

                (long)
                reasonOther,

                schedulerInitialized != 0
                ? "YES"
                : "NO",

                (long long)
                predictedLeadAvgNs,

                (long long)
                predictedLeadMinNs,

                (long long)
                predictedLeadMaxNs,

                (long)
                predictedLeadValid,

                (long long)
                schedulerErrorNs,

                (long long)
                schedulerLateByNs,

                (long)
                runtimeRecoveryEvents,

                (long)
                runtimeRecoverySkipped,

                (long)
                runtimeRecoveryMaxSkip,

                (long long)
                runtimeRecoveryTotal,

                (long long)
                runtimeRecoveryTotalSkipped,

                (long long)
                runtimeRecoveryLeadBefore,

                (long long)
                runtimeRecoveryLeadAfter,

                (long)
                pdoBootstrapSequenceAfter);
        }
    }


    // =============================================================
    // PDO One-Shot Scheduler V1B Infrastructure Main Reader
    //
    // Priority 64:
    //     GlobalTimerHandler_PDO() publishes snapshot only.
    //
    // Priority 50:
    //     Read + print here.
    //
    // 正式狀態：
    //     One-Shot Control = ON。
    //     Fine Wait        = OFF。
    // 正常穩態應以 COARSE_ACTIVE 為主，RearmFail=0；啟動時少量 Bootstrap
    // 可接受，運轉中持續增加則要對照 RuntimeRecover/Skip 與 RX timeout。
    // =============================================================

    static LONG
        lastPdoOneShotInfraPrintedSequence =
        0;


    LONG oneShotInfraSequenceBefore =
        g_pdoOneShotInfraSequence;


    if (oneShotInfraSequenceBefore != 0 &&
        (oneShotInfraSequenceBefore & 1) == 0 &&
        oneShotInfraSequenceBefore !=
        lastPdoOneShotInfraPrintedSequence)
    {
        MemoryBarrier();


        // =========================================================
        // Copy volatile snapshot
        // =========================================================

        LONG infraState =
            g_pdoOneShotInfraState;

        LONG infraValid =
            g_pdoOneShotInfraValid;


        LONG controlEnabled =
            g_pdoOneShotInfraControlEnabled;

        LONG fineWaitEnabled =
            g_pdoOneShotInfraFineWaitEnabled;


        LONG warmupRequired =
            g_pdoOneShotInfraWarmupRequired;

        LONG warmupComplete =
            g_pdoOneShotInfraWarmupComplete;


        LONGLONG qpcFrequency =
            g_pdoOneShotInfraQpcFrequency;


        LONGLONG finalTargetQpc =
            g_pdoOneShotInfraFinalTargetQpc;

        LONGLONG coarseTargetQpc =
            g_pdoOneShotInfraCoarseTargetQpc;

        LONGLONG actualWakeQpc =
            g_pdoOneShotInfraActualWakeQpc;


        LONGLONG guardNs =
            g_pdoOneShotInfraGuardNs;

        LONGLONG toCoarseNs =
            g_pdoOneShotInfraToCoarseNs;

        LONGLONG toFinalNs =
            g_pdoOneShotInfraToFinalNs;

        LONGLONG finalLateByNs =
            g_pdoOneShotInfraFinalLateByNs;


        LONG dcDiagSequence =
            g_pdoOneShotInfraDcDiagSequence;

        LONG windowSamples =
            g_pdoOneShotInfraWindowSamples;


        LONG readyCount =
            g_pdoOneShotInfraReadyCount;

        LONG coarseWindowCount =
            g_pdoOneShotInfraCoarseWindowCount;

        LONG finalLateCount =
            g_pdoOneShotInfraFinalLateCount;


        LONG rearmOkCount =
            g_pdoOneShotInfraRearmOkCount;

        LONG rearmFailCount =
            g_pdoOneShotInfraRearmFailCount;

        LONG bootstrapCount =
            g_pdoOneShotInfraBootstrapCount;

        LONG activeCount =
            g_pdoOneShotInfraActiveCount;


        LONGLONG coarseErrorAvgNs =
            g_pdoOneShotInfraCoarseErrorAvgNs;

        LONGLONG coarseErrorMinNs =
            g_pdoOneShotInfraCoarseErrorMinNs;

        LONGLONG coarseErrorMaxNs =
            g_pdoOneShotInfraCoarseErrorMaxNs;


        LONGLONG finalMarginAvgNs =
            g_pdoOneShotInfraFinalMarginAvgNs;

        LONGLONG finalMarginMinNs =
            g_pdoOneShotInfraFinalMarginMinNs;

        LONGLONG finalMarginMaxNs =
            g_pdoOneShotInfraFinalMarginMaxNs;


        LONGLONG rearmCostAvgNs =
            g_pdoOneShotInfraRearmCostAvgNs;

        LONGLONG rearmCostMinNs =
            g_pdoOneShotInfraRearmCostMinNs;

        LONGLONG rearmCostMaxNs =
            g_pdoOneShotInfraRearmCostMaxNs;


        MemoryBarrier();


        LONG oneShotInfraSequenceAfter =
            g_pdoOneShotInfraSequence;
        // =========================================================
                    // Seqlock validation
                    // =========================================================

        if (oneShotInfraSequenceBefore ==
            oneShotInfraSequenceAfter &&
            (oneShotInfraSequenceAfter & 1) == 0)
        {
            lastPdoOneShotInfraPrintedSequence =
                oneShotInfraSequenceAfter;


            const char* infraStateText =
                "UNKNOWN";


            switch (infraState)
            {
            case 0:
                infraStateText =
                    "WAIT_TARGET";
                break;

            case 1:
                infraStateText =
                    "READY_BEFORE_COARSE";
                break;

            case 2:
                infraStateText =
                    "IN_COARSE_WINDOW";
                break;

            case 3:
                infraStateText =
                    "FINAL_TARGET_LATE";
                break;

            case 4:
                infraStateText =
                    "COARSE_ACTIVE";
                break;

            default:
                infraStateText =
                    "UNKNOWN";
                break;
            }


            // One-Shot 判讀重點：Control=ON、FineWait=OFF、WarmupDone=YES、
            // RearmFail=0。FinalMargin 應為正；LateBy>0 或 Bootstrap/Recovery 增加
            // 表示 callback／re-arm 已落後，不可只靠放寬 RX deadline 掩蓋。
            RtPrintf(
                "[PDO-ONESHOT-INFRA-MAIN] "
                "State:%s | "
                "Valid:%s | "
                "Control:%s | "
                "FineWait:%s | "
                "WarmupReq:%s | "
                "WarmupDone:%s | "
                "QPCFreq:%lld Hz | "
                "Final:%lld | "
                "Coarse:%lld | "
                "Wake:%lld | "
                "Guard:%lld ns | "
                "ToCoarse:%+lld ns | "
                "ToFinal:%+lld ns | "
                "LateBy:%lld ns | "
                "RearmOK:%ld RearmFail:%ld | "
                "Bootstrap:%ld Active:%ld | "
                "CoarseErr Avg:%+lld Min:%+lld Max:%+lld ns | "
                "FinalMargin Avg:%lld Min:%lld Max:%lld ns | "
                "RearmCost Avg:%lld Min:%lld Max:%lld ns | "
                "DCSeq:%ld | "
                "Samples:%ld | "
                "Snap:%ld\n",

                infraStateText,

                infraValid
                ? "YES"
                : "NO",

                controlEnabled
                ? "ON"
                : "OFF",

                fineWaitEnabled
                ? "ON"
                : "OFF",

                warmupRequired
                ? "YES"
                : "NO",

                warmupComplete
                ? "YES"
                : "NO",

                (long long)
                qpcFrequency,

                (long long)
                finalTargetQpc,

                (long long)
                coarseTargetQpc,

                (long long)
                actualWakeQpc,

                (long long)
                guardNs,

                (long long)
                toCoarseNs,

                (long long)
                toFinalNs,

                (long long)
                finalLateByNs,

                (long)
                rearmOkCount,

                (long)
                rearmFailCount,

                (long)
                bootstrapCount,

                (long)
                activeCount,

                (long long)
                coarseErrorAvgNs,

                (long long)
                coarseErrorMinNs,

                (long long)
                coarseErrorMaxNs,

                (long long)
                finalMarginAvgNs,

                (long long)
                finalMarginMinNs,

                (long long)
                finalMarginMaxNs,

                (long long)
                rearmCostAvgNs,

                (long long)
                rearmCostMinNs,

                (long long)
                rearmCostMaxNs,

                (long)
                dcDiagSequence,

                (long)
                windowSamples,

                (long)
                oneShotInfraSequenceAfter);
        }
    }


    // =============================================================
// PDO Fine Scheduler Timing Snapshot Reader
//
// Priority 64:
//     QPC/TSC measurement + publish
//
// Priority 50:
//     Print here
// =============================================================

    static LONG
        lastFineDiagPrintedSequence =
        0;


    LONG fineSequenceBefore =
        g_pdoFineDiagSequence;


    if (fineSequenceBefore != 0 &&
        (fineSequenceBefore & 1) == 0 &&
        fineSequenceBefore !=
        lastFineDiagPrintedSequence)
    {
        MemoryBarrier();


        // =========================================================
        // Copy volatile snapshot
        // =========================================================

        LONGLONG qpcFrequency =
            g_pdoFineQpcFrequency;


        LONGLONG intervalAvgNs =
            g_pdoFineIntervalAvgNs;

        LONGLONG intervalMinNs =
            g_pdoFineIntervalMinNs;

        LONGLONG intervalMaxNs =
            g_pdoFineIntervalMaxNs;


        LONGLONG wakeToEcatAvgNs =
            g_pdoFineWakeToEcatAvgNs;

        LONGLONG wakeToEcatMinNs =
            g_pdoFineWakeToEcatMinNs;

        LONGLONG wakeToEcatMaxNs =
            g_pdoFineWakeToEcatMaxNs;


        LONGLONG qpcReadAvgNs =
            g_pdoFineQpcReadAvgNs;

        LONGLONG qpcReadMaxNs =
            g_pdoFineQpcReadMaxNs;


        LONG qpcValid =
            g_pdoFineQpcValid;


        MemoryBarrier();


        LONG fineSequenceAfter =
            g_pdoFineDiagSequence;


        // =========================================================
        // Seqlock validation
        // =========================================================

        if (fineSequenceBefore ==
            fineSequenceAfter &&
            (fineSequenceAfter & 1) == 0)
        {
            lastFineDiagPrintedSequence =
                fineSequenceAfter;


            RtPrintf(
                "[PDO-FINE-MAIN] "
                "QPCFreq:%lld Hz | "
                "Interval Avg:%lld Min:%lld Max:%lld ns | "
                "WakeToEcat Avg:%lld Min:%lld Max:%lld ns | "
                "QPCRead Avg:%lld Max:%lld ns | "
                "Valid:%ld | "
                "Snap:%ld\n",

                (long long)
                qpcFrequency,

                (long long)
                intervalAvgNs,

                (long long)
                intervalMinNs,

                (long long)
                intervalMaxNs,

                (long long)
                wakeToEcatAvgNs,

                (long long)
                wakeToEcatMinNs,

                (long long)
                wakeToEcatMaxNs,

                (long long)
                qpcReadAvgNs,

                (long long)
                qpcReadMaxNs,

                (long)
                qpcValid,

                (long)
                fineSequenceAfter);
        }
    }

    PrintDcHealthSummary();
}
