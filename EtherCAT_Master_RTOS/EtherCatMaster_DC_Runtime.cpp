#include "EtherCatMaster.h"
#include <windows.h> 
#include <rtapi.h> 
#include <rtssapi.h> 
#include <cstring>
#include <stdio.h>
#include "GlobalConfig.h"
#include "PLCManager.h" // 🌟 1. 記得 include PLCManager 標頭檔
#define MAX_MBX_SIZE 1024

// ============================================================================
// EtherCatMaster_DC_Runtime.cpp
// EtherCAT DC 即時循環正式版候選 RC1.2（完整調參／Debug 維護註解版）
//
// 本檔責任：
//   1. 執行 4 kHz／250 us PDO 即時循環。
//   2. 用 LRW + FRMW 在同一個 Ethernet frame 交換 PDO，並擷取 S4 DC 時間。
//   3. 維護 QPC <-> S4 DC 對映、漂移觀測器、Real FF 與 Phase-P 控制器。
//   4. 在 EtherCAT 通訊有效時更新 PLC Input、Motion 與非同步命令。
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
//   - 固定安全漂移基準：-8300 ppb。
//   - One-Shot coarse guard：100000 ns；Fine Wait 仍為 OFF。
//   - Real FF 範圍：-10000..-7800 ppb，每個觀測窗最多變更 10 ppb。
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
//   REAL_FF_V0_MIN_PPB / MAX_PPB = -10000 / -7800 ppb
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


// LRW PDO WKC 與 FRMW DC WKC；用來判斷本週期資料是否可採用。
volatile LONG g_pdoRtLrwWkc = 0;
volatile LONG g_pdoRtDcWkc = 0;


// =============================================================
// QPC <-> EtherCAT S4 DC 頻率估測快照（僅診斷）
//
// 用 EtherCAT 呼叫前後 QPC 中點對應 S4 DC time，降低固定通訊延遲的影響。
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

extern volatile LONG g_ecatRxDiagCurrentConsecutiveTimeout;

// ============================================================================
// Real FF V0 與 Phase-P V0 正式控制快照
//
// Real FF 狀態：0=WAIT、1=ARM、2=ACTIVE、3=HOLD、4=TRIP/FALLBACK。
//   - ACTIVE 時把合格的 Frequency FF V2 建議值，限幅與限速後套入 QPC period。
//   - TripMask 0x01 表示相位觀測器進入 FALLBACK；0x02 表示連續 RX timeout。
//   - 一旦進入 state 4，本次執行期間維持固定 -8300 ppb 安全值。
//
// Phase-P 狀態：0=WAIT、1=ARM、2=ACTIVE、3=HOLD、4=TRIP。
//   - 只有 Real FF ACTIVE、phase gate 合格、TripMask=0 時才修正 offset。
//   - ActualErr/Offset/Step 都是 ns；offset 會真正加到 One-Shot final target。
// ============================================================================
volatile LONG g_qpcRealFfV0Seq = 0;
volatile LONG g_qpcRealFfV0State = 0;
volatile LONG g_qpcRealFfV0PhaseGood = 0;
volatile LONG g_qpcRealFfV0ArmGood = 0;
volatile LONG g_qpcRealFfV0TripMask = 0;
volatile LONG g_qpcRealFfV0TripCount = 0;
volatile LONGLONG g_qpcRealFfV0RecommendedPpb = -8300;
volatile LONGLONG g_qpcRealFfV0DesiredPpb = -8300;
volatile LONGLONG g_qpcRealFfV0AppliedPpb = -8300;
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
// QPC <-> S4 Robust Drift V1 快照（僅觀測）
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
// QPC <-> S4 Trusted Drift V1A 快照（僅觀測）
//
// 狀態：0=WARMUP、1=TRACK、2=HOLD、3=UNTRUSTED。
// WARMUP 需連續合格窗；TRACK 以受限 slew 更新；HOLD 保留最後可信值；
// UNTRUSTED 回到 -8300 ppb 並等待重新鎖定。本觀測器不直接修改 timer/HAL。
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
-8300;

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
-8300;


// =============================================================
// Trusted Drift -> QPC Live Feed-Forward V1 Shadow 快照
//
// 使用獨立 target accumulator，比較 Trusted 與固定 -8300 ppb 的長期差異。
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
-8300;

volatile LONGLONG
g_qpcLiveFfAppliedDriftPpb =
-8300;

volatile LONGLONG
g_qpcLiveFfAppliedPeriodFfPs =
-2075;

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
// 把 Fixed target 與 Trusted shadow target 都投影到 S4 DC 時域，比較 wrapped／
// unwrapped phase、Sync0 margin、平均絕對誤差與優劣次數。對映使用當前有效的
// QPC midpoint / DC_reference_time pair；結果不直接驅動 timer。
// =============================================================

volatile LONG
g_qpcLiveFfDcPhaseDiagSequence =
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
// S4 DC Phase Residual Drift Observer V1 快照（僅診斷）
//
// 以固定 -8300 ppb timeline 的相位在約一秒內的變化估算 residual drift；
// RobustResidualPpb 是通過門檻樣本的 rolling median；建議排程值為
// -8300 - RobustResidualPpb。此 V1 結果不直接控制 scheduler。
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
-8300;

volatile LONGLONG
g_qpcDcPhaseResidualTrustedDriftPpb =
-8300;

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
// S4 DC Phase Residual Drift Observer V1A 快照（僅診斷）
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
-8300;

volatile LONGLONG
g_qpcDcPhaseResidualV1ATrustedDriftPpb =
-8300;

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
-8300;

volatile LONGLONG
g_qpcPhaseFfV2DesiredPpb =
-8300;

volatile LONGLONG
g_qpcPhaseFfV2AppliedPpb =
-8300;

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
// Frequency FF V2 與 S4 DC 相位比較快照（V2 shadow 效果驗證）
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
//   3. Flush PLC outputs，送出 LRW+FRMW，驗證 PDO/DC WKC。
//   4. 更新 QPC<->S4 DC 與各種 shadow observer snapshot。
//   5. Fetch PLC inputs，更新 DC 軟體估測器／控制器。
//   6. 處理低頻 async command，更新 Motion，發布執行時間快照。
//
// 安全原則：下一次 timer 必須在 EtherCAT 通訊前 re-arm；通訊失敗時不採用 Input，
// 連續兩個無效週期才觸發各軸 EmergencyStop，避免單次雜訊造成不必要停機。
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
    static int64_t realFfV0DesiredPpb = -8300LL;
    static int64_t realFfV0AppliedPpb = -8300LL;
    static int64_t realFfV0LastStepPpb = 0;
    static LONG realFfV0TripMask = 0;
    static uint32_t realFfV0TripCount = 0;
    static bool realFfV0OneShotHealthy = false;
    static LONG realFfV0PhaseRejectMask = 0;
    static LONG realFfV0LastPhaseRejectMask = 0;
    static uint32_t realFfV0HoldGood = 0;
    static uint32_t realFfV0HoldBad = 0;
    static uint32_t realFfV0HoldEntries = 0;
    static bool realFfV0ClampActive = false;
    static bool realFfClampSelfTestDone = false;

    static LONG phasePActV0State = 0;
    static uint32_t phasePActV0ArmGood = 0;
    static uint32_t phasePActV0HoldGood = 0;
    static uint32_t phasePActV0HoldEntries = 0;
    static uint32_t phasePActV0TripCount = 0;
    static int64_t phasePActV0OffsetNs = 0;
    static int64_t phasePActV0LastStepNs = 0;


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
        -8300LL;

    static int64_t
        qpcPhaseFfV2AppliedPpb =
        -8300LL;

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
    // 正式控制參數集中區
    //
    // QPC_SCHEDULER_ASSUMED_DRIFT_PPB 是啟動、Trip 與觀測不可信時的安全基準。
    // Real FF 必須連續 3 個合格觀測窗才能 ACTIVE；ACTIVE 每窗最多走 10 ppb，
    // 並限制在 -10000..-7800 ppb，避免單一估測異常直接改變週期。
    // Phase-P 每次使用 wrapped phase error 的 1/8；小於 500 ns 不動作；
    // command、step、累積 offset 各有獨立飽和，防止相位迴路突跳。
    // ---------------------------------------------------------------------
    const int64_t QPC_SCHEDULER_ASSUMED_DRIFT_PPB =
        -8300LL;

    const int64_t REAL_FF_V0_MIN_PPB = -10000LL; // 安全下限；更負代表目標週期更短。
    const int64_t REAL_FF_V0_MAX_PPB = -7800LL;  // 安全上限；不可只為消除 Clamp 而放寬。
    const int64_t REAL_FF_V0_MAX_STEP_PPB = 10LL; // 每個約一秒觀測窗最大頻率變更。
    const uint32_t REAL_FF_V0_ARM_WINDOWS = 3U;   // 連續合格 3 窗才進入 ACTIVE。
    const uint32_t REAL_FF_V0_HOLD_RECOVERY_WINDOWS = 3U; // HOLD 連續合格 3 窗才恢復。
    const uint32_t REAL_FF_V0_HOLD_BAD_LIMIT = 5U; // HOLD 連續失敗 5 窗即 LATCHED。

    const int64_t PHASE_P_ACT_CYCLE_NS = 250000LL; // PDO 週期；不可單獨調整。
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
        { -8750LL, -10044LL, -7600LL, -10000LL, -7800LL };

        const int64_t expectedDesired[5] =
        { -8750LL, -10000LL, -7800LL, -10000LL, -7800LL };

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
        // 只有 mask=0 才能累積 ARM 或在 ACTIVE 中更新 drift。
        // -----------------------------------------------------------------
        LONG realPhaseSeq1 = g_qpcPhaseFfV2DiagSequence;
        LONG realPhaseState = 0;
        LONG realPhaseCandidate = 0;
        LONG realPhaseLock = 0;
        LONG realPhasePoint = 0;
        LONG realPhasePoints = 0;
        LONG realPhasePairs = 0;
        LONGLONG realPhaseMad = 0;
        LONGLONG realPhaseRecommended = -8300;
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
        }

        realFfV0PhaseRejectMask = phaseReject;
        bool realPhaseGood = phaseReject == 0;

        if ((realFfV0State == 2 || realFfV0State == 3) &&
            g_ecatRxDiagCurrentConsecutiveTimeout != 0)
        {
            // ACTIVE/HOLD 期間只要看到連續 RX timeout 非零，立即永久 Trip 到
            // 本次執行的固定 -8300 ppb fallback，避免錯誤時間樣本影響 FF。
            realFfV0State = 4;
            realFfV0AppliedPpb = QPC_SCHEDULER_ASSUMED_DRIFT_PPB;
            realFfV0DesiredPpb = QPC_SCHEDULER_ASSUMED_DRIFT_PPB;
            realFfV0LastStepPpb = 0;
            realFfV0TripMask |= 0x02;
            realFfV0TripCount++;
        }

        if (realPhaseSnapshot &&
            realPhaseSeq1 != realFfV0LastPhaseSeq)
        {
            // 狀態機只在 V2 發布「新的完整觀測窗」時走一次；不能在 4 kHz
            // 每個 callback 重複累積 ARM/HOLD 計數。
            realFfV0LastPhaseSeq = realPhaseSeq1;

            if (realFfV0State == 4)
            {
                realFfV0AppliedPpb = QPC_SCHEDULER_ASSUMED_DRIFT_PPB;
            }
            else if (realFfV0State == 0 || realFfV0State == 1)
            {
                if (realPhaseGood &&
                    realFfV0OneShotHealthy &&
                    qpcSchedulerInitialized &&
                    g_ecatRxDiagCurrentConsecutiveTimeout == 0)
                {
                    realFfV0State = 1;
                    realFfV0ArmGood++;

                    if (realFfV0ArmGood >= REAL_FF_V0_ARM_WINDOWS)
                        realFfV0State = 2;
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
                        realFfV0AppliedPpb = QPC_SCHEDULER_ASSUMED_DRIFT_PPB;
                        realFfV0DesiredPpb = QPC_SCHEDULER_ASSUMED_DRIFT_PPB;
                        realFfV0TripMask |= 0x01;
                        realFfV0TripCount++;
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

                if (realPhaseGood)
                {
                    realFfV0HoldGood++;
                    realFfV0HoldBad = 0;

                    if (realFfV0HoldGood >= REAL_FF_V0_HOLD_RECOVERY_WINDOWS)
                    {
                        realFfV0State = 2;
                        realFfV0HoldGood = 0;
                        realFfV0DesiredPpb = realFfV0AppliedPpb;
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
                        realFfV0AppliedPpb = QPC_SCHEDULER_ASSUMED_DRIFT_PPB;
                        realFfV0DesiredPpb = QPC_SCHEDULER_ASSUMED_DRIFT_PPB;
                        realFfV0TripMask |= 0x01;
                        realFfV0TripCount++;
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
        // With frozen -8300 ppb:
        //     749993.775 counts
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
                -8300LL;

            qpcPhaseFfV2AppliedPpb =
                -8300LL;

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
            // The current -8300 ppb target is intentionally untouched.
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
                g_qpcRealFfV0TripMask = realFfV0TripMask;
                g_qpcRealFfV0TripCount = (LONG)realFfV0TripCount;
                g_qpcRealFfV0RecommendedPpb = realPhaseRecommended;
                g_qpcRealFfV0DesiredPpb = (LONGLONG)realFfV0DesiredPpb;
                g_qpcRealFfV0AppliedPpb = (LONGLONG)realFfV0AppliedPpb;
                g_qpcRealFfV0LastStepPpb = (LONGLONG)realFfV0LastStepPpb;
                g_qpcRealFfV0TargetVsFixedNs = (LONGLONG)realVsFixedNs;
                g_qpcRealFfV0DcErrEstNs =
                    (LONGLONG)(g_qpcLiveFfDcPhaseFixedUnwrappedErrorNs +
                        realVsFixedNs);
                g_qpcRealFfV0PhaseSeq = realPhaseSeq1;
                g_qpcRealFfV0PhaseRejectMask = realFfV0PhaseRejectMask;
                g_qpcRealFfV0LastPhaseRejectMask = realFfV0LastPhaseRejectMask;
                g_qpcRealFfV0HoldGood = (LONG)realFfV0HoldGood;
                g_qpcRealFfV0HoldBad = (LONG)realFfV0HoldBad;
                g_qpcRealFfV0HoldEntries = (LONG)realFfV0HoldEntries;
                g_qpcRealFfV0ClampActive = realFfV0ClampActive ? 1L : 0L;
                MemoryBarrier();
                InterlockedIncrement(&g_qpcRealFfV0Seq);

                int64_t pBaseErrNs =
                    (int64_t)g_qpcLiveFfDcPhaseFixedUnwrappedErrorNs +
                    realVsFixedNs;

                int64_t pActualErrNs =
                    pBaseErrNs + phasePActV0OffsetNs;

                int64_t pWrappedErrNs =
                    pActualErrNs % PHASE_P_ACT_CYCLE_NS;

                if (pWrappedErrNs > PHASE_P_ACT_CYCLE_NS / 2LL)
                    pWrappedErrNs -= PHASE_P_ACT_CYCLE_NS;

                if (pWrappedErrNs < -PHASE_P_ACT_CYCLE_NS / 2LL)
                    pWrappedErrNs += PHASE_P_ACT_CYCLE_NS;

                bool pGateGood =
                    realFfV0State == 2 &&
                    realPhaseGood &&
                    realFfV0TripMask == 0 &&
                    qpcSchedulerInitialized;

                // ---------------------------------------------------------
                // Phase-P 正式控制器
                //
                // BaseErr 是 FF timeline 對 S4 DC 的未包絡誤差；ActualErr 再加上
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
                        phasePActV0ArmGood++;

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
                        phasePActV0HoldGood++;

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

                int64_t pRawCorrectionNs = 0;
                int64_t pCommandNs = 0;
                int64_t pStepNs = 0;
                bool pCommandSat = false;
                bool pOffsetSat = false;

                int64_t pAbsErrNs =
                    pWrappedErrNs >= 0
                    ? pWrappedErrNs : -pWrappedErrNs;

                if (phasePActV0State == 2 &&
                    pGateGood &&
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


        bool realFfCycleSafe =
            rearmOk &&
            !usedBootstrap &&
            !runtimeRecoveryThisCycle &&
            g_ecatRxDiagCurrentConsecutiveTimeout == 0;

        if ((realFfV0State == 2 || realFfV0State == 3) &&
            !realFfCycleSafe)
        {
            LONG mask = 0;
            if (g_ecatRxDiagCurrentConsecutiveTimeout != 0) mask |= 0x02;
            if (usedBootstrap) mask |= 0x04;
            if (!rearmOk) mask |= 0x08;
            if (runtimeRecoveryThisCycle) mask |= 0x10;

            realFfV0State = 4;
            realFfV0AppliedPpb = QPC_SCHEDULER_ASSUMED_DRIFT_PPB;
            realFfV0DesiredPpb = QPC_SCHEDULER_ASSUMED_DRIFT_PPB;
            realFfV0LastStepPpb = 0;
            realFfV0TripMask |= mask;
            realFfV0TripCount++;
        }

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


                RtPrintf(
                    "[DC-HAL-BURST-START] "
                    "Current:%lu | "
                    "Base:%lu | "
                    "Command:%lld ppb\n",

                    (unsigned long)
                    currentCounts,

                    (unsigned long)
                    baseCounts,

                    (long long)
                    pMaster->
                    m_dcHalFrequencyCommandPpb);
            }
            else
            {
                RtPrintf(
                    "[DC-HAL-BURST-FAULT] "
                    "GetHalTimerPeriodCounts FAILED | "
                    "Error:%lu\n",

                    (unsigned long)
                    GetLastError());


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
                    RtPrintf(
                        "[DC-HAL-BURST-FAULT] "
                        "AltCycles invalid:%u\n",

                        (unsigned int)
                        burstAltCycles);


                    halActuatorFault =
                        true;
                }
                else
                {
                    RtPrintf(
                        "[DC-HAL-BURST-PLAN] "
                        "AltCycles:%u | "
                        "BaseCycles:%u | "
                        "Remainder:%llu | "
                        "Step:%llu\n",

                        (unsigned int)
                        burstAltCycles,

                        (unsigned int)
                        (
                            4000U -
                            burstAltCycles
                            ),

                        (unsigned long long)
                        burstFractionRemainder,

                        (unsigned long long)
                        pMaster->
                        m_dcHalDitherStep);
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


                        RtPrintf(
                            "[DC-HAL-BURST-FAULT] "
                            "Set:%lu FAILED | "
                            "Error:%lu\n",

                            (unsigned long)
                            desiredCounts,

                            (unsigned long)
                            errorCode);


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


                            RtPrintf(
                                "[DC-HAL-BURST-FAULT] "
                                "Base restored:%u\n",

                                (unsigned int)
                                pMaster->
                                m_dcHalBaseCounts);
                        }
                        else
                        {
                            RtPrintf(
                                "[DC-HAL-BURST-FAULT] "
                                "BASE RESTORE FAILED | "
                                "Error:%lu\n",

                                (unsigned long)
                                GetLastError());
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


                    RtPrintf(
                        "[DC-HAL-BURST] "
                        "BaseCycles:%llu | "
                        "AltCycles:%llu | "
                        "Duty:%llu.%03llu %% | "
                        "Applied:%lu | "
                        "SetOK:%llu | "
                        "SetFail:%llu | "
                        "Remainder:%llu\n",

                        (unsigned long long)
                        burstBaseCycleCount,

                        (unsigned long long)
                        burstAlternateCycleCount,

                        (unsigned long long)
                        (
                            altDutyPercentX1000 /
                            1000ULL
                            ),

                        (unsigned long long)
                        (
                            altDutyPercentX1000 %
                            1000ULL
                            ),

                        (unsigned long)
                        lastAppliedHalCounts,

                        (unsigned long long)
                        halSetSuccessCount,

                        (unsigned long long)
                        halSetFailureCount,

                        (unsigned long long)
                        burstFractionRemainder);


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


                RtPrintf(
                    "[DC-HAL-BURST] "
                    "Actuator disabled -> "
                    "Base restored:%u\n",

                    (unsigned int)
                    pMaster->
                    m_dcHalBaseCounts);
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

    // 上一週期有效才把新的 PLC output 刷入實體 IO Map；若通訊已失效，保留
    // 最後送出資料並等待安全處置，避免錯誤期間繼續注入變化的命令。
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
// QPC <-> S4 DC estimator 的 call 前時間戳；call 後再取一次，兩者中點
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
    if (Motor_Start_Index >= 0)
    {
        wkc =
            pMaster->ecx_LRW_FRMW(
                0x00000000,
                (uint16_t)
                pMaster->m_IoMapSize,
                pMaster->m_IoMap,
                m_slaveInfo[
                    Motor_Start_Index
                ].configAddr,
                &pMaster->
                        DC_reference_time,
                        &dcWkc,
                        50);
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

    const bool pdoWkcValid =
        wkc == pMaster->EXPECTED_WKC_PDO;

    const bool dcWkcValid =
        Motor_Start_Index < 0 ||
        dcWkc > 0;

    const bool pdoCycleValid =
        pdoWkcValid && dcWkcValid;

    // 只有 PDO WKC 與 DC WKC 同時有效才允許採用 Input 與更新控制器。
    // 無 DC reference slave 時 dcWkcValid 固定為 true，系統退化為普通 LRW 模式。
    if (pdoCycleValid)
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
// QPC <-> S4 DC Frequency Estimator V1（約一秒視窗）
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
    // Frequency FF V2 - S4 DC phase 比較內部狀態（shadow）
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


    // Actual S4 DC elapsed span for each 4000-sample phase window.
    static uint64_t
        qpcLiveFfDcPhaseWindowStartDcNs =
        0;

    static uint64_t
        qpcLiveFfDcPhaseWindowEndDcNs =
        0;


    // =============================================================
    // S4 DC Phase Residual Drift Observer V1（僅診斷）
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
            // S4 DC Phase Residual Drift Observer V1A（僅診斷）
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
                            // The current -8300 ppb scheduler value is used ONLY as the
                            // bootstrap safety reference. Trusted Drift still has no control
                            // authority in this revision.
                            // =============================================================

                            static int64_t
                                qpcDcTrustedDriftPpb =
                                -8300LL;

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
                                -8300LL;


                            const int64_t
                                QPC_DC_TRUSTED_BOOTSTRAP_PPB =
                                -8300LL;

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


                            bool qpcDcSampleValid =
                                false;


                            uint64_t qpcDcRttNs =
                                0;

                            uint64_t qpcDcMidCount =
                                0;


                            // =============================================================
                            // Validate sample
                            // =============================================================

                            if (pdoCycleValid &&
                                Motor_Start_Index >= 0 &&
                                qpcDcBeforeValid &&
                                qpcDcAfterValid &&
                                qpcFrequency > 0 &&
                                dcWkc > 0 &&
                                pMaster->DC_reference_time > 0)
                            {
                                uint64_t qpcDcRttCounts =
                                    (uint64_t)
                                    (
                                        qpcDcAfter.QuadPart -
                                        qpcDcBefore.QuadPart
                                        );


                                qpcDcRttNs =
                                    (
                                        qpcDcRttCounts *
                                        1000000000ULL
                                        )
                                    /
                                    qpcFrequency;


                                if (qpcDcRttNs > 0 &&
                                    qpcDcRttNs <=
                                    QPC_DC_MAX_RTT_NS)
                                {
                                    qpcDcMidCount =
                                        (uint64_t)
                                        qpcDcBefore.QuadPart +
                                        (
                                            qpcDcRttCounts /
                                            2ULL
                                            );


                                    qpcDcSampleValid =
                                        true;
                                }
                            }


                            // =============================================================
                            // Accumulate valid sample
                            // =============================================================

                            if (qpcDcSampleValid)
                            {
                                // =========================================================
                                // Trusted Live-FF DC Phase Predictor Dry Run V1
                                //
                                // Map BOTH scheduler targets into the SAME current S4 DC
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
                                        // S4 DC Phase Residual Drift Observer V1
                                        //
                                        // Frequency error is inferred from the slope of the
                                        // FIXED target's unwrapped S4 DC phase.
                                        //
                                        // Use the actual S4 DC elapsed time rather than assuming
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
                                                    -8300LL;


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
                                                // S4 DC Phase Residual Drift Observer V1A
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
                                                // point carries its own absolute S4 DC timestamp.
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
                                                            -8300LL;


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
                                    //     The real scheduler still uses fixed -8300 ppb.
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


                                        // =====================================================
                                        // Trusted Drift V1 - DRY RUN STATE MACHINE
                                        //
                                        // This layer intentionally treats the existing robust
                                        // median as a CANDIDATE, not automatically as truth.
                                        //
                                        // The real scheduler remains fixed at -8300 ppb.
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
                                            // clean windows near the known-safe -8300 baseline.
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
                            // PLC INPUT：EtherCAT IO Map -> Shadow Input
                            //
                            // 只有 pdoCycleValid 才更新 shadow input，避免應用層讀到
                            // timeout／WKC 錯誤週期的半成品。PDO Handler 是 IO Map 唯一 Owner。
                            // =========================================================

                            if (pdoCycleValid)
                            {
                                pMaster->m_Plc.FetchInputs();
                            }


                            // =========================================================
                            // EtherCAT DC Estimator / Controller（目前啟用）
                            // =========================================================

                            if (Motor_Start_Index >= 0)
                            {
                                pMaster->wk_read =
                                    dcWkc;


                                // =========================================================
                                // DC Software Estimator / Phase Controller
                                //
                                // 目前 enableDcSoftwareDiagnostics=true，因此有效 PDO
                                // 週期會更新 Master/DC estimator 與 PDO phase controller。
                                // 兩個函式必須維持無高頻 RtPrintf 的即時安全版本。
                                // 即使日後關閉此 gate，LRW+FRMW 與從站 DC/Sync0 仍會運作；
                                // 關閉的只會是主站軟體估測與 phase controller 更新。
                                // =========================================================

                                const bool enableDcSoftwareDiagnostics =
                                    true;


                                if (pdoCycleValid &&
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


                                    RtPrintf(
                                        "[DC] "
                                        "Servo:%d Slave:%d "
                                        "Diff:%d ns "
                                        "Delay:%u ns "
                                        "DelayWKC:%d "
                                        "Raw:0x%08X\n",

                                        (int)
                                        dcDiagServoIndex,

                                        slaveIndex,

                                        dcDifferenceNs,

                                        (unsigned int)
                                        dcPropagationDelay,

                                        delayWkc,

                                        rawDcDifference);
                                }
                                else
                                {
                                    RtPrintf(
                                        "[DC] "
                                        "Servo:%d Slave:%d "
                                        "Read 0x092C FAILED\n",

                                        (int)
                                        dcDiagServoIndex,

                                        slaveIndex);
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


                            if (pdoCycleValid &&
                                subTick == 2)
                            {
                                if (pMaster->
                                    m_asyncCmd.status ==
                                    (int)
                                    EcatCmdStatus::
                                    ECAT_STATUS_PENDING)
                                {
                                    int cmdWKC =
                                        0;


                                    switch (
                                        pMaster->
                                        m_asyncCmd.type)
                                    {
                                        case (int)
                                            EcatCmdType::
                                        CMD_SET_STATE:
                                        {
                                            state =
                                                (uint16_t)
                                                pMaster->
                                                m_asyncCmd.
                                                dataValue;


                                            cmdWKC =
                                                pMaster->ecx_BWR(
                                                    0x0000,
                                                    0x0120,
                                                    2,
                                                    &state,
                                                    20);


                                            // DEBUG_PRINT("COMCOM\n");


                                            break;
                                        }


                                        case (int)
                                            EcatCmdType::
                                        CMD_SDO_WRITE:
                                        {
                                            cmdWKC =
                                                pMaster->
                                                ecx_SDOwrite(
                                                    pMaster->
                                                    m_asyncCmd.
                                                    slaveAddr,

                                                    pMaster->
                                                    m_asyncCmd.
                                                    index,

                                                    pMaster->
                                                    m_asyncCmd.
                                                    subIndex,

                                                    FALSE,

                                                    pMaster->
                                                    m_asyncCmd.
                                                    dataSize,

                                                    &pMaster->
                                                    m_asyncCmd.
                                                    dataValue,

                                                    200);


                                            break;
                                        }


                                        default:
                                        {
                                            cmdWKC =
                                                0;


                                            break;
                                        }
                                    }


                                    pMaster->
                                        m_asyncCmd.resultWKC =
                                        cmdWKC;


                                    pMaster->
                                        m_asyncCmd.status =
                                        (int)
                                        EcatCmdStatus::
                                        ECAT_STATUS_DONE;
                                }
                            }


                            // =========================================================
                            // PDO 通訊結果分類：wkc<0 算 timeout；frame 有回覆但 PDO/DC
                            // WKC 不符合則算 wkc_error，兩者不可混為同一種故障。
                            // =========================================================

                            if (wkc < 0)
                            {
                                pMaster->
                                    timeout_count_PDO++;
                            }
                            else if (!pdoWkcValid ||
                                !dcWkcValid)
                            {
                                pMaster->
                                    wkc_error_count_PDO++;
                            }


                            // =========================================================
                            // PDO Tick：不論本週期有效與否都遞增，供 slot 與統計視窗使用。
                            // =========================================================

                            pMaster->tickCount_PDO++;


                            // =========================================================
                            // Motion
                            //
                            // 有效週期才更新插補與軸控制；第一個無效週期先保留狀態，
                            // 連續第二個無效週期起執行 EmergencyStopAllAxes，再發布輸出。
                            // =========================================================

                            uint64_t stageMotionStartNs =
                                pMaster->
                                GetCurrentMasterTimeNs();


                            if (pdoCycleValid)
                            {
                                pMaster->
                                    m_Motion.
                                    UpdateInterpolation();


                                pMaster->
                                    m_Motion.
                                    UpdateAllMotion();
                            }
                            else if (
                                pdoConsecutiveInvalidCycles >=
                                2U)
                            {
                                pMaster->
                                    m_Motion.
                                    EmergencyStopAllAxes();


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
