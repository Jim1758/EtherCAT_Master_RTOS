#pragma once
#include <windows.h>

/*
 * 檔案：EtherCatMaster_DC_Internal.h
 * 版本：EtherCAT DC Release Candidate RC1.7
 *
 * 此檔案是 EtherCAT/DC 各模組之間的內部診斷快照介面。
 * 它只宣告 extern 變數，不建立控制物件，也不執行控制演算法。
 *
 * 命名規則：
 * - *Sequence：seqlock 序號；odd 表示 writer 正在更新，even 表示完成。
 * - *AvgNs/*MinNs/*MaxNs：一個統計視窗內的奈秒數值。
 * - *Total*：程序啟動後累積值，不會因每秒 publish 歸零。
 * - 沒有 Total 的 Count：通常是最近一個 4000-cycle 視窗的值。
 *
 * 執行緒規則：
 * - Priority 64 是主要 writer，只負責計數與快照發布。
 * - Priority 50 是 reader，必須使用 Sequence + MemoryBarrier() 讀取。
 * - Reader 不得透過這些變數反向修改 PDO、DC、Motion 或 NC 控制。
 *
 * 維護規則：
 * - 新增 snapshot 欄位時，definition、extern、writer publish 與 reader copy
 *   必須一起更新。
 * - 64-bit 欄位必須放在 seqlock 保護範圍內讀寫。
 */

 // =============================================================
 // 啟動 Drift 設定與自動校正快照
 //
 // ConfigReady/ConfiguredMode/ConfiguredFixedPpb 由 Startup 在正式 PDO timer
 // 啟動前寫入。其餘欄位由 Priority 64 Runtime 發布，Priority 50 只讀取。
 // Mode：0=AUTO、1=FIXED。State：0=WARMUP、1=LOCKED、2=FIXED。
 // =============================================================

extern volatile LONG
g_dcDriftConfigReady;

extern volatile LONG
g_dcDriftConfiguredMode;

extern volatile LONGLONG
g_dcDriftConfiguredFixedPpb;

extern volatile LONG
g_dcDriftCalibrationDiagSequence;

extern volatile LONG
g_dcDriftCalibrationMode;

extern volatile LONG
g_dcDriftCalibrationState;

extern volatile LONG
g_dcDriftCalibrationCandidateGood;

extern volatile LONG
g_dcDriftCalibrationGoodWindows;

extern volatile LONG
g_dcDriftCalibrationRequiredWindows;

extern volatile LONG
g_dcDriftCalibrationRobustSequence;

extern volatile LONGLONG
g_dcDriftCalibrationRawPpb;

extern volatile LONGLONG
g_dcDriftCalibrationMedianPpb;

extern volatile LONGLONG
g_dcDriftCalibrationMadPpb;

extern volatile LONGLONG
g_dcDriftCalibrationRawMedianDeviationPpb;

extern volatile LONGLONG
g_dcDriftCalibrationBaselinePpb;

extern volatile LONG
g_dcDriftCalibrationLockCount;

// =============================================================
// PDO RT Diagnostic Snapshot
//
// 實體變數定義在 EtherCatMaster.cpp
// 這裡 Main Thread 只讀取。
// =============================================================
// =============================================================
// DC PLL Diagnostic Snapshot
//
// Definition:
//     UpdateDCPdoPhaseController() 所在 .cpp
//
// Reader:
//     System_EDM_SINKER_MODE.cpp
// =============================================================

extern volatile LONG
g_dcPllDiagSequence;

extern volatile LONGLONG
g_dcPllDiagEstimatorSequence;

extern volatile LONGLONG
g_dcPllDiagEstimatorOffsetNs;

extern volatile LONGLONG
g_dcPllDiagDriftPpb;

extern volatile LONGLONG
g_dcPllDiagPdoPhaseNs;

extern volatile LONGLONG
g_dcPllDiagTargetPhaseNs;

extern volatile LONGLONG
g_dcPllDiagPhaseStepNs;

extern volatile LONGLONG
g_dcPllDiagWrappedErrorNs;

extern volatile LONGLONG
g_dcPllDiagUnwrappedErrorNs;

extern volatile LONGLONG
g_dcPllDiagSync0MarginNs;

extern volatile LONG
g_dcPllDiagTargetCaptured;

extern volatile LONG
g_dcPllDiagStableWindows;
extern volatile LONGLONG
g_dcPllDiagPCommandNs;

extern volatile LONGLONG
g_dcPllDiagPeriodCorrectionPs;
extern volatile LONG
g_pdoRtDiagSequence;


// PDO Timer
extern volatile LONGLONG
g_pdoRtTimerAvgNs;

extern volatile LONGLONG
g_pdoRtTimerMinNs;

extern volatile LONGLONG
g_pdoRtTimerMaxNs;

extern volatile LONG
g_pdoRtTimerShortCount;

extern volatile LONG
g_pdoRtTimerNormalCount;

extern volatile LONG
g_pdoRtTimerLongCount;


// EtherCAT Combined
extern volatile LONGLONG
g_pdoRtCombinedAvgNs;

extern volatile LONGLONG
g_pdoRtCombinedMinNs;

extern volatile LONGLONG
g_pdoRtCombinedMaxNs;


// PDO Execution
extern volatile LONGLONG
g_pdoRtExecAvgNs;

extern volatile LONGLONG
g_pdoRtExecMinNs;

extern volatile LONGLONG
g_pdoRtExecMaxNs;

extern volatile LONG
g_pdoRtExecOver250Count;

extern volatile LONG
g_pdoRtExecOver300Count;

extern volatile LONG
g_pdoRtExecOver400Count;


// WKC
extern volatile LONG
g_pdoRtLrwWkc;

extern volatile LONG
g_pdoRtDcWkc;

// =============================================================
// PDO Fine Scheduler Timing Diagnostic Snapshot
// =============================================================

extern volatile LONG
g_pdoFineDiagSequence;

extern volatile LONGLONG
g_pdoFineQpcFrequency;

extern volatile LONGLONG
g_pdoFineIntervalAvgNs;

extern volatile LONGLONG
g_pdoFineIntervalMinNs;

extern volatile LONGLONG
g_pdoFineIntervalMaxNs;

extern volatile LONGLONG
g_pdoFineWakeToEcatAvgNs;

extern volatile LONGLONG
g_pdoFineWakeToEcatMinNs;

extern volatile LONGLONG
g_pdoFineWakeToEcatMaxNs;

extern volatile LONGLONG
g_pdoFineQpcReadAvgNs;

extern volatile LONGLONG
g_pdoFineQpcReadMaxNs;

extern volatile LONG
g_pdoFineQpcValid;

// =============================================================
// EtherCAT Send Point Diagnostic Snapshot
// =============================================================

extern volatile LONG
g_ecatSendDiagSequence;

extern volatile LONGLONG
g_ecatSendBuildAvgNs;

extern volatile LONGLONG
g_ecatSendBuildMinNs;

extern volatile LONGLONG
g_ecatSendBuildMaxNs;

extern volatile LONGLONG
g_ecatSendCallAvgNs;

extern volatile LONGLONG
g_ecatSendCallMinNs;

extern volatile LONGLONG
g_ecatSendCallMaxNs;

extern volatile LONG
g_ecatSendQpcValid;


// =============================================================
// EtherCAT RX Soft/Hard Deadline Diagnostic Snapshot
//
// Publish 視窗：4000 次 ecx_LRW_FRMW()。
// Soft Deadline：205000 ns；超過 Soft 但仍早於 Hard 的有效封包可接受。
// Hard Deadline：210000 ns；超過 Hard 的封包不再交給 PDO process image。
//
// Calls/FirstRx/EmptyRx/InvalidFrame：最近一個視窗。
// SoftLateAccepted/HardTimeout：最近一個視窗。
// TotalSoftLateAccepted/TotalHardTimeout：程序啟動後永久累積。
// CurrentConsecutiveTimeout：目前連續 Hard Timeout 次數。
// RecoveryAfterTimeout：Timeout 後重新收到有效封包的次數。
// TimeoutPreReceive：呼叫 ReceivePacket() 前已超過 Hard Deadline。
// TimeoutSleep0/1/2：Timeout 發生前完成的 coarse sleep 次數。
// ReceiveCallMaxNs：單次 ReceivePacket() 的最大執行時間。
// TimeoutReceiveCallMaxNs：直接造成 Hard Timeout 的 ReceivePacket() 最大時間。
// =============================================================

extern volatile LONG g_ecatRxDiagSequence;
extern volatile LONG g_ecatRxDiagCalls;
extern volatile LONG g_ecatRxDiagFirstRxSuccess;
extern volatile LONG g_ecatRxDiagEmptyRx;
extern volatile LONG g_ecatRxDiagInvalidFrame;
extern volatile LONG g_ecatRxDiagSoftLateAccepted;
extern volatile LONGLONG g_ecatRxDiagTotalSoftLateAccepted;
extern volatile LONGLONG g_ecatRxDiagSoftLateElapsedMaxNs;
extern volatile LONG g_ecatRxDiagHardTimeout;
extern volatile LONG g_ecatRxDiagPostReceiveLate;
extern volatile LONG g_ecatRxDiagCurrentConsecutiveTimeout;
extern volatile LONG g_ecatRxDiagMaxConsecutiveTimeout;
extern volatile LONGLONG g_ecatRxDiagTotalHardTimeout;
extern volatile LONG g_ecatRxDiagRecoveryAfterTimeout;
extern volatile LONG g_ecatRxDiagQpcFail;
extern volatile LONG g_ecatRxDiagSleepCount;
extern volatile LONG g_ecatRxDiagElapsedValid;
extern volatile LONGLONG g_ecatRxDiagElapsedAvgNs;
extern volatile LONGLONG g_ecatRxDiagElapsedMaxNs;
extern volatile LONG g_ecatRxDiagTimeoutPreReceive;
extern volatile LONG g_ecatRxDiagTimeoutSleep0;
extern volatile LONG g_ecatRxDiagTimeoutSleep1;
extern volatile LONG g_ecatRxDiagTimeoutSleep2;
extern volatile LONG g_ecatRxDiagTimeoutAttemptAvg;
extern volatile LONG g_ecatRxDiagTimeoutAttemptMax;
extern volatile LONGLONG g_ecatRxDiagReceiveCallMaxNs;
extern volatile LONGLONG g_ecatRxDiagTimeoutReceiveCallMaxNs;
extern volatile LONG g_ecatRxDiagSoftDeadlineNs;
extern volatile LONG g_ecatRxDiagHardDeadlineNs;


// =============================================================
// QPC <-> EtherCAT S4 DC Estimator Snapshot
// =============================================================

extern volatile LONG
g_qpcDcDiagSequence;

extern volatile LONGLONG
g_qpcDcQpcElapsedNs;

extern volatile LONGLONG
g_qpcDcDcElapsedNs;

extern volatile LONGLONG
g_qpcDcDeltaNs;

extern volatile LONGLONG
g_qpcDcDriftPpb;

extern volatile LONGLONG
g_qpcDcRttAvgNs;

extern volatile LONGLONG
g_qpcDcRttMinNs;

extern volatile LONGLONG
g_qpcDcRttMaxNs;

extern volatile LONG
g_qpcDcValidSamples;

extern volatile LONG
g_qpcDcRejectedSamples;

extern volatile LONG
g_qpcDcValid;


// =============================================================
// QPC <-> S4 Robust Drift Dry-Run V1 Snapshot
// =============================================================

extern volatile LONG
g_qpcDcRobustDiagSequence;

extern volatile LONGLONG
g_qpcDcRobustRawDriftPpb;

extern volatile LONGLONG
g_qpcDcRobustMedianDriftPpb;

extern volatile LONGLONG
g_qpcDcRobustMadPpb;

extern volatile LONGLONG
g_qpcDcRobustPeriodFfPs;

extern volatile LONG
g_qpcDcRobustBufferCount;

extern volatile LONG
g_qpcDcRobustCurrentAccepted;

extern volatile LONG
g_qpcDcRobustLocked;

extern volatile LONG
g_qpcDcRobustAcceptedTotal;

extern volatile LONG
g_qpcDcRobustRejectedTotal;


// =============================================================
// QPC <-> S4 Trusted Drift V1A Dry-Run Snapshot
// =============================================================

extern volatile LONG
g_qpcDcTrustedDiagSequence;

extern volatile LONGLONG
g_qpcDcTrustedRawDriftPpb;

extern volatile LONGLONG
g_qpcDcTrustedRobustMedianPpb;

extern volatile LONGLONG
g_qpcDcTrustedRobustMadPpb;

extern volatile LONGLONG
g_qpcDcTrustedDriftPpb;

extern volatile LONGLONG
g_qpcDcTrustedCandidateDeviationPpb;

extern volatile LONGLONG
g_qpcDcTrustedRawMedianDeviationPpb;

extern volatile LONGLONG
g_qpcDcTrustedSlewAppliedPpb;

extern volatile LONGLONG
g_qpcDcTrustedPeriodFfPs;

extern volatile LONG
g_qpcDcTrustedCandidateGood;

extern volatile LONG
g_qpcDcTrustedValid;

extern volatile LONG
g_qpcDcTrustedState;

extern volatile LONG
g_qpcDcTrustedWarmupGoodCount;

extern volatile LONG
g_qpcDcTrustedBadCount;

extern volatile LONG
g_qpcDcTrustedRecoveryGoodCount;

extern volatile LONG
g_qpcDcTrustedUpdateTotal;

extern volatile LONG
g_qpcDcTrustedHoldTotal;

extern volatile LONG
g_qpcDcTrustedUnlockTotal;

extern volatile LONG
g_qpcDcTrustedRelockTotal;


// =============================================================
// QPC <-> S4 Trusted Drift V1A
// Reject / Transition Reason Diagnostic Snapshot
// =============================================================

extern volatile LONG
g_qpcDcTrustedReasonDiagSequence;

extern volatile LONG
g_qpcDcTrustedRejectRobustAcceptTotal;

extern volatile LONG
g_qpcDcTrustedRejectRobustLockTotal;

extern volatile LONG
g_qpcDcTrustedRejectBufferNotFullTotal;

extern volatile LONG
g_qpcDcTrustedRejectMadTotal;

extern volatile LONG
g_qpcDcTrustedRejectCandidateDeviationTotal;

extern volatile LONG
g_qpcDcTrustedRejectRawMedianDeviationTotal;

extern volatile LONG
g_qpcDcTrustedCurrentRejectMask;

extern volatile LONG
g_qpcDcTrustedLastRejectMask;

extern volatile LONG
g_qpcDcTrustedMaxBadStreak;

extern volatile LONG
g_qpcDcTrustedWarmupToTrackTotal;

extern volatile LONG
g_qpcDcTrustedTrackToHoldTotal;

extern volatile LONG
g_qpcDcTrustedHoldToTrackTotal;

extern volatile LONG
g_qpcDcTrustedHoldToUntrustedTotal;

extern volatile LONG
g_qpcDcTrustedUntrustedToTrackTotal;

extern volatile LONG
g_qpcDcTrustedLastTransitionFrom;

extern volatile LONG
g_qpcDcTrustedLastTransitionTo;

extern volatile LONGLONG
g_qpcDcTrustedLastTransitionRawPpb;

extern volatile LONGLONG
g_qpcDcTrustedLastTransitionMedianPpb;

extern volatile LONGLONG
g_qpcDcTrustedLastTransitionMadPpb;

extern volatile LONGLONG
g_qpcDcTrustedLastTransitionTrustedPpb;


// =============================================================
// Trusted Drift -> QPC Live Feed-Forward Dry Run V1 Snapshot
// =============================================================

extern volatile LONG
g_qpcLiveFfDiagSequence;

extern volatile LONG
g_qpcLiveFfInitialized;

extern volatile LONG
g_qpcLiveFfMode;

extern volatile LONG
g_qpcLiveFfTrustedSnapshotValid;

extern volatile LONG
g_qpcLiveFfTrustedValid;

extern volatile LONG
g_qpcLiveFfTrustedState;

extern volatile LONGLONG
g_qpcLiveFfTrustedDriftPpb;

extern volatile LONGLONG
g_qpcLiveFfAppliedDriftPpb;

extern volatile LONGLONG
g_qpcLiveFfAppliedPeriodFfPs;

extern volatile LONGLONG
g_qpcLiveFfFixedErrorNs;

extern volatile LONGLONG
g_qpcLiveFfShadowErrorNs;

extern volatile LONGLONG
g_qpcLiveFfShadowVsFixedTargetNs;

extern volatile LONGLONG
g_qpcLiveFfShadowWindowStartErrorNs;

extern volatile LONGLONG
g_qpcLiveFfShadowWindowEndErrorNs;

extern volatile LONGLONG
g_qpcLiveFfShadowWindowDeltaErrorNs;

extern volatile LONGLONG
g_qpcLiveFfShadowWindowMinErrorNs;

extern volatile LONGLONG
g_qpcLiveFfShadowWindowMaxErrorNs;

extern volatile LONGLONG
g_qpcLiveFfTargetDeltaWindowStartNs;

extern volatile LONGLONG
g_qpcLiveFfTargetDeltaWindowEndNs;

extern volatile LONGLONG
g_qpcLiveFfTargetDeltaWindowDeltaNs;

extern volatile LONG
g_qpcLiveFfTrustedCyclesWindow;

extern volatile LONG
g_qpcLiveFfFallbackCyclesWindow;

extern volatile LONG
g_qpcLiveFfModeSwitchesWindow;

extern volatile LONGLONG
g_qpcLiveFfTotalTrustedCycles;

extern volatile LONGLONG
g_qpcLiveFfTotalFallbackCycles;

extern volatile LONG
g_qpcLiveFfTotalModeSwitches;

extern volatile LONG
g_qpcLiveFfInitCount;

extern volatile LONG
g_qpcLiveFfSamples;


// =============================================================
// Trusted Live-FF DC Phase Predictor Dry Run V1 Snapshot
// =============================================================

extern volatile LONG
g_qpcLiveFfDcPhaseDiagSequence;

extern volatile LONG
g_qpcLiveFfDcPhaseInitialized;

extern volatile LONG
g_qpcLiveFfDcPhaseBoundInitCount;

extern volatile LONGLONG
g_qpcLiveFfDcPhaseBaselinePhaseNs;

extern volatile LONGLONG
g_qpcLiveFfDcPhaseBaselineSync0MarginNs;

extern volatile LONGLONG
g_qpcLiveFfDcPhaseFixedPhaseNs;

extern volatile LONGLONG
g_qpcLiveFfDcPhaseShadowPhaseNs;

extern volatile LONGLONG
g_qpcLiveFfDcPhaseFixedSync0MarginNs;

extern volatile LONGLONG
g_qpcLiveFfDcPhaseShadowSync0MarginNs;

extern volatile LONGLONG
g_qpcLiveFfDcPhaseFixedWrappedErrorNs;

extern volatile LONGLONG
g_qpcLiveFfDcPhaseShadowWrappedErrorNs;

extern volatile LONGLONG
g_qpcLiveFfDcPhaseFixedUnwrappedErrorNs;

extern volatile LONGLONG
g_qpcLiveFfDcPhaseShadowUnwrappedErrorNs;

extern volatile LONGLONG
g_qpcLiveFfDcPhaseShadowVsFixedNs;

extern volatile LONGLONG
g_qpcLiveFfDcPhaseFixedWindowStartNs;

extern volatile LONGLONG
g_qpcLiveFfDcPhaseFixedWindowEndNs;

extern volatile LONGLONG
g_qpcLiveFfDcPhaseFixedWindowDeltaNs;

extern volatile LONGLONG
g_qpcLiveFfDcPhaseFixedWindowMinNs;

extern volatile LONGLONG
g_qpcLiveFfDcPhaseFixedWindowMaxNs;

extern volatile LONGLONG
g_qpcLiveFfDcPhaseShadowWindowStartNs;

extern volatile LONGLONG
g_qpcLiveFfDcPhaseShadowWindowEndNs;

extern volatile LONGLONG
g_qpcLiveFfDcPhaseShadowWindowDeltaNs;

extern volatile LONGLONG
g_qpcLiveFfDcPhaseShadowWindowMinNs;

extern volatile LONGLONG
g_qpcLiveFfDcPhaseShadowWindowMaxNs;

extern volatile LONGLONG
g_qpcLiveFfDcPhaseFixedAbsAvgNs;

extern volatile LONGLONG
g_qpcLiveFfDcPhaseShadowAbsAvgNs;

extern volatile LONG
g_qpcLiveFfDcPhaseShadowBetterCount;

extern volatile LONG
g_qpcLiveFfDcPhaseShadowWorseCount;

extern volatile LONG
g_qpcLiveFfDcPhaseEqualCount;

extern volatile LONGLONG
g_qpcLiveFfDcPhaseMapRttAvgNs;

extern volatile LONGLONG
g_qpcLiveFfDcPhaseMapRttMinNs;

extern volatile LONGLONG
g_qpcLiveFfDcPhaseMapRttMaxNs;

extern volatile LONG
g_qpcLiveFfDcPhaseSamples;


// =============================================================
// S4 DC Phase Residual Drift Observer V1 Dry-Run Snapshot
// =============================================================

extern volatile LONG
g_qpcDcPhaseResidualDiagSequence;

extern volatile LONG
g_qpcDcPhaseResidualInitialized;

extern volatile LONG
g_qpcDcPhaseResidualBoundInitCount;

extern volatile LONGLONG
g_qpcDcPhaseResidualRawPpb;

extern volatile LONGLONG
g_qpcDcPhaseResidualMedianPpb;

extern volatile LONGLONG
g_qpcDcPhaseResidualMadPpb;

extern volatile LONGLONG
g_qpcDcPhaseResidualRecommendedSchedulerPpb;

extern volatile LONGLONG
g_qpcDcPhaseResidualTrustedDriftPpb;

extern volatile LONGLONG
g_qpcDcPhaseResidualTrustedMinusRecommendedPpb;

extern volatile LONGLONG
g_qpcDcPhaseResidualFixedWindowDeltaNs;

extern volatile LONGLONG
g_qpcDcPhaseResidualWindowElapsedNs;

extern volatile LONGLONG
g_qpcDcPhaseResidualWindowRttMaxNs;

extern volatile LONG
g_qpcDcPhaseResidualCurrentAccepted;

extern volatile LONG
g_qpcDcPhaseResidualBufferCount;

extern volatile LONG
g_qpcDcPhaseResidualLocked;

extern volatile LONG
g_qpcDcPhaseResidualAcceptedTotal;

extern volatile LONG
g_qpcDcPhaseResidualRejectedTotal;

extern volatile LONG
g_qpcDcPhaseResidualRejectElapsedTotal;

extern volatile LONG
g_qpcDcPhaseResidualRejectRttTotal;

extern volatile LONG
g_qpcDcPhaseResidualRejectMagnitudeTotal;

extern volatile LONGLONG
g_qpcDcPhaseResidualRingMinPpb;

extern volatile LONGLONG
g_qpcDcPhaseResidualRingMaxPpb;


// =============================================================
// S4 DC Phase Residual Drift Observer V1A Snapshot
// =============================================================

extern volatile LONG
g_qpcDcPhaseResidualV1ADiagSequence;

extern volatile LONG
g_qpcDcPhaseResidualV1AInitialized;

extern volatile LONG
g_qpcDcPhaseResidualV1ABoundInitCount;

extern volatile LONGLONG
g_qpcDcPhaseResidualV1AMeanPhaseNs;

extern volatile LONGLONG
g_qpcDcPhaseResidualV1APhaseSpanNs;

extern volatile LONGLONG
g_qpcDcPhaseResidualV1APointTimeNs;

extern volatile LONG
g_qpcDcPhaseResidualV1APointAccepted;

extern volatile LONG
g_qpcDcPhaseResidualV1AMeanSamples;

extern volatile LONG
g_qpcDcPhaseResidualV1APointBufferCount;

extern volatile LONG
g_qpcDcPhaseResidualV1APairSlopeCount;

extern volatile LONGLONG
g_qpcDcPhaseResidualV1ATheilSenPpb;

extern volatile LONGLONG
g_qpcDcPhaseResidualV1ASlopeMadPpb;

extern volatile LONGLONG
g_qpcDcPhaseResidualV1ATimeSpanNs;

extern volatile LONG
g_qpcDcPhaseResidualV1ALocked;

extern volatile LONGLONG
g_qpcDcPhaseResidualV1ARecommendedSchedulerPpb;

extern volatile LONGLONG
g_qpcDcPhaseResidualV1ATrustedDriftPpb;

extern volatile LONGLONG
g_qpcDcPhaseResidualV1ATrustedMinusRecommendedPpb;

extern volatile LONGLONG
g_qpcDcPhaseResidualV1AWindowElapsedNs;

extern volatile LONGLONG
g_qpcDcPhaseResidualV1AWindowRttMaxNs;

extern volatile LONG
g_qpcDcPhaseResidualV1AAcceptedPointTotal;

extern volatile LONG
g_qpcDcPhaseResidualV1ARejectedPointTotal;

extern volatile LONG
g_qpcDcPhaseResidualV1ARejectSamplesTotal;

extern volatile LONG
g_qpcDcPhaseResidualV1ARejectElapsedTotal;

extern volatile LONG
g_qpcDcPhaseResidualV1ARejectRttTotal;

extern volatile LONG
g_qpcDcPhaseResidualV1ARejectSpanTotal;

extern volatile LONGLONG
g_qpcDcPhaseResidualV1ASlopeMinPpb;

extern volatile LONGLONG
g_qpcDcPhaseResidualV1ASlopeMaxPpb;

extern volatile LONG g_qpcV1aCostSeq;
extern volatile LONGLONG g_qpcV1aCostLastNs;
extern volatile LONGLONG g_qpcV1aCostAvgNs;
extern volatile LONGLONG g_qpcV1aCostMaxNs;
extern volatile LONG g_qpcV1aCostEvents;
extern volatile LONG g_qpcV1aCostOver20us;
extern volatile LONG g_qpcV1aCostOver40us;
extern volatile LONG g_qpcV1aCostOver80us;
extern volatile LONG g_qpcV1aCostQpcFail;


// =============================================================
// Frequency FF Dry Run V2 Snapshot
// =============================================================

extern volatile LONG
g_qpcPhaseFfV2DiagSequence;

extern volatile LONG
g_qpcPhaseFfV2Initialized;

extern volatile LONG
g_qpcPhaseFfV2State;

extern volatile LONG
g_qpcPhaseFfV2CandidateGood;

extern volatile LONG
g_qpcPhaseFfV2WarmupGoodCount;

extern volatile LONG
g_qpcPhaseFfV2BadCount;

extern volatile LONG
g_qpcPhaseFfV2RecoveryGoodCount;

extern volatile LONG
g_qpcPhaseFfV2ObserverSequence;

extern volatile LONGLONG
g_qpcPhaseFfV2RecommendedPpb;

extern volatile LONGLONG
g_qpcPhaseFfV2DesiredPpb;

extern volatile LONGLONG
g_qpcPhaseFfV2AppliedPpb;

extern volatile LONGLONG
g_qpcPhaseFfV2SlewAppliedPpb;

extern volatile LONGLONG
g_qpcPhaseFfV2SlopeMadPpb;

extern volatile LONG
g_qpcPhaseFfV2Points;

extern volatile LONG
g_qpcPhaseFfV2Pairs;

extern volatile LONG
g_qpcPhaseFfV2ObserverLocked;

extern volatile LONG
g_qpcPhaseFfV2PointAccepted;

extern volatile LONGLONG
g_qpcPhaseFfV2TargetVsFixedNs;

extern volatile LONGLONG
g_qpcPhaseFfV2WakeErrorNs;

extern volatile LONGLONG
g_qpcPhaseFfV2TargetDeltaWindowStartNs;

extern volatile LONGLONG
g_qpcPhaseFfV2TargetDeltaWindowEndNs;

extern volatile LONGLONG
g_qpcPhaseFfV2TargetDeltaWindowDeltaNs;

extern volatile LONG
g_qpcPhaseFfV2TrackCyclesWindow;

extern volatile LONG
g_qpcPhaseFfV2HoldCyclesWindow;

extern volatile LONG
g_qpcPhaseFfV2FallbackCyclesWindow;

extern volatile LONG
g_qpcPhaseFfV2StateSwitchesWindow;

extern volatile LONG
g_qpcPhaseFfV2TotalStateSwitches;

extern volatile LONG
g_qpcPhaseFfV2WarmupToTrackTotal;

extern volatile LONG
g_qpcPhaseFfV2TrackToHoldTotal;

extern volatile LONG
g_qpcPhaseFfV2HoldToTrackTotal;

extern volatile LONG
g_qpcPhaseFfV2HoldToFallbackTotal;

extern volatile LONG
g_qpcPhaseFfV2FallbackToTrackTotal;

extern volatile LONG
g_qpcPhaseFfV2Samples;


// =============================================================
// Frequency FF Dry Run V2 - S4 DC Phase Comparison Snapshot
// =============================================================

extern volatile LONG
g_qpcPhaseFfV2DcDiagSequence;

extern volatile LONG
g_qpcPhaseFfV2DcInitialized;

extern volatile LONGLONG
g_qpcPhaseFfV2DcPhaseNs;

extern volatile LONGLONG
g_qpcPhaseFfV2DcSync0MarginNs;

extern volatile LONGLONG
g_qpcPhaseFfV2DcUnwrappedErrorNs;

extern volatile LONGLONG
g_qpcPhaseFfV2DcVsFixedNs;

extern volatile LONGLONG
g_qpcPhaseFfV2DcVsTrustedNs;

extern volatile LONGLONG
g_qpcPhaseFfV2DcWindowStartNs;

extern volatile LONGLONG
g_qpcPhaseFfV2DcWindowEndNs;

extern volatile LONGLONG
g_qpcPhaseFfV2DcWindowDeltaNs;

extern volatile LONGLONG
g_qpcPhaseFfV2DcWindowMinNs;

extern volatile LONGLONG
g_qpcPhaseFfV2DcWindowMaxNs;

extern volatile LONGLONG
g_qpcPhaseFfV2DcAbsAvgNs;

extern volatile LONG
g_qpcPhaseFfV2DcBetterThanFixed;

extern volatile LONG
g_qpcPhaseFfV2DcBetterThanTrusted;

extern volatile LONG
g_qpcPhaseFfV2DcSamples;


// =============================================================
// QPC Coarse Re-Anchor Dry Run V1 Snapshot
// =============================================================

extern volatile LONG g_qpcCoarseReanchorDiagSequence;
extern volatile LONG g_qpcCoarseReanchorInitialized;
extern volatile LONGLONG g_qpcCoarseReanchorRawErrorNs;
extern volatile LONGLONG g_qpcCoarseReanchorVirtualErrorNs;
extern volatile LONGLONG g_qpcCoarseReanchorVirtualMarginNs;
extern volatile LONGLONG g_qpcCoarseReanchorVirtualOffsetNs;
extern volatile LONGLONG g_qpcCoarseReanchorWindowStartErrorNs;
extern volatile LONGLONG g_qpcCoarseReanchorWindowEndErrorNs;
extern volatile LONGLONG g_qpcCoarseReanchorWindowDeltaErrorNs;
extern volatile LONGLONG g_qpcCoarseReanchorWindowMinErrorNs;
extern volatile LONGLONG g_qpcCoarseReanchorWindowMaxErrorNs;
extern volatile LONG g_qpcCoarseReanchorCorrectionEventsWindow;
extern volatile LONG g_qpcCoarseReanchorCorrectionTicksWindow;
extern volatile LONG g_qpcCoarseReanchorMaxTicksPerEventWindow;
extern volatile LONG g_qpcCoarseReanchorMultiTickEventsWindow;
extern volatile LONGLONG g_qpcCoarseReanchorTotalCorrectionEvents;
extern volatile LONGLONG g_qpcCoarseReanchorTotalCorrectionTicks;
extern volatile LONG g_qpcCoarseReanchorLastEventSpacingCycles;
extern volatile LONG g_qpcCoarseReanchorMinEventSpacingCycles;
extern volatile LONG g_qpcCoarseReanchorMaxEventSpacingCycles;
extern volatile LONGLONG g_qpcCoarseReanchorRobustMedianDriftPpb;
extern volatile LONGLONG g_qpcCoarseReanchorRobustMadPpb;
extern volatile LONG g_qpcCoarseReanchorRobustLocked;
extern volatile LONG g_qpcCoarseReanchorWindowSamples;
extern volatile LONG g_qpcCoarseReanchorOverflow;


// =============================================================
// QPC Scheduler Dry Run V1 Snapshot
// =============================================================

extern volatile LONG
g_qpcSchedulerDiagSequence;

extern volatile LONGLONG
g_qpcSchedulerTargetQpc;

extern volatile LONGLONG
g_qpcSchedulerActualQpc;

extern volatile LONGLONG
g_qpcSchedulerErrorCounts;

extern volatile LONGLONG
g_qpcSchedulerErrorNs;

extern volatile LONGLONG
g_qpcSchedulerWindowStartErrorNs;

extern volatile LONGLONG
g_qpcSchedulerWindowEndErrorNs;

extern volatile LONGLONG
g_qpcSchedulerWindowDeltaErrorNs;

extern volatile LONGLONG
g_qpcSchedulerWindowMinErrorNs;

extern volatile LONGLONG
g_qpcSchedulerWindowMaxErrorNs;

extern volatile LONGLONG
g_qpcSchedulerPeriodWholeCounts;

extern volatile LONGLONG
g_qpcSchedulerPeriodFractionScaled;

extern volatile LONGLONG
g_qpcSchedulerAssumedDriftPpb;

extern volatile LONG
g_qpcSchedulerWindowSamples;

extern volatile LONG
g_qpcSchedulerValid;

extern volatile LONG g_qpcRealFfV0Seq;
extern volatile LONG g_qpcRealFfV0State;
extern volatile LONG g_qpcRealFfV0PhaseGood;
extern volatile LONG g_qpcRealFfV0ArmGood;
extern volatile LONG g_qpcRealFfV0TripMask;
extern volatile LONG g_qpcRealFfV0TripCount;
extern volatile LONGLONG g_qpcRealFfV0RecommendedPpb;
extern volatile LONGLONG g_qpcRealFfV0DesiredPpb;
extern volatile LONGLONG g_qpcRealFfV0AppliedPpb;
extern volatile LONGLONG g_qpcRealFfV0LastStepPpb;
extern volatile LONGLONG g_qpcRealFfV0TargetVsFixedNs;
extern volatile LONGLONG g_qpcRealFfV0DcErrEstNs;
extern volatile LONG g_qpcRealFfV0PhaseSeq;
extern volatile LONG g_qpcRealFfV0PhaseRejectMask;
extern volatile LONG g_qpcRealFfV0LastPhaseRejectMask;
extern volatile LONG g_qpcRealFfV0HoldGood;
extern volatile LONG g_qpcRealFfV0HoldBad;
extern volatile LONG g_qpcRealFfV0HoldEntries;
extern volatile LONG g_qpcRealFfV0ClampActive;

extern volatile LONG g_qpcRealFfClampSelfTestSeq;
extern volatile LONG g_qpcRealFfClampSelfTestPass;
extern volatile LONG g_qpcRealFfClampSelfTestCases;
extern volatile LONG g_qpcRealFfClampSelfTestFail;
extern volatile LONGLONG g_qpcRealFfClampSelfTestRec0;
extern volatile LONGLONG g_qpcRealFfClampSelfTestDesired0;
extern volatile LONG g_qpcRealFfClampSelfTestClamp0;
extern volatile LONGLONG g_qpcRealFfClampSelfTestRec1;
extern volatile LONGLONG g_qpcRealFfClampSelfTestDesired1;
extern volatile LONG g_qpcRealFfClampSelfTestClamp1;
extern volatile LONGLONG g_qpcRealFfClampSelfTestRec2;
extern volatile LONGLONG g_qpcRealFfClampSelfTestDesired2;
extern volatile LONG g_qpcRealFfClampSelfTestClamp2;
extern volatile LONGLONG g_qpcRealFfClampSelfTestRec3;
extern volatile LONGLONG g_qpcRealFfClampSelfTestDesired3;
extern volatile LONG g_qpcRealFfClampSelfTestClamp3;
extern volatile LONGLONG g_qpcRealFfClampSelfTestRec4;
extern volatile LONGLONG g_qpcRealFfClampSelfTestDesired4;
extern volatile LONG g_qpcRealFfClampSelfTestClamp4;

extern volatile LONG g_qpcPhasePActV0Seq;
extern volatile LONG g_qpcPhasePActV0State;
extern volatile LONG g_qpcPhasePActV0GateGood;
extern volatile LONG g_qpcPhasePActV0ArmGood;
extern volatile LONGLONG g_qpcPhasePActV0BaseErrNs;
extern volatile LONGLONG g_qpcPhasePActV0ActualErrNs;
extern volatile LONGLONG g_qpcPhasePActV0WrappedErrNs;
extern volatile LONGLONG g_qpcPhasePActV0RawCorrectionNs;
extern volatile LONGLONG g_qpcPhasePActV0CommandNs;
extern volatile LONGLONG g_qpcPhasePActV0StepNs;
extern volatile LONGLONG g_qpcPhasePActV0OffsetNs;
extern volatile LONGLONG g_qpcPhasePActV0PredictedErrNs;
extern volatile LONG g_qpcPhasePActV0CommandSat;
extern volatile LONG g_qpcPhasePActV0OffsetSat;
extern volatile LONG g_qpcPhasePActV0Improve;
extern volatile LONG g_qpcPhasePActV0HoldGood;
extern volatile LONG g_qpcPhasePActV0HoldEntries;
extern volatile LONG g_qpcPhasePActV0TripCount;
extern volatile LONG g_qpcPhasePActV0RealFfState;

// =============================================================
// QPC Scheduler Re-Anchor + Guard V1B
// =============================================================

extern volatile LONG
g_qpcSchedulerState;

extern volatile LONGLONG
g_qpcSchedulerGuardNs;

extern volatile LONGLONG
g_qpcSchedulerRemainingToTargetNs;

extern volatile LONGLONG
g_qpcSchedulerLateByNs;

extern volatile LONGLONG
g_qpcSchedulerAnchorQpc;

extern volatile LONG
g_qpcSchedulerAnchorDcDiagSequence;

extern volatile LONG
g_qpcSchedulerStableWaitCycles;

extern volatile LONG
g_qpcSchedulerEarlyCount;

extern volatile LONG
g_qpcSchedulerNearCount;

extern volatile LONG
g_qpcSchedulerLateCount;

// =============================================================
// PDO One-Shot Bootstrap Fallback Diagnostic V1 Snapshot
// =============================================================

extern volatile LONG g_pdoBootstrapDiagSequence;

extern volatile LONG g_pdoBootstrapWindowBootstrap;
extern volatile LONG g_pdoBootstrapCurrentConsecutive;
extern volatile LONG g_pdoBootstrapMaxConsecutive;
extern volatile LONGLONG g_pdoBootstrapTotal;
extern volatile LONG g_pdoBootstrapRecoveryAfterBootstrap;

extern volatile LONG g_pdoBootstrapReasonNotReady;
extern volatile LONG g_pdoBootstrapReasonFinalUnderGuard;
extern volatile LONG g_pdoBootstrapReasonQpcBeforeArmFail;
extern volatile LONG g_pdoBootstrapReasonLeadTooShort;
extern volatile LONG g_pdoBootstrapReasonOther;

extern volatile LONG g_pdoBootstrapSchedulerInitialized;

extern volatile LONG g_pdoBootstrapPredictedLeadValid;
extern volatile LONGLONG g_pdoBootstrapPredictedLeadAvgNs;
extern volatile LONGLONG g_pdoBootstrapPredictedLeadMinNs;
extern volatile LONGLONG g_pdoBootstrapPredictedLeadMaxNs;

extern volatile LONGLONG g_pdoBootstrapSchedulerErrorNs;
extern volatile LONGLONG g_pdoBootstrapSchedulerLateByNs;

extern volatile LONG g_pdoRuntimeRecoveryWindowEvents;
extern volatile LONG g_pdoRuntimeRecoveryWindowSkippedCycles;
extern volatile LONG g_pdoRuntimeRecoveryMaxSkipCycles;
extern volatile LONGLONG g_pdoRuntimeRecoveryTotalEvents;
extern volatile LONGLONG g_pdoRuntimeRecoveryTotalSkippedCycles;
extern volatile LONGLONG g_pdoRuntimeRecoveryLastLeadBeforeNs;
extern volatile LONGLONG g_pdoRuntimeRecoveryLastLeadAfterNs;

extern volatile LONG g_pdoRecoveryForensicSeq;
extern volatile LONG g_pdoRecoveryForensicCaptured;
extern volatile LONGLONG g_pdoRecoveryForensicWakeIntervalNs;
extern volatile LONGLONG g_pdoRecoveryForensicLeadAtWakeNs;
extern volatile LONGLONG g_pdoRecoveryForensicWakeToArmNs;
extern volatile LONGLONG g_pdoRecoveryForensicLeadAtArmNs;
extern volatile LONGLONG g_pdoRecoveryForensicLeadAfterNs;
extern volatile LONG g_pdoRecoveryForensicSkipCycles;

extern volatile LONG g_pdoRecoverySelfTestSeq;
extern volatile LONG g_pdoRecoverySelfTestPass;
extern volatile LONG g_pdoRecoverySelfTestCases;
extern volatile LONG g_pdoRecoverySelfTestFail;
extern volatile LONGLONG g_pdoRecoverySelfTestBefore0;
extern volatile LONGLONG g_pdoRecoverySelfTestAfter0;
extern volatile LONG g_pdoRecoverySelfTestSkip0;
extern volatile LONGLONG g_pdoRecoverySelfTestBefore1;
extern volatile LONGLONG g_pdoRecoverySelfTestAfter1;
extern volatile LONG g_pdoRecoverySelfTestSkip1;
extern volatile LONGLONG g_pdoRecoverySelfTestBefore2;
extern volatile LONGLONG g_pdoRecoverySelfTestAfter2;
extern volatile LONG g_pdoRecoverySelfTestSkip2;
extern volatile LONGLONG g_pdoRecoverySelfTestBefore3;
extern volatile LONGLONG g_pdoRecoverySelfTestAfter3;
extern volatile LONG g_pdoRecoverySelfTestSkip3;
extern volatile LONG g_pdoRecoverySelfTestShadowPreserve;


// =============================================================
// PDO One-Shot Scheduler V1A Infrastructure Snapshot
//
// Producer:
//     GlobalTimerHandler_PDO() / EtherCatMaster.cpp
//
// Reader:
//     Main Priority 50 here.
//
// V1A is diagnostic only:
//     ControlEnabled  = 0
//     FineWaitEnabled = 0
// =============================================================

extern volatile LONG
g_pdoOneShotInfraSequence;

extern volatile LONG
g_pdoOneShotInfraState;

extern volatile LONG
g_pdoOneShotInfraValid;

extern volatile LONG
g_pdoOneShotInfraControlEnabled;

extern volatile LONG
g_pdoOneShotInfraFineWaitEnabled;

extern volatile LONG
g_pdoOneShotInfraWarmupRequired;

extern volatile LONG
g_pdoOneShotInfraWarmupComplete;

extern volatile LONGLONG
g_pdoOneShotInfraQpcFrequency;

extern volatile LONGLONG
g_pdoOneShotInfraFinalTargetQpc;

extern volatile LONGLONG
g_pdoOneShotInfraCoarseTargetQpc;

extern volatile LONGLONG
g_pdoOneShotInfraActualWakeQpc;

extern volatile LONGLONG
g_pdoOneShotInfraGuardNs;

extern volatile LONGLONG
g_pdoOneShotInfraToCoarseNs;

extern volatile LONGLONG
g_pdoOneShotInfraToFinalNs;

extern volatile LONGLONG
g_pdoOneShotInfraFinalLateByNs;

extern volatile LONG
g_pdoOneShotInfraDcDiagSequence;

extern volatile LONG
g_pdoOneShotInfraWindowSamples;

extern volatile LONG
g_pdoOneShotInfraReadyCount;

extern volatile LONG
g_pdoOneShotInfraCoarseWindowCount;

extern volatile LONG
g_pdoOneShotInfraFinalLateCount;


// =============================================================
// PDO One-Shot Scheduler V1B Live Bridge
// Definition: EtherCatMaster.cpp
// =============================================================

extern HANDLE
g_pdoOneShotTimerHandle;

extern volatile LONG
g_pdoOneShotStartupMode;

extern volatile LONG
g_pdoOneShotWarmupCallbackDone;


extern volatile LONG
g_pdoOneShotInfraRearmOkCount;

extern volatile LONG
g_pdoOneShotInfraRearmFailCount;

extern volatile LONG
g_pdoOneShotInfraBootstrapCount;

extern volatile LONG
g_pdoOneShotInfraActiveCount;

extern volatile LONGLONG
g_pdoOneShotInfraCoarseErrorAvgNs;

extern volatile LONGLONG
g_pdoOneShotInfraCoarseErrorMinNs;

extern volatile LONGLONG
g_pdoOneShotInfraCoarseErrorMaxNs;

extern volatile LONGLONG
g_pdoOneShotInfraFinalMarginAvgNs;

extern volatile LONGLONG
g_pdoOneShotInfraFinalMarginMinNs;

extern volatile LONGLONG
g_pdoOneShotInfraFinalMarginMaxNs;

extern volatile LONGLONG
g_pdoOneShotInfraRearmCostAvgNs;

extern volatile LONGLONG
g_pdoOneShotInfraRearmCostMinNs;

extern volatile LONGLONG
g_pdoOneShotInfraRearmCostMaxNs;
