#pragma once
#include "NC_Types.h"
#include "EDMGapSignal.h"
#include "MotionCore.h"
#include "HomingManager.h"
#include "CoordinateManager.h"
#include "MacroEngine.h"       // 必須要有
#include "MacroParser.h"       // 必須要有
#include "GCodeParser.h"       // 🌟 解決 Parser 找不到的關鍵！
#include "NCProgramCache.h"    // Stage NC-0.2C：Parsed Program Cache
#include "NCPreparedBlockQueueShadow.h" // Stage NC-0.2K.1：Prepared Queue Shadow
#include "NCPreparedHeadEquivalenceShadow.h" // Stage NC-0.2K.2：Prepared Head Exact Equivalence
#include "NCPreparedHeadCutoverGate.h" // Stage NC-0.2K.3.1：Ordinary G00 Cutover
#include "NCPreparedHeadPreResolveAdmissionShadow.h" // Stage NC-0.2K.4：Pre-Resolve Admission Shadow
#include "NCPreparedHeadResolverBypassGate.h" // Stage NC-0.2K.4.2：Ordinary G00 Controlled Resolver Bypass
#include "NCOrdinaryG00BufferedExactStopAdmissionShadow.h" // Stage NC-0.2K.5：Ordinary G00 Buffered Exact-Stop Admission Shadow
#include "NCOrdinaryG00InflightTerminalRegistryShadow.h" // Stage NC-0.2K.6.3：Bounded In-Flight Terminal Registry Shadow
#include "NCOrdinaryG00FeedHoldCohortShadow.h" // Stage NC-0.2K.7.3：Two-Entry Feed Hold / Resume Terminal Cohort Shadow
#include "NCOrdinaryG00FeedHoldCohortCutoverGate.h" // Stage NC-0.2K.7.4：Terminal Cohort Controlled Cutover
#include "NCOrdinaryG00FeedHoldCohortRearmShadow.h" // Stage NC-0.2K.7.5：Repeated Cohort Re-arm Shadow
#include "NCOrdinaryG00FeedHoldCohortRearmCutoverGate.h" // Stage NC-0.2K.7.6：Repeated Cohort Re-arm Controlled Cutover
#include "NCOrdinaryG00FeedHoldRollingRearmShadow.h" // Stage NC-0.2K.7.7：Same-Session Rolling Re-arm Continuity Shadow
#include "NCOrdinaryG00FeedHoldRollingRearmCutoverGate.h" // Stage NC-0.2K.7.8：Rolling Re-arm Continuity Controlled Cutover
#include "NCOrdinaryG00ReadAheadCutoverGate.h" // Stage NC-0.2K.7.1：Two-Entry Ordinary G00 Read-Ahead Cutover
#include "NCBlockLifecycleLedger.h" // Stage NC-0.2D：Block / Motion Lifecycle
#include "NCBlockCompletionBoundary.h" // Stage NC-0.2F：Motion Completion Dual-Key Guard
#include "NCProgramEndBoundary.h" // Stage NC-0.2G：Program End / Cycle End Gate
#include "NCSingleBlockBoundary.h" // Stage NC-0.2I.1：Single Block Shadow Boundary
#include "NCSingleBlockHoldGate.h" // Stage NC-0.2I.4：Single Block Controlled HOLD Cutover
#include "NCFeedHoldBoundary.h" // Stage NC-0.2I.2：Feed Hold Request/Ack Shadow
#include "NCFeedHoldResumeGate.h" // Stage NC-0.2I.3：Feed Hold ACK-Gated Resume Cutover
#include "NCLifecycleInterruptionBoundary.h" // Stage NC-0.2J.1：Failure / Epoch Cancellation Shadow
#include "NCResetReleaseGate.h" // Stage NC-0.2J.3：Reset Stable-Standstill Release Gate
#include "NCAlarmEmergencyStopBoundary.h" // Stage NC-0.2J.6.1：Alarm / E-stop RT ACK Shadow
#include "NCPathCoreInputHandoffCompactShadow.h" // Stage NC-0.2L.2A：Path Core accepted-input contract
#include "NCPathCoreCommittedGeometryShadow.h" // Stage NC-0.2L.2D：committed endpoint-pair contract
#include "NCPathCoreCommittedGeometryLinkShadow.h" // Stage NC-0.2L.2E：immediate committed link relation
#include "NCPathCoreLinkedCommittedSegmentShadow.h" // Stage NC-0.2L.2F：linked committed segment geometry
#include "NCPathCoreLinkedCommittedSegmentPairShadow.h" // Stage NC-0.2L.2G：immediate linked segment-pair continuity
#include "NCPathCoreLinkedCommittedSegmentRunShadow.h" // Stage NC-0.2L.2H：proven linked segment run-length
#include "NCPathCoreLinkedCommittedSegmentRunBoundaryShadow.h" // Stage NC-0.2L.2I：run endpoint boundary
#include "NCPathCoreLinkedCommittedSegmentRunDisplacementShadow.h" // Stage NC-0.2L.2J：run endpoint net displacement
#include "NCPathCoreLinkedCommittedSegmentRunClosureShadow.h" // Stage NC-0.2L.2K：run endpoint closure / returned-axis relation
#include "NCPathCoreLinkedCommittedSegmentRunClosureTransitionShadow.h" // Stage NC-0.2L.2L：run endpoint return transition
#include "NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryShadow.h" // Stage NC-0.2L.2M：run endpoint return transition coverage
#include "NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationShadow.h" // Stage NC-0.2L.2N：return coverage qualification
#include "NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionShadow.h" // Stage NC-0.2L.2O: return coverage qualification transition
#include "NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairShadow.h" // Stage NC-0.2L.2P: qualification transition pair relation
#include "NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityShadow.h" // Stage NC-0.2L.2Q: qualification transition pair continuity
#include "NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunShadow.h" // Stage NC-0.2L.2R: local continuity certificate run length
#include "NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryShadow.h" // Stage NC-0.2L.2S: observed local continuity run boundary
#include "NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionShadow.h" // Stage NC-0.2L.2T: boundary certificate availability transition
#include "NCPathCoreBoundaryAvailabilityTransitionPairShadow.h" // Stage NC-0.2L.2U: adjacent availability transition pair continuity
#include "NCPathCoreBoundaryAvailabilityTransitionPairRunShadow.h" // Stage NC-0.2L.2V: local availability continuity certificate count
#include "NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageShadow.h" // Stage NC-0.2L.2W: observed local run pattern coverage
#include "NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionShadow.h" // Stage NC-0.2L.2X: adjacent run pattern coverage change
#include "NCPathCoreRunCoverageTransitionPairShadow.h" // Stage NC-0.2L.2Y: adjacent pattern discovery transition pair
#include "NCPathCoreRunCoverageTransitionPairRunShadow.h" // Stage NC-0.2L.2Z: local consecutive discovery pair certificates
#include "NCPathCoreRunCoverageBoundaryShadow.h" // Stage NC-0.2L.2AA: observed first-Y coverage boundary
#include "NCPathCoreRunCoverageBoundaryAvailabilityShadow.h" // Stage NC-0.2L.2AB: adjacent boundary availability
#include "NCPathCoreRunCoverageBoundaryAvailabilityPairShadow.h" // Stage NC-0.2L.2AC: adjacent availability transition pair
#include "NCPathCoreRunCoverageBoundaryAvailabilityPairRunShadow.h" // Stage NC-0.2L.2AD: local consecutive availability pair certificates
#include "NCPathCoreCurrentProofScopeShadow.h" // Stage NC-0.2L.2AE: current local proof-scope coherence audit
#include "NCPathCoreProofFrontierShadow.h" // Stage NC-0.2L.2AF: coherent current proof-frontier certificate
#include "NCPathCoreFrontierTicketContract.h" // Stage NC-0.2L.2AG: contract-only newest-owner revalidation; no runtime consumer
#include "NCPathCoreFrontierReadContract.h" // Stage NC-0.2L.2AH: ticket-bound scalar value read; no runtime consumer
#include "NCPathCoreFrontierReadProbe.h" // Stage NC-0.2L.2AI: first diagnostic-only AG/AH runtime probe
#include "NCPathCoreSampleReadContract.h" // Stage NC-0.2L.2AJ: revalidated last-sample read boundary
#include "NCPathCoreReadbackCheckContract.h" // Stage NC-0.2L.2AL: exact AI/result/copy-out consistency
#include "NCPathCoreCommandedSegmentCheck.h" // NC-0.2L.2AU: current D/E/F source revalidation
#include "NCPathCoreCommandedChordEval.h" // NC-0.2L.2AW: commanded chord, not actual motion
#include "NCPathCoreCommandedChordLocate.h" // NC-0.2L.2AX: inverse single-axis chord query
#include "NCPathCoreCommandedChordSegment.h" // NC-0.2L.2AY: explicit single commanded chord value
#include "NCPathCoreLiveRetention.h" // BN: finite live commanded retention
// BP-BEGIN
#include "NCPathCoreCompletedSnapshot.h" // BP: detached completed retained set
// BQ-BEGIN
#include "NCPathCoreCommittedRun.h"
#include "MotionFeedLineReceipt.h"
#include "NCPathCoreRetainedPath.h"
#include "MotionPathCoreRetainedReceipt.h"
// BQ-END

// BP-END
#include "NCPathCoreExecutionLink.h" // BO: retained segment / Motion feedback association

#include <queue>
#include <vector>
#include <string>
#include <map>                 // Parsed Macro Cache 使用穩定節點位址
#include <stack>               // 🌟 新增：為了支援副程式返回堆疊
#include <cstdint>
#include <atomic>
#include <type_traits>
class NCManager;

// 🌟 終極解法：定義一個「檢查條件」的函數指標。
// 回傳 true 代表條件滿足 (等待結束)，false 代表繼續等
using WaitConditionFunc = bool (*)(NCManager* nc);

// =============================================================================
// Stage NC-0.2H - G/M Same-Block Transaction Barrier
//
// A single NC block may contain one G action and one M action.  Their wait
// callbacks must never overwrite each other.  The transaction waits for both
// sub-actions, then applies the deferred control action (M00/M01/M98/M99/M02/
// M30) only after the existing NC-0.2F Motion Completion Guard has released the
// block.  Transaction bookkeeping is fixed-size; existing Macro file loading
// remains on the non-servo 10 ms supervisory path.
// =============================================================================
enum class NCGMBlockPostAction : std::uint8_t
{
    NONE = 0,
    PROGRAM_STOP_M00,
    OPTIONAL_STOP_M01,
    CALL_M98,
    RETURN_M99,
    PROGRAM_END_M02,
    PROGRAM_END_M30
};

enum class NCGMBlockTransactionPhase : std::uint8_t
{
    IDLE = 0,
    WAITING,
    READY_TO_FINALIZE,
    FINALIZED,
    CANCELLED,
    FAILED
};

struct NCGMBlockTransactionSnapshot
{
    std::uint64_t sequence = 0ULL;
    NCBlockDispatchId dispatchId = NC_BLOCK_DISPATCH_ID_INVALID;
    NCGMBlockTransactionPhase phase = NCGMBlockTransactionPhase::IDLE;
    NCGMBlockPostAction postAction = NCGMBlockPostAction::NONE;

    int sourcePC = -1;
    int sourceLineNumber = 0;
    int mCode = -1;
    int pValue = 0;
    int repeatCount = 1;

    bool active = false;
    bool gWaitRequired = false;
    bool gWaitComplete = true;
    bool mWaitRequired = false;
    bool mWaitComplete = true;
    bool fromMainProgram = false;
    bool postActionApplied = false;
};

struct NCGMBlockTransactionCounters
{
    std::uint64_t started = 0ULL;
    std::uint64_t gWaitComponents = 0ULL;
    std::uint64_t mWaitComponents = 0ULL;
    std::uint64_t dualComponentTransactions = 0ULL;
    std::uint64_t evaluations = 0ULL;
    std::uint64_t gWaitSamples = 0ULL;
    std::uint64_t mWaitSamples = 0ULL;
    std::uint64_t readyTransitions = 0ULL;
    std::uint64_t finalized = 0ULL;
    std::uint64_t cancelled = 0ULL;
    std::uint64_t finalizeFailed = 0ULL;
    std::uint64_t m00Stops = 0ULL;
    std::uint64_t m01Stops = 0ULL;
    std::uint64_t m98Calls = 0ULL;
    std::uint64_t m99Returns = 0ULL;
    std::uint64_t m02Ends = 0ULL;
    std::uint64_t m30Ends = 0ULL;
};

static_assert(
    std::is_trivially_copyable<NCGMBlockTransactionSnapshot>::value,
    "NCGMBlockTransactionSnapshot must remain trivially copyable.");

static_assert(
    std::is_trivially_copyable<NCGMBlockTransactionCounters>::value,
    "NCGMBlockTransactionCounters must remain trivially copyable.");

// =============================================================================
// Stage NC-0.2J.4 - Pre-dispatch Stop / Settle Barrier Shadow
//
// M00/M01/M02/M30, macro flow and other barrier blocks may wait before a
// lifecycle block or Program-End request exists.  This fixed-size observer
// makes that otherwise invisible wait explicit without changing the barrier.
// =============================================================================
enum class NCPreDispatchBarrierKind : std::uint8_t
{
    NONE = 0,
    MACRO_EOF,
    ASSIGNMENT,
    GOTO_CONTROL,
    MACRO_DEPENDENCY,
    M00,
    M01,
    M02,
    M30,
    M98,
    M99,
    G_CODE_BARRIER,
    SINGLE_BLOCK_BARRIER,
    BLOCK_BARRIER
};

struct NCPreDispatchBarrierSnapshot
{
    std::uint64_t sequence = 0ULL;
    NCPreDispatchBarrierKind kind = NCPreDispatchBarrierKind::NONE;

    int sourcePC = -1;
    int sourceLineNumber = 0;
    int mCode = -1;

    std::uint64_t waitSamples = 0ULL;
    std::uint64_t commandQueueDepth = 0ULL;

    bool active = false;
    bool commandQueuePending = false;
    bool groupStandstill = false;
};

struct NCPreDispatchBarrierCounters
{
    std::uint64_t activations = 0ULL;
    std::uint64_t evaluations = 0ULL;
    std::uint64_t waitCommandQueue = 0ULL;
    std::uint64_t waitGroupStandstill = 0ULL;
    std::uint64_t cleared = 0ULL;
};

static_assert(
    std::is_trivially_copyable<NCPreDispatchBarrierSnapshot>::value,
    "NCPreDispatchBarrierSnapshot must remain trivially copyable.");
static_assert(
    std::is_trivially_copyable<NCPreDispatchBarrierCounters>::value,
    "NCPreDispatchBarrierCounters must remain trivially copyable.");
static_assert(
    sizeof(std::atomic<NCState>) == sizeof(NCState),
    "NCState atomic must remain one machine word.");
static_assert(
    ATOMIC_INT_LOCK_FREE == 2,
    "NCState atomic requires always-lock-free int atomics.");

class NCManager {
public:
    NCManager(MotionCore& motion);

    // NC-0.2L.2AJ: internal NC-thread diagnostic only; NEVER HMI/SHM/API polling.
    // No reentrancy/owner mutation during the entire call. Caller-owned output
    // must be disjoint from this manager. Failure clears it; never recaptures.
    // A successful copy is current only at this call, not a lifecycle/permit.
    NCPathCoreSampleReadResult ReadPathCoreLastFrontierSampleSameThread(
        NCPathCoreFrontierReadValueV1& output) const noexcept;

    // NC-0.2L.2AU: instantaneous NC-thread D/E/F check; no geometry is returned.
    // AV samples this privately after F. Not a ticket, control permit or HMI/SHM API.
    NCPathCoreCommandedSegmentCheck CheckPathCoreCurrentCommandedSegmentSameThread() const noexcept;

    // AW: evaluate one axis of the newest D endpoint chord, including first D.
    // Same NC thread, live sources/no reentry; output must not overlap this.
    // No production caller; not G00 interpolation, length, progress or permit.
    NCPathCoreCommandedChordCode EvaluatePathCoreCurrentCommandedChordAxisSameThread(
        std::uint32_t axisIndex, double unitParameter,
        NCPathCoreCommandedChordAxisValueV1& output) const noexcept;

    // AX: locate u from ONE axis coordinate, not measured path/execution progress.
    // Same unchanged owner/thread as AW; output must not overlap this manager.
    // Constant-axis/point queries are non-unique; no runtime consumer is added.
    NCPathCoreCommandedChordLocateCode LocatePathCoreCurrentCommandedChordAxisSameThread(
        std::uint32_t axisIndex, double queryCoordinateMCS,
        NCPathCoreCommandedChordLocationV1& output) const noexcept;

    // AY: capture newest D into a disjoint caller-provided HEAP-OWNED value.
    // Same NC thread/live owner/no reentry. No member, observer or caller added.
    // Later value queries need no D slot; NOT current proof or Motion history.
    NCPathCoreCommandedChordSegmentCaptureCode CapturePathCoreCurrentCommandedChordSegmentSameThread(
        NCPathCoreCommandedChordSegmentV1& output) const noexcept;


    // BN: internal NC-thread data access, not an exported HMI/SHM interface.
    bool ReadPathCoreLiveExecutionSameThread(
        const NCPathCoreCommandedChordStoreHandleV1& handle,
        NCPathCoreExecutionRecordV1& output) noexcept;
    // All outputs are caller-owned/disjoint; Segment output must be heap-owned.
    // Each call revalidates live NC scope. Held/stale/fenced payload reads fail.
    void GetPathCoreLiveRetentionStatusSameThread(
        NCPathCoreLiveRetentionStatusV1& output) noexcept;
    NCPathCoreCommandedChordStoreCode GetPathCoreLiveRetainedHandleSameThread(
        std::uint32_t ordinal, NCPathCoreCommandedChordStoreHandleV1& output) noexcept;
    NCPathCoreCommandedChordStoreCode ReadPathCoreLiveRetainedSegmentSameThread(
        const NCPathCoreCommandedChordStoreHandleV1& handle,
        NCPathCoreCommandedChordSegmentV1& output) noexcept;

    // BN: detached mathematical snapshot; all three referenced objects are
    // disjoint caller-owned heap values. No snapshot member or Motion permit.
    NCPathCoreCommandedChordSnapshotCode CapturePathCoreLiveSnapshotSameThread(
        const NCPathCoreCommandedChordSubpathV1& subpath,
        NCPathCoreCommandedChordSegmentV1& workspace,
        NCPathCoreCommandedChordSnapshotV1& output) noexcept;

    // BP-BEGIN
        // Internal NC-thread queries of the LAST successfully finalized retained set.
        // All outputs are disjoint caller-owned heap values. Not current permission,
        // full NC coverage, physical trajectory, or a cross-thread/SHM interface.
    bool GetPathCoreCompletedSnapshotInfoSameThread(
        NCPathCoreCompletedSnapshotInfoV1& output) const noexcept;
    NCPathCoreCompletedSnapshotCode ReadPathCoreCompletedPieceSameThread(
        std::uint32_t index, NCPathCoreCommandedChordSegmentV1& segment,
        NCPathCoreExecutionRecordV1& execution,
        NCPathCoreCommandedChordSubpathPieceV1& piece,
        NCPathCoreCommandedChordSubpathInfoV1& info) const noexcept;
    NCPathCoreCompletedSnapshotCode EvaluatePathCoreCompletedPieceSameThread(
        std::uint32_t index, double sourceU,
        NCPathCoreCommandedChordPositionSampleV1& sample,
        NCPathCoreExecutionRecordV1& execution) const noexcept;

    // BP-END
    // BQ-BEGIN
        // BQ: NC-thread producer workspace and published native-MCS commanded run.
        // Outputs are disjoint caller-owned heap values, never Motion authority.
    MotionCommandedEndpointReceiptV1* GetPathCoreCommandedReceiptWorkspaceSameThread() noexcept;
    bool ReadPathCoreCommittedRunPieceSameThread(std::uint32_t index,
        NCPathCoreCommittedRecordV1& output) noexcept;
    bool EvaluatePathCoreCommittedRunPieceSameThread(std::uint32_t index, double u,
        NCPathCoreCommittedSampleV1& output) noexcept;
    // BQ-END
        // 1. 系統狀態控制
    void ChangeMode(NCOperationMode newMode);
    void ChangeState(NCState newState);
    bool TryCommitHomingResume(
        const MotionOwnerLease& expectedHomeLease) noexcept;
    void TryRollbackHomingResume() noexcept;

    // =========================================================
// NC State Query
//
// 提供給外部控制層（例如 NCPLCManager）判斷
// NC 當前狀態。
// =========================================================
    NCState GetState() const
    {
        return m_state.load(std::memory_order_acquire);
    }

    NCOperationMode GetMode() const
    {
        return m_mode;
    }

    // 🌟 新增功能：載入 NC 程式檔
    bool LoadProgram(const std::string& filepath);
    bool RejectProgramLoad(const char* reason, const std::string& filepath) noexcept;

    // 🌟 新增：呼叫與返回副程式的介面
    bool CallMacro(const std::string& filename);
    void ReturnMacro(bool queueAlreadyDrained = false);

    // 2. 指令交握介面 (給 HMI 人機介面呼叫的)
    void CycleStart();  // 按下啟動鍵
    void FeedHold();    // 按下暫停鍵
    void Reset();       // 按下重置鍵
    // =========================================================
// External Machine Ready Interlock
//
// NCManager 不知道這個訊號來自 PLC C11、Safety PLC、
// EtherCAT 或其他來源。
//
// true  = 外部條件允許 NC 運轉
// false = 外部條件尚未 Ready
// =========================================================

    void SetExternalReadyInterlock(bool ready)
    {
        m_externalReadyInterlock = ready;
    }

    bool IsExternalReadyInterlock() const
    {
        return m_externalReadyInterlock;
    }

    void Reset_Gode();       // 重置G碼相關
    bool IsFeedHoldActive() const {
        return m_state == NCState::HOLD;
    }



    // 3. 接收解碼器傳來的單節指令 (MDI 或 自動模式)
    void PushBlock(const NCBlock& block);

    // 4. 核心執行緒 (放在 Main Loop 執行)
    void ProcessTask();

    // ======================================================
    // 🌟 5. 新增：資源存取介面 (讓外部的 Handler 檔案可以操作系統資源)
    // ======================================================
    MotionCore& GetMotion() { return m_motion; }

    // 開放 Ticks 讓外部檢查函式可以使用
    void SetG04TimeMs(double ms) { m_G04_TimeMs = ms; }
    double GetG04TimeMs() const { return m_G04_TimeMs; }


    int GetSimulatedTicks() const { return m_simulatedTicks; }
    void SetSimulatedTicks(int ticks) { m_simulatedTicks = ticks; }

    CoordinateManager& GetCoordSys() { return CoordSys; }

    // --- 子系統 ---
    CoordinateManager CoordSys;
    MacroEngine MacroSys;       // 變數引擎
    MacroParser MathParser;     // 🌟 2. 補上數學解譯器 (夾在中間)
    GCodeParser Parser;         // 字串翻譯官

    // ⚠️ 註解掉舊的 ExecState，因為我們已經全面採用頂端的 WaitState 來做狀態機了
    // enum class ExecState { RUNNING, WAITING_DWELL, WAITING_SYNC };
    // ExecState m_execState = ExecState::RUNNING;

    // ======================================================
    // 🌟 動態軸對應系統 (Dynamic Axis Mapping)
    // ======================================================
    char m_axisNames[8]; // 開機時從 ini 讀入：{'X','Y','Z','C','U','V','A','W'}

    // 從 D 槽讀取 AXIS_CFG.ini
    void LoadAxisConfiguration();

    // 🌟 核心 API：傳入英文字母 (如 'X')，回傳它是 0~7 的哪一軸。找不到回傳 -1。
    int GetAxisIndex(char gcodeLetter) const;


    // ======================================================
    // Dynamic Axis Mapping Query
    //
    // 回傳 AXIS_CFG.ini 對應的軸代號。
    // 例如 Axis3=C -> GetAxisName(3) == 'C'。
    // 無效或未設定則回傳 '?'。
    // ======================================================
    char GetAxisName(int axisIndex) const;


    const size_t MAX_MDI_LINES = 20;                        // MDI 模式最大行數
    const size_t MAX_MANUAL_AUTO_BYTES = 1024 * 1024;       // MANUAL 自動模式最大字串 (1024KB = 1MB)

    // 🌟 模式專用 API
    bool LoadMDI(const std::string& mdiContent);
    bool LoadManualAuto(const std::string& manualContent);

    // 🌟 任務分流函式
    void ProcessExecutionEngine();
    void ProcessManualMode(); // 只保留純手動 JOG 的部分

    // 🌟 新增：將 NC 狀態同步給 PLC
    void SyncNCStateToPLC();


    // 🌟 1. 新增：面板操作功能開關 (可由 HMI 或 PLC 同步設定)
    bool m_isSingleBlockEnabled = false;   // 單步執行
    bool m_isOptionalStopEnabled = false;  // 選擇性暫停 (M01)
    bool m_isBlockSkipEnabled = false;     // 選擇性跳躍 (/)


    // =========================================================
// Single Block
// =========================================================

    void SetSingleBlockEnabled(bool enabled);

    bool IsSingleBlockEnabled() const
    {
        return m_isSingleBlockEnabled;
    }


    // =========================================================
    // Optional Stop
    // =========================================================

    void SetOptionalStopEnabled(bool enabled)
    {
        m_isOptionalStopEnabled = enabled;
    }

    bool IsOptionalStopEnabled() const
    {
        return m_isOptionalStopEnabled;
    }


    // =========================================================
    // Block Skip
    // =========================================================

    void SetBlockSkipEnabled(bool enabled)
    {
        m_isBlockSkipEnabled = enabled;
    }

    bool IsBlockSkipEnabled() const
    {
        return m_isBlockSkipEnabled;
    }

    // =========================================================
    // Stage NC-0.1D - NC Motion Feedback Snapshot / Counters
    //
    // NCManager 是 Final Feedback Ring 的唯一 Consumer。
    // 這些欄位由 NC 10 ms Task 更新；外部執行緒不可直接競爭讀取。
    // HMI / API 應透過既有 SHM / Snapshot 邊界複製後查詢，
    // 且不可直接從 MotionCore 再 Pop 一次。
    // =========================================================
    bool GetLastMotionFeedback(
        MotionFeedbackEvent& event) const noexcept
    {
        if (m_lastMotionFeedback.sequence ==
            MOTION_FEEDBACK_SEQUENCE_INVALID)
        {
            return false;
        }

        event = m_lastMotionFeedback;
        return true;
    }

    MotionExecutionIdentity GetLastAcceptedMotionIdentity() const noexcept
    {
        return m_lastAcceptedMotionIdentity;
    }

    MotionExecutionIdentity GetLastStartedMotionIdentity() const noexcept
    {
        return m_lastStartedMotionIdentity;
    }

    MotionExecutionIdentity GetLastCompletedMotionIdentity() const noexcept
    {
        return m_lastCompletedMotionIdentity;
    }

    MotionExecutionIdentity GetLastRejectedMotionIdentity() const noexcept
    {
        return m_lastRejectedMotionIdentity;
    }

    MotionExecutionIdentity GetLastAbortedMotionIdentity() const noexcept
    {
        return m_lastAbortedMotionIdentity;
    }

    MotionExecutionIdentity GetLastCancelledMotionIdentity() const noexcept
    {
        return m_lastCancelledMotionIdentity;
    }

    MotionExecutionIdentity GetLastFaultedMotionIdentity() const noexcept
    {
        return m_lastFaultedMotionIdentity;
    }

    MotionFeedbackSequence GetLastConsumedMotionFeedbackSequence() const noexcept
    {
        return m_lastConsumedMotionFeedbackSequence;
    }

    std::uint64_t GetMotionFeedbackSequenceGapCount() const noexcept
    {
        return m_motionFeedbackSequenceGapCount;
    }

    std::uint64_t GetProcessedMotionFeedbackCount() const noexcept
    {
        return m_processedMotionFeedbackCount;
    }

    std::uint64_t GetAcceptedMotionFeedbackCount() const noexcept
    {
        return m_acceptedMotionFeedbackCount;
    }

    std::uint64_t GetStartedMotionFeedbackCount() const noexcept
    {
        return m_startedMotionFeedbackCount;
    }

    std::uint64_t GetCompletedMotionFeedbackCount() const noexcept
    {
        return m_completedMotionFeedbackCount;
    }

    std::uint64_t GetRejectedMotionFeedbackCount() const noexcept
    {
        return m_rejectedMotionFeedbackCount;
    }

    std::uint64_t GetAbortedMotionFeedbackCount() const noexcept
    {
        return m_abortedMotionFeedbackCount;
    }

    std::uint64_t GetCancelledMotionFeedbackCount() const noexcept
    {
        return m_cancelledMotionFeedbackCount;
    }

    std::uint64_t GetFaultedMotionFeedbackCount() const noexcept
    {
        return m_faultedMotionFeedbackCount;
    }

    // Stage NC-0.2C：Dispatch PC 與 Program Commit PC 明確分離。
    // Commit 表示 Pure Parse -> Runtime Resolve -> NC Side Effect / Downstream
    // Dispatch 已完成；它不等同馬達 COMPLETED，後者仍由 Motion Feedback 表示。
    int GetActiveDispatchPC() const noexcept;
    int GetActiveCommittedPC() const noexcept;

    NCProgramCommitSnapshot GetLastProgramCommitSnapshot() const noexcept
    {
        return m_lastProgramCommit;
    }


    bool GetLastBlockLifecycleSnapshot(
        NCBlockLifecycleSnapshot& snapshot) const noexcept
    {
        return m_blockLifecycleLedger.GetLastDispatchedSnapshot(snapshot);
    }

    bool GetLastProgramCommittedBlockLifecycleSnapshot(
        NCBlockLifecycleSnapshot& snapshot) const noexcept
    {
        return m_blockLifecycleLedger.GetLastProgramCommittedSnapshot(snapshot);
    }

    bool GetLastMotionCompletedBlockLifecycleSnapshot(
        NCBlockLifecycleSnapshot& snapshot) const noexcept
    {
        return m_blockLifecycleLedger.GetLastMotionCompletedSnapshot(snapshot);
    }

    bool GetLastTerminalBlockLifecycleSnapshot(
        NCBlockLifecycleSnapshot& snapshot) const noexcept
    {
        return m_blockLifecycleLedger.GetLastTerminalSnapshot(snapshot);
    }

    NCBlockLifecycleCounters GetBlockLifecycleCounters() const noexcept
    {
        return m_blockLifecycleLedger.GetCounters();
    }

    NCBlockCompletionBoundarySnapshot
        GetLastBlockCompletionBoundarySnapshot() const noexcept
    {
        return m_blockCompletionBoundaryObserver.GetLastSnapshot();
    }

    NCBlockCompletionBoundaryCounters
        GetBlockCompletionBoundaryCounters() const noexcept
    {
        return m_blockCompletionBoundaryObserver.GetCounters();
    }

    NCProgramEndGateSnapshot GetProgramEndGateSnapshot() const noexcept
    {
        return m_programEndBoundary.GetSnapshot();
    }

    NCProgramEndGateCounters GetProgramEndGateCounters() const noexcept
    {
        return m_programEndBoundary.GetCounters();
    }

    NCGMBlockTransactionSnapshot
        GetGMBlockTransactionSnapshot() const noexcept
    {
        return m_gmBlockTransaction.snapshot;
    }

    NCGMBlockTransactionCounters
        GetGMBlockTransactionCounters() const noexcept
    {
        return m_gmBlockTransactionCounters;
    }

    NCPreDispatchBarrierSnapshot
        GetPreDispatchBarrierSnapshot() const noexcept
    {
        return m_preDispatchBarrierSnapshot;
    }

    NCPreDispatchBarrierCounters
        GetPreDispatchBarrierCounters() const noexcept
    {
        return m_preDispatchBarrierCounters;
    }

    NCPreparedBlockQueueSnapshot
        GetPreparedBlockQueueSnapshot() const noexcept
    {
        return m_preparedBlockQueueShadow.GetSnapshot();
    }

    NCPreparedBlockQueueCounters
        GetPreparedBlockQueueCounters() const noexcept
    {
        return m_preparedBlockQueueShadow.GetCounters();
    }

    bool GetPreparedBlockQueueEntry(
        std::size_t logicalOffset,
        NCPreparedBlockEntrySnapshot& entry) const noexcept
    {
        return m_preparedBlockQueueShadow.TryGetEntry(
            logicalOffset,
            entry);
    }

    NCPreparedHeadEquivalenceSnapshot
        GetPreparedHeadEquivalenceSnapshot() const noexcept
    {
        return m_preparedHeadEquivalenceShadow.GetSnapshot();
    }

    NCPreparedHeadEquivalenceCounters
        GetPreparedHeadEquivalenceCounters() const noexcept
    {
        return m_preparedHeadEquivalenceShadow.GetCounters();
    }

    void SetPreparedHeadCutoverEnabled(bool enabled) noexcept
    {
        m_preparedHeadCutoverGate.SetEnabled(enabled);
    }

    bool IsPreparedHeadCutoverEnabled() const noexcept
    {
        return m_preparedHeadCutoverGate.IsEnabled();
    }

    NCPreparedHeadCutoverSnapshot
        GetPreparedHeadCutoverSnapshot() const noexcept
    {
        return m_preparedHeadCutoverGate.GetSnapshot();
    }

    NCPreparedHeadCutoverCounters
        GetPreparedHeadCutoverCounters() const noexcept
    {
        return m_preparedHeadCutoverGate.GetCounters();
    }

    NCPreparedPreResolveAdmissionSnapshot
        GetPreparedPreResolveAdmissionSnapshot() const noexcept
    {
        return m_preparedHeadPreResolveAdmissionShadow.GetSnapshot();
    }

    NCPreparedPreResolveAdmissionCounters
        GetPreparedPreResolveAdmissionCounters() const noexcept
    {
        return m_preparedHeadPreResolveAdmissionShadow.GetCounters();
    }

    void SetPreparedResolverBypassEnabled(bool enabled) noexcept
    {
        m_preparedHeadResolverBypassGate.SetEnabled(enabled);
    }

    bool IsPreparedResolverBypassEnabled() const noexcept
    {
        return m_preparedHeadResolverBypassGate.IsEnabled();
    }

    NCPreparedResolverBypassSnapshot
        GetPreparedResolverBypassSnapshot() const noexcept
    {
        return m_preparedHeadResolverBypassGate.GetSnapshot();
    }

    NCPreparedResolverBypassCounters
        GetPreparedResolverBypassCounters() const noexcept
    {
        return m_preparedHeadResolverBypassGate.GetCounters();
    }

    NCOrdinaryG00AdmissionSnapshot
        GetOrdinaryG00AdmissionSnapshot() const noexcept
    {
        return m_ordinaryG00AdmissionShadow.GetSnapshot();
    }

    NCOrdinaryG00AdmissionCounters
        GetOrdinaryG00AdmissionCounters() const noexcept
    {
        return m_ordinaryG00AdmissionShadow.GetCounters();
    }

    NCOrdinaryG00InflightRegistrySnapshot
        GetOrdinaryG00InflightRegistrySnapshot() const noexcept
    {
        return m_ordinaryG00InflightRegistryShadow.GetSnapshot();
    }

    NCOrdinaryG00InflightRegistryCounters
        GetOrdinaryG00InflightRegistryCounters() const noexcept
    {
        return m_ordinaryG00InflightRegistryShadow.GetCounters();
    }

    bool GetOrdinaryG00InflightRegistryEntry(
        std::size_t slot,
        NCOrdinaryG00InflightEntrySnapshot& entry) const noexcept
    {
        return m_ordinaryG00InflightRegistryShadow.TryGetEntry(
            slot,
            entry);
    }

    NCOrdinaryG00FeedHoldCohortSnapshot
        GetOrdinaryG00FeedHoldCohortSnapshot() const noexcept
    {
        return m_ordinaryG00FeedHoldCohortShadow.GetSnapshot();
    }

    NCOrdinaryG00FeedHoldCohortCounters
        GetOrdinaryG00FeedHoldCohortCounters() const noexcept
    {
        return m_ordinaryG00FeedHoldCohortShadow.GetCounters();
    }

    void SetOrdinaryG00FeedHoldCohortCutoverEnabled(
        bool enabled) noexcept
    {
        m_ordinaryG00FeedHoldCohortCutoverGate.SetEnabled(enabled);
    }

    bool IsOrdinaryG00FeedHoldCohortCutoverEnabled() const noexcept
    {
        return m_ordinaryG00FeedHoldCohortCutoverGate.IsEnabled();
    }

    NCOrdinaryG00FeedHoldCohortCutoverSnapshot
        GetOrdinaryG00FeedHoldCohortCutoverSnapshot() const noexcept
    {
        return m_ordinaryG00FeedHoldCohortCutoverGate.GetSnapshot();
    }

    NCOrdinaryG00FeedHoldCohortCutoverCounters
        GetOrdinaryG00FeedHoldCohortCutoverCounters() const noexcept
    {
        return m_ordinaryG00FeedHoldCohortCutoverGate.GetCounters();
    }

    NCOrdinaryG00FeedHoldCohortRearmSnapshot
        GetOrdinaryG00FeedHoldCohortRearmSnapshot() const noexcept
    {
        return m_ordinaryG00FeedHoldCohortRearmShadow.GetSnapshot();
    }

    NCOrdinaryG00FeedHoldCohortRearmCounters
        GetOrdinaryG00FeedHoldCohortRearmCounters() const noexcept
    {
        return m_ordinaryG00FeedHoldCohortRearmShadow.GetCounters();
    }

    void SetOrdinaryG00FeedHoldRearmCutoverEnabled(
        bool enabled) noexcept
    {
        m_ordinaryG00FeedHoldCohortRearmCutoverGate.SetEnabled(enabled);
    }

    bool IsOrdinaryG00FeedHoldRearmCutoverEnabled() const noexcept
    {
        return m_ordinaryG00FeedHoldCohortRearmCutoverGate.IsEnabled();
    }

    NCOrdinaryG00FeedHoldRearmCutoverSnapshot
        GetOrdinaryG00FeedHoldRearmCutoverSnapshot() const noexcept
    {
        return m_ordinaryG00FeedHoldCohortRearmCutoverGate.GetSnapshot();
    }

    NCOrdinaryG00FeedHoldRearmCutoverCounters
        GetOrdinaryG00FeedHoldRearmCutoverCounters() const noexcept
    {
        return m_ordinaryG00FeedHoldCohortRearmCutoverGate.GetCounters();
    }

    NCOrdinaryG00FeedHoldRollingRearmSnapshot
        GetOrdinaryG00FeedHoldRollingRearmSnapshot() const noexcept
    {
        return m_ordinaryG00FeedHoldRollingRearmShadow.GetSnapshot();
    }

    NCOrdinaryG00FeedHoldRollingRearmCounters
        GetOrdinaryG00FeedHoldRollingRearmCounters() const noexcept
    {
        return m_ordinaryG00FeedHoldRollingRearmShadow.GetCounters();
    }

    void SetOrdinaryG00FeedHoldRollingCutoverEnabled(
        bool enabled) noexcept
    {
        m_ordinaryG00FeedHoldRollingRearmCutoverGate.SetEnabled(enabled);
    }

    bool IsOrdinaryG00FeedHoldRollingCutoverEnabled() const noexcept
    {
        return m_ordinaryG00FeedHoldRollingRearmCutoverGate.IsEnabled();
    }

    NCOrdinaryG00FeedHoldRollingCutoverSnapshot
        GetOrdinaryG00FeedHoldRollingCutoverSnapshot() const noexcept
    {
        return m_ordinaryG00FeedHoldRollingRearmCutoverGate.GetSnapshot();
    }

    NCOrdinaryG00FeedHoldRollingCutoverCounters
        GetOrdinaryG00FeedHoldRollingCutoverCounters() const noexcept
    {
        return m_ordinaryG00FeedHoldRollingRearmCutoverGate.GetCounters();
    }

    void SetOrdinaryG00ReadAheadCutoverEnabled(bool enabled) noexcept
    {
        m_ordinaryG00ReadAheadCutoverGate.SetEnabled(enabled);
    }

    bool IsOrdinaryG00ReadAheadCutoverEnabled() const noexcept
    {
        return m_ordinaryG00ReadAheadCutoverGate.IsEnabled();
    }

    NCOrdinaryG00ReadAheadSnapshot
        GetOrdinaryG00ReadAheadSnapshot() const noexcept
    {
        return m_ordinaryG00ReadAheadCutoverGate.GetSnapshot();
    }

    NCOrdinaryG00ReadAheadCounters
        GetOrdinaryG00ReadAheadCounters() const noexcept
    {
        return m_ordinaryG00ReadAheadCutoverGate.GetCounters();
    }

    bool ConsumeOrdinaryG00ReadAheadMotionAuthorization() noexcept
    {
        return m_ordinaryG00ReadAheadCutoverGate.
            ConsumeMotionAuthorization(
                m_currentExecutingBlockDispatchId);
    }

    NCSingleBlockShadowSnapshot
        GetSingleBlockShadowSnapshot() const noexcept
    {
        return m_singleBlockBoundaryShadow.GetSnapshot();
    }

    NCSingleBlockShadowCounters
        GetSingleBlockShadowCounters() const noexcept
    {
        return m_singleBlockBoundaryShadow.GetCounters();
    }

    // =========================================================
    // Stage NC-0.2I.4 - Single Block Completion-Gated HOLD
    //
    // Default is enabled. Set false to return subsequent Single
    // Block execution to the legacy m_pauseAfterBlock path.
    // Disabling never creates an automatic RUN transition.
    // =========================================================
    void SetSingleBlockCompletionGateEnabled(bool enabled) noexcept;

    bool IsSingleBlockCompletionGateEnabled() const noexcept
    {
        return m_singleBlockHoldGate.IsEnabled();
    }

    NCSingleBlockHoldGateSnapshot
        GetSingleBlockHoldGateSnapshot() const noexcept
    {
        return m_singleBlockHoldGate.GetSnapshot();
    }

    NCSingleBlockHoldGateCounters
        GetSingleBlockHoldGateCounters() const noexcept
    {
        return m_singleBlockHoldGate.GetCounters();
    }


    NCFeedHoldBoundarySnapshot
        GetFeedHoldBoundarySnapshot() const noexcept
    {
        return m_feedHoldBoundaryShadow.GetSnapshot();
    }

    NCFeedHoldBoundaryCounters
        GetFeedHoldBoundaryCounters() const noexcept
    {
        return m_feedHoldBoundaryShadow.GetCounters();
    }

    // =========================================================
    // Stage NC-0.2I.3 - Program Feed Hold ACK-Gated Resume
    //
    // Default is enabled.  Set false to return Cycle Start to the
    // legacy immediate-resume path.  Disabling while a resume is
    // deferred leaves the machine in HOLD; it never auto-runs.
    // =========================================================
    void SetFeedHoldAckGateEnabled(bool enabled) noexcept
    {
        m_feedHoldResumeGate.SetEnabled(enabled);
    }

    bool IsFeedHoldAckGateEnabled() const noexcept
    {
        return m_feedHoldResumeGate.IsEnabled();
    }

    NCFeedHoldResumeGateSnapshot
        GetFeedHoldResumeGateSnapshot() const noexcept
    {
        return m_feedHoldResumeGate.GetSnapshot();
    }

    NCFeedHoldResumeGateCounters
        GetFeedHoldResumeGateCounters() const noexcept
    {
        return m_feedHoldResumeGate.GetCounters();
    }

    // =========================================================
    // Stage NC-0.2J.1 - Lifecycle Failure / Epoch Cancellation
    // Shadow Boundary. Diagnostics only; no SHM ABI expansion and
    // no Reset / Alarm / Motion behavior cutover in this stage.
    // =========================================================
    NCLifecycleInterruptionSnapshot
        GetLifecycleInterruptionSnapshot() const noexcept
    {
        return m_lifecycleInterruptionShadow.GetSnapshot();
    }

    NCLifecycleInterruptionCounters
        GetLifecycleInterruptionCounters() const noexcept
    {
        return m_lifecycleInterruptionShadow.GetCounters();
    }

    // =========================================================
    // Stage NC-0.2J.3 - Reset Stable-Standstill Release Gate.
    // READY and SAFETY-owner release require an exact matching
    // QUIESCENT_PROVED boundary with the full stable sample count.
    // =========================================================
    NCResetReleaseGateSnapshot
        GetResetReleaseGateSnapshot() const noexcept
    {
        return m_resetReleaseGate.GetSnapshot();
    }

    NCResetReleaseGateCounters
        GetResetReleaseGateCounters() const noexcept
    {
        return m_resetReleaseGate.GetCounters();
    }

    // =========================================================
    // Stage NC-0.2J.6.1 - Alarm / Emergency-stop RT ACK Shadow.
    // Read-only diagnostics; no Alarm clear or recovery permission.
    // =========================================================
    NCAlarmEmergencyStopSnapshot
        GetAlarmEmergencyStopSnapshot() const noexcept
    {
        return m_alarmEmergencyStopShadow.GetSnapshot();
    }

    NCAlarmEmergencyStopCounters
        GetAlarmEmergencyStopCounters() const noexcept
    {
        return m_alarmEmergencyStopShadow.GetCounters();
    }

    // =========================================================
    // Stage NC-0.2L.1E2 - scalar HMI state and change token only.
    // The HMI receives no counters, snapshot or control permit.
    // =========================================================
    NCPathCoreInputHandoffCompactState
        GetPathCoreInputHandoffCompactState() const noexcept
    {
        return m_pathCoreInputHandoffCompactShadow.GetCompactState();
    }

    std::uint32_t GetPathCoreInputHandoffChangeToken() const noexcept
    {
        return m_pathCoreInputHandoffCompactShadow.GetChangeToken();
    }

    static bool WaitForGMBlockTransactionCallback(NCManager* nc);
    static bool WaitAndHoldCallback(NCManager* nc);
    static bool WaitAndClearQueueCallback(NCManager* nc);
    static bool WaitForSingleBlockControlledHoldCallback(NCManager* nc);
    static bool WaitForCycleStartCallback(NCManager* nc); // 新增：專等 CycleStart 按鈕

    // Stage NC-0.1F：G81 HOME 完成後，接回 HOME 交還的新一代 Program Lease。
    bool AdoptProgramMotionLease(
        const MotionOwnerLease& lease) noexcept;
private:
    // Stage NC-0.1E：NC Program Owner Lease 生命週期。
    bool AcquireProgramMotionOwner() noexcept;
    void ReleaseProgramMotionOwner() noexcept;

    MotionOwnerLease m_programMotionLease{};
    MotionOwnerLease m_safetyMotionLease{};

    // Stage NC-0.1D：每個 NC 10 ms Cycle 先 Drain Motion Feedback Ring。
    void ProcessMotionFeedback() noexcept;
    // Startup may inherit a quiescent SAFETY owner from the runtime bootstrap.
    // This is a one-time, read-only-evidence-gated handoff; it never clears
    // alarms/faults and never enables physical motion.
    void ObserveBootstrapSafetyHandoff() noexcept;
    bool EnsureMappingIntegrityAlarmBoundaryBeforeFeedback() noexcept;
    enum class ResetPreDrainReconcileResult : std::uint8_t
    {
        READY = 0,
        DEFERRED,
        SUPERSEDED
    };
    ResetPreDrainReconcileResult
        ReconcileResetPreDrainMappingAlarmBoundary() noexcept;

    // Stage NC-0.2D：Program Commit 與 Motion Segment Feedback 的對照表。
    NCBlockLifecycleLedger m_blockLifecycleLedger{};

    // Stage NC-0.2J.1：Reset / Alarm / Epoch replacement and failed
    // terminal feedback share one fixed-size diagnostic interruption chain.
    NCLifecycleInterruptionBoundaryShadow m_lifecycleInterruptionShadow{};
    NCResetReleaseGate m_resetReleaseGate{};
    NCAlarmEmergencyStopBoundaryShadow m_alarmEmergencyStopShadow{};
    bool m_lifecycleInterruptionAlarmLatched = false;
    std::uint64_t m_lastHandledMappingIntegrityAlarmRequestCount = 0ULL;

    // The first program image is a boot image, not a replacement of an
    // already-running program.  Defer a retained SAFETY owner handoff until
    // the 250 us runtime publishes clean standstill evidence.
    bool m_bootProgramImageLoaded = false;
    bool m_programLoadStartBlocked = false;
    bool m_bootSafetyHandoffPending = true;

    // Stage NC-0.2F：已追蹤 Motion Block 的 Wait Callback 採 Dual-Key
    // Guard；非 Motion Callback 維持 Legacy 行為。
    NCBlockCompletionBoundaryObserver m_blockCompletionBoundaryObserver{};
    NCBlockDispatchId m_waitingBlockDispatchId =
        NC_BLOCK_DISPATCH_ID_INVALID;

    // Stage NC-0.2G：M02 / M30 / Natural EOF 共用同一個 Cycle End Gate。
    NCProgramEndBoundary m_programEndBoundary{};
    bool m_programEndAlarmRaised = false;

    // NC-0.2J.5.3: Cycle Start publishes a fresh Execution Epoch before the
    // Program Run boundary is opened.  The 250 us Motion consumer must first
    // acknowledge that publication; otherwise the run-start safety sample
    // sees its own Epoch PENDING bit and rejects every start as START_DIRTY.
    // Keep the exact Epoch latched so a newer Reset/Stop/Fault publication
    // can only cancel this start, never accidentally authorize it.
    bool m_programRunStartPending = false;
    enum class ProgramRunStartPhase : std::uint8_t
    {
        IDLE = 0,
        ALARM_ADMISSION,
        EPOCH_ACK
    };
    ProgramRunStartPhase m_pendingProgramRunPhase =
        ProgramRunStartPhase::IDLE;
    MotionExecutionEpoch m_pendingProgramRunExecutionEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    MotionOwnerLease m_pendingProgramRunOwnerLease{};
    NCOperationMode m_pendingProgramRunMode = NCOperationMode::EDIT;
    NCState m_pendingProgramRunOriginState = NCState::NOT_READY;
    NCProgramScope m_pendingProgramRunScope = NCProgramScope::NONE;
    NCProgramCacheGeneration m_pendingProgramRunCacheGeneration =
        NC_PROGRAM_CACHE_GENERATION_INVALID;
    std::uint32_t m_pendingProgramRunAlarmUpdateCount = 0U;
    std::uint64_t m_pendingProgramRunAlarmSafetyIntentState = 0ULL;

    // Stage NC-0.2K.6.2: GOTO publishes a replacement Epoch before the
    // 250 us consumer can acknowledge/drain it.  Keep that exact identity
    // separate from the legacy m_programChanged macro-flow flag so a later
    // Safety/Reset Epoch can never authorize a stale GOTO rebase.
    bool m_gotoQueueTailRebasePending = false;
    MotionExecutionEpoch m_pendingGotoQueueTailRebaseExecutionEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    MotionOwnerLease m_pendingGotoQueueTailRebaseOwnerLease{};
    int m_pendingGotoQueueTailRebaseSourceLine = 0;

    struct NCGMBlockTransactionState
    {
        NCGMBlockTransactionSnapshot snapshot{};
        WaitConditionFunc gCallback = nullptr;
        WaitConditionFunc mCallback = nullptr;
    };

    NCGMBlockTransactionState m_gmBlockTransaction{};
    NCGMBlockTransactionCounters m_gmBlockTransactionCounters{};
    std::uint64_t m_nextGMBlockTransactionSequence = 1ULL;

    NCPreDispatchBarrierSnapshot m_preDispatchBarrierSnapshot{};
    NCPreDispatchBarrierCounters m_preDispatchBarrierCounters{};
    std::uint64_t m_nextPreDispatchBarrierSequence = 1ULL;

    // Stage NC-0.2K.1: fixed-capacity, observation-only Prepared Block Queue.
    // It is deliberately separate from m_blockQueue and MotionCore queues.
    NCPreparedBlockQueueShadow m_preparedBlockQueueShadow{};

    // Stage NC-0.2K.2: one exact-head dual-path proof.  It remains a pure
    // observer; K.3 owns the separate, reversible value-source decision.
    NCPreparedHeadEquivalenceShadow
        m_preparedHeadEquivalenceShadow{};

    // Stage NC-0.2K.3.1: after one full K.2 lifecycle proof has qualified the
    // current session, an exact current head may supply the NCBlock value to
    // the unchanged handler path.  Ordinary no-P G00 additionally requires
    // the same-pass legacy drain proof.  Default enabled; setter above is the
    // immediate rollback switch for undispatched heads.
    NCPreparedHeadCutoverGate m_preparedHeadCutoverGate{};

    // Stage NC-0.2K.4: captures one immutable Prepared head before the legacy
    // resolver, then uses the unchanged K.2/K.3 result as its oracle.  This is
    // observation-only: it cannot bypass ResolveBlock or influence Runtime.
    NCPreparedHeadPreResolveAdmissionShadow
        m_preparedHeadPreResolveAdmissionShadow{};

    // Stage NC-0.2K.4.1: after one complete per-lane legacy proof, this
    // reversible gate may skip ResolveBlock only for PURE_MODAL_COPY or an
    // exact literal G00 P1 head.  It owns a separate lifecycle proof and
    // never fabricates a K.2/K.3/K.4 legacy comparison for bypassed tokens.
    NCPreparedHeadResolverBypassGate
        m_preparedHeadResolverBypassGate{};

    // Stage NC-0.2K.5: models the future ordinary no-P G00
    // BUFFERED + command-local EXACT_STOP admission contract.  It only reads
    // K.4.2 and lifecycle evidence; the accepted drain/ABORTING/callback path
    // remains the sole Runtime path in this stage.
    NCOrdinaryG00BufferedExactStopAdmissionShadow
        m_ordinaryG00AdmissionShadow{};

    // Stage NC-0.2K.6.3/K.7.1: independently binds each accepted ordinary G00
    // to one fixed-capacity terminal slot.  It never writes Motion, PC,
    // callback, Epoch, owner, or PDO state; K.7.1 makes successful registration
    // a mandatory post-submit proof for its controlled path.
    NCOrdinaryG00InflightTerminalRegistryShadow
        m_ordinaryG00InflightRegistryShadow{};

    // Stage NC-0.2K.7.3: captures the exact two active K.7.1 identities at a
    // PROGRAM Feed Hold edge and observes ACK, Resume and ordered terminals.
    // K.7.4 may consume its immutable proof, but this observer never acts.
    NCOrdinaryG00FeedHoldCohortShadow
        m_ordinaryG00FeedHoldCohortShadow{};

    // Stage NC-0.2K.7.4: consumes only the exact K.7.3 cohort proof at the
    // K.7.1 admission seam.  It prevents a third read-ahead G00 from joining
    // a Feed-Hold cohort before both captured members reach ordered terminal.
    // Failure selects the existing legacy drain path; Motion remains untouched.
    NCOrdinaryG00FeedHoldCohortCutoverGate
        m_ordinaryG00FeedHoldCohortCutoverGate{};

    // Stage NC-0.2K.7.5: observes two consecutive exact K.7.4 cohorts in one
    // Queue session and proves clean release-before-rearm, monotonic
    // generations and cross-generation Motion identity isolation.  Shadow
    // only; no admission or Motion result consumes this evidence.
    NCOrdinaryG00FeedHoldCohortRearmShadow
        m_ordinaryG00FeedHoldCohortRearmShadow{};

    // Stage NC-0.2K.7.6: consumes the exact K.7.5 second-generation proof at
    // the K.7.1 admission seam.  It waits until SECOND_RELEASED /
    // REARM_PROVEN and selects legacy drain on any cross-generation mismatch.
    // It owns no Motion storage and never writes Motion or PDO state.
    NCOrdinaryG00FeedHoldCohortRearmCutoverGate
        m_ordinaryG00FeedHoldCohortRearmCutoverGate{};

    // Stage NC-0.2K.7.7: the exact K.7.5/K.7.6 two-generation proof seeds
    // continuous same-session observation of later K.7.3/K.7.4 cohorts.
    // Observation only; no admission decision or Motion/PDO write.
    NCOrdinaryG00FeedHoldRollingRearmShadow
        m_ordinaryG00FeedHoldRollingRearmShadow{};

    // Stage NC-0.2K.7.8: consumes only K.7.7 generation-three-and-later
    // continuity at the K.7.1 ordinary G00 admission seam.  A bound rolling
    // generation waits for exact K.7.7/K.7.4/Registry release; mismatch keeps
    // the current Queue session on legacy drain.  No Motion/PDO write.
    NCOrdinaryG00FeedHoldRollingRearmCutoverGate
        m_ordinaryG00FeedHoldRollingRearmCutoverGate{};

    // Stage NC-0.2K.7.1: reversible, qualification-gated Runtime cutover.
    // It owns no Motion storage and cannot exceed two active registry entries.
    NCOrdinaryG00ReadAheadCutoverGate
        m_ordinaryG00ReadAheadCutoverGate{};

    // Stage NC-0.2I.1：只觀察 Single Block 正確完成點。
    NCSingleBlockBoundaryShadow m_singleBlockBoundaryShadow{};
    bool m_legacySingleBlockPausePending = false;

    // Stage NC-0.2I.4：已通過 Shadow 驗證的 Boundary 正式接管
    // Program Single Block HOLD，並保留 Runtime Legacy 回退。
    NCSingleBlockHoldGate m_singleBlockHoldGate{};


    // NC-0.2J.5：區分 Feed Hold Request、Legacy HOLD 顯示與正式的
    // RT 連續 Settle Acknowledge。
    NCFeedHoldBoundaryShadowObserver m_feedHoldBoundaryShadow{};
    // NC-0.2L.2T_M00_FIX1: current PROGRAM Feed Hold lifetime only.
    // Retained FAILED diagnostics must not own a later M00/M01 HOLD.
    // Explicit cancellation clears this even when the observer is inactive.
    bool m_programFeedHoldLifetimeActive = false;
    MotionNCSettleRequestSequence m_feedHoldNCSettleRequestSequence =
        MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID;

    // Stage NC-0.2J.5：Reset release is tied to one RT-owned settle/rebase
    // transaction.  Sequence zero is never a valid request.
    MotionNCSettleRequestSequence m_resetNCSettleRequestSequence =
        MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID;

    // Stage NC-0.2K.6.2: one operator Reset owns one fresh SAFETY
    // generation request.  If a bounded Motion reservation defers the
    // handshake, subsequent 10 ms scans continue against this original
    // generation instead of minting another generation/Epoch.
    enum class ResetContinuationPhase : std::uint8_t
    {
        IDLE = 0,
        // A Reset received during an ordinary G00/G01 interpolation group
        // first lets the existing SAFETY-owned StopGroup trajectory decelerate
        // the physical axes.  No Reset PDO hold or Reset batch is published
        // until that trajectory has terminated.
        PRE_RESET_CONTROLLED_STOP,
        BUTTON_ADMISSION,
        OUTPUT_HOLD,
        PRE_DRAIN,
        AUTHORITY,
        EPOCH,
        BATCH,
        ALARM_CLEAR,
        CLEANUP,
        SETTLE,
        RELEASE_GATE,
        BLOCKED
    };

    ResetContinuationPhase m_resetContinuationPhase =
        ResetContinuationPhase::IDLE;
    MotionOwnerGeneration m_resetAuthorityEntryGeneration =
        MOTION_OWNER_GENERATION_INVALID;
    std::uint32_t m_resetAuthorityBaselineTicket = 0U;
    std::uint32_t m_resetAuthorityRequestTicket = 0U;
    std::uint64_t m_resetButtonCutoffProvenanceGeneration = 0ULL;
    std::uint64_t m_resetAuthorityProvenanceGeneration = 0ULL;
    bool m_resetSafetyOutputHoldActive = false;
    std::uint32_t m_resetAuthorityAlarmUpdateCount = 0U;
    std::uint64_t m_resetAuthorityAlarmSafetyIntentState = 0ULL;
    std::uint64_t m_resetAuthorityMappingAlarmRequestCount = 0ULL;
    std::uint64_t m_resetLifecycleInterruptionSequence = 0ULL;
    // NC-0.2K.7.1.1: a Reset pressed while an ordinary group is active opens
    // its lifecycle boundary before the smooth controlled-stop pre-phase.
    // Keep that exact old Epoch/AUTO/active-block identity across a deferred
    // button-admission retry so the later ABORTED active command and stale
    // read-ahead retirement remain provably owned by this operator Reset.
    bool m_resetLifecycleBoundaryPrearmedByControlledStop = false;
    MotionExecutionEpoch m_resetContinuationExecutionEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    MotionNCResetExecutionState m_resetContinuationExecutionState{};

    // Stage NC-0.2I.3：PROGRAM Feed Hold 的 Cycle Start 在 ACK 前只
    // 先鎖存；ACK 成立後才允許真正恢復。保留 Runtime Legacy 回退開關。
    NCFeedHoldResumeGate m_feedHoldResumeGate{};
    enum class HoldResumeAdmissionKind : std::uint8_t
    {
        NONE = 0,
        CONTROLLED_SINGLE_BLOCK,
        PROGRAM_HOLD
    };
    HoldResumeAdmissionKind m_holdResumeAdmissionKind =
        HoldResumeAdmissionKind::NONE;
    std::uint32_t m_holdResumeAlarmUpdateCount = 0U;
    std::uint64_t m_holdResumeAlarmSafetyIntentState = 0ULL;
    bool m_holdResumeGateControlled = false;

    MotionFeedbackEvent m_lastMotionFeedback{};
    MotionFeedbackEvent m_deferredResetMotionFeedback{};
    bool m_deferredResetMotionFeedbackValid = false;
    MotionExecutionIdentity m_lastAcceptedMotionIdentity{};
    MotionExecutionIdentity m_lastStartedMotionIdentity{};
    MotionExecutionIdentity m_lastCompletedMotionIdentity{};
    MotionExecutionIdentity m_lastRejectedMotionIdentity{};
    MotionExecutionIdentity m_lastCancelledMotionIdentity{};
    MotionExecutionIdentity m_lastAbortedMotionIdentity{};
    MotionExecutionIdentity m_lastFaultedMotionIdentity{};

    MotionFeedbackSequence m_lastConsumedMotionFeedbackSequence =
        MOTION_FEEDBACK_SEQUENCE_INVALID;
    std::uint64_t m_motionFeedbackSequenceGapCount = 0ULL;

    std::uint64_t m_processedMotionFeedbackCount = 0ULL;
    std::uint64_t m_acceptedMotionFeedbackCount = 0ULL;
    std::uint64_t m_startedMotionFeedbackCount = 0ULL;
    std::uint64_t m_completedMotionFeedbackCount = 0ULL;
    std::uint64_t m_rejectedMotionFeedbackCount = 0ULL;
    std::uint64_t m_cancelledMotionFeedbackCount = 0ULL;
    std::uint64_t m_abortedMotionFeedbackCount = 0ULL;
    std::uint64_t m_faultedMotionFeedbackCount = 0ULL;

    // 🌟 新增：統一暫停旗標 (用來標記這行跑完後是否需要停下來)
    bool m_pauseAfterBlock = false;
    // 🌟 判斷這行單節是否為「真的會產生機台移動」的指令
    bool IsRealMotionBlock(const NCBlock& block);

    // Stage NC-0.2A：同一 Block 先 Commit 相容 Modal，再擷取 Motion 標籤。
    void CapturePendingCommandState(int sourcePC);
    WaitConditionFunc DispatchSingleGCode(
        const NCBlock& sourceBlock,
        int gCode);

    void BeginGMBlockTransaction(
        WaitConditionFunc gCallback,
        WaitConditionFunc mCallback,
        NCGMBlockPostAction postAction,
        int sourcePC,
        int sourceLineNumber,
        NCBlockDispatchId dispatchId,
        int mCode,
        int pValue,
        int repeatCount,
        bool fromMainProgram) noexcept;
    bool EvaluateGMBlockTransaction() noexcept;
    bool FinalizeGMBlockTransaction();
    void CancelGMBlockTransaction(bool superseded) noexcept;
    std::uint64_t AllocateGMBlockTransactionSequence() noexcept;

    void ObservePreDispatchBarrier(
        NCPreDispatchBarrierKind kind,
        int sourcePC,
        int sourceLineNumber,
        int mCode,
        std::uint64_t commandQueueDepth,
        bool groupStandstill) noexcept;
    void ClearPreDispatchBarrier() noexcept;

    NCPreparedSourceIdentity
        BuildPreparedBlockSourceIdentity() const noexcept;
    NCPreparedModalSnapshot
        BuildPreparedBlockModalSnapshot() const noexcept;
    NCPreparedRuntimeProof
        BuildPreparedBlockRuntimeProof() const noexcept;
    NCPreparedInvalidationReason
        GetPreparedBlockInactiveReason() const noexcept;
    void ObservePreparedBlockQueueShadow(bool allowPlanning) noexcept;
    void ObservePathCoreAcceptedReadAheadInput(
        const NCOrdinaryG00InflightRegistrationProof& proof,
        const NCPreparedHeadCutoverContext& context) noexcept;
    NCPreparedHeadCutoverContext CapturePreparedHeadBeforeResolve(
        int sourcePC,
        int sourceLineNumber) const noexcept;
    bool ObservePreparedHeadEquivalenceResolved(
        int sourcePC,
        int sourceLineNumber,
        const NCParsedBlock& parsedBlock,
        const NCBlock& legacyBlock,
        bool legacyDrainRequired,
        NCPreparedHeadCutoverContext& cutoverContext) noexcept;
    void ObservePreparedHeadEquivalenceResolveFailure(
        const NCPreparedHeadCutoverContext& cutoverContext) noexcept;
    void ObservePreparedHeadEquivalenceUpstreamProof(
        const NCPreparedRuntimeProof& proof) noexcept;
    bool BindPreparedHeadEquivalenceDispatch(
        NCBlockDispatchId dispatchId,
        const NCProgramCommitSnapshot& dispatchTarget) noexcept;
    void CompletePreparedHeadEquivalence(
        NCBlockDispatchId dispatchId,
        const NCProgramCommitSnapshot& commitTarget,
        bool commitSucceeded) noexcept;
    void FailPreparedHeadEquivalenceRuntime(
        NCBlockDispatchId dispatchId) noexcept;

    static NCSingleBlockCandidateKind ClassifySingleBlockCandidate(
        const NCBlock& block) noexcept;
    void ArmSingleBlockShadow(
        NCSingleBlockCandidateKind candidateKind,
        NCBlockDispatchId dispatchId,
        const NCProgramCommitSnapshot& target,
        int sourceLineNumber) noexcept;
    void EvaluateSingleBlockShadow(bool callbackComplete) noexcept;
    void ObserveSingleBlockHoldGate() noexcept;
    bool ApplyControlledSingleBlockHold() noexcept;
    bool ApplyControlledSingleBlockResume() noexcept;
    void ObserveLegacySingleBlockHold() noexcept;
    void CancelSingleBlockShadow(bool superseded) noexcept;


    NCFeedHoldBoundarySample BuildFeedHoldBoundarySample() const noexcept;
    void BeginFeedHoldBoundaryShadow(NCFeedHoldSource source) noexcept;
    void ObserveFeedHoldBoundaryShadow() noexcept;
    void ObserveFeedHoldLegacyHoldShadow() noexcept;
    void ObserveFeedHoldResumeRequestedShadow() noexcept;
    void ObserveFeedHoldResumeAppliedShadow() noexcept;
    void CancelFeedHoldBoundaryShadow(bool superseded) noexcept;
    void BeginOrdinaryG00FeedHoldCohortShadow() noexcept;
    void ObserveOrdinaryG00FeedHoldCohortBoundary() noexcept;
    void ObserveOrdinaryG00FeedHoldCohortCutover() noexcept;

    bool IsProgramFeedHoldResumeCandidate() const noexcept;
    bool ApplyProgramHoldResume(bool gateControlled) noexcept;
    bool ProcessFeedHoldResumeGate() noexcept;
    bool ArmHoldResumeAlarmAdmission(
        HoldResumeAdmissionKind kind) noexcept;
    void ClearHoldResumeAlarmAdmission(
        HoldResumeAdmissionKind kind) noexcept;

    NCLifecycleInterruptionSample
        BuildLifecycleInterruptionSample() const noexcept;
    void BeginLifecycleInterruptionShadow(
        NCLifecycleInterruptionCause cause,
        bool expectsEpochChange) noexcept;
    void RecordLifecycleInterruptionEpochPublished(
        MotionExecutionEpoch executionEpoch) noexcept;
    void ObserveLifecycleInterruptionShadow() noexcept;
    NCAlarmEmergencyStopSample
        BuildAlarmEmergencyStopSample() const noexcept;
    void BeginAlarmEmergencyStopShadow() noexcept;
    void ObserveAlarmEmergencyStopShadow() noexcept;
    static bool IsLifecycleFailureFeedback(
        MotionFeedbackType type) noexcept;
    static NCLifecycleInterruptionCause
        LifecycleInterruptionCauseFromFeedback(
            MotionFeedbackType type) noexcept;

    // Stage NC-0.2C：Parsed Program Cache / Program Commit Boundary。
    NCProgramCache& GetBaseProgramCache() noexcept;
    const NCProgramCache& GetBaseProgramCache() const noexcept;

    int GetBasePCValue() const noexcept;
    int& GetBaseCommittedPC() noexcept;
    int GetBaseCommittedPCValue() const noexcept;
    NCProgramScope GetBaseProgramScope() const noexcept;

    bool TryGetCurrentJumpTarget(
        int sequenceNumber,
        int& targetPC) const;

    NCProgramCommitSnapshot MakeCurrentProgramCommitTarget(
        int sourcePC) const noexcept;
    bool CommitProgramBlock(
        const NCProgramCommitSnapshot& target,
        NCProgramCommitSnapshot& committedSnapshot) noexcept;
    void BindProgramBlockMotionCapture(
        NCBlockDispatchId dispatchId,
        const MotionProgramBlockCapture& capture) noexcept;

    void BindCompletionWaitBoundary(
        NCBlockDispatchId dispatchId,
        WaitConditionFunc callback) noexcept;
    bool ApplyCompletionWaitBoundaryGuard(bool legacyReady) noexcept;
    void ClearCompletionWaitBoundary(bool superseded) noexcept;

    NCProgramEndGateSample BuildProgramEndGateSample() const noexcept;
    bool BeginProgramRunBoundary(MotionExecutionEpoch executionEpoch) noexcept;
    bool IsPendingProgramRunStartIdentityCurrent() const noexcept;
    void ReleasePendingProgramRunMotionOwner() noexcept;
    void ClearPendingProgramRunStart(bool cancelled) noexcept;
    bool ProcessPendingProgramRunStart() noexcept;
    bool ArmPendingGotoQueueTailRebase(
        MotionExecutionEpoch executionEpoch,
        int sourceLineNumber) noexcept;
    bool IsPendingGotoQueueTailRebaseIdentityCurrent() const noexcept;
    void ClearPendingGotoQueueTailRebase() noexcept;
    bool RequestProgramEnd(
        NCProgramEndCause cause,
        int sourcePC,
        int sourceLineNumber,
        NCBlockDispatchId markerDispatchId) noexcept;
    void ProcessProgramEndBoundary();
    void FinalizeProgramEnd();
    void CancelProgramEndBoundary() noexcept;

    void ResetActiveProgramCommitBoundary() noexcept;
    void ResetAllProgramCommitBoundaries() noexcept;
    NCProgramFrameId AllocateMacroFrameId() noexcept;
public:
    uint32_t NC_RunCount;//NC執行迴圈數
    uint32_t API_RunCount;//API執行迴圈數
    MotionCore& m_motion;

    // =========================================================
    // G81 HOME Manager
    //
    // NCManager 擁有一個 HomingManager。
    //
    // HomingManager：
    //     負責 G81 HOME 流程 / 狀態機 / 多軸排程。
    //
    // MotionCore：
    //     仍負責真正的軸運動。
    //
    // 這裡直接使用 m_motion 建立，
    // 所以不需要另外 new / delete。
    // =========================================================

    HomingManager Homing{ m_motion };

    NCOperationMode m_mode = NCOperationMode::MANUAL;
    std::atomic<NCState> m_state{ NCState::NOT_READY };
    EDMState m_edmState = EDMState::NOT_READY;


    // =========================================================
// External Machine Ready Interlock
// =========================================================
    bool m_externalReadyInterlock = false;

    bool Close_System_Com_flag = 0;//關閉核心命令

    // 🌟 新增：主程式與副程式追蹤變數
    std::string m_mainProgramName = "";
    std::string m_macroProgramName = "";
    int m_macroProgramPC = -1; // -1 代表目前沒有在執行副程式


  // ==========================================
    // 🌟 新增：多層副程式 (Macro) 執行框架結構
    // ==========================================
    struct MacroFrame {
        std::string programName;            // 這層副程式的檔名 (例如 O1234.nc)
        const NCProgramCache* program = nullptr; // 指向穩定的 Parsed Macro Cache
        NCProgramFrameId frameId = NC_PROGRAM_FRAME_ID_INVALID;
        int currentPC = 0;                  // 下一個要 Dispatch 的 PC
        int committedPC = -1;               // 最近完成 Program Commit 的 PC
        int returnPC = 0;                   // M99 返回上一層的 PC
        int repeatCount = 1;
    };

    // 🌟 這是解決錯誤的關鍵：用來儲存最多 8 層的副程式堆疊
    // (請把舊的 m_macroMemory 和 m_returnStack 刪掉，換成這個)
    std::vector<MacroFrame> m_macroStack;

    bool m_programChanged = false;          // 🌟 標記是否發生了程式跳轉 (M98/M99)

    // Stage NC-0.2C：主程式只在 LoadProgram 時 Parse 一次。
    NCProgramCache m_programCache;
    int m_programPC = 0;                  // 下一個要 Dispatch 的 PC
    int m_programCommittedPC = -1;        // 最近完成 Program Commit 的 PC

    // 同一主程式執行期間，Macro 第一次載入後共用 Parsed Cache。
    // Reset / 載入新主程式會清除，避免編輯後沿用舊內容。
    std::map<std::string, NCProgramCache> m_macroProgramCaches;

    // NC 指令緩衝區
    std::queue<NCBlock> m_blockQueue;

    // 內部執行功能
    void ExecuteBlock(
        const NCBlock& block,
        int sourcePC,
        int sourceLineNumber,
        NCBlockDispatchId dispatchId);

    // Valid only while ExecuteBlock is synchronously dispatching one block.
    // G00 consumes a K.7.1 authorization through the public one-shot API.
    NCBlockDispatchId m_currentExecutingBlockDispatchId =
        NC_BLOCK_DISPATCH_ID_INVALID;

    // 🌟 替換：捨棄 Enum，改用統一的檢查回呼函式
    WaitConditionFunc m_waitCallback = nullptr;

    int m_simulatedTicks = 0; // (測試用) 模擬馬達跑了多久
    double m_G04_TimeMs = 0.0;


    // 🌟 MDI 專屬變數
    NCProgramCache m_mdiProgramCache;
    int m_mdiPC = 0;
    int m_mdiCommittedPC = -1;

    // 🌟 MANUAL (輕量自動) 專屬變數
    NCProgramCache m_manualProgramCache;
    int m_manualPC = 0;
    int m_manualCommittedPC = -1;
    bool m_manualAutoRunning = false;

    NCProgramCommitSnapshot m_lastProgramCommit{};
    NCProgramCommitSequence m_nextProgramCommitSequence = 1ULL;
    NCProgramFrameId m_nextMacroFrameId = 1ULL;

    // 動態獲取目前模式的 Base Dispatch PC。
    int& GetBasePC();

    bool LoadDynamicCode(const std::string& content);

    // 🌟 新增：提供給 HMI 狀態廣播用的動態指標
    int GetActivePC() {
        return GetBasePC();
    }

    EDMState GetMachineEDMState();
    // 取得已經存好的狀態變數
    EDMState GetCurrentEDMState() const { return m_edmState; }


    // 🌟 集中更新所有 $ 變數 (供 G00, G01, G92, Reset 結束時即時呼叫)
    void UpdateSystemVariables();

    int UpdateSystemVariables_initialize_flag = 0;

    // 🌟 G66 模態巨集專用狀態
    bool m_isG66Active = false; // G66 開關
    int m_g66P = 0;             // 記住呼叫的副程式名稱 (P)
    int m_g66L = 1;             // 記住重複次數 (L)
    NCBlock m_g66Block;         // 記住 G66 當下夾帶的所有變數 (A, B, C...)

private:
    // NC-0.2L.1A2 through L.2A: HMI remains scalar-only. NCManager is
    // heap-owned, so these fixed tail members do not consume RT thread stack.
    NCPathCoreInputHandoffCompactShadow
        m_pathCoreInputHandoffCompactShadow{};

    // NC-0.2L.2A: last two accepted K.7 handoff identities. History-only;
    // never consulted by a Runtime, Gate, PC, Alarm or Motion decision.
    NCPathCoreInputContractShadow m_pathCoreInputContractShadow{};

    // NC-0.2L.2D: last two accepted ordinary G00 committed commanded
    // endpoint-pair observations. Heap-resident, same-thread and history-only.
    NCPathCoreCommittedGeometryShadow m_pathCoreCommittedGeometryShadow{};

    // NC-0.2L.2E: immediate L.2D endpoint seam and opaque queue-tail seal.
    // Fixed heap-resident history only; never consulted by control decisions.
    NCPathCoreCommittedGeometryLinkShadow
        m_pathCoreCommittedGeometryLinkShadow{};

    // NC-0.2L.2F: current committed endpoint segment formed only from a
    // proven L.2E link. Fixed heap-resident history; no control consumer.
    NCPathCoreLinkedCommittedSegmentShadow
        m_pathCoreLinkedCommittedSegmentShadow{};

    // NC-0.2L.2G: immediate continuity relation between the newest two
    // formed L.2F segments. Compact fixed history; no control consumer.
    NCPathCoreLinkedCommittedSegmentPairShadow
        m_pathCoreLinkedCommittedSegmentPairShadow{};

    // NC-0.2L.2H: scalar run summary formed only from directly overlapping
    // proven L.2G pairs. No endpoint arrays, traversal or control consumer.
    NCPathCoreLinkedCommittedSegmentRunShadow
        m_pathCoreLinkedCommittedSegmentRunShadow{};

    // NC-0.2L.2I: commanded head/tail endpoint boundary of the newest
    // proven L.2H run. No intermediate segment history or control consumer.
    NCPathCoreLinkedCommittedSegmentRunBoundaryShadow
        m_pathCoreLinkedCommittedSegmentRunBoundaryShadow{};

    // NC-0.2L.2J: component-wise net displacement from the proven L.2I
    // run head to current tail. No endpoint list, traversal or control use.
    NCPathCoreLinkedCommittedSegmentRunDisplacementShadow
        m_pathCoreLinkedCommittedSegmentRunDisplacementShadow{};

    // NC-0.2L.2K: scalar endpoint-open / returned-axis classification of
    // the proven L.2J run displacement. No geometry array or control use.
    NCPathCoreLinkedCommittedSegmentRunClosureShadow
        m_pathCoreLinkedCommittedSegmentRunClosureShadow{};

    // NC-0.2L.2L: immediate scalar transition between adjacent proven
    // L.2K closure states. No geometry, traversal or control consumer.
    NCPathCoreLinkedCommittedSegmentRunClosureTransitionShadow
        m_pathCoreLinkedCommittedSegmentRunClosureTransitionShadow{};

    // NC-0.2L.2M: bounded scalar coverage of directly adjacent proven
    // L.2L transition events. No event order, geometry or control consumer.
    NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryShadow
        m_pathCoreLinkedCommittedSegmentRunClosureTransitionSummaryShadow{};

    // NC-0.2L.2N: scalar qualification of which endpoint states and direct
    // return/reopen transitions L.2M has covered. No event order or geometry.
    NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationShadow
        m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationShadow{};

    // NC-0.2L.2O: direct scalar qualification changes inside one proven
    // coverage interval. Fixed heap-resident history; no control consumer.
    NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionShadow
        m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionShadow{};

    // NC-0.2L.2P: direct pair relations across overlapping L.2O scalar
    // projections. Fixed heap-resident history; no control consumer.
    NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairShadow
        m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairShadow{};

    // NC-0.2L.2Q: retained scalar overlap continuity across adjacent L.2P
    // pairs. Fixed heap-resident history; no control consumer.
    NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityShadow
        m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityShadow{};

    // NC-0.2L.2R: counts only consecutive proven L.2Q certificates.
    // Fixed heap-resident history; no control consumer.
    NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunShadow
        m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunShadow{};

    // NC-0.2L.2S: retains head/latest references only after observing
    // the actual R run head. Fixed heap history; no control consumer.
    NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryShadow
        m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryShadow{};

    // NC-0.2L.2T: direct S certificate availability changes only.
    // Fixed heap history; unavailable does not mean run completion.
    NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionShadow
        m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionShadow{};

    // NC-0.2L.2U: joins adjacent T certificates on shared identity/state.
    // Two scalar heap records; no same-run restoration or control claim.
    NCPathCoreBoundaryAvailabilityTransitionPairShadow
        m_pathCoreBoundaryAvailabilityTransitionPairShadow{};

    // NC-0.2L.2V: counts only directly bound U certificates in a local run.
    // Two fixed scalar heap records; no segment count or execution claim.
    NCPathCoreBoundaryAvailabilityTransitionPairRunShadow
        m_pathCoreBoundaryAvailabilityTransitionPairRunShadow{};

    // NC-0.2L.2W: summarizes eight U patterns only after observing V head.
    // Fixed scalar heap history; missing prefix and event order stay unknown.
    NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageShadow
        m_pathCoreBoundaryAvailabilityTransitionPairRunCoverageShadow{};

    // NC-0.2L.2X: compares two adjacent proven W coverage summaries.
    // Fixed scalar heap history; interruption never implies pattern loss.
    NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionShadow
        m_pathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionShadow{};

    // NC-0.2L.2Y: joins two directly adjacent proven X transitions.
    // Fixed scalar heap history; no event list or continuity through gaps.
    NCPathCoreRunCoverageTransitionPairShadow
        m_pathCoreRunCoverageTransitionPairShadow{};

    // NC-0.2L.2Z: counts a local suffix of directly linked Y certificates.
    // Fixed scalar heap history; a gap clears the count and starts no bridge.
    NCPathCoreRunCoverageTransitionPairRunShadow
        m_pathCoreRunCoverageTransitionPairRunShadow{};

    // NC-0.2L.2AA: pins first-Y coverage only at an observed Z START.
    // Missing the head leaves this scalar boundary unproven until a new START.
    NCPathCoreRunCoverageBoundaryShadow
        m_pathCoreRunCoverageBoundaryShadow{};

    // NC-0.2L.2AB: classifies directly adjacent AA certificate availability.
    // Invalid or missing proof creates no transition and controls no motion.
    NCPathCoreRunCoverageBoundaryAvailabilityShadow
        m_pathCoreRunCoverageBoundaryAvailabilityShadow{};

    // NC-0.2L.2AC: one compatible pair of adjacent AB availability transitions.
    // Missing or invalid proof yields no pair and controls no motion.
    NCPathCoreRunCoverageBoundaryAvailabilityPairShadow
        m_pathCoreRunCoverageBoundaryAvailabilityPairShadow{};

    // NC-0.2L.2AD: counts only consecutive overlapping AC certificates here.
    // Any proof interruption ends the local count; no motion interval is inferred.
    NCPathCoreRunCoverageBoundaryAvailabilityPairRunShadow
        m_pathCoreRunCoverageBoundaryAvailabilityPairRunShadow{};

    // NC-0.2L.2AE: terminal current-source coherence and explicit local proof scope.
    // Fixed small heap history only; no control consumer or new run accumulator.
    NCPathCoreCurrentProofScopeShadow m_pathCoreCurrentProofScopeShadow{};

    // NC-0.2L.2AF: compact identity for the currently rebound AE proof frontier.
    // No path admission, queue ownership, geometry capture or motion consumer.
    NCPathCoreProofFrontierShadow m_pathCoreProofFrontierShadow{};
    // L.2AI: one last-sampled read workspace; no history/publication or control use.
    NCPathCoreFrontierReadProbe m_pathCoreFrontierReadProbe{};

    // NC-0.2L.2AK: last CALL receipt only, not a retained frontier/certificate.
    // This private diagnostic never drives control and is not HMI/SHM data.
    enum class PathCoreSampleReadbackDisposition : std::uint8_t
    {
        NOT_CALLED = 0U,
        READ_REJECTED_EMPTY_OUTPUT = 1U,
        READ_BOUNDARY_AVAILABLE = 2U,
        READ_BOUNDARY_UNAVAILABLE = 3U,
        READBACK_CONTRACT_MISMATCH = 4U
    };
    static_assert(sizeof(PathCoreSampleReadbackDisposition) == 1U,
        "AK readback disposition is one byte.");
    NCPathCoreSampleReadResult m_pathCoreSampleReadbackResult{};
    PathCoreSampleReadbackDisposition m_pathCoreSampleReadbackDisposition =
        PathCoreSampleReadbackDisposition::NOT_CALLED;
    // NC-0.2L.2AP / Declared Path Core Member Storage Budget Guard.
    // Audit the ACTUAL member declarations, not only their named record types.
    // An array, pointer/reference or same-sized substitute is not the approved
    // in-object owner. This block has no data, constructor or runtime work.
    // The 34-member inventory totals 9769 sizeof-bytes (9760 owners/workspace
    // plus 9 diagnostic bytes). Inter-member/NCManager padding, other members,
    // allocator metadata, external allocations and call-chain stack are NOT
    // included; this is NOT sizeof(NCManager) or process/RTSS memory usage.
    // New members MUST be added to this reviewed inventory explicitly: C++14
    // has no automatic enumeration of future class members. Equal size/traits
    // do not prove no allocation, a history capacity, or safe observer copying.
    // Keep existing ownership/lifetime/source-proof rules and header guards.
    // A failed assertion requires a storage-contract review, NOT an automatic
    // increase of this budget. Existing early bounded geometry is preserved.
#define NC_PATH_CORE_AP_MEMBER_CHECK(member, expected, bytes, alignment) \
    static_assert(std::is_same<decltype(member), expected>::value, \
        "AP member type changed: " #member); \
    static_assert(sizeof(member) == bytes && alignof(decltype(member)) == alignment, \
        "AP member storage changed: " #member); \
    static_assert(std::is_trivially_copyable<decltype(member)>::value && \
        std::is_trivially_destructible<decltype(member)>::value, \
        "AP member lifetime traits changed: " #member)
#define NC_PATH_CORE_AP_NONCOPYABLE_MEMBER_CHECK(member, expected, bytes, alignment) \
    static_assert(std::is_same<decltype(member), expected>::value, \
        "AP member type changed: " #member); \
    static_assert(sizeof(member) == bytes && alignof(decltype(member)) == alignment, \
        "AP member storage changed: " #member); \
    static_assert(std::is_trivially_destructible<decltype(member)>::value && \
        !std::is_copy_constructible<decltype(member)>::value && \
        !std::is_copy_assignable<decltype(member)>::value && \
        !std::is_move_constructible<decltype(member)>::value && \
        !std::is_move_assignable<decltype(member)>::value, \
        "AP noncopyable member lifetime contract changed: " #member)

    static_assert(sizeof(void*) == 8U, "AP member budget requires the x64 target.");
    NC_PATH_CORE_AP_MEMBER_CHECK(m_pathCoreInputHandoffCompactShadow,
        NCPathCoreInputHandoffCompactShadow, 8U, 4U);
    NC_PATH_CORE_AP_MEMBER_CHECK(m_pathCoreInputContractShadow,
        NCPathCoreInputContractShadow, 120U, 8U);
    NC_PATH_CORE_AP_MEMBER_CHECK(m_pathCoreCommittedGeometryShadow,
        NCPathCoreCommittedGeometryShadow, 504U, 8U);
    NC_PATH_CORE_AP_MEMBER_CHECK(m_pathCoreCommittedGeometryLinkShadow,
        NCPathCoreCommittedGeometryLinkShadow, 224U, 8U);
    NC_PATH_CORE_AP_MEMBER_CHECK(m_pathCoreLinkedCommittedSegmentShadow,
        NCPathCoreLinkedCommittedSegmentShadow, 448U, 8U);
    NC_PATH_CORE_AP_MEMBER_CHECK(m_pathCoreLinkedCommittedSegmentPairShadow,
        NCPathCoreLinkedCommittedSegmentPairShadow, 288U, 8U);
    NC_PATH_CORE_AP_MEMBER_CHECK(m_pathCoreLinkedCommittedSegmentRunShadow,
        NCPathCoreLinkedCommittedSegmentRunShadow, 280U, 8U);
    NC_PATH_CORE_AP_MEMBER_CHECK(m_pathCoreLinkedCommittedSegmentRunBoundaryShadow,
        NCPathCoreLinkedCommittedSegmentRunBoundaryShadow, 592U, 8U);
    NC_PATH_CORE_AP_MEMBER_CHECK(m_pathCoreLinkedCommittedSegmentRunDisplacementShadow,
        NCPathCoreLinkedCommittedSegmentRunDisplacementShadow, 464U, 8U);
    NC_PATH_CORE_AP_MEMBER_CHECK(m_pathCoreLinkedCommittedSegmentRunClosureShadow,
        NCPathCoreLinkedCommittedSegmentRunClosureShadow, 368U, 8U);
    NC_PATH_CORE_AP_MEMBER_CHECK(m_pathCoreLinkedCommittedSegmentRunClosureTransitionShadow,
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionShadow, 400U, 8U);
    NC_PATH_CORE_AP_MEMBER_CHECK(m_pathCoreLinkedCommittedSegmentRunClosureTransitionSummaryShadow,
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionSummaryShadow, 488U, 8U);
    NC_PATH_CORE_AP_MEMBER_CHECK(m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationShadow,
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationShadow, 576U, 8U);
    NC_PATH_CORE_AP_MEMBER_CHECK(m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionShadow,
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionShadow, 320U, 8U);
    NC_PATH_CORE_AP_MEMBER_CHECK(m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairShadow,
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairShadow, 320U, 8U);
    NC_PATH_CORE_AP_MEMBER_CHECK(m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityShadow,
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityShadow, 192U, 8U);
    NC_PATH_CORE_AP_MEMBER_CHECK(m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunShadow,
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunShadow, 240U, 8U);
    NC_PATH_CORE_AP_MEMBER_CHECK(m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryShadow,
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryShadow, 320U, 8U);
    NC_PATH_CORE_AP_MEMBER_CHECK(m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionShadow,
        NCPathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionShadow, 80U, 8U);
    NC_PATH_CORE_AP_MEMBER_CHECK(m_pathCoreBoundaryAvailabilityTransitionPairShadow,
        NCPathCoreBoundaryAvailabilityTransitionPairShadow, 112U, 8U);
    NC_PATH_CORE_AP_MEMBER_CHECK(m_pathCoreBoundaryAvailabilityTransitionPairRunShadow,
        NCPathCoreBoundaryAvailabilityTransitionPairRunShadow, 176U, 8U);
    NC_PATH_CORE_AP_MEMBER_CHECK(m_pathCoreBoundaryAvailabilityTransitionPairRunCoverageShadow,
        NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageShadow, 208U, 8U);
    NC_PATH_CORE_AP_MEMBER_CHECK(m_pathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionShadow,
        NCPathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionShadow, 240U, 8U);
    NC_PATH_CORE_AP_MEMBER_CHECK(m_pathCoreRunCoverageTransitionPairShadow,
        NCPathCoreRunCoverageTransitionPairShadow, 256U, 8U);
    NC_PATH_CORE_AP_MEMBER_CHECK(m_pathCoreRunCoverageTransitionPairRunShadow,
        NCPathCoreRunCoverageTransitionPairRunShadow, 320U, 8U);
    NC_PATH_CORE_AP_MEMBER_CHECK(m_pathCoreRunCoverageBoundaryShadow,
        NCPathCoreRunCoverageBoundaryShadow, 352U, 8U);
    NC_PATH_CORE_AP_MEMBER_CHECK(m_pathCoreRunCoverageBoundaryAvailabilityShadow,
        NCPathCoreRunCoverageBoundaryAvailabilityShadow, 400U, 8U);
    NC_PATH_CORE_AP_MEMBER_CHECK(m_pathCoreRunCoverageBoundaryAvailabilityPairShadow,
        NCPathCoreRunCoverageBoundaryAvailabilityPairShadow, 448U, 8U);
    NC_PATH_CORE_AP_MEMBER_CHECK(m_pathCoreRunCoverageBoundaryAvailabilityPairRunShadow,
        NCPathCoreRunCoverageBoundaryAvailabilityPairRunShadow, 512U, 8U);
    NC_PATH_CORE_AP_MEMBER_CHECK(m_pathCoreCurrentProofScopeShadow,
        NCPathCoreCurrentProofScopeShadow, 208U, 8U);
    NC_PATH_CORE_AP_MEMBER_CHECK(m_pathCoreProofFrontierShadow,
        NCPathCoreProofFrontierShadow, 176U, 8U);
    NC_PATH_CORE_AP_NONCOPYABLE_MEMBER_CHECK(m_pathCoreFrontierReadProbe,
        NCPathCoreFrontierReadProbe, 120U, 8U);
    NC_PATH_CORE_AP_MEMBER_CHECK(m_pathCoreSampleReadbackResult,
        NCPathCoreSampleReadResult, 8U, 1U);
    NC_PATH_CORE_AP_MEMBER_CHECK(m_pathCoreSampleReadbackDisposition,
        PathCoreSampleReadbackDisposition, 1U, 1U);
#undef NC_PATH_CORE_AP_NONCOPYABLE_MEMBER_CHECK
#undef NC_PATH_CORE_AP_MEMBER_CHECK

    static_assert(
        sizeof(m_pathCoreInputHandoffCompactShadow) +
        sizeof(m_pathCoreInputContractShadow) +
        sizeof(m_pathCoreCommittedGeometryShadow) +
        sizeof(m_pathCoreCommittedGeometryLinkShadow) +
        sizeof(m_pathCoreLinkedCommittedSegmentShadow) +
        sizeof(m_pathCoreLinkedCommittedSegmentPairShadow) +
        sizeof(m_pathCoreLinkedCommittedSegmentRunShadow) +
        sizeof(m_pathCoreLinkedCommittedSegmentRunBoundaryShadow) +
        sizeof(m_pathCoreLinkedCommittedSegmentRunDisplacementShadow) +
        sizeof(m_pathCoreLinkedCommittedSegmentRunClosureShadow) +
        sizeof(m_pathCoreLinkedCommittedSegmentRunClosureTransitionShadow) +
        sizeof(m_pathCoreLinkedCommittedSegmentRunClosureTransitionSummaryShadow) +
        sizeof(m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationShadow) +
        sizeof(m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionShadow) +
        sizeof(m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairShadow) +
        sizeof(m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityShadow) +
        sizeof(m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunShadow) +
        sizeof(m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryShadow) +
        sizeof(m_pathCoreLinkedCommittedSegmentRunClosureTransitionCoverageQualificationTransitionPairContinuityRunBoundaryAvailabilityTransitionShadow) +
        sizeof(m_pathCoreBoundaryAvailabilityTransitionPairShadow) +
        sizeof(m_pathCoreBoundaryAvailabilityTransitionPairRunShadow) +
        sizeof(m_pathCoreBoundaryAvailabilityTransitionPairRunCoverageShadow) +
        sizeof(m_pathCoreBoundaryAvailabilityTransitionPairRunCoverageTransitionShadow) +
        sizeof(m_pathCoreRunCoverageTransitionPairShadow) +
        sizeof(m_pathCoreRunCoverageTransitionPairRunShadow) +
        sizeof(m_pathCoreRunCoverageBoundaryShadow) +
        sizeof(m_pathCoreRunCoverageBoundaryAvailabilityShadow) +
        sizeof(m_pathCoreRunCoverageBoundaryAvailabilityPairShadow) +
        sizeof(m_pathCoreRunCoverageBoundaryAvailabilityPairRunShadow) +
        sizeof(m_pathCoreCurrentProofScopeShadow) +
        sizeof(m_pathCoreProofFrontierShadow) +
        sizeof(m_pathCoreFrontierReadProbe) +
        sizeof(m_pathCoreSampleReadbackResult) +
        sizeof(m_pathCoreSampleReadbackDisposition) == 9769U,
        "AP reviewed Path Core member-size sum changed; review the inventory and budget.");

    // NC-0.2L.2AV: last synchronous AU check, diagnostic only. Initial
    // NO_CURRENT_SEGMENT does not prove this call has run. This byte is NOT
    // a live proof, ticket, lifecycle epoch or Motion/Path Queue permission.
    // A subsequent rejected check overwrites success; no retained fallback.
    // No reset/stop/idle hook is added. Only the NC thread may inspect it.
    NCPathCoreCommandedSegmentCheck m_pathCoreCommandedSegmentLastCheck =
        NCPathCoreCommandedSegmentCheck::NO_CURRENT_SEGMENT;
    // Keep AP's original 34-member 9769-byte subtotal and FIX1 unchanged.
    // AV adds one separately checked byte: reviewed named total = 9770.
    static_assert(std::is_same<decltype(m_pathCoreCommandedSegmentLastCheck),
        NCPathCoreCommandedSegmentCheck>::value &&
        sizeof(m_pathCoreCommandedSegmentLastCheck) == 1U &&
        alignof(decltype(m_pathCoreCommandedSegmentLastCheck)) == 1U,
        "AV commanded-segment diagnostic must remain its exact one-byte type.");
    static_assert(std::is_trivially_copyable<decltype(m_pathCoreCommandedSegmentLastCheck)>::value&&
        std::is_trivially_destructible<decltype(m_pathCoreCommandedSegmentLastCheck)>::value,
        "AV commanded-segment diagnostic lifetime traits changed.");


    static_assert(static_cast<std::uint32_t>(NCProgramScope::MEMORY) == 1U &&
        static_cast<std::uint32_t>(NCProgramScope::MDI) == 2U &&
        static_cast<std::uint32_t>(NCProgramScope::MANUAL_AUTO) == 3U &&
        static_cast<std::uint32_t>(NCOperationMode::MEMORY) == 0U &&
        static_cast<std::uint32_t>(NCOperationMode::MDI) == 1U &&
        static_cast<std::uint32_t>(NCOperationMode::MANUAL) == 2U,
        "BN scope encodings must match existing NC enums.");
    // BN is a separate 5904-byte named heap-member budget; AP/AV stay 9770.
    using PathCoreLiveFenceReason = NCPathCoreLiveRetentionReason;
    struct PathCoreLiveNativeConfig
    {
        double lead[8U]{};
        double resolution[8U]{};
        std::int32_t axisIndex[8U]{};
        std::uint8_t axisType[8U]{};
        char axisName[8U]{};
        std::uint8_t exists[8U]{};
    };
    struct PathCoreLiveBookkeeping
    {
        std::uint64_t lastRunToken = 0ULL;
        std::uint64_t currentRunToken = 0ULL;
        std::uint64_t readbackCount = 0ULL;
        std::uint32_t holdCount = 0U;
        std::uint32_t resumeCount = 0U;
        bool lastReadbackMatched = false;
        bool nativeBound = false;
        bool runSummaryQueued = false;
        bool summaryPending = false;
        std::uint8_t reserved[4U]{};
    };
    struct PathCoreLiveSummary
    {
        NCPathCoreLiveRetentionStatusV1 status{};
        std::uint64_t readbackCount = 0ULL;
        std::uint32_t holdCount = 0U;
        std::uint32_t resumeCount = 0U;
        bool lastReadbackMatched = false;
        std::uint8_t reserved[7U]{};
    };
    NCPathCoreLiveRetention m_pathCoreLiveRetention;
    NCPathCoreCommandedChordSegmentV1 m_pathCoreLiveCapture{};
    NCPathCoreCommandedChordSegmentV1 m_pathCoreLiveReadback{};
    NCPathCoreCommandedChordStoreHandleV1 m_pathCoreLiveHandle{};
    NCPathCoreLiveRetentionStatusV1 m_pathCoreLiveStatus{};
    PathCoreLiveNativeConfig m_pathCoreLiveNative{};
    PathCoreLiveBookkeeping m_pathCoreLiveBookkeeping{};
    PathCoreLiveSummary m_pathCoreLiveSummary{};
    static_assert(sizeof(PathCoreLiveNativeConfig) == 184U &&
        alignof(PathCoreLiveNativeConfig) == 8U &&
        sizeof(PathCoreLiveBookkeeping) == 40U &&
        alignof(PathCoreLiveBookkeeping) == 8U &&
        sizeof(PathCoreLiveSummary) == 72U &&
        alignof(PathCoreLiveSummary) == 8U,
        "BN fixed native/small-summary layout changed.");
    static_assert(std::is_trivially_copyable<PathCoreLiveNativeConfig>::value&&
        std::is_trivially_copyable<PathCoreLiveBookkeeping>::value&&
        std::is_trivially_copyable<PathCoreLiveSummary>::value,
        "BN scalar scope and summary must remain fixed values.");
    static_assert(std::is_same<decltype(m_pathCoreLiveRetention),
        NCPathCoreLiveRetention>::value &&
        sizeof(m_pathCoreLiveRetention) == 5200U &&
        alignof(NCPathCoreLiveRetention) == 8U &&
        std::is_trivially_destructible<NCPathCoreLiveRetention>::value &&
        !std::is_copy_constructible<NCPathCoreLiveRetention>::value &&
        !std::is_copy_assignable<NCPathCoreLiveRetention>::value &&
        !std::is_move_constructible<NCPathCoreLiveRetention>::value &&
        !std::is_move_assignable<NCPathCoreLiveRetention>::value,
        "BN retention is one fixed, noncopyable heap-owned member.");
    static_assert(sizeof(m_pathCoreLiveRetention) +
        sizeof(m_pathCoreLiveCapture) + sizeof(m_pathCoreLiveReadback) +
        sizeof(m_pathCoreLiveHandle) + sizeof(m_pathCoreLiveStatus) +
        sizeof(m_pathCoreLiveNative) + sizeof(m_pathCoreLiveBookkeeping) +
        sizeof(m_pathCoreLiveSummary) == 5904U,
        "BN named runtime addition changed; review independently of AP/AV.");

    // BO adds 3200 named heap-member bytes; BN/AP/AV budgets stay separate.
    NCPathCoreExecutionLink m_pathCoreExecutionLink{};
    NCPathCoreExecutionLinkStatusV1 m_pathCoreExecutionSummary{};
    struct PathCoreExecutionBookkeeping
    {
        bool summaryPending = false;
        std::uint8_t reserved[7U]{};
    } m_pathCoreExecutionBookkeeping{};
    static_assert(sizeof(m_pathCoreExecutionLink) == 3128U &&
        sizeof(m_pathCoreExecutionSummary) == 64U &&
        sizeof(m_pathCoreExecutionBookkeeping) == 8U &&
        sizeof(m_pathCoreExecutionLink) + sizeof(m_pathCoreExecutionSummary) +
        sizeof(m_pathCoreExecutionBookkeeping) == 3200U,
        "BO fixed heap member budget changed.");
    void ClosePathCoreExecutionLinkSameThread(PathCoreLiveFenceReason reason) noexcept;
    void FlushPathCoreExecutionSummarySameThread() noexcept;

    // BP-BEGIN
    NCPathCoreCompletedSnapshotV1 m_pathCoreCompletedSnapshot{};
    NCPathCoreCompletedSnapshotWorkspaceV1 m_pathCoreCompletedWorkspace{};
    NCPathCoreCompletedSnapshotInfoV1 m_pathCoreCompletedInfo{};
    NCPathCoreExecutionRecordV1 m_pathCoreCompletedRecord{};
    NCPathCoreCommandedChordPositionSampleV1 m_pathCoreCompletedSample{};
    struct PathCoreCompletedSummary
    {
        std::uint64_t run = 0ULL, lifetime = 0ULL;
        std::uint32_t count = 0U, reads = 0U, evaluations = 0U;
        NCPathCoreCompletedSnapshotCode code = NCPathCoreCompletedSnapshotCode::NONE;
        bool published = false, pending = false;
        std::uint8_t reserved = 0U;
    } m_pathCoreCompletedSummary{};
    struct PathCoreCompletedBookkeeping
    {
        bool prepared = false, readable = false;
        std::uint8_t reserved[6U]{};
    } m_pathCoreCompletedBookkeeping{};
    static_assert(sizeof(m_pathCoreCompletedSnapshot) == 8336U &&
        sizeof(m_pathCoreCompletedWorkspace) == 432U &&
        sizeof(m_pathCoreCompletedInfo) == 40U &&
        sizeof(m_pathCoreCompletedRecord) == 96U &&
        sizeof(m_pathCoreCompletedSample) == 120U &&
        sizeof(m_pathCoreCompletedSummary) == 32U &&
        sizeof(m_pathCoreCompletedBookkeeping) == 8U &&
        sizeof(m_pathCoreCompletedSnapshot) + sizeof(m_pathCoreCompletedWorkspace) +
        sizeof(m_pathCoreCompletedInfo) + sizeof(m_pathCoreCompletedRecord) +
        sizeof(m_pathCoreCompletedSample) + sizeof(m_pathCoreCompletedSummary) +
        sizeof(m_pathCoreCompletedBookkeeping) == 9064U,
        "BP adds 9064 named heap bytes; AP/AV/BN/BO budgets remain separate.");
    void DiscardPathCoreCompletedSnapshotSameThread() noexcept;
    void PreparePathCoreCompletedSnapshotSameThread() noexcept;
    void PublishPathCoreCompletedSnapshotSameThread() noexcept;
    void FlushPathCoreCompletedSummarySameThread() noexcept;

    // BP-END
    // BQ-BEGIN
    NCPathCoreCommittedRun m_pathCoreCommittedRun{};
    NCPathCoreCommittedScopeV1 m_pathCoreCommittedScope{};
    std::array<double, 8U> m_pathCoreCommittedRotaryModulo{};
    MotionCommandedEndpointReceiptV1 m_pathCoreCommandedReceipt{};
    NCPathCoreCommittedRunStatusV1 m_pathCoreCommittedStatus{};
    NCPathCoreCommittedRunStatusV1 m_pathCoreCommittedSummary{};
    NCPathCoreCommittedRecordV1 m_pathCoreCommittedRecord{};
    NCPathCoreCommittedSampleV1 m_pathCoreCommittedSample{};
    struct PathCoreCommittedBookkeeping
    {
        NCBlockDispatchId dispatchId = NC_BLOCK_DISPATCH_ID_INVALID;
        std::uint32_t reads = 0U, evaluations = 0U;
        bool eligible = false, requested = false, summaryPending = false;
        bool published = false, closed = false, pendingAdmission = false;
        std::uint8_t reserved[2U]{};
        std::uint64_t pendingCommitSequence = 0ULL;
    } m_pathCoreCommittedBookkeeping{};
    static_assert(sizeof(m_pathCoreCommittedRun) == 8272U &&
        sizeof(m_pathCoreCommittedScope) == 40U && sizeof(m_pathCoreCommittedRotaryModulo) == 64U &&
        sizeof(m_pathCoreCommandedReceipt) == 216U && sizeof(m_pathCoreCommittedStatus) == 72U &&
        sizeof(m_pathCoreCommittedSummary) == 72U && sizeof(m_pathCoreCommittedRecord) == 256U &&
        sizeof(m_pathCoreCommittedSample) == 128U && sizeof(m_pathCoreCommittedBookkeeping) == 32U &&
        sizeof(m_pathCoreCommittedRun) + sizeof(m_pathCoreCommittedScope) +
        sizeof(m_pathCoreCommittedRotaryModulo) + sizeof(m_pathCoreCommandedReceipt) +
        sizeof(m_pathCoreCommittedStatus) + sizeof(m_pathCoreCommittedSummary) +
        sizeof(m_pathCoreCommittedRecord) + sizeof(m_pathCoreCommittedSample) +
        sizeof(m_pathCoreCommittedBookkeeping) == 9152U,
        "BQ adds 9152 named heap bytes; previous AP/AV/BN/BO/BP stays 27938.");
    NCPathCoreCommittedScopeV1 BuildPathCoreCommittedScopeSameThread() const noexcept;
    bool IsPathCoreCommittedNativeConfigCurrentSameThread() noexcept;
    bool ValidatePathCoreCommittedBaseScopeSameThread() noexcept;
    bool ValidatePathCoreCommittedRunSameThread() noexcept;
    void DrainPathCorePendingCommandedCaptureSameThread() noexcept;
    bool ValidatePathCorePublishedCommittedRunSameThread() noexcept;
    void ArmPathCoreCommittedRunSameThread() noexcept;
    void ClosePathCoreCommittedRunSameThread() noexcept;
    void BeginPathCoreCommandedCaptureSameThread(const NCBlock& block,
        NCBlockDispatchId dispatchId) noexcept;
    void CommitPathCoreCommandedCaptureSameThread(NCBlockDispatchId dispatchId,
        const MotionProgramBlockCapture& capture, const NCProgramCommitSnapshot& commit,
        bool commitSucceeded, bool ledgerFound, const NCBlockLifecycleSnapshot& ledger,
        int sourcePC, int sourceLineNumber) noexcept;
    void PreparePathCoreCommittedRunSameThread() noexcept;
    void PublishPathCoreCommittedRunSameThread() noexcept;
    void FlushPathCoreCommittedRunSummarySameThread() noexcept;
    // BQ-END
    static std::uint64_t AllocatePathCoreLiveOwnerTagStartup() noexcept;
    NCPathCoreLiveRetentionScopeV1 BuildPathCoreLiveScopeSameThread() const noexcept;
    void CapturePathCoreLiveNativeConfigSameThread() noexcept;
    bool IsPathCoreLiveNativeConfigCurrentSameThread() noexcept;
    bool ValidatePathCoreLiveRetentionSameThread() noexcept;
    void ArmPathCoreLiveRetentionSameThread() noexcept;
    void FencePathCoreLiveRetentionSameThread(PathCoreLiveFenceReason reason) noexcept;
    void PausePathCoreLiveRetentionSameThread() noexcept;
    void ResumePathCoreLiveRetentionSameThread() noexcept;
    void AdmitPathCoreLiveRetentionSameThread(
        const NCOrdinaryG00InflightRegistrationProof& proof,
        const NCPreparedHeadCutoverContext& context) noexcept;
    void QueuePathCoreLiveSummarySameThread() noexcept;
    void FlushPathCoreLiveSummarySameThread() noexcept;

    // BS: one G171 request uses at most eight native linear-axis slots.
    // Allocate at NCManager construction; request-time resize stays within
    // this startup capacity. No additional retained history or snapshot.
    std::vector<int> m_pathCoreReturnAxes = std::vector<int>(8U, 0);
    std::vector<double> m_pathCoreReturnTargets = std::vector<double>(8U, 0.0);
    struct PathCoreReturnSummary
    {
        std::uint64_t run = 0ULL, dispatch = 0ULL;
        MotionExecutionIdentity source{}, submitted{};
        double fromX = 0.0, targetX = 0.0, rapidPercent = 0.0;
        double newStartX = 0.0, newEndX = 0.0;
        std::uint32_t code = 0U, seamsBefore = 0U;
        // Qualified reads bound the ordinal to 1..32; receipt flags are uint8.
        // Both fit in uint16. Fill the 128-byte summary without tail padding
        // so its value reset does not need a larger aggregate temporary.
        std::uint16_t sourceOrdinal = 0U, sourceFlags = 0U;
        bool pending = false;
        std::uint8_t startByteMask = 0U, startNumericMask = 0U, otherEndMask = 0U;
        std::uint8_t sourceValidMask = 0U, newValidMask = 0U;
        bool outputValid = false, targetExact = false;
        std::uint8_t sourceAxisMask = 0U, newAxisMask = 0U;
        std::uint8_t axisCount = 0U, targetMismatchMask = 0U;
    } m_pathCoreReturnSummary{};
    static_assert(sizeof(PathCoreReturnSummary) == 128U,
        "BS summary remains 128 named heap bytes; two eight-element vectors are separate.");
    // BT: a bounded cursor names a frozen suffix in the same BQ owner.
    // New return receipts remain in BQ but never become this cursor's sources.
    enum class PathCoreReturnCursorState : std::uint8_t
    {
        NEW = 0U, ACTIVE = 1U, PENDING = 2U, EXHAUSTED = 3U, INVALID = 4U
    };
    struct PathCoreReturnCursor
    {
        std::uint64_t run = 0ULL, completedDispatch = 0ULL;
        MotionExecutionIdentity expectedIdentity{};
        std::uint32_t sourceCount = 0U, requested = 0U, remaining = 0U;
        std::uint32_t nextOrdinal = 0U, lowerBound = 0U, expectedCount = 0U;
        std::uint32_t selectedOrdinal = 0U;
        std::uint8_t sourceAxisMask = 0U, sourceValidMask = 0U;
        PathCoreReturnCursorState state = PathCoreReturnCursorState::NEW;
        bool completionReady = false;
        // BU: opt-in P1 retains each source row's programmed axis mask.
        bool mixedAxes = false;
        // BV: one forward traversal may follow a fully completed retreat.
        bool forward = false, forwardAvailable = false;
    } m_pathCoreReturnCursor{};
    static_assert(sizeof(PathCoreReturnCursor) <= 96U,
        "BT cursor is bounded scalar state; it never owns a path snapshot.");
    std::uint16_t m_pathCoreReturnCommand = 171U;
    bool PreparePathCoreReturnCursorSourceSameThread(const NCBlock& block);
    bool CompletePathCoreReturnCursorSameThread();
    bool PreparePathCoreAdvanceCursorSourceSameThread();
    bool CheckPathCoreMixedReturnOutputSameThread() noexcept;
    void InvalidatePathCoreReturnCursorSameThread() noexcept;
    void ObservePathCoreReturnDispatchSameThread(const NCBlock& block) noexcept;
    void LogPathCoreReturnCursorSameThread(const char* phase) const noexcept;
    static bool IsPathCoreReturnBlockShapeValid(const NCBlock& block) noexcept;
    WaitConditionFunc StartPathCoreReturnSameThread(const NCBlock& block);
    void RejectPathCoreReturnSameThread(std::uint32_t code, int alarmCode);
    void FlushPathCoreReturnSummarySameThread() noexcept;

    // BX-FEED-BEGIN: fixed startup-owned G01 state, distinct from V1 G00 history.
    MotionFeedLineWorkspace m_pathFeedMotion{};
    std::vector<int> m_pathFeedAxes = std::vector<int>(3U, 0);
    std::vector<double> m_pathFeedTargets = std::vector<double>(3U, 0.0);
    std::array<double, 8U> m_pathFeedWCS{}, m_pathFeedCandidate{};
    std::array<bool, 8U> m_pathFeedProgrammed{};
    struct PathFeedState
    {
        std::uint64_t run = 0ULL, cache = 0ULL, dispatch = 0ULL, commit = 0ULL;
        MotionFeedbackSequence lastSequence = 0ULL;
        std::uint32_t submitted = 0U, accepted = 0U, started = 0U, done = 0U;
        std::uint32_t rejected = 0U, failed = 0U, code = 0U;
        int sourcePC = -1, sourceLine = 0;
        bool armed = false, pending = false, bound = false, consumerAccepted = false;
        bool consumerStarted = false, completed = false, explicitFeed = false;
    } m_pathFeed{};
    static bool IsPathCoreFeedBlockShapeValid(const NCBlock& block) noexcept;
    bool IsPathCoreFeedInputOmission(const NCBlock& block) const noexcept;
    bool IsPathCoreFeedConfigurationValid() noexcept;
    void ArmPathCoreFeedSameThread() noexcept;
    void InvalidatePathCoreFeedSameThread() noexcept;
    void ValidatePathCoreFeedSameThread();
    void BeginPathCoreFeedCaptureSameThread(const NCBlock& block,
        NCBlockDispatchId dispatchId) noexcept;
    WaitConditionFunc StartPathCoreFeedSameThread(const NCBlock& block);
    void CommitPathCoreFeedCaptureSameThread(NCBlockDispatchId dispatchId,
        const MotionProgramBlockCapture& capture, const NCProgramCommitSnapshot& commit,
        bool committed, bool ledgerFound, const NCBlockLifecycleSnapshot& ledger,
        int sourcePC, int sourceLine);
    void ObservePathCoreFeedFeedbackSameThread(const MotionFeedbackEvent& event,
        bool ledgerAccepted);
    bool CompletePathCoreFeedSameThread();
    void RejectPathCoreFeedSameThread(std::uint32_t code, int alarmCode);
    void FinalizePathCoreFeedSameThread() noexcept;
    void LogPathCoreFeedSameThread(const char* phase) const noexcept;
    void LogPathCoreFeedGeometrySameThread() const noexcept;
    // BX-FEED-END

    // BY-ARC-BEGIN: fixed NC-owned arc workspace, separate from G01 and G00.
    MotionFeedArcWorkspace m_pathArcMotion{};
    MotionCommand m_pathArcCommand{};
    std::array<double, 2U> m_pathArcCenterOffset{};
    std::array<double, 8U> m_pathArcWCS{}, m_pathArcCandidate{};
    std::array<bool, 8U> m_pathArcProgrammed{};
    struct PathArcState
    {
        std::uint64_t run = 0ULL, cache = 0ULL, dispatch = 0ULL, commit = 0ULL;
        MotionFeedbackSequence lastSequence = 0ULL;
        std::uint32_t submitted = 0U, accepted = 0U, started = 0U, done = 0U;
        std::uint32_t rejected = 0U, failed = 0U, code = 0U;
        int sourcePC = -1, sourceLine = 0;
        bool armed = false, pending = false, bound = false, consumerAccepted = false;
        bool consumerStarted = false, completed = false, explicitArc = false;
    } m_pathArc{};
    static bool IsPathCoreArcBlockShapeValid(const NCBlock& block) noexcept;
    bool IsPathCoreArcInputOmission(const NCBlock& block) const noexcept;
    bool IsPathCoreArcConfigurationValid() noexcept;
    void ArmPathCoreArcSameThread() noexcept;
    void InvalidatePathCoreArcSameThread() noexcept;
    void ValidatePathCoreArcSameThread();
    void BeginPathCoreArcCaptureSameThread(const NCBlock& block,
        NCBlockDispatchId dispatchId) noexcept;
    WaitConditionFunc StartPathCoreArcSameThread(const NCBlock& block);
    void CommitPathCoreArcCaptureSameThread(NCBlockDispatchId dispatchId,
        const MotionProgramBlockCapture& capture, const NCProgramCommitSnapshot& commit,
        bool committed, bool ledgerFound, const NCBlockLifecycleSnapshot& ledger,
        int sourcePC, int sourceLine);
    void ObservePathCoreArcFeedbackSameThread(const MotionFeedbackEvent& event,
        bool ledgerAccepted);
    bool CompletePathCoreArcSameThread();
    void RejectPathCoreArcSameThread(std::uint32_t code, int alarmCode);
    void FinalizePathCoreArcSameThread() noexcept;
    void LogPathCoreArcSameThread(const char* phase) const noexcept;
    void LogPathCoreArcGeometrySameThread() const noexcept;
    // BY-ARC-END

    // BZ-REPLAY-BEGIN: 16 immutable completed G01/arc rows and one live receipt.
    NCPathCoreRetainedPath m_pathReplayStore{};
    NCPathCoreRetainedGeometry m_pathReplayGeometry{};
    MotionPathCoreRetainedWorkspace m_pathReplayMotion{};
    MotionCommand m_pathReplayCommand{};
    std::array<double, 8U> m_pathReplayPulsePerMM{}, m_pathReplayRotaryModulo{};
    struct PathReplaySource
    {
        MotionExecutionIdentity identity{};
        std::uint64_t dispatch = 0ULL, commit = 0ULL;
        int sourcePC = -1, sourceLine = 0;
    };
    std::array<PathReplaySource, 16U> m_pathReplaySource{};
    MotionOwnerLease m_pathReplayLease{};
    struct PathReplayState
    {
        std::uint64_t run = 0ULL, cache = 0ULL, dispatch = 0ULL, commit = 0ULL;
        MotionFeedbackSequence lastSequence = 0ULL;
        std::uint32_t submitted = 0U, accepted = 0U, started = 0U, done = 0U;
        std::uint32_t rejected = 0U, failed = 0U, code = 0U;
        std::uint32_t command = 0U, ordinal = 0U;
        int sourcePC = -1, sourceLine = 0;
        double feedMMMin = 0.0;
        double requestedD = 0.0, appliedD = 0.0, startU = 0.0, endU = 0.0;
        bool armed = false, pending = false, bound = false, consumerAccepted = false;
        bool consumerStarted = false, completed = false, reverse = false, explicitReplay = false;
        bool distanceMode = false, capped = false;
    } m_pathReplay{};
    static_assert(sizeof(PathReplaySource) <= 64U, "BZ provenance row storage budget changed.");
    static_assert(sizeof(PathReplayState) <= 144U, "CA live replay state storage budget changed.");
    bool IsPathCoreReplayConfigurationValid() noexcept;
    static bool IsPathCoreReplayBlockShapeValid(const NCBlock& block) noexcept;
    bool IsPathCoreReplayInputOmission(const NCBlock& block) const noexcept;
    void ArmPathCoreReplaySameThread() noexcept;
    void InvalidatePathCoreReplaySameThread() noexcept;
    void ClearPathCoreReplayHistorySameThread() noexcept;
    void ValidatePathCoreReplaySameThread();
    void BeginPathCoreReplayCaptureSameThread(const NCBlock& block, NCBlockDispatchId dispatchId) noexcept;
    void RetainPathCoreFeedSameThread() noexcept;
    void RetainPathCoreArcSameThread() noexcept;
    void AppendPathCoreReplayGeometrySameThread(const MotionExecutionIdentity& identity,
        std::uint32_t validAxisMask, std::uint64_t dispatch, std::uint64_t commit,
        int sourcePC, int sourceLine) noexcept;
    WaitConditionFunc StartPathCoreReplaySameThread(const NCBlock& block);
    void CommitPathCoreReplayCaptureSameThread(NCBlockDispatchId dispatchId,
        const MotionProgramBlockCapture& capture, const NCProgramCommitSnapshot& commit,
        bool committed, bool ledgerFound, const NCBlockLifecycleSnapshot& ledger,
        int sourcePC, int sourceLine);
    void ObservePathCoreReplayFeedbackSameThread(const MotionFeedbackEvent& event, bool ledgerAccepted);
    bool CompletePathCoreReplaySameThread();
    void RejectPathCoreReplaySameThread(std::uint32_t code, int alarmCode);
    void FinalizePathCoreReplaySameThread() noexcept;
    void LogPathCoreReplaySameThread(const char* phase) const noexcept;
    void LogPathCoreReplayGeometrySameThread(std::uint32_t ordinal, bool reverse) const noexcept;
    // BZ-REPLAY-END

    // CG-GAP-BEGIN: explicit simulation, fixed NC-owned data; no RT observer.
    EDMGap::Monitor m_gapInput{};
    struct GapDryRunState
    {
        MotionOwnerLease lease{};
        MotionExecutionEpoch epoch = MOTION_EXECUTION_EPOCH_INVALID;
        EDMGap::Sample sample{};
        std::uint64_t test = 0ULL, run = 0ULL, cache = 0ULL, dispatch = 0ULL;
        std::uint64_t frequency = 0ULL, lastTicks = 0ULL;
        std::uint64_t phaseStartMs = 0ULL, lastServiceMs = 0ULL, sequence = 0ULL;
        std::uint32_t phase = 0U, passedMask = 0U, observations = 0U, phaseSamples = 0U;
        std::uint32_t restarts = 0U, result = 0U; // result: pending / pass / cancelled / fail.
        int sourceLine = 0;
        bool active = false, paused = false;
    } m_gapDryRun{};
    std::uint64_t m_gapDryRunSerial = 0ULL;
    static_assert(sizeof(GapDryRunState) + sizeof(EDMGap::Monitor) <= 512U,
        "CG fixed NC storage budget changed.");
    static bool IsGapDryRunBlockShapeValid(const NCBlock& block) noexcept;
    WaitConditionFunc StartGapDryRunSameThread(const NCBlock& block);
    static bool WaitForGapDryRunCallback(NCManager* nc);
    bool ProcessGapDryRunSameThread();
    bool RestartGapDryRunSameThread();
    bool ReadGapDryRunClockSameThread(std::uint64_t& nowMs) noexcept;
    void ValidateGapDryRunSameThread();
    void PauseGapDryRunSameThread(const char* reason) noexcept;
    void CancelGapDryRunSameThread(const char* reason) noexcept;
    void RejectGapDryRunSameThread(int alarmCode, int line, const char* reason);
    // CG-GAP-END

    // CH uses operator resume; CI admits one recovery; CK repeats after proven returns.
    struct GapPathSimulationState
    {
        std::uint64_t frequency = 0ULL, lastTicks = 0ULL, lastServiceMs = 0ULL;
        std::uint64_t sequence = 0ULL, holdStartMs = 0ULL, firstServiceMs = 0ULL;
        std::uint32_t stalledCalls = 0U;
        bool active = false, clockStarted = false, lowInjected = false, held = false;
        bool recoveryInjected = false, normalLogged = false, recoveryLogged = false, ackLogged = false;
        bool automaticResume = false, waitJ5Logged = false, repeating = false;
        bool lowRetreat = false, returnHold = false;
    } m_gapPath{};
    static_assert(sizeof(GapPathSimulationState) <= 128U,
        "CH/CI/CK simulation must remain fixed NC-owned storage.");
    struct GapServiceDiagnostic
    {
        const char* site = "NOT_SERVICED";
        std::uint64_t nowMs = 0ULL, lastServiceMs = 0ULL, sampledAtMs = 0ULL;
        std::uint64_t sampleSequence = 0ULL, run = 0ULL, dispatch = 0ULL;
        std::uint32_t ageOnlyCalls = 0U;
        EDMGap::Quality quality = EDMGap::Quality::NO_SAMPLE;
        bool present = false, nowValid = false, lastServiceValid = false, publishSample = false;
    } m_gapServiceCurrent{}, m_gapServiceLastFault{};
    std::uint32_t m_gapServiceAgeOnlyCalls = 0U;
    static_assert(2U * sizeof(GapServiceDiagnostic) + sizeof(std::uint32_t) <= 192U,
        "CK service evidence must remain fixed NC-owned storage.");
    void LogGapServiceFaultSameThread() const noexcept;
    bool StartGapPathSimulationSameThread(bool automaticResume = false, bool repeating = false,
        bool lowRetreat = false) noexcept;
    bool IsGapPathAutomaticResumeSignalSameThread() const noexcept;
    bool IsGapPathAutomaticNormalSameThread() const noexcept;
    bool ValidateGapPathAutomaticResumeSameThread() noexcept;
    bool ServiceGapPathSimulationSameThread(double activeS, bool publishSample = true,
        const char* site = "AUTOMATIC") noexcept;
    void RejectGapPathSimulationSameThread(const char* reason) noexcept;
    void LogGapPathSimulationSameThread(const char* phase) const noexcept;

    // CB-HOLD-BEGIN: bounded opt-in excursions inside the next original source.
    struct PathHoldState
    {
        MotionExecutionIdentity identity{};
        MotionOwnerLease lease{};
        std::uint64_t run = 0ULL, cache = 0ULL, dispatch = 0ULL, commit = 0ULL;
        std::uint64_t candidateDispatch = 0ULL, observedTransition = 0ULL, observedSeamCount = 0ULL;
        MotionNCSettleRequestSequence requestedHoldSequence = MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID;
        double distanceMM = 0.0, feedMMMin = 0.0;
        double automaticIntervalMM = 0.0, automaticIntervalPulse = 0.0, automaticNextS = 0.0;
        std::uint64_t automaticObservedReturns = 0ULL, automaticBoundarySequence = 0ULL;
        MotionNCSettleRequestSequence automaticSettleSequence = MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID;
        bool automaticEnabled = false, automaticHoldOwned = false, automaticAdmissionOwned = false;
        int sourcePC = -1, sourceLine = 0;
        std::uint32_t code = 0U, cycleLimit = 1U;
        bool armed = false, bound = false, requested = false, blocked = false;
        bool explicitControl = false, startCommitted = false, crossSegment = false;
        // CL endpoint authorization survives revocation of automatic GAP control.
        bool requireReturnAuthorization = false, returnHoldRequested = false;
    } m_pathHold{};
    // CD: NC-owned immutable handoff scratch. No large automatic view copies.
    MotionPathCoreHoldExcursionView m_pathHoldView{};
    bool PreparePathCoreHoldHistorySameThread() noexcept;
    bool BuildPathCoreHoldViewSameThread(bool line) noexcept;
    // CB FIX1: NC-thread-only last fault survives revocation and rolling logs.
    // Diagnostic storage only; never authorizes Motion or restores a source.
    struct PathHoldLastFault
    {
        MotionPathCoreHoldExcursionSnapshot snapshot{};
        MotionExecutionIdentity identity{};
        MotionOwnerLease lease{};
        std::uint64_t run = 0ULL, dispatch = 0ULL;
        std::uint32_t code = 0U, origin = 0U, repeatCalls = 0U;
        int alarmCode = 0, sourcePC = -1, sourceLine = 0;
        bool present = false, rtValid = false, identityMatch = false, leaseMatch = false;
    } m_pathHoldLastFault{};
    static_assert(sizeof(PathHoldLastFault) <= 320U,
        "CB last-fault diagnostics must remain fixed and bounded.");
    static bool IsPathCoreHoldBlockShapeValid(const NCBlock& block) noexcept;
    bool IsPathCoreHoldInputOmission(const NCBlock& block) const noexcept;
    WaitConditionFunc StartPathCoreHoldSameThread(const NCBlock& block);
    void BeginPathCoreHoldCaptureSameThread(const NCBlock& block,
        NCBlockDispatchId dispatchId) noexcept;
    void CommitPathCoreHoldCaptureSameThread(NCBlockDispatchId dispatchId);
    void InvalidatePathCoreHoldSameThread(bool cancelMotion = true) noexcept;
    void FeedHoldInternal();
    void CancelPathCoreHoldAutomaticSameThread(const char* reason) noexcept;
    bool ProcessPathCoreHoldAutomaticSameThread() noexcept;
    bool ProcessPathCoreReturnWaitSameThread() noexcept;
    void LogPathCoreHoldAutomaticSameThread(const char* phase) const noexcept;
    void ObservePathCoreHoldSameThread();
    bool PreparePathCoreHoldResumeSameThread(bool gateControlled) noexcept;
    bool CommitPathCoreHoldResumeSameThread() noexcept;
    void RejectPathCoreHoldSameThread(std::uint32_t code, int alarmCode,
        const MotionPathCoreHoldExcursionSnapshot* observed = nullptr);
    void CapturePathCoreHoldFaultSameThread(std::uint32_t code, int alarmCode,
        std::uint32_t origin, const MotionPathCoreHoldExcursionSnapshot* observed = nullptr) noexcept;
    void LogPathCoreHoldLastFaultSameThread() const noexcept;
    void LogPathCoreHoldSameThread(const char* phase) const noexcept;
    // CB-HOLD-END


    // L.2AO: borrowed D/F pairs end before the downstream J->AF/readback chain.
    void ObservePathCoreCommittedLinkAndSegmentSameThread() noexcept;
    void ObservePathCoreLinkedSegmentRunBoundarySameThread() noexcept;
    // L.2AN: reference-only D geometry observation with its own frame.
    void ObservePathCoreAcceptedGeometrySameThread(
        const NCOrdinaryG00InflightRegistrationProof& proof,
        const NCPreparedHeadCutoverContext& context) noexcept;
    // L.2AQ: one reviewed mapping for AI/AJ; borrowed view, not current proof.
    NCPathCoreFrontierOwnersSameThread BorrowPathCoreFrontierOwnersSameThread() const noexcept;
    // L.2AN: scoped six-owner view, independent of the readback output frame.
    void SamplePathCoreCurrentFrontierSameThread() noexcept;
    // Isolated noinline implementation: one bounded 112-byte CALLER output,
    // disjoint from the manager as AJ requires. No source snapshot is made.
    void ProbePathCoreLastSampleReadbackSameThread() noexcept;
};
