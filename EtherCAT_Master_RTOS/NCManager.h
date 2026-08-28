#pragma once
#include "NC_Types.h"
#include "MotionCore.h"
#include "HomingManager.h"
#include "CoordinateManager.h"
#include "MacroEngine.h"       // 必須要有
#include "MacroParser.h"       // 必須要有
#include "GCodeParser.h"       // 🌟 解決 Parser 找不到的關鍵！
#include "NCProgramCache.h"    // Stage NC-0.2C：Parsed Program Cache
#include "NCBlockLifecycleLedger.h" // Stage NC-0.2D：Block / Motion Lifecycle
#include "NCBlockCompletionBoundary.h" // Stage NC-0.2F：Motion Completion Dual-Key Guard
#include "NCProgramEndBoundary.h" // Stage NC-0.2G：Program End / Cycle End Gate
#include "NCSingleBlockBoundary.h" // Stage NC-0.2I.1：Single Block Shadow Boundary
#include "NCFeedHoldBoundary.h" // Stage NC-0.2I.2：Feed Hold Request/Ack Shadow

#include <queue>
#include <vector>
#include <string>
#include <map>                 // Parsed Macro Cache 使用穩定節點位址
#include <stack>               // 🌟 新增：為了支援副程式返回堆疊
#include <cstdint>
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

class NCManager {
public:
    NCManager(MotionCore& motion);

    // 1. 系統狀態控制
    void ChangeMode(NCOperationMode newMode);
    void ChangeState(NCState newState);

    // =========================================================
// NC State Query
//
// 提供給外部控制層（例如 NCPLCManager）判斷
// NC 當前狀態。
// =========================================================
    NCState GetState() const
    {
        return m_state;
    }

    NCOperationMode GetMode() const
    {
        return m_mode;
    }

    // 🌟 新增功能：載入 NC 程式檔
    bool LoadProgram(const std::string& filepath);

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

    void SetSingleBlockEnabled(bool enabled)
    {
        if (m_isSingleBlockEnabled == enabled)
        {
            return;
        }

        m_isSingleBlockEnabled = enabled;
        if (!enabled)
        {
            m_legacySingleBlockPausePending = false;
            m_singleBlockBoundaryShadow.Cancel(false);
        }
    }

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

    static bool WaitForGMBlockTransactionCallback(NCManager* nc);
    static bool WaitAndHoldCallback(NCManager* nc);
    static bool WaitAndClearQueueCallback(NCManager* nc);
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

    // Stage NC-0.2D：Program Commit 與 Motion Segment Feedback 的對照表。
    NCBlockLifecycleLedger m_blockLifecycleLedger{};

    // Stage NC-0.2F：已追蹤 Motion Block 的 Wait Callback 採 Dual-Key
    // Guard；非 Motion Callback 維持 Legacy 行為。
    NCBlockCompletionBoundaryObserver m_blockCompletionBoundaryObserver{};
    NCBlockDispatchId m_waitingBlockDispatchId =
        NC_BLOCK_DISPATCH_ID_INVALID;

    // Stage NC-0.2G：M02 / M30 / Natural EOF 共用同一個 Cycle End Gate。
    NCProgramEndBoundary m_programEndBoundary{};
    bool m_programEndAlarmRaised = false;

    struct NCGMBlockTransactionState
    {
        NCGMBlockTransactionSnapshot snapshot{};
        WaitConditionFunc gCallback = nullptr;
        WaitConditionFunc mCallback = nullptr;
    };

    NCGMBlockTransactionState m_gmBlockTransaction{};
    NCGMBlockTransactionCounters m_gmBlockTransactionCounters{};
    std::uint64_t m_nextGMBlockTransactionSequence = 1ULL;

    // Stage NC-0.2I.1：只觀察 Single Block 正確完成點，不改變控制。
    NCSingleBlockBoundaryShadow m_singleBlockBoundaryShadow{};
    bool m_legacySingleBlockPausePending = false;


    // Stage NC-0.2I.2：區分 Feed Hold Request、Legacy HOLD 顯示與
    // 命令／實際速度真正停止 Acknowledge；本階段仍不接管控制。
    NCFeedHoldBoundaryShadowObserver m_feedHoldBoundaryShadow{};

    MotionFeedbackEvent m_lastMotionFeedback{};
    MotionExecutionIdentity m_lastAcceptedMotionIdentity{};
    MotionExecutionIdentity m_lastStartedMotionIdentity{};
    MotionExecutionIdentity m_lastCompletedMotionIdentity{};
    MotionExecutionIdentity m_lastRejectedMotionIdentity{};
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

    static NCSingleBlockCandidateKind ClassifySingleBlockCandidate(
        const NCBlock& block) noexcept;
    void ArmSingleBlockShadow(
        NCSingleBlockCandidateKind candidateKind,
        NCBlockDispatchId dispatchId,
        const NCProgramCommitSnapshot& target,
        int sourceLineNumber) noexcept;
    void EvaluateSingleBlockShadow(bool callbackComplete) noexcept;
    void ObserveLegacySingleBlockHold() noexcept;
    void CancelSingleBlockShadow(bool superseded) noexcept;


    NCFeedHoldBoundarySample BuildFeedHoldBoundarySample() const noexcept;
    void BeginFeedHoldBoundaryShadow(NCFeedHoldSource source) noexcept;
    void ObserveFeedHoldBoundaryShadow() noexcept;
    void ObserveFeedHoldLegacyHoldShadow() noexcept;
    void ObserveFeedHoldResumeRequestedShadow() noexcept;
    void ObserveFeedHoldResumeAppliedShadow() noexcept;
    void CancelFeedHoldBoundaryShadow(bool superseded) noexcept;

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
    NCState m_state = NCState::NOT_READY;
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




};