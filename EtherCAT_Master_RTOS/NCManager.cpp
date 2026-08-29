#include "NCManager.h"
#include "MacroEngine.h"
#include "MacroParser.h"
#include "GCodeParser.h"
#include "NCGCodeSemantics.h"
#include "NCExpressionResolver.h"
#include "NCProgramCache.h"
#include "NCBlockLifecycleLedger.h"
#include "NCBlockCompletionBoundary.h"
#include "NCProgramEndBoundary.h"
#include "GlobalConfig.h" // 如果你有用到 DEBUG_PRINT 等功能
#include "AlarmManager.h"
#include "GMCodeHandlers.h" // 🌟 引入 G 碼處理器總表
#include <fstream>
#include <iostream>
#include <sstream>
#include <limits>
#include <utility>
#include <cmath>
namespace GCodeHandlers
{
    WaitConditionFunc Handle_G81(const NCBlock& block, NCManager* nc);
}

namespace
{
    MotionCommandSource GetMotionCommandSourceForMode(
        NCOperationMode mode)
    {
        switch (mode)
        {
        case NCOperationMode::MEMORY:
            return MotionCommandSource::NC_MEMORY;

        case NCOperationMode::MDI:
            return MotionCommandSource::NC_MDI;

        case NCOperationMode::MANUAL:
            return MotionCommandSource::NC_MANUAL_AUTO;

        case NCOperationMode::EDIT:
        default:
            return MotionCommandSource::UNKNOWN;
        }
    }


    MotionOwner GetMotionOwnerForMode(
        NCOperationMode mode)
    {
        return
            ResolveMotionOwnerForSource(
                GetMotionCommandSourceForMode(mode));
    }


    bool TryGetPositiveIntegerAddress(
        const NCBlock& block,
        char address,
        int& value) noexcept
    {
        if (!block.has(address))
        {
            return false;
        }

        const double rawValue = block.val(address);
        if (!std::isfinite(rawValue))
        {
            return false;
        }

        const double roundedValue = std::round(rawValue);
        if (std::fabs(rawValue - roundedValue) > 1.0e-9 ||
            roundedValue < 1.0 ||
            roundedValue >
            static_cast<double>(
                (std::numeric_limits<int>::max)()))
        {
            return false;
        }

        value = static_cast<int>(roundedValue);
        return true;
    }
}


bool NCManager::AcquireProgramMotionOwner() noexcept
{
    const MotionOwner requestedOwner =
        GetMotionOwnerForMode(m_mode);

    if (requestedOwner == MotionOwner::NONE)
    {
        return false;
    }

    if (m_programMotionLease.owner == requestedOwner &&
        m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease))
    {
        return true;
    }

    m_programMotionLease = MotionOwnerLease{};

    return
        m_motion.TryAcquireMotionOwner(
            requestedOwner,
            m_programMotionLease);
}


void NCManager::ReleaseProgramMotionOwner() noexcept
{
    if (m_programMotionLease.IsValid())
    {
        // Generation 不符時 Release 會安全失敗，不會釋放新 Owner。
        m_motion.ReleaseMotionOwner(
            m_programMotionLease);
    }

    m_programMotionLease = MotionOwnerLease{};
}


bool NCManager::AdoptProgramMotionLease(
    const MotionOwnerLease& lease) noexcept
{
    const MotionOwner expectedOwner =
        GetMotionOwnerForMode(m_mode);

    if (expectedOwner == MotionOwner::NONE ||
        lease.owner != expectedOwner ||
        !m_motion.IsMotionOwnerLeaseCurrent(lease))
    {
        return false;
    }

    m_programMotionLease = lease;
    return true;
}


NCManager::NCManager(MotionCore& motion) : m_motion(motion), MathParser(MacroSys), Parser()
{
    // =========================================================
    // G81 HOME Manager Link
    //
    // NCManager 建立完成後，將自己交給 HomingManager。
    //
    // 目前只建立 NC <-> HomingManager 連結，
    // 不會啟動任何 HOME Motion。
    // =========================================================

    Homing.LinkNCManager(this);

    // 初始化軸名稱為空白字元 (防呆)
    for (int i = 0; i < 8; i++) {
        m_axisNames[i] = ' ';
    }

    // 🌟 開機立刻載入軸定義檔！
    LoadAxisConfiguration();

    // 初始化設定
    m_state = NCState::IDLE;
    m_mode = NCOperationMode::MEMORY; // 預設記憶體模式

    m_motion.SetPendingCommandSource(
        MotionCommandSource::NC_MEMORY);
}

bool NCManager::LoadProgram(const std::string& filepath)
{
    // NC-0.2J.3：Reset release 尚未完成時不得以 Program Replace
    // 發布新 Epoch 或提早把 NC 狀態改回 READY。
    if (m_state == NCState::RESET_STATE)
    {
        return false;
    }

    std::ifstream file(filepath);
    if (!file.is_open())
    {
        return false;
    }

    std::vector<std::string> rawLines;
    std::string line;
    while (std::getline(file, line))
    {
        rawLines.push_back(line);
    }
    file.close();

    // Build the new immutable image first. A failed build must not destroy
    // the currently loaded program or its execution boundary.
    NCProgramCache newProgramCache;
    if (!newProgramCache.Build(std::move(rawLines), Parser))
    {
        return false;
    }

    // Only after a complete image exists do we invalidate the old execution.
    BeginLifecycleInterruptionShadow(
        NCLifecycleInterruptionCause::PROGRAM_REPLACED,
        true);
    CancelProgramEndBoundary();
    ClearCompletionWaitBoundary(true);
    CancelGMBlockTransaction(true);
    CancelSingleBlockShadow(true);
    CancelFeedHoldBoundaryShadow(true);
    m_waitCallback = nullptr;
    ReleaseProgramMotionOwner();
    const MotionExecutionEpoch replacementEpoch =
        m_motion.BeginNewExecutionEpoch(
            MotionCommandSource::NC_MEMORY);
    RecordLifecycleInterruptionEpochPublished(replacementEpoch);

    const std::size_t pos = filepath.find_last_of("/\\");
    m_mainProgramName =
        pos != std::string::npos
        ? filepath.substr(pos + 1U)
        : filepath;

    m_macroStack.clear();
    m_macroProgramCaches.clear();
    MacroSys.Reset();

    m_macroProgramName = "";
    m_macroProgramPC = -1;

    m_programCache = std::move(newProgramCache);
    m_programPC = 0;
    ResetAllProgramCommitBoundaries();
    m_motion.ResetPhysicalPC();

    // 🌟 取得大腦目前的狀態，並同步給馬達標籤機
    int currentBrainWCS = CoordSys.GetCurrentWCSGCode();
    int currentBrainToolMode = CoordSys.toolLengthMode;
    int currentBrainHCode = CoordSys.currentHCode;
    int currentBraintoolRadiusMode = CoordSys.toolRadiusMode;
    int currentBraintoolDCode = CoordSys.currentDCode;
    bool curIsAbs = CoordSys.isAbsoluteMode;
    bool curG68 = CoordSys.isG68Active;
    double curG68Angle = CoordSys.g68Angle; // 讀取你存的 R 參數角度
    bool curG168 = CoordSys.isWorkpieceRotationActive; // 讀取你原本寫好的狀態
    int curWCode = CoordSys.currentWCode; // 讀取你存的 W 碼
    bool curG51 = CoordSys.isScalingActive;
    double curScale = CoordSys.scaleFactor;

    uint8_t curMirrorMask = 0;
    for (int i = 0; i < 8; i++) {
        if (CoordSys.isMirrorActive[i]) {
            curMirrorMask |= (1 << i); // 如果這軸有鏡像，就把對應的 bit 設為 1
        }
    }

    // 🌟 讀取大腦的極座標狀態 (你原本應該就有這個變數)
    bool curG16 = CoordSys.isPolarCoordinateActive;

    // 🌟 讀取大腦的狀態 (變數名稱請對應你的 CoordSys)
    bool curG162 = CoordSys.isCAxisOffsetRotationEnabled;
    int curPlane = CoordSys.activePlane; // 17, 18 或是 19

    // 🌟 拿大腦最乾淨的狀態強制洗掉馬達的殘影
    m_motion.ResetPhysicalTags(
        currentBrainWCS,
        currentBrainToolMode,
        currentBrainHCode,
        currentBraintoolRadiusMode,
        currentBraintoolDCode,
        curIsAbs,
        curG68,
        curG68Angle,
        curG168,
        curWCode,
        curG51,
        curScale,
        curMirrorMask,
        curG16,
        curG162,
        curPlane);


    m_state = NCState::READY;
    return true;
}

void NCManager::ChangeMode(NCOperationMode newMode)
{
    // 只有在 IDLE 或 READY 狀態才能切換模式
    if (m_state == NCState::IDLE || m_state == NCState::READY || m_state == NCState::P_END) {
        CancelProgramEndBoundary();
        ClearCompletionWaitBoundary(true);
        CancelGMBlockTransaction(true);
        CancelSingleBlockShadow(true);
        CancelFeedHoldBoundaryShadow(true);
        m_waitCallback = nullptr;
        ReleaseProgramMotionOwner();
        m_mode = newMode;

        m_motion.SetPendingCommandSource(
            GetMotionCommandSourceForMode(m_mode));
    }

    if (m_state == NCState::P_END)
    {
        Reset();
    }
}

void NCManager::ChangeState(NCState newState) {
    m_state = newState;
}

// ==========================================
// 🌟 升級版 CycleStart (支援 M30 P_END 乾淨重啟)
// ==========================================
void NCManager::CycleStart()
{
    // =========================================================
    // G81 HOME Resume
    //
    // HOME Feed Hold 不再 Cancel Request。
    // m_active 保持 true，G81 Wait Callback 仍卡在原行。
    // =========================================================

    if (m_state == NCState::HOLD &&
        Homing.IsActive())
    {
        const bool resumeAccepted =
            Homing.Resume();

        if (!resumeAccepted)
        {
            return;
        }

        ObserveFeedHoldResumeRequestedShadow();


        // 已完全 PAUSED 時 Resume() 會立即恢復 RUNNING。
        // 若仍在 HOLD_DECEL_STOP，Resume Request 先排隊，
        // HomingManager 會在真正停妥後把 NC 切回 RUN。
        if (!Homing.IsHoldDecelerating())
        {
            m_state =
                NCState::RUN;
            ObserveFeedHoldResumeAppliedShadow();
        }

        m_pauseAfterBlock =
            false;

        return;
    }


    // =========================================================
    // NC-0.2I.4 Controlled Single Block HOLD Resume
    //
    // 只有由 Completion-Gated Single Block 建立的 HOLD 才走此路徑。
    // Feed Hold、M00/M01 與 HOME 的 Resume 語意保持完全分離。
    // =========================================================
    if (m_state == NCState::HOLD &&
        m_singleBlockHoldGate.IsHoldApplied())
    {
        (void)ApplyControlledSingleBlockResume();
        return;
    }


    // =========================================================
    // 一般 NC Program HOLD Resume
    //
    // NC-0.2I.3：只有真正由 PROGRAM Feed Hold 建立的 HOLD 才進入
    // ACK Gate。M00/M01、Single Block 及其他 HOLD 原因仍走 Legacy。
    // =========================================================
    if (m_state == NCState::HOLD)
    {
        const bool programFeedHoldCandidate =
            IsProgramFeedHoldResumeCandidate();

        // Shadow Observer 仍保留完整 Request/Ack/Resume 證據。
        // 非 Feed Hold HOLD 時這個呼叫會被 Observer 安全忽略。
        ObserveFeedHoldResumeRequestedShadow();

        if (programFeedHoldCandidate)
        {
            const NCFeedHoldResumeGateRequestResult gateResult =
                m_feedHoldResumeGate.RequestResume(
                    m_feedHoldBoundaryShadow.GetSnapshot());

            if (gateResult ==
                NCFeedHoldResumeGateRequestResult::DEFERRED ||
                gateResult ==
                NCFeedHoldResumeGateRequestResult::BLOCKED)
            {
                // ACK 前只鎖存 Cycle Start。保持 HOLD、Override=0、
                // Callback/PC/Queue 全部原封不動。
                m_pauseAfterBlock = false;
                return;
            }

            if (gateResult ==
                NCFeedHoldResumeGateRequestResult::APPLY_NOW)
            {
                // 已 ACK：同一個 Gate 立即套用 Resume。
                (void)ApplyProgramHoldResume(true);
                return;
            }

            // BYPASS_LEGACY 只可能發生在 Runtime 回退或 Boundary 已
            // 不再是 PROGRAM Feed Hold，沿用既有 Resume 行為。
        }

        (void)ApplyProgramHoldResume(false);
        return;
    }


    // 正常 READY 或 P_END：全新啟動。
    if (m_state == NCState::READY ||
        m_state == NCState::P_END)
    {
        // NC-0.2J.5.3: a second button edge while the exact same start is
        // waiting for its RT Epoch acknowledgement is idempotent.  Publishing
        // another Epoch here would make the first pending start superseded and
        // can create an endless START_DIRTY loop under repeated HMI polling.
        if (m_programRunStartPending)
        {
            return;
        }

        CancelSingleBlockShadow(false);
        CancelFeedHoldBoundaryShadow(false);
        m_legacySingleBlockPausePending = false;

        if (!AcquireProgramMotionOwner())
        {
            return;
        }

        // READY / P_END 都代表一個新的 Program Run。上一輪的
        // Semantic Commit Boundary 不可被新 Execution Epoch 沿用。
        ResetActiveProgramCommitBoundary();

        if (m_state == NCState::P_END)
        {
            GetBasePC() =
                0;

            Reset_Gode();
            m_macroStack.clear();
        }


        m_pauseAfterBlock =
            false;

        // READY / P_END 的 Cycle Start 是全新的執行世代；
        // HOLD Resume 不會走到這裡，所以不會誤殺暫停中的路徑。
        const MotionExecutionEpoch executionEpoch =
            m_motion.BeginNewExecutionEpoch(
                GetMotionCommandSourceForMode(m_mode));

        // Do not classify this run until the 250 us consumer has observed the
        // exact Epoch publication.  HasPendingSafetyOrRecoveryRequests() must
        // continue to include Epoch PENDING for Reset/Stop/Fault fail-closed
        // protection, so the supervisory side waits instead of weakening the
        // admission predicate.
        m_pendingProgramRunExecutionEpoch = executionEpoch;
        m_pendingProgramRunOwnerLease = m_programMotionLease;
        m_pendingProgramRunMode = m_mode;
        m_pendingProgramRunOriginState = m_state;
        m_pendingProgramRunScope = GetBaseProgramScope();
        m_pendingProgramRunCacheGeneration =
            GetBaseProgramCache().GetGeneration();
        m_programRunStartPending = true;
    }
}

void NCManager::FeedHold()
{
    // =========================================================
    // G81 HOME Feed Hold
    //
    // Hold：可 Resume。
    // Reset / Alarm：不可 Resume。
    // =========================================================

    if (Homing.IsActive())
    {
        if (Homing.RequestHold())
        {
            if (!m_feedHoldBoundaryShadow.IsActive())
            {
                // HOME owns its stop/PAUSED contract.  It must not inherit a
                // PROGRAM Feed Hold RT settle request from an older run.
                m_feedHoldNCSettleRequestSequence =
                    MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID;
                BeginFeedHoldBoundaryShadow(
                    NCFeedHoldSource::HOME);
            }

            m_state =
                NCState::HOLD;
            ObserveFeedHoldLegacyHoldShadow();
        }

        return;
    }


    if (m_state == NCState::RUN)
    {
        // NC-0.2J.5: arm one fresh RT proof before publishing the PROGRAM
        // Feed Hold boundary.  The observer will only accept a settled proof
        // carrying this exact request sequence, Epoch and Program lease.
        m_feedHoldNCSettleRequestSequence =
            m_motion.RequestFeedHoldNCSettle(
                m_motion.GetCurrentExecutionEpoch(),
                m_programMotionLease);

        BeginFeedHoldBoundaryShadow(
            NCFeedHoldSource::PROGRAM);

        m_state =
            NCState::HOLD;

        m_motion.SetGroupFeedrateOverride(
            0.0);

        m_pauseAfterBlock =
            false;

        ObserveFeedHoldLegacyHoldShadow();
    }
}

void NCManager::Reset()
{
    // Stage NC-0.2J.5.1：Reset safety batch 尚未完成時，重複 Reset 必須
    // 保持冪等。只有 release gate 已進入 terminal BLOCKED，操作員再次
    // 明確按 Reset 才建立新的 Epoch / request / gate transaction。
    if (m_state == NCState::RESET_STATE)
    {
        const NCResetReleaseGateSnapshot resetGate =
            m_resetReleaseGate.GetSnapshot();
        const bool terminalBlockedReset =
            !resetGate.active &&
            resetGate.blocked &&
            resetGate.phase == NCResetReleaseGatePhase::BLOCKED;
        if (!terminalBlockedReset)
        {
            return;
        }
    }

    m_resetNCSettleRequestSequence =
        MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID;

    BeginLifecycleInterruptionShadow(
        NCLifecycleInterruptionCause::RESET,
        true);

    CancelProgramEndBoundary();
    if (Homing.IsActive()) Homing.Cancel();

    // Safety 取得新的 Generation，讓舊 AUTO / MDI 命令與晚到 Release 失效。
    m_safetyMotionLease =
        m_motion.TakeSafetyMotionOwner();

    m_programMotionLease =
        MotionOwnerLease{};

    // Stage NC-0.1B：先切換 Epoch。即使舊 Producer 晚一步派單，
    // 250 us Motion Runtime 也會依 Epoch 拒絕載入。
    const MotionExecutionEpoch resetEpoch =
        m_motion.BeginNewExecutionEpoch(
            MotionCommandSource::SAFETY);
    RecordLifecycleInterruptionEpochPublished(resetEpoch);


    //重置馬達區塊--------------------------------------------------
    const bool resetNeedsFaultOrEstopRecovery =
        m_motion.IsAnyAxisFaulted() ||
        m_motion.IsGroupFaulted() ||
        m_motion.IsGroupEmergencyStopped();

    // Stage NC-0.2J.2：Reset 是唯一的頂層 Epoch owner。
    // ResetAllFaults + controlled Stop 由 250 us Runtime 當成同一個
    // correlated safety batch 執行，不允許每個 leaf 再各自發布 Epoch。
    m_motion.RequestResetSafetyBatch(
        resetEpoch,
        resetNeedsFaultOrEstopRecovery);

    // 🌟 [新增] 如果有放電跳刀/排渣，必須強制解鎖跳刀狀態機！
    // m_motion.ResetAllFaults(); // (如果您有寫清除跳刀狀態的 API，建議在這裡呼叫)

    // ==========================================================
    // 2. 【順序修正】：先洗乾淨大腦的 G 碼與 M 碼！
    // ==========================================================
    Reset_Gode();       // 🌟 必須先執行！將 G90, G49, G50, 平面等全部洗回預設值

    // 🌟 [強烈建議新增]：通知 PLC 關閉主軸與切削水 (相當於執行 M05, M09)
    // PLCManager::GetInstance().SetSpindleStop();
    // PLCManager::GetInstance().SetCoolantOff();

    // 🌟 2. 取得大腦洗乾淨後的 3 大狀態
    int currentBrainWCS = CoordSys.GetCurrentWCSGCode();
    int currentBrainToolMode = CoordSys.toolLengthMode;
    int currentBrainHCode = CoordSys.currentHCode;
    int currentBraintoolRadiusMode = CoordSys.toolRadiusMode;
    int currentBraintoolDCode = CoordSys.currentDCode;
    bool curIsAbs = CoordSys.isAbsoluteMode;
    // 🌟 讀取大腦的 G68 狀態 (假設你在 CoordSys 有這個變數)
    bool curG68 = CoordSys.isG68Active;
    double curG68Angle = CoordSys.g68Angle; // 讀取你存的 R 參數角度
    bool curG168 = CoordSys.isWorkpieceRotationActive; // 讀取你原本寫好的狀態
    int curWCode = CoordSys.currentWCode; // 讀取你存的 W 碼
    bool curG51 = CoordSys.isScalingActive;
    double curScale = CoordSys.scaleFactor;

    uint8_t curMirrorMask = 0;
    for (int i = 0; i < 8; i++) {
        if (CoordSys.isMirrorActive[i]) {
            curMirrorMask |= (1 << i); // 如果這軸有鏡像，就把對應的 bit 設為 1
        }
    }
    // 🌟 讀取大腦的極座標狀態 (你原本應該就有這個變數)
    bool curG16 = CoordSys.isPolarCoordinateActive;

    // 🌟 讀取大腦的狀態 (變數名稱請對應你的 CoordSys)
    bool curG162 = CoordSys.isCAxisOffsetRotationEnabled;
    int curPlane = CoordSys.activePlane; // 17, 18 或是 19

    // NC-0.2J.5: NC only captures the clean post-Reset modal image.  The
    // 250 us Motion owner applies both these physical tags and the actual-
    // position rebase as one correlated transaction; the 10 ms task no
    // longer writes RT-owned Group/axis state directly.
    MotionNCResetExecutionState resetExecutionState{};
    resetExecutionState.physicalExecutionPC = 0;
    resetExecutionState.physicalExecutionWCS = currentBrainWCS;
    resetExecutionState.physicalToolMode = currentBrainToolMode;
    resetExecutionState.physicalHCode = currentBrainHCode;
    resetExecutionState.physicalToolRadiusMode =
        currentBraintoolRadiusMode;
    resetExecutionState.physicalDCode = currentBraintoolDCode;
    resetExecutionState.physicalWCode = curWCode;
    resetExecutionState.physicalPlaneMode = curPlane;
    resetExecutionState.physicalMirrorMask = curMirrorMask;
    resetExecutionState.physicalIsAbsoluteMode = curIsAbs;
    resetExecutionState.physicalG68Active = curG68;
    resetExecutionState.physicalG168Active = curG168;
    resetExecutionState.physicalG51Active = curG51;
    resetExecutionState.physicalG16Active = curG16;
    resetExecutionState.physicalG162Active = curG162;
    resetExecutionState.physicalG68Angle = curG68Angle;
    resetExecutionState.physicalScaleRatio = curScale;

    // Producer-side pending tags belong to the 10 ms NC producer and may be
    // updated here.  Current-execution tags remain exclusively RT-owned and
    // are applied later by the correlated rebase transaction.
    m_motion.SetPendingResetExecutionState(
        resetExecutionState);

    // Stage NC-0.2J.6.2：上面的 bool 表示「本次 Reset 需要 RT 執行
    // Fault / ESTOP recovery」，不是 settle/rebase 永久不支援。250 us
    // Runtime 會先 ApplyPendingSafetyAndRecoveryRequests()，再消費這筆
    // settle request；後續 proof 仍會 fail-closed 檢查實際 Fault / ESTOP、
    // pending safety work、Epoch、Owner、軸狀態與速度。若把 recovery 需求
    // 傳成 unsupported，第一次 Reset 即使已清除 ESTOP 也會被永久 BLOCKED，
    // 操作員便被迫再按一次 Reset 才能建立可接受的新交易。
    constexpr bool resetRebaseUnsupported = false;

    m_resetNCSettleRequestSequence =
        m_motion.RequestResetNCSettleAndRebase(
            resetEpoch,
            m_safetyMotionLease,
            resetExecutionState,
            resetRebaseUnsupported);

    // The release gate is armed only after the exact RT transaction sequence
    // exists.  A zero/rejected request therefore remains fail-closed in
    // RESET_STATE and cannot fall back to legacy standstill.
    m_resetReleaseGate.Arm(
        m_lifecycleInterruptionShadow.GetSnapshot(),
        resetEpoch,
        m_safetyMotionLease,
        m_resetNCSettleRequestSequence,
        m_motion.IsMotionOwnerLeaseCurrent(m_safetyMotionLease));

    //m_motion.EmergencyStopGroup();//急停
    //m_motion.ResetAllFaults();//軸清除錯誤

    // 🌟 清理完成後刷新變數
    UpdateSystemVariables();

    //重置NC區塊--------------------------------------------------
    MacroSys.Reset();//重置Macro變數

    m_macroStack.clear();
    m_macroProgramCaches.clear();
    m_programPC = 0;
    m_macroProgramName = "";
    m_macroProgramPC = -1;

    //清空 MDI 與 MANUAL 執行狀態
    m_mdiPC = 0;
    m_manualPC = 0;
    m_manualAutoRunning = false;
    ResetAllProgramCommitBoundaries();

    // 🌟 [新增]：清理我們為了單步與暫停所加的防暴衝旗標
    m_pauseAfterBlock = false;
    m_programChanged = false;
    ClearCompletionWaitBoundary(true);
    CancelGMBlockTransaction(true);
    CancelSingleBlockShadow(true);
    CancelFeedHoldBoundaryShadow(true);
    m_waitCallback = nullptr;


    std::queue<NCBlock> empty;
    std::swap(m_blockQueue, empty);
    m_waitCallback = nullptr;



    //重置Alarm--------------------------------------------------
    AlarmManager::GetInstance().Clear();


    m_state = NCState::RESET_STATE;





}
void NCManager::Reset_Gode()       // 重置G碼相關
{
    GCodeHandlers::Reset_G04(this);
    CoordSys.Set_G90G91(90, this);//重置G90 絕對模式
    CoordSys.CancelToolLengthCompensation(this);//取消刀常補正
    CoordSys.CancelWorkpieceRotation(this);//工件補償取消
    CoordSys.SetActivePlane(17, this);//平面選擇
    CoordSys.isCAxisOffsetRotationEnabled = true;//C 軸電極偏心旋轉補償
    CoordSys.CancelScaling(this);//關閉縮放功能
    bool hasAxis[8] = { false };
    CoordSys.CancelMirror(hasAxis, this);//關閉鏡像功能
    CoordSys.CancelPolarCoordinate(this);//關閉極座標
    CoordSys.CancelToolRadiusCompensation(this);//關閉刀徑補償

    m_isG66Active = false; // 🌟 Reset 必須強制取消 G66
}

// ==========================================
// 🌟 1. 標準且安全的實作呼叫副程式邏輯
// ==========================================
bool NCManager::CallMacro(const std::string& filename)
{
    if (MacroSys.PushCallStack() == false)
    {
        AlarmManager::GetInstance().Trigger(AlarmManager::MACRO_OVERFLOW);
        m_state = NCState::HOLD;
        return false;
    }

    auto cacheIt = m_macroProgramCaches.find(filename);
    if (cacheIt == m_macroProgramCaches.end())
    {
        std::string macroDir = GlobalConfig::GetInstance().NCMacroProgramDir;
        if (!macroDir.empty() && macroDir.back() != '/' && macroDir.back() != '\\')
        {
            macroDir += "/";
        }

        const std::string fullPath = macroDir + filename;
        std::ifstream file(fullPath);
        if (!file.is_open())
        {
            AlarmManager::GetInstance().Trigger(AlarmManager::Macro_File_Not_Found);
            MacroSys.PopCallStack();
            m_state = NCState::HOLD;
            return false;
        }

        std::vector<std::string> rawLines;
        std::string line;
        while (std::getline(file, line))
        {
            rawLines.push_back(line);
        }
        file.close();

        NCProgramCache cache;
        if (!cache.Build(std::move(rawLines), Parser))
        {
            MacroSys.PopCallStack();
            AlarmManager::GetInstance().Trigger(AlarmManager::SYNTAX_ERROR);
            m_state = NCState::HOLD;
            return false;
        }

        cacheIt = m_macroProgramCaches.emplace(filename, std::move(cache)).first;
    }

    MacroFrame newFrame{};
    newFrame.programName = filename;
    newFrame.program = &cacheIt->second;
    newFrame.frameId = AllocateMacroFrameId();
    newFrame.currentPC = 0;
    newFrame.committedPC = -1;
    newFrame.returnPC = m_macroStack.empty()
        ? (GetBasePC() + 1)
        : (m_macroStack.back().currentPC + 1);
    newFrame.repeatCount = 1;

    m_macroStack.push_back(std::move(newFrame));
    m_programChanged = true;
    return true;
}
// ==========================================
// 🌟 2. 實作返回主程式邏輯
// ==========================================
void NCManager::ReturnMacro(bool queueAlreadyDrained)
{
    if (m_macroStack.empty()) return;

    // =========================================================
    // 🌟 【L 重複次數核心】：如果 repeatCount 還大於 1，PC 歸零重跑！
    // =========================================================
    if (m_macroStack.back().repeatCount > 1) {
        m_macroStack.back().repeatCount--;  // 次數減 1
        m_macroStack.back().currentPC = 0;  // PC 歸零，回到副程式第一行
        m_macroStack.back().committedPC = -1;
        m_programChanged = true;
        if (!queueAlreadyDrained)
        {
            m_waitCallback = WaitAndClearQueueCallback;
        }
        return; // ⚠️ 不彈出堆疊，繼續留在副程式內重跑！
    }

    // --- 標準返回主程式邏輯 ---
    int retPC = m_macroStack.back().returnPC;
    m_macroStack.pop_back();
    MacroSys.PopCallStack();

    if (m_macroStack.empty()) {
        GetBasePC() = retPC;
    }
    else {
        m_macroStack.back().currentPC = retPC;
    }

    m_programChanged = true;
}

void NCManager::PushBlock(const NCBlock& block) {
    // 略過單節跳躍
    if (block.isBlockSkip /* && 系統開啟了單節跳躍開關 */) return;

    m_blockQueue.push(block);
}

// 🌟 (測試用) M 碼專用的檢查函式
static bool CheckMCodeDone(NCManager* nc) {
    int ticks = nc->GetSimulatedTicks() - 1;
    nc->SetSimulatedTicks(ticks);
    if (ticks > 0) {
        //DEBUG_PRINT("    -> [Waiting] IO processing M codes... Ticks left: %d\n", ticks);
        return false;
    }
    return true;
}

// =============================================================================
// Stage NC-0.2J.1 - Lifecycle Failure / Epoch Cancellation Shadow Boundary
// =============================================================================
bool NCManager::IsLifecycleFailureFeedback(
    MotionFeedbackType type) noexcept
{
    return
        type == MotionFeedbackType::REJECTED ||
        type == MotionFeedbackType::CANCELLED ||
        type == MotionFeedbackType::ABORTED ||
        type == MotionFeedbackType::FAULTED;
}

NCLifecycleInterruptionCause
NCManager::LifecycleInterruptionCauseFromFeedback(
    MotionFeedbackType type) noexcept
{
    switch (type)
    {
    case MotionFeedbackType::REJECTED:
        return NCLifecycleInterruptionCause::MOTION_REJECTED;
    case MotionFeedbackType::CANCELLED:
        return NCLifecycleInterruptionCause::MOTION_CANCELLED;
    case MotionFeedbackType::ABORTED:
        return NCLifecycleInterruptionCause::MOTION_ABORTED;
    case MotionFeedbackType::FAULTED:
        return NCLifecycleInterruptionCause::MOTION_FAULTED;
    default:
        return NCLifecycleInterruptionCause::NONE;
    }
}

NCLifecycleInterruptionSample
NCManager::BuildLifecycleInterruptionSample() const noexcept
{
    NCLifecycleInterruptionSample sample{};
    const NCBlockLifecycleCounters lifecycle =
        m_blockLifecycleLedger.GetCounters();

    sample.executionEpoch = m_motion.GetCurrentExecutionEpoch();
    sample.ownerLease = m_motion.GetMotionOwnerLease();
    // The current Motion owner may already be SAFETY when Alarm is latched.
    // Preserve the exact NC execution lease that produced the active Ledger
    // blocks so J.6.3.1 can correlate their later retirement feedback.
    sample.executionOwnerLease = m_programMotionLease;
    sample.activePC = GetActiveDispatchPC();
    sample.activeBlocks = lifecycle.activeBlocks;

    NCBlockLifecycleSnapshot lastLifecycle{};
    if (m_blockLifecycleLedger.GetLastDispatchedSnapshot(lastLifecycle))
    {
        sample.lastDispatchId = lastLifecycle.dispatchId;
    }

    sample.axisCommandDepth = m_motion.GetAxisCommandMailboxDepth();
    sample.axisResultDepth = m_motion.GetAxisCommandResultDepth();
    sample.commandQueueDepth = m_motion.GetQueueSize();
    sample.commandIngressDepth = m_motion.GetCommandIngressSize();
    sample.commandReplayDepth = m_motion.GetCommandReplaySize();
    sample.feedbackDepth = m_motion.GetMotionFeedbackDepth();
    sample.feedbackNoticeDepth =
        m_motion.GetMotionFeedbackProducerNoticeDepth();
    sample.lastPublishedFeedbackSequence =
        m_motion.GetLastPublishedMotionFeedbackSequence();
    sample.lastConsumedFeedbackSequence =
        m_lastConsumedMotionFeedbackSequence;

    sample.blockFailed = lifecycle.blockFailed;
    sample.blocksDispatched = lifecycle.dispatched;
    sample.feedbackRejected = lifecycle.feedbackRejected;
    sample.feedbackCancelled = lifecycle.feedbackCancelled;
    sample.feedbackAborted = lifecycle.feedbackAborted;
    sample.feedbackFaulted = lifecycle.feedbackFaulted;
    sample.motionCaptureOverflow = lifecycle.motionCaptureOverflow;
    sample.orphanFeedback = lifecycle.orphanFeedback;
    sample.duplicateTerminalFeedback =
        lifecycle.duplicateTerminalFeedback;
    sample.terminalFeedbackConflict =
        lifecycle.terminalFeedbackConflict;
    sample.activeBlockOverwrite = lifecycle.activeBlockOverwrite;
    sample.activeSegmentIndexOverwrite =
        lifecycle.activeSegmentIndexOverwrite;

    sample.feedbackOverflow =
        m_motion.GetMotionFeedbackOverflowCount();
    sample.feedbackNoticeOverflow =
        m_motion.GetMotionFeedbackProducerNoticeOverflowCount();
    sample.feedbackSequenceGap = m_motionFeedbackSequenceGapCount;

    sample.safetyOrRecoveryPending =
        m_motion.HasPendingSafetyOrRecoveryRequests();
    sample.waitCallbackActive = m_waitCallback != nullptr;
    sample.completionBindingActive =
        m_blockCompletionBoundaryObserver.HasActiveBinding();

    // NC-0.2J.5: lifecycle causes have different completion contracts.
    // Reset requires its exact RT settle/rebase acknowledgement; Alarm keeps
    // the legacy physical diagnostic; replacement/GOTO/failure paths need
    // only a coherent transport/group drain.
    const NCLifecycleInterruptionCause interruptionCause =
        m_lifecycleInterruptionShadow.GetSnapshot().cause;
    if (interruptionCause == NCLifecycleInterruptionCause::RESET)
    {
        const MotionNCResetRebaseAck resetAck =
            m_motion.GetNCResetRebaseAck();
        sample.groupStandstill =
            m_resetNCSettleRequestSequence !=
            MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID &&
            resetAck.requestSequence ==
            m_resetNCSettleRequestSequence &&
            resetAck.executionEpoch == sample.executionEpoch &&
            resetAck.owner == MotionOwner::SAFETY &&
            m_safetyMotionLease.IsValid() &&
            sample.ownerLease.owner == MotionOwner::SAFETY &&
            sample.ownerLease.generation ==
            m_safetyMotionLease.generation &&
            resetAck.ownerGeneration ==
            m_safetyMotionLease.generation &&
            resetAck.requestedAxisMask != 0U &&
            resetAck.requestedAxisMask == resetAck.appliedAxisMask &&
            resetAck.requestAccepted &&
            resetAck.rebaseApplied &&
            resetAck.postVerifyPassed &&
            resetAck.acknowledged &&
            resetAck.acked &&
            !resetAck.blocked &&
            !resetAck.superseded;
    }
    else if (interruptionCause == NCLifecycleInterruptionCause::ALARM)
    {
        sample.groupStandstill = m_motion.IsGroupStandstill();
    }
    else
    {
        sample.groupStandstill = m_motion.IsGroupNCDrained();
    }
    return sample;
}

void NCManager::BeginLifecycleInterruptionShadow(
    NCLifecycleInterruptionCause cause,
    bool expectsEpochChange) noexcept
{
    m_lifecycleInterruptionShadow.Begin(
        cause,
        expectsEpochChange,
        BuildLifecycleInterruptionSample());
}

void NCManager::RecordLifecycleInterruptionEpochPublished(
    MotionExecutionEpoch executionEpoch) noexcept
{
    m_lifecycleInterruptionShadow.RecordEpochPublished(executionEpoch);
}

void NCManager::ObserveLifecycleInterruptionShadow() noexcept
{
    if (!m_lifecycleInterruptionShadow.IsActive())
    {
        return;
    }

    m_lifecycleInterruptionShadow.Observe(
        BuildLifecycleInterruptionSample());
}


// =============================================================================
// Stage NC-0.2J.6.1/J.6.3.2 - Alarm / Emergency-stop RT ACK Shadow
// =============================================================================
NCAlarmEmergencyStopSample
NCManager::BuildAlarmEmergencyStopSample() const noexcept
{
    NCAlarmEmergencyStopSample sample{};

    const AlarmManager& alarms = AlarmManager::GetInstance();
    sample.alarmActive =
        alarms.HasAlarm() ||
        m_state == NCState::ALARM;
    sample.alarmUpdateCount = alarms.GetUpdateCount();
    sample.alarmCount = alarms.GetAlarmCount();
    if (sample.alarmCount > 0)
    {
        const int alarmIndex = sample.alarmCount - 1;
        sample.alarmCode = alarms.GetAlarmId(alarmIndex);
        sample.alarmAxisIndex =
            alarms.GetAlarmAxisIndex(alarmIndex);
    }

    sample.emergencyEvidenceCoherent =
        m_motion.TryGetEmergencyStopEvidence(
            sample.emergency,
            sample.emergencyCounters);
    sample.epochInvalidation =
        m_motion.GetEmergencyStopEpochInvalidationEvidence();

    // J.6.3.2 correlates Epoch, Owner and E-stop application from one coherent
    // 250 us publication.  Reading the standalone atomics first can straddle
    // the exact RT invalidation that this observer is trying to prove.
    if (sample.emergencyEvidenceCoherent &&
        sample.emergency.publicationGeneration != 0ULL &&
        sample.emergency.currentExecutionEpoch !=
        MOTION_EXECUTION_EPOCH_INVALID)
    {
        sample.executionEpoch =
            sample.emergency.currentExecutionEpoch;
        sample.ownerLease.owner =
            sample.emergency.currentOwner;
        sample.ownerLease.generation =
            sample.emergency.currentOwnerGeneration;
    }
    else
    {
        sample.executionEpoch =
            m_motion.GetCurrentExecutionEpoch();
        sample.ownerLease =
            m_motion.GetMotionOwnerLease();
    }

    sample.lifecycle =
        m_lifecycleInterruptionShadow.GetSnapshot();
    return sample;
}


void NCManager::BeginAlarmEmergencyStopShadow() noexcept
{
    m_alarmEmergencyStopShadow.Begin(
        BuildAlarmEmergencyStopSample());
}


void NCManager::ObserveAlarmEmergencyStopShadow() noexcept
{
    if (m_alarmEmergencyStopShadow.IsActive())
    {
        m_alarmEmergencyStopShadow.Observe(
            BuildAlarmEmergencyStopSample());
    }

    // Stage NC-0.2J.6.3: the exact J.6 RT acknowledgement is the authority
    // for classifying the runtime-owned Alarm Epoch change.  Importing this
    // evidence is idempotent and remains observer-only.
    const NCAlarmEmergencyStopSnapshot alarmStop =
        m_alarmEmergencyStopShadow.GetSnapshot();
    if (alarmStop.acknowledged)
    {
        m_lifecycleInterruptionShadow.RecordAlarmStopAcknowledged(
            alarmStop.lastAppliedExecutionEpoch,
            alarmStop.epochChangeRequired);
    }
}

// ============================================================================
// Stage NC-0.1D - NC Motion Feedback Snapshot / Counters
// ============================================================================
void NCManager::ProcessMotionFeedback() noexcept
{
    MotionFeedbackEvent event{};

    // 每個 NC Cycle 最多處理固定筆數，避免異常事件 Burst 讓
    // 10 ms NC Task 出現過大的單圈負擔。2048 筆 Ring 可容納完整
    // Epoch 淘汰 Burst，未讀事件由後續 Cycle 繼續 Drain。
    for (std::size_t i = 0U;
        i < MOTION_FEEDBACK_NC_DRAIN_LIMIT_PER_TASK;
        ++i)
    {
        if (!m_motion.TryReadMotionFeedback(event))
        {
            break;
        }

        const bool lifecycleFailureFeedback =
            IsLifecycleFailureFeedback(event.type);

        // Capture the pre-event state. This must happen before the sequence
        // check and Ledger apply so a gap revealed by this terminal event and
        // the active Block that it is expected to close remain observable.
        if (lifecycleFailureFeedback &&
            !m_lifecycleInterruptionShadow.IsActive())
        {
            BeginLifecycleInterruptionShadow(
                LifecycleInterruptionCauseFromFeedback(event.type),
                false);
        }

        // Runtime Sequence 必須單調連續（UINT64_MAX 後回到 1）。
        // 發現不連續不在此處改變 NC 行為；先留下診斷計數，後續
        // Alarm Policy / HMI Snapshot 階段再決定是否升級處置。
        if (event.sequence !=
            MOTION_FEEDBACK_SEQUENCE_INVALID)
        {
            if (m_lastConsumedMotionFeedbackSequence !=
                MOTION_FEEDBACK_SEQUENCE_INVALID)
            {
                const MotionFeedbackSequence expectedSequence =
                    (m_lastConsumedMotionFeedbackSequence ==
                        (std::numeric_limits<MotionFeedbackSequence>::max)())
                    ? 1ULL
                    : static_cast<MotionFeedbackSequence>(
                        m_lastConsumedMotionFeedbackSequence + 1ULL);

                if (event.sequence != expectedSequence)
                {
                    ++m_motionFeedbackSequenceGapCount;
                }
            }

            m_lastConsumedMotionFeedbackSequence =
                event.sequence;
        }

        m_lastMotionFeedback = event;
        ++m_processedMotionFeedbackCount;

        // Stage NC-0.2D：只做觀察式 Lifecycle 更新，不改變既有 NC PC、
        // Wait Callback、Single Block 或 Motion 執行結果。
        const bool ledgerAccepted =
            m_blockLifecycleLedger.ApplyMotionFeedback(event);

        if (lifecycleFailureFeedback)
        {
            m_lifecycleInterruptionShadow.RecordTerminalFeedback(
                event,
                ledgerAccepted);
        }

        switch (event.type)
        {
        case MotionFeedbackType::ACCEPTED:
            m_lastAcceptedMotionIdentity = event.identity;
            ++m_acceptedMotionFeedbackCount;
            break;

        case MotionFeedbackType::STARTED:
            m_lastStartedMotionIdentity = event.identity;
            ++m_startedMotionFeedbackCount;
            break;

        case MotionFeedbackType::COMPLETED:
            m_lastCompletedMotionIdentity = event.identity;
            ++m_completedMotionFeedbackCount;
            break;

        case MotionFeedbackType::REJECTED:
            m_lastRejectedMotionIdentity = event.identity;
            ++m_rejectedMotionFeedbackCount;
            break;

        case MotionFeedbackType::ABORTED:
            m_lastAbortedMotionIdentity = event.identity;
            ++m_abortedMotionFeedbackCount;
            break;

        case MotionFeedbackType::CANCELLED:
            m_lastCancelledMotionIdentity = event.identity;
            ++m_cancelledMotionFeedbackCount;
            break;

        case MotionFeedbackType::FAULTED:
            m_lastFaultedMotionIdentity = event.identity;
            ++m_faultedMotionFeedbackCount;
            break;

        case MotionFeedbackType::NONE:
        case MotionFeedbackType::PROGRESS:
        case MotionFeedbackType::HELD:
        case MotionFeedbackType::RESUMED:
        default:
            break;
        }
    }
}


// 🌟 放在 RTOS 迴圈的核心任務
void NCManager::ProcessTask()
{
    NC_RunCount++;

    // 即使 NC 正處於 Alarm / Reset / Not Ready，也必須先 Drain Feedback，
    // 否則 Runtime Terminal Event 可能在上層長時間停住時累積。
    ProcessMotionFeedback();
    ObserveFeedHoldBoundaryShadow();

    if (UpdateSystemVariables_initialize_flag == 0)//第一次初始更新Macro變數
    {
        UpdateSystemVariables();
        UpdateSystemVariables_initialize_flag = 1;//
    }

    // =========================================================
    // 🌟 4. 【結尾動作】將最新的 NC 狀態刷給 PLC S 點！
    // =========================================================
    SyncNCStateToPLC();

    // =========================================================
    // 🌟 1. 【最高優先】每一圈都重新結算並更新機台總合狀態！
    // =========================================================
    m_edmState = GetMachineEDMState();

    if (m_state != NCState::RUN)
    {
        ClearPreDispatchBarrier();
    }





    // =========================================================
    // 🚨 2. 【絕對防禦攔截網】警報與急停鎖死區
    // =========================================================
    // 不論是軟體觸發的 Alarm，或是從 UI 傳下來的 Alarm 狀態
    const bool alarmActive =
        AlarmManager::GetInstance().HasAlarm() ||
        m_state == NCState::ALARM;

    if (alarmActive && !m_lifecycleInterruptionAlarmLatched)
    {
        BeginLifecycleInterruptionShadow(
            NCLifecycleInterruptionCause::ALARM,
            false);
        m_lifecycleInterruptionAlarmLatched = true;
        BeginAlarmEmergencyStopShadow();
    }
    else if (!alarmActive)
    {
        ObserveAlarmEmergencyStopShadow();
        m_lifecycleInterruptionAlarmLatched = false;
    }

    if (alarmActive)
    {
        ClearPreDispatchBarrier();
        m_state = NCState::ALARM; // 確保 NC 大腦確實進入警報狀態
        CancelFeedHoldBoundaryShadow(false);

        // A latched READY/P_END Cycle Start must never auto-run after an
        // alarm is cleared.  The operator must Reset and press Cycle Start
        // again from a newly verified ready state.
        if (m_programRunStartPending)
        {
            ReleasePendingProgramRunMotionOwner();
            CancelProgramEndBoundary();
        }

        // 🌟 [關鍵新增]：只要在警報狀態，每一毫秒都強制下達急停！
        // (底層的 EmergencyStop 有防重複機制，所以這樣寫既安全又暴力)

        m_motion.RequestEmergencyStopAllAxes();

        // First refresh generic Ledger/transport evidence.  While J.6 proof
        // is pending, that observer deliberately defers Epoch classification.
        // J.6 then consumes the fresh lifecycle sample and, on exact RT ACK,
        // authorises the Epoch for the following bounded observation.
        ObserveLifecycleInterruptionShadow();
        ObserveAlarmEmergencyStopShadow();

        // Alarm 或其他 lifecycle event 若取代進行中的 Reset，立即把
        // Reset release gate 標成 blocked；不可保留過期的放行資格。
        m_resetReleaseGate.ObserveBoundary(
            m_lifecycleInterruptionShadow.GetSnapshot(),
            m_motion.GetNCResetRebaseAck(),
            m_motion.IsMotionOwnerLeaseCurrent(m_safetyMotionLease));

        // ⚠️ 立刻退出迴圈，絕對不准往下執行任何軌跡運算或 G 碼解析！
        return;
    }

    // =========================================================
    // 🌟 2.5 【新增：滑行煞車攔截網】等待 Reset 後的馬達完全靜止
    // =========================================================
    if (m_state == NCState::RESET_STATE)
    {
        // NC-0.2J.3：先完成本圈 interruption evidence 觀察，再讓
        // release gate 判斷。Legacy 的單次 IsGroupStandstill() 不再能
        // 提早釋放 SAFETY owner 或將 NC 宣告為 READY。
        ObserveLifecycleInterruptionShadow();

        const NCLifecycleInterruptionSnapshot resetBoundary =
            m_lifecycleInterruptionShadow.GetSnapshot();
        const MotionNCResetRebaseAck resetRebaseAck =
            m_motion.GetNCResetRebaseAck();
        m_resetReleaseGate.ObserveBoundary(
            resetBoundary,
            resetRebaseAck,
            m_motion.IsMotionOwnerLeaseCurrent(m_safetyMotionLease));

        if (m_resetReleaseGate.ShouldReleaseSafetyOwner())
        {
            // NC-0.2J.5: close the proof-to-release window with one fresh ACK
            // and one fresh exact-lease observation.  The second gate pass
            // must still grant permission; release never consumes the older
            // ACK that made the first pass ready.
            const MotionNCResetRebaseAck releaseResetRebaseAck =
                m_motion.GetNCResetRebaseAck();
            const bool releaseSafetyLeaseCurrent =
                m_motion.IsMotionOwnerLeaseCurrent(
                    m_safetyMotionLease);

            m_resetReleaseGate.ObserveBoundary(
                resetBoundary,
                releaseResetRebaseAck,
                releaseSafetyLeaseCurrent);

            if (!m_resetReleaseGate.ShouldReleaseSafetyOwner())
            {
                return;
            }

            bool releaseSucceeded = false;
            if (releaseSafetyLeaseCurrent)
            {
                // Use the exact RT Actual snapshot that was rebased and
                // post-verified.  A later supervisory read must not create a
                // different NC/Motion coordinate boundary.
                CoordSys.SyncMachinePosition(
                    releaseResetRebaseAck.actualMcsUnit);
                releaseSucceeded =
                    m_motion.ReleaseMotionOwner(
                        m_safetyMotionLease);
            }

            m_resetReleaseGate.MarkReleaseResult(
                resetBoundary,
                releaseResetRebaseAck,
                releaseSafetyLeaseCurrent,
                releaseSucceeded);

            if (releaseSucceeded &&
                m_resetReleaseGate.GetSnapshot().releaseApplied)
            {
                m_safetyMotionLease = MotionOwnerLease{};
                m_state = NCState::READY;
                UpdateSystemVariables();
            }
        }

        // ⚠️ 只要還在滑行，就立刻 return，不准執行下面的 G 碼解析與模式分流！
        return;
    }

    // =========================================================
// Machine Ready Interlock
// =========================================================
    if (m_edmState == EDMState::NOT_READY)
    {
        ClearPreDispatchBarrier();

        // Do not turn a temporary external/servo interlock recovery into an
        // implicit Cycle Start.  Cancel the exact pending request and require
        // a fresh operator edge after the machine is READY again.
        if (m_programRunStartPending)
        {
            ReleasePendingProgramRunMotionOwner();
            CancelProgramEndBoundary();
        }

        if (m_state == NCState::RUN)
        {
            FeedHold();
        }

        ObserveLifecycleInterruptionShadow();
        return;
    }

    // =========================================================
    // NC-0.2J.5.3：Fresh Program Run 必須等本次 Execution Epoch 已由
    // 250 us Motion Runtime 消費後才能建立 Program-End baseline。
    // 等待或套用的當圈一律 return，禁止同一個 10 ms scan 立刻 Dispatch。
    // =========================================================
    if (ProcessPendingProgramRunStart())
    {
        ObserveLifecycleInterruptionShadow();
        return;
    }

    // =========================================================
    // NC-0.2I.3：Deferred Feed Hold Resume 只能在 Alarm / Reset / Machine
    // Ready Interlock 全部通過後套用。套用當圈直接 return，避免同一
    // 10 ms 週期內又立刻 Dispatch 下一個 Block。
    // =========================================================
    if (ProcessFeedHoldResumeGate())
    {
        ObserveLifecycleInterruptionShadow();
        return;
    }

    // =========================================================
    // 🌟 3. 正常任務分流 (只有在無警報時才會走到這裡)
    // =========================================================
    switch (m_mode)
    {
    case NCOperationMode::MEMORY:
        if (m_state == NCState::RUN) {
            ProcessExecutionEngine();
        }
        break;

    case NCOperationMode::MDI:
        // 🌟 關鍵修改：MDI 模式現在也支援手動操作了！
        if (m_state == NCState::RUN) {
            ProcessExecutionEngine(); // 如果按下 Cycle Start，執行 MDI 字串
        }
        else {
            ProcessManualMode();      // 閒置時，允許操作員直接使用手輪或 JOG
        }
        break;

    case NCOperationMode::MANUAL:
        if (m_state == NCState::RUN && m_manualAutoRunning) {
            ProcessExecutionEngine();
        }
        else {
            ProcessManualMode();
        }
        break;

    case NCOperationMode::EDIT:
        break;
    }

    ObserveLifecycleInterruptionShadow();
}


// ==========================================
// 🚀 終極統一執行引擎 (完美 M30 歸零卡住、M00 單擊解鎖)
// ==========================================
void NCManager::ProcessExecutionEngine()
{
    // 只有 RUN 狀態才會解析 / 預讀。
    if (m_state != NCState::RUN) return;

    // Stage NC-0.2G：一旦遇到 M02 / M30 / Natural EOF，就停止派送
    // 新 Block，只執行 Cycle End Drain Gate。
    if (m_programEndBoundary.IsEndPending())
    {
        ProcessProgramEndBoundary();
        return;
    }

    if (!m_motion.IsMotionOwnerLeaseCurrent(
        m_programMotionLease))
    {
        return;
    }

    // 安全的 PC 控制器
    auto advancePC = [&]() {
        if (!m_macroStack.empty()) m_macroStack.back().currentPC++;
        else GetBasePC()++;
    };

    auto setPC = [&](int newPC) {
        if (!m_macroStack.empty()) m_macroStack.back().currentPC = newPC;
        else GetBasePC() = newPC;
    };

    bool isMacro = !m_macroStack.empty();
    if (isMacro) {
        m_macroProgramName = m_macroStack.back().programName;
        m_macroProgramPC = m_macroStack.back().currentPC;
    }
    else {
        m_macroProgramName = "";
        m_macroProgramPC = -1;
    }

    // ==========================================================
    // --- 階段 A：等待條件檢查與【神級任務接力】 ---
    // ==========================================================
    if (m_waitCallback != nullptr) {
        const WaitConditionFunc activeWaitCallback = m_waitCallback;
        const bool wasWaitingForStart =
            (activeWaitCallback == WaitForCycleStartCallback);
        const bool wasGMBlockTransaction =
            (activeWaitCallback == WaitForGMBlockTransactionCallback);
        const bool wasControlledSingleBlockWait =
            (activeWaitCallback ==
                WaitForSingleBlockControlledHoldCallback);

        const bool legacyReady = activeWaitCallback(this);
        bool effectiveReady = legacyReady;
        if (!wasWaitingForStart)
        {
            if (!wasControlledSingleBlockWait)
            {
                effectiveReady =
                    ApplyCompletionWaitBoundaryGuard(legacyReady);
            }

            // Stage NC-0.2I.1 / I.4：先由 Shadow 證明完成點，再讓
            // Controlled Gate 決定是否可建立真正的 Single Block HOLD。
            EvaluateSingleBlockShadow(legacyReady);
            ObserveSingleBlockHoldGate();
        }

        if (!effectiveReady) return; // Legacy + Ledger 雙鑰尚未同時完成

        // 先卸下已完成的舊 Callback / Binding；交易 Finalize 可能建立
        // Macro Flow 或 Program End 等下一個狀態，不可再被舊指標覆蓋。
        m_waitCallback = nullptr;
        if (!wasWaitingForStart &&
            !wasControlledSingleBlockWait)
        {
            ClearCompletionWaitBoundary(false);
        }

        if (wasGMBlockTransaction)
        {
            const bool transactionFinalized =
                FinalizeGMBlockTransaction();

            // Post Action 成功或失敗都要留下 Single Block Shadow 證據。
            // 成功時可判斷 Transaction Boundary 已完整；失敗時則記錄
            // TXN_FAILED，仍然不改變既有 Alarm / Flow Control 行為。
            EvaluateSingleBlockShadow(true);
            ObserveSingleBlockHoldGate();

            if (!transactionFinalized)
            {
                return;
            }

            // 某些 Program Flow Post Action 可能合法建立新的等待條件。
            if (m_waitCallback != nullptr)
            {
                return;
            }
        }

        // 🌟 如果剛才是在「等按鈕」(Cycle Start)，現在按鈕解開了，
        // 代表操作員要開始跑這行了，直接 return 進入底下解析派發！
        if (wasWaitingForStart) {
            return;
        }

        // Controlled Gate 若仍在等待 Completion Boundary，接手成為
        // 下一個等待條件。PC 不前進，NC 也不會先進入 HOLD。
        if (m_singleBlockHoldGate.HasPendingControl() &&
            !m_singleBlockHoldGate.ShouldApplyHold())
        {
            m_waitCallback =
                WaitForSingleBlockControlledHoldCallback;
            return;
        }

        // Boundary Failure 採 Fail-Closed。既有 Motion / Transaction Alarm
        // 路徑會負責復歸；此處絕不前進到下一個 Block。
        const NCSingleBlockHoldGateSnapshot singleBlockGateSnapshot =
            m_singleBlockHoldGate.GetSnapshot();
        if (singleBlockGateSnapshot.blocked &&
            !singleBlockGateSnapshot.holdApplied)
        {
            return;
        }

        // 🌟 動作跑完了 (例如 G00 移動到位或 M00 完成)
        // 1. 先安全推進 PC 到下一行 (讓 UI 畫面精準亮起下一行)
        if (m_state != NCState::ALARM && m_state != NCState::P_END && !m_programChanged) {
            advancePC();
        }

        // NC-0.2I.4：只有在完整 Boundary Ready 後才真正 HOLD。
        if (m_singleBlockHoldGate.ShouldApplyHold())
        {
            (void)ApplyControlledSingleBlockHold();
            return;
        }

        // 2. 如果這行有暫停要求 (M00/M01 或 Legacy 單步模式)
        if (m_pauseAfterBlock &&
            !m_programEndBoundary.IsEndPending() &&
            m_state != NCState::ALARM &&
            m_state != NCState::P_END) {
            ObserveLegacySingleBlockHold();
            m_pauseAfterBlock = false;
            m_state = NCState::HOLD;                    // 切換為暫停
            m_waitCallback = WaitForCycleStartCallback; // 掛上「等待 Start 按鈕」
            return; // 結束本回合，定格在下一行！
        }

        return; // 防暴衝：本回合結束，下一毫秒才處理下一行
    }

    // 預讀閘門：容量限制
    if (m_motion.GetQueueSize() >= 50)
    {
        return;
    }

    if (m_waitCallback == nullptr)
    {
        const bool currentIsMacro = !m_macroStack.empty();
        const int currentPC =
            currentIsMacro
            ? m_macroStack.back().currentPC
            : GetBasePC();

        const NCProgramCache* currentProgram =
            currentIsMacro
            ? m_macroStack.back().program
            : &GetBaseProgramCache();

        if (currentProgram == nullptr)
        {
            AlarmManager::GetInstance().Trigger(
                AlarmManager::SYNTAX_ERROR,
                currentPC + 1);
            m_state = NCState::ALARM;
            return;
        }

        // ==========================================================
        // --- 結束判斷 (檔尾到達) ---
        // ==========================================================
        if (currentPC < 0 ||
            static_cast<std::size_t>(currentPC) >= currentProgram->Size())
        {
            if (currentIsMacro)
            {
                // Macro EOF 仍是返回邊界，不是整份 Program End。
                const std::uint64_t commandQueueDepth =
                    static_cast<std::uint64_t>(m_motion.GetQueueSize());
                const bool groupStandstill =
                    m_motion.IsGroupNCDrained();
                if (commandQueueDepth > 0ULL || !groupStandstill)
                {
                    ObservePreDispatchBarrier(
                        NCPreDispatchBarrierKind::MACRO_EOF,
                        currentPC,
                        static_cast<int>(currentProgram->Size()) + 1,
                        -1,
                        commandQueueDepth,
                        groupStandstill);
                    return;
                }
                ClearPreDispatchBarrier();
                ReturnMacro(true);
            }
            else
            {
                ClearPreDispatchBarrier();
                // Stage NC-0.2G：自然檔尾不再直接 Release Owner / P_END。
                // 由統一 Gate 等待所有預讀 Segment、Feedback 與實體停止。
                RequestProgramEnd(
                    NCProgramEndCause::NATURAL_EOF,
                    currentPC,
                    static_cast<int>(currentProgram->Size()) + 1,
                    NC_BLOCK_DISPATCH_ID_INVALID);
            }
            return;
        }

        m_programChanged = false;
        m_pauseAfterBlock = false;
        m_legacySingleBlockPausePending = false;

        const NCProgramCacheLine* cachedLine =
            currentProgram->TryGetLine(currentPC);
        if (cachedLine == nullptr)
        {
            AlarmManager::GetInstance().Trigger(
                AlarmManager::SYNTAX_ERROR,
                currentPC + 1);
            m_state = NCState::ALARM;
            return;
        }

        // Runtime 只讀取 Load Time 建立的 Pure Parsed Cache。
        const NCParsedBlock& parsedBlock = cachedLine->parsedBlock;
        const int sourceLineNumber = cachedLine->sourceLineNumber;
        const NCProgramCommitSnapshot commitTarget =
            MakeCurrentProgramCommitTarget(currentPC);
        NCBlockDispatchId blockDispatchId =
            NC_BLOCK_DISPATCH_ID_INVALID;
        bool lineCommitted = false;
        bool blockSkippedBySwitch = false;
        NCSingleBlockCandidateKind singleBlockCandidateKind =
            NCSingleBlockCandidateKind::NONE;
        bool singleBlockExplicitStopBypass = false;

        const auto ensureBlockLifecycle = [&]() -> NCBlockDispatchId
        {
            if (blockDispatchId == NC_BLOCK_DISPATCH_ID_INVALID)
            {
                blockDispatchId = m_blockLifecycleLedger.BeginBlock(
                    commitTarget,
                    sourceLineNumber);
            }
            return blockDispatchId;
        };

        const auto markDispatchFailed = [&](std::uint32_t errorCode)
        {
            const NCBlockDispatchId dispatchId = ensureBlockLifecycle();
            m_blockLifecycleLedger.MarkNCDispatchFailed(
                dispatchId,
                errorCode);
        };

        const auto commitCurrentLine = [&]()
        {
            if (!lineCommitted)
            {
                const NCBlockDispatchId dispatchId = ensureBlockLifecycle();
                NCProgramCommitSnapshot committedSnapshot{};
                if (CommitProgramBlock(
                    commitTarget,
                    committedSnapshot))
                {
                    m_blockLifecycleLedger.MarkProgramCommitted(
                        dispatchId,
                        committedSnapshot);
                }
                else
                {
                    // 只影響 Ledger 診斷；不改變既有 NC 行為。
                    m_blockLifecycleLedger.MarkNCDispatchFailed(
                        dispatchId,
                        0x020D0001U);
                }
                lineCommitted = true;
            }
        };

        if (parsedBlock.error != NCParseError::NONE)
        {
            markDispatchFailed(
                static_cast<std::uint32_t>(
                    AlarmManager::SYNTAX_ERROR));
            AlarmManager::GetInstance().Trigger(
                AlarmManager::SYNTAX_ERROR,
                sourceLineNumber);
            m_state = NCState::ALARM;
            return;
        }

        // 選擇性跳躍開啟時，整行不求值、不 Commit 任何 Macro Side Effect。
        if (parsedBlock.isBlockSkip && m_isBlockSkipEnabled)
        {
            blockSkippedBySwitch = true;
            // 直接略過，由共用收網邏輯推進 PC。
        }
        else if (parsedBlock.controlType == NCParsedControlType::ASSIGNMENT)
        {
            // Macro 指派是 Program Commit Barrier。前段運動完整結束後，
            // 才能改變後續 Block 會讀到的變數狀態。
            const std::uint64_t commandQueueDepth =
                static_cast<std::uint64_t>(m_motion.GetQueueSize());
            const bool groupStandstill =
                m_motion.IsGroupNCDrained();
            if (commandQueueDepth > 0ULL || !groupStandstill)
            {
                ObservePreDispatchBarrier(
                    NCPreDispatchBarrierKind::ASSIGNMENT,
                    currentPC,
                    sourceLineNumber,
                    -1,
                    commandQueueDepth,
                    groupStandstill);
                return;
            }
            ClearPreDispatchBarrier();

            NCMacroAssignmentCommit assignment{};
            NCExpressionResolveError resolveError =
                NCExpressionResolveError::NONE;
            if (!NCExpressionResolver::ResolveAssignment(
                parsedBlock,
                MathParser,
                assignment,
                resolveError))
            {
                const int alarmCode =
                    resolveError == NCExpressionResolveError::INVALID_VARIABLE_INDEX
                    ? AlarmManager::MACRO_VARIABLE_INDEX_OUT_OF_RANGE
                    : AlarmManager::MATH_ERROR;
                markDispatchFailed(
                    static_cast<std::uint32_t>(alarmCode));
                AlarmManager::GetInstance().Trigger(
                    alarmCode,
                    sourceLineNumber);
                m_state = NCState::ALARM;
                return;
            }

            // 唯一 Side Effect Commit 點。
            MacroSys.SetVar(
                assignment.prefix,
                assignment.index,
                assignment.value);

            singleBlockCandidateKind =
                NCSingleBlockCandidateKind::PROGRAM_CONTROL;
        }
        else if (parsedBlock.controlType == NCParsedControlType::GOTO)
        {
            // IF / GOTO 是控制流程 Barrier。條件也只在真正到達此 PC、
            // 且前段 Motion 已完成後才求值。
            const std::uint64_t commandQueueDepth =
                static_cast<std::uint64_t>(m_motion.GetQueueSize());
            const bool groupStandstill =
                m_motion.IsGroupNCDrained();
            if (commandQueueDepth > 0ULL || !groupStandstill)
            {
                ObservePreDispatchBarrier(
                    NCPreDispatchBarrierKind::GOTO_CONTROL,
                    currentPC,
                    sourceLineNumber,
                    -1,
                    commandQueueDepth,
                    groupStandstill);
                return;
            }
            ClearPreDispatchBarrier();

            NCGotoDecision decision{};
            NCExpressionResolveError resolveError =
                NCExpressionResolveError::NONE;
            if (!NCExpressionResolver::ResolveGoto(
                parsedBlock,
                MathParser,
                decision,
                resolveError))
            {
                int alarmCode = AlarmManager::MATH_ERROR;
                if (resolveError == NCExpressionResolveError::INVALID_VARIABLE_INDEX)
                {
                    alarmCode = AlarmManager::MACRO_VARIABLE_INDEX_OUT_OF_RANGE;
                }
                else if (resolveError == NCExpressionResolveError::INVALID_GOTO_TARGET)
                {
                    alarmCode = AlarmManager::SYNTAX_ERROR;
                }
                markDispatchFailed(
                    static_cast<std::uint32_t>(alarmCode));
                AlarmManager::GetInstance().Trigger(
                    alarmCode,
                    sourceLineNumber);
                m_state = NCState::ALARM;
                return;
            }

            if (decision.shouldJump)
            {
                int targetPC = -1;
                if (!TryGetCurrentJumpTarget(
                    decision.targetSequence,
                    targetPC))
                {
                    markDispatchFailed(
                        static_cast<std::uint32_t>(
                            AlarmManager::GOTO_NOT_FOUND));
                    AlarmManager::GetInstance().Trigger(
                        AlarmManager::GOTO_NOT_FOUND,
                        sourceLineNumber);
                    m_state = NCState::ALARM;
                    return;
                }

                BeginLifecycleInterruptionShadow(
                    NCLifecycleInterruptionCause::GOTO_EPOCH,
                    true);
                setPC(targetPC);
                const MotionExecutionEpoch gotoEpoch =
                    m_motion.BeginNewExecutionEpoch(
                        GetMotionCommandSourceForMode(m_mode));
                RecordLifecycleInterruptionEpochPublished(gotoEpoch);
                m_motion.SyncVirtualEndPosition();
                m_programChanged = true;
            }

            singleBlockCandidateKind =
                NCSingleBlockCandidateKind::PROGRAM_CONTROL;
        }
        else
        {
            // G/M/Address Expression 只要讀取 #/@/$，就必須等前段
            // Motion 完整 Commit 後才求值。這也涵蓋會隨 Runtime
            // 更新的 $ System Variable，避免 Lookahead 提早取樣。
            if (parsedBlock.dependsOnMacroState)
            {
                const std::uint64_t commandQueueDepth =
                    static_cast<std::uint64_t>(m_motion.GetQueueSize());
                const bool groupStandstill =
                    m_motion.IsGroupNCDrained();
                if (commandQueueDepth > 0ULL || !groupStandstill)
                {
                    ObservePreDispatchBarrier(
                        NCPreDispatchBarrierKind::MACRO_DEPENDENCY,
                        currentPC,
                        sourceLineNumber,
                        -1,
                        commandQueueDepth,
                        groupStandstill);
                    return;
                }
                ClearPreDispatchBarrier();
            }

            NCBlock block{};
            NCExpressionResolveError resolveError =
                NCExpressionResolveError::NONE;
            if (!NCExpressionResolver::ResolveBlock(
                parsedBlock,
                MathParser,
                block,
                resolveError))
            {
                int alarmCode = AlarmManager::MATH_ERROR;
                if (resolveError == NCExpressionResolveError::TOO_MANY_G_CODES)
                {
                    alarmCode = AlarmManager::G_code_Count_Error;
                }
                else if (resolveError == NCExpressionResolveError::TOO_MANY_M_CODES)
                {
                    alarmCode = AlarmManager::M_code_Count_Error;
                }
                else if (resolveError == NCExpressionResolveError::INVALID_VARIABLE_INDEX)
                {
                    alarmCode = AlarmManager::MACRO_VARIABLE_INDEX_OUT_OF_RANGE;
                }
                else if (resolveError == NCExpressionResolveError::INVALID_CODE_VALUE ||
                    resolveError == NCExpressionResolveError::INVALID_PARSED_BLOCK)
                {
                    alarmCode = AlarmManager::SYNTAX_ERROR;
                }

                markDispatchFailed(
                    static_cast<std::uint32_t>(alarmCode));
                AlarmManager::GetInstance().Trigger(
                    alarmCode,
                    sourceLineNumber);
                m_state = NCState::ALARM;
                return;
            }

            singleBlockCandidateKind =
                ClassifySingleBlockCandidate(block);

            if (block.mCount > 0)
            {
                const int singleBlockMCode = block.mCode[0];
                singleBlockExplicitStopBypass =
                    singleBlockMCode == 0 ||
                    (singleBlockMCode == 1 &&
                        m_isOptionalStopEnabled);
            }

            bool isBarrier = false;
            if (block.mCount > 0)
            {
                const int m = block.mCode[0];
                if (m == 98 || m == 99 || m == 0 || m == 1 ||
                    m == 2 || m == 30)
                {
                    isBarrier = true;
                }
            }

            isBarrier =
                isBarrier ||
                NCGCodeSemantics::IsBlockBarrier(block);

            if (m_isSingleBlockEnabled &&
                (block.hasG || block.mCount > 0 ||
                    block.has('X') || block.has('Y') || block.has('Z')))
            {
                isBarrier = true;
            }

            if (isBarrier)
            {
                const std::uint64_t commandQueueDepth =
                    static_cast<std::uint64_t>(m_motion.GetQueueSize());
                const bool groupStandstill =
                    m_motion.IsGroupNCDrained();
                if (commandQueueDepth > 0ULL || !groupStandstill)
                {
                    NCPreDispatchBarrierKind barrierKind =
                        NCPreDispatchBarrierKind::BLOCK_BARRIER;
                    int barrierMCode = -1;

                    if (block.mCount > 0)
                    {
                        barrierMCode = block.mCode[0];
                        switch (barrierMCode)
                        {
                        case 0: barrierKind = NCPreDispatchBarrierKind::M00; break;
                        case 1: barrierKind = NCPreDispatchBarrierKind::M01; break;
                        case 2: barrierKind = NCPreDispatchBarrierKind::M02; break;
                        case 30: barrierKind = NCPreDispatchBarrierKind::M30; break;
                        case 98: barrierKind = NCPreDispatchBarrierKind::M98; break;
                        case 99: barrierKind = NCPreDispatchBarrierKind::M99; break;
                        default: break;
                        }
                    }
                    else if (m_isSingleBlockEnabled)
                    {
                        barrierKind =
                            NCPreDispatchBarrierKind::SINGLE_BLOCK_BARRIER;
                    }
                    else if (block.hasG)
                    {
                        barrierKind =
                            NCPreDispatchBarrierKind::G_CODE_BARRIER;
                    }

                    ObservePreDispatchBarrier(
                        barrierKind,
                        currentPC,
                        sourceLineNumber,
                        barrierMCode,
                        commandQueueDepth,
                        groupStandstill);
                    return;
                }
                ClearPreDispatchBarrier();
            }
            else
            {
                ClearPreDispatchBarrier();
            }

            const NCBlockDispatchId dispatchId = ensureBlockLifecycle();

            // Stage NC-0.2D：只在 NC Producer 執行緒收集此 Block 建立的
            // Segment Identity；不把 NC 型別帶入 250 us Motion Runtime。
            m_motion.BeginProgramBlockMotionCapture();

            // Stage NC-0.2A：同一 Block 的 Modal 已先 Commit，才擷取 Snapshot。
            if (!block.isEmpty)
            {
                ExecuteBlock(
                    block,
                    currentPC,
                    sourceLineNumber,
                    dispatchId);
            }

            const MotionProgramBlockCapture motionCapture =
                m_motion.EndProgramBlockMotionCapture();
            BindProgramBlockMotionCapture(
                dispatchId,
                motionCapture);

            if (AlarmManager::GetInstance().HasAlarm())
            {
                m_blockLifecycleLedger.MarkNCDispatchFailed(
                    dispatchId,
                    0U);
                m_state = NCState::ALARM;
                if (m_mode == NCOperationMode::MANUAL)
                {
                    m_manualAutoRunning = false;
                }
                return;
            }

            // G66 模態巨集自動攔截。
            if (m_isG66Active &&
                !NCGCodeSemantics::Contains(block, 66) &&
                !NCGCodeSemantics::Contains(block, 67))
            {
                const bool isRealMotion = IsRealMotionBlock(block);
                if (isRealMotion && m_macroStack.empty())
                {
                    const std::string macroFile =
                        "O" + std::to_string(m_g66P) + ".nc";

                    if (CallMacro(macroFile))
                    {
                        m_macroStack.back().repeatCount = m_g66L;
                        for (int i = 0; i < 26; ++i)
                        {
                            const char c = static_cast<char>('A' + i);
                            if (c != 'P' && c != 'G' && c != 'L' &&
                                m_g66Block.has(c))
                            {
                                MacroSys.SetVar(
                                    '#',
                                    i + 1,
                                    m_g66Block.val(c));
                            }
                        }
                    }
                }
            }

            // G66 可能在這裡建立 Macro Frame；失敗時不可提交本行。
            if (AlarmManager::GetInstance().HasAlarm())
            {
                m_blockLifecycleLedger.MarkNCDispatchFailed(
                    dispatchId,
                    0U);
                m_state = NCState::ALARM;
                if (m_mode == NCOperationMode::MANUAL)
                {
                    m_manualAutoRunning = false;
                }
                return;
            }

            // 本行已完成 Runtime Resolve 與 NC Side Effect / Downstream Dispatch。
            // Motion 實際完成仍由 Feedback / Physical PC 表示。
            commitCurrentLine();

            // Stage NC-0.2H：M00/M01/M98/M99/M02/M30 的 Post Action
            // 由 G/M Transaction 在所有同行動作與 Motion Ledger 完成後套用。
            // 此處不可再提前改變 PC、HOLD 或 Program End 狀態。
        }

        // Block Skip、Assignment、GOTO 與空白行會在這裡提交；
        // 一般 G/M Block 已先提交，重複呼叫由 lambda 保護。
        commitCurrentLine();

        // NC-0.2I.4 Single Block Controlled Cutover。
        //
        // Gate Enabled:
        //   只有真正可執行的 Block 才消耗一次 Cycle Start；完成點由
        //   Program Commit + Callback/Transaction + Motion Ledger 證明。
        //
        // Gate Disabled：
        //   完整回到原本 m_pauseAfterBlock 行為，包含空白/Label/Skip
        //   可能產生的 Legacy HOLD，作為可驗證的 Rollback。
        bool controlledSingleBlockRequested = false;
        if (m_isSingleBlockEnabled &&
            m_state != NCState::P_END)
        {
            const NCBlockDispatchId dispatchId =
                ensureBlockLifecycle();
            const bool singleBlockEligible =
                !blockSkippedBySwitch &&
                singleBlockCandidateKind !=
                NCSingleBlockCandidateKind::NONE;

            if (singleBlockEligible)
            {
                const bool legacyGateDisabled =
                    !m_singleBlockHoldGate.IsEnabled();

                // M00 與 Enabled M01 已有自己的 Explicit Stop。它們仍
                // 由既有 Post Action 建立唯一 HOLD，不再疊加第二個
                // Controlled Single Block HOLD。
                m_legacySingleBlockPausePending =
                    legacyGateDisabled ||
                    singleBlockExplicitStopBypass;

                if (legacyGateDisabled)
                {
                    m_pauseAfterBlock = true;
                }

                ArmSingleBlockShadow(
                    singleBlockCandidateKind,
                    dispatchId,
                    commitTarget,
                    sourceLineNumber);

                const NCSingleBlockHoldGateRequestResult gateResult =
                    m_singleBlockHoldGate.RequestControl(
                        m_singleBlockBoundaryShadow.GetSnapshot(),
                        singleBlockExplicitStopBypass);

                controlledSingleBlockRequested =
                    gateResult ==
                    NCSingleBlockHoldGateRequestResult::CONTROLLED;

                if (gateResult ==
                    NCSingleBlockHoldGateRequestResult::BYPASS_LEGACY &&
                    !singleBlockExplicitStopBypass)
                {
                    m_pauseAfterBlock = true;
                    m_legacySingleBlockPausePending = true;
                }
            }
            else
            {
                m_singleBlockBoundaryShadow.NoteNotEligible(
                    dispatchId,
                    commitTarget,
                    sourceLineNumber);

                if (!m_singleBlockHoldGate.IsEnabled())
                {
                    // Legacy Rollback 必須保留原本的 Phantom-Hold 行為。
                    m_pauseAfterBlock = true;
                    m_legacySingleBlockPausePending = true;
                }
            }
        }

        // 🌟 派發後收網處理
        if (m_waitCallback == nullptr)
        {
            if (controlledSingleBlockRequested)
            {
                // 不使用 Queue Empty 當作 Single Block 完成依據。
                // Dedicated Callback 只等待已驗證的 Completion Boundary。
                m_waitCallback =
                    WaitForSingleBlockControlledHoldCallback;
            }
            else if (m_pauseAfterBlock || m_programChanged)
            {
                m_waitCallback = WaitAndClearQueueCallback;
            }
            else
            {
                advancePC();
            }
        }

        if (m_waitCallback != nullptr &&
            m_waitCallback != WaitForCycleStartCallback &&
            m_waitCallback !=
            WaitForSingleBlockControlledHoldCallback &&
            blockDispatchId != NC_BLOCK_DISPATCH_ID_INVALID)
        {
            BindCompletionWaitBoundary(
                blockDispatchId,
                m_waitCallback);
        }
    }
}

// ==========================================
// 🌟 MANUAL 模式專屬邏輯 (自動指令優先，JOG 墊後)
// ==========================================
void NCManager::ProcessManualMode()
{
    // ==========================================
    // 🕹️ 正常處理：純硬體 JOG / MPG
    // ==========================================
    // SHM_Data* pShm = SHMManager::GetInstance().GetData();
    // if (pShm == nullptr) return;

    // (將你原本讀取 pShm 按鈕，呼叫 m_motion.Jog(...) 的邏輯寫在這裡)
}
// ==========================================
// Base Program Cache / Dispatch PC / Commit PC
// ==========================================
int& NCManager::GetBasePC()
{
    if (m_mode == NCOperationMode::MDI) return m_mdiPC;
    if (m_mode == NCOperationMode::MANUAL) return m_manualPC;
    return m_programPC;
}

int NCManager::GetBasePCValue() const noexcept
{
    if (m_mode == NCOperationMode::MDI) return m_mdiPC;
    if (m_mode == NCOperationMode::MANUAL) return m_manualPC;
    return m_programPC;
}

NCProgramCache& NCManager::GetBaseProgramCache() noexcept
{
    if (m_mode == NCOperationMode::MDI) return m_mdiProgramCache;
    if (m_mode == NCOperationMode::MANUAL) return m_manualProgramCache;
    return m_programCache;
}

const NCProgramCache& NCManager::GetBaseProgramCache() const noexcept
{
    if (m_mode == NCOperationMode::MDI) return m_mdiProgramCache;
    if (m_mode == NCOperationMode::MANUAL) return m_manualProgramCache;
    return m_programCache;
}

int& NCManager::GetBaseCommittedPC() noexcept
{
    if (m_mode == NCOperationMode::MDI) return m_mdiCommittedPC;
    if (m_mode == NCOperationMode::MANUAL) return m_manualCommittedPC;
    return m_programCommittedPC;
}

int NCManager::GetBaseCommittedPCValue() const noexcept
{
    if (m_mode == NCOperationMode::MDI) return m_mdiCommittedPC;
    if (m_mode == NCOperationMode::MANUAL) return m_manualCommittedPC;
    return m_programCommittedPC;
}

NCProgramScope NCManager::GetBaseProgramScope() const noexcept
{
    switch (m_mode)
    {
    case NCOperationMode::MDI:
        return NCProgramScope::MDI;
    case NCOperationMode::MANUAL:
        return NCProgramScope::MANUAL_AUTO;
    case NCOperationMode::MEMORY:
    case NCOperationMode::EDIT:
    default:
        return NCProgramScope::MEMORY;
    }
}

bool NCManager::TryGetCurrentJumpTarget(
    int sequenceNumber,
    int& targetPC) const
{
    if (!m_macroStack.empty())
    {
        const NCProgramCache* program = m_macroStack.back().program;
        return program != nullptr &&
            program->TryFindSequence(sequenceNumber, targetPC);
    }

    return GetBaseProgramCache().TryFindSequence(sequenceNumber, targetPC);
}

NCProgramCommitSnapshot NCManager::MakeCurrentProgramCommitTarget(
    int sourcePC) const noexcept
{
    NCProgramCommitSnapshot target{};
    target.sourcePC = sourcePC;

    if (!m_macroStack.empty())
    {
        const MacroFrame& frame = m_macroStack.back();
        target.scope = NCProgramScope::MACRO;
        target.frameId = frame.frameId;
        if (frame.program != nullptr)
        {
            target.cacheGeneration = frame.program->GetGeneration();
        }
        return target;
    }

    target.scope = GetBaseProgramScope();
    target.cacheGeneration = GetBaseProgramCache().GetGeneration();
    return target;
}

bool NCManager::CommitProgramBlock(
    const NCProgramCommitSnapshot& target,
    NCProgramCommitSnapshot& committedSnapshot) noexcept
{
    committedSnapshot = NCProgramCommitSnapshot{};

    if (target.scope == NCProgramScope::NONE ||
        target.cacheGeneration == NC_PROGRAM_CACHE_GENERATION_INVALID ||
        target.sourcePC < 0)
    {
        return false;
    }

    // Cache Generation 是 Commit 的最後一道防線。
    // 若程式在 Resolve / Dispatch 期間已被重新載入或 Reset，舊 Target
    // 不可更新 Committed PC，也不可覆蓋 Last Commit Snapshot。
    bool targetIsCurrent = false;
    switch (target.scope)
    {
    case NCProgramScope::MEMORY:
        targetIsCurrent =
            m_programCache.GetGeneration() == target.cacheGeneration &&
            static_cast<std::size_t>(target.sourcePC) < m_programCache.Size();
        break;
    case NCProgramScope::MDI:
        targetIsCurrent =
            m_mdiProgramCache.GetGeneration() == target.cacheGeneration &&
            static_cast<std::size_t>(target.sourcePC) < m_mdiProgramCache.Size();
        break;
    case NCProgramScope::MANUAL_AUTO:
        targetIsCurrent =
            m_manualProgramCache.GetGeneration() == target.cacheGeneration &&
            static_cast<std::size_t>(target.sourcePC) < m_manualProgramCache.Size();
        break;
    case NCProgramScope::MACRO:
        if (target.frameId != NC_PROGRAM_FRAME_ID_INVALID)
        {
            // M99 會先 Pop Frame，再回到此 Commit 點；因此允許 Frame 已離開
            // Stack，但其 Parsed Cache 必須仍屬於目前主程式 Session。
            for (const auto& entry : m_macroProgramCaches)
            {
                if (entry.second.GetGeneration() == target.cacheGeneration &&
                    static_cast<std::size_t>(target.sourcePC) < entry.second.Size())
                {
                    targetIsCurrent = true;
                    break;
                }
            }
        }
        break;
    case NCProgramScope::NONE:
    default:
        break;
    }

    if (!targetIsCurrent)
    {
        return false;
    }

    NCProgramCommitSequence sequence =
        NC_PROGRAM_COMMIT_SEQUENCE_INVALID;
    do
    {
        sequence = m_nextProgramCommitSequence++;
    } while (sequence == NC_PROGRAM_COMMIT_SEQUENCE_INVALID);

    m_lastProgramCommit = target;
    m_lastProgramCommit.sequence = sequence;

    switch (target.scope)
    {
    case NCProgramScope::MEMORY:
        m_programCommittedPC = target.sourcePC;
        break;
    case NCProgramScope::MDI:
        m_mdiCommittedPC = target.sourcePC;
        break;
    case NCProgramScope::MANUAL_AUTO:
        m_manualCommittedPC = target.sourcePC;
        break;
    case NCProgramScope::MACRO:
        for (MacroFrame& frame : m_macroStack)
        {
            if (frame.frameId == target.frameId &&
                frame.program != nullptr &&
                frame.program->GetGeneration() == target.cacheGeneration)
            {
                frame.committedPC = target.sourcePC;
                break;
            }
        }
        break;
    case NCProgramScope::NONE:
    default:
        break;
    }

    committedSnapshot = m_lastProgramCommit;
    return true;
}

void NCManager::BindProgramBlockMotionCapture(
    NCBlockDispatchId dispatchId,
    const MotionProgramBlockCapture& capture) noexcept
{
    if (dispatchId == NC_BLOCK_DISPATCH_ID_INVALID)
    {
        return;
    }

    for (std::size_t i = 0U; i < capture.count; ++i)
    {
        const MotionProgramBlockSubmission& submission =
            capture.submissions[i];

        m_blockLifecycleLedger.BindMotionSegment(
            dispatchId,
            submission.identity,
            submission.producerAccepted,
            submission.immediateRejectReason);
    }

    if (capture.overflow)
    {
        m_blockLifecycleLedger.MarkMotionCaptureOverflow(
            dispatchId);
    }
}

void NCManager::ResetActiveProgramCommitBoundary() noexcept
{
    if (!m_macroStack.empty())
        m_macroStack.back().committedPC = -1;
    else
        GetBaseCommittedPC() = -1;

    m_lastProgramCommit = NCProgramCommitSnapshot{};
}

void NCManager::ResetAllProgramCommitBoundaries() noexcept
{
    m_programCommittedPC = -1;
    m_mdiCommittedPC = -1;
    m_manualCommittedPC = -1;
    for (MacroFrame& frame : m_macroStack)
        frame.committedPC = -1;
    m_lastProgramCommit = NCProgramCommitSnapshot{};
}

NCProgramFrameId NCManager::AllocateMacroFrameId() noexcept
{
    NCProgramFrameId frameId = m_nextMacroFrameId++;
    if (frameId == NC_PROGRAM_FRAME_ID_INVALID)
        frameId = m_nextMacroFrameId++;
    return frameId;
}

int NCManager::GetActiveDispatchPC() const noexcept
{
    if (!m_macroStack.empty()) return m_macroStack.back().currentPC;
    return GetBasePCValue();
}

int NCManager::GetActiveCommittedPC() const noexcept
{
    if (!m_macroStack.empty()) return m_macroStack.back().committedPC;
    return GetBaseCommittedPCValue();
}
void NCManager::CapturePendingCommandState(int sourcePC)
{
    const int currentBrainWCS = CoordSys.GetCurrentWCSGCode();
    const int currentBrainToolMode = CoordSys.toolLengthMode;
    const int currentBrainHCode = CoordSys.currentHCode;
    const int currentToolRadiusMode = CoordSys.toolRadiusMode;
    const int currentDCode = CoordSys.currentDCode;
    const bool currentIsAbsolute = CoordSys.isAbsoluteMode;
    const bool currentG68 = CoordSys.isG68Active;
    const double currentG68Angle = CoordSys.g68Angle;
    const bool currentG168 = CoordSys.isWorkpieceRotationActive;
    const int currentWCode = CoordSys.currentWCode;
    const bool currentG51 = CoordSys.isScalingActive;
    const double currentScale = CoordSys.scaleFactor;

    std::uint8_t currentMirrorMask = 0;
    for (int i = 0; i < 8; ++i)
    {
        if (CoordSys.isMirrorActive[i])
        {
            currentMirrorMask |=
                static_cast<std::uint8_t>(1u << i);
        }
    }

    const bool currentG16 = CoordSys.isPolarCoordinateActive;
    const bool currentG162 = CoordSys.isCAxisOffsetRotationEnabled;
    const int currentPlane = CoordSys.activePlane;

    m_motion.SetPendingCommandSource(
        GetMotionCommandSourceForMode(m_mode));

    m_motion.SetNextCommandState(
        sourcePC,
        currentBrainWCS,
        currentBrainToolMode,
        currentBrainHCode,
        currentToolRadiusMode,
        currentDCode,
        currentIsAbsolute,
        currentG68,
        currentG68Angle,
        currentG168,
        currentWCode,
        currentG51,
        currentScale,
        currentMirrorMask,
        currentG16,
        currentG162,
        currentPlane);
}

WaitConditionFunc NCManager::DispatchSingleGCode(
    const NCBlock& sourceBlock,
    int gCode)
{
    NCBlock block = sourceBlock;
    block.hasG = true;
    block.gCode = gCode;
    block.gCount = 1;
    for (int i = 0; i < NC_MAX_G_CODES_PER_BLOCK; ++i)
    {
        block.gCodes[i] = 0;
    }
    block.gCodes[0] = gCode;

    switch (gCode)
    {
    case 0:
        return GCodeHandlers::Handle_G00(block, this);
    case 7:
        return GCodeHandlers::Handle_G07(block, this);
    case 12:
        return GCodeHandlers::Handle_G12(block, this);
    case 161:
        return GCodeHandlers::Handle_G161(block, this);
    case 53:
        return GCodeHandlers::Handle_G53(block, this);
    case 81:
        return GCodeHandlers::Handle_G81(block, this);
    case 28:
        return GCodeHandlers::Handle_G28(block, this);
    case 30:
        return GCodeHandlers::Handle_G30(block, this);
    case 32:
        return GCodeHandlers::Handle_G32(block, this);
    case 4:
        return GCodeHandlers::Handle_G04(block, this);

    case 54: case 55: case 56: case 57: case 58: case 59:
    case 154: case 155: case 156: case 157: case 158: case 159:
    case 254: case 255: case 256: case 257: case 258: case 259:
    case 354: case 355: case 356: case 357: case 358: case 359:
    case 454: case 455: case 456: case 457: case 458: case 459:
    case 554: case 555: case 556: case 557: case 558: case 559:
    case 654: case 655: case 656: case 657: case 658: case 659:
    case 754: case 755: case 756: case 757: case 758: case 759:
    case 854: case 855: case 856: case 857: case 858: case 859:
    case 954: case 955: case 956: case 957: case 958: case 959:
        return GCodeHandlers::Handle_GCode(block, this);

    case 10:
        return GCodeHandlers::Handle_G10(block, this);
    case 160:
        return GCodeHandlers::Handle_G160(block, this);
    case 68:
        return GCodeHandlers::Handle_G68(block, this);
    case 69:
        return GCodeHandlers::Handle_G69(block, this);

    case 90: case 91: case 92:
    case 20: case 21:
    case 22: case 23:
    case 43: case 44: case 49:
    case 17: case 18: case 19:
    case 65: case 66: case 67:
    case 162: case 163:
        return GCodeHandlers::Handle_GCode(block, this);

    case 168:
        return GCodeHandlers::Handle_G168(block, this);
    case 169:
        return GCodeHandlers::Handle_G169(block, this);
    case 40:
        return GCodeHandlers::Handle_G40(block, this);
    case 41:
        return GCodeHandlers::Handle_G41(block, this);
    case 42:
        return GCodeHandlers::Handle_G42(block, this);
    case 50:
        return GCodeHandlers::Handle_G50(block, this);
    case 51:
        return GCodeHandlers::Handle_G51(block, this);
    case 150:
        return GCodeHandlers::Handle_G150(block, this);
    case 151:
        return GCodeHandlers::Handle_G151(block, this);
    case 15:
        return GCodeHandlers::Handle_G15(block, this);
    case 16:
        return GCodeHandlers::Handle_G16(block, this);

    default:
        AlarmManager::GetInstance().Trigger(
            AlarmManager::Unable_to_recognize_G_code);
        m_state = NCState::HOLD;
        return nullptr;
    }
}

void NCManager::ExecuteBlock(
    const NCBlock& block,
    int sourcePC,
    int sourceLineNumber,
    NCBlockDispatchId dispatchId)
{
    m_waitCallback = nullptr;
    const bool blockStartedInMainProgram = m_macroStack.empty();

    NCGCodeExecutionPlan plan{};
    NCGCodePlanError planError = NCGCodePlanError::NONE;
    int firstConflictCode = -1;
    int secondConflictCode = -1;

    if (!NCGCodeSemantics::BuildExecutionPlan(
        block,
        plan,
        planError,
        firstConflictCode,
        secondConflictCode))
    {
        (void)firstConflictCode;
        (void)secondConflictCode;

        const int alarmCode =
            planError == NCGCodePlanError::UNSUPPORTED_G_CODE
            ? AlarmManager::Unable_to_recognize_G_code
            : AlarmManager::G_code_Count_Error;

        AlarmManager::GetInstance().Trigger(alarmCode);
        m_state = NCState::HOLD;
        return;
    }

    // 單一 Block 目前仍只允許一個 M-code。
    if (block.mCount > 1)
    {
        AlarmManager::GetInstance().Trigger(
            AlarmManager::M_code_Count_Error);
        m_state = NCState::HOLD;
        return;
    }

    // G65 本身會建立新的 Macro Program Scope。同行的普通 Auxiliary M
    // 可以由 Transaction 等待，但不可再搭配另一個 Program Flow M。
    if (block.mCount > 0 &&
        NCGCodeSemantics::Contains(block, 65))
    {
        const int m = block.mCode[0];
        if (m == 98 || m == 99 || m == 2 || m == 30)
        {
            AlarmManager::GetInstance().Trigger(
                AlarmManager::G_Code_Invalid_parameter,
                sourceLineNumber);
            m_state = NCState::HOLD;
            return;
        }
    }

    int validatedM98P = 0;
    int validatedM98L = 1;
    if (block.mCount > 0 && block.mCode[0] == 98)
    {
        // M98 的 P/L 參數由 M-code 擁有。為避免與同行 Primary G
        // Action 共用 P/L 產生歧義，M98 目前只允許搭配純設定型 G。
        const bool validP =
            TryGetPositiveIntegerAddress(
                block,
                'P',
                validatedM98P);

        const bool validL =
            !block.has('L') ||
            TryGetPositiveIntegerAddress(
                block,
                'L',
                validatedM98L);

        if (!validP || !validL || plan.hasPrimaryAction)
        {
            AlarmManager::GetInstance().Trigger(
                AlarmManager::G_Code_Invalid_parameter,
                sourceLineNumber);
            m_state = NCState::HOLD;
            return;
        }
    }

    // 瞬間完成的設定。
    if (block.has('E'))
    {
        // m_edmManager.ApplyE(block.val('E'));
    }
    if (block.has('B'))
    {
        // m_edmManager.ApplyB(block.val('B'));
    }
    if (block.has('T'))
    {
        CoordSys.SetToolNumber(
            static_cast<int>(block.val('T')),
            this);
    }

    WaitConditionFunc lastSettingCallback = nullptr;
    WaitConditionFunc primaryActionCallback = nullptr;

    for (int i = 0; i < plan.count; ++i)
    {
        const int gCode = plan.orderedCodes[i];
        NCGCodeDescriptor descriptor{};
        if (!NCGCodeSemantics::TryGetDescriptor(
            gCode,
            descriptor))
        {
            AlarmManager::GetInstance().Trigger(
                AlarmManager::Unable_to_recognize_G_code);
            m_state = NCState::HOLD;
            return;
        }

        // 關鍵順序：同一 Block 的 G20/G17/G90/G54/G43/G40... 已先 Commit，
        // 再擷取 Motion Frame Snapshot，最後才派送唯一 Primary Action。
        if (descriptor.role == NCGCodeRole::PRIMARY_ACTION)
        {
            CapturePendingCommandState(sourcePC);
        }

        WaitConditionFunc callback =
            DispatchSingleGCode(block, gCode);

        if (AlarmManager::GetInstance().HasAlarm())
        {
            m_waitCallback = callback;
            return;
        }

        if (descriptor.role == NCGCodeRole::PRIMARY_ACTION)
        {
            primaryActionCallback = callback;
        }
        else if (callback != nullptr)
        {
            lastSettingCallback = callback;
        }
    }

    // 有 Primary Action 時，其 Callback 是 G 子動作的唯一等待來源。
    // G00 P1 可合法回傳 nullptr；Motion Ledger 仍會提供第二把鑰匙。
    const WaitConditionFunc gWaitCallback =
        plan.hasPrimaryAction
        ? primaryActionCallback
        : lastSettingCallback;

    if (block.mCount > 0)
    {
        const int m = block.mCode[0];
        WaitConditionFunc mWaitCallback = nullptr;
        NCGMBlockPostAction postAction = NCGMBlockPostAction::NONE;
        int pValue = 0;
        int repeatCount = 1;
        const bool fromMainProgram = blockStartedInMainProgram;

        switch (m)
        {
        case 0:
            postAction = NCGMBlockPostAction::PROGRAM_STOP_M00;
            break;

        case 1:
            mWaitCallback = GCodeHandlers::Handle_MCode(block, this);
            postAction = NCGMBlockPostAction::OPTIONAL_STOP_M01;
            break;

        case 2:
            postAction = NCGMBlockPostAction::PROGRAM_END_M02;
            break;

        case 30:
            postAction = NCGMBlockPostAction::PROGRAM_END_M30;
            break;

        case 98:
            pValue = validatedM98P;
            repeatCount = validatedM98L;
            postAction = NCGMBlockPostAction::CALL_M98;
            break;

        case 99:
            postAction = NCGMBlockPostAction::RETURN_M99;
            break;

        default:
            mWaitCallback = GCodeHandlers::Handle_MCode(block, this);
            break;
        }

        BeginGMBlockTransaction(
            gWaitCallback,
            mWaitCallback,
            postAction,
            sourcePC,
            sourceLineNumber,
            dispatchId,
            m,
            pValue,
            repeatCount,
            fromMainProgram);

        m_waitCallback = WaitForGMBlockTransactionCallback;
    }
    else
    {
        m_waitCallback = gWaitCallback;
    }

    UpdateSystemVariables();
}

// =========================================================
// 🌟 2. 實作讀取 AXIS_CFG.ini
// =========================================================
void NCManager::LoadAxisConfiguration()
{

    std::string filepath = GlobalConfig::GetInstance().NCDataDir + "AXIS_CFG.ini";
    std::ifstream inFile(filepath);

    if (!inFile.is_open()) {
        //printf("[Error] 無法開啟 AXIS_CFG.ini！將套用預設 X, Y, Z, A, B, C, U, V\n");
        // 如果找不到檔案，塞一組預設值給機台保命
        const char defaultAxes[8] = { 'X', 'Y', 'Z', 'A', 'B', 'C', 'U', 'V' };
        for (int i = 0; i < 8; i++) m_axisNames[i] = defaultAxes[i];
        return;
    }

    std::string line;
    while (std::getline(inFile, line)) {
        if (line.empty() || line[0] == ';') continue; // 略過空白與註解

        std::stringstream ss(line);
        std::string key, value;

        // 以 '=' 切割字串 (例如 "Axis0=X")
        if (std::getline(ss, key, '=') && std::getline(ss, value)) {
            // 解析 Axis0 ~ Axis7
            if (key.length() >= 5 && key.substr(0, 4) == "Axis") {
                int index = key[4] - '0'; // 把字元 '0' 轉成整數 0
                if (index >= 0 && index < 8) {
                    // 只取等號後面的第一個字元，如果寫 NONE 或空，就會抓不到英文字母
                    if (value.length() > 0 && value != "NONE") {
                        m_axisNames[index] = value[0];
                        //printf("[Config] 軸 %d 對應字元: %c\n", index, m_axisNames[index]);
                    }
                }
            }
        }
    }
    inFile.close();
}

// =========================================================
// 🌟 3. 提供給直譯器 (Parser) 搜尋用的 API
// =========================================================
int NCManager::GetAxisIndex(char gcodeLetter) const
{
    for (int i = 0; i < 8; i++) {
        if (m_axisNames[i] == gcodeLetter) {
            return i; // 找到對應的陣列 Index 了！
        }
    }
    return -1; // -1 代表這台機器沒有設定這個軸！
}


// =========================================================
// Dynamic Axis Mapping Query
// =========================================================
char NCManager::GetAxisName(
    int axisIndex) const
{
    if (axisIndex < 0 ||
        axisIndex >= 8)
    {
        return '?';
    }


    const char axisName =
        m_axisNames[axisIndex];


    if (axisName == '\0' ||
        axisName == ' ')
    {
        return '?';
    }


    return
        axisName;
}


// ==========================================
// 🌟 載入 MDI 字串
// ==========================================
bool NCManager::LoadMDI(const std::string& mdiContent)
{
    if (m_state == NCState::RESET_STATE)
    {
        return false;
    }

    std::vector<std::string> rawLines;
    std::stringstream ss(mdiContent);
    std::string line;
    int lineCount = 0;

    while (std::getline(ss, line, '\n'))
    {
        if (!line.empty() &&
            line.find_first_not_of("\r\t ") != std::string::npos)
        {
            rawLines.push_back(line);
            ++lineCount;
        }

        if (lineCount >= static_cast<int>(MAX_MDI_LINES))
        {
            break;
        }
    }

    NCProgramCache newProgramCache;
    if (!newProgramCache.Build(std::move(rawLines), Parser))
    {
        return false;
    }

    BeginLifecycleInterruptionShadow(
        NCLifecycleInterruptionCause::MDI_REPLACED,
        true);
    CancelProgramEndBoundary();
    ClearCompletionWaitBoundary(true);
    CancelGMBlockTransaction(true);
    CancelSingleBlockShadow(true);
    CancelFeedHoldBoundaryShadow(true);
    m_waitCallback = nullptr;
    ReleaseProgramMotionOwner();
    const MotionExecutionEpoch replacementEpoch =
        m_motion.BeginNewExecutionEpoch(
            MotionCommandSource::NC_MDI);
    RecordLifecycleInterruptionEpochPublished(replacementEpoch);

    // 新的 Base Program Source 不可沿用上一份 MDI 的 Macro Frame / Cache。
    // 先清 Frame 再清 Cache，避免任何 Frame 指標懸空。
    m_macroStack.clear();
    m_macroProgramCaches.clear();
    MacroSys.Reset();
    m_macroProgramName = "";
    m_macroProgramPC = -1;

    m_mdiProgramCache = std::move(newProgramCache);
    m_mdiPC = 0;
    m_mdiCommittedPC = -1;
    m_lastProgramCommit = NCProgramCommitSnapshot{};

    return !m_mdiProgramCache.Empty();
}

// ==========================================
// 🌟 載入 MANUAL 模式輕量自動指令
// ==========================================
bool NCManager::LoadManualAuto(const std::string& manualContent)
{
    if (m_state == NCState::RESET_STATE)
    {
        return false;
    }

    if (manualContent.length() > MAX_MANUAL_AUTO_BYTES)
    {
        return false;
    }

    std::vector<std::string> rawLines;
    std::stringstream ss(manualContent);
    std::string line;

    while (std::getline(ss, line, '\n'))
    {
        if (!line.empty() &&
            line.find_first_not_of("\r\t ") != std::string::npos)
        {
            rawLines.push_back(line);
        }
    }

    NCProgramCache newProgramCache;
    if (!newProgramCache.Build(std::move(rawLines), Parser))
    {
        return false;
    }

    BeginLifecycleInterruptionShadow(
        NCLifecycleInterruptionCause::MANUAL_AUTO_REPLACED,
        true);
    CancelProgramEndBoundary();
    ClearCompletionWaitBoundary(true);
    CancelGMBlockTransaction(true);
    CancelSingleBlockShadow(true);
    CancelFeedHoldBoundaryShadow(true);
    m_waitCallback = nullptr;
    ReleaseProgramMotionOwner();
    const MotionExecutionEpoch replacementEpoch =
        m_motion.BeginNewExecutionEpoch(
            MotionCommandSource::NC_MANUAL_AUTO);
    RecordLifecycleInterruptionEpochPublished(replacementEpoch);

    // 新的 Manual-Auto Source 建立全新的 Macro Session。
    m_macroStack.clear();
    m_macroProgramCaches.clear();
    MacroSys.Reset();
    m_macroProgramName = "";
    m_macroProgramPC = -1;

    m_manualProgramCache = std::move(newProgramCache);
    m_manualPC = 0;
    m_manualCommittedPC = -1;
    m_manualAutoRunning = false;
    m_lastProgramCommit = NCProgramCommitSnapshot{};

    return !m_manualProgramCache.Empty();
}

// ==========================================
// 🌟 整合版：動態載入短程式碼 (自動判斷 MDI 還是 MANUAL)
// ==========================================
bool NCManager::LoadDynamicCode(const std::string& content)
{
    if (m_state == NCState::RESET_STATE)
    {
        return false;
    }

    NCProgramCache* targetProgram = nullptr;
    int* targetPC = nullptr;
    int* targetCommittedPC = nullptr;

    if (m_mode == NCOperationMode::MDI)
    {
        targetProgram = &m_mdiProgramCache;
        targetPC = &m_mdiPC;
        targetCommittedPC = &m_mdiCommittedPC;
    }
    else if (m_mode == NCOperationMode::MANUAL)
    {
        targetProgram = &m_manualProgramCache;
        targetPC = &m_manualPC;
        targetCommittedPC = &m_manualCommittedPC;
        m_manualAutoRunning = false;
    }
    else
    {
        return false;
    }

    // The source is parsed below before the current dynamic image is replaced.

    std::vector<std::string> rawLines;
    std::stringstream ss(content);
    std::string line;
    while (std::getline(ss, line, '\n'))
    {
        if (!line.empty() &&
            line.find_first_not_of("\r\t ") != std::string::npos)
        {
            rawLines.push_back(line);
        }
    }

    NCProgramCache newProgramCache;
    if (!newProgramCache.Build(std::move(rawLines), Parser))
    {
        return false;
    }

    BeginLifecycleInterruptionShadow(
        NCLifecycleInterruptionCause::DYNAMIC_CODE_REPLACED,
        true);
    CancelProgramEndBoundary();
    ClearCompletionWaitBoundary(true);
    CancelGMBlockTransaction(true);
    CancelSingleBlockShadow(true);
    CancelFeedHoldBoundaryShadow(true);
    m_waitCallback = nullptr;
    ReleaseProgramMotionOwner();
    const MotionExecutionEpoch replacementEpoch =
        m_motion.BeginNewExecutionEpoch(
            GetMotionCommandSourceForMode(m_mode));
    RecordLifecycleInterruptionEpochPublished(replacementEpoch);

    // Dynamic Code 也是新的 Base Program Source；先摧毀所有指向 Macro
    // Cache 的 Frame，再清除 Cache，避免保留舊檔案或懸空指標。
    m_macroStack.clear();
    m_macroProgramCaches.clear();
    MacroSys.Reset();
    m_macroProgramName = "";
    m_macroProgramPC = -1;

    *targetProgram = std::move(newProgramCache);
    *targetPC = 0;
    *targetCommittedPC = -1;
    m_lastProgramCommit = NCProgramCommitSnapshot{};
    m_motion.ResetPhysicalPC();

    int currentBrainWCS = CoordSys.GetCurrentWCSGCode();
    int currentBrainToolMode = CoordSys.toolLengthMode;
    int currentBrainHCode = CoordSys.currentHCode;
    int currentBraintoolRadiusMode = CoordSys.toolRadiusMode;
    int currentBraintoolDCode = CoordSys.currentDCode;
    bool curIsAbs = CoordSys.isAbsoluteMode;
    bool curG68 = CoordSys.isG68Active;
    double curG68Angle = CoordSys.g68Angle;
    bool curG168 = CoordSys.isWorkpieceRotationActive;
    int curWCode = CoordSys.currentWCode;
    bool curG51 = CoordSys.isScalingActive;
    double curScale = CoordSys.scaleFactor;
    uint8_t curMirrorMask = 0;
    for (int i = 0; i < 8; ++i)
        if (CoordSys.isMirrorActive[i]) curMirrorMask |= (1 << i);
    bool curG16 = CoordSys.isPolarCoordinateActive;
    bool curG162 = CoordSys.isCAxisOffsetRotationEnabled;
    int curPlane = CoordSys.activePlane;

    m_motion.ResetPhysicalTags(
        currentBrainWCS, currentBrainToolMode, currentBrainHCode,
        currentBraintoolRadiusMode, currentBraintoolDCode, curIsAbs,
        curG68, curG68Angle, curG168, curWCode, curG51, curScale,
        curMirrorMask, curG16, curG162, curPlane);

    return !targetProgram->Empty();
}

// ==========================================
// 🌟 獲取機台綜合狀態 (結算 NC 大腦與馬達硬體)
// ==========================================
EDMState NCManager::GetMachineEDMState()
{
    // ----------------------------------------------------
    // 🚨 1. [最高優先權] 警報與急停檢查 (回傳 ALARM)
    // ----------------------------------------------------
    // 檢查軟體警報 (AlarmManager 或是 NC 狀態為 ALARM)
    if (AlarmManager::GetInstance().HasAlarm() || m_state == NCState::ALARM) {
        return EDMState::ALARM;
    }

    // 檢查硬體馬達是否報警或處於急停狀態
    for (int i = 0; i < 8; ++i) {
        auto& axis = m_motion.GetAxisContext(i);
        // 只要有一軸急停、錯誤 (Fault) 或追隨誤差警報 (LagAlarm)
        if (axis.state == MotionState::MotionState_ESTOP ||
            axis.state == MotionState::MotionState_ERROR ||
            axis.isFault || axis.isLagAlarm)
        {

            if (axis.isFault)
            {
                AlarmManager::GetInstance().Trigger(AlarmManager::AXIS_Fault, 0, axis.axisIndex);
            }
            if (axis.isLagAlarm)
            {
                AlarmManager::GetInstance().Trigger(AlarmManager::AXIS_LAG_ERROR, 0, axis.axisIndex);
            }


            return EDMState::ALARM;
        }
    }


    // ----------------------------------------------------
 // 🔌 2. External Machine Ready Interlock
 //
 // NCManager 不知道來源是 PLC C11。
 // 它只知道外部條件目前是否允許機台 Ready。
 //
 // false:
 //     Machine / Servo Power 條件尚未成立
 //
 // true:
 //     才繼續檢查各實體軸 Servo On
 // ----------------------------------------------------
    if (!m_externalReadyInterlock)
    {
        return EDMState::NOT_READY;
    }
    // ----------------------------------------------------
      // 🔌 2. [次高優先權] 激磁 (Servo On) 檢查 (回傳 NOT_READY)
      // ----------------------------------------------------
    for (int i = 0; i < 8; ++i) {
        auto& axis = m_motion.GetAxisContext(i);

        // 🌟 只檢查「物理上確實存在（或被啟用）」的軸！
        // 這樣就算跳號 (例如有 0,1,2，跳過 3，有 4)，也不會卡死！
        if (axis.isExist) {
            if (!axis.isServoOn) {
                return EDMState::NOT_READY;
            }
        }
    }

    // ----------------------------------------------------
    // 🧠 3. [邏輯判斷] 根據 NC 大腦狀態推導機台狀態
    // ----------------------------------------------------
    switch (m_state)
    {
    case NCState::RUN:
        return EDMState::START;  // 🟢 正在跑程式

    case NCState::HOLD:
        return EDMState::HOLD;   // 🟡 操作員按下了暫停 (Feed Hold)

    case NCState::RESET_STATE:
    case NCState::P_END:
        return EDMState::STOP;   // ⚪ 程式結束或剛被 Reset 斬斷

    case NCState::IDLE:
    case NCState::READY:
        return EDMState::READY;  // 🔵 一切正常，等待 Cycle Start

    default:
        return EDMState::NOT_READY;
    }
}



// =============================================================================
// Stage NC-0.2G - Program End / Cycle End Completion Gate
// =============================================================================
NCProgramEndGateSample NCManager::BuildProgramEndGateSample() const noexcept
{
    NCProgramEndGateSample sample{};
    const NCBlockLifecycleCounters lifecycle =
        m_blockLifecycleLedger.GetCounters();

    sample.executionEpoch = m_motion.GetCurrentExecutionEpoch();
    const MotionOwnerLease currentOwnerLease =
        m_motion.GetMotionOwnerLease();
    sample.currentOwner = currentOwnerLease.owner;
    sample.currentOwnerGeneration = currentOwnerLease.generation;

    sample.activeBlocks = lifecycle.activeBlocks;
    sample.axisCommandDepth = m_motion.GetAxisCommandMailboxDepth();
    sample.axisResultDepth = m_motion.GetAxisCommandResultDepth();
    sample.commandQueueDepth = m_motion.GetQueueSize();
    sample.commandIngressDepth = m_motion.GetCommandIngressSize();
    sample.commandReplayDepth = m_motion.GetCommandReplaySize();
    sample.feedbackDepth = m_motion.GetMotionFeedbackDepth();
    sample.feedbackNoticeDepth =
        m_motion.GetMotionFeedbackProducerNoticeDepth();
    sample.lastPublishedFeedbackSequence =
        m_motion.GetLastPublishedMotionFeedbackSequence();
    sample.lastConsumedFeedbackSequence =
        m_lastConsumedMotionFeedbackSequence;

    sample.ownerLeaseCurrent =
        m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease);
    sample.safetyOrRecoveryPending =
        m_motion.HasPendingSafetyOrRecoveryRequests();
    sample.waitCallbackActive = m_waitCallback != nullptr;
    sample.completionBindingActive =
        m_blockCompletionBoundaryObserver.HasActiveBinding();

    // NC-0.2J.5: acquire truth, publication identity and proof sequence from
    // one coherent atomic-bank read.  ProgramEndBoundary can then require two
    // distinct RT publications instead of counting one stale image twice.
    MotionNCSettleSnapshot settleSnapshot{};
    MotionNCSettleCounters settleCounters{};
    const bool settleRead =
        m_motion.TryGetNCSettleEvidence(
            MotionNCSettleProfile::GROUP_COMPLETION,
            settleSnapshot,
            settleCounters);
    sample.ncSettlePublicationGeneration =
        settleRead ? settleSnapshot.publicationGeneration : 0ULL;
    sample.ncSettleProofSequence =
        settleRead ? settleSnapshot.proofSequence : 0ULL;

    const bool exactProgramOwner =
        m_programMotionLease.IsValid() &&
        currentOwnerLease.Matches(m_programMotionLease);
    const bool exactSettleIdentity =
        settleSnapshot.executionEpoch == sample.executionEpoch &&
        settleSnapshot.owner == currentOwnerLease.owner &&
        settleSnapshot.ownerGeneration == currentOwnerLease.generation;

    sample.groupStandstill =
        settleRead &&
        settleSnapshot.profile ==
        MotionNCSettleProfile::GROUP_COMPLETION &&
        sample.ownerLeaseCurrent &&
        exactProgramOwner &&
        exactSettleIdentity &&
        settleSnapshot.publicationGeneration != 0ULL &&
        settleSnapshot.proofSequence != 0ULL &&
        settleSnapshot.scopeMask != 0U &&
        settleSnapshot.runtimeObserved &&
        settleSnapshot.runtimeCycleValid &&
        settleSnapshot.runtimeCycleContiguous &&
        settleSnapshot.requiredCycles ==
        MOTION_NC_SETTLE_REQUIRED_CYCLES &&
        settleSnapshot.dwellCycles >= settleSnapshot.requiredCycles &&
        settleSnapshot.groupDrained &&
        settleSnapshot.settled;

    sample.integrity.blockFailed = lifecycle.blockFailed;
    sample.integrity.ncDispatchFailed = lifecycle.ncDispatchFailed;
    sample.integrity.motionCaptureOverflow =
        lifecycle.motionCaptureOverflow;
    sample.integrity.orphanFeedback = lifecycle.orphanFeedback;
    sample.integrity.duplicateTerminalFeedback =
        lifecycle.duplicateTerminalFeedback;
    sample.integrity.terminalFeedbackConflict =
        lifecycle.terminalFeedbackConflict;
    sample.integrity.activeBlockOverwrite =
        lifecycle.activeBlockOverwrite;
    sample.integrity.activeSegmentIndexOverwrite =
        lifecycle.activeSegmentIndexOverwrite;

    sample.integrity.axisCommandQueueFull =
        m_motion.GetAxisCommandQueueFullCount();
    sample.integrity.axisCommandResultOverflow =
        m_motion.GetAxisCommandResultOverflowCount();
    sample.integrity.staleCommandDiscard =
        m_motion.GetStaleCommandDiscardCount();
    sample.integrity.ownerConflictReject =
        m_motion.GetMotionOwnerConflictRejectCount();
    sample.integrity.commandQueueFullReject =
        m_motion.GetCommandQueueFullRejectCount();
    sample.integrity.commandReplayOverflow =
        m_motion.GetCommandReplayOverflowCount();
    sample.integrity.feedbackOverflow =
        m_motion.GetMotionFeedbackOverflowCount();
    sample.integrity.feedbackNoticeOverflow =
        m_motion.GetMotionFeedbackProducerNoticeOverflowCount();
    sample.integrity.feedbackSequenceGap =
        m_motionFeedbackSequenceGapCount;

    return sample;
}

bool NCManager::BeginProgramRunBoundary(
    MotionExecutionEpoch executionEpoch) noexcept
{
    m_programEndAlarmRaised = false;
    return m_programEndBoundary.BeginRun(
        GetBaseProgramScope(),
        GetBaseProgramCache().GetGeneration(),
        executionEpoch,
        m_programMotionLease,
        BuildProgramEndGateSample());
}

bool NCManager::IsPendingProgramRunStartIdentityCurrent() const noexcept
{
    return
        m_programRunStartPending &&
        m_pendingProgramRunExecutionEpoch !=
        MOTION_EXECUTION_EPOCH_INVALID &&
        (m_pendingProgramRunOriginState == NCState::READY ||
            m_pendingProgramRunOriginState == NCState::P_END) &&
        m_state == m_pendingProgramRunOriginState &&
        m_mode == m_pendingProgramRunMode &&
        GetBaseProgramScope() == m_pendingProgramRunScope &&
        GetBaseProgramCache().GetGeneration() ==
        m_pendingProgramRunCacheGeneration &&
        m_programMotionLease.Matches(
            m_pendingProgramRunOwnerLease) &&
        m_motion.IsMotionOwnerLeaseCurrent(
            m_pendingProgramRunOwnerLease) &&
        m_motion.GetCurrentExecutionEpoch() ==
        m_pendingProgramRunExecutionEpoch;
}

void NCManager::ReleasePendingProgramRunMotionOwner() noexcept
{
    const MotionOwnerLease pendingLease =
        m_pendingProgramRunOwnerLease;

    if (pendingLease.IsValid())
    {
        // Release only the lease captured by this button edge.  If HOME,
        // Reset or another lifecycle path has already installed a newer lease,
        // the generation guard makes this a harmless failed release.
        (void)m_motion.ReleaseMotionOwner(pendingLease);
    }

    if (m_programMotionLease.Matches(pendingLease))
    {
        m_programMotionLease = MotionOwnerLease{};
    }
}

void NCManager::ClearPendingProgramRunStart(bool cancelled) noexcept
{
    if (cancelled &&
        m_programRunStartPending &&
        m_pendingProgramRunMode == NCOperationMode::MANUAL)
    {
        m_manualAutoRunning = false;
    }

    m_programRunStartPending = false;
    m_pendingProgramRunExecutionEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    m_pendingProgramRunOwnerLease = MotionOwnerLease{};
    m_pendingProgramRunMode = NCOperationMode::EDIT;
    m_pendingProgramRunOriginState = NCState::NOT_READY;
    m_pendingProgramRunScope = NCProgramScope::NONE;
    m_pendingProgramRunCacheGeneration =
        NC_PROGRAM_CACHE_GENERATION_INVALID;
}

bool NCManager::ProcessPendingProgramRunStart() noexcept
{
    if (!m_programRunStartPending)
    {
        return false;
    }

    // Any state transition or newer lifecycle Epoch supersedes this exact
    // button request.  Never retarget a pending start to whatever Epoch happens
    // to be current after Reset/Stop/Fault.
    if (!IsPendingProgramRunStartIdentityCurrent())
    {
        ReleasePendingProgramRunMotionOwner();
        ClearPendingProgramRunStart(true);
        m_programEndBoundary.Cancel();
        m_programEndAlarmRaised = false;
        return true;
    }

    const MotionExecutionEpoch pendingEpoch =
        m_pendingProgramRunExecutionEpoch;

    // This includes the start's own Epoch PENDING bit.  It can clear only in
    // the 250 us consumer.  Real safety/recovery requests use the same wait and
    // therefore remain fail-closed without being confused with START_DIRTY.
    if (m_motion.HasPendingSafetyOrRecoveryRequests())
    {
        return true;
    }

    // Stage NC-0.2G：新 Program Run 只能從乾淨的 Lifecycle / Transport
    // 邊界開始，避免把上一輪殘留算進新的 Cycle End。
    if (!BeginProgramRunBoundary(pendingEpoch))
    {
        ReleasePendingProgramRunMotionOwner();
        ClearPendingProgramRunStart(true);
        return true;
    }

    // Close the sample-to-RUN seam.  A newer lifecycle publication or owner
    // transfer after BeginRun invalidates the just-created baseline.
    if (!IsPendingProgramRunStartIdentityCurrent() ||
        m_motion.HasPendingSafetyOrRecoveryRequests())
    {
        ReleasePendingProgramRunMotionOwner();
        ClearPendingProgramRunStart(true);
        m_programEndBoundary.Cancel();
        return true;
    }

    const bool startManualAuto =
        m_pendingProgramRunMode == NCOperationMode::MANUAL &&
        !m_manualProgramCache.Empty();
    ClearPendingProgramRunStart(false);

    if (m_mode == NCOperationMode::MANUAL)
    {
        m_manualAutoRunning = startManualAuto;
    }

    m_motion.SyncVirtualEndPosition();
    UpdateSystemVariables();
    m_state = NCState::RUN;

    // The first block is intentionally dispatched on the next NC task.
    return true;
}

bool NCManager::RequestProgramEnd(
    NCProgramEndCause cause,
    int sourcePC,
    int sourceLineNumber,
    NCBlockDispatchId markerDispatchId) noexcept
{
    m_pauseAfterBlock = false;
    m_legacySingleBlockPausePending = false;
    m_singleBlockBoundaryShadow.SuppressForProgramEnd();
    ObserveSingleBlockHoldGate();
    m_programChanged = false;

    const MotionExecutionEpoch requestExecutionEpoch =
        m_motion.GetCurrentExecutionEpoch();
    const MotionOwnerLease requestOwnerLease =
        m_programMotionLease;

    if (m_programEndBoundary.RequestEnd(
        cause,
        sourcePC,
        sourceLineNumber,
        markerDispatchId,
        requestExecutionEpoch,
        requestOwnerLease))
    {
        return true;
    }

    if (!m_programEndAlarmRaised)
    {
        AlarmManager::GetInstance().Trigger(
            AlarmManager::PROGRAM_END_GATE_ERROR,
            sourceLineNumber);
        m_programEndAlarmRaised = true;
    }
    m_state = NCState::ALARM;
    return false;
}

void NCManager::ProcessProgramEndBoundary()
{
    if (!m_programEndBoundary.IsEndPending())
    {
        return;
    }

    // M02 / M30 may legally share a Block with an action that returned a
    // repeated Wait Callback. Drain that callback through the already proven
    // NC-0.2F Dual-Key Guard, but never advance the PC beyond the End marker.
    if (m_waitCallback != nullptr &&
        m_waitCallback != WaitForCycleStartCallback)
    {
        const bool legacyReady = m_waitCallback(this);
        const bool callbackReady =
            ApplyCompletionWaitBoundaryGuard(legacyReady);
        if (callbackReady)
        {
            m_waitCallback = nullptr;
            ClearCompletionWaitBoundary(false);
        }
    }

    const NCProgramEndGateSample sample =
        BuildProgramEndGateSample();

    if (m_programEndBoundary.Evaluate(sample))
    {
        FinalizeProgramEnd();
        return;
    }

    if (m_programEndBoundary.IsFailClosed() &&
        !m_programEndAlarmRaised)
    {
        const NCProgramEndGateSnapshot snapshot =
            m_programEndBoundary.GetSnapshot();
        AlarmManager::GetInstance().Trigger(
            AlarmManager::PROGRAM_END_GATE_ERROR,
            snapshot.sourceLineNumber);
        m_programEndAlarmRaised = true;
        m_state = NCState::ALARM;
    }
}

void NCManager::FinalizeProgramEnd()
{
    // NC-0.2J.5: rebuild and re-evaluate every formal input immediately
    // before the permission action.  Do not finalize from the earlier scan's
    // READY_TO_FINALIZE snapshot.
    const NCProgramEndGateSample releaseSample =
        BuildProgramEndGateSample();
    if (!m_programEndBoundary.Evaluate(releaseSample))
    {
        return;
    }

    // CAS-release the exact Program lease before any modal, queue, callback,
    // or NC state cleanup.  On failure retain both the lease and pending End
    // boundary; the next Evaluate observes the current owner and fails closed.
    if (!m_programMotionLease.IsValid() ||
        !m_motion.ReleaseMotionOwner(m_programMotionLease))
    {
        return;
    }

    m_programMotionLease = MotionOwnerLease{};

    if (!m_programEndBoundary.MarkFinalized())
    {
        return;
    }

    m_waitCallback = nullptr;
    ClearCompletionWaitBoundary(false);
    CancelGMBlockTransaction(false);
    CancelFeedHoldBoundaryShadow(false);
    m_singleBlockBoundaryShadow.SuppressForProgramEnd();
    ObserveSingleBlockHoldGate();
    m_pauseAfterBlock = false;
    m_legacySingleBlockPausePending = false;
    m_programChanged = false;

    m_macroStack.clear();
    m_macroProgramName.clear();
    m_macroProgramPC = -1;
    GetBasePC() = 0;

    Reset_Gode();
    UpdateSystemVariables();

    if (m_mode == NCOperationMode::MANUAL)
    {
        m_manualAutoRunning = false;
        m_state = NCState::READY;
    }
    else if (m_mode == NCOperationMode::MDI)
    {
        m_state = NCState::READY;
    }
    else
    {
        m_state = NCState::P_END;
    }
}

void NCManager::CancelProgramEndBoundary() noexcept
{
    m_programEndBoundary.Cancel();
    m_programEndAlarmRaised = false;
    ClearPendingProgramRunStart(true);
}

// =============================================================================
// Stage NC-0.2J.4 - Pre-dispatch Stop / Settle Barrier Shadow
// =============================================================================
void NCManager::ObservePreDispatchBarrier(
    NCPreDispatchBarrierKind kind,
    int sourcePC,
    int sourceLineNumber,
    int mCode,
    std::uint64_t commandQueueDepth,
    bool groupStandstill) noexcept
{
    const bool sameBarrier =
        m_preDispatchBarrierSnapshot.active &&
        m_preDispatchBarrierSnapshot.kind == kind &&
        m_preDispatchBarrierSnapshot.sourcePC == sourcePC &&
        m_preDispatchBarrierSnapshot.sourceLineNumber == sourceLineNumber &&
        m_preDispatchBarrierSnapshot.mCode == mCode;

    if (!sameBarrier)
    {
        NCPreDispatchBarrierSnapshot snapshot{};
        snapshot.sequence = m_nextPreDispatchBarrierSequence++;
        if (snapshot.sequence == 0ULL)
        {
            snapshot.sequence = m_nextPreDispatchBarrierSequence++;
        }
        snapshot.kind = kind;
        snapshot.sourcePC = sourcePC;
        snapshot.sourceLineNumber = sourceLineNumber;
        snapshot.mCode = mCode;
        snapshot.active = true;
        m_preDispatchBarrierSnapshot = snapshot;
        ++m_preDispatchBarrierCounters.activations;
    }

    ++m_preDispatchBarrierSnapshot.waitSamples;
    m_preDispatchBarrierSnapshot.commandQueueDepth = commandQueueDepth;
    m_preDispatchBarrierSnapshot.commandQueuePending =
        commandQueueDepth != 0ULL;
    m_preDispatchBarrierSnapshot.groupStandstill = groupStandstill;

    ++m_preDispatchBarrierCounters.evaluations;
    if (commandQueueDepth != 0ULL)
    {
        ++m_preDispatchBarrierCounters.waitCommandQueue;
    }
    else if (!groupStandstill)
    {
        ++m_preDispatchBarrierCounters.waitGroupStandstill;
    }
}

void NCManager::ClearPreDispatchBarrier() noexcept
{
    if (!m_preDispatchBarrierSnapshot.active)
    {
        return;
    }

    m_preDispatchBarrierSnapshot.active = false;
    m_preDispatchBarrierSnapshot.commandQueueDepth = 0ULL;
    m_preDispatchBarrierSnapshot.commandQueuePending = false;
    ++m_preDispatchBarrierCounters.cleared;
}

// =============================================================================
// Stage NC-0.2H - G/M Same-Block Transaction Barrier
// =============================================================================
std::uint64_t NCManager::AllocateGMBlockTransactionSequence() noexcept
{
    std::uint64_t sequence = m_nextGMBlockTransactionSequence++;
    if (sequence == 0ULL)
    {
        sequence = m_nextGMBlockTransactionSequence++;
    }
    return sequence;
}

void NCManager::BeginGMBlockTransaction(
    WaitConditionFunc gCallback,
    WaitConditionFunc mCallback,
    NCGMBlockPostAction postAction,
    int sourcePC,
    int sourceLineNumber,
    NCBlockDispatchId dispatchId,
    int mCode,
    int pValue,
    int repeatCount,
    bool fromMainProgram) noexcept
{
    if (m_gmBlockTransaction.snapshot.active)
    {
        CancelGMBlockTransaction(true);
    }

    NCGMBlockTransactionSnapshot snapshot{};
    snapshot.sequence = AllocateGMBlockTransactionSequence();
    snapshot.dispatchId = dispatchId;
    snapshot.phase = NCGMBlockTransactionPhase::WAITING;
    snapshot.postAction = postAction;
    snapshot.sourcePC = sourcePC;
    snapshot.sourceLineNumber = sourceLineNumber;
    snapshot.mCode = mCode;
    snapshot.pValue = pValue;
    snapshot.repeatCount = repeatCount > 0 ? repeatCount : 1;
    snapshot.active = true;
    snapshot.gWaitRequired = gCallback != nullptr;
    snapshot.gWaitComplete = gCallback == nullptr;
    snapshot.mWaitRequired = mCallback != nullptr;
    snapshot.mWaitComplete = mCallback == nullptr;
    snapshot.fromMainProgram = fromMainProgram;

    m_gmBlockTransaction.snapshot = snapshot;
    m_gmBlockTransaction.gCallback = gCallback;
    m_gmBlockTransaction.mCallback = mCallback;

    ++m_gmBlockTransactionCounters.started;
    if (snapshot.gWaitRequired)
    {
        ++m_gmBlockTransactionCounters.gWaitComponents;
    }
    if (snapshot.mWaitRequired)
    {
        ++m_gmBlockTransactionCounters.mWaitComponents;
    }
    if (snapshot.gWaitRequired && snapshot.mWaitRequired)
    {
        ++m_gmBlockTransactionCounters.dualComponentTransactions;
    }
}

bool NCManager::WaitForGMBlockTransactionCallback(NCManager* nc)
{
    return nc != nullptr && nc->EvaluateGMBlockTransaction();
}

bool NCManager::EvaluateGMBlockTransaction() noexcept
{
    NCGMBlockTransactionSnapshot& snapshot =
        m_gmBlockTransaction.snapshot;

    if (!snapshot.active)
    {
        return true;
    }

    ++m_gmBlockTransactionCounters.evaluations;

    if (!snapshot.gWaitComplete)
    {
        if (m_gmBlockTransaction.gCallback == nullptr ||
            m_gmBlockTransaction.gCallback(this))
        {
            snapshot.gWaitComplete = true;
        }
        else
        {
            ++m_gmBlockTransactionCounters.gWaitSamples;
        }
    }

    if (!snapshot.mWaitComplete)
    {
        if (m_gmBlockTransaction.mCallback == nullptr ||
            m_gmBlockTransaction.mCallback(this))
        {
            snapshot.mWaitComplete = true;
        }
        else
        {
            ++m_gmBlockTransactionCounters.mWaitSamples;
        }
    }

    const bool ready =
        snapshot.gWaitComplete &&
        snapshot.mWaitComplete;

    if (ready &&
        snapshot.phase == NCGMBlockTransactionPhase::WAITING)
    {
        snapshot.phase =
            NCGMBlockTransactionPhase::READY_TO_FINALIZE;
        ++m_gmBlockTransactionCounters.readyTransitions;
    }

    return ready;
}

bool NCManager::FinalizeGMBlockTransaction()
{
    NCGMBlockTransactionSnapshot& snapshot =
        m_gmBlockTransaction.snapshot;

    if (!snapshot.active ||
        snapshot.phase !=
        NCGMBlockTransactionPhase::READY_TO_FINALIZE)
    {
        return false;
    }

    bool success = true;

    switch (snapshot.postAction)
    {
    case NCGMBlockPostAction::PROGRAM_STOP_M00:
        ++m_gmBlockTransactionCounters.m00Stops;
        m_pauseAfterBlock = true;
        snapshot.postActionApplied = true;
        break;

    case NCGMBlockPostAction::OPTIONAL_STOP_M01:
        ++m_gmBlockTransactionCounters.m01Stops;
        if (m_isOptionalStopEnabled)
        {
            m_pauseAfterBlock = true;
        }
        snapshot.postActionApplied = true;
        break;

    case NCGMBlockPostAction::CALL_M98:
    {
        ++m_gmBlockTransactionCounters.m98Calls;
        const std::string macroFile =
            "O" + std::to_string(snapshot.pValue) + ".nc";
        success = CallMacro(macroFile);
        if (success)
        {
            m_macroStack.back().repeatCount =
                snapshot.repeatCount > 0
                ? snapshot.repeatCount
                : 1;
            snapshot.postActionApplied = true;
        }
        break;
    }

    case NCGMBlockPostAction::RETURN_M99:
        ++m_gmBlockTransactionCounters.m99Returns;
        if (snapshot.fromMainProgram)
        {
            GetBasePC() = 0;
            m_programChanged = true;
        }
        else
        {
            // Transaction + Completion Guard 已證明同行 G/M 動作完成。
            ReturnMacro(true);
        }
        snapshot.postActionApplied = true;
        break;

    case NCGMBlockPostAction::PROGRAM_END_M02:
    case NCGMBlockPostAction::PROGRAM_END_M30:
    {
        const bool isM02 =
            snapshot.postAction ==
            NCGMBlockPostAction::PROGRAM_END_M02;

        if (isM02)
        {
            ++m_gmBlockTransactionCounters.m02Ends;
        }
        else
        {
            ++m_gmBlockTransactionCounters.m30Ends;
        }

        // Program End 不可再被 Single Block / Optional Stop 改成 HOLD。
        m_pauseAfterBlock = false;
        success = RequestProgramEnd(
            isM02
            ? NCProgramEndCause::M02
            : NCProgramEndCause::M30,
            snapshot.sourcePC,
            snapshot.sourceLineNumber,
            snapshot.dispatchId);

        if (success)
        {
            m_programChanged = true;
            snapshot.postActionApplied = true;
        }
        else
        {
            AlarmManager::GetInstance().Trigger(
                AlarmManager::PROGRAM_END_GATE_ERROR,
                snapshot.sourceLineNumber);
            m_state = NCState::ALARM;
        }
        break;
    }

    case NCGMBlockPostAction::NONE:
    default:
        break;
    }

    m_gmBlockTransaction.gCallback = nullptr;
    m_gmBlockTransaction.mCallback = nullptr;
    snapshot.active = false;

    if (success)
    {
        // M98/M99 可能改變 Macro Frame，M00/M01/M02/M30 可能改變
        // NC Flow；沿用舊 ExecuteBlock 的時序，在正式 Post Action
        // Commit 後刷新 System Variable Snapshot。
        UpdateSystemVariables();

        snapshot.phase = NCGMBlockTransactionPhase::FINALIZED;
        ++m_gmBlockTransactionCounters.finalized;
    }
    else
    {
        snapshot.phase = NCGMBlockTransactionPhase::FAILED;
        ++m_gmBlockTransactionCounters.finalizeFailed;
    }

    return success;
}

void NCManager::CancelGMBlockTransaction(bool superseded) noexcept
{
    (void)superseded;

    if (!m_gmBlockTransaction.snapshot.active)
    {
        return;
    }

    m_gmBlockTransaction.gCallback = nullptr;
    m_gmBlockTransaction.mCallback = nullptr;
    m_gmBlockTransaction.snapshot.active = false;
    m_gmBlockTransaction.snapshot.phase =
        NCGMBlockTransactionPhase::CANCELLED;
    ++m_gmBlockTransactionCounters.cancelled;
}

// =============================================================================
// Single Block panel mode
// =============================================================================
void NCManager::SetSingleBlockEnabled(bool enabled)
{
    if (m_isSingleBlockEnabled == enabled)
    {
        return;
    }

    const bool pendingControlledBoundary =
        m_singleBlockHoldGate.HasPendingControl();
    const bool controlledHoldApplied =
        m_singleBlockHoldGate.IsHoldApplied();
    const NCSingleBlockShadowSnapshot shadow =
        m_singleBlockBoundaryShadow.GetSnapshot();

    m_isSingleBlockEnabled = enabled;

    if (enabled)
    {
        return;
    }

    m_pauseAfterBlock = false;
    m_legacySingleBlockPausePending = false;

    // Turning Single Block OFF while already stopped must not auto-run. Keep
    // the applied HOLD until the operator presses Cycle Start once.
    if (controlledHoldApplied)
    {
        return;
    }

    m_singleBlockBoundaryShadow.Cancel(false);
    m_singleBlockHoldGate.Cancel(false);

    // If the dedicated gate callback was the only current wait, convert it to
    // the legacy completion drain without re-arming a HOLD. The current Block
    // still completes safely, then continuous execution continues.
    if (pendingControlledBoundary &&
        m_waitCallback ==
        WaitForSingleBlockControlledHoldCallback)
    {
        m_waitCallback = WaitAndClearQueueCallback;
        if (shadow.dispatchId != NC_BLOCK_DISPATCH_ID_INVALID)
        {
            BindCompletionWaitBoundary(
                shadow.dispatchId,
                m_waitCallback);
        }
    }
}

// =============================================================================
// Stage NC-0.2I.4 - Single Block Completion-Gated HOLD Controlled Cutover
// =============================================================================
void NCManager::SetSingleBlockCompletionGateEnabled(
    bool enabled) noexcept
{
    if (m_singleBlockHoldGate.IsEnabled() == enabled)
    {
        return;
    }

    const bool hadPendingControl =
        m_singleBlockHoldGate.HasPendingControl();
    const NCSingleBlockShadowSnapshot shadow =
        m_singleBlockBoundaryShadow.GetSnapshot();

    m_singleBlockHoldGate.SetEnabled(enabled);

    if (!enabled &&
        hadPendingControl &&
        m_isSingleBlockEnabled &&
        m_state == NCState::RUN)
    {
        // Runtime Rollback：目前尚未完成的 Controlled Boundary 轉回
        // Legacy Pause，不改變 PC、Motion Queue 或已派送 Segment。
        m_pauseAfterBlock = true;
        m_legacySingleBlockPausePending = true;

        if (m_waitCallback == nullptr ||
            m_waitCallback ==
            WaitForSingleBlockControlledHoldCallback)
        {
            m_waitCallback = WaitAndClearQueueCallback;
            if (shadow.dispatchId != NC_BLOCK_DISPATCH_ID_INVALID)
            {
                BindCompletionWaitBoundary(
                    shadow.dispatchId,
                    m_waitCallback);
            }
        }
    }
}

// =============================================================================
// Stage NC-0.2I.1 - Single Block Completion Boundary Shadow
// =============================================================================
NCSingleBlockCandidateKind NCManager::ClassifySingleBlockCandidate(
    const NCBlock& block) noexcept
{
    if (block.gCount > 0 || block.mCount > 0)
    {
        return NCSingleBlockCandidateKind::G_M_BLOCK;
    }

    // N/O-only lines are labels / program identifiers.  They are not an
    // executable Single Block boundary.  All other address words are.
    for (int index = 0; index < 26; ++index)
    {
        if (!block.hasParam[index])
        {
            continue;
        }

        const char address =
            static_cast<char>('A' + index);
        if (address != 'N' && address != 'O')
        {
            return NCSingleBlockCandidateKind::ADDRESS_BLOCK;
        }
    }

    return NCSingleBlockCandidateKind::NONE;
}

void NCManager::ArmSingleBlockShadow(
    NCSingleBlockCandidateKind candidateKind,
    NCBlockDispatchId dispatchId,
    const NCProgramCommitSnapshot& target,
    int sourceLineNumber) noexcept
{
    if (!m_isSingleBlockEnabled ||
        candidateKind == NCSingleBlockCandidateKind::NONE ||
        dispatchId == NC_BLOCK_DISPATCH_ID_INVALID)
    {
        return;
    }

    const NCGMBlockTransactionSnapshot transaction =
        m_gmBlockTransaction.snapshot;
    const bool transactionRequired =
        transaction.active &&
        transaction.dispatchId == dispatchId;

    NCSingleBlockShadowArmRequest request{};
    request.dispatchId = dispatchId;
    request.programTarget = target;
    request.sourceLineNumber = sourceLineNumber;
    request.candidateKind = candidateKind;
    request.transactionRequired = transactionRequired;
    request.callbackRequired =
        !transactionRequired &&
        m_waitCallback != nullptr &&
        m_waitCallback != WaitForCycleStartCallback;
    request.legacyPausePending =
        m_legacySingleBlockPausePending;

    m_singleBlockBoundaryShadow.Arm(request);
}

void NCManager::EvaluateSingleBlockShadow(
    bool callbackComplete) noexcept
{
    if (!m_singleBlockBoundaryShadow.HasActiveBoundary())
    {
        return;
    }

    const NCSingleBlockShadowSnapshot shadow =
        m_singleBlockBoundaryShadow.GetSnapshot();

    NCSingleBlockShadowSample sample{};
    sample.lifecycleFound =
        m_blockLifecycleLedger.GetMotionBoundarySnapshot(
            shadow.dispatchId,
            sample.motionBoundary);
    sample.callbackComplete = callbackComplete;
    sample.programEndPending =
        m_programEndBoundary.IsEndPending();

    if (shadow.transactionRequired)
    {
        const NCGMBlockTransactionSnapshot transaction =
            m_gmBlockTransaction.snapshot;

        if (transaction.dispatchId == shadow.dispatchId)
        {
            sample.transactionComplete =
                !transaction.active &&
                transaction.phase ==
                NCGMBlockTransactionPhase::FINALIZED;

            sample.transactionFailed =
                transaction.phase ==
                NCGMBlockTransactionPhase::FAILED;
        }
    }
    else
    {
        sample.transactionComplete = true;
    }

    m_singleBlockBoundaryShadow.Evaluate(sample);
}

void NCManager::ObserveSingleBlockHoldGate() noexcept
{
    m_singleBlockHoldGate.ObserveBoundary(
        m_singleBlockBoundaryShadow.GetSnapshot());
}

bool NCManager::ApplyControlledSingleBlockHold() noexcept
{
    if (m_state != NCState::RUN ||
        !m_singleBlockHoldGate.ShouldApplyHold() ||
        m_programEndBoundary.IsEndPending())
    {
        return false;
    }

    const NCSingleBlockShadowSnapshot boundary =
        m_singleBlockBoundaryShadow.GetSnapshot();
    if (!boundary.boundaryReady ||
        boundary.motionFailed)
    {
        return false;
    }

    m_singleBlockBoundaryShadow.ObserveControlledHold();
    m_singleBlockHoldGate.MarkHoldApplied(
        m_singleBlockBoundaryShadow.GetSnapshot());

    if (!m_singleBlockHoldGate.IsHoldApplied())
    {
        return false;
    }

    m_pauseAfterBlock = false;
    m_legacySingleBlockPausePending = false;
    m_state = NCState::HOLD;
    m_waitCallback = WaitForCycleStartCallback;
    return true;
}

bool NCManager::ApplyControlledSingleBlockResume() noexcept
{
    if (m_state != NCState::HOLD ||
        !m_singleBlockHoldGate.IsHoldApplied())
    {
        return false;
    }

    if (!AcquireProgramMotionOwner())
    {
        return false;
    }

    m_singleBlockBoundaryShadow.ObserveControlledResume();
    m_singleBlockHoldGate.MarkResumeApplied();

    m_state = NCState::RUN;
    m_motion.SetGroupFeedrateOverride(1.0);
    m_pauseAfterBlock = false;
    m_legacySingleBlockPausePending = false;
    return true;
}

void NCManager::ObserveLegacySingleBlockHold() noexcept
{
    // Refresh once more after NC-0.2H Post Action finalization.  This is still
    // observation-only; the return value is intentionally ignored.
    EvaluateSingleBlockShadow(true);
    m_singleBlockBoundaryShadow.ObserveLegacyHold(
        m_legacySingleBlockPausePending);
    m_legacySingleBlockPausePending = false;
}

void NCManager::CancelSingleBlockShadow(
    bool superseded) noexcept
{
    m_singleBlockBoundaryShadow.Cancel(superseded);
    m_singleBlockHoldGate.Cancel(superseded);
    m_legacySingleBlockPausePending = false;
}

// =============================================================================
// Stage NC-0.2I.2 - Feed Hold Request / Acknowledge Boundary Shadow
// =============================================================================
NCFeedHoldBoundarySample NCManager::BuildFeedHoldBoundarySample() const noexcept
{
    NCFeedHoldBoundarySample sample{};
    sample.executionEpoch =
        m_motion.GetCurrentExecutionEpoch();
    sample.ownerLease =
        m_motion.GetMotionOwnerLease();
    sample.motion =
        m_motion.GetFeedHoldStopSnapshot();
    sample.expectedSettleRequestSequence =
        m_feedHoldNCSettleRequestSequence;
    sample.activePC =
        GetActiveDispatchPC();
    sample.legacyHoldState =
        m_state == NCState::HOLD;
    sample.homeActive =
        Homing.IsActive();
    sample.homeHoldDecelerating =
        Homing.IsHoldDecelerating();
    sample.homePaused =
        Homing.IsPaused();
    sample.homeResumeRequested =
        Homing.IsResumeRequested();

    const NCGMBlockTransactionSnapshot transaction =
        m_gmBlockTransaction.snapshot;
    if (transaction.active &&
        transaction.dispatchId != NC_BLOCK_DISPATCH_ID_INVALID)
    {
        sample.dispatchId = transaction.dispatchId;
        sample.activePC = transaction.sourcePC;
        return sample;
    }

    if (m_waitingBlockDispatchId != NC_BLOCK_DISPATCH_ID_INVALID)
    {
        sample.dispatchId = m_waitingBlockDispatchId;

        NCBlockLifecycleSnapshot lifecycle{};
        if (m_blockLifecycleLedger.TryGetSnapshot(
            sample.dispatchId,
            lifecycle))
        {
            sample.activePC = lifecycle.programTarget.sourcePC;
        }
        return sample;
    }

    NCBlockLifecycleSnapshot lifecycle{};
    if (m_blockLifecycleLedger.GetLastDispatchedSnapshot(lifecycle))
    {
        sample.dispatchId = lifecycle.dispatchId;
        sample.activePC = lifecycle.programTarget.sourcePC;
    }

    return sample;
}

void NCManager::BeginFeedHoldBoundaryShadow(
    NCFeedHoldSource source) noexcept
{
    if (source != NCFeedHoldSource::PROGRAM)
    {
        m_feedHoldNCSettleRequestSequence =
            MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID;
    }

    // A new Feed Hold Request supersedes any deferred Resume belonging to an
    // older Request sequence.  Terminal Gate diagnostics remain untouched.
    m_feedHoldResumeGate.Cancel(true);

    m_feedHoldBoundaryShadow.BeginRequest(
        source,
        BuildFeedHoldBoundarySample());
}

void NCManager::ObserveFeedHoldBoundaryShadow() noexcept
{
    if (m_feedHoldBoundaryShadow.IsActive())
    {
        NCFeedHoldBoundarySample sample =
            BuildFeedHoldBoundarySample();
        m_feedHoldBoundaryShadow.Observe(sample);

        NCFeedHoldBoundarySnapshot snapshot =
            m_feedHoldBoundaryShadow.GetSnapshot();

        // HOME permits Cycle Start while controlled deceleration is still
        // active. HomingManager queues the request and later changes NC back
        // to RUN after it has internally reached PAUSED. Record that
        // asynchronous apply point. NC-0.2I.3 does not gate HOME yet.
        if (snapshot.active &&
            snapshot.source == NCFeedHoldSource::HOME &&
            snapshot.resumeRequested &&
            m_state == NCState::RUN &&
            !Homing.IsHoldDecelerating() &&
            !Homing.IsPaused())
        {
            sample = BuildFeedHoldBoundarySample();
            m_feedHoldBoundaryShadow.ObserveResumeApplied(sample);
            snapshot = m_feedHoldBoundaryShadow.GetSnapshot();
        }

        // The control gate observes the exact same immutable Boundary
        // snapshot that is published to diagnostics.  It never samples Axis
        // data independently and therefore cannot disagree with the ACK
        // observer about Request identity or stop completion.
        m_feedHoldResumeGate.ObserveBoundary(snapshot);
        return;
    }

    // Keep a pending Gate synchronized with a terminal FAILED/CANCELLED/
    // RESUMED Boundary snapshot even after the observer itself became inactive.
    m_feedHoldResumeGate.ObserveBoundary(
        m_feedHoldBoundaryShadow.GetSnapshot());
}

void NCManager::ObserveFeedHoldLegacyHoldShadow() noexcept
{
    m_feedHoldBoundaryShadow.ObserveLegacyHoldEntered(
        BuildFeedHoldBoundarySample());
}

void NCManager::ObserveFeedHoldResumeRequestedShadow() noexcept
{
    m_feedHoldBoundaryShadow.ObserveResumeRequested(
        BuildFeedHoldBoundarySample());
}

void NCManager::ObserveFeedHoldResumeAppliedShadow() noexcept
{
    m_feedHoldBoundaryShadow.ObserveResumeApplied(
        BuildFeedHoldBoundarySample());
}

void NCManager::CancelFeedHoldBoundaryShadow(
    bool superseded) noexcept
{
    m_feedHoldBoundaryShadow.Cancel(superseded);
    m_feedHoldResumeGate.Cancel(superseded);
}

// =============================================================================
// Stage NC-0.2I.3 - Program Feed Hold ACK-Gated Resume Controlled Cutover
// =============================================================================
bool NCManager::IsProgramFeedHoldResumeCandidate() const noexcept
{
    const NCFeedHoldBoundarySnapshot snapshot =
        m_feedHoldBoundaryShadow.GetSnapshot();

    return
        snapshot.sequence != 0ULL &&
        snapshot.source == NCFeedHoldSource::PROGRAM &&
        snapshot.requestLatched &&
        !snapshot.resumeApplied &&
        !snapshot.cancelled;
}

bool NCManager::ApplyProgramHoldResume(
    bool gateControlled) noexcept
{
    if (m_state != NCState::HOLD)
    {
        return false;
    }

    if (!AcquireProgramMotionOwner())
    {
        // The Gate remains RELEASE_READY and will retry from ProcessTask after
        // the current Owner arbitration becomes valid.  No state or Override
        // is changed on this failed attempt.
        return false;
    }

    // A Cycle Start is considered applied only here.  Early button presses do
    // not consume Single Block state and do not clear the current Wait Callback.
    m_singleBlockBoundaryShadow.ObserveLegacyResume();
    m_legacySingleBlockPausePending = false;

    m_state = NCState::RUN;
    m_motion.SetGroupFeedrateOverride(1.0);
    m_pauseAfterBlock = false;

    ObserveFeedHoldResumeAppliedShadow();

    if (gateControlled)
    {
        m_feedHoldResumeGate.MarkResumeApplied(
            m_feedHoldBoundaryShadow.GetSnapshot());
    }

    return true;
}

bool NCManager::ProcessFeedHoldResumeGate() noexcept
{
    // Refresh terminal failure / ACK information before deciding whether the
    // deferred button request may become a real Resume.
    m_feedHoldResumeGate.ObserveBoundary(
        m_feedHoldBoundaryShadow.GetSnapshot());

    if (!m_feedHoldResumeGate.ShouldApplyResume())
    {
        return false;
    }

    // Fail closed: an ACK release can only be applied while the legacy NC flow
    // is still frozen in HOLD.  Alarm/Reset/Not-Ready paths return earlier and
    // cancel or preserve the request without starting motion.
    if (m_state != NCState::HOLD)
    {
        return false;
    }

    return ApplyProgramHoldResume(true);
}

// =============================================================================
// Stage NC-0.2F - Motion Completion Dual-Key Guard
// =============================================================================
void NCManager::BindCompletionWaitBoundary(
    NCBlockDispatchId dispatchId,
    WaitConditionFunc callback) noexcept
{
    if (dispatchId == NC_BLOCK_DISPATCH_ID_INVALID ||
        callback == nullptr ||
        callback == WaitForCycleStartCallback)
    {
        return;
    }

    NCBlockMotionBoundarySnapshot boundary{};
    const bool hasBoundary =
        m_blockLifecycleLedger.GetMotionBoundarySnapshot(
            dispatchId,
            boundary);

    NCBlockWaitKind waitKind = NCBlockWaitKind::AUXILIARY_CALLBACK;
    const bool isQueueDrain =
        callback == WaitAndClearQueueCallback ||
        callback == WaitAndHoldCallback;

    if (hasBoundary &&
        boundary.state != NCBlockMotionBoundaryState::NOT_TRACKED &&
        boundary.state != NCBlockMotionBoundaryState::NONE)
    {
        waitKind = isQueueDrain
            ? NCBlockWaitKind::MOTION_QUEUE_DRAIN
            : NCBlockWaitKind::MOTION_HANDLER;
    }
    else if (isQueueDrain)
    {
        waitKind = NCBlockWaitKind::PROGRAM_FLOW_DRAIN;
    }

    if (m_waitingBlockDispatchId != dispatchId)
    {
        ClearCompletionWaitBoundary(true);
    }

    m_waitingBlockDispatchId = dispatchId;
    m_blockCompletionBoundaryObserver.Bind(
        dispatchId,
        waitKind);
}

bool NCManager::ApplyCompletionWaitBoundaryGuard(
    bool legacyReady) noexcept
{
    if (m_waitingBlockDispatchId == NC_BLOCK_DISPATCH_ID_INVALID)
    {
        return legacyReady;
    }

    NCBlockMotionBoundarySnapshot boundary{};
    const bool hasBoundary =
        m_blockLifecycleLedger.GetMotionBoundarySnapshot(
            m_waitingBlockDispatchId,
            boundary);

    return m_blockCompletionBoundaryObserver.ObserveAndGate(
        hasBoundary,
        boundary,
        legacyReady);
}

void NCManager::ClearCompletionWaitBoundary(
    bool superseded) noexcept
{
    m_blockCompletionBoundaryObserver.ClearBinding(superseded);
    m_waitingBlockDispatchId = NC_BLOCK_DISPATCH_ID_INVALID;
}

// 1. 等待馬達靜止
bool NCManager::WaitAndHoldCallback(NCManager* nc) {
    if (nc->m_motion.GetQueueSize() > 0 || !nc->m_motion.IsGroupNCDrained()) return false;
    return true;
}

// NC-0.2I.4：只等待已驗證的 Single Block Completion Boundary。
bool NCManager::WaitForSingleBlockControlledHoldCallback(NCManager* nc)
{
    if (nc == nullptr)
    {
        return false;
    }

    nc->EvaluateSingleBlockShadow(true);
    nc->ObserveSingleBlockHoldGate();
    return nc->m_singleBlockHoldGate.ShouldApplyHold();
}

// 2. 🌟 專門等待操作員按下 Cycle Start 的卡點
bool NCManager::WaitForCycleStartCallback(NCManager* nc) {
    if (nc->m_state == NCState::RUN) {
        return true; // 操作員按下 Start 了！解除卡點！
    }
    return false; // 還沒按，繼續乖乖卡住
}

// 3. 只等待馬達靜止 (清空預讀)
bool NCManager::WaitAndClearQueueCallback(NCManager* nc) {
    if (nc->m_motion.GetQueueSize() > 0 || !nc->m_motion.IsGroupNCDrained()) return false;
    return true;
}
// ==========================================================
// 🌟 G 碼屬性過濾器：判斷是否為真實的移動指令
// ==========================================================
bool NCManager::IsRealMotionBlock(const NCBlock& block)
{
    // 保留目前 G66 觸發規則：至少要有 XYZ 字元。
    if (!block.has('X') &&
        !block.has('Y') &&
        !block.has('Z'))
    {
        return false;
    }

    const int primaryActionCode =
        NCGCodeSemantics::GetPrimaryActionCode(block);

    if (primaryActionCode >= 0)
    {
        return NCGCodeSemantics::IsMotionAction(
            primaryActionCode);
    }

    if (NCGCodeSemantics::BlockSuppressesImplicitMotion(block))
    {
        return false;
    }

    // 沒有 Exclusive Action 時，保留未來 Modal Motion 的可能性。
    return true;
}
