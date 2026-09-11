#include "NCManager.h"
#include "MacroEngine.h"
#include <memory>
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
#include <windows.h>
#include <rtapi.h>
// NC-0.2L.2AT / Split-Unit Link Pairing Guard.
// Link-only MSVC/COFF pairing of the two AS implementation units. Each unit
// contributes its own revisioned witness and requires the peer's witness.
// A partial AT/legacy-AS update must not silently link. detect_mismatch also
// rejects different TAGGED contract revisions; it does not hash source bytes.
// These non-exported empty symbols are never called by NC/CRT/PDO code. They
// are retained by /INCLUDE, not a callback, object or runtime registration.
// Keep both stamps in sync when changing this split-unit contract. Do not
// remove a witness or enable /FORCE:UNRESOLVED to bypass a missing-peer error.
// This is not a complete duplicate detector: extra untagged legacy objects,
// both-old files, or same-tag altered code still require source/SHA auditing.
#if defined(_MSC_VER)
#pragma detect_mismatch("NCPathCore.SplitPair", "AT1")
#if defined(_M_IX86)
#pragma comment(linker, "/include:_NCPathCoreSplit_AT1_PathCoreWitness")
#else
#pragma comment(linker, "/include:NCPathCoreSplit_AT1_PathCoreWitness")
#endif
extern "C" void __cdecl NCPathCoreSplit_AT1_ManagerWitness() noexcept {}
#endif

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


    void TriggerMappingIntegrityAlarmOnce(int sourceLineNumber = 0)
    {
        AlarmManager& alarms = AlarmManager::GetInstance();
        for (int alarmIndex = 0;
            alarmIndex < alarms.GetAlarmCount();
            ++alarmIndex)
        {
            if (alarms.GetAlarmId(alarmIndex) ==
                AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY)
            {
                return;
            }
        }

        alarms.Trigger(
            AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY,
            sourceLineNumber);
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


NCManager::NCManager(MotionCore& motion) : m_motion(motion), MathParser(MacroSys), Parser(),
m_pathCoreLiveRetention(AllocatePathCoreLiveOwnerTagStartup())
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

bool NCManager::RejectProgramLoad(
    const char* reason, const std::string& filepath) noexcept
{
    m_programLoadStartBlocked = true;
    RtPrintf("[NC-LOAD-CJ] REJECTED reason=%s requested=%.240s current=%.240s startBlocked=1\n",
        reason, filepath.c_str(), m_mainProgramName.c_str());
    return false;
}

bool NCManager::LoadProgram(const std::string& filepath)
{
    const NCState originState = m_state.load(std::memory_order_acquire);
    if (originState != NCState::IDLE && originState != NCState::READY &&
        originState != NCState::P_END)
        return RejectProgramLoad("STATE_NOT_IDLE", filepath);

    const NCOperationMode originMode = m_mode;
    const NCProgramCacheGeneration originCache = m_programCache.GetGeneration();
    const bool bootstrapProgramLoad = !m_bootProgramImageLoaded &&
        originState == NCState::IDLE && m_programCache.Empty();
    const MotionOwnerLease originOwner = m_motion.GetMotionOwnerLease();
    const MotionExecutionEpoch originEpoch = m_motion.GetCurrentExecutionEpoch();
    AlarmManager& alarms = AlarmManager::GetInstance();
    const std::uint32_t alarmRevision = alarms.GetUpdateCount();

    const auto contextCurrent = [&]() noexcept -> bool
    {
        const MotionOwnerLease owner = m_motion.GetMotionOwnerLease();
        if (m_state != originState || m_mode != originMode ||
            m_programCache.GetGeneration() != originCache ||
            owner.owner != originOwner.owner || owner.generation != originOwner.generation ||
            m_motion.GetCurrentExecutionEpoch() != originEpoch ||
            m_programRunStartPending || Homing.IsActive()) return false;
        // The retained boot SAFETY owner still belongs to startup, not a LOAD.
        if (bootstrapProgramLoad) return true;
        if (alarms.HasAlarm() || alarms.GetUpdateCount() != alarmRevision ||
            m_motion.HasPendingSafetyOrRecoveryRequests() ||
            m_lifecycleInterruptionShadow.IsActive() ||
            (owner.owner != MotionOwner::NONE && owner.owner != MotionOwner::IDLE_HOLD))
            return false;
        const NCLifecycleInterruptionSample sample = BuildLifecycleInterruptionSample();
        if (sample.activeBlocks != 0U || sample.axisCommandDepth != 0U ||
            sample.axisResultDepth != 0U || sample.commandQueueDepth != 0U ||
            sample.commandIngressDepth != 0U || sample.commandReplayDepth != 0U ||
            sample.feedbackDepth != 0U || sample.feedbackNoticeDepth != 0U ||
            sample.lastPublishedFeedbackSequence != sample.lastConsumedFeedbackSequence ||
            sample.waitCallbackActive || sample.completionBindingActive ||
            sample.safetyOrRecoveryPending || !m_motion.IsGroupNCDrained()) return false;
        if (owner.owner == MotionOwner::IDLE_HOLD)
        {
            MotionNCSettleSnapshot proof{};
            MotionNCSettleCounters counters{};
            if (!m_motion.TryGetNCSettleEvidence(MotionNCSettleProfile::GROUP_COMPLETION,
                proof, counters) ||
                proof.profile != MotionNCSettleProfile::GROUP_COMPLETION ||
                proof.publicationGeneration == 0ULL || proof.proofSequence == 0ULL ||
                proof.executionEpoch != originEpoch || proof.owner != owner.owner ||
                proof.ownerGeneration != owner.generation || proof.scopeMask == 0U ||
                !proof.runtimeObserved || !proof.runtimeCycleValid ||
                !proof.runtimeCycleContiguous || !proof.groupDrained || proof.groupActive ||
                !proof.settled || proof.safetyOrRecoveryPending ||
                proof.requiredCycles != MOTION_NC_SETTLE_REQUIRED_CYCLES ||
                proof.dwellCycles < proof.requiredCycles || proof.commandQueueDepth != 0U ||
                proof.commandIngressDepth != 0U || proof.commandReplayDepth != 0U)
                return false;
        }
        const MotionOwnerLease checkedOwner = m_motion.GetMotionOwnerLease();
        return checkedOwner.owner == originOwner.owner &&
            checkedOwner.generation == originOwner.generation &&
            m_motion.GetCurrentExecutionEpoch() == originEpoch &&
            m_state == originState && !m_motion.HasPendingSafetyOrRecoveryRequests();
    };
    if (!contextCurrent()) return RejectProgramLoad("CONTEXT_BUSY", filepath);

    NCProgramCache newProgramCache;
    std::string newProgramName;
    std::unique_ptr<MacroEngine> newMacroState;
    try
    {
        std::ifstream file(filepath);
        if (!file.is_open()) return RejectProgramLoad("FILE_OPEN", filepath);
        std::vector<std::string> rawLines;
        std::string line;
        while (std::getline(file, line)) rawLines.push_back(line);
        if (file.bad()) return RejectProgramLoad("FILE_READ", filepath);
        if (!newProgramCache.Build(std::move(rawLines), Parser))
            return RejectProgramLoad("CACHE_BUILD", filepath);
        const std::size_t pos = filepath.find_last_of("/\\");
        newProgramName = pos == std::string::npos ? filepath : filepath.substr(pos + 1U);
        newMacroState.reset(new MacroEngine());
    }
    catch (...)
    {
        return RejectProgramLoad("IMAGE_ALLOCATION", filepath);
    }

    if (!contextCurrent()) return RejectProgramLoad("CONTEXT_CHANGED", filepath);
    const auto committedContextCurrent = [&]() noexcept -> bool
    {
        const MotionOwnerLease owner = m_motion.GetMotionOwnerLease();
        return m_mode == originMode && owner.owner == originOwner.owner &&
            owner.generation == originOwner.generation &&
            m_motion.GetCurrentExecutionEpoch() == originEpoch &&
            (bootstrapProgramLoad || (!alarms.HasAlarm() &&
                alarms.GetUpdateCount() == alarmRevision &&
                !m_motion.HasPendingSafetyOrRecoveryRequests()));
    };
    AlarmManager::MotionAdmissionReservation admission{};
    if (!bootstrapProgramLoad && !alarms.BeginMotionAdmission(alarmRevision, admission))
        return RejectProgramLoad("ADMISSION_BUSY", filepath);
    const auto endAdmission = [&]() noexcept -> bool
    {
        return bootstrapProgramLoad || alarms.EndMotionAdmission(admission);
    };
    if (!contextCurrent() ||
        (!bootstrapProgramLoad && !alarms.IsMotionAdmissionCurrent(admission)))
    {
        (void)endAdmission();
        return RejectProgramLoad("ADMISSION_CHANGED", filepath);
    }
    // The reservation protects only bounded swaps. In particular, neither
    // allocation nor old-cache destruction may stall RT holding output here.
    m_programLoadStartBlocked = true;
    m_programCache.Swap(newProgramCache);
    m_mainProgramName.swap(newProgramName);
    MacroSys.SwapLocalState(*newMacroState);
    const bool admissionEnded = endAdmission();
    if (!admissionEnded || !committedContextCurrent() || m_state != originState)
        return RejectProgramLoad("POST_COMMIT_SUPERSEDED", filepath);

    if (!bootstrapProgramLoad)
    {
        BeginLifecycleInterruptionShadow(
            NCLifecycleInterruptionCause::PROGRAM_REPLACED, false);
        CancelProgramEndBoundary();
        ClearCompletionWaitBoundary(true);
        CancelGMBlockTransaction(true);
        CancelSingleBlockShadow(true);
        CancelFeedHoldBoundaryShadow(true);
        m_waitCallback = nullptr;
    }
    m_programMotionLease = MotionOwnerLease{};
    m_macroStack.clear();
    m_macroProgramCaches.clear();
    m_macroProgramName.clear();
    m_macroProgramPC = -1;
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



    NCState expectedState = originState;
    const bool stateCommitted = committedContextCurrent() &&
        m_state.compare_exchange_strong(expectedState, NCState::READY,
            std::memory_order_acq_rel, std::memory_order_acquire);
    if (!stateCommitted || !committedContextCurrent() ||
        m_state != NCState::READY)
    {
        // A late safety change may follow the inert image commit; it may not
        // become a successful LOAD or authorize START of this installed image.
        return RejectProgramLoad("POST_COMMIT_SUPERSEDED", filepath);
    }
    m_programLoadStartBlocked = false;
    m_bootProgramImageLoaded = true;
    RtPrintf("[NC-LOAD-CJ] LOADED file=%.240s cache=%llu owner=%u generation=%u epoch=%u keepHold=%u startBlocked=0\n",
        m_mainProgramName.c_str(), static_cast<unsigned long long>(m_programCache.GetGeneration()),
        static_cast<unsigned>(originOwner.owner), originOwner.generation, originEpoch,
        originOwner.owner == MotionOwner::IDLE_HOLD ? 1U : 0U);
    return true;
}


void NCManager::ChangeMode(NCOperationMode newMode)
{
    // 只有在 IDLE 或 READY 狀態才能切換模式
    if (m_state == NCState::IDLE || m_state == NCState::READY || m_state == NCState::P_END) {
        CancelGapDryRunSameThread("MODE_CHANGE");
        FencePathCoreLiveRetentionSameThread(PathCoreLiveFenceReason::MODE_CHANGE);
        InvalidatePathCoreFeedSameThread(); // BX-FEED
        InvalidatePathCoreArcSameThread(); // BY-ARC
        InvalidatePathCoreReplaySameThread(); // BZ: revoke saved geometry and active replay.
        InvalidatePathCoreHoldSameThread(); // CB: revoke unconsumed arm and original-source excursion.
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
    if (newState == NCState::HOLD) PauseGapDryRunSameThread("STATE_HOLD");
    else if (newState != NCState::RUN) CancelGapDryRunSameThread("STATE_CHANGE");
    // BN: public state writes cannot arm or resume retained geometry.
    if (newState == NCState::HOLD)
        PausePathCoreLiveRetentionSameThread();
    else if (newState != NCState::RUN)
    {
        FencePathCoreLiveRetentionSameThread(PathCoreLiveFenceReason::STATE_CHANGE);
        InvalidatePathCoreFeedSameThread(); // BX-FEED
        InvalidatePathCoreArcSameThread(); // BY-ARC
        InvalidatePathCoreReplaySameThread(); // BZ: revoke saved geometry and active replay.
        InvalidatePathCoreHoldSameThread(); // CB: revoke unconsumed arm and original-source excursion.
    }
    m_state = newState;
}

bool NCManager::TryCommitHomingResume(
    const MotionOwnerLease& expectedHomeLease) noexcept
{
    if (!expectedHomeLease.IsValid() ||
        !m_motion.IsMotionOwnerLeaseCurrent(expectedHomeLease) ||
        AlarmManager::GetInstance().HasAlarm())
    {
        return false;
    }

    NCState expectedState = NCState::HOLD;
    if (!m_state.compare_exchange_strong(
        expectedState,
        NCState::RUN,
        std::memory_order_acq_rel,
        std::memory_order_acquire))
    {
        return false;
    }

    if (m_motion.IsMotionOwnerLeaseCurrent(expectedHomeLease) &&
        !AlarmManager::GetInstance().HasAlarm())
    {
        return true;
    }

    // Roll back only our own provisional RUN.  A concurrent Reset/Alarm
    // state always wins and is never overwritten with HOLD.
    NCState provisionalRun = NCState::RUN;
    (void)m_state.compare_exchange_strong(
        provisionalRun,
        NCState::HOLD,
        std::memory_order_acq_rel,
        std::memory_order_acquire);
    return false;
}

void NCManager::TryRollbackHomingResume() noexcept
{
    NCState provisionalRun = NCState::RUN;
    (void)m_state.compare_exchange_strong(
        provisionalRun,
        NCState::HOLD,
        std::memory_order_acq_rel,
        std::memory_order_acquire);
}

// ==========================================
// 🌟 升級版 CycleStart (支援 M30 P_END 乾淨重啟)
// ==========================================
void NCManager::CycleStart()
{
    if (m_programLoadStartBlocked && m_mode == NCOperationMode::MEMORY &&
        (m_state == NCState::READY || m_state == NCState::P_END || m_state == NCState::IDLE))
    {
        RtPrintf("[NC-LOAD-CJ] START_BLOCKED current=%.240s action=LOAD_OR_RESET\n",
            m_mainProgramName.c_str());
        return;
    }
    CancelPathCoreHoldAutomaticSameThread("MANUAL_START");
    // A late Alarm may be published after the preceding ProcessTask sample.
    // No HOLD resume or READY/P_END start may acquire/program Motion before
    // the next periodic alarm trap observes it.
    if (AlarmManager::GetInstance().HasAlarm() ||
        m_state == NCState::ALARM)
    {
        return;
    }

    // =========================================================
    // G81 HOME Resume
    //
    // HOME Feed Hold 不再 Cancel Request。
    // m_active 保持 true，G81 Wait Callback 仍卡在原行。
    // =========================================================

    if (m_state == NCState::HOLD &&
        Homing.IsActive())
    {
        const HomingManager::ResumeResult resumeResult =
            Homing.Resume();

        if (resumeResult == HomingManager::ResumeResult::REJECTED ||
            resumeResult == HomingManager::ResumeResult::SUPERSEDED)
        {
            return;
        }

        ObserveFeedHoldResumeRequestedShadow();

        // Homing owns the exact PAUSED->RUNNING commit and never returns
        // APPLIED until its Alarm/owner admission has completed.  A request
        // made during controlled deceleration remains latched and is applied
        // asynchronously at that same protected seam.
        if (resumeResult == HomingManager::ResumeResult::APPLIED)
        {
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
        if (!ArmHoldResumeAlarmAdmission(
            HoldResumeAdmissionKind::CONTROLLED_SINGLE_BLOCK))
        {
            return;
        }
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
        if (!ArmHoldResumeAlarmAdmission(
            HoldResumeAdmissionKind::PROGRAM_HOLD))
        {
            return;
        }
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
                NCFeedHoldResumeGateRequestResult::DEFERRED)
            {
                m_holdResumeGateControlled = true;
                // ACK 前只鎖存 Cycle Start。保持 HOLD、Override=0、
                // Callback/PC/Queue 全部原封不動。
                m_pauseAfterBlock = false;
                return;
            }

            if (gateResult ==
                NCFeedHoldResumeGateRequestResult::BLOCKED)
            {
                ClearHoldResumeAlarmAdmission(
                    HoldResumeAdmissionKind::PROGRAM_HOLD);
                m_pauseAfterBlock = false;
                return;
            }

            if (gateResult ==
                NCFeedHoldResumeGateRequestResult::APPLY_NOW)
            {
                m_holdResumeGateControlled = true;
                // 已 ACK：同一個 Gate 立即套用 Resume。
                (void)ApplyProgramHoldResume(true);
                return;
            }

            // BYPASS_LEGACY 只可能發生在 Runtime 回退或 Boundary 已
            // 不再是 PROGRAM Feed Hold，沿用既有 Resume 行為。
            m_holdResumeGateControlled = false;
        }

        m_holdResumeGateControlled = false;
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

        AlarmManager& startAlarms = AlarmManager::GetInstance();
        const std::uint32_t alarmUpdateBefore =
            startAlarms.GetUpdateCount();
        const std::uint64_t alarmIntentBefore =
            AlarmManager::MotionAdmissionBaseState(
                startAlarms.GetMotionSafetyIntentState());
        const bool alarmPresent = startAlarms.HasAlarm();
        const std::uint32_t alarmUpdateAfter =
            startAlarms.GetUpdateCount();
        const std::uint64_t alarmIntentAfter =
            AlarmManager::MotionAdmissionBaseState(
                startAlarms.GetMotionSafetyIntentState());
        if (alarmPresent ||
            alarmUpdateBefore != alarmUpdateAfter ||
            alarmIntentBefore != alarmIntentAfter)
        {
            return;
        }

        // Latch the operator edge before taking Motion owner/Epoch authority.
        // A benign PDO Alarm reservation is retried by ProcessTask against
        // this immutable revision; a later Alarm can only cancel it.
        m_pendingProgramRunPhase =
            ProgramRunStartPhase::ALARM_ADMISSION;
        m_pendingProgramRunExecutionEpoch =
            MOTION_EXECUTION_EPOCH_INVALID;
        m_pendingProgramRunOwnerLease = MotionOwnerLease{};
        m_pendingProgramRunMode = m_mode;
        m_pendingProgramRunOriginState = m_state;
        m_pendingProgramRunScope = GetBaseProgramScope();
        m_pendingProgramRunCacheGeneration =
            GetBaseProgramCache().GetGeneration();
        m_pendingProgramRunAlarmUpdateCount = alarmUpdateBefore;
        m_pendingProgramRunAlarmSafetyIntentState = alarmIntentBefore;
        m_programRunStartPending = true;
        (void)ProcessPendingProgramRunStart();
    }
}

void NCManager::FeedHold()
{
    PauseGapDryRunSameThread("MANUAL_HOLD");
    // Operator Hold revokes an automatic resume even when already in HOLD.
    CancelPathCoreHoldAutomaticSameThread("MANUAL_HOLD");
    if (m_state == NCState::HOLD)
    {
        // A later HOLD also supersedes a manual START waiting for RT proof.
        // Retain the physical stop boundary so a fresh START can request
        // admission again; do not create or acknowledge another stop here.
        m_feedHoldResumeGate.Cancel(true);
        ClearHoldResumeAlarmAdmission(
            HoldResumeAdmissionKind::PROGRAM_HOLD);
    }
    FeedHoldInternal();
}

void NCManager::FeedHoldInternal()
{
    // =========================================================
    // G81 HOME Feed Hold
    //
    // Hold：可 Resume。
    // Reset / Alarm：不可 Resume。
    // =========================================================

    if (Homing.IsActive())
    {
        FencePathCoreLiveRetentionSameThread(PathCoreLiveFenceReason::HOME);
        InvalidatePathCoreFeedSameThread(); // BX-FEED
        InvalidatePathCoreArcSameThread(); // BY-ARC
        InvalidatePathCoreReplaySameThread(); // BZ: revoke saved geometry and active replay.
        InvalidatePathCoreHoldSameThread(); // CB: revoke unconsumed arm and original-source excursion.
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
        PausePathCoreLiveRetentionSameThread();
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
    m_programLoadStartBlocked = false;
    CancelGapDryRunSameThread("RESET");
    // BN: revoke only commanded-retention reads at the operator boundary.
    FencePathCoreLiveRetentionSameThread(PathCoreLiveFenceReason::RESET);
    InvalidatePathCoreFeedSameThread(); // BX-FEED
    InvalidatePathCoreArcSameThread(); // BY-ARC
    InvalidatePathCoreReplaySameThread(); // BZ: revoke saved geometry and active replay.
    // CL_FIX1: revoke all NC arm/resume state at the button boundary, including
    // bound, so the next observer scan cannot cancel Motion ahead of RESET.
    // RT still needs its immutable excursion/union geometry while the old
    // AUTO source is authorized or a controlled stop is decelerating.
    InvalidatePathCoreHoldSameThread(false);
    const bool waitingForPreResetControlledStop =
        m_resetContinuationPhase ==
        ResetContinuationPhase::PRE_RESET_CONTROLLED_STOP;
    const bool waitingForResetButtonAdmission =
        m_resetContinuationPhase ==
        ResetContinuationPhase::BUTTON_ADMISSION;
    const bool waitingForResetOutputHold =
        m_resetContinuationPhase ==
        ResetContinuationPhase::OUTPUT_HOLD;
    bool continuingResetTransaction =
        waitingForPreResetControlledStop ||
        waitingForResetButtonAdmission ||
        waitingForResetOutputHold ||
        m_resetContinuationPhase ==
        ResetContinuationPhase::PRE_DRAIN ||
        m_resetContinuationPhase ==
        ResetContinuationPhase::AUTHORITY ||
        m_resetContinuationPhase ==
        ResetContinuationPhase::EPOCH ||
        m_resetContinuationPhase ==
        ResetContinuationPhase::BATCH ||
        m_resetContinuationPhase ==
        ResetContinuationPhase::ALARM_CLEAR ||
        m_resetContinuationPhase ==
        ResetContinuationPhase::CLEANUP ||
        m_resetContinuationPhase ==
        ResetContinuationPhase::SETTLE;
    bool bypassResetStateIdempotence = false;

    if (m_resetContinuationPhase ==
        ResetContinuationPhase::CLEANUP)
    {
        // The NC task which changed ALARM_CLEAR -> CLEANUP owns the bounded
        // destructive reset section synchronously. A concurrent Reset call
        // may observe this phase but can never re-enter that section.
        return;
    }

    if (waitingForPreResetControlledStop)
    {
        // The Reset-controlled-stop ingress is deliberately pre-ticket until
        // the 250 us runtime owns it.  Keep RESET_STATE stable during that
        // short handoff as well as throughout the exact SAFETY stop ticket.
        // Do not overlap this pre-phase with Reset's PDO output-hold/batch:
        // the whole-PDO hold is for fault recovery and would turn a normal
        // G00 deceleration into an abrupt zero-command stop.
        const ResetControlledStopPhase resetStopPhase =
            m_motion.GetResetControlledStopPhase();
        if (resetStopPhase == ResetControlledStopPhase::PENDING ||
            resetStopPhase == ResetControlledStopPhase::APPLYING ||
            resetStopPhase == ResetControlledStopPhase::ACTIVE ||
            m_motion.HasPendingSafetyOrRecoveryRequests())
        {
            return;
        }

        // Only a physically settled RT-controlled stop may continue into the
        // original Reset output-hold/batch transaction. A real Alarm/E-stop
        // or any exact-ticket race supersedes this button press; require a
        // fresh explicit Reset after the operator has handled that condition.
        if (resetStopPhase != ResetControlledStopPhase::COMPLETED ||
            AlarmManager::GetInstance().HasAlarm() ||
            !m_motion.ConsumeCompletedResetControlledStop())
        {
            m_resetContinuationPhase = ResetContinuationPhase::BLOCKED;
            m_resetLifecycleBoundaryPrearmedByControlledStop = false;
            m_programMotionLease = MotionOwnerLease{};
            m_state = NCState::RESET_STATE;
            return;
        }

        // The controlled stop has been RT-completed.  Fall through to start
        // the original proven Reset transaction while retaining RESET_STATE;
        // an HMI producer must never see a transient IDLE window between the
        // StopGroup ticket and the Reset button admission.
        m_resetContinuationPhase = ResetContinuationPhase::IDLE;
        continuingResetTransaction = false;
        bypassResetStateIdempotence = true;
    }

    // Stage NC-0.2J.5.1：Reset safety batch 尚未完成時，重複 Reset 必須
    // 保持冪等。只有 release gate 已進入 terminal BLOCKED，操作員再次
    // 明確按 Reset 才建立新的 Epoch / request / gate transaction。
    if (!continuingResetTransaction &&
        m_state == NCState::RESET_STATE)
    {
        if (!bypassResetStateIdempotence)
        {
            const NCResetReleaseGateSnapshot resetGate =
                m_resetReleaseGate.GetSnapshot();
            const bool terminalBlockedReset =
                m_resetContinuationPhase ==
                ResetContinuationPhase::BLOCKED ||
                !resetGate.active &&
                resetGate.blocked &&
                resetGate.phase == NCResetReleaseGatePhase::BLOCKED;
            if (!terminalBlockedReset)
            {
                return;
            }

            // A prior G00 RESET may have been superseded by Alarm/E-stop.
            // This is a *new explicit* RESET press after the terminal block,
            // so it may retire the bookkeeping outcome and start the normal
            // fail-closed RESET batch.  It does not re-arm the old smooth
            // stop or release any Safety owner.
            (void)m_motion.RetireSupersededResetControlledStop();
            m_resetLifecycleBoundaryPrearmedByControlledStop = false;
        }
    }

    if (!continuingResetTransaction)
    {
        // Do not apply Reset's whole-PDO zero-output hold across an active
        // G00/G01 group.  A RUN state can also exist before its first command
        // is consumed or after a group has already drained; issuing a no-op
        // StopGroup in either case would create a needless safety transition.
        // Therefore only the coherent RT observation of an active group uses
        // the controlled-deceleration pre-phase.  All other cases retain the
        // original Reset safety-output-hold transaction.
        const MotionStopSettleSnapshot resetStopSnapshot =
            m_motion.GetStopSettleSnapshot();
        const bool cleanProgramRunReset =
            m_state == NCState::RUN &&
            resetStopSnapshot.publicationGeneration != 0ULL &&
            resetStopSnapshot.groupActive &&
            !m_resetSafetyOutputHoldActive &&
            !AlarmManager::GetInstance().HasAlarm();
        if (cleanProgramRunReset)
        {
            // NC-0.2K.7.1.1: the operator button is the lifecycle cutoff.
            // K.7.1 can have one STARTED command plus one accepted read-ahead
            // command at this instant.  The controlled-stop pre-phase may
            // publish ABORTED for the former and STALE_EPOCH/OWNER_CONFLICT
            // REJECTED for the latter before the original Reset batch begins.
            // Capture the exact old execution lease and both active Ledger
            // blocks now; reopening this boundary after deceleration would
            // lose that immutable correlation and misreport the expected
            // retirement as a transport failure.
            BeginLifecycleInterruptionShadow(
                NCLifecycleInterruptionCause::RESET,
                true);
            const NCLifecycleInterruptionSnapshot preStopBoundary =
                m_lifecycleInterruptionShadow.GetSnapshot();
            m_resetLifecycleInterruptionSequence =
                preStopBoundary.active &&
                preStopBoundary.cause ==
                NCLifecycleInterruptionCause::RESET
                ? preStopBoundary.sequence
                : 0ULL;
            if (m_resetLifecycleInterruptionSequence == 0ULL)
            {
                m_resetContinuationPhase =
                    ResetContinuationPhase::BLOCKED;
                m_resetLifecycleBoundaryPrearmedByControlledStop = false;
                m_programMotionLease = MotionOwnerLease{};
                m_state = NCState::RESET_STATE;
                m_motion.RequestEmergencyStopAllAxes();
                return;
            }
            m_resetLifecycleBoundaryPrearmedByControlledStop = true;
            m_resetContinuationPhase =
                ResetContinuationPhase::PRE_RESET_CONTROLLED_STOP;
            m_programMotionLease = MotionOwnerLease{};
            m_state = NCState::RESET_STATE;
            m_motion.RequestResetControlledStop();
            return;
        }

        AlarmManager& resetButtonAlarms = AlarmManager::GetInstance();
        m_resetAuthorityAlarmUpdateCount =
            resetButtonAlarms.GetUpdateCount();
        m_resetAuthorityAlarmSafetyIntentState =
            AlarmManager::MotionAdmissionBaseState(
                resetButtonAlarms.GetMotionSafetyIntentState());
        m_resetAuthorityMappingAlarmRequestCount =
            m_lastHandledMappingIntegrityAlarmRequestCount;
        if (!m_resetSafetyOutputHoldActive)
        {
            m_resetButtonCutoffProvenanceGeneration =
                m_motion.BeginResetSafetyOutputHold();
            m_resetSafetyOutputHoldActive = true;
        }
        else
        {
            // A new operator press after terminal BLOCKED reuses the already
            // active zero-output hold but owns a fresh immutable cutoff.
            m_resetButtonCutoffProvenanceGeneration =
                m_motion.GetSafetyProvenanceGeneration();
        }

        // A frame which was already at its physical send point may own the
        // Alarm reservation for a few microseconds.  Latch the button once
        // and retry that exact revision/base instead of asking for another
        // operator press.  Any Alarm sequence drift remains terminal.
        m_resetContinuationPhase =
            ResetContinuationPhase::BUTTON_ADMISSION;
        m_programMotionLease = MotionOwnerLease{};
        m_state = NCState::RESET_STATE;
    }

    if (m_resetContinuationPhase ==
        ResetContinuationPhase::BUTTON_ADMISSION)
    {
        AlarmManager& resetButtonAlarms = AlarmManager::GetInstance();
        AlarmManager::MotionAdmissionReservation resetButtonAdmission{};
        const AlarmManager::MotionAdmissionResult admissionResult =
            resetButtonAlarms.TryBeginMotionAdmission(
                m_resetAuthorityAlarmUpdateCount,
                m_resetAuthorityAlarmSafetyIntentState,
                resetButtonAdmission,
                false);
        if (admissionResult ==
            AlarmManager::MotionAdmissionResult::BUSY)
        {
            return;
        }
        if (admissionResult !=
            AlarmManager::MotionAdmissionResult::ACQUIRED ||
            resetButtonAdmission.baseState !=
            m_resetAuthorityAlarmSafetyIntentState ||
            m_motion.GetSafetyProvenanceGeneration() !=
            m_resetButtonCutoffProvenanceGeneration)
        {
            if (resetButtonAdmission.acquired)
            {
                (void)resetButtonAlarms.EndMotionAdmission(
                    resetButtonAdmission);
            }
            m_resetContinuationPhase =
                ResetContinuationPhase::BLOCKED;
            m_motion.RequestEmergencyStopAllAxes();
            return;
        }

        if (m_lastHandledMappingIntegrityAlarmRequestCount !=
            m_resetAuthorityMappingAlarmRequestCount)
        {
            (void)resetButtonAlarms.EndMotionAdmission(
                resetButtonAdmission);
            m_resetContinuationPhase =
                ResetContinuationPhase::BLOCKED;
            m_motion.RequestEmergencyStopAllAxes();
            return;
        }

        m_resetNCSettleRequestSequence =
            MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID;

        const NCLifecycleInterruptionSnapshot existingResetBoundary =
            m_lifecycleInterruptionShadow.GetSnapshot();
        const bool controlledStopBoundaryExpected =
            m_resetLifecycleBoundaryPrearmedByControlledStop;
        const bool reuseControlledStopBoundary =
            controlledStopBoundaryExpected &&
            m_resetLifecycleInterruptionSequence != 0ULL &&
            existingResetBoundary.active &&
            existingResetBoundary.cause ==
            NCLifecycleInterruptionCause::RESET &&
            existingResetBoundary.sequence ==
            m_resetLifecycleInterruptionSequence;

        // A pre-armed boundary which lost coherence cannot be silently
        // replaced after its terminal feedback has already passed.  Preserve
        // that evidence gap and stop closed; only a new explicit Reset after
        // BLOCKED may open a fresh transaction.
        if (controlledStopBoundaryExpected &&
            !reuseControlledStopBoundary)
        {
            (void)resetButtonAlarms.EndMotionAdmission(
                resetButtonAdmission);
            m_resetLifecycleBoundaryPrearmedByControlledStop = false;
            m_resetContinuationPhase =
                ResetContinuationPhase::BLOCKED;
            m_motion.RequestEmergencyStopAllAxes();
            return;
        }

        if (!controlledStopBoundaryExpected)
        {
            BeginLifecycleInterruptionShadow(
                NCLifecycleInterruptionCause::RESET,
                true);

            const NCLifecycleInterruptionSnapshot resetBoundary =
                m_lifecycleInterruptionShadow.GetSnapshot();
            m_resetLifecycleInterruptionSequence =
                resetBoundary.active &&
                resetBoundary.cause ==
                NCLifecycleInterruptionCause::RESET
                ? resetBoundary.sequence
                : 0ULL;
        }
        m_resetLifecycleBoundaryPrearmedByControlledStop = false;

        if (m_resetLifecycleInterruptionSequence == 0ULL)
        {
            (void)resetButtonAlarms.EndMotionAdmission(
                resetButtonAdmission);
            m_resetContinuationPhase =
                ResetContinuationPhase::BLOCKED;
            m_motion.RequestEmergencyStopAllAxes();
            return;
        }

        // RESET now owns the exact lifecycle boundary even when the button
        // was pressed from an already-latched Alarm. Clear only the generic
        // latch marker (not AlarmManager) so a genuinely later Alarm can
        // supersede this immutable button transaction.
        m_lifecycleInterruptionAlarmLatched = false;

        CancelProgramEndBoundary();
        if (Homing.IsActive()) Homing.Cancel();

        m_resetAuthorityEntryGeneration =
            MOTION_OWNER_GENERATION_INVALID;
        m_resetAuthorityBaselineTicket = 0U;
        m_resetAuthorityRequestTicket = 0U;
        m_resetAuthorityProvenanceGeneration = 0ULL;
        m_resetContinuationExecutionEpoch =
            MOTION_EXECUTION_EPOCH_INVALID;
        m_resetContinuationExecutionState =
            MotionNCResetExecutionState{};

        // Fail closed immediately at the button cutoff. The physical Motion
        // authority edge is intentionally deferred until any already-finalized
        // NIC frame has completed, but these baselines are never recaptured.
        if (!resetButtonAlarms.EndMotionAdmission(
            resetButtonAdmission) ||
            resetButtonAlarms.GetUpdateCount() !=
            m_resetAuthorityAlarmUpdateCount ||
            AlarmManager::MotionAdmissionBaseState(
                resetButtonAlarms.GetMotionSafetyIntentState()) !=
            m_resetAuthorityAlarmSafetyIntentState ||
            m_motion.GetSafetyProvenanceGeneration() !=
            m_resetButtonCutoffProvenanceGeneration)
        {
            m_resetContinuationPhase =
                ResetContinuationPhase::BLOCKED;
            m_motion.RequestEmergencyStopAllAxes();
            return;
        }

        m_resetContinuationPhase =
            ResetContinuationPhase::OUTPUT_HOLD;
    }

    if (m_resetContinuationPhase ==
        ResetContinuationPhase::OUTPUT_HOLD)
    {
        const ResetPreDrainReconcileResult firstReconcile =
            ReconcileResetPreDrainMappingAlarmBoundary();
        if (firstReconcile ==
            ResetPreDrainReconcileResult::DEFERRED)
        {
            return;
        }
        if (firstReconcile ==
            ResetPreDrainReconcileResult::SUPERSEDED)
        {
            m_resetContinuationPhase =
                ResetContinuationPhase::BLOCKED;
            m_motion.RequestEmergencyStopAllAxes();
            return;
        }

        // Every producer which was already in flight at the button cutoff
        // must finish before the physical authority edge. A later producer
        // changes the immutable cutoff and is rejected by Reconcile above.
        if (m_motion.HasPendingSafetyOrRecoveryRequests() ||
            !m_motion.IsResetSafetyOutputHoldEstablished())
        {
            return;
        }

        // Close publisher-end, Alarm and mapping seams one last time before
        // the physical edge. No waiting scan may rebuild these baselines.
        const ResetPreDrainReconcileResult finalOutputReconcile =
            ReconcileResetPreDrainMappingAlarmBoundary();
        if (finalOutputReconcile ==
            ResetPreDrainReconcileResult::DEFERRED)
        {
            return;
        }
        if (finalOutputReconcile ==
            ResetPreDrainReconcileResult::SUPERSEDED)
        {
            m_resetContinuationPhase =
                ResetContinuationPhase::BLOCKED;
            m_motion.RequestEmergencyStopAllAxes();
            return;
        }

        const std::uint64_t expectedAuthorityGeneration =
            m_resetButtonCutoffProvenanceGeneration + 1ULL;
        const std::uint64_t authorityGeneration =
            m_motion.MarkResetSafetyOperatorEdge();
        const NCLifecycleInterruptionSnapshot authorityBoundary =
            m_lifecycleInterruptionShadow.GetSnapshot();
        if (expectedAuthorityGeneration == 0ULL ||
            authorityGeneration != expectedAuthorityGeneration ||
            m_motion.GetSafetyProvenanceGeneration() !=
            authorityGeneration ||
            AlarmManager::GetInstance().GetUpdateCount() !=
            m_resetAuthorityAlarmUpdateCount ||
            AlarmManager::MotionAdmissionBaseState(
                AlarmManager::GetInstance().
                GetMotionSafetyIntentState()) !=
            m_resetAuthorityAlarmSafetyIntentState ||
            m_lastHandledMappingIntegrityAlarmRequestCount !=
            m_resetAuthorityMappingAlarmRequestCount ||
            authorityBoundary.sequence !=
            m_resetLifecycleInterruptionSequence ||
            authorityBoundary.cause !=
            NCLifecycleInterruptionCause::RESET)
        {
            m_resetContinuationPhase =
                ResetContinuationPhase::BLOCKED;
            m_motion.RequestEmergencyStopAllAxes();
            return;
        }

        m_resetAuthorityProvenanceGeneration = authorityGeneration;
        m_resetContinuationPhase =
            ResetContinuationPhase::PRE_DRAIN;
    }

    if (m_resetContinuationPhase ==
        ResetContinuationPhase::PRE_DRAIN)
    {
        const ResetPreDrainReconcileResult firstPreDrainReconcile =
            ReconcileResetPreDrainMappingAlarmBoundary();
        if (firstPreDrainReconcile ==
            ResetPreDrainReconcileResult::DEFERRED)
        {
            return;
        }
        if (firstPreDrainReconcile ==
            ResetPreDrainReconcileResult::SUPERSEDED)
        {
            m_resetContinuationPhase =
                ResetContinuationPhase::BLOCKED;
            m_motion.RequestEmergencyStopAllAxes();
            return;
        }

        const NCLifecycleInterruptionSnapshot preDrainBoundary =
            m_lifecycleInterruptionShadow.GetSnapshot();
        if (m_resetLifecycleInterruptionSequence == 0ULL ||
            preDrainBoundary.sequence !=
            m_resetLifecycleInterruptionSequence ||
            preDrainBoundary.cause !=
            NCLifecycleInterruptionCause::RESET)
        {
            m_resetContinuationPhase =
                ResetContinuationPhase::BLOCKED;
            m_motion.RequestEmergencyStopAllAxes();
            return;
        }

        // The Reset baseline is captured only after every action which was
        // already pending at the operator edge has reached RT.  Otherwise an
        // old Stop/ResetFault action can publish H2 after this Reset recorded
        // the fresh G/H acknowledgement and force a second button press.
        if (m_motion.HasPendingSafetyOrRecoveryRequests())
        {
            return;
        }

        // Close the final publisher-end -> baseline-capture seam.  A mapping
        // producer which began before the operator edge may publish its NC
        // mailbox after the first reconciliation.  The Motion pending check
        // above now includes that mailbox; this second exact pass consumes it
        // before the immutable owner/ticket baseline is sampled.
        const ResetPreDrainReconcileResult finalPreDrainReconcile =
            ReconcileResetPreDrainMappingAlarmBoundary();
        if (finalPreDrainReconcile ==
            ResetPreDrainReconcileResult::DEFERRED)
        {
            return;
        }
        if (finalPreDrainReconcile ==
            ResetPreDrainReconcileResult::SUPERSEDED)
        {
            m_resetContinuationPhase =
                ResetContinuationPhase::BLOCKED;
            m_motion.RequestEmergencyStopAllAxes();
            return;
        }
        const MotionCore::ResetSafetyAuthorityResult baseline =
            m_motion.TryCaptureResetSafetyAuthorityBaseline(
                m_resetAuthorityProvenanceGeneration);
        if (baseline.status ==
            MotionCore::ResetSafetyAuthorityStatus::SUPERSEDED)
        {
            m_resetContinuationPhase =
                ResetContinuationPhase::BLOCKED;
            m_motion.RequestEmergencyStopAllAxes();
            return;
        }
        if (baseline.status !=
            MotionCore::ResetSafetyAuthorityStatus::ACQUIRED)
        {
            return;
        }

        m_resetAuthorityEntryGeneration =
            baseline.lease.generation;
        m_resetAuthorityBaselineTicket =
            baseline.requestTicket;
        m_resetContinuationPhase =
            ResetContinuationPhase::AUTHORITY;
    }

    if (m_resetContinuationPhase ==
        ResetContinuationPhase::AUTHORITY)
    {
        // Safety 取得新的 Generation，讓舊 AUTO / MDI 命令與晚到 Release
        // 失效。Every retry joins the original entry generation.
        const MotionCore::ResetSafetyAuthorityResult authority =
            m_motion.ContinueResetSafetyMotionOwnerGeneration(
                m_resetAuthorityEntryGeneration,
                m_resetAuthorityBaselineTicket,
                m_resetAuthorityRequestTicket,
                m_resetAuthorityProvenanceGeneration);
        m_resetAuthorityRequestTicket = authority.requestTicket;
        m_resetAuthorityProvenanceGeneration =
            authority.provenanceGeneration;

        if (authority.status ==
            MotionCore::ResetSafetyAuthorityStatus::SUPERSEDED)
        {
            m_resetContinuationPhase =
                ResetContinuationPhase::BLOCKED;
            m_motion.RequestEmergencyStopAllAxes();
            return;
        }
        if (authority.status !=
            MotionCore::ResetSafetyAuthorityStatus::ACQUIRED ||
            !authority.lease.IsValid() ||
            !m_motion.IsMotionOwnerLeaseCurrent(authority.lease))
        {
            return;
        }
        m_safetyMotionLease = authority.lease;
        m_resetContinuationPhase =
            ResetContinuationPhase::EPOCH;
        // Do not fall through.  The 250 us owner must first consume the
        // published SAFETY Epoch (and any older E-stop mailbox) before this
        // Reset is allowed to publish its correlated batch.
        return;
    }

    if (m_resetContinuationPhase ==
        ResetContinuationPhase::EPOCH)
    {
        // The fresh SAFETY-generation handshake already published the one
        // Reset Epoch. Latch its exact generation acknowledgement instead of
        // sampling a mutable live Epoch or publishing a second one.
        MotionExecutionEpoch acknowledgedResetEpoch =
            MOTION_EXECUTION_EPOCH_INVALID;
        const MotionCore::ResetSafetyAuthorityStatus epochStatus =
            m_motion.TryGetResetSafetyMotionOwnerEpoch(
                m_safetyMotionLease,
                m_resetAuthorityRequestTicket,
                m_resetAuthorityProvenanceGeneration,
                acknowledgedResetEpoch);
        if (epochStatus ==
            MotionCore::ResetSafetyAuthorityStatus::DEFERRED)
        {
            return;
        }
        if (epochStatus !=
            MotionCore::ResetSafetyAuthorityStatus::ACQUIRED)
        {
            // ContinueNewSafetyMotionOwnerGeneration() returns a valid lease
            // only after its exact owner/Epoch acknowledgement exists.  A
            // later mismatch therefore means another lifecycle publication
            // superseded this Reset; waiting cannot make the old tuple valid.
            m_resetContinuationPhase =
                ResetContinuationPhase::BLOCKED;
            m_motion.RequestEmergencyStopAllAxes();
            return;
        }
        m_resetContinuationExecutionEpoch =
            acknowledgedResetEpoch;
        m_resetContinuationPhase =
            ResetContinuationPhase::BATCH;
        return;
    }

    const MotionExecutionEpoch resetEpoch =
        m_resetContinuationExecutionEpoch;
    MotionExecutionEpoch exactResetEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    const MotionCore::ResetSafetyAuthorityStatus exactResetStatus =
        m_motion.TryGetResetSafetyMotionOwnerEpoch(
            m_safetyMotionLease,
            m_resetAuthorityRequestTicket,
            m_resetAuthorityProvenanceGeneration,
            exactResetEpoch);
    if (exactResetStatus ==
        MotionCore::ResetSafetyAuthorityStatus::DEFERRED)
    {
        return;
    }
    const NCLifecycleInterruptionSnapshot resetBoundary =
        m_lifecycleInterruptionShadow.GetSnapshot();
    const bool resetBoundaryIdentityCurrent =
        m_resetLifecycleInterruptionSequence != 0ULL &&
        resetBoundary.sequence ==
        m_resetLifecycleInterruptionSequence &&
        resetBoundary.cause ==
        NCLifecycleInterruptionCause::RESET &&
        (m_resetContinuationPhase ==
            ResetContinuationPhase::BATCH ||
            resetBoundary.publishedExecutionEpoch == resetEpoch);
    if ((m_resetContinuationPhase !=
        ResetContinuationPhase::BATCH &&
        m_resetContinuationPhase !=
        ResetContinuationPhase::ALARM_CLEAR &&
        m_resetContinuationPhase !=
        ResetContinuationPhase::SETTLE) ||
        resetEpoch == MOTION_EXECUTION_EPOCH_INVALID ||
        exactResetStatus !=
        MotionCore::ResetSafetyAuthorityStatus::ACQUIRED ||
        exactResetEpoch != resetEpoch ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_safetyMotionLease) ||
        m_motion.GetCurrentExecutionEpoch() != resetEpoch ||
        !resetBoundaryIdentityCurrent ||
        AlarmManager::GetInstance().GetUpdateCount() !=
        m_resetAuthorityAlarmUpdateCount ||
        AlarmManager::MotionAdmissionBaseState(
            AlarmManager::GetInstance().
            GetMotionSafetyIntentState()) !=
        m_resetAuthorityAlarmSafetyIntentState)
    {
        m_resetContinuationPhase =
            ResetContinuationPhase::BLOCKED;
        m_motion.RequestEmergencyStopAllAxes();
        return;
    }

    // Queue Full is a bounded producer result, not a reason to require a
    // second operator Reset.  BATCH performs every destructive/modal action
    // exactly once and saves the clean execution image.  SETTLE only retries
    // publishing that same immutable request until its SPSC release-push
    // succeeds, then and only then arms the release gate.
    const auto TryPublishResetSettleAndArmGate =
        [this, resetEpoch]() noexcept -> bool
    {
        if (m_resetContinuationPhase !=
            ResetContinuationPhase::SETTLE ||
            resetEpoch == MOTION_EXECUTION_EPOCH_INVALID ||
            !m_motion.IsMotionOwnerLeaseCurrent(m_safetyMotionLease) ||
            m_motion.GetCurrentExecutionEpoch() != resetEpoch ||
            m_resetLifecycleInterruptionSequence == 0ULL ||
            AlarmManager::GetInstance().GetUpdateCount() !=
            m_resetAuthorityAlarmUpdateCount ||
            AlarmManager::MotionAdmissionBaseState(
                AlarmManager::GetInstance().
                GetMotionSafetyIntentState()) !=
            m_resetAuthorityAlarmSafetyIntentState)
        {
            m_resetContinuationPhase =
                ResetContinuationPhase::BLOCKED;
            m_motion.RequestEmergencyStopAllAxes();
            return false;
        }

        MotionExecutionEpoch exactArmEpoch =
            MOTION_EXECUTION_EPOCH_INVALID;
        const MotionCore::ResetSafetyAuthorityStatus exactArmStatus =
            m_motion.TryGetResetSafetyMotionOwnerEpoch(
                m_safetyMotionLease,
                m_resetAuthorityRequestTicket,
                m_resetAuthorityProvenanceGeneration,
                exactArmEpoch);
        if (exactArmStatus ==
            MotionCore::ResetSafetyAuthorityStatus::DEFERRED)
        {
            return false;
        }
        if (exactArmStatus !=
            MotionCore::ResetSafetyAuthorityStatus::ACQUIRED ||
            exactArmEpoch != resetEpoch)
        {
            m_resetContinuationPhase =
                ResetContinuationPhase::BLOCKED;
            m_motion.RequestEmergencyStopAllAxes();
            return false;
        }

        const NCLifecycleInterruptionSnapshot armBoundary =
            m_lifecycleInterruptionShadow.GetSnapshot();
        if (armBoundary.sequence !=
            m_resetLifecycleInterruptionSequence ||
            armBoundary.cause !=
            NCLifecycleInterruptionCause::RESET ||
            armBoundary.publishedExecutionEpoch != resetEpoch)
        {
            m_resetContinuationPhase =
                ResetContinuationPhase::BLOCKED;
            m_motion.RequestEmergencyStopAllAxes();
            return false;
        }

        const MotionNCSettleRequestSequence requestSequence =
            m_motion.RequestResetNCSettleAndRebase(
                resetEpoch,
                m_safetyMotionLease,
                m_resetAuthorityRequestTicket,
                m_resetAuthorityProvenanceGeneration,
                m_resetContinuationExecutionState,
                false);
        if (requestSequence ==
            MOTION_NC_SETTLE_REQUEST_SEQUENCE_INVALID)
        {
            // Preserve SETTLE and the exact immutable tuple. ProcessTask()
            // retries automatically after pending safety work has drained.
            return false;
        }

        m_resetNCSettleRequestSequence = requestSequence;
        m_resetReleaseGate.Arm(
            armBoundary,
            resetEpoch,
            m_safetyMotionLease,
            m_resetNCSettleRequestSequence,
            m_motion.IsMotionOwnerLeaseCurrent(m_safetyMotionLease));
        m_resetContinuationPhase =
            ResetContinuationPhase::RELEASE_GATE;
        m_resetAuthorityEntryGeneration =
            MOTION_OWNER_GENERATION_INVALID;
        return true;
    };

    if (m_resetContinuationPhase ==
        ResetContinuationPhase::SETTLE)
    {
        if (m_motion.HasPendingSafetyOrRecoveryRequests())
        {
            return;
        }
        (void)TryPublishResetSettleAndArmGate();
        return;
    }

    if (m_motion.HasPendingSafetyOrRecoveryRequests())
    {
        return;
    }

    if (m_resetContinuationPhase ==
        ResetContinuationPhase::BATCH)
    {
        const bool resetNeedsFaultOrEstopRecovery =
            m_motion.IsAnyAxisFaulted() ||
            m_motion.IsGroupFaulted() ||
            m_motion.IsGroupEmergencyStopped();
        const MotionCore::ResetSafetyAuthorityResult resetBatch =
            m_motion.RequestExactResetSafetyBatch(
                resetEpoch,
                m_safetyMotionLease,
                m_resetAuthorityRequestTicket,
                m_resetAuthorityProvenanceGeneration,
                resetNeedsFaultOrEstopRecovery);
        m_resetAuthorityRequestTicket = resetBatch.requestTicket;
        m_resetAuthorityProvenanceGeneration =
            resetBatch.provenanceGeneration;
        if (resetBatch.status ==
            MotionCore::ResetSafetyAuthorityStatus::DEFERRED)
        {
            return;
        }
        if (resetBatch.status !=
            MotionCore::ResetSafetyAuthorityStatus::ACQUIRED)
        {
            m_resetContinuationPhase =
                ResetContinuationPhase::BLOCKED;
            m_motion.RequestEmergencyStopAllAxes();
            return;
        }

        RecordLifecycleInterruptionEpochPublished(resetEpoch);
        const NCLifecycleInterruptionSnapshot publishedResetBoundary =
            m_lifecycleInterruptionShadow.GetSnapshot();
        if (publishedResetBoundary.sequence !=
            m_resetLifecycleInterruptionSequence ||
            publishedResetBoundary.cause !=
            NCLifecycleInterruptionCause::RESET ||
            publishedResetBoundary.publishedExecutionEpoch != resetEpoch)
        {
            m_resetContinuationPhase =
                ResetContinuationPhase::BLOCKED;
            m_motion.RequestEmergencyStopAllAxes();
            return;
        }

        // Persist the fact that the child batch was published before any
        // Alarm-clear retry. A transient PDO reservation can delay the next
        // phase, but no scan is allowed to publish this batch a second time.
        m_resetContinuationPhase =
            ResetContinuationPhase::ALARM_CLEAR;
        return;
    }

    // The old Alarm image is acknowledged exactly once, only after the
    // correlated SAFETY batch is published and consumed. A frame reservation
    // is ordinary BUSY and retains ALARM_CLEAR; every Alarm sequence drift is
    // SUPERSEDED and therefore preserves the newer Alarm fail-closed.
    AlarmManager& resetAlarms = AlarmManager::GetInstance();
    AlarmManager::MotionAdmissionReservation clearAdmission{};
    const AlarmManager::MotionAdmissionResult clearAdmissionResult =
        resetAlarms.TryBeginMotionAdmission(
            m_resetAuthorityAlarmUpdateCount,
            m_resetAuthorityAlarmSafetyIntentState,
            clearAdmission,
            false);
    if (clearAdmissionResult ==
        AlarmManager::MotionAdmissionResult::BUSY)
    {
        return;
    }
    if (clearAdmissionResult !=
        AlarmManager::MotionAdmissionResult::ACQUIRED ||
        clearAdmission.baseState !=
        m_resetAuthorityAlarmSafetyIntentState)
    {
        m_resetContinuationPhase =
            ResetContinuationPhase::BLOCKED;
        m_motion.RequestEmergencyStopAllAxes();
        return;
    }

    MotionExecutionEpoch clearExactEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    const MotionCore::ResetSafetyAuthorityStatus clearExactStatus =
        m_motion.TryGetResetSafetyMotionOwnerEpoch(
            m_safetyMotionLease,
            m_resetAuthorityRequestTicket,
            m_resetAuthorityProvenanceGeneration,
            clearExactEpoch);
    const NCLifecycleInterruptionSnapshot clearBoundary =
        m_lifecycleInterruptionShadow.GetSnapshot();
    if (clearExactStatus !=
        MotionCore::ResetSafetyAuthorityStatus::ACQUIRED ||
        clearExactEpoch != resetEpoch ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_safetyMotionLease) ||
        m_motion.GetCurrentExecutionEpoch() != resetEpoch ||
        m_motion.HasPendingSafetyOrRecoveryRequests() ||
        clearBoundary.sequence !=
        m_resetLifecycleInterruptionSequence ||
        clearBoundary.cause !=
        NCLifecycleInterruptionCause::RESET ||
        clearBoundary.publishedExecutionEpoch != resetEpoch)
    {
        (void)resetAlarms.EndMotionAdmission(clearAdmission);
        m_resetContinuationPhase =
            ResetContinuationPhase::BLOCKED;
        m_motion.RequestEmergencyStopAllAxes();
        return;
    }

    if (!resetAlarms.ClearUnderMotionAdmission(clearAdmission))
    {
        (void)resetAlarms.EndMotionAdmission(clearAdmission);
        m_resetContinuationPhase =
            ResetContinuationPhase::BLOCKED;
        m_motion.RequestEmergencyStopAllAxes();
        return;
    }

    const std::uint32_t postClearAlarmUpdateCount =
        clearAdmission.expectedUpdateCount;
    const std::uint64_t postClearAlarmIntentState =
        clearAdmission.baseState;
    // Claim the destructive section before releasing the admission. The
    // owning NC call remains synchronous; every concurrent Reset/ProcessTask
    // observes CLEANUP and returns instead of repeating modal/cache cleanup.
    m_resetContinuationPhase = ResetContinuationPhase::CLEANUP;

    // CL_FIX1: the exact RESET batch has been consumed and the old AUTO
    // owner/epoch is fenced. Retire RT receipts here, once, before cleanup.
    // Publishing this generation at button entry races the still-authorized
    // excursion and correctly trips its orphan/mapping integrity guard.
    m_motion.CancelPathCoreHoldExcursion();


    //重置馬達區塊--------------------------------------------------
    // The correlated RT batch was already published above through the exact
    // parent-ticket -> child-ticket transition.  Only after that immutable
    // publication succeeds may this Reset perform destructive modal cleanup.

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
    m_resetContinuationExecutionState = resetExecutionState;

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

    // Keep the exact Alarm reservation across the bounded destructive NC
    // cleanup.  A producer which overlaps this section is never allowed to
    // wait; AlarmManager records it in the deferred slot.  End is therefore
    // the commit point: it promotes any overlap and makes this Reset
    // terminal BLOCKED instead of allowing the stale quiet image to reach
    // SETTLE.
    const bool clearAdmissionEnded =
        resetAlarms.EndMotionAdmission(clearAdmission);
    const std::uint32_t alarmUpdateAfterCleanup =
        resetAlarms.GetUpdateCount();
    const std::uint64_t alarmIntentAfterCleanup =
        AlarmManager::MotionAdmissionBaseState(
            resetAlarms.GetMotionSafetyIntentState());
    MotionExecutionEpoch cleanupExactEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    const MotionCore::ResetSafetyAuthorityStatus cleanupExactStatus =
        m_motion.TryGetResetSafetyMotionOwnerEpoch(
            m_safetyMotionLease,
            m_resetAuthorityRequestTicket,
            m_resetAuthorityProvenanceGeneration,
            cleanupExactEpoch);
    const NCLifecycleInterruptionSnapshot cleanupBoundary =
        m_lifecycleInterruptionShadow.GetSnapshot();
    if (!clearAdmissionEnded ||
        alarmUpdateAfterCleanup != postClearAlarmUpdateCount ||
        alarmIntentAfterCleanup != postClearAlarmIntentState ||
        resetAlarms.HasAlarm() ||
        cleanupExactStatus !=
        MotionCore::ResetSafetyAuthorityStatus::ACQUIRED ||
        cleanupExactEpoch != resetEpoch ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_safetyMotionLease) ||
        m_motion.GetCurrentExecutionEpoch() != resetEpoch ||
        m_motion.GetSafetyProvenanceGeneration() !=
        m_resetAuthorityProvenanceGeneration ||
        m_motion.HasPendingSafetyOrRecoveryRequests() ||
        m_lastHandledMappingIntegrityAlarmRequestCount !=
        m_resetAuthorityMappingAlarmRequestCount ||
        cleanupBoundary.sequence !=
        m_resetLifecycleInterruptionSequence ||
        cleanupBoundary.cause !=
        NCLifecycleInterruptionCause::RESET ||
        cleanupBoundary.publishedExecutionEpoch != resetEpoch)
    {
        m_resetContinuationPhase =
            ResetContinuationPhase::BLOCKED;
        m_state = NCState::RESET_STATE;
        m_motion.RequestEmergencyStopAllAxes();
        return;
    }

    // Alarm was cleared once at the BATCH safety boundary above.  Commit
    // the post-clear identity only after admission End proves that the whole
    // cleanup was quiet. SETTLE retries never clear Alarm again; every later
    // Alarm owns a newer lifecycle boundary and blocks this transaction.
    m_resetAuthorityAlarmUpdateCount = postClearAlarmUpdateCount;
    m_resetAuthorityAlarmSafetyIntentState = postClearAlarmIntentState;
    m_state = NCState::RESET_STATE;

    // All BATCH work above is exactly-once.  From this point a failed ring
    // push can only retry the saved tuple; it cannot clear modal/macro state
    // again or publish another Safety batch/Epoch.
    m_resetContinuationPhase =
        ResetContinuationPhase::SETTLE;
    // Do not publish the settle request in this same call.  The next scan
    // first proves that the correlated Reset safety batch has been consumed;
    // otherwise an older/higher-priority E-stop branch could clear the batch
    // after the settle request was already visible.
    return;





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
    FencePathCoreLiveRetentionSameThread(PathCoreLiveFenceReason::MACRO);
    InvalidatePathCoreFeedSameThread(); // BX-FEED
    InvalidatePathCoreArcSameThread(); // BY-ARC
    InvalidatePathCoreReplaySameThread(); // BZ: revoke saved geometry and active replay.
    InvalidatePathCoreHoldSameThread(); // CB: revoke unconsumed arm and original-source excursion.
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
    FencePathCoreLiveRetentionSameThread(PathCoreLiveFenceReason::MACRO);
    InvalidatePathCoreFeedSameThread(); // BX-FEED
    InvalidatePathCoreArcSameThread(); // BY-ARC
    InvalidatePathCoreReplaySameThread(); // BZ: revoke saved geometry and active replay.
    InvalidatePathCoreHoldSameThread(); // CB: revoke unconsumed arm and original-source excursion.

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
    CancelGapDryRunSameThread("LIFECYCLE");
    FencePathCoreLiveRetentionSameThread(PathCoreLiveFenceReason::INTERRUPTION);
    InvalidatePathCoreFeedSameThread(); // BX-FEED
    InvalidatePathCoreArcSameThread(); // BY-ARC
    InvalidatePathCoreReplaySameThread(); // BZ: revoke saved geometry and active replay.
    InvalidatePathCoreHoldSameThread(); // CB: revoke unconsumed arm and original-source excursion.

    // Any newly opened lifecycle interruption supersedes an older deferred
    // GOTO tail rebase.  The GOTO path arms its replacement identity only
    // after publishing and validating the new Epoch below its Begin() call.
    ClearPendingGotoQueueTailRebase();

    NCLifecycleInterruptionSample sample =
        BuildLifecycleInterruptionSample();

    // K.2.1 internal mapping faults are detected and contained by the 250 us
    // Runtime in the same pass. If the 10 ms NC observer arrives later, its
    // ordinary sample may already be on the post-stop Epoch. Seed the exact
    // pre-fault Epoch only for Alarm 3021 from the dedicated release-published
    // request record written before AlarmManager::Trigger(). J.6 remains the
    // authority that proves the later coherent old->new E-stop application;
    // subsequent Observe() samples remain on the current Epoch.
    if (cause == NCLifecycleInterruptionCause::ALARM)
    {
        const AlarmManager& alarms = AlarmManager::GetInstance();
        const bool alarmActive = alarms.HasAlarm();
        const int alarmCount = alarmActive
            ? alarms.GetAlarmCount()
            : 0;
        bool mappingIntegrityAlarm = false;
        for (int alarmIndex = 0;
            alarmIndex < alarmCount;
            ++alarmIndex)
        {
            if (alarms.GetAlarmId(alarmIndex) ==
                AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY)
            {
                mappingIntegrityAlarm = true;
                break;
            }
        }

        const MotionP1HandoverSafetySnapshot p1Safety =
            m_motion.GetP1HandoverSafetySnapshot();
        const bool exactInternalAlarmRequest =
            p1Safety.mappingIntegrityAlarmPending &&
            p1Safety.mappingIntegrityAlarmRequests != 0ULL &&
            p1Safety.mappingIntegrityAlarmRequests !=
            m_lastHandledMappingIntegrityAlarmRequestCount &&
            p1Safety.lastMappingIntegrityAlarmExecutionEpoch !=
            MOTION_EXECUTION_EPOCH_INVALID;

        // Prefer the explicit 3021 entry. If the fixed Alarm list was already
        // full, an earlier active Alarm is a broader interruption boundary;
        // the still-pending coherent P1 mailbox supplies the exact old Epoch
        // and must be consumed so one Reset can recover.
        if (mappingIntegrityAlarm || exactInternalAlarmRequest)
        {
            if (exactInternalAlarmRequest)
            {
                sample.executionEpoch =
                    p1Safety.lastMappingIntegrityAlarmExecutionEpoch;
                m_lastHandledMappingIntegrityAlarmRequestCount =
                    p1Safety.mappingIntegrityAlarmRequests;
            }
        }
    }

    m_lifecycleInterruptionShadow.Begin(
        cause,
        expectsEpochChange,
        sample);
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
        // BR FIX2: an idle NC Alarm may have no execution to invalidate,
        // while its AUTO -> SAFETY ownership takeover still publishes an
        // Epoch. Import that change only with the exact existing handshake
        // acknowledgement, not merely because a runtime Epoch changed.
        MotionExecutionEpoch successor = alarmStop.requestExecutionEpoch + 1U;
        if (successor == MOTION_EXECUTION_EPOCH_INVALID) successor = 1U;
        bool idleSafetyTakeoverProven = false;
        if (!alarmStop.epochChangeRequired && !alarmStop.hadExecutionToInvalidate &&
            alarmStop.trigger == NCAlarmEmergencyStopTrigger::NC_PROGRAM &&
            alarmStop.phase == NCAlarmEmergencyStopPhase::ACKNOWLEDGED &&
            alarmStop.emergencyEvidenceCoherent && alarmStop.motionPublicationGeneration != 0ULL &&
            alarmStop.emergencyRequestObserved && alarmStop.rtApplyObserved &&
            alarmStop.rtApplyDelta != 0ULL && alarmStop.safetyOwnerMatched &&
            alarmStop.executionEpochMatched && alarmStop.groupStopApplied &&
            alarmStop.allExistingAxesSafe && alarmStop.allExistingAxisCommandsZero &&
            alarmStop.allExistingAxisTargetsSealed && alarmStop.feedbackSequenceSynchronized &&
            !alarmStop.lifecycleEvidenceGap && !alarmStop.postAlarmDispatchObserved &&
            !alarmStop.evidenceGap && !alarmStop.superseded &&
            alarmStop.activeBlocks == 0U && alarmStop.commandQueueDepth == 0ULL &&
            alarmStop.commandIngressDepth == 0ULL && alarmStop.commandReplayDepth == 0ULL &&
            alarmStop.currentOwner == MotionOwner::SAFETY &&
            alarmStop.lastAppliedOwner == MotionOwner::SAFETY &&
            alarmStop.currentOwnerGeneration == alarmStop.lastAppliedOwnerGeneration &&
            alarmStop.lastAppliedExecutionEpoch == successor &&
            alarmStop.currentExecutionEpoch == successor &&
            m_lifecycleInterruptionShadow.MatchesIdleAlarmRequestForSafetyTakeover(
                alarmStop.lifecycleSequence, alarmStop.requestExecutionEpoch,
                alarmStop.currentOwnerGeneration))
        {
            MotionOwnerLease safetyLease{};
            safetyLease.owner = alarmStop.currentOwner;
            safetyLease.generation = alarmStop.currentOwnerGeneration;
            MotionExecutionEpoch provenEpoch = MOTION_EXECUTION_EPOCH_INVALID;
            idleSafetyTakeoverProven =
                m_motion.TryGetSafetyMotionOwnerEpoch(safetyLease, provenEpoch) &&
                provenEpoch == alarmStop.lastAppliedExecutionEpoch;
        }
        m_lifecycleInterruptionShadow.RecordAlarmStopAcknowledged(
            alarmStop.lastAppliedExecutionEpoch,
            alarmStop.epochChangeRequired || idleSafetyTakeoverProven,
            alarmStop.preLatchedRTApplication);
    }
}

// ============================================================================
// Stage NC-0.1D - NC Motion Feedback Snapshot / Counters
// ============================================================================
NCManager::ResetPreDrainReconcileResult
NCManager::ReconcileResetPreDrainMappingAlarmBoundary() noexcept
{
    const bool waitingForOutputHold =
        m_resetContinuationPhase ==
        ResetContinuationPhase::OUTPUT_HOLD;
    if (!waitingForOutputHold &&
        m_resetContinuationPhase !=
        ResetContinuationPhase::PRE_DRAIN)
    {
        return ResetPreDrainReconcileResult::READY;
    }

    // OUTPUT_HOLD uses the immutable button cutoff; PRE_DRAIN uses the later
    // physical Motion authority edge. A producer which began before the
    // selected boundary may publish its mailbox later without advancing the
    // generation. Every producer which begins after it must advance first.
    const std::uint64_t expectedProvenanceGeneration =
        waitingForOutputHold
        ? m_resetButtonCutoffProvenanceGeneration
        : m_resetAuthorityProvenanceGeneration;
    if ((!waitingForOutputHold &&
        expectedProvenanceGeneration == 0ULL) ||
        m_motion.GetSafetyProvenanceGeneration() !=
        expectedProvenanceGeneration)
    {
        return ResetPreDrainReconcileResult::SUPERSEDED;
    }

    AlarmManager& alarms = AlarmManager::GetInstance();
    AlarmManager::MotionAdmissionReservation alarmAdmission{};
    const AlarmManager::MotionAdmissionResult admissionResult =
        alarms.TryBeginMotionAdmission(
            m_resetAuthorityAlarmUpdateCount,
            m_resetAuthorityAlarmSafetyIntentState,
            alarmAdmission,
            false);
    if (admissionResult ==
        AlarmManager::MotionAdmissionResult::BUSY)
    {
        return ResetPreDrainReconcileResult::DEFERRED;
    }
    if (admissionResult !=
        AlarmManager::MotionAdmissionResult::ACQUIRED ||
        alarmAdmission.baseState !=
        m_resetAuthorityAlarmSafetyIntentState)
    {
        return ResetPreDrainReconcileResult::SUPERSEDED;
    }

    const auto endAsSuperseded = [&alarms,
        &alarmAdmission]() noexcept
        -> ResetPreDrainReconcileResult
    {
        (void)alarms.EndMotionAdmission(alarmAdmission);
        return ResetPreDrainReconcileResult::SUPERSEDED;
    };

    const std::uint64_t handledRequestBefore =
        m_lastHandledMappingIntegrityAlarmRequestCount;
    if (handledRequestBefore !=
        m_resetAuthorityMappingAlarmRequestCount)
    {
        // An Alarm or mapping request was materialized outside this exact
        // classification window.  It cannot be absorbed by the old Reset.
        return endAsSuperseded();
    }

    const NCLifecycleInterruptionSnapshot boundaryBefore =
        m_lifecycleInterruptionShadow.GetSnapshot();
    if (m_resetLifecycleInterruptionSequence == 0ULL ||
        boundaryBefore.sequence !=
        m_resetLifecycleInterruptionSequence ||
        boundaryBefore.cause !=
        NCLifecycleInterruptionCause::RESET ||
        m_motion.GetSafetyProvenanceGeneration() !=
        expectedProvenanceGeneration)
    {
        return endAsSuperseded();
    }

    const MotionP1HandoverSafetySnapshot mappingBefore =
        m_motion.GetP1HandoverSafetySnapshot();
    if (!mappingBefore.mappingIntegrityAlarmPending)
    {
        // A pre-edge producer may still be between its provenance operation
        // and mailbox publication.  Leave PRE_DRAIN intact; the next NC scan
        // will classify the exact published request before draining feedback.
        const std::uint32_t candidateAlarmUpdateCount =
            alarmAdmission.expectedUpdateCount;
        const std::uint64_t candidateAlarmIntentState =
            alarmAdmission.baseState;
        if (!alarms.EndMotionAdmission(alarmAdmission) ||
            alarms.GetUpdateCount() != candidateAlarmUpdateCount ||
            AlarmManager::MotionAdmissionBaseState(
                alarms.GetMotionSafetyIntentState()) !=
            candidateAlarmIntentState ||
            m_motion.GetSafetyProvenanceGeneration() !=
            expectedProvenanceGeneration)
        {
            return ResetPreDrainReconcileResult::SUPERSEDED;
        }
        return ResetPreDrainReconcileResult::READY;
    }

    const std::uint64_t capturedRequest =
        mappingBefore.mappingIntegrityAlarmRequests;
    if (capturedRequest == 0ULL)
    {
        return endAsSuperseded();
    }

    const bool requestAlreadyMaterialized =
        capturedRequest == handledRequestBefore;
    bool mappingAlarmPresentBefore = false;
    for (int alarmIndex = 0;
        alarmIndex < alarms.GetAlarmCount();
        ++alarmIndex)
    {
        if (alarms.GetAlarmId(alarmIndex) ==
            AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY)
        {
            mappingAlarmPresentBefore = true;
            break;
        }
    }

    if (!requestAlreadyMaterialized)
    {
        if (!mappingAlarmPresentBefore)
        {
            if (!alarms.TriggerUnderMotionAdmission(
                alarmAdmission,
                AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY,
                0,
                mappingBefore.lastOrphanAxisIndex))
            {
                return endAsSuperseded();
            }
        }

        bool formalMappingAlarmPresent = false;
        for (int alarmIndex = 0;
            alarmIndex < alarms.GetAlarmCount();
            ++alarmIndex)
        {
            if (alarms.GetAlarmId(alarmIndex) ==
                AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY)
            {
                formalMappingAlarmPresent = true;
                break;
            }
        }
        if (!formalMappingAlarmPresent && !alarms.HasAlarm())
        {
            return endAsSuperseded();
        }

        // Preserve the original operator RESET lifecycle sequence.  Replacing
        // it with ALARM and then opening a new RESET boundary would put an
        // already-published causal terminal before the new baseline and lose
        // its interruption accounting.  Alarm-stop diagnostics remain a
        // separate observer and do not own the lifecycle boundary here.
    }

    const bool acknowledged =
        m_motion.AcknowledgeP1MappingIntegrityAlarmRequest(
            capturedRequest);
    const MotionP1HandoverSafetySnapshot mappingAfter =
        m_motion.GetP1HandoverSafetySnapshot();
    const std::uint64_t provenanceAfter =
        m_motion.GetSafetyProvenanceGeneration();
    const bool capturedRequestRetired =
        mappingAfter.mappingIntegrityAlarmRequests ==
        capturedRequest &&
        !mappingAfter.mappingIntegrityAlarmPending;
    const bool nextPreEdgeRequestPending =
        mappingAfter.mappingIntegrityAlarmRequests != 0ULL &&
        mappingAfter.mappingIntegrityAlarmRequests !=
        capturedRequest &&
        mappingAfter.mappingIntegrityAlarmPending;
    if (!acknowledged ||
        provenanceAfter !=
        expectedProvenanceGeneration ||
        (!capturedRequestRetired &&
            !nextPreEdgeRequestPending) ||
        !alarms.IsMotionAdmissionCurrent(alarmAdmission))
    {
        return endAsSuperseded();
    }

    const std::uint32_t candidateAlarmUpdateCount =
        alarmAdmission.expectedUpdateCount;
    const std::uint64_t candidateAlarmIntentState =
        alarmAdmission.baseState;

    // A second producer may also have linearized before the operator edge but
    // been unable to publish into the single P1 slot until this exact ACK.
    // Stable provenance proves it is still pre-edge.  Keep PRE_DRAIN and let
    // the next bounded NC pass materialize that newer immutable sequence.

    const NCLifecycleInterruptionSnapshot boundaryBeforeEnd =
        m_lifecycleInterruptionShadow.GetSnapshot();
    if (boundaryBeforeEnd.sequence !=
        m_resetLifecycleInterruptionSequence ||
        boundaryBeforeEnd.cause !=
        NCLifecycleInterruptionCause::RESET ||
        !alarms.EndMotionAdmission(alarmAdmission) ||
        alarms.GetUpdateCount() != candidateAlarmUpdateCount ||
        AlarmManager::MotionAdmissionBaseState(
            alarms.GetMotionSafetyIntentState()) !=
        candidateAlarmIntentState ||
        m_motion.GetSafetyProvenanceGeneration() !=
        expectedProvenanceGeneration)
    {
        return ResetPreDrainReconcileResult::SUPERSEDED;
    }

    m_lastHandledMappingIntegrityAlarmRequestCount = capturedRequest;
    m_resetAuthorityAlarmUpdateCount = candidateAlarmUpdateCount;
    m_resetAuthorityAlarmSafetyIntentState = candidateAlarmIntentState;
    m_resetAuthorityMappingAlarmRequestCount = capturedRequest;
    if (!requestAlreadyMaterialized &&
        !m_lifecycleInterruptionAlarmLatched)
    {
        BeginAlarmEmergencyStopShadow();
    }
    return ResetPreDrainReconcileResult::READY;
}


bool NCManager::EnsureMappingIntegrityAlarmBoundaryBeforeFeedback() noexcept
{
    const MotionP1HandoverSafetySnapshot p1Safety =
        m_motion.GetP1HandoverSafetySnapshot();
    if (!p1Safety.mappingIntegrityAlarmPending ||
        p1Safety.mappingIntegrityAlarmRequests == 0ULL)
    {
        return false;
    }

    if (p1Safety.mappingIntegrityAlarmRequests ==
        m_lastHandledMappingIntegrityAlarmRequestCount)
    {
        return m_motion.AcknowledgeP1MappingIntegrityAlarmRequest(
            p1Safety.mappingIntegrityAlarmRequests);
    }

    // AlarmManager is materialized only by this 10 ms NC owner. Motion has
    // already release-published the exact old Epoch and pending request before
    // it can publish any causal or retirement terminal feedback.  A G00
    // producer rejection may have materialized the same 3021 immediately to
    // stop the remainder of its NC block; do not duplicate that exact alarm.
    AlarmManager& alarms = AlarmManager::GetInstance();
    bool mappingIntegrityAlarmPresent = false;
    for (int alarmIndex = 0;
        alarmIndex < alarms.GetAlarmCount();
        ++alarmIndex)
    {
        if (alarms.GetAlarmId(alarmIndex) ==
            AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY)
        {
            mappingIntegrityAlarmPresent = true;
            break;
        }
    }

    if (!mappingIntegrityAlarmPresent)
    {
        alarms.Trigger(
            AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY,
            0,
            p1Safety.lastOrphanAxisIndex);

        for (int alarmIndex = 0;
            alarmIndex < alarms.GetAlarmCount();
            ++alarmIndex)
        {
            if (alarms.GetAlarmId(alarmIndex) ==
                AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY)
            {
                mappingIntegrityAlarmPresent = true;
                break;
            }
        }
    }

    if (!mappingIntegrityAlarmPresent && !alarms.HasAlarm())
    {
        // No formal Alarm path exists yet. Keep the mailbox pending and retry;
        // never clear the only old-Epoch correlation without a boundary.
        return false;
    }

    if (!m_lifecycleInterruptionAlarmLatched)
    {
        BeginLifecycleInterruptionShadow(
            NCLifecycleInterruptionCause::ALARM,
            false);
        m_lifecycleInterruptionAlarmLatched = true;
        BeginAlarmEmergencyStopShadow();
    }
    else
    {
        // An already-open Alarm boundary is earlier than this mapping stop
        // and therefore safely contains it. Consume the mailbox without
        // replacing that broader interruption boundary.
        m_lastHandledMappingIntegrityAlarmRequestCount =
            p1Safety.mappingIntegrityAlarmRequests;
    }

    return m_motion.AcknowledgeP1MappingIntegrityAlarmRequest(
        p1Safety.mappingIntegrityAlarmRequests);
}

void NCManager::ProcessMotionFeedback() noexcept
{
    // BQ-FIX2-BEGIN
        // Admit the saved, NC-committed receipt after RT clears pending requests,
        // before this same consumer applies its ACCEPTED/COMPLETED events.
    DrainPathCorePendingCommandedCaptureSameThread();
    // BQ-FIX2-END
    MotionFeedbackEvent event{};

    // Stage NC-0.2K.6.3: a terminal registry can only qualify while the
    // existing bounded transports and Lifecycle Ledger have never lost or
    // ambiguously terminalised an event.  This reads counters only; it adds
    // no EtherCAT traffic and performs no Motion write.
    const NCBlockLifecycleCounters preDrainLedgerCounters =
        m_blockLifecycleLedger.GetCounters();
    m_ordinaryG00InflightRegistryShadow.ObserveTransportHealth(
        m_motion.GetMotionFeedbackOverflowCount(),
        m_motion.GetMotionFeedbackProducerNoticeOverflowCount(),
        m_motionFeedbackSequenceGapCount,
        preDrainLedgerCounters.activeBlockOverwrite,
        preDrainLedgerCounters.activeSegmentIndexOverwrite,
        preDrainLedgerCounters.orphanFeedback,
        preDrainLedgerCounters.duplicateTerminalFeedback,
        preDrainLedgerCounters.terminalFeedbackConflict);

    // BO reuses the authoritative counters already read by this NC consumer.
    m_pathCoreExecutionLink.CheckTransport(
        m_motion.GetMotionFeedbackOverflowCount() == 0ULL &&
        m_motion.GetMotionFeedbackProducerNoticeOverflowCount() == 0ULL &&
        m_motionFeedbackSequenceGapCount == 0ULL &&
        preDrainLedgerCounters.activeBlockOverwrite == 0ULL &&
        preDrainLedgerCounters.activeSegmentIndexOverwrite == 0ULL &&
        preDrainLedgerCounters.orphanFeedback == 0ULL &&
        preDrainLedgerCounters.duplicateTerminalFeedback == 0ULL &&
        preDrainLedgerCounters.terminalFeedbackConflict == 0ULL);

    // BQ-BEGIN
    m_pathCoreCommittedRun.CheckTransport(
        m_motion.GetMotionFeedbackOverflowCount() == 0ULL &&
        m_motion.GetMotionFeedbackProducerNoticeOverflowCount() == 0ULL &&
        m_motionFeedbackSequenceGapCount == 0ULL &&
        preDrainLedgerCounters.activeBlockOverwrite == 0ULL &&
        preDrainLedgerCounters.activeSegmentIndexOverwrite == 0ULL &&
        preDrainLedgerCounters.orphanFeedback == 0ULL &&
        preDrainLedgerCounters.duplicateTerminalFeedback == 0ULL &&
        preDrainLedgerCounters.terminalFeedbackConflict == 0ULL);
    // BQ-END
        // 每個 NC Cycle 最多處理固定筆數，避免異常事件 Burst 讓
        // 10 ms NC Task 出現過大的單圈負擔。2048 筆 Ring 可容納完整
        // Epoch 淘汰 Burst，未讀事件由後續 Cycle 繼續 Drain。
    for (std::size_t i = 0U;
        i < MOTION_FEEDBACK_NC_DRAIN_LIMIT_PER_TASK;
        ++i)
    {
        if (m_deferredResetMotionFeedbackValid)
        {
            event = m_deferredResetMotionFeedback;
            m_deferredResetMotionFeedback = MotionFeedbackEvent{};
            m_deferredResetMotionFeedbackValid = false;
        }
        else if (!m_motion.TryReadMotionFeedback(event))
        {
            break;
        }

        const bool lifecycleFailureFeedback =
            IsLifecycleFailureFeedback(event.type);

        if (lifecycleFailureFeedback)
        {
            // Close the narrow preflight->drain race. Mapping Alarm and its
            // old-Epoch record are release-published before every causal or
            // retirement terminal, so this idempotent check always opens the
            // Alarm boundary before the event reaches the Ledger observer.
            // PRE_DRAIN is the one exception: its exact provenance classifier
            // must decide whether a delayed mailbox belongs before or after
            // the operator edge before any Alarm/lifecycle state is changed.
            if (m_resetContinuationPhase ==
                ResetContinuationPhase::OUTPUT_HOLD ||
                m_resetContinuationPhase ==
                ResetContinuationPhase::PRE_DRAIN)
            {
                const ResetPreDrainReconcileResult reconcileResult =
                    ReconcileResetPreDrainMappingAlarmBoundary();
                if (reconcileResult ==
                    ResetPreDrainReconcileResult::DEFERRED)
                {
                    // The event was already acquired from the SPSC ring.
                    // Preserve exactly this item and retry its Reset/Alarm
                    // classification before consuming any later feedback.
                    m_deferredResetMotionFeedback = event;
                    m_deferredResetMotionFeedbackValid = true;
                    return;
                }
                if (reconcileResult ==
                    ResetPreDrainReconcileResult::SUPERSEDED)
                {
                    m_resetContinuationPhase =
                        ResetContinuationPhase::BLOCKED;
                    m_resetAuthorityEntryGeneration =
                        MOTION_OWNER_GENERATION_INVALID;
                    m_motion.RequestEmergencyStopAllAxes();
                    (void)EnsureMappingIntegrityAlarmBoundaryBeforeFeedback();
                }
            }
            else
            {
                (void)EnsureMappingIntegrityAlarmBoundaryBeforeFeedback();
            }
        }

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

        // K.6.3 consumes the exact same immutable event only after the
        // authoritative NC-0.2D Ledger has classified it.  The observer
        // cannot acknowledge, remove, replay, or otherwise influence it.
        m_ordinaryG00InflightRegistryShadow.ObserveMotionFeedback(
            event,
            ledgerAccepted);

        // BO updates only pre-bound retained identities from this same event.
        // No second dequeue, acknowledgement, registry mutation or Motion write.
        m_pathCoreExecutionLink.CheckTransport(
            m_motion.GetMotionFeedbackOverflowCount() == 0ULL &&
            m_motion.GetMotionFeedbackProducerNoticeOverflowCount() == 0ULL &&
            m_motionFeedbackSequenceGapCount == 0ULL);
        m_pathCoreExecutionLink.Observe(event, ledgerAccepted);
        // BQ-BEGIN
        m_pathCoreCommittedRun.CheckTransport(
            m_motion.GetMotionFeedbackOverflowCount() == 0ULL &&
            m_motion.GetMotionFeedbackProducerNoticeOverflowCount() == 0ULL &&
            m_motionFeedbackSequenceGapCount == 0ULL);
        m_pathCoreCommittedRun.Observe(event, ledgerAccepted);
        ObservePathCoreFeedFeedbackSameThread(event, ledgerAccepted); // BX-FEED
        ObservePathCoreArcFeedbackSameThread(event, ledgerAccepted); // BY-ARC
        ObservePathCoreReplayFeedbackSameThread(event, ledgerAccepted); // BZ-REPLAY
// BQ-END

        // K.7.3 observes the same already-classified immutable event.  The
        // observer itself cannot act on Registry, Ledger, NC flow or Motion.
        m_ordinaryG00FeedHoldCohortShadow.ObserveMotionFeedback(
            event,
            ledgerAccepted,
            m_ordinaryG00InflightRegistryShadow.GetSnapshot());

        // K.7.4 consumes only K.7.3's published proof and the already-updated
        // K.6.3 Registry accounting.  It cannot consume or modify the event.
        ObserveOrdinaryG00FeedHoldCohortCutover();

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


void NCManager::ObserveBootstrapSafetyHandoff() noexcept
{
    // The EtherCAT/RT startup path can deliberately leave the Motion owner
    // as SAFETY while it establishes PDO and standstill evidence.  That is a
    // safe default, but it must not require an operator Reset merely to use
    // a clean, alarm-free UI.  This one-time handoff is intentionally much
    // narrower than Reset: it never changes NC modal state, clears no Alarm
    // or Fault, performs no coordinate rebase, and never enables motion.
    if (!m_bootSafetyHandoffPending ||
        (m_state != NCState::IDLE && m_state != NCState::READY) ||
        m_resetContinuationPhase != ResetContinuationPhase::IDLE ||
        m_resetSafetyOutputHoldActive ||
        m_lifecycleInterruptionShadow.IsActive() ||
        AlarmManager::GetInstance().HasAlarm() ||
        m_motion.HasPendingSafetyOrRecoveryRequests())
    {
        return;
    }

    const MotionOwnerLease bootstrapLease =
        m_motion.GetMotionOwnerLease();
    if (bootstrapLease.owner == MotionOwner::NONE)
    {
        m_bootSafetyHandoffPending = false;
        return;
    }
    if (bootstrapLease.owner != MotionOwner::SAFETY ||
        !bootstrapLease.IsValid() ||
        !m_motion.IsMotionOwnerLeaseCurrent(bootstrapLease))
    {
        // A real non-bootstrap producer has already taken ownership.  Never
        // reinterpret it as an idle startup lease on a later scan.
        m_bootSafetyHandoffPending = false;
        return;
    }

    const MotionStopSettleSnapshot settle =
        m_motion.GetStopSettleSnapshot();
    MotionEmergencyStopEvidence emergency{};
    MotionEmergencyStopCounters emergencyCounters{};
    if (settle.publicationGeneration == 0ULL ||
        settle.sampleSequence == 0ULL ||
        !settle.standstill ||
        settle.groupActive ||
        settle.commandQueueDepth != 0U ||
        settle.nonIdleAxisCount != 0U ||
        settle.commandMovingAxisCount != 0U ||
        settle.actualMovingAxisCount != 0U ||
        settle.pdoTargetVelocityNonzeroAxisCount != 0U ||
        !m_motion.TryGetEmergencyStopEvidence(
            emergency,
            emergencyCounters) ||
        emergency.publicationGeneration != settle.publicationGeneration ||
        emergency.estopAxisMask != 0U ||
        emergency.errorAxisMask != 0U ||
        emergency.faultAxisMask != 0U ||
        emergency.lagAlarmAxisMask != 0U)
    {
        return;
    }

    // ReleaseMotionOwner repeats the packed owner/ticket/pending checks at
    // its CAS seam. A concurrent Safety request therefore wins fail-closed;
    // we mark this bootstrap handoff consumed only after our own release CAS
    // succeeds, so a later real Safety owner is never released by this path.
    if (m_motion.ReleaseMotionOwner(bootstrapLease))
    {
        m_bootSafetyHandoffPending = false;
        // Keep this one-time ownership handoff independent of the RTX API
        // header/include order.  Existing NC01F runtime diagnostics expose
        // the resulting NONE owner, while this supervisory path must remain
        // buildable in every project configuration that hosts NCManager.
    }
}


// NC-0.2L.2AS: Path Core shadow methods are defined in NCManager_PathCore.cpp.


// 🌟 放在 RTOS 迴圈的核心任務
void NCManager::ProcessTask()
{
    ValidateGapDryRunSameThread(); // CG: revoke/pause before early control returns.
    ValidatePathCoreFeedSameThread(); // BX-FEED: observe lifecycle before early returns.
    ValidatePathCoreArcSameThread(); // BY-ARC
    ValidatePathCoreReplaySameThread(); // BZ-REPLAY
    ObservePathCoreHoldSameThread(); // CB: validate scope and report RT phase transitions before early returns.
    FlushPathCoreLiveSummarySameThread();
    // BN: bounded data-only guard, before any early-returning control phase.
    (void)ValidatePathCoreLiveRetentionSameThread();
    // BQ-BEGIN
    (void)ValidatePathCoreCommittedRunSameThread();
    // BQ-END
    NC_RunCount++;

    // NC-0.2L.2A: retain the accepted B2 baseline publication exactly once.
    // Do not overwrite an accepted-input event at the start of each 10 ms
    // scan and do not create a 100 Hz compact-token oscillation.
    m_pathCoreInputHandoffCompactShadow.EnsureNotRunningPublished();

    // A Reset button which collided only with the preceding PDO frame keeps
    // its immutable Alarm/mapping/provenance cutoff here.  Do not run the
    // ordinary Alarm materializer or drain feedback until that exact button
    // admission is either acquired or superseded.
    if (m_resetContinuationPhase ==
        ResetContinuationPhase::BUTTON_ADMISSION)
    {
        Reset();
        if (m_resetContinuationPhase ==
            ResetContinuationPhase::BUTTON_ADMISSION)
        {
            return;
        }
    }

    if (m_resetContinuationPhase ==
        ResetContinuationPhase::CLEANUP)
    {
        return;
    }

    // A K.2.1 mapping-integrity Alarm can be published and physically stopped
    // entirely between two NC scans. OUTPUT_HOLD/PRE_DRAIN first classify the
    // exact request against the immutable button/authority provenance while
    // preserving its RESET lifecycle boundary. Every other phase uses the
    // ordinary ALARM path.
    if (m_resetContinuationPhase ==
        ResetContinuationPhase::OUTPUT_HOLD ||
        m_resetContinuationPhase ==
        ResetContinuationPhase::PRE_DRAIN)
    {
        const ResetPreDrainReconcileResult reconcileResult =
            ReconcileResetPreDrainMappingAlarmBoundary();
        if (reconcileResult ==
            ResetPreDrainReconcileResult::DEFERRED)
        {
            return;
        }
        if (reconcileResult ==
            ResetPreDrainReconcileResult::SUPERSEDED)
        {
            m_resetContinuationPhase =
                ResetContinuationPhase::BLOCKED;
            m_resetAuthorityEntryGeneration =
                MOTION_OWNER_GENERATION_INVALID;
            m_motion.RequestEmergencyStopAllAxes();
            (void)EnsureMappingIntegrityAlarmBoundaryBeforeFeedback();
        }
    }
    else
    {
        (void)EnsureMappingIntegrityAlarmBoundaryBeforeFeedback();
    }

    // K.6.3 revocation is deliberately owned by this NC task.  Mark an
    // already-visible Reset/Alarm/Program-End boundary before draining its
    // delayed terminal feedback, while retaining every exact entry until the
    // terminal event itself is observed.
    if (AlarmManager::GetInstance().HasAlarm() ||
        m_state == NCState::ALARM)
    {
        m_ordinaryG00ReadAheadCutoverGate.ObserveQueueInactive(
            NCPreparedInvalidationReason::ALARM);
        m_ordinaryG00InflightRegistryShadow.ObserveQueueInactive(
            NCPreparedInvalidationReason::ALARM);
    }
    else if (m_state == NCState::RESET_STATE ||
        m_resetContinuationPhase != ResetContinuationPhase::IDLE)
    {
        m_ordinaryG00ReadAheadCutoverGate.ObserveQueueInactive(
            NCPreparedInvalidationReason::RESET);
        m_ordinaryG00InflightRegistryShadow.ObserveQueueInactive(
            NCPreparedInvalidationReason::RESET);
    }
    else if (m_state == NCState::P_END ||
        m_programEndBoundary.IsEndPending())
    {
        m_ordinaryG00ReadAheadCutoverGate.ObserveQueueInactive(
            NCPreparedInvalidationReason::PROGRAM_END);
        m_ordinaryG00InflightRegistryShadow.ObserveQueueInactive(
            NCPreparedInvalidationReason::PROGRAM_END);
    }
    else if (m_state == NCState::RUN || m_state == NCState::HOLD)
    {
        // A program/cache/frame/flow/owner/panel replacement can remain in a
        // RUN-compatible state.  Compare it on this same NC thread before a
        // delayed terminal is consumed.  The normal one-Epoch legacy G00
        // advance is explicitly compatible with its registered identity.
        m_ordinaryG00InflightRegistryShadow.ObserveLiveSource(
            BuildPreparedBlockSourceIdentity());
    }
    else
    {
        m_ordinaryG00ReadAheadCutoverGate.ObserveQueueInactive(
            NCPreparedInvalidationReason::NOT_RUNNING);
        m_ordinaryG00InflightRegistryShadow.ObserveQueueInactive(
            NCPreparedInvalidationReason::NOT_RUNNING);
    }

    // 即使 NC 正處於 Alarm / Reset / Not Ready，也必須先 Drain Feedback，
    // 否則 Runtime Terminal Event 可能在上層長時間停住時累積。
    ProcessMotionFeedback();
    ObserveFeedHoldBoundaryShadow();

    // Stage NC-0.2K.6.2: a whole-PDO/Epoch reservation may defer the first
    // bounded Reset authority handshake.  Continue the exact original
    // generation automatically on later 10 ms scans; while it is still
    // deferred, Reset() keeps a Motion-visible E-stop mailbox latched and the
    // NC state fail-closed in RESET_STATE.
    const auto resetContinuationIsAdvancing = [this]() noexcept -> bool
    {
        return
            m_resetContinuationPhase ==
            ResetContinuationPhase::BUTTON_ADMISSION ||
            m_resetContinuationPhase ==
            ResetContinuationPhase::OUTPUT_HOLD ||
            m_resetContinuationPhase ==
            ResetContinuationPhase::PRE_DRAIN ||
            m_resetContinuationPhase ==
            ResetContinuationPhase::AUTHORITY ||
            m_resetContinuationPhase ==
            ResetContinuationPhase::EPOCH ||
            m_resetContinuationPhase ==
            ResetContinuationPhase::BATCH ||
            m_resetContinuationPhase ==
            ResetContinuationPhase::ALARM_CLEAR ||
            m_resetContinuationPhase ==
            ResetContinuationPhase::CLEANUP ||
            m_resetContinuationPhase ==
            ResetContinuationPhase::SETTLE;
    };
    if (resetContinuationIsAdvancing())
    {
        if ((m_resetContinuationPhase !=
            ResetContinuationPhase::OUTPUT_HOLD &&
            m_resetContinuationPhase !=
            ResetContinuationPhase::PRE_DRAIN &&
            (AlarmManager::GetInstance().GetUpdateCount() !=
                m_resetAuthorityAlarmUpdateCount ||
                AlarmManager::MotionAdmissionBaseState(
                    AlarmManager::GetInstance().
                    GetMotionSafetyIntentState()) !=
                m_resetAuthorityAlarmSafetyIntentState)) ||
            (m_resetContinuationPhase ==
                ResetContinuationPhase::SETTLE &&
                AlarmManager::GetInstance().HasAlarm()))
        {
            // A later Alarm owns the newer lifecycle boundary.  Do not let
            // the older Reset continuation clear or retarget it.
            m_resetContinuationPhase =
                ResetContinuationPhase::BLOCKED;
            m_resetAuthorityEntryGeneration =
                MOTION_OWNER_GENERATION_INVALID;
        }
        else if (m_motion.HasPendingSafetyOrRecoveryRequests())
        {
            // The fail-closed fallback E-stop must reach its exact RT
            // acknowledgement before this Reset publishes its correlated
            // batch; otherwise the E-stop priority branch would consume and
            // discard the just-published Reset batch.
            return;
        }
        else
        {
            Reset();
        }
        if (resetContinuationIsAdvancing())
        {
            return;
        }
    }

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
        if (m_resetContinuationPhase !=
            ResetContinuationPhase::IDLE)
        {
            m_resetContinuationPhase =
                ResetContinuationPhase::BLOCKED;
        }
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

        ObservePreparedBlockQueueShadow(false);

        // ⚠️ 立刻退出迴圈，絕對不准往下執行任何軌跡運算或 G 碼解析！
        return;
    }

    // This phase deliberately runs after the normal Alarm branch.  A real
    // Alarm therefore supersedes a decelerating Reset with the established
    // immediate E-stop path instead of being hidden behind an outstanding
    // StopGroup ticket.  The ticket itself remains pending until the RT
    // controlled-stop trajectory has reached physical convergence, so no
    // cross-thread snapshot is used as an early completion permit here.
    if (m_resetContinuationPhase ==
        ResetContinuationPhase::PRE_RESET_CONTROLLED_STOP)
    {
        Reset();
        ObservePreparedBlockQueueShadow(false);
        return;
    }

    // Clean startup only: hand off a retained bootstrap SAFETY lease after
    // the RT owner has published one coherent, stationary, no-fault image.
    // This allows program selection and JOG admission to acquire their normal
    // owners without requiring a cosmetic Reset at power-up.
    ObserveBootstrapSafetyHandoff();

    // =========================================================
    // 🌟 2.5 【新增：滑行煞車攔截網】等待 Reset 後的馬達完全靜止
    // =========================================================
    if (m_state == NCState::RESET_STATE)
    {
        if (m_resetContinuationPhase ==
            ResetContinuationPhase::RELEASE_GATE)
        {
            MotionExecutionEpoch exactReleaseEpoch =
                MOTION_EXECUTION_EPOCH_INVALID;
            const MotionCore::ResetSafetyAuthorityStatus
                exactReleaseStatus =
                m_motion.TryGetResetSafetyMotionOwnerEpoch(
                    m_safetyMotionLease,
                    m_resetAuthorityRequestTicket,
                    m_resetAuthorityProvenanceGeneration,
                    exactReleaseEpoch);
            if (exactReleaseStatus ==
                MotionCore::ResetSafetyAuthorityStatus::DEFERRED)
            {
                ObservePreparedBlockQueueShadow(false);
                return;
            }

            if (exactReleaseStatus !=
                MotionCore::ResetSafetyAuthorityStatus::ACQUIRED ||
                exactReleaseEpoch !=
                m_resetContinuationExecutionEpoch)
            {
                m_resetContinuationPhase =
                    ResetContinuationPhase::BLOCKED;
                m_motion.RequestEmergencyStopAllAxes();
            }
        }

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

        if (m_resetContinuationPhase ==
            ResetContinuationPhase::RELEASE_GATE &&
            m_resetReleaseGate.ShouldReleaseSafetyOwner())
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
                ObservePreparedBlockQueueShadow(false);
                return;
            }

            const auto resetSupervisoryIdentityCurrent =
                [this]() noexcept -> bool
            {
                AlarmManager& alarms = AlarmManager::GetInstance();
                const std::uint32_t alarmRevisionBefore =
                    alarms.GetUpdateCount();
                const bool alarmPresent = alarms.HasAlarm();
                const NCLifecycleInterruptionSnapshot boundary =
                    m_lifecycleInterruptionShadow.GetSnapshot();
                const std::uint32_t alarmRevisionAfter =
                    alarms.GetUpdateCount();

                return
                    alarmRevisionBefore ==
                    m_resetAuthorityAlarmUpdateCount &&
                    alarmRevisionAfter == alarmRevisionBefore &&
                    !alarmPresent &&
                    AlarmManager::MotionAdmissionBaseState(
                        alarms.GetMotionSafetyIntentState()) ==
                    m_resetAuthorityAlarmSafetyIntentState &&
                    m_lastHandledMappingIntegrityAlarmRequestCount ==
                    m_resetAuthorityMappingAlarmRequestCount &&
                    boundary.sequence ==
                    m_resetLifecycleInterruptionSequence &&
                    boundary.cause ==
                    NCLifecycleInterruptionCause::RESET &&
                    boundary.publishedExecutionEpoch ==
                    m_resetContinuationExecutionEpoch &&
                    m_motion.GetSafetyProvenanceGeneration() ==
                    m_resetAuthorityProvenanceGeneration &&
                    !m_motion.HasPendingSafetyOrRecoveryRequests();
            };

            // AlarmManager and the RESET lifecycle are independent of the
            // Motion owner word. Re-prove both immediately before consuming
            // the one-shot RT authorization; a later Alarm must own a new
            // operator Reset rather than be cleared by this transaction.
            if (!resetSupervisoryIdentityCurrent())
            {
                m_resetContinuationPhase =
                    ResetContinuationPhase::BLOCKED;
                m_motion.RequestEmergencyStopAllAxes();
                ObservePreparedBlockQueueShadow(false);
                return;
            }

            AlarmManager& releaseAlarms = AlarmManager::GetInstance();
            AlarmManager::MotionAdmissionReservation
                releaseAlarmAdmission{};
            const AlarmManager::MotionAdmissionResult
                releaseAlarmAdmissionResult =
                releaseAlarms.TryBeginMotionAdmission(
                    m_resetAuthorityAlarmUpdateCount,
                    m_resetAuthorityAlarmSafetyIntentState,
                    releaseAlarmAdmission,
                    true);
            if (releaseAlarmAdmissionResult ==
                AlarmManager::MotionAdmissionResult::BUSY)
            {
                ObservePreparedBlockQueueShadow(false);
                return;
            }
            if (releaseAlarmAdmissionResult !=
                AlarmManager::MotionAdmissionResult::ACQUIRED ||
                releaseAlarmAdmission.baseState !=
                m_resetAuthorityAlarmSafetyIntentState)
            {
                if (releaseAlarmAdmission.acquired)
                {
                    (void)releaseAlarms.EndMotionAdmission(
                        releaseAlarmAdmission);
                }
                m_resetContinuationPhase =
                    ResetContinuationPhase::BLOCKED;
                m_motion.RequestEmergencyStopAllAxes();
                ObservePreparedBlockQueueShadow(false);
                return;
            }

            MotionCore::SafetyMotionOwnerReleaseStatus releaseStatus =
                MotionCore::SafetyMotionOwnerReleaseStatus::SUPERSEDED;
            if (releaseSafetyLeaseCurrent)
            {
                MotionNCResetSafetyReleaseAuthorization
                    releaseAuthorization{};
                releaseAuthorization.requestSequence =
                    m_resetNCSettleRequestSequence;
                releaseAuthorization.drainRevocationGeneration =
                    m_resetAuthorityProvenanceGeneration;
                releaseAuthorization.executionEpoch =
                    m_resetContinuationExecutionEpoch;
                releaseAuthorization.ownerGeneration =
                    m_safetyMotionLease.generation;
                releaseAuthorization.safetyRequestTicket =
                    m_resetAuthorityRequestTicket;
                releaseStatus =
                    m_motion.TryReleaseSafetyMotionOwner(
                        m_safetyMotionLease,
                        releaseAuthorization);
                if (releaseStatus ==
                    MotionCore::SafetyMotionOwnerReleaseStatus::RELEASED)
                {
                    // Owner NONE is already published and is not rolled back.
                    // A concurrent Alarm/Safety edge keeps the physical output
                    // hold active, blocks READY and asks RT for a new stop.
                    if (!resetSupervisoryIdentityCurrent())
                    {
                        releaseStatus = MotionCore::
                            SafetyMotionOwnerReleaseStatus::SUPERSEDED;
                        m_resetContinuationPhase =
                            ResetContinuationPhase::BLOCKED;
                        m_motion.RequestEmergencyStopAllAxes();
                    }
                }
            }

            // A transient RT/frame reservation is not a terminal release
            // failure. Retire the Alarm admission and retry the same one-shot
            // authorization on the next NC scan without marking the gate.
            if (releaseStatus ==
                MotionCore::SafetyMotionOwnerReleaseStatus::DEFERRED)
            {
                if (!releaseAlarms.EndMotionAdmission(
                    releaseAlarmAdmission))
                {
                    m_resetContinuationPhase =
                        ResetContinuationPhase::BLOCKED;
                    m_motion.RequestEmergencyStopAllAxes();
                    m_resetReleaseGate.MarkReleaseResult(
                        resetBoundary,
                        releaseResetRebaseAck,
                        releaseSafetyLeaseCurrent,
                        false);
                }
                ObservePreparedBlockQueueShadow(false);
                return;
            }

            bool releaseSucceeded =
                releaseStatus == MotionCore::
                SafetyMotionOwnerReleaseStatus::RELEASED;
            if (!releaseAlarms.EndMotionAdmission(
                releaseAlarmAdmission))
            {
                releaseSucceeded = false;
                m_resetContinuationPhase =
                    ResetContinuationPhase::BLOCKED;
                m_motion.RequestEmergencyStopAllAxes();
            }

            if (releaseStatus == MotionCore::
                SafetyMotionOwnerReleaseStatus::SUPERSEDED)
            {
                m_resetContinuationPhase =
                    ResetContinuationPhase::BLOCKED;
                m_motion.RequestEmergencyStopAllAxes();
            }

            if (releaseSucceeded)
            {
                // End is the Alarm commit point. Coordinate synchronization
                // is applied only after that exact quiet transaction commits;
                // output hold and owner NONE still prevent physical motion.
                CoordSys.SyncMachinePosition(
                    releaseResetRebaseAck.actualMcsUnit);
                if (!resetSupervisoryIdentityCurrent())
                {
                    releaseSucceeded = false;
                    m_resetContinuationPhase =
                        ResetContinuationPhase::BLOCKED;
                    m_motion.RequestEmergencyStopAllAxes();
                }
            }

            m_resetReleaseGate.MarkReleaseResult(
                resetBoundary,
                releaseResetRebaseAck,
                releaseSafetyLeaseCurrent,
                releaseSucceeded);

            if (releaseSucceeded &&
                m_resetReleaseGate.GetSnapshot().releaseApplied)
            {
                if (m_resetSafetyOutputHoldActive)
                {
                    m_motion.EndResetSafetyOutputHold();
                    m_resetSafetyOutputHoldActive = false;
                }
                m_safetyMotionLease = MotionOwnerLease{};
                m_resetContinuationPhase =
                    ResetContinuationPhase::IDLE;
                m_resetLifecycleBoundaryPrearmedByControlledStop = false;
                m_resetContinuationExecutionEpoch =
                    MOTION_EXECUTION_EPOCH_INVALID;
                m_resetLifecycleInterruptionSequence = 0ULL;
                m_resetAuthorityBaselineTicket = 0U;
                m_resetAuthorityRequestTicket = 0U;
                m_resetButtonCutoffProvenanceGeneration = 0ULL;
                m_resetAuthorityProvenanceGeneration = 0ULL;
                m_resetContinuationExecutionState =
                    MotionNCResetExecutionState{};
                m_state = NCState::READY;
                UpdateSystemVariables();
            }
        }

        ObservePreparedBlockQueueShadow(false);

        // ⚠️ 只要還在滑行，就立刻 return，不准執行下面的 G 碼解析與模式分流！
        return;
    }

    // =========================================================
// Machine Ready Interlock
// =========================================================
    if (m_edmState == EDMState::NOT_READY)
    {
        PauseGapDryRunSameThread("INTERLOCK");
        CancelPathCoreHoldAutomaticSameThread("INTERLOCK");
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
        ObservePreparedBlockQueueShadow(false);
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
        ObservePreparedBlockQueueShadow(false);
        return;
    }

    // =========================================================
    // NC-0.2I.3：Deferred Feed Hold Resume 只能在 Alarm / Reset / Machine
    // Ready Interlock 全部通過後套用。套用當圈直接 return，避免同一
    // 10 ms 週期內又立刻 Dispatch 下一個 Block。
    // =========================================================
    // CF dry run: the automatic request shares the existing stop/resume gates.
    if (ProcessPathCoreHoldAutomaticSameThread())
    {
        ObserveLifecycleInterruptionShadow();
        ObservePreparedBlockQueueShadow(false);
        return;
    }
    if (ProcessFeedHoldResumeGate())
    {
        ObserveLifecycleInterruptionShadow();
        ObservePreparedBlockQueueShadow(false);
        return;
    }

    if (m_holdResumeAdmissionKind ==
        HoldResumeAdmissionKind::CONTROLLED_SINGLE_BLOCK)
    {
        (void)ApplyControlledSingleBlockResume();
        ObserveLifecycleInterruptionShadow();
        ObservePreparedBlockQueueShadow(false);
        return;
    }

    if (m_holdResumeAdmissionKind ==
        HoldResumeAdmissionKind::PROGRAM_HOLD)
    {
        if (!m_holdResumeGateControlled)
        {
            (void)ApplyProgramHoldResume(false);
        }
        ObserveLifecycleInterruptionShadow();
        ObservePreparedBlockQueueShadow(false);
        return;
    }

    // =========================================================
    // 🌟 3. 正常任務分流 (只有在無警報時才會走到這裡)
    // =========================================================
    // K.1 pre-observation fills only the fixed Shadow window.  The existing
    // mode switch remains the sole Runtime dispatcher.
    ObservePreparedBlockQueueShadow(true);

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

    // Observe the Dispatch / Commit evidence created by the unchanged
    // Runtime path in this same 10 ms task.
    ObservePreparedBlockQueueShadow(false);
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

                // Stage NC-0.2K.4.2: ordinary no-P G00 must consume the real
                // Stage F callback/dual-key result before the callback is
                // cleared or PC can retire.  Other callback lanes are no-ops.
                const NCBlockDispatchId completionDispatchId =
                    m_waitingBlockDispatchId;
                const bool resolverBypassCompletionValid =
                    m_preparedHeadResolverBypassGate.
                    ObserveCompletionWaitSample(
                        completionDispatchId,
                        m_blockCompletionBoundaryObserver.
                        GetLastSnapshot(),
                        legacyReady,
                        effectiveReady);
                if (!resolverBypassCompletionValid)
                {
                    m_ordinaryG00AdmissionShadow.
                        ObserveRuntimeFailure(completionDispatchId);
                    m_ordinaryG00InflightRegistryShadow.
                        ObserveRuntimeFailure(completionDispatchId);
                    const NCPreparedResolverBypassSnapshot bypassSnapshot =
                        m_preparedHeadResolverBypassGate.GetSnapshot();
                    if (completionDispatchId !=
                        NC_BLOCK_DISPATCH_ID_INVALID)
                    {
                        m_blockLifecycleLedger.MarkNCDispatchFailed(
                            completionDispatchId,
                            static_cast<std::uint32_t>(
                                AlarmManager::SYNTAX_ERROR));
                    }
                    AlarmManager::GetInstance().Trigger(
                        AlarmManager::SYNTAX_ERROR,
                        bypassSnapshot.sourceLineNumber);
                    m_state = NCState::ALARM;
                    return;
                }

                // Stage NC-0.2K.5 is read-only.  It correlates the accepted
                // K.4.2 ordinary callback/dual-key completion with the same
                // immutable candidate, before callback clear and PC advance.
                m_ordinaryG00AdmissionShadow.
                    ObserveLegacyCompletionSample(
                        completionDispatchId,
                        m_preparedHeadResolverBypassGate.GetSnapshot(),
                        legacyReady,
                        effectiveReady);
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
            PausePathCoreLiveRetentionSameThread();
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
        bool lineCommitSucceeded = false;
        NCProgramCommitSnapshot lineCommitSnapshot{};
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
                if (CommitProgramBlock(
                    commitTarget,
                    lineCommitSnapshot))
                {
                    lineCommitSucceeded =
                        m_blockLifecycleLedger.MarkProgramCommitted(
                            dispatchId,
                            lineCommitSnapshot);
                }
                else
                {
                    // 只影響 Ledger 診斷；不改變既有 NC 行為。
                    m_blockLifecycleLedger.MarkNCDispatchFailed(
                        dispatchId,
                        0x020D0001U);
                    lineCommitSucceeded = false;
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

            // BT-BEGIN
            // These control branches bypass ordinary NCBlock capture. Revoke
            // only an already chosen suffix, before evaluating side effects.
            if (m_pathCoreReturnCursor.state == PathCoreReturnCursorState::ACTIVE ||
                m_pathCoreReturnCursor.state == PathCoreReturnCursorState::PENDING ||
                m_pathCoreReturnCursor.forwardAvailable)
                InvalidatePathCoreReturnCursorSameThread();
            // BT-END
            ClearPathCoreReplayHistorySameThread(); // BZ: control side effect interrupts saved path.
            InvalidatePathCoreHoldSameThread(); // CB: assignments cannot carry a one-shot arm.

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

            // BT-BEGIN
            // These control branches bypass ordinary NCBlock capture. Revoke
            // only an already chosen suffix, before evaluating side effects.
            if (m_pathCoreReturnCursor.state == PathCoreReturnCursorState::ACTIVE ||
                m_pathCoreReturnCursor.state == PathCoreReturnCursorState::PENDING ||
                m_pathCoreReturnCursor.forwardAvailable)
                InvalidatePathCoreReturnCursorSameThread();
            // BT-END
            ClearPathCoreReplayHistorySameThread(); // BZ: branch interrupts saved path.
            InvalidatePathCoreHoldSameThread(); // CB: branches cannot carry a one-shot arm.

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
                if (gotoEpoch == MOTION_EXECUTION_EPOCH_INVALID)
                {
                    markDispatchFailed(
                        static_cast<std::uint32_t>(
                            AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY));
                    TriggerMappingIntegrityAlarmOnce(sourceLineNumber);
                    m_state = NCState::ALARM;
                    return;
                }
                RecordLifecycleInterruptionEpochPublished(gotoEpoch);
                if (!ArmPendingGotoQueueTailRebase(
                    gotoEpoch,
                    sourceLineNumber))
                {
                    markDispatchFailed(
                        static_cast<std::uint32_t>(
                            AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY));
                    TriggerMappingIntegrityAlarmOnce(sourceLineNumber);
                    m_state = NCState::ALARM;
                    return;
                }
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

            // Stage NC-0.2K.4.2: capture one immutable head before either
            // value-source path is chosen.  A true K.4.2 decision is the only
            // path that skips the resolver; a rejection enters the accepted
            // K.4 legacy chain without changing its internal order.
            NCPreparedHeadCutoverContext preparedCutoverContext =
                CapturePreparedHeadBeforeResolve(
                    currentPC,
                    sourceLineNumber);
            std::uint64_t preResolveDrainDepth = 0ULL;
            bool preResolveGroupStandstill = true;
            if (preparedCutoverContext.legacyDrainRequired)
            {
                preResolveDrainDepth =
                    static_cast<std::uint64_t>(
                        m_motion.GetQueueSize());
                preResolveGroupStandstill =
                    m_motion.IsGroupNCDrained();
                preparedCutoverContext.legacyDrainSatisfied =
                    preResolveDrainDepth == 0ULL &&
                    preResolveGroupStandstill;
            }
            const auto isOrdinaryProgramAxis = [](char letter) noexcept
            {
                return
                    letter == 'X' || letter == 'Y' || letter == 'Z' ||
                    letter == 'A' || letter == 'B' || letter == 'C' ||
                    letter == 'U' || letter == 'V' || letter == 'W';
            };
            bool ordinaryConfiguredAxisPresent = false;
            if (preparedCutoverContext.hasHead)
            {
                // The ordinary no-P lane may bypass only when the immutable
                // Prepared block addresses at least one axis that this
                // runtime actually maps and enables.  This keeps an
                // unconfigured U/V/W-only line on the accepted legacy path.
                const int axisSlotCount = static_cast<int>(
                    sizeof(m_axisNames) / sizeof(m_axisNames[0]));
                for (int axisIndex = 0;
                    axisIndex < axisSlotCount;
                    ++axisIndex)
                {
                    const char axisLetter = GetAxisName(axisIndex);
                    if (isOrdinaryProgramAxis(axisLetter) &&
                        preparedCutoverContext.head.preparedBlock.has(
                            axisLetter) &&
                        m_motion.GetAxisContext(axisIndex).isExist)
                    {
                        ordinaryConfiguredAxisPresent = true;
                        break;
                    }
                }
            }
            NCBlock block{};
            NCBlock readAheadCandidateBlock{};
            const bool readAheadCandidateExact =
                m_preparedHeadResolverBypassGate.
                TryInspectOrdinaryG00ReadAheadCandidate(
                    preparedCutoverContext,
                    parsedBlock,
                    m_preparedHeadEquivalenceShadow.GetCounters(),
                    m_preparedHeadCutoverGate.GetSnapshot(),
                    m_preparedHeadCutoverGate.GetCounters(),
                    m_preparedHeadPreResolveAdmissionShadow.GetSnapshot(),
                    m_preparedHeadPreResolveAdmissionShadow.GetCounters(),
                    readAheadCandidateBlock,
                    ordinaryConfiguredAxisPresent);

            NCOrdinaryG00ReadAheadSelectResult readAheadSelectResult =
                NCOrdinaryG00ReadAheadSelectResult::NOT_SELECTED;
            if (readAheadCandidateExact)
            {
                const NCOrdinaryG00InflightRegistrySnapshot
                    ordinaryRegistrySnapshot =
                    m_ordinaryG00InflightRegistryShadow.GetSnapshot();
                const NCOrdinaryG00FeedHoldCohortAdmissionResult
                    cohortAdmission =
                    m_ordinaryG00FeedHoldCohortCutoverGate.
                    EvaluateAdmission(
                        ordinaryRegistrySnapshot);
                const NCOrdinaryG00FeedHoldRearmAdmissionResult
                    rearmAdmission =
                    m_ordinaryG00FeedHoldCohortRearmCutoverGate.
                    EvaluateAdmission(ordinaryRegistrySnapshot);
                const NCOrdinaryG00FeedHoldRollingAdmissionResult
                    rollingAdmission =
                    m_ordinaryG00FeedHoldRollingRearmCutoverGate.
                    EvaluateAdmission(ordinaryRegistrySnapshot);
                const bool feedHoldLegacyFallback =
                    cohortAdmission ==
                    NCOrdinaryG00FeedHoldCohortAdmissionResult::
                    FALLBACK_LEGACY ||
                    rearmAdmission ==
                    NCOrdinaryG00FeedHoldRearmAdmissionResult::
                    FALLBACK_LEGACY ||
                    rollingAdmission ==
                    NCOrdinaryG00FeedHoldRollingAdmissionResult::
                    FALLBACK_LEGACY;
                const bool feedHoldAdmissionWait =
                    cohortAdmission ==
                    NCOrdinaryG00FeedHoldCohortAdmissionResult::
                    WAIT_COHORT ||
                    rearmAdmission ==
                    NCOrdinaryG00FeedHoldRearmAdmissionResult::
                    WAIT_REARM ||
                    rollingAdmission ==
                    NCOrdinaryG00FeedHoldRollingAdmissionResult::
                    WAIT_ROLLING_REARM;
                if (!feedHoldLegacyFallback && !feedHoldAdmissionWait)
                {
                    block = readAheadCandidateBlock;
                    readAheadSelectResult =
                        m_ordinaryG00ReadAheadCutoverGate.TrySelect(
                            preparedCutoverContext,
                            true,
                            m_ordinaryG00AdmissionShadow.GetSnapshot(),
                            m_ordinaryG00AdmissionShadow.GetCounters(),
                            m_ordinaryG00InflightRegistryShadow.GetSnapshot(),
                            m_ordinaryG00InflightRegistryShadow.GetCounters(),
                            preResolveDrainDepth,
                            block);
                }
                if (!feedHoldLegacyFallback && feedHoldAdmissionWait)
                {
                    ObservePreDispatchBarrier(
                        NCPreDispatchBarrierKind::G_CODE_BARRIER,
                        currentPC,
                        sourceLineNumber,
                        -1,
                        preResolveDrainDepth,
                        preResolveGroupStandstill);
                    return;
                }
            }
            if (readAheadSelectResult ==
                NCOrdinaryG00ReadAheadSelectResult::WAIT_CAPACITY)
            {
                ObservePreDispatchBarrier(
                    NCPreDispatchBarrierKind::G_CODE_BARRIER,
                    currentPC,
                    sourceLineNumber,
                    -1,
                    preResolveDrainDepth,
                    preResolveGroupStandstill);
                return;
            }
            const bool ordinaryReadAheadSelected =
                readAheadSelectResult ==
                NCOrdinaryG00ReadAheadSelectResult::SELECTED;

            bool preparedResolverBypassed = false;
            if (!ordinaryReadAheadSelected)
            {
                preparedResolverBypassed =
                    m_preparedHeadResolverBypassGate.
                    TrySelectPreparedBlock(
                        preparedCutoverContext,
                        parsedBlock,
                        m_preparedHeadEquivalenceShadow.GetCounters(),
                        m_preparedHeadCutoverGate.GetSnapshot(),
                        m_preparedHeadCutoverGate.GetCounters(),
                        m_preparedHeadPreResolveAdmissionShadow.
                        GetSnapshot(),
                        m_preparedHeadPreResolveAdmissionShadow.
                        GetCounters(),
                        block,
                        ordinaryConfiguredAxisPresent);
            }

            const NCPreparedResolverBypassSnapshot
                preparedResolverBypassSelection =
                m_preparedHeadResolverBypassGate.GetSnapshot();

            // Stage NC-0.2K.5 observes the K.4.2 decision only.  Its result is
            // deliberately not used by Resolver selection, barrier handling,
            // ExecuteBlock, callback assignment, Commit, or PC control.
            if (!ordinaryReadAheadSelected)
            {
                m_ordinaryG00AdmissionShadow.ObserveResolverDecision(
                    preparedCutoverContext,
                    preparedResolverBypassSelection,
                    preparedResolverBypassed,
                    preResolveDrainDepth,
                    preResolveGroupStandstill,
                    ordinaryConfiguredAxisPresent);
            }
            if (!ordinaryReadAheadSelected &&
                !preparedResolverBypassed &&
                preparedResolverBypassSelection.decision ==
                NCPreparedResolverBypassDecision::WAIT_LEGACY_DRAIN)
            {
                // The ordinary lane is already qualified, so this exact token
                // waits before Resolver/K.2/K.3/K.4 and cannot leave a
                // synthetic legacy pending token behind.
                ObservePreDispatchBarrier(
                    NCPreDispatchBarrierKind::G_CODE_BARRIER,
                    currentPC,
                    sourceLineNumber,
                    -1,
                    preResolveDrainDepth,
                    preResolveGroupStandstill);
                return;
            }

            if (!ordinaryReadAheadSelected &&
                !preparedResolverBypassed)
            {
                // Stage NC-0.2K.4 remains an observation-only oracle on the
                // complete legacy path.  Bypassed tokens never call this
                // observer, so K.4 cannot retain a synthetic pending token.
                (void)m_preparedHeadPreResolveAdmissionShadow.
                    ObserveBeforeResolve(
                        preparedCutoverContext,
                        m_preparedHeadEquivalenceShadow.GetCounters(),
                        m_preparedHeadCutoverGate.GetSnapshot(),
                        m_preparedHeadCutoverGate.GetCounters());

                NCExpressionResolveError resolveError =
                    NCExpressionResolveError::NONE;
                if (!NCExpressionResolver::ResolveBlock(
                    parsedBlock,
                    MathParser,
                    block,
                    resolveError))
                {
                    // K.2 must see the asymmetric outcome where K.1 accepted
                    // a literal Prepared head but the Runtime resolver
                    // rejected it.  This remains diagnostic-only.
                    m_preparedHeadPreResolveAdmissionShadow.
                        ObserveResolveFailure(
                            preparedCutoverContext.runtimeSource,
                            currentPC,
                            sourceLineNumber);
                    ObservePreparedHeadEquivalenceResolveFailure(
                        preparedCutoverContext);

                    int alarmCode = AlarmManager::MATH_ERROR;
                    if (resolveError ==
                        NCExpressionResolveError::TOO_MANY_G_CODES)
                    {
                        alarmCode = AlarmManager::G_code_Count_Error;
                    }
                    else if (resolveError ==
                        NCExpressionResolveError::TOO_MANY_M_CODES)
                    {
                        alarmCode = AlarmManager::M_code_Count_Error;
                    }
                    else if (resolveError ==
                        NCExpressionResolveError::INVALID_VARIABLE_INDEX)
                    {
                        alarmCode =
                            AlarmManager::MACRO_VARIABLE_INDEX_OUT_OF_RANGE;
                    }
                    else if (resolveError ==
                        NCExpressionResolveError::INVALID_CODE_VALUE ||
                        resolveError ==
                        NCExpressionResolveError::INVALID_PARSED_BLOCK)
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

            // NC-0.2K.7.1 owns the only ordinary no-P G00 exception to the
            // legacy drain barrier.  Its exact-stop boundary is command-local
            // in Motion, so the NC producer may commit and inspect the next
            // Prepared head without waiting on a per-block callback.
            if (ordinaryReadAheadSelected)
            {
                isBarrier = false;
            }

            // K.4.2 adds exactly one expected barrier lane: ordinary no-P G00.
            // PURE_MODAL/P1 must remain no-barrier.  Ordinary must retain its
            // exact K.1 drain classification and the pre-resolve drain proof.
            const bool selectedOrdinaryG00 =
                preparedResolverBypassed &&
                preparedResolverBypassSelection.lane ==
                NCPreparedResolverBypassLane::G00_NO_P &&
                preparedCutoverContext.hasHead &&
                preparedCutoverContext.legacyDrainRequired &&
                preparedCutoverContext.legacyDrainSatisfied &&
                preparedCutoverContext.head.classification.
                legacyDrainRequired;
            const bool selectedBarrierInvariant =
                ordinaryReadAheadSelected
                ? !isBarrier
                : (!preparedResolverBypassed ||
                    (selectedOrdinaryG00 ? isBarrier : !isBarrier));
            if (!selectedBarrierInvariant)
            {
                if (ordinaryReadAheadSelected)
                {
                    m_ordinaryG00ReadAheadCutoverGate.
                        ObserveRuntimeFailure(
                            NC_BLOCK_DISPATCH_ID_INVALID);
                }
                else
                {
                    m_preparedHeadResolverBypassGate.
                        ObserveSelectedInvariantFailure();
                    m_ordinaryG00AdmissionShadow.
                        ObserveRuntimeFailure(0ULL);
                }
                m_ordinaryG00InflightRegistryShadow.
                    ObserveRuntimeFailure(0ULL);
                markDispatchFailed(
                    static_cast<std::uint32_t>(
                        AlarmManager::SYNTAX_ERROR));
                AlarmManager::GetInstance().Trigger(
                    AlarmManager::SYNTAX_ERROR,
                    sourceLineNumber);
                m_state = NCState::ALARM;
                return;
            }

            // Stage NC-0.2K.2: compare the exact Prepared head against the
            // unchanged Runtime-resolved value.  This is deliberately before
            // lifecycle creation, handler execution and every side effect.
            // A mismatch only closes future readiness; legacy execution below
            // remains available as the fail-closed Runtime path.
            bool preparedEquivalencePending = false;
            if (!ordinaryReadAheadSelected &&
                !preparedResolverBypassed)
            {
                preparedEquivalencePending =
                    ObservePreparedHeadEquivalenceResolved(
                        currentPC,
                        sourceLineNumber,
                        parsedBlock,
                        block,
                        isBarrier,
                        preparedCutoverContext);
            }

            if (isBarrier)
            {
                const std::uint64_t commandQueueDepth =
                    static_cast<std::uint64_t>(m_motion.GetQueueSize());
                const bool groupStandstill =
                    m_motion.IsGroupNCDrained();
                preparedCutoverContext.legacyDrainSatisfied =
                    commandQueueDepth == 0ULL && groupStandstill;
                if (commandQueueDepth > 0ULL || !groupStandstill)
                {
                    if (preparedResolverBypassed)
                    {
                        // Resolver selection is irreversible.  A drain race
                        // after selection is a proof failure, never a retry.
                        m_preparedHeadResolverBypassGate.
                            ObserveSelectedInvariantFailure();
                        m_ordinaryG00AdmissionShadow.
                            ObserveRuntimeFailure(0ULL);
                        m_ordinaryG00InflightRegistryShadow.
                            ObserveRuntimeFailure(0ULL);
                        markDispatchFailed(
                            static_cast<std::uint32_t>(
                                AlarmManager::SYNTAX_ERROR));
                        AlarmManager::GetInstance().Trigger(
                            AlarmManager::SYNTAX_ERROR,
                            sourceLineNumber);
                        m_state = NCState::ALARM;
                        return;
                    }

                    m_preparedHeadPreResolveAdmissionShadow.
                        ObserveLegacyDrainWait(
                            preparedCutoverContext,
                            m_preparedHeadEquivalenceShadow.GetSnapshot(),
                            preparedEquivalencePending);

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
                preparedCutoverContext.legacyDrainSatisfied = true;
                ClearPreDispatchBarrier();
            }

            const NCBlockDispatchId dispatchId = ensureBlockLifecycle();
            bool preparedEquivalenceDispatchBound = false;
            if (ordinaryReadAheadSelected)
            {
                NCBlockLifecycleSnapshot readAheadLedger{};
                const bool readAheadLedgerFound =
                    m_blockLifecycleLedger.TryGetSnapshot(
                        dispatchId,
                        readAheadLedger);
                if (!m_ordinaryG00ReadAheadCutoverGate.BindDispatch(
                    preparedCutoverContext,
                    dispatchId,
                    commitTarget,
                    BuildPreparedBlockSourceIdentity(),
                    readAheadLedgerFound,
                    readAheadLedger.programTarget,
                    readAheadLedger.sourceLineNumber))
                {
                    m_ordinaryG00InflightRegistryShadow.
                        ObserveRuntimeFailure(dispatchId);
                    m_blockLifecycleLedger.MarkNCDispatchFailed(
                        dispatchId,
                        static_cast<std::uint32_t>(
                            AlarmManager::SYNTAX_ERROR));
                    AlarmManager::GetInstance().Trigger(
                        AlarmManager::SYNTAX_ERROR,
                        sourceLineNumber);
                    m_state = NCState::ALARM;
                    return;
                }
            }
            else if (preparedResolverBypassed)
            {
                NCBlockLifecycleSnapshot bypassLedger{};
                const bool bypassLedgerFound =
                    m_blockLifecycleLedger.TryGetSnapshot(
                        dispatchId,
                        bypassLedger);
                if (!m_preparedHeadResolverBypassGate.BindDispatch(
                    preparedCutoverContext,
                    dispatchId,
                    commitTarget,
                    BuildPreparedBlockSourceIdentity(),
                    bypassLedgerFound,
                    bypassLedger.programTarget,
                    bypassLedger.sourceLineNumber))
                {
                    m_ordinaryG00AdmissionShadow.
                        ObserveRuntimeFailure(dispatchId);
                    m_ordinaryG00InflightRegistryShadow.
                        ObserveRuntimeFailure(dispatchId);
                    m_blockLifecycleLedger.MarkNCDispatchFailed(
                        dispatchId,
                        static_cast<std::uint32_t>(
                            AlarmManager::SYNTAX_ERROR));
                    AlarmManager::GetInstance().Trigger(
                        AlarmManager::SYNTAX_ERROR,
                        sourceLineNumber);
                    m_state = NCState::ALARM;
                    return;
                }
            }
            else
            {
                preparedEquivalenceDispatchBound =
                    preparedEquivalencePending &&
                    BindPreparedHeadEquivalenceDispatch(
                        dispatchId,
                        commitTarget);
            }

            // Stage NC-0.2K.3.1: the legacy resolver has already run and the
            // current Prepared head has already passed K.2 exact comparison,
            // the drain gate and exact Ledger Dispatch binding.  Only now may
            // a previously qualified session replace the value source.  A
            // rejection leaves block untouched and the legacy path continues.
            // EMPTY has no handler value to replace, so it remains a K.2
            // proof only and is not counted as a K.3 use attempt.  Ordinary
            // literal G00 no longer requires the P1 admission sentinel.
            bool lastMileValueExact = false;
            bool preparedCutoverApplied = false;
            if (!ordinaryReadAheadSelected &&
                !preparedResolverBypassed)
            {
                lastMileValueExact =
                    preparedEquivalenceDispatchBound &&
                    !block.isEmpty &&
                    m_preparedHeadEquivalenceShadow.
                    RevalidateBoundPreparedValue(
                        preparedCutoverContext.head,
                        block);
                if (preparedEquivalenceDispatchBound && !block.isEmpty)
                {
                    NCBlock selectedBlock = block;
                    preparedCutoverApplied =
                        m_preparedHeadCutoverGate.SelectExactPreparedValue(
                            preparedCutoverContext,
                            m_preparedHeadEquivalenceShadow.GetSnapshot(),
                            m_preparedHeadEquivalenceShadow.GetCounters(),
                            BuildPreparedBlockSourceIdentity(),
                            dispatchId,
                            lastMileValueExact,
                            block,
                            selectedBlock);
                    if (preparedCutoverApplied)
                    {
                        block = selectedBlock;
                    }
                }

                // K.4's structural prediction is accepted only when the
                // complete legacy/K.2/K.3 same-token path independently
                // proves it.  K.4.1 records this as a pending per-lane
                // qualification; it is not armed until post-Commit proof.
                m_preparedHeadPreResolveAdmissionShadow.
                    ObserveResolvedOutcome(
                        preparedCutoverContext,
                        m_preparedHeadEquivalenceShadow.GetSnapshot(),
                        m_preparedHeadCutoverGate.GetSnapshot(),
                        dispatchId,
                        preparedEquivalencePending,
                        preparedEquivalenceDispatchBound,
                        lastMileValueExact,
                        preparedCutoverApplied);
                m_preparedHeadResolverBypassGate.ObserveLegacyConfirmation(
                    preparedCutoverContext,
                    m_preparedHeadCutoverGate.GetSnapshot(),
                    m_preparedHeadPreResolveAdmissionShadow.GetSnapshot(),
                    dispatchId,
                    ordinaryConfiguredAxisPresent);
            }

            // Stage NC-0.2D：只在 NC Producer 執行緒收集此 Block 建立的
            // Segment Identity；不把 NC 型別帶入 250 us Motion Runtime。
// BQ-BEGIN
            BeginPathCoreHoldCaptureSameThread(block, dispatchId); // CB: bind only the next explicit original source.
            BeginPathCoreReplayCaptureSameThread(block, dispatchId); // BZ-REPLAY
            BeginPathCoreFeedCaptureSameThread(block, dispatchId); // BX-FEED
            BeginPathCoreArcCaptureSameThread(block, dispatchId); // BY-ARC
            BeginPathCoreCommandedCaptureSameThread(block, dispatchId);
            // BQ-END
            m_motion.BeginProgramBlockMotionCapture();

            // Stage NC-0.2A：同一 Block 的 Modal 已先 Commit，才擷取 Snapshot。
            m_currentExecutingBlockDispatchId = dispatchId;
            if (!block.isEmpty)
            {
                ExecuteBlock(
                    block,
                    currentPC,
                    sourceLineNumber,
                    dispatchId);
            }
            m_currentExecutingBlockDispatchId =
                NC_BLOCK_DISPATCH_ID_INVALID;

            const MotionProgramBlockCapture motionCapture =
                m_motion.EndProgramBlockMotionCapture();
            BindProgramBlockMotionCapture(
                dispatchId,
                motionCapture);

            // A producer-side invalid command publishes the K.2.1 mailbox in
            // this same NC scan, after the ordinary ProcessTask preflight.
            // Materialize 3021 now so the existing dispatch-failure branch
            // owns the block and PC cannot commit past the rejected command.
            EnsureMappingIntegrityAlarmBoundaryBeforeFeedback();

            if (AlarmManager::GetInstance().HasAlarm())
            {
                if (ordinaryReadAheadSelected)
                {
                    m_ordinaryG00ReadAheadCutoverGate.
                        ObserveRuntimeFailure(dispatchId);
                    m_ordinaryG00InflightRegistryShadow.
                        ObserveRuntimeFailure(dispatchId);
                }
                else if (preparedResolverBypassed)
                {
                    m_preparedHeadResolverBypassGate.
                        ObserveRuntimeFailure(dispatchId);
                    m_ordinaryG00AdmissionShadow.
                        ObserveRuntimeFailure(dispatchId);
                    m_ordinaryG00InflightRegistryShadow.
                        ObserveRuntimeFailure(dispatchId);
                }
                else
                {
                    FailPreparedHeadEquivalenceRuntime(dispatchId);
                }
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
                if (ordinaryReadAheadSelected)
                {
                    m_ordinaryG00ReadAheadCutoverGate.
                        ObserveRuntimeFailure(dispatchId);
                    m_ordinaryG00InflightRegistryShadow.
                        ObserveRuntimeFailure(dispatchId);
                }
                else if (preparedResolverBypassed)
                {
                    m_preparedHeadResolverBypassGate.
                        ObserveRuntimeFailure(dispatchId);
                    m_ordinaryG00AdmissionShadow.
                        ObserveRuntimeFailure(dispatchId);
                    m_ordinaryG00InflightRegistryShadow.
                        ObserveRuntimeFailure(dispatchId);
                }
                else
                {
                    FailPreparedHeadEquivalenceRuntime(dispatchId);
                }
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

            // 本行已完成已選定的 value-source path 與 NC Side Effect /
            // Downstream Dispatch。Motion 實際完成仍由 Feedback /
            // Physical PC 表示。
            commitCurrentLine();

            if (!ordinaryReadAheadSelected &&
                !preparedResolverBypassed &&
                preparedEquivalenceDispatchBound)
            {
                CompletePreparedHeadEquivalence(
                    dispatchId,
                    lineCommitSnapshot,
                    lineCommitSucceeded);
            }

            NCBlockLifecycleSnapshot resolverBypassCommitLedger{};
            const bool resolverBypassCommitLedgerFound =
                m_blockLifecycleLedger.TryGetSnapshot(
                    dispatchId,
                    resolverBypassCommitLedger);
            const bool resolverBypassCommitValid =
                m_preparedHeadResolverBypassGate.ObserveProgramCommit(
                    dispatchId,
                    lineCommitSnapshot,
                    BuildPreparedBlockModalSnapshot(),
                    BuildPreparedBlockSourceIdentity(),
                    m_waitCallback != nullptr,
                    lineCommitSucceeded,
                    resolverBypassCommitLedgerFound,
                    resolverBypassCommitLedger.programCommitted,
                    resolverBypassCommitLedger.programTarget,
                    resolverBypassCommitLedger.programCommit,
                    resolverBypassCommitLedger.sourceLineNumber);
            if (preparedResolverBypassed &&
                !resolverBypassCommitValid)
            {
                m_ordinaryG00AdmissionShadow.
                    ObserveRuntimeFailure(dispatchId);
                m_ordinaryG00InflightRegistryShadow.
                    ObserveRuntimeFailure(dispatchId);
                m_blockLifecycleLedger.MarkNCDispatchFailed(
                    dispatchId,
                    static_cast<std::uint32_t>(
                        AlarmManager::SYNTAX_ERROR));
                AlarmManager::GetInstance().Trigger(
                    AlarmManager::SYNTAX_ERROR,
                    sourceLineNumber);
                m_state = NCState::ALARM;
                return;
            }

            // K.5/K.6.2 read the completed Producer capture and K.4.2 Commit
            // snapshot.  K.6.2 additionally binds the immutable queue-tail
            // receipt; admission still cannot change the callback, epoch,
            // Motion queue, Commit, or Runtime state.
            NCOrdinaryG00LegacyCommitEvidence ordinaryAdmissionEvidence{};
            ordinaryAdmissionEvidence.dispatchId = dispatchId;
            ordinaryAdmissionEvidence.commitSequence =
                lineCommitSnapshot.sequence;
            ordinaryAdmissionEvidence.currentExecutionEpoch =
                static_cast<std::uint64_t>(
                    m_motion.GetCurrentExecutionEpoch());
            ordinaryAdmissionEvidence.submissionCount =
                motionCapture.count;
            ordinaryAdmissionEvidence.captureOverflow =
                motionCapture.overflow;
            ordinaryAdmissionEvidence.waitCallbackActive =
                m_waitCallback != nullptr;
            ordinaryAdmissionEvidence.commitSucceeded =
                lineCommitSucceeded;
            if (motionCapture.count == 1U)
            {
                const MotionProgramBlockSubmission& submission =
                    motionCapture.submissions[0U];
                ordinaryAdmissionEvidence.segmentExecutionEpoch =
                    static_cast<std::uint64_t>(
                        submission.identity.epoch);
                ordinaryAdmissionEvidence.segmentId =
                    static_cast<std::uint64_t>(
                        submission.identity.segmentId);
                ordinaryAdmissionEvidence.submissionIdentity =
                    submission.identity;
                ordinaryAdmissionEvidence.commandPathMode =
                    submission.commandPathMode;
                ordinaryAdmissionEvidence.queueTailReceipt =
                    submission.queueTailReceipt;
                ordinaryAdmissionEvidence.producerAccepted =
                    submission.producerAccepted;
                ordinaryAdmissionEvidence.immediateRejectNone =
                    submission.immediateRejectReason ==
                    MotionRejectReason::NONE;
            }
            if (!ordinaryReadAheadSelected)
            {
                m_ordinaryG00AdmissionShadow.ObserveLegacyCommit(
                    preparedCutoverContext,
                    m_preparedHeadResolverBypassGate.GetSnapshot(),
                    ordinaryAdmissionEvidence);
            }

            // NC-0.2K.7.1: a selected ordinary G00 is already accepted by
            // Motion at this point.  Registration and local Commit proof are
            // therefore mandatory; any mismatch is contained immediately by
            // AL3021 plus the existing RT emergency-stop request path.
            if (ordinaryReadAheadSelected)
            {
                NCOrdinaryG00InflightRegistrationEvidence
                    readAheadInflightEvidence{};
                readAheadInflightEvidence.dispatchId =
                    ordinaryAdmissionEvidence.dispatchId;
                readAheadInflightEvidence.commitSequence =
                    ordinaryAdmissionEvidence.commitSequence;
                readAheadInflightEvidence.currentExecutionEpoch =
                    ordinaryAdmissionEvidence.currentExecutionEpoch;
                readAheadInflightEvidence.segmentExecutionEpoch =
                    ordinaryAdmissionEvidence.segmentExecutionEpoch;
                readAheadInflightEvidence.segmentId =
                    ordinaryAdmissionEvidence.segmentId;
                readAheadInflightEvidence.submissionCount =
                    ordinaryAdmissionEvidence.submissionCount;
                readAheadInflightEvidence.submissionIdentity =
                    ordinaryAdmissionEvidence.submissionIdentity;
                readAheadInflightEvidence.commandPathMode =
                    ordinaryAdmissionEvidence.commandPathMode;
                readAheadInflightEvidence.queueTailReceipt =
                    ordinaryAdmissionEvidence.queueTailReceipt;
                readAheadInflightEvidence.captureOverflow =
                    ordinaryAdmissionEvidence.captureOverflow;
                readAheadInflightEvidence.producerAccepted =
                    ordinaryAdmissionEvidence.producerAccepted;
                readAheadInflightEvidence.immediateRejectNone =
                    ordinaryAdmissionEvidence.immediateRejectNone;
                readAheadInflightEvidence.waitCallbackActive =
                    ordinaryAdmissionEvidence.waitCallbackActive;
                readAheadInflightEvidence.commitSucceeded =
                    ordinaryAdmissionEvidence.commitSucceeded;

                readAheadInflightEvidence.ledgerFound =
                    resolverBypassCommitLedgerFound;
                readAheadInflightEvidence.ledgerDispatchId =
                    resolverBypassCommitLedger.dispatchId;
                readAheadInflightEvidence.ledgerCommitSequence =
                    resolverBypassCommitLedger.programCommit.sequence;
                readAheadInflightEvidence.ledgerScope =
                    resolverBypassCommitLedger.programCommit.scope;
                readAheadInflightEvidence.ledgerCacheGeneration =
                    resolverBypassCommitLedger.
                    programCommit.cacheGeneration;
                readAheadInflightEvidence.ledgerFrameId =
                    resolverBypassCommitLedger.programCommit.frameId;
                readAheadInflightEvidence.ledgerSourcePC =
                    resolverBypassCommitLedger.programCommit.sourcePC;
                readAheadInflightEvidence.ledgerSourceLineNumber =
                    resolverBypassCommitLedger.sourceLineNumber;
                readAheadInflightEvidence.ledgerProgramCommitted =
                    resolverBypassCommitLedger.programCommitted;
                readAheadInflightEvidence.ledgerCaptureOverflow =
                    resolverBypassCommitLedger.motionCaptureOverflow;
                readAheadInflightEvidence.ledgerMotionSegmentCount =
                    resolverBypassCommitLedger.motionSegmentCount;
                if (resolverBypassCommitLedgerFound &&
                    resolverBypassCommitLedger.motionSegmentCount == 1U)
                {
                    const NCBlockMotionSegmentSnapshot& ledgerSegment =
                        resolverBypassCommitLedger.motionSegments[0U];
                    readAheadInflightEvidence.ledgerIdentity =
                        ledgerSegment.identity;
                    readAheadInflightEvidence.ledgerProducerAccepted =
                        ledgerSegment.producerAccepted;
                    readAheadInflightEvidence.
                        ledgerImmediateRejectReason =
                        ledgerSegment.immediateRejectReason;
                }

                NCOrdinaryG00InflightRegistrationProof
                    readAheadInflightProof{};
                const bool readAheadRegistered =
                    m_ordinaryG00InflightRegistryShadow.
                    TryRegisterReadAhead(
                        preparedCutoverContext,
                        readAheadInflightEvidence,
                        readAheadInflightProof);
                const bool readAheadCommitValid =
                    readAheadRegistered &&
                    m_ordinaryG00ReadAheadCutoverGate.ObserveCommit(
                        preparedCutoverContext,
                        readAheadInflightEvidence,
                        readAheadInflightProof);
                if (!readAheadCommitValid)
                {
                    m_ordinaryG00ReadAheadCutoverGate.
                        ObserveRuntimeFailure(dispatchId);
                    m_ordinaryG00InflightRegistryShadow.
                        ObserveRuntimeFailure(dispatchId);
                    m_blockLifecycleLedger.MarkNCDispatchFailed(
                        dispatchId,
                        static_cast<std::uint32_t>(
                            AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY));
                    AlarmManager::GetInstance().Trigger(
                        AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY,
                        sourceLineNumber);
                    m_motion.RequestEmergencyStopAllAxes();
                    m_state = NCState::ALARM;
                    return;
                }

                // NC-0.2L.2A: publish only after the existing mandatory K.7
                // registry and commit proofs have both succeeded. The result
                // is history-only and is never read by this control path.
                ObservePathCoreAcceptedReadAheadInput(
                    readAheadInflightProof,
                    preparedCutoverContext);
            }

            // Stage NC-0.2K.6.3: after K.6.1/K.6.2 have independently bound
            // this exact ordinary G00 Commit, register its one already-
            // accepted Motion segment in the bounded terminal observer.  A
            // registry proof is diagnostic input to K.5 only; neither result
            // can change the accepted callback/PC/Motion path.
            const NCOrdinaryG00AdmissionSnapshot admissionAfterCommit =
                m_ordinaryG00AdmissionShadow.GetSnapshot();
            if (!ordinaryReadAheadSelected &&
                admissionAfterCommit.pending &&
                admissionAfterCommit.legacyCommitBound &&
                !admissionAfterCommit.inflightRegistryProven &&
                admissionAfterCommit.dispatchId == dispatchId)
            {
                NCOrdinaryG00InflightRegistrationEvidence
                    inflightEvidence{};
                inflightEvidence.dispatchId =
                    ordinaryAdmissionEvidence.dispatchId;
                inflightEvidence.commitSequence =
                    ordinaryAdmissionEvidence.commitSequence;
                inflightEvidence.currentExecutionEpoch =
                    ordinaryAdmissionEvidence.currentExecutionEpoch;
                inflightEvidence.segmentExecutionEpoch =
                    ordinaryAdmissionEvidence.segmentExecutionEpoch;
                inflightEvidence.segmentId =
                    ordinaryAdmissionEvidence.segmentId;
                inflightEvidence.submissionCount =
                    ordinaryAdmissionEvidence.submissionCount;
                inflightEvidence.submissionIdentity =
                    ordinaryAdmissionEvidence.submissionIdentity;
                inflightEvidence.commandPathMode =
                    ordinaryAdmissionEvidence.commandPathMode;
                inflightEvidence.queueTailReceipt =
                    ordinaryAdmissionEvidence.queueTailReceipt;
                inflightEvidence.captureOverflow =
                    ordinaryAdmissionEvidence.captureOverflow;
                inflightEvidence.producerAccepted =
                    ordinaryAdmissionEvidence.producerAccepted;
                inflightEvidence.immediateRejectNone =
                    ordinaryAdmissionEvidence.immediateRejectNone;
                inflightEvidence.waitCallbackActive =
                    ordinaryAdmissionEvidence.waitCallbackActive;
                inflightEvidence.commitSucceeded =
                    ordinaryAdmissionEvidence.commitSucceeded;

                inflightEvidence.ledgerFound =
                    resolverBypassCommitLedgerFound;
                inflightEvidence.ledgerDispatchId =
                    resolverBypassCommitLedger.dispatchId;
                inflightEvidence.ledgerCommitSequence =
                    resolverBypassCommitLedger.programCommit.sequence;
                inflightEvidence.ledgerScope =
                    resolverBypassCommitLedger.programCommit.scope;
                inflightEvidence.ledgerCacheGeneration =
                    resolverBypassCommitLedger.
                    programCommit.cacheGeneration;
                inflightEvidence.ledgerFrameId =
                    resolverBypassCommitLedger.programCommit.frameId;
                inflightEvidence.ledgerSourcePC =
                    resolverBypassCommitLedger.programCommit.sourcePC;
                inflightEvidence.ledgerSourceLineNumber =
                    resolverBypassCommitLedger.sourceLineNumber;
                inflightEvidence.ledgerProgramCommitted =
                    resolverBypassCommitLedger.programCommitted;
                inflightEvidence.ledgerCaptureOverflow =
                    resolverBypassCommitLedger.motionCaptureOverflow;
                inflightEvidence.ledgerMotionSegmentCount =
                    resolverBypassCommitLedger.motionSegmentCount;
                if (resolverBypassCommitLedgerFound &&
                    resolverBypassCommitLedger.motionSegmentCount == 1U)
                {
                    const NCBlockMotionSegmentSnapshot& ledgerSegment =
                        resolverBypassCommitLedger.motionSegments[0U];
                    inflightEvidence.ledgerIdentity =
                        ledgerSegment.identity;
                    inflightEvidence.ledgerProducerAccepted =
                        ledgerSegment.producerAccepted;
                    inflightEvidence.ledgerImmediateRejectReason =
                        ledgerSegment.immediateRejectReason;
                }

                NCOrdinaryG00InflightRegistrationProof inflightProof{};
                (void)m_ordinaryG00InflightRegistryShadow.TryRegister(
                    preparedCutoverContext,
                    m_preparedHeadResolverBypassGate.GetSnapshot(),
                    inflightEvidence,
                    inflightProof);
                m_ordinaryG00AdmissionShadow.
                    ObserveInflightRegistryRegistration(inflightProof);
            }

            // BQ-BEGIN
            CommitPathCoreCommandedCaptureSameThread(dispatchId, motionCapture,
                lineCommitSnapshot, lineCommitSucceeded, resolverBypassCommitLedgerFound,
                resolverBypassCommitLedger, currentPC, sourceLineNumber);
            CommitPathCoreFeedCaptureSameThread(dispatchId, motionCapture,
                lineCommitSnapshot, lineCommitSucceeded, resolverBypassCommitLedgerFound,
                resolverBypassCommitLedger, currentPC, sourceLineNumber); // BX-FEED
            CommitPathCoreArcCaptureSameThread(dispatchId, motionCapture,
                lineCommitSnapshot, lineCommitSucceeded, resolverBypassCommitLedgerFound,
                resolverBypassCommitLedger, currentPC, sourceLineNumber); // BY-ARC
            CommitPathCoreReplayCaptureSameThread(dispatchId, motionCapture,
                lineCommitSnapshot, lineCommitSucceeded, resolverBypassCommitLedgerFound,
                resolverBypassCommitLedger, currentPC, sourceLineNumber); // BZ-REPLAY
            CommitPathCoreHoldCaptureSameThread(dispatchId); // CB: only after authoritative BX/BY ledger binding.
            if (m_state == NCState::ALARM) return;
            // BQ-END
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
    case 1: // BX-FEED: explicit G01 uses its own feed producer.
        return StartPathCoreFeedSameThread(block);
    case 2: // BY-ARC: G17 XY circular interpolation, exact stop.
    case 3:
        return StartPathCoreArcSameThread(block);
    case 180: // CG: standalone simulated GAP input self-test.
        return StartGapDryRunSameThread(block);
    case 178: // CB: arm one Feed Hold excursion on the next original source.
    case 179: // CB: cancel an unused one-shot arm.
        return StartPathCoreHoldSameThread(block);
    case 174: // BZ: one retained segment in reverse, explicit F mm/min.
    case 175: // BZ: one retained segment forward after full retreat.
    case 176: // CA: bounded distance retreat within one saved source.
    case 177: // CA: bounded distance advance, including an early turn.
        return StartPathCoreReplaySameThread(block);
        // BR-BEGIN
    case 171:
    case 172: // BT: explicit frozen-suffix step; same Motion transaction.
    case 173: // BV: advance to the next original saved end after full retreat.
        InvalidatePathCoreArcSameThread(); // BY-ARC: no saved-G00 permission reuse.
        InvalidatePathCoreReplaySameThread(); // BZ: revoke saved geometry and active replay.
        InvalidatePathCoreHoldSameThread(); // CB: revoke unconsumed arm and original-source excursion.
        return StartPathCoreReturnSameThread(block);
        // BR-END
    case 7:
        return GCodeHandlers::Handle_G07(block, this);
    case 12:
        return GCodeHandlers::Handle_G12(block, this);
    case 161:
        return GCodeHandlers::Handle_G161(block, this);
    case 53:
        return GCodeHandlers::Handle_G53(block, this);
    case 81:
        FencePathCoreLiveRetentionSameThread(PathCoreLiveFenceReason::HOME);
        InvalidatePathCoreFeedSameThread(); // BX-FEED
        InvalidatePathCoreArcSameThread(); // BY-ARC
        InvalidatePathCoreReplaySameThread(); // BZ: revoke saved geometry and active replay.
        InvalidatePathCoreHoldSameThread(); // CB: revoke unconsumed arm and original-source excursion.
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
    // CG: reject mixed/implicit test commands before settings, tools or M outputs.
    if (NCGCodeSemantics::Contains(block, 180))
    {
        if (!IsGapDryRunBlockShapeValid(block))
        {
            RejectGapDryRunSameThread(AlarmManager::G_Code_Invalid_parameter,
                sourceLineNumber, "BLOCK_SHAPE");
            return;
        }
        m_gapDryRun.sourceLine = sourceLineNumber;
    }
    // CB: whole-block guard runs before any setting/tool/M side effect.
    const bool explicitHoldControl = NCGCodeSemantics::Contains(block, 178) || NCGCodeSemantics::Contains(block, 179);
    const bool invalidArmedFeed = m_pathHold.armed && m_pathHold.candidateDispatch == dispatchId &&
        (!block.has('F') || !std::isfinite(block.val('F')) || m_pathHold.feedMMMin > block.val('F'));
    if ((explicitHoldControl && !IsPathCoreHoldBlockShapeValid(block)) ||
        IsPathCoreHoldInputOmission(block) || invalidArmedFeed)
    {
        RejectPathCoreHoldSameThread(2U, AlarmManager::G_Code_Invalid_parameter);
        return;
    }
    // BZ: reject malformed replay before any setting/tool/M-code side effect.
    const bool explicitReplay = NCGCodeSemantics::Contains(block, 174) || NCGCodeSemantics::Contains(block, 175) ||
        NCGCodeSemantics::Contains(block, 176) || NCGCodeSemantics::Contains(block, 177);
    if ((explicitReplay && (!IsPathCoreReplayBlockShapeValid(block) || m_pathFeed.pending || m_pathArc.pending)) ||
        IsPathCoreReplayInputOmission(block) || (m_pathReplay.pending && !block.isEmpty))
    {
        if (!m_pathReplay.pending)
        {
            m_pathReplayMotion.receipt.Clear();
            m_pathReplay.dispatch = dispatchId;
            m_pathReplay.commit = 0ULL;
            m_pathReplay.bound = false;
            m_pathReplay.consumerAccepted = false;
            m_pathReplay.consumerStarted = false;
            m_pathReplay.completed = false;
            m_pathReplay.sourcePC = sourcePC;
            m_pathReplay.sourceLine = sourceLineNumber;
        }
        RejectPathCoreReplaySameThread(2U, AlarmManager::G_Code_Invalid_parameter);
        return;
    }
    if (explicitReplay)
    {
        m_pathReplay.sourcePC = sourcePC;
        m_pathReplay.sourceLine = sourceLineNumber;
    }
    // BY-ARC-BEGIN: whole-block guard before any setting/tool/M side effect.
    const bool explicitArc = NCGCodeSemantics::Contains(block, 2) || NCGCodeSemantics::Contains(block, 3);
    if ((explicitArc && (!IsPathCoreArcBlockShapeValid(block) || m_pathFeed.pending)) ||
        IsPathCoreArcInputOmission(block) ||
        (m_pathArc.pending && (explicitArc || NCGCodeSemantics::Contains(block, 0) || NCGCodeSemantics::Contains(block, 1))))
    {
        if (!m_pathArc.pending)
        {
            m_pathArcMotion.receipt.Clear();
            m_pathArc.dispatch = dispatchId;
            m_pathArc.commit = 0ULL;
            m_pathArc.sourcePC = sourcePC;
            m_pathArc.sourceLine = sourceLineNumber;
            m_pathArc.bound = false;
            m_pathArc.consumerAccepted = false;
            m_pathArc.consumerStarted = false;
            m_pathArc.completed = false;
        }
        RejectPathCoreArcSameThread(2U, AlarmManager::G_Code_Invalid_parameter);
        return;
    }
    if (explicitArc && !m_pathArc.pending)
    {
        m_pathArc.sourcePC = sourcePC;
        m_pathArc.sourceLine = sourceLineNumber;
    }
    // BY-ARC-END
    // BX-FEED: reject unsupported shape before setting/tool/M-code side effects.
    if ((NCGCodeSemantics::Contains(block, 1) && !IsPathCoreFeedBlockShapeValid(block)) ||
        IsPathCoreFeedInputOmission(block))
    {
        if (!m_pathFeed.pending)
        {
            m_pathFeedMotion.receipt.Clear();
            m_pathFeed.dispatch = dispatchId;
            m_pathFeed.commit = 0ULL;
            m_pathFeed.sourcePC = sourcePC;
            m_pathFeed.sourceLine = sourceLineNumber;
            m_pathFeed.bound = false;
            m_pathFeed.consumerAccepted = false;
            m_pathFeed.consumerStarted = false;
            m_pathFeed.completed = false;
        }
        RejectPathCoreFeedSameThread(2U, AlarmManager::G_Code_Invalid_parameter);
        return;
    }
    if (NCGCodeSemantics::Contains(block, 1) && !m_pathFeed.pending)
    {
        m_pathFeed.sourcePC = sourcePC;
        m_pathFeed.sourceLine = sourceLineNumber;
    }
    // BR-BEGIN
        // Reject malformed G171 before setting/tool/M-code side effects.
    if ((NCGCodeSemantics::Contains(block, 171) || NCGCodeSemantics::Contains(block, 172) ||
        NCGCodeSemantics::Contains(block, 173)) &&
        !IsPathCoreReturnBlockShapeValid(block))
    {
        FlushPathCoreReturnSummarySameThread();
        m_pathCoreReturnCommand = NCGCodeSemantics::Contains(block, 173) ? 173U :
            (NCGCodeSemantics::Contains(block, 172) ? 172U : 171U);
        m_pathCoreReturnSummary = PathCoreReturnSummary{};
        m_pathCoreReturnSummary.run = m_pathCoreLiveBookkeeping.currentRunToken;
        m_pathCoreReturnSummary.dispatch = dispatchId;
        RejectPathCoreReturnSameThread(2U, AlarmManager::G_Code_Invalid_parameter);
        return;
    }
    // BR-END
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
    FencePathCoreLiveRetentionSameThread(PathCoreLiveFenceReason::AXIS_CONFIG);
    InvalidatePathCoreFeedSameThread(); // BX-FEED
    InvalidatePathCoreArcSameThread(); // BY-ARC
    InvalidatePathCoreReplaySameThread(); // BZ: revoke saved geometry and active replay.
    InvalidatePathCoreHoldSameThread(); // CB: revoke unconsumed arm and original-source excursion.

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
    if (replacementEpoch == MOTION_EXECUTION_EPOCH_INVALID)
    {
        return false;
    }
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
    if (replacementEpoch == MOTION_EXECUTION_EPOCH_INVALID)
    {
        return false;
    }
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
    if (replacementEpoch == MOTION_EXECUTION_EPOCH_INVALID)
    {
        return false;
    }
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
    CancelGapDryRunSameThread("NEW_RUN");
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
    const bool sourceIdentityCurrent =
        m_programRunStartPending &&
        m_pendingProgramRunPhase != ProgramRunStartPhase::IDLE &&
        (m_pendingProgramRunOriginState == NCState::READY ||
            m_pendingProgramRunOriginState == NCState::P_END) &&
        m_state == m_pendingProgramRunOriginState &&
        m_mode == m_pendingProgramRunMode &&
        GetBaseProgramScope() == m_pendingProgramRunScope &&
        GetBaseProgramCache().GetGeneration() ==
        m_pendingProgramRunCacheGeneration;
    if (!sourceIdentityCurrent)
    {
        return false;
    }

    if (m_pendingProgramRunPhase ==
        ProgramRunStartPhase::ALARM_ADMISSION)
    {
        return
            m_pendingProgramRunExecutionEpoch ==
            MOTION_EXECUTION_EPOCH_INVALID &&
            !m_pendingProgramRunOwnerLease.IsValid();
    }

    return
        m_pendingProgramRunPhase == ProgramRunStartPhase::EPOCH_ACK &&
        m_pendingProgramRunExecutionEpoch !=
        MOTION_EXECUTION_EPOCH_INVALID &&
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
    m_pendingProgramRunPhase = ProgramRunStartPhase::IDLE;
    m_pendingProgramRunExecutionEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    m_pendingProgramRunOwnerLease = MotionOwnerLease{};
    m_pendingProgramRunMode = NCOperationMode::EDIT;
    m_pendingProgramRunOriginState = NCState::NOT_READY;
    m_pendingProgramRunScope = NCProgramScope::NONE;
    m_pendingProgramRunCacheGeneration =
        NC_PROGRAM_CACHE_GENERATION_INVALID;
    m_pendingProgramRunAlarmUpdateCount = 0U;
    m_pendingProgramRunAlarmSafetyIntentState = 0ULL;
}

bool NCManager::ArmPendingGotoQueueTailRebase(
    MotionExecutionEpoch executionEpoch,
    int sourceLineNumber) noexcept
{
    ClearPendingGotoQueueTailRebase();

    const MotionOwnerLease currentOwnerLease =
        m_motion.GetMotionOwnerLease();
    if (executionEpoch == MOTION_EXECUTION_EPOCH_INVALID ||
        !currentOwnerLease.IsValid() ||
        !currentOwnerLease.Matches(m_programMotionLease) ||
        !m_motion.IsMotionOwnerLeaseCurrent(currentOwnerLease) ||
        m_motion.GetCurrentExecutionEpoch() != executionEpoch)
    {
        return false;
    }

    m_gotoQueueTailRebasePending = true;
    m_pendingGotoQueueTailRebaseExecutionEpoch = executionEpoch;
    m_pendingGotoQueueTailRebaseOwnerLease = currentOwnerLease;
    m_pendingGotoQueueTailRebaseSourceLine = sourceLineNumber;
    return true;
}

bool NCManager::IsPendingGotoQueueTailRebaseIdentityCurrent() const noexcept
{
    return
        m_gotoQueueTailRebasePending &&
        m_pendingGotoQueueTailRebaseExecutionEpoch !=
        MOTION_EXECUTION_EPOCH_INVALID &&
        m_pendingGotoQueueTailRebaseOwnerLease.IsValid() &&
        m_pendingGotoQueueTailRebaseOwnerLease.Matches(
            m_programMotionLease) &&
        m_motion.IsMotionOwnerLeaseCurrent(
            m_pendingGotoQueueTailRebaseOwnerLease) &&
        m_motion.GetCurrentExecutionEpoch() ==
        m_pendingGotoQueueTailRebaseExecutionEpoch;
}

void NCManager::ClearPendingGotoQueueTailRebase() noexcept
{
    m_gotoQueueTailRebasePending = false;
    m_pendingGotoQueueTailRebaseExecutionEpoch =
        MOTION_EXECUTION_EPOCH_INVALID;
    m_pendingGotoQueueTailRebaseOwnerLease = MotionOwnerLease{};
    m_pendingGotoQueueTailRebaseSourceLine = 0;
}

bool NCManager::ProcessPendingProgramRunStart() noexcept
{
    if (!m_programRunStartPending)
    {
        return false;
    }

    const auto cancelPendingStart = [this](bool alarmSuperseded) noexcept
    {
        ReleasePendingProgramRunMotionOwner();
        ClearPendingProgramRunStart(true);
        m_programEndBoundary.Cancel();
        m_programEndAlarmRaised = false;
        if (alarmSuperseded)
        {
            NCState observedState =
                m_state.load(std::memory_order_acquire);
            if (observedState != NCState::RESET_STATE &&
                observedState != NCState::ALARM)
            {
                (void)m_state.compare_exchange_strong(
                    observedState,
                    NCState::ALARM,
                    std::memory_order_acq_rel,
                    std::memory_order_acquire);
            }
            m_motion.RequestEmergencyStopAllAxes();
        }
    };

    // Any state/source/cache change supersedes this exact button request.
    // Never retarget a pending start to a newer owner or execution Epoch.
    if (!IsPendingProgramRunStartIdentityCurrent())
    {
        cancelPendingStart(false);
        return true;
    }

    AlarmManager& startAlarms = AlarmManager::GetInstance();
    const auto alarmIdentityCurrent = [this, &startAlarms]() noexcept
    {
        return
            startAlarms.GetUpdateCount() ==
            m_pendingProgramRunAlarmUpdateCount &&
            AlarmManager::MotionAdmissionBaseState(
                startAlarms.GetMotionSafetyIntentState()) ==
            m_pendingProgramRunAlarmSafetyIntentState &&
            !startAlarms.HasAlarm();
    };

    if (m_programLoadStartBlocked && m_pendingProgramRunMode == NCOperationMode::MEMORY)
    {
        cancelPendingStart(startAlarms.HasAlarm());
        return true;
    }
    const auto replacement = m_lifecycleInterruptionShadow.GetSnapshot();
    if (replacement.active &&
        replacement.cause == NCLifecycleInterruptionCause::PROGRAM_REPLACED &&
        !replacement.expectsEpochChange)
    {
        // Ordinary NC observations close the inert replacement before START
        // can publish an epoch. Never count two observations inside LOAD.
        if (!alarmIdentityCurrent()) cancelPendingStart(startAlarms.HasAlarm());
        return true;
    }

    if (m_pendingProgramRunPhase ==
        ProgramRunStartPhase::ALARM_ADMISSION)
    {
        if (!alarmIdentityCurrent())
        {
            cancelPendingStart(startAlarms.HasAlarm());
            return true;
        }

        AlarmManager::MotionAdmissionReservation admission{};
        const AlarmManager::MotionAdmissionResult beginResult =
            startAlarms.TryBeginMotionAdmission(
                m_pendingProgramRunAlarmUpdateCount,
                m_pendingProgramRunAlarmSafetyIntentState,
                admission,
                true);
        if (beginResult == AlarmManager::MotionAdmissionResult::BUSY)
        {
            return true;
        }
        if (beginResult !=
            AlarmManager::MotionAdmissionResult::ACQUIRED)
        {
            cancelPendingStart(startAlarms.HasAlarm());
            return true;
        }

        if (!IsPendingProgramRunStartIdentityCurrent() ||
            admission.baseState !=
            m_pendingProgramRunAlarmSafetyIntentState)
        {
            const bool ended = startAlarms.EndMotionAdmission(admission);
            cancelPendingStart(!ended || startAlarms.HasAlarm());
            return true;
        }

        if (!AcquireProgramMotionOwner())
        {
            if (!startAlarms.EndMotionAdmission(admission))
            {
                cancelPendingStart(true);
            }
            return true;
        }

        // A fresh run has its own Epoch/baseline transaction and must never
        // inherit an unfinished GOTO rebase from an older execution.
        ClearPendingGotoQueueTailRebase();
        CancelSingleBlockShadow(false);
        CancelFeedHoldBoundaryShadow(false);
        m_legacySingleBlockPausePending = false;
        ResetActiveProgramCommitBoundary();

        if (m_pendingProgramRunOriginState == NCState::P_END)
        {
            GetBasePC() = 0;
            Reset_Gode();
            m_macroStack.clear();
        }
        m_pauseAfterBlock = false;

        const MotionExecutionEpoch executionEpoch =
            m_motion.BeginNewExecutionEpoch(
                GetMotionCommandSourceForMode(
                    m_pendingProgramRunMode));
        if (executionEpoch == MOTION_EXECUTION_EPOCH_INVALID)
        {
            ReleaseProgramMotionOwner();
            const bool ended = startAlarms.EndMotionAdmission(admission);
            if (!ended)
            {
                cancelPendingStart(true);
            }
            return true;
        }

        m_pendingProgramRunExecutionEpoch = executionEpoch;
        m_pendingProgramRunOwnerLease = m_programMotionLease;
        m_pendingProgramRunPhase = ProgramRunStartPhase::EPOCH_ACK;
        if (!startAlarms.EndMotionAdmission(admission))
        {
            cancelPendingStart(true);
        }
        return true;
    }

    if (!alarmIdentityCurrent())
    {
        cancelPendingStart(startAlarms.HasAlarm());
        return true;
    }

    const MotionExecutionEpoch pendingEpoch =
        m_pendingProgramRunExecutionEpoch;
    if (m_motion.HasPendingSafetyOrRecoveryRequests() ||
        !m_motion.HasExactProgramStartQuiescenceAcknowledgement(
            pendingEpoch,
            m_pendingProgramRunOwnerLease))
    {
        return true;
    }

    AlarmManager::MotionAdmissionReservation runAdmission{};
    const AlarmManager::MotionAdmissionResult runAdmissionResult =
        startAlarms.TryBeginMotionAdmission(
            m_pendingProgramRunAlarmUpdateCount,
            m_pendingProgramRunAlarmSafetyIntentState,
            runAdmission,
            true);
    if (runAdmissionResult ==
        AlarmManager::MotionAdmissionResult::BUSY)
    {
        return true;
    }
    if (runAdmissionResult !=
        AlarmManager::MotionAdmissionResult::ACQUIRED)
    {
        cancelPendingStart(startAlarms.HasAlarm());
        return true;
    }

    if (!IsPendingProgramRunStartIdentityCurrent() ||
        !m_motion.HasExactProgramStartQuiescenceAcknowledgement(
            pendingEpoch,
            m_pendingProgramRunOwnerLease) ||
        m_motion.HasPendingSafetyOrRecoveryRequests())
    {
        const bool ended = startAlarms.EndMotionAdmission(runAdmission);
        cancelPendingStart(!ended || startAlarms.HasAlarm());
        return true;
    }

    if (!BeginProgramRunBoundary(pendingEpoch))
    {
        const bool ended = startAlarms.EndMotionAdmission(runAdmission);
        cancelPendingStart(!ended || startAlarms.HasAlarm());
        return true;
    }

    m_motion.SyncVirtualEndPosition();
    double synchronizedQueueTailMCS[MAX_AXES] = {};
    if (!m_motion.TryGetSynchronizedG00QueueTailMCS(
        synchronizedQueueTailMCS) ||
        !IsPendingProgramRunStartIdentityCurrent() ||
        !m_motion.HasExactProgramStartQuiescenceAcknowledgement(
            pendingEpoch,
            m_pendingProgramRunOwnerLease) ||
        m_motion.HasPendingSafetyOrRecoveryRequests())
    {
        (void)startAlarms.EndMotionAdmission(runAdmission);
        TriggerMappingIntegrityAlarmOnce();
        cancelPendingStart(true);
        return true;
    }

    CoordSys.SyncMachinePosition(synchronizedQueueTailMCS);
    if (!IsPendingProgramRunStartIdentityCurrent() ||
        !m_motion.HasExactProgramStartQuiescenceAcknowledgement(
            pendingEpoch,
            m_pendingProgramRunOwnerLease) ||
        m_motion.HasPendingSafetyOrRecoveryRequests() ||
        !startAlarms.IsMotionAdmissionCurrent(runAdmission))
    {
        const bool ended = startAlarms.EndMotionAdmission(runAdmission);
        cancelPendingStart(!ended || startAlarms.HasAlarm());
        return true;
    }

    const NCState originState = m_pendingProgramRunOriginState;
    const bool startManualAuto =
        m_pendingProgramRunMode == NCOperationMode::MANUAL &&
        !m_manualProgramCache.Empty();
    if (m_pendingProgramRunMode == NCOperationMode::MANUAL)
    {
        m_manualAutoRunning = startManualAuto;
    }
    NCState expectedOriginState = originState;
    const bool committedRun =
        m_state.compare_exchange_strong(
            expectedOriginState,
            NCState::RUN,
            std::memory_order_acq_rel,
            std::memory_order_acquire);
    if (!committedRun ||
        !m_motion.IsMotionOwnerLeaseCurrent(
            m_pendingProgramRunOwnerLease) ||
        !m_motion.HasExactProgramStartQuiescenceAcknowledgement(
            pendingEpoch,
            m_pendingProgramRunOwnerLease))
    {
        if (committedRun)
        {
            NCState provisionalRun = NCState::RUN;
            (void)m_state.compare_exchange_strong(
                provisionalRun,
                originState,
                std::memory_order_acq_rel,
                std::memory_order_acquire);
        }
        m_manualAutoRunning = false;
        const bool ended =
            startAlarms.EndMotionAdmission(runAdmission);
        cancelPendingStart(!ended || startAlarms.HasAlarm());
        return true;
    }

    if (!startAlarms.EndMotionAdmission(runAdmission))
    {
        NCState provisionalRun = NCState::RUN;
        (void)m_state.compare_exchange_strong(
            provisionalRun,
            originState,
            std::memory_order_acq_rel,
            std::memory_order_acquire);
        m_manualAutoRunning = false;
        cancelPendingStart(true);
        return true;
    }

    // BN: this is the completed fresh-start admission, never a provisional RUN.
    ArmPathCoreLiveRetentionSameThread();
    ArmPathCoreFeedSameThread(); // BX-FEED: completed fresh NC Start only.
    ArmPathCoreArcSameThread(); // BY-ARC: fresh NC Start only.
    ArmPathCoreReplaySameThread(); // BZ: completed fresh run, never M00 resume.
    InvalidatePathCoreHoldSameThread(); // CB: a fresh run never inherits an old arm.
    ClearPendingProgramRunStart(false);
    UpdateSystemVariables();

    // The first block is intentionally dispatched on the next NC task.
    return true;
}

bool NCManager::RequestProgramEnd(
    NCProgramEndCause cause,
    int sourcePC,
    int sourceLineNumber,
    NCBlockDispatchId markerDispatchId) noexcept
{
    ClearPendingGotoQueueTailRebase();
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
    const NCState entryState = m_state.load(std::memory_order_acquire);

    if (entryState != NCState::RUN && entryState != NCState::HOLD)
    {
        return;
    }

    ClearPendingGotoQueueTailRebase();

    // BP-BEGIN
        // Bounded data capture precedes the ORIGINAL final fresh permission sample.
        // Failure never changes any existing End/Gate decision.
    PreparePathCoreCompletedSnapshotSameThread();
    // BP-END
    // BQ-BEGIN
    PreparePathCoreCommittedRunSameThread();
    // BQ-END
        // NC-0.2J.5: rebuild and re-evaluate every formal input immediately
        // before the permission action.  Do not finalize from the earlier scan's
        // READY_TO_FINALIZE snapshot.
    const NCProgramEndGateSample releaseSample =
        BuildProgramEndGateSample();
    if (!m_programEndBoundary.Evaluate(releaseSample))
    {
        // BP-BEGIN
        DiscardPathCoreCompletedSnapshotSameThread();
        // BP-END
        // BQ-BEGIN
        ClosePathCoreCommittedRunSameThread();
        // BQ-END
        return;
    }

    // CJ: hand the exact, freshly settled Program lease to output-only idle
    // holding before NC cleanup. A busy reservation preserves the pending End
    // transaction; ordinary producers cannot create this holding authority.
    if (!m_programMotionLease.IsValid() ||
        !m_motion.TryEnterProgramEndIdleHold(
            m_programMotionLease, releaseSample.executionEpoch))
    {
        // BP-BEGIN
        DiscardPathCoreCompletedSnapshotSameThread();
        // BP-END
        // BQ-BEGIN
        ClosePathCoreCommittedRunSameThread();
        // BQ-END
        return;
    }

    // CJ: Program authority has ended; no old live reads or NC lease survive.
    FencePathCoreLiveRetentionSameThread(PathCoreLiveFenceReason::PROGRAM_END);
    m_programMotionLease = MotionOwnerLease{};

    // The Motion CAS is a committed handoff even when immediately superseded.
    // Never retry its consumed Program lease or hide a newer Reset/Alarm state.
    const auto cancelSupersededEnd = [this, entryState, &releaseSample]() -> bool
    {
        AlarmManager& alarms = AlarmManager::GetInstance();
        const bool alarmPresent = alarms.HasAlarm();
        const bool authorityChanged =
            m_motion.GetCurrentExecutionEpoch() != releaseSample.executionEpoch ||
            m_motion.HasPendingSafetyOrRecoveryRequests();
        const NCState currentState = m_state.load(std::memory_order_acquire);
        if (!alarmPresent && !authorityChanged && currentState == entryState)
        {
            return false;
        }
        const int sourceLine = m_programEndBoundary.GetSnapshot().sourceLineNumber;
        CancelProgramEndBoundary();
        DiscardPathCoreCompletedSnapshotSameThread();
        ClosePathCoreCommittedRunSameThread();
        if (currentState == entryState)
        {
            if (!alarmPresent)
            {
                alarms.Trigger(AlarmManager::PROGRAM_END_GATE_ERROR,
                    sourceLine);
            }
            NCState expectedState = entryState;
            (void)m_state.compare_exchange_strong(expectedState, NCState::ALARM,
                std::memory_order_acq_rel, std::memory_order_acquire);
        }
        return true;
    };
    if (cancelSupersededEnd())
    {
        return;
    }

    if (!m_programEndBoundary.MarkFinalized())
    {
        // BP-BEGIN
        DiscardPathCoreCompletedSnapshotSameThread();
        // BP-END
        // BQ-BEGIN
        ClosePathCoreCommittedRunSameThread();
        // BQ-END
        return;
    }

    // BP-BEGIN
    PublishPathCoreCompletedSnapshotSameThread();
    // BP-END
    // BQ-BEGIN
    PublishPathCoreCommittedRunSameThread();
    FinalizePathCoreFeedSameThread(); // BX-FEED: original End gate already finalized.
    FinalizePathCoreArcSameThread(); // BY-ARC: current run summary only.
    FinalizePathCoreReplaySameThread(); // BZ-REPLAY
    InvalidatePathCoreHoldSameThread(); // CB: program end closes any unused arm.
// BQ-END
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

    if (cancelSupersededEnd())
    {
        return;
    }
    const NCState endState =
        (m_mode == NCOperationMode::MANUAL || m_mode == NCOperationMode::MDI)
        ? NCState::READY : NCState::P_END;
    if (m_mode == NCOperationMode::MANUAL)
    {
        m_manualAutoRunning = false;
    }
    NCState expectedState = entryState;
    (void)m_state.compare_exchange_strong(expectedState, endState,
        std::memory_order_acq_rel, std::memory_order_acquire);
}

void NCManager::CancelProgramEndBoundary() noexcept
{
    ClearPendingGotoQueueTailRebase();
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
// Stage NC-0.2K.1 - Prepared Block Queue Shadow
//
// This observer reads the immutable parsed-program cache and the already
// published Dispatch / Commit evidence.  It does not call a G/M handler,
// advance a PC, open a lifecycle block or submit anything to MotionCore.
// =============================================================================
NCPreparedSourceIdentity
NCManager::BuildPreparedBlockSourceIdentity() const noexcept
{
    NCPreparedSourceIdentity source{};

    if (!m_macroStack.empty())
    {
        const MacroFrame& frame = m_macroStack.back();
        source.scope = NCProgramScope::MACRO;
        source.frameId = frame.frameId;
        if (frame.program != nullptr)
        {
            source.cacheGeneration = frame.program->GetGeneration();
        }
    }
    else
    {
        source.scope = GetBaseProgramScope();
        source.cacheGeneration = GetBaseProgramCache().GetGeneration();
    }

    source.executionEpoch = static_cast<std::uint64_t>(
        m_motion.GetCurrentExecutionEpoch());
    source.programFlowGeneration =
        m_gmBlockTransactionCounters.m98Calls +
        m_gmBlockTransactionCounters.m99Returns;
    source.owner = static_cast<std::uint8_t>(m_programMotionLease.owner);
    source.ownerGeneration = static_cast<std::uint64_t>(
        m_programMotionLease.generation);
    source.panel.blockSkipEnabled = m_isBlockSkipEnabled;
    source.panel.singleBlockEnabled = m_isSingleBlockEnabled;
    source.panel.optionalStopEnabled = m_isOptionalStopEnabled;
    return source;
}

NCPreparedModalSnapshot
NCManager::BuildPreparedBlockModalSnapshot() const noexcept
{
    NCPreparedModalSnapshot modal{};
    modal.distanceMode = CoordSys.isAbsoluteMode ? 90 : 91;
    modal.unitsMode = CoordSys.isInchMode ? 20 : 21;
    modal.planeMode = CoordSys.activePlane;
    modal.workCoordinateCode = CoordSys.GetCurrentWCSGCode();
    modal.storedStrokeMode =
        CoordSys.IsProgrammableTravelLimitEnabled() ? 22 : 23;
    modal.toolLengthMode = CoordSys.toolLengthMode;
    modal.hCode = CoordSys.currentHCode;
    modal.toolRadiusMode = CoordSys.toolRadiusMode;
    modal.dCode = CoordSys.currentDCode;
    modal.toolCode = CoordSys.currentTCode;
    modal.g68Active = CoordSys.isG68Active;
    modal.g68Angle = CoordSys.g68Angle;
    modal.g168Active = CoordSys.isWorkpieceRotationActive;
    modal.workpieceCode = CoordSys.currentWCode;
    modal.scalingActive = CoordSys.isScalingActive;
    modal.scalingFactor = CoordSys.scaleFactor;
    modal.mirrorMask = 0U;
    for (int axis = 0; axis < 8; ++axis)
    {
        if (CoordSys.isMirrorActive[axis])
        {
            modal.mirrorMask |= static_cast<std::uint8_t>(1U << axis);
        }
        modal.commandedMCS[axis] = CoordSys.commandedMCS[axis];
    }
    modal.polarActive = CoordSys.isPolarCoordinateActive;
    modal.cAxisOffsetRotationEnabled =
        CoordSys.isCAxisOffsetRotationEnabled;
    modal.modalMacroActive = m_isG66Active;
    modal.g00OverrideRatio = m_motion.G00_overrideRatio;
    modal.commandedMCSValid = true;
    modal.imageValid = true;
    return modal;
}

NCPreparedRuntimeProof
NCManager::BuildPreparedBlockRuntimeProof() const noexcept
{
    NCPreparedRuntimeProof proof{};
    NCBlockLifecycleSnapshot lifecycle{};
    if (m_blockLifecycleLedger.GetLastDispatchedSnapshot(lifecycle))
    {
        proof.hasDispatch = lifecycle.IsValid();
        proof.dispatchId = lifecycle.dispatchId;
        proof.dispatchTarget = lifecycle.programTarget;
        if (lifecycle.programCommitted)
        {
            proof.commitTarget = lifecycle.programCommit;
        }
    }
    return proof;
}

NCPreparedInvalidationReason
NCManager::GetPreparedBlockInactiveReason() const noexcept
{
    if (AlarmManager::GetInstance().HasAlarm() ||
        m_state == NCState::ALARM)
    {
        return NCPreparedInvalidationReason::ALARM;
    }
    if (m_state == NCState::RESET_STATE)
    {
        return NCPreparedInvalidationReason::RESET;
    }
    if (m_state == NCState::P_END ||
        m_programEndBoundary.IsEndPending())
    {
        return NCPreparedInvalidationReason::PROGRAM_END;
    }
    return NCPreparedInvalidationReason::NOT_RUNNING;
}

void NCManager::ObservePreparedBlockQueueShadow(
    bool allowPlanning) noexcept
{
    const bool runtimeModeEligible =
        m_mode == NCOperationMode::MEMORY ||
        m_mode == NCOperationMode::MDI ||
        (m_mode == NCOperationMode::MANUAL && m_manualAutoRunning);
    const bool observationEligible =
        runtimeModeEligible &&
        (m_state == NCState::RUN || m_state == NCState::HOLD) &&
        !AlarmManager::GetInstance().HasAlarm() &&
        !m_programRunStartPending &&
        !m_programEndBoundary.IsEndPending() &&
        m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease);

    if (!observationEligible)
    {
        NCPreparedInvalidationReason reason =
            GetPreparedBlockInactiveReason();
        const NCPreparedRuntimeProof inactiveProof =
            BuildPreparedBlockRuntimeProof();
        if ((m_state == NCState::RUN || m_state == NCState::HOLD) &&
            !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease))
        {
            reason = NCPreparedInvalidationReason::OWNER_CHANGED;
        }
        if (allowPlanning)
        {
            m_preparedBlockQueueShadow.ObserveInactive(reason);
        }
        else
        {
            m_preparedBlockQueueShadow.ObserveFinalProofAndInvalidate(
                inactiveProof,
                reason);
            m_preparedHeadResolverBypassGate.ObserveUpstreamProof(
                m_preparedBlockQueueShadow.GetSnapshot(),
                m_preparedBlockQueueShadow.GetCounters(),
                inactiveProof,
                m_preparedHeadEquivalenceShadow.GetSnapshot(),
                m_preparedHeadEquivalenceShadow.GetCounters());
        }
        m_preparedHeadEquivalenceShadow.ObserveQueueInactive(reason);
        m_preparedHeadCutoverGate.ObserveQueueInactive(reason);
        m_preparedHeadPreResolveAdmissionShadow.ObserveQueueInactive(reason);
        m_preparedHeadResolverBypassGate.ObserveQueueInactive(reason);
        m_ordinaryG00AdmissionShadow.ObserveQueueInactive(reason);
        m_ordinaryG00ReadAheadCutoverGate.ObserveQueueInactive(reason);
        m_ordinaryG00InflightRegistryShadow.ObserveQueueInactive(reason);
        return;
    }

    const NCPreparedSourceIdentity source =
        BuildPreparedBlockSourceIdentity();
    const NCPreparedModalSnapshot modal =
        BuildPreparedBlockModalSnapshot();
    const NCPreparedRuntimeProof proof =
        BuildPreparedBlockRuntimeProof();

    const NCProgramCache* program = nullptr;
    int runtimePC = -1;
    int committedPC = -1;
    if (!m_macroStack.empty())
    {
        const MacroFrame& frame = m_macroStack.back();
        program = frame.program;
        runtimePC = frame.currentPC;
        committedPC = frame.committedPC;
    }
    else
    {
        program = &GetBaseProgramCache();
        runtimePC = GetBasePCValue();
        committedPC = GetBaseCommittedPCValue();
    }

    if (program == nullptr ||
        !m_preparedBlockQueueShadow.BeginObservation(
            source,
            runtimePC,
            committedPC,
            modal,
            proof))
    {
        m_preparedBlockQueueShadow.FinishObservation();
        ObservePreparedHeadEquivalenceUpstreamProof(proof);
        m_preparedHeadPreResolveAdmissionShadow.ObserveQueueInactive(
            NCPreparedInvalidationReason::IDENTITY_INVALID);
        m_preparedHeadResolverBypassGate.ObserveQueueInactive(
            NCPreparedInvalidationReason::IDENTITY_INVALID);
        m_ordinaryG00AdmissionShadow.ObserveQueueInactive(
            NCPreparedInvalidationReason::IDENTITY_INVALID);
        m_ordinaryG00ReadAheadCutoverGate.ObserveQueueInactive(
            NCPreparedInvalidationReason::IDENTITY_INVALID);
        m_ordinaryG00InflightRegistryShadow.ObserveQueueInactive(
            NCPreparedInvalidationReason::IDENTITY_INVALID);
        return;
    }

    const bool planningEligible =
        allowPlanning && m_state == NCState::RUN;
    std::size_t plannedSourceBytes = 0U;
    for (std::size_t budget = 0U;
        planningEligible &&
        budget < NC_PREPARED_BLOCK_PLAN_BUDGET_PER_TASK &&
        m_preparedBlockQueueShadow.CanPrepare();
        ++budget)
    {
        const int planPC = m_preparedBlockQueueShadow.GetNextPlanPC();
        if (planPC < 0 ||
            static_cast<std::size_t>(planPC) >= program->Size())
        {
            m_preparedBlockQueueShadow.ObserveNaturalEOF(planPC);
            break;
        }

        const NCProgramCacheLine* line = program->TryGetLine(planPC);
        if (line == nullptr)
        {
            // A cache hole is impossible for a valid NCProgramCache.  Turn
            // it into an explicit fail-closed planning fence without
            // affecting the existing Runtime parser or Alarm path.
            NCParsedBlock invalid{};
            invalid.error = NCParseError::INVALID_GOTO;
            (void)m_preparedBlockQueueShadow.PrepareParsedLine(
                planPC,
                planPC + 1,
                invalid);
            break;
        }

        const std::size_t sourceBytes = line->rawLine.size();
        if (sourceBytes > NC_PREPARED_BLOCK_MAX_SOURCE_BYTES)
        {
            // The Pure Parse cache may legally contain a very long line.
            // K.1 does not scan unbounded expression text in the 10 ms task;
            // retain the line as a conservative flow fence instead.
            NCParsedBlock boundedFence{};
            boundedFence.dependsOnMacroState = true;
            boundedFence.isBlockSkip = line->parsedBlock.isBlockSkip;
            (void)m_preparedBlockQueueShadow.PrepareParsedLine(
                planPC,
                line->sourceLineNumber,
                boundedFence);
            break;
        }

        if (plannedSourceBytes + sourceBytes >
            NC_PREPARED_BLOCK_SOURCE_BYTE_BUDGET_PER_TASK)
        {
            // Defer this PC to the next 10 ms planning pass.  Deferral is not
            // a parse failure and cannot invalidate the existing window.
            break;
        }
        plannedSourceBytes += sourceBytes;

        if (!m_preparedBlockQueueShadow.PrepareParsedLine(
            planPC,
            line->sourceLineNumber,
            line->parsedBlock))
        {
            break;
        }
    }

    m_preparedBlockQueueShadow.FinishObservation();
    ObservePreparedHeadEquivalenceUpstreamProof(proof);
}

// =============================================================================
// Stage NC-0.2K.2 - Prepared Head Exact Equivalence Shadow
//
// All calls execute on the existing NC producer thread.  The observer receives
// value snapshots only and cannot call ExecuteBlock, CommitProgramBlock or
// MotionCore.  K.3 may later choose the already-compared Prepared value, but
// this K.2 observer itself remains incapable of Runtime influence.
// =============================================================================
NCPreparedHeadCutoverContext
NCManager::CapturePreparedHeadBeforeResolve(
    int sourcePC,
    int sourceLineNumber) const noexcept
{
    NCPreparedHeadCutoverContext context{};
    context.queue = m_preparedBlockQueueShadow.GetSnapshot();
    context.queueCounters = m_preparedBlockQueueShadow.GetCounters();
    context.hasHead = m_preparedBlockQueueShadow.TryGetEntry(
        0U,
        context.head);
    context.runtimeSource = BuildPreparedBlockSourceIdentity();
    context.sourcePC = sourcePC;
    context.sourceLineNumber = sourceLineNumber;
    context.runtimeModalBefore = BuildPreparedBlockModalSnapshot();
    context.capturedBeforeResolve = true;
    context.runtimeModalBeforeValid =
        context.runtimeModalBefore.imageValid;
    context.legacyDrainRequired =
        context.hasHead &&
        context.head.classification.legacyDrainRequired;
    context.legacyDrainSatisfied = !context.legacyDrainRequired;
    return context;
}

bool NCManager::ObservePreparedHeadEquivalenceResolved(
    int sourcePC,
    int sourceLineNumber,
    const NCParsedBlock& parsedBlock,
    const NCBlock& legacyBlock,
    bool legacyDrainRequired,
    NCPreparedHeadCutoverContext& cutoverContext) noexcept
{
    // K.4 requires the immutable pre-resolve capture.  Do not fetch the Queue
    // head or live modal image a second time after ResolveBlock.
    if (!cutoverContext.capturedBeforeResolve ||
        cutoverContext.sourcePC != sourcePC ||
        cutoverContext.sourceLineNumber != sourceLineNumber)
    {
        return false;
    }
    cutoverContext.legacyDrainRequired = legacyDrainRequired;
    cutoverContext.legacyDrainSatisfied = !legacyDrainRequired;

    return m_preparedHeadEquivalenceShadow.ObserveResolvedHead(
        cutoverContext.queue,
        cutoverContext.queueCounters,
        cutoverContext.hasHead,
        cutoverContext.head,
        cutoverContext.runtimeSource,
        sourcePC,
        sourceLineNumber,
        parsedBlock,
        legacyBlock,
        cutoverContext.runtimeModalBefore,
        legacyDrainRequired);
}

void NCManager::ObservePreparedHeadEquivalenceResolveFailure(
    const NCPreparedHeadCutoverContext& cutoverContext) noexcept
{
    m_preparedHeadEquivalenceShadow.ObserveResolveFailure(
        cutoverContext.queue,
        cutoverContext.queueCounters,
        cutoverContext.hasHead,
        cutoverContext.head,
        cutoverContext.runtimeSource,
        cutoverContext.sourcePC,
        cutoverContext.sourceLineNumber);
}

void NCManager::ObservePreparedHeadEquivalenceUpstreamProof(
    const NCPreparedRuntimeProof& proof) noexcept
{
    const NCPreparedBlockQueueSnapshot queue =
        m_preparedBlockQueueShadow.GetSnapshot();
    const NCPreparedBlockQueueCounters queueCounters =
        m_preparedBlockQueueShadow.GetCounters();
    m_preparedHeadEquivalenceShadow.ObserveUpstreamProof(
        queue,
        queueCounters,
        proof);
    m_preparedHeadCutoverGate.ObserveEquivalenceState(
        queue,
        queueCounters,
        m_preparedHeadEquivalenceShadow.GetSnapshot(),
        m_preparedHeadEquivalenceShadow.GetCounters());
    m_preparedHeadPreResolveAdmissionShadow.
        ObserveActiveQueueSession(queue);
    m_ordinaryG00InflightRegistryShadow.ObserveActiveSession(
        queue.session);
    m_preparedHeadResolverBypassGate.ObserveUpstreamProof(
        queue,
        queueCounters,
        proof,
        m_preparedHeadEquivalenceShadow.GetSnapshot(),
        m_preparedHeadEquivalenceShadow.GetCounters());
    m_ordinaryG00AdmissionShadow.ObserveUpstreamProof(
        m_preparedHeadResolverBypassGate.GetSnapshot());
}

bool NCManager::BindPreparedHeadEquivalenceDispatch(
    NCBlockDispatchId dispatchId,
    const NCProgramCommitSnapshot& dispatchTarget) noexcept
{
    NCBlockLifecycleSnapshot ledger{};
    const bool ledgerFound =
        m_blockLifecycleLedger.TryGetSnapshot(dispatchId, ledger);
    return m_preparedHeadEquivalenceShadow.BindDispatch(
        dispatchId,
        dispatchTarget,
        BuildPreparedBlockSourceIdentity(),
        ledgerFound,
        ledger.programTarget,
        ledger.sourceLineNumber);
}

void NCManager::CompletePreparedHeadEquivalence(
    NCBlockDispatchId dispatchId,
    const NCProgramCommitSnapshot& commitTarget,
    bool commitSucceeded) noexcept
{
    NCBlockLifecycleSnapshot ledger{};
    const bool ledgerFound =
        m_blockLifecycleLedger.TryGetSnapshot(dispatchId, ledger);
    m_preparedHeadEquivalenceShadow.ObserveCommit(
        dispatchId,
        commitTarget,
        BuildPreparedBlockModalSnapshot(),
        BuildPreparedBlockSourceIdentity(),
        m_waitCallback != nullptr,
        commitSucceeded,
        ledgerFound,
        ledger.programCommitted,
        ledger.programTarget,
        ledger.programCommit,
        ledger.sourceLineNumber);
}

void NCManager::FailPreparedHeadEquivalenceRuntime(
    NCBlockDispatchId dispatchId) noexcept
{
    m_preparedHeadEquivalenceShadow.ObserveRuntimeFailure();
    m_preparedHeadCutoverGate.ObserveRuntimeFailure(dispatchId);
    m_preparedHeadPreResolveAdmissionShadow.
        ObserveConfirmedRuntimeFailure(dispatchId);
    m_preparedHeadResolverBypassGate.
        ObserveRuntimeFailure(dispatchId);
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
        // BN: main-program M99 loops without entering ReturnMacro().
        FencePathCoreLiveRetentionSameThread(PathCoreLiveFenceReason::MACRO);
        InvalidatePathCoreFeedSameThread(); // BX-FEED
        InvalidatePathCoreArcSameThread(); // BY-ARC
        InvalidatePathCoreReplaySameThread(); // BZ: revoke saved geometry and active replay.
        InvalidatePathCoreHoldSameThread(); // CB: revoke unconsumed arm and original-source excursion.
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
        CancelPathCoreHoldAutomaticSameThread("SINGLE_BLOCK");
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
    PausePathCoreLiveRetentionSameThread();
    m_state = NCState::HOLD;
    m_waitCallback = WaitForCycleStartCallback;
    return true;
}

bool NCManager::ArmHoldResumeAlarmAdmission(
    HoldResumeAdmissionKind kind) noexcept
{
    if (kind == HoldResumeAdmissionKind::NONE)
    {
        return false;
    }
    if (m_holdResumeAdmissionKind == kind)
    {
        return true;
    }
    if (m_holdResumeAdmissionKind != HoldResumeAdmissionKind::NONE)
    {
        return false;
    }

    AlarmManager& alarms = AlarmManager::GetInstance();
    const std::uint32_t updateBefore = alarms.GetUpdateCount();
    const std::uint64_t intentBefore =
        AlarmManager::MotionAdmissionBaseState(
            alarms.GetMotionSafetyIntentState());
    const bool alarmPresent = alarms.HasAlarm();
    const std::uint32_t updateAfter = alarms.GetUpdateCount();
    const std::uint64_t intentAfter =
        AlarmManager::MotionAdmissionBaseState(
            alarms.GetMotionSafetyIntentState());
    if (alarmPresent ||
        updateBefore != updateAfter ||
        intentBefore != intentAfter)
    {
        return false;
    }

    m_holdResumeAlarmUpdateCount = updateBefore;
    m_holdResumeAlarmSafetyIntentState = intentBefore;
    m_holdResumeAdmissionKind = kind;
    return true;
}

void NCManager::ClearHoldResumeAlarmAdmission(
    HoldResumeAdmissionKind kind) noexcept
{
    if (kind != HoldResumeAdmissionKind::NONE &&
        m_holdResumeAdmissionKind != kind)
    {
        return;
    }
    m_holdResumeAdmissionKind = HoldResumeAdmissionKind::NONE;
    m_holdResumeAlarmUpdateCount = 0U;
    m_holdResumeAlarmSafetyIntentState = 0ULL;
    m_holdResumeGateControlled = false;
}

bool NCManager::ApplyControlledSingleBlockResume() noexcept
{
    if (m_state != NCState::HOLD ||
        !m_singleBlockHoldGate.IsHoldApplied() ||
        m_holdResumeAdmissionKind !=
        HoldResumeAdmissionKind::CONTROLLED_SINGLE_BLOCK)
    {
        return false;
    }

    AlarmManager& alarms = AlarmManager::GetInstance();
    AlarmManager::MotionAdmissionReservation admission{};
    const AlarmManager::MotionAdmissionResult beginResult =
        alarms.TryBeginMotionAdmission(
            m_holdResumeAlarmUpdateCount,
            m_holdResumeAlarmSafetyIntentState,
            admission,
            true);
    if (beginResult == AlarmManager::MotionAdmissionResult::BUSY)
    {
        return false;
    }
    if (beginResult != AlarmManager::MotionAdmissionResult::ACQUIRED)
    {
        ClearHoldResumeAlarmAdmission(
            HoldResumeAdmissionKind::CONTROLLED_SINGLE_BLOCK);
        NCState heldState = NCState::HOLD;
        (void)m_state.compare_exchange_strong(
            heldState,
            NCState::ALARM,
            std::memory_order_acq_rel,
            std::memory_order_acquire);
        m_motion.RequestEmergencyStopAllAxes();
        return false;
    }

    if (m_state != NCState::HOLD ||
        !m_singleBlockHoldGate.IsHoldApplied())
    {
        const bool ended = alarms.EndMotionAdmission(admission);
        ClearHoldResumeAlarmAdmission(
            HoldResumeAdmissionKind::CONTROLLED_SINGLE_BLOCK);
        if (!ended)
        {
            NCState heldState = NCState::HOLD;
            (void)m_state.compare_exchange_strong(
                heldState,
                NCState::ALARM,
                std::memory_order_acq_rel,
                std::memory_order_acquire);
            m_motion.RequestEmergencyStopAllAxes();
        }
        return false;
    }

    if (!AcquireProgramMotionOwner())
    {
        if (!alarms.EndMotionAdmission(admission))
        {
            ClearHoldResumeAlarmAdmission(
                HoldResumeAdmissionKind::CONTROLLED_SINGLE_BLOCK);
            NCState heldState = NCState::HOLD;
            (void)m_state.compare_exchange_strong(
                heldState,
                NCState::ALARM,
                std::memory_order_acq_rel,
                std::memory_order_acquire);
            m_motion.RequestEmergencyStopAllAxes();
        }
        return false;
    }

    const bool leaseCurrentBeforeCommit =
        m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease);
    NCState heldState = NCState::HOLD;
    const bool committedRun =
        leaseCurrentBeforeCommit &&
        m_state.compare_exchange_strong(
            heldState,
            NCState::RUN,
            std::memory_order_acq_rel,
            std::memory_order_acquire);
    if (!committedRun ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease))
    {
        if (committedRun)
        {
            NCState provisionalRun = NCState::RUN;
            (void)m_state.compare_exchange_strong(
                provisionalRun,
                NCState::HOLD,
                std::memory_order_acq_rel,
                std::memory_order_acquire);
        }
        const bool ended = alarms.EndMotionAdmission(admission);
        if (!ended)
        {
            NCState restoredHold = NCState::HOLD;
            (void)m_state.compare_exchange_strong(
                restoredHold,
                NCState::ALARM,
                std::memory_order_acq_rel,
                std::memory_order_acquire);
            m_motion.RequestEmergencyStopAllAxes();
        }
        return false;
    }
    m_motion.SetGroupFeedrateOverride(1.0);
    if (!alarms.EndMotionAdmission(admission))
    {
        NCState provisionalRun = NCState::RUN;
        (void)m_state.compare_exchange_strong(
            provisionalRun,
            NCState::ALARM,
            std::memory_order_acq_rel,
            std::memory_order_acquire);
        if (m_state != NCState::RUN)
        {
            m_motion.SetGroupFeedrateOverride(0.0);
        }
        ClearHoldResumeAlarmAdmission(
            HoldResumeAdmissionKind::CONTROLLED_SINGLE_BLOCK);
        m_motion.RequestEmergencyStopAllAxes();
        return false;
    }

    m_singleBlockBoundaryShadow.ObserveControlledResume();
    m_singleBlockHoldGate.MarkResumeApplied();
    m_pauseAfterBlock = false;
    m_legacySingleBlockPausePending = false;
    ClearHoldResumeAlarmAdmission(
        HoldResumeAdmissionKind::CONTROLLED_SINGLE_BLOCK);
    ResumePathCoreLiveRetentionSameThread();
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
    ClearHoldResumeAlarmAdmission(
        HoldResumeAdmissionKind::CONTROLLED_SINGLE_BLOCK);
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
    ClearHoldResumeAlarmAdmission(
        HoldResumeAdmissionKind::PROGRAM_HOLD);
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

    // NC-0.2L.2T_M00_FIX1: establish ownership from the request source.
    // A failed current PROGRAM request still owns HOLD and must fail closed.
    m_programFeedHoldLifetimeActive =
        source == NCFeedHoldSource::PROGRAM;

    BeginOrdinaryG00FeedHoldCohortShadow();
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
        ObserveOrdinaryG00FeedHoldCohortBoundary();
        return;
    }

    // Keep a pending Gate synchronized with a terminal FAILED/CANCELLED/
    // RESUMED Boundary snapshot even after the observer itself became inactive.
    m_feedHoldResumeGate.ObserveBoundary(
        m_feedHoldBoundaryShadow.GetSnapshot());
    ObserveOrdinaryG00FeedHoldCohortBoundary();
}

void NCManager::ObserveFeedHoldLegacyHoldShadow() noexcept
{
    m_feedHoldBoundaryShadow.ObserveLegacyHoldEntered(
        BuildFeedHoldBoundarySample());
    ObserveOrdinaryG00FeedHoldCohortBoundary();
}

void NCManager::ObserveFeedHoldResumeRequestedShadow() noexcept
{
    m_feedHoldBoundaryShadow.ObserveResumeRequested(
        BuildFeedHoldBoundarySample());
    ObserveOrdinaryG00FeedHoldCohortBoundary();
}

void NCManager::ObserveFeedHoldResumeAppliedShadow() noexcept
{
    // NC-0.2L.2T_M00_FIX1: callers notify only after Resume commits.
    // End current ownership even if terminal diagnostics stay retained.
    m_programFeedHoldLifetimeActive = false;
    m_feedHoldBoundaryShadow.ObserveResumeApplied(
        BuildFeedHoldBoundarySample());
    ObserveOrdinaryG00FeedHoldCohortBoundary();
}

void NCManager::CancelFeedHoldBoundaryShadow(
    bool superseded) noexcept
{
    // NC-0.2L.2T_M00_FIX1: retire control ownership independently of
    // retained terminal diagnostics; observer Cancel may be a no-op.
    m_programFeedHoldLifetimeActive = false;
    m_feedHoldBoundaryShadow.Cancel(superseded);
    m_feedHoldResumeGate.Cancel(superseded);
    m_ordinaryG00FeedHoldCohortShadow.Cancel(superseded);
    ObserveOrdinaryG00FeedHoldCohortCutover();
    ClearHoldResumeAlarmAdmission(
        HoldResumeAdmissionKind::PROGRAM_HOLD);
}

void NCManager::BeginOrdinaryG00FeedHoldCohortShadow() noexcept
{
    NCOrdinaryG00FeedHoldCohortShadow::CandidateArray candidates{};
    std::size_t candidateCount = 0U;
    for (std::size_t slot = 0U;
        slot < NC_ORDINARY_G00_INFLIGHT_REGISTRY_CAPACITY;
        ++slot)
    {
        NCOrdinaryG00InflightEntrySnapshot entry{};
        if (!m_ordinaryG00InflightRegistryShadow.TryGetEntry(slot, entry) ||
            !entry.active)
        {
            continue;
        }
        if (candidateCount < candidates.size())
        {
            candidates[candidateCount] = entry;
        }
        ++candidateCount;
    }

    // The Registry snapshot remains the authoritative active-count proof.
    // A scan/count disagreement therefore enters K.7.3's diagnostic failure
    // path without changing any existing Runtime state.
    m_ordinaryG00FeedHoldCohortShadow.Begin(
        m_feedHoldBoundaryShadow.GetSnapshot(),
        m_ordinaryG00InflightRegistryShadow.GetSnapshot(),
        candidates);
    ObserveOrdinaryG00FeedHoldCohortCutover();
}

void NCManager::ObserveOrdinaryG00FeedHoldCohortBoundary() noexcept
{
    m_ordinaryG00FeedHoldCohortShadow.ObserveBoundary(
        m_feedHoldBoundaryShadow.GetSnapshot(),
        m_feedHoldResumeGate.GetSnapshot(),
        m_ordinaryG00InflightRegistryShadow.GetSnapshot());
    ObserveOrdinaryG00FeedHoldCohortCutover();
}

void NCManager::ObserveOrdinaryG00FeedHoldCohortCutover() noexcept
{
    m_ordinaryG00FeedHoldCohortCutoverGate.Observe(
        m_ordinaryG00FeedHoldCohortShadow.GetSnapshot(),
        m_ordinaryG00InflightRegistryShadow.GetSnapshot());

    // NC-0.2K.7.5 is a diagnostic-only consumer of the already-published
    // K.7.3/K.7.4 evidence.  Its result never enters admission or Motion.
    m_ordinaryG00FeedHoldCohortRearmShadow.Observe(
        m_ordinaryG00FeedHoldCohortShadow.GetSnapshot(),
        m_ordinaryG00FeedHoldCohortCutoverGate.GetSnapshot(),
        m_ordinaryG00InflightRegistryShadow.GetSnapshot());

    // NC-0.2K.7.6 is the reversible controlled consumer of K.7.5.  It may
    // influence only the ordinary G00 admission seam above; it never writes
    // Motion, PDO, NC state, PC, callback, Epoch or owner state.
    m_ordinaryG00FeedHoldCohortRearmCutoverGate.Observe(
        m_ordinaryG00FeedHoldCohortRearmShadow.GetSnapshot(),
        m_ordinaryG00FeedHoldCohortCutoverGate.GetSnapshot(),
        m_ordinaryG00InflightRegistryShadow.GetSnapshot());

    // NC-0.2K.7.7 observes the already-published K.7.5/K.7.6 seed and the
    // current K.7.3/K.7.4/Registry evidence.  It is shadow-only and cannot
    // influence admission or write Motion, PDO, state, PC, Epoch or owner.
    m_ordinaryG00FeedHoldRollingRearmShadow.Observe(
        m_ordinaryG00FeedHoldCohortRearmShadow.GetSnapshot(),
        m_ordinaryG00FeedHoldCohortRearmCutoverGate.GetSnapshot(),
        m_ordinaryG00FeedHoldCohortShadow.GetSnapshot(),
        m_ordinaryG00FeedHoldCohortCutoverGate.GetSnapshot(),
        m_ordinaryG00InflightRegistryShadow.GetSnapshot());

    // NC-0.2K.7.8 is the reversible controlled consumer of K.7.7.  Only its
    // admission result is used by the ordinary G00 seam; it owns no Motion
    // storage and cannot write Motion, PDO, NC state, PC, Epoch or owner.
    m_ordinaryG00FeedHoldRollingRearmCutoverGate.Observe(
        m_ordinaryG00FeedHoldRollingRearmShadow.GetSnapshot(),
        m_ordinaryG00FeedHoldCohortCutoverGate.GetSnapshot(),
        m_ordinaryG00InflightRegistryShadow.GetSnapshot());
}

// =============================================================================
// Stage NC-0.2I.3 - Program Feed Hold ACK-Gated Resume Controlled Cutover
// =============================================================================
bool NCManager::IsProgramFeedHoldResumeCandidate() const noexcept
{
    const NCFeedHoldBoundarySnapshot snapshot =
        m_feedHoldBoundaryShadow.GetSnapshot();

    return
        m_programFeedHoldLifetimeActive &&
        snapshot.sequence != 0ULL &&
        snapshot.source == NCFeedHoldSource::PROGRAM &&
        snapshot.requestLatched &&
        !snapshot.resumeApplied &&
        !snapshot.cancelled;
}

bool NCManager::ApplyProgramHoldResume(
    bool gateControlled) noexcept
{
    if (m_state != NCState::HOLD ||
        m_holdResumeAdmissionKind !=
        HoldResumeAdmissionKind::PROGRAM_HOLD)
    {
        return false;
    }

    AlarmManager& alarms = AlarmManager::GetInstance();
    AlarmManager::MotionAdmissionReservation admission{};
    const AlarmManager::MotionAdmissionResult beginResult =
        alarms.TryBeginMotionAdmission(
            m_holdResumeAlarmUpdateCount,
            m_holdResumeAlarmSafetyIntentState,
            admission,
            true);
    if (beginResult == AlarmManager::MotionAdmissionResult::BUSY)
    {
        return false;
    }
    if (beginResult != AlarmManager::MotionAdmissionResult::ACQUIRED)
    {
        InvalidatePathCoreHoldSameThread();
        ClearHoldResumeAlarmAdmission(
            HoldResumeAdmissionKind::PROGRAM_HOLD);
        NCState heldState = NCState::HOLD;
        (void)m_state.compare_exchange_strong(
            heldState,
            NCState::ALARM,
            std::memory_order_acq_rel,
            std::memory_order_acquire);
        m_motion.RequestEmergencyStopAllAxes();
        return false;
    }

    if (m_state != NCState::HOLD ||
        (gateControlled &&
            !m_feedHoldResumeGate.ShouldApplyResume()))
    {
        const bool ended = alarms.EndMotionAdmission(admission);
        ClearHoldResumeAlarmAdmission(
            HoldResumeAdmissionKind::PROGRAM_HOLD);
        if (!ended)
        {
            InvalidatePathCoreHoldSameThread();
            NCState heldState = NCState::HOLD;
            (void)m_state.compare_exchange_strong(
                heldState,
                NCState::ALARM,
                std::memory_order_acq_rel,
                std::memory_order_acquire);
            m_motion.RequestEmergencyStopAllAxes();
        }
        return false;
    }

    if (!AcquireProgramMotionOwner())
    {
        // The Gate remains RELEASE_READY and will retry from ProcessTask after
        // the current Owner arbitration becomes valid.  No state or Override
        // is changed on this failed attempt.
        if (!alarms.EndMotionAdmission(admission))
        {
            InvalidatePathCoreHoldSameThread();
            ClearHoldResumeAlarmAdmission(
                HoldResumeAdmissionKind::PROGRAM_HOLD);
            NCState heldState = NCState::HOLD;
            (void)m_state.compare_exchange_strong(
                heldState,
                NCState::ALARM,
                std::memory_order_acq_rel,
                std::memory_order_acquire);
            m_motion.RequestEmergencyStopAllAxes();
        }
        return false;
    }

    const MotionNCSettleRequestSequence resumeHoldRequest = m_feedHoldNCSettleRequestSequence;
    // CB: a refused/deferred excursion leaves the exact HOLD and Override=0.
    // CB keeps Override=0 until successful End and explicit RT start commit.
    if (!PreparePathCoreHoldResumeSameThread(gateControlled))
    {
        if (!alarms.EndMotionAdmission(admission))
        {
            InvalidatePathCoreHoldSameThread();
            ClearHoldResumeAlarmAdmission(HoldResumeAdmissionKind::PROGRAM_HOLD);
            NCState heldState = NCState::HOLD;
            (void)m_state.compare_exchange_strong(heldState, NCState::ALARM,
                std::memory_order_acq_rel, std::memory_order_acquire);
            m_motion.RequestEmergencyStopAllAxes();
        }
        return false;
    }

    const bool pathHoldResume = m_pathHold.bound;
    const bool leaseCurrentBeforeCommit =
        m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease);
    NCState heldState = NCState::HOLD;
    const bool committedRun =
        leaseCurrentBeforeCommit &&
        m_state.compare_exchange_strong(
            heldState,
            NCState::RUN,
            std::memory_order_acq_rel,
            std::memory_order_acquire);
    if (!committedRun ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease))
    {
        InvalidatePathCoreHoldSameThread();
        if (committedRun)
        {
            NCState provisionalRun = NCState::RUN;
            (void)m_state.compare_exchange_strong(
                provisionalRun,
                NCState::HOLD,
                std::memory_order_acq_rel,
                std::memory_order_acquire);
        }
        const bool ended = alarms.EndMotionAdmission(admission);
        if (!ended)
        {
            InvalidatePathCoreHoldSameThread();
            NCState restoredHold = NCState::HOLD;
            (void)m_state.compare_exchange_strong(
                restoredHold,
                NCState::ALARM,
                std::memory_order_acq_rel,
                std::memory_order_acquire);
            m_motion.RequestEmergencyStopAllAxes();
        }
        return false;
    }
    if (m_pathHold.bound && (m_state != NCState::RUN ||
        m_feedHoldNCSettleRequestSequence != resumeHoldRequest ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease)))
    {
        InvalidatePathCoreHoldSameThread();
        NCState provisionalRun = NCState::RUN;
        (void)m_state.compare_exchange_strong(provisionalRun, NCState::HOLD,
            std::memory_order_acq_rel, std::memory_order_acquire);
        m_motion.SetGroupFeedrateOverride(0.0);
        if (!alarms.EndMotionAdmission(admission))
        {
            InvalidatePathCoreHoldSameThread();
            NCState failedAdmissionHold = NCState::HOLD;
            (void)m_state.compare_exchange_strong(failedAdmissionHold, NCState::ALARM,
                std::memory_order_acq_rel, std::memory_order_acquire);
            m_motion.RequestEmergencyStopAllAxes();
        }
        ClearHoldResumeAlarmAdmission(HoldResumeAdmissionKind::PROGRAM_HOLD);
        return false;
    }
    if (!pathHoldResume) m_motion.SetGroupFeedrateOverride(1.0);
    if (!alarms.EndMotionAdmission(admission))
    {
        InvalidatePathCoreHoldSameThread();
        NCState provisionalRun = NCState::RUN;
        (void)m_state.compare_exchange_strong(
            provisionalRun,
            NCState::ALARM,
            std::memory_order_acq_rel,
            std::memory_order_acquire);
        m_motion.SetGroupFeedrateOverride(0.0);
        ClearHoldResumeAlarmAdmission(
            HoldResumeAdmissionKind::PROGRAM_HOLD);
        m_motion.RequestEmergencyStopAllAxes();
        return false;
    }

    // CB, including repeated Hold within an excursion: End succeeds before
    // Commit and Override=1. Keep the J.5 zero-override proof intact until then.
    if (pathHoldResume && (!m_pathHold.bound || m_state != NCState::RUN ||
        m_feedHoldNCSettleRequestSequence != resumeHoldRequest ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease) ||
        m_motion.HasPendingSafetyOrRecoveryRequests() || AlarmManager::GetInstance().HasAlarm() ||
        !CommitPathCoreHoldResumeSameThread()))
    {
        m_pathHold.code = 13U;
        LogPathCoreHoldSameThread("COMMIT_BLOCKED");
        m_motion.CancelPathCoreHoldExcursion();
        m_pathHold.blocked = true;
        NCState provisionalRun = NCState::RUN;
        (void)m_state.compare_exchange_strong(provisionalRun, NCState::HOLD,
            std::memory_order_acq_rel, std::memory_order_acquire);
        m_motion.SetGroupFeedrateOverride(0.0);
        return false;
    }

    if (pathHoldResume)
    {
        m_motion.SetGroupFeedrateOverride(1.0);
        // Controls are serviced on the NC thread; these final checks also
        // revoke a concurrently superseded owner/safety/hold publication.
        if (m_state != NCState::RUN || m_feedHoldNCSettleRequestSequence != resumeHoldRequest ||
            !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease) ||
            m_motion.HasPendingSafetyOrRecoveryRequests() || AlarmManager::GetInstance().HasAlarm())
        {
            m_motion.SetGroupFeedrateOverride(0.0);
            m_motion.CancelPathCoreHoldExcursion();
            m_pathHold.blocked = true;
            NCState provisionalRun = NCState::RUN;
            (void)m_state.compare_exchange_strong(provisionalRun, NCState::HOLD,
                std::memory_order_acq_rel, std::memory_order_acquire);
            return false;
        }
    }

    // A Cycle Start is considered applied only after the admission End CAS.
    m_singleBlockBoundaryShadow.ObserveLegacyResume();
    m_legacySingleBlockPausePending = false;
    m_pauseAfterBlock = false;
    ObserveFeedHoldResumeAppliedShadow();
    if (gateControlled)
    {
        m_feedHoldResumeGate.MarkResumeApplied(
            m_feedHoldBoundaryShadow.GetSnapshot());
    }
    ClearHoldResumeAlarmAdmission(
        HoldResumeAdmissionKind::PROGRAM_HOLD);
    ResumePathCoreLiveRetentionSameThread();
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

    (void)ApplyProgramHoldResume(true);
    // A BUSY reservation retains the exact resume ticket and must consume
    // this NC scan; otherwise normal dispatch could run past the HOLD gate.
    return true;
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
    if (nc->m_motion.GetQueueSize() > 0 ||
        !nc->m_motion.IsGroupNCDrained())
    {
        return false;
    }

    // A GOTO publishes its replacement Epoch before arming this callback.
    // Wait for the 250 us consumer acknowledgement (and for any other
    // safety/recovery request) before sampling RT-owned logicalCmdPos.
    if (nc->m_motion.HasPendingSafetyOrRecoveryRequests())
    {
        return false;
    }

    // Macro call/return/repeat also sets m_programChanged, but does not publish
    // a replacement Epoch.  Only the dedicated GOTO identity owns this
    // lifecycle rebase.
    if (!nc->m_gotoQueueTailRebasePending)
    {
        return true;
    }

    const auto failGotoRebaseClosed = [nc]() -> bool
    {
        const int sourceLineNumber =
            nc->m_pendingGotoQueueTailRebaseSourceLine;
        nc->ClearPendingGotoQueueTailRebase();
        TriggerMappingIntegrityAlarmOnce(sourceLineNumber);
        nc->m_state = NCState::ALARM;
        return true;
    };

    // A later Reset/Safety/owner transition must cancel, never retarget, the
    // deferred GOTO rebase to whatever identity happens to be current.
    if (!nc->IsPendingGotoQueueTailRebaseIdentityCurrent())
    {
        return failGotoRebaseClosed();
    }

    if (!nc->m_motion.HasExactExecutionDrainAcknowledgement(
        nc->m_pendingGotoQueueTailRebaseExecutionEpoch,
        nc->m_pendingGotoQueueTailRebaseOwnerLease))
    {
        return false;
    }

    nc->m_motion.SyncVirtualEndPosition();

    double synchronizedQueueTailMCS[MAX_AXES] = {};
    if (!nc->m_motion.TryGetSynchronizedG00QueueTailMCS(
        synchronizedQueueTailMCS) ||
        !nc->IsPendingGotoQueueTailRebaseIdentityCurrent() ||
        !nc->m_motion.HasExactExecutionDrainAcknowledgement(
            nc->m_pendingGotoQueueTailRebaseExecutionEpoch,
            nc->m_pendingGotoQueueTailRebaseOwnerLease))
    {
        return failGotoRebaseClosed();
    }

    // Only this exact drain/acknowledgement identity may rebase the producer
    // tail and NC MCS endpoint.  The next G90/G91 preview therefore starts
    // from one coherent post-GOTO baseline.
    nc->CoordSys.SyncMachinePosition(
        synchronizedQueueTailMCS);

    if (!nc->IsPendingGotoQueueTailRebaseIdentityCurrent() ||
        !nc->m_motion.HasExactExecutionDrainAcknowledgement(
            nc->m_pendingGotoQueueTailRebaseExecutionEpoch,
            nc->m_pendingGotoQueueTailRebaseOwnerLease))
    {
        return failGotoRebaseClosed();
    }

    nc->ClearPendingGotoQueueTailRebase();
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
