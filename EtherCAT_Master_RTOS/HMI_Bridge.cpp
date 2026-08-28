#include "HMI_Bridge.h"
#include "SHMManager.h"
#include "GlobalConfig.h" 
#include "NCManager.h"
#include "AlarmManager.h"
#include <cstring> 
#include "PLCManager.h"
#include "NCPLCMap.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <rtapi.h>

namespace HMI_Bridge
{
    namespace
    {
        const char* NCBlockLifecycleStateToDiagnosticName(
            NCBlockLifecycleState state) noexcept
        {
            switch (state)
            {
            case NCBlockLifecycleState::DISPATCHED: return "DISPATCHED";
            case NCBlockLifecycleState::PROGRAM_COMMITTED: return "PROGRAM_COMMIT";
            case NCBlockLifecycleState::PROGRAM_ONLY_COMPLETED: return "PROGRAM_ONLY_DONE";
            case NCBlockLifecycleState::MOTION_PENDING: return "MOTION_PENDING";
            case NCBlockLifecycleState::MOTION_ACCEPTED: return "MOTION_ACCEPTED";
            case NCBlockLifecycleState::MOTION_ACTIVE: return "MOTION_ACTIVE";
            case NCBlockLifecycleState::MOTION_HELD: return "MOTION_HELD";
            case NCBlockLifecycleState::MOTION_COMPLETED: return "MOTION_DONE";
            case NCBlockLifecycleState::MOTION_REJECTED: return "MOTION_REJECTED";
            case NCBlockLifecycleState::MOTION_CANCELLED: return "MOTION_CANCELLED";
            case NCBlockLifecycleState::MOTION_ABORTED: return "MOTION_ABORTED";
            case NCBlockLifecycleState::MOTION_FAULTED: return "MOTION_FAULTED";
            case NCBlockLifecycleState::NC_DISPATCH_FAILED: return "NC_DISPATCH_FAILED";
            case NCBlockLifecycleState::TRACKING_OVERFLOW: return "TRACKING_OVERFLOW";
            case NCBlockLifecycleState::NONE:
            default: return "NONE";
            }
        }

        const char* NCBlockMotionBoundaryStateToDiagnosticName(
            NCBlockMotionBoundaryState state) noexcept
        {
            switch (state)
            {
            case NCBlockMotionBoundaryState::NOT_TRACKED: return "NOT_TRACKED";
            case NCBlockMotionBoundaryState::PENDING: return "PENDING";
            case NCBlockMotionBoundaryState::SUCCEEDED: return "SUCCEEDED";
            case NCBlockMotionBoundaryState::FAILED: return "FAILED";
            case NCBlockMotionBoundaryState::TRACKING_OVERFLOW: return "OVERFLOW";
            case NCBlockMotionBoundaryState::NONE:
            default: return "NONE";
            }
        }

        const char* NCBlockWaitKindToDiagnosticName(
            NCBlockWaitKind kind) noexcept
        {
            switch (kind)
            {
            case NCBlockWaitKind::MOTION_HANDLER: return "MOTION_CB";
            case NCBlockWaitKind::MOTION_QUEUE_DRAIN: return "MOTION_DRAIN";
            case NCBlockWaitKind::AUXILIARY_CALLBACK: return "AUX_CB";
            case NCBlockWaitKind::PROGRAM_FLOW_DRAIN: return "FLOW_DRAIN";
            case NCBlockWaitKind::NONE:
            default: return "NONE";
            }
        }

        const char* NCBlockCompletionComparisonToDiagnosticName(
            NCBlockCompletionComparison comparison) noexcept
        {
            switch (comparison)
            {
            case NCBlockCompletionComparison::AGREE_WAITING: return "AGREE_WAIT";
            case NCBlockCompletionComparison::AGREE_READY: return "AGREE_READY";
            case NCBlockCompletionComparison::LEDGER_READY_LEGACY_WAITING: return "LEDGER_EARLY";
            case NCBlockCompletionComparison::LEGACY_READY_LEDGER_PENDING: return "LEGACY_EARLY";
            case NCBlockCompletionComparison::LEDGER_FAILED_LEGACY_WAITING: return "LEDGER_FAIL_WAIT";
            case NCBlockCompletionComparison::LEGACY_READY_LEDGER_FAILED: return "RELEASE_ON_FAIL";
            case NCBlockCompletionComparison::NOT_MOTION_TRACKED: return "NOT_TRACKED";
            case NCBlockCompletionComparison::TRACKING_OVERFLOW: return "OVERFLOW";
            case NCBlockCompletionComparison::MISSING_LIFECYCLE: return "MISSING";
            case NCBlockCompletionComparison::NONE:
            default: return "NONE";
            }
        }

        const char* NCBlockCompletionGateDecisionToDiagnosticName(
            NCBlockCompletionGateDecision decision) noexcept
        {
            switch (decision)
            {
            case NCBlockCompletionGateDecision::LEGACY_BYPASS_WAIT: return "BYPASS_WAIT";
            case NCBlockCompletionGateDecision::LEGACY_BYPASS_READY: return "BYPASS_READY";
            case NCBlockCompletionGateDecision::WAIT_BOTH: return "WAIT_BOTH";
            case NCBlockCompletionGateDecision::WAIT_LEGACY: return "WAIT_LEGACY";
            case NCBlockCompletionGateDecision::BLOCK_LEDGER_PENDING: return "BLOCK_PENDING";
            case NCBlockCompletionGateDecision::RELEASE_DUAL_KEY: return "RELEASE_DUAL";
            case NCBlockCompletionGateDecision::BLOCK_LEDGER_FAILED: return "BLOCK_FAILED";
            case NCBlockCompletionGateDecision::BLOCK_TRACKING_OVERFLOW: return "BLOCK_OVERFLOW";
            case NCBlockCompletionGateDecision::BLOCK_MISSING_LIFECYCLE: return "BLOCK_MISSING";
            case NCBlockCompletionGateDecision::BLOCK_NOT_TRACKED: return "BLOCK_NOT_TRACKED";
            case NCBlockCompletionGateDecision::NONE:
            default: return "NONE";
            }
        }

        const char* NCProgramEndCauseToDiagnosticName(
            NCProgramEndCause cause) noexcept
        {
            switch (cause)
            {
            case NCProgramEndCause::NATURAL_EOF: return "EOF";
            case NCProgramEndCause::M02: return "M02";
            case NCProgramEndCause::M30: return "M30";
            case NCProgramEndCause::NONE:
            default: return "NONE";
            }
        }

        const char* NCProgramEndPhaseToDiagnosticName(
            NCProgramEndPhase phase) noexcept
        {
            switch (phase)
            {
            case NCProgramEndPhase::RUN_ACTIVE: return "RUN_ACTIVE";
            case NCProgramEndPhase::DRAINING: return "DRAINING";
            case NCProgramEndPhase::READY_TO_FINALIZE: return "READY";
            case NCProgramEndPhase::FINALIZED: return "FINALIZED";
            case NCProgramEndPhase::CANCELLED: return "CANCELLED";
            case NCProgramEndPhase::FAIL_CLOSED: return "FAIL_CLOSED";
            case NCProgramEndPhase::START_BLOCKED: return "START_BLOCKED";
            case NCProgramEndPhase::IDLE:
            default: return "IDLE";
            }
        }

        const char* NCProgramEndDecisionToDiagnosticName(
            NCProgramEndDecision decision) noexcept
        {
            switch (decision)
            {
            case NCProgramEndDecision::RUN_STARTED: return "RUN_STARTED";
            case NCProgramEndDecision::RUN_START_BLOCKED_DIRTY: return "START_DIRTY";
            case NCProgramEndDecision::END_REQUESTED: return "END_REQUESTED";
            case NCProgramEndDecision::WAIT_ACTIVE_BLOCKS: return "WAIT_BLOCKS";
            case NCProgramEndDecision::WAIT_AXIS_COMMAND: return "WAIT_AXISQ";
            case NCProgramEndDecision::WAIT_AXIS_RESULT: return "WAIT_AXIS_RESULT";
            case NCProgramEndDecision::WAIT_COMMAND_INGRESS: return "WAIT_INGRESS";
            case NCProgramEndDecision::WAIT_COMMAND_REPLAY: return "WAIT_REPLAY";
            case NCProgramEndDecision::WAIT_COMMAND_QUEUE: return "WAIT_CMDQ";
            case NCProgramEndDecision::WAIT_FEEDBACK_NOTICE: return "WAIT_NOTICE";
            case NCProgramEndDecision::WAIT_FEEDBACK: return "WAIT_FEEDBACK";
            case NCProgramEndDecision::WAIT_FEEDBACK_SEQUENCE: return "WAIT_FB_SEQ";
            case NCProgramEndDecision::WAIT_CALLBACK: return "WAIT_CALLBACK";
            case NCProgramEndDecision::WAIT_COMPLETION_BINDING: return "WAIT_BINDING";
            case NCProgramEndDecision::WAIT_SAFETY_REQUEST: return "WAIT_SAFETY";
            case NCProgramEndDecision::WAIT_GROUP_STANDSTILL: return "WAIT_STANDSTILL";
            case NCProgramEndDecision::WAIT_STABLE_CONFIRMATION: return "WAIT_STABLE";
            case NCProgramEndDecision::READY_TO_FINALIZE: return "READY_FINALIZE";
            case NCProgramEndDecision::FAIL_EXECUTION_EPOCH_CHANGED: return "FAIL_EPOCH";
            case NCProgramEndDecision::FAIL_OWNER_LEASE_LOST: return "FAIL_OWNER";
            case NCProgramEndDecision::FAIL_INTEGRITY_COUNTER_ADVANCED: return "FAIL_INTEGRITY";
            case NCProgramEndDecision::FINALIZED: return "FINALIZED";
            case NCProgramEndDecision::CANCELLED: return "CANCELLED";
            case NCProgramEndDecision::NONE:
            default: return "NONE";
            }
        }

        const char* NCGMBlockPostActionToDiagnosticName(
            NCGMBlockPostAction action) noexcept
        {
            switch (action)
            {
            case NCGMBlockPostAction::PROGRAM_STOP_M00: return "M00_STOP";
            case NCGMBlockPostAction::OPTIONAL_STOP_M01: return "M01_STOP";
            case NCGMBlockPostAction::CALL_M98: return "M98_CALL";
            case NCGMBlockPostAction::RETURN_M99: return "M99_RETURN";
            case NCGMBlockPostAction::PROGRAM_END_M02: return "M02_END";
            case NCGMBlockPostAction::PROGRAM_END_M30: return "M30_END";
            case NCGMBlockPostAction::NONE:
            default: return "NONE";
            }
        }

        const char* NCGMBlockTransactionPhaseToDiagnosticName(
            NCGMBlockTransactionPhase phase) noexcept
        {
            switch (phase)
            {
            case NCGMBlockTransactionPhase::WAITING: return "WAITING";
            case NCGMBlockTransactionPhase::READY_TO_FINALIZE: return "READY";
            case NCGMBlockTransactionPhase::FINALIZED: return "FINALIZED";
            case NCGMBlockTransactionPhase::CANCELLED: return "CANCELLED";
            case NCGMBlockTransactionPhase::FAILED: return "FAILED";
            case NCGMBlockTransactionPhase::IDLE:
            default: return "IDLE";
            }
        }

        const char* NCSingleBlockCandidateKindToDiagnosticName(
            NCSingleBlockCandidateKind kind) noexcept
        {
            switch (kind)
            {
            case NCSingleBlockCandidateKind::PROGRAM_CONTROL: return "CONTROL";
            case NCSingleBlockCandidateKind::G_M_BLOCK: return "G_M";
            case NCSingleBlockCandidateKind::ADDRESS_BLOCK: return "ADDRESS";
            case NCSingleBlockCandidateKind::NONE:
            default: return "NONE";
            }
        }

        const char* NCSingleBlockShadowPhaseToDiagnosticName(
            NCSingleBlockShadowPhase phase) noexcept
        {
            switch (phase)
            {
            case NCSingleBlockShadowPhase::ARMED: return "ARMED";
            case NCSingleBlockShadowPhase::WAITING_BOUNDARY: return "WAITING";
            case NCSingleBlockShadowPhase::BOUNDARY_READY: return "READY";
            case NCSingleBlockShadowPhase::LEGACY_HOLD_CONFIRMED: return "HOLD_OK";
            case NCSingleBlockShadowPhase::LEGACY_HOLD_MISMATCH: return "HOLD_MISMATCH";
            case NCSingleBlockShadowPhase::PROGRAM_END_SUPPRESSED: return "END_SUPPRESSED";
            case NCSingleBlockShadowPhase::CANCELLED: return "CANCELLED";
            case NCSingleBlockShadowPhase::IDLE:
            default: return "IDLE";
            }
        }

        const char* NCSingleBlockShadowDecisionToDiagnosticName(
            NCSingleBlockShadowDecision decision) noexcept
        {
            switch (decision)
            {
            case NCSingleBlockShadowDecision::NOT_ELIGIBLE: return "NOT_ELIGIBLE";
            case NCSingleBlockShadowDecision::WAIT_LIFECYCLE: return "WAIT_LIFECYCLE";
            case NCSingleBlockShadowDecision::WAIT_PROGRAM_COMMIT: return "WAIT_COMMIT";
            case NCSingleBlockShadowDecision::WAIT_TRANSACTION: return "WAIT_TXN";
            case NCSingleBlockShadowDecision::WAIT_CALLBACK: return "WAIT_CALLBACK";
            case NCSingleBlockShadowDecision::WAIT_MOTION: return "WAIT_MOTION";
            case NCSingleBlockShadowDecision::READY_FOR_HOLD: return "READY_HOLD";
            case NCSingleBlockShadowDecision::AGREE_HOLD: return "AGREE_HOLD";
            case NCSingleBlockShadowDecision::LEGACY_EARLY_HOLD: return "LEGACY_EARLY";
            case NCSingleBlockShadowDecision::LEGACY_HOLD_WITHOUT_ARM: return "PHANTOM_HOLD";
            case NCSingleBlockShadowDecision::PROGRAM_END_SUPPRESSED: return "END_SUPPRESSED";
            case NCSingleBlockShadowDecision::MOTION_FAILED: return "MOTION_FAILED";
            case NCSingleBlockShadowDecision::TRACKING_OVERFLOW: return "OVERFLOW";
            case NCSingleBlockShadowDecision::TRANSACTION_FAILED: return "TXN_FAILED";
            case NCSingleBlockShadowDecision::RESUMED: return "RESUMED";
            case NCSingleBlockShadowDecision::CANCELLED: return "CANCELLED";
            case NCSingleBlockShadowDecision::NONE:
            default: return "NONE";
            }
        }

        const char* NCFeedHoldSourceToDiagnosticName(
            NCFeedHoldSource source) noexcept
        {
            switch (source)
            {
            case NCFeedHoldSource::PROGRAM: return "PROGRAM";
            case NCFeedHoldSource::HOME: return "HOME";
            case NCFeedHoldSource::NONE:
            default: return "NONE";
            }
        }

        const char* NCFeedHoldShadowPhaseToDiagnosticName(
            NCFeedHoldShadowPhase phase) noexcept
        {
            switch (phase)
            {
            case NCFeedHoldShadowPhase::REQUESTED: return "REQUESTED";
            case NCFeedHoldShadowPhase::DECELERATING: return "DECEL";
            case NCFeedHoldShadowPhase::STOPPED_UNSTABLE: return "UNSTABLE";
            case NCFeedHoldShadowPhase::ACKNOWLEDGED: return "ACK";
            case NCFeedHoldShadowPhase::RESUME_REQUESTED: return "RESUME_REQ";
            case NCFeedHoldShadowPhase::RESUMED: return "RESUMED";
            case NCFeedHoldShadowPhase::CANCELLED: return "CANCELLED";
            case NCFeedHoldShadowPhase::FAILED: return "FAILED";
            case NCFeedHoldShadowPhase::IDLE:
            default: return "IDLE";
            }
        }

        const char* NCFeedHoldShadowDecisionToDiagnosticName(
            NCFeedHoldShadowDecision decision) noexcept
        {
            switch (decision)
            {
            case NCFeedHoldShadowDecision::REQUEST_LATCHED: return "REQUEST_LATCHED";
            case NCFeedHoldShadowDecision::WAIT_OVERRIDE_ZERO: return "WAIT_OVERRIDE";
            case NCFeedHoldShadowDecision::WAIT_COMMAND_STOP: return "WAIT_COMMAND";
            case NCFeedHoldShadowDecision::WAIT_ACTUAL_STOP: return "WAIT_ACTUAL";
            case NCFeedHoldShadowDecision::WAIT_HOME_PAUSED: return "WAIT_HOME";
            case NCFeedHoldShadowDecision::WAIT_STABLE: return "WAIT_STABLE";
            case NCFeedHoldShadowDecision::ACK_READY: return "ACK_READY";
            case NCFeedHoldShadowDecision::LEGACY_HOLD_EARLY: return "LEGACY_EARLY";
            case NCFeedHoldShadowDecision::LEGACY_HOLD_AGREE: return "LEGACY_AGREE";
            case NCFeedHoldShadowDecision::RESUME_BEFORE_ACK: return "RESUME_EARLY";
            case NCFeedHoldShadowDecision::RESUME_AFTER_ACK: return "RESUME_AFTER_ACK";
            case NCFeedHoldShadowDecision::RESUMED: return "RESUMED";
            case NCFeedHoldShadowDecision::OWNER_CHANGED: return "OWNER_CHANGED";
            case NCFeedHoldShadowDecision::EPOCH_CHANGED: return "EPOCH_CHANGED";
            case NCFeedHoldShadowDecision::MOTION_FAULT: return "MOTION_FAULT";
            case NCFeedHoldShadowDecision::ACK_LOST: return "ACK_LOST";
            case NCFeedHoldShadowDecision::CANCELLED: return "CANCELLED";
            case NCFeedHoldShadowDecision::SUPERSEDED: return "SUPERSEDED";
            case NCFeedHoldShadowDecision::NONE:
            default: return "NONE";
            }
        }

        const char* MotionOwnerToDiagnosticName(
            MotionOwner owner) noexcept
        {
            switch (owner)
            {
            case MotionOwner::NONE:        return "NONE";
            case MotionOwner::AUTO:        return "AUTO";
            case MotionOwner::MDI:         return "MDI";
            case MotionOwner::MANUAL_AUTO: return "MANUAL_AUTO";
            case MotionOwner::JOG:         return "JOG";
            case MotionOwner::MPG:         return "MPG";
            case MotionOwner::HOME:        return "HOME";
            case MotionOwner::EDM_PATH:    return "EDM_PATH";
            case MotionOwner::EDM_RETRACT: return "EDM_RETRACT";
            case MotionOwner::RECOVERY:    return "RECOVERY";
            case MotionOwner::SAFETY:      return "SAFETY";
            default:                       return "UNKNOWN";
            }
        }
    }
    // =========================================================================
    // 🚀 1. ProcessTask (每一圈執行) - 處理高優先級命令與即時座標
    // =========================================================================
    void ProcessTask(NCManager* nc)
    {
        SHM_Data* pShm = SHMManager::GetInstance().GetData();
        if (pShm == nullptr || nc == nullptr) return;

        // 🌟 [新增] 1. 取得當前的公英制倍率 (公制=1.0, 英制=1/25.4)
        double unitScale = nc->CoordSys.isInchMode ? (1.0 / 25.4) : 1.0;


        // --- 狀態迴圈更新 ---
        pShm->API_Status.SHM_API_RunCount = nc->API_RunCount++;
        pShm->NC_Status.SHM_NC_RunCount = nc->NC_RunCount;

        pShm->NC_Status.SHM_m_mode = static_cast<int32_t>(nc->m_mode);

        pShm->NC_Status.SHM_NC_State = static_cast<int32_t>(nc->m_state);
        pShm->NC_Status.SHM_EDM_State = static_cast<int32_t>(nc->m_edmState);


        pShm->NC_Status.m_isSingleBlockEnabled = nc->m_isSingleBlockEnabled;
        pShm->NC_Status.m_isOptionalStopEnabled = nc->m_isOptionalStopEnabled;
        pShm->NC_Status.m_isBlockSkipEnabled = nc->m_isBlockSkipEnabled;


        // --- 即時座標廣播 ---
        double currentWCS[8] = { 0.0 };
        nc->CoordSys.GetActualWCS(currentWCS);

        // 🌟 [神級修復] 2. 將 DTG 的計算提拔到迴圈外部！避免重複運算與變數遮蔽(Shadowing) Bug
        double currentDTG[8] = { 0.0 };
        nc->CoordSys.GetDistanceToGo(currentDTG, nc);

        for (int i = 0; i < 8; i++)
        {
            // 取得軸的 Context
            auto& axis = nc->m_motion.GetAxisContext(i);
            AxisType type = axis.axisType;

            // 🌟 3. 判斷是否為旋轉軸 (旋轉軸永遠是度數 deg，絕對不可以套用 inch 轉換)
            double axisScale = (type == AxisType::ROTARY || type == AxisType::ROTARY_CONTINUOUS) ? 1.0 : unitScale;

            double currentComp = axis.currentCompOffset_unit;

            // 🌟 4. 扣除補償量，並套用公英制倍率
            double displayMCS = (nc->CoordSys.actualMCS[i] - currentComp) * axisScale;
            double displayWCS = (currentWCS[i] - currentComp) * axisScale;

            // 把 DTG 塞進共享記憶體，同樣套用倍率
            pShm->NC_Status.DistanceToGo[i] = currentDTG[i] * axisScale;

            // ========================================================
            // 🌟 旋轉軸的 0~360 度顯示處理
            // ========================================================
            if (type == AxisType::ROTARY || type == AxisType::ROTARY_CONTINUOUS)
            {
                // 對 360 取餘數 (折疊座標)
                displayMCS = std::fmod(displayMCS, 360.0);
                displayWCS = std::fmod(displayWCS, 360.0);

                // 防呆：如果是負角度 (例如 -10 度)，轉回正的 350 度
                if (displayMCS < 0.0) displayMCS += 360.0;
                if (displayWCS < 0.0) displayWCS += 360.0;
            }

            // 寫入 Shared Memory 廣播給人機
            pShm->NC_Status.actualMCS[i] = displayMCS;
            pShm->NC_Status.actualWCS[i] = displayWCS;


            pShm->NC_Status.manualFrameEnabled = nc->CoordSys.IsManualFrameEnabled() ? 1 : 0;
            pShm->NC_Status.manualFrameYawDeg = nc->CoordSys.GetManualFrameYaw();
            pShm->NC_Status.manualFramePitchDeg = nc->CoordSys.GetManualFramePitch();
            pShm->NC_Status.manualFrameRollDeg = nc->CoordSys.GetManualFrameRoll();
        }


        // --- HMI 控制指令交握 ---
        if (pShm->varCmd.writeReq)
        {
            nc->MacroSys.SetVar(pShm->varCmd.prefix, pShm->varCmd.index, pShm->varCmd.writeValue);
            pShm->varCmd.writeReq = false;
        }
        if (pShm->NC_Command.cycleStart) { nc->CycleStart(); pShm->NC_Command.cycleStart = false; }
        if (pShm->NC_Command.feedHold) { nc->FeedHold(); pShm->NC_Command.feedHold = false; }
        if (pShm->NC_Command.reset) { nc->Reset(); pShm->NC_Command.reset = false; }
        if (pShm->NC_Command.Close_System) { nc->Close_System_Com_flag = true; pShm->NC_Command.Close_System = false; }


        //單步執行切換
        if (pShm->NC_Command.Set_isSingleBlockEnabled_ON)
        {
            g_PLC->Set_C(NCPLC::C::SINGLE_BLOCK, true);

            nc->SetSingleBlockEnabled(true);
            pShm->NC_Command.Set_isSingleBlockEnabled_ON = false;
        }
        if (pShm->NC_Command.Set_isSingleBlockEnabled_OFF)
        {
            g_PLC->Set_C(NCPLC::C::SINGLE_BLOCK, false);
            nc->SetSingleBlockEnabled(false);
            pShm->NC_Command.Set_isSingleBlockEnabled_OFF = false;
        }
        //選擇性暫停切換
        if (pShm->NC_Command.Set_isOptionalStopEnabled_ON)
        {
            g_PLC->Set_C(NCPLC::C::OPTIONAL_STOP, true);
            nc->m_isOptionalStopEnabled = true;
            pShm->NC_Command.Set_isOptionalStopEnabled_ON = false;
        }
        if (pShm->NC_Command.Set_isOptionalStopEnabled_OFF)
        {
            g_PLC->Set_C(NCPLC::C::OPTIONAL_STOP, false);
            nc->m_isOptionalStopEnabled = false;
            pShm->NC_Command.Set_isOptionalStopEnabled_OFF = false;
        }
        //選擇性跳躍切換
        if (pShm->NC_Command.Set_isBlockSkipEnabled_ON)
        {
            g_PLC->Set_C(NCPLC::C::BLOCK_SKIP, true);
            nc->m_isBlockSkipEnabled = true;
            pShm->NC_Command.Set_isBlockSkipEnabled_ON = false;
        }
        if (pShm->NC_Command.Set_isBlockSkipEnabled_OFF)
        {
            g_PLC->Set_C(NCPLC::C::BLOCK_SKIP, false);
            nc->m_isBlockSkipEnabled = false;
            pShm->NC_Command.Set_isBlockSkipEnabled_OFF = false;
        }



        if (pShm->Coord_Command.reqSwitchWCS)
        {
            nc->CoordSys.SetWCS(pShm->Coord_Command.targetWCS_GCode, nc);
            pShm->Coord_Command.reqSwitchWCS = false;
        }

        if (pShm->Coord_Command.reqSetManualFrame)
        {
            // 1. Read Requested Values
            const bool requestedEnabled = pShm->Coord_Command.manualFrameEnabled;
            const double requestedYawDeg = pShm->Coord_Command.manualFrameYawDeg;
            const double requestedPitchDeg = pShm->Coord_Command.manualFramePitchDeg;
            const double requestedRollDeg = pShm->Coord_Command.manualFrameRollDeg;

            // 2. Parameter Validation (Shared Memory 是外部輸入邊界。NaN/INF 絕對不能進 CoordinateManager)
            const bool parameterValid = std::isfinite(requestedYawDeg) && std::isfinite(requestedPitchDeg) && std::isfinite(requestedRollDeg);

            if (!parameterValid)
            {
                pShm->Coord_Command.reqSetManualFrame = false; // 無效命令直接丟棄
            }
            else
            {
                // 3. Determine Whether Manual XYZ May Be Moving
                // NCPLCManager 的 Manual Motion Gate 只允許 IDLE / READY。
                // 因此只有在 IDLE / READY 時，VELOCITY / MOVING / MPG 才需要視為 Manual Motion 並安全停止。
                // RUN 中的 NC Program 不受 Manual Frame 影響，不可以因為修改 Manual Frame 而停止加工。
                const NCState ncState = nc->GetState();
                const bool manualOperationState = (ncState == NCState::IDLE || ncState == NCState::READY);
                bool waitingForManualStop = false;

                const MotionOwnerLease manualLease =
                    nc->GetMotion().GetMotionOwnerLease();
                const bool manualOwnerActive =
                    nc->GetMotion().IsMotionOwnerLeaseCurrent(manualLease) &&
                    (manualLease.owner == MotionOwner::JOG ||
                        manualLease.owner == MotionOwner::MPG);
                const bool incompatibleOwnerActive =
                    manualLease.IsValid() && !manualOwnerActive;

                // HOME / AUTO / SAFETY motion must never be stopped by a
                // Manual Frame request. Keep the request pending until that
                // owner has released Motion.
                if (manualOperationState &&
                    (nc->Homing.IsActive() || incompatibleOwnerActive))
                {
                    waitingForManualStop = true;
                }
                else if (manualOperationState && manualOwnerActive)
                {
                    const MotionCommandSource manualSource =
                        ResolveMotionCommandSourceForOwner(manualLease.owner);

                    // Manual Frame 只作用 XYZ，所以只處理 Machine X/Y/Z。
                    for (int axisIndex = 0; axisIndex < 3; ++axisIndex)
                    {
                        AxisContext& axis = nc->GetMotion().GetAxisContext(axisIndex);
                        if (!axis.isExist) continue;

                        if (axis.state == MotionState::MotionState_VELOCITY) // Continuous / Fine JOG
                        {
                            nc->GetMotion().SubmitAxisStopMove(axisIndex, axis.JOG_dec_time, manualSource, manualLease);
                            waitingForManualStop = true;
                            continue;
                        }

                        if (axis.state == MotionState::MotionState_MOVING) // INCH JOG
                        {
                            nc->GetMotion().SubmitAxisStopMove(axisIndex, axis.INCH_dec_time, manualSource, manualLease);
                            waitingForManualStop = true;
                            continue;
                        }

                        if (axis.state == MotionState::MotionState_MPG) // MPG
                        {
                            nc->GetMotion().SubmitAxisStopMove(axisIndex, axis.JOG_dec_time, manualSource, manualLease);
                            waitingForManualStop = true;
                            continue;
                        }

                        if (axis.state == MotionState::MotionState_STOPPING) // Stop still in progress
                        {
                            waitingForManualStop = true;
                            continue;
                        }
                    }
                }

                // 4. Apply Manual Frame
                // Manual Motion 尚未完全停止: Request 保持 true，下一個 ProcessTask Scan 再檢查。
                // 全部停止: 正式套用。
                if (!waitingForManualStop)
                {
                    nc->CoordSys.SetManualFrameAngles(requestedYawDeg, requestedPitchDeg, requestedRollDeg);
                    nc->CoordSys.SetManualFrameEnabled(requestedEnabled); // Enabled 最後才更新，避免先 Enable 但 Angle 尚未完整更新的短暫狀態
                    pShm->Coord_Command.reqSetManualFrame = false; // Request 最後清除
                }
            }
        }

        //PLC----------------------------------------------------------------------
        if (g_PLC != nullptr)
        {
            pShm->PLC_Status.SHM_PLC_RunCount = g_PLC->PLC_RunCount;

            // PLC 狀態全廣播
            g_PLC->ExportPLCStatus(&pShm->PLC_Status);
        }

        // 🌟 處理 PLC 點位寫入
        if (pShm->PLC_Command.writeReq)
        {
            if (g_PLC != nullptr)
            {
                g_PLC->SetMemory(pShm->PLC_Command.regionPrefix, pShm->PLC_Command.index, pShm->PLC_Command.writeValue);
            }
            pShm->PLC_Command.writeReq = false;
        }

        // 🌟 處理 PLC 變數名稱直接寫入
        if (pShm->PLC_Command.writeByNameReq)
        {
            pShm->PLC_Command.varName[63] = '\0';
            if (g_PLC != nullptr)
            {
                g_PLC->SetVar(std::string(pShm->PLC_Command.varName), pShm->PLC_Command.writeValue);
            }
            pShm->PLC_Command.writeByNameReq = false;
        }

    }

    // =========================================================================
    // 🐢 2. ProcessTask_100ms - 處理大資料表格與字串 (HMI 畫面刷新用)
    // =========================================================================
    void ProcessTask_100ms(NCManager* nc)
    {
        SHM_Data* pShm = SHMManager::GetInstance().GetData();
        if (pShm == nullptr || nc == nullptr) return;


        // --- 警報檢查 (自帶條件判斷，極快) ---
       // 1. 取得最新狀態
        uint32_t currentUpdateCount = AlarmManager::GetInstance().GetUpdateCount();

        // 2. 如果計數器有變，代表有新警報或警報剛被清除
        if (pShm->Alarm_Status.alarmUpdateCount != currentUpdateCount)
        {
            auto& am = AlarmManager::GetInstance();

            // 更新數量
            pShm->Alarm_Status.activeAlarmCount = am.GetAlarmCount();

            // 清空舊資料
            std::memset(pShm->Alarm_Status.activeAlarms, 0, sizeof(pShm->Alarm_Status.activeAlarms));

            // 🌟 將軸索引預設填滿 -1
            std::fill(std::begin(pShm->Alarm_Status.activeAlarmAxes),
                std::end(pShm->Alarm_Status.activeAlarmAxes), -1);

            // 🌟 把錯誤碼和軸號，一對一打包送進 SHM
            for (int i = 0; i < pShm->Alarm_Status.activeAlarmCount; i++) {
                pShm->Alarm_Status.activeAlarms[i] = am.GetAlarmId(i);
                pShm->Alarm_Status.activeAlarmAxes[i] = am.GetAlarmAxisIndex(i); // 抓取軸號
            }

            // 更新完成
            pShm->Alarm_Status.alarmUpdateCount = currentUpdateCount;
        }



        // --- 字串複製 (程式名稱) ---
        std::strncpy(pShm->NC_Status.mainProgName, nc->m_mainProgramName.c_str(), 63);
        pShm->NC_Status.mainProgName[63] = '\0';

        pShm->NC_Status.mainCurrentLine = nc->GetActivePC();
        std::strncpy(pShm->NC_Status.macroProgName, nc->m_macroProgramName.c_str(), 63);
        pShm->NC_Status.macroProgName[63] = '\0';


        // ==========================================================
            // 🌟 雙指標神同步 (Dual PC Synchronization) - 智慧混合版
            // ==========================================================

            // 1. 取得大腦的指標 (Interpreter PC)
        int interpreterMainPC = nc->GetBasePC();
        int interpreterMacroPC = nc->m_macroProgramPC;

        // 2. 取得實體馬達的指標 (Motion PC)
        int physicalPC = nc->GetMotion().GetPhysicalExecutionPC();

        // 3. 🌟 【神級判斷】：手腳追上大腦了嗎？
        // 如果 IsGroupDone() 為 true，代表底層倉庫全空，馬達完全靜止。
        // 這意味著目前的指令是「非運動指令」(如 M00, G04, 巨集變數)，機台正在執行大腦的狀態！
        bool isMachineIdle = nc->GetMotion().IsGroupDone();

        // 4. 判斷目前這張單子是屬於主程式還是副程式
        bool isMacroRunning = !nc->m_macroStack.empty();

        if (isMacroRunning) {
            // 在跑副程式
            // 🌟 如果機台靜止，游標顯示大腦卡住的地方(如 M00)；如果機台在動，顯示馬達正在跑的路徑
            pShm->NC_Status.macroCurrentLine = isMachineIdle ? interpreterMacroPC : physicalPC;
            pShm->NC_Status.mainCurrentLine = interpreterMainPC; // 主程式永遠顯示呼叫副程式的那一行
        }
        else {
            // 在跑主程式
            pShm->NC_Status.mainCurrentLine = isMachineIdle ? interpreterMainPC : physicalPC;
            pShm->NC_Status.macroCurrentLine = -1;
        }

        int interpreterWCS = nc->CoordSys.GetCurrentWCSGCode(); // 大腦的座標系
        int physicalWCS = nc->GetMotion().GetPhysicalExecutionWCS(); // 馬達的座標系

        pShm->NC_Status.currentWCS_GCode = isMachineIdle ? interpreterWCS : physicalWCS;//坐標系


        // 3. 🌟 刀具長度補正顯示 (Tool Length Comp)
        int interpreterToolMode = nc->CoordSys.toolLengthMode;
        int physicalToolMode = nc->GetMotion().GetPhysicalExecutionToolMode();

        int interpreterHCode = nc->CoordSys.currentHCode;
        int physicalHCode = nc->GetMotion().GetPhysicalExecutionHCode();

        //刀具號
        pShm->NC_Status.currentTCode = nc->CoordSys.currentTCode;
        //工件號
        pShm->NC_Status.currentWorkpieceNum = nc->CoordSys.currentWorkpieceNum;

        // 如果機台靜止(大腦卡住)，顯示大腦狀態；如果機台在跑，顯示馬達標籤狀態！
        // 假設 pShm->NC_Status 有這兩個變數供 UI 綁定
        pShm->NC_Status.currentToolLengthMode = isMachineIdle ? interpreterToolMode : physicalToolMode;
        pShm->NC_Status.currentHCode = isMachineIdle ? interpreterHCode : physicalHCode;

        // 4. 🌟 刀徑補正顯示 (Tool Radius Comp)
        int interpreterTRadMode = nc->CoordSys.toolRadiusMode;
        int physicalTRadMode = nc->GetMotion().GetPhysicalExecutionToolRadiusMode();

        int interpreterDCode = nc->CoordSys.currentDCode;
        int physicalDCode = nc->GetMotion().GetPhysicalExecutionDCode();

        // 🌟 取得大腦與馬達的 isAbsoluteMode
        bool interpreterAbs = nc->CoordSys.isAbsoluteMode;
        bool physicalAbs = nc->GetMotion().GetPhysicalExecutionIsAbsoluteMode();

        // 根據機台是否靜止，決定 UI 要聽誰的，然後轉成 1 或 0 傳給 HMI
        bool finalAbs = isMachineIdle ? interpreterAbs : physicalAbs;
        pShm->NC_Status.isAbsoluteMode = finalAbs ? 1 : 0;


        // 寫入 SHM 供 HMI 讀取
        pShm->NC_Status.currentToolRadiusMode = isMachineIdle ? interpreterTRadMode : physicalTRadMode;
        pShm->NC_Status.currentDCode = isMachineIdle ? interpreterDCode : physicalDCode;


        // 🌟 G68 旋轉狀態與角度顯示
        bool interpreterG68 = nc->CoordSys.isG68Active;
        bool physicalG68 = nc->GetMotion().GetPhysicalExecutionG68Active();

        double interpreterG68Angle = nc->CoordSys.g68Angle;
        double physicalG68Angle = nc->GetMotion().GetPhysicalExecutionG68Angle();

        // 寫入 SHM 供 HMI 讀取
        pShm->NC_Status.currentG68State = (isMachineIdle ? interpreterG68 : physicalG68) ? 1 : 0;
        pShm->NC_Status.currentG68Angle = isMachineIdle ? interpreterG68Angle : physicalG68Angle;



        // 🌟 G168 狀態與 W 碼顯示
        bool interpreterG168 = nc->CoordSys.isWorkpieceRotationActive;
        bool physicalG168 = nc->GetMotion().GetPhysicalExecutionG168Active();

        int interpreterWCode = nc->CoordSys.currentWCode;
        int physicalWCode = nc->GetMotion().GetPhysicalExecutionWCode();

        // 寫入 SHM 供 HMI 讀取
        pShm->NC_Status.currentG168State = (isMachineIdle ? interpreterG168 : physicalG168) ? 1 : 0;
        pShm->NC_Status.currentWCode = isMachineIdle ? interpreterWCode : physicalWCode;


        // 🌟 7. G51 縮放狀態顯示
        bool interpreterG51 = nc->CoordSys.isScalingActive;
        bool physicalG51 = nc->GetMotion().GetPhysicalExecutionG51Active();

        double interpreterScale = nc->CoordSys.scaleFactor;
        double physicalScale = nc->GetMotion().GetPhysicalExecutionScaleRatio();

        // 🌟 8. G151 / G150 鏡像狀態顯示

        // 算出大腦目前的 Mask
        uint8_t interpreterMirrorMask = 0;
        for (int i = 0; i < 8; i++) {
            if (nc->CoordSys.isMirrorActive[i]) {
                interpreterMirrorMask |= (1 << i);
            }
        }

        // 拿取馬達目前的 Mask
        uint8_t physicalMirrorMask = nc->GetMotion().GetPhysicalExecutionMirrorMask();

        // 寫入 SHM 供 HMI 讀取
        pShm->NC_Status.currentMirrorMask = isMachineIdle ? interpreterMirrorMask : physicalMirrorMask;

        // 寫入 SHM 供 HMI 讀取
        pShm->NC_Status.currentG51State = (isMachineIdle ? interpreterG51 : physicalG51) ? 1 : 0;
        pShm->NC_Status.currentScaleRatio = isMachineIdle ? interpreterScale : physicalScale;

        // 🌟 9. G16 極座標狀態顯示
        bool interpreterG16 = nc->CoordSys.isPolarCoordinateActive;
        bool physicalG16 = nc->GetMotion().GetPhysicalExecutionG16Active();

        // 寫入 SHM 供 HMI 讀取燈號
        bool finalG16 = isMachineIdle ? interpreterG16 : physicalG16;
        pShm->NC_Status.currentG16State = finalG16 ? 1 : 0;


        // 🌟 G162 偏心補償與 G17/18/19 平面顯示
        bool interpreterG162 = nc->CoordSys.isCAxisOffsetRotationEnabled;
        bool physicalG162 = nc->GetMotion().GetPhysicalExecutionG162Active();

        int interpreterPlane = nc->CoordSys.activePlane;
        int physicalPlane = nc->GetMotion().GetPhysicalExecutionPlaneMode();

        // 寫入 SHM 供 HMI 讀取
        pShm->NC_Status.currentG162State = (isMachineIdle ? interpreterG162 : physicalG162) ? 1 : 0;
        pShm->NC_Status.currentPlaneMode = isMachineIdle ? interpreterPlane : physicalPlane;

        pShm->NC_Status.currentG20State = nc->CoordSys.isInchMode;

        bool m_programmableTravelLimitEnabled = nc->CoordSys.m_programmableTravelLimitEnabled;
        pShm->NC_Status.m_programmableTravelLimitEnabled = m_programmableTravelLimitEnabled;


        if (pShm->NC_Command.reqChangeMode)//處理 OP 模式切換請求 (來自 NC_Command)
        {
            nc->ChangeMode(static_cast<NCOperationMode>(pShm->NC_Command.targetMode));
            pShm->NC_Command.reqChangeMode = false;
        }

        if (pShm->NC_Command.loadProgramReq)
        {
            size_t safeLength = strnlen(pShm->NC_Command.loadprogramName, 256);
            std::string NC_loadprogramName(pShm->NC_Command.loadprogramName, safeLength);
            nc->LoadProgram(GlobalConfig::GetInstance().NCProgramDir + NC_loadprogramName + ".nc");
            pShm->NC_Command.loadProgramReq = false;
            std::memset(pShm->NC_Command.loadprogramName, 0, sizeof(pShm->NC_Command.loadprogramName));
        }



        if (pShm->String_Command.reqLoadCode)//處理動態程式碼載入請求 (來自 String_Command)
        {
            pShm->String_Command.codeContent[2047] = '\0'; // 安全結尾防溢位
            std::string codeStr(pShm->String_Command.codeContent);

            // 呼叫統一的 API，它會自己看現在是 MDI 還是 MANUAL
            nc->LoadDynamicCode(codeStr);

            // 處理完畢，降下旗標並清空緩衝區
            pShm->String_Command.reqLoadCode = false;
            std::memset(pShm->String_Command.codeContent, 0, sizeof(pShm->String_Command.codeContent));
        }

        // =========================================================
             // 🌟 處理 HMI 寫入座標偏移量 (reqWriteOffset)
             // =========================================================
        if (pShm->Coord_Command.reqWriteOffset)
        {
            // 🌟 宣告 unitScale 讓這個區塊也能用公英制轉換
            double unitScale = nc->CoordSys.isInchMode ? (1.0 / 25.4) : 1.0;

            int row = pShm->Coord_Command.rowIndex;
            int axis = pShm->Coord_Command.axisIndex;
            double val = pShm->Coord_Command.writeValue;

            if (axis >= 0 && axis < 8)
            {
                // 🌟 取得對應軸的屬性，旋轉軸(度數)不套用英制轉換
                AxisType type = nc->m_motion.GetAxisContext(axis).axisType;
                double axisScale = (type == AxisType::ROTARY || type == AxisType::ROTARY_CONTINUOUS) ? 1.0 : unitScale;

                // 🌟 關鍵：將 HMI 輸入的數值 (可能為 inch) 除以倍率，還原回系統底層的公制 (mm)
                double systemVal = val / axisScale;

                switch (pShm->Coord_Command.offsetType)
                {
                case 0: nc->CoordSys.extOffset[axis] = systemVal; break;
                case 1: if (row >= 0 && row < 60)  nc->CoordSys.m_WCSTable[row][axis] = systemVal; break;
                case 2: if (row >= 0 && row < 100) nc->CoordSys.m_ToolOffset[row][axis] = systemVal; break;
                case 3: if (row >= 0 && row < 100) nc->CoordSys.m_WorkOffset[row][axis] = systemVal; break;
                case 10:
                    // G54 等原點設定功能：此處 systemVal 是從 HMI 傳入的期望座標
                    nc->CoordSys.m_WCSTable[nc->CoordSys.currentWCSIndex][axis] =
                        nc->CoordSys.actualMCS[axis] - nc->CoordSys.extOffset[axis] - systemVal;
                    break;
                }
            }
            pShm->Coord_Command.reqWriteOffset = false;
        }

        if (pShm->Coord_Command.reqSave)
        {
            switch (pShm->Coord_Command.saveType)
            {
            case 0: nc->CoordSys.SaveAllParameters(); break;
            case 1: nc->CoordSys.SaveWCSStatus(); break;
            case 2: nc->CoordSys.SaveExtOffset(); break;
            case 3: nc->CoordSys.SaveWCSTable(); break;
            case 4: nc->CoordSys.SaveToolOffset(); break;
            case 5: nc->CoordSys.SaveWorkOffset(); break;
            }
            pShm->Coord_Command.reqSave = false;
        }





        //軸狀態-------------------------------------------------------
        // =======================================================
        // 🌟 [修改這裡] 軸狀態：直接請 MotionCore 把資料填入 pShm 
        // =======================================================
        // 取代你原本手寫的 for 迴圈與 m_pContexts
        nc->GetMotion().ExportDebugInfo(pShm->axisDebug, true);






    }

    // =========================================================================
    // 🚶 3. ProcessTask_500ms - 中慢速任務 (半秒 1次)
    // =========================================================================
    void ProcessTask_500ms(NCManager* nc)
    {
        SHM_Data* pShm = SHMManager::GetInstance().GetData();
        if (pShm == nullptr || nc == nullptr) return;


        // 🌟 [新增] 1. 取得當前的公英制倍率
        double unitScale = nc->CoordSys.isInchMode ? (1.0 / 25.4) : 1.0;


        // --- Macro 變數全廣播 (約 18KB) ---
        pShm->macroStatus.currentCallDepth = nc->MacroSys.GetCurrentDepth();

        if (pShm->varCmd.refresh_global)
        {
            memcpy(pShm->macroStatus.globalVars, nc->MacroSys.GetGlobalVarsArray(), sizeof(pShm->macroStatus.globalVars));
            pShm->varCmd.refresh_global = false;
        }

        if (pShm->varCmd.refresh_system)
        {
            memcpy(pShm->macroStatus.sysVars, nc->MacroSys.GetSysVarsArray(), sizeof(pShm->macroStatus.sysVars));
            pShm->varCmd.refresh_system = false;
        }

        if (pShm->varCmd.refresh_local)
        {
            for (int i = 0; i < 8; i++)
            {
                memcpy(pShm->macroStatus.localVars[i], nc->MacroSys.GetLocalVarsArray(i), sizeof(double) * 101);
            }
            pShm->varCmd.refresh_local = false;
        }

        // =====================================================================
                // 🌟 2. 座標與補償表格拷貝 (捨棄 memcpy，改用迴圈逐軸套用單位轉換)
                // =====================================================================
        for (int col = 0; col < 8; col++)
        {
            // 判斷是否為旋轉軸
            AxisType type = nc->m_motion.GetAxisContext(col).axisType;
            double axisScale = (type == AxisType::ROTARY || type == AxisType::ROTARY_CONTINUOUS) ? 1.0 : unitScale;

            // A. 外部偏移 (EXT)
            pShm->Coord_Table.extOffset[col] = nc->CoordSys.extOffset[col] * axisScale;

            // B. G54 ~ G59 表格
            for (size_t row = 0; row < 60 && row < nc->CoordSys.m_WCSTable.size(); ++row) {
                pShm->Coord_Table.wcsTable[row][col] = nc->CoordSys.m_WCSTable[row][col] * axisScale;
            }

            // C. 刀具長度/半徑補正表
            for (size_t row = 0; row < 100 && row < nc->CoordSys.m_ToolOffset.size(); ++row) {
                pShm->Coord_Table.toolOffset[row][col] = nc->CoordSys.m_ToolOffset[row][col] * axisScale;
            }

            // D. G168 獨立工件補正表
            for (size_t row = 0; row < 100 && row < nc->CoordSys.m_WorkOffset.size(); ++row) {
                pShm->Coord_Table.workOffset[row][col] = nc->CoordSys.m_WorkOffset[row][col] * axisScale;
            }
        }




        //PLC----------------------------------------------------------------------
        //重置PLC邏輯檔案
        if (pShm->PLC_Command.ReloadLogicProgram)
        {
            g_PLC->ReloadLogicProgram();//重置PLC邏輯檔案
            pShm->PLC_Command.ReloadLogicProgram = false;
        }

        // V7.5.3 - PLC Runtime Diagnostics read-only broadcast.
        // Kept outside SHM_PLC_Status so PLC_Command offset remains unchanged.
        if (g_PLC != nullptr)
        {
            const PLCRuntimeDiagnostics diag = g_PLC->GetRuntimeDiagnostics();

            pShm->PLC_Diagnostics.RuntimeFault = diag.faultActive ? 1 : 0;
            pShm->PLC_Diagnostics.LastFaultCode = diag.lastFaultCode;
            pShm->PLC_Diagnostics.LastOpcode = diag.lastOpcode;
            pShm->PLC_Diagnostics.LastOperandRegion = diag.lastOperandRegion;
            pShm->PLC_Diagnostics.LastTaskIndex = diag.lastTaskIndex;
            pShm->PLC_Diagnostics.LastInstructionIndex = diag.lastInstructionIndex;
            pShm->PLC_Diagnostics.LastOperandAddress = diag.lastOperandAddress;
            pShm->PLC_Diagnostics.LastFaultRunCount = diag.lastFaultRunCount;
            pShm->PLC_Diagnostics.TotalFaultCount = diag.totalFaultCount;
            pShm->PLC_Diagnostics.InvalidOperandCount = diag.invalidOperandCount;
            pShm->PLC_Diagnostics.InvalidTimerIndexCount = diag.invalidTimerIndexCount;
            pShm->PLC_Diagnostics.InvalidCounterIndexCount = diag.invalidCounterIndexCount;
            pShm->PLC_Diagnostics.UnknownOpcodeCount = diag.unknownOpcodeCount;
            pShm->PLC_Diagnostics.RuntimeStateMismatchCount = diag.runtimeStateMismatchCount;
            pShm->PLC_Diagnostics.InvalidFlowRowCount = diag.invalidFlowRowCount;

            // V7.6.0 - actual accepted logic.bin identity.
            const PLCLogicVerificationState logicState = g_PLC->GetLogicVerificationState();
            pShm->PLC_Diagnostics.LoadedLogicCrc32 = logicState.loadedLogicCrc32;
            pShm->PLC_Diagnostics.LoadedLogicSize = logicState.loadedLogicSize;
            pShm->PLC_Diagnostics.LogicLoadGeneration = logicState.logicLoadGeneration;
            pShm->PLC_Diagnostics.LastLogicLoadResult = logicState.lastLogicLoadResult;

            // V7.6.4.1 Runtime Scan Health
            const PLCScanHealth scanHealth = g_PLC->GetScanHealth();
            pShm->PLC_Diagnostics.ScanLastCycleUs = scanHealth.lastCycleUs;
            pShm->PLC_Diagnostics.ScanWorstCycleUs = scanHealth.worstCycleUs;
            pShm->PLC_Diagnostics.ScanCycleBudgetUs = scanHealth.cycleBudgetUs;
            pShm->PLC_Diagnostics.ScanCycleOverrunCount = scanHealth.cycleOverrunCount;
            pShm->PLC_Diagnostics.ScanLastTaskUs = scanHealth.lastTaskUs;
            pShm->PLC_Diagnostics.ScanWorstTaskUs = scanHealth.worstTaskUs;
            pShm->PLC_Diagnostics.ScanLastTaskBudgetUs = scanHealth.lastTaskBudgetUs;
            pShm->PLC_Diagnostics.ScanLastTaskIndex = scanHealth.lastTaskIndex;
            pShm->PLC_Diagnostics.ScanWorstTaskIndex = scanHealth.worstTaskIndex;
            pShm->PLC_Diagnostics.ScanTaskOverrunCount = scanHealth.taskOverrunCount;
            pShm->PLC_Diagnostics.ScanMeasuredCycleCount = scanHealth.measuredCycleCount;
            pShm->PLC_Diagnostics.ScanLastOverrunRunCount = scanHealth.lastOverrunRunCount;
            pShm->PLC_Diagnostics.ScanLastOverrunTaskIndex = scanHealth.lastOverrunTaskIndex;
        }


    }

    // =========================================================================
    // 🐌 3. ProcessTask_1000ms - 慢速背景任務 (1秒 1次)
    // =========================================================================
    void ProcessTask_1000ms(NCManager* nc)
    {
        SHM_Data* pShm = SHMManager::GetInstance().GetData();
        if (pShm == nullptr || nc == nullptr) return;

        // =============================================================
        // Stage NC-0.1F.2 - No-motion Runtime Acceptance Diagnostics
        //
        // This code runs only in the existing 1000 ms supervisory task.
        // It never writes AxisContext and never consumes either command
        // or feedback rings.  It only reads atomic / snapshot counters.
        //
        // Output policy:
        //   1. Print the first 15 one-second samples after startup.
        //   2. Afterwards, print only while a transport is busy, Safety is
        //      pending, Owner changes, or an error counter changes.
        //
        // This keeps the acceptance trace useful without creating an
        // unlimited one-line-per-second production log.
        // =============================================================
        MotionCore& motion = nc->GetMotion();

        const MotionOwnerLease ownerLease =
            motion.GetMotionOwnerLease();

        const bool safetyPending =
            motion.HasPendingSafetyOrRecoveryRequests();

        const std::size_t axisCommandDepth =
            motion.GetAxisCommandMailboxDepth();
        const std::size_t commandIngressDepth =
            motion.GetCommandIngressSize();
        const std::size_t commandReplayDepth =
            motion.GetCommandReplaySize();
        const std::size_t feedbackDepth =
            motion.GetMotionFeedbackDepth();
        const std::size_t feedbackNoticeDepth =
            motion.GetMotionFeedbackProducerNoticeDepth();

        const std::uint64_t axisQueueFull =
            motion.GetAxisCommandQueueFullCount();
        const std::uint64_t axisResultOverflow =
            motion.GetAxisCommandResultOverflowCount();
        const std::uint64_t ownerConflictReject =
            motion.GetMotionOwnerConflictRejectCount();
        const std::uint64_t staleDiscard =
            motion.GetStaleCommandDiscardCount();
        const std::uint64_t commandQueueFull =
            motion.GetCommandQueueFullRejectCount();
        const std::uint64_t replayOverflow =
            motion.GetCommandReplayOverflowCount();
        const std::uint64_t feedbackOverflow =
            motion.GetMotionFeedbackOverflowCount();
        const std::uint64_t feedbackNoticeOverflow =
            motion.GetMotionFeedbackProducerNoticeOverflowCount();

        const std::uint64_t feedbackProcessed =
            nc->GetProcessedMotionFeedbackCount();
        const std::uint64_t feedbackSequenceGap =
            nc->GetMotionFeedbackSequenceGapCount();
        const std::uint64_t feedbackAccepted =
            nc->GetAcceptedMotionFeedbackCount();
        const std::uint64_t feedbackStarted =
            nc->GetStartedMotionFeedbackCount();
        const std::uint64_t feedbackCompleted =
            nc->GetCompletedMotionFeedbackCount();
        const std::uint64_t feedbackRejected =
            nc->GetRejectedMotionFeedbackCount();
        const std::uint64_t feedbackAborted =
            nc->GetAbortedMotionFeedbackCount();
        const std::uint64_t feedbackFaulted =
            nc->GetFaultedMotionFeedbackCount();

        NCBlockLifecycleSnapshot blockLifecycle{};
        const bool hasBlockLifecycle =
            nc->GetLastBlockLifecycleSnapshot(blockLifecycle);
        const NCBlockLifecycleCounters blockCounters =
            nc->GetBlockLifecycleCounters();

        const NCBlockCompletionBoundarySnapshot completionBoundary =
            nc->GetLastBlockCompletionBoundarySnapshot();
        const NCBlockCompletionBoundaryCounters completionCounters =
            nc->GetBlockCompletionBoundaryCounters();

        const NCProgramEndGateSnapshot programEndSnapshot =
            nc->GetProgramEndGateSnapshot();
        const NCProgramEndGateCounters programEndCounters =
            nc->GetProgramEndGateCounters();

        const NCGMBlockTransactionSnapshot gmTransactionSnapshot =
            nc->GetGMBlockTransactionSnapshot();
        const NCGMBlockTransactionCounters gmTransactionCounters =
            nc->GetGMBlockTransactionCounters();

        const NCSingleBlockShadowSnapshot singleBlockShadowSnapshot =
            nc->GetSingleBlockShadowSnapshot();
        const NCSingleBlockShadowCounters singleBlockShadowCounters =
            nc->GetSingleBlockShadowCounters();

        const NCFeedHoldBoundarySnapshot feedHoldSnapshot =
            nc->GetFeedHoldBoundarySnapshot();
        const NCFeedHoldBoundaryCounters feedHoldCounters =
            nc->GetFeedHoldBoundaryCounters();

        const std::uint64_t lifecycleChangeToken =
            blockCounters.dispatched +
            blockCounters.programCommitted +
            blockCounters.motionSegmentsBound +
            blockCounters.feedbackAccepted +
            blockCounters.feedbackStarted +
            blockCounters.feedbackCompleted +
            blockCounters.feedbackRejected +
            blockCounters.feedbackCancelled +
            blockCounters.feedbackAborted +
            blockCounters.feedbackFaulted +
            blockCounters.blockCompleted +
            blockCounters.blockFailed +
            blockCounters.orphanFeedback +
            blockCounters.motionCaptureOverflow;

        const std::uint64_t completionBoundaryChangeToken =
            completionCounters.bindings +
            completionCounters.releaseChecks +
            completionCounters.agreeRelease +
            completionCounters.ledgerReadyBeforeLegacy +
            completionCounters.legacyEarlyRelease +
            completionCounters.ledgerFailureObserved +
            completionCounters.releaseOnLedgerFailure +
            completionCounters.trackingOverflow +
            completionCounters.missingLifecycle +
            completionCounters.nonMotionWait +
            completionCounters.supersededBindings +
            completionCounters.dualKeyRelease +
            completionCounters.blockedLegacyEarly +
            completionCounters.blockedLedgerFailure +
            completionCounters.blockedTrackingOverflow +
            completionCounters.blockedMissingLifecycle +
            completionCounters.blockedNotTracked +
            completionCounters.failClosedBindings;

        const std::uint64_t programEndChangeToken =
            programEndSnapshot.sequence +
            programEndCounters.runStartAttempts +
            programEndCounters.requests +
            programEndCounters.evaluations +
            programEndCounters.readyToFinalize +
            programEndCounters.finalized +
            programEndCounters.cancelled +
            programEndCounters.failClosed;

        const std::uint64_t gmTransactionChangeToken =
            gmTransactionSnapshot.sequence +
            gmTransactionCounters.started +
            gmTransactionCounters.evaluations +
            gmTransactionCounters.readyTransitions +
            gmTransactionCounters.finalized +
            gmTransactionCounters.cancelled +
            gmTransactionCounters.finalizeFailed;

        const std::uint64_t singleBlockShadowChangeToken =
            singleBlockShadowSnapshot.sequence +
            singleBlockShadowCounters.armAttempts +
            singleBlockShadowCounters.armed +
            singleBlockShadowCounters.notEligible +
            singleBlockShadowCounters.evaluations +
            singleBlockShadowCounters.boundaryReady +
            singleBlockShadowCounters.legacyHolds +
            singleBlockShadowCounters.agreeHolds +
            singleBlockShadowCounters.legacyEarlyHolds +
            singleBlockShadowCounters.legacyHoldWithoutArm +
            singleBlockShadowCounters.legacyMissingHold +
            singleBlockShadowCounters.programEndSuppressed +
            singleBlockShadowCounters.cancelled;

        const std::uint64_t feedHoldChangeToken =
            feedHoldSnapshot.sequence +
            feedHoldCounters.requestAttempts +
            feedHoldCounters.requestsLatched +
            feedHoldCounters.evaluations +
            feedHoldCounters.acknowledged +
            feedHoldCounters.legacyHolds +
            feedHoldCounters.legacyEarlyHolds +
            feedHoldCounters.resumeRequests +
            feedHoldCounters.resumed +
            feedHoldCounters.ownerChanged +
            feedHoldCounters.executionEpochChanged +
            feedHoldCounters.motionFault +
            feedHoldCounters.acknowledgeLost +
            feedHoldCounters.cancelled +
            feedHoldCounters.superseded;

        const std::uint64_t transportErrorTotal =
            axisQueueFull +
            axisResultOverflow +
            ownerConflictReject +
            staleDiscard +
            commandQueueFull +
            replayOverflow +
            feedbackOverflow +
            feedbackNoticeOverflow +
            feedbackSequenceGap +
            feedbackRejected +
            feedbackFaulted;

        const bool transportBusy =
            axisCommandDepth != 0U ||
            commandIngressDepth != 0U ||
            commandReplayDepth != 0U ||
            feedbackDepth != 0U ||
            feedbackNoticeDepth != 0U;

        static std::uint32_t startupSamples = 0U;
        static MotionOwner previousOwner = MotionOwner::NONE;
        static MotionOwnerGeneration previousGeneration =
            MOTION_OWNER_GENERATION_INVALID;
        static std::uint64_t previousTransportErrorTotal = 0ULL;
        static std::uint64_t previousLifecycleChangeToken = 0ULL;
        static std::uint64_t previousCompletionBoundaryChangeToken = 0ULL;
        static std::uint64_t previousProgramEndChangeToken = 0ULL;
        static std::uint64_t previousGMTransactionChangeToken = 0ULL;
        static std::uint64_t previousSingleBlockShadowChangeToken = 0ULL;
        static std::uint64_t previousFeedHoldChangeToken = 0ULL;

        const bool ownerChanged =
            startupSamples != 0U &&
            (ownerLease.owner != previousOwner ||
                ownerLease.generation != previousGeneration);

        const bool errorCounterChanged =
            startupSamples != 0U &&
            transportErrorTotal != previousTransportErrorTotal;

        const bool lifecycleChanged =
            startupSamples != 0U &&
            lifecycleChangeToken != previousLifecycleChangeToken;

        const bool completionBoundaryChanged =
            startupSamples != 0U &&
            completionBoundaryChangeToken !=
            previousCompletionBoundaryChangeToken;

        const bool programEndChanged =
            startupSamples != 0U &&
            programEndChangeToken != previousProgramEndChangeToken;

        const bool gmTransactionChanged =
            startupSamples != 0U &&
            gmTransactionChangeToken != previousGMTransactionChangeToken;

        const bool singleBlockShadowChanged =
            startupSamples != 0U &&
            singleBlockShadowChangeToken !=
            previousSingleBlockShadowChangeToken;

        const bool feedHoldChanged =
            startupSamples != 0U &&
            feedHoldChangeToken != previousFeedHoldChangeToken;

        const bool shouldPrint =
            startupSamples < 15U ||
            lifecycleChanged ||
            completionBoundaryChanged ||
            programEndChanged ||
            gmTransactionChanged ||
            singleBlockShadowChanged ||
            feedHoldChanged ||
            ownerChanged ||
            errorCounterChanged ||
            safetyPending ||
            transportBusy;

        if (shouldPrint)
        {
            RtPrintf(
                "[NC01F-RT] NC:%d Mode:%d Owner:%s(%u) Gen:%u "
                "Safety:%u AxisQ:%llu CmdIn:%llu Replay:%llu "
                "FbQ:%llu Notice:%llu\n",
                static_cast<int>(nc->GetState()),
                static_cast<int>(nc->GetMode()),
                MotionOwnerToDiagnosticName(ownerLease.owner),
                static_cast<unsigned int>(ownerLease.owner),
                static_cast<unsigned int>(ownerLease.generation),
                safetyPending ? 1U : 0U,
                static_cast<unsigned long long>(axisCommandDepth),
                static_cast<unsigned long long>(commandIngressDepth),
                static_cast<unsigned long long>(commandReplayDepth),
                static_cast<unsigned long long>(feedbackDepth),
                static_cast<unsigned long long>(feedbackNoticeDepth));

            RtPrintf(
                "[NC01F-CNT] AxisQFull:%llu AxisResOv:%llu "
                "OwnerReject:%llu Stale:%llu CmdFull:%llu ReplayOv:%llu "
                "FbOv:%llu NoticeOv:%llu FbProc:%llu FbGap:%llu "
                "FbAcc:%llu FbStart:%llu FbDone:%llu FbReject:%llu "
                "FbAbort:%llu FbFault:%llu\n",
                static_cast<unsigned long long>(axisQueueFull),
                static_cast<unsigned long long>(axisResultOverflow),
                static_cast<unsigned long long>(ownerConflictReject),
                static_cast<unsigned long long>(staleDiscard),
                static_cast<unsigned long long>(commandQueueFull),
                static_cast<unsigned long long>(replayOverflow),
                static_cast<unsigned long long>(feedbackOverflow),
                static_cast<unsigned long long>(feedbackNoticeOverflow),
                static_cast<unsigned long long>(feedbackProcessed),
                static_cast<unsigned long long>(feedbackSequenceGap),
                static_cast<unsigned long long>(feedbackAccepted),
                static_cast<unsigned long long>(feedbackStarted),
                static_cast<unsigned long long>(feedbackCompleted),
                static_cast<unsigned long long>(feedbackRejected),
                static_cast<unsigned long long>(feedbackAborted),
                static_cast<unsigned long long>(feedbackFaulted));

            RtPrintf(
                "[NC02D-BLK] D:%llu Scope:%u Cache:%llu Frame:%llu "
                "PC:%d Line:%d State:%s Commit:%u Seg:%u "
                "Acc:%u Start:%u Done:%u Term:%u Fail:%u Ov:%u\n",
                static_cast<unsigned long long>(
                    hasBlockLifecycle ? blockLifecycle.dispatchId : 0ULL),
                static_cast<unsigned int>(
                    hasBlockLifecycle ? blockLifecycle.programTarget.scope :
                    NCProgramScope::NONE),
                static_cast<unsigned long long>(
                    hasBlockLifecycle ? blockLifecycle.programTarget.cacheGeneration : 0ULL),
                static_cast<unsigned long long>(
                    hasBlockLifecycle ? blockLifecycle.programTarget.frameId : 0ULL),
                hasBlockLifecycle ? blockLifecycle.programTarget.sourcePC : -1,
                hasBlockLifecycle ? blockLifecycle.sourceLineNumber : 0,
                NCBlockLifecycleStateToDiagnosticName(
                    hasBlockLifecycle ? blockLifecycle.state :
                    NCBlockLifecycleState::NONE),
                hasBlockLifecycle && blockLifecycle.programCommitted ? 1U : 0U,
                hasBlockLifecycle ?
                static_cast<unsigned int>(blockLifecycle.motionSegmentCount) : 0U,
                hasBlockLifecycle ?
                static_cast<unsigned int>(blockLifecycle.motionAcceptedCount) : 0U,
                hasBlockLifecycle ?
                static_cast<unsigned int>(blockLifecycle.motionStartedCount) : 0U,
                hasBlockLifecycle ?
                static_cast<unsigned int>(blockLifecycle.motionCompletedCount) : 0U,
                hasBlockLifecycle ?
                static_cast<unsigned int>(blockLifecycle.motionTerminalCount) : 0U,
                hasBlockLifecycle ?
                static_cast<unsigned int>(blockLifecycle.motionFailedCount) : 0U,
                hasBlockLifecycle && blockLifecycle.motionCaptureOverflow ? 1U : 0U);

            RtPrintf(
                "[NC02D-CNT] Dispatch:%llu Commit:%llu ProgOnly:%llu "
                "MotionBlk:%llu Seg:%llu PAcc:%llu PRej:%llu "
                "FAcc:%llu FStart:%llu FDone:%llu FRej:%llu "
                "FCancel:%llu FAbort:%llu FFault:%llu "
                "BlkDone:%llu BlkFail:%llu NCFail:%llu "
                "CaptureOv:%llu Orphan:%llu DupTerm:%llu Conflict:%llu "
                "Active:%u\n",
                static_cast<unsigned long long>(blockCounters.dispatched),
                static_cast<unsigned long long>(blockCounters.programCommitted),
                static_cast<unsigned long long>(blockCounters.programOnlyCompleted),
                static_cast<unsigned long long>(blockCounters.motionBlocks),
                static_cast<unsigned long long>(blockCounters.motionSegmentsBound),
                static_cast<unsigned long long>(blockCounters.producerAccepted),
                static_cast<unsigned long long>(blockCounters.producerRejected),
                static_cast<unsigned long long>(blockCounters.feedbackAccepted),
                static_cast<unsigned long long>(blockCounters.feedbackStarted),
                static_cast<unsigned long long>(blockCounters.feedbackCompleted),
                static_cast<unsigned long long>(blockCounters.feedbackRejected),
                static_cast<unsigned long long>(blockCounters.feedbackCancelled),
                static_cast<unsigned long long>(blockCounters.feedbackAborted),
                static_cast<unsigned long long>(blockCounters.feedbackFaulted),
                static_cast<unsigned long long>(blockCounters.blockCompleted),
                static_cast<unsigned long long>(blockCounters.blockFailed),
                static_cast<unsigned long long>(blockCounters.ncDispatchFailed),
                static_cast<unsigned long long>(blockCounters.motionCaptureOverflow),
                static_cast<unsigned long long>(blockCounters.orphanFeedback),
                static_cast<unsigned long long>(blockCounters.duplicateTerminalFeedback),
                static_cast<unsigned long long>(blockCounters.terminalFeedbackConflict),
                static_cast<unsigned int>(blockCounters.activeBlocks));

            RtPrintf(
                "[NC02E-BND] D:%llu Kind:%s Legacy:%s Ledger:%s "
                "Compare:%s State:%s Seg:%u Done:%u Term:%u Fail:%u\n",
                static_cast<unsigned long long>(completionBoundary.dispatchId),
                NCBlockWaitKindToDiagnosticName(completionBoundary.waitKind),
                completionBoundary.legacyReady ? "READY" : "WAIT",
                NCBlockMotionBoundaryStateToDiagnosticName(
                    completionBoundary.ledgerBoundary),
                NCBlockCompletionComparisonToDiagnosticName(
                    completionBoundary.comparison),
                NCBlockLifecycleStateToDiagnosticName(
                    completionBoundary.lifecycleState),
                static_cast<unsigned int>(completionBoundary.motionSegmentCount),
                static_cast<unsigned int>(completionBoundary.motionCompletedCount),
                static_cast<unsigned int>(completionBoundary.motionTerminalCount),
                static_cast<unsigned int>(completionBoundary.motionFailedCount));

            RtPrintf(
                "[NC02E-CNT] Bind:%llu Motion:%llu Aux:%llu Flow:%llu "
                "Obs:%llu Release:%llu Agree:%llu LedgerEarly:%llu "
                "LegacyEarly:%llu LedgerFail:%llu ReleaseFail:%llu "
                "Overflow:%llu Missing:%llu NonMotion:%llu Supersede:%llu\n",
                static_cast<unsigned long long>(completionCounters.bindings),
                static_cast<unsigned long long>(completionCounters.motionBindings),
                static_cast<unsigned long long>(completionCounters.auxiliaryBindings),
                static_cast<unsigned long long>(completionCounters.programFlowBindings),
                static_cast<unsigned long long>(completionCounters.observations),
                static_cast<unsigned long long>(completionCounters.releaseChecks),
                static_cast<unsigned long long>(completionCounters.agreeRelease),
                static_cast<unsigned long long>(completionCounters.ledgerReadyBeforeLegacy),
                static_cast<unsigned long long>(completionCounters.legacyEarlyRelease),
                static_cast<unsigned long long>(completionCounters.ledgerFailureObserved),
                static_cast<unsigned long long>(completionCounters.releaseOnLedgerFailure),
                static_cast<unsigned long long>(completionCounters.trackingOverflow),
                static_cast<unsigned long long>(completionCounters.missingLifecycle),
                static_cast<unsigned long long>(completionCounters.nonMotionWait),
                static_cast<unsigned long long>(completionCounters.supersededBindings));

            RtPrintf(
                "[NC02F-GATE] D:%llu Eligible:%u Applied:%u Legacy:%s "
                "Ledger:%s Decision:%s Effective:%s FailClosed:%u\n",
                static_cast<unsigned long long>(completionBoundary.dispatchId),
                completionBoundary.guardEligible ? 1U : 0U,
                completionBoundary.guardApplied ? 1U : 0U,
                completionBoundary.legacyReady ? "READY" : "WAIT",
                NCBlockMotionBoundaryStateToDiagnosticName(
                    completionBoundary.ledgerBoundary),
                NCBlockCompletionGateDecisionToDiagnosticName(
                    completionBoundary.gateDecision),
                completionBoundary.effectiveReady ? "READY" : "WAIT",
                completionBoundary.failClosed ? 1U : 0U);

            RtPrintf(
                "[NC02F-CNT] Eval:%llu Eligible:%llu Bypass:%llu "
                "Wait:%llu Release:%llu BlockEarly:%llu BlockFail:%llu "
                "BlockOv:%llu BlockMissing:%llu BlockNotTracked:%llu "
                "FailClosed:%llu\n",
                static_cast<unsigned long long>(completionCounters.guardEvaluations),
                static_cast<unsigned long long>(completionCounters.guardEligibleSamples),
                static_cast<unsigned long long>(completionCounters.guardBypassSamples),
                static_cast<unsigned long long>(completionCounters.guardWaitSamples),
                static_cast<unsigned long long>(completionCounters.dualKeyRelease),
                static_cast<unsigned long long>(completionCounters.blockedLegacyEarly),
                static_cast<unsigned long long>(completionCounters.blockedLedgerFailure),
                static_cast<unsigned long long>(completionCounters.blockedTrackingOverflow),
                static_cast<unsigned long long>(completionCounters.blockedMissingLifecycle),
                static_cast<unsigned long long>(completionCounters.blockedNotTracked),
                static_cast<unsigned long long>(completionCounters.failClosedBindings));
            RtPrintf(
                "[NC02G-END] Run:%llu Req:%llu Cause:%s Phase:%s Decision:%s "
                "Scope:%u Cache:%llu RunEpoch:%u ReqEpoch:%u "
                "RunOwner:%u/%u ReqOwner:%u/%u CurOwner:%u/%u "
                "PC:%d Line:%d D:%llu Pending:%u Ready:%u Fail:%u\n",
                static_cast<unsigned long long>(programEndSnapshot.run.runId),
                static_cast<unsigned long long>(programEndSnapshot.requestId),
                NCProgramEndCauseToDiagnosticName(programEndSnapshot.cause),
                NCProgramEndPhaseToDiagnosticName(programEndSnapshot.phase),
                NCProgramEndDecisionToDiagnosticName(programEndSnapshot.decision),
                static_cast<unsigned int>(programEndSnapshot.run.scope),
                static_cast<unsigned long long>(programEndSnapshot.run.cacheGeneration),
                static_cast<unsigned int>(programEndSnapshot.run.executionEpoch),
                static_cast<unsigned int>(programEndSnapshot.requestExecutionEpoch),
                static_cast<unsigned int>(programEndSnapshot.run.owner),
                static_cast<unsigned int>(programEndSnapshot.run.ownerGeneration),
                static_cast<unsigned int>(programEndSnapshot.requestOwner),
                static_cast<unsigned int>(programEndSnapshot.requestOwnerGeneration),
                static_cast<unsigned int>(programEndSnapshot.currentOwner),
                static_cast<unsigned int>(programEndSnapshot.currentOwnerGeneration),
                programEndSnapshot.sourcePC,
                programEndSnapshot.sourceLineNumber,
                static_cast<unsigned long long>(programEndSnapshot.markerDispatchId),
                programEndSnapshot.requestPending ? 1U : 0U,
                programEndSnapshot.readyToFinalize ? 1U : 0U,
                programEndSnapshot.failClosed ? 1U : 0U);

            RtPrintf(
                "[NC02G-DRN] Active:%u AxisQ:%llu AxisR:%llu CmdQ:%llu "
                "CmdIn:%llu Replay:%llu FbQ:%llu Notice:%llu "
                "FbPub:%llu FbCon:%llu FbSync:%u Stable:%u/%u "
                "Stand:%u Safety:%u Wait:%u Bind:%u OwnerOK:%u Integrity:%llu\n",
                static_cast<unsigned int>(programEndSnapshot.activeBlocks),
                static_cast<unsigned long long>(programEndSnapshot.axisCommandDepth),
                static_cast<unsigned long long>(programEndSnapshot.axisResultDepth),
                static_cast<unsigned long long>(programEndSnapshot.commandQueueDepth),
                static_cast<unsigned long long>(programEndSnapshot.commandIngressDepth),
                static_cast<unsigned long long>(programEndSnapshot.commandReplayDepth),
                static_cast<unsigned long long>(programEndSnapshot.feedbackDepth),
                static_cast<unsigned long long>(programEndSnapshot.feedbackNoticeDepth),
                static_cast<unsigned long long>(programEndSnapshot.lastPublishedFeedbackSequence),
                static_cast<unsigned long long>(programEndSnapshot.lastConsumedFeedbackSequence),
                programEndSnapshot.feedbackSequenceSynchronized ? 1U : 0U,
                static_cast<unsigned int>(programEndSnapshot.stablePasses),
                static_cast<unsigned int>(NC_PROGRAM_END_STABLE_PASSES_REQUIRED),
                programEndSnapshot.groupStandstill ? 1U : 0U,
                programEndSnapshot.safetyOrRecoveryPending ? 1U : 0U,
                programEndSnapshot.waitCallbackActive ? 1U : 0U,
                programEndSnapshot.completionBindingActive ? 1U : 0U,
                programEndSnapshot.ownerLeaseCurrent ? 1U : 0U,
                static_cast<unsigned long long>(programEndSnapshot.integrityDelta));

            RtPrintf(
                "[NC02G-CNT] StartTry:%llu Started:%llu StartBlock:%llu "
                "Req:%llu EOF:%llu M02:%llu M30:%llu ReqReject:%llu "
                "Eval:%llu Ready:%llu Final:%llu Cancel:%llu Fail:%llu "
                "EpochFail:%llu OwnerFail:%llu IntegrityFail:%llu\n",
                static_cast<unsigned long long>(programEndCounters.runStartAttempts),
                static_cast<unsigned long long>(programEndCounters.runsStarted),
                static_cast<unsigned long long>(programEndCounters.runStartBlocked),
                static_cast<unsigned long long>(programEndCounters.requests),
                static_cast<unsigned long long>(programEndCounters.naturalEofRequests),
                static_cast<unsigned long long>(programEndCounters.m02Requests),
                static_cast<unsigned long long>(programEndCounters.m30Requests),
                static_cast<unsigned long long>(programEndCounters.rejectedRequests),
                static_cast<unsigned long long>(programEndCounters.evaluations),
                static_cast<unsigned long long>(programEndCounters.readyToFinalize),
                static_cast<unsigned long long>(programEndCounters.finalized),
                static_cast<unsigned long long>(programEndCounters.cancelled),
                static_cast<unsigned long long>(programEndCounters.failClosed),
                static_cast<unsigned long long>(programEndCounters.epochMismatch),
                static_cast<unsigned long long>(programEndCounters.ownerLeaseLost),
                static_cast<unsigned long long>(programEndCounters.integrityFailure));

            RtPrintf(
                "[NC02G-WAIT] Blocks:%llu AxisQ:%llu AxisR:%llu CmdQ:%llu "
                "Ingress:%llu Replay:%llu Notice:%llu Feedback:%llu FbSeq:%llu "
                "Callback:%llu Binding:%llu Safety:%llu Standstill:%llu Stable:%llu\n",
                static_cast<unsigned long long>(programEndCounters.waitActiveBlocks),
                static_cast<unsigned long long>(programEndCounters.waitAxisCommand),
                static_cast<unsigned long long>(programEndCounters.waitAxisResult),
                static_cast<unsigned long long>(programEndCounters.waitCommandQueue),
                static_cast<unsigned long long>(programEndCounters.waitCommandIngress),
                static_cast<unsigned long long>(programEndCounters.waitCommandReplay),
                static_cast<unsigned long long>(programEndCounters.waitFeedbackNotice),
                static_cast<unsigned long long>(programEndCounters.waitFeedback),
                static_cast<unsigned long long>(programEndCounters.waitFeedbackSequence),
                static_cast<unsigned long long>(programEndCounters.waitCallback),
                static_cast<unsigned long long>(programEndCounters.waitCompletionBinding),
                static_cast<unsigned long long>(programEndCounters.waitSafetyRequest),
                static_cast<unsigned long long>(programEndCounters.waitGroupStandstill),
                static_cast<unsigned long long>(programEndCounters.waitStableConfirmation));


            RtPrintf(
                "[NC02H-TXN] Seq:%llu D:%llu Phase:%s Post:%s Active:%u "
                "GReq:%u GDone:%u MReq:%u MDone:%u PC:%d Line:%d "
                "M:%d P:%d L:%d Main:%u Applied:%u\n",
                static_cast<unsigned long long>(gmTransactionSnapshot.sequence),
                static_cast<unsigned long long>(gmTransactionSnapshot.dispatchId),
                NCGMBlockTransactionPhaseToDiagnosticName(
                    gmTransactionSnapshot.phase),
                NCGMBlockPostActionToDiagnosticName(
                    gmTransactionSnapshot.postAction),
                gmTransactionSnapshot.active ? 1U : 0U,
                gmTransactionSnapshot.gWaitRequired ? 1U : 0U,
                gmTransactionSnapshot.gWaitComplete ? 1U : 0U,
                gmTransactionSnapshot.mWaitRequired ? 1U : 0U,
                gmTransactionSnapshot.mWaitComplete ? 1U : 0U,
                gmTransactionSnapshot.sourcePC,
                gmTransactionSnapshot.sourceLineNumber,
                gmTransactionSnapshot.mCode,
                gmTransactionSnapshot.pValue,
                gmTransactionSnapshot.repeatCount,
                gmTransactionSnapshot.fromMainProgram ? 1U : 0U,
                gmTransactionSnapshot.postActionApplied ? 1U : 0U);

            RtPrintf(
                "[NC02H-CNT] Start:%llu GComp:%llu MComp:%llu Dual:%llu "
                "Eval:%llu GWait:%llu MWait:%llu Ready:%llu Final:%llu "
                "Cancel:%llu Fail:%llu M00:%llu M01:%llu M98:%llu "
                "M99:%llu M02:%llu M30:%llu\n",
                static_cast<unsigned long long>(gmTransactionCounters.started),
                static_cast<unsigned long long>(gmTransactionCounters.gWaitComponents),
                static_cast<unsigned long long>(gmTransactionCounters.mWaitComponents),
                static_cast<unsigned long long>(gmTransactionCounters.dualComponentTransactions),
                static_cast<unsigned long long>(gmTransactionCounters.evaluations),
                static_cast<unsigned long long>(gmTransactionCounters.gWaitSamples),
                static_cast<unsigned long long>(gmTransactionCounters.mWaitSamples),
                static_cast<unsigned long long>(gmTransactionCounters.readyTransitions),
                static_cast<unsigned long long>(gmTransactionCounters.finalized),
                static_cast<unsigned long long>(gmTransactionCounters.cancelled),
                static_cast<unsigned long long>(gmTransactionCounters.finalizeFailed),
                static_cast<unsigned long long>(gmTransactionCounters.m00Stops),
                static_cast<unsigned long long>(gmTransactionCounters.m01Stops),
                static_cast<unsigned long long>(gmTransactionCounters.m98Calls),
                static_cast<unsigned long long>(gmTransactionCounters.m99Returns),
                static_cast<unsigned long long>(gmTransactionCounters.m02Ends),
                static_cast<unsigned long long>(gmTransactionCounters.m30Ends));

            RtPrintf(
                "[NC02I-SB] Seq:%llu D:%llu Phase:%s Decision:%s Kind:%s "
                "Active:%u PC:%d Line:%d Commit:%u Cb:%u/%u Txn:%u/%u "
                "Motion:%s Done:%u Fail:%u Ready:%u Pause:%u Hold:%u EndSup:%u\n",
                static_cast<unsigned long long>(singleBlockShadowSnapshot.sequence),
                static_cast<unsigned long long>(singleBlockShadowSnapshot.dispatchId),
                NCSingleBlockShadowPhaseToDiagnosticName(
                    singleBlockShadowSnapshot.phase),
                NCSingleBlockShadowDecisionToDiagnosticName(
                    singleBlockShadowSnapshot.decision),
                NCSingleBlockCandidateKindToDiagnosticName(
                    singleBlockShadowSnapshot.candidateKind),
                singleBlockShadowSnapshot.active ? 1U : 0U,
                singleBlockShadowSnapshot.sourcePC,
                singleBlockShadowSnapshot.sourceLineNumber,
                singleBlockShadowSnapshot.programCommitted ? 1U : 0U,
                singleBlockShadowSnapshot.callbackComplete ? 1U : 0U,
                singleBlockShadowSnapshot.callbackRequired ? 1U : 0U,
                singleBlockShadowSnapshot.transactionComplete ? 1U : 0U,
                singleBlockShadowSnapshot.transactionRequired ? 1U : 0U,
                NCBlockMotionBoundaryStateToDiagnosticName(
                    singleBlockShadowSnapshot.motionState),
                singleBlockShadowSnapshot.motionComplete ? 1U : 0U,
                singleBlockShadowSnapshot.motionFailed ? 1U : 0U,
                singleBlockShadowSnapshot.boundaryReady ? 1U : 0U,
                singleBlockShadowSnapshot.legacyPausePending ? 1U : 0U,
                singleBlockShadowSnapshot.legacyHoldObserved ? 1U : 0U,
                singleBlockShadowSnapshot.programEndSuppressed ? 1U : 0U);

            RtPrintf(
                "[NC02I-CNT] ArmTry:%llu Armed:%llu NotEligible:%llu Eval:%llu "
                "WaitLife:%llu WaitCommit:%llu WaitTxn:%llu WaitCb:%llu "
                "WaitMotion:%llu Ready:%llu LegacyHold:%llu Agree:%llu "
                "Early:%llu Phantom:%llu Missing:%llu MotionFail:%llu "
                "Overflow:%llu TxnFail:%llu EndSup:%llu Resume:%llu "
                "Cancel:%llu Supersede:%llu\n",
                static_cast<unsigned long long>(singleBlockShadowCounters.armAttempts),
                static_cast<unsigned long long>(singleBlockShadowCounters.armed),
                static_cast<unsigned long long>(singleBlockShadowCounters.notEligible),
                static_cast<unsigned long long>(singleBlockShadowCounters.evaluations),
                static_cast<unsigned long long>(singleBlockShadowCounters.waitLifecycle),
                static_cast<unsigned long long>(singleBlockShadowCounters.waitProgramCommit),
                static_cast<unsigned long long>(singleBlockShadowCounters.waitTransaction),
                static_cast<unsigned long long>(singleBlockShadowCounters.waitCallback),
                static_cast<unsigned long long>(singleBlockShadowCounters.waitMotion),
                static_cast<unsigned long long>(singleBlockShadowCounters.boundaryReady),
                static_cast<unsigned long long>(singleBlockShadowCounters.legacyHolds),
                static_cast<unsigned long long>(singleBlockShadowCounters.agreeHolds),
                static_cast<unsigned long long>(singleBlockShadowCounters.legacyEarlyHolds),
                static_cast<unsigned long long>(singleBlockShadowCounters.legacyHoldWithoutArm),
                static_cast<unsigned long long>(singleBlockShadowCounters.legacyMissingHold),
                static_cast<unsigned long long>(singleBlockShadowCounters.motionFailures),
                static_cast<unsigned long long>(singleBlockShadowCounters.trackingOverflow),
                static_cast<unsigned long long>(singleBlockShadowCounters.transactionFailures),
                static_cast<unsigned long long>(singleBlockShadowCounters.programEndSuppressed),
                static_cast<unsigned long long>(singleBlockShadowCounters.resumed),
                static_cast<unsigned long long>(singleBlockShadowCounters.cancelled),
                static_cast<unsigned long long>(singleBlockShadowCounters.superseded));

            RtPrintf(
                "[NC02I-FH] Seq:%llu Src:%s Phase:%s Decision:%s Active:%u "
                "PC:%d D:%llu Legacy:%u Ack:%u Stable:%u/%u "
                "Ovr:%.3f CmdStop:%u ActStop:%u HomePause:%u "
                "CmdMax:%.0f ActMax:%.0f Q:%u/%u/%u "
                "Owner:%s/%u->%s/%u Epoch:%llu->%llu Fail:%u\n",
                static_cast<unsigned long long>(feedHoldSnapshot.sequence),
                NCFeedHoldSourceToDiagnosticName(feedHoldSnapshot.source),
                NCFeedHoldShadowPhaseToDiagnosticName(feedHoldSnapshot.phase),
                NCFeedHoldShadowDecisionToDiagnosticName(feedHoldSnapshot.decision),
                feedHoldSnapshot.active ? 1U : 0U,
                feedHoldSnapshot.requestPC,
                static_cast<unsigned long long>(feedHoldSnapshot.dispatchId),
                feedHoldSnapshot.legacyHoldEntered ? 1U : 0U,
                feedHoldSnapshot.acknowledged ? 1U : 0U,
                static_cast<unsigned int>(feedHoldSnapshot.stableSamples),
                static_cast<unsigned int>(feedHoldSnapshot.requiredStableSamples),
                feedHoldSnapshot.motion.feedrateOverride,
                feedHoldSnapshot.motion.commandStopped ? 1U : 0U,
                feedHoldSnapshot.motion.actualStopped ? 1U : 0U,
                feedHoldSnapshot.homePaused ? 1U : 0U,
                feedHoldSnapshot.motion.maxAxisCommandVelocityPps,
                feedHoldSnapshot.motion.maxAxisActualVelocityPps,
                static_cast<unsigned int>(feedHoldSnapshot.motion.commandQueueDepth),
                static_cast<unsigned int>(feedHoldSnapshot.motion.commandIngressDepth),
                static_cast<unsigned int>(feedHoldSnapshot.motion.commandReplayDepth),
                MotionOwnerToDiagnosticName(feedHoldSnapshot.requestOwner),
                static_cast<unsigned int>(feedHoldSnapshot.requestOwnerGeneration),
                MotionOwnerToDiagnosticName(feedHoldSnapshot.currentOwner),
                static_cast<unsigned int>(feedHoldSnapshot.currentOwnerGeneration),
                static_cast<unsigned long long>(feedHoldSnapshot.requestExecutionEpoch),
                static_cast<unsigned long long>(feedHoldSnapshot.currentExecutionEpoch),
                feedHoldSnapshot.failed ? 1U : 0U);

            RtPrintf(
                "[NC02I-FCNT] ReqTry:%llu Req:%llu Prog:%llu Home:%llu Eval:%llu "
                "WaitOvr:%llu WaitCmd:%llu WaitAct:%llu WaitHome:%llu WaitStable:%llu "
                "Ack:%llu Legacy:%llu Early:%llu Agree:%llu "
                "ResumeReq:%llu ResumeEarly:%llu ResumeAck:%llu Resumed:%llu "
                "OwnerFail:%llu EpochFail:%llu MotionFail:%llu AckLost:%llu "
                "Cancel:%llu Super:%llu\n",
                static_cast<unsigned long long>(feedHoldCounters.requestAttempts),
                static_cast<unsigned long long>(feedHoldCounters.requestsLatched),
                static_cast<unsigned long long>(feedHoldCounters.programRequests),
                static_cast<unsigned long long>(feedHoldCounters.homeRequests),
                static_cast<unsigned long long>(feedHoldCounters.evaluations),
                static_cast<unsigned long long>(feedHoldCounters.waitOverrideZero),
                static_cast<unsigned long long>(feedHoldCounters.waitCommandStop),
                static_cast<unsigned long long>(feedHoldCounters.waitActualStop),
                static_cast<unsigned long long>(feedHoldCounters.waitHomePaused),
                static_cast<unsigned long long>(feedHoldCounters.waitStable),
                static_cast<unsigned long long>(feedHoldCounters.acknowledged),
                static_cast<unsigned long long>(feedHoldCounters.legacyHolds),
                static_cast<unsigned long long>(feedHoldCounters.legacyEarlyHolds),
                static_cast<unsigned long long>(feedHoldCounters.legacyAgreeHolds),
                static_cast<unsigned long long>(feedHoldCounters.resumeRequests),
                static_cast<unsigned long long>(feedHoldCounters.resumeBeforeAcknowledge),
                static_cast<unsigned long long>(feedHoldCounters.resumeAfterAcknowledge),
                static_cast<unsigned long long>(feedHoldCounters.resumed),
                static_cast<unsigned long long>(feedHoldCounters.ownerChanged),
                static_cast<unsigned long long>(feedHoldCounters.executionEpochChanged),
                static_cast<unsigned long long>(feedHoldCounters.motionFault),
                static_cast<unsigned long long>(feedHoldCounters.acknowledgeLost),
                static_cast<unsigned long long>(feedHoldCounters.cancelled),
                static_cast<unsigned long long>(feedHoldCounters.superseded));
        }

        if (startupSamples < 15U)
        {
            ++startupSamples;
        }

        previousOwner = ownerLease.owner;
        previousGeneration = ownerLease.generation;
        previousTransportErrorTotal = transportErrorTotal;
        previousLifecycleChangeToken = lifecycleChangeToken;
        previousCompletionBoundaryChangeToken =
            completionBoundaryChangeToken;
        previousProgramEndChangeToken = programEndChangeToken;
        previousGMTransactionChangeToken = gmTransactionChangeToken;
        previousSingleBlockShadowChangeToken =
            singleBlockShadowChangeToken;
        previousFeedHoldChangeToken =
            feedHoldChangeToken;
    }
}
