// ==========================================================
// 檔案：NCManager_PLCSync.cpp
// 功能：處理 NCManager 將大腦與軸狀態同步給 PLC 的 S 點
// ==========================================================

#include "NCManager.h"
#include "MotionCore.h"
#include "PLCManager.h"
#include "NCPLCMap.h"
#include "AlarmManager.h"
#include <rtapi.h>

extern PLCManager* g_PLC; // 引入 PLC 大腦

void NCManager::SyncNCStateToPLC()
{
    if (g_PLC == nullptr) return; // 防呆

    // =========================================================
    // 1. 機台綜合狀態 (S0 ~ S5)
    // =========================================================
    g_PLC->Set_S(0, (m_edmState == EDMState::READY || m_edmState == EDMState::START)); // 系統備妥
    g_PLC->Set_S(1, (m_edmState == EDMState::START)); // 執行中
    g_PLC->Set_S(2, (m_edmState == EDMState::HOLD));  // 暫停中
    g_PLC->Set_S(3, (m_state == NCState::P_END));     // 程式結束
    g_PLC->Set_S(4, (m_state == NCState::RESET_STATE)); // 系統重置中
    g_PLC->Set_S(5, AlarmManager::GetInstance().HasAlarm()); // 系統警報中

    // =========================================================
    // 2. 操作模式狀態 (S10 ~ S13)
    // =========================================================
    g_PLC->Set_S(10, (m_mode == NCOperationMode::MEMORY));
    g_PLC->Set_S(11, (m_mode == NCOperationMode::MDI));
    g_PLC->Set_S(12, (m_mode == NCOperationMode::MANUAL));
    g_PLC->Set_S(13, (m_mode == NCOperationMode::EDIT));

    // =========================================================
    // 3. NC 功能啟用狀態 (S20 ~ S26) - 這些您可以之後再接上 UI 變數
    // =========================================================
    g_PLC->Set_S(20, m_isSingleBlockEnabled);
    // g_PLC->Set_S(21, m_isDryRunEnabled);
    g_PLC->Set_S(22, m_isOptionalStopEnabled);
    g_PLC->Set_S(23, m_isBlockSkipEnabled);
    // g_PLC->Set_S(24, m_isMachineLockEnabled);
    // g_PLC->Set_S(25, m_isZAxisLockEnabled);
    // g_PLC->Set_S(26, false); // 預留

    // =========================================================
    // 4. M/S/T 碼執行狀態 (S30 ~ S34)
    // =========================================================
    // 注意：目前 M/S/T 碼由 GCodeHandlers 處理，如果有 M 碼卡住等待 (FIN)，
    // 我們可以透過檢查 m_waitCallback 是否有值來簡單判斷。
    // 如果未來您有專門的 M-Code Manager，可以直接讀取它的狀態。
    bool isWaitingForMCode = (m_waitCallback != nullptr);
    g_PLC->Set_S(30, isWaitingForMCode); // M 碼等待中

    // (M00, M01 這些狀態需要您在 GCodeHandlers 處理 M00 時設定一個旗標來讀取)
    // g_PLC->Set_S(33, m_isM00Active);
    // g_PLC->Set_S(34, m_isM01Active);

    // =========================================================
    // 5. G81 HOME 全域執行狀態
    //
    // S203 = HOME Request 還存在
    // S204 = Feed Hold 正在 Controlled Stop
    // S205 = 已完全停止，等待 Cycle Start Resume
    // =========================================================

    g_PLC->Set_S(
        NCPLC::S::HOME_ACTIVE,
        Homing.IsActive());

    g_PLC->Set_S(
        NCPLC::S::HOME_HOLD_DECEL,
        Homing.IsHoldDecelerating());

    g_PLC->Set_S(
        NCPLC::S::HOME_PAUSED,
        Homing.IsPaused());


    // =========================================================
    // 6. 各軸獨立狀態 (S100 ~ S157)
    // =========================================================
    for (int i = 0; i < 8; i++)
    {
        auto& axis = m_motion.GetAxisContext(i);

        // 只有該軸存在時才更新，否則填 0
        if (axis.isExist)
        {
            g_PLC->Set_S(100 + i, axis.isServoOn);      // S100~107: 軸啟用狀態
            g_PLC->Set_S(110 + i, axis.isHomed);        // S110~117: 尋原點完成 (需要您在 AxisContext 加這變數)
            g_PLC->Set_S(150 + i, axis.isLagAlarm);     // S150~157: 追隨誤差錯誤

            const HomeState homeState = axis.homeRuntime.state;
            const bool homeSearchSwitch = axis.homeRuntime.active &&
                (homeState == HomeState::SEARCH_SWITCH || homeState == HomeState::SWITCH_DECEL_STOP || homeState == HomeState::BACK_OFF || homeState == HomeState::VALIDATE_RELEASE || homeState == HomeState::WAIT_GROUP_BACKOFF);
            const bool homeSearchIndex = axis.homeRuntime.active &&
                (homeState == HomeState::ARM_REFERENCE || homeState == HomeState::SEARCH_INDEX || homeState == HomeState::INDEX_CAPTURED || homeState == HomeState::INDEX_DECEL_STOP || homeState == HomeState::WAIT_GROUP_INDEX_STOP);

            g_PLC->Set_S(NCPLC::S::AxisPoint(NCPLC::S::HOME_SEARCH_DOG_BASE, i), homeSearchSwitch);
            g_PLC->Set_S(NCPLC::S::AxisPoint(NCPLC::S::HOME_SEARCH_INDEX_BASE, i), homeSearchIndex);

            // =====================================================
            // Software Travel Limit Warning State
            //
            // S210~217 = +Software Limit
            // S220~227 = -Software Limit
            //
            // 只提供 PLC / HMI 顯示警訊。
            // 不進 AlarmManager。
            // =====================================================

            const bool softwarePositiveLimit =
                axis.travelLimit1PositiveActive ||
                axis.travelLimit2PositiveActive ||
                axis.travelLimit3PositiveActive;

            const bool softwareNegativeLimit =
                axis.travelLimit1NegativeActive ||
                axis.travelLimit2NegativeActive ||
                axis.travelLimit3NegativeActive;

            g_PLC->Set_S(
                NCPLC::S::AxisPoint(
                    NCPLC::S::SOFTWARE_POSITIVE_LIMIT_BASE,
                    i),
                softwarePositiveLimit);

            g_PLC->Set_S(
                NCPLC::S::AxisPoint(
                    NCPLC::S::SOFTWARE_NEGATIVE_LIMIT_BASE,
                    i),
                softwareNegativeLimit);
        }
        else
        {
            g_PLC->Set_S(100 + i, false);
            g_PLC->Set_S(110 + i, false);
            g_PLC->Set_S(150 + i, false);
            g_PLC->Set_S(NCPLC::S::AxisPoint(NCPLC::S::HOME_SEARCH_DOG_BASE, i), false);
            g_PLC->Set_S(NCPLC::S::AxisPoint(NCPLC::S::HOME_SEARCH_INDEX_BASE, i), false);

            g_PLC->Set_S(
                NCPLC::S::AxisPoint(
                    NCPLC::S::SOFTWARE_POSITIVE_LIMIT_BASE,
                    i),
                false);

            g_PLC->Set_S(
                NCPLC::S::AxisPoint(
                    NCPLC::S::SOFTWARE_NEGATIVE_LIMIT_BASE,
                    i),
                false);
        }
    }
}
// EDM137: these points are a coherent NC-owned image. A diagnostic fixture
// may move all six axes but never admits the discharge power supply.
void NCManager::ForceEDMPLCOutputsOffSameThread(const char* reason) noexcept
{
    const auto old=m_edm137PLC.Current();
    const auto session=m_edm137PLC.Session();
    m_edm137PLC.Clear();
    if(g_PLC)g_PLC->PublishEDMOutputs(0U);
    if(old.word||old.requested)
        RtPrintf("[EDM137] event=PLC_OUTPUT session=%llu reason=%s S80=0 S92=0 S93=0 requested=0 inhibited=1 production=0\n",
            static_cast<unsigned long long>(session),reason?reason:"STOP");
}

void NCManager::SyncEDMPLCOutputsSameThread() noexcept
{
    EDM137PLC::Input input{};
    const auto& s=m_edm94;
    const auto& guard=s.feedback;
    const auto& rows=m_edm116;
    const auto executionEpoch=m_motion.GetCurrentExecutionEpoch();
    // armEpoch identifies the frozen session. Each accepted G01/G02/G03
    // advances executionEpoch through the NC-authorized row ledger.
    const bool rowCounts=rows.consumed<=16U&&rows.acknowledgedRows<=rows.consumed&&
        rows.consumed-rows.acknowledgedRows<=1U;
    const auto issuedEpoch=rowCounts&&rows.consumed?rows.issuedEpoch[rows.consumed-1U]:s.armEpoch;
    const auto previousEpoch=rowCounts&&rows.consumed>1U?rows.issuedEpoch[rows.consumed-2U]:s.armEpoch;
    const bool issued=rowCounts&&(!rows.consumed||
        (rows.issuedDispatch[rows.consumed-1U]&&issuedEpoch&&previousEpoch&&
            issuedEpoch==(previousEpoch==0xFFFFFFFFULL?1ULL:previousEpoch+1ULL)));
    const bool handoff=issued&&rows.consumed>rows.acknowledgedRows;
    const bool epochCurrent=issued&&(executionEpoch==issuedEpoch||
        (handoff&&executionEpoch==previousEpoch));
    // Only the explicit new fixture binds E12. Earlier diagnostics and all
    // ordinary CNC programs retain an all-OFF image; RUN alone is no permit.
    const bool fixture=s.gapHistoryArm==202U||s.gapHistoryArm==204U||s.gapHistoryArm==206U||s.gapHistoryArm==208U;
    const bool live=fixture&&m_state==NCState::RUN&&m_mode==NCOperationMode::MEMORY&&
        !Close_System_Com_flag&&m_externalReadyInterlock&&!AlarmManager::GetInstance().HasAlarm()&&
        m_resetContinuationPhase==ResetContinuationPhase::IDLE&&s.latched&&s.ready&&s.everArmed&&
        !s.armPending&&!s.complete&&!s.cancelled&&!s.resetSeen&&!s.resetCommitted&&
        !s.releaseRequested&&!s.rtFaultReported&&s.gapWorkTime&&m_edm130.bound&&
        s.run==m_pathCoreLiveBookkeeping.currentRunToken&&s.cache==GetBaseProgramCache().GetGeneration()&&
        EDM130::HasBoundFlushStages(m_edm130.recipe)&&EDM130::ValidRecipe(m_edm130.recipe)&&
        epochCurrent&&m_motion.IsMotionOwnerLeaseCurrent(s.lease)&&
        !m_motion.HasPendingSafetyOrRecoveryRequests()&&guard.lastTick&&
        guard.identity.session==s.session&&guard.identity.epoch==s.armEpoch&&
        guard.identity.ownerGeneration==s.lease.generation&&guard.active&&guard.armAccepted&&
        !guard.cancelled&&!guard.faulted&&!guard.released;
    EDM128::Phase phase=EDM128::Phase::Gap;
    if(live) {
        input.session=s.session;input.epoch=s.armEpoch;input.owner=s.lease.generation;
        input.recipe=m_edm130.recipe.generation;input.admitted=true;
        input.polarity=EDM130::FrozenPolarity(m_edm130.recipe.flushStages)!=0U;
        EDM128::MotionFeedback mode{};
        const bool read=m_motion.ReadEDM130Feedback(mode);
        const bool timely=mode.tick&&guard.lastTick&&
            (mode.tick>=guard.lastTick||guard.lastTick-mode.tick<=4ULL);
        const bool receiptRows=mode.authorizedRows<=rows.consumed&&
            rows.consumed-mode.authorizedRows<=1U&&mode.completedRows<=mode.authorizedRows&&
            mode.authorizedRows-mode.completedRows<=1U;
        const auto receiptEpoch=receiptRows&&mode.authorizedRows?
            rows.issuedEpoch[mode.authorizedRows-1U]:s.armEpoch;
        const bool receiptHandoff=handoff&&receiptRows&&mode.authorizedRows+1U==rows.consumed&&
            mode.completedRows==mode.authorizedRows&&mode.coordinator.phase==EDM128::Phase::Gap;
        const bool receiptIdentity=receiptRows&&mode.liveEpoch==receiptEpoch&&
            (mode.authorizedRows==rows.consumed||receiptHandoff);
        const bool validReceipt=read&&receiptIdentity&&mode.enabled&&mode.active&&!mode.faulted&&
            mode.physicalAd&&mode.workTimeFlush&&mode.fixture==((s.gapHistoryArm==204U||s.gapHistoryArm==208U)?1U:0U)&&mode.session==s.session&&mode.armEpoch==s.armEpoch&&
            mode.ownerGeneration==s.lease.generation&&guard.identity.session==s.session&&
            guard.identity.epoch==s.armEpoch&&guard.identity.ownerGeneration==s.lease.generation&&
            guard.active&&guard.armAccepted&&!guard.cancelled&&!guard.faulted&&!guard.released&&
            EDM130::SameRecipe(mode.timerRecipe,m_edm130.recipe)&&!mode.coordinator.faulted&&
            !mode.coordinator.stopped&&mode.authorizedRows<=16U&&mode.completedRows<=mode.authorizedRows;
        // Authorization, epoch publication and RT feedback are separate
        // bounded publications. During that one recorded handoff keep the
        // last coherent image; a foreign or unissued epoch still retires it.
        input.unavailable=!read||(validReceipt&&(!timely||mode.liveEpoch!=executionEpoch));
        const bool current=validReceipt&&timely&&mode.liveEpoch==executionEpoch;
        if(!current&&!input.unavailable)input.stop=true;
        if(current) {
            input.machiningAccepted=mode.authorizedRows!=0U;
            phase=mode.coordinator.phase;
            switch(phase) {
            case EDM128::Phase::Brake:case EDM128::Phase::AnchorDwell:
            case EDM128::Phase::Outbound:case EDM128::Phase::TopDwell:
            case EDM128::Phase::Returning:case EDM128::Phase::ReturnDwell:
            case EDM128::Phase::Resume:input.flushActive=true;break;
            case EDM128::Phase::Gap:break;
            default:input.stop=true;break;
            }
        }
    }
    const auto previous=m_edm137PLC.Current();
    const auto previousSession=m_edm137PLC.Session();
    const auto output=m_edm137PLC.Observe(input);
    // The PLC publication masks S92 again even if an unexpected caller were
    // to corrupt a logical request. No hardware discharge enable exists here.
    if(g_PLC)g_PLC->PublishEDMOutputs(output.word);
    if(previous.word!=output.word||previous.requested!=output.requested||
        previousSession!=m_edm137PLC.Session())
        RtPrintf("[EDM137] event=PLC_OUTPUT session=%llu phase=%s S80=%u S92=%u S93=%u requested=%u inhibited=1 production=0\n",
            static_cast<unsigned long long>(input.session?input.session:previousSession),
            input.unavailable?"UNAVAILABLE":EDM128::PhaseName(phase),
            (output.word&EDM137PLC::FlushBit)?1U:0U,(output.word&EDM137PLC::DischargeBit)?1U:0U,
            (output.word&EDM137PLC::PolarityBit)?1U:0U,output.requested?1U:0U);
}
