// ==========================================================
// 檔案：NCManager_PLCSync.cpp
// 功能：處理 NCManager 將大腦與軸狀態同步給 PLC 的 S 點
// ==========================================================

#include "NCManager.h"
#include "MotionCore.h"
#include "PLCManager.h"
#include "NCPLCMap.h"
#include "NCSpindleCodes.h"
#include "EDM149VoltageMotion.h"
#include "AlarmManager.h"
#include <windows.h> // Explicit Windows SDK types before RTX SDK declarations.
#include <rtapi.h>

extern PLCManager* g_PLC; // 引入 PLC 大腦

// EDM282: only the owning NC scan may mutate this transaction.
bool NCManager::BeginAuxiliaryHandshakeSameThread(int tCode, int mCode) noexcept
{
    // EDM288: no direct helper call may bypass spindle ownership through PLC.
    if(NCSpindleCodes::IsReserved(mCode))return false;
    return m_auxiliaryHandshake.Begin(tCode, mCode);
}

bool NCManager::PollAuxiliaryHandshakeSameThread() noexcept
{
    return m_auxiliaryHandshake.IsComplete();
}

void NCManager::ServiceAuxiliaryHandshakeSameThread(bool allowDispatch) noexcept
{
    const auto state = m_state.load(std::memory_order_acquire);
    if (AlarmManager::GetInstance().HasAlarm() || state == NCState::ALARM ||
        state == NCState::RESET_STATE)
    {
        ResetAuxiliaryHandshakeSameThread();
        return; // Cancellation dominates an ACK sampled in this same scan.
    }
    if (state != NCState::RUN && state != NCState::HOLD)
    {
        if (m_auxiliaryHandshake.IsPending()) CancelAuxiliaryHandshakeSameThread();
        return;
    }
    // Auxiliary requests obey the same program ownership and safety fence
    // as motion admission. An existing request may still collect its ACK.
    allowDispatch = allowDispatch && state == NCState::RUN &&
        m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease) &&
        !m_motion.HasPendingSafetyOrRecoveryRequests();
    if (g_PLC == nullptr) return; // Missing PLC must never auto-complete a command.
    const auto beforeKind = m_auxiliaryHandshake.ActiveKind();
    const int beforeCode = m_auxiliaryHandshake.ActiveCode();
    const bool fin = g_PLC->Get_C(NCPLC::C::AUX_FIN);
    m_auxiliaryHandshake.Service(fin, allowDispatch);
    const bool acceptedFin = beforeKind != NCAuxiliaryHandshake::Kind::None &&
        fin && !m_auxiliaryHandshake.IsActive();
    const bool acceptedFinalFin = acceptedFin && m_auxiliaryHandshake.IsComplete();
    SyncAuxiliaryHandshakeToPLCSameThread();
    const auto afterKind = m_auxiliaryHandshake.ActiveKind();
    const auto afterState = m_state.load(std::memory_order_acquire);
    if (acceptedFinalFin &&
        m_auxiliaryHandshake.CurrentPhase() == NCAuxiliaryHandshake::Phase::Idle)
    {
        // Sync has revoked the final ACK; its pending-only CANCEL log did not run.
        RtPrintf("[AUX] build=EDM282 event=CANCEL reason=FIN_REVOKED S51=0 S52=0\n");
    }
    else if (acceptedFin && afterKind == NCAuxiliaryHandshake::Kind::None &&
        m_auxiliaryHandshake.CurrentPhase() != NCAuxiliaryHandshake::Phase::Idle &&
        afterState != NCState::ALARM && afterState != NCState::RESET_STATE &&
        !AlarmManager::GetInstance().HasAlarm())
    {
        RtPrintf("[AUX] build=EDM282 event=FIN kind=%s code=%d C10=1 S51=0 S52=0\n",
            beforeKind == NCAuxiliaryHandshake::Kind::Tool ? "T" : "M", beforeCode);
    }
    else if (beforeKind == NCAuxiliaryHandshake::Kind::None &&
        afterKind != NCAuxiliaryHandshake::Kind::None)
    {
        RtPrintf("[AUX] build=EDM282 event=REQUEST kind=%s code=%d R1=%d R2=%d S51=%u S52=%u C10=0\n",
            afterKind == NCAuxiliaryHandshake::Kind::Tool ? "T" : "M",
            m_auxiliaryHandshake.ActiveCode(), m_auxiliaryHandshake.LastM(),
            m_auxiliaryHandshake.LastT(), m_auxiliaryHandshake.IsMActive() ? 1U : 0U,
            m_auxiliaryHandshake.IsTActive() ? 1U : 0U);
    }
}

void NCManager::CancelAuxiliaryHandshakeSameThread() noexcept
{
    const bool pending = m_auxiliaryHandshake.IsPending();
    m_auxiliaryHandshake.Cancel();
    SyncAuxiliaryHandshakeToPLCSameThread();
    if (pending)
        RtPrintf("[AUX] build=EDM282 event=CANCEL retainCodes=1 S51=0 S52=0\n");
}

void NCManager::ResetAuxiliaryHandshakeSameThread() noexcept
{
    const bool pending = m_auxiliaryHandshake.IsPending();
    m_auxiliaryHandshake.Reset();
    SyncAuxiliaryHandshakeToPLCSameThread();
    if (pending)
        RtPrintf("[AUX] build=EDM282 event=CANCEL R1=-1 R2=-1 S51=0 S52=0\n");
}

void NCManager::SyncAuxiliaryHandshakeToPLCSameThread() noexcept
{
    // This method also runs on the ProcessTask exit guard: a late alarm or
    // state change must revoke a request before PLC can use it next scan.
    const auto state = m_state.load(std::memory_order_acquire);
    const bool clearCodes = AlarmManager::GetInstance().HasAlarm() ||
        state == NCState::ALARM || state == NCState::RESET_STATE;
    const bool cancelPending = state != NCState::RUN && state != NCState::HOLD &&
        m_auxiliaryHandshake.IsPending();
    const bool pending = m_auxiliaryHandshake.IsPending();
    if (clearCodes) m_auxiliaryHandshake.Reset();
    else if (cancelPending) m_auxiliaryHandshake.Cancel();
    if (pending && (clearCodes || cancelPending))
        RtPrintf("[AUX] build=EDM282 event=CANCEL reason=STATE clearCodes=%u S51=0 S52=0\n",
            clearCodes ? 1U : 0U);
    if (g_PLC == nullptr) return;
    const bool mActive = m_auxiliaryHandshake.IsMActive();
    const bool tActive = m_auxiliaryHandshake.IsTActive();
    // Release-publish data and request together through the NC-owned PLC view.
    const unsigned requestWord = (mActive ? 1U : 0U) | (tActive ? 2U : 0U);
    g_PLC->PublishAuxiliaryOutputs(m_auxiliaryHandshake.LastM(),
        m_auxiliaryHandshake.LastT(), requestWord);
    g_PLC->Set_S(NCPLC::S::AUX_REQUEST, mActive || tActive);
    g_PLC->Set_S(NCPLC::S::AUX_WAIT_FIN, mActive || tActive);
}

// S206 is an NC-owned level indication, never a motion or discharge permit.
void NCManager::SyncG80ActiveToPLCSameThread() noexcept
{
    if(g_PLC==nullptr)return;
    auto& s=m_g80;
    const bool active=IsG80ActiveSameThread();
    const bool previous=g_PLC->Get_S(NCPLC::S::G80_ACTIVE);
    g_PLC->Set_S(NCPLC::S::G80_ACTIVE,active);
    const auto state=m_state.load(std::memory_order_acquire);
    const unsigned nc=static_cast<unsigned>(state);
    const bool stateChanged=s.plcDiagnosticInitialized&&
        (nc!=s.plcDiagnosticState||s.feedback.held!=s.plcDiagnosticHeld);
    const bool initial=s.session&&!s.plcDiagnosticInitialized&&(active||s.submitted);
    const bool terminal=previous&&!active&&!s.plcTerminalLogged;
    const bool report=previous!=active||initial||(s.session&&s.submitted&&stateChanged);
    s.plcDiagnosticInitialized=true;s.plcDiagnosticState=nc;s.plcDiagnosticHeld=s.feedback.held;
    if(!report)return;
    // Bound HOLD/state chatter per session; retain its one final S206 OFF event.
    if(s.plcDiagnosticEvents>=128U&&!terminal){
        if(s.plcDiagnosticEvents==128U){
            ++s.plcDiagnosticEvents;
            RtPrintf("[G80] build=EDM278 event=PLC_EVENT_LIMIT session=%llu limit=128\n",
                static_cast<unsigned long long>(s.session));
        }
        return;
    }
    ++s.plcDiagnosticEvents;if(terminal)s.plcTerminalLogged=true;
    const char* reason=s.cancelReason[0]?s.cancelReason:
        (state==NCState::RESET_STATE?"RESET_STATE":
        (state==NCState::HOLD||s.feedback.held?"HOLD":
        (s.success?"COMPLETE":
        (s.ncFault||s.runtimeFault||s.feedback.fault!=G80::Fault::None?"FAULT":
        (active?(stateChanged?"RESUME_OR_STATE":"ACTIVE"):"INACTIVE")))));
    RtPrintf("[G80] build=EDM278 event=PLC_ACTIVE S206=%u previous=%u session=%llu reason=%s nc=%u phase=%u held=%u slowCount=%u activeLevel=1 motionPermit=0\n",
        active?1U:0U,previous?1U:0U,static_cast<unsigned long long>(s.session),reason,nc,
        static_cast<unsigned>(s.feedback.phase),s.feedback.held?1U:0U,s.feedback.slowCount);
}

void NCManager::SyncNCStateToPLC()
{
    if (g_PLC == nullptr) return; // 防呆

    SyncG80ActiveToPLCSameThread(); // Explicit ON/OFF each NC status scan.

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

    // EDM282: a G-code callback alone is not an auxiliary request.
    SyncAuxiliaryHandshakeToPLCSameThread();

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
// EDM139: these points are a coherent NC-owned image. S92 follows the
// admitted machining session until a lifecycle or positioning fence retires it.
void NCManager::ForceEDMPLCOutputsOffSameThread(const char* reason) noexcept
{
    const auto old=m_edm137PLC.Current();
    const auto session=m_edm137PLC.Session();
    const bool powerInhibited=m_edm137PLC.PowerInhibited();
    m_edm137PLC.Clear();
    if(g_PLC)g_PLC->PublishEDMOutputs(0U);
    if(old.word||old.requested)
        RtPrintf("[EDM139] event=PLC_OUTPUT session=%llu reason=%s S80=0 S92=0 S93=0 requested=0 policy=%s\n",
            static_cast<unsigned long long>(session),reason?reason:"STOP",
            powerInhibited?"MOTION_ONLY_POWER_INHIBITED":"NORMAL_EDM");
}

void NCManager::SyncEDMPLCOutputsSameThread() noexcept
{
    // EDM173 is motion-only from arm through release, independently of the
    // AIR latch. Stop before the legacy per-row/epoch arrays are inspected.
    if (m_g76PowerInhibited || EDM173::IsArm(m_edm94.gapHistoryArm))
    {
        ForceEDMPLCOutputsOffSameThread(EDM173::IsArm(m_edm94.gapHistoryArm) ?
            "EDM173_G76_GAP_INHIBITED" : "G76_AIR_RUN_INHIBITED");
        return;
    }
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
    // Only these explicitly admitted EDM profiles bind the power and E12
    // output image. Other diagnostics and ordinary CNC do not gain a permit.
    const bool motionOnly=EDM149::IsTestArm(s.gapHistoryArm);
    const bool fixture=motionOnly||s.gapHistoryArm==202U||s.gapHistoryArm==204U||s.gapHistoryArm==206U||s.gapHistoryArm==208U||
        s.gapHistoryArm==210U||s.gapHistoryArm==212U||s.gapHistoryArm==214U||s.gapHistoryArm==216U||s.gapHistoryArm==218U||s.gapHistoryArm==220U||s.gapHistoryArm==222U||s.gapHistoryArm==224U||s.gapHistoryArm==226U||s.gapHistoryArm==228U;
    const bool diagnosticControl=s.gapHistoryArm==210U||s.gapHistoryArm==212U||s.gapHistoryArm==214U||s.gapHistoryArm==216U||s.gapHistoryArm==218U||s.gapHistoryArm==220U||s.gapHistoryArm==226U||s.gapHistoryArm==228U;
    const bool sourceBound=motionOnly?
        (EDM130::IsMotionTest(m_edm130.recipe)&&
            m_edm130.recipe.diagnosticControlSource==EDM149::SourceForArm(s.gapHistoryArm)):
        m_edm130.recipe.diagnosticControlSource==(diagnosticControl?1U:0U);
    const unsigned expectedFixture=motionOnly?(EDM149::IsSixVisibleFlushTestArm(s.gapHistoryArm)?EDM149::SixVisibleFlushHistoryFixture:EDM149::IsXYZVisibleFlushTestArm(s.gapHistoryArm)?EDM149::XYZVisibleFlushHistoryFixture:EDM149::IsVisibleFlushTestArm(s.gapHistoryArm)?EDM149::VisibleFlushHistoryFixture:EDM149::IsPauseTestArm(s.gapHistoryArm)?EDM149::PauseHistoryFixture:EDM149::IsProgramTestArm(s.gapHistoryArm)?EDM149::ProgramHistoryFixture:EDM149::IsSparseArcTestArm(s.gapHistoryArm)?EDM149::SparseArcHistoryFixture:EDM149::IsSparseTestArm(s.gapHistoryArm)?EDM149::SparseHistoryFixture:EDM149::IsOrderTestArm(s.gapHistoryArm)?EDM149::OrderHistoryFixture:EDM149::IsArcTestArm(s.gapHistoryArm)?EDM149::ArcHistoryFixture:EDM149::IsSixTestArm(s.gapHistoryArm)?EDM149::SixHistoryFixture:EDM149::IsXYZTestArm(s.gapHistoryArm)?EDM149::XYZHistoryFixture:EDM149::ZHistoryFixture):
        ((s.gapHistoryArm==204U||s.gapHistoryArm==208U||s.gapHistoryArm==212U||
            s.gapHistoryArm==216U||s.gapHistoryArm==220U||s.gapHistoryArm==224U||s.gapHistoryArm==228U)?1U:0U);
    unsigned nominalMovementMask=0U;
    if(EDM149::IsBoundedProgramTestArm(s.gapHistoryArm)&&s.moves&&s.moves<=16U) {
        for(unsigned row=0U;row<s.moves;++row) {
            if(s.authoredGCode[row]==1U) {
                if(!EDM149::ValidSparseIncrement(s.increment[row],s.feed[row])) {nominalMovementMask=0U;break;}
                for(unsigned axis=0U;axis<6U;++axis)if(s.increment[row][axis]!=0.0)nominalMovementMask|=1U<<axis;
            } else if(s.authoredGCode[row]==2U||s.authoredGCode[row]==3U) {
                bool valid=s.feed[row]==0.12&&(s.increment[row][0]!=0.0||s.increment[row][1]!=0.0);
                for(unsigned axis=0U;axis<6U;++axis)
                    valid=valid&&std::isfinite(s.increment[row][axis])&&(axis<2U||s.increment[row][axis]==0.0);
                if(!valid) {nominalMovementMask=0U;break;}
                nominalMovementMask|=3U;
            }
            else {nominalMovementMask=0U;break;}
        }
    }
    const bool visiblePlan=!EDM149::IsVisibleFlushTestArm(s.gapHistoryArm)||
        (s.gapModeFixture==EDM116::Fixture::ModalVisibleFlush158&&s.moves==1U&&nominalMovementMask==4U&&
            s.authoredGCode[0]==1U&&s.feed[0]==0.12&&s.increment[0][2]==-0.0015&&
            m_edm127.authoredInline[0]&&m_edm127.expectedMode[0]==EDM116::Mode::B1);
    bool xyzVisiblePlan=true;
    if(EDM149::IsXYZVisibleFlushTestArm(s.gapHistoryArm)) {
        xyzVisiblePlan=s.gapModeFixture==EDM116::Fixture::ModalXYZVisibleFlush159&&s.moves==3U&&
            nominalMovementMask==7U&&EDM130::IsVisibleFlush159(m_edm130.recipe)&&
            guard.config.commissioningProfile==2U&&guard.config.visibleZFlush158&&
            m_edm127.expectedOriginPC[0]==s.pc[0];
        for(unsigned row=0U;row<3U&&xyzVisiblePlan;++row) {
            xyzVisiblePlan=s.authoredGCode[row]==row+1U&&s.feed[row]==0.12&&
                m_edm127.expectedMode[row]==EDM116::Mode::B1&&m_edm127.authoredInline[row]==(row==0U)&&
                m_edm127.expectedOriginPC[row]==m_edm127.expectedOriginPC[0]&&
                m_edm127.expectedOriginRevision[row]==m_edm127.expectedOriginRevision[0];
            for(unsigned axis=0U;axis<6U;++axis) {
                const double expected=row==0U?(axis==0U?0.0015:axis<3U?-0.0015:0.0):
                    (axis==0U?(row==1U?0.002:-0.002):0.0);
                xyzVisiblePlan=xyzVisiblePlan&&s.increment[row][axis]==expected;
            }
        }
    }
    bool sixVisiblePlan=true;
    if(EDM149::IsSixVisibleFlushTestArm(s.gapHistoryArm)) {
        sixVisiblePlan=s.gapModeFixture==EDM116::Fixture::ModalSixVisibleFlush167&&s.moves==3U&&
            nominalMovementMask==63U&&EDM130::IsVisibleFlush167(m_edm130.recipe)&&
            guard.config.commissioningProfile==3U&&guard.config.visibleZFlush158&&
            m_edm127.expectedOriginPC[0]==s.pc[0];
        for(unsigned row=0U;row<3U&&sixVisiblePlan;++row) {
            sixVisiblePlan=s.authoredGCode[row]==row+1U&&s.feed[row]==0.12&&
                m_edm127.expectedMode[row]==EDM116::Mode::B1&&m_edm127.authoredInline[row]==(row==0U)&&
                m_edm127.expectedOriginPC[row]==m_edm127.expectedOriginPC[0]&&
                m_edm127.expectedOriginRevision[row]==m_edm127.expectedOriginRevision[0];
            for(unsigned axis=0U;axis<6U;++axis) {
                const double expected=row==0U?(axis==0U?0.0015:axis<3U?-0.0015:axis==4U?-0.020:0.020):
                    (axis==0U?(row==1U?0.002:-0.002):0.0);
                sixVisiblePlan=sixVisiblePlan&&s.increment[row][axis]==expected;
            }
        }
    }
    const bool programPlan=EDM149::IsBoundedProgramTestArm(s.gapHistoryArm)?
        (visiblePlan&&xyzVisiblePlan&&sixVisiblePlan&&EDM130::ValidHistoryRecipeBinding(expectedFixture,m_edm130.recipe)&&
            EDM130::HasBoundProgramPlan(m_edm130.recipe)&&EDM130::ProgramRows(m_edm130.recipe)==s.moves&&
            EDM130::ProgramMovementMask(m_edm130.recipe)==nominalMovementMask&&
            guard.boundedProgram&&guard.requiredMovementMask==nominalMovementMask):
        (!m_edm130.recipe.planRows&&!m_edm130.recipe.planMovementMask&&!m_edm130.recipe.commissioningProfile);
    const bool live=fixture&&sourceBound&&programPlan&&m_state==NCState::RUN&&m_mode==NCOperationMode::MEMORY&&
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
        input.powerInhibited=motionOnly;
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
            mode.physicalAd==!EDM130::UsesSoftwareVoltage(m_edm130.recipe)&&
            mode.workTimeFlush&&mode.fixture==expectedFixture&&mode.session==s.session&&mode.armEpoch==s.armEpoch&&
            mode.ownerGeneration==s.lease.generation&&guard.identity.session==s.session&&
            guard.identity.epoch==s.armEpoch&&guard.identity.ownerGeneration==s.lease.generation&&
            guard.active&&guard.armAccepted&&!guard.cancelled&&!guard.faulted&&!guard.released&&
            EDM130::SameRecipe(mode.timerRecipe,m_edm130.recipe)&&!mode.coordinator.faulted&&
            !mode.coordinator.stopped&&mode.authorizedRows<=EDM130::ProgramRows(m_edm130.recipe)&&mode.completedRows<=mode.authorizedRows;
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
    // Publish one session-owned image. Motion tests retain flush status while
    // the frozen PLC state prohibits discharge and polarity in every phase.
    // Generic PLC writes cannot modify these reserved outputs.
    if(g_PLC)g_PLC->PublishEDMOutputs(output.word);
    if(previous.word!=output.word||previous.requested!=output.requested||
        previousSession!=m_edm137PLC.Session())
        RtPrintf("[EDM139] event=PLC_OUTPUT session=%llu phase=%s S80=%u S92=%u S93=%u requested=%u policy=%s\n",
            static_cast<unsigned long long>(input.session?input.session:previousSession),
            input.unavailable?"UNAVAILABLE":EDM128::PhaseName(phase),
            (output.word&EDM137PLC::FlushBit)?1U:0U,(output.word&EDM137PLC::DischargeBit)?1U:0U,
            (output.word&EDM137PLC::PolarityBit)?1U:0U,output.requested?1U:0U,
            input.powerInhibited?"MOTION_ONLY_POWER_INHIBITED":"NORMAL_EDM");
}
