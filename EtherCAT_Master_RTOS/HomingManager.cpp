#include "HomingManager.h"
#include "MotionCore.h"
#include "NCManager.h"
#include "AlarmManager.h"
#include "HomePersistenceManager.h"
#include "PLCManager.h"
#include "NCPLCMap.h"
#include <cmath>
#include <rtapi.h>
#include <rtssapi.h>

namespace
{
    // ========================================================
    // HOME Error -> Alarm Code
    //
    // HomeErrorReason：
    //     HomingManager 內部的詳細失敗原因。
    //
    // Alarm Code：
    //     提供 NC / HMI / Alarm History 使用。
    //
    // 多個內部原因可以對應到同一個對外 Alarm。
    // ========================================================

    int GetHomeAlarmCode(
        HomeErrorReason error)
    {
        switch (error)
        {
            // ----------------------------------------------------
            // HOME 參數 / 啟動條件錯誤
            // ----------------------------------------------------

        case HomeErrorReason::INVALID_CONFIG:
        case HomeErrorReason::AXIS_NOT_EXIST:

            return
                AlarmManager::HOME_INVALID_CONFIG;


            // ----------------------------------------------------
            // 找不到 DOG / 預期方向 LIMIT
            // ----------------------------------------------------

        case HomeErrorReason::SEARCH_TIMEOUT:
        case HomeErrorReason::SEARCH_MAX_DISTANCE:
        case HomeErrorReason::DOG_NOT_FOUND:

            return
                AlarmManager::HOME_SEARCH_NOT_FOUND;


            // ----------------------------------------------------
            // DOG / LIMIT 觸發後滑行距離過大
            // ----------------------------------------------------

        case HomeErrorReason::SWITCH_STOP_MAX_DISTANCE:

            return
                AlarmManager::HOME_SWITCH_STOP_DISTANCE;


            // ----------------------------------------------------
            // Backoff 失敗
            // ----------------------------------------------------

        case HomeErrorReason::BACKOFF_TIMEOUT:
        case HomeErrorReason::BACKOFF_MAX_DISTANCE:

            return
                AlarmManager::HOME_BACKOFF_FAILED;


            // ----------------------------------------------------
            // Backoff 完成後訊號未解除
            // ----------------------------------------------------

        case HomeErrorReason::DOG_NOT_RELEASED:

            return
                AlarmManager::HOME_DOG_NOT_RELEASED;


        case HomeErrorReason::HARD_LIMIT_NOT_RELEASED:

            return
                AlarmManager::HOME_LIMIT_NOT_RELEASED;


            // ----------------------------------------------------
            // 找不到 INDEX
            // ----------------------------------------------------

        case HomeErrorReason::INDEX_TIMEOUT:
        case HomeErrorReason::INDEX_MAX_DISTANCE:
        case HomeErrorReason::INDEX_NOT_FOUND:

            return
                AlarmManager::HOME_INDEX_NOT_FOUND;


            // ----------------------------------------------------
            // Reference Capture 無效
            // ----------------------------------------------------

        case HomeErrorReason::REFERENCE_NOT_ARMED:
        case HomeErrorReason::REFERENCE_INVALID:

            return
                AlarmManager::HOME_REFERENCE_INVALID;


            // ----------------------------------------------------
            // HOME 過程撞到不允許的 Hard Limit
            // ----------------------------------------------------

        case HomeErrorReason::OPPOSITE_HARD_LIMIT:

            return
                AlarmManager::HOME_OPPOSITE_LIMIT;


        case HomeErrorReason::BOTH_HARD_LIMITS:

            return
                AlarmManager::HOME_BOTH_LIMITS;


            // ----------------------------------------------------
            // Servo / Motion 執行異常
            // ----------------------------------------------------

        case HomeErrorReason::SERVO_NOT_READY:
        case HomeErrorReason::MOTION_BUSY:
        case HomeErrorReason::SERVO_FAULT:
        case HomeErrorReason::MOTION_FAULT:

            return
                AlarmManager::HOME_MOTION_FAULT;


            // ----------------------------------------------------
            // 正常取消不是 Alarm
            // ----------------------------------------------------

        case HomeErrorReason::CANCELLED:
        case HomeErrorReason::NONE:
        default:

            return 0;
        }
    }
}


// ============================================================
// Constructor
// ============================================================

HomingManager::HomingManager(
    MotionCore& motion)
    : m_motion(motion)
{
}


// ============================================================
// System Link
// ============================================================

void HomingManager::LinkNCManager(
    NCManager* nc)
{
    m_nc = nc;
}


void HomingManager::LinkPLCManager(
    PLCManager* plc)
{
    m_plc = plc;
}

// ============================================================================
// Stage NC-0.1F - HOME Motion Owner and RT Axis Command Mailbox
// ============================================================================
bool HomingManager::AcquireHomeMotionOwner() noexcept
{
    m_homeMotionLease = MotionOwnerLease{};
    m_returnMotionOwner = MotionOwner::NONE;

    const MotionOwnerLease currentLease = m_motion.GetMotionOwnerLease();

    if (currentLease.owner == MotionOwner::NONE)
    {
        return m_motion.TryAcquireMotionOwner(
            MotionOwner::HOME, m_homeMotionLease);
    }

    if (currentLease.owner == MotionOwner::HOME &&
        m_motion.IsMotionOwnerLeaseCurrent(currentLease))
    {
        m_homeMotionLease = currentLease;
        return true;
    }

    if (currentLease.owner == MotionOwner::AUTO ||
        currentLease.owner == MotionOwner::MDI ||
        currentLease.owner == MotionOwner::MANUAL_AUTO)
    {
        MotionOwnerLease homeLease{};
        if (!m_motion.TryTransferMotionOwner(
            currentLease, MotionOwner::HOME, homeLease))
        {
            return false;
        }

        m_returnMotionOwner = currentLease.owner;
        m_homeMotionLease = homeLease;
        return true;
    }

    return false;
}

void HomingManager::RestoreOrReleaseHomeMotionOwner() noexcept
{
    const MotionOwnerLease homeLease = m_homeMotionLease;
    const MotionOwner returnOwner = m_returnMotionOwner;

    if (!homeLease.IsValid())
    {
        m_homeMotionLease = MotionOwnerLease{};
        m_returnMotionOwner = MotionOwner::NONE;
        return;
    }

    if (!m_motion.IsMotionOwnerLeaseCurrent(homeLease))
    {
        // Safety / Reset already owns a newer Generation.
        m_homeMotionLease = MotionOwnerLease{};
        m_returnMotionOwner = MotionOwner::NONE;
        for (int i = 0; i < HOME_AXIS_COUNT; ++i)
        {
            m_pendingProbeDisarmSequence[i] =
                MOTION_AXIS_COMMAND_SEQUENCE_INVALID;
        }
        return;
    }

    // A zero queue depth is not sufficient: RT may already have popped the
    // command but not yet applied it. Wait for each exact result before
    // changing Owner Generation.
    for (int i = 0; i < HOME_AXIS_COUNT; ++i)
    {
        const MotionAxisCommandSequence sequence =
            m_pendingProbeDisarmSequence[i];

        if (sequence == MOTION_AXIS_COMMAND_SEQUENCE_INVALID)
        {
            continue;
        }

        MotionAxisCommandResult result{};
        if (!m_motion.TryGetAxisCommandResult(sequence, result))
        {
            return;
        }

        m_pendingProbeDisarmSequence[i] =
            MOTION_AXIS_COMMAND_SEQUENCE_INVALID;

        if (result.resultType != MotionAxisCommandResultType::APPLIED)
        {
            m_hasError = true;
            m_completed = false;
            m_lastError = HomeErrorReason::REFERENCE_INVALID;
            m_lastErrorAxis = result.axisIndex;

            if (!AlarmManager::GetInstance().HasAlarm())
            {
                AlarmManager::GetInstance().Trigger(
                    AlarmManager::HOME_MOTION_FAULT, 0, result.axisIndex);
            }

            m_motion.RequestEmergencyStopAllAxes();
            if (m_nc != nullptr)
            {
                m_nc->ChangeState(NCState::ALARM);
            }

            m_homeMotionLease = MotionOwnerLease{};
            m_returnMotionOwner = MotionOwner::NONE;
            return;
        }
    }

    if (returnOwner == MotionOwner::AUTO ||
        returnOwner == MotionOwner::MDI ||
        returnOwner == MotionOwner::MANUAL_AUTO)
    {
        MotionOwnerLease restoredLease{};
        if (m_motion.TryTransferMotionOwner(
            homeLease, returnOwner, restoredLease))
        {
            const bool adopted =
                m_nc != nullptr &&
                m_nc->AdoptProgramMotionLease(restoredLease);

            if (!adopted)
            {
                // Do not leave an owner with no corresponding NC program lease.
                m_motion.ReleaseMotionOwner(restoredLease);
            }
        }
    }
    else
    {
        m_motion.ReleaseMotionOwner(homeLease);
    }

    m_homeMotionLease = MotionOwnerLease{};
    m_returnMotionOwner = MotionOwner::NONE;
}


MotionOwnerLease HomingManager::GetProbeCommandLease() const noexcept
{
    if (m_motion.IsMotionOwnerLeaseCurrent(m_homeMotionLease))
    {
        return m_homeMotionLease;
    }

    const MotionOwnerLease currentLease = m_motion.GetMotionOwnerLease();
    if (currentLease.owner == MotionOwner::SAFETY &&
        m_motion.IsMotionOwnerLeaseCurrent(currentLease))
    {
        return currentLease;
    }

    return MotionOwnerLease{};
}

bool HomingManager::QueueHomeStop(
    int axisIndex, double decelerationTime) noexcept
{
    return m_motion.SubmitAxisStopMove(
        axisIndex, decelerationTime, MotionCommandSource::HOME,
        m_homeMotionLease);
}

bool HomingManager::QueueHomeVelocity(
    int axisIndex, double velocity, double accelerationTime) noexcept
{
    return m_motion.SubmitAxisVelocityMove(
        axisIndex, velocity, accelerationTime, MotionCommandSource::HOME,
        m_homeMotionLease);
}

bool HomingManager::QueueHomeMove(
    int axisIndex, double targetPosition, double targetVelocity,
    double accelerationTime, double decelerationTime,
    bool useShortestPath) noexcept
{
    return m_motion.SubmitAxisMoveToPosition(
        axisIndex, targetPosition, targetVelocity, accelerationTime,
        decelerationTime, useShortestPath, MotionCommandSource::HOME,
        m_homeMotionLease);
}

bool HomingManager::QueueHomeProbeFunction(
    int axisIndex, uint16_t value,
    MotionAxisCommandSequence* outSequence) noexcept
{
    const MotionOwnerLease lease = GetProbeCommandLease();
    if (!lease.IsValid()) return false;
    return m_motion.SubmitDriveTouchProbeFunction(
        axisIndex, value, lease, outSequence);
}


// ============================================================
// Start HOME Request
//
// 建立本次 G81 / Panel HOME：
//
// - 選軸
// - P0 / P1 排程
// - HomeRuntime
// - HOME Motion Ownership
//
// 真正的 DOG / LIMIT / INDEX / Backoff 運動，
// 由 Process() 週期性執行完整狀態機。
// ============================================================

bool HomingManager::Start(
    const HomeRequest& request)
{
    // 已經有 HOME Request 在執行時，
    // 不允許第二個 Request 搶控制權。
    if (m_active)
    {
        return false;
    }


    // 清除上一個 Request 的結果。
    m_completed = false;
    m_hasError = false;
    m_cancelRequested = false;
    ClearResumeRequest();
    m_runControlState = HomeRunControlState::IDLE;

    m_lastError =
        HomeErrorReason::NONE;

    m_lastErrorAxis =
        -1;

    m_selectedAxisMask =
        0;

    m_activeAxisMask =
        0;

    m_currentOrder =
        -1;

    for (int i = 0; i < HOME_AXIS_COUNT; ++i)
    {
        m_pendingApplyHomeSequence[i] =
            MOTION_AXIS_COMMAND_SEQUENCE_INVALID;
        m_pendingProbeDisarmSequence[i] =
            MOTION_AXIS_COMMAND_SEQUENCE_INVALID;
    }


    // --------------------------------------------------------
    // axisMask == 0
    //
    // 自動選取：
    //
    // isExist
    // &&
    // home.enabled
    // --------------------------------------------------------

    uint8_t selectedAxisMask =
        request.axisMask;

    if (selectedAxisMask == 0)
    {
        selectedAxisMask =
            BuildEnabledAxisMask();
    }


    // 沒有任何可 HOME 軸。
    if (selectedAxisMask == 0)
    {
        m_lastError =
            HomeErrorReason::INVALID_CONFIG;

        return false;
    }


    // --------------------------------------------------------
    // 驗證並初始化全部 Selected Axis。
    // --------------------------------------------------------

    if (!AcquireHomeMotionOwner())
    {
        m_lastError = HomeErrorReason::MOTION_BUSY;
        return false;
    }

    if (!ValidateAndInitializeRequest(
        request,
        selectedAxisMask))
    {
        RestoreOrReleaseHomeMotionOwner();
        return false;
    }


    m_sequenceMode =
        request.sequenceMode;

    m_selectedAxisMask =
        selectedAxisMask;

    m_active =
        true;

    m_runControlState =
        HomeRunControlState::RUNNING;


    // --------------------------------------------------------
    // 啟動第一個 HOME Group。
    //
    // P0：
    //     全部 Selected Axis
    //
    // P1：
    //     最小 HomeOrder Group
    // --------------------------------------------------------

    ActivateInitialGroup();


    // 如果排程器沒有成功選出 Active Axis，
    // 視為 Request 無效。
    if (m_activeAxisMask == 0)
    {
        m_active =
            false;

        m_runControlState =
            HomeRunControlState::IDLE;

        m_lastError =
            HomeErrorReason::INVALID_CONFIG;

        RestoreOrReleaseHomeMotionOwner();
        return false;
    }


    return true;
}


// ============================================================
// Cyclic Process
//
// 週期執行完整 HOME State Machine、P0/P1 Scheduler
// 與多軸 Backoff / INDEX Barrier。
// ============================================================

void HomingManager::Process(
    double cycleTimeSec)
{
    if (!m_active)
    {
        // Completion may leave the final Probe Disarm in the mailbox.
        // Retry the owner hand-off on each 10 ms pass until RT consumes it.
        if (m_homeMotionLease.IsValid())
        {
            RestoreOrReleaseHomeMotionOwner();
        }
        return;
    }

    if (!std::isfinite(cycleTimeSec) ||
        cycleTimeSec < 0.0)
    {
        cycleTimeSec = 0.0;
    }


    // Reset / Abort 永遠比 Hold 優先。
    if (m_cancelRequested)
    {
        ProcessCancel();
        return;
    }


    // 外部 E-Stop / Safety / Servo Alarm 在 RUNNING、Hold 減速、
    // PAUSED 任一狀態都必須讓 HOME 失效，不可 Resume。
    if (AlarmManager::GetInstance().HasAlarm() ||
        (m_nc != nullptr &&
            m_nc->GetState() == NCState::ALARM))
    {
        for (int i = 0;
            i < HOME_AXIS_COUNT;
            ++i)
        {
            if (!IsAxisRequested(
                m_selectedAxisMask,
                i))
            {
                continue;
            }

            AxisContext& axis =
                m_motion.GetAxisContext(i);

            if (axis.homeRuntime.completed)
            {
                continue;
            }

            SetAxisError(
                i,
                axis,
                HomeErrorReason::MOTION_FAULT);

            break;
        }

        return;
    }


    // Feed Hold：只處理 Controlled Stop、Sensor / INDEX Capture。
    // 不執行正常 State Machine，也不累加 State Timeout。
    if (m_runControlState ==
        HomeRunControlState::HOLD_DECEL_STOP)
    {
        ProcessHold(cycleTimeSec);
        return;
    }


    // 完全暫停後所有 HOME Timeout 與距離 State Machine 凍結。
    // Hard Limit / E-Stop 仍由 NCPLCManager::ProcessSafetyInputs() 監控。
    if (m_runControlState ==
        HomeRunControlState::PAUSED)
    {
        if (m_resumeRequested)
        {
            (void)TryResumeFromPaused();
        }
        return;
    }


    for (int i = 0;
        i < HOME_AXIS_COUNT;
        ++i)
    {
        if (!IsAxisRequested(
            m_activeAxisMask,
            i))
        {
            continue;
        }

        AxisContext& axis =
            m_motion.GetAxisContext(i);

        ProcessAxis(
            i,
            axis,
            cycleTimeSec);

        if (!m_active ||
            m_hasError)
        {
            return;
        }
    }


    ProcessBackoffBarrier();

    if (!m_active ||
        m_hasError)
    {
        return;
    }


    ProcessIndexStopBarrier();

    if (!m_active ||
        m_hasError)
    {
        return;
    }


    if (IsActiveGroupComplete())
    {
        AdvanceScheduler();
    }
}


// ============================================================
// Hold / Resume / Cancel / Reset
// ============================================================

bool HomingManager::RequestHold()
{
    if (!m_active ||
        m_hasError ||
        m_cancelRequested)
    {
        return false;
    }

    if (m_runControlState ==
        HomeRunControlState::HOLD_DECEL_STOP ||
        m_runControlState ==
        HomeRunControlState::PAUSED)
    {
        return true;
    }

    ClearResumeRequest();

    m_runControlState =
        HomeRunControlState::HOLD_DECEL_STOP;

    return true;
}


void HomingManager::ClearResumeRequest() noexcept
{
    m_resumeRequested = false;
    m_resumeAlarmUpdateCount = 0U;
    m_resumeAlarmIntentBaseState = 0ULL;
    m_resumeHomeMotionLease = MotionOwnerLease{};
    m_resumeAdmissionTicketValid = false;
}


HomingManager::ResumeResult HomingManager::Resume() noexcept
{
    if (!m_active ||
        m_hasError ||
        m_cancelRequested ||
        (m_runControlState !=
            HomeRunControlState::HOLD_DECEL_STOP &&
            m_runControlState !=
            HomeRunControlState::PAUSED) ||
        m_nc == nullptr ||
        !m_homeMotionLease.IsValid() ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_homeMotionLease))
    {
        ClearResumeRequest();
        return ResumeResult::REJECTED;
    }

    if (!m_resumeRequested)
    {
        // Capture the button edge once.  MotionAdmissionBaseState removes only
        // a benign concurrent reservation; Alarm/Clear sequence and active
        // intent bits remain part of the immutable identity.
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
            ClearResumeRequest();
            return ResumeResult::SUPERSEDED;
        }

        m_resumeAlarmUpdateCount = updateAfter;
        m_resumeAlarmIntentBaseState = intentAfter;
        m_resumeHomeMotionLease = m_homeMotionLease;
        m_resumeAdmissionTicketValid = true;
        m_resumeRequested = true;
    }

    if (!m_resumeAdmissionTicketValid ||
        !m_resumeHomeMotionLease.Matches(m_homeMotionLease))
    {
        ClearResumeRequest();
        return ResumeResult::SUPERSEDED;
    }

    // 操作員可以在 Controlled Stop 尚未完全結束前先按 Start。
    // 所有 Active Axis 停妥後會自動 Resume。
    if (m_runControlState ==
        HomeRunControlState::HOLD_DECEL_STOP)
    {
        return ResumeResult::DEFERRED;
    }

    return TryResumeFromPaused();
}


void HomingManager::Cancel()
{
    if (!m_active)
    {
        return;
    }


    m_cancelRequested =
        true;
}


void HomingManager::Reset()
{
    RestoreOrReleaseHomeMotionOwner();
    for (int i = 0; i < HOME_AXIS_COUNT; ++i)
    {
        m_pendingApplyHomeSequence[i] =
            MOTION_AXIS_COMMAND_SEQUENCE_INVALID;
        m_pendingProbeDisarmSequence[i] =
            MOTION_AXIS_COMMAND_SEQUENCE_INVALID;
    }

    m_sequenceMode =
        HomeSequenceMode::SIMULTANEOUS;

    m_selectedAxisMask =
        0;

    m_activeAxisMask =
        0;

    m_currentOrder =
        -1;

    m_active =
        false;

    m_completed =
        false;

    m_hasError =
        false;

    m_cancelRequested =
        false;

    ClearResumeRequest();

    m_runControlState =
        HomeRunControlState::IDLE;

    m_lastError =
        HomeErrorReason::NONE;

    m_lastErrorAxis =
        -1;


    // --------------------------------------------------------
    // 只清除 HOME Runtime。
    //
    // 不改：
    //
    // axis.isHomed
    // MotionState
    // Position
    //
    // 因為 HomingManager Reset 本身不是 Motion Reset。
    // --------------------------------------------------------

    for (int i = 0;
        i < HOME_AXIS_COUNT;
        ++i)
    {
        AxisContext& axis =
            m_motion.GetAxisContext(i);

        if (!axis.isExist)
        {
            continue;
        }

        DisarmDriveProbe(
            i,
            axis);

        ResetAxisRuntime(axis);
    }
}


// ============================================================
// Global State Query
// ============================================================

bool HomingManager::IsActive() const
{
    return m_active;
}


bool HomingManager::IsCompleted() const
{
    return m_completed;
}


bool HomingManager::HasError() const
{
    return m_hasError;
}


bool HomingManager::IsCancelRequested() const
{
    return m_cancelRequested;
}


bool HomingManager::IsHoldDecelerating() const
{
    return
        m_active &&
        m_runControlState ==
        HomeRunControlState::HOLD_DECEL_STOP;
}


bool HomingManager::IsPaused() const
{
    return
        m_active &&
        m_runControlState ==
        HomeRunControlState::PAUSED;
}


bool HomingManager::IsResumeRequested() const
{
    return m_resumeRequested;
}


HomeRunControlState HomingManager::GetRunControlState() const
{
    return m_runControlState;
}


// ============================================================
// Scheduler Query
// ============================================================

uint8_t HomingManager::GetSelectedAxisMask() const
{
    return m_selectedAxisMask;
}


uint8_t HomingManager::GetActiveAxisMask() const
{
    return m_activeAxisMask;
}


HomeSequenceMode HomingManager::GetSequenceMode() const
{
    return m_sequenceMode;
}


int HomingManager::GetCurrentOrder() const
{
    return m_currentOrder;
}


// ============================================================
// Error Query
// ============================================================

HomeErrorReason HomingManager::GetLastError() const
{
    return m_lastError;
}


int HomingManager::GetLastErrorAxis() const
{
    return m_lastErrorAxis;
}


// ============================================================
// Per-Axis Query
// ============================================================

bool HomingManager::IsAxisSelected(
    int axisIndex) const
{
    if (!IsValidAxisIndex(
        axisIndex))
    {
        return false;
    }


    return IsAxisRequested(
        m_selectedAxisMask,
        axisIndex);
}


bool HomingManager::IsAxisActive(
    int axisIndex) const
{
    if (!IsValidAxisIndex(
        axisIndex))
    {
        return false;
    }


    if (!IsAxisRequested(
        m_activeAxisMask,
        axisIndex))
    {
        return false;
    }


    const AxisContext& axis =
        m_motion.GetAxisContext(
            axisIndex);


    return
        axis.homeRuntime.active;
}


bool HomingManager::IsAxisCompleted(
    int axisIndex) const
{
    if (!IsValidAxisIndex(
        axisIndex))
    {
        return false;
    }


    const AxisContext& axis =
        m_motion.GetAxisContext(
            axisIndex);


    return
        axis.homeRuntime.completed;
}


bool HomingManager::IsAxisHoming(
    int axisIndex) const
{
    if (!IsValidAxisIndex(
        axisIndex))
    {
        return false;
    }


    const AxisContext& axis =
        m_motion.GetAxisContext(
            axisIndex);


    if (!axis.homeRuntime.selected ||
        !axis.homeRuntime.active)
    {
        return false;
    }


    if (axis.homeRuntime.state ==
        HomeState::IDLE ||
        axis.homeRuntime.state ==
        HomeState::DONE ||
        axis.homeRuntime.state ==
        HomeState::HOME_ERROR)
    {
        return false;
    }


    return true;
}


HomeState HomingManager::GetAxisState(
    int axisIndex) const
{
    if (!IsValidAxisIndex(
        axisIndex))
    {
        return
            HomeState::IDLE;
    }


    const AxisContext& axis =
        m_motion.GetAxisContext(
            axisIndex);


    return
        axis.homeRuntime.state;
}


HomeErrorReason HomingManager::GetAxisError(
    int axisIndex) const
{
    if (!IsValidAxisIndex(
        axisIndex))
    {
        return
            HomeErrorReason::INVALID_CONFIG;
    }


    const AxisContext& axis =
        m_motion.GetAxisContext(
            axisIndex);


    return
        axis.homeRuntime.error;
}


// ============================================================
// Expected HOME Hard Limit
// ============================================================

bool HomingManager::IsExpectedPositiveHardLimit(
    int axisIndex) const
{
    return IsExpectedHomeHardLimit(
        axisIndex,
        true);
}


bool HomingManager::IsExpectedNegativeHardLimit(
    int axisIndex) const
{
    return IsExpectedHomeHardLimit(
        axisIndex,
        false);
}


bool HomingManager::IsExpectedHomeHardLimit(
    int axisIndex,
    bool positiveDirection) const
{
    if (!m_active ||
        !IsAxisHoming(axisIndex))
    {
        return false;
    }


    const AxisContext& axis =
        m_motion.GetAxisContext(
            axisIndex);


    // 只有 LIMIT_INDEX / LIMIT_ONLY
    // 才能把 Physical Limit 當正常 HOME Event。
    if (!UsesHardLimitSwitch(
        axis.home.method))
    {
        return false;
    }


    // HomeDirection 必須與 Limit 方向相同。
    if (positiveDirection)
    {
        if (axis.home.direction != 1)
        {
            return false;
        }
    }
    else
    {
        if (axis.home.direction != -1)
        {
            return false;
        }
    }


    // 只有指定 HOME State 可以暫時把該方向
    // Hard Limit 解讀成 HOME Event。
    if (!IsHardLimitAllowedHomeState(
        axis.homeRuntime.state))
    {
        return false;
    }


    return true;
}


// ============================================================
// Basic Validation
// ============================================================

bool HomingManager::IsValidAxisIndex(
    int axisIndex) const
{
    return
        axisIndex >= 0 &&
        axisIndex < HOME_AXIS_COUNT;
}


bool HomingManager::IsAxisRequested(
    uint8_t axisMask,
    int axisIndex) const
{
    if (!IsValidAxisIndex(
        axisIndex))
    {
        return false;
    }


    const uint8_t bit =
        static_cast<uint8_t>(
            1u << axisIndex);


    return
        (axisMask & bit) != 0;
}


uint8_t HomingManager::BuildEnabledAxisMask() const
{
    uint8_t mask =
        0;


    for (int i = 0;
        i < HOME_AXIS_COUNT;
        ++i)
    {
        const AxisContext& axis =
            m_motion.GetAxisContext(i);


        if (!axis.isExist)
        {
            continue;
        }


        if (!axis.home.enabled)
        {
            continue;
        }


        mask |=
            static_cast<uint8_t>(
                1u << i);
    }


    return mask;
}


// ============================================================
// Request Validation / Initialization
// ============================================================

bool HomingManager::ValidateAndInitializeRequest(
    const HomeRequest& request,
    uint8_t selectedAxisMask)
{
    // --------------------------------------------------------
    // Sequence Mode 防呆
    // --------------------------------------------------------

    if (request.sequenceMode !=
        HomeSequenceMode::SIMULTANEOUS &&
        request.sequenceMode !=
        HomeSequenceMode::BY_ORDER)
    {
        m_lastError =
            HomeErrorReason::INVALID_CONFIG;

        return false;
    }


    // --------------------------------------------------------
    // 第一輪只做 Validation。
    //
    // 驗證全部通過後才真正修改 HomeRuntime，
    // 避免檢查到一半失敗造成部分軸已被改狀態。
    // --------------------------------------------------------

    for (int i = 0;
        i < HOME_AXIS_COUNT;
        ++i)
    {
        if (!IsAxisRequested(
            selectedAxisMask,
            i))
        {
            continue;
        }


        AxisContext& axis =
            m_motion.GetAxisContext(i);


        if (!axis.isExist)
        {
            m_lastError =
                HomeErrorReason::AXIS_NOT_EXIST;

            m_lastErrorAxis =
                i;

            return false;
        }


        if (!axis.home.enabled)
        {
            m_lastError =
                HomeErrorReason::INVALID_CONFIG;

            m_lastErrorAxis =
                i;

            return false;
        }


        if (!axis.isServoOn)
        {
            m_lastError =
                HomeErrorReason::SERVO_NOT_READY;

            m_lastErrorAxis =
                i;

            return false;
        }


        if (axis.isFault ||
            axis.isLagAlarm ||
            axis.state ==
            MotionState::MotionState_ERROR ||
            axis.state ==
            MotionState::MotionState_ESTOP)
        {
            m_lastError =
                HomeErrorReason::SERVO_FAULT;

            m_lastErrorAxis =
                i;

            return false;
        }


        // HOME 第一版只允許從完全 IDLE 接管軸。
        //
        // 不允許搶：
        //
        // JOG
        // MPG
        // INCH
        // G00
        // Interpolation
        // STOPPING
        if (axis.state !=
            MotionState::MotionState_IDLE)
        {
            m_lastError =
                HomeErrorReason::MOTION_BUSY;

            m_lastErrorAxis =
                i;

            return false;
        }


        if (axis.home.direction != -1 &&
            axis.home.direction != 1)
        {
            m_lastError =
                HomeErrorReason::INVALID_CONFIG;

            m_lastErrorAxis =
                i;

            return false;
        }


        // 第一版尚未實作 Mechanical Stop Homing。
        if (axis.home.method ==
            HomeMethod::MECHANICAL_STOP)
        {
            m_lastError =
                HomeErrorReason::INVALID_CONFIG;

            m_lastErrorAxis =
                i;

            return false;
        }


        // 需要 INDEX 的模式不允許 ReferenceSource = NONE。
        if (UsesIndexReference(
            axis.home.method) &&
            axis.home.referenceSource ==
            HomeReferenceSource::NONE)
        {
            m_lastError =
                HomeErrorReason::INVALID_CONFIG;

            m_lastErrorAxis =
                i;

            return false;
        }


        // ABSOLUTE_REFERENCE 尚未接入 Delta Absolute Position Provider。
        // 不可用「目前位置」或 60BA 假裝成 Absolute Home。
        if (axis.home.method ==
            HomeMethod::ABSOLUTE_REFERENCE)
        {
            m_lastError =
                HomeErrorReason::INVALID_CONFIG;

            m_lastErrorAxis =
                i;

            return false;
        }


        if (UsesIndexReference(axis.home.method) &&
            (axis.home.referenceSource ==
                HomeReferenceSource::MOTOR_ENCODER_INDEX ||
                axis.home.referenceSource ==
                HomeReferenceSource::LINEAR_SCALE_INDEX_DRIVE) &&
            !ValidateDriveProbeConfig(axis))
        {
            m_lastError =
                HomeErrorReason::INVALID_CONFIG;

            m_lastErrorAxis =
                i;

            return false;
        }
    }


    // --------------------------------------------------------
    // 第二輪：全部驗證成功後，
    // 才建立本次 HOME Runtime。
    // --------------------------------------------------------

    for (int i = 0;
        i < HOME_AXIS_COUNT;
        ++i)
    {
        AxisContext& axis =
            m_motion.GetAxisContext(i);


        if (!axis.isExist)
        {
            continue;
        }


        ResetAxisRuntime(axis);


        if (!IsAxisRequested(
            selectedAxisMask,
            i))
        {
            continue;
        }


        PrepareSelectedAxis(axis);
    }


    return true;
}


void HomingManager::ResetAxisRuntime(
    AxisContext& axis)
{
    axis.homeRuntime =
        HomeRuntime{};
}


void HomingManager::PrepareSelectedAxis(
    AxisContext& axis)
{
    axis.homeRuntime.selected =
        true;

    axis.homeRuntime.active =
        false;

    axis.homeRuntime.completed =
        false;

    axis.homeRuntime.error =
        HomeErrorReason::NONE;

    axis.homeRuntime.state =
        HomeState::PREPARE;


    // HOME 一旦正式接受，
    // 舊的 Machine Home 即失效。
    //
    // 直到 CompleteAxis() 成功後才重新設為 true。
    //
    // 這也會讓既有 Software Travel Limit
    // 在 HOME 過程自動 bypass。
    axis.isHomed =
        false;
}


// ============================================================
// P0 / P1 Scheduler
// ============================================================

void HomingManager::ActivateInitialGroup()
{
    m_activeAxisMask =
        0;


    if (m_sequenceMode ==
        HomeSequenceMode::SIMULTANEOUS)
    {
        m_currentOrder =
            -1;

        ActivateAllSelectedAxes();

        return;
    }


    const int firstOrder =
        FindLowestPendingOrder();


    if (firstOrder < 0)
    {
        return;
    }


    m_currentOrder =
        firstOrder;

    ActivateOrderGroup(
        firstOrder);
}


void HomingManager::ActivateAllSelectedAxes()
{
    m_activeAxisMask =
        0;


    for (int i = 0;
        i < HOME_AXIS_COUNT;
        ++i)
    {
        if (!IsAxisRequested(
            m_selectedAxisMask,
            i))
        {
            continue;
        }


        AxisContext& axis =
            m_motion.GetAxisContext(i);


        if (!axis.homeRuntime.selected ||
            axis.homeRuntime.completed ||
            axis.homeRuntime.error !=
            HomeErrorReason::NONE)
        {
            continue;
        }


        axis.homeRuntime.active =
            true;

        axis.homeRuntime.state =
            HomeState::PREPARE;

        axis.homeRuntime.stateElapsedSec =
            0.0;


        m_activeAxisMask |=
            static_cast<uint8_t>(
                1u << i);
    }
}


void HomingManager::ActivateOrderGroup(
    int order)
{
    m_activeAxisMask =
        0;


    for (int i = 0;
        i < HOME_AXIS_COUNT;
        ++i)
    {
        if (!IsAxisRequested(
            m_selectedAxisMask,
            i))
        {
            continue;
        }


        AxisContext& axis =
            m_motion.GetAxisContext(i);


        if (!axis.homeRuntime.selected ||
            axis.homeRuntime.completed ||
            axis.homeRuntime.error !=
            HomeErrorReason::NONE)
        {
            continue;
        }


        if (axis.home.order !=
            order)
        {
            continue;
        }


        axis.homeRuntime.active =
            true;

        axis.homeRuntime.state =
            HomeState::PREPARE;

        axis.homeRuntime.stateElapsedSec =
            0.0;


        m_activeAxisMask |=
            static_cast<uint8_t>(
                1u << i);
    }
}


int HomingManager::FindLowestPendingOrder() const
{
    int lowestOrder =
        -1;


    for (int i = 0;
        i < HOME_AXIS_COUNT;
        ++i)
    {
        if (!IsAxisRequested(
            m_selectedAxisMask,
            i))
        {
            continue;
        }


        const AxisContext& axis =
            m_motion.GetAxisContext(i);


        if (!axis.homeRuntime.selected ||
            axis.homeRuntime.completed ||
            axis.homeRuntime.active ||
            axis.homeRuntime.error !=
            HomeErrorReason::NONE)
        {
            continue;
        }


        if (lowestOrder < 0 ||
            axis.home.order <
            lowestOrder)
        {
            lowestOrder =
                axis.home.order;
        }
    }


    return lowestOrder;
}


bool HomingManager::HasPendingAxis() const
{
    for (int i = 0;
        i < HOME_AXIS_COUNT;
        ++i)
    {
        if (!IsAxisRequested(
            m_selectedAxisMask,
            i))
        {
            continue;
        }


        const AxisContext& axis =
            m_motion.GetAxisContext(i);


        if (axis.homeRuntime.selected &&
            !axis.homeRuntime.completed &&
            axis.homeRuntime.error ==
            HomeErrorReason::NONE &&
            !axis.homeRuntime.active)
        {
            return true;
        }
    }


    return false;
}


bool HomingManager::IsActiveGroupComplete() const
{
    if (m_activeAxisMask == 0)
    {
        return false;
    }


    for (int i = 0;
        i < HOME_AXIS_COUNT;
        ++i)
    {
        if (!IsAxisRequested(
            m_activeAxisMask,
            i))
        {
            continue;
        }


        const AxisContext& axis =
            m_motion.GetAxisContext(i);


        if (!axis.homeRuntime.completed &&
            axis.homeRuntime.error ==
            HomeErrorReason::NONE)
        {
            return false;
        }
    }


    return true;
}


void HomingManager::AdvanceScheduler()
{
    if (m_hasError)
    {
        m_active =
            false;

        return;
    }


    // 已完成的 Active Group 先釋放。
    m_activeAxisMask =
        0;


    // P0：
    // 所有 Selected Axis 都在同一 Group。
    if (m_sequenceMode ==
        HomeSequenceMode::SIMULTANEOUS)
    {
        CompleteRequest();
        return;
    }


    // P1：
    // 找下一個尚未完成的 HomeOrder。
    if (HasPendingAxis())
    {
        const int nextOrder =
            FindLowestPendingOrder();


        if (nextOrder >= 0)
        {
            m_currentOrder =
                nextOrder;

            ActivateOrderGroup(
                nextOrder);

            return;
        }
    }


    CompleteRequest();
}


// ============================================================
// HOME Cyclic Helpers / Barriers
// ============================================================

void HomingManager::ProcessCancel()
{
    bool allStopped =
        true;


    for (int i = 0;
        i < HOME_AXIS_COUNT;
        ++i)
    {
        if (!IsAxisRequested(
            m_selectedAxisMask,
            i))
        {
            continue;
        }

        AxisContext& axis =
            m_motion.GetAxisContext(i);

        if (!axis.homeRuntime.selected ||
            axis.homeRuntime.completed)
        {
            continue;
        }


        if (axis.state !=
            MotionState::MotionState_IDLE)
        {
            allStopped =
                false;

            const double dec =
                GetHoldDecTime(axis);

            if (axis.state !=
                MotionState::MotionState_STOPPING)
            {
                QueueHomeStop(i, dec);
            }
        }
    }


    if (!allStopped)
    {
        return;
    }


    bool allDisarmQueued = true;

    for (int i = 0;
        i < HOME_AXIS_COUNT;
        ++i)
    {
        if (!IsAxisRequested(
            m_selectedAxisMask,
            i))
        {
            continue;
        }

        AxisContext& axis =
            m_motion.GetAxisContext(i);

        if (axis.homeRuntime.completed)
        {
            continue;
        }

        if (!DisarmDriveProbe(
            i,
            axis,
            true))
        {
            allDisarmQueued = false;
            continue;
        }

        axis.homeRuntime.active =
            false;

        axis.homeRuntime.selected =
            false;

        axis.homeRuntime.error =
            HomeErrorReason::CANCELLED;

        axis.homeRuntime.state =
            HomeState::IDLE;

        axis.homeRuntime.motionCommandIssued =
            false;

        axis.homeRuntime.stateTargetValid =
            false;

        axis.isHomed =
            false;
    }


    if (!allDisarmQueued)
    {
        return;
    }

    m_activeAxisMask =
        0;

    m_selectedAxisMask =
        0;

    m_active =
        false;

    m_completed =
        false;

    m_cancelRequested =
        false;

    ClearResumeRequest();

    m_runControlState =
        HomeRunControlState::IDLE;

    m_currentOrder =
        -1;

    RestoreOrReleaseHomeMotionOwner();
}


// ============================================================
// HOME Feed Hold
//
// 重要原則：
//
// 1. m_active 保持 true，G81 Wait Callback 不會越過本行。
// 2. HomeState / HomeOrder / Barrier / Capture Position 全部保留。
// 3. HOME Timeout 在 HOLD_DECEL_STOP / PAUSED 期間不累加。
// 4. 減速途中仍監看預期 DOG / LIMIT 與 INDEX Capture。
// ============================================================

void HomingManager::ProcessHold(
    double cycleTimeSec)
{
    (void)cycleTimeSec;

    bool allStopped =
        true;


    for (int i = 0;
        i < HOME_AXIS_COUNT;
        ++i)
    {
        if (!IsAxisRequested(
            m_activeAxisMask,
            i))
        {
            continue;
        }

        AxisContext& axis =
            m_motion.GetAxisContext(i);

        if (!axis.homeRuntime.active ||
            axis.homeRuntime.completed)
        {
            continue;
        }


        // Feed Hold 減速期間若剛好碰到 DOG / 預期 LIMIT，
        // 必須保存事件，Resume 後不可再往 Sensor 裡推。
        if (!MonitorSwitchDuringHold(
            i,
            axis))
        {
            return;
        }


        // Feed Hold 減速期間若剛好經過 INDEX，
        // 必須保存 60B9/60BA 或 External IO Capture。
        if (!MonitorReferenceDuringHold(
            i,
            axis))
        {
            return;
        }


        if (axis.state !=
            MotionState::MotionState_IDLE)
        {
            allStopped =
                false;

            if (axis.state !=
                MotionState::MotionState_STOPPING)
            {
                QueueHomeStop(i, GetHoldDecTime(axis));
            }
        }
    }


    if (!allStopped)
    {
        return;
    }


    // P2P 在 StopMove 後 finalTargetPos 已變成暫停停止點。
    // 保留 stateTargetPulse，Resume 時重新派送原始目標。
    for (int i = 0;
        i < HOME_AXIS_COUNT;
        ++i)
    {
        if (!IsAxisRequested(
            m_activeAxisMask,
            i))
        {
            continue;
        }

        AxisContext& axis =
            m_motion.GetAxisContext(i);

        if (IsP2PResumeState(
            axis.homeRuntime.state) &&
            axis.homeRuntime.stateTargetValid)
        {
            axis.homeRuntime.motionCommandIssued =
                false;
        }
    }


    m_runControlState =
        HomeRunControlState::PAUSED;


    // 操作員可能在減速尚未完成前就先按 C12。
    if (m_resumeRequested)
    {
        (void)TryResumeFromPaused();
    }
}


HomingManager::ResumeResult HomingManager::TryResumeFromPaused() noexcept
{
    if (!m_active ||
        m_hasError ||
        m_cancelRequested ||
        m_runControlState !=
        HomeRunControlState::PAUSED ||
        !m_resumeRequested ||
        !m_resumeAdmissionTicketValid ||
        !m_resumeHomeMotionLease.IsValid() ||
        !m_resumeHomeMotionLease.Matches(m_homeMotionLease) ||
        m_nc == nullptr)
    {
        ClearResumeRequest();
        return ResumeResult::SUPERSEDED;
    }

    AlarmManager& alarms = AlarmManager::GetInstance();
    AlarmManager::MotionAdmissionReservation admission{};
    const AlarmManager::MotionAdmissionResult beginResult =
        alarms.TryBeginMotionAdmission(
            m_resumeAlarmUpdateCount,
            m_resumeAlarmIntentBaseState,
            admission,
            true);
    if (beginResult ==
        AlarmManager::MotionAdmissionResult::BUSY)
    {
        return ResumeResult::DEFERRED;
    }
    if (beginResult !=
        AlarmManager::MotionAdmissionResult::ACQUIRED)
    {
        ClearResumeRequest();
        return ResumeResult::SUPERSEDED;
    }

    // Revalidate all HOME/NC/Motion identities only after admission.  A
    // queued request never retargets a newer HOME owner generation.
    if (!m_active ||
        m_hasError ||
        m_cancelRequested ||
        m_runControlState !=
        HomeRunControlState::PAUSED ||
        !m_resumeRequested ||
        !m_resumeAdmissionTicketValid ||
        !m_resumeHomeMotionLease.Matches(m_homeMotionLease) ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_resumeHomeMotionLease) ||
        m_nc == nullptr ||
        m_nc->GetState() != NCState::HOLD)
    {
        const bool admissionEnded =
            alarms.EndMotionAdmission(admission);
        ClearResumeRequest();
        (void)admissionEnded;
        return ResumeResult::SUPERSEDED;
    }


    int errorAxisIndex = -1;
    HomeErrorReason resumeError = HomeErrorReason::NONE;
    for (int i = 0;
        i < HOME_AXIS_COUNT;
        ++i)
    {
        if (!IsAxisRequested(
            m_activeAxisMask,
            i))
        {
            continue;
        }

        AxisContext& axis =
            m_motion.GetAxisContext(i);

        if (!axis.homeRuntime.active ||
            axis.homeRuntime.completed)
        {
            continue;
        }


        if (!axis.isServoOn)
        {
            errorAxisIndex = i;
            resumeError = HomeErrorReason::SERVO_NOT_READY;
            break;
        }


        if (axis.isFault ||
            axis.isLagAlarm ||
            axis.state ==
            MotionState::MotionState_ERROR ||
            axis.state ==
            MotionState::MotionState_ESTOP)
        {
            errorAxisIndex = i;
            resumeError = HomeErrorReason::MOTION_FAULT;
            break;
        }


        if (axis.state !=
            MotionState::MotionState_IDLE)
        {
            const bool admissionEnded =
                alarms.EndMotionAdmission(admission);
            if (!admissionEnded)
            {
                ClearResumeRequest();
                return ResumeResult::SUPERSEDED;
            }
            return ResumeResult::DEFERRED;
        }
    }

    if (resumeError != HomeErrorReason::NONE)
    {
        const bool admissionEnded =
            alarms.EndMotionAdmission(admission);
        AxisContext& errorAxis =
            m_motion.GetAxisContext(errorAxisIndex);
        SetAxisError(
            errorAxisIndex,
            errorAxis,
            resumeError);
        return admissionEnded
            ? ResumeResult::REJECTED
            : ResumeResult::SUPERSEDED;
    }

    if (!alarms.IsMotionAdmissionCurrent(admission) ||
        !m_active ||
        m_hasError ||
        m_cancelRequested ||
        m_runControlState !=
        HomeRunControlState::PAUSED ||
        !m_resumeRequested ||
        !m_resumeAdmissionTicketValid ||
        !m_resumeHomeMotionLease.Matches(m_homeMotionLease) ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_resumeHomeMotionLease) ||
        m_nc == nullptr ||
        m_nc->GetState() != NCState::HOLD)
    {
        const bool admissionEnded =
            alarms.EndMotionAdmission(admission);
        ClearResumeRequest();
        (void)admissionEnded;
        return ResumeResult::SUPERSEDED;
    }

    // Validation and mutation are intentionally separate.  No earlier axis
    // receives a partial PID reset when a later axis is still stopping.
    for (int i = 0;
        i < HOME_AXIS_COUNT;
        ++i)
    {
        if (!IsAxisRequested(
            m_activeAxisMask,
            i))
        {
            continue;
        }

        AxisContext& axis =
            m_motion.GetAxisContext(i);
        if (!axis.homeRuntime.active ||
            axis.homeRuntime.completed)
        {
            continue;
        }

        // Bumpless Resume：不改 Search / Backoff / Index 起點，
        // 只清除 PID 歷史，避免暫停期間殘留積分造成起步突波。
        axis.pid.prevError =
            0.0;

        axis.pid.integralAcc =
            0.0;
    }

    // Reset/Alarm-independent owner and NC transitions are not covered by the
    // Alarm word.  Re-prove them at the final mutation seam as well.
    if (!alarms.IsMotionAdmissionCurrent(admission) ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_resumeHomeMotionLease) ||
        m_nc == nullptr ||
        m_nc->GetState() != NCState::HOLD)
    {
        (void)alarms.EndMotionAdmission(admission);
        ClearResumeRequest();
        return ResumeResult::SUPERSEDED;
    }

    // NC owns the state CAS.  Its lease -> HOLD/RUN CAS -> lease proof makes a
    // concurrent Reset/Alarm state win without a raw store from HOME.
    if (!m_nc->TryCommitHomingResume(m_resumeHomeMotionLease))
    {
        (void)alarms.EndMotionAdmission(admission);
        ClearResumeRequest();
        return ResumeResult::SUPERSEDED;
    }

    m_runControlState =
        HomeRunControlState::RUNNING;
    if (m_nc->GetState() != NCState::RUN ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_resumeHomeMotionLease))
    {
        m_runControlState =
            HomeRunControlState::PAUSED;
        m_nc->TryRollbackHomingResume();
        (void)alarms.EndMotionAdmission(admission);
        ClearResumeRequest();
        return ResumeResult::SUPERSEDED;
    }

    ClearResumeRequest();

    if (!alarms.EndMotionAdmission(admission))
    {
        // RUN was only provisional while the reservation was held.  No NC
        // dispatch can occur inside this HOME call.  Restore HOLD before
        // returning and force the overlapping Alarm to remain fail-closed.
        if (m_active &&
            !m_hasError &&
            !m_cancelRequested)
        {
            m_runControlState =
                HomeRunControlState::PAUSED;
        }
        m_nc->TryRollbackHomingResume();
        m_motion.RequestEmergencyStopAllAxes();
        return ResumeResult::SUPERSEDED;
    }

    return ResumeResult::APPLIED;
}


double HomingManager::GetHoldDecTime(
    const AxisContext& axis) const
{
    switch (axis.homeRuntime.state)
    {
    case HomeState::SEARCH_SWITCH:
    case HomeState::SWITCH_DECEL_STOP:

        return
            axis.home.switchStopDecTime;


    case HomeState::SEARCH_INDEX:
    case HomeState::INDEX_CAPTURED:
    case HomeState::INDEX_DECEL_STOP:

        return
            axis.home.indexStopDecTime;


    case HomeState::BACK_OFF:

        return
            axis.home.backoffDecTime;


    case HomeState::MOVE_TO_ZERO:

        return
            axis.home.moveToZeroDecTime;


    default:

        return
            axis.home.searchDecTime;
    }
}


bool HomingManager::IsP2PResumeState(
    HomeState state) const
{
    return
        state == HomeState::BACK_OFF ||
        state == HomeState::MOVE_TO_ZERO;
}


bool HomingManager::MonitorSwitchDuringHold(
    int axisIndex,
    AxisContext& axis)
{
    if (axis.homeRuntime.state !=
        HomeState::SEARCH_SWITCH)
    {
        return true;
    }


    if (axis.hardLimitPositive &&
        axis.hardLimitNegative)
    {
        SetAxisError(
            axisIndex,
            axis,
            HomeErrorReason::BOTH_HARD_LIMITS);

        return false;
    }


    if (UsesDogSwitch(
        axis.home.method))
    {
        if (axis.hardLimitPositive ||
            axis.hardLimitNegative)
        {
            SetAxisError(
                axisIndex,
                axis,
                HomeErrorReason::OPPOSITE_HARD_LIMIT);

            return false;
        }
    }
    else if (UsesHardLimitSwitch(
        axis.home.method))
    {
        const bool oppositeLimit =
            (axis.home.direction > 0 &&
                axis.hardLimitNegative) ||
            (axis.home.direction < 0 &&
                axis.hardLimitPositive);

        if (oppositeLimit)
        {
            SetAxisError(
                axisIndex,
                axis,
                HomeErrorReason::OPPOSITE_HARD_LIMIT);

            return false;
        }
    }


    bool switchActive =
        false;

    if (!ReadHomeSwitch(
        axisIndex,
        axis,
        switchActive))
    {
        SetAxisError(
            axisIndex,
            axis,
            HomeErrorReason::INVALID_CONFIG);

        return false;
    }


    if (!switchActive)
    {
        return true;
    }


    axis.homeRuntime.switchDetectedPulse =
        m_motion.GetRawLogicalPositionPulse(axis);

    axis.homeRuntime.dogDetected =
        UsesDogSwitch(axis.home.method);

    EnterState(
        axis,
        HomeState::SWITCH_DECEL_STOP);

    return true;
}


bool HomingManager::MonitorReferenceDuringHold(
    int axisIndex,
    AxisContext& axis)
{
    if (axis.homeRuntime.state !=
        HomeState::SEARCH_INDEX)
    {
        return true;
    }


    if (!axis.homeRuntime.referenceArmed)
    {
        SetAxisError(
            axisIndex,
            axis,
            HomeErrorReason::REFERENCE_NOT_ARMED);

        return false;
    }


    if (axis.hardLimitPositive &&
        axis.hardLimitNegative)
    {
        SetAxisError(
            axisIndex,
            axis,
            HomeErrorReason::BOTH_HARD_LIMITS);

        return false;
    }


    if (axis.hardLimitPositive ||
        axis.hardLimitNegative)
    {
        SetAxisError(
            axisIndex,
            axis,
            HomeErrorReason::OPPOSITE_HARD_LIMIT);

        return false;
    }


    bool detected =
        false;

    double captured =
        0.0;


    if (axis.home.referenceSource ==
        HomeReferenceSource::EXTERNAL_IO_INDEX)
    {
        bool active =
            false;

        if (!ReadExternalReferenceInput(
            axisIndex,
            axis,
            active))
        {
            SetAxisError(
                axisIndex,
                axis,
                HomeErrorReason::REFERENCE_INVALID);

            return false;
        }


        detected =
            active &&
            !axis.homeRuntime.previousReferenceInput;

        axis.homeRuntime.previousReferenceInput =
            active;


        if (detected)
        {
            captured =
                m_motion.GetRawLogicalPositionPulse(axis);
        }
    }
    else
    {
        if (!TryCaptureDriveProbe(
            axisIndex,
            axis,
            detected,
            captured))
        {
            SetAxisError(
                axisIndex,
                axis,
                HomeErrorReason::REFERENCE_INVALID);

            return false;
        }
    }


    if (!detected)
    {
        return true;
    }


    axis.homeRuntime.capturedReferencePulse =
        captured;

    axis.homeRuntime.referenceDetected =
        true;

    EnterState(
        axis,
        HomeState::INDEX_DECEL_STOP);

    return true;
}

void HomingManager::ProcessBackoffBarrier()
{
    bool any = false;
    bool all = true;
    for (int i = 0; i < HOME_AXIS_COUNT; ++i)
    {
        if (!IsAxisRequested(m_activeAxisMask, i)) continue;
        const HomeState state = m_motion.GetAxisContext(i).homeRuntime.state;
        if (state == HomeState::WAIT_GROUP_BACKOFF) any = true;
        else all = false;
    }
    if (!any || !all) return;

    for (int i = 0; i < HOME_AXIS_COUNT; ++i)
    {
        if (!IsAxisRequested(m_activeAxisMask, i)) continue;
        AxisContext& axis = m_motion.GetAxisContext(i);
        axis.homeRuntime.waitingGroupBarrier = false;
        if (UsesIndexReference(axis.home.method))
            EnterState(axis, HomeState::ARM_REFERENCE);
        else
            EnterState(axis, HomeState::APPLY_HOME);
    }
}

void HomingManager::ProcessIndexStopBarrier()
{
    bool any = false;
    bool all = true;
    for (int i = 0; i < HOME_AXIS_COUNT; ++i)
    {
        if (!IsAxisRequested(m_activeAxisMask, i)) continue;
        const AxisContext& axis = m_motion.GetAxisContext(i);
        if (!UsesIndexReference(axis.home.method)) continue;
        any = true;
        if (axis.homeRuntime.state != HomeState::WAIT_GROUP_INDEX_STOP) all = false;
    }
    if (!any || !all) return;

    for (int i = 0; i < HOME_AXIS_COUNT; ++i)
    {
        if (!IsAxisRequested(m_activeAxisMask, i)) continue;
        AxisContext& axis = m_motion.GetAxisContext(i);
        if (UsesIndexReference(axis.home.method)) EnterState(axis, HomeState::APPLY_HOME);
    }
}

bool HomingManager::ReadHomeSwitch(int axisIndex, AxisContext& axis, bool& active) const
{
    active = false;
    if (UsesDogSwitch(axis.home.method))
    {
        if (m_plc == nullptr) return false;
        const bool raw = m_plc->Get_C(NCPLC::C::AxisPoint(NCPLC::C::HOME_DOG_BASE, axisIndex));
        active = axis.home.dogActiveHigh ? raw : !raw;
        axis.homeRuntime.previousDogInput = raw;
        return true;
    }
    if (UsesHardLimitSwitch(axis.home.method))
    {
        active = axis.home.direction > 0 ? axis.hardLimitPositive : axis.hardLimitNegative;
        return true;
    }
    return false;
}

bool HomingManager::ReadExternalReferenceInput(int axisIndex, const AxisContext& axis, bool& active) const
{
    active = false;
    if (m_plc == nullptr) return false;
    const int cPoint = axis.home.externalReferenceCPoint >= 0
        ? axis.home.externalReferenceCPoint
        : NCPLC::C::AxisPoint(NCPLC::C::HOME_INDEX_BASE, axisIndex);
    const bool raw = m_plc->Get_C(cPoint);
    active = axis.home.referenceActiveHigh ? raw : !raw;
    return true;
}

// ============================================================
// Drive Touch Probe Provider
//
// 0x60B8：Controller -> Drive
// 0x60B9：Drive -> Controller Status
// 0x60BA：Drive -> Controller Positive Edge Capture Position
//
// Bit values are parameterized in HomeConfig.  The state machine does not
// scatter Delta-specific magic numbers through the motion logic.
// ============================================================

bool HomingManager::UsesDriveTouchProbe(
    const AxisContext& axis) const
{
    return
        axis.home.captureMode ==
        HomeReferenceCaptureMode::DRIVE_HARDWARE_LATCH &&
        (axis.home.referenceSource ==
            HomeReferenceSource::MOTOR_ENCODER_INDEX ||
            axis.home.referenceSource ==
            HomeReferenceSource::LINEAR_SCALE_INDEX_DRIVE);
}


bool HomingManager::ValidateDriveProbeConfig(
    const AxisContext& axis) const
{
    if (!UsesDriveTouchProbe(axis))
    {
        return true;
    }

    if (axis.home.driveProbeArmMode !=
        HomeDriveProbeArmMode::CONTROLLER_60B8 &&
        axis.home.driveProbeArmMode !=
        HomeDriveProbeArmMode::DRIVE_AUTO_ARM)
    {
        return false;
    }

    if (axis.home.driveProbeCapturedMask == 0 &&
        axis.home.driveProbeCaptureToggleMask == 0 &&
        !axis.home.driveProbeAllowPositionChangeDetection)
    {
        return false;
    }

    if (!std::isfinite(
        axis.home.driveProbeClearTimeoutSec) ||
        !std::isfinite(
            axis.home.driveProbeArmTimeoutSec) ||
        axis.home.driveProbeClearTimeoutSec < 0.0 ||
        axis.home.driveProbeArmTimeoutSec < 0.0)
    {
        return false;
    }

    if (axis.home.driveProbeArmMode ==
        HomeDriveProbeArmMode::CONTROLLER_60B8)
    {
        if (axis.home.driveProbeArmValue ==
            axis.home.driveProbeDisarmValue)
        {
            return false;
        }

        if (axis.home.driveProbeRequireArmedStatus &&
            axis.home.driveProbeArmedMask == 0)
        {
            return false;
        }

        if (axis.home.driveProbeClearTimeoutSec <= 0.0 ||
            axis.home.driveProbeArmTimeoutSec <= 0.0)
        {
            return false;
        }
    }

    return true;
}


bool HomingManager::IsDriveProbeSourceStatusValid(
    const AxisContext& axis,
    uint16_t status) const
{
    if (axis.home.driveProbeSourceMask == 0)
    {
        return true;
    }

    return
        (status & axis.home.driveProbeSourceMask) ==
        (axis.home.driveProbeExpectedSourceValue &
            axis.home.driveProbeSourceMask);
}


void HomingManager::ResetDriveProbeRuntime(
    AxisContext& axis)
{
    axis.homeRuntime.driveProbePhase =
        HomeDriveProbePhase::IDLE;

    axis.homeRuntime.driveProbePhaseElapsedSec =
        0.0;

    axis.homeRuntime.driveProbeBaselineStatus =
        0;

    axis.homeRuntime.driveProbeBaselinePosition =
        0;

    axis.homeRuntime.driveProbeLastFunction =
        0;

    axis.homeRuntime.driveProbeLastStatus =
        0;

    axis.homeRuntime.driveProbeLastPosition =
        0;

    axis.homeRuntime.driveProbeControlWritten =
        false;

    axis.homeRuntime.driveProbeClearConfirmed =
        false;

    axis.homeRuntime.driveProbeArmedConfirmed =
        false;

    axis.homeRuntime.driveProbeCaptureConfirmed =
        false;

    axis.homeRuntime.driveProbeDisarmedAfterCapture =
        false;
}


bool HomingManager::DisarmDriveProbe(
    int axisIndex,
    AxisContext& axis,
    bool trackForOwnerRelease)
{
    if (!UsesDriveTouchProbe(axis))
    {
        return true;
    }

    if (axis.home.driveProbeArmMode !=
        HomeDriveProbeArmMode::CONTROLLER_60B8)
    {
        return true;
    }

    if (trackForOwnerRelease &&
        axisIndex >= 0 &&
        axisIndex < HOME_AXIS_COUNT &&
        m_pendingProbeDisarmSequence[axisIndex] !=
        MOTION_AXIS_COMMAND_SEQUENCE_INVALID)
    {
        return true;
    }

    MotionAxisCommandSequence sequence =
        MOTION_AXIS_COMMAND_SEQUENCE_INVALID;

    if (!QueueHomeProbeFunction(
        axisIndex,
        axis.home.driveProbeDisarmValue,
        trackForOwnerRelease ? &sequence : nullptr))
    {
        return false;
    }

    if (trackForOwnerRelease &&
        axisIndex >= 0 &&
        axisIndex < HOME_AXIS_COUNT)
    {
        m_pendingProbeDisarmSequence[axisIndex] = sequence;
    }

    axis.homeRuntime.driveProbeLastFunction =
        axis.home.driveProbeDisarmValue;

    axis.homeRuntime.driveProbeDisarmedAfterCapture =
        true;

    return true;
}


bool HomingManager::ProcessDriveProbeArm(
    int axisIndex,
    AxisContext& axis,
    double cycleTimeSec)
{
    if (!UsesDriveTouchProbe(axis) ||
        !ValidateDriveProbeConfig(axis))
    {
        SetAxisError(
            axisIndex,
            axis,
            HomeErrorReason::INVALID_CONFIG);

        return false;
    }

    if (cycleTimeSec > 0.0 &&
        std::isfinite(cycleTimeSec))
    {
        axis.homeRuntime.driveProbePhaseElapsedSec +=
            cycleTimeSec;
    }

    uint16_t functionValue =
        0;

    uint16_t status =
        0;

    int32_t position =
        0;

    if (!m_motion.GetDriveTouchProbeFunction(
        axisIndex,
        functionValue) ||
        !m_motion.GetDriveTouchProbeData(
            axisIndex,
            status,
            position))
    {
        SetAxisError(
            axisIndex,
            axis,
            HomeErrorReason::REFERENCE_INVALID);

        return false;
    }

    axis.homeRuntime.driveProbeLastFunction =
        functionValue;

    axis.homeRuntime.driveProbeLastStatus =
        status;

    axis.homeRuntime.driveProbeLastPosition =
        position;

    // Source Status is validated only after the probe is armed.
    // Some drives do not publish the source bit while 0x60B8 is disabled.

    // --------------------------------------------------------
    // Drive Parameter Auto Arm
    //
    // Do not write 0x60B8.  Establish a baseline and only accept a
    // new 0x60B9 event / optional 0x60BA change during SEARCH_INDEX.
    // --------------------------------------------------------

    if (axis.home.driveProbeArmMode ==
        HomeDriveProbeArmMode::DRIVE_AUTO_ARM)
    {
        if (!IsDriveProbeSourceStatusValid(
            axis,
            status))
        {
            SetAxisError(
                axisIndex,
                axis,
                HomeErrorReason::REFERENCE_INVALID);

            return false;
        }

        axis.homeRuntime.driveProbeBaselineStatus =
            status;

        axis.homeRuntime.driveProbeBaselinePosition =
            position;

        axis.homeRuntime.previousTouchProbeStatus =
            status;

        axis.homeRuntime.capturedReferenceRaw =
            position;

        axis.homeRuntime.driveProbeArmedConfirmed =
            true;

        axis.homeRuntime.driveProbePhase =
            HomeDriveProbePhase::READY;

        axis.homeRuntime.referenceArmed =
            true;

        axis.homeRuntime.indexSearchStartPulse =
            m_motion.GetRawLogicalPositionPulse(axis);

        return true;
    }

    // --------------------------------------------------------
    // Controller Arm through 0x60B8
    // --------------------------------------------------------

    switch (axis.homeRuntime.driveProbePhase)
    {
    case HomeDriveProbePhase::IDLE:

        axis.homeRuntime.driveProbePhase =
            HomeDriveProbePhase::WRITE_DISARM;

        axis.homeRuntime.driveProbePhaseElapsedSec =
            0.0;

        return false;


    case HomeDriveProbePhase::WRITE_DISARM:

        if (!QueueHomeProbeFunction(
            axisIndex,
            axis.home.driveProbeDisarmValue))
        {
            SetAxisError(
                axisIndex,
                axis,
                HomeErrorReason::REFERENCE_INVALID);

            return false;
        }

        axis.homeRuntime.driveProbeControlWritten =
            true;

        axis.homeRuntime.driveProbeLastFunction =
            axis.home.driveProbeDisarmValue;

        axis.homeRuntime.driveProbePhase =
            HomeDriveProbePhase::WAIT_CLEAR;

        axis.homeRuntime.driveProbePhaseElapsedSec =
            0.0;

        return false;


    case HomeDriveProbePhase::WAIT_CLEAR:
    {
        const bool captureCleared =
            (status & axis.home.driveProbeCapturedMask) == 0;

        const bool armedCleared =
            !axis.home.driveProbeRequireArmedStatus ||
            axis.home.driveProbeArmedMask == 0 ||
            (status & axis.home.driveProbeArmedMask) == 0;

        if (captureCleared &&
            armedCleared)
        {
            axis.homeRuntime.driveProbeClearConfirmed =
                true;

            axis.homeRuntime.driveProbeBaselineStatus =
                status;

            axis.homeRuntime.driveProbeBaselinePosition =
                position;

            axis.homeRuntime.driveProbePhase =
                HomeDriveProbePhase::WRITE_ARM;

            axis.homeRuntime.driveProbePhaseElapsedSec =
                0.0;

            return false;
        }

        if (axis.homeRuntime.driveProbePhaseElapsedSec >=
            axis.home.driveProbeClearTimeoutSec)
        {
            SetAxisError(
                axisIndex,
                axis,
                HomeErrorReason::REFERENCE_NOT_ARMED);
        }

        return false;
    }


    case HomeDriveProbePhase::WRITE_ARM:

        if (!QueueHomeProbeFunction(
            axisIndex,
            axis.home.driveProbeArmValue))
        {
            SetAxisError(
                axisIndex,
                axis,
                HomeErrorReason::REFERENCE_INVALID);

            return false;
        }

        axis.homeRuntime.driveProbeLastFunction =
            axis.home.driveProbeArmValue;

        axis.homeRuntime.driveProbePhase =
            HomeDriveProbePhase::WAIT_ARMED;

        axis.homeRuntime.driveProbePhaseElapsedSec =
            0.0;

        return false;


    case HomeDriveProbePhase::WAIT_ARMED:
    {
        const bool armed =
            !axis.home.driveProbeRequireArmedStatus ||
            axis.home.driveProbeArmedMask == 0 ||
            (status & axis.home.driveProbeArmedMask) != 0;

        const bool staleCapture =
            axis.home.driveProbeRequireNewCapture &&
            axis.home.driveProbeCapturedMask != 0 &&
            (status & axis.home.driveProbeCapturedMask) != 0;

        const bool sourceValid =
            IsDriveProbeSourceStatusValid(
                axis,
                status);

        if (armed &&
            !staleCapture &&
            sourceValid)
        {
            axis.homeRuntime.driveProbeArmedConfirmed =
                true;

            axis.homeRuntime.driveProbeBaselineStatus =
                status;

            axis.homeRuntime.driveProbeBaselinePosition =
                position;

            axis.homeRuntime.previousTouchProbeStatus =
                status;

            axis.homeRuntime.capturedReferenceRaw =
                position;

            axis.homeRuntime.driveProbePhase =
                HomeDriveProbePhase::READY;

            axis.homeRuntime.driveProbePhaseElapsedSec =
                0.0;

            axis.homeRuntime.referenceArmed =
                true;

            axis.homeRuntime.indexSearchStartPulse =
                m_motion.GetRawLogicalPositionPulse(axis);

            return true;
        }

        if (axis.homeRuntime.driveProbePhaseElapsedSec >=
            axis.home.driveProbeArmTimeoutSec)
        {
            SetAxisError(
                axisIndex,
                axis,
                HomeErrorReason::REFERENCE_NOT_ARMED);
        }

        return false;
    }


    case HomeDriveProbePhase::READY:

        return true;


    case HomeDriveProbePhase::CAPTURED:
    case HomeDriveProbePhase::DISARM_AFTER_CAPTURE:

        return axis.homeRuntime.referenceArmed;


    case HomeDriveProbePhase::PROBE_ERROR:
    default:

        SetAxisError(
            axisIndex,
            axis,
            HomeErrorReason::REFERENCE_INVALID);

        return false;
    }
}


bool HomingManager::TryCaptureDriveProbe(
    int axisIndex,
    AxisContext& axis,
    bool& detected,
    double& capturedPulse)
{
    detected =
        false;

    capturedPulse =
        0.0;

    if (!UsesDriveTouchProbe(axis) ||
        !axis.homeRuntime.referenceArmed)
    {
        return false;
    }

    uint16_t functionValue =
        0;

    uint16_t status =
        0;

    int32_t position =
        0;

    if (!m_motion.GetDriveTouchProbeFunction(
        axisIndex,
        functionValue) ||
        !m_motion.GetDriveTouchProbeData(
            axisIndex,
            status,
            position))
    {
        return false;
    }

    axis.homeRuntime.driveProbeLastFunction =
        functionValue;

    axis.homeRuntime.driveProbeLastStatus =
        status;

    axis.homeRuntime.driveProbeLastPosition =
        position;

    if (!IsDriveProbeSourceStatusValid(
        axis,
        status))
    {
        return false;
    }

    const uint16_t previousStatus =
        axis.homeRuntime.previousTouchProbeStatus;

    const bool capturedLevel =
        axis.home.driveProbeCapturedMask != 0 &&
        (status & axis.home.driveProbeCapturedMask) != 0;

    const bool previousCapturedLevel =
        axis.home.driveProbeCapturedMask != 0 &&
        (previousStatus & axis.home.driveProbeCapturedMask) != 0;

    const bool captureRisingEdge =
        capturedLevel &&
        !previousCapturedLevel;

    const bool captureToggleChanged =
        axis.home.driveProbeCaptureToggleMask != 0 &&
        ((status ^ previousStatus) &
            axis.home.driveProbeCaptureToggleMask) != 0;

    const bool positionChanged =
        axis.home.driveProbeAllowPositionChangeDetection &&
        position != axis.homeRuntime.capturedReferenceRaw;

    if (axis.home.driveProbeRequireNewCapture)
    {
        detected =
            captureRisingEdge ||
            captureToggleChanged ||
            positionChanged;
    }
    else
    {
        detected =
            capturedLevel ||
            captureToggleChanged ||
            positionChanged;
    }

    axis.homeRuntime.previousTouchProbeStatus =
        status;

    if (!detected)
    {
        return true;
    }

    axis.homeRuntime.capturedReferenceRaw =
        position;

    axis.homeRuntime.driveProbeCaptureConfirmed =
        true;

    axis.homeRuntime.driveProbePhase =
        HomeDriveProbePhase::CAPTURED;

    capturedPulse =
        m_motion.ConvertDriveCaptureToRawLogicalPulse(
            axisIndex,
            position,
            axis.home.referenceSource);

    if (!std::isfinite(capturedPulse))
    {
        return false;
    }

    if (axis.home.driveProbeDisarmAfterCapture)
    {
        DisarmDriveProbe(
            axisIndex,
            axis);

        axis.homeRuntime.driveProbePhase =
            HomeDriveProbePhase::DISARM_AFTER_CAPTURE;
    }

    return true;
}


double HomingManager::GetPulsePerUnit(const AxisContext& axis) const
{
    if (axis.resolution_PPR <= 0.0 || std::abs(axis.finalLead) < 1.0e-12) return 0.0;
    return axis.resolution_PPR / std::abs(axis.finalLead);
}

double HomingManager::GetTravelDistanceUnit(const AxisContext& axis, double startRawPulse) const
{
    const double ppu = GetPulsePerUnit(axis);
    if (ppu <= 0.0) return 0.0;
    return std::abs(m_motion.GetRawLogicalPositionPulse(axis) - startRawPulse) / ppu;
}

void HomingManager::EnterState(AxisContext& axis, HomeState state)
{
    axis.homeRuntime.state = state;
    axis.homeRuntime.stateElapsedSec = 0.0;
    axis.homeRuntime.motionCommandIssued = false;
    axis.homeRuntime.stateTargetValid = false;
    axis.homeRuntime.stateTargetPulse = 0.0;

    if (state == HomeState::ARM_REFERENCE)
    {
        ResetDriveProbeRuntime(axis);
    }
}

// ============================================================
// Per-Axis HOME State Machine
//
// PREPARE -> SEARCH_SWITCH -> SWITCH_DECEL_STOP
// -> BACK_OFF -> VALIDATE_RELEASE -> Group Barrier
// -> ARM_REFERENCE -> SEARCH_INDEX -> INDEX_DECEL_STOP
// -> Group Barrier -> APPLY_HOME -> Optional MOVE_TO_ZERO
// ============================================================

void HomingManager::ProcessAxis(
    int axisIndex,
    AxisContext& axis,
    double cycleTimeSec)
{
    if (!axis.homeRuntime.active)
    {
        return;
    }


    axis.homeRuntime.stateElapsedSec +=
        cycleTimeSec;


    // ========================================================
    // PREPARE
    //
    // 這一階段只做：
    //
    // 1. 再次確認 HOME 參數安全
    // 2. 建立本次 HOME Runtime 起始快照
    // 3. 依 HomeMethod 決定下一個 State
    //
    // 這裡「不會」下達任何 Motion。
    // ========================================================

    if (axis.homeRuntime.state ==
        HomeState::PREPARE)
    {
        // ----------------------------------------------------
        // A. Runtime 基本狀態再次確認
        //
        // Start() 到 Process() 之間即使狀態發生變化，
        // 也不能直接進入 HOME Motion。
        // ----------------------------------------------------

        if (!axis.isExist)
        {
            SetAxisError(
                axisIndex,
                axis,
                HomeErrorReason::AXIS_NOT_EXIST);

            return;
        }


        if (!axis.home.enabled)
        {
            SetAxisError(
                axisIndex,
                axis,
                HomeErrorReason::INVALID_CONFIG);

            return;
        }


        if (!axis.isServoOn)
        {
            SetAxisError(
                axisIndex,
                axis,
                HomeErrorReason::SERVO_NOT_READY);

            return;
        }


        if (axis.isFault ||
            axis.isLagAlarm ||
            axis.state ==
            MotionState::MotionState_ERROR ||
            axis.state ==
            MotionState::MotionState_ESTOP)
        {
            SetAxisError(
                axisIndex,
                axis,
                HomeErrorReason::SERVO_FAULT);

            return;
        }


        if (axis.state !=
            MotionState::MotionState_IDLE)
        {
            SetAxisError(
                axisIndex,
                axis,
                HomeErrorReason::MOTION_BUSY);

            return;
        }


        if (axis.home.direction != -1 &&
            axis.home.direction != 1)
        {
            SetAxisError(
                axisIndex,
                axis,
                HomeErrorReason::INVALID_CONFIG);

            return;
        }


        // HOME 距離與速度換算都依賴 Encoder Resolution
        // 與 Final Lead，這兩個機械參數必須有效。
        if (!std::isfinite(axis.resolution_PPR) ||
            !std::isfinite(axis.finalLead) ||
            axis.resolution_PPR <= 0.0 ||
            std::abs(axis.finalLead) < 1.0e-12)
        {
            SetAxisError(
                axisIndex,
                axis,
                HomeErrorReason::INVALID_CONFIG);

            return;
        }


        // ----------------------------------------------------
        // B. 共用參數防呆
        // ----------------------------------------------------

        auto IsPositiveFinite =
            [](double value) -> bool
        {
            return
                std::isfinite(value) &&
                value > 0.0;
        };


        auto IsNonNegativeFinite =
            [](double value) -> bool
        {
            return
                std::isfinite(value) &&
                value >= 0.0;
        };


        const bool usesSwitch =
            UsesDogSwitch(
                axis.home.method) ||
            UsesHardLimitSwitch(
                axis.home.method);


        const bool usesIndex =
            UsesIndexReference(
                axis.home.method);


        // ----------------------------------------------------
        // C. DOG / LIMIT Search 參數
        //
        // 需要搜尋 Switch 的模式：
        //
        // DOG_INDEX
        // LIMIT_INDEX
        // DOG_ONLY
        // LIMIT_ONLY
        //
        // 必須具備：
        //
        // Search Speed
        // Search Max Distance
        // Controlled Stop Dec Time
        // Backoff Speed
        // Backoff Max Distance
        // ----------------------------------------------------

        if (usesSwitch)
        {
            if (!IsPositiveFinite(
                axis.home.searchSpeed_PPS) ||
                !IsPositiveFinite(
                    axis.home.searchMaxDistance_unit) ||
                !IsPositiveFinite(
                    axis.home.switchStopDecTime) ||
                !IsPositiveFinite(
                    axis.home.backoffSpeed_PPS) ||
                !IsPositiveFinite(
                    axis.home.backoffMaxDistance_unit))
            {
                SetAxisError(
                    axisIndex,
                    axis,
                    HomeErrorReason::INVALID_CONFIG);

                return;
            }


            if (!IsNonNegativeFinite(
                axis.home.searchTimeoutSec) ||
                !IsNonNegativeFinite(
                    axis.home.switchStopMaxDistance_unit) ||
                !IsNonNegativeFinite(
                    axis.home.backoffTimeoutSec))
            {
                SetAxisError(
                    axisIndex,
                    axis,
                    HomeErrorReason::INVALID_CONFIG);

                return;
            }


            // ------------------------------------------------
            // Backoff Mode 專用距離檢查
            // ------------------------------------------------

            if (axis.home.backoffMode ==
                HomeBackoffMode::FIXED_DISTANCE)
            {
                if (!IsPositiveFinite(
                    axis.home.backoffDistance_unit))
                {
                    SetAxisError(
                        axisIndex,
                        axis,
                        HomeErrorReason::INVALID_CONFIG);

                    return;
                }
            }
            else if (axis.home.backoffMode ==
                HomeBackoffMode::UNTIL_DOG_OFF_PLUS_DISTANCE)
            {
                if (!IsNonNegativeFinite(
                    axis.home.backoffExtraDistance_unit))
                {
                    SetAxisError(
                        axisIndex,
                        axis,
                        HomeErrorReason::INVALID_CONFIG);

                    return;
                }
            }
            else
            {
                SetAxisError(
                    axisIndex,
                    axis,
                    HomeErrorReason::INVALID_CONFIG);

                return;
            }


            // DOG Mode 必須要有 PLC，
            // 因為 HOME DOG 是 PLC C Point。
            if (UsesDogSwitch(
                axis.home.method) &&
                m_plc == nullptr)
            {
                SetAxisError(
                    axisIndex,
                    axis,
                    HomeErrorReason::INVALID_CONFIG);

                return;
            }
        }


        // ----------------------------------------------------
        // D. INDEX Search 參數
        //
        // 需要 INDEX 的模式：
        //
        // DOG_INDEX
        // LIMIT_INDEX
        // INDEX_ONLY
        //
        // HomeIndexMaxDistance 強制要求 > 0。
        //
        // 不允許無限搜尋 INDEX。
        // ----------------------------------------------------

        if (usesIndex)
        {
            if (!IsPositiveFinite(
                axis.home.indexSearchSpeed_PPS) ||
                !IsPositiveFinite(
                    axis.home.indexStopDecTime) ||
                !IsPositiveFinite(
                    axis.home.indexMaxDistance_unit) ||
                !IsNonNegativeFinite(
                    axis.home.indexTimeoutSec))
            {
                SetAxisError(
                    axisIndex,
                    axis,
                    HomeErrorReason::INVALID_CONFIG);

                return;
            }


            if (axis.home.referenceSource ==
                HomeReferenceSource::NONE)
            {
                SetAxisError(
                    axisIndex,
                    axis,
                    HomeErrorReason::INVALID_CONFIG);

                return;
            }


            // ------------------------------------------------
            // Reference Source / Capture Mode 組合驗證
            //
            // 第一版正式可測組合：
            //
            // 1. Motor / Drive Scale INDEX
            //      -> DRIVE_HARDWARE_LATCH
            //
            // 2. External IO INDEX
            //      -> SOFTWARE_SAMPLE
            //
            // EXTERNAL_HARDWARE_LATCH 需要外部板子的
            // Capture Register API，目前尚未提供，不能假裝支援。
            // ------------------------------------------------

            if (axis.home.referenceSource ==
                HomeReferenceSource::MOTOR_ENCODER_INDEX ||
                axis.home.referenceSource ==
                HomeReferenceSource::LINEAR_SCALE_INDEX_DRIVE)
            {
                if (axis.home.captureMode !=
                    HomeReferenceCaptureMode::DRIVE_HARDWARE_LATCH)
                {
                    SetAxisError(
                        axisIndex,
                        axis,
                        HomeErrorReason::INVALID_CONFIG);

                    return;
                }
            }
            else if (axis.home.referenceSource ==
                HomeReferenceSource::EXTERNAL_IO_INDEX)
            {
                if (axis.home.captureMode !=
                    HomeReferenceCaptureMode::SOFTWARE_SAMPLE ||
                    m_plc == nullptr)
                {
                    SetAxisError(
                        axisIndex,
                        axis,
                        HomeErrorReason::INVALID_CONFIG);

                    return;
                }
            }
            else
            {
                SetAxisError(
                    axisIndex,
                    axis,
                    HomeErrorReason::INVALID_CONFIG);

                return;
            }


            if (UsesDriveTouchProbe(axis) &&
                !ValidateDriveProbeConfig(axis))
            {
                SetAxisError(
                    axisIndex,
                    axis,
                    HomeErrorReason::INVALID_CONFIG);

                return;
            }
        }


        // ----------------------------------------------------
        // E. Absolute Reference
        //
        // 目前沒有 Delta Absolute Position / Multi-turn Provider，
        // 因此明確拒絕，不用目前位置冒充 Absolute Reference。
        // ----------------------------------------------------

        if (axis.home.method ==
            HomeMethod::ABSOLUTE_REFERENCE)
        {
            SetAxisError(
                axisIndex,
                axis,
                HomeErrorReason::INVALID_CONFIG);

            return;
        }


        // ----------------------------------------------------
        // F. Mechanical Stop
        //
        // 第一版尚未實作。
        // ----------------------------------------------------

        if (axis.home.method ==
            HomeMethod::MECHANICAL_STOP)
        {
            SetAxisError(
                axisIndex,
                axis,
                HomeErrorReason::INVALID_CONFIG);

            return;
        }


        // ----------------------------------------------------
        // G. Runtime Snapshot 初始化
        // ----------------------------------------------------

        axis.homeRuntime.error =
            HomeErrorReason::NONE;

        axis.homeRuntime.completed =
            false;

        axis.homeRuntime.dogDetected =
            false;

        axis.homeRuntime.dogReleased =
            false;

        axis.homeRuntime.hardLimitReleased =
            false;

        axis.homeRuntime.referenceArmed =
            false;

        axis.homeRuntime.referenceDetected =
            false;

        axis.homeRuntime.waitingGroupBarrier =
            false;

        axis.homeRuntime.previousDogInput =
            false;

        axis.homeRuntime.previousReferenceInput =
            false;

        axis.homeRuntime.previousTouchProbeStatus =
            0;

        ResetDriveProbeRuntime(axis);


        // HOME Runtime 的距離起點全部保存為 Raw Logical Pulse。
        //
        // 不可混用 axis.currentActPos（Machine Pulse），
        // 否則 ApplyMachineHome() 建立 Machine Offset 後，
        // 下一次 HOME 的距離保護會把 Offset 誤算成實際移動距離。
        const double prepareRawPulse =
            m_motion.GetRawLogicalPositionPulse(
                axis);

        axis.homeRuntime.searchStartPulse =
            prepareRawPulse;

        axis.homeRuntime.stateStartPulse =
            prepareRawPulse;

        axis.homeRuntime.backoffStartPulse =
            prepareRawPulse;

        axis.homeRuntime.indexSearchStartPulse =
            prepareRawPulse;

        axis.homeRuntime.switchDetectedPulse =
            prepareRawPulse;

        axis.homeRuntime.capturedReferenceRaw =
            0;

        axis.homeRuntime.capturedReferencePulse =
            0.0;

        axis.homeRuntime.stateElapsedSec =
            0.0;


        // ----------------------------------------------------
        // H. HomeMethod -> 下一個 State
        //
        // 注意：
        // 這裡只改 State，
        // 下一個 Scan 才會開始該 State 的真正工作。
        // ----------------------------------------------------

        switch (axis.home.method)
        {
        case HomeMethod::DOG_INDEX:
        case HomeMethod::LIMIT_INDEX:
        case HomeMethod::DOG_ONLY:
        case HomeMethod::LIMIT_ONLY:

            axis.homeRuntime.state =
                HomeState::SEARCH_SWITCH;

            break;


        case HomeMethod::INDEX_ONLY:

            // 即使不需要 DOG / LIMIT，也先進入 Backoff Barrier，
            // 確保多軸 P0/P1 在同一時點開始 Reference 階段。
            axis.homeRuntime.waitingGroupBarrier = true;
            axis.homeRuntime.state = HomeState::WAIT_GROUP_BACKOFF;

            break;


        case HomeMethod::CURRENT_POSITION:

            // 將目前實際位置當成 Reference。
            //
            // 先保存 Pulse，
            // 後續 APPLY_HOME 再正式建立 Machine Coordinate。
            axis.homeRuntime.capturedReferencePulse =
                m_motion.GetRawLogicalPositionPulse(axis);

            axis.homeRuntime.referenceDetected = true;
            axis.homeRuntime.waitingGroupBarrier = true;
            axis.homeRuntime.state = HomeState::WAIT_GROUP_BACKOFF;

            break;


        case HomeMethod::MECHANICAL_STOP:
        default:

            SetAxisError(
                axisIndex,
                axis,
                HomeErrorReason::INVALID_CONFIG);

            return;
        }


        return;
    }


    // ========================================================
    // SEARCH_SWITCH
    //
    // 第一個真正會移動的 HOME State。
    //
    // DOG_INDEX / DOG_ONLY：
    //
    //     依 HomeDirection 使用 VelocityMove()
    //     搜尋 PLC HOME DOG。
    //
    // LIMIT_INDEX / LIMIT_ONLY：
    //
    //     依 HomeDirection 使用 VelocityMove()
    //     搜尋預期方向的 Physical Hard Limit。
    //
    // 找到後：
    //
    //     StopMove(HomeSwitchStopDecTime)
    //     ↓
    //     SWITCH_DECEL_STOP
    //
    // 正常找到 Sensor 不是 Alarm。
    // ========================================================

    if (axis.homeRuntime.state ==
        HomeState::SEARCH_SWITCH)
    {
        // ----------------------------------------------------
        // A. Runtime Safety
        // ----------------------------------------------------

        if (!axis.isServoOn)
        {
            SetAxisError(
                axisIndex,
                axis,
                HomeErrorReason::SERVO_NOT_READY);

            return;
        }


        if (axis.isFault ||
            axis.isLagAlarm ||
            axis.state ==
            MotionState::MotionState_ERROR ||
            axis.state ==
            MotionState::MotionState_ESTOP)
        {
            // 若外層 Safety 已經先建立 Alarm，
            // 這裡只結束 HOME，不再額外堆第二個 Alarm。
            SetAxisError(
                axisIndex,
                axis,
                HomeErrorReason::MOTION_FAULT);

            return;
        }


        // SEARCH_SWITCH 正常只可能是：
        //
        // IDLE：
        //     尚未真正啟動搜尋。
        //
        // VELOCITY：
        //     正在持續搜尋。
        //
        // 其他 MotionState 都代表 Ownership 被破壞。
        if (axis.state !=
            MotionState::MotionState_IDLE &&
            axis.state !=
            MotionState::MotionState_VELOCITY)
        {
            SetAxisError(
                axisIndex,
                axis,
                HomeErrorReason::MOTION_FAULT);

            return;
        }


        // ----------------------------------------------------
        // B. Hard Limit Safety
        //
        // +OT 與 -OT 同時 ON 永遠是異常。
        //
        // 搜尋 HOME DOG 時：
        //     任一 Hard Limit 都不是預期 HOME Event。
        //
        // 搜尋 LIMIT 時：
        //     只有 HomeDirection 對應的那一側可以是正常 Event。
        // ----------------------------------------------------

        const bool positiveHardLimit =
            axis.hardLimitPositive;

        const bool negativeHardLimit =
            axis.hardLimitNegative;


        if (positiveHardLimit &&
            negativeHardLimit)
        {
            SetAxisError(
                axisIndex,
                axis,
                HomeErrorReason::BOTH_HARD_LIMITS);

            return;
        }


        const bool usesDog =
            UsesDogSwitch(
                axis.home.method);

        const bool usesHardLimit =
            UsesHardLimitSwitch(
                axis.home.method);


        if (usesDog)
        {
            if (positiveHardLimit ||
                negativeHardLimit)
            {
                SetAxisError(
                    axisIndex,
                    axis,
                    HomeErrorReason::OPPOSITE_HARD_LIMIT);

                return;
            }
        }
        else if (usesHardLimit)
        {
            const bool oppositeLimit =
                (axis.home.direction > 0 &&
                    negativeHardLimit) ||
                (axis.home.direction < 0 &&
                    positiveHardLimit);


            if (oppositeLimit)
            {
                SetAxisError(
                    axisIndex,
                    axis,
                    HomeErrorReason::OPPOSITE_HARD_LIMIT);

                return;
            }
        }
        else
        {
            // 能進 SEARCH_SWITCH 的方法一定必須使用
            // DOG 或 LIMIT。
            SetAxisError(
                axisIndex,
                axis,
                HomeErrorReason::INVALID_CONFIG);

            return;
        }


        // ----------------------------------------------------
        // C. Read Target Switch
        // ----------------------------------------------------

        bool switchDetected =
            false;


        if (usesDog)
        {
            if (m_plc == nullptr)
            {
                SetAxisError(
                    axisIndex,
                    axis,
                    HomeErrorReason::INVALID_CONFIG);

                return;
            }


            const bool rawDogInput =
                m_plc->Get_C(
                    NCPLC::C::AxisPoint(
                        NCPLC::C::HOME_DOG_BASE,
                        axisIndex));


            const bool dogActive =
                axis.home.dogActiveHigh
                ? rawDogInput
                : !rawDogInput;


            axis.homeRuntime.previousDogInput =
                rawDogInput;


            switchDetected =
                dogActive;


            if (dogActive)
            {
                axis.homeRuntime.dogDetected =
                    true;
            }
        }
        else
        {
            switchDetected =
                axis.home.direction > 0
                ? positiveHardLimit
                : negativeHardLimit;
        }


        // ----------------------------------------------------
        // D. Switch Found
        //
        // Sensor 已經 ON 時優先處理 Found，
        // 再判斷 MaxDistance / Timeout。
        //
        // 這也支援：
        //
        // HOME 開始前軸已經停在預期 DOG / LIMIT 上。
        //
        // 此時不會繼續往 Sensor 裡面推，
        // 而是直接進 Controlled Stop / Backoff 流程。
        // ----------------------------------------------------

        if (switchDetected)
        {
            axis.homeRuntime.switchDetectedPulse =
                m_motion.GetRawLogicalPositionPulse(axis);

            axis.homeRuntime.stateStartPulse =
                axis.homeRuntime.switchDetectedPulse;

            axis.homeRuntime.capturedReferencePulse =
                axis.homeRuntime.switchDetectedPulse;

            axis.homeRuntime.referenceDetected =
                !UsesIndexReference(axis.home.method);

            axis.homeRuntime.stateElapsedSec =
                0.0;


            if (axis.state ==
                MotionState::MotionState_VELOCITY)
            {
                QueueHomeStop(axisIndex, axis.home.switchStopDecTime);
            }


            axis.homeRuntime.state =
                HomeState::SWITCH_DECEL_STOP;


            return;
        }


        // ----------------------------------------------------
        // E. Search Distance Protection
        //
        // currentActPos / searchStartPulse：
        //     Pulse
        //
        // HomeSearchMaxDistance：
        //     Machine Unit
        //
        // Linear = mm
        // Rotary = degree
        // ----------------------------------------------------

        // ====================================================
        // SEARCH Distance 必須使用同一個座標 Domain
        //
        // searchStartPulse：
        //     Raw Logical Pulse
        //
        // axis.currentActPos：
        //     Machine Pulse
        //     已扣除 machineCoordinateOffsetPulse
        //
        // HOME 成功後 machineCoordinateOffsetPulse 通常很大。
        // 若用 currentActPos - searchStartPulse，
        // 下一次 G81 會把整個 Machine Offset 誤判成已移動距離，
        // 幾乎立刻觸發 3009 HOME_SEARCH_NOT_FOUND。
        //
        // 因此目前位置也必須轉回 Raw Logical Pulse，
        // 再計算本次真正的搜尋距離。
        // ====================================================

        const double searchDistanceUnit =
            GetTravelDistanceUnit(
                axis,
                axis.homeRuntime.searchStartPulse);


        if (searchDistanceUnit >=
            axis.home.searchMaxDistance_unit)
        {
            SetAxisError(
                axisIndex,
                axis,
                HomeErrorReason::SEARCH_MAX_DISTANCE);

            return;
        }


        // ----------------------------------------------------
        // F. Search Timeout Protection
        //
        // 0 = Disable
        // ----------------------------------------------------

        if (axis.home.searchTimeoutSec > 0.0 &&
            axis.homeRuntime.stateElapsedSec >=
            axis.home.searchTimeoutSec)
        {
            SetAxisError(
                axisIndex,
                axis,
                HomeErrorReason::SEARCH_TIMEOUT);

            return;
        }


        // ----------------------------------------------------
        // G. Start / Continue Search Motion
        //
        // HOME Search 使用 Machine Logical Direction。
        //
        // isReverse / Axis_Reverse 等硬體方向反轉，
        // 仍由 MotionCore / Servo Output Layer 負責。
        // ----------------------------------------------------

        const double searchVelocity =
            axis.home.searchSpeed_PPS *
            static_cast<double>(
                axis.home.direction);


        // 只有本 State 第一次啟動時才能建立 Search 起點。
        // Hold -> Resume 後 axis.state 也會回到 IDLE，
        // 但不能重設 SearchMaxDistance / Timeout 起點。
        if (!axis.homeRuntime.motionCommandIssued)
        {
            axis.pid.prevError =
                0.0;

            axis.pid.integralAcc =
                0.0;


            axis.homeRuntime.searchStartPulse =
                m_motion.GetRawLogicalPositionPulse(axis);

            axis.homeRuntime.stateStartPulse =
                axis.homeRuntime.searchStartPulse;

            axis.homeRuntime.stateElapsedSec =
                0.0;

            axis.homeRuntime.motionCommandIssued =
                true;
        }


        QueueHomeVelocity(
            axisIndex, searchVelocity, axis.home.searchAccTime);


        return;
    }


    // ========================================================
    // SWITCH_DECEL_STOP
    // ========================================================
    if (axis.homeRuntime.state == HomeState::SWITCH_DECEL_STOP)
    {
        if (axis.state == MotionState::MotionState_VELOCITY)
        {
            QueueHomeStop(axisIndex, axis.home.switchStopDecTime);
            return;
        }

        if (axis.home.switchStopMaxDistance_unit > 0.0 &&
            GetTravelDistanceUnit(axis, axis.homeRuntime.switchDetectedPulse) > axis.home.switchStopMaxDistance_unit)
        {
            SetAxisError(axisIndex, axis, HomeErrorReason::SWITCH_STOP_MAX_DISTANCE);
            return;
        }

        if (axis.state == MotionState::MotionState_STOPPING) return;
        if (axis.state != MotionState::MotionState_IDLE)
        {
            SetAxisError(axisIndex, axis, HomeErrorReason::MOTION_FAULT);
            return;
        }

        axis.homeRuntime.backoffStartPulse = m_motion.GetRawLogicalPositionPulse(axis);
        axis.homeRuntime.dogReleased = false;
        EnterState(axis, HomeState::BACK_OFF);
        return;
    }

    // ========================================================
    // BACK_OFF
    // ========================================================
    if (axis.homeRuntime.state == HomeState::BACK_OFF)
    {
        if (axis.hardLimitPositive && axis.hardLimitNegative)
        {
            SetAxisError(axisIndex, axis, HomeErrorReason::BOTH_HARD_LIMITS);
            return;
        }

        const bool oppositeLimit =
            (axis.home.direction > 0 && axis.hardLimitNegative) ||
            (axis.home.direction < 0 && axis.hardLimitPositive);
        if (oppositeLimit)
        {
            SetAxisError(axisIndex, axis, HomeErrorReason::OPPOSITE_HARD_LIMIT);
            return;
        }

        if (GetTravelDistanceUnit(axis, axis.homeRuntime.backoffStartPulse) >= axis.home.backoffMaxDistance_unit)
        {
            SetAxisError(axisIndex, axis, HomeErrorReason::BACKOFF_MAX_DISTANCE);
            return;
        }
        if (axis.home.backoffTimeoutSec > 0.0 && axis.homeRuntime.stateElapsedSec >= axis.home.backoffTimeoutSec)
        {
            SetAxisError(axisIndex, axis, HomeErrorReason::BACKOFF_TIMEOUT);
            return;
        }

        const double dir = -static_cast<double>(axis.home.direction);
        const double ppu = GetPulsePerUnit(axis);

        if (axis.home.backoffMode == HomeBackoffMode::FIXED_DISTANCE)
        {
            if (!axis.homeRuntime.stateTargetValid)
            {
                axis.homeRuntime.stateTargetPulse =
                    axis.currentActPos +
                    dir *
                    axis.home.backoffDistance_unit *
                    ppu;

                axis.homeRuntime.stateTargetValid =
                    true;
            }


            if (!axis.homeRuntime.motionCommandIssued)
            {
                if (axis.state !=
                    MotionState::MotionState_IDLE)
                {
                    return;
                }

                if (!QueueHomeMove(
                    axisIndex, axis.homeRuntime.stateTargetPulse,
                    axis.home.backoffSpeed_PPS, axis.home.backoffAccTime,
                    axis.home.backoffDecTime, axis.useShortestPath))
                {
                    return;
                }

                axis.homeRuntime.motionCommandIssued =
                    true;

                return;
            }


            if (axis.state ==
                MotionState::MotionState_IDLE)
            {
                EnterState(
                    axis,
                    HomeState::VALIDATE_RELEASE);

                return;
            }


            if (axis.state !=
                MotionState::MotionState_MOVING &&
                axis.state !=
                MotionState::MotionState_STOPPING)
            {
                SetAxisError(
                    axisIndex,
                    axis,
                    HomeErrorReason::MOTION_FAULT);
            }

            return;
        }

        bool switchActive = false;
        if (!ReadHomeSwitch(axisIndex, axis, switchActive))
        {
            SetAxisError(axisIndex, axis, HomeErrorReason::INVALID_CONFIG);
            return;
        }

        if (!axis.homeRuntime.dogReleased)
        {
            if (switchActive)
            {
                if (axis.state != MotionState::MotionState_IDLE && axis.state != MotionState::MotionState_VELOCITY)
                {
                    SetAxisError(axisIndex, axis, HomeErrorReason::MOTION_FAULT);
                    return;
                }
                QueueHomeVelocity(axisIndex, dir * axis.home.backoffSpeed_PPS, axis.home.backoffAccTime);
                return;
            }

            axis.homeRuntime.dogReleased = true;
            axis.homeRuntime.hardLimitReleased = !axis.hardLimitPositive && !axis.hardLimitNegative;
            if (axis.state == MotionState::MotionState_VELOCITY) QueueHomeStop(axisIndex, axis.home.backoffDecTime);
            return;
        }

        if (axis.state != MotionState::MotionState_IDLE)
        {
            if (axis.state != MotionState::MotionState_STOPPING) QueueHomeStop(axisIndex, axis.home.backoffDecTime);
            return;
        }

        if (axis.home.backoffExtraDistance_unit <= 0.0)
        {
            EnterState(axis, HomeState::VALIDATE_RELEASE);
            return;
        }

        if (!axis.homeRuntime.stateTargetValid)
        {
            axis.homeRuntime.stateTargetPulse =
                axis.currentActPos +
                dir *
                axis.home.backoffExtraDistance_unit *
                ppu;

            axis.homeRuntime.stateTargetValid =
                true;
        }


        if (!axis.homeRuntime.motionCommandIssued)
        {
            if (!QueueHomeMove(
                axisIndex, axis.homeRuntime.stateTargetPulse,
                axis.home.backoffSpeed_PPS, axis.home.backoffAccTime,
                axis.home.backoffDecTime, axis.useShortestPath))
            {
                return;
            }

            axis.homeRuntime.motionCommandIssued =
                true;

            return;
        }

        if (axis.state == MotionState::MotionState_IDLE)
        {
            EnterState(axis, HomeState::VALIDATE_RELEASE);
            return;
        }
        return;
    }

    // ========================================================
    // VALIDATE_RELEASE
    // ========================================================
    if (axis.homeRuntime.state == HomeState::VALIDATE_RELEASE)
    {
        if (axis.state != MotionState::MotionState_IDLE) return;
        bool switchActive = false;
        if (!ReadHomeSwitch(axisIndex, axis, switchActive))
        {
            SetAxisError(axisIndex, axis, HomeErrorReason::INVALID_CONFIG);
            return;
        }
        if (UsesDogSwitch(axis.home.method) && switchActive && axis.home.alarmIfDogNotReleasedBeforeIndex)
        {
            SetAxisError(axisIndex, axis, HomeErrorReason::DOG_NOT_RELEASED);
            return;
        }
        const bool anyLimit = axis.hardLimitPositive || axis.hardLimitNegative;
        if (anyLimit && axis.home.alarmIfHardLimitNotReleasedBeforeIndex)
        {
            SetAxisError(axisIndex, axis, HomeErrorReason::HARD_LIMIT_NOT_RELEASED);
            return;
        }
        axis.homeRuntime.dogReleased = !switchActive;
        axis.homeRuntime.hardLimitReleased = !anyLimit;
        axis.homeRuntime.waitingGroupBarrier = true;
        EnterState(axis, HomeState::WAIT_GROUP_BACKOFF);
        return;
    }

    if (axis.homeRuntime.state == HomeState::WAIT_GROUP_BACKOFF ||
        axis.homeRuntime.state == HomeState::WAIT_GROUP_INDEX_STOP)
        return;

    // ========================================================
    // ARM_REFERENCE
    // ========================================================
    if (axis.homeRuntime.state == HomeState::ARM_REFERENCE)
    {
        if (axis.state != MotionState::MotionState_IDLE) return;

        if (axis.home.method == HomeMethod::ABSOLUTE_REFERENCE)
        {
            SetAxisError(
                axisIndex,
                axis,
                HomeErrorReason::INVALID_CONFIG);

            return;
        }

        if (axis.home.referenceSource == HomeReferenceSource::EXTERNAL_IO_INDEX)
        {
            if (axis.home.captureMode != HomeReferenceCaptureMode::SOFTWARE_SAMPLE)
            {
                SetAxisError(axisIndex, axis, HomeErrorReason::INVALID_CONFIG);
                return;
            }
            bool active = false;
            if (!ReadExternalReferenceInput(axisIndex, axis, active))
            {
                SetAxisError(axisIndex, axis, HomeErrorReason::REFERENCE_INVALID);
                return;
            }
            if (active)
            {
                axis.homeRuntime.previousReferenceInput = true;
                if (axis.home.indexTimeoutSec > 0.0 && axis.homeRuntime.stateElapsedSec >= axis.home.indexTimeoutSec)
                    SetAxisError(axisIndex, axis, HomeErrorReason::REFERENCE_NOT_ARMED);
                return;
            }
            axis.homeRuntime.previousReferenceInput = false;
            axis.homeRuntime.referenceArmed = true;
            axis.homeRuntime.indexSearchStartPulse = m_motion.GetRawLogicalPositionPulse(axis);
            EnterState(axis, HomeState::SEARCH_INDEX);
            return;
        }

        if (!UsesDriveTouchProbe(axis))
        {
            SetAxisError(
                axisIndex,
                axis,
                HomeErrorReason::INVALID_CONFIG);

            return;
        }

        if (ProcessDriveProbeArm(
            axisIndex,
            axis,
            cycleTimeSec))
        {
            EnterState(
                axis,
                HomeState::SEARCH_INDEX);
        }

        return;
    }

    // ========================================================
    // SEARCH_INDEX
    // ========================================================
    if (axis.homeRuntime.state == HomeState::SEARCH_INDEX)
    {
        if (!axis.homeRuntime.referenceArmed)
        {
            SetAxisError(axisIndex, axis, HomeErrorReason::REFERENCE_NOT_ARMED);
            return;
        }
        if (axis.hardLimitPositive && axis.hardLimitNegative)
        {
            SetAxisError(axisIndex, axis, HomeErrorReason::BOTH_HARD_LIMITS);
            return;
        }
        if (axis.hardLimitPositive || axis.hardLimitNegative)
        {
            SetAxisError(axisIndex, axis, HomeErrorReason::OPPOSITE_HARD_LIMIT);
            return;
        }

        bool detected = false;
        double captured = 0.0;
        if (axis.home.referenceSource == HomeReferenceSource::EXTERNAL_IO_INDEX)
        {
            bool active = false;
            if (!ReadExternalReferenceInput(axisIndex, axis, active))
            {
                SetAxisError(axisIndex, axis, HomeErrorReason::REFERENCE_INVALID);
                return;
            }
            detected = active && !axis.homeRuntime.previousReferenceInput;
            axis.homeRuntime.previousReferenceInput = active;
            if (detected) captured = m_motion.GetRawLogicalPositionPulse(axis);
        }
        else
        {
            if (!TryCaptureDriveProbe(
                axisIndex,
                axis,
                detected,
                captured))
            {
                SetAxisError(
                    axisIndex,
                    axis,
                    HomeErrorReason::REFERENCE_INVALID);

                return;
            }
        }

        if (detected)
        {
            axis.homeRuntime.capturedReferencePulse = captured;
            axis.homeRuntime.referenceDetected = true;
            if (axis.state == MotionState::MotionState_VELOCITY) QueueHomeStop(axisIndex, axis.home.indexStopDecTime);
            EnterState(axis, HomeState::INDEX_DECEL_STOP);
            return;
        }

        if (GetTravelDistanceUnit(axis, axis.homeRuntime.indexSearchStartPulse) >= axis.home.indexMaxDistance_unit)
        {
            SetAxisError(axisIndex, axis, HomeErrorReason::INDEX_MAX_DISTANCE);
            return;
        }
        if (axis.home.indexTimeoutSec > 0.0 && axis.homeRuntime.stateElapsedSec >= axis.home.indexTimeoutSec)
        {
            SetAxisError(axisIndex, axis, HomeErrorReason::INDEX_TIMEOUT);
            return;
        }
        if (axis.state != MotionState::MotionState_IDLE && axis.state != MotionState::MotionState_VELOCITY)
        {
            SetAxisError(axisIndex, axis, HomeErrorReason::MOTION_FAULT);
            return;
        }
        const int indexDirection =
            (UsesDogSwitch(axis.home.method) ||
                UsesHardLimitSwitch(axis.home.method))
            ? -axis.home.direction
            : axis.home.direction;


        // Hold -> Resume 不可重新建立 INDEX Search 起點或 Timeout。
        if (!axis.homeRuntime.motionCommandIssued)
        {
            axis.pid.prevError =
                0.0;

            axis.pid.integralAcc =
                0.0;

            axis.homeRuntime.motionCommandIssued =
                true;
        }


        QueueHomeVelocity(
            axisIndex,
            axis.home.indexSearchSpeed_PPS *
            static_cast<double>(indexDirection),
            axis.home.indexSearchAccTime);
        return;
    }

    // ========================================================
    // INDEX_DECEL_STOP
    // ========================================================
    if (axis.homeRuntime.state == HomeState::INDEX_CAPTURED)
    {
        EnterState(axis, HomeState::INDEX_DECEL_STOP);
        return;
    }
    if (axis.homeRuntime.state == HomeState::INDEX_DECEL_STOP)
    {
        if (axis.state == MotionState::MotionState_VELOCITY)
        {
            QueueHomeStop(axisIndex, axis.home.indexStopDecTime);
            return;
        }
        if (axis.state == MotionState::MotionState_STOPPING) return;
        if (axis.state != MotionState::MotionState_IDLE)
        {
            SetAxisError(axisIndex, axis, HomeErrorReason::MOTION_FAULT);
            return;
        }
        EnterState(axis, HomeState::WAIT_GROUP_INDEX_STOP);
        return;
    }

    // ========================================================
    // APPLY_HOME
    // ========================================================
    if (axis.homeRuntime.state == HomeState::APPLY_HOME)
    {
        if (axis.state != MotionState::MotionState_IDLE || !axis.homeRuntime.referenceDetected || !std::isfinite(axis.homeRuntime.capturedReferencePulse))
        {
            SetAxisError(axisIndex, axis, HomeErrorReason::REFERENCE_INVALID);
            return;
        }
        MotionAxisCommandSequence& pendingSequence =
            m_pendingApplyHomeSequence[axisIndex];

        if (pendingSequence == MOTION_AXIS_COMMAND_SEQUENCE_INVALID)
        {
            if (!m_motion.SubmitApplyMachineHome(
                axisIndex, axis.homeRuntime.capturedReferencePulse,
                axis.home.homeOffset_unit, m_homeMotionLease,
                pendingSequence))
            {
                SetAxisError(axisIndex, axis, HomeErrorReason::MOTION_FAULT);
            }
            return;
        }

        MotionAxisCommandResult applyResult{};
        if (!m_motion.TryGetAxisCommandResult(pendingSequence, applyResult))
        {
            return;
        }

        pendingSequence = MOTION_AXIS_COMMAND_SEQUENCE_INVALID;
        if (applyResult.resultType != MotionAxisCommandResultType::APPLIED)
        {
            SetAxisError(axisIndex, axis, HomeErrorReason::REFERENCE_INVALID);
            return;
        }

        axis.isHomed = true;
        if (m_nc != nullptr)
            m_nc->CoordSys.commandedMCS[axisIndex] = axis.currentActPos * (axis.finalLead / axis.resolution_PPR);

        if (axis.home.moveToZero)
        {
            if (axis.home.moveToZeroSpeed_PPS <= 0.0)
            {
                SetAxisError(axisIndex, axis, HomeErrorReason::INVALID_CONFIG);
                return;
            }
            EnterState(axis, HomeState::MOVE_TO_ZERO);
        }
        else CompleteAxis(axisIndex, axis);
        return;
    }

    // ========================================================
    // MOVE_TO_ZERO
    // ========================================================
    if (axis.homeRuntime.state == HomeState::MOVE_TO_ZERO)
    {
        if (axis.hardLimitPositive || axis.hardLimitNegative)
        {
            SetAxisError(axisIndex, axis, HomeErrorReason::OPPOSITE_HARD_LIMIT);
            return;
        }
        if (!axis.homeRuntime.stateTargetValid)
        {
            // ApplyMachineHome 後 Machine Zero 對應 MotionCore Pulse = 0。
            axis.homeRuntime.stateTargetPulse =
                0.0;

            axis.homeRuntime.stateTargetValid =
                true;
        }


        if (!axis.homeRuntime.motionCommandIssued)
        {
            if (axis.state !=
                MotionState::MotionState_IDLE)
            {
                return;
            }

            if (m_nc != nullptr &&
                !m_nc->CoordSys.IsTargetWithinSoftwareTravelLimit(
                    axis,
                    0.0))
            {
                SetAxisError(
                    axisIndex,
                    axis,
                    HomeErrorReason::INVALID_CONFIG);

                return;
            }

            if (!QueueHomeMove(
                axisIndex, axis.homeRuntime.stateTargetPulse,
                axis.home.moveToZeroSpeed_PPS, axis.home.moveToZeroAccTime,
                axis.home.moveToZeroDecTime, axis.useShortestPath))
            {
                return;
            }

            axis.homeRuntime.motionCommandIssued =
                true;

            return;
        }
        if (axis.state == MotionState::MotionState_IDLE)
        {
            CompleteAxis(axisIndex, axis);
            return;
        }
        if (axis.state != MotionState::MotionState_MOVING && axis.state != MotionState::MotionState_STOPPING)
            SetAxisError(axisIndex, axis, HomeErrorReason::MOTION_FAULT);
        return;
    }
}


// ============================================================
// Completion / Error
// ============================================================

void HomingManager::CompleteAxis(
    int axisIndex,
    AxisContext& axis)
{
    if (!DisarmDriveProbe(
        axisIndex,
        axis,
        true))
    {
        // Mailbox full: keep this axis at the completion point and retry.
        return;
    }


    axis.homeRuntime.active =
        false;

    axis.homeRuntime.completed =
        true;

    axis.homeRuntime.error =
        HomeErrorReason::NONE;

    axis.homeRuntime.state =
        HomeState::DONE;

    axis.homeRuntime.stateElapsedSec =
        0.0;


    // HOME 真正成功完成後，
    // 才重新開啟 Machine Homed 狀態。
    axis.isHomed =
        true;


    // ========================================================
    // HOME Persistence
    //
    // 只 Queue 記憶體 Snapshot。
    // 這裡不做任何磁碟 I/O。
    //
    // 真正寫檔由 1000ms supervisory task 執行。
    // ========================================================

    RtPrintf(
        "[HOME-PERSIST] CompleteAxis Axis:%d IsHomed:%d\n",
        axisIndex,
        axis.isHomed
        ? 1
        : 0);


    HomePersistenceManager::GetInstance()
        .QueueSuccessfulHome(
            axisIndex,
            axis);
}


void HomingManager::SetAxisError(
    int axisIndex,
    AxisContext& axis,
    HomeErrorReason error)
{
    // ========================================================
    // 1. 第一錯誤鎖存
    //
    // 多軸 HOME 可能在同一 Scan 發現多個問題。
    //
    // Alarm History 只保留第一個真正失敗原因，
    // 避免同一事件塞入多個衍生 Alarm。
    // ========================================================

    const bool firstHomeError =
        m_lastError ==
        HomeErrorReason::NONE;

    const int immediateAlarmCode = GetHomeAlarmCode(error);
    if (immediateAlarmCode != 0)
    {
        m_motion.RequestEmergencyStopAllAxes();
    }

    DisarmDriveProbe(
        axisIndex,
        axis);


    // ========================================================
    // 2. 標記發生錯誤的軸
    // ========================================================

    axis.homeRuntime.active =
        false;

    axis.homeRuntime.completed =
        false;

    axis.homeRuntime.error =
        error;

    axis.homeRuntime.state =
        HomeState::HOME_ERROR;

    axis.homeRuntime.stateElapsedSec =
        0.0;


    // HOME 失敗後，此軸 Machine Home 無效。
    axis.isHomed =
        false;


    // ========================================================
    // 3. 終止整個 HOME Request
    //
    // 只要其中一軸 HOME 失敗，
    // 本次 G81 / Panel HOME 不可繼續執行其他軸。
    //
    // 已經成功完成 HOME 的前一個 Order 軸，
    // isHomed 狀態保留，不任意清除。
    // ========================================================

    m_hasError =
        true;

    m_active =
        false;

    m_completed =
        false;

    m_cancelRequested =
        false;

    ClearResumeRequest();

    m_runControlState =
        HomeRunControlState::IDLE;

    m_currentOrder =
        -1;


    for (int i = 0;
        i < HOME_AXIS_COUNT;
        ++i)
    {
        if (!IsAxisRequested(
            m_selectedAxisMask,
            i))
        {
            continue;
        }


        AxisContext& selectedAxis =
            m_motion.GetAxisContext(i);


        // 已經成功 DONE 的軸保留完成狀態。
        if (selectedAxis.homeRuntime.completed)
        {
            continue;
        }


        // 真正發生錯誤的軸已在上方寫入 Error。
        if (i == axisIndex)
        {
            continue;
        }


        // 其他正在參與同一 HOME Request 的軸只停止 Ownership。
        //
        // 不替它們建立第二個 Alarm，
        // 也不假裝它們各自發生相同故障。
        DisarmDriveProbe(
            i,
            selectedAxis);

        selectedAxis.homeRuntime.active =
            false;

        selectedAxis.homeRuntime.waitingGroupBarrier =
            false;
    }


    m_activeAxisMask =
        0;


    // ========================================================
    // 4. 保存第一錯誤現場
    // ========================================================

    if (firstHomeError)
    {
        m_lastError =
            error;

        m_lastErrorAxis =
            axisIndex;
    }


    // ========================================================
    // 5. Error -> Alarm
    //
    // 使用者定義：
    //
    //     只要進 AlarmManager
    //     就必須 Emergency Stop。
    //
    // CANCELLED 是正常取消，不在這裡產生 Alarm。
    // ========================================================

    const int alarmCode =
        GetHomeAlarmCode(
            error);


    if (firstHomeError &&
        alarmCode != 0 &&
        !AlarmManager::GetInstance().HasAlarm())
    {
        const int alarmAxisIndex =
            axis.isExist
            ? axis.axisIndex
            : axisIndex;


        AlarmManager::GetInstance().Trigger(
            alarmCode,
            0,
            alarmAxisIndex);
    }

    // 只要是 Alarm 類型錯誤，一律全軸急停。
    if (alarmCode != 0)
    {
        m_motion.RequestEmergencyStopAllAxes();
        if (m_nc != nullptr) m_nc->ChangeState(NCState::ALARM);
    }

    RestoreOrReleaseHomeMotionOwner();
}


void HomingManager::CompleteRequest()
{
    if (m_nc != nullptr)
    {
        double actualMCS[HOME_AXIS_COUNT] = { 0.0 };
        for (int i = 0; i < HOME_AXIS_COUNT; ++i)
        {
            const AxisContext& axis = m_motion.GetAxisContext(i);
            if (axis.isExist && axis.resolution_PPR > 0.0)
                actualMCS[i] = axis.currentActPos * (axis.finalLead / axis.resolution_PPR);
        }
        m_nc->CoordSys.UpdateActualMCS(actualMCS);
        m_nc->CoordSys.SyncMachinePosition(actualMCS);
        m_motion.SyncVirtualEndPosition();
        m_nc->UpdateSystemVariables();
    }

    m_activeAxisMask =
        0;

    m_active =
        false;

    m_completed =
        true;

    m_cancelRequested =
        false;

    ClearResumeRequest();

    m_runControlState =
        HomeRunControlState::IDLE;

    m_currentOrder =
        -1;

    RestoreOrReleaseHomeMotionOwner();
}


// ============================================================
// HOME Method Helper
// ============================================================

bool HomingManager::UsesDogSwitch(
    HomeMethod method) const
{
    return
        method ==
        HomeMethod::DOG_INDEX ||
        method ==
        HomeMethod::DOG_ONLY;
}


bool HomingManager::UsesHardLimitSwitch(
    HomeMethod method) const
{
    return
        method ==
        HomeMethod::LIMIT_INDEX ||
        method ==
        HomeMethod::LIMIT_ONLY;
}


bool HomingManager::UsesIndexReference(
    HomeMethod method) const
{
    return
        method ==
        HomeMethod::DOG_INDEX ||
        method ==
        HomeMethod::LIMIT_INDEX ||
        method ==
        HomeMethod::INDEX_ONLY;
}


bool HomingManager::IsHardLimitAllowedHomeState(
    HomeState state) const
{
    switch (state)
    {
    case HomeState::SEARCH_SWITCH:

        // 正在主動尋找預期方向 Limit。
        return true;


    case HomeState::SWITCH_DECEL_STOP:

        // Limit 已經觸發，
        // 正在 Controlled Deceleration。
        return true;


    case HomeState::BACK_OFF:

        // 正在反方向退出 Limit。
        return true;


    case HomeState::VALIDATE_RELEASE:

        // Backoff 完成後正在確認 Limit 是否解除。
        //
        // 若仍未解除且參數要求 Alarm，
        // 後續由 HomingManager 產生更明確的
        // HOME_LIMIT_NOT_RELEASED 類型 Alarm。
        return true;


    default:
        return false;
    }
}
