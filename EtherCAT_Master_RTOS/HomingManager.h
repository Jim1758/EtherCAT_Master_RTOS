#pragma once

#include "HomeTypes.h"
#include "MotionExecutionContract.h"
#include "MotionAxisCommandMailbox.h"
#include <cstdint>

class MotionCore;
class PLCManager;
class NCManager;
struct AxisContext;


// ============================================================
// HomingManager
//
// G81 尋原點流程管理器。
//
// 職責：
//
// 1. 接收 G81 HomeRequest
// 2. 選出參與 HOME 的軸
// 3. 管理 P0 同時尋原點
// 4. 管理 P1 依 HomeOrder 分組尋原點
// 5. 管理每軸 HomeRuntime / HomeState
// 6. 提供 Hard Limit HOME 例外判斷
// 7. 後續統一管理 DOG / Backoff / INDEX / Offset
//
// 不負責：
//
// - Servo PID 底層計算
// - EtherCAT PDO 收發
// - PLC Ladder 執行
// - 一般 G00 / JOG / MPG / INCH
//
// 真正運動仍交給 MotionCore：
//
// SEARCH DOG / LIMIT
//     -> VelocityMove()
//
// DOG / INDEX 找到
//     -> StopMove()
//
// Backoff / Move To Zero
//     -> MoveToPosition()
//
// Alarm 異常
//     -> Emergency Stop
// ============================================================

class HomingManager
{
public:
    static constexpr int HOME_AXIS_COUNT = 8;

    enum class ResumeResult : std::uint8_t
    {
        REJECTED = 0,
        DEFERRED,
        APPLIED,
        SUPERSEDED
    };


    // ========================================================
    // Constructor
    //
    // MotionCore 一定存在，所以使用 Reference。
    //
    // NCManager / PLCManager 可能在系統建構後才完成，
    // 因此使用 Link API 後續連結。
    // ========================================================

    explicit HomingManager(
        MotionCore& motion);


    // ========================================================
    // System Link
    // ========================================================

    void LinkNCManager(
        NCManager* nc);

    void LinkPLCManager(
        PLCManager* plc);


    // ========================================================
    // Start HOME
    //
    // request.axisMask：
    //
    // Bit 0 = Axis 0
    // Bit 1 = Axis 1
    // ...
    // Bit 7 = Axis 7
    //
    // axisMask == 0：
    //
    // 自動選取：
    //
    // axis.isExist == true
    // &&
    // axis.home.enabled == true
    //
    // request.sequenceMode：
    //
    // P0 = SIMULTANEOUS
    //      所有指定軸同時開始。
    //
    // P1 = BY_ORDER
    //      依 axis.home.order 分組執行。
    //
    // true：
    //      Request 已接受。
    //
    // false：
    //      Request 不合法或系統目前不能開始 HOME。
    // ========================================================

    bool Start(
        const HomeRequest& request);


    // ========================================================
    // Cyclic Process
    //
    // cycleTimeSec：
    //      呼叫此函式的實際週期，單位秒。
    //
    // 第一版預計由：
    //
    // NCPLCManager::ProcessHomeInputs()
    //
    // 每個 PLC / NC Scan 呼叫。
    //
    // 後續若需要更快的 Drive Capture 判斷，
    // 可將 Reference Capture 移到 PDO 層，
    // HomingManager 仍只處理狀態轉移。
    // ========================================================

    void Process(
        double cycleTimeSec);


    // ========================================================
    // Cancel / Reset
    //
    // Cancel：
    //      正在 HOME 時提出正常取消。
    //      後續實作會對正在移動的軸使用 Controlled Stop。
    //
    // Reset：
    //      將 HomingManager 與所有 HomeRuntime 洗回初始狀態。
    //      不代表軸已完成 HOME。
    // ========================================================

    // Feed Hold：受控減速並保留完整 HOME Request / HomeState。
    bool RequestHold();

    // Cycle Start：從 PAUSED 恢復；若仍在減速，先排入 Resume Request。
    ResumeResult Resume() noexcept;

    // Reset / Abort：受控停止後清除尚未完成的 HOME Runtime，不可恢復。
    void Cancel();

    // Active requests cancel first. Retry after owner retirement to clear.
    void Reset();


    // ========================================================
    // Global State Query
    // ========================================================

    bool IsActive() const;

    bool IsCompleted() const;

    bool HasError() const;

    bool IsCancelRequested() const;

    bool IsHoldDecelerating() const;

    bool IsPaused() const;

    bool IsResumeRequested() const;

    HomeRunControlState GetRunControlState() const;


    // ========================================================
    // Request / Scheduler Query
    // ========================================================

    uint8_t GetSelectedAxisMask() const;

    uint8_t GetActiveAxisMask() const;

    HomeSequenceMode GetSequenceMode() const;

    int GetCurrentOrder() const;


    // ========================================================
    // Last Error Query
    //
    // Request 初始化失敗或 Runtime 發生錯誤時，
    // 記錄第一個失敗原因與軸編號。
    //
    // -1 = 無特定軸。
    // ========================================================

    HomeErrorReason GetLastError() const;

    int GetLastErrorAxis() const;


    // ========================================================
    // Per-Axis Query
    // ========================================================

    bool IsAxisSelected(
        int axisIndex) const;

    bool IsAxisActive(
        int axisIndex) const;

    bool IsAxisCompleted(
        int axisIndex) const;

    bool IsAxisHoming(
        int axisIndex) const;

    HomeState GetAxisState(
        int axisIndex) const;

    HomeErrorReason GetAxisError(
        int axisIndex) const;


    // ========================================================
    // Expected HOME Hard Limit
    //
    // 給 NCPLCManager::ProcessSafetyInputs() 使用。
    //
    // 正常狀態：
    //
    // Physical +OT / -OT
    //     -> HARD_LIMIT Alarm
    //
    // LIMIT_INDEX / LIMIT_ONLY HOME：
    //
    // 只有符合以下全部條件時，
    // 預期方向的 Hard Limit 才是 HOME Event：
    //
    // 1. 該軸正在 HOME
    // 2. HomeMethod 使用 LIMIT
    // 3. HomeDirection 與 Limit 方向相同
    // 4. HomeState 位於允許接觸 / 退出 Limit 的階段
    //
    // 相反方向 Limit：
    //     永遠不是正常 HOME Event。
    //
    // +OT 與 -OT 同時 ON：
    //     永遠視為異常。
    // ========================================================

    bool IsExpectedPositiveHardLimit(
        int axisIndex) const;

    bool IsExpectedNegativeHardLimit(
        int axisIndex) const;

    bool IsExpectedHomeHardLimit(
        int axisIndex,
        bool positiveDirection) const;


private:
    // ========================================================
    // Stage NC-0.1F - HOME ownership and RT command mailbox
    // ========================================================
    bool AcquireHomeMotionOwner() noexcept;
    void RestoreOrReleaseHomeMotionOwner() noexcept;
    MotionOwnerLease GetProbeCommandLease() const noexcept;

    bool QueueHomeStop(int axisIndex, double decelerationTime) noexcept;
    bool QueueHomeControlStop(int axisIndex, double decelerationTime) noexcept;
    bool PollHomeCommandResult(
        int axisIndex, MotionAxisCommandSequence& sequence,
        MotionAxisCommandType commandType) noexcept;
    bool IsHomeAxisControlStopped(int axisIndex, AxisContext& axis) noexcept;
    bool QueueHomeVelocity(
        int axisIndex, double velocity, double accelerationTime) noexcept;
    bool QueueHomeMove(
        int axisIndex, double targetPosition, double targetVelocity,
        double accelerationTime, double decelerationTime,
        bool useShortestPath) noexcept;
    bool QueueHomeProbeFunction(
        int axisIndex, uint16_t value,
        MotionAxisCommandSequence* outSequence = nullptr) noexcept;

    // ========================================================
    // Basic Validation
    // ========================================================

    bool IsValidAxisIndex(
        int axisIndex) const;

    bool IsAxisRequested(
        uint8_t axisMask,
        int axisIndex) const;

    uint8_t BuildEnabledAxisMask() const;


    // ========================================================
    // Request Initialization
    // ========================================================

    bool ValidateRequest(
        const HomeRequest& request,
        uint8_t selectedAxisMask);

    HomeErrorReason ValidateAxisForHome(
        const AxisContext& axis,
        HomeSequenceMode sequenceMode) const;

    void InitializeRequest(uint8_t selectedAxisMask);

    void ResetAxisRuntime(
        AxisContext& axis);

    void PrepareSelectedAxis(
        AxisContext& axis);


    // ========================================================
    // P0 / P1 Scheduler
    // ========================================================

    void ActivateInitialGroup();

    void ActivateAllSelectedAxes();

    void ActivateOrderGroup(
        int order);

    int FindLowestPendingOrder() const;

    bool HasPendingAxis() const;

    bool IsActiveGroupComplete() const;

    void AdvanceScheduler();


    // ========================================================
    // Per-Axis State Machine
    //
    // 這一步只先宣告。
    // 真正 DOG / Backoff / INDEX Motion
    // 會在後續逐 State 實作。
    // ========================================================

    void ProcessAxis(
        int axisIndex,
        AxisContext& axis,
        double cycleTimeSec);

    void ProcessBackoffPointMove(
        int axisIndex, AxisContext& axis, double distanceUnit);

    void ProcessCancel();
    void ProcessHold(double cycleTimeSec);
    ResumeResult TryResumeFromPaused() noexcept;
    void ClearResumeRequest() noexcept;

    bool MonitorSwitchDuringHold(int axisIndex, AxisContext& axis);
    bool MonitorReferenceDuringHold(int axisIndex, AxisContext& axis);
    double GetHoldDecTime(const AxisContext& axis) const;
    bool IsP2PResumeState(HomeState state) const;

    void ProcessBackoffBarrier();
    void ProcessIndexStopBarrier();

    bool ReadHomeSwitch(int axisIndex, AxisContext& axis, bool& active) const;
    bool ReadExternalReferenceInput(int axisIndex, const AxisContext& axis, bool& active) const;

    // Delta / CiA402 Drive Touch Probe Provider
    bool UsesDriveTouchProbe(const AxisContext& axis) const;
    bool ValidateDriveProbeConfig(const AxisContext& axis) const;
    bool ProcessDriveProbeArm(int axisIndex, AxisContext& axis, double cycleTimeSec);
    bool TryCaptureDriveProbe(int axisIndex, AxisContext& axis, bool& detected, double& capturedPulse);
    bool IsDriveProbeSourceStatusValid(const AxisContext& axis, uint16_t status) const;
    bool DisarmDriveProbe(
        int axisIndex, AxisContext& axis,
        bool trackForOwnerRelease = false);
    void ResetDriveProbeRuntime(AxisContext& axis);

    double GetPulsePerUnit(const AxisContext& axis) const;
    double GetTravelDistanceUnit(const AxisContext& axis, double startRawPulse) const;
    void EnterState(AxisContext& axis, HomeState state);


    // ========================================================
    // Completion / Error
    // ========================================================

    void CompleteAxis(
        int axisIndex,
        AxisContext& axis);

    void SetAxisError(
        int axisIndex,
        AxisContext& axis,
        HomeErrorReason error);

    void CompleteRequest();


    // ========================================================
    // HOME Method Helper
    // ========================================================

    bool UsesDogSwitch(
        HomeMethod method) const;

    bool UsesHardLimitSwitch(
        HomeMethod method) const;

    bool UsesIndexReference(
        HomeMethod method) const;

    bool IsHardLimitAllowedHomeState(
        HomeState state) const;


private:
    // ========================================================
    // Linked Systems
    // ========================================================

    MotionCore& m_motion;

    NCManager* m_nc =
        nullptr;

    PLCManager* m_plc =
        nullptr;


    // Stage NC-0.1F ownership snapshot.  G81 may transfer AUTO / MDI /
    // MANUAL_AUTO to HOME and restore a new generation when HOME finishes.
    MotionOwnerLease m_homeMotionLease{};
    MotionOwner m_returnMotionOwner = MotionOwner::NONE;
    MotionAxisCommandSequence
        m_pendingApplyHomeSequence[HOME_AXIS_COUNT]{};
    MotionAxisCommandSequence
        m_pendingProbeDisarmSequence[HOME_AXIS_COUNT]{};
    // Control-thread receipts only; no HomeRuntime / RT_SHARED ABI changes.
    MotionAxisCommandSequence m_pendingMoveSequence[HOME_AXIS_COUNT]{};
    MotionAxisCommandSequence m_pendingControlStopSequence[HOME_AXIS_COUNT]{};
    bool m_controlStopIssued[HOME_AXIS_COUNT]{};

    // ========================================================
    // Request State
    // ========================================================

    HomeSequenceMode m_sequenceMode =
        HomeSequenceMode::SIMULTANEOUS;

    uint8_t m_selectedAxisMask =
        0;

    uint8_t m_activeAxisMask =
        0;

    int m_currentOrder =
        -1;


    // ========================================================
    // Global Runtime State
    // ========================================================

    bool m_active =
        false;

    bool m_completed =
        false;

    bool m_hasError =
        false;

    bool m_cancelRequested =
        false;

    HomeRunControlState m_runControlState =
        HomeRunControlState::IDLE;

    // Cycle Start 可能在 Hold 減速完成前先按下。
    // 此旗標讓 HomingManager 在所有軸真正停止後自動恢復。
    bool m_resumeRequested =
        false;

    // Immutable Alarm/Motion identity captured by the operator Cycle Start.
    // A queued HOME resume may wait through controlled deceleration, but it
    // must never recapture a newer post-Alarm baseline and resume implicitly.
    std::uint32_t m_resumeAlarmUpdateCount =
        0U;

    std::uint64_t m_resumeAlarmIntentBaseState =
        0ULL;

    MotionOwnerLease m_resumeHomeMotionLease{};

    bool m_resumeAdmissionTicketValid =
        false;


    // ========================================================
    // First Error Snapshot
    // ========================================================

    HomeErrorReason m_lastError =
        HomeErrorReason::NONE;

    int m_lastErrorAxis =
        -1;
};
