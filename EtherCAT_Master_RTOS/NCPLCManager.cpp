#include "NCPLCManager.h"
#include "NCManager.h"
#include "MotionCore.h"
#include "PLCManager.h"
#include "NCPLCMap.h"
#include "AlarmManager.h"
#include <cstdint>
#include <cmath>

// =========================================================
// Constructor
// =========================================================
NCPLCManager::NCPLCManager(NCManager& nc, MotionCore& motion, PLCManager& plc)
    : m_nc(nc), m_motion(motion), m_plc(plc)
{
    m_lastMPGCount = static_cast<int32_t>(m_plc.GetMemory("DR", NCPLC::DR::MPG_ENCODER_COUNT)); // MPG DR200 baseline. Avoid a startup jump if DR200 is already non-zero.
}

// =========================================================
// Main Cyclic Process
// =========================================================
void NCPLCManager::Process()
{
    ProcessSafetyInputs(); // 1. Safety First

    // =====================================================
    // 2. Active Safety Input Gate
    // C5 Emergency, C140~147 Axis Protection
    // 只要這些 Level Safety Input 還存在，後面的 Start / Reset / JOG / HOME / AUX 全部禁止。
    // Hard Limit C180/C190 不放在這裡，因為之後需要允許反方向 Recovery。
    // =====================================================
    bool safetyInputActive = m_plc.Get_C(NCPLC::C::EMERGENCY_STOP);

    for (int i = 0; i < NCPLC::AXIS_COUNT; ++i)
    {
        const bool axisProtect = m_plc.Get_C(NCPLC::C::AxisPoint(NCPLC::C::AXIS_PROTECT_STOP_BASE, i));
        if (axisProtect)
        {
            safetyInputActive = true;
            break;
        }
    }

    if (safetyInputActive)
    {
        m_lastMPGCount = static_cast<int32_t>(m_plc.GetMemory("DR", NCPLC::DR::MPG_ENCODER_COUNT)); // Safety gate skips ProcessManualInputs(). Keep DR200 baseline synchronized so clearing safety cannot replay handwheel counts accumulated during the stop.
        SyncNCStateToPLC();
        return;
    }

    // 3. PLC -> NC
    ProcessGlobalInputs();
    ProcessHomeInputs();
    ProcessManualInputs();
    ProcessAuxiliaryHandshake();

    // 4. NC -> PLC
    SyncNCStateToPLC();
}

// =========================================================
// Global PLC Inputs
// =========================================================
void NCPLCManager::ProcessGlobalInputs()
{
    m_servoReady = m_plc.Get_C(NCPLC::C::SERVO_READY); // C11 - Servo / Machine Ready (Level Signal)
    m_nc.SetExternalReadyInterlock(m_servoReady);

    // =====================================================
    // C12 - Cycle Start (Rising Edge)
    // Cycle Start 必須確認：1. C11 Servo Ready 2. 沒有任何 Hard Limit 3. 不在任何 Manual Move Mode 4. 沒有 JOG Axis 還在動作
    // =====================================================
    const bool cycleStart = m_plc.Get_C(NCPLC::C::CYCLE_START);

    if (cycleStart && !m_prevCycleStart)
    {
        // =================================================
        // HOME Resume Gate
        //
        // 一般 Cycle Start：任何 Hard Limit 都禁止。
        //
        // LIMIT_ONLY / LIMIT_INDEX HOME 暫停時：
        // 預期方向 Limit 可能仍然 ON，必須允許 C12 Resume，
        // 讓軸繼續 Backoff 離開 Limit。
        // =================================================

        const bool homingResume =
            m_nc.Homing.IsActive() &&
            (m_nc.Homing.IsPaused() ||
                m_nc.Homing.IsHoldDecelerating());


        bool hardLimitActive =
            false;

        for (int i = 0;
            i < NCPLC::AXIS_COUNT;
            ++i)
        {
            const bool positiveLimit =
                m_plc.Get_C(
                    NCPLC::C::AxisPoint(
                        NCPLC::C::POSITIVE_LIMIT_BASE,
                        i));

            const bool negativeLimit =
                m_plc.Get_C(
                    NCPLC::C::AxisPoint(
                        NCPLC::C::NEGATIVE_LIMIT_BASE,
                        i));


            const bool expectedPositive =
                homingResume &&
                m_nc.Homing.IsExpectedPositiveHardLimit(i);

            const bool expectedNegative =
                homingResume &&
                m_nc.Homing.IsExpectedNegativeHardLimit(i);


            if ((positiveLimit &&
                !expectedPositive) ||
                (negativeLimit &&
                    !expectedNegative))
            {
                hardLimitActive =
                    true;

                break;
            }
        }

        // Manual Move Mode Check: 直接讀 PLC Raw Mode，不依賴上一 Scan 的 m_manualMoveMode，因為 ProcessGlobalInputs() 比 ProcessManualInputs() 先執行。
        const bool continuousJogMode = m_plc.Get_C(NCPLC::C::JOG_MODE);
        const bool fineJogMode = m_plc.Get_C(NCPLC::C::FINE_JOG_MODE);
        const bool inchJogMode = m_plc.Get_C(NCPLC::C::INCH_JOG_MODE);
        const bool mpgMode = m_plc.Get_C(NCPLC::C::MPG_MODE);
        const bool manualMoveModeActive = continuousJogMode || fineJogMode || inchJogMode || mpgMode;

        // JOG Motion Ownership Check: 即使 C19 已經 OFF，軸可能還在 JOG_dec_time 減速。此時不能 Start 自動程式。
        bool jogAxisActive = false;
        for (int i = 0; i < NCPLC::AXIS_COUNT; ++i)
        {
            if (m_jogActive[i])
            {
                jogAxisActive = true;
                break;
            }
        }

        const bool noHomeOwnershipConflict =
            !m_nc.Homing.IsActive() ||
            homingResume;


        const bool cycleStartAllowed =
            m_servoReady &&
            !hardLimitActive &&
            !manualMoveModeActive &&
            !jogAxisActive &&
            noHomeOwnershipConflict;
        if (cycleStartAllowed)
        {
            m_nc.CycleStart();
        }
    }
    m_prevCycleStart = cycleStart;

    const bool reset = m_plc.Get_C(NCPLC::C::RESET); // C13 - NC Reset (Rising Edge)
    if (reset && !m_prevReset)
    {
        m_nc.Reset();
    }
    m_prevReset = reset;

    const bool servoFaultReset = m_plc.Get_C(NCPLC::C::SERVO_FAULT_RESET); // C14 - Servo Fault Reset (Rising Edge) 不使用 ResetAllFaults() 避免清除 Group Command Queue。
    if (servoFaultReset && !m_prevServoFaultReset)
    {
        if (m_nc.GetState() != NCState::RUN) // RUN 中禁止 Servo Fault Reset
        {
            for (int i = 0; i < NCPLC::AXIS_COUNT; ++i)
            {
                AxisContext& axis = m_motion.GetAxisContext(i);
                if (!axis.isExist) continue;

                const bool needsReset = axis.isFault || axis.isLagAlarm || axis.state == MotionState::MotionState_ERROR || axis.state == MotionState::MotionState_ESTOP;
                if (!needsReset) continue;

                m_motion.ResetFault(axis);
            }
        }
    }
    m_prevServoFaultReset = servoFaultReset;

    const bool singleBlock = m_plc.Get_C(NCPLC::C::SINGLE_BLOCK); // C16 - Single Block (Level Signal)
    m_nc.SetSingleBlockEnabled(singleBlock);

    const bool optionalStop = m_plc.Get_C(NCPLC::C::OPTIONAL_STOP); // C17 - Optional Stop (Level Signal)
    m_nc.SetOptionalStopEnabled(optionalStop);

    const bool blockSkip = m_plc.Get_C(NCPLC::C::BLOCK_SKIP); // C18 - Block Skip (Level Signal)
    m_nc.SetBlockSkipEnabled(blockSkip);

    m_edmProtectionBypass = m_plc.Get_C(NCPLC::C::EDM_PROTECTION_BYPASS); // C20 - EDM Protection Bypass: 只保存狀態。不能 bypass: Emergency, Hard Limit, Axis Protection, Servo Fault, Lag Alarm, Safety Chain

    const bool controlledStop = m_plc.Get_C(NCPLC::C::CONTROLLED_STOP); // C4 - Controlled Stop / Feed Hold (Rising Edge)
    if (controlledStop && !m_prevControlledStop)
    {
        m_nc.FeedHold();
    }
    m_prevControlledStop = controlledStop;
}

// =========================================================
// Safety Inputs
// =========================================================
void NCPLCManager::ProcessSafetyInputs()
{
    const bool emergencyStop = m_plc.Get_C(NCPLC::C::EMERGENCY_STOP); // C5 - Emergency Stop (Level Sensitive Stop, Alarm only Rising Edge)
    if (emergencyStop)
    {
        m_motion.EmergencyStopAllAxes(); // 每 Scan 維持全軸停止
        m_nc.ChangeState(NCState::ALARM);

        if (!m_prevEmergencyStop) // Alarm 只建立一次
        {
            AlarmManager::GetInstance().Trigger(AlarmManager::EMG_STOP);
        }
    }
    m_prevEmergencyStop = emergencyStop;

    bool axisProtectionActive = false; // C140 ~ C147 Axis Protection (Level Sensitive Stop, Rising Edge Alarm)
    for (int i = 0; i < NCPLC::AXIS_COUNT; ++i)
    {
        const bool axisProtect = m_plc.Get_C(NCPLC::C::AxisPoint(NCPLC::C::AXIS_PROTECT_STOP_BASE, i));
        if (axisProtect)
        {
            axisProtectionActive = true;
            if (!m_prevAxisProtect[i])
            {
                AxisContext& axis = m_motion.GetAxisContext(i);
                const int alarmAxisIndex = axis.isExist ? axis.axisIndex : i;
                AlarmManager::GetInstance().Trigger(AlarmManager::MANUAL_AXIS_PROTECT, 0, alarmAxisIndex);
            }
        }
        m_prevAxisProtect[i] = axisProtect;
    }

    if (axisProtectionActive) // C140~147 持續 ON，就持續維持全機 Emergency Stop。
    {
        m_motion.EmergencyStopAllAxes();
        m_nc.ChangeState(NCState::ALARM);
    }

    // =====================================================
    // C180 ~ C187 (+OT), C190 ~ C197 (-OT)
    // Hard Limit 只在 Rising Edge: 1. Trigger Alarm 2. Emergency Stop All Axes 3. NC -> ALARM
    // 持續 ON 不重新 Emergency Stop，因為之後 Manual Recovery 必須允許朝離開 Limit 的方向移動。
    // =====================================================
    bool hardLimitTriggered = false;
    for (int i = 0; i < NCPLC::AXIS_COUNT; ++i)
    {
        AxisContext& axis = m_motion.GetAxisContext(i);
        const int alarmAxisIndex = axis.isExist ? axis.axisIndex : i;

        const bool positiveLimit = m_plc.Get_C(NCPLC::C::AxisPoint(NCPLC::C::POSITIVE_LIMIT_BASE, i)); // +OT
        const bool negativeLimit = m_plc.Get_C(NCPLC::C::AxisPoint(NCPLC::C::NEGATIVE_LIMIT_BASE, i)); // -OT

        axis.hardLimitPositive = positiveLimit;
        axis.hardLimitNegative = negativeLimit;

        // =====================================================
        // G81 HOME Expected Hard Limit
        //
        // 一般情況：
        //
        //     +OT / -OT Rising Edge
        //         -> 3002 HARD_LIMIT
        //         -> Emergency Stop
        //
        // LIMIT_INDEX / LIMIT_ONLY 尋原點時：
        //
        // 只有「正在 HOME 的軸」
        // +
        // 「正確 HomeDirection」
        // +
        // 「允許接觸 Hard Limit 的 HomeState」
        //
        // 才能把該方向 Hard Limit 視為正常 HOME Event。
        //
        // 相反方向 Hard Limit：永遠 Alarm。
        // +OT / -OT 同時 ON：永遠異常。
        // =====================================================

        const bool bothHardLimits =
            positiveLimit &&
            negativeLimit;

        const bool expectedPositiveHomeLimit =
            !bothHardLimits &&
            m_nc.Homing.IsExpectedPositiveHardLimit(i);

        const bool expectedNegativeHomeLimit =
            !bothHardLimits &&
            m_nc.Homing.IsExpectedNegativeHardLimit(i);

        const bool positiveLimitRising =
            positiveLimit &&
            !m_prevPositiveLimit[i];

        const bool negativeLimitRising =
            negativeLimit &&
            !m_prevNegativeLimit[i];

        const bool axisHoming =
            m_nc.Homing.IsAxisHoming(i);

        // HOME 中正負極限同時 ON，使用專用 HOME Alarm，
        // 避免同一 Scan 重複建立兩筆一般 HARD_LIMIT。
        if (bothHardLimits &&
            (positiveLimitRising || negativeLimitRising))
        {
            AlarmManager::GetInstance().Trigger(
                axisHoming
                ? AlarmManager::HOME_BOTH_LIMITS
                : AlarmManager::HARD_LIMIT,
                0,
                alarmAxisIndex);

            hardLimitTriggered = true;
        }
        else
        {
            if (positiveLimitRising &&
                !expectedPositiveHomeLimit)
            {
                AlarmManager::GetInstance().Trigger(
                    axisHoming
                    ? AlarmManager::HOME_OPPOSITE_LIMIT
                    : AlarmManager::HARD_LIMIT,
                    0,
                    alarmAxisIndex);

                hardLimitTriggered = true;
            }

            if (negativeLimitRising &&
                !expectedNegativeHomeLimit)
            {
                AlarmManager::GetInstance().Trigger(
                    axisHoming
                    ? AlarmManager::HOME_OPPOSITE_LIMIT
                    : AlarmManager::HARD_LIMIT,
                    0,
                    alarmAxisIndex);

                hardLimitTriggered = true;
            }
        }
        // =====================================================
        // Final Travel Direction Block State
        //
        // Physical +OT / -OT 永遠有效。
        // Software Limit 只有 CoordinateManager 判定 Active 時生效。
        //
        // 這兩個欄位只代表「該方向目前能不能再繼續走」，
        // 不代表 Alarm。
        // =====================================================

        axis.positiveTravelBlocked =
            axis.hardLimitPositive ||
            !m_nc.GetCoordSys().CanMoveSoftwarePositive(axis);

        axis.negativeTravelBlocked =
            axis.hardLimitNegative ||
            !m_nc.GetCoordSys().CanMoveSoftwareNegative(axis);

        m_prevPositiveLimit[i] = positiveLimit; // Edge Memory
        m_prevNegativeLimit[i] = negativeLimit;
    }

    if (hardLimitTriggered) // 本 Scan 有新的 Hard Limit
    {
        m_motion.EmergencyStopAllAxes();
        m_nc.ChangeState(NCState::ALARM);
    }
}

// =========================================================
// Manual Inputs
// =========================================================
void NCPLCManager::ProcessManualInputs()
{
    // =========================================================
    // Manual Move Mode (四種模式必須 One-Hot)
    // C19=Normal Continuous JOG, C23=Fine Continuous JOG, C24=INCH JOG, C21=MPG
    // =========================================================
    const bool continuousJogMode = m_plc.Get_C(NCPLC::C::JOG_MODE);
    const bool fineJogMode = m_plc.Get_C(NCPLC::C::FINE_JOG_MODE);
    const bool inchJogMode = m_plc.Get_C(NCPLC::C::INCH_JOG_MODE);
    const bool mpgMode = m_plc.Get_C(NCPLC::C::MPG_MODE);
    const int activeModeCount = (continuousJogMode ? 1 : 0) + (fineJogMode ? 1 : 0) + (inchJogMode ? 1 : 0) + (mpgMode ? 1 : 0);

    if (activeModeCount == 1)
    {
        if (continuousJogMode)      m_manualMoveMode = ManualMoveMode::CONTINUOUS_JOG;
        else if (fineJogMode)       m_manualMoveMode = ManualMoveMode::FINE_JOG;
        else if (inchJogMode)       m_manualMoveMode = ManualMoveMode::INCH_JOG;
        else                        m_manualMoveMode = ManualMoveMode::MPG;
    }
    else
    {
        m_manualMoveMode = ManualMoveMode::NONE;
    }

    // =========================================================
    // Manual Permission Gate
    // 只允許 NC Mode: MANUAL/MDI, NC State: IDLE/READY, C11 Servo Ready, No Alarm
    // =========================================================
    const NCOperationMode ncMode = m_nc.GetMode();
    const NCState ncState = m_nc.GetState();
    const bool operationModeAllowed = ncMode == NCOperationMode::MANUAL || ncMode == NCOperationMode::MDI;
    const bool ncStateAllowed = ncState == NCState::IDLE || ncState == NCState::READY;
    const bool alarmActive = AlarmManager::GetInstance().HasAlarm();

    if (!operationModeAllowed || !ncStateAllowed || !m_servoReady || alarmActive || m_nc.Homing.IsActive())
    {
        m_manualMoveMode = ManualMoveMode::NONE;
    }



    int jogSpeedPercent = static_cast<int>(m_plc.GetMemory("R", NCPLC::R::JOG_SPEED_PERCENT)); // R200 Continuous JOG Speed % (0 ~ 100)
    if (jogSpeedPercent < 0) jogSpeedPercent = 0;
    if (jogSpeedPercent > 100) jogSpeedPercent = 100;
    m_jogSpeedPercent = static_cast<double>(jogSpeedPercent);

    // Fine JOG Speed Select (必須 One-Hot)
    const bool fineJog0001 = m_plc.Get_C(NCPLC::C::FINE_JOG_SPEED_0001);
    const bool fineJog0010 = m_plc.Get_C(NCPLC::C::FINE_JOG_SPEED_0010);
    const bool fineJog0100 = m_plc.Get_C(NCPLC::C::FINE_JOG_SPEED_0100);
    const bool fineJog1000 = m_plc.Get_C(NCPLC::C::FINE_JOG_SPEED_1000);
    const int fineJogSpeedSelectCount = (fineJog0001 ? 1 : 0) + (fineJog0010 ? 1 : 0) + (fineJog0100 ? 1 : 0) + (fineJog1000 ? 1 : 0);

    int inchLevel = static_cast<int>(m_plc.GetMemory("R", NCPLC::R::INCH_DISTANCE_LEVEL)); // R201 INCH Distance Level (0~3)
    if (inchLevel < 0) inchLevel = 0;
    if (inchLevel > 3) inchLevel = 3;
    m_inchDistanceLevel = inchLevel; // R201 只負責選擇 INCH Distance Level。真正距離由每一軸 AxisContext 的 INCH 參數決定。

    // =========================================================
    // Manual Frame - Continuous / Fine JOG
    // CoordinateManager owns: Enabled, Yaw, Pitch, Roll
    // NCPLCManager only creates a Manual XYZ vector and asks CoordinateManager to transform it into Machine XYZ.
    // IMPORTANT: Vector rotation is done in physical units, NOT directly in Pulse space.
    // =========================================================
    const bool manualFrameVelocityMode = m_nc.GetCoordSys().IsManualFrameEnabled() && (m_manualMoveMode == ManualMoveMode::CONTINUOUS_JOG || m_manualMoveMode == ManualMoveMode::FINE_JOG);
    bool manualFrameXYZHasCommand = false;
    bool manualFrameXYZValid = true;
    bool manualFrameXYZGroupStop = false;
    double manualFrameTargetVelocityPPS[3] = { 0.0, 0.0, 0.0 };

    auto GetFineJogPPS = [&](const AxisContext& axis) -> double // Fine JOG PPS Helper
    {
        if (fineJogSpeedSelectCount != 1) return 0.0;
        if (fineJog0001) return axis.FINE_JOG_0001_PPS;
        if (fineJog0010) return axis.FINE_JOG_0010_PPS;
        if (fineJog0100) return axis.FINE_JOG_0100_PPS;
        if (fineJog1000) return axis.FINE_JOG_1000_PPS;
        return 0.0;
    };

    if (manualFrameVelocityMode)
    {
        double manualVector[8] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }; // 1. Build Manual XYZ Direction Vector (C120/C130)
        bool logicalAxisActive[3] = { false, false, false };
        bool directionConflict = false;

        for (int logicalAxis = 0; logicalAxis < 3; ++logicalAxis)
        {
            const bool positive = m_plc.Get_C(NCPLC::C::AxisPoint(NCPLC::C::JOG_POSITIVE_BASE, logicalAxis));
            const bool negative = m_plc.Get_C(NCPLC::C::AxisPoint(NCPLC::C::JOG_NEGATIVE_BASE, logicalAxis));

            if (positive && negative) // + / - 同時 ON
            {
                directionConflict = true;
                continue;
            }

            if (positive)
            {
                manualVector[logicalAxis] = 1.0;
                logicalAxisActive[logicalAxis] = true;
            }
            else if (negative)
            {
                manualVector[logicalAxis] = -1.0;
                logicalAxisActive[logicalAxis] = true;
            }
        }

        if (directionConflict)
        {
            manualFrameXYZValid = false;
        }

        const double manualMagnitude = std::sqrt(manualVector[0] * manualVector[0] + manualVector[1] * manualVector[1] + manualVector[2] * manualVector[2]); // 2. Normalize Manual XYZ Vector
        constexpr double VECTOR_EPSILON = 1.0e-12;

        if (manualMagnitude > VECTOR_EPSILON)
        {
            manualFrameXYZHasCommand = true;
            manualVector[0] /= manualMagnitude;
            manualVector[1] /= manualMagnitude;
            manualVector[2] /= manualMagnitude;

            double machineVector[8] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }; // 3. Manual Frame -> Machine Frame
            m_nc.GetCoordSys().TransformManualVector(manualVector, machineVector);

            for (int machineAxis = 0; machineAxis < 3; ++machineAxis) // 消除 sin/cos 造成的極小浮點殘值。
            {
                if (std::abs(machineVector[machineAxis]) < VECTOR_EPSILON)
                {
                    machineVector[machineAxis] = 0.0;
                }
            }

            double baseSpeedUnitPerSec = 0.0; // 4. Determine Manual Base Speed (Pulse/sec -> Unit/sec)
            bool baseSpeedInitialized = false;

            for (int logicalAxis = 0; logicalAxis < 3; ++logicalAxis)
            {
                if (!logicalAxisActive[logicalAxis]) continue;

                AxisContext& logicalContext = m_motion.GetAxisContext(logicalAxis);
                if (!logicalContext.isExist || logicalContext.resolution_PPR <= 0.0 || logicalContext.finalLead <= 0.0)
                {
                    manualFrameXYZValid = false;
                    break;
                }

                const double pulsePerUnit = logicalContext.resolution_PPR / logicalContext.finalLead;
                double logicalPPS = 0.0;

                if (m_manualMoveMode == ManualMoveMode::CONTINUOUS_JOG)
                {
                    logicalPPS = logicalContext.JOG_MAX_PPS * (m_jogSpeedPercent / 100.0);
                }
                else
                {
                    logicalPPS = GetFineJogPPS(logicalContext);
                }

                if (logicalPPS <= 0.0)
                {
                    manualFrameXYZValid = false;
                    break;
                }

                if (logicalContext.maxVel_PPS > 0.0 && logicalPPS > logicalContext.maxVel_PPS)
                {
                    logicalPPS = logicalContext.maxVel_PPS;
                }

                const double logicalSpeedUnit = logicalPPS / pulsePerUnit;
                if (!baseSpeedInitialized || logicalSpeedUnit < baseSpeedUnitPerSec)
                {
                    baseSpeedUnitPerSec = logicalSpeedUnit;
                    baseSpeedInitialized = true;
                }
            }

            if (!baseSpeedInitialized || baseSpeedUnitPerSec <= 0.0)
            {
                manualFrameXYZValid = false;
            }

            // 5. Physical XYZ Validation + Speed Limit (旋轉後必須看真正 Machine Axis: Servo, Fault, +/-OT, JOG Speed, Max Vel)
            if (manualFrameXYZValid)
            {
                for (int machineAxis = 0; machineAxis < 3; ++machineAxis)
                {
                    const double component = machineVector[machineAxis];
                    if (std::abs(component) < VECTOR_EPSILON) continue;

                    AxisContext& physicalAxis = m_motion.GetAxisContext(machineAxis);
                    const bool axisFault = physicalAxis.isFault || physicalAxis.isLagAlarm || physicalAxis.state == MotionState::MotionState_ERROR || physicalAxis.state == MotionState::MotionState_ESTOP;

                    if (!physicalAxis.isExist || !physicalAxis.isServoOn || axisFault || physicalAxis.resolution_PPR <= 0.0 || physicalAxis.finalLead <= 0.0)
                    {
                        manualFrameXYZValid = false;
                        break;
                    }

                    // =====================================================
                    // Final Travel Direction Block
                    //
                    // Manual Frame 經過旋轉後，必須看真正的
                    // Physical Machine Axis Direction。
                    //
                    // positiveTravelBlocked / negativeTravelBlocked
                    // 已經整合 Physical Hard Limit + Software Limit。
                    // =====================================================

                    if (component > 0.0 &&
                        physicalAxis.positiveTravelBlocked)
                    {
                        manualFrameXYZValid = false;
                        break;
                    }

                    if (component < 0.0 &&
                        physicalAxis.negativeTravelBlocked)
                    {
                        manualFrameXYZValid = false;
                        break;
                    }

                    const double pulsePerUnit = physicalAxis.resolution_PPR / physicalAxis.finalLead; // Physical Axis Speed Capacity
                    double physicalLimitPPS = 0.0;

                    if (m_manualMoveMode == ManualMoveMode::CONTINUOUS_JOG)
                    {
                        physicalLimitPPS = physicalAxis.JOG_MAX_PPS * (m_jogSpeedPercent / 100.0);
                    }
                    else
                    {
                        physicalLimitPPS = GetFineJogPPS(physicalAxis);
                    }

                    if (physicalLimitPPS <= 0.0)
                    {
                        manualFrameXYZValid = false;
                        break;
                    }

                    if (physicalAxis.maxVel_PPS > 0.0 && physicalLimitPPS > physicalAxis.maxVel_PPS)
                    {
                        physicalLimitPPS = physicalAxis.maxVel_PPS;
                    }

                    const double physicalLimitUnit = physicalLimitPPS / pulsePerUnit;
                    const double allowedBaseSpeed = physicalLimitUnit / std::abs(component); // base <= AxisLimit / component

                    if (allowedBaseSpeed < baseSpeedUnitPerSec)
                    {
                        baseSpeedUnitPerSec = allowedBaseSpeed;
                    }
                }
            }

            // 6. Build Physical Axis Target PPS
            if (manualFrameXYZValid && baseSpeedUnitPerSec > 0.0)
            {
                for (int machineAxis = 0; machineAxis < 3; ++machineAxis)
                {
                    AxisContext& physicalAxis = m_motion.GetAxisContext(machineAxis);
                    if (physicalAxis.resolution_PPR <= 0.0 || physicalAxis.finalLead <= 0.0)
                    {
                        manualFrameXYZValid = false;
                        break;
                    }
                    const double pulsePerUnit = physicalAxis.resolution_PPR / physicalAxis.finalLead;
                    manualFrameTargetVelocityPPS[machineAxis] = baseSpeedUnitPerSec * machineVector[machineAxis] * pulsePerUnit;
                }
            }

            // 7. Coordinated Direction Change (任一 XYZ 需要反向，整組先停)
            if (manualFrameXYZValid)
            {
                for (int machineAxis = 0; machineAxis < 3; ++machineAxis)
                {
                    AxisContext& physicalAxis = m_motion.GetAxisContext(machineAxis);
                    const double targetVelocity = manualFrameTargetVelocityPPS[machineAxis];
                    const bool needsMotion = std::abs(targetVelocity) > 0.01;

                    if (needsMotion)
                    {
                        if (physicalAxis.state != MotionState::MotionState_IDLE && physicalAxis.state != MotionState::MotionState_VELOCITY) // Manual Frame 不能搶其他 Motion。
                        {
                            manualFrameXYZValid = false;
                            break;
                        }

                        if (physicalAxis.state == MotionState::MotionState_VELOCITY)
                        {
                            if (!m_jogActive[machineAxis]) // 只有我們持有的 Manual JOG 才允許更新 Velocity。
                            {
                                manualFrameXYZValid = false;
                                break;
                            }
                            const bool reversing = (physicalAxis.currentCmdVel > 0.0 && targetVelocity < 0.0) || (physicalAxis.currentCmdVel < 0.0 && targetVelocity > 0.0);
                            if (reversing)
                            {
                                manualFrameXYZGroupStop = true;
                            }
                        }
                    }
                    else
                    {
                        if (m_jogActive[machineAxis] && physicalAxis.state == MotionState::MotionState_VELOCITY) // 原本是旋轉向量的一部分，新向量現在變成 0，也先整組停再重建方向。
                        {
                            manualFrameXYZGroupStop = true;
                        }
                    }
                }
            }
        }
    }

    // =========================================================
    // MPG Input Decode
    // C210~217=Axis Select, C220~223=Multiplier, DR200=Accumulated Handwheel Count
    // =========================================================
    int mpgSelectedAxis = -1;
    int mpgAxisSelectCount = 0;

    for (int i = 0; i < NCPLC::AXIS_COUNT; ++i)
    {
        const bool selected = m_plc.Get_C(NCPLC::C::AxisPoint(NCPLC::C::MPG_AXIS_SELECT_BASE, i));
        if (selected)
        {
            ++mpgAxisSelectCount;
            mpgSelectedAxis = i;
        }
    }

    const bool mpgX1 = m_plc.Get_C(NCPLC::C::MPG_MULTIPLIER_X1);
    const bool mpgX10 = m_plc.Get_C(NCPLC::C::MPG_MULTIPLIER_X10);
    const bool mpgX100 = m_plc.Get_C(NCPLC::C::MPG_MULTIPLIER_X100);
    const bool mpgX1000 = m_plc.Get_C(NCPLC::C::MPG_MULTIPLIER_X1000);
    const int mpgMultiplierSelectCount = (mpgX1 ? 1 : 0) + (mpgX10 ? 1 : 0) + (mpgX100 ? 1 : 0) + (mpgX1000 ? 1 : 0);

    double mpgMultiplier = 1.0;
    if (mpgX10) mpgMultiplier = 10.0;
    else if (mpgX100) mpgMultiplier = 100.0;
    else if (mpgX1000) mpgMultiplier = 1000.0;

    const int32_t currentMPGCount = static_cast<int32_t>(m_plc.GetMemory("DR", NCPLC::DR::MPG_ENCODER_COUNT));
    int64_t mpgDelta64 = static_cast<int64_t>(currentMPGCount) - static_cast<int64_t>(m_lastMPGCount); // Signed 32-bit wrap-safe delta

    if (mpgDelta64 > 2147483647LL) mpgDelta64 -= 4294967296LL;
    else if (mpgDelta64 < -2147483648LL) mpgDelta64 += 4294967296LL;
    int32_t mpgDeltaCount = static_cast<int32_t>(mpgDelta64);

    m_lastMPGCount = currentMPGCount; // Always consume the current counter value.

    const bool mpgCommandValid = m_manualMoveMode == ManualMoveMode::MPG && mpgAxisSelectCount == 1 && mpgMultiplierSelectCount == 1;
    if (!mpgCommandValid)
    {
        mpgDeltaCount = 0; // Invalid selection must not queue movement for later.
    }

    // =========================================================
    // Axis Manual Motion
    // =========================================================
    for (int i = 0; i < NCPLC::AXIS_COUNT; ++i)
    {
        AxisContext& axis = m_motion.GetAxisContext(i);

        const bool rawPositive = m_plc.Get_C(NCPLC::C::AxisPoint(NCPLC::C::JOG_POSITIVE_BASE, i)); // Raw Direction: + : C120 ~ C127
        const bool rawNegative = m_plc.Get_C(NCPLC::C::AxisPoint(NCPLC::C::JOG_NEGATIVE_BASE, i)); // Raw Direction: - : C130 ~ C137

        const bool positiveRising = rawPositive && !m_prevJogPositive[i]; // Rising Edge (INCH 使用)
        const bool negativeRising = rawNegative && !m_prevJogNegative[i];

        m_prevJogPositive[i] = rawPositive; // Edge Memory (必須每 Scan 都更新)
        m_prevJogNegative[i] = rawNegative;

        m_jogPositive[i] = false; // Default Valid Direction
        m_jogNegative[i] = false;

        // Manual Motion Ownership Release (真正 IDLE 後才釋放 ownership)
        if (m_jogActive[i] && axis.state == MotionState::MotionState_IDLE)
        {
            m_jogActive[i] = false;
        }

        if (!axis.isExist) // Axis Existence
        {
            m_jogActive[i] = false;
            continue;
        }

        const bool axisFault = axis.isFault || axis.isLagAlarm || axis.state == MotionState::MotionState_ERROR || axis.state == MotionState::MotionState_ESTOP; // Axis Fault Gate
        if (axisFault)
        {
            m_jogActive[i] = false;
            continue;
        }

        if (!axis.isServoOn) // Servo On Gate
        {
            if (m_jogActive[i])
            {
                if (axis.state == MotionState::MotionState_VELOCITY) m_motion.StopMove(axis, axis.JOG_dec_time);
                else if (axis.state == MotionState::MotionState_MOVING) m_motion.StopMove(axis, axis.INCH_dec_time);
                else if (axis.state == MotionState::MotionState_MPG) m_motion.StopMove(axis, axis.JOG_dec_time);
            }
            continue;
        }

        // Mode Change Stop: Continuous JOG 還沒停完，卻切到其他 Manual Mode：先停止。
        const bool velocityJogModeActive = m_manualMoveMode == ManualMoveMode::CONTINUOUS_JOG || m_manualMoveMode == ManualMoveMode::FINE_JOG;
        if (m_jogActive[i] && axis.state == MotionState::MotionState_VELOCITY && !velocityJogModeActive)
        {
            m_motion.StopMove(axis, axis.JOG_dec_time);
            continue;
        }

        // INCH 還在 P2P，卻切離 INCH Mode：做 Controlled Stop。
        if (m_jogActive[i] && axis.state == MotionState::MotionState_MOVING && m_manualMoveMode != ManualMoveMode::INCH_JOG)
        {
            m_motion.StopMove(axis, axis.INCH_dec_time);
            continue;
        }

        // MPG Mode Change Stop
        if (m_jogActive[i] && axis.state == MotionState::MotionState_MPG && m_manualMoveMode != ManualMoveMode::MPG)
        {
            m_motion.StopMove(axis, axis.JOG_dec_time);
            continue;
        }

        if (m_manualMoveMode == ManualMoveMode::NONE) // Manual Mode NONE
        {
            continue;
        }

        // =====================================================
        // MPG / Manual Pulse Generator
        // =====================================================
        if (m_manualMoveMode == ManualMoveMode::MPG)
        {
            // Manual Frame MPG 只處理 XYZ
            const bool manualFrameMPG = m_nc.GetCoordSys().IsManualFrameEnabled() && mpgCommandValid && mpgSelectedAxis >= 0 && mpgSelectedAxis < 3;
            if (manualFrameMPG)
            {
                if (i >= 3) // 其他實體軸如果之前還停留在 MPG，先 Controlled Stop。
                {
                    if (axis.state == MotionState::MotionState_MPG)
                    {
                        m_motion.StopMove(axis, axis.JOG_dec_time);
                        m_jogActive[i] = true;
                    }
                    continue;
                }

                if (i != mpgSelectedAxis) continue; // 整組 XYZ 只執行一次。以目前 MPG Selected Logical Axis 當作 Group Executor。

                auto StopManualFrameMPGXYZ = [&]() // Helper: Stop all XYZ MPG axes
                {
                    for (int machineAxis = 0; machineAxis < 3; ++machineAxis)
                    {
                        AxisContext& physicalAxis = m_motion.GetAxisContext(machineAxis);
                        if (physicalAxis.state == MotionState::MotionState_MPG)
                        {
                            m_motion.StopMove(physicalAxis, physicalAxis.JOG_dec_time);
                            m_jogActive[machineAxis] = true;
                        }
                    }
                };

                if (mpgDeltaCount == 0) continue; // No New Count 不呼叫 MPGMove。已經是 MPG State 的軸會繼續追之前 finalTargetPos。

                AxisContext& logicalAxis = axis; // Logical Manual Axis Parameters
                if (logicalAxis.MPG_BASE_DISTANCE <= 0.0 || logicalAxis.MPG_MAX_PPS <= 0.0 || logicalAxis.resolution_PPR <= 0.0 || logicalAxis.finalLead <= 0.0)
                {
                    StopManualFrameMPGXYZ();
                    continue;
                }

                const double logicalPulsePerUnit = logicalAxis.resolution_PPR / logicalAxis.finalLead;
                double manualVector[8] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }; // Manual Unit Direction
                manualVector[mpgSelectedAxis] = 1.0;

                double machineDirection[8] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }; // Manual Frame -> Machine Frame
                m_nc.GetCoordSys().TransformManualVector(manualVector, machineDirection);

                constexpr double MPG_VECTOR_EPSILON = 1.0e-10;
                for (int machineAxis = 0; machineAxis < 3; ++machineAxis) // 清掉旋轉矩陣的微小浮點殘值
                {
                    if (std::abs(machineDirection[machineAxis]) < MPG_VECTOR_EPSILON)
                    {
                        machineDirection[machineAxis] = 0.0;
                    }
                }

                const double deltaDistance = static_cast<double>(mpgDeltaCount) * logicalAxis.MPG_BASE_DISTANCE * mpgMultiplier; // DR200 Count -> Logical Distance
                if (std::abs(deltaDistance) <= MPG_VECTOR_EPSILON) continue;

                double logicalMaxPPS = logicalAxis.MPG_MAX_PPS; // Logical MPG Max Path Speed
                if (logicalAxis.maxVel_PPS > 0.0 && logicalMaxPPS > logicalAxis.maxVel_PPS) logicalMaxPPS = logicalAxis.maxVel_PPS;
                double pathMaxUnitPerSec = logicalMaxPPS / logicalPulsePerUnit;

                if (pathMaxUnitPerSec <= 0.0)
                {
                    StopManualFrameMPGXYZ();
                    continue;
                }

                double groupAccTime = logicalAxis.JOG_acc_time; // Group Acc / Dec (所有 XYZ 使用同一組時間)
                double groupDecTime = logicalAxis.JOG_dec_time;
                if (groupAccTime < 0.001) groupAccTime = 0.2;
                if (groupDecTime < 0.001) groupDecTime = groupAccTime;
                bool groupValid = true;
                bool softwareTargetRejected = false;

                for (int machineAxis = 0; machineAxis < 3; ++machineAxis) // Validate Physical Machine XYZ (Limit 要依照旋轉後實際方向判斷)
                {
                    const double component = machineDirection[machineAxis];
                    if (std::abs(component) <= MPG_VECTOR_EPSILON) continue;

                    AxisContext& physicalAxis = m_motion.GetAxisContext(machineAxis);
                    const bool physicalFault = physicalAxis.isFault || physicalAxis.isLagAlarm || physicalAxis.state == MotionState::MotionState_ERROR || physicalAxis.state == MotionState::MotionState_ESTOP;

                    if (!physicalAxis.isExist || !physicalAxis.isServoOn || physicalFault || physicalAxis.resolution_PPR <= 0.0 || physicalAxis.finalLead <= 0.0 || physicalAxis.MPG_MAX_PPS <= 0.0)
                    {
                        groupValid = false;
                        break;
                    }

                    if (physicalAxis.state != MotionState::MotionState_IDLE && physicalAxis.state != MotionState::MotionState_MPG)
                    {
                        groupValid = false;
                        break;
                    }

                    const double physicalDeltaUnit = deltaDistance * component; // Rotated Physical Delta
                    if (physicalAxis.hardLimitPositive &&
                        physicalAxis.hardLimitNegative)
                    {
                        groupValid = false;
                        break;
                    }

                    if (physicalDeltaUnit > 0.0 &&
                        physicalAxis.positiveTravelBlocked)
                    {
                        groupValid = false;

                        if (!physicalAxis.hardLimitPositive)
                        {
                            softwareTargetRejected = true;
                        }

                        break;
                    }

                    if (physicalDeltaUnit < 0.0 &&
                        physicalAxis.negativeTravelBlocked)
                    {
                        groupValid = false;

                        if (!physicalAxis.hardLimitNegative)
                        {
                            softwareTargetRejected = true;
                        }

                        break;
                    }

                    const double pulsePerUnit = physicalAxis.resolution_PPR / physicalAxis.finalLead; // Physical Axis MPG Speed Capacity
                    const double deltaPulse = physicalDeltaUnit * pulsePerUnit;
                    const double baseTarget = (physicalAxis.state == MotionState::MotionState_MPG) ? physicalAxis.finalTargetPos : physicalAxis.currentCmdPos;
                    const double targetPosition = baseTarget + deltaPulse;
                    const double targetMCS = targetPosition / pulsePerUnit;

                    const bool targetWithinSoftwareLimit = m_nc.GetCoordSys().IsTargetWithinSoftwareTravelLimit(physicalAxis, targetMCS);
                    const bool positiveSoftwareLimitActive = physicalAxis.travelLimit1PositiveActive || physicalAxis.travelLimit2PositiveActive || physicalAxis.travelLimit3PositiveActive;
                    const bool negativeSoftwareLimitActive = physicalAxis.travelLimit1NegativeActive || physicalAxis.travelLimit2NegativeActive || physicalAxis.travelLimit3NegativeActive;
                    const bool recoveringFromPositiveLimit = positiveSoftwareLimitActive && physicalDeltaUnit < 0.0;
                    const bool recoveringFromNegativeLimit = negativeSoftwareLimitActive && physicalDeltaUnit > 0.0;

                    if (!targetWithinSoftwareLimit && !recoveringFromPositiveLimit && !recoveringFromNegativeLimit)
                    {
                        groupValid = false;
                        softwareTargetRejected = true;
                        break;
                    }

                    double physicalMaxPPS = physicalAxis.MPG_MAX_PPS;
                    if (physicalAxis.maxVel_PPS > 0.0 && physicalMaxPPS > physicalAxis.maxVel_PPS) physicalMaxPPS = physicalAxis.maxVel_PPS;
                    const double physicalMaxUnitPerSec = physicalMaxPPS / pulsePerUnit;
                    const double allowedPathSpeed = physicalMaxUnitPerSec / std::abs(component);

                    if (allowedPathSpeed < pathMaxUnitPerSec)
                    {
                        pathMaxUnitPerSec = allowedPathSpeed;
                    }

                    double physicalAccTime = physicalAxis.JOG_acc_time; // Group Acc / Dec 取參與軸較慢的時間
                    double physicalDecTime = physicalAxis.JOG_dec_time;
                    if (physicalAccTime < 0.001) physicalAccTime = 0.2;
                    if (physicalDecTime < 0.001) physicalDecTime = physicalAccTime;
                    if (physicalAccTime > groupAccTime) groupAccTime = physicalAccTime;
                    if (physicalDecTime > groupDecTime) groupDecTime = physicalDecTime;
                }

                if (!groupValid || pathMaxUnitPerSec <= 0.0)
                {
                    if (!softwareTargetRejected) StopManualFrameMPGXYZ();
                    continue;
                }

                for (int machineAxis = 0; machineAxis < 3; ++machineAxis) // Execute Physical XYZ MPG
                {
                    const double component = machineDirection[machineAxis];
                    AxisContext& physicalAxis = m_motion.GetAxisContext(machineAxis);

                    if (std::abs(component) <= MPG_VECTOR_EPSILON) // 此軸不參與目前旋轉方向
                    {
                        if (physicalAxis.state == MotionState::MotionState_MPG)
                        {
                            m_motion.StopMove(physicalAxis, physicalAxis.JOG_dec_time);
                            m_jogActive[machineAxis] = true;
                        }
                        continue;
                    }

                    const double pulsePerUnit = physicalAxis.resolution_PPR / physicalAxis.finalLead;
                    const double physicalDeltaUnit = deltaDistance * component;
                    const double deltaPulse = physicalDeltaUnit * pulsePerUnit;
                    if (std::abs(deltaPulse) <= MPG_VECTOR_EPSILON) continue;

                    const double baseTarget = (physicalAxis.state == MotionState::MotionState_MPG) ? physicalAxis.finalTargetPos : physicalAxis.currentCmdPos; // Dynamic Target Accumulation
                    const double targetPosition = baseTarget + deltaPulse;

                    double physicalMPGMaxPPS = pathMaxUnitPerSec * std::abs(component) * pulsePerUnit; // Physical Axis Max PPS
                    if (physicalAxis.MPG_MAX_PPS > 0.0 && physicalMPGMaxPPS > physicalAxis.MPG_MAX_PPS) physicalMPGMaxPPS = physicalAxis.MPG_MAX_PPS;
                    if (physicalAxis.maxVel_PPS > 0.0 && physicalMPGMaxPPS > physicalAxis.maxVel_PPS) physicalMPGMaxPPS = physicalAxis.maxVel_PPS;
                    if (physicalMPGMaxPPS <= 0.0) continue;

                    m_motion.MPGMove(physicalAxis, targetPosition, physicalMPGMaxPPS, groupAccTime, groupDecTime); // MPG Motion
                    if (physicalAxis.state == MotionState::MotionState_MPG) m_jogActive[machineAxis] = true;
                }
                continue;
            }

            // Original MPG: 以下情況全部走原本行為 (Manual Frame OFF 或 A/B/C/U/V)
            if (!mpgCommandValid || i != mpgSelectedAxis) // Invalid selector or this is not selected axis.
            {
                if (axis.state == MotionState::MotionState_MPG)
                {
                    m_motion.StopMove(axis, axis.JOG_dec_time);
                    m_jogActive[i] = true;
                }
                continue;
            }

            if (axis.state != MotionState::MotionState_IDLE && axis.state != MotionState::MotionState_MPG) continue; // MPG cannot steal another Motion owner.
            if (mpgDeltaCount == 0) continue; // No new handwheel count

            const bool blockedPositive =
                mpgDeltaCount > 0 &&
                axis.positiveTravelBlocked;

            const bool blockedNegative =
                mpgDeltaCount < 0 &&
                axis.negativeTravelBlocked;

            if (axis.hardLimitPositive &&
                axis.hardLimitNegative)
            {
                if (axis.state == MotionState::MotionState_MPG)
                {
                    m_motion.StopMove(axis, axis.JOG_dec_time);
                    m_jogActive[i] = true;
                }

                continue;
            }

            if (blockedPositive || blockedNegative)
            {
                const bool blockedByPhysicalLimit =
                    (blockedPositive && axis.hardLimitPositive) ||
                    (blockedNegative && axis.hardLimitNegative);

                if (blockedByPhysicalLimit &&
                    axis.state == MotionState::MotionState_MPG)
                {
                    m_motion.StopMove(axis, axis.JOG_dec_time);
                    m_jogActive[i] = true;
                }

                continue;
            }

            if (axis.MPG_BASE_DISTANCE <= 0.0 || axis.MPG_MAX_PPS <= 0.0 || axis.resolution_PPR <= 0.0 || axis.finalLead <= 0.0) continue; // Axis Parameters

            const double pulsePerUnit = axis.resolution_PPR / axis.finalLead;
            const double deltaDistance = static_cast<double>(mpgDeltaCount) * axis.MPG_BASE_DISTANCE * mpgMultiplier;
            const double deltaPulse = deltaDistance * pulsePerUnit;
            if (deltaPulse == 0.0) continue;

            const double baseTarget = (axis.state == MotionState::MotionState_MPG) ? axis.finalTargetPos : axis.currentCmdPos; // Dynamic Absolute Target
            const double targetPosition = baseTarget + deltaPulse;
            const double targetMCS = targetPosition / pulsePerUnit;

            const bool targetWithinSoftwareLimit = m_nc.GetCoordSys().IsTargetWithinSoftwareTravelLimit(axis, targetMCS);
            const bool positiveSoftwareLimitActive = axis.travelLimit1PositiveActive || axis.travelLimit2PositiveActive || axis.travelLimit3PositiveActive;
            const bool negativeSoftwareLimitActive = axis.travelLimit1NegativeActive || axis.travelLimit2NegativeActive || axis.travelLimit3NegativeActive;
            const bool recoveringFromPositiveLimit = positiveSoftwareLimitActive && deltaDistance < 0.0;
            const bool recoveringFromNegativeLimit = negativeSoftwareLimitActive && deltaDistance > 0.0;

            if (!targetWithinSoftwareLimit && !recoveringFromPositiveLimit && !recoveringFromNegativeLimit)
            {
                continue;
            }

            m_motion.MPGMove(axis, targetPosition, axis.MPG_MAX_PPS, axis.JOG_acc_time, axis.JOG_dec_time); // Original MPG MotionCore
            if (axis.state == MotionState::MotionState_MPG) m_jogActive[i] = true;

            continue;
        }

        // =====================================================
        // Manual Frame - Normal / Fine JOG XYZ
        // =====================================================
        if (manualFrameVelocityMode && i < 3)
        {
            if (!manualFrameXYZHasCommand || !manualFrameXYZValid || manualFrameXYZGroupStop) // No command / Invalid / Coordinated Stop
            {
                if (m_jogActive[i] && axis.state == MotionState::MotionState_VELOCITY) m_motion.StopMove(axis, axis.JOG_dec_time);
                continue;
            }

            const double targetVelocity = manualFrameTargetVelocityPPS[i];
            if (std::abs(targetVelocity) <= 0.01) // This physical axis does not participate in the rotated vector.
            {
                if (m_jogActive[i] && axis.state == MotionState::MotionState_VELOCITY) m_motion.StopMove(axis, axis.JOG_dec_time);
                continue;
            }

            if (axis.state == MotionState::MotionState_STOPPING) continue; // STOPPING must finish first.
            if (axis.state != MotionState::MotionState_IDLE && axis.state != MotionState::MotionState_VELOCITY) continue; // Do not steal P2P / Interpolation / MPG.

            m_motion.VelocityMove(axis, targetVelocity, axis.JOG_acc_time); // Execute physical machine-axis velocity.
            m_jogActive[i] = true;
            continue;
        }

        // =====================================================
        // Direction Conflict: + / - 同時 ON 不允許移動。
        // =====================================================
        if (rawPositive && rawNegative)
        {
            if (m_jogActive[i] && axis.state == MotionState::MotionState_VELOCITY) m_motion.StopMove(axis, axis.JOG_dec_time);
            continue;
        }

        // =====================================================
 // Physical Hard Limit
 //
 // +OT / -OT
 //
 // Physical Limit 永遠有效。
 // =====================================================

        // =====================================================
        // Final Travel Direction Block
        //
        // axis.positiveTravelBlocked / negativeTravelBlocked
        // 已經整合：
        //
        // 1. Physical +OT / -OT
        // 2. Software Travel Limit 1 / 2 / 3
        //
        // 這裡只需要套用 Operator Direction。
        // =====================================================

        const bool allowPositive =
            rawPositive &&
            !axis.positiveTravelBlocked;

        const bool allowNegative =
            rawNegative &&
            !axis.negativeTravelBlocked;


        // =====================================================
        // Physical +OT 與 -OT 同時 ON
        //
        // 這屬於異常 Hardware Input State。
        //
        // Software Limit 不使用這個判斷，
        // 因為 Software + / - Block 理論上可能因 Parameter
        // 設定問題而另外處理。
        // =====================================================

        if (axis.hardLimitPositive &&
            axis.hardLimitNegative)
        {
            if (m_jogActive[i] && axis.state == MotionState::MotionState_VELOCITY)
            {
                m_motion.StopMove(axis, axis.JOG_dec_time);
            }

            continue;
        }

        m_jogPositive[i] = allowPositive;
        m_jogNegative[i] = allowNegative;

        // =====================================================
        // 1. CONTINUOUS JOG
        // =====================================================
        if (m_manualMoveMode == ManualMoveMode::CONTINUOUS_JOG)
        {
            if (!allowPositive && !allowNegative) // 沒有方向
            {
                if (m_jogActive[i] && axis.state == MotionState::MotionState_VELOCITY) m_motion.StopMove(axis, axis.JOG_dec_time);
                continue;
            }

            if (axis.JOG_MAX_PPS <= 0.0) continue; // 基本 JOG 速度

            double jogVelocity = axis.JOG_MAX_PPS * (m_jogSpeedPercent / 100.0);
            if (axis.maxVel_PPS > 0.0 && jogVelocity > axis.maxVel_PPS) jogVelocity = axis.maxVel_PPS; // Clamp Axis Maximum

            if (jogVelocity <= 0.0) // R200 = 0
            {
                if (m_jogActive[i] && axis.state == MotionState::MotionState_VELOCITY) m_motion.StopMove(axis, axis.JOG_dec_time);
                continue;
            }

            double targetVelocity = 0.0; // Direction
            if (allowPositive) targetVelocity = jogVelocity;
            else if (allowNegative) targetVelocity = -jogVelocity;

            if (axis.state == MotionState::MotionState_STOPPING) continue; // STOPPING 一定等真正停到 IDLE
            if (axis.state == MotionState::MotionState_MOVING || axis.state == MotionState::MotionState_INTERPOLATING) continue; // P2P / Interpolation 正在使用 Axis

            if (axis.state == MotionState::MotionState_VELOCITY) // Direction Reversal
            {
                const bool reversing = (axis.currentCmdVel > 0.0 && targetVelocity < 0.0) || (axis.currentCmdVel < 0.0 && targetVelocity > 0.0);
                if (reversing)
                {
                    m_motion.StopMove(axis, axis.JOG_dec_time);
                    continue;
                }
            }

            m_motion.VelocityMove(axis, targetVelocity, axis.JOG_acc_time); // Velocity Move
            m_jogActive[i] = true;
            continue;
        }

        // =====================================================
        // 2. FINE CONTINUOUS JOG
        // =====================================================
        if (m_manualMoveMode == ManualMoveMode::FINE_JOG)
        {
            if (!allowPositive && !allowNegative)
            {
                if (m_jogActive[i] && axis.state == MotionState::MotionState_VELOCITY) m_motion.StopMove(axis, axis.JOG_dec_time);
                continue;
            }

            if (fineJogSpeedSelectCount != 1) // Fine Speed Selector 必須 One-Hot。
            {
                if (m_jogActive[i] && axis.state == MotionState::MotionState_VELOCITY) m_motion.StopMove(axis, axis.JOG_dec_time);
                continue;
            }

            double fineJogVelocity = 0.0;
            if (fineJog0001) fineJogVelocity = axis.FINE_JOG_0001_PPS;
            else if (fineJog0010) fineJogVelocity = axis.FINE_JOG_0010_PPS;
            else if (fineJog0100) fineJogVelocity = axis.FINE_JOG_0100_PPS;
            else if (fineJog1000) fineJogVelocity = axis.FINE_JOG_1000_PPS;

            if (fineJogVelocity <= 0.0)
            {
                if (m_jogActive[i] && axis.state == MotionState::MotionState_VELOCITY) m_motion.StopMove(axis, axis.JOG_dec_time);
                continue;
            }

            if (axis.maxVel_PPS > 0.0 && fineJogVelocity > axis.maxVel_PPS) fineJogVelocity = axis.maxVel_PPS;

            double targetVelocity = 0.0;
            if (allowPositive) targetVelocity = fineJogVelocity;
            else if (allowNegative) targetVelocity = -fineJogVelocity;

            if (axis.state == MotionState::MotionState_STOPPING) continue;
            if (axis.state == MotionState::MotionState_MOVING || axis.state == MotionState::MotionState_INTERPOLATING) continue;

            if (axis.state == MotionState::MotionState_VELOCITY) // 反向時先停到 IDLE，再反向。
            {
                const bool reversing = (axis.currentCmdVel > 0.0 && targetVelocity < 0.0) || (axis.currentCmdVel < 0.0 && targetVelocity > 0.0);
                if (reversing)
                {
                    m_motion.StopMove(axis, axis.JOG_dec_time);
                    continue;
                }
            }

            m_motion.VelocityMove(axis, targetVelocity, axis.JOG_acc_time); // Fine JOG 與 Normal JOG 共用加減速。C200~C203 切換時可直接更新 targetVelocity。
            m_jogActive[i] = true;
            continue;
        }

        // =====================================================
        // 3. INCH JOG
        // =====================================================
        if (m_manualMoveMode == ManualMoveMode::INCH_JOG)
        {
            // A. Manual Frame XYZ INCH
            if (m_nc.GetCoordSys().IsManualFrameEnabled() && i < 3)
            {
                if (axis.state != MotionState::MotionState_IDLE) continue; // 一次 INCH 必須從完全停止開始

                double inchDistance = 0.0; // Logical Axis INCH Distance
                switch (m_inchDistanceLevel)
                {
                case 0: inchDistance = axis.INCH_0001_DISTANCE; break;
                case 1: inchDistance = axis.INCH_0010_DISTANCE; break;
                case 2: inchDistance = axis.INCH_0100_DISTANCE; break;
                case 3: inchDistance = axis.INCH_1000_DISTANCE; break;
                default: inchDistance = axis.INCH_0001_DISTANCE; break;
                }
                if (inchDistance <= 0.0) continue;

                const bool inchPositive = positiveRising; // Rising Edge: INCH 一次按壓只動一次。
                const bool inchNegative = negativeRising;
                if (!inchPositive && !inchNegative) continue;
                if (inchPositive && inchNegative) continue;

                double manualVector[8] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }; // Manual Distance Vector (INCH 是距離，不做 Normalize)
                manualVector[i] = inchPositive ? inchDistance : -inchDistance;

                double machineVector[8] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }; // Manual Frame -> Machine Frame
                m_nc.GetCoordSys().TransformManualVector(manualVector, machineVector);

                constexpr double INCH_VECTOR_EPSILON = 1.0e-10;
                const double pathDistance = std::sqrt(machineVector[0] * machineVector[0] + machineVector[1] * machineVector[1] + machineVector[2] * machineVector[2]); // Rotated Path Length
                if (pathDistance <= INCH_VECTOR_EPSILON) continue;

                if (axis.resolution_PPR <= 0.0 || axis.finalLead <= 0.0 || axis.INCH_JOG_PPS <= 0.0) continue; // Logical Axis Speed
                const double logicalPulsePerUnit = axis.resolution_PPR / axis.finalLead;
                double pathSpeedUnitPerSec = axis.INCH_JOG_PPS / logicalPulsePerUnit;

                if (axis.maxVel_PPS > 0.0)
                {
                    const double logicalMaxUnitPerSec = axis.maxVel_PPS / logicalPulsePerUnit;
                    if (pathSpeedUnitPerSec > logicalMaxUnitPerSec) pathSpeedUnitPerSec = logicalMaxUnitPerSec;
                }
                if (pathSpeedUnitPerSec <= 0.0) continue;

                bool groupValid = true; // Validate Physical XYZ Group
                double groupAccTime = 0.001;
                double groupDecTime = 0.001;

                for (int machineAxis = 0; machineAxis < 3; ++machineAxis)
                {
                    const double deltaUnit = machineVector[machineAxis];
                    if (std::abs(deltaUnit) <= INCH_VECTOR_EPSILON) continue;

                    AxisContext& physicalAxis = m_motion.GetAxisContext(machineAxis);
                    const bool physicalAxisFault = physicalAxis.isFault || physicalAxis.isLagAlarm || physicalAxis.state == MotionState::MotionState_ERROR || physicalAxis.state == MotionState::MotionState_ESTOP;

                    if (!physicalAxis.isExist || !physicalAxis.isServoOn || physicalAxisFault || physicalAxis.state != MotionState::MotionState_IDLE) // Axis / Servo / Motion Ownership
                    {
                        groupValid = false;
                        break;
                    }
                    if (physicalAxis.resolution_PPR <= 0.0 || physicalAxis.finalLead <= 0.0 || physicalAxis.INCH_JOG_PPS <= 0.0)
                    {
                        groupValid = false;
                        break;
                    }

                    if (physicalAxis.hardLimitPositive &&
                        physicalAxis.hardLimitNegative)
                    {
                        groupValid = false;
                        break;
                    }

                    if (deltaUnit > 0.0 &&
                        physicalAxis.positiveTravelBlocked)
                    {
                        groupValid = false;
                        break;
                    }

                    if (deltaUnit < 0.0 &&
                        physicalAxis.negativeTravelBlocked)
                    {
                        groupValid = false;
                        break;
                    }

                    const double pulsePerUnit = physicalAxis.resolution_PPR / physicalAxis.finalLead; // Physical Axis Speed Capacity
                    const double deltaPulse = deltaUnit * pulsePerUnit;
                    const double targetPosition = physicalAxis.currentActPos + deltaPulse;
                    const double targetMCS = targetPosition / pulsePerUnit;

                    const bool targetWithinSoftwareLimit = m_nc.GetCoordSys().IsTargetWithinSoftwareTravelLimit(physicalAxis, targetMCS);
                    const bool positiveSoftwareLimitActive = physicalAxis.travelLimit1PositiveActive || physicalAxis.travelLimit2PositiveActive || physicalAxis.travelLimit3PositiveActive;
                    const bool negativeSoftwareLimitActive = physicalAxis.travelLimit1NegativeActive || physicalAxis.travelLimit2NegativeActive || physicalAxis.travelLimit3NegativeActive;
                    const bool recoveringFromPositiveLimit = positiveSoftwareLimitActive && deltaUnit < 0.0;
                    const bool recoveringFromNegativeLimit = negativeSoftwareLimitActive && deltaUnit > 0.0;

                    if (!targetWithinSoftwareLimit && !recoveringFromPositiveLimit && !recoveringFromNegativeLimit)
                    {
                        groupValid = false;
                        break;
                    }

                    double physicalMaxPPS = physicalAxis.INCH_JOG_PPS;
                    if (physicalAxis.maxVel_PPS > 0.0 && physicalMaxPPS > physicalAxis.maxVel_PPS) physicalMaxPPS = physicalAxis.maxVel_PPS;

                    const double physicalMaxUnitPerSec = physicalMaxPPS / pulsePerUnit;
                    const double directionComponent = std::abs(deltaUnit) / pathDistance;

                    if (directionComponent > INCH_VECTOR_EPSILON)
                    {
                        const double allowedPathSpeed = physicalMaxUnitPerSec / directionComponent;
                        if (allowedPathSpeed < pathSpeedUnitPerSec) pathSpeedUnitPerSec = allowedPathSpeed;
                    }

                    double axisAccTime = physicalAxis.INCH_acc_time; // Group Acc / Dec (所有參與 XYZ 使用相同時間)
                    double axisDecTime = physicalAxis.INCH_dec_time;
                    if (axisAccTime < 0.001) axisAccTime = 0.2;
                    if (axisDecTime < 0.001) axisDecTime = axisAccTime;
                    if (axisAccTime > groupAccTime) groupAccTime = axisAccTime;
                    if (axisDecTime > groupDecTime) groupDecTime = axisDecTime;
                }

                if (!groupValid || pathSpeedUnitPerSec <= 0.0) continue;

                for (int machineAxis = 0; machineAxis < 3; ++machineAxis) // Execute Rotated XYZ INCH
                {
                    const double deltaUnit = machineVector[machineAxis];
                    if (std::abs(deltaUnit) <= INCH_VECTOR_EPSILON) continue;

                    AxisContext& physicalAxis = m_motion.GetAxisContext(machineAxis);
                    const double pulsePerUnit = physicalAxis.resolution_PPR / physicalAxis.finalLead;
                    const double deltaPulse = deltaUnit * pulsePerUnit;
                    const double targetPosition = physicalAxis.currentActPos + deltaPulse;

                    const double directionComponent = std::abs(deltaUnit) / pathDistance;
                    double axisVelocity = pathSpeedUnitPerSec * directionComponent * pulsePerUnit;

                    if (axisVelocity <= 0.0) continue;
                    if (physicalAxis.maxVel_PPS > 0.0 && axisVelocity > physicalAxis.maxVel_PPS) axisVelocity = physicalAxis.maxVel_PPS;

                    m_motion.MoveToPosition(physicalAxis, targetPosition, axisVelocity, groupAccTime, groupDecTime); // XYZ 是 Linear Axis，不需要 Rotary Shortest Path 處理。
                    m_jogActive[machineAxis] = true;
                }

                continue; // 已完成 Manual Frame XYZ INCH。絕對不可再往下執行舊單軸 INCH。
            }

            // B. Original INCH (Manual Frame OFF 或 A/B/C/U/V)
            if (axis.state != MotionState::MotionState_IDLE) continue; // INCH 只能從完全停止開始

            double inchDistance = 0.0; // R201 -> 每軸自己的 INCH Distance Parameter
            switch (m_inchDistanceLevel)
            {
            case 0: inchDistance = axis.INCH_0001_DISTANCE; break;
            case 1: inchDistance = axis.INCH_0010_DISTANCE; break;
            case 2: inchDistance = axis.INCH_0100_DISTANCE; break;
            case 3: inchDistance = axis.INCH_1000_DISTANCE; break;
            default: inchDistance = axis.INCH_0001_DISTANCE; break;
            }

            if (inchDistance <= 0.0) continue;

            // =====================================================
     // INCH Direction Permission
     //
     // allowPositive / allowNegative 前面已經整合：
     //
     // 1. Physical +OT / -OT
     // 2. Software Travel Limit 1 / 2 / 3
     //
     // 所以 INCH 直接沿用相同 Direction Permission。
     // =====================================================

            const bool inchPositive = positiveRising && allowPositive;
            const bool inchNegative = negativeRising && allowNegative;

            if (!inchPositive && !inchNegative)
            {
                continue;
            }

            if (inchPositive && inchNegative)
            {
                continue;
            }


            // =====================================================
            // Mechanical Conversion
            // =====================================================

            if (axis.resolution_PPR <= 0.0 || axis.finalLead <= 0.0)
            {
                continue;
            }

            const double pulsePerUnit = axis.resolution_PPR / axis.finalLead;
            const double inchDistancePulse = inchDistance * pulsePerUnit;

            if (inchDistancePulse <= 0.0)
            {
                continue;
            }


            // =====================================================
            // Calculate Relative INCH Target
            //
            // MotionCore Position:
            //     Pulse
            // =====================================================

            double targetPosition = axis.currentActPos;

            if (inchPositive)
            {
                targetPosition += inchDistancePulse;
            }
            else
            {
                targetPosition -= inchDistancePulse;
            }


            // =====================================================
            // Software Travel Limit Target Pre-Check
            //
            // CoordinateManager 的 Target Check 使用：
            //
            // Linear Axis:
            //     mm
            //
            // Rotary Axis:
            //     degree
            //
            // 所以把 Pulse Target 轉回 Machine Unit。
            //
            // 例如：
            //
            // Current X = 99.999
            // INCH     = +0.010
            // Limit    = +100.000
            //
            // Target   = 100.009
            //
            // => Reject
            //
            // 不讓 MotionCore 收到這一筆 MoveToPosition。
            // =====================================================

            const double targetMCS = targetPosition / pulsePerUnit;
            const bool targetWithinSoftwareLimit = m_nc.GetCoordSys().IsTargetWithinSoftwareTravelLimit(axis, targetMCS);
            const bool positiveSoftwareLimitActive = axis.travelLimit1PositiveActive || axis.travelLimit2PositiveActive || axis.travelLimit3PositiveActive;
            const bool negativeSoftwareLimitActive = axis.travelLimit1NegativeActive || axis.travelLimit2NegativeActive || axis.travelLimit3NegativeActive;
            const bool recoveringFromPositiveLimit = positiveSoftwareLimitActive && inchNegative;
            const bool recoveringFromNegativeLimit = negativeSoftwareLimitActive && inchPositive;

            if (!targetWithinSoftwareLimit && !recoveringFromPositiveLimit && !recoveringFromNegativeLimit)
            {
                continue;
            }


            // =====================================================
            // INCH Speed
            // =====================================================

            double inchVelocity = axis.INCH_JOG_PPS;

            if (inchVelocity <= 0.0) continue;
            if (axis.maxVel_PPS > 0.0 && inchVelocity > axis.maxVel_PPS) inchVelocity = axis.maxVel_PPS;

            double inchAccTime = axis.INCH_acc_time; // INCH Acc / Dec
            double inchDecTime = axis.INCH_dec_time;
            if (inchAccTime < 0.001) inchAccTime = 0.2;
            if (inchDecTime < 0.001) inchDecTime = inchAccTime;

            const bool originalShortestPath = axis.useShortestPath; // Rotary Relative INCH
            if (axis.axisType == AxisType::ROTARY)
            {
                axis.useShortestPath = false;
            }

            m_motion.MoveToPosition(axis, targetPosition, inchVelocity, inchAccTime, inchDecTime); // Execute Original Single Axis INCH
            axis.useShortestPath = originalShortestPath;
            m_jogActive[i] = true;
            continue;
        }
    }
}

// =========================================================
// Home Inputs
// =========================================================
void NCPLCManager::ProcessHomeInputs()
{
    // =====================================================
    // 1. HOME Request Input Decode
    //
    // C15：
    //     HOME_ALL
    //
    // C150~157：
    //     Axis 0~7 單軸 HOME Request
    //
    // 所有輸入都只接受 OFF -> ON Rising Edge。
    //
    // 同一個 Scan 若有多個單軸 HOME Request，
    // 會合併成同一個 axisMask，一次送給 HomingManager。
    //
    // PLC / Panel HOME 預設使用：
    //
    // P0 = SIMULTANEOUS
    //
    // P1 = BY_ORDER
    // 之後由 G81 P1 或未來專用 PLC Mode 再提供。
    // =====================================================

    const bool homeAll =
        m_plc.Get_C(
            NCPLC::C::HOME_ALL);

    const bool homeAllRising =
        homeAll &&
        !m_prevHomeAll;

    m_prevHomeAll =
        homeAll;


    uint8_t axisHomeRequestMask =
        0;

    for (int i = 0;
        i < NCPLC::AXIS_COUNT;
        ++i)
    {
        const bool homeRequest =
            m_plc.Get_C(
                NCPLC::C::AxisPoint(
                    NCPLC::C::HOME_REQUEST_BASE,
                    i));

        const bool homeRequestRising =
            homeRequest &&
            !m_prevHomeRequest[i];

        m_prevHomeRequest[i] =
            homeRequest;


        if (homeRequestRising)
        {
            axisHomeRequestMask |=
                static_cast<uint8_t>(
                    1u << i);
        }
    }


    // =====================================================
    // 2. PLC / Panel HOME Permission Gate
    //
    // 這裡只限制 PLC 面板入口。
    //
    // 未來 G81 是 NC Program Command，
    // 會直接從 GCode Handler 呼叫 HomingManager，
    // 不受這個 Manual Panel Gate 限制。
    //
    // PLC HOME 允許：
    //
    // Mode：
    //     MANUAL
    //     MDI
    //
    // State：
    //     IDLE
    //     READY
    //
    // 並且：
    //
    // Servo Ready
    // No Alarm
    // HomingManager 目前沒有正在執行其他 HOME
    // =====================================================

    const NCOperationMode ncMode =
        m_nc.GetMode();

    const NCState ncState =
        m_nc.GetState();

    const bool operationModeAllowed =
        ncMode == NCOperationMode::MANUAL ||
        ncMode == NCOperationMode::MDI;

    const bool ncStateAllowed =
        ncState == NCState::IDLE ||
        ncState == NCState::READY;

    const bool alarmActive =
        AlarmManager::GetInstance().HasAlarm();

    const bool panelHomeAllowed =
        operationModeAllowed &&
        ncStateAllowed &&
        m_servoReady &&
        !alarmActive &&
        !m_nc.Homing.IsActive();


    // =====================================================
    // 3. Build HomeRequest
    //
    // HOME_ALL Rising：
    //
    //     axisMask = 0
    //
    // HomingManager 會自動選取：
    //
    //     axis.isExist
    //     &&
    //     axis.home.enabled
    //
    // 單軸 / 多軸 Rising：
    //
    //     axisMask = 對應 Bit Mask
    //
    // 若 HOME_ALL 與單軸 Request 同 Scan 發生，
    // HOME_ALL 優先。
    // =====================================================

    if (panelHomeAllowed)
    {
        if (homeAllRising)
        {
            HomeRequest request{};

            request.axisMask =
                0;

            request.sequenceMode =
                HomeSequenceMode::SIMULTANEOUS;

            const bool started = m_nc.Homing.Start(request);
            if (!started)
            {
                const HomeErrorReason reason = m_nc.Homing.GetLastError();
                const int alarmCode = (reason == HomeErrorReason::SERVO_NOT_READY || reason == HomeErrorReason::MOTION_BUSY || reason == HomeErrorReason::SERVO_FAULT || reason == HomeErrorReason::MOTION_FAULT)
                    ? AlarmManager::HOME_MOTION_FAULT : AlarmManager::HOME_INVALID_CONFIG;
                if (!AlarmManager::GetInstance().HasAlarm())
                    AlarmManager::GetInstance().Trigger(alarmCode, 0, m_nc.Homing.GetLastErrorAxis());
                m_motion.EmergencyStopAllAxes();
                m_nc.ChangeState(NCState::ALARM);
            }
        }
        else if (axisHomeRequestMask != 0)
        {
            HomeRequest request{};

            request.axisMask =
                axisHomeRequestMask;

            request.sequenceMode =
                HomeSequenceMode::SIMULTANEOUS;

            const bool started = m_nc.Homing.Start(request);
            if (!started)
            {
                const HomeErrorReason reason = m_nc.Homing.GetLastError();
                const int alarmCode = (reason == HomeErrorReason::SERVO_NOT_READY || reason == HomeErrorReason::MOTION_BUSY || reason == HomeErrorReason::SERVO_FAULT || reason == HomeErrorReason::MOTION_FAULT)
                    ? AlarmManager::HOME_MOTION_FAULT : AlarmManager::HOME_INVALID_CONFIG;
                if (!AlarmManager::GetInstance().HasAlarm())
                    AlarmManager::GetInstance().Trigger(alarmCode, 0, m_nc.Homing.GetLastErrorAxis());
                m_motion.EmergencyStopAllAxes();
                m_nc.ChangeState(NCState::ALARM);
            }
        }
    }


    // =====================================================
    // 4. G81 HOME State Machine Cyclic Process
    //
    // NCPLCManager::Process() 目前由系統 10ms 週期呼叫，
    // 因此 HomingManager 使用 0.010 秒更新 Runtime Timer。
    //
    // HomingManager::Process() 會執行完整 HOME 狀態機；
    // 實機測試前必須先使用低速與單軸參數驗證。
    // =====================================================

    constexpr double HOME_PROCESS_CYCLE_SEC =
        0.010;

    m_nc.Homing.Process(
        HOME_PROCESS_CYCLE_SEC);
}

// =========================================================
// Auxiliary Handshake
// =========================================================
void NCPLCManager::ProcessAuxiliaryHandshake()
{
    // Future: NC -> PLC (S30 AUX_REQUEST), PLC -> NC (C10 AUX_FIN)
    // Data: R100 M, R101 S, R102 T, R103 Valid Mask
}

// =========================================================
// NC -> PLC Status
// =========================================================
void NCPLCManager::SyncNCStateToPLC()
{

}