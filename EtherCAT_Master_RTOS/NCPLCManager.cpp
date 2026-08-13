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

NCPLCManager::NCPLCManager(
    NCManager& nc,
    MotionCore& motion,
    PLCManager& plc)
    :
    m_nc(nc),
    m_motion(motion),
    m_plc(plc)
{
    // MPG DR200 baseline.
    // Avoid a startup jump if DR200 is already non-zero.
    m_lastMPGCount =
        static_cast<int32_t>(
            m_plc.GetMemory(
                "DR",
                NCPLC::DR::MPG_ENCODER_COUNT));
}


// =========================================================
// Main Cyclic Process
// =========================================================

void NCPLCManager::Process()
{
    // =====================================================
    // 1. Safety First
    // =====================================================

    ProcessSafetyInputs();


    // =====================================================
    // 2. Active Safety Input Gate
    //
    // C5 Emergency
    // C140~147 Axis Protection
    //
    // 只要這些 Level Safety Input 還存在，
    // 後面的 Start / Reset / JOG / HOME / AUX
    // 全部禁止。
    //
    // Hard Limit C180/C190 不放在這裡，
    // 因為之後需要允許反方向 Recovery。
    // =====================================================

    bool safetyInputActive =
        m_plc.Get_C(
            NCPLC::C::EMERGENCY_STOP);


    for (int i = 0;
        i < NCPLC::AXIS_COUNT;
        ++i)
    {
        const bool axisProtect =
            m_plc.Get_C(
                NCPLC::C::AxisPoint(
                    NCPLC::C::AXIS_PROTECT_STOP_BASE,
                    i));

        if (axisProtect)
        {
            safetyInputActive = true;
            break;
        }
    }


    if (safetyInputActive)
    {
        // Safety gate skips ProcessManualInputs().
        // Keep DR200 baseline synchronized so clearing safety
        // cannot replay handwheel counts accumulated during the stop.
        m_lastMPGCount =
            static_cast<int32_t>(
                m_plc.GetMemory(
                    "DR",
                    NCPLC::DR::MPG_ENCODER_COUNT));

        SyncNCStateToPLC();
        return;
    }


    // =====================================================
    // 3. PLC -> NC
    // =====================================================

    ProcessGlobalInputs();

    ProcessManualInputs();

    ProcessHomeInputs();

    ProcessAuxiliaryHandshake();


    // =====================================================
    // 4. NC -> PLC
    // =====================================================

    SyncNCStateToPLC();
}


// =========================================================
// Global PLC Inputs
// =========================================================

void NCPLCManager::ProcessGlobalInputs()
{
    // =====================================================
    // C11 - Servo / Machine Ready
    //
    // Level Signal
    // =====================================================

    m_servoReady =
        m_plc.Get_C(
            NCPLC::C::SERVO_READY);


    m_nc.SetExternalReadyInterlock(
        m_servoReady);


    // =====================================================
  // C12 - Cycle Start
  //
  // Rising Edge
  //
  // Cycle Start 必須確認：
  //
  // 1. C11 Servo Ready
  // 2. 沒有任何 Hard Limit
  // 3. 不在任何 Manual Move Mode
  // 4. 沒有 JOG Axis 還在動作
  // =====================================================

    const bool cycleStart =
        m_plc.Get_C(
            NCPLC::C::CYCLE_START);


    if (cycleStart &&
        !m_prevCycleStart)
    {
        // =================================================
        // Hard Limit Check
        // =================================================

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


            if (positiveLimit ||
                negativeLimit)
            {
                hardLimitActive =
                    true;

                break;
            }
        }


        // =================================================
        // Manual Move Mode Check
        //
        // 直接讀 PLC Raw Mode，
        // 不依賴上一 Scan 的 m_manualMoveMode。
        //
        // 因為 ProcessGlobalInputs()
        // 比 ProcessManualInputs() 先執行。
        // =================================================

        const bool continuousJogMode =
            m_plc.Get_C(
                NCPLC::C::JOG_MODE);


        const bool fineJogMode =
            m_plc.Get_C(
                NCPLC::C::FINE_JOG_MODE);


        const bool inchJogMode =
            m_plc.Get_C(
                NCPLC::C::INCH_JOG_MODE);


        const bool mpgMode =
            m_plc.Get_C(
                NCPLC::C::MPG_MODE);


        const bool manualMoveModeActive =
            continuousJogMode ||
            fineJogMode ||
            inchJogMode ||
            mpgMode;


        // =================================================
        // JOG Motion Ownership Check
        //
        // 即使 C19 已經 OFF，
        // 軸可能還在 JOG_dec_time 減速。
        //
        // 此時不能 Start 自動程式。
        // =================================================

        bool jogAxisActive =
            false;


        for (int i = 0;
            i < NCPLC::AXIS_COUNT;
            ++i)
        {
            if (m_jogActive[i])
            {
                jogAxisActive =
                    true;

                break;
            }
        }


        // =================================================
        // Final Cycle Start Permission
        // =================================================

        const bool cycleStartAllowed =
            m_servoReady &&
            !hardLimitActive &&
            !manualMoveModeActive &&
            !jogAxisActive;


        if (cycleStartAllowed)
        {
            m_nc.CycleStart();
        }
    }


    m_prevCycleStart =
        cycleStart;

    // =====================================================
    // C13 - NC Reset
    //
    // Rising Edge
    // =====================================================

    const bool reset =
        m_plc.Get_C(
            NCPLC::C::RESET);


    if (reset &&
        !m_prevReset)
    {
        m_nc.Reset();
    }


    m_prevReset =
        reset;


    // =====================================================
    // C14 - Servo Fault Reset
    //
    // Rising Edge
    //
    // 不使用 ResetAllFaults()，
    // 避免清除 Group Command Queue。
    // =====================================================

    const bool servoFaultReset =
        m_plc.Get_C(
            NCPLC::C::SERVO_FAULT_RESET);


    if (servoFaultReset &&
        !m_prevServoFaultReset)
    {
        // RUN 中禁止 Servo Fault Reset
        if (m_nc.GetState() != NCState::RUN)
        {
            for (int i = 0;
                i < NCPLC::AXIS_COUNT;
                ++i)
            {
                AxisContext& axis =
                    m_motion.GetAxisContext(i);


                if (!axis.isExist)
                {
                    continue;
                }


                const bool needsReset =
                    axis.isFault ||
                    axis.isLagAlarm ||
                    axis.state ==
                    MotionState::MotionState_ERROR ||
                    axis.state ==
                    MotionState::MotionState_ESTOP;


                if (!needsReset)
                {
                    continue;
                }


                m_motion.ResetFault(
                    axis);
            }
        }
    }


    m_prevServoFaultReset =
        servoFaultReset;


    // =====================================================
    // C16 - Single Block
    //
    // Level Signal
    // =====================================================

    const bool singleBlock =
        m_plc.Get_C(
            NCPLC::C::SINGLE_BLOCK);


    m_nc.SetSingleBlockEnabled(
        singleBlock);


    // =====================================================
    // C17 - Optional Stop
    //
    // Level Signal
    // =====================================================

    const bool optionalStop =
        m_plc.Get_C(
            NCPLC::C::OPTIONAL_STOP);


    m_nc.SetOptionalStopEnabled(
        optionalStop);


    // =====================================================
    // C18 - Block Skip
    //
    // Level Signal
    // =====================================================

    const bool blockSkip =
        m_plc.Get_C(
            NCPLC::C::BLOCK_SKIP);


    m_nc.SetBlockSkipEnabled(
        blockSkip);


    // =====================================================
    // C20 - EDM Protection Bypass
    //
    // 只保存狀態。
    //
    // 不能 bypass：
    //
    // Emergency
    // Hard Limit
    // Axis Protection
    // Servo Fault
    // Lag Alarm
    // Safety Chain
    // =====================================================

    m_edmProtectionBypass =
        m_plc.Get_C(
            NCPLC::C::EDM_PROTECTION_BYPASS);


    // =====================================================
    // C4 - Controlled Stop / Feed Hold
    //
    // Rising Edge
    // =====================================================

    const bool controlledStop =
        m_plc.Get_C(
            NCPLC::C::CONTROLLED_STOP);


    if (controlledStop &&
        !m_prevControlledStop)
    {
        m_nc.FeedHold();
    }


    m_prevControlledStop =
        controlledStop;
}


// =========================================================
// Safety Inputs
// =========================================================

void NCPLCManager::ProcessSafetyInputs()
{
    // =====================================================
    // C5 - Emergency Stop
    //
    // Level Sensitive Stop
    //
    // Alarm only Rising Edge
    // =====================================================

    const bool emergencyStop =
        m_plc.Get_C(
            NCPLC::C::EMERGENCY_STOP);


    if (emergencyStop)
    {
        // 每 Scan 維持全軸停止
        m_motion.EmergencyStopAllAxes();


        m_nc.ChangeState(
            NCState::ALARM);


        // Alarm 只建立一次
        if (!m_prevEmergencyStop)
        {
            AlarmManager::GetInstance().Trigger(
                AlarmManager::EMG_STOP);
        }
    }


    m_prevEmergencyStop =
        emergencyStop;


    // =====================================================
    // C140 ~ C147
    //
    // Axis Protection
    //
    // Level Sensitive Stop
    // Rising Edge Alarm
    // =====================================================

    bool axisProtectionActive =
        false;


    for (int i = 0;
        i < NCPLC::AXIS_COUNT;
        ++i)
    {
        const bool axisProtect =
            m_plc.Get_C(
                NCPLC::C::AxisPoint(
                    NCPLC::C::AXIS_PROTECT_STOP_BASE,
                    i));


        if (axisProtect)
        {
            axisProtectionActive =
                true;


            if (!m_prevAxisProtect[i])
            {
                AxisContext& axis =
                    m_motion.GetAxisContext(i);


                const int alarmAxisIndex =
                    axis.isExist
                    ? axis.axisIndex
                    : i;


                AlarmManager::GetInstance().Trigger(
                    AlarmManager::MANUAL_AXIS_PROTECT,
                    0,
                    alarmAxisIndex);
            }
        }


        m_prevAxisProtect[i] =
            axisProtect;
    }


    // -----------------------------------------------------
    // C140~147 持續 ON
    //
    // 就持續維持全機 Emergency Stop。
    // -----------------------------------------------------

    if (axisProtectionActive)
    {
        m_motion.EmergencyStopAllAxes();


        m_nc.ChangeState(
            NCState::ALARM);
    }


    // =====================================================
    // C180 ~ C187
    // Positive Hard Limit (+OT)
    //
    // C190 ~ C197
    // Negative Hard Limit (-OT)
    //
    // Hard Limit 只在 Rising Edge：
    //
    // 1. Trigger Alarm
    // 2. Emergency Stop All Axes
    // 3. NC -> ALARM
    //
    // 持續 ON 不重新 Emergency Stop，
    // 因為之後 Manual Recovery 必須允許
    // 朝離開 Limit 的方向移動。
    // =====================================================

    bool hardLimitTriggered =
        false;


    for (int i = 0;
        i < NCPLC::AXIS_COUNT;
        ++i)
    {
        AxisContext& axis =
            m_motion.GetAxisContext(i);


        const int alarmAxisIndex =
            axis.isExist
            ? axis.axisIndex
            : i;


        // =================================================
        // +OT
        // =================================================

        const bool positiveLimit =
            m_plc.Get_C(
                NCPLC::C::AxisPoint(
                    NCPLC::C::POSITIVE_LIMIT_BASE,
                    i));


        if (positiveLimit &&
            !m_prevPositiveLimit[i])
        {
            AlarmManager::GetInstance().Trigger(
                AlarmManager::HARD_LIMIT,
                0,
                alarmAxisIndex);


            hardLimitTriggered =
                true;
        }


        // =================================================
        // -OT
        // =================================================

        const bool negativeLimit =
            m_plc.Get_C(
                NCPLC::C::AxisPoint(
                    NCPLC::C::NEGATIVE_LIMIT_BASE,
                    i));


        if (negativeLimit &&
            !m_prevNegativeLimit[i])
        {
            AlarmManager::GetInstance().Trigger(
                AlarmManager::HARD_LIMIT,
                0,
                alarmAxisIndex);


            hardLimitTriggered =
                true;
        }


        // =================================================
        // Edge Memory
        // =================================================

        m_prevPositiveLimit[i] =
            positiveLimit;


        m_prevNegativeLimit[i] =
            negativeLimit;
    }


    // =====================================================
    // 本 Scan 有新的 Hard Limit
    //
    // 你原本這段漏掉了。
    // =====================================================

    if (hardLimitTriggered)
    {
        m_motion.EmergencyStopAllAxes();


        m_nc.ChangeState(
            NCState::ALARM);
    }
}


// =========================================================
// Manual Inputs
// =========================================================

void NCPLCManager::ProcessManualInputs()
{
    // =========================================================
    // Manual Move Mode
    //
    // C19 = Normal Continuous JOG
    // C23 = Fine Continuous JOG
    // C24 = INCH JOG
    // C21 = MPG
    //
    // 四種模式必須 One-Hot。
    // =========================================================

    const bool continuousJogMode =
        m_plc.Get_C(
            NCPLC::C::JOG_MODE);


    const bool fineJogMode =
        m_plc.Get_C(
            NCPLC::C::FINE_JOG_MODE);


    const bool inchJogMode =
        m_plc.Get_C(
            NCPLC::C::INCH_JOG_MODE);


    const bool mpgMode =
        m_plc.Get_C(
            NCPLC::C::MPG_MODE);


    const int activeModeCount =
        (continuousJogMode ? 1 : 0) +
        (fineJogMode ? 1 : 0) +
        (inchJogMode ? 1 : 0) +
        (mpgMode ? 1 : 0);


    if (activeModeCount == 1)
    {
        if (continuousJogMode)
        {
            m_manualMoveMode =
                ManualMoveMode::CONTINUOUS_JOG;
        }
        else if (fineJogMode)
        {
            m_manualMoveMode =
                ManualMoveMode::FINE_JOG;
        }
        else if (inchJogMode)
        {
            m_manualMoveMode =
                ManualMoveMode::INCH_JOG;
        }
        else
        {
            m_manualMoveMode =
                ManualMoveMode::MPG;
        }
    }
    else
    {
        m_manualMoveMode =
            ManualMoveMode::NONE;
    }


    // =========================================================
    // Manual Permission Gate
    //
    // 只允許：
    //
    // NC Mode:
    //     MANUAL
    //     MDI
    //
    // NC State:
    //     IDLE
    //     READY
    //
    // 並且：
    //     C11 Servo Ready
    //     No Alarm
    // =========================================================

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
        AlarmManager::GetInstance()
        .HasAlarm();


    if (!operationModeAllowed ||
        !ncStateAllowed ||
        !m_servoReady ||
        alarmActive)
    {
        m_manualMoveMode =
            ManualMoveMode::NONE;
    }


    // =========================================================
    // Legacy Compatibility
    //
    // 等這次測試完成後 Header 再正式清掉。
    // =========================================================

    m_jogMode =
        m_manualMoveMode ==
        ManualMoveMode::CONTINUOUS_JOG ||
        m_manualMoveMode ==
        ManualMoveMode::FINE_JOG;


    // 舊 JOG Rapid 語意已停用。
    // C23 現在正式是 FINE_JOG_MODE。
    m_jogRapid = false;


    // =========================================================
    // R200
    //
    // Continuous JOG Speed %
    //
    // 0 ~ 100
    // =========================================================

    int jogSpeedPercent =
        static_cast<int>(
            m_plc.GetMemory(
                "R",
                NCPLC::R::JOG_SPEED_PERCENT));


    if (jogSpeedPercent < 0)
    {
        jogSpeedPercent = 0;
    }


    if (jogSpeedPercent > 100)
    {
        jogSpeedPercent = 100;
    }


    m_jogSpeedPercent =
        static_cast<double>(
            jogSpeedPercent);


    // =========================================================
    // Fine JOG Speed Select
    //
    // C200 = 0.001 Speed Parameter
    // C201 = 0.010 Speed Parameter
    // C202 = 0.100 Speed Parameter
    // C203 = 1.000 Speed Parameter
    //
    // 必須 One-Hot。
    // =========================================================

    const bool fineJog0001 =
        m_plc.Get_C(
            NCPLC::C::FINE_JOG_SPEED_0001);

    const bool fineJog0010 =
        m_plc.Get_C(
            NCPLC::C::FINE_JOG_SPEED_0010);

    const bool fineJog0100 =
        m_plc.Get_C(
            NCPLC::C::FINE_JOG_SPEED_0100);

    const bool fineJog1000 =
        m_plc.Get_C(
            NCPLC::C::FINE_JOG_SPEED_1000);


    const int fineJogSpeedSelectCount =
        (fineJog0001 ? 1 : 0) +
        (fineJog0010 ? 1 : 0) +
        (fineJog0100 ? 1 : 0) +
        (fineJog1000 ? 1 : 0);


    // =========================================================
    // R201
    //
    // INCH Distance Level
    //
    // 0 = 0.001
    // 1 = 0.010
    // 2 = 0.100
    // 3 = 1.000
    //
    // 單位：
    //
    // Linear = mm
    // Rotary = degree
    // =========================================================

    int inchLevel =
        static_cast<int>(
            m_plc.GetMemory(
                "R",
                NCPLC::R::INCH_DISTANCE_LEVEL));


    if (inchLevel < 0)
    {
        inchLevel = 0;
    }


    if (inchLevel > 3)
    {
        inchLevel = 3;
    }


    // R201 只負責選擇 INCH Distance Level。
    // 真正距離由每一軸 AxisContext 的 INCH 參數決定。
    m_inchDistanceLevel =
        inchLevel;

    // =========================================================
// Manual Frame - Continuous / Fine JOG
//
// CoordinateManager owns:
//
//     Enabled
//     Yaw
//     Pitch
//     Roll
//
// NCPLCManager only creates a Manual XYZ vector and asks
// CoordinateManager to transform it into Machine XYZ.
//
// IMPORTANT:
//
//     Vector rotation is done in physical units,
//     NOT directly in Pulse space.
//
// This keeps the rotation correct even if X/Y/Z have
// different resolution_PPR / finalLead values.
// =========================================================

    const bool manualFrameVelocityMode =
        m_nc.GetCoordSys()
        .IsManualFrameEnabled() &&
        (
            m_manualMoveMode ==
            ManualMoveMode::CONTINUOUS_JOG ||
            m_manualMoveMode ==
            ManualMoveMode::FINE_JOG
            );


    bool manualFrameXYZHasCommand =
        false;

    bool manualFrameXYZValid =
        true;

    bool manualFrameXYZGroupStop =
        false;


    double manualFrameTargetVelocityPPS[3] =
    {
        0.0,
        0.0,
        0.0
    };


    // ---------------------------------------------------------
    // Fine JOG PPS Helper
    //
    // Fine selector remains global One-Hot C200~203,
    // but every physical axis keeps its own PPS parameters.
    // ---------------------------------------------------------

    auto GetFineJogPPS =
        [&](const AxisContext& axis) -> double
    {
        if (fineJogSpeedSelectCount != 1)
        {
            return 0.0;
        }


        if (fineJog0001)
        {
            return axis.FINE_JOG_0001_PPS;
        }


        if (fineJog0010)
        {
            return axis.FINE_JOG_0010_PPS;
        }


        if (fineJog0100)
        {
            return axis.FINE_JOG_0100_PPS;
        }


        if (fineJog1000)
        {
            return axis.FINE_JOG_1000_PPS;
        }


        return 0.0;
    };


    if (manualFrameVelocityMode)
    {
        // =====================================================
        // 1. Build Manual XYZ Direction Vector
        //
        // C120 / C130 still represent the operator's
        // Manual Frame X/Y/Z directions.
        // =====================================================

        double manualVector[8] =
        {
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0
        };


        bool logicalAxisActive[3] =
        {
            false,
            false,
            false
        };


        bool directionConflict =
            false;


        for (int logicalAxis = 0;
            logicalAxis < 3;
            ++logicalAxis)
        {
            const bool positive =
                m_plc.Get_C(
                    NCPLC::C::AxisPoint(
                        NCPLC::C::JOG_POSITIVE_BASE,
                        logicalAxis));


            const bool negative =
                m_plc.Get_C(
                    NCPLC::C::AxisPoint(
                        NCPLC::C::JOG_NEGATIVE_BASE,
                        logicalAxis));


            // + / - 同時 ON
            if (positive &&
                negative)
            {
                directionConflict =
                    true;

                continue;
            }


            if (positive)
            {
                manualVector[logicalAxis] =
                    1.0;

                logicalAxisActive[logicalAxis] =
                    true;
            }
            else if (negative)
            {
                manualVector[logicalAxis] =
                    -1.0;

                logicalAxisActive[logicalAxis] =
                    true;
            }
        }


        if (directionConflict)
        {
            manualFrameXYZValid =
                false;
        }


        // =====================================================
        // 2. Normalize Manual XYZ Vector
        //
        // X+ only:
        //     length = 1
        //
        // X+ + Y+:
        //     [1,1,0]
        //          ↓
        //     [0.7071,0.7071,0]
        //
        // 避免對角 JOG 變成 sqrt(2) 倍速度。
        // =====================================================

        const double manualMagnitude =
            std::sqrt(
                manualVector[0] *
                manualVector[0] +
                manualVector[1] *
                manualVector[1] +
                manualVector[2] *
                manualVector[2]);


        constexpr double VECTOR_EPSILON =
            1.0e-12;


        if (manualMagnitude >
            VECTOR_EPSILON)
        {
            manualFrameXYZHasCommand =
                true;


            manualVector[0] /=
                manualMagnitude;

            manualVector[1] /=
                manualMagnitude;

            manualVector[2] /=
                manualMagnitude;


            // =================================================
            // 3. Manual Frame -> Machine Frame
            // =================================================

            double machineVector[8] =
            {
                0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0
            };


            m_nc.GetCoordSys()
                .TransformManualVector(
                    manualVector,
                    machineVector);


            // 消除 sin/cos 造成的極小浮點殘值。
            for (int machineAxis = 0;
                machineAxis < 3;
                ++machineAxis)
            {
                if (std::abs(
                    machineVector[machineAxis]) <
                    VECTOR_EPSILON)
                {
                    machineVector[machineAxis] =
                        0.0;
                }
            }


            // =================================================
            // 4. Determine Manual Base Speed
            //
            // 先用「操作員按下的 Logical Axis」決定
            // Manual Frame 內的速度。
            //
            // Pulse/sec
            //      ↓
            // Unit/sec
            //
            // Linear XYZ 的 Unit = mm。
            // =================================================

            double baseSpeedUnitPerSec =
                0.0;

            bool baseSpeedInitialized =
                false;


            for (int logicalAxis = 0;
                logicalAxis < 3;
                ++logicalAxis)
            {
                if (!logicalAxisActive[
                    logicalAxis])
                {
                    continue;
                }


                    AxisContext& logicalContext =
                        m_motion.GetAxisContext(
                            logicalAxis);


                    if (!logicalContext.isExist ||
                        logicalContext.resolution_PPR <=
                        0.0 ||
                        logicalContext.finalLead <=
                        0.0)
                    {
                        manualFrameXYZValid =
                            false;

                        break;
                    }


                    const double pulsePerUnit =
                        logicalContext.resolution_PPR /
                        logicalContext.finalLead;


                    double logicalPPS =
                        0.0;


                    if (m_manualMoveMode ==
                        ManualMoveMode::CONTINUOUS_JOG)
                    {
                        logicalPPS =
                            logicalContext.JOG_MAX_PPS *
                            (
                                m_jogSpeedPercent /
                                100.0
                                );
                    }
                    else
                    {
                        logicalPPS =
                            GetFineJogPPS(
                                logicalContext);
                    }


                    if (logicalPPS <= 0.0)
                    {
                        manualFrameXYZValid =
                            false;

                        break;
                    }


                    if (logicalContext.maxVel_PPS >
                        0.0 &&
                        logicalPPS >
                        logicalContext.maxVel_PPS)
                    {
                        logicalPPS =
                            logicalContext.maxVel_PPS;
                    }


                    const double logicalSpeedUnit =
                        logicalPPS /
                        pulsePerUnit;


                    if (!baseSpeedInitialized ||
                        logicalSpeedUnit <
                        baseSpeedUnitPerSec)
                    {
                        baseSpeedUnitPerSec =
                            logicalSpeedUnit;

                        baseSpeedInitialized =
                            true;
                    }
            }


            if (!baseSpeedInitialized ||
                baseSpeedUnitPerSec <= 0.0)
            {
                manualFrameXYZValid =
                    false;
            }


            // =================================================
            // 5. Physical XYZ Validation + Speed Limit
            //
            // 旋轉後必須看「真正 Machine Axis」：
            //
            //     Servo
            //     Fault
            //     +OT / -OT
            //     Axis JOG Speed
            //     Axis Max Velocity
            //
            // 任一參與軸不能動：
            //     整組 XYZ 都不動。
            //
            // 否則會破壞旋轉方向。
            // =================================================

            if (manualFrameXYZValid)
            {
                for (int machineAxis = 0;
                    machineAxis < 3;
                    ++machineAxis)
                {
                    const double component =
                        machineVector[
                            machineAxis];


                    if (std::abs(component) <
                        VECTOR_EPSILON)
                    {
                        continue;
                    }


                    AxisContext& physicalAxis =
                        m_motion.GetAxisContext(
                            machineAxis);


                    const bool axisFault =
                        physicalAxis.isFault ||
                        physicalAxis.isLagAlarm ||
                        physicalAxis.state ==
                        MotionState::
                        MotionState_ERROR ||
                        physicalAxis.state ==
                        MotionState::
                        MotionState_ESTOP;


                    if (!physicalAxis.isExist ||
                        !physicalAxis.isServoOn ||
                        axisFault ||
                        physicalAxis.resolution_PPR <=
                        0.0 ||
                        physicalAxis.finalLead <=
                        0.0)
                    {
                        manualFrameXYZValid =
                            false;

                        break;
                    }


                    // -----------------------------------------
                    // Hard Limit uses MACHINE direction.
                    // -----------------------------------------

                    const bool positiveLimit =
                        m_plc.Get_C(
                            NCPLC::C::AxisPoint(
                                NCPLC::C::
                                POSITIVE_LIMIT_BASE,
                                machineAxis));


                    const bool negativeLimit =
                        m_plc.Get_C(
                            NCPLC::C::AxisPoint(
                                NCPLC::C::
                                NEGATIVE_LIMIT_BASE,
                                machineAxis));


                    if (positiveLimit &&
                        negativeLimit)
                    {
                        manualFrameXYZValid =
                            false;

                        break;
                    }


                    if (component > 0.0 &&
                        positiveLimit)
                    {
                        manualFrameXYZValid =
                            false;

                        break;
                    }


                    if (component < 0.0 &&
                        negativeLimit)
                    {
                        manualFrameXYZValid =
                            false;

                        break;
                    }


                    // -----------------------------------------
                    // Physical Axis Speed Capacity
                    // -----------------------------------------

                    const double pulsePerUnit =
                        physicalAxis.resolution_PPR /
                        physicalAxis.finalLead;


                    double physicalLimitPPS =
                        0.0;


                    if (m_manualMoveMode ==
                        ManualMoveMode::
                        CONTINUOUS_JOG)
                    {
                        physicalLimitPPS =
                            physicalAxis.JOG_MAX_PPS *
                            (
                                m_jogSpeedPercent /
                                100.0
                                );
                    }
                    else
                    {
                        physicalLimitPPS =
                            GetFineJogPPS(
                                physicalAxis);
                    }


                    if (physicalLimitPPS <= 0.0)
                    {
                        manualFrameXYZValid =
                            false;

                        break;
                    }


                    if (physicalAxis.maxVel_PPS >
                        0.0 &&
                        physicalLimitPPS >
                        physicalAxis.maxVel_PPS)
                    {
                        physicalLimitPPS =
                            physicalAxis.maxVel_PPS;
                    }


                    const double physicalLimitUnit =
                        physicalLimitPPS /
                        pulsePerUnit;


                    // Machine component 例如 0.707：
                    //
                    // base * 0.707 <= Axis Limit
                    //
                    // 所以：
                    //
                    // base <= AxisLimit / 0.707
                    const double allowedBaseSpeed =
                        physicalLimitUnit /
                        std::abs(component);


                    if (allowedBaseSpeed <
                        baseSpeedUnitPerSec)
                    {
                        baseSpeedUnitPerSec =
                            allowedBaseSpeed;
                    }
                }
            }


            // =================================================
            // 6. Build Physical Axis Target PPS
            // =================================================

            if (manualFrameXYZValid &&
                baseSpeedUnitPerSec > 0.0)
            {
                for (int machineAxis = 0;
                    machineAxis < 3;
                    ++machineAxis)
                {
                    AxisContext& physicalAxis =
                        m_motion.GetAxisContext(
                            machineAxis);


                    if (physicalAxis.resolution_PPR <=
                        0.0 ||
                        physicalAxis.finalLead <=
                        0.0)
                    {
                        manualFrameXYZValid =
                            false;

                        break;
                    }


                    const double pulsePerUnit =
                        physicalAxis.resolution_PPR /
                        physicalAxis.finalLead;


                    manualFrameTargetVelocityPPS[
                        machineAxis] =
                        baseSpeedUnitPerSec *
                            machineVector[
                                machineAxis] *
                            pulsePerUnit;
                }
            }


            // =================================================
            // 7. Coordinated Direction Change
            //
            // 任一 XYZ 需要反向，整組先停。
            //
            // 不允許：
            //
            // X 已開始反轉，
            // Y 卻還沿舊方向跑。
            // =================================================

            if (manualFrameXYZValid)
            {
                for (int machineAxis = 0;
                    machineAxis < 3;
                    ++machineAxis)
                {
                    AxisContext& physicalAxis =
                        m_motion.GetAxisContext(
                            machineAxis);


                    const double targetVelocity =
                        manualFrameTargetVelocityPPS[
                            machineAxis];


                    const bool needsMotion =
                        std::abs(targetVelocity) >
                        0.01;


                    if (needsMotion)
                    {
                        // Manual Frame 不能搶其他 Motion。
                        if (physicalAxis.state !=
                            MotionState::
                            MotionState_IDLE &&
                            physicalAxis.state !=
                            MotionState::
                            MotionState_VELOCITY)
                        {
                            manualFrameXYZValid =
                                false;

                            break;
                        }


                        if (physicalAxis.state ==
                            MotionState::
                            MotionState_VELOCITY)
                        {
                            // 只有我們持有的 Manual JOG
                            // 才允許更新 Velocity。
                            if (!m_jogActive[
                                machineAxis])
                            {
                                manualFrameXYZValid =
                                    false;

                                break;
                            }


                                const bool reversing =
                                    (
                                        physicalAxis.
                                        currentCmdVel >
                                        0.0 &&
                                        targetVelocity <
                                        0.0
                                        )
                                    ||
                                    (
                                        physicalAxis.
                                        currentCmdVel <
                                        0.0 &&
                                        targetVelocity >
                                        0.0
                                        );


                                if (reversing)
                                {
                                    manualFrameXYZGroupStop =
                                        true;
                                }
                        }
                    }
                    else
                    {
                        // 原本是旋轉向量的一部分，
                        // 新向量現在變成 0：
                        // 也先整組停再重建方向。
                        if (m_jogActive[
                            machineAxis] &&
                            physicalAxis.state ==
                                MotionState::
                                MotionState_VELOCITY)
                        {
                            manualFrameXYZGroupStop =
                                true;
                        }
                    }
                }
            }
        }
    }


    // =========================================================
    // MPG Input Decode
    //
    // C210~217 = Axis Select (One-Hot)
    // C220~223 = Multiplier  (One-Hot)
    // DR200     = Accumulated Handwheel Count
    //
    // DR200 is consumed as a per-scan delta.
    // m_lastMPGCount is updated every scan, even when the
    // selector is invalid, so invalid periods never accumulate
    // a delayed movement command.
    // =========================================================

    int mpgSelectedAxis = -1;
    int mpgAxisSelectCount = 0;

    for (int i = 0;
        i < NCPLC::AXIS_COUNT;
        ++i)
    {
        const bool selected =
            m_plc.Get_C(
                NCPLC::C::AxisPoint(
                    NCPLC::C::MPG_AXIS_SELECT_BASE,
                    i));

        if (selected)
        {
            ++mpgAxisSelectCount;
            mpgSelectedAxis = i;
        }
    }


    const bool mpgX1 =
        m_plc.Get_C(
            NCPLC::C::MPG_MULTIPLIER_X1);

    const bool mpgX10 =
        m_plc.Get_C(
            NCPLC::C::MPG_MULTIPLIER_X10);

    const bool mpgX100 =
        m_plc.Get_C(
            NCPLC::C::MPG_MULTIPLIER_X100);

    const bool mpgX1000 =
        m_plc.Get_C(
            NCPLC::C::MPG_MULTIPLIER_X1000);


    const int mpgMultiplierSelectCount =
        (mpgX1 ? 1 : 0) +
        (mpgX10 ? 1 : 0) +
        (mpgX100 ? 1 : 0) +
        (mpgX1000 ? 1 : 0);


    double mpgMultiplier = 1.0;

    if (mpgX10)
    {
        mpgMultiplier = 10.0;
    }
    else if (mpgX100)
    {
        mpgMultiplier = 100.0;
    }
    else if (mpgX1000)
    {
        mpgMultiplier = 1000.0;
    }


    const int32_t currentMPGCount =
        static_cast<int32_t>(
            m_plc.GetMemory(
                "DR",
                NCPLC::DR::MPG_ENCODER_COUNT));


    // ---------------------------------------------------------
    // Signed 32-bit wrap-safe delta
    //
    // Example:
    // INT32_MAX -> INT32_MIN is +1 count, not -4294967295.
    // ---------------------------------------------------------

    int64_t mpgDelta64 =
        static_cast<int64_t>(currentMPGCount) -
        static_cast<int64_t>(m_lastMPGCount);

    if (mpgDelta64 > 2147483647LL)
    {
        mpgDelta64 -= 4294967296LL;
    }
    else if (mpgDelta64 < -2147483648LL)
    {
        mpgDelta64 += 4294967296LL;
    }


    int32_t mpgDeltaCount =
        static_cast<int32_t>(mpgDelta64);


    // Always consume the current counter value.
    m_lastMPGCount =
        currentMPGCount;


    const bool mpgCommandValid =
        m_manualMoveMode ==
        ManualMoveMode::MPG &&
        mpgAxisSelectCount == 1 &&
        mpgMultiplierSelectCount == 1;


    if (!mpgCommandValid)
    {
        // Invalid selection must not queue movement for later.
        mpgDeltaCount = 0;
    }


    // =========================================================
    // Axis Manual Motion
    // =========================================================

    for (int i = 0;
        i < NCPLC::AXIS_COUNT;
        ++i)
    {
        AxisContext& axis =
            m_motion.GetAxisContext(i);


        // =====================================================
        // Raw Direction
        //
        // + : C120 ~ C127
        // - : C130 ~ C137
        // =====================================================

        const bool rawPositive =
            m_plc.Get_C(
                NCPLC::C::AxisPoint(
                    NCPLC::C::JOG_POSITIVE_BASE,
                    i));


        const bool rawNegative =
            m_plc.Get_C(
                NCPLC::C::AxisPoint(
                    NCPLC::C::JOG_NEGATIVE_BASE,
                    i));


        // =====================================================
        // Rising Edge
        //
        // INCH 使用。
        // =====================================================

        const bool positiveRising =
            rawPositive &&
            !m_prevJogPositive[i];


        const bool negativeRising =
            rawNegative &&
            !m_prevJogNegative[i];


        // =====================================================
        // Edge Memory
        //
        // 注意：
        // 必須每 Scan 都更新。
        // 即使 Manual Gate 不允許，也不能留下假 Rising Edge。
        // =====================================================

        m_prevJogPositive[i] =
            rawPositive;


        m_prevJogNegative[i] =
            rawNegative;


        // =====================================================
        // Default Valid Direction
        // =====================================================

        m_jogPositive[i] =
            false;


        m_jogNegative[i] =
            false;


        // =====================================================
        // Manual Motion Ownership Release
        //
        // Continuous：
        //     Release -> STOPPING -> IDLE
        //
        // INCH：
        //     MOVING -> IDLE
        //
        // 真正 IDLE 後才釋放 ownership。
        // =====================================================

        if (m_jogActive[i] &&
            axis.state ==
            MotionState::MotionState_IDLE)
        {
            m_jogActive[i] =
                false;
        }


        // =====================================================
        // Axis Existence
        // =====================================================

        if (!axis.isExist)
        {
            m_jogActive[i] =
                false;

            continue;
        }


        // =====================================================
        // Axis Fault Gate
        // =====================================================

        const bool axisFault =
            axis.isFault ||
            axis.isLagAlarm ||
            axis.state ==
            MotionState::MotionState_ERROR ||
            axis.state ==
            MotionState::MotionState_ESTOP;


        if (axisFault)
        {
            m_jogActive[i] =
                false;

            continue;
        }


        // =====================================================
        // Servo On Gate
        //
        // C11 是 Machine Ready。
        //
        // axis.isServoOn 才是真正該軸已激磁。
        // =====================================================

        if (!axis.isServoOn)
        {
            if (m_jogActive[i])
            {
                if (axis.state ==
                    MotionState::MotionState_VELOCITY)
                {
                    m_motion.StopMove(
                        axis,
                        axis.JOG_dec_time);
                }
                else if (axis.state ==
                    MotionState::MotionState_MOVING)
                {
                    m_motion.StopMove(
                        axis,
                        axis.INCH_dec_time);
                }
                else if (axis.state ==
                    MotionState::MotionState_MPG)
                {
                    m_motion.StopMove(
                        axis,
                        axis.JOG_dec_time);
                }
            }

            continue;
        }


        // =====================================================
        // Mode Change Stop
        //
        // Continuous JOG 還沒停完，
        // 卻切到其他 Manual Mode：
        //     先停止。
        // =====================================================

        const bool velocityJogModeActive =
            m_manualMoveMode ==
            ManualMoveMode::CONTINUOUS_JOG ||
            m_manualMoveMode ==
            ManualMoveMode::FINE_JOG;


        if (m_jogActive[i] &&
            axis.state ==
            MotionState::MotionState_VELOCITY &&
            !velocityJogModeActive)
        {
            m_motion.StopMove(
                axis,
                axis.JOG_dec_time);

            continue;
        }


        // =====================================================
        // INCH 還在 P2P，
        // 卻切離 INCH Mode：
        //
        // 做 Controlled Stop。
        // =====================================================

        if (m_jogActive[i] &&
            axis.state ==
            MotionState::MotionState_MOVING &&
            m_manualMoveMode !=
            ManualMoveMode::INCH_JOG)
        {
            m_motion.StopMove(
                axis,
                axis.INCH_dec_time);

            continue;
        }


        // =====================================================
        // MPG Mode Change Stop
        //
        // MotionState_MPG deliberately stays active even when
        // the axis has reached the current handwheel target.
        // Leaving MPG therefore must explicitly return it through
        // STOPPING -> IDLE before another manual mode can own it.
        // =====================================================

        if (m_jogActive[i] &&
            axis.state ==
            MotionState::MotionState_MPG &&
            m_manualMoveMode !=
            ManualMoveMode::MPG)
        {
            m_motion.StopMove(
                axis,
                axis.JOG_dec_time);

            continue;
        }


        // =====================================================
        // Manual Mode NONE
        // =====================================================

        if (m_manualMoveMode ==
            ManualMoveMode::NONE)
        {
            continue;
        }


        // =====================================================
        // MPG / Manual Pulse Generator
        //
        // C21       = MPG Mode
        // C210~217  = Axis Select
        // C220~223  = x1 / x10 / x100 / x1000
        // DR200      = Accumulated Handwheel Count
        //
        // MPG does NOT use C120 / C130.
        // Direction comes from the sign of DR200 delta.
        // =====================================================

        if (m_manualMoveMode ==
            ManualMoveMode::MPG)
        {
            // =================================================
            // Manual Frame MPG
            //
            // Manual Frame ON + MPG selects X/Y/Z:
            //
            // DR200 Delta
            //      ↓
            // Logical Manual Axis Distance
            //      ↓
            // TransformManualVector()
            //      ↓
            // Physical Machine XYZ Delta
            //      ↓
            // MPGMove() for participating XYZ axes
            //
            // Manual Frame OFF:
            //      保持原本 MPG 行為。
            //
            // A/B/C/U/V:
            //      保持原本 MPG 行為。
            // =================================================

            const bool manualFrameMPG =
                m_nc.GetCoordSys()
                .IsManualFrameEnabled() &&
                mpgCommandValid &&
                mpgSelectedAxis >= 0 &&
                mpgSelectedAxis < 3;


            if (manualFrameMPG)
            {
                // =============================================
                // Manual Frame MPG 只處理 XYZ。
                //
                // 其他實體軸如果之前還停留在 MPG，
                // 先 Controlled Stop。
                // =============================================

                if (i >= 3)
                {
                    if (axis.state ==
                        MotionState::MotionState_MPG)
                    {
                        m_motion.StopMove(
                            axis,
                            axis.JOG_dec_time);

                        m_jogActive[i] =
                            true;
                    }

                    continue;
                }


                // =============================================
                // 整組 XYZ 只執行一次。
                //
                // 以目前 MPG Selected Logical Axis
                // 當作 Group Executor。
                // =============================================

                if (i != mpgSelectedAxis)
                {
                    continue;
                }


                // =============================================
                // Helper:
                // Stop all XYZ MPG axes
                // =============================================

                auto StopManualFrameMPGXYZ =
                    [&]()
                {
                    for (int machineAxis = 0;
                        machineAxis < 3;
                        ++machineAxis)
                    {
                        AxisContext& physicalAxis =
                            m_motion.GetAxisContext(
                                machineAxis);


                        if (physicalAxis.state ==
                            MotionState::
                            MotionState_MPG)
                        {
                            m_motion.StopMove(
                                physicalAxis,
                                physicalAxis.
                                JOG_dec_time);

                            m_jogActive[
                                machineAxis] =
                                true;
                        }
                    }
                };


                // =============================================
                // No New Count
                //
                // 不呼叫 MPGMove。
                //
                // 已經是 MPG State 的軸會繼續追之前
                // finalTargetPos。
                // =============================================

                if (mpgDeltaCount == 0)
                {
                    continue;
                }


                // =============================================
                // Logical Manual Axis Parameters
                // =============================================

                AxisContext& logicalAxis =
                    axis;


                if (logicalAxis.MPG_BASE_DISTANCE <=
                    0.0 ||
                    logicalAxis.MPG_MAX_PPS <=
                    0.0 ||
                    logicalAxis.resolution_PPR <=
                    0.0 ||
                    logicalAxis.finalLead <=
                    0.0)
                {
                    StopManualFrameMPGXYZ();

                    continue;
                }


                const double logicalPulsePerUnit =
                    logicalAxis.resolution_PPR /
                    logicalAxis.finalLead;


                // =============================================
                // Manual Unit Direction
                //
                // Selected X:
                //     [1,0,0]
                //
                // Selected Y:
                //     [0,1,0]
                //
                // Selected Z:
                //     [0,0,1]
                // =============================================

                double manualVector[8] =
                {
                    0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0
                };


                manualVector[
                    mpgSelectedAxis] =
                    1.0;


                    // =============================================
                    // Manual Frame -> Machine Frame
                    // =============================================

                    double machineDirection[8] =
                    {
                        0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0
                    };


                    m_nc.GetCoordSys()
                        .TransformManualVector(
                            manualVector,
                            machineDirection);


                    constexpr double
                        MPG_VECTOR_EPSILON =
                        1.0e-10;


                    // 清掉旋轉矩陣的微小浮點殘值
                    for (int machineAxis = 0;
                        machineAxis < 3;
                        ++machineAxis)
                    {
                        if (std::abs(
                            machineDirection[
                                machineAxis]) <
                            MPG_VECTOR_EPSILON)
                        {
                            machineDirection[
                                machineAxis] =
                                0.0;
                        }
                    }


                    // =============================================
                    // DR200 Count -> Logical Distance
                    //
                    // 例如：
                    //
                    // BASE = 0.001 mm
                    // ×10
                    // Delta Count = +3
                    //
                    // Distance = +0.030 mm
                    // =============================================

                    const double deltaDistance =
                        static_cast<double>(
                            mpgDeltaCount) *
                        logicalAxis.MPG_BASE_DISTANCE *
                        mpgMultiplier;


                    if (std::abs(deltaDistance) <=
                        MPG_VECTOR_EPSILON)
                    {
                        continue;
                    }


                    // =============================================
                    // Logical MPG Max Path Speed
                    //
                    // Pulse/sec
                    //      ↓
                    // Unit/sec
                    //
                    // XYZ Unit = mm
                    // =============================================

                    double logicalMaxPPS =
                        logicalAxis.MPG_MAX_PPS;


                    if (logicalAxis.maxVel_PPS >
                        0.0 &&
                        logicalMaxPPS >
                        logicalAxis.maxVel_PPS)
                    {
                        logicalMaxPPS =
                            logicalAxis.maxVel_PPS;
                    }


                    double pathMaxUnitPerSec =
                        logicalMaxPPS /
                        logicalPulsePerUnit;


                    if (pathMaxUnitPerSec <= 0.0)
                    {
                        StopManualFrameMPGXYZ();

                        continue;
                    }


                    // =============================================
                    // Group Acc / Dec
                    //
                    // 所有 XYZ 使用同一組時間，
                    // 才能維持速度向量比例。
                    // =============================================

                    double groupAccTime =
                        logicalAxis.JOG_acc_time;


                    double groupDecTime =
                        logicalAxis.JOG_dec_time;


                    if (groupAccTime < 0.001)
                    {
                        groupAccTime =
                            0.2;
                    }


                    if (groupDecTime < 0.001)
                    {
                        groupDecTime =
                            groupAccTime;
                    }


                    bool groupValid =
                        true;


                    // =============================================
                    // Validate Physical Machine XYZ
                    //
                    // 這裡非常重要：
                    //
                    // Limit 要依照「旋轉後實際 Machine Axis」
                    // 的方向判斷。
                    //
                    // 不能再看原始 Selected Axis。
                    // =============================================

                    for (int machineAxis = 0;
                        machineAxis < 3;
                        ++machineAxis)
                    {
                        const double component =
                            machineDirection[
                                machineAxis];


                        if (std::abs(component) <=
                            MPG_VECTOR_EPSILON)
                        {
                            continue;
                        }


                        AxisContext& physicalAxis =
                            m_motion.GetAxisContext(
                                machineAxis);


                        const bool physicalFault =
                            physicalAxis.isFault ||
                            physicalAxis.isLagAlarm ||
                            physicalAxis.state ==
                            MotionState::
                            MotionState_ERROR ||
                            physicalAxis.state ==
                            MotionState::
                            MotionState_ESTOP;


                        // -----------------------------------------
                        // Axis / Servo / Ownership
                        // -----------------------------------------

                        if (!physicalAxis.isExist ||
                            !physicalAxis.isServoOn ||
                            physicalFault ||
                            physicalAxis.resolution_PPR <=
                            0.0 ||
                            physicalAxis.finalLead <=
                            0.0 ||
                            physicalAxis.MPG_MAX_PPS <=
                            0.0)
                        {
                            groupValid =
                                false;

                            break;
                        }


                        // MPG 可以：
                        //
                        // IDLE -> MPG
                        // MPG  -> update target
                        //
                        // 不可搶其他 Motion。
                        if (physicalAxis.state !=
                            MotionState::
                            MotionState_IDLE &&
                            physicalAxis.state !=
                            MotionState::
                            MotionState_MPG)
                        {
                            groupValid =
                                false;

                            break;
                        }


                        // =========================================
                        // Rotated Physical Delta
                        // =========================================

                        const double physicalDeltaUnit =
                            deltaDistance *
                            component;


                        // =========================================
                        // Hard Limit
                        // =========================================

                        const bool positiveLimit =
                            m_plc.Get_C(
                                NCPLC::C::AxisPoint(
                                    NCPLC::C::
                                    POSITIVE_LIMIT_BASE,
                                    machineAxis));


                        const bool negativeLimit =
                            m_plc.Get_C(
                                NCPLC::C::AxisPoint(
                                    NCPLC::C::
                                    NEGATIVE_LIMIT_BASE,
                                    machineAxis));


                        if (positiveLimit &&
                            negativeLimit)
                        {
                            groupValid =
                                false;

                            break;
                        }


                        if (physicalDeltaUnit > 0.0 &&
                            positiveLimit)
                        {
                            groupValid =
                                false;

                            break;
                        }


                        if (physicalDeltaUnit < 0.0 &&
                            negativeLimit)
                        {
                            groupValid =
                                false;

                            break;
                        }


                        // =========================================
                        // Physical Axis MPG Speed Capacity
                        // =========================================

                        const double pulsePerUnit =
                            physicalAxis.resolution_PPR /
                            physicalAxis.finalLead;


                        double physicalMaxPPS =
                            physicalAxis.MPG_MAX_PPS;


                        if (physicalAxis.maxVel_PPS >
                            0.0 &&
                            physicalMaxPPS >
                            physicalAxis.maxVel_PPS)
                        {
                            physicalMaxPPS =
                                physicalAxis.maxVel_PPS;
                        }


                        const double
                            physicalMaxUnitPerSec =
                            physicalMaxPPS /
                            pulsePerUnit;


                        // pathSpeed * |component|
                        // <= physical axis max
                        const double
                            allowedPathSpeed =
                            physicalMaxUnitPerSec /
                            std::abs(component);


                        if (allowedPathSpeed <
                            pathMaxUnitPerSec)
                        {
                            pathMaxUnitPerSec =
                                allowedPathSpeed;
                        }


                        // =========================================
                        // Group Acc / Dec
                        //
                        // 取參與軸較慢的時間。
                        // =========================================

                        double physicalAccTime =
                            physicalAxis.JOG_acc_time;


                        double physicalDecTime =
                            physicalAxis.JOG_dec_time;


                        if (physicalAccTime < 0.001)
                        {
                            physicalAccTime =
                                0.2;
                        }


                        if (physicalDecTime < 0.001)
                        {
                            physicalDecTime =
                                physicalAccTime;
                        }


                        if (physicalAccTime >
                            groupAccTime)
                        {
                            groupAccTime =
                                physicalAccTime;
                        }


                        if (physicalDecTime >
                            groupDecTime)
                        {
                            groupDecTime =
                                physicalDecTime;
                        }
                    }


                    if (!groupValid ||
                        pathMaxUnitPerSec <= 0.0)
                    {
                        StopManualFrameMPGXYZ();

                        continue;
                    }


                    // =============================================
                    // Execute Physical XYZ MPG
                    //
                    // 每一個 Physical Axis 都有自己的：
                    //
                    // finalTargetPos
                    //
                    // 所以 DR200 轉得很快時：
                    //
                    // 新 Delta 會累積在舊 finalTargetPos 上。
                    //
                    // 不會因為 Axis 還沒追到而掉 Count。
                    // =============================================

                    for (int machineAxis = 0;
                        machineAxis < 3;
                        ++machineAxis)
                    {
                        const double component =
                            machineDirection[
                                machineAxis];


                        AxisContext& physicalAxis =
                            m_motion.GetAxisContext(
                                machineAxis);


                        // -----------------------------------------
                        // 此軸不參與目前旋轉方向
                        // -----------------------------------------

                        if (std::abs(component) <=
                            MPG_VECTOR_EPSILON)
                        {
                            if (physicalAxis.state ==
                                MotionState::
                                MotionState_MPG)
                            {
                                m_motion.StopMove(
                                    physicalAxis,
                                    physicalAxis.
                                    JOG_dec_time);

                                m_jogActive[
                                    machineAxis] =
                                    true;
                            }

                            continue;
                        }


                        const double pulsePerUnit =
                            physicalAxis.resolution_PPR /
                            physicalAxis.finalLead;


                        const double physicalDeltaUnit =
                            deltaDistance *
                            component;


                        const double deltaPulse =
                            physicalDeltaUnit *
                            pulsePerUnit;


                        if (std::abs(deltaPulse) <=
                            MPG_VECTOR_EPSILON)
                        {
                            continue;
                        }


                        // =========================================
                        // Dynamic Target Accumulation
                        //
                        // 這是原本 MPG 最重要的行為，
                        // 必須保留下來。
                        // =========================================

                        const double baseTarget =
                            physicalAxis.state ==
                            MotionState::
                            MotionState_MPG
                            ? physicalAxis.
                            finalTargetPos
                            : physicalAxis.
                            currentCmdPos;


                        const double targetPosition =
                            baseTarget +
                            deltaPulse;


                        // =========================================
                        // Physical Axis Max PPS
                        //
                        // Path Speed × Direction Component
                        // =========================================

                        double physicalMPGMaxPPS =
                            pathMaxUnitPerSec *
                            std::abs(component) *
                            pulsePerUnit;


                        if (physicalAxis.MPG_MAX_PPS >
                            0.0 &&
                            physicalMPGMaxPPS >
                            physicalAxis.MPG_MAX_PPS)
                        {
                            physicalMPGMaxPPS =
                                physicalAxis.MPG_MAX_PPS;
                        }


                        if (physicalAxis.maxVel_PPS >
                            0.0 &&
                            physicalMPGMaxPPS >
                            physicalAxis.maxVel_PPS)
                        {
                            physicalMPGMaxPPS =
                                physicalAxis.maxVel_PPS;
                        }


                        if (physicalMPGMaxPPS <= 0.0)
                        {
                            continue;
                        }


                        // =========================================
                        // MPG Motion
                        // =========================================

                        m_motion.MPGMove(
                            physicalAxis,
                            targetPosition,
                            physicalMPGMaxPPS,
                            groupAccTime,
                            groupDecTime);


                        if (physicalAxis.state ==
                            MotionState::
                            MotionState_MPG)
                        {
                            m_jogActive[
                                machineAxis] =
                                true;
                        }
                    }


                    continue;
            }


            // =================================================
            // Original MPG
            //
            // 以下情況全部走原本行為：
            //
            // Manual Frame OFF
            //
            // 或 MPG 選擇：
            //
            // A / B / C / U / V
            // =================================================


            // -------------------------------------------------
            // Invalid selector or this is not selected axis.
            // -------------------------------------------------

            if (!mpgCommandValid ||
                i != mpgSelectedAxis)
            {
                if (axis.state ==
                    MotionState::MotionState_MPG)
                {
                    m_motion.StopMove(
                        axis,
                        axis.JOG_dec_time);

                    m_jogActive[i] =
                        true;
                }

                continue;
            }


            // -------------------------------------------------
            // MPG cannot steal another Motion owner.
            // -------------------------------------------------

            if (axis.state !=
                MotionState::MotionState_IDLE &&
                axis.state !=
                MotionState::MotionState_MPG)
            {
                continue;
            }


            // -------------------------------------------------
            // No new handwheel count
            // -------------------------------------------------

            if (mpgDeltaCount == 0)
            {
                continue;
            }


            // -------------------------------------------------
            // Hard Limit Direction Gate
            // -------------------------------------------------

            const bool positiveLimit =
                m_plc.Get_C(
                    NCPLC::C::AxisPoint(
                        NCPLC::C::
                        POSITIVE_LIMIT_BASE,
                        i));


            const bool negativeLimit =
                m_plc.Get_C(
                    NCPLC::C::AxisPoint(
                        NCPLC::C::
                        NEGATIVE_LIMIT_BASE,
                        i));


            const bool blockedPositive =
                mpgDeltaCount > 0 &&
                positiveLimit;


            const bool blockedNegative =
                mpgDeltaCount < 0 &&
                negativeLimit;


            if ((positiveLimit &&
                negativeLimit) ||
                blockedPositive ||
                blockedNegative)
            {
                if (axis.state ==
                    MotionState::
                    MotionState_MPG)
                {
                    m_motion.StopMove(
                        axis,
                        axis.JOG_dec_time);

                    m_jogActive[i] =
                        true;
                }

                continue;
            }


            // -------------------------------------------------
            // Axis Parameters
            // -------------------------------------------------

            if (axis.MPG_BASE_DISTANCE <= 0.0 ||
                axis.MPG_MAX_PPS <= 0.0 ||
                axis.resolution_PPR <= 0.0 ||
                axis.finalLead <= 0.0)
            {
                continue;
            }


            const double pulsePerUnit =
                axis.resolution_PPR /
                axis.finalLead;


            const double deltaDistance =
                static_cast<double>(
                    mpgDeltaCount) *
                axis.MPG_BASE_DISTANCE *
                mpgMultiplier;


            const double deltaPulse =
                deltaDistance *
                pulsePerUnit;


            if (deltaPulse == 0.0)
            {
                continue;
            }


            // -------------------------------------------------
            // Dynamic Absolute Target
            //
            // 保留目前已驗證成功的 MPG 累積方式。
            // -------------------------------------------------

            const double baseTarget =
                axis.state ==
                MotionState::
                MotionState_MPG
                ? axis.finalTargetPos
                : axis.currentCmdPos;


            const double targetPosition =
                baseTarget +
                deltaPulse;


            // -------------------------------------------------
            // Original MPG MotionCore
            // -------------------------------------------------

            m_motion.MPGMove(
                axis,
                targetPosition,
                axis.MPG_MAX_PPS,
                axis.JOG_acc_time,
                axis.JOG_dec_time);


            if (axis.state ==
                MotionState::MotionState_MPG)
            {
                m_jogActive[i] =
                    true;
            }


            continue;
        }

        // =====================================================
// Manual Frame - Normal / Fine JOG XYZ
//
// XYZ 已經在 Axis Loop 外：
//
// Manual Vector
//      ↓
// CoordinateManager
//      ↓
// Machine Vector
//      ↓
// Physical PPS
//
// 所以 XYZ 不再走下面舊的單軸 JOG 邏輯。
//
// A/B/C/U/V 繼續走原本邏輯。
// =====================================================

        if (manualFrameVelocityMode &&
            i < 3)
        {
            // -------------------------------------------------
            // No command / Invalid / Coordinated Stop
            // -------------------------------------------------

            if (!manualFrameXYZHasCommand ||
                !manualFrameXYZValid ||
                manualFrameXYZGroupStop)
            {
                if (m_jogActive[i] &&
                    axis.state ==
                    MotionState::
                    MotionState_VELOCITY)
                {
                    m_motion.StopMove(
                        axis,
                        axis.JOG_dec_time);
                }

                continue;
            }


            const double targetVelocity =
                manualFrameTargetVelocityPPS[i];


            // -------------------------------------------------
            // This physical axis does not participate
            // in the rotated vector.
            // -------------------------------------------------

            if (std::abs(targetVelocity) <=
                0.01)
            {
                if (m_jogActive[i] &&
                    axis.state ==
                    MotionState::
                    MotionState_VELOCITY)
                {
                    m_motion.StopMove(
                        axis,
                        axis.JOG_dec_time);
                }

                continue;
            }


            // -------------------------------------------------
            // STOPPING must finish first.
            // -------------------------------------------------

            if (axis.state ==
                MotionState::
                MotionState_STOPPING)
            {
                continue;
            }


            // -------------------------------------------------
            // Do not steal P2P / Interpolation / MPG.
            // -------------------------------------------------

            if (axis.state !=
                MotionState::
                MotionState_IDLE &&
                axis.state !=
                MotionState::
                MotionState_VELOCITY)
            {
                continue;
            }


            // -------------------------------------------------
            // Execute physical machine-axis velocity.
            //
            // Normal + Fine share JOG acc/dec parameters.
            // -------------------------------------------------

            m_motion.VelocityMove(
                axis,
                targetVelocity,
                axis.JOG_acc_time);


            m_jogActive[i] =
                true;


            continue;
        }

        // =====================================================
        // Direction Conflict
        //
        // + / - 同時 ON：
        //     不允許移動。
        //
        // Continuous 正在移動則 Controlled Stop。
        // =====================================================

        if (rawPositive &&
            rawNegative)
        {
            if (m_jogActive[i] &&
                axis.state ==
                MotionState::MotionState_VELOCITY)
            {
                m_motion.StopMove(
                    axis,
                    axis.JOG_dec_time);
            }

            continue;
        }


        // =====================================================
        // Hard Limit
        // =====================================================

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


        // =====================================================
        // Direction Permission
        //
        // +OT：
        //     禁止 +
        //     允許 -
        //
        // -OT：
        //     禁止 -
        //     允許 +
        //
        // 兩個都 ON：
        //     完全禁止。
        // =====================================================

        const bool allowPositive =
            rawPositive &&
            !positiveLimit;


        const bool allowNegative =
            rawNegative &&
            !negativeLimit;


        if (positiveLimit &&
            negativeLimit)
        {
            if (m_jogActive[i] &&
                axis.state ==
                MotionState::MotionState_VELOCITY)
            {
                m_motion.StopMove(
                    axis,
                    axis.JOG_dec_time);
            }

            continue;
        }


        m_jogPositive[i] =
            allowPositive;


        m_jogNegative[i] =
            allowNegative;


        // =====================================================
        // 1. CONTINUOUS JOG
        //
        // Speed:
        //
        // axis.JOG_MAX_PPS × R200 %
        // =====================================================

        if (m_manualMoveMode ==
            ManualMoveMode::CONTINUOUS_JOG)
        {
            // -------------------------------------------------
            // 沒有方向
            // -------------------------------------------------

            if (!allowPositive &&
                !allowNegative)
            {
                if (m_jogActive[i] &&
                    axis.state ==
                    MotionState::MotionState_VELOCITY)
                {
                    m_motion.StopMove(
                        axis,
                        axis.JOG_dec_time);
                }

                continue;
            }


            // -------------------------------------------------
            // 基本 JOG 速度
            // -------------------------------------------------

            if (axis.JOG_MAX_PPS <= 0.0)
            {
                continue;
            }


            double jogVelocity =
                axis.JOG_MAX_PPS *
                (
                    m_jogSpeedPercent /
                    100.0
                    );


            // -------------------------------------------------
            // Clamp Axis Maximum
            // -------------------------------------------------

            if (axis.maxVel_PPS > 0.0 &&
                jogVelocity >
                axis.maxVel_PPS)
            {
                jogVelocity =
                    axis.maxVel_PPS;
            }


            // R200 = 0
            if (jogVelocity <= 0.0)
            {
                if (m_jogActive[i] &&
                    axis.state ==
                    MotionState::MotionState_VELOCITY)
                {
                    m_motion.StopMove(
                        axis,
                        axis.JOG_dec_time);
                }

                continue;
            }


            // -------------------------------------------------
            // Direction
            // -------------------------------------------------

            double targetVelocity =
                0.0;


            if (allowPositive)
            {
                targetVelocity =
                    jogVelocity;
            }
            else if (allowNegative)
            {
                targetVelocity =
                    -jogVelocity;
            }


            // =================================================
            // STOPPING
            //
            // 一定等真正停到 IDLE。
            // 不允許途中直接反轉。
            // =================================================

            if (axis.state ==
                MotionState::MotionState_STOPPING)
            {
                continue;
            }


            // =================================================
            // P2P / Interpolation 正在使用 Axis
            //
            // Manual JOG 不搶 ownership。
            // =================================================

            if (axis.state ==
                MotionState::MotionState_MOVING ||
                axis.state ==
                MotionState::MotionState_INTERPOLATING)
            {
                continue;
            }


            // =================================================
            // Direction Reversal
            //
            // 目前速度與要求方向相反：
            //
            // 先 StopMove
            // -> STOPPING
            // -> IDLE
            // -> 下一 Scan 才反向。
            // =================================================

            if (axis.state ==
                MotionState::MotionState_VELOCITY)
            {
                const bool reversing =
                    (
                        axis.currentCmdVel > 0.0 &&
                        targetVelocity < 0.0
                        )
                    ||
                    (
                        axis.currentCmdVel < 0.0 &&
                        targetVelocity > 0.0
                        );


                if (reversing)
                {
                    m_motion.StopMove(
                        axis,
                        axis.JOG_dec_time);

                    continue;
                }
            }


            // =================================================
            // Velocity Move
            //
            // 可以每個 Scan 更新 targetVelocity。
            // =================================================

            m_motion.VelocityMove(
                axis,
                targetVelocity,
                axis.JOG_acc_time);


            m_jogActive[i] =
                true;


            continue;
        }


        // =====================================================
        // 2. FINE CONTINUOUS JOG
        //
        // C23 = Fine JOG Mode
        //
        // C200 -> axis.FINE_JOG_0001_PPS
        // C201 -> axis.FINE_JOG_0010_PPS
        // C202 -> axis.FINE_JOG_0100_PPS
        // C203 -> axis.FINE_JOG_1000_PPS
        //
        // 不使用 R200。
        // 加減速共用 JOG_acc_time / JOG_dec_time。
        // =====================================================

        if (m_manualMoveMode ==
            ManualMoveMode::FINE_JOG)
        {
            if (!allowPositive &&
                !allowNegative)
            {
                if (m_jogActive[i] &&
                    axis.state ==
                    MotionState::MotionState_VELOCITY)
                {
                    m_motion.StopMove(
                        axis,
                        axis.JOG_dec_time);
                }

                continue;
            }


            // Fine Speed Selector 必須 One-Hot。
            if (fineJogSpeedSelectCount != 1)
            {
                if (m_jogActive[i] &&
                    axis.state ==
                    MotionState::MotionState_VELOCITY)
                {
                    m_motion.StopMove(
                        axis,
                        axis.JOG_dec_time);
                }

                continue;
            }


            double fineJogVelocity =
                0.0;


            if (fineJog0001)
            {
                fineJogVelocity =
                    axis.FINE_JOG_0001_PPS;
            }
            else if (fineJog0010)
            {
                fineJogVelocity =
                    axis.FINE_JOG_0010_PPS;
            }
            else if (fineJog0100)
            {
                fineJogVelocity =
                    axis.FINE_JOG_0100_PPS;
            }
            else if (fineJog1000)
            {
                fineJogVelocity =
                    axis.FINE_JOG_1000_PPS;
            }


            if (fineJogVelocity <= 0.0)
            {
                if (m_jogActive[i] &&
                    axis.state ==
                    MotionState::MotionState_VELOCITY)
                {
                    m_motion.StopMove(
                        axis,
                        axis.JOG_dec_time);
                }

                continue;
            }


            if (axis.maxVel_PPS > 0.0 &&
                fineJogVelocity >
                axis.maxVel_PPS)
            {
                fineJogVelocity =
                    axis.maxVel_PPS;
            }


            double targetVelocity =
                0.0;


            if (allowPositive)
            {
                targetVelocity =
                    fineJogVelocity;
            }
            else if (allowNegative)
            {
                targetVelocity =
                    -fineJogVelocity;
            }


            if (axis.state ==
                MotionState::MotionState_STOPPING)
            {
                continue;
            }


            if (axis.state ==
                MotionState::MotionState_MOVING ||
                axis.state ==
                MotionState::MotionState_INTERPOLATING)
            {
                continue;
            }


            // 反向時先停到 IDLE，再反向。
            if (axis.state ==
                MotionState::MotionState_VELOCITY)
            {
                const bool reversing =
                    (
                        axis.currentCmdVel > 0.0 &&
                        targetVelocity < 0.0
                        )
                    ||
                    (
                        axis.currentCmdVel < 0.0 &&
                        targetVelocity > 0.0
                        );


                if (reversing)
                {
                    m_motion.StopMove(
                        axis,
                        axis.JOG_dec_time);

                    continue;
                }
            }


            // Fine JOG 與 Normal JOG 共用加減速。
            // C200~C203 切換時可直接更新 targetVelocity。
            m_motion.VelocityMove(
                axis,
                targetVelocity,
                axis.JOG_acc_time);


            m_jogActive[i] =
                true;


            continue;
        }


        // =====================================================
// 3. INCH JOG
//
// Manual Frame OFF:
//     保持原本單軸 INCH 行為
//
// Manual Frame ON + XYZ:
//     Manual Distance Vector
//          ↓
//     CoordinateManager
//          ↓
//     Machine XYZ Distance Vector
//          ↓
//     XYZ 同步定距移動
//
// A/B/C/U/V：
//     永遠維持原本單軸 INCH
// =====================================================

        if (m_manualMoveMode ==
            ManualMoveMode::INCH_JOG)
        {
            // =================================================
            // A. Manual Frame XYZ INCH
            //
            // 只有：
            //
            //     Manual Frame = ON
            //     i = X/Y/Z
            //
            // 才走這裡。
            // =================================================

            if (m_nc.GetCoordSys()
                .IsManualFrameEnabled() &&
                i < 3)
            {
                // ---------------------------------------------
                // 一次 INCH 必須從完全停止開始
                // ---------------------------------------------

                if (axis.state !=
                    MotionState::MotionState_IDLE)
                {
                    continue;
                }


                // =============================================
                // Logical Axis INCH Distance
                //
                // 使用「使用者現在按的軸」自己的
                // INCH Distance Parameter。
                // =============================================

                double inchDistance =
                    0.0;


                switch (m_inchDistanceLevel)
                {
                case 0:

                    inchDistance =
                        axis.INCH_0001_DISTANCE;

                    break;


                case 1:

                    inchDistance =
                        axis.INCH_0010_DISTANCE;

                    break;


                case 2:

                    inchDistance =
                        axis.INCH_0100_DISTANCE;

                    break;


                case 3:

                    inchDistance =
                        axis.INCH_1000_DISTANCE;

                    break;


                default:

                    inchDistance =
                        axis.INCH_0001_DISTANCE;

                    break;
                }


                if (inchDistance <= 0.0)
                {
                    continue;
                }


                // =============================================
                // Rising Edge
                //
                // INCH 一次按壓只動一次。
                // =============================================

                const bool inchPositive =
                    positiveRising;


                const bool inchNegative =
                    negativeRising;


                if (!inchPositive &&
                    !inchNegative)
                {
                    continue;
                }


                if (inchPositive &&
                    inchNegative)
                {
                    continue;
                }


                // =============================================
                // Manual Distance Vector
                //
                // 例如：
                //
                // X+ / 0.100
                //
                // [0.100, 0, 0]
                //
                // 注意：
                // INCH 是距離，不做 Normalize。
                // =============================================

                double manualVector[8] =
                {
                    0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0
                };


                manualVector[i] =
                    inchPositive
                    ? inchDistance
                    : -inchDistance;


                // =============================================
                // Manual Frame -> Machine Frame
                // =============================================

                double machineVector[8] =
                {
                    0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0
                };


                m_nc.GetCoordSys()
                    .TransformManualVector(
                        manualVector,
                        machineVector);


                constexpr double
                    INCH_VECTOR_EPSILON =
                    1.0e-10;


                // =============================================
                // Rotated Path Length
                // =============================================

                const double pathDistance =
                    std::sqrt(
                        machineVector[0] *
                        machineVector[0] +
                        machineVector[1] *
                        machineVector[1] +
                        machineVector[2] *
                        machineVector[2]);


                if (pathDistance <=
                    INCH_VECTOR_EPSILON)
                {
                    continue;
                }


                // =============================================
                // Logical Axis Speed
                //
                // 原本 INCH_JOG_PPS
                // 先由 Pulse/sec 轉成 mm/sec。
                // =============================================

                if (axis.resolution_PPR <= 0.0 ||
                    axis.finalLead <= 0.0 ||
                    axis.INCH_JOG_PPS <= 0.0)
                {
                    continue;
                }


                const double logicalPulsePerUnit =
                    axis.resolution_PPR /
                    axis.finalLead;


                double pathSpeedUnitPerSec =
                    axis.INCH_JOG_PPS /
                    logicalPulsePerUnit;


                if (axis.maxVel_PPS > 0.0)
                {
                    const double
                        logicalMaxUnitPerSec =
                        axis.maxVel_PPS /
                        logicalPulsePerUnit;


                    if (pathSpeedUnitPerSec >
                        logicalMaxUnitPerSec)
                    {
                        pathSpeedUnitPerSec =
                            logicalMaxUnitPerSec;
                    }
                }


                if (pathSpeedUnitPerSec <= 0.0)
                {
                    continue;
                }


                // =============================================
                // Validate Physical XYZ Group
                // =============================================

                bool groupValid =
                    true;


                double groupAccTime =
                    0.001;


                double groupDecTime =
                    0.001;


                for (int machineAxis = 0;
                    machineAxis < 3;
                    ++machineAxis)
                {
                    const double deltaUnit =
                        machineVector[
                            machineAxis];


                    if (std::abs(deltaUnit) <=
                        INCH_VECTOR_EPSILON)
                    {
                        continue;
                    }


                    AxisContext& physicalAxis =
                        m_motion.GetAxisContext(
                            machineAxis);


                    const bool physicalAxisFault =
                        physicalAxis.isFault ||
                        physicalAxis.isLagAlarm ||
                        physicalAxis.state ==
                        MotionState::
                        MotionState_ERROR ||
                        physicalAxis.state ==
                        MotionState::
                        MotionState_ESTOP;


                    // -----------------------------------------
                    // Axis / Servo / Motion Ownership
                    // -----------------------------------------

                    if (!physicalAxis.isExist ||
                        !physicalAxis.isServoOn ||
                        physicalAxisFault ||
                        physicalAxis.state !=
                        MotionState::
                        MotionState_IDLE)
                    {
                        groupValid =
                            false;

                        break;
                    }


                    if (physicalAxis.resolution_PPR <=
                        0.0 ||
                        physicalAxis.finalLead <=
                        0.0 ||
                        physicalAxis.INCH_JOG_PPS <=
                        0.0)
                    {
                        groupValid =
                            false;

                        break;
                    }


                    // =========================================
                    // Physical Machine Hard Limit
                    //
                    // 注意：
                    //
                    // 現在不能看「按的 Logical Axis」。
                    //
                    // 必須看旋轉後真正會動的實體軸方向。
                    // =========================================

                    const bool physicalPositiveLimit =
                        m_plc.Get_C(
                            NCPLC::C::AxisPoint(
                                NCPLC::C::
                                POSITIVE_LIMIT_BASE,
                                machineAxis));


                    const bool physicalNegativeLimit =
                        m_plc.Get_C(
                            NCPLC::C::AxisPoint(
                                NCPLC::C::
                                NEGATIVE_LIMIT_BASE,
                                machineAxis));


                    if (physicalPositiveLimit &&
                        physicalNegativeLimit)
                    {
                        groupValid =
                            false;

                        break;
                    }


                    if (deltaUnit > 0.0 &&
                        physicalPositiveLimit)
                    {
                        groupValid =
                            false;

                        break;
                    }


                    if (deltaUnit < 0.0 &&
                        physicalNegativeLimit)
                    {
                        groupValid =
                            false;

                        break;
                    }


                    // =========================================
                    // Physical Axis Speed Capacity
                    // =========================================

                    const double pulsePerUnit =
                        physicalAxis.resolution_PPR /
                        physicalAxis.finalLead;


                    double physicalMaxPPS =
                        physicalAxis.INCH_JOG_PPS;


                    if (physicalAxis.maxVel_PPS >
                        0.0 &&
                        physicalMaxPPS >
                        physicalAxis.maxVel_PPS)
                    {
                        physicalMaxPPS =
                            physicalAxis.maxVel_PPS;
                    }


                    const double
                        physicalMaxUnitPerSec =
                        physicalMaxPPS /
                        pulsePerUnit;


                    const double
                        directionComponent =
                        std::abs(deltaUnit) /
                        pathDistance;


                    if (directionComponent >
                        INCH_VECTOR_EPSILON)
                    {
                        const double
                            allowedPathSpeed =
                            physicalMaxUnitPerSec /
                            directionComponent;


                        if (allowedPathSpeed <
                            pathSpeedUnitPerSec)
                        {
                            pathSpeedUnitPerSec =
                                allowedPathSpeed;
                        }
                    }


                    // =========================================
                    // Group Acc / Dec
                    //
                    // 所有參與 XYZ 使用相同時間。
                    // =========================================

                    double axisAccTime =
                        physicalAxis.INCH_acc_time;


                    double axisDecTime =
                        physicalAxis.INCH_dec_time;


                    if (axisAccTime < 0.001)
                    {
                        axisAccTime =
                            0.2;
                    }


                    if (axisDecTime < 0.001)
                    {
                        axisDecTime =
                            axisAccTime;
                    }


                    if (axisAccTime >
                        groupAccTime)
                    {
                        groupAccTime =
                            axisAccTime;
                    }


                    if (axisDecTime >
                        groupDecTime)
                    {
                        groupDecTime =
                            axisDecTime;
                    }
                }


                if (!groupValid ||
                    pathSpeedUnitPerSec <= 0.0)
                {
                    continue;
                }


                // =============================================
                // Execute Rotated XYZ INCH
                // =============================================

                for (int machineAxis = 0;
                    machineAxis < 3;
                    ++machineAxis)
                {
                    const double deltaUnit =
                        machineVector[
                            machineAxis];


                    if (std::abs(deltaUnit) <=
                        INCH_VECTOR_EPSILON)
                    {
                        continue;
                    }


                    AxisContext& physicalAxis =
                        m_motion.GetAxisContext(
                            machineAxis);


                    const double pulsePerUnit =
                        physicalAxis.resolution_PPR /
                        physicalAxis.finalLead;


                    const double deltaPulse =
                        deltaUnit *
                        pulsePerUnit;


                    const double targetPosition =
                        physicalAxis.currentActPos +
                        deltaPulse;


                    const double
                        directionComponent =
                        std::abs(deltaUnit) /
                        pathDistance;


                    double axisVelocity =
                        pathSpeedUnitPerSec *
                        directionComponent *
                        pulsePerUnit;


                    if (axisVelocity <= 0.0)
                    {
                        continue;
                    }


                    if (physicalAxis.maxVel_PPS >
                        0.0 &&
                        axisVelocity >
                        physicalAxis.maxVel_PPS)
                    {
                        axisVelocity =
                            physicalAxis.maxVel_PPS;
                    }


                    // =========================================
                    // XYZ 是 Linear Axis，
                    // 不需要 Rotary Shortest Path 處理。
                    // =========================================

                    m_motion.MoveToPosition(
                        physicalAxis,
                        targetPosition,
                        axisVelocity,
                        groupAccTime,
                        groupDecTime);


                    m_jogActive[
                        machineAxis] =
                        true;
                }


                // 已完成 Manual Frame XYZ INCH。
                // 絕對不可再往下執行舊單軸 INCH。
                continue;
            }


            // =================================================
            // B. Original INCH
            //
            // 以下情況使用原本程式：
            //
            // Manual Frame OFF
            //
            // 或：
            //
            // A/B/C/U/V
            // =================================================


            // -------------------------------------------------
            // INCH 只能從完全停止開始
            // -------------------------------------------------

            if (axis.state !=
                MotionState::MotionState_IDLE)
            {
                continue;
            }


            // -------------------------------------------------
            // R201 -> 每軸自己的 INCH Distance Parameter
            // -------------------------------------------------

            double inchDistance =
                0.0;


            switch (m_inchDistanceLevel)
            {
            case 0:

                inchDistance =
                    axis.INCH_0001_DISTANCE;

                break;


            case 1:

                inchDistance =
                    axis.INCH_0010_DISTANCE;

                break;


            case 2:

                inchDistance =
                    axis.INCH_0100_DISTANCE;

                break;


            case 3:

                inchDistance =
                    axis.INCH_1000_DISTANCE;

                break;


            default:

                inchDistance =
                    axis.INCH_0001_DISTANCE;

                break;
            }


            if (inchDistance <= 0.0)
            {
                continue;
            }


            // -------------------------------------------------
            // Rising Edge
            // -------------------------------------------------

            const bool inchPositive =
                positiveRising &&
                !positiveLimit;


            const bool inchNegative =
                negativeRising &&
                !negativeLimit;


            if (!inchPositive &&
                !inchNegative)
            {
                continue;
            }


            if (inchPositive &&
                inchNegative)
            {
                continue;
            }


            // -------------------------------------------------
            // Mechanical Conversion
            // -------------------------------------------------

            if (axis.resolution_PPR <= 0.0 ||
                axis.finalLead <= 0.0)
            {
                continue;
            }


            const double pulsePerUnit =
                axis.resolution_PPR /
                axis.finalLead;


            const double inchDistancePulse =
                inchDistance *
                pulsePerUnit;


            if (inchDistancePulse <= 0.0)
            {
                continue;
            }


            // -------------------------------------------------
            // Relative Target
            // -------------------------------------------------

            double targetPosition =
                axis.currentActPos;


            if (inchPositive)
            {
                targetPosition +=
                    inchDistancePulse;
            }
            else
            {
                targetPosition -=
                    inchDistancePulse;
            }


            // -------------------------------------------------
            // INCH Speed
            // -------------------------------------------------

            double inchVelocity =
                axis.INCH_JOG_PPS;


            if (inchVelocity <= 0.0)
            {
                continue;
            }


            if (axis.maxVel_PPS > 0.0 &&
                inchVelocity >
                axis.maxVel_PPS)
            {
                inchVelocity =
                    axis.maxVel_PPS;
            }


            // -------------------------------------------------
            // INCH Acc / Dec
            // -------------------------------------------------

            double inchAccTime =
                axis.INCH_acc_time;


            double inchDecTime =
                axis.INCH_dec_time;


            if (inchAccTime < 0.001)
            {
                inchAccTime =
                    0.2;
            }


            if (inchDecTime < 0.001)
            {
                inchDecTime =
                    inchAccTime;
            }


            // =================================================
            // Rotary Relative INCH
            // =================================================

            const bool originalShortestPath =
                axis.useShortestPath;


            if (axis.axisType ==
                AxisType::ROTARY)
            {
                axis.useShortestPath =
                    false;
            }


            // =================================================
            // Execute Original Single Axis INCH
            // =================================================

            m_motion.MoveToPosition(
                axis,
                targetPosition,
                inchVelocity,
                inchAccTime,
                inchDecTime);


            axis.useShortestPath =
                originalShortestPath;


            m_jogActive[i] =
                true;


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
    // Future:
    //
    // C100~107 DOG
    // C108~115 INDEX
    // C150~157 HOME Request
    // C15       HOME ALL
    // =====================================================
}


// =========================================================
// Auxiliary Handshake
// =========================================================

void NCPLCManager::ProcessAuxiliaryHandshake()
{
    // =====================================================
    // Future:
    //
    // NC -> PLC
    //
    // S30 AUX_REQUEST
    //
    // PLC -> NC
    //
    // C10 AUX_FIN
    //
    // Data:
    //
    // R100 M
    // R101 S
    // R102 T
    // R103 Valid Mask
    // =====================================================
}


// =========================================================
// NC -> PLC Status
// =========================================================

void NCPLCManager::SyncNCStateToPLC()
{
    // =====================================================
    // Future:
    //
    // 搬移舊的：
    //
    // NCManager::SyncNCStateToPLC()
    //
    // 最後移除 NCManager_PLCSync.cpp
    // =====================================================
}