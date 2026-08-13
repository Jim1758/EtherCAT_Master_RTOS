#pragma once

#include "NCPLCMap.h"

#include <cstdint>


class NCManager;
class MotionCore;
class PLCManager;



// =========================================================
// Manual Move Mode
//
// C19 = Normal Continuous JOG
// C23 = Fine Continuous JOG
// C24 = INCH JOG
// C21 = MPG
//
// 四種模式必須互斥。
// =========================================================

enum class ManualMoveMode
{
    NONE = 0,

    // C19
    CONTINUOUS_JOG = 1,

    // C23
    FINE_JOG = 2,

    // C24
    INCH_JOG = 3,

    // 舊名稱暫時相容
    INCREMENTAL_JOG = INCH_JOG,

    // C21
    MPG = 4
};

// =========================================================
// NCPLCManager
//
// PLC <-> NC Semantic Layer
//
// PLCManager
//     只負責 Raw PLC Memory / Runtime
//
// NCPLCManager
//     C Point / R Register
//     -> NC / Motion Semantic
// =========================================================

class NCPLCManager
{
public:

    // =====================================================
    // Constructor
    // =====================================================

    NCPLCManager(
        NCManager& nc,
        MotionCore& motion,
        PLCManager& plc);


    // =====================================================
    // Main Cyclic Process
    //
    // 每個 NC Scan 呼叫一次。
    // =====================================================

    void Process();


private:

    // =====================================================
    // PLC -> NC
    // =====================================================

    void ProcessGlobalInputs();

    void ProcessSafetyInputs();

    void ProcessManualInputs();

    void ProcessHomeInputs();

    void ProcessAuxiliaryHandshake();


    // =====================================================
    // NC -> PLC
    // =====================================================

    void SyncNCStateToPLC();


private:

    // =====================================================
    // References
    // =====================================================

    NCManager&
        m_nc;

    MotionCore&
        m_motion;

    PLCManager&
        m_plc;


    // =====================================================
    // Global State
    // =====================================================

    bool
        m_servoReady = false;


    bool
        m_edmProtectionBypass = false;


    // =====================================================
    // Global Input Edge Memory
    // =====================================================

    bool
        m_prevCycleStart = false;


    bool
        m_prevReset = false;


    bool
        m_prevServoFaultReset = false;


    bool
        m_prevControlledStop = false;


    bool
        m_prevEmergencyStop = false;


    // =====================================================
    // Safety Edge Memory
    // =====================================================

    bool
        m_prevAxisProtect[NCPLC::AXIS_COUNT] =
    {
        false,
        false,
        false,
        false,
        false,
        false,
        false,
        false
    };


    bool
        m_prevPositiveLimit[NCPLC::AXIS_COUNT] =
    {
        false,
        false,
        false,
        false,
        false,
        false,
        false,
        false
    };


    bool
        m_prevNegativeLimit[NCPLC::AXIS_COUNT] =
    {
        false,
        false,
        false,
        false,
        false,
        false,
        false,
        false
    };


    // =====================================================
    // Manual Move Mode
    //
    // C19 = Continuous JOG
    // C24 = INCH JOG
    // C21 = MPG
    // =====================================================

    ManualMoveMode
        m_manualMoveMode =
        ManualMoveMode::NONE;


    // =====================================================
    // Continuous JOG
    //
    // R200 = 0 ~ 100 %
    // =====================================================

    double
        m_jogSpeedPercent = 0.0;


    // =====================================================
    // INCH JOG
    //
    // R201:
    //
    // 0 = 0.001
    // 1 = 0.010
    // 2 = 0.100
    // 3 = 1.000
    // =====================================================

    int
        m_inchDistanceLevel = 0;


    double
        m_inchDistance = 0.001;


    // =====================================================
    // Axis Direction
    //
    // C120 ~ C127 = +
    // C130 ~ C137 = -
    // =====================================================

    bool
        m_jogPositive[NCPLC::AXIS_COUNT] =
    {
        false,
        false,
        false,
        false,
        false,
        false,
        false,
        false
    };


    bool
        m_jogNegative[NCPLC::AXIS_COUNT] =
    {
        false,
        false,
        false,
        false,
        false,
        false,
        false,
        false
    };


    // =====================================================
    // Direction Edge Memory
    //
    // Continuous JOG：
    //     可用來處理 Direction Change
    //
    // INCH JOG：
    //     OFF -> ON 只執行一次。
    // =====================================================

    bool
        m_prevJogPositive[NCPLC::AXIS_COUNT] =
    {
        false,
        false,
        false,
        false,
        false,
        false,
        false,
        false
    };


    bool
        m_prevJogNegative[NCPLC::AXIS_COUNT] =
    {
        false,
        false,
        false,
        false,
        false,
        false,
        false,
        false
    };


    // =====================================================
    // Continuous JOG Ownership
    //
    // true：
    // NCPLCManager 目前仍擁有該軸的 Manual JOG。
    //
    // 放開按鍵後：
    // StopMove()
    //     ↓
    // STOPPING
    //     ↓
    // IDLE
    //     ↓
    // 才釋放 m_jogActive
    // =====================================================

    bool
        m_jogActive[NCPLC::AXIS_COUNT] =
    {
        false,
        false,
        false,
        false,
        false,
        false,
        false,
        false
    };


    // =====================================================
    // MPG
    //
    // DR200 = Encoder Accumulated Count
    // =====================================================

    int
        m_lastMPGCount = 0;


    // =====================================================
    // Temporary Compatibility
    //
    // 舊 NCPLCManager.cpp 還可能引用。
    //
    // 下一步換完新版 cpp 後會刪除。
    // =====================================================

    bool
        m_jogMode = false;


    bool
        m_jogRapid = false;


    double
        m_incrementDistance = 0.001;
};