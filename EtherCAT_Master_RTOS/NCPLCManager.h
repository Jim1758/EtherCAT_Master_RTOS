#pragma once
#include "NCPLCMap.h"
#include <cstdint>

class NCManager;
class MotionCore;
class PLCManager;

// =========================================================
// Manual Move Mode (NCPLCManager 支援的手動運動模式。四種模式必須互斥，同一個 Scan 只能有一種有效)
// =========================================================
enum class ManualMoveMode
{
    NONE = 0,                   // 無任何手動運動模式
    CONTINUOUS_JOG = 1,         // C19: Normal Continuous JOG 一般連續寸動
    FINE_JOG = 2,               // C23: Fine Continuous JOG 精細連續寸動
    INCH_JOG = 3,               // C24: INCH JOG 固定距離寸動
    INCREMENTAL_JOG = INCH_JOG, // 舊名稱暫時相容: 舊 Incremental JOG 名稱，對應 INCH_JOG
    MPG = 4                     // C21: MPG 手輪模式 (Manual Pulse Generator)
};

// =========================================================
// NCPLCManager: PLC <-> NC Semantic Layer
// 負責 PLC 與 NC/Motion 之間的語意轉換 (如 C19 -> Continuous JOG Mode)。不直接處理 G-Code 或插補運算。
// =========================================================
class NCPLCManager
{
public:
    NCPLCManager(NCManager& nc, MotionCore& motion, PLCManager& plc); // Constructor (NCManager: 狀態/模式控制, MotionCore: 實際軸運動, PLCManager: PLC存取)

    void Process(); // Main Cyclic Process: 每個 NC Scan 呼叫一次 (Safety -> Global -> Manual -> HOME -> Aux -> NC Status -> PLC)

private:
    // PLC -> NC (將 PLC Input/Register 狀態轉換成 NCManager/MotionCore 的實際控制行為)
    void ProcessGlobalInputs();       // 處理 Start / Reset / Servo Ready 等全域輸入
    void ProcessSafetyInputs();       // 處理 Emergency / Protection / Hard Limit
    void ProcessManualInputs();       // 處理 JOG / Fine JOG / INCH / MPG
    void ProcessHomeInputs();         // 處理 HOME Sensor / HOME Request
    void ProcessAuxiliaryHandshake(); // 處理 M / S / T Auxiliary Handshake

    // NC -> PLC 
    void SyncNCStateToPLC();          // 將 NC / Motion 目前狀態同步回 PLC S Point。

private:
    // References (NCPLCManager 不擁有這些物件，僅保存 Reference 並呼叫其功能)
    NCManager& m_nc;                  // NC 核心管理器
    MotionCore& m_motion;             // Motion 運動核心
    PLCManager& m_plc;                // PLC Runtime / Memory 管理器

    // Global State
    bool m_servoReady = false;          // C11 Servo / Machine Ready 狀態
    bool m_edmProtectionBypass = false; // C20 EDM Protection Bypass 狀態

    // Global Input Edge Memory (保存上一個 Scan 的狀態，用來判斷 false->true 的 Rising Edge，避免重複執行)
    bool m_prevCycleStart = false;      // C12 Cycle Start 前一 Scan 狀態
    bool m_prevReset = false;           // C13 NC Reset 前一 Scan 狀態
    bool m_prevServoFaultReset = false; // C14 Servo Fault Reset 前一 Scan 狀態
    bool m_prevControlledStop = false;  // C4 Controlled Stop 前一 Scan 狀態
    bool m_prevEmergencyStop = false;   // C5 Emergency Stop 前一 Scan 狀態

    // Safety Edge Memory (主要用來讓 Alarm 只在 Rising Edge 建立一次)
    bool m_prevAxisProtect[NCPLC::AXIS_COUNT] = { false, false, false, false, false, false, false, false }; // C140~147 Axis Protection 前一 Scan 狀態
    bool m_prevPositiveLimit[NCPLC::AXIS_COUNT] = { false, false, false, false, false, false, false, false }; // C180~187 Positive Hard Limit 前一 Scan 狀態
    bool m_prevNegativeLimit[NCPLC::AXIS_COUNT] = { false, false, false, false, false, false, false, false }; // C190~197 Negative Hard Limit 前一 Scan 狀態

    ManualMoveMode m_manualMoveMode = ManualMoveMode::NONE; // 目前有效的 Manual Move Mode (C19, C23, C24, C21)

    double m_jogSpeedPercent = 0.0;     // R200 JOG Speed Percent (0 ~ 100)
    int m_inchDistanceLevel = 0;        // R201 INCH Distance Level (0 ~ 3)
    double m_inchDistance = 0.001;      // INCH 移動距離

    // Axis Direction (保存經過 Safety / Hard Limit 判斷之後，目前真正允許的移動方向)
    bool m_jogPositive[NCPLC::AXIS_COUNT] = { false, false, false, false, false, false, false, false }; // 各軸目前允許的正方向 JOG 狀態 (C120~127)
    bool m_jogNegative[NCPLC::AXIS_COUNT] = { false, false, false, false, false, false, false, false }; // 各軸目前允許的負方向 JOG 狀態 (C130~137)

    // Direction Edge Memory (保存 C120~137 上一個 Scan 原始狀態。用於 Continuous 的方向切換，或 INCH 的 Rising Edge)
    bool m_prevJogPositive[NCPLC::AXIS_COUNT] = { false, false, false, false, false, false, false, false }; // C120~127 前一 Scan 狀態
    bool m_prevJogNegative[NCPLC::AXIS_COUNT] = { false, false, false, false, false, false, false, false }; // C130~137 前一 Scan 狀態

    // Continuous JOG Ownership (true: 目前仍擁有該軸 Manual Motion 控制權，放開按鍵後停到 IDLE 才釋放，避免被搶控制權)
    bool m_jogActive[NCPLC::AXIS_COUNT] = { false, false, false, false, false, false, false, false }; // 各軸 Manual Motion Ownership

    int m_lastMPGCount = 0;             // 上一個 Scan 的 DR200 累積 Count (用來取得本 Scan 的 MPG Delta Count)

    // Temporary Compatibility 
    double m_incrementDistance = 0.001; // Incremental JOG Distance
};