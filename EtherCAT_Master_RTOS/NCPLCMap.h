#pragma once


// =========================================================
// NCPLCMap.h
//
// PLC <-> NC Fixed Mapping
//
// Convention:
//
// C = PLC -> NC
// S = NC  -> PLC
// R / DR / L = PLC Data Registers
//
// 這裡只定義點位。
// 不放任何 NC / Motion 行為。
// =========================================================

namespace NCPLC
{
    constexpr int AXIS_COUNT = 8;


    // =====================================================
    // C Point
    //
    // PLC -> NC
    // =====================================================

    namespace C
    {
        // -------------------------------------------------
        // Global NC Control
        // -------------------------------------------------

        constexpr int CONTROLLED_STOP = 4;
        constexpr int EMERGENCY_STOP = 5;

        constexpr int AUX_FIN = 10;
        constexpr int SERVO_READY = 11;

        constexpr int CYCLE_START = 12;
        constexpr int RESET = 13;
        constexpr int SERVO_FAULT_RESET = 14;
        constexpr int HOME_ALL = 15;

        constexpr int SINGLE_BLOCK = 16;
        constexpr int OPTIONAL_STOP = 17;
        constexpr int BLOCK_SKIP = 18;


        // =========================================================
        // Manual Move Mode
        //
        // 必須互斥：
        //
        // C19 = Normal Continuous JOG
        // C23 = Fine Continuous JOG
        // C24 = INCH JOG
        // C21 = MPG
        // =========================================================

        constexpr int JOG_MODE =
            19;

        constexpr int EDM_PROTECTION_BYPASS =
            20;

        constexpr int MPG_MODE =
            21;

        constexpr int MANUAL_FRAME_ENABLE =
            22;

        constexpr int FINE_JOG_MODE =
            23;

        constexpr int INCH_JOG_MODE =
            24;

        // =========================================================
        // C23
        //
        // 舊版：JOG Rapid
        // 新版：Reserved
        //
        // 暫時保留 JOG_RAPID alias，
        // 下一步 NCPLCManager 改完後再移除。
        // =========================================================

        constexpr int RESERVED_23 =
            23;

        constexpr int JOG_RAPID =
            RESERVED_23;


        // =========================================================
        // 舊名稱相容
        //
        // 如果目前 NCPLCManager 還叫
        // INCREMENTAL_JOG_MODE，先讓它可以 Build。
        // 下一步會正式改成 INCH_JOG_MODE。
        // =========================================================

        constexpr int INCREMENTAL_JOG_MODE =
            INCH_JOG_MODE;
        // -------------------------------------------------
        // HOME Sensor
        //
        // Axis 0 ~ Axis 7
        // -------------------------------------------------

        constexpr int HOME_DOG_BASE = 100;

        constexpr int HOME_INDEX_BASE = 108;


        // -------------------------------------------------
        // Manual Direction
        //
        // C120~127 = JOG+
        // C130~137 = JOG-
        // -------------------------------------------------

        constexpr int JOG_POSITIVE_BASE = 120;

        constexpr int JOG_NEGATIVE_BASE = 130;


        // -------------------------------------------------
        // Axis Protection
        //
        // C140~147
        //
        // External / Manual Axis Protection
        // -------------------------------------------------

        constexpr int AXIS_PROTECT_STOP_BASE = 140;


        // -------------------------------------------------
        // HOME Request
        //
        // C150~157
        // -------------------------------------------------

        constexpr int HOME_REQUEST_BASE = 150;


        // -------------------------------------------------
        // Per Axis Servo Reset
        //
        // C160~167
        // -------------------------------------------------

        constexpr int SERVO_RESET_BASE = 160;


        // -------------------------------------------------
        // Hard Limit
        //
        // C180~187 = +OT
        // C190~197 = -OT
        // -------------------------------------------------

        constexpr int POSITIVE_LIMIT_BASE = 180;

        constexpr int NEGATIVE_LIMIT_BASE = 190;


        // =========================================================
     // Fine Continuous JOG Speed Select
     //
     // C200~C203 必須 One-Hot。
     //
     // 注意：
     // 這四個不是距離。
     // 是四段 Fine JOG Speed Parameter 選擇。
     // =========================================================

        constexpr int FINE_JOG_SPEED_0001 =
            200;

        constexpr int FINE_JOG_SPEED_0010 =
            201;

        constexpr int FINE_JOG_SPEED_0100 =
            202;

        constexpr int FINE_JOG_SPEED_1000 =
            203;


        // -------------------------------------------------
        // MPG Axis Select
        //
        // One-Hot
        //
        // C210~217
        // -------------------------------------------------

        constexpr int MPG_AXIS_SELECT_BASE = 210;


        // =========================================================
    // MPG Magnification
    //
    // 必須 One-Hot
    // =========================================================

        constexpr int MPG_MULTIPLIER_X1 =
            220;

        constexpr int MPG_MULTIPLIER_X10 =
            221;

        constexpr int MPG_MULTIPLIER_X100 =
            222;

        constexpr int MPG_MULTIPLIER_X1000 =
            223;


        // -------------------------------------------------
        // Helper
        // -------------------------------------------------

        constexpr int AxisPoint(
            int base,
            int axisIndex)
        {
            return base + axisIndex;
        }
    }


    // =====================================================
    // S Point
    //
    // NC -> PLC
    // =====================================================

    namespace S
    {
        // -------------------------------------------------
        // NC State
        // -------------------------------------------------

        constexpr int SYSTEM_READY = 0;

        constexpr int NC_START = 1;

        constexpr int NC_HOLD = 2;

        constexpr int PROGRAM_END = 3;

        constexpr int RESET_STATE = 4;

        constexpr int ALARM_ACTIVE = 5;


        // -------------------------------------------------
        // NC Mode
        // -------------------------------------------------

        constexpr int MODE_MEMORY = 10;

        constexpr int MODE_MDI = 11;

        constexpr int MODE_MANUAL = 12;

        constexpr int MODE_EDIT = 13;


        // -------------------------------------------------
        // Program Control
        // -------------------------------------------------

        constexpr int SINGLE_BLOCK = 20;

        constexpr int OPTIONAL_STOP = 22;

        constexpr int BLOCK_SKIP = 23;


        // -------------------------------------------------
        // Auxiliary M / S / T
        // -------------------------------------------------

        constexpr int AUX_REQUEST = 30;

        constexpr int AUX_WAIT_FIN = 31;


        // -------------------------------------------------
        // Axis State
        // -------------------------------------------------

        constexpr int SERVO_ON_BASE = 100;

        constexpr int HOMED_BASE = 110;


        // -------------------------------------------------
        // Limit
        // -------------------------------------------------

        constexpr int POSITIVE_LIMIT_BASE = 120;

        constexpr int NEGATIVE_LIMIT_BASE = 130;


        // -------------------------------------------------
        // HOME State
        // -------------------------------------------------

        constexpr int HOME_SEARCH_DOG_BASE = 140;


        // -------------------------------------------------
        // Alarm
        // -------------------------------------------------

        constexpr int LAG_ALARM_BASE = 150;

        constexpr int DRIVE_FAULT_BASE = 160;


        // -------------------------------------------------
        // Coordinate / Mirror
        // -------------------------------------------------

        constexpr int MIRROR_ACTIVE_BASE = 170;


        // -------------------------------------------------
        // Manual JOG Active
        // -------------------------------------------------

        constexpr int JOG_ACTIVE_BASE = 180;


        // -------------------------------------------------
        // HOME Index Search
        // -------------------------------------------------

        constexpr int HOME_SEARCH_INDEX_BASE = 190;


        // -------------------------------------------------
        // Manual Function State
        // -------------------------------------------------

        constexpr int MANUAL_FRAME_ACTIVE = 200;

        constexpr int MPG_ACTIVE = 201;

        constexpr int EDM_PROTECTION_BYPASS_ACTIVE = 202;


        // -------------------------------------------------
        // Helper
        // -------------------------------------------------

        constexpr int AxisPoint(
            int base,
            int axisIndex)
        {
            return base + axisIndex;
        }
    }


    // =====================================================
    // R Register
    //
    // Integer data
    // =====================================================

    namespace R
    {
        // -------------------------------------------------
        // Auxiliary M / S / T
        // -------------------------------------------------

        constexpr int AUX_M_CODE = 100;

        constexpr int AUX_S_CODE = 101;

        constexpr int AUX_T_CODE = 102;

        constexpr int AUX_VALID_MASK = 103;


        // =========================================================
        // Manual Motion Registers
        // =========================================================

        // Continuous JOG Speed
        //
        // 0 ~ 100 %
        //

        constexpr int JOG_SPEED_PERCENT = 200;


        // INCH Distance Level
//
// 0 = 0.001
// 1 = 0.010
// 2 = 0.100
// 3 = 1.000
//
        constexpr int INCH_DISTANCE_LEVEL = 201;
    }


    // =====================================================
    // DR Register
    //
    // 32-bit Integer
    // =====================================================

    namespace DR
    {
        // =====================================================
  // MPG Handwheel Accumulated Encoder Count
  //
  // 正轉：
  // 100 -> 101 -> 102...
  //
  // 反轉：
  // 102 -> 101 -> 100...
  //
  // NCPLCManager 只處理 Delta Count。
  // =====================================================

        constexpr int MPG_ENCODER_COUNT =
            200;
    }


    // =====================================================
    // L Register
    //
    // Double / LREAL
    //
    // Manual Coordinate Frame
    // =====================================================

    namespace L
    {
        // -------------------------------------------------
        // Rotation angles
        //
        // Degree
        // -------------------------------------------------

        constexpr int MANUAL_YAW = 210;

        constexpr int MANUAL_PITCH = 211;

        constexpr int MANUAL_ROLL = 212;
    }
}