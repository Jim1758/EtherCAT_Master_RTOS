#pragma once


namespace NCPLC
{
    constexpr int AXIS_COUNT = 8;                                  // NC 最大軸數：X/Y/Z/A/B/C/U/V


    // ============================================================
    // C Point
    //
    // PLC -> NC
    // ============================================================

    namespace C
    {
        // --------------------------------------------------------
        // NC 全域控制
        // --------------------------------------------------------

        constexpr int CONTROLLED_STOP = 4;                         // 控制減速停止 / Feed Hold
        constexpr int EMERGENCY_STOP = 5;                          // 緊急停止

        constexpr int AUX_FIN = 10;                                // M/S/T 輔助功能完成訊號
        constexpr int SERVO_READY = 11;                            // Servo / Machine Ready

        constexpr int CYCLE_START = 12;                            // Cycle Start
        constexpr int RESET = 13;                                  // NC Reset
        constexpr int SERVO_FAULT_RESET = 14;                      // Servo Fault Reset
        constexpr int HOME_ALL = 15;                               // 全軸 HOME

        constexpr int SINGLE_BLOCK = 16;                           // Single Block
        constexpr int OPTIONAL_STOP = 17;                          // Optional Stop
        constexpr int BLOCK_SKIP = 18;                             // Block Skip

        constexpr int JOG_MODE = 19;                               // Normal Continuous JOG
        constexpr int EDM_PROTECTION_BYPASS = 20;                  // EDM 加工保護暫時解除
        constexpr int MPG_MODE = 21;                               // MPG 手輪模式
        constexpr int RESERVED_22 = 22;                            // 保留，Manual Frame 不使用 PLC 點位
        constexpr int FINE_JOG_MODE = 23;                          // Fine Continuous JOG
        constexpr int INCH_JOG_MODE = 24;                          // INCH 定距移動


        // --------------------------------------------------------
        // HOME Sensor
        //
        // Axis 0~7
        // --------------------------------------------------------

        constexpr int HOME_DOG_BASE = 100;                         // C100~107 HOME DOG Sensor
        constexpr int HOME_INDEX_BASE = 108;                       // C108~115 HOME Encoder Index


        // --------------------------------------------------------
        // Manual Direction
        //
        // Axis 0~7
        // --------------------------------------------------------

        constexpr int JOG_POSITIVE_BASE = 120;                     // C120~127 手動正方向 JOG+
        constexpr int JOG_NEGATIVE_BASE = 130;                     // C130~137 手動負方向 JOG-


        // --------------------------------------------------------
        // Axis Protection
        //
        // Axis 0~7
        // --------------------------------------------------------

        constexpr int AXIS_PROTECT_STOP_BASE = 140;                // C140~147 外部軸保護停止


        // --------------------------------------------------------
        // HOME Request
        //
        // Axis 0~7
        // --------------------------------------------------------

        constexpr int HOME_REQUEST_BASE = 150;                     // C150~157 單軸 HOME Request


        // --------------------------------------------------------
        // Servo Reset
        //
        // Axis 0~7
        // --------------------------------------------------------

        constexpr int SERVO_RESET_BASE = 160;                      // C160~167 單軸 Servo Reset


        // --------------------------------------------------------
        // Hard Limit
        //
        // Axis 0~7
        // --------------------------------------------------------

        constexpr int POSITIVE_LIMIT_BASE = 180;                   // C180~187 正極限 +OT
        constexpr int NEGATIVE_LIMIT_BASE = 190;                   // C190~197 負極限 -OT


        // --------------------------------------------------------
        // Fine JOG Speed Select
        //
        // 必須 One-Hot
        // 這些是速度參數選擇，不是移動距離
        // --------------------------------------------------------

        constexpr int FINE_JOG_SPEED_0001 = 200;                   // C200 選擇 Fine JOG 0.001 速度參數
        constexpr int FINE_JOG_SPEED_0010 = 201;                   // C201 選擇 Fine JOG 0.010 速度參數
        constexpr int FINE_JOG_SPEED_0100 = 202;                   // C202 選擇 Fine JOG 0.100 速度參數
        constexpr int FINE_JOG_SPEED_1000 = 203;                   // C203 選擇 Fine JOG 1.000 速度參數


        // --------------------------------------------------------
        // MPG Axis Select
        //
        // 必須 One-Hot
        // Axis 0~7
        // --------------------------------------------------------

        constexpr int MPG_AXIS_SELECT_BASE = 210;                  // C210~217 MPG 軸選擇


        // --------------------------------------------------------
        // MPG Multiplier
        //
        // 必須 One-Hot
        // --------------------------------------------------------

        constexpr int MPG_MULTIPLIER_X1 = 220;                     // C220 MPG ×1
        constexpr int MPG_MULTIPLIER_X10 = 221;                    // C221 MPG ×10
        constexpr int MPG_MULTIPLIER_X100 = 222;                   // C222 MPG ×100
        constexpr int MPG_MULTIPLIER_X1000 = 223;                  // C223 MPG ×1000


        // --------------------------------------------------------
        // Helper
        // --------------------------------------------------------

        constexpr int AxisPoint(int base, int axisIndex)           // 取得某軸對應的 C Point
        {
            return base + axisIndex;
        }
    }


    // ============================================================
    // S Point
    //
    // NC -> PLC
    // ============================================================

    namespace S
    {
        // --------------------------------------------------------
        // NC State
        // --------------------------------------------------------

        constexpr int SYSTEM_READY = 0;                            // NC 系統 Ready
        constexpr int NC_START = 1;                                // NC Program Running
        constexpr int NC_HOLD = 2;                                 // NC Feed Hold
        constexpr int PROGRAM_END = 3;                             // Program End
        constexpr int RESET_STATE = 4;                             // Reset State
        constexpr int ALARM_ACTIVE = 5;                            // NC Alarm Active


        // --------------------------------------------------------
        // NC Mode
        // --------------------------------------------------------

        constexpr int MODE_MEMORY = 10;                            // MEMORY Mode
        constexpr int MODE_MDI = 11;                               // MDI Mode
        constexpr int MODE_MANUAL = 12;                            // MANUAL Mode
        constexpr int MODE_EDIT = 13;                              // EDIT Mode


        // --------------------------------------------------------
        // Program Control State
        // --------------------------------------------------------

        constexpr int SINGLE_BLOCK = 20;                           // Single Block Active
        constexpr int OPTIONAL_STOP = 22;                          // Optional Stop Active
        constexpr int BLOCK_SKIP = 23;                             // Block Skip Active


        // --------------------------------------------------------
        // Auxiliary M / S / T
        // --------------------------------------------------------

        constexpr int AUX_REQUEST = 30;                            // NC 發出 M/S/T 執行要求
        constexpr int AUX_WAIT_FIN = 31;                           // NC 等待 AUX FIN


        // --------------------------------------------------------
        // Axis Servo State
        //
        // Axis 0~7
        // --------------------------------------------------------

        constexpr int SERVO_ON_BASE = 100;                         // S100~107 Servo ON
        constexpr int HOMED_BASE = 110;                            // S110~117 HOME 完成


        // --------------------------------------------------------
        // Axis Limit State
        //
        // Axis 0~7
        // --------------------------------------------------------

        constexpr int POSITIVE_LIMIT_BASE = 120;                   // S120~127 正極限 +OT
        constexpr int NEGATIVE_LIMIT_BASE = 130;                   // S130~137 負極限 -OT


        // --------------------------------------------------------
        // HOME State
        //
        // Axis 0~7
        // --------------------------------------------------------

        constexpr int HOME_SEARCH_DOG_BASE = 140;                  // S140~147 HOME DOG Search


        // --------------------------------------------------------
        // Axis Alarm
        //
        // Axis 0~7
        // --------------------------------------------------------

        constexpr int LAG_ALARM_BASE = 150;                        // S150~157 Following Error / Lag Alarm
        constexpr int DRIVE_FAULT_BASE = 160;                      // S160~167 Servo Drive Fault


        // --------------------------------------------------------
        // Coordinate / Mirror
        //
        // Axis 0~7
        // --------------------------------------------------------

        constexpr int MIRROR_ACTIVE_BASE = 170;                    // S170~177 Axis Mirror Active


        // --------------------------------------------------------
        // Manual JOG State
        //
        // Axis 0~7
        // --------------------------------------------------------

        constexpr int JOG_ACTIVE_BASE = 180;                       // S180~187 Manual JOG Active


        // --------------------------------------------------------
        // HOME Index State
        //
        // Axis 0~7
        // --------------------------------------------------------

        constexpr int HOME_SEARCH_INDEX_BASE = 190;                // S190~197 HOME Index Search


        // --------------------------------------------------------
        // Manual Function State
        // --------------------------------------------------------

        constexpr int MANUAL_FRAME_ACTIVE = 200;                   // S200 Manual Frame 已啟用
        constexpr int MPG_ACTIVE = 201;                            // S201 MPG Mode Active
        constexpr int EDM_PROTECTION_BYPASS_ACTIVE = 202;          // S202 EDM Protection Bypass Active


        // --------------------------------------------------------
        // Helper
        // --------------------------------------------------------

        constexpr int AxisPoint(int base, int axisIndex)           // 取得某軸對應的 S Point
        {
            return base + axisIndex;
        }
    }


    // ============================================================
    // R Register
    //
    // PLC <-> NC
    // Integer
    // ============================================================

    namespace R
    {
        // --------------------------------------------------------
        // Auxiliary M / S / T
        // --------------------------------------------------------

        constexpr int AUX_M_CODE = 100;                            // R100 M Code
        constexpr int AUX_S_CODE = 101;                            // R101 S Code
        constexpr int AUX_T_CODE = 102;                            // R102 T Code
        constexpr int AUX_VALID_MASK = 103;                        // R103 M/S/T 有效資料 Mask


        // --------------------------------------------------------
        // Manual Motion
        // --------------------------------------------------------

        constexpr int JOG_SPEED_PERCENT = 200;                     // R200 Normal JOG Speed 0~100%
        constexpr int INCH_DISTANCE_LEVEL = 201;                   // R201 INCH 距離等級 0~3
    }


    // ============================================================
    // DR Register
    //
    // PLC <-> NC
    // 32-bit Integer
    // ============================================================

    namespace DR
    {
        constexpr int MPG_ENCODER_COUNT = 200;                     // DR200 MPG 手輪累積 Encoder Count
    }
}