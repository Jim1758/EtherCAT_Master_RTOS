// ==========================================================
// 檔案：NCManager_SysVars.cpp
// 功能：處理 NCManager 的系統變數 ($ 變數) 更新邏輯
// ==========================================================

#include "NCManager.h"
#include "CoordinateManager.h"
#include "MotionCore.h"
#include "MacroEngine.h"

// 🌟 注意這裡：只要掛上 NCManager::，它就還是 NCManager 的一部分！
void NCManager::UpdateSystemVariables()
{
    // =========================================================
    // 1. 軸啟用狀態 ($30 ~ $37)
    // =========================================================
    for (int i = 0; i < 8; i++) {
        bool isExist = m_motion.GetAxisContext(i).isExist;
        MacroSys.SetVar('$', 30 + i, isExist ? 1.0 : 0.0);
    }

    // =========================================================
    // 2. 模式與代碼狀態 ($40 ~ $47)
    // =========================================================
    MacroSys.SetVar('$', 40, (double)CoordSys.GetCurrentWCSGCode());
    MacroSys.SetVar('$', 41, (double)CoordSys.currentHCode);
    MacroSys.SetVar('$', 42, (double)CoordSys.currentDCode);
    MacroSys.SetVar('$', 43, (double)CoordSys.currentWCode);
    MacroSys.SetVar('$', 44, (double)CoordSys.activePlane);
    MacroSys.SetVar('$', 45, CoordSys.isAbsoluteMode ? 90.0 : 91.0);
    MacroSys.SetVar('$', 46, (double)CoordSys.toolLengthMode); // 43, 44, 或 49
    MacroSys.SetVar('$', 47, (double)CoordSys.toolRadiusMode); // 41, 42, 或 40
    MacroSys.SetVar('$', 48, (double)CoordSys.currentTCode);        // 🌟 新增：目前 T 碼 (刀號)
    MacroSys.SetVar('$', 49, (double)CoordSys.currentWorkpieceNum);  // 🌟 新增：獨立工件號

    // =========================================================
    // 3. 理想工作座標 ($100 ~ $107) 與 理想機械座標 ($110 ~ $117)
    // =========================================================
    double cmdWCS[8] = { 0.0 };
    CoordSys.GetCommandedWCS(cmdWCS); // 取得包含所有補償後的純理論 WCS

    for (int i = 0; i < 8; i++) {
        MacroSys.SetVar('$', 100 + i, cmdWCS[i]);                 // $100~$107
        MacroSys.SetVar('$', 110 + i, CoordSys.commandedMCS[i]);  // $110~$117
    }

    // =========================================================
    // 4. 無額外補償的原始工作座標 ($120 ~ $127)
    // 算式：Commanded MCS - EXT - G54 Table
    // =========================================================
    for (int i = 0; i < 8; i++) {
        double rawWCS = CoordSys.commandedMCS[i]
            - CoordSys.extOffset[i]
            - CoordSys.m_WCSTable[CoordSys.currentWCSIndex][i];
        MacroSys.SetVar('$', 120 + i, rawWCS);
    }

    // =========================================================
    // 5. 刀長補償實際值 ($130 ~ $137) 與 刀徑半徑 ($138)
    // =========================================================
    double cAngleMCS = CoordSys.commandedMCS[CoordSys.C_AXIS_INDEX];
    for (int i = 0; i < 8; i++) {
        double activeToolLen = CoordSys.GetActiveToolOffset(i, cAngleMCS);
        MacroSys.SetVar('$', 130 + i, activeToolLen);
    }
    double activeToolRad = CoordSys.GetActiveToolRadius();
    MacroSys.SetVar('$', 138, activeToolRad);

    // =========================================================
    // 6. G68 旋轉參數 ($150 ~ $153)
    // =========================================================
    MacroSys.SetVar('$', 150, CoordSys.g68CenterWCS[0]);
    MacroSys.SetVar('$', 151, CoordSys.g68CenterWCS[1]);
    MacroSys.SetVar('$', 152, CoordSys.g68CenterWCS[2]);
    MacroSys.SetVar('$', 153, CoordSys.isG68Active ? CoordSys.g68Angle : 0.0);

    // =========================================================
    // 7. G168 旋轉參數 ($160 ~ $165)
    // =========================================================
    MacroSys.SetVar('$', 160, CoordSys.rotationCenterMCS[0]);
    MacroSys.SetVar('$', 161, CoordSys.rotationCenterMCS[1]);
    MacroSys.SetVar('$', 162, CoordSys.rotationCenterMCS[2]);

    double yaw = 0.0, pitch = 0.0, roll = 0.0;
    if (CoordSys.isWorkpieceRotationActive && CoordSys.currentWCode > 0 && CoordSys.currentWCode <= CoordSys.m_WorkOffset.size()) {
        int wIdx = CoordSys.currentWCode - 1;
        yaw = CoordSys.m_WorkOffset[wIdx][CoordinateManager::WO_ANGLE_XY_YAW];
        pitch = CoordSys.m_WorkOffset[wIdx][CoordinateManager::WO_ANGLE_XZ_PITCH];
        roll = CoordSys.m_WorkOffset[wIdx][CoordinateManager::WO_ANGLE_YZ_ROLL];
    }
    MacroSys.SetVar('$', 163, yaw);
    MacroSys.SetVar('$', 164, pitch);
    MacroSys.SetVar('$', 165, roll);

    // =========================================================
    // 8. G51 縮放倍率 ($166)
    // =========================================================
    MacroSys.SetVar('$', 166, CoordSys.isScalingActive ? CoordSys.scaleFactor : 1.0);

    // =========================================================
    // 9. 座標系底層表格數值 ($180 ~ $197)
    // =========================================================
    for (int i = 0; i < 8; i++) {
        // EXT 外部偏移 ($180 ~ $187)
        MacroSys.SetVar('$', 180 + i, CoordSys.extOffset[i]);

        // 當前使用的工作座標系表格數值 ($190 ~ $197)
        MacroSys.SetVar('$', 190 + i, CoordSys.m_WCSTable[CoordSys.currentWCSIndex][i]);
    }
}