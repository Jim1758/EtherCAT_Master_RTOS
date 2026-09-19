// ==========================================================
// 檔案：NCManager_SysVars.cpp
// 功能：處理 NCManager 的系統變數 ($ 變數) 更新邏輯
// ==========================================================

#include "NCManager.h"
#include "CoordinateManager.h"
#include "MotionCore.h"
#include "MacroEngine.h"
#include <limits>

// 🌟 注意這裡：只要掛上 NCManager::，它就還是 NCManager 的一部分！
void NCManager::UpdateSystemVariables()
{
   
    // 🌟 新增：取得目前的顯示單位倍率 (公制=1.0, 英制=1/25.4)
    // RESET may clean live modes before the frozen display frame is retired.
    const bool displayFrozen = CoordSys.IsTranslationRunFrozen();
    const NCTranslationSnapshot display = displayFrozen ?
        CoordSys.GetTranslationSnapshot() : NCTranslationSnapshot{};
    const bool displayInch = displayFrozen ? display.unitsMode == 20 : CoordSys.isInchMode;
    const double unitScale = displayInch ? (1.0 / 25.4) : 1.0;
    double axisUnitScale[8] = {};
    for (int axis = 0; axis < 8; ++axis)
        axisUnitScale[axis] = (displayFrozen ? display.axisIdentity.nativeUnit[axis] == 2U :
            (m_motion.GetAxisContext(axis).axisType == AxisType::ROTARY ||
             m_motion.GetAxisContext(axis).axisType == AxisType::ROTARY_CONTINUOUS)) ? 1.0 : unitScale;

    // =========================================================
    // 1. 軸啟用狀態 ($30 ~ $37)
    // =========================================================
    for (int i = 0; i < 8; i++) {
        const bool isExist = displayFrozen ? display.axisIdentity.exists[i] == 1U :
            m_motion.GetAxisContext(i).isExist;
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
        MacroSys.SetVar('$', 100 + i, cmdWCS[i] * axisUnitScale[i]);         // $100~$107
        MacroSys.SetVar('$', 110 + i, CoordSys.commandedMCS[i] * axisUnitScale[i]);  // $110~$117
    }

    // =========================================================
    // 4. 無額外補償的原始工作座標 ($120 ~ $127)
    // 算式：Commanded MCS - EXT - G54 Table
    // =========================================================
    for (int i = 0; i < 8; i++) {
        double rawWCS = CoordSys.commandedMCS[i]
            - (displayFrozen ? display.extOffsetMM[i] : CoordSys.extOffset[i])
            - (displayFrozen ? display.wcsOffsetMM[i] : CoordSys.m_WCSTable[CoordSys.currentWCSIndex][i]);
        // 🌟 修改：乘上 unitScale
        MacroSys.SetVar('$', 120 + i, rawWCS * axisUnitScale[i]);
    }

    // =========================================================
    // 5. 刀長補償實際值 ($130 ~ $137) 與 刀徑半徑 ($138)
    // =========================================================
    double cAngleMCS = CoordSys.GetElectrodeRotationAngleMCS(CoordSys.commandedMCS);
    for (int i = 0; i < 8; i++) {
        const double activeToolLen = displayFrozen ? NCTranslationToolOffsetMM(display, i) :
            CoordSys.GetActiveToolOffset(i, cAngleMCS);
        MacroSys.SetVar('$', 130 + i, activeToolLen * axisUnitScale[i]);
    }
    const double activeToolRad = displayFrozen ? display.cutterRadiusMM : CoordSys.GetActiveToolRadius();
    MacroSys.SetVar('$', 138, activeToolRad * unitScale); // 🌟 修改

    // =========================================================
    // 6. G68 旋轉參數 ($150 ~ $153)
    // =========================================================
    if (displayFrozen)
    {
        // Report the frame used by commanded/actual inverse coordinates.
        // Interpreter modal group $16 retains its existing RESET semantics.
        if (display.rotationPlane == 17)
        {
            MacroSys.SetVar('$', 150, display.rotationCenterMM[0] * unitScale);
            MacroSys.SetVar('$', 151, display.rotationCenterMM[1] * unitScale);
            MacroSys.SetVar('$', 152, 0.0);
        }
        else
        {
            // BASE-PLANE-3: the snapshot owns canonical UV, but $150..152
            // remain physical X/Y/Z centre readback, including during RESET.
            NCArcPlaneAxes plane{};
            double center[3] = {};
            if (TryGetNCArcPlaneAxes(display.rotationPlane, plane))
            {
                center[plane.u] = display.rotationCenterMM[0];
                center[plane.v] = display.rotationCenterMM[1];
            }
            else
                for (unsigned axis = 0U; axis < 3U; ++axis)
                    center[axis] = (std::numeric_limits<double>::quiet_NaN)();
            for (unsigned axis = 0U; axis < 3U; ++axis)
                MacroSys.SetVar('$', 150 + axis, center[axis] * unitScale);
        }
        MacroSys.SetVar('$', 153, display.rotationAngleDeg);
    }
    else
    {
        MacroSys.SetVar('$', 150, CoordSys.g68CenterWCS[0] * unitScale);
        MacroSys.SetVar('$', 151, CoordSys.g68CenterWCS[1] * unitScale);
        MacroSys.SetVar('$', 152, CoordSys.g68CenterWCS[2] * unitScale);
        MacroSys.SetVar('$', 153, CoordSys.isG68Active ? CoordSys.g68Angle : 0.0);
    }

    // =========================================================
    // 7. G168 旋轉參數 ($160 ~ $165)
    // =========================================================
    double workCenterX = CoordSys.rotationCenterMCS[0];
    double workCenterY = CoordSys.rotationCenterMCS[1];
    double workCenterZ = CoordSys.rotationCenterMCS[2];
    double yaw = 0.0, pitch = 0.0, roll = 0.0;
    if (displayFrozen)
    {
        // The displayed frame stays on the same source while RESET cleans
        // live modes. The frozen G168 centre is canonical active-plane (u,v),
        // so publish it back into physical XYZ system-variable slots.
        workCenterX = workCenterY = workCenterZ = 0.0;
        NCArcPlaneAxes plane{};
        if (TryGetNCArcPlaneAxes(display.rotationPlane, plane))
        {
            double* center[3] = { &workCenterX, &workCenterY, &workCenterZ };
            *center[plane.u] = display.workRotationCenterMM[0];
            *center[plane.v] = display.workRotationCenterMM[1];
        }
        else
            workCenterX = workCenterY = workCenterZ =
                (std::numeric_limits<double>::quiet_NaN)();
        yaw = display.workOffset[CoordinateManager::WO_ANGLE_XY_YAW];
        pitch = display.workOffset[CoordinateManager::WO_ANGLE_XZ_PITCH];
        roll = display.workOffset[CoordinateManager::WO_ANGLE_YZ_ROLL];
    }
    else if (CoordSys.isWorkpieceRotationActive && CoordSys.currentWCode > 0 && CoordSys.currentWCode <= CoordSys.m_WorkOffset.size()) {
        int wIdx = CoordSys.currentWCode - 1;
        yaw = CoordSys.m_WorkOffset[wIdx][CoordinateManager::WO_ANGLE_XY_YAW];
        pitch = CoordSys.m_WorkOffset[wIdx][CoordinateManager::WO_ANGLE_XZ_PITCH];
        roll = CoordSys.m_WorkOffset[wIdx][CoordinateManager::WO_ANGLE_YZ_ROLL];
    }
    MacroSys.SetVar('$', 160, workCenterX * unitScale);
    MacroSys.SetVar('$', 161, workCenterY * unitScale);
    MacroSys.SetVar('$', 162, workCenterZ * unitScale);
    MacroSys.SetVar('$', 163, yaw);
    MacroSys.SetVar('$', 164, pitch);
    MacroSys.SetVar('$', 165, roll);

    // =========================================================
    // 8. G51 縮放倍率 ($166)
    // =========================================================
    // Keep this parameter on the same immutable frame as WCS during RESET.
    const double displayScale = displayFrozen ? display.scalingFactor :
        (CoordSys.isScalingActive ? CoordSys.scaleFactor : 1.0);
    MacroSys.SetVar('$', 166, displayScale);

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