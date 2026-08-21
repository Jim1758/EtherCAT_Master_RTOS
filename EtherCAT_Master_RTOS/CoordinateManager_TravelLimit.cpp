// ============================================================
// CoordinateManager_TravelLimit.cpp
//
// Software Travel Limit / Stored Stroke Check
//
// 功能歸屬：CoordinateManager
//
// 負責：
//
// 1. G22 / G23 Modal State
// 2. Software Travel Limit 1
// 3. Software Travel Limit 2
// 4. Software Travel Limit 3
// 5. Automatic Target Pre-Check
// 6. Manual Direction Permission
//
// 不負責：
//
// - Physical +OT / -OT
// - Alarm
// - Motion Stop
// - HOME Limit Sequence
//
// ============================================================

#include "CoordinateManager.h"
#include "MotionCore.h"
#include <cmath>
#include "NCManager.h"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include "GlobalConfig.h"
#include "EtherCatMaster.h"
#include "GlobalConfig.h" // 如果你有用到 DEBUG_PRINT 等功能
namespace
{
    // ========================================================
    // Travel Limit Range Validation
    //
    // 正常行程範圍必須：
    //
    // Negative Limit < Positive Limit
    //
    // 例如：
    //
    // -10.0 ~ +500.0
    //
    // Enable=true 但範圍錯誤時，
    // 目前先視為無效範圍。
    //
    // 正式 Parameter Alarm 後續再加入。
    // ========================================================
    bool IsValidTravelLimitRange(
        double negativeLimit,
        double positiveLimit)
    {
        if (!std::isfinite(negativeLimit) ||
            !std::isfinite(positiveLimit))
        {
            return false;
        }

        return negativeLimit < positiveLimit;
    }


    // ========================================================
    // Get Actual Machine Position
    //
    // AxisContext.currentActPos：
    //     Pulse
    //
    // 轉換為：
    //
    // Linear Axis:
    //     mm
    //
    // Rotary Axis:
    //     degree
    //
    // --------------------------------------------------------
    // 為什麼不用 CoordinateManager::actualMCS[]：
    //
    // 現有 MotionCore 對 Rotary Axis 顯示座標可能做
    // modulo 360。
    //
    // Software Travel Limit 必須使用真正展開後的位置，
    // 不能因為 370 degree 顯示成 10 degree
    // 就失去行程保護。
    // ========================================================
    double GetActualMachinePositionUnit(
        const AxisContext& axis)
    {
        if (axis.resolution_PPR <= 0.0)
        {
            return 0.0;
        }

        return
            axis.currentActPos *
            (axis.finalLead / axis.resolution_PPR);
    }
}


// ============================================================
// G22 / G23
// ============================================================

void CoordinateManager::SetStoredStrokeCheckMode( int gCode,NCManager* nc)
{
    // 目前不使用 NCManager。
    // 之後若要做 Alarm / System Variable / Log
    // 可以直接從這個入口擴充。
    (void)nc;


    switch (gCode)
    {
    case 22:

        // G22
        //
        // Enable Programmable Travel Limit 1
        m_programmableTravelLimitEnabled = true;
        nc->MacroSys.SetVar('$', 4, 22.0);
        break;


    case 23:

        // G23
        //
        // Disable Programmable Travel Limit 1
        m_programmableTravelLimitEnabled = false;
        nc->MacroSys.SetVar('$', 4, 23.0);
        break;


    default:

        // 這個 API 只接受 G22 / G23。
        //
        // 不在 CoordinateManager 產生 G-Code Alarm。
        // Unsupported G-Code 仍由 NCManager / Parser 負責。


        break;
    }
}


// ============================================================
// Direct Runtime State Setter
//
// 給未來 Power-On Parameter / Reset 使用。
// ============================================================

void CoordinateManager::SetProgrammableTravelLimitEnabled(
    bool enabled, NCManager* nc)
{
    m_programmableTravelLimitEnabled =
        enabled;

    if (enabled == true)
    {
        nc->MacroSys.SetVar('$', 4, 22.0);
    }
    else
    {
        nc->MacroSys.SetVar('$', 4, 23.0);
    }



}


// ============================================================
// G22 / G23 State Query
// ============================================================

bool CoordinateManager::IsProgrammableTravelLimitEnabled() const
{
    return
        m_programmableTravelLimitEnabled;
}


// ============================================================
// Update Software Travel Limit Runtime State
// ============================================================

void CoordinateManager::UpdateSoftwareTravelLimitState(
    AxisContext& axis) const
{
    // ========================================================
    // 1. 每次 Scan 先清除舊狀態
    // ========================================================

    axis.travelLimit1PositiveActive = false;
    axis.travelLimit1NegativeActive = false;

    axis.travelLimit2PositiveActive = false;
    axis.travelLimit2NegativeActive = false;

    axis.travelLimit3PositiveActive = false;
    axis.travelLimit3NegativeActive = false;


    // ========================================================
    // 2. 不存在的軸不做判斷
    // ========================================================

    if (!axis.isExist)
    {
        return;
    }


    // ========================================================
    // 3. 尚未建立 Machine Coordinate 時不啟用 Soft Limit
    //
    // Software Travel Limit 是 Machine Coordinate Protection。
    //
    // HOME 尚未完成以前，
    // Machine Zero 尚未可靠建立，
    // 不應拿錯誤座標做 Software Limit 判斷。
    //
    // 注意：
    // 你現在 axis.isHomed 預設仍是 true，
    // 所以目前既有系統行為不會因此改變。
    //
    // HOME 正式完成後再把 Power-On 預設改成 false。
    // ========================================================

    if (!axis.isHomed)
    {
        return;
    }


    // ========================================================
    // 4. 取得真正 Machine Position
    //
    // Linear = mm
    // Rotary = degree
    // ========================================================

    const double currentPosition =
        GetActualMachinePositionUnit(axis);


    // ========================================================
    // 5. Software Travel Limit 1
    //
    // Parameter Enable
    // +
    // G22 / G23
    // ========================================================

    if (axis.travelLimit1Enable &&
        m_programmableTravelLimitEnabled &&
        IsValidTravelLimitRange(
            axis.travelLimit1Negative_unit,
            axis.travelLimit1Positive_unit))
    {
        if (currentPosition >=
            axis.travelLimit1Positive_unit)
        {
            axis.travelLimit1PositiveActive =
                true;
        }

        if (currentPosition <=
            axis.travelLimit1Negative_unit)
        {
            axis.travelLimit1NegativeActive =
                true;
        }
    }


    // ========================================================
    // 6. Software Travel Limit 2
    //
    // Parameter Enable Only
    //
    // 不受 G22 / G23 影響。
    // ========================================================

    if (axis.travelLimit2Enable &&
        IsValidTravelLimitRange(
            axis.travelLimit2Negative_unit,
            axis.travelLimit2Positive_unit))
    {
        if (currentPosition >=
            axis.travelLimit2Positive_unit)
        {
            axis.travelLimit2PositiveActive =
                true;
        }

        if (currentPosition <=
            axis.travelLimit2Negative_unit)
        {
            axis.travelLimit2NegativeActive =
                true;
        }
    }


    // ========================================================
    // 7. Software Travel Limit 3
    //
    // Parameter Enable Only
    //
    // 不受 G22 / G23 影響。
    // ========================================================

    if (axis.travelLimit3Enable &&
        IsValidTravelLimitRange(
            axis.travelLimit3Negative_unit,
            axis.travelLimit3Positive_unit))
    {
        if (currentPosition >=
            axis.travelLimit3Positive_unit)
        {
            axis.travelLimit3PositiveActive =
                true;
        }

        if (currentPosition <=
            axis.travelLimit3Negative_unit)
        {
            axis.travelLimit3NegativeActive =
                true;
        }
    }
}


// ============================================================
// Automatic Target Pre-Check
//
// Target 使用 Machine Coordinate Unit：
//
// Linear = mm
// Rotary = degree
//
// 邊界本身允許到達：
//
// target == Positive Limit
// target == Negative Limit
//
// 只有真正超出去才回傳 false。
// ============================================================

bool CoordinateManager::IsTargetWithinSoftwareTravelLimit(
    const AxisContext& axis,
    double targetMCS) const
{
    // ========================================================
    // 1. 防呆
    // ========================================================

    if (!axis.isExist)
    {
        return true;
    }


    if (!std::isfinite(targetMCS))
    {
        return false;
    }


    // ========================================================
    // 2. HOME 前不使用 Software Travel Limit
    // ========================================================

    if (!axis.isHomed)
    {
        return true;
    }


    // ========================================================
    // 3. Travel Limit 1
    //
    // Parameter Enable + G22/G23
    // ========================================================

    if (axis.travelLimit1Enable &&
        m_programmableTravelLimitEnabled &&
        IsValidTravelLimitRange(
            axis.travelLimit1Negative_unit,
            axis.travelLimit1Positive_unit))
    {
        if (targetMCS <
            axis.travelLimit1Negative_unit ||
            targetMCS >
            axis.travelLimit1Positive_unit)
        {
            return false;
        }
    }


    // ========================================================
    // 4. Travel Limit 2
    // ========================================================

    if (axis.travelLimit2Enable &&
        IsValidTravelLimitRange(
            axis.travelLimit2Negative_unit,
            axis.travelLimit2Positive_unit))
    {
        if (targetMCS <
            axis.travelLimit2Negative_unit ||
            targetMCS >
            axis.travelLimit2Positive_unit)
        {
            return false;
        }
    }


    // ========================================================
    // 5. Travel Limit 3
    // ========================================================

    if (axis.travelLimit3Enable &&
        IsValidTravelLimitRange(
            axis.travelLimit3Negative_unit,
            axis.travelLimit3Positive_unit))
    {
        if (targetMCS <
            axis.travelLimit3Negative_unit ||
            targetMCS >
            axis.travelLimit3Positive_unit)
        {
            return false;
        }
    }


    return true;
}


// ============================================================
// Manual Positive Direction Permission
//
// Software Limit 到達 + Side 時：
//
// + Direction = Block
// - Direction = Allowed
//
// 注意：
// 這裡完全不看 Physical +OT。
// Physical Limit 之後由 NCPLCManager 再疊加。
// ============================================================

bool CoordinateManager::CanMoveSoftwarePositive(
    const AxisContext& axis) const
{
    if (axis.travelLimit1PositiveActive)
    {
        return false;
    }

    if (axis.travelLimit2PositiveActive)
    {
        return false;
    }

    if (axis.travelLimit3PositiveActive)
    {
        return false;
    }

    return true;
}


// ============================================================
// Manual Negative Direction Permission
//
// Software Limit 到達 - Side 時：
//
// - Direction = Block
// + Direction = Allowed
// ============================================================

bool CoordinateManager::CanMoveSoftwareNegative(
    const AxisContext& axis) const
{
    if (axis.travelLimit1NegativeActive)
    {
        return false;
    }

    if (axis.travelLimit2NegativeActive)
    {
        return false;
    }

    if (axis.travelLimit3NegativeActive)
    {
        return false;
    }

    return true;
}