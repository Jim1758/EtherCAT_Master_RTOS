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
#include "AlarmManager.h"
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
    // Active ranges require finite endpoints and negative < positive.
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


    // Configuration-only query: HOME state never weakens configured ranges.
    // Callers retain their own existence / HOME policy without copying axes.
    unsigned GetInvalidConfiguredSoftwareTravelLimitMask(
        const AxisContext& axis, bool strokeEnabled)
    {
        const bool enabled[3] = { axis.travelLimit1Enable && strokeEnabled,
            axis.travelLimit2Enable, axis.travelLimit3Enable };
        const double negative[3] = { axis.travelLimit1Negative_unit,
            axis.travelLimit2Negative_unit, axis.travelLimit3Negative_unit };
        const double positive[3] = { axis.travelLimit1Positive_unit,
            axis.travelLimit2Positive_unit, axis.travelLimit3Positive_unit };
        unsigned invalidMask = 0U;
        unsigned validCount = 0U;
        double intersectionNegative = 0.0;
        double intersectionPositive = 0.0;
        for (unsigned group = 0U; group < 3U; ++group)
        {
            if (!enabled[group]) continue;
            if (!IsValidTravelLimitRange(negative[group], positive[group]))
            {
                invalidMask |= 1U << group;
                continue;
            }
            if (validCount == 0U)
            {
                intersectionNegative = negative[group];
                intersectionPositive = positive[group];
            }
            else
            {
                if (negative[group] > intersectionNegative)
                    intersectionNegative = negative[group];
                if (positive[group] < intersectionPositive)
                    intersectionPositive = positive[group];
            }
            ++validCount;
        }
        // Inclusive target limits retain a shared single point as valid.
        if (validCount > 1U && intersectionNegative > intersectionPositive)
            invalidMask |= 8U;
        return invalidMask;
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

void CoordinateManager::SetStoredStrokeCheckMode(int gCode, NCManager* nc)
{
    if (gCode != 22 && gCode != 23) return;
    const bool enabled = gCode == 22;
    const bool changed = m_programmableTravelLimitEnabled != enabled;
    if (changed && !GuardCoordinateMutation("STROKE_MODE", nc)) return;
    // The normal dispatcher repeats the exact selector after a drained commit.
    // A no-op must still prove the complete frozen source was not changed.
    if (!changed && m_translationFrozen && !m_translationResetBypass && !IsTranslationRunCurrent())
    {
        RejectCoordinateMutation("STROKE_MODE", "SOURCE_CHANGED", nc, true);
        return;
    }
    m_programmableTravelLimitEnabled = enabled;
    if (nc != nullptr) nc->MacroSys.SetVar('$', 4, static_cast<double>(gCode));
}

// Direct power-on/runtime entry obeys the same mutation and optional-NC rules.
void CoordinateManager::SetProgrammableTravelLimitEnabled(bool enabled, NCManager* nc)
{
    SetStoredStrokeCheckMode(enabled ? 22 : 23, nc);
}


// ============================================================
// G22 / G23 State Query
// ============================================================

bool CoordinateManager::IsProgrammableTravelLimitEnabled() const
{
    return
        m_programmableTravelLimitEnabled;
}


// Pure configuration validation. Never publish an Alarm from a coordinate query.
unsigned CoordinateManager::GetInvalidSoftwareTravelLimitMask(
    const AxisContext& axis, int storedStrokeMode) const
{
    if (!axis.isExist || !axis.isHomed) return 0U;
    const bool strokeEnabled = storedStrokeMode == 0 ?
        m_programmableTravelLimitEnabled : storedStrokeMode == 22;
    return GetInvalidConfiguredSoftwareTravelLimitMask(axis, strokeEnabled);
}

int CoordinateManager::GetSoftwareTravelLimitAlarmCode(
    const AxisContext& axis, int fallbackAlarmCode) const
{
    return GetInvalidSoftwareTravelLimitMask(axis) != 0U ?
        static_cast<int>(AlarmManager::SOFTWARE_TRAVEL_LIMIT_INVALID_CONFIG) : fallbackAlarmCode;
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

    const unsigned invalidMask = GetInvalidSoftwareTravelLimitMask(axis);
    const bool emptyIntersection = (invalidMask & 8U) != 0U;
    if ((invalidMask & 1U) != 0U ||
        (emptyIntersection && axis.travelLimit1Enable && m_programmableTravelLimitEnabled))
    {
        axis.travelLimit1PositiveActive = true;
        axis.travelLimit1NegativeActive = true;
    }
    if ((invalidMask & 2U) != 0U || (emptyIntersection && axis.travelLimit2Enable))
    {
        axis.travelLimit2PositiveActive = true;
        axis.travelLimit2NegativeActive = true;
    }
    if ((invalidMask & 4U) != 0U || (emptyIntersection && axis.travelLimit3Enable))
    {
        axis.travelLimit3PositiveActive = true;
        axis.travelLimit3NegativeActive = true;
    }


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

    return IsTargetWithinConfiguredSoftwareTravelLimit(axis, targetMCS);
}


// Explicit configuration query for targets that will be used after HOME.
// It never marks an axis homed and never publishes an alarm.
bool CoordinateManager::IsTargetWithinConfiguredSoftwareTravelLimit(
    const AxisContext& axis,
    double targetMCS) const
{
    if (!axis.isExist) return true;
    if (!std::isfinite(targetMCS)) return false;
    if (GetInvalidConfiguredSoftwareTravelLimitMask(
        axis, m_programmableTravelLimitEnabled) != 0U) return false;


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
    if (GetInvalidSoftwareTravelLimitMask(axis) != 0U) return false;

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
    if (GetInvalidSoftwareTravelLimitMask(axis) != 0U) return false;

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