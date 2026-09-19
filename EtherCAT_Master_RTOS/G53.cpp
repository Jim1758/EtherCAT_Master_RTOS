#include "GMCodeHandlers.h"
#include "AlarmManager.h"
#include "MotionCore.h"
#include <cmath>
#include <cstring>
#include <vector>
#include <windows.h>
#include <rtapi.h>

namespace GCodeHandlers
{
    namespace
    {
        // Pure whole-block decode. Outputs remain unpublished on any failure.
        // Native absolute words bypass transforms; a frozen frame remains provenance.
        bool DecodeG53(const NCBlock& block, NCManager* nc,
            bool* fields, double* nativeTargets, int& alarm, int& alarmAxis)
        {
            alarm = AlarmManager::G_Code_Invalid_parameter;
            alarmAxis = -1;
            const int count = block.gCount > 0 ? block.gCount : (block.hasG ? 1 : 0);
            if (nc == nullptr || fields == nullptr || nativeTargets == nullptr ||
                block.gCount < 0 || !block.hasG || count != 1 || block.gCode != 53 ||
                (block.gCount > 0 && block.gCodes[0] != 53) ||
                (block.has('G') && block.val('G') != 53.0) ||
                block.mCount != 0 || block.isGoto || block.isBlockSkip) return false;
            const CoordinateManager& coord = nc->CoordSys;
            const int scopeAlarm = nc->GetG53NativeScopeAlarmSameThread();
            if (scopeAlarm != 0) { alarm = scopeAlarm; return false; }
            int owners[26];
            for (unsigned letter = 0U; letter < 26U; ++letter) owners[letter] = -1;
            for (int axis = 0; axis < 8; ++axis)
            {
                const AxisContext& native = nc->GetMotion().GetAxisContext(axis);
                if (!native.isExist) continue;
                const char letter = nc->m_axisNames[axis];
                if (native.axisIndex != axis ||
                    (native.axisType != AxisType::LINEAR && native.axisType != AxisType::ROTARY &&
                        native.axisType != AxisType::ROTARY_CONTINUOUS) ||
                    letter < 'A' || letter > 'Z' || letter == 'G' || letter == 'N' ||
                    letter == 'M' || letter == 'T' || letter == 'H' || letter == 'D' ||
                    letter == 'F' || letter == 'P' || letter == 'Q' || letter == 'L' ||
                    owners[letter - 'A'] >= 0) return false;
                owners[letter - 'A'] = axis;
            }
            bool selected[8] = {};
            double targets[8] = {};
            bool any = false;
            // All words must have an owner before any home/limit check or motion.
            for (char letter = 'A'; letter <= 'Z'; ++letter)
            {
                if (!block.has(letter)) continue;
                if (!std::isfinite(block.val(letter))) return false;
                if (letter == 'G' || letter == 'N') continue;
                const int axis = owners[letter - 'A'];
                if (axis < 0) return false;
                // The current fixed-frame consumer contract is XYZ linear.
                // The existing unfrozen native lane still owns configured axes 1-8.
                if (coord.IsTranslationRunFrozen() && axis > 2) return false;
                selected[axis] = true;
                const AxisContext& native = nc->GetMotion().GetAxisContext(axis);
                targets[axis] = coord.ToInternalUnit(block.val(letter), native.axisType != AxisType::LINEAR);
                if (!std::isfinite(targets[axis])) return false;
                any = true;
            }
            if (!any) return false;
            for (int axis = 0; axis < 8; ++axis)
            {
                if (!selected[axis]) continue;
                alarmAxis = axis;
                const AxisContext& native = nc->GetMotion().GetAxisContext(axis);
                if (!native.isHomed)
                {
                    alarm = AlarmManager::axis_is_not_Homed;
                    return false;
                }
                if (!coord.IsTargetWithinSoftwareTravelLimit(native, targets[axis]))
                {
                    alarm = coord.GetSoftwareTravelLimitAlarmCode(native, AlarmManager::PROGRAMMED_OVER_TRAVEL);
                    return false;
                }
            }
            std::memcpy(fields, selected, sizeof(selected));
            std::memcpy(nativeTargets, targets, sizeof(targets));
            return true;
        }

        void RejectG53(NCManager* nc, int alarm, int axis)
        {
            // One integer-only event per refused block; no cyclic diagnostic.
            RtPrintf("[G53][REJECT] alarm=%d axis=%d beforeCommit=1\n", alarm, axis);
            AlarmManager::GetInstance().Trigger(alarm, 0, axis);
            if (nc != nullptr) nc->ChangeState(NCState::HOLD);
        }

        bool CheckG53MotionDone(NCManager* nc)
        {
            return !nc->IsFeedHoldActive() && nc->GetMotion().IsGroupDone();
        }
    }

    bool ValidateG53Block(const NCBlock& block, NCManager* nc)
    {
        bool fields[8] = {};
        double targets[8] = {};
        int alarm = 0, axis = -1;
        if (DecodeG53(block, nc, fields, targets, alarm, axis)) return true;
        RejectG53(nc, alarm, axis);
        return false;
    }

    WaitConditionFunc Handle_G53(const NCBlock& block, NCManager* nc)
    {
        bool fields[8] = {};
        double targets[8] = {};
        int alarm = 0, alarmAxis = -1;
        if (!DecodeG53(block, nc, fields, targets, alarm, alarmAxis))
        {
            RejectG53(nc, alarm, alarmAxis);
            return nullptr;
        }
        std::vector<int> axes;
        std::vector<double> nativeTargets;
        for (int axis = 0; axis < 8; ++axis)
            if (fields[axis]) { axes.push_back(axis); nativeTargets.push_back(targets[axis]); }
        if (!nc->GetMotion().TryG53MoveTransactionalTail(axes, nativeTargets, nc->CoordSys.commandedMCS))
        {
            if (!AlarmManager::GetInstance().HasAlarm())
                AlarmManager::GetInstance().Trigger(AlarmManager::PATH_EXECUTION_NOT_READY);
            nc->ChangeState(NCState::HOLD);
            return nullptr;
        }
        nc->CommitG53NativeHandoffSameThread();
        return CheckG53MotionDone;
    }
}
