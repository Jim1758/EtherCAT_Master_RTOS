#include "GMCodeHandlers.h"
#include "PositioningBlockValidation.h"
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
        bool DecodePositioningTarget(const NCBlock& block, NCManager* nc,
            int profileCode, NCPositioningBlockTarget& decoded,
            double (&nativeTargets)[8], int& alarm, int& alarmAxis)
        {
            alarm = AlarmManager::G_Code_Invalid_parameter;
            alarmAxis = -1;
            if (nc == nullptr) return false;
            CoordinateManager& coord = nc->CoordSys;
            // This stage repairs neutral positioning; the existing audited
            // G00/G01 producers retain ownership of transformed geometry.
            if (coord.activePlane != 17 || !coord.isAbsoluteMode || coord.isInchMode ||
                coord.isPolarCoordinateActive || coord.toolRadiusMode != 40 ||
                coord.isG68Active || coord.isWorkpieceRotationActive ||
                coord.currentWCode != 0 || coord.IsScaleMirrorActive()) return false;
            const int scopeAlarm = nc->GetG53NativeScopeAlarmSameThread();
            if (scopeAlarm != 0) { alarm = scopeAlarm; return false; }

            NCPositioningAxisWord words[8] = {};
            for (int axis = 0; axis < 8; ++axis)
            {
                const AxisContext& native = nc->GetMotion().GetAxisContext(axis);
                words[axis].address = nc->m_axisNames[axis];
                words[axis].enabled = native.isExist;
                words[axis].identityValid = native.axisIndex == axis &&
                    (native.axisType == AxisType::LINEAR || native.axisType == AxisType::ROTARY ||
                        native.axisType == AxisType::ROTARY_CONTINUOUS);
            }
            NCPositioningBlockTarget candidate{};
            if (!TryDecodeNCPositioningBlock(block, profileCode, words,
                coord.IsTranslationRunFrozen(), candidate, alarmAxis))
            {
                if (alarmAxis >= 0) alarm = AlarmManager::axis_is_not_enabledr;
                return false;
            }

            // G21 words already use native mm/degrees. Preview never commits
            // commandedMCS, including a rejected limit, mapping or queue request.
            double targets[8] = {};
            coord.Preview_WCS_to_MCS(candidate.authored, candidate.selected, targets);
            for (int axis = 0; axis < 8; ++axis)
            {
                if (!candidate.selected[axis]) continue;
                alarmAxis = axis;
                const AxisContext& native = nc->GetMotion().GetAxisContext(axis);
                if (!std::isfinite(targets[axis])) return false;
                const unsigned invalidMask = coord.GetInvalidSoftwareTravelLimitMask(native);
                if (invalidMask != 0U)
                {
                    alarm = AlarmManager::SOFTWARE_TRAVEL_LIMIT_INVALID_CONFIG;
                    return false;
                }
                if (!coord.IsTargetWithinSoftwareTravelLimit(native, targets[axis]))
                {
                    alarm = coord.GetSoftwareTravelLimitAlarmCode(native, AlarmManager::PROGRAMMED_OVER_TRAVEL);
                    return false;
                }
            }
            decoded = candidate;
            std::memcpy(nativeTargets, targets, sizeof(targets));
            return true;
        }

        void RejectPositioning(NCManager* nc, int profileCode, int alarm, int axis)
        {
            RtPrintf("[POSITIONING][BASE41][REJECT] g=%d alarm=%d axis=%d beforeCommit=1\n",
                profileCode, alarm, axis);
            AlarmManager::GetInstance().Trigger(alarm, 0, axis);
            if (nc != nullptr) nc->ChangeState(NCState::HOLD);
        }

        bool CheckPositioningMotionDone(NCManager* nc)
        {
            return !nc->IsFeedHoldActive() && nc->GetMotion().IsGroupDone();
        }
    }

    bool ValidatePositioningBlock(const NCBlock& block, NCManager* nc, int profileCode)
    {
        NCPositioningBlockTarget decoded{};
        double targets[8] = {};
        int alarm = 0, axis = -1;
        if (DecodePositioningTarget(block, nc, profileCode, decoded, targets, alarm, axis)) return true;
        RejectPositioning(nc, profileCode, alarm, axis);
        return false;
    }

    WaitConditionFunc HandlePositioningBlock(const NCBlock& block, NCManager* nc, int profileCode)
    {
        NCPositioningBlockTarget decoded{};
        double targets[8] = {};
        int alarm = 0, axis = -1;
        if (!DecodePositioningTarget(block, nc, profileCode, decoded, targets, alarm, axis))
        {
            RejectPositioning(nc, profileCode, alarm, axis);
            return nullptr;
        }
        std::vector<int> axes;
        std::vector<double> nativeTargets;
        for (int slot = 0; slot < 8; ++slot)
            if (decoded.selected[slot]) { axes.push_back(slot); nativeTargets.push_back(targets[slot]); }
        if (!nc->GetMotion().TryPositioningMoveTransactionalTail(
            axes, nativeTargets, profileCode, nc->CoordSys.commandedMCS))
        {
            if (!AlarmManager::GetInstance().HasAlarm())
                AlarmManager::GetInstance().Trigger(AlarmManager::PATH_EXECUTION_NOT_READY);
            nc->ChangeState(NCState::HOLD);
            return nullptr;
        }
        nc->MacroSys.SetVar('$', 1, profileCode);
        nc->CommitPositioningHandoffSameThread(profileCode);
        return CheckPositioningMotionDone;
    }

    WaitConditionFunc Handle_G07(const NCBlock& block, NCManager* nc)
    {
        return HandlePositioningBlock(block, nc, 7);
    }
}
