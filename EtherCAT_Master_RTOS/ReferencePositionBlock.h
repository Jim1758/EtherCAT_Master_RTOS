#pragma once
#include "GMCodeHandlers.h"
#include "AlarmManager.h"
#include "MotionCore.h"
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>
#include <windows.h>
#include <rtapi.h>

namespace GCodeHandlers
{
    namespace ReferencePositionDetail
    {
        struct Plan
        {
            bool selected[8] = {};
            double reference[8] = {};
            double intermediate[8] = {};
            std::uint32_t intermediateMask = 0U;
        };

        // BASE42: axis words are absolute native/MCS intermediate positions,
        // never a selected-axis return mask. Unspecified intermediate axes
        // retain their exact sampled pulses inside the atomic pair producer.
        inline bool Decode(const NCBlock& block, NCManager* nc, int code,
            Plan& output, int& alarm, int& alarmAxis, const char*& reason)
        {
            alarm = AlarmManager::G_Code_Invalid_parameter;
            alarmAxis = -1;
            reason = "BLOCK_SHAPE";
            const int count = block.gCount > 0 ? block.gCount : (block.hasG ? 1 : 0);
            if (nc == nullptr || (code != 28 && code != 30 && code != 32) ||
                block.gCount < 0 || !block.hasG || count != 1 || block.gCode != code ||
                (block.gCount > 0 && block.gCodes[0] != code) ||
                (block.has('G') && block.val('G') != static_cast<double>(code)) ||
                block.mCount != 0 || block.isGoto || block.isBlockSkip) return false;

            Plan candidate{};
            int owners[26];
            for (int& owner : owners) owner = -1;
            bool any = false;
            const CoordinateManager& coord = nc->CoordSys;
            // Validate the address owners before interpreting any axis word.
            // Disabled placeholder N/blank slots are not configured owners.
            for (int axis = 0; axis < 8; ++axis)
            {
                const AxisContext& native = nc->GetMotion().GetAxisContext(axis);
                const char letter = nc->m_axisNames[axis];
                if (!native.isExist && (letter == ' ' || letter == '\0' || letter == 'N')) continue;
                alarmAxis = axis;
                reason = "AXIS_MAPPING";
                if (letter < 'A' || letter > 'Z' ||
                    letter == 'G' || letter == 'N' || letter == 'M' || letter == 'T' ||
                    letter == 'H' || letter == 'D' || letter == 'F' || letter == 'P' ||
                    letter == 'Q' || letter == 'L' || owners[letter - 'A'] >= 0) return false;
                owners[letter - 'A'] = axis;
                if (!native.isExist) continue;
                if (native.axisIndex != axis ||
                    (native.axisType != AxisType::LINEAR && native.axisType != AxisType::ROTARY &&
                        native.axisType != AxisType::ROTARY_CONTINUOUS) ||
                    (coord.IsTranslationRunFrozen() && axis > 2)) return false;
                candidate.selected[axis] = true;
                any = true;
            }

            alarmAxis = -1;
            int referenceCode = 1;
            for (char word = 'A'; word <= 'Z'; ++word)
            {
                if (!block.has(word)) continue;
                const double value = block.val(word);
                reason = "NONFINITE_WORD";
                if (!std::isfinite(value)) return false;
                if (word == 'G' || word == 'N') continue;
                if (word == 'P')
                {
                    reason = "REFERENCE_P";
                    if (code == 28 || value < 1.0 || value > 100.0 ||
                        std::floor(value) != value) return false;
                    referenceCode = static_cast<int>(value);
                    continue;
                }
                const int axis = owners[word - 'A'];
                reason = "UNOWNED_WORD";
                if (axis < 0) return false;
                if (!candidate.selected[axis])
                {
                    reason = "AXIS_DISABLED";
                    alarm = AlarmManager::axis_is_not_enabledr;
                    alarmAxis = axis;
                    return false;
                }
                candidate.intermediate[axis] = value;
                candidate.intermediateMask |= (1U << static_cast<unsigned>(axis));
            }

            // The unfrozen native-scope query does not test feed hold.
            // A pair must never establish fresh motion while HOLD is active.
            if (candidate.intermediateMask != 0U && nc->IsFeedHoldActive())
            {
                reason = "FEED_HOLD";
                return false;
            }
            reason = "NEUTRAL_SCOPE";
            if (coord.activePlane != 17 || !coord.isAbsoluteMode || coord.isInchMode ||
                coord.isPolarCoordinateActive || coord.isCAxisOffsetRotationEnabled ||
                coord.toolLengthMode != 49 || coord.currentHCode != 0 ||
                coord.toolRadiusMode != 40 || coord.currentDCode != 0 ||
                coord.isG68Active || coord.isWorkpieceRotationActive || coord.currentWCode != 0 ||
                coord.IsScaleMirrorActive()) return false;
            const int scopeAlarm = nc->GetG53NativeScopeAlarmSameThread();
            if (scopeAlarm != 0) { alarm = scopeAlarm; return false; }
            if (!any) { reason = "NO_ENABLED_AXIS"; return false; }

            // GetRefPoint's legacy copy assumes an eight-field table row.
            // Validate storage before calling it; malformed rows never index it.
            reason = "REFERENCE_TABLE";
            const std::size_t row = static_cast<std::size_t>(referenceCode - 1);
            if (row >= coord.m_RefPoints.size() || coord.m_RefPoints[row].size() != 8U)
                return false;
            if (!coord.GetRefPoint(referenceCode, candidate.reference)) return false;
            for (unsigned axis = 0U; axis < 8U; ++axis)
                if (!std::isfinite(candidate.reference[axis])) return false;

            // Every enabled axis goes to the final reference, including a zero
            // leg. Both endpoints are checked before either can be published.
            for (int axis = 0; axis < 8; ++axis)
            {
                if (!candidate.selected[axis]) continue;
                const AxisContext& native = nc->GetMotion().GetAxisContext(axis);
                alarmAxis = axis;
                if (!native.isHomed)
                {
                    reason = "HOME_REQUIRED";
                    alarm = AlarmManager::axis_is_not_Homed;
                    return false;
                }
                if (!coord.IsTargetWithinSoftwareTravelLimit(native, candidate.reference[axis]))
                {
                    reason = "REFERENCE_TRAVEL";
                    alarm = coord.GetSoftwareTravelLimitAlarmCode(native, AlarmManager::PROGRAMMED_OVER_TRAVEL);
                    return false;
                }
                if ((candidate.intermediateMask & (1U << static_cast<unsigned>(axis))) != 0U &&
                    !coord.IsTargetWithinSoftwareTravelLimit(native, candidate.intermediate[axis]))
                {
                    reason = "INTERMEDIATE_TRAVEL";
                    alarm = coord.GetSoftwareTravelLimitAlarmCode(native, AlarmManager::PROGRAMMED_OVER_TRAVEL);
                    return false;
                }
            }
            output = candidate;
            return true;
        }

        inline void Reject(NCManager* nc, int code, int alarm, int axis, const char* reason)
        {
            // Producer-side only, one event per refused block.
            RtPrintf("[REFERENCE][REJECT] g=%d alarm=%d axis=%d reason=%s beforeCommit=1\n",
                code, alarm, axis, reason);
            AlarmManager::GetInstance().Trigger(alarm, 0, axis);
            if (nc != nullptr) nc->ChangeState(NCState::HOLD);
        }

        inline bool Validate(const NCBlock& block, NCManager* nc, int code)
        {
            Plan plan{};
            int alarm = 0, axis = -1;
            const char* reason = "BLOCK_SHAPE";
            if (Decode(block, nc, code, plan, alarm, axis, reason)) return true;
            Reject(nc, code, alarm, axis, reason);
            return false;
        }

        inline WaitConditionFunc Handle(const NCBlock& block, NCManager* nc, int code)
        {
            Plan plan{};
            int alarm = 0, alarmAxis = -1;
            const char* reason = "BLOCK_SHAPE";
            if (!Decode(block, nc, code, plan, alarm, alarmAxis, reason))
            {
                Reject(nc, code, alarm, alarmAxis, reason);
                return nullptr;
            }
            std::vector<int> axes;
            std::vector<double> nativeTargets;
            std::vector<double> intermediateTargets;
            for (int axis = 0; axis < 8; ++axis)
            {
                if (!plan.selected[axis]) continue;
                axes.push_back(axis);
                nativeTargets.push_back(plan.reference[axis]);
                intermediateTargets.push_back(plan.intermediate[axis]);
            }
            const bool accepted = plan.intermediateMask != 0U
                ? nc->GetMotion().TryReferencePositionPairTransactionalTail(
                    axes, intermediateTargets, nativeTargets, plan.intermediateMask,
                    code, nc->CoordSys.commandedMCS)
                : nc->GetMotion().TryPositioningMoveTransactionalTail(
                    axes, nativeTargets, code, nc->CoordSys.commandedMCS);
            if (!accepted)
            {
                if (!AlarmManager::GetInstance().HasAlarm())
                    AlarmManager::GetInstance().Trigger(AlarmManager::PATH_EXECUTION_NOT_READY);
                nc->ChangeState(NCState::HOLD);
                return nullptr;
            }
            nc->CommitPositioningHandoffSameThread(code);
            return [](NCManager* owner)
            {
                return !owner->IsFeedHoldActive() && owner->GetMotion().IsGroupDone();
            };
        }
    }
}
