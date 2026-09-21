#include "GMCodeHandlers.h"
#include "NCManager.h"
#include "MotionCore.h"
#include "AlarmManager.h"
#include <cmath>
#include <windows.h> // RTX API declarations require Windows base types first.
#include <rtapi.h>

namespace GCodeHandlers
{
    namespace
    {
        // BASE41: decode the complete block before HOME selection or any
        // same-line setting, tool or M-code can take effect. Only an actual
        // omission of all axis words means ALL enabled HOME axes.
        bool DecodeG81(const NCBlock& block, NCManager* nc, HomeRequest& request)
        {
            const int count = block.gCount > 0 ? block.gCount : (block.hasG ? 1 : 0);
            if (nc == nullptr || block.gCount < 0 || !block.hasG || count != 1 ||
                block.gCode != 81 || (block.gCount > 0 && block.gCodes[0] != 81) ||
                (block.has('G') && block.val('G') != 81.0) || block.mCount != 0 ||
                block.isGoto || block.isBlockSkip) return false;
            // HOME changes the machine origin and homed travel policy. It
            // must precede the first frozen motion in a fresh program run.
            // Reject here instead of changing HOME state and faulting the
            // immutable coordinate descriptor on the following NC pass.
            if (nc->CoordSys.IsTranslationRunFrozen()) return false;

            int owners[26];
            for (unsigned i = 0U; i < 26U; ++i) owners[i] = -1;
            for (int i = 0; i < 8; ++i)
            {
                const AxisContext& axis = nc->GetMotion().GetAxisContext(i);
                if (!axis.isExist) continue;
                const char letter = nc->m_axisNames[i];
                if (axis.axisIndex != i ||
                    (axis.axisType != AxisType::LINEAR && axis.axisType != AxisType::ROTARY &&
                        axis.axisType != AxisType::ROTARY_CONTINUOUS) ||
                    letter < 'A' || letter > 'Z' || letter == 'G' || letter == 'N' ||
                    letter == 'M' || letter == 'T' || letter == 'H' || letter == 'D' ||
                    letter == 'F' || letter == 'P' || letter == 'Q' || letter == 'L' ||
                    owners[letter - 'A'] >= 0) return false;
                owners[letter - 'A'] = i;
            }

            uint8_t axisMask = 0U;
            bool hasAxisWord = false;
            HomeSequenceMode sequence = HomeSequenceMode::SIMULTANEOUS;
            for (char letter = 'A'; letter <= 'Z'; ++letter)
            {
                if (!block.has(letter)) continue;
                const double value = block.val(letter);
                if (!std::isfinite(value)) return false;
                if (letter == 'G' || letter == 'N') continue;
                if (letter == 'P')
                {
                    // Discrete modes need no floating-to-integer conversion.
                    if (value != 0.0 && value != 1.0) return false;
                    sequence = value == 1.0 ? HomeSequenceMode::BY_ORDER :
                        HomeSequenceMode::SIMULTANEOUS;
                    continue;
                }
                const int axis = owners[letter - 'A'];
                if (axis < 0) return false;
                hasAxisWord = true;
                // Preserve the existing selection flag convention, including
                // its zero tolerance; these values are not axis destinations.
                if (std::abs(value) > 1.0e-12)
                    axisMask |= static_cast<uint8_t>(1u << axis);
            }
            if (hasAxisWord && axisMask == 0U) return false;

            HomeRequest decoded{};
            decoded.axisMask = axisMask;
            decoded.sequenceMode = sequence;
            request = decoded;
            return true;
        }

        void RejectG81(NCManager* nc, int alarm, int axis = -1)
        {
            if (nc == nullptr) return;
            RtPrintf("[G81][REJECT] alarm=%d axis=%d beforeStart=1\n", alarm, axis);
            if (!AlarmManager::GetInstance().HasAlarm())
                AlarmManager::GetInstance().Trigger(alarm, 0, axis);
            nc->GetMotion().RequestEmergencyStopAllAxes();
            nc->ChangeState(NCState::ALARM);
        }

        bool CheckG81Done(NCManager* nc)
        {
            if (nc == nullptr) return true;
            if (AlarmManager::GetInstance().HasAlarm() || nc->GetState() == NCState::ALARM ||
                nc->Homing.HasError()) return true;
            // CompleteRequest can finish the axes while its final probe
            // disarm acknowledgement still owns HOME. Do not advance the NC
            // block until the existing HOME owner hand-off has completed.
            return !nc->Homing.IsActive() &&
                nc->GetMotion().GetMotionOwnerLease().owner != MotionOwner::HOME;
        }
    }

    bool ValidateG81Block(const NCBlock& block, NCManager* nc)
    {
        HomeRequest request{};
        if (DecodeG81(block, nc, request)) return true;
        RejectG81(nc, AlarmManager::G_Code_Invalid_parameter);
        return false;
    }

    WaitConditionFunc Handle_G81(const NCBlock& block, NCManager* nc)
    {
        if (nc == nullptr) return nullptr;
        HomeRequest request{};
        if (!DecodeG81(block, nc, request))
        {
            RejectG81(nc, AlarmManager::G_Code_Invalid_parameter);
            return [](NCManager*) { return true; };
        }
        if (nc->Homing.IsActive()) return CheckG81Done;

        if (!nc->Homing.Start(request))
        {
            const HomeErrorReason reason = nc->Homing.GetLastError();
            const int alarm = (reason == HomeErrorReason::SERVO_NOT_READY ||
                reason == HomeErrorReason::MOTION_BUSY || reason == HomeErrorReason::SERVO_FAULT ||
                reason == HomeErrorReason::MOTION_FAULT)
                ? AlarmManager::HOME_MOTION_FAULT : AlarmManager::HOME_INVALID_CONFIG;
            RejectG81(nc, alarm, nc->Homing.GetLastErrorAxis());
            return [](NCManager*) { return true; };
        }
        return CheckG81Done;
    }
}
