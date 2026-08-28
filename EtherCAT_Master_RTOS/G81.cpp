#include "GMCodeHandlers.h"
#include "NCManager.h"
#include "MotionCore.h"
#include "AlarmManager.h"
#include <cmath>

namespace GCodeHandlers
{
    static bool CheckG81Done(NCManager* nc)
    {
        if (nc == nullptr) return true;
        if (AlarmManager::GetInstance().HasAlarm() || nc->GetState() == NCState::ALARM || nc->Homing.HasError()) return true;
        return !nc->Homing.IsActive();
    }

    WaitConditionFunc Handle_G81(const NCBlock& block, NCManager* nc)
    {
        if (nc == nullptr) return nullptr;
        if (nc->Homing.IsActive()) return CheckG81Done;

        int sequence = 0;
        if (block.has('P'))
        {
            const double p = block.val('P');
            sequence = static_cast<int>(p);
            if ((sequence != 0 && sequence != 1) || std::abs(p - static_cast<double>(sequence)) > 1.0e-9)
            {
                AlarmManager::GetInstance().Trigger(AlarmManager::G_Code_Invalid_parameter);
                nc->GetMotion().RequestEmergencyStopAllAxes();
                nc->ChangeState(NCState::ALARM);
                return [](NCManager*) { return true; };
            }
        }

        uint8_t axisMask = 0;
        bool hasAxisWord = false;
        for (int i = 0; i < 8; ++i)
        {
            const char letter = nc->m_axisNames[i];
            if (letter == ' ' || letter == '\0' || letter == 'N') continue;
            if (!block.has(letter)) continue;
            hasAxisWord = true;
            if (std::abs(block.val(letter)) > 1.0e-12)
                axisMask |= static_cast<uint8_t>(1u << i);
        }

        if (hasAxisWord && axisMask == 0)
        {
            AlarmManager::GetInstance().Trigger(AlarmManager::G_Code_Invalid_parameter);
            nc->GetMotion().RequestEmergencyStopAllAxes();
            nc->ChangeState(NCState::ALARM);
            return [](NCManager*) { return true; };
        }

        HomeRequest request{};
        request.axisMask = axisMask; // 0 = ©Ò¦³ HomeEnable ¶b
        request.sequenceMode = sequence == 1 ? HomeSequenceMode::BY_ORDER : HomeSequenceMode::SIMULTANEOUS;

        if (!nc->Homing.Start(request))
        {
            const HomeErrorReason reason = nc->Homing.GetLastError();
            const int alarmCode = (reason == HomeErrorReason::SERVO_NOT_READY || reason == HomeErrorReason::MOTION_BUSY || reason == HomeErrorReason::SERVO_FAULT || reason == HomeErrorReason::MOTION_FAULT)
                ? AlarmManager::HOME_MOTION_FAULT
                : AlarmManager::HOME_INVALID_CONFIG;
            if (!AlarmManager::GetInstance().HasAlarm())
                AlarmManager::GetInstance().Trigger(alarmCode, 0, nc->Homing.GetLastErrorAxis());
            nc->GetMotion().RequestEmergencyStopAllAxes();
            nc->ChangeState(NCState::ALARM);
            return [](NCManager*) { return true; };
        }

        return CheckG81Done;
    }
}
