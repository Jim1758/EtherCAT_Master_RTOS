#include "GMCodeHandlers.h"
#include "NCManager.h"
#include "NCGCodeSemantics.h"
#include "AlarmManager.h"
#include "NCSpindleSource.h"

namespace GCodeHandlers {
    WaitConditionFunc Handle_MCode(const NCBlock& block, NCManager* nc)
    {
        if (nc == nullptr) return nullptr;
        // EDM288: reserved spindle commands never enter the generic PLC AUX route.
        if (NCSpindleSource::HasReservedM(block))
            return nc->HandleRotarySpindleMCodeSameThread(block);
        if (block.mCount == 0) return nullptr;
        const int m = block.mCode[0];
        switch (m)
        {
        case 0: case 1: case 2: case 30: case 98: case 99:
            // NCManager owns these program-flow post-actions.
            return nullptr;
        default:
            break;
        }
        const bool toolRequest = block.has('T') &&
            !NCGCodeSemantics::Contains(block, 65) && !NCGCodeSemantics::Contains(block, 66);
        const int t = toolRequest ? static_cast<int>(block.val('T')) : -1;
        if (m < 0 || !nc->BeginAuxiliaryHandshakeSameThread(t, m))
        {
            AlarmManager::GetInstance().Trigger(AlarmManager::G_Code_Invalid_parameter);
            nc->ChangeState(NCState::ALARM);
        }
        return NCManager::WaitForAuxiliaryHandshakeCallback;
    }
}
