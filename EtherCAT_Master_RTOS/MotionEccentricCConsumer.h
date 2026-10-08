#pragma once

#include "MotionEccentricCTransaction.h"

// BASE79F: one RT consumer, explicitly allocated before the cyclic threads.
// No decoded plan, transport workspace or staged executor lives on the RT
// stack or inside InterpolationGroup. No operation resizes this storage.
struct MotionEccentricCConsumerState
{
    MotionEccentricCTransportWorkspace transport{};
    NCEccentricCProfileValue decoded{};
    MotionEccentricCTransaction transaction{};
    MotionEccentricCTicket ticket{};
    MotionCommand dequeued{};
};
