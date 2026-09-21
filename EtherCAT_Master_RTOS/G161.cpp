#include "GMCodeHandlers.h"

namespace GCodeHandlers
{
    // Shared decoder and transactional neutral-positioning handler live in G07.cpp.
    WaitConditionFunc HandlePositioningBlock(const NCBlock& block, NCManager* nc, int profileCode);

    WaitConditionFunc Handle_G161(const NCBlock& block, NCManager* nc)
    {
        return HandlePositioningBlock(block, nc, 161);
    }
}
