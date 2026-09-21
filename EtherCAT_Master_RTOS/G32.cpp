#include "ReferencePositionBlock.h"

namespace GCodeHandlers
{
    WaitConditionFunc Handle_G32(const NCBlock& block, NCManager* nc)
    {
        return ReferencePositionDetail::Handle(block, nc, 32);
    }
}
