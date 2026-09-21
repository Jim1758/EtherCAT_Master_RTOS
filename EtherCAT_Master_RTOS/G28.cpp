#include "ReferencePositionBlock.h"

namespace GCodeHandlers
{
    bool ValidateReferencePositionBlock(const NCBlock& block, NCManager* nc)
    {
        return ReferencePositionDetail::Validate(block, nc, block.gCode);
    }

    WaitConditionFunc Handle_G28(const NCBlock& block, NCManager* nc)
    {
        return ReferencePositionDetail::Handle(block, nc, 28);
    }
}
