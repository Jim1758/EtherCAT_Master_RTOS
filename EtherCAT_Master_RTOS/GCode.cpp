#include "GMCodeHandlers.h"
#include "EtherCatMaster.h"
#include "GlobalConfig.h" // 如果你有用到 DEBUG_PRINT 等功能

namespace GCodeHandlers
{

    WaitConditionFunc Handle_GCode(const NCBlock& block, NCManager* nc) 
    {
        switch (block.gCode) 
        {
        case 90:
            break;
        case 91:
            break;
        default:
            break;
        }

        // 瞬間完成，回傳 nullptr 代表不用等
        return nullptr;
    }

} // end namespace