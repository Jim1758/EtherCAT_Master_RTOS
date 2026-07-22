#include "GMCodeHandlers.h"
#include "EtherCatMaster.h"
#include "GlobalConfig.h" // 如果你有用到 DEBUG_PRINT 等功能

namespace GCodeHandlers 
{

    // 🌟 專屬的檢查邏輯 (封裝在這裡，不污染主程式)
    static bool CheckG04Done(NCManager* nc) 
    {
        int ticks = nc->GetDwellTicks() - 1;
        nc->SetDwellTicks(ticks);

        if (ticks > 0) {
            if (ticks % 1000 == 0) 
            {
                DEBUG_PRINT("    -> [Waiting] G04 Dwell... %d sec left\n", ticks / 1000);
            }
            return false; // 時間還沒到，繼續等
        }
        return true; // 時間到，等待結束！
    }

    WaitConditionFunc Handle_G04(const NCBlock& block, NCManager* nc) 
    {
        double seconds = 0.0;

        if (block.has('X')) seconds = block.val('X');
        else if (block.has('P')) seconds = block.val('P') / 1000.0;

        int ticks = (int)(seconds * 1000.0);

        if (ticks > 0)
        {
            nc->SetDwellTicks(ticks);
            // 🌟 回傳這個「檢查函式」，讓 NCManager 每個迴圈去呼叫它
            return CheckG04Done;
        }

        return nullptr; // 如果秒數是 0，回傳 nullptr (不需等待)
    }

} // end namespace