#include "GMCodeHandlers.h"
#include "EtherCatMaster.h"
#include "GlobalConfig.h" // 如果你有用到 DEBUG_PRINT 等功能

namespace GCodeHandlers 
{

    // 🌟 專屬的檢查邏輯
    static bool CheckMotionDone(NCManager* nc) 
    {
        // 未來實機請改成：return nc->GetMotion().IsGroupDone();

        int ticks = nc->GetSimulatedTicks() - 1;
        nc->SetSimulatedTicks(ticks);

        if (ticks > 0) {
            DEBUG_PRINT("    -> [Waiting] Motor moving... Ticks left: %d\n", ticks);
            return false; // 還沒走完
        }
        return true; // 走完了！
    }

    WaitConditionFunc Handle_G00(const NCBlock& block, NCManager* nc) 
    {
        double currentMCS[8] = { 0 };
        double targetMCS[8] = { 0 };

        bool hasXYZ[8] = { block.has('X'), block.has('Y'), block.has('Z'), block.has('U'), block.has('V'), block.has('W'), block.has('A'), block.has('C') };
        double target[8] = { block.val('X'), block.val('Y'), block.val('Z'), block.val('U'), block.val('V'), block.val('W'), block.val('A'), block.val('C') };

        // CoordSys.Transform_WCS_to_MCS(target, hasXYZ, currentMCS, targetMCS);

        // nc->GetMotion().LineMove(...); 

        // 模擬需要 3 個 Ticks
        nc->SetSimulatedTicks(3);

        // 🌟 回傳專屬的檢查函式
        return CheckMotionDone;
    }

} // end namespace