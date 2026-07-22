#include "GMCodeHandlers.h"
// #include "GlobalConfig.h"
#include "EtherCatMaster.h"
#include "GlobalConfig.h" // 如果你有用到 DEBUG_PRINT 等功能

namespace GCodeHandlers {

    // 🌟 專屬的檢查 M 碼邏輯 (從 NCManager 搬過來的)
    static bool CheckMCodeDone(NCManager* nc) {
        int ticks = nc->GetSimulatedTicks() - 1;
        nc->SetSimulatedTicks(ticks);
        if (ticks > 0) {
            DEBUG_PRINT("    -> [Waiting] IO processing M codes... Ticks left: %d\n", ticks);
            return false;
        }
        return true;
    }

    WaitConditionFunc Handle_MCode(const NCBlock& block, NCManager* nc) 
    {
        if (block.mCount > 0) {
            int m = block.mCode[0];

            if (m == 30) {
                DEBUG_PRINT("[NC_SIM] M30 Read\n");
                // M30 不需要等，回傳 nullptr
                return nullptr;
            }
            else 
            {
                DEBUG_PRINT("[NC_SIM] Triggering M%02d\n", m);

                // 模擬 IO 需要 2 個 Ticks 處理
                nc->SetSimulatedTicks(2);
                // 🌟 回傳專屬的檢查函式給 NCManager 掛載
                return CheckMCodeDone;
            }
        }

        return nullptr;
    }

} // end namespace