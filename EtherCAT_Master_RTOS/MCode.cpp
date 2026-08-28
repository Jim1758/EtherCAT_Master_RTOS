#include "GMCodeHandlers.h"
#include "NCManager.h"      // 必須包含

namespace GCodeHandlers {

    // 模擬 IO 處理的模擬器 (維持不變)
    static bool CheckMCodeDone(NCManager* nc) {
        int ticks = nc->GetSimulatedTicks() - 1;
        nc->SetSimulatedTicks(ticks);
        if (ticks > 0) {
            //DEBUG_PRINT("    -> [Waiting] IO processing M codes... Ticks left: %d\n", ticks);
            return false;
        }
        return true;
    }

    WaitConditionFunc Handle_MCode(const NCBlock& block, NCManager* nc)
    {
        if (block.mCount == 0) return nullptr;

        int m = block.mCode[0];

        switch (m)
        {
        case 0: // M00 Program Stop
            // Stage NC-0.2H：M00 不可在同一 Block 的 G 動作尚未完成時
            // 立即切入 HOLD。NCManager 會在 G/M Transaction 全部完成、
            // 且 Motion Completion Guard 正式放行後，再套用暫停。
            return nullptr;

        case 1: // M01 Optional Stop
            // M01 只有 NC Flow Side Effect，不是 PLC Auxiliary I/O。
            // 是否真正停下由 NCManager 在 Transaction Finalize 判斷。
            return nullptr;

        case 2:  // M02 Program End
        case 30: // M30 Program End / Rewind
            // Stage NC-0.2G：M02 / M30 都由 NCManager 的統一 Cycle-End
            // Gate 完成，不可再落入一般 IO 模擬 Callback。
            return nullptr;

        default: // 🌟 其他 IO 型 M 碼 (如 M03, M08)
            //DEBUG_PRINT("[NC_SIM] Triggering M%02d\n", m);

            // 模擬 IO 需要 2 個 Ticks 處理
            nc->SetSimulatedTicks(2);

            // 回傳專屬的檢查函式給 NCManager 掛載 (ProcessTask 會自動呼叫這個)
            return CheckMCodeDone;
        }
    }

} // end namespace