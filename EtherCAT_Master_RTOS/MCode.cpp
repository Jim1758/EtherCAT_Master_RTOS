#include "GMCodeHandlers.h"
#include "NCManager.h"      // 必須包含
#include "EtherCatMaster.h"
#include "GlobalConfig.h" 

namespace GCodeHandlers {

    // 模擬 IO 處理的模擬器 (維持不變)
    static bool CheckMCodeDone(NCManager* nc) {
        int ticks = nc->GetSimulatedTicks() - 1;
        nc->SetSimulatedTicks(ticks);
        if (ticks > 0) {
            DEBUG_PRINT("    -> [Waiting] IO processing M codes... Ticks left: %d\n", ticks);
            return false;
        }
        return true;
    }

    static bool CheckM00Done(NCManager* nc) {
        // 為什麼直接回傳 true？
        // 因為當機台處於 HOLD 狀態時，ProcessTask 根本不會進來檢查 Callback。
        // 當操作員按下 Cycle Start，狀態變成 RUN，ProcessTask 才會呼叫這裡。
        // 所以只要這個函式被呼叫，就代表「Cycle Start 已經被按下了」！
        // 既然重新啟動了，就直接回傳 true，讓這行指令正式結束。
        return true;
    }

    WaitConditionFunc Handle_MCode(const NCBlock& block, NCManager* nc)
    {
        if (block.mCount == 0) return nullptr;

        int m = block.mCode[0];

        switch (m)
        {
        case 0: // 🌟 M00 程式暫停
            DEBUG_PRINT("[NC] -> M00 Program Stop\n");
            nc->ChangeState(NCState::HOLD);
            // 🌟 關鍵修改：不要回傳 nullptr，改回傳 Callback，讓系統「掛起」這行指令
            return CheckM00Done;

        case 30: // 🌟 M30 程式結束
            DEBUG_PRINT("[NC_SIM] M30 Read\n");
            // M30 不需要 IO 模擬，直接回傳 nullptr
            return nullptr;

        default: // 🌟 其他 IO 型 M 碼 (如 M03, M08)
            DEBUG_PRINT("[NC_SIM] Triggering M%02d\n", m);

            // 模擬 IO 需要 2 個 Ticks 處理
            nc->SetSimulatedTicks(2);

            // 回傳專屬的檢查函式給 NCManager 掛載 (ProcessTask 會自動呼叫這個)
            return CheckMCodeDone;
        }
    }

} // end namespace