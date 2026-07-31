#include "GMCodeHandlers.h"
#include "EtherCatMaster.h"
#include "GlobalConfig.h" 
#include <vector>

namespace GCodeHandlers
{
    // ==========================================================
    // 🌟 狀態檢查邏輯 (NC 系統會每一 Tick 持續呼叫它)
    // ==========================================================
    static bool CheckMotionDone(NCManager* nc)
    {
        // 🌟 1. 【新增攔截】如果系統處於 Hold (暫停) 狀態，強迫等待！
          // 假設你在 NCManager 中新增了一個 bool m_isFeedHold 的狀態
        if (nc->IsFeedHoldActive()) {
            return false; // 告訴 NC 系統：別動，繼續等！
        }

        // 2. 正常的結束判斷
        if (nc->GetMotion().IsGroupDone()) {
            return true;  // 走完了，通知 NC 系統可以執行下一行
        }

        return false;
    }

    // ==========================================================
    // 🌟 指令下達邏輯 (讀到 G00 當下只會執行一次)
    // ==========================================================
    WaitConditionFunc Handle_G00(const NCBlock& block, NCManager* nc)
    {
        // 1. 準備空陣列給 CoordinateManager
        bool axisProgrammed[8] = { false };
        double axisTarget[8] = { 0.0 };
        bool hasAnyAxis = false;

        // 🌟 2. 直接走訪機台定義的 8 個軸，動態抓取字母與座標
        for (int i = 0; i < 8; i++)
        {
            char axisLetter = nc->m_axisNames[i]; // 拿出設定檔定義的字母 (如 'X', 'Y', 'Z')

            // 防呆：如果這個軸未啟用 (空白字元) 則跳過
            if (axisLetter == ' ' || axisLetter == '\0' || axisLetter == 'N') continue;

            // 利用 NCBlock API 抓取數值
            if (block.has(axisLetter))
            {
                axisProgrammed[i] = true;
                axisTarget[i] = block.val(axisLetter); // 這裡是工作座標(WCS)或增量值
                hasAnyAxis = true;
            }
        }

        // 3. 防呆：如果這行 G00 沒有帶任何有效軸座標
        if (!hasAnyAxis) {
            return [](NCManager*) { return true; }; // 瞬間通過，不浪費時間
        }

        // =========================================================
        // 🌟 4. 座標轉換 (WCS -> MCS)
        // =========================================================
        double targetMCS[8] = { 0.0 };

        // 執行轉換：將工作座標 (包含 G90/G91、G54-G59、G92 偏移) 轉為真實的絕對機械座標
        // ⚠️ 這裡只傳入 3 個參數，與你的 CoordinateManager 實作對齊
        nc->CoordSys.Transform_WCS_to_MCS(axisTarget, axisProgrammed, targetMCS);

        // =========================================================
        // 🌟 5. 打包派單給 MotionCore
        // =========================================================
        std::vector<int> activeAxes;
        std::vector<double> targetPos;

        for (int i = 0; i < 8; i++)
        {
            if (axisProgrammed[i])
            {
                activeAxes.push_back(i);           // 記錄軸號
                targetPos.push_back(targetMCS[i]); // 放入轉換後純淨的「機械絕對座標」
            }
        }
       
        // 6. 下達移動命令！(底層會自動套用 G00 的快速定位 PID 與速度)
        nc->GetMotion().G00_Move(activeAxes, targetPos);

        // 7. 回傳檢查函式，交給 NC 系統去輪詢 (Polling)
        return CheckMotionDone;
    }

} // end namespace