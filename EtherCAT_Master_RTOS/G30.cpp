#include "GMCodeHandlers.h"
#include "EtherCatMaster.h"
#include "GlobalConfig.h" 
#include <vector>
#include "AlarmManager.h" 
#include "MotionCore.h"

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

   
    WaitConditionFunc Handle_G30(const NCBlock& block, NCManager* nc)
    {
        std::vector<int> refAxes;
        std::vector<double> refPos;
        std::vector<double> intPos;
        bool hasIntermediate = false;

        // ---------------------------------------------------------
        // 1. 取得參考點座標 (從 P 參數抓取)
        // ---------------------------------------------------------
        int pCode = block.has('P') ? (int)block.val('P') : 1;
        double refPointCoords[8] = { 0.0 };
        if (!nc->CoordSys.GetRefPoint(pCode, refPointCoords)) {
            RtPrintf(">>> [ALARM] G30 P%d invalid!\n", pCode);
            return [](NCManager*) { return true; };
        }

        // ---------------------------------------------------------
        // 2. 預先檢查：這行指令是否有帶任何中間點座標？
        // ---------------------------------------------------------
        for (int i = 0; i < 8; i++) {
            char axisLetter = nc->m_axisNames[i];
            if (axisLetter != ' ' && block.has(axisLetter)) {
                hasIntermediate = true;
                break;
            }
        }

        // ---------------------------------------------------------
        // 3. 🌟 【關鍵修正】同步打包 參考點 與 中間點 陣列
        // 確保 refAxes, refPos, intPos 的陣列長度永遠一模一樣！
        // ---------------------------------------------------------
        for (int i = 0; i < 8; i++) {
            auto& axisCtx = nc->m_motion.GetAxisContext(i);

            // 如果軸有啟用且已經歸零
            if (axisCtx.isExist && axisCtx.isHomed)
            {
                // A. 綁定參考點 (所有啟用的軸都要一起回歸)
                refAxes.push_back(i);
                refPos.push_back(refPointCoords[i]);

                // B. 綁定中間點 (陣列長度必須與 refAxes 同步)
                if (hasIntermediate) {
                    char axisLetter = nc->m_axisNames[i];

                    if (axisLetter != ' ' && block.has(axisLetter)) {
                        // 如果 G 碼有輸入這個軸，去中間點
                        intPos.push_back(block.val(axisLetter));
                    }
                    else {
                        // 🌟 【神級防呆】：G 碼沒打的軸，補上「當下的機械座標(mm)」
                        // 這樣這根軸在前往中間點的過程中距離就是 0 (不移動)，且不會造成記憶體越界！
                        double lead = (axisCtx.finalLead < 1e-6) ? 1.0 : axisCtx.finalLead;
                        double pulsePerUnit = axisCtx.resolution_PPR / lead;
                        double currentMCS_mm = axisCtx.logicalCmdPos / pulsePerUnit;

                        intPos.push_back(currentMCS_mm);
                    }
                }
            }
            else // 防呆：如果軸沒準備好，但操作員卻打了這個軸的指令
            {
                char axisLetter = nc->m_axisNames[i];
                if (hasIntermediate && axisLetter != ' ' && block.has(axisLetter)) {
                    AlarmManager::GetInstance().Trigger(AlarmManager::axis_is_not_enabledr);
                    return [](NCManager*) { return true; };
                }
            }
        }

        // ---------------------------------------------------------
        // 4. 一次性發送給 MotionCore
        // ---------------------------------------------------------
        if (hasIntermediate) {
            // 有中間點：傳入中間點指標 (intPos)
            nc->GetMotion().G30_Move(refAxes, refPos, &intPos, BufferMode::BUFFERED);
        }
        else {
            // 無中間點：傳入 nullptr
            nc->GetMotion().G30_Move(refAxes, refPos, nullptr, BufferMode::BUFFERED);
        }
        // 🌟 一樣要手動更新大腦！
        for (size_t i = 0; i < refAxes.size(); ++i) {
            int axisIdx = refAxes[i];
            nc->CoordSys.commandedMCS[axisIdx] = refPos[i]; // 更新到最終參考點
        }
        // 回傳檢查函式，監控整個 G30 的移動過程
        return CheckMotionDone;
    }

} // end namespace