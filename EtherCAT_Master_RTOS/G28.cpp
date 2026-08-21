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


    // ==========================================================
    // 🌟 G28 回第一參考點 (機械原點)
    // ==========================================================
    WaitConditionFunc Handle_G28(const NCBlock& block, NCManager* nc)
    {
        std::vector<int> refAxes;
        std::vector<double> refPos;
        std::vector<double> intPos;
        bool hasIntermediate = false;

        // ---------------------------------------------------------
        // 1. 取得第一參考點座標 (G28 預設直接回機械絕對 0.0)
        // ---------------------------------------------------------
        double refPointCoords[8] = { 0.0 };
        // 💡 註：如果你們的 P1 (GetRefPoint(1)) 就是機械原點，
        nc->CoordSys.GetRefPoint(1, refPointCoords);


        for (int i = 0; i < 8; i++)
        {
            // 檢查軸是否存在
            if (!nc->m_motion.GetAxisContext(i).isExist) {
                AlarmManager::GetInstance().Trigger(AlarmManager::axis_is_not_enabledr);
                return [](NCManager*) { return true; };
            }
            else
            {
                if (nc->m_motion.GetAxisContext(i).isHomed == false)
                {
                    AlarmManager::GetInstance().Trigger(AlarmManager::axis_is_not_Homed);
                    return [](NCManager*) { return true; };
                }

            }


        }



        // ---------------------------------------------------------
        // 2. 預先檢查：這行指令是否有帶任何中間點座標？
        // ---------------------------------------------------------
        for (int i = 0; i < 8; i++) 
        {
            char axisLetter = nc->m_axisNames[i];
            if (axisLetter != ' ' && block.has(axisLetter)) {
                hasIntermediate = true;
                break;
            }



        }

        // ---------------------------------------------------------
        // 3. 同步打包 參考點 與 中間點 陣列
        // ---------------------------------------------------------
        for (int i = 0; i < 8; i++) 
        {
            auto& axisCtx = nc->m_motion.GetAxisContext(i);

            // 如果軸有啟用且已經歸零
            if (axisCtx.isExist && axisCtx.isHomed)
            {
                // A. 綁定參考點 (G28 是全啟用軸一起回原點)
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
                        // 🌟 【神級防呆】：沒打的軸，補上「當下的機械座標(mm)」
                        double lead = (axisCtx.finalLead < 1e-6) ? 1.0 : axisCtx.finalLead;
                        double pulsePerUnit = axisCtx.resolution_PPR / lead;
                        double currentMCS_mm = axisCtx.logicalCmdPos / pulsePerUnit;

                        intPos.push_back(currentMCS_mm);
                    }
                }
            }
            else
            {
                // 防呆：如果軸沒準備好，但操作員卻打了這個軸的指令
                char axisLetter = nc->m_axisNames[i];
                if (hasIntermediate && axisLetter != ' ' && block.has(axisLetter)) {
                    AlarmManager::GetInstance().Trigger(AlarmManager::axis_is_not_enabledr);
                    return [](NCManager*) { return true; };
                }
            }
        }

        // =========================================================
       // Software Travel Limit - G28 Target Pre-Check
       //
       // G28 可能包含兩段移動：
       //
       // 1. Intermediate Point
       // 2. Reference Point
       //
       // 所以兩段都必須在送入 MotionCore 前先檢查。
       //
       // 任一 Target 超過 Software Travel Limit：
       //
       // 1. 不送入 MotionCore
       // 2. Trigger OVER_TRAVEL
       // 3. NC 進入 ALARM
       // =========================================================

        for (size_t i = 0; i < refAxes.size(); ++i)
        {
            const int axisIndex = refAxes[i];

            AxisContext& axis =  nc->m_motion.GetAxisContext(  axisIndex);


            // =====================================================
            // A. Intermediate Point
            // =====================================================

            if (hasIntermediate)
            {
                const bool intermediateTargetValid =  nc->CoordSys.IsTargetWithinSoftwareTravelLimit( axis, intPos[i]);

                if (!intermediateTargetValid)
                {
                    AlarmManager::GetInstance().Trigger(  AlarmManager::PROGRAMMED_OVER_TRAVEL,   0, axis.axisIndex);

                    nc->ChangeState( NCState::ALARM);

                    return [](NCManager*)
                    {
                        return true;
                    };
                }
            }


            // =====================================================
            // B. Final Reference Point
            // =====================================================

            const bool referenceTargetValid = nc->CoordSys.IsTargetWithinSoftwareTravelLimit( axis, refPos[i]);

            if (!referenceTargetValid)
            {
                AlarmManager::GetInstance().Trigger( AlarmManager::PROGRAMMED_OVER_TRAVEL, 0, axis.axisIndex);

                nc->ChangeState(  NCState::ALARM);

                return [](NCManager*)
                {
                    return true;
                };
            }
        }

        // ---------------------------------------------------------
        // 4. 一次性發送給 MotionCore
        // ---------------------------------------------------------
        // 💡 由於 G28 和 G30 的軌跡行為 100% 相同，直接共用 G30_Move！
        if (hasIntermediate) {
            nc->GetMotion().G30_Move(refAxes, refPos, &intPos, BufferMode::BUFFERED);
        }
        else {
            nc->GetMotion().G30_Move(refAxes, refPos, nullptr, BufferMode::BUFFERED);
        }

        // 🌟 一樣要手動更新大腦！
        for (size_t i = 0; i < refAxes.size(); ++i) {
            int axisIdx = refAxes[i];
            nc->CoordSys.commandedMCS[axisIdx] = refPos[i]; // 更新到最終參考點
        }



        // 回傳檢查函式，監控整個 G28 的移動過程
        return CheckMotionDone;
    }
} // end namespace