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
     // 🌟 G53 機械座標定位 (無視所有偏移與旋轉)
     // 格式範例：G53 X0 Y0 Z0 (回到機械原點)
     // ==========================================================
    WaitConditionFunc Handle_G53(const NCBlock& block, NCManager* nc)
    {
        // 1. 準備容器
        std::vector<int> activeAxes;
        std::vector<double> targetPos;

        // 2. 走訪所有軸，抓取 G53 後面指定的座標
        for (int i = 0; i < 8; i++)
        {
            char axisLetter = nc->m_axisNames[i];

            // 跳過未啟用軸
            if (axisLetter == ' ' || axisLetter == '\0' || axisLetter == 'N') continue;

            // 如果 block 有帶這個軸的數值，直接當作機械座標 (MCS)
            if (block.has(axisLetter))
            {
                // 檢查軸是否存在
                if (!nc->m_motion.GetAxisContext(i).isExist) {
                    AlarmManager::GetInstance().Trigger(AlarmManager::axis_is_not_enabledr);
                    return [](NCManager*) { return true; };
                }
                else
                {
                    if (nc->m_motion.GetAxisContext(i).isHomed==false)
                    {
                        AlarmManager::GetInstance().Trigger(AlarmManager::axis_is_not_enabledr);
                        return [](NCManager*) { return true; };
                    }
                    
                }

                activeAxes.push_back(i);
                targetPos.push_back(block.val(axisLetter)); // 🌟 直接拿值，不加偏移！
            }
        }

        // 3. 防呆：如果沒指定軸，直接通過
        if (activeAxes.empty()) {
            return [](NCManager*) { return true; };
        }

        // 4. 下達移動指令 (G53 通常視為快速定位，所以走 G00 通道)
        nc->GetMotion().G53_Move(activeAxes, targetPos, BufferMode::ABORTING);

        // =========================================================
        // 🌟 終極修復：手動把大腦的 commandedMCS 更新到 Gxx 的目標點！
        // =========================================================
        for (size_t i = 0; i < activeAxes.size(); ++i) {
            int axisIdx = activeAxes[i];
            nc->CoordSys.commandedMCS[axisIdx] = targetPos[i];
        }


        // 5. 等待結束
        return CheckMotionDone;
    }

} // end namespace