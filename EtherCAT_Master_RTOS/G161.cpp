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
    // 🌟 指令下達邏輯 (讀到 G00 當下只會執行一次)
    // ==========================================================
    WaitConditionFunc Handle_G161(const NCBlock& block, NCManager* nc)
    {
        nc->MacroSys.SetVar('$', 1, 0);//設定群組1變數


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


                if (!nc->m_motion.GetAxisContext(i).isExist)
                {
                    // 1. 印出錯誤 Log，方便除錯
                    //RtPrintf(">>> [ALARM] G-Code Error: Axis '%c' is disabled but commanded!\n", axisLetter);

                    // 2. 觸發系統警報 (請換成你系統實際跳 Alarm 的 API)
                    AlarmManager::GetInstance().Trigger(AlarmManager::axis_is_not_enabledr);

                    // 3. 強制中斷，直接回傳 true 結束這行，絕對不准派單給底層！
                    return [](NCManager*) { return true; };
                }

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
             // 🌟 【神級修復】：在轉換 "前"，強迫綁定 XYZ，並補齊未下達的 WCS 座標！
             // =========================================================
        bool isG168Active = nc->CoordSys.isWorkpieceRotationActive;
        bool isG68Active = nc->CoordSys.isG68Active;
        bool isG16Active = nc->CoordSys.isPolarCoordinateActive; // 👈 新增 G16 狀態

        bool moveXYZ = axisProgrammed[0] || axisProgrammed[1] || axisProgrammed[2];

        // 🌟 條件加入 isG16Active
        if ((isG168Active || isG68Active || isG16Active) && moveXYZ)
        {
            double currentWCS[8] = { 0.0 };
            // =========================================================
            // 🌟 終極修復：絕對不能吃 Actual！改吃 Commanded 理論座標！
            // =========================================================
            nc->CoordSys.GetCommandedWCS(currentWCS);

            // =========================================================
            // 🌟 針對 G16 的極座標逆運算：把 (X, Y) 轉回 (半徑, 角度)
            // =========================================================
            if (isG16Active) {
                int p1 = 0, p2 = 1; // 預設 G17 (XY 平面)
                if (nc->CoordSys.activePlane == 18) { p1 = 0; p2 = 2; }
                if (nc->CoordSys.activePlane == 19) { p1 = 1; p2 = 2; }

                // 利用目前所在位置，逆推算回現在的 半徑(r) 與 角度(a)
                double r = std::sqrt(currentWCS[p1] * currentWCS[p1] + currentWCS[p2] * currentWCS[p2]);
                double a = std::atan2(currentWCS[p2], currentWCS[p1]) * (180.0 / 3.14159265359);

                currentWCS[p1] = r; // 將補齊用的座標替換成 半徑
                currentWCS[p2] = a; // 將補齊用的座標替換成 角度
            }

            for (int i = 0; i < 3; i++) {
                if (!axisProgrammed[i]) {
                    axisProgrammed[i] = true;
                    axisTarget[i] = currentWCS[i];
                }
            }
        }

        // =========================================================
        // 🌟 4. 座標轉換 (WCS -> MCS)
        // 此時 axisProgrammed 的 XYZ 絕對都是 true 了！轉換引擎才會真的去算旋轉！
        // =========================================================
        double targetMCS[8] = { 0.0 };
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
                activeAxes.push_back(i);
                targetPos.push_back(targetMCS[i]); // 這裡拿到的 Y 軸，就會是完美旋轉後的 7.0711 了！
            }
        }




        // 6. 下達移動命令！(底層會自動套用 G00 的快速定位 PID 與速度)
        nc->GetMotion().G161_Move(activeAxes, targetPos, BufferMode::ABORTING);

        // 7. 回傳檢查函式，交給 NC 系統去輪詢 (Polling)
        return CheckMotionDone;
    }

} // end namespace