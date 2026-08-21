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
        nc->MacroSys.SetVar('$', 1, 161);//設定群組1變數


        // 🌟 1. 判斷公英制倍率 (G20英制 = 25.4, G21公制 = 1.0)
        // 因為底層引擎一律吃 mm，所以讀到英制數值要放大 25.4 倍轉回 mm
        double unitScale = nc->CoordSys.isInchMode ? 25.4 : 1.0;


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
                    AlarmManager::GetInstance().Trigger(AlarmManager::axis_is_not_enabledr);
                    return [](NCManager*) { return true; };
                }

                // 🌟 取得對應軸的屬性，旋轉軸(度數)絕對不套用英制轉換！
                AxisType type = nc->m_motion.GetAxisContext(i).axisType;
                double axisScale = (type == AxisType::ROTARY || type == AxisType::ROTARY_CONTINUOUS) ? 1.0 : unitScale;

                // 🌟 [神級防呆] 針對 G16 極座標模式的特殊處理
                // 在極座標下，第二軸是「角度」，角度不能變成英制！
                if (nc->CoordSys.isPolarCoordinateActive) {
                    int angleAxisIdx = 1; // 預設 G17 的 Y 軸 (Index 1) 是角度
                    if (nc->CoordSys.activePlane == 18) angleAxisIdx = 2; // G18: Z 軸
                    if (nc->CoordSys.activePlane == 19) angleAxisIdx = 2; // G19: Z 軸

                    if (i == angleAxisIdx) {
                        axisScale = 1.0; // 強制角度維持度數，不乘 25.4
                    }
                }

                axisProgrammed[i] = true;

                // 🌟 關鍵：將 G 碼讀到的數值 (可能為 inch) 乘上 axisScale，轉回底層標準的 mm
                axisTarget[i] = block.val(axisLetter) * axisScale;

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
// Software Travel Limit - G161 Target Pre-Check
//
// 此時 targetMCS 已經是最後的 Machine Coordinate。
//
// 任一軸 Target 超出 Software Travel Limit：
//
// 1. 不送入 MotionCore
// 2. Trigger OVER_TRAVEL
// 3. NC 進入 ALARM
//
// Travel Limit 1：
//     travelLimit1Enable && G22
//
// Travel Limit 2：
//     travelLimit2Enable
//
// Travel Limit 3：
//     travelLimit3Enable
// =========================================================

        for (int i = 0; i < 8; i++)
        {
            if (!axisProgrammed[i])
            {
                continue;
            }

            AxisContext& axis =nc->m_motion.GetAxisContext(i);

            const bool targetWithinSoftwareLimit =nc->CoordSys.IsTargetWithinSoftwareTravelLimit( axis, targetMCS[i]);

            if (!targetWithinSoftwareLimit)
            {
                AlarmManager::GetInstance().Trigger( AlarmManager::PROGRAMMED_OVER_TRAVEL, 0, axis.axisIndex);

                nc->ChangeState( NCState::ALARM);

                return [](NCManager*)
                {
                    return true;
                };
            }
        }

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