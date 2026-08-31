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
    WaitConditionFunc Handle_G00(const NCBlock& block, NCManager* nc)
    {
        nc->MacroSys.SetVar('$', 1, 0);//設定群組1變數


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
        // K.6.2 preview only: commandedMCS remains unchanged until the
        // matching Motion command has been accepted by the ingress queue.
        nc->CoordSys.Preview_WCS_to_MCS(
            axisTarget,
            axisProgrammed,
            targetMCS);




        // =========================================================
        // Software Travel Limit - G00 Target Pre-Check
        //
        // 此時 targetMCS 已經完成所有座標轉換，
        // 所以這裡檢查的是最終 Machine Coordinate。
        //
        // 任一軸 Target 超過 Software Travel Limit：
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

            AxisContext& axis = nc->m_motion.GetAxisContext(i);

            const bool targetWithinSoftwareLimit = nc->CoordSys.IsTargetWithinSoftwareTravelLimit(axis, targetMCS[i]);

            if (!targetWithinSoftwareLimit)
            {
                AlarmManager::GetInstance().Trigger(AlarmManager::PROGRAMMED_OVER_TRAVEL, 0, axis.axisIndex);

                nc->ChangeState(NCState::ALARM);

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


        // =========================================================
        // 🌟 【新增邏輯】讀取 G00 的專屬倍率
        // =========================================================
        double rapidOverrideCandidate =
            nc->GetMotion().G00_overrideRatio;

        // 檢查有沒有下達 F 參數 (例如 G00 X100 F20)
        if (block.has('F'))
        {
            double f_val = block.val('F');
            rapidOverrideCandidate = f_val / 100.0;
        }
        else
        {
            // 💡 實務擴充建議：
            // 如果這行沒有寫 F，就去讀取人機介面上的「G00 旋鈕」變數
            // rapidOverride = nc->GetGlobalRapidOverride(); 
        }

        // 6. 下達移動命令！(底層會自動套用 G00 的快速定位 PID 與速度)

        const bool continuousPath =
            block.has('P') && block.val('P') == 1.0;
        bool motionAccepted = false;

        if (continuousPath)
        {
            motionAccepted =
                nc->GetMotion().TryG00MoveTransactionalTail(
                    activeAxes,
                    targetPos,
                    BufferMode::BUFFERED,
                    MotionCommandPathMode::CONTINUOUS,
                    rapidOverrideCandidate,
                    nc->CoordSys.commandedMCS);//連續路徑
        }
        else
        {
            motionAccepted =
                nc->GetMotion().TryG00MoveTransactionalTail(
                    activeAxes,
                    targetPos,
                    BufferMode::ABORTING,
                    MotionCommandPathMode::EXACT_STOP,
                    rapidOverrideCandidate,
                    nc->CoordSys.commandedMCS);//不連續
        }

        // K.6.2: an ingress rejection is a failed NC dispatch.  Never let a
        // P1 block return nullptr (or an ordinary block wait for a command
        // that does not exist), because either path could commit the PC past
        // an unaccepted G00.  Queue-full already materializes 3019; all other
        // producer-side integrity rejections are contained by 3021.
        if (!motionAccepted)
        {
            if (!AlarmManager::GetInstance().HasAlarm())
            {
                AlarmManager::GetInstance().Trigger(
                    AlarmManager::MOTION_GROUP_MAPPING_INTEGRITY);
            }

            nc->ChangeState(NCState::ALARM);
            return [](NCManager*) { return true; };
        }

        if (continuousPath)
        {
            // 🔓 解開第二道鎖：
            // 回傳 nullptr 代表「不要等我走完，大腦請立刻去讀下一行！」
            // 這個 G00 包裹會乖乖排在倉庫裡，第二個 G00 也會馬上被送進來排隊。
            return nullptr;
        }



        // 7. 回傳檢查函式，交給 NC 系統去輪詢 (Polling)
        return CheckMotionDone;
    }

} // end namespace
