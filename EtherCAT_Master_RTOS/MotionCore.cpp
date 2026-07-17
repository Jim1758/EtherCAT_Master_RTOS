#include "MotionCore.h"
#include <algorithm> // for std::abs, std::sqrt, std::max, std::min
#include <iostream>  // for debug prints if needed
#include "EtherCatMaster.h"
MotionCore::MotionCore() {}


// [修正] 改用 ServoDrive
void MotionCore::Link(std::vector<ENI_ServoDrive>* pAxisList)
{
    m_pAxes = pAxisList;
}
void MotionCore::Link(std::vector<ENI_ServoDrive>* pDriveList, std::vector<AxisContext>* pContextList)
{
    m_pDrives = pDriveList;
    m_pContexts = pContextList;
}
// 輔助函式: 符號判斷
int MotionCore::Sgn(double val) {
    return (0.0 < val) - (val < 0.0);
}

// 輔助函式: 單位轉換
double MotionCore::RpmToPps(double rpm, double resolution) {
    return (rpm / 60.0) * resolution;
}

double MotionCore::PpsToRpm(double pps, double resolution) {
    if (resolution == 0) return 0.0;
    return (pps * 60.0) / resolution;
}

// ==========================================
// [API] 初始化與設定
// ==========================================
void MotionCore::InitAxis(AxisContext& axis, double resolution)
{
    // 1. 物理參數
    axis.resolution_PPR = resolution;

    // 2. 運動參數 (預設值)
    // 設定最大速度 3000 RPM (轉成 PPS)
    axis.maxVel_PPS = RpmToPps(3000.0, axis.resolution_PPR);

    // 設定加減速 (預設 0.5 秒達到滿速)
    axis.acc_PPS2 = axis.maxVel_PPS * 2.0;//2.0
    axis.dec_PPS2 = axis.maxVel_PPS * 2.0;//2.0

    // 3. 雙回授預設
    axis.fbMode = FeedbackSource::MOTOR_ENCODER;
    axis.pScaleActualPos = nullptr; // 預設沒有光學尺
    axis.scaleToMotorRatio = 1.0;
    axis.maxDeviation = axis.resolution_PPR * 0.1; // 允許 0.1 圈誤差

    // 4. 狀態初始化
    axis.currentCmdPos = 0.0;
    axis.currentCmdVel = 0.0;
    axis.currentActPos = 0.0;
    axis.logicalCmdPos = 0.0; // 🟢 確保同步
    

    axis.state = MotionState::MotionState_IDLE;
    axis.inPosition = true;
    axis.isFault = false;

    axis.isServoOn = false;   // 開機必須強制為 false，直到 CiA 402 狀態機建立激磁
    axis.targetMode = 9;      // 預設 CSV

    // 5. PID 參數 (自動依解析度縮放)
    // 基準: 16777216 解析度下, Kp = 20
    double ratio = axis.resolution_PPR / 16777216.0;

    axis.pid.Kp = 20.0 * ratio;       // 剛性
    axis.pid.Ki = 0.0;                // 積分 (預設關閉)
    axis.pid.Kd = 0.0;                // 微分
    axis.pid.MaxIntegral = RpmToPps(100.0, axis.resolution_PPR);
    axis.pid.MaxLag = 100000.0 * ratio; // 允許誤差 (約2度)

    axis.pid.prevError = 0.0;
    axis.pid.integralAcc = 0.0;

    // 在你初始化 AxisContext 的地方
    axis.targetEndVel = 0.0;
}

void MotionCore::InitSmoothBuffer(AxisContext& axis, double smoothTime_ms)
{
    // 計算需要幾個 Cycle 的 Buffer
    // 例如：平滑時間 100ms，週期 1ms -> 需要 100 個格子
    int steps = (int)(smoothTime_ms / (CYCLE_TIME_SEC * 1000.0));

    if (steps < 2) steps = 2; // 最少要有 2 格

    axis.velBuffer.resize(steps, 0.0); // 初始化為 0
    axis.bufferIndex = 0;
    axis.bufferSum = 0.0;
    axis.smoothTime_ms = smoothTime_ms;
}

// 檔案：MotionCore.cpp
void MotionCore::UpdateAllMotion()//更新全部軸狀態 逐步激磁
{
    // 1. 防呆：確保指標沒丟失
    if (m_pDrives == nullptr || m_pContexts == nullptr) {
        return;
    }

    // 2. 防呆：確保兩個清單長度一致
    if (m_pDrives->size() != m_pContexts->size()) {
        // 這裡可以丟個錯誤 log
        return;
    }

    // 3. 迴圈迭代每一軸
    for (size_t i = 0; i < m_pDrives->size(); ++i)
    {
        // 呼叫原本寫好的單軸更新邏輯
        UpdateMotion((*m_pDrives)[i], (*m_pContexts)[i]);
    }
}
void MotionCore::UpdateServoState(ENI_ServoDrive& servo, AxisContext& axis)//更新單軸狀態 逐步激磁
{
    if (servo.pInput == nullptr || servo.pOutput == nullptr) return;

    uint16_t statusWord = servo.pInput->StatusWord;

    // 1. 強制設定 CSV 模式 (Mode 9)
    servo.pOutput->ModesOfOperation = 9;

    // 2. [CiA 402 狀態機]
    // 遮罩與值定義 (為了可讀性)
    const uint16_t MASK_STATE = 0x006F;
    const uint16_t MASK_FAULT = 0x0008;

    // A. 檢查故障 (Fault)
    if ((statusWord & MASK_FAULT) != 0)
    {
        servo.pOutput->ControlWord = 0x0080; // Fault Reset
        axis.isServoOn = false;
        axis.state = MotionState::MotionState_ERROR;
    }
    // B. Switch On Disabled (驅動器剛上電，未準備好)
    else if ((statusWord & 0x004F) == 0x0040)
    {
        servo.pOutput->ControlWord = 0x0006; // Shutdown
    }
    // C. Ready to Switch On (準備就緒)
    else if ((statusWord & MASK_STATE) == 0x0021)
    {
        servo.pOutput->ControlWord = 0x0007; // Switch On
    }
    // D. Switched On (電路已接通，等待最後一指令)
    else if ((statusWord & MASK_STATE) == 0x0023)
    {
        servo.pOutput->ControlWord = 0x000F; // Enable Operation (激磁！)
    }
    // E. Operation Enabled (已成功激磁)
    else if ((statusWord & MASK_STATE) == 0x0027)
    {
        servo.pOutput->ControlWord = 0x000F; // 維持激磁狀態
        axis.isServoOn = true;
    }
    else
    {
        // 若狀態未知或正在切換中，維持原本指令或歸零
        axis.isServoOn = false;
    }
}

void MotionCore::MoveToPosition(AxisContext& axis, double targetPos, double targetVel, double acc_time, double dec_time)
{
    // =========================================================
    //  1. 【起跑線對齊 (Bumpless Transfer)】
    // 防止 PID 瞬間爆衝，消除高達上億的 Lag Error！
    // =========================================================
    if (axis.state == MotionState::MotionState_IDLE ||
        axis.state == MotionState::MotionState_ERROR)
    {
        axis.currentCmdPos = axis.currentActPos;
        axis.logicalCmdPos = axis.currentActPos; // 🟢 同步起跑線
        axis.currentCmdVel = 0.0;
    }

    // =========================================================
    //  2. 【安全限速檢查】(必須在計算加減速之前！)
    // =========================================================
    if (targetVel > axis.maxVel_PPS) {
        targetVel = axis.maxVel_PPS;
    }
    // 防呆：防止使用者輸入 0 造成除以零或不動
    if (targetVel <= 1.0) {
        targetVel = axis.maxVel_PPS * 0.1;
    }

    // =========================================================
    //  3. 【儲存原始指令速度】
    // 把它鎖進保險箱，供 UpdateMotion 乘上進給倍率 (Feedrate Override)
    // =========================================================
    axis.programmedVel_PPS = targetVel;

    // =========================================================
    //  4. 【計算真實加減速度 (PPS^2)】
    // =========================================================
    double calc_acc = targetVel / acc_time;
    double calc_dec = targetVel / dec_time;

    // 防呆：避免時間給 0 算出無限大，導致馬達瞬間爆震
    if (calc_acc <= 10.0) calc_acc = 10000.0;
    if (calc_dec <= 10.0) calc_dec = 10000.0;

    // =========================================================
    //  5. 【設定運動參數給軌跡規劃器】
    // =========================================================
    axis.cruiseVel_PPS = targetVel;  // 給定當前巡航速度
    axis.acc_PPS2 = calc_acc;
    axis.dec_PPS2 = calc_dec;
    axis.finalTargetPos = targetPos;
    axis.targetEndVel = 0.0;         // 單軸定位，終點速度強制為 0

    // =========================================================
    //  6. 【單軸軌跡初始化】
    // =========================================================
    axis.planningPos = axis.currentCmdPos; // 梯形規劃從這裡開始算
    axis.startCmdPos = axis.currentCmdPos; // 紀錄起點 (備用)
    axis.motionTime = 0.0;                 // 碼表歸零

    // [極度重要] 清空 S-Curve 濾波緩衝區！
    // 避免上一次測試殘留的「負速度」影響到這次剛起步的波形
    for (size_t i = 0; i < axis.velBuffer.size(); ++i) {
        axis.velBuffer[i] = 0.0;
    }
    axis.bufferSum = 0.0;
    axis.bufferIndex = 0;

    // =========================================================
    //  7. 【扣下扳機，開始移動！】
    // =========================================================
    axis.state = MotionState::MotionState_MOVING;
    axis.inPosition = false;
    axis.isFault = false;
}

void MotionCore::VelocityMove(AxisContext& axis, double velocity, double acc_time)
{



    // 1. 無論如何，都要更新目標速度 (這是每一圈都要做的)
    axis.targetVelocity = velocity;



    double calculated_acc = 0.0;

    // 1. 如果時間設為 0 (或極小)，代表要瞬間到位
    if (acc_time < 0.001)
    {
        calculated_acc = 0.0; // 傳入 0 給底層，代表無限大加速度
    }
    // 2. 正常計算：加速度 = 速度變化量 / 時間
    else
    {
        // 為了計算斜率，我們取目標速度的絕對值
        double target_mag = std::abs(velocity);

        // [特殊處理] 如果目標速度是 0 (要停車)
        // 我們不能用 0 除以時間 (會得到 0 加速度，導致卡死)
        // 所以這裡我們假設是要從 "最大速度" 煞車到 0 的斜率，或是用當前速度
        // 簡單做法：如果目標是 0，就用 MaxVel 來算斜率，保證煞車力道足夠
        if (target_mag < 1.0) {
            target_mag = axis.maxVel_PPS;
        }

        // 公式：A = V / t
        calculated_acc = target_mag / acc_time;
    }



    axis.VelocityMove_Acc = calculated_acc;






    // =========================================================
    // 3. [關鍵修正] 狀態切換保護
    // 只有當 "原本不是速度模式" 時，才執行初始化！
    // =========================================================
    if (axis.state != MotionState::MotionState_VELOCITY)
    {
        // 這些程式碼 "絕對不能" 在每一圈都執行，否則積分會被清零
        axis.state = MotionState::MotionState_VELOCITY;

        // 讓虛擬規劃點同步當前位置 (防止跳動)
        axis.planningPos = axis.currentCmdPos;

        // 注意：千萬不要在這裡把 axis.currentCmdVel 歸零！
        // 如果你要從運動中無縫切換，保留原本的速度是正確的。
    }
}
void  MotionCore::StopMove(AxisContext& axis, double dec_time)
{
    if (axis.state == MotionState::MotionState_IDLE) return;

    // 安全保護：避免除以零
    if (dec_time < 0.001) dec_time = 0.001;

    // 【核心邏輯：固定斜率煞車】
    // 減速度 = 最高速度 / 減速時間常數
    // 這保證了不管當前速度多少，煞車的「陡度」永遠一致
    double max_v = (axis.maxVel_PPS > 0.1) ? axis.maxVel_PPS : 1000.0;
    axis.dec_PPS2 = max_v / dec_time;

    // 強制進入煞車狀態，交給 Calc_Trajectory_Velocity 處理
    axis.state = MotionState::MotionState_STOPPING;

    // RtPrintf("[DEBUG] Stop Triggered. DecRate: %.1f PPS2\n", axis.dec_PPS2);
}
void MotionCore::EmergencyStop(AxisContext& axis)
{
    if (axis.state == MotionState::MotionState_ERROR) return;

    // 1. 狀態強制切換為 ERROR (或設計一個專屬的 ESTOP 狀態)
    axis.state = MotionState::MotionState_ESTOP;

    // 2. 瞬間掐斷大腦所有的速度與目標
    axis.currentCmdVel = 0.0;
    axis.targetVelocity = 0.0;
    axis.planningPos = axis.currentCmdPos;

    // 3. 暴力清空 S-Curve 緩衝區！
    // 絕對不能讓煞車前的餘速留在 Buffer 裡，否則解除急停時機台會抖一下
    if (axis.velBuffer.size() > 0) {
        std::fill(axis.velBuffer.begin(), axis.velBuffer.end(), 0.0);
        axis.bufferSum = 0.0;
    }

    // RtPrintf(">>> [ALARM] Axis E-STOP Triggered! Velocity Killed Instantly.\n");
}
void MotionCore::SetAxisFeedrateOverride(int axisIndex, double overrideRatio)
{
    (*m_pContexts)[axisIndex].feedrateOverride = overrideRatio;
}
void MotionCore::Stop(AxisContext& axis)
{
    // 簡易急停：將目標設為當前規劃位置，讓速度歸零
    axis.finalTargetPos = axis.currentCmdPos;
    axis.state = MotionState::MotionState_STOPPING;
}

void MotionCore::ResetFault(AxisContext& axis)
{
    axis.isFault = false;
    axis.pid.integralAcc = 0.0;
    axis.pid.prevError = 0.0;
    axis.state = MotionState::MotionState_IDLE;
    //RtPrintf("Axis Fault Reset.\n");
}

// ==========================================
// [Layer 3] 軌跡規劃 (Trajectory Generator)
// ==========================================
void MotionCore::Calc_Trajectory_Trapezoidal(AxisContext& axis, AxisCommand& outCmd)
{
    if (axis.state == MotionState::MotionState_IDLE || axis.state == MotionState::MotionState_ERROR) {
        axis.currentCmdVel = 0.0;
        outCmd.instantCmdPos = axis.currentCmdPos;
        outCmd.instantCmdVel = 0.0;
        return;
    }

    double dt = CYCLE_TIME_SEC;
    double planDistErr = axis.finalTargetPos - axis.planningPos;

    // 🟢 [新增] 判斷當前的移動方向 (1.0 代表正向，-1.0 代表負向)
    double dir = (planDistErr >= 0.0) ? 1.0 : -1.0;
    double planDist = std::abs(planDistErr);

    // 捕獲式到站偵測
    bool isPlanDone = (planDist <= std::abs(axis.currentCmdVel * dt)) || (planDist < 1.0);

    if (isPlanDone)
    {
        axis.planningPos = axis.finalTargetPos;

        if (std::abs(axis.targetEndVel) > 0.1) {
            axis.currentCmdVel = dir * std::abs(axis.targetEndVel); // 帶上方向
        }
        else {
            axis.currentCmdVel = 0.0;
        }
    }
    else
    {
        double max_v = (axis.cruiseVel_PPS > 0.1) ? axis.cruiseVel_PPS : axis.maxVel_PPS;
        double dec = axis.dec_PPS2;
        double v_end = std::abs(axis.targetEndVel);
        if (v_end > max_v) v_end = max_v;

        // 算出允許的最大絕對速度
        double max_allowable_vel = std::sqrt(v_end * v_end + 2.0 * dec * planDist);
        if (max_allowable_vel > max_v) max_allowable_vel = max_v;

        // 🟢 目標速度帶上方向 (+ 或 -)
        double target_v = dir * max_allowable_vel;

        // 🟢 雙向加減速邏輯
        if (dir > 0.0)
        {
            // 正向移動
            if (axis.currentCmdVel < target_v) axis.currentCmdVel += axis.acc_PPS2 * dt;
            else if (axis.currentCmdVel > target_v) axis.currentCmdVel -= axis.dec_PPS2 * dt;

            if (axis.currentCmdVel < 0.0) axis.currentCmdVel = 0.0; // 煞車不可煞到倒車
        }
        else
        {
            // 負向移動
            if (axis.currentCmdVel > target_v) axis.currentCmdVel -= axis.acc_PPS2 * dt; // 往負向加速
            else if (axis.currentCmdVel < target_v) axis.currentCmdVel += axis.dec_PPS2 * dt; // 煞車回零

            if (axis.currentCmdVel > 0.0) axis.currentCmdVel = 0.0; // 煞車不可煞到變正向
        }

        axis.planningPos += axis.currentCmdVel * dt;
    }

    // =========================================================
    // [Layer B] S-Curve Buffer (完全不用動，負數平均後也是完美的負 S 曲線！)
    // =========================================================
    double finalOutputVel = axis.currentCmdVel;
    if (axis.velBuffer.size() > 1) {
        axis.bufferSum -= axis.velBuffer[axis.bufferIndex];
        axis.velBuffer[axis.bufferIndex] = axis.currentCmdVel;
        axis.bufferSum += axis.currentCmdVel;
        axis.bufferIndex = (axis.bufferIndex + 1) % (int)axis.velBuffer.size();
        finalOutputVel = axis.bufferSum / (double)axis.velBuffer.size();
    }

    // =========================================================
    // 🌟 [Layer C] 積分實際位置 與 完美停靠 (消除尾端突波)
    // =========================================================
    double stepPos = finalOutputVel * dt; // 這 1 毫秒原本打算走多遠

    if (dir > 0.0 && (axis.currentCmdPos + stepPos) >= axis.finalTargetPos)
    {
        // 正向：如果這步跨出去會超過終點 -> 精準截斷最後一步！
        stepPos = axis.finalTargetPos - axis.currentCmdPos;
        finalOutputVel = stepPos / dt; // 讓前饋速度完美吻合最後這段微小距離
        axis.currentCmdPos = axis.finalTargetPos;

        // 既然已經完美到站，清空 S-Curve 殘留，避免下次啟動被干擾
        for (size_t i = 0; i < axis.velBuffer.size(); ++i) axis.velBuffer[i] = 0.0;
        axis.bufferSum = 0.0;
    }
    else if (dir < 0.0 && (axis.currentCmdPos + stepPos) <= axis.finalTargetPos)
    {
        // 負向：如果這步跨出去會超過終點 -> 精準截斷最後一步！
        stepPos = axis.finalTargetPos - axis.currentCmdPos;
        finalOutputVel = stepPos / dt;
        axis.currentCmdPos = axis.finalTargetPos;

        for (size_t i = 0; i < axis.velBuffer.size(); ++i) axis.velBuffer[i] = 0.0;
        axis.bufferSum = 0.0;
    }
    else
    {
        // 正常移動，還沒到終點
        axis.currentCmdPos += stepPos;
    }

    outCmd.instantCmdVel = finalOutputVel; // 確保前饋速度與實際位置變化 100% 吻合
    outCmd.instantCmdPos = axis.currentCmdPos;

    // [到位判斷]
    bool isBufferDry = (std::abs(finalOutputVel) < 1.0);
    bool isHandoverReady = (std::abs(axis.targetEndVel) > 0.1) ? isPlanDone : (isPlanDone && isBufferDry);

    if (isHandoverReady && axis.state == MotionState::MotionState_MOVING)
    {
        if (std::abs(axis.targetEndVel) <= 0.1) {
            axis.state = MotionState::MotionState_IDLE; // 完美進入閒置
        }
        axis.inPosition = true;
    }
}


void MotionCore::Calc_Trajectory_Velocity(AxisContext& axis, AxisCommand& outCmd)
{
    if (axis.state == MotionState::MotionState_IDLE || axis.state == MotionState::MotionState_ERROR) {
        axis.currentCmdVel = 0.0;
        outCmd.instantCmdPos = axis.currentCmdPos;
        outCmd.instantCmdVel = 0.0;
        return;
    }



    double dt = CYCLE_TIME_SEC;

    // =========================================================
    // 1. [大腦規劃層] 變速與煞車邏輯
    // =========================================================
    if (axis.state == MotionState::MotionState_STOPPING)
    {
        // [修正 1] 移除魔法數字 0.1，利用精準的數學運算降至 0
        if (axis.currentCmdVel > 0.0) {
            axis.currentCmdVel -= axis.dec_PPS2 * dt;
            if (axis.currentCmdVel < 0.0) axis.currentCmdVel = 0.0; // 完美截斷
        }
        else if (axis.currentCmdVel < 0.0) {
            axis.currentCmdVel += axis.dec_PPS2 * dt;
            if (axis.currentCmdVel > 0.0) axis.currentCmdVel = 0.0; // 完美截斷
        }
    }
    else // MotionState::MotionState_MOVING
    {
        double targetVel = axis.targetVelocity;

        // 限制最大速度
        if (targetVel > axis.maxVel_PPS) targetVel = axis.maxVel_PPS;
        if (targetVel < -axis.maxVel_PPS) targetVel = -axis.maxVel_PPS;

        // [修正 3] 動態判斷現在是「變快」還是「變慢」，決定要用 Acc 還是 Dec
        double activeSlope;
        if (std::abs(targetVel) > std::abs(axis.currentCmdVel)) {
            // 速度變快 -> 使用加速度
            activeSlope = (axis.VelocityMove_Acc > 0) ? axis.VelocityMove_Acc : axis.acc_PPS2;
        }
        else {
            // 速度變慢 -> 使用減速度 (如果沒設定，才退回使用加速度)
            activeSlope = (axis.dec_PPS2 > 0) ? axis.dec_PPS2 : axis.acc_PPS2;
        }

        // 斜坡追隨
        if (axis.currentCmdVel < targetVel) {
            axis.currentCmdVel += activeSlope * dt;
            if (axis.currentCmdVel > targetVel) axis.currentCmdVel = targetVel;
        }
        else if (axis.currentCmdVel > targetVel) {
            axis.currentCmdVel -= activeSlope * dt;
            if (axis.currentCmdVel < targetVel) axis.currentCmdVel = targetVel;
        }
    }

    // =========================================================
    // 2. [Layer B] S-Curve 濾波
    // =========================================================
    double finalOutputVel = axis.currentCmdVel;

    if (axis.velBuffer.size() > 1) {
        axis.bufferSum -= axis.velBuffer[axis.bufferIndex];
        axis.velBuffer[axis.bufferIndex] = axis.currentCmdVel;
        axis.bufferSum += axis.currentCmdVel;
        axis.bufferIndex = (axis.bufferIndex + 1) % (int)axis.velBuffer.size();

        finalOutputVel = axis.bufferSum / (double)axis.velBuffer.size();
    }

    // =========================================================
    // 3. [Layer C] 積分與同步
    // =========================================================
    axis.planningPos += finalOutputVel * dt;
    axis.currentCmdPos += finalOutputVel * dt;

    outCmd.instantCmdVel = finalOutputVel;
    outCmd.instantCmdPos = axis.currentCmdPos;

    // =========================================================
    // 4. [完全停止判斷] 
    // =========================================================
    // [修正 2] 解決卡死 BUG：如果是 STOPPING，或「正在移動但目標速度被設為 0」
    bool isStoppingState = (axis.state == MotionState::MotionState_STOPPING);
    bool isTargetZero = (axis.state == MotionState::MotionState_MOVING && std::abs(axis.targetVelocity) < 0.001);

    if (isStoppingState || isTargetZero) {
        // 條件：大腦速度歸零，且 Buffer 內的餘速也流乾
        if (std::abs(axis.currentCmdVel) < 0.001 && std::abs(finalOutputVel) < 0.1) {

            axis.state = MotionState::MotionState_IDLE;
            axis.inPosition = true;
            axis.targetVelocity = 0.0; // 確保目標也清空

            // RtPrintf(">>> [IDLE] VelocityMove / Stop Complete.\n");
        }
    }
}









// ==========================================
// [Layer 2] 伺服迴路 (Servo Loop)
// ==========================================
template <typename DriveType>
void MotionCore::Run_Servo_Loop(DriveType& servo, AxisContext& axis, const AxisCommand& cmd)
{
    // 1. [Feedback Selection] 雙回授處理
    double rawMotorPos = (double)servo.pInput->ActualPosition;

    if (axis.fbMode == FeedbackSource::LINEAR_SCALE && axis.pScaleActualPos != nullptr)
    {
        // 全閉迴路：讀取光學尺並換算
        double rawScalePos = (double)(*axis.pScaleActualPos);
        double convertedScalePos = rawScalePos * axis.scaleToMotorRatio;

        axis.currentActPos = convertedScalePos;

        // [Safety] 斷帶保護 (Slip Detection)
        // 檢查馬達與光學尺是否偏差過大
        if (std::abs(rawMotorPos - convertedScalePos) > axis.maxDeviation) {
            axis.isFault = true;
            servo.pOutput->TargetVelocity = 0;
            RtPrintf("ALARM: Dual Loop Deviation Error!\n");
            return;
        }
    }
    else
    {
        // 半閉迴路：只看馬達
        axis.currentActPos = rawMotorPos;
    }




    // 2. [Lag Monitor] 跟隨誤差檢查
    double error = cmd.instantCmdPos - axis.currentActPos;

    if (axis.pid.EnableLagCheck == true)
    {
        if (std::abs(error) > axis.pid.MaxLag)
        {
            axis.isFault = true;
            servo.pOutput->TargetVelocity = 0;
            axis.state = MotionState::MotionState_ERROR;
            RtPrintf("ALARM! Lag:%d | Cmd:%d | Act:%d\n", (int)error, (int)cmd.instantCmdPos, (int)axis.currentActPos);
            return;
        }
    }


    // 3. [PID Calculation]
    // P term
    double p_term = error * axis.pid.Kp;

    // I term
    axis.pid.integralAcc += (error * CYCLE_TIME_SEC);
    // Anti-windup
    if (axis.pid.integralAcc > axis.pid.MaxIntegral) axis.pid.integralAcc = axis.pid.MaxIntegral;
    if (axis.pid.integralAcc < -axis.pid.MaxIntegral) axis.pid.integralAcc = -axis.pid.MaxIntegral;
    double i_term = axis.pid.integralAcc * axis.pid.Ki;

    // D term (CSV Mode usually 0)
    double d_term = 0.0;

    // 4. [Feedforward] 前饋控制 (關鍵！)
    // 最終輸出 = 理論速度(VFF) + PID修正量
    double finalVel = cmd.instantCmdVel + (p_term + i_term + d_term);

    // 5. [Output Clamp] 輸出限速保護
    // 限制不超過最大速度的 1.2 倍
    double limit = axis.maxVel_PPS * 1.2;
    if (finalVel > limit) finalVel = limit;
    if (finalVel < -limit) finalVel = -limit;

    // 6. [Write PDO] 寫入 EtherCAT
    servo.pOutput->TargetVelocity = (int32_t)finalVel;
}

// ==========================================
// [Core] 主更新迴圈
// ==========================================
template <typename DriveType>
void MotionCore::UpdateMotion(DriveType& servo, AxisContext& axis)
{


    // 0. 檢查 EtherCAT 狀態 (必須 Servo On 且 Mode 9)
    // 0x0027 = Operation Enabled
    bool isServoOn = (servo.pInput->StatusWord & 0x0027) == 0x0027;
    int opMode = servo.pInput->ModesOfOperationDisplay;

    // 若未激磁或不在 CSV 模式，只做狀態追隨，不運算
    if (!isServoOn || opMode != 9) {
        // 將規劃位置重置為實際位置，避免下次啟動暴衝
        axis.currentCmdPos = (double)servo.pInput->ActualPosition;
        axis.logicalCmdPos = axis.currentCmdPos; // 🟢 同步邏輯座標
        axis.currentCmdVel = 0.0;
        axis.logicalCmdVel = 0.0; // 🟢 同步邏輯速度
        axis.pid.integralAcc = 0.0;
        axis.state = MotionState::MotionState_IDLE;
        servo.pOutput->TargetVelocity = 0;
        return;
    }

    // 若故障，鎖死輸出
    if (axis.isFault) {
        servo.pOutput->TargetVelocity = 0;
        return;
    }






    // ==========================================
     // 1. [Layer 3] 軌跡規劃分流 (核心修改)
     // ==========================================
    AxisCommand cmd;

    // 初始化 cmd (防呆)
    cmd.instantCmdPos = axis.currentCmdPos;
    cmd.instantCmdVel = 0.0;

    if (axis.state == MotionState::MotionState_MOVING ||
        axis.state == MotionState::MotionState_VELOCITY)
    {
        // 🟢 用「使用者下單的速度」去乘上「倍率旋鈕」
        axis.cruiseVel_PPS = axis.programmedVel_PPS * axis.feedrateOverride;

        // 安全底線：不管倍率轉多大，絕對不准超過物理機台極速
        if (axis.cruiseVel_PPS > axis.maxVel_PPS) {
            axis.cruiseVel_PPS = axis.maxVel_PPS;
        }
    }


    switch (axis.state)
    {
    case MotionState::MotionState_MOVING:
        // [模式 A] 定位模式 (Jump) -> 執行梯形 S-Curve
        Calc_Trajectory_Trapezoidal(axis, cmd);
        break;

    case MotionState::MotionState_VELOCITY:
        // [模式 B] 速度模式 (放電) -> 執行純速度規劃
        // 這裡會呼叫你剛剛新增的函式
        Calc_Trajectory_Velocity(axis, cmd);
        break;



        // =========================================================
    // [新增] 模式 C: 多軸插補
    // =========================================================
    case MotionState::MotionState_INTERPOLATING:
    case MotionState::MotionState_STOPPING:
        // 既然 UpdateInterpolation() 已經在迴圈外面把
        // axis.currentCmdPos 和 axis.currentCmdVel 都算好並填進去了
        // 這裡我們只要負責 "傳遞" 給後面的 PID 就好

        if (m_Group.isActive)
        {
            // 如果群組還在動，實體軸只需傳遞 UpdateInterpolation 算好的值
            cmd.instantCmdPos = axis.currentCmdPos;
            cmd.instantCmdVel = axis.currentCmdVel;
        }
        else
        {
            // 如果群組停了，才走一般煞車邏輯
            Calc_Trajectory_Velocity(axis, cmd);
        }
        break;

    case MotionState::MotionState_ESTOP:

        axis.currentCmdVel = 0.0;
        cmd.instantCmdVel = 0.0;
        cmd.instantCmdPos = axis.currentCmdPos;
        axis.planningPos = axis.currentCmdPos;
        break;

    default:
    case MotionState::MotionState_ERROR:



        axis.currentCmdVel = 0.0;
        axis.logicalCmdVel = 0.0; // 🟢
        cmd.instantCmdVel = 0.0;
        cmd.instantCmdPos = axis.currentCmdPos;
        axis.planningPos = axis.currentCmdPos;
        axis.logicalCmdPos = axis.currentCmdPos; // 🟢 閒置時邏輯跟隨物理
        break;
    }

    // ==========================================
    // 2. [Layer 2] 執行伺服控制
    // ==========================================
    // 無論是 "定位" 還是 "速度" 模式，算出來的 cmd 
    // 最後都統一進 PID 迴圈算出 TargetVelocity
    Run_Servo_Loop(servo, axis, cmd);
}



//多軸插補------------------------------------------------------------------------------------------------

// MotionCore.cpp
void MotionCore::InitVirtualAxisSmooth(int windowSize)
{
    // ==========================================
    // 1. 初始化插補群組 (m_Group) 的大腦狀態
    // ==========================================
    m_Group.isActive = false;
    m_Group.mode = InterpolationMode::LINEAR; // 預設為直線模式
    m_Group.feedrateOverride = 1.0;           // 倍率預設 100%
    m_Group.axisCount = 0;

    // ==========================================
    // 2. 初始化虛擬主軸 (Virtual Axis) 的位置與狀態
    // ==========================================
    AxisContext& vAxis = m_Group.virtualAxis;
    vAxis.state = MotionState::MotionState_IDLE; // 確保一開機是閒置狀態
    vAxis.currentCmdPos = 0.0;
    vAxis.currentCmdVel = 0.0;
    vAxis.planningPos = 0.0;
    vAxis.inPosition = true;
    vAxis.feedrateOverride = 1.0;

    // ==========================================
    // 3. 配置 S-Curve 平滑緩衝區 (保留你原本的完美邏輯)
    // ==========================================
    if (windowSize <= 1) windowSize = 1; // 至少要為 1，防止除以零

    vAxis.velBuffer.resize(windowSize);
    vAxis.bufferSum = 0.0;
    vAxis.bufferIndex = 0;
    std::fill(vAxis.velBuffer.begin(), vAxis.velBuffer.end(), 0.0);

    // RtPrintf("[DEBUG] Virtual Axis & Group Initialized. Smooth Buffer: %d\n", windowSize);
}

void MotionCore::LineMove(const std::vector<int>& axes, const std::vector<double>& targetPos, double targetVel, double acc_time, double dec_time, BufferMode mode)
{
    if (m_pContexts == nullptr || axes.empty()) return;

    // 1. 建立一個全新的任務包裹
    MotionCommand cmd;
    cmd.mode = InterpolationMode::LINEAR;
    cmd.axisCount = (int)axes.size();

    // 將座標與參數抄寫到包裹裡
    for (int i = 0; i < cmd.axisCount; ++i) {
        cmd.axisIndices[i] = axes[i];
        cmd.targetPos[i] = targetPos[i];
    }
    cmd.targetVel = std::abs(targetVel);
    cmd.accTime = acc_time;
    cmd.decTime = dec_time;

    // 2. 判斷是「乖乖排隊」還是「緊急覆寫」？
    if (mode == BufferMode::ABORTING)
    {
        // 舊的清空 queue 的寫法可以刪掉，改用 deque 內建的 clear()
        m_Group.cmdQueue.clear();     // 清空未來
        m_Group.historyQueue.clear(); // 清空歷史 (新增這行)

        m_Group.isActive = false;
        m_Group.virtualAxis.state = MotionState::MotionState_IDLE;
    }

    // 3. 把包裹推入倉庫
    m_Group.cmdQueue.push_back(cmd);
}

void MotionCore::LoadNextCommand()
{
    // 1. 【解開派單鎖】：沒包裹就跳出
    if (m_Group.cmdQueue.empty()) return;

    // 🔴 關鍵：如果正在動，且「還沒」舉手要交接，才阻擋。
    // 這代表只要虛擬軸跑到終點舉手 (inPosition == true)，就可以進來拿新指令！
    if (m_Group.isActive && !m_Group.virtualAxis.inPosition) return;

    // ======================================================
    // 🔴 【時光機 Step 2：把剛跑完的指令存進歷史】
    // 如果 isActive 是 true，代表我們剛剛「才剛跑完」一個指令！
    // ======================================================
    if (m_Group.isActive && m_Group.enableHistory)
    {
        // 把剛剛跑完的 currentCmd 塞進歷史的尾巴
        m_Group.historyQueue.push_back(m_Group.currentCmd);

        // 【防呆保護】避免機台連跑三天三夜把記憶體塞爆，最多記住 1000 條線就夠退刀了
        if (m_Group.historyQueue.size() > 1000) {
            m_Group.historyQueue.pop_front(); // 把最舊的歷史刪掉
        }
    }



    // 2. 從倉庫拿出最前面的一個包裹 (正式取件)
    MotionCommand cmd = m_Group.cmdQueue.front();
    m_Group.cmdQueue.pop_front();

    // 🔴 【時光機：緊緊抓在手上】
    // 把拿出來的新包裹，存進大腦的 currentCmd 變數裡
    m_Group.currentCmd = cmd;

    // 3. 更新群組目前的運動模式與軸清單
    m_Group.mode = cmd.mode;
    m_Group.axisCount = cmd.axisCount;

    // 取得虛擬主軸引用
    AxisContext& vAxis = m_Group.virtualAxis;

    // ======================================================
    // 🟢 【終極修復：計算 S-Curve 殘留距離】
    // 因為 S-Curve 會有延遲，大腦(planningPos)跑到終點時，
    // 實際輸出(currentCmdPos)還差一點點。我們把這段差距記下來。
    // ======================================================
    double trappedDist = vAxis.planningPos - vAxis.currentCmdPos;
    if (trappedDist < 0.0) trappedDist = 0.0; // 防呆，避免負數

    // 拿完包裹，把舉手狀態放下，準備跑下一段
    vAxis.inPosition = false;

    // 🔴 進入邏輯前先強制將 V_End 歸零
    vAxis.targetEndVel = 0.0;

    // ======================================================
    // 4. 幾何數學運算 (計算起點、終點、距離/弧長)
    // ======================================================
    if (m_Group.mode == InterpolationMode::LINEAR)
    {
        double sum_sq = 0.0;
        for (int i = 0; i < m_Group.axisCount; ++i)
        {
            int idx = cmd.axisIndices[i];
            m_Group.axisIndices[i] = idx;
            AxisContext& realAxis = (*m_pContexts)[idx];

            // 1. 🟢 核心修正：起點一律抓取「邏輯座標」
            // 這樣連續下 LineMove 時，大腦才會從「邏輯終點」接續下去
            m_Group.startPos[i] = realAxis.logicalCmdPos;

            // 2. 計算邏輯位移 (Delta)
            double delta = cmd.targetPos[i] - m_Group.startPos[i];

            // 3. 累加平方和 (用於計算 3D 直線總長度)
            sum_sq += (delta * delta);

            // 4. 儲存分量比例
            m_Group.ratio[i] = delta;

            // 5. 切換狀態
            realAxis.state = MotionState::MotionState_INTERPOLATING;
        }

        double totalDist = std::sqrt(sum_sq);
        if (totalDist < 1.0) {
            LoadNextCommand();
            return;
        }
        for (int i = 0; i < m_Group.axisCount; ++i) m_Group.ratio[i] /= totalDist;

        // 🟢 【扣除殘留】：讓下一段大腦少跑一點，因為 S-Curve 稍後會自動幫忙吐出來補上！
        vAxis.finalTargetPos = totalDist; // 🟢 恢復成原本的 totalDist
      
    }
    else if (m_Group.mode == InterpolationMode::CIRCULAR_CW || m_Group.mode == InterpolationMode::CIRCULAR_CCW)
    {
        int axisX = cmd.axisIndices[0];
        int axisY = cmd.axisIndices[1];
        m_Group.axisIndices[0] = axisX;
        m_Group.axisIndices[1] = axisY;

        // 🟢 1. 取得起點 (邏輯座標)
        m_Group.startPos[0] = (*m_pContexts)[axisX].logicalCmdPos;
        m_Group.startPos[1] = (*m_pContexts)[axisY].logicalCmdPos;
        (*m_pContexts)[axisX].state = MotionState::MotionState_INTERPOLATING;
        (*m_pContexts)[axisY].state = MotionState::MotionState_INTERPOLATING;

        // 🟢 2. 處理 Z 軸 (垂直位移)
        double deltaZ = 0.0;
        if (m_Group.axisCount >= 3) {
            int axisZ = cmd.axisIndices[2];
            m_Group.axisIndices[2] = axisZ;
            m_Group.startPos[2] = (*m_pContexts)[axisZ].logicalCmdPos;
            (*m_pContexts)[axisZ].state = MotionState::MotionState_INTERPOLATING;
            deltaZ = cmd.targetPos[2] - m_Group.startPos[2];
        }

        // 🟢 3. 幾何參數準備
        double sx = m_Group.startPos[0], sy = m_Group.startPos[1];
        double cx = cmd.centerPos[0], cy = cmd.centerPos[1];
        double ex = cmd.targetPos[0], ey = cmd.targetPos[1];

        m_Group.centerX = cx;
        m_Group.centerY = cy;

        // 🟢 4. 計算角度 (這必須先算，才能算弧長)
        m_Group.startAngle = std::atan2(sy - cy, sx - cx);
        double endAngle = std::atan2(ey - cy, ex - cx);

        double totalAngle = endAngle - m_Group.startAngle;
        if (m_Group.mode == InterpolationMode::CIRCULAR_CCW) {
            if (totalAngle <= 0.0) totalAngle += 2.0 * 3.14159265359;
        }
        else {
            if (totalAngle >= 0.0) totalAngle -= 2.0 * 3.14159265359;
        }

        // 加上整圈數 (如果有 turns 參數的話，目前先維持原本邏輯)
        m_Group.totalAngle = totalAngle;

        // 🟢 5. 半徑處理 (支援螺旋出去)
        double startRadius = std::sqrt((sx - cx) * (sx - cx) + (sy - cy) * (sy - cy));
        double endRadius = std::sqrt((ex - cx) * (ex - cx) + (ey - cy) * (ey - cy));

        // 為了記錄給 UpdateInterpolation 使用，存入 currentCmd
        m_Group.currentCmd.startRadius = startRadius;
        m_Group.currentCmd.endRadius = endRadius;
        m_Group.radius = startRadius; // 預設半徑

        // 🟢 6. 計算路徑總長度 (3D Distance)
        // 如果是變半徑螺旋，弧長 = 平均半徑 * 總角度
        double avgRadius = (startRadius + endRadius) / 2.0;
        double arcLength = avgRadius * std::abs(totalAngle);

        // 勾股定理算 3D 總長：sqrt(弧長^2 + Z位移^2)
        double totalDist3D = std::sqrt(arcLength * arcLength + deltaZ * deltaZ);

        if (totalDist3D < 1.0) {
            LoadNextCommand();
            return;
        }

        m_Group.totalDist3D = totalDist3D;
        vAxis.finalTargetPos = totalDist3D;
    }

    // ======================================================
    // 5. 虛擬主軸狀態機與加減速參數設定
    // ======================================================

    vAxis.planningPos = trappedDist;
    vAxis.currentCmdPos = 0.0;

   

    if (m_Group.pathMode == PathMode::EXACT_STOP) {
        vAxis.currentCmdVel = 0.0;
    }

    vAxis.maxVel_PPS = std::abs(cmd.targetVel);
    vAxis.cruiseVel_PPS = vAxis.maxVel_PPS * m_Group.feedrateOverride;

    vAxis.acc_PPS2 = (cmd.accTime < 0.0001) ? 1e10 : (vAxis.maxVel_PPS / cmd.accTime);
    double f_dec = (cmd.decTime < 0.0) ? cmd.accTime : cmd.decTime;
    vAxis.dec_PPS2 = (f_dec < 0.0001) ? 1e10 : (vAxis.maxVel_PPS / f_dec);

    // ======================================================
     // 6. 智能轉角限速邏輯 (支援圓弧切線計算)
     // ======================================================
    if (m_Group.pathMode == PathMode::CONTINUOUS && !m_Group.cmdQueue.empty())
    {
        MotionCommand& nextCmd = m_Group.cmdQueue.front();
        double curVx = 0.0, curVy = 0.0, nextVx = 0.0, nextVy = 0.0;

        // --- Lambda 函數：計算任意軌跡在指定點的切線向量 ---
        auto getTangent = [](const MotionCommand& c, double startX, double startY, bool isExit, double& vx, double& vy) {
            if (c.mode == InterpolationMode::LINEAR) {
                // 直線的切線永遠是 (終點 - 起點)
                vx = c.targetPos[0] - startX;
                vy = c.targetPos[1] - startY;
            }
            else {
                // 圓弧的切線計算
                // isExit=true 代表算「終點」切線，false 代表算「起點」切線
                double px = isExit ? c.targetPos[0] : startX;
                double py = isExit ? c.targetPos[1] : startY;
                double cx = c.centerPos[0];
                double cy = c.centerPos[1];

                // 從圓心指向圓弧上一點的半徑向量 R
                double rx = px - cx;
                double ry = py - cy;

                // 切線向量與半徑向量垂直
                if (c.mode == InterpolationMode::CIRCULAR_CCW) {
                    vx = -ry; vy = rx;  // 逆時針切線
                }
                else {
                    vx = ry;  vy = -rx; // 順時針切線
                }
            }

            // 向量單位化 (Normalize)
            double len = std::sqrt(vx * vx + vy * vy);
            if (len > 1e-6) { vx /= len; vy /= len; }
        };

        // 1. 取得【當前指令】在「終點」的離開切線
        getTangent(cmd, m_Group.startPos[0], m_Group.startPos[1], true, curVx, curVy);

        // 2. 取得【下一個指令】在「起點」的進入切線
        getTangent(nextCmd, cmd.targetPos[0], cmd.targetPos[1], false, nextVx, nextVy);

        // 3. 計算內積 (Dot Product)
        double dot = curVx * nextVx + curVy * nextVy;
        if (dot < -1.0) dot = -1.0;
        if (dot > 1.0) dot = 1.0;

        // 計算降速因子 (1.0 = 全速過彎, 0.0 = 停下)
        double angleFactor = (1.0 + dot) / 2.0;

        double nextV = std::abs(nextCmd.targetVel * m_Group.feedrateOverride);
        vAxis.targetEndVel = (std::min)(vAxis.cruiseVel_PPS, nextV) * angleFactor;
    }
    else {
        vAxis.targetEndVel = 0.0;
    }

    // ======================================================
    // 7. S-Curve 緩衝區處理 
    // ======================================================
    if (m_Group.pathMode == PathMode::EXACT_STOP)
    {
        if (vAxis.velBuffer.empty()) vAxis.velBuffer.resize(400);
        vAxis.bufferSum = 0.0;
        vAxis.bufferIndex = 0;
        std::fill(vAxis.velBuffer.begin(), vAxis.velBuffer.end(), 0.0);
    }

    // 8. 正式啟動
    vAxis.state = MotionState::MotionState_MOVING;
    m_Group.isActive = true;

    // ======================================================
    // 🔴 【時光機 Step 2.5：拍下幾何快照】
    // 把大腦剛剛算好的參數，通通備份到 currentCmd 裡面！
    // ======================================================
    if (m_Group.enableHistory)
    {
        for (int i = 0; i < 8; i++) {
            m_Group.currentCmd.mem_startPos[i] = m_Group.startPos[i];
            m_Group.currentCmd.mem_ratio[i] = m_Group.ratio[i];
            m_Group.currentCmd.axisIndices[i] = m_Group.axisIndices[i];
        }
        m_Group.currentCmd.mem_radius = m_Group.radius;
        m_Group.currentCmd.mem_startAngle = m_Group.startAngle;
        m_Group.currentCmd.mem_centerX = m_Group.centerX;
        m_Group.currentCmd.mem_centerY = m_Group.centerY;

        // 🟢 備份 3D 幾何資訊
        m_Group.currentCmd.mem_totalDist = vAxis.finalTargetPos;
        m_Group.currentCmd.mem_totalAngle = m_Group.totalAngle;

        // 🟢 備份空間旋轉狀態 (為了 G68 原路退刀！)
        m_Group.currentCmd.mem_enableTransform = m_Group.enableTransform;
        for (int i = 0; i < 3; ++i) {
            m_Group.currentCmd.mem_transformOrigin[i] = m_Group.transformOrigin[i];
            for (int j = 0; j < 3; ++j) {
                m_Group.currentCmd.mem_transformMatrix[i][j] = m_Group.transformMatrix[i][j];
            }
        }
    }


    // 🔍 加入 LOG 看載入了什麼
    //RtPrintf("[LOAD] CmdLoaded! TgtV:%d | EndV:%d | QSize:%d\n",(int)vAxis.cruiseVel_PPS, (int)vAxis.targetEndVel, (int)m_Group.cmdQueue.size());
}
// 計算指令的方向向量 (Normalized)
void MotionCore::GetDirectionVector(const MotionCommand& cmd, double startX, double startY, double& vx, double& vy) {
    if (cmd.mode == InterpolationMode::LINEAR) {
        double dx = cmd.targetPos[0] - startX;
        double dy = cmd.targetPos[1] - startY;
        double len = std::sqrt(dx * dx + dy * dy);
        vx = (len < 1.0) ? 0 : dx / len;
        vy = (len < 1.0) ? 0 : dy / len;
    }
    else {
        // 圓弧的出口方向是終點的切線方向 (假設是在平面 0,1)
        double rx = cmd.targetPos[0] - cmd.centerPos[0];
        double ry = cmd.targetPos[1] - cmd.centerPos[1];
        double len = std::sqrt(rx * rx + ry * ry);
        // 切線向量 (CCW: [-y, x], CW: [y, -x])
        if (cmd.mode == InterpolationMode::CIRCULAR_CCW) {
            vx = -ry / len; vy = rx / len;
        }
        else {
            vx = ry / len; vy = -rx / len;
        }
    }
}

// dir: 1 代表 CCW (逆時針 G03), -1 代表 CW (順時針 G02)
void MotionCore::ArcMove(const std::vector<int>& axes, const std::vector<double>& targetPos, const std::vector<double>& centerPos, int dir, double targetVel, double acc_time, double dec_time, BufferMode mode)
{
    if (m_pContexts == nullptr || axes.size() < 2 || targetPos.size() < 2 || centerPos.size() < 2) return;

    // 1. 打包包裹
    MotionCommand cmd;
    cmd.mode = (dir == 1) ? InterpolationMode::CIRCULAR_CCW : InterpolationMode::CIRCULAR_CW;

    // 🟢 動態打包軸數
    cmd.axisCount = (int)axes.size();
    for (int i = 0; i < cmd.axisCount; ++i) {
        cmd.axisIndices[i] = axes[i];
        cmd.targetPos[i] = targetPos[i];
    }

    cmd.centerPos[0] = centerPos[0];
    cmd.centerPos[1] = centerPos[1];
    cmd.dir = dir;
    cmd.targetVel = std::abs(targetVel);
    cmd.accTime = acc_time;
    cmd.decTime = dec_time;

    // 2. 判斷插隊或排隊
    if (mode == BufferMode::ABORTING)
    {
        m_Group.cmdQueue.clear();     // deque 內建的清空語法
        m_Group.historyQueue.clear(); // 順便把歷史也清掉
        m_Group.isActive = false;
        m_Group.virtualAxis.state = MotionState::MotionState_IDLE;
    }

    // 3. 推入佇列
    m_Group.cmdQueue.push_back(cmd);
}

void MotionCore::StopGroup(std::vector<AxisContext>& axes, double decTime)
{
    if (!m_Group.isActive) return;

    // 只叫領頭羊(虛擬軸)煞車
    StopMove(m_Group.virtualAxis, decTime);

    // 實體軸不要動狀態，讓它們繼續留在 INTERPOLATING 
    // 這樣它們才會繼續接收 UpdateInterpolation 分配的位置
}

void MotionCore::EmergencyStopGroup()
{
    // 1. 關閉群組插補引擎，防止 UpdateInterpolation 繼續寫入位置
    m_Group.isActive = false;

    // 2. 徹底殺掉虛擬主軸 (清空 Buffer 是關鍵)
    EmergencyStop(m_Group.virtualAxis);

    // 3. 遍歷所有實體軸執行急停
    for (int i = 0; i < m_Group.axisCount; ++i)
    {
        int idx = m_Group.axisIndices[i];
        AxisContext& realAxis = (*m_pContexts)[idx];

        // 呼叫我們先前寫好的單軸 EmergencyStop
        EmergencyStop(realAxis);
    }

    // RtPrintf(">>> [ALARM] EmergencyStopGroup Executed. All motions killed.\n");
}
void MotionCore::SetGroupFeedrateOverride(double overrideRatio)
{
    // 1. 防呆檢查 (限制在 0.0 到 1.2 之間，最高允許 120% 超頻)
    // 如果你要做 EDM 退刀，這裡可以允許負數 (如 -0.5)，否則先鎖在 >= 0

    // 2. 把倍率寫進群組與虛擬主軸
    m_Group.feedrateOverride = overrideRatio;
    m_Group.virtualAxis.feedrateOverride = overrideRatio;
}
void MotionCore::SetGroupPathMode(PathMode mode)
{
    m_Group.pathMode = mode;

    // 如果切換回 EXACT_STOP (G61)，為了安全，我們確保虛擬主軸的目標終點速度立刻歸零
    if (mode == PathMode::EXACT_STOP) {
        m_Group.virtualAxis.targetEndVel = 0.0;
    }

    // RtPrintf("[INFO] Path Mode Switched to: %s\n", 

}

void MotionCore::UpdatePathServoVelocity(double velocity_pps)
{
    // 這裡的速度由外部感測器計算後傳入
    m_Group.pathServoVel = velocity_pps;
    //RtPrintf("UpdatePathServoVelocity >>> %d\n", (int)velocity_pps);
}

void MotionCore::EnableHistoryBuffer(bool enable)
{
    // 1. 切換群組內的時光機開關
    m_Group.enableHistory = enable;

    // 2. 如果使用者決定「關閉」時光機，順手把歷史垃圾清掉，釋放記憶體
    if (!enable)
    {
        m_Group.historyQueue.clear();
    }

    // RtPrintf("[INFO] History Buffer (Time Machine) is now: %s\n", enable ? "ON" : "OFF");
}

// =================================================================
// 🟢 [新增] 空間座標旋轉設定 (G68)
// =================================================================
void MotionCore::SetCoordinateTransform(bool enable, double ox, double oy, double oz, double yaw_deg, double pitch_deg, double roll_deg)
{
    m_Group.enableTransform = enable;
    if (!enable) return;

    m_Group.transformOrigin[0] = ox;
    m_Group.transformOrigin[1] = oy;
    m_Group.transformOrigin[2] = oz;

    // 將角度轉為弧度 (Radian)
    double a = yaw_deg * (3.14159265359 / 180.0);   // 繞 Z 軸 (Yaw)
    double b = pitch_deg * (3.14159265359 / 180.0); // 繞 Y 軸 (Pitch)
    double c = roll_deg * (3.14159265359 / 180.0);  // 繞 X 軸 (Roll)

    // 計算預先準備的 sin 與 cos
    double ca = std::cos(a), sa = std::sin(a);
    double cb = std::cos(b), sb = std::sin(b);
    double cc = std::cos(c), sc = std::sin(c);

    // 產生 3D 空間的合成旋轉矩陣 R = Rz(a) * Ry(b) * Rx(c)
    m_Group.transformMatrix[0][0] = ca * cb;
    m_Group.transformMatrix[0][1] = ca * sb * sc - sa * cc;
    m_Group.transformMatrix[0][2] = ca * sb * cc + sa * sc;

    m_Group.transformMatrix[1][0] = sa * cb;
    m_Group.transformMatrix[1][1] = sa * sb * sc + ca * cc;
    m_Group.transformMatrix[1][2] = sa * sb * cc - ca * sc;

    m_Group.transformMatrix[2][0] = -sb;
    m_Group.transformMatrix[2][1] = cb * sc;
    m_Group.transformMatrix[2][2] = cb * cc;
}

PathMode MotionCore::GetGroupPathMode() const
{
    return m_Group.pathMode;
}





void MotionCore::TriggerPathJump(JumpMode mode, const std::vector<JumpSegment>& retract, const std::vector<JumpSegment>& approach, double dwellTime_ms, int b1_axis)
{
    if (m_Group.jumpManager.state != JumpState::IDLE) return;

    // 🟢 儲存模式與專用參數
    m_Group.jumpManager.mode = mode;
    m_Group.jumpManager.b1_AxisIndex = b1_axis;




    // 🟢 [修復 1] 宣告 jm 參照，解決 E0020 "jm 未定義"
    PathJumpManager& jm = m_Group.jumpManager;


    // 🌟 [修正 1]：先清空舊任務，確保沒有殘留
    jm.retractSteps.clear();
    jm.approachSteps.clear();

    // 🌟 [修正 2]：手動逐一拷貝，不要直接用 = 賦值 (防止 RTX64 vector 記憶體坑)
    for (const auto& s : retract) jm.retractSteps.push_back(s);
    for (const auto& s : approach) jm.approachSteps.push_back(s);

    // =========================================================
    // 🌟 [神級修復]：在做任何事之前，先拍下現在所有實體軸的絕對位置！
    // 沒有這一步，後面的數學全部都會算錯導致瞬移！
    // =========================================================
    for (int i = 0; i < m_Group.axisCount; ++i) {
        jm.frozenPos[i] = (*m_pContexts)[i].logicalCmdPos;
    }
    // =========================================================

    // 儲存模式與專用參數
    jm.mode = mode;


 


    // =========================================================
    // 🟢 [新增] B0 模式：拍下瞬間車頭方向的「快照」！
    // =========================================================
    if (mode == JumpMode::B0_REVERSE && m_pContexts != nullptr && m_Group.axisCount >= 2)
    {
        int idxX = m_Group.axisIndices[0];
        int idxY = m_Group.axisIndices[1];
        int idxZ = (m_Group.axisCount >= 3) ? m_Group.axisIndices[2] : -1;

        double vx = (*m_pContexts)[idxX].logicalCmdVel;
        double vy = (*m_pContexts)[idxY].logicalCmdVel;
        double vz = (idxZ != -1) ? (*m_pContexts)[idxZ].logicalCmdVel : 0.0;

        // 計算向量長度
        double v_norm = std::sqrt(vx * vx + vy * vy + vz * vz);

        // 防呆：如果剛好靜止不動，預設往 Z 軸正向退
        if (v_norm > 1e-6)
        {
            m_Group.jumpManager.b0_Vector[0] = -vx / v_norm; // 掛負號！反向！
            m_Group.jumpManager.b0_Vector[1] = -vy / v_norm;
            m_Group.jumpManager.b0_Vector[2] = -vz / v_norm;
        }
        else {
            m_Group.jumpManager.b0_Vector[0] = 0.0;
            m_Group.jumpManager.b0_Vector[1] = 0.0;
            m_Group.jumpManager.b0_Vector[2] = 1.0;
        }
    }



    // 裝填任務
    m_Group.jumpManager.retractSteps = retract;
    m_Group.jumpManager.approachSteps = approach;
    m_Group.jumpManager.dwellTimeTarget = dwellTime_ms;

    // 拍下快照
    m_Group.jumpManager.triggerPos = m_Group.virtualAxis.currentCmdPos;
    m_Group.jumpManager.currentOffset = 0.0;
    m_Group.jumpManager.jumpVel = 0.0;
    m_Group.jumpManager.currentStepIdx = 0;


 


    // =========================================================
    // 🟢 [新增] 徹底洗除大腦的 S-Curve 時光記憶！
    // 這樣跳刀結束接回 PATH_SERVO 時，才不會把剛剛的高速噴出來(紫色的尖刺)
    // =========================================================
    m_Group.virtualAxis.currentCmdVel = 0.0;
    m_Group.virtualAxis.planningPos = m_Group.virtualAxis.currentCmdPos;
    if (!m_Group.virtualAxis.velBuffer.empty()) {
        std::fill(m_Group.virtualAxis.velBuffer.begin(), m_Group.virtualAxis.velBuffer.end(), 0.0);
    }
    m_Group.virtualAxis.bufferSum = 0.0;
    // =========================================================


    if (!retract.empty()) {
        m_Group.jumpManager.targetOffset = -retract[0].distance;
        m_Group.jumpManager.state = JumpState::RETRACTING;
        m_Group.pathMode = PathMode::JUMP_TRACKING;
    }
}

void MotionCore::TriggerCenterJump_B3(
    double cx, double cy, double cz, double vx, double vy, double vz,
    const std::vector<JumpSegment>& toCenter, const std::vector<JumpSegment>& toApex,
    const std::vector<JumpSegment>& fromApex, const std::vector<JumpSegment>& toWorkpiece,
    double dwellTime_ms)
{
    if (m_Group.jumpManager.state != JumpState::IDLE) return;
    PathJumpManager& jm = m_Group.jumpManager;
    jm.mode = JumpMode::B3_CENTER;

    // 1. 拍下放電點快照
    for (int i = 0; i < m_Group.axisCount; ++i) {
        jm.frozenPos[i] = (*m_pContexts)[m_Group.axisIndices[i]].logicalCmdPos;
    }

    // ==========================================================
  // 🌟 核心關鍵：拍下路徑進度快照 (這就是它的家！)
  // ==========================================================
    AxisContext& vAxis = m_Group.virtualAxis;
    jm.triggerPos = vAxis.currentCmdPos;
    // ==========================================================

    // 2. 存下安全中心點與 3D 拔高向量
    jm.b3_centerPos[0] = cx; jm.b3_centerPos[1] = cy; jm.b3_centerPos[2] = cz;

    double v_len = std::sqrt(vx * vx + vy * vy + vz * vz);
    jm.b3_retractVector[0] = (v_len > 1e-6) ? vx / v_len : 0.0;
    jm.b3_retractVector[1] = (v_len > 1e-6) ? vy / v_len : 0.0;
    jm.b3_retractVector[2] = (v_len > 1e-6) ? vz / v_len : 1.0;

    // 3. 算出【真實的橫移距離】
    double dx = cx - jm.frozenPos[0];
    double dy = cy - jm.frozenPos[1];
    double dz = cz - jm.frozenPos[2];
    jm.b3_distToCenter = std::sqrt(dx * dx + dy * dy + dz * dz);

    // ========================================================
    // 🌟 [神級等比例平帳]：把使用者的腳本，無縫縮放成真實距離！
    // ========================================================
    jm.b3_toCenterSteps = toCenter;
    jm.b3_toWorkpieceSteps = toWorkpiece;

    // 處理去程：把腳本距離縮放成 b3_distToCenter
    double sum1 = 0; for (auto& s : jm.b3_toCenterSteps) sum1 += s.distance;
    if (sum1 > 1e-6) {
        for (auto& s : jm.b3_toCenterSteps) s.distance = s.distance * (jm.b3_distToCenter / sum1);
    }
    else if (!jm.b3_toCenterSteps.empty()) {
        jm.b3_toCenterSteps[0].distance = jm.b3_distToCenter; // 防呆
    }

    // 處理回程：同樣必須等比例縮放成 b3_distToCenter
    double sum4 = 0; for (auto& s : jm.b3_toWorkpieceSteps) sum4 += s.distance;
    if (sum4 > 1e-6) {
        for (auto& s : jm.b3_toWorkpieceSteps) s.distance = s.distance * (jm.b3_distToCenter / sum4);
    }
    else if (!jm.b3_toWorkpieceSteps.empty()) {
        jm.b3_toWorkpieceSteps[0].distance = jm.b3_distToCenter;
    }

    // =====================================================================
    // 🟢 [數學驗證 LOG 1：觸發瞬間的幾何計算]
    // =====================================================================
    RtPrintf("[MATH_TRIGGER] cx:%d, cy:%d, frozenX:%d, frozenY:%d\n",
        (int)(cx * 1000.0), (int)(cy * 1000.0),
        (int)(jm.frozenPos[0] * 1000.0), (int)(jm.frozenPos[1] * 1000.0));

    RtPrintf("[MATH_TRIGGER] dx:%d, dy:%d, dz:%d => distToCenter:%d\n",
        (int)((cx - jm.frozenPos[0]) * 1000.0),
        (int)((cy - jm.frozenPos[1]) * 1000.0),
        (int)((cz - jm.frozenPos[2]) * 1000.0),
        (int)(jm.b3_distToCenter * 1000.0));

    if (!jm.b3_toWorkpieceSteps.empty()) {
        RtPrintf("[MATH_TRIGGER] sum4:%d, Step4_Dist1:%d, Step4_Dist2:%d\n",
            (int)(sum4 * 1000.0),
            (int)(jm.b3_toWorkpieceSteps[0].distance * 1000.0),
            jm.b3_toWorkpieceSteps.size() > 1 ? (int)(jm.b3_toWorkpieceSteps[1].distance * 1000.0) : 0);
    }
    // =====================================================================

    // 4. 裝填拔高/降落腳本
    jm.b3_toApexSteps = toApex;
    jm.b3_fromApexSteps = fromApex;
    jm.b3_distToApex = 0;
    for (auto& s : jm.b3_toApexSteps) jm.b3_distToApex += s.distance; // 算出拔高總長

    // 5. 啟動 B3 狀態機 (進入第一段)
    jm.dwellTimeTarget = dwellTime_ms;
    jm.currentStepIdx = 0;
    jm.currentOffset = 0.0;
    jm.jumpVel = 0.0;
    jm.state = JumpState::B3_TO_CENTER;
    m_Group.pathMode = PathMode::JUMP_TRACKING;

    // 清空原本軌跡速度
    m_Group.virtualAxis.currentCmdVel = 0.0;
}


void MotionCore::TriggerOrbitalJump_B4(
    double upperCx, double upperCy, double upperCz,
    double vx, double vy, double vz,
    const std::vector<JumpSegment>& toUpper,
    const std::vector<JumpSegment>& toApex,
    const std::vector<JumpSegment>& fromApex,
    const std::vector<JumpSegment>& toWorkpiece,
    double dwellTime_ms)
{
    if (m_Group.jumpManager.state != JumpState::IDLE) return;

    PathJumpManager& jm = m_Group.jumpManager;
    AxisContext& vAxis = m_Group.virtualAxis;
    jm.mode = JumpMode::B4_ORBITAL_DIAGONAL;

    // 1. 拍下實體軸「放電點」快照
    for (int i = 0; i < m_Group.axisCount; ++i) {
        jm.frozenPos[i] = (*m_pContexts)[m_Group.axisIndices[i]].logicalCmdPos;
    }

    // 🌟 2. 拍下路徑進度快照 (這就是之前查很久的家！)
    jm.triggerPos = vAxis.currentCmdPos;

    // 3. 儲存上中心點與拔高向量
    jm.b4_upperCenterPos[0] = upperCx;
    jm.b4_upperCenterPos[1] = upperCy;
    jm.b4_upperCenterPos[2] = upperCz;

    double v_len = std::sqrt(vx * vx + vy * vy + vz * vz);
    jm.b4_retractVector[0] = (v_len > 1e-6) ? vx / v_len : 0.0;
    jm.b4_retractVector[1] = (v_len > 1e-6) ? vy / v_len : 0.0;
    jm.b4_retractVector[2] = (v_len > 1e-6) ? vz / v_len : 1.0;

    // 4. 算出【斜向真實距離】(放電點 -> 上中心)
    double dx = upperCx - jm.frozenPos[0];
    double dy = upperCy - jm.frozenPos[1];
    double dz = upperCz - jm.frozenPos[2];
    jm.b4_distToUpper = std::sqrt(dx * dx + dy * dy + dz * dz);

    // 5. 絕對安全深拷貝腳本 (防止記憶體坑)
    jm.b4_toUpperSteps.clear();
    for (const auto& s : toUpper) jm.b4_toUpperSteps.push_back(s);
    jm.b4_toApexSteps.clear();
    for (const auto& s : toApex) jm.b4_toApexSteps.push_back(s);
    jm.b4_fromApexSteps.clear();
    for (const auto& s : fromApex) jm.b4_fromApexSteps.push_back(s);
    jm.b4_toWorkpieceSteps.clear();
    for (const auto& s : toWorkpiece) jm.b4_toWorkpieceSteps.push_back(s);

    // 6. 等比例平帳 (縮放去程與回程腳本距離)
    double sum1 = 0; for (auto& s : jm.b4_toUpperSteps) sum1 += s.distance;
    if (sum1 > 1e-6) {
        for (auto& s : jm.b4_toUpperSteps) s.distance = s.distance * (jm.b4_distToUpper / sum1);
    }
    else if (!jm.b4_toUpperSteps.empty()) {
        jm.b4_toUpperSteps[0].distance = jm.b4_distToUpper;
    }

    double sum4 = 0; for (auto& s : jm.b4_toWorkpieceSteps) sum4 += s.distance;
    if (sum4 > 1e-6) {
        for (auto& s : jm.b4_toWorkpieceSteps) s.distance = s.distance * (jm.b4_distToUpper / sum4);
    }
    else if (!jm.b4_toWorkpieceSteps.empty()) {
        jm.b4_toWorkpieceSteps[0].distance = jm.b4_distToUpper;
    }

    // 7. 算出拔高總長
    jm.b4_distToApex = 0;
    for (auto& s : jm.b4_toApexSteps) jm.b4_distToApex += s.distance;

    // 8. 啟動狀態機
    jm.dwellTimeTarget = dwellTime_ms;
    jm.currentStepIdx = 0;
    jm.currentOffset = 0.0;
    jm.jumpVel = 0.0;
    jm.state = JumpState::B4_TO_UPPER_CENTER;
    m_Group.pathMode = PathMode::JUMP_TRACKING;

    // 清空原本軌跡速度
    vAxis.currentCmdVel = 0.0;
}

void MotionCore::FinalizeSafePath(const std::vector<JumpSegment>& retract, std::vector<JumpSegment>& approach)//排渣跳躍腳本安全保護
{
    if (retract.empty() || approach.empty()) return;

    // 1. 算出這趟去程「總負債」(Total Debt)
    double totalOutbound = 0;
    for (const auto& s : retract) totalOutbound += s.distance;

    // 2. 遍歷進刀段落進行動態平帳
    double currentReturnSum = 0;
    size_t count = approach.size();

    if (count==1)
    {
        approach[0].distance = totalOutbound;
    }
    else  
    {
        double total_Back_dir = 0;

        for (size_t i = 0; i < count; ++i) 
        {
            total_Back_dir += approach[i].distance;
        }

        //暫時不寫 顯下斷點防止暴衝
        double Sum = totalOutbound - total_Back_dir;
        if (Sum == 0)
        {
          
        }
        else if (Sum > 0)
        {
            approach[0].distance += Sum;
        }
        else 
        {
            if (approach[0].distance <std::abs(Sum))
            {
                //通常不會
            }
            else
            {
                approach[0].distance -= Sum;
            }
          
        }
    }
  
   

    // 3. 再次檢查：如果債務已經還完但還有多餘段落，將多餘段落距離設為 0
    // (這對應你說的「刪除或修改中間段落導致變長」的情況)
}


// 🌟 1. 觸發 B0 暫停 (一次給齊兩個腳本)
void MotionCore::TriggerPause_B0(const std::vector<JumpSegment>& retractScript, const std::vector<JumpSegment>& approachScript)
{
    if (m_Group.jumpManager.state != JumpState::IDLE) return;
    PathJumpManager& jm = m_Group.jumpManager;
    AxisContext& vAxis = m_Group.virtualAxis;

    jm.mode = JumpMode::B0_REVERSE;

    // 🌟 這是讓機台停在空中的「絕對關鍵」！沒有這行，它就會直接降落！
    jm.isPauseMode = true;

    jm.triggerPos = vAxis.currentCmdPos;

    for (int i = 0; i < m_Group.axisCount; ++i) {
        jm.frozenPos[i] = (*m_pContexts)[m_Group.axisIndices[i]].logicalCmdPos;
    }

    // 拍下反向向量
    int idxX = m_Group.axisIndices[0], idxY = m_Group.axisIndices[1];
    int idxZ = (m_Group.axisCount >= 3) ? m_Group.axisIndices[2] : -1;
    double vx = (*m_pContexts)[idxX].logicalCmdVel;
    double vy = (*m_pContexts)[idxY].logicalCmdVel;
    double vz = (idxZ != -1) ? (*m_pContexts)[idxZ].logicalCmdVel : 0.0;
    double v_norm = std::sqrt(vx * vx + vy * vy + vz * vz);

    if (v_norm > 1e-6) {
        jm.b0_Vector[0] = -vx / v_norm; jm.b0_Vector[1] = -vy / v_norm; jm.b0_Vector[2] = -vz / v_norm;
    }
    else {
        jm.b0_Vector[0] = 0.0; jm.b0_Vector[1] = 0.0; jm.b0_Vector[2] = 1.0;
    }

    // 🌟 一次把退刀與進刀腳本都載入大腦
    jm.retractSteps = retractScript;
    jm.approachSteps = approachScript;

    jm.currentStepIdx = 0;
    jm.currentOffset = 0.0;
    jm.jumpVel = 0.0;
    jm.state = JumpState::RETRACTING;

    m_Group.pathMode = PathMode::JUMP_TRACKING;
    vAxis.currentCmdVel = 0.0;
}
void MotionCore::TriggerPause_B1(const std::vector<JumpSegment>& retractScript, const std::vector<JumpSegment>& approachScript, int axisIndex, double dir)
{
    if (m_Group.jumpManager.state != JumpState::IDLE) return;
    PathJumpManager& jm = m_Group.jumpManager;
    AxisContext& vAxis = m_Group.virtualAxis;

    jm.mode = JumpMode::B1_SPECIFIC_AXIS;
    jm.isPauseMode = true; // 🌟 這是讓它停在空中的護身符！

    jm.triggerPos = vAxis.currentCmdPos;

    // 拍下放電點快照
    for (int i = 0; i < m_Group.axisCount; ++i) {
        jm.frozenPos[i] = (*m_pContexts)[m_Group.axisIndices[i]].logicalCmdPos;
    }

    // 🌟 B1 專屬設定：指定要跳哪一軸，以及方向 (+1.0 或 -1.0)
    jm.b1_AxisIndex = axisIndex;
    jm.b1_dir = (dir >= 0.0) ? 1.0 : -1.0;

    // 載入退刀與降落腳本
    jm.retractSteps = retractScript;
    jm.approachSteps = approachScript;

    jm.currentStepIdx = 0;
    jm.currentOffset = 0.0;
    jm.jumpVel = 0.0;
    jm.state = JumpState::RETRACTING;

    m_Group.pathMode = PathMode::JUMP_TRACKING;
    vAxis.currentCmdVel = 0.0;
}

void MotionCore::TriggerPause_B2(const std::vector<JumpSegment>& retractScript, const std::vector<JumpSegment>& approachScript)
{
    if (m_Group.jumpManager.state != JumpState::IDLE) return;

   

    PathJumpManager& jm = m_Group.jumpManager;
    AxisContext& vAxis = m_Group.virtualAxis;

    jm.mode = JumpMode::B2_PATH_REVERSE;
    jm.isPauseMode = true; // 🌟 空中懸停護身符

    jm.triggerPos = vAxis.currentCmdPos;

    for (int i = 0; i < m_Group.axisCount; ++i) {
        jm.frozenPos[i] = (*m_pContexts)[m_Group.axisIndices[i]].logicalCmdPos;
    }

    jm.retractSteps = retractScript;


    jm.approachSteps = approachScript;

    jm.currentStepIdx = 0;
    jm.currentOffset = 0.0;
    jm.jumpVel = vAxis.currentCmdVel;
    jm.jumpVel = m_Group.virtualAxis.currentCmdVel;
    jm.state = JumpState::RETRACTING;

    m_Group.pathMode = PathMode::JUMP_TRACKING;
}
void MotionCore::Process_B2_Approach_Planner(double dt)
{
    PathJumpManager& jm = m_Group.jumpManager;
    JumpSegment& seg = jm.approachSteps[jm.currentStepIdx];
    double realAcc = (seg.accTime > 0.0001) ? (seg.velocity / seg.accTime) : 1e10;
    double realDec = (seg.decTime > 0.0001) ? (seg.velocity / seg.decTime) : 1e10;

    // =========================================================
    // 🌟 1. 無限前瞻 (Look-Ahead)：尋找未來的「真正轉角」
    // =========================================================
    double accumulatedDist = m_Group.currentCmd.mem_totalDist;
    bool needToStop = false;

    if (!m_Group.cmdQueue.empty())
    {
        double curRatio[8];
        for (int i = 0; i < m_Group.axisCount; i++) curRatio[i] = m_Group.ratio[i];
        InterpolationMode curMode = m_Group.mode;

        // ⚠️ 修正：直接掃描整個幾何軌跡 Queue，不要被 approachSteps 的段數限制！
        for (auto it_cmd = m_Group.cmdQueue.begin(); it_cmd != m_Group.cmdQueue.end(); ++it_cmd)
        {
            bool isTurn = false;

            // 條件 A：模式改變 (直線變圓弧，或圓弧變直線)，絕對要煞車
            if (curMode != it_cmd->mode) {
                isTurn = true;
            }
            // 條件 B：都是直線，用內積檢查折角
            else if (curMode == InterpolationMode::LINEAR)
            {
                double dotProduct = 0.0, len1 = 0.0, len2 = 0.0;
                for (int a = 0; a < m_Group.axisCount; a++) {
                    double cR = curRatio[a];
                    double nR = it_cmd->mem_ratio[a];
                    dotProduct += cR * nR;
                    len1 += cR * cR;
                    len2 += nR * nR;
                }
                if (len1 > 0.01 && len2 > 0.01) {
                    double cosTheta = dotProduct / (std::sqrt(len1) * std::sqrt(len2));
                    if (cosTheta < 0.999) isTurn = true; // 夾角大於 2.5 度就算轉彎
                }
            }
            // 條件 C：連續圓弧，安全起見當作轉折煞車
            else {
                isTurn = true;
            }

            if (isTurn) {
                needToStop = true;
                break; // 找到轉角了！紅燈線就畫在這！
            }

            // 如果是直走，累加距離
            accumulatedDist += it_cmd->mem_totalDist;

            // 更新比較基準，繼續看下一段
            curMode = it_cmd->mode;
            for (int a = 0; a < m_Group.axisCount; a++) curRatio[a] = it_cmd->mem_ratio[a];
        }
    }

    // 如果掃到底都沒轉彎，代表一路直通放電原點！
    if (!needToStop) {
        needToStop = true;
        accumulatedDist = jm.triggerPos; // 這樣 targetPhysicalOffset 才會是 0.0 (原點)
    }

    // =========================================================
    // 🌟 2. 計算出絕對精準的物理紅燈線
    // =========================================================
    double targetPhysicalOffset = accumulatedDist - jm.triggerPos;

    // =========================================================
    // 🌟 3. 呼叫規劃器 
    // =========================================================
    double plannerTarget = targetPhysicalOffset + 0.1;
    jm.currentOffset = PlanTrapezoidal_B2(jm.currentOffset, plannerTarget, seg.velocity, realAcc, realDec, jm.jumpVel, dt);

    // =========================================================
    // 🌟 4. 終極護城河：卡死時光機
    // =========================================================
    if (needToStop && jm.currentOffset >= targetPhysicalOffset - 1.0)
    {
        if (std::abs(jm.jumpVel) > 50000.0) {
            jm.currentOffset = targetPhysicalOffset - 1.0;
        }
    }

    // =========================================================
    // 🌟 5. 速度劇本切換 (這才是真正的速度換檔！)
    // =========================================================
    // 算出「當前腳本段落」的換檔邊界。
    // 因為進刀的最終終點是 0.0，所以這一段的邊界，就是「後面所有段落距離總和的負數」
    double speedStepBoundary = 0.0;
    for (int i = jm.currentStepIdx + 1; i < (int)jm.approachSteps.size(); i++) {
        speedStepBoundary -= jm.approachSteps[i].distance;
    }

    // 如果當前進度 (currentOffset) 越過了邊界，就切換下一段速度！
    if (jm.currentOffset >= speedStepBoundary && jm.currentStepIdx < (int)jm.approachSteps.size() - 1) {
        jm.currentStepIdx++;
        RtPrintf("[B2_SPEED_SHIFT] Switched to Step %d! Boundary: %d\n", jm.currentStepIdx, (int)speedStepBoundary);
    }
}


void MotionCore::TriggerPause_B3(double cx, double cy, double cz, double vx, double vy, double vz,const std::vector<JumpSegment>& toCenter, const std::vector<JumpSegment>& toApex,const std::vector<JumpSegment>& fromApex, const std::vector<JumpSegment>& toWorkpiece)
{
    if (m_Group.jumpManager.state != JumpState::IDLE) return;
    PathJumpManager& jm = m_Group.jumpManager;
    AxisContext& vAxis = m_Group.virtualAxis;

    jm.mode = JumpMode::B3_CENTER;
    jm.isPauseMode = true; // 🌟 關鍵標記：這是一次暫停，不是排渣

    // 1. 拍下放電點與路徑進度快照 (暫停的起點)
    for (int i = 0; i < m_Group.axisCount; ++i) {
        jm.frozenPos[i] = (*m_pContexts)[m_Group.axisIndices[i]].logicalCmdPos;
    }
    jm.triggerPos = vAxis.currentCmdPos;

    // 2. 存下安全中心點與 3D 拔高向量
    jm.b3_centerPos[0] = cx; jm.b3_centerPos[1] = cy; jm.b3_centerPos[2] = cz;

    double v_len = std::sqrt(vx * vx + vy * vy + vz * vz);
    jm.b3_retractVector[0] = (v_len > 1e-6) ? vx / v_len : 0.0;
    jm.b3_retractVector[1] = (v_len > 1e-6) ? vy / v_len : 0.0;
    jm.b3_retractVector[2] = (v_len > 1e-6) ? vz / v_len : 1.0;

    // 3. 算出【真實的橫移距離】與【拔高總距離】
    double dx = cx - jm.frozenPos[0];
    double dy = cy - jm.frozenPos[1];
    double dz = cz - jm.frozenPos[2];
    jm.b3_distToCenter = std::sqrt(dx * dx + dy * dy + dz * dz);

    jm.b3_toApexSteps = toApex;
    jm.b3_fromApexSteps = fromApex;
    jm.b3_distToApex = 0;
    for (const auto& s : jm.b3_toApexSteps) jm.b3_distToApex += s.distance;

    // ========================================================
    // 🌟 [神級等比例平帳]：針對「暫停」重新縮放腳本
    // ========================================================
    jm.b3_toCenterSteps = toCenter;
    jm.b3_toWorkpieceSteps = toWorkpiece;

    // 處理第一階段 (To Center) 平帳
    double sum1 = 0; for (auto& s : jm.b3_toCenterSteps) sum1 += s.distance;
    if (sum1 > 1e-6) {
        for (auto& s : jm.b3_toCenterSteps) s.distance *= (jm.b3_distToCenter / sum1);
    }
    else if (!jm.b3_toCenterSteps.empty()) {
        jm.b3_toCenterSteps[0].distance = jm.b3_distToCenter;
    }

    // 處理第五階段 (To Workpiece) 平帳
    double sum4 = 0; for (auto& s : jm.b3_toWorkpieceSteps) sum4 += s.distance;
    if (sum4 > 1e-6) {
        for (auto& s : jm.b3_toWorkpieceSteps) s.distance *= (jm.b3_distToCenter / sum4);
    }
    else if (!jm.b3_toWorkpieceSteps.empty()) {
        jm.b3_toWorkpieceSteps[0].distance = jm.b3_distToCenter;
    }

    // 🌟 數學驗證 LOG：確保暫停幾何正確
    RtPrintf("[PAUSE_B3_MATH] DistCenter:%d, DistApex:%d, TriggerPos:%d\n",
        (int)(jm.b3_distToCenter * 1000.0), (int)(jm.b3_distToApex * 1000.0), (int)jm.triggerPos);

    // 4. 啟動 B3 狀態機
    jm.currentStepIdx = 0;
    jm.currentOffset = 0.0;
    jm.jumpVel = 0.0;
    jm.state = JumpState::B3_TO_CENTER;
    m_Group.pathMode = PathMode::JUMP_TRACKING;

    // 凍結加工速度
    vAxis.currentCmdVel = 0.0;
}

void MotionCore::TriggerPause_B4(double upperCx, double upperCy, double upperCz,double vx, double vy, double vz,const std::vector<JumpSegment>& toUpper,const std::vector<JumpSegment>& toApex,const std::vector<JumpSegment>& fromApex, const std::vector<JumpSegment>& toWorkpiece)
{
    if (m_Group.jumpManager.state != JumpState::IDLE) return;

    PathJumpManager& jm = m_Group.jumpManager;
    AxisContext& vAxis = m_Group.virtualAxis;

    jm.mode = JumpMode::B4_ORBITAL_DIAGONAL;
    jm.isPauseMode = true; // 🌟 關鍵標記：這是一次暫停

    // 1. 拍下實體軸「放電點」快照
    for (int i = 0; i < m_Group.axisCount; ++i) {
        jm.frozenPos[i] = (*m_pContexts)[m_Group.axisIndices[i]].logicalCmdPos;
    }

    // 2. 拍下路徑進度快照 (鎖定生命線)
    jm.triggerPos = vAxis.currentCmdPos;

    // 3. 儲存上中心點與 3D 拔高向量
    jm.b4_upperCenterPos[0] = upperCx;
    jm.b4_upperCenterPos[1] = upperCy;
    jm.b4_upperCenterPos[2] = upperCz;

    double v_len = std::sqrt(vx * vx + vy * vy + vz * vz);
    jm.b4_retractVector[0] = (v_len > 1e-6) ? vx / v_len : 0.0;
    jm.b4_retractVector[1] = (v_len > 1e-6) ? vy / v_len : 0.0;
    jm.b4_retractVector[2] = (v_len > 1e-6) ? vz / v_len : 1.0;

    // 4. 算出【斜向真實距離】(放電點 -> 上中心)
    double dx = upperCx - jm.frozenPos[0];
    double dy = upperCy - jm.frozenPos[1];
    double dz = upperCz - jm.frozenPos[2];
    jm.b4_distToUpper = std::sqrt(dx * dx + dy * dy + dz * dz);

    // 5. 載入腳本
    jm.b4_toApexSteps = toApex;
    jm.b4_fromApexSteps = fromApex;
    jm.b4_distToApex = 0;
    for (const auto& s : jm.b4_toApexSteps) jm.b4_distToApex += s.distance;

    // 🌟 6. [神級等比例平帳] 處理去程與回程
    jm.b4_toUpperSteps = toUpper;
    jm.b4_toWorkpieceSteps = toWorkpiece;

    double sum1 = 0; for (auto& s : jm.b4_toUpperSteps) sum1 += s.distance;
    if (sum1 > 1e-6) {
        for (auto& s : jm.b4_toUpperSteps) s.distance *= (jm.b4_distToUpper / sum1);
    }
    else if (!jm.b4_toUpperSteps.empty()) {
        jm.b4_toUpperSteps[0].distance = jm.b4_distToUpper;
    }

    double sum4 = 0; for (auto& s : jm.b4_toWorkpieceSteps) sum4 += s.distance;
    if (sum4 > 1e-6) {
        for (auto& s : jm.b4_toWorkpieceSteps) s.distance *= (jm.b4_distToUpper / sum4);
    }
    else if (!jm.b4_toWorkpieceSteps.empty()) {
        jm.b4_toWorkpieceSteps[0].distance = jm.b4_distToUpper;
    }

    RtPrintf("[PAUSE_B4] Triggered! DistToUpper:%d, DistToApex:%d\n",
        (int)(jm.b4_distToUpper * 1000.0), (int)(jm.b4_distToApex * 1000.0));

    // 7. 啟動狀態機
    jm.currentStepIdx = 0;
    jm.currentOffset = 0.0;
    jm.jumpVel = 0.0;
    jm.state = JumpState::B4_TO_UPPER_CENTER;
    m_Group.pathMode = PathMode::JUMP_TRACKING;

    // 清空原本軌跡速度
    vAxis.currentCmdVel = 0.0;
}

void MotionCore::Process_Forward_Crossing()
{
    AxisContext& vAxis = m_Group.virtualAxis;
    PathJumpManager& jm = m_Group.jumpManager;

    // 檢查有沒有下一段，沒有就不需要換檔
    if (m_Group.cmdQueue.empty()) return;

    double lastDist = m_Group.currentCmd.mem_totalDist;

    // ==========================================
    // 🌟 核心 A：物理座標補償 (身體跨步)
    // ==========================================
    for (int i = 0; i < m_Group.axisCount; i++) {
        m_Group.startPos[i] += (lastDist * m_Group.ratio[i]);
    }

    // ==========================================
    // 🌟 核心 B：換包裹 (時光機切換)
    // ==========================================
    m_Group.historyQueue.push_back(m_Group.currentCmd);
    m_Group.currentCmd = m_Group.cmdQueue.front();
    m_Group.cmdQueue.pop_front();

    // ==========================================
    // 🌟 核心 C：虛擬進度同步縮放 (維持連續性)
    // ==========================================
    vAxis.currentCmdPos -= lastDist;
    if (jm.state != JumpState::IDLE) {
        jm.triggerPos -= lastDist; // B2 生命線平移
    }

    // ==========================================
    // 🌟 核心 D：大腦索引同步 (通知 Planner 換下一段)
    // ==========================================
    /*
    if (jm.state == JumpState::APPROACHING && jm.mode == JumpMode::B2_PATH_REVERSE) {
        if (jm.currentStepIdx < (int)jm.approachSteps.size() - 1) {
            jm.currentStepIdx++;
        }
    }*/

    // ==========================================
    // 🌟 核心 E：更新方向盤 (Ratio)
    // ==========================================
    m_Group.mode = m_Group.currentCmd.mode;
    for (int i = 0; i < 8; i++) {
        m_Group.ratio[i] = m_Group.currentCmd.mem_ratio[i];
    }

    // 換檔成功 LOG
    RtPrintf("[NORMAL_CROSS] Sync Idx to:%d | vVel:%d\n", jm.currentStepIdx, (int)jm.jumpVel);
}


// 🌟 2. 觸發復歸 (只需給對齊模式和速度)
void MotionCore::TriggerPauseResume(int alignMode, double alignVel, int firstStageMask)
{
    if (m_Group.jumpManager.state != JumpState::PAUSED_HOLD) return;

    PathJumpManager& jm = m_Group.jumpManager;




    // =========================================================
    // 🌟 [神級自動化]：B2 專屬 - 鏡像翻轉並裁剪劇本
    // =========================================================
    if (jm.mode == JumpMode::B2_PATH_REVERSE)
    {
        double actualRetracted = std::abs(jm.currentOffset); // 剛才實際上退了多遠
        jm.approachSteps.clear();
        double accumulated = 0.0;

        // 從退刀劇本的第一段開始，吃到滿為止
        for (const auto& s : jm.retractSteps) {
            double remaining = actualRetracted - accumulated;
            if (remaining <= 1e-6) break;

            JumpSegment newSeg = s;
            if (s.distance > remaining) newSeg.distance = remaining; // 裁剪多出的距離

            jm.approachSteps.push_back(newSeg);
            accumulated += newSeg.distance;
        }
        // 翻轉順序，讓最後退的變成最先回來的
        std::reverse(jm.approachSteps.begin(), jm.approachSteps.end());
    }








    jm.resumeAlignMode = alignMode;
    jm.alignVel = alignVel;
    // 🌟 存入遮罩
    jm.firstStageMask = firstStageMask;

    // 拍下操作員 Jog 完的 8 軸現有座標，並算出要回頂點的總距離！
    double sum_sq = 0.0;
    for (int i = 0; i < m_Group.axisCount; ++i) {
        jm.joggedStartPos[i] = (*m_pContexts)[m_Group.axisIndices[i]].logicalCmdPos;
        double delta = jm.apexPos[i] - jm.joggedStartPos[i];
        sum_sq += (delta * delta);
    }

    jm.alignDist = std::sqrt(sum_sq); // 要走回頂點的總距離
    jm.alignOffset = 0.0;             // 對齊進度歸零
    jm.jumpVel = 0.0;

    // 啟動對齊引擎
    if (alignMode == 0) jm.state = JumpState::RESUME_ALIGN_PRIMARY;
    else if (alignMode == 1) jm.state = JumpState::RESUME_ALIGN_OTHERS;
    else jm.state = JumpState::RESUME_ALIGN_ALL;

    m_Group.pathMode = PathMode::JUMP_TRACKING;
}





// =================================================================
// 🟢 [新增] 單軸獨立梯形規劃器 (跳刀專用)
// 負責計算每一毫秒的加減速與位移，並自動處理方向
// =================================================================
double MotionCore::PlanTrapezoidal(double currentPos, double targetPos, double maxVel, double acc, double dec, double& currentVel, double dt)
{
    double error = targetPos - currentPos;
    double dir = (error >= 0.0) ? 1.0 : -1.0;
    double distLeft = std::abs(error);
    double currentSpeed = std::abs(currentVel);

    // 🟢 [改良 1] 完美降落：必須「距離極短」且「速度已經極慢」才能切斷動力，徹底消滅紫色尖刺！
    if (distLeft < 0.001 || (currentSpeed < 0.01 && distLeft < 0.5)) {
        currentVel = 0.0;
        return targetPos;
    }

    // 2. 計算煞車距離 (公式: v^2 / 2a)
    // 預判目前的車速，需要多少距離才能煞停
    double stopDist = (currentSpeed * currentSpeed) / (2.0 * dec);

    double targetSpeed = 0.0;

    // 3. 判斷要加速還是減速
    if (distLeft <= stopDist) {
        // 已經進入煞車區！強迫把目標速度設為 0
        targetSpeed = 0.0;
    }
    else {
        // 還在安全區，可以往最高速衝刺
        targetSpeed = maxVel;
    }

    // 4. 執行速度斜坡 (Acc / Dec)
    double activeAcc = (targetSpeed >= currentSpeed) ? acc : dec;

    if (currentSpeed < targetSpeed) {
        currentSpeed += activeAcc * dt;
        if (currentSpeed > targetSpeed) currentSpeed = targetSpeed;
    }
    else if (currentSpeed > targetSpeed) {
        currentSpeed -= activeAcc * dt;
        if (currentSpeed < targetSpeed) currentSpeed = targetSpeed;
    }
    // 🟢 [改良 2] 防卡死蠕動：如果速度變 0 但還沒碰到終點，給予微小推力，保證 100% 抵達
    if (currentSpeed < 0.0001 && distLeft > 0.001) {
        currentSpeed = 0.0001;
    }

    currentVel = dir * currentSpeed;
    double nextPos = currentPos + (currentVel * dt);

    // 🟢 [改良 3] 防過沖保護：確保這 1ms 跨出去絕對不會超過目標點，避免抖動
    if ((dir > 0.0 && nextPos > targetPos) || (dir < 0.0 && nextPos < targetPos)) {
        currentVel = 0.0;
        return targetPos;
    }

    return nextPos;
}
double MotionCore::PlanTrapezoidal_B2(double currentPos, double targetPos, double maxVel, double acc, double dec, double& currentVel, double dt)
{
    double error = targetPos - currentPos;
    double dir = (error >= 0.0) ? 1.0 : -1.0;
    double distLeft = std::abs(error);

    // 🌟 1. 方向反轉偵測 (把門檻降到極低：只要有 0.1 的微小速度，就必須保護)
    bool isReversing = (currentVel > 0.1 && dir < 0) || (currentVel < -0.1 && dir > 0);

    // 🌟 2. 目標速度決策
    double targetSpeed = 0.0;
    if (isReversing) {
        targetSpeed = 0.0; // 只要方向不對，唯一目標就是停下來
    }
    else {
        // 計算煞車距離
        double stopDist = (currentVel * currentVel) / (2.0 * dec);
        targetSpeed = (distLeft <= stopDist) ? 0.0 : maxVel;
    }

    // 🌟 3. 核心：強制斜坡爬升與下降 (消滅垂直線)
    double targetVelWithDir = dir * targetSpeed;
    if (isReversing) targetVelWithDir = 0.0; // 煞車時維持原本的符號，朝 0 逼近

    if (currentVel < targetVelWithDir) {
        currentVel += acc * dt;
        if (currentVel > targetVelWithDir && !isReversing) currentVel = targetVelWithDir;
    }
    else if (currentVel > targetVelWithDir) {
        currentVel -= dec * dt;
        if (currentVel < targetVelWithDir && !isReversing) currentVel = targetVelWithDir;
    }

    // 容許誤差：萬分之一圈 (1/10000 Rev)
    double posTolerance = 16777216.0 / 10000.0; // 大約 1677 Pulse
    double velTolerance = 16777216.0 / 1000.0;  // 大約 16777 Pulse/sec (約 0.06 RPM)

    if (distLeft < posTolerance && std::abs(currentVel) < velTolerance) {
        currentVel = 0.0;
        return targetPos;
    }

    return currentPos + (currentVel * dt);
}
void MotionCore::UpdateInterpolation()
{
    // 1. 基本防呆
    if (m_pContexts == nullptr) return;


    // [安全門] 檢查參與群組的所有實體軸是否全部激磁
    // 如果有任何一軸沒激磁，嚴禁進行任何插補計算，直接跳出。
    for (int i = 0; i < m_Group.axisCount; i++)
    {
        int axisIdx = m_Group.axisIndices[i];
        if (!(*m_pContexts)[axisIdx].isServoOn)
        {
            m_Group.isActive = false; // 強制將群組設為非運作狀態
            return; // 這裡不執行任何插補計算，也不輸出任何位置
        }
    }




    AxisContext& vAxis = m_Group.virtualAxis; // 先取得 vAxis 的引用
    double dt = CYCLE_TIME_SEC;
    PathJumpManager& jm = m_Group.jumpManager;
    AxisCommand vCmd; // 準備統一收集速度與位置




    // ======================================================
    // 🌟 [司機 A] 跳刀狀態機接管 (優先權最高)
    // ======================================================
    if (jm.state != JumpState::IDLE)
    {
        if (jm.state == JumpState::RETRACTING)
        {
            JumpSegment& seg = jm.retractSteps[jm.currentStepIdx];
            double realAcc = (seg.accTime > 0.0001) ? (seg.velocity / seg.accTime) : 1e10;
            double realDec = (seg.decTime > 0.0001) ? (seg.velocity / seg.decTime) : 1e10;

            // 🌟 1. 找出「整個退刀」的最終目標 (不到最後一刻，規劃器不會煞車！)
            double finalTarget = 0.0;
            for (const auto& s : jm.retractSteps) finalTarget -= s.distance;

            // 🌟 2. 找出「當前這一段」的換檔邊界
            double stepBoundary = 0.0;
            for (int i = 0; i <= jm.currentStepIdx; ++i) stepBoundary -= jm.retractSteps[i].distance;




            // =========================================================
            // 🌟 [只有 B2 才需要] 無腦防呆機制：自動偵測歷史極限
            // =========================================================
            if (jm.mode == JumpMode::B2_PATH_REVERSE && m_Group.historyQueue.empty()) {
                double absoluteOrigin = -jm.triggerPos;
                // 強制把大腦的目標截斷在原點，絕對不准多退！
                if (finalTarget < absoluteOrigin) finalTarget = absoluteOrigin;
            }


            // =========================================================
            // 🌟 3. [微創修改] 獨立隔離 B2 與加工暫停的規劃器
            // =========================================================
            if (jm.mode == JumpMode::B2_PATH_REVERSE || jm.isPauseMode)
            {
                // 👉 關鍵 A：時光機的觸發點是 vAxis.currentCmdPos < 0
                double trueBoundary = -jm.triggerPos - 1.0;

                // 確保退刀不要退超過最終的總目標
                if (trueBoundary < finalTarget) {
                    trueBoundary = finalTarget;
                }

                // 呼叫規劃器
                jm.currentOffset = PlanTrapezoidal_B2(jm.currentOffset, trueBoundary, seg.velocity, realAcc, realDec, jm.jumpVel, dt);

                // 👉👉👉 【關鍵修正】：比對對象改為 trueBoundary 👈👈👈
                // 這樣不管是退到腳本極限，還是退到歷史盡頭，大腦都會認可「到站了」
                if (std::abs(jm.currentOffset - trueBoundary) < 5.0 && std::abs(jm.jumpVel) < 10.0) {

                    // 補回換檔邏輯 (如果還有下一段腳本)
                    if (jm.currentStepIdx < (int)jm.retractSteps.size() - 1 && std::abs(jm.currentOffset - finalTarget) > 10.0) {
                        jm.currentStepIdx++;
                    }
                    // 抵達最終目標 (不管是物理的還是腳本的)
                    else {
                        if (jm.isPauseMode) {
                            jm.state = JumpState::PAUSED_HOLD;
                            jm.jumpVel = 0.0;
                            for (int i = 0; i < m_Group.axisCount; ++i) {
                                jm.apexPos[i] = (*m_pContexts)[m_Group.axisIndices[i]].logicalCmdPos;
                            }
                           // RtPrintf("[B2] 成功抵達物理頂點，切換至 PAUSED_HOLD\n");
                        }
                        else {
                            jm.state = JumpState::DWELL;
                            jm.dwellTimer = 0.0;
                        }
                    }
                }
            }
            else 
            {
                // B0, B1 等其他模式，走原本的規劃器 (完全不影響舊功能)
                jm.currentOffset = PlanTrapezoidal(jm.currentOffset, finalTarget, seg.velocity, realAcc, realDec, jm.jumpVel, dt);
            }
            // =========================================================

            // 🌟 4. 動態換檔：只要跨越了邊界，立刻切換下一段的速度 (不要求速度降到 0)
            
            if (jm.currentOffset <= stepBoundary && jm.currentStepIdx < (int)jm.retractSteps.size() - 1) {
                jm.currentStepIdx++;
            }

            // 🌟 5. 真正抵達最高點，才進入 DWELL
            if (std::abs(jm.currentOffset - finalTarget) < 0.0001 && std::abs(jm.jumpVel) < 0.1) {
                // 🌟 修改 1：在這裡攔截暫停！如果是暫停，就停在空中並拍下快照。
                if (jm.isPauseMode) {
                    jm.state = JumpState::PAUSED_HOLD;
                    jm.jumpVel = 0.0;
                    for (int i = 0; i < m_Group.axisCount; ++i) {
                        jm.apexPos[i] = (*m_pContexts)[m_Group.axisIndices[i]].logicalCmdPos;
                    }
                }
                else {
                    jm.state = JumpState::DWELL;
                    jm.dwellTimer = 0.0;
                }
            }
        }
        else if (jm.state == JumpState::DWELL)
        {
            jm.dwellTimer += dt * 1000.0;
            if (jm.dwellTimer >= jm.dwellTimeTarget) {
                jm.state = JumpState::APPROACHING;
                jm.currentStepIdx = 0;
            }

            // =========================================================
                 // 🌟 [神級邏輯]：自動將退刀劇本翻轉為進刀劇本
                 // =========================================================
            if (jm.mode == JumpMode::B2_PATH_REVERSE)
            {
                // 1. 直接複製退刀劇本 (確保段數一模一樣)
                jm.approachSteps = jm.retractSteps;

                // 2. 翻轉順序：原本最後退的段落，變成最先跑的進刀段落
                std::reverse(jm.approachSteps.begin(), jm.approachSteps.end());

                // 3. 處理「殘留距離」：扣除多出來的部分
                // 實際退刀總距離就是 std::abs(jm.currentOffset)
                double totalActualMoved = std::abs(jm.currentOffset);
                double accumulatedDist = 0.0;

                // 從最後一段進刀往回看 (也就是最初的退刀段落)
                // 我們要確保進刀的總距離剛好等於實際退刀的距離
                for (int i = (int)jm.approachSteps.size() - 1; i >= 0; --i) {
                    double stepDist = jm.approachSteps[i].distance;
                    if (accumulatedDist + stepDist > totalActualMoved) {
                        // 這一小段就是被「截斷」的地方
                        jm.approachSteps[i].distance = totalActualMoved - accumulatedDist;
                        // 前面那些還沒算到的段落通通歸零 (因為退刀根本沒走到那邊)
                        for (int k = 0; k < i; ++k) jm.approachSteps[k].distance = 0.0;
                        break;
                    }
                    accumulatedDist += stepDist;
                }

                // 4. (選配) 如果你有特殊的尋邊速度要求，可以在這裡覆寫最後一段的速度
                // jm.approachSteps.back().velocity = ONE_REV * 1.0; 
            }
        }
        else if (jm.state == JumpState::APPROACHING)
        {
            // 🌟 [防爆衝護欄] 防止 Idx 越界崩潰 (Access Violation 殺手)
            if (jm.approachSteps.empty() || jm.currentStepIdx >= (int)jm.approachSteps.size() || jm.currentStepIdx < 0) 
            {
            RtPrintf("!!! ERROR !!! Approach Index Out of Range!\n");
            jm.state = JumpState::IDLE;
            m_Group.pathMode = PathMode::PATH_SERVO;
            return;
            }

            // =========================================================
            // 🌟 [完全分流]：只有 B2 模式才執行的邏輯
            // =========================================================
            if (jm.mode == JumpMode::B2_PATH_REVERSE)
            {
                // 👉 沒錯！就只要這一行！把大腦完全交給副程式去算！
                Process_B2_Approach_Planner(dt);
            }
            else
            {
                // ✅ [原本程式]：B0, B1, B3, B4 維持原封不動的邏輯
                JumpSegment& seg = jm.approachSteps[jm.currentStepIdx];
                double realAcc = (seg.accTime > 0.0001) ? (seg.velocity / seg.accTime) : 1e10;
                double realDec = (seg.decTime > 0.0001) ? (seg.velocity / seg.decTime) : 1e10;

                // 🌟 1. 進刀的最終目標：回到出發點 (0.0)
                double finalTarget = 0.0;

                // 🌟 2. 算出總起點 (腳本極限)
                double startOffset = 0.0;
                for (const auto& s : jm.retractSteps) startOffset -= s.distance;

                double stepBoundary = startOffset;
                for (int i = 0; i <= jm.currentStepIdx; ++i) stepBoundary += jm.approachSteps[i].distance;

                // 原本的規劃器
                jm.currentOffset = PlanTrapezoidal(jm.currentOffset, finalTarget, seg.velocity, realAcc, realDec, jm.jumpVel, dt);

                // 原本的動態換檔 (不要求速降為 0)
                if (jm.currentOffset >= stepBoundary && jm.currentStepIdx < (int)jm.approachSteps.size() - 1) {
                    jm.currentStepIdx++;
                }
            }

          

          
            // 🌟 5. 真正降落完畢 (這段保留)
            double finalTarget = 0.0;
            // 🌟 5. 真正降落完畢 (這段是共用的，但判斷門檻放寬到 5.0 Pulse)
            if (std::abs(jm.currentOffset - finalTarget) < 5.0 && std::abs(jm.jumpVel) < 10.0) {

                int idxX = m_Group.axisIndices[0], idxY = m_Group.axisIndices[1];
                (*m_pContexts)[idxX].logicalCmdPos = jm.frozenPos[0];
                (*m_pContexts)[idxY].logicalCmdPos = jm.frozenPos[1];

                RtPrintf("[JUMP END] Back to Spark Point! Offset:%d\n", (int)jm.currentOffset);

                jm.state = JumpState::IDLE;
                jm.currentOffset = 0.0;
                m_Group.pathMode = PathMode::PATH_SERVO;
            }
        }







        // 🔴 把跳刀算出來的 Offset 套用到虛擬主軸上
        // ======================================================
        // 🔴 核心分流：B2 是倒退嚕，B1/B3/B4 是凍結放電進度！
        // ======================================================
        if (jm.mode == JumpMode::B2_PATH_REVERSE) 
        {
            // B2：把跳刀 Offset 套用到虛擬主軸上 (原路徑倒退)
            vAxis.currentCmdPos = jm.triggerPos + jm.currentOffset;
            vAxis.currentCmdVel = jm.jumpVel;
        }
        else 
        {
            // B1, B0, B3, B4：【凍結】虛擬主軸！讓放電軌跡停在半空中
            vAxis.currentCmdPos = jm.triggerPos;
            vAxis.currentCmdVel = 0.0;
        }

        vCmd.instantCmdPos = vAxis.currentCmdPos;
        vCmd.instantCmdVel = vAxis.currentCmdVel;
    }
    // ======================================================
    // 🌟 [司機 B] PATH_SERVO 放電上帝模式 (優先權次高)
    // ======================================================
    else if (m_Group.pathMode == PathMode::PATH_SERVO)
    {
        // 拆包裹邏輯
        if ((!m_Group.isActive || vAxis.inPosition) && !m_Group.cmdQueue.empty()) {
            LoadNextCommand();
        }
        if (!m_Group.isActive) return;

        vAxis.cruiseVel_PPS = vAxis.maxVel_PPS * m_Group.feedrateOverride;

        

       // 🟢 [修復]：把外部速度當作「目標」，交給速度規劃器去產生平滑的斜坡與 S-Curve！
        //vAxis.targetVelocity = m_Group.pathServoVel;
        //Calc_Trajectory_Velocity(vAxis, vCmd);




        // =========================================================
        // 🟢 智慧分流：排渣回來的那一次 (S-Curve) vs 正常放電 (即時速度)
        // =========================================================
        if (jm.isRecovering)
        {
            // 【狀態 1：軟著陸中】交給規劃器，畫出平滑起步的斜坡，避免突刺！
            vAxis.targetVelocity = m_Group.pathServoVel;
            Calc_Trajectory_Velocity(vAxis, vCmd);

         
            //jm.isRecovering = false;//
        }
        else
        {
            // 【Path Servo 模式】：由外部速度積分
            vAxis.currentCmdVel = m_Group.pathServoVel;
            vAxis.currentCmdPos += vAxis.currentCmdVel * dt;
        }
        // =========================================================








        // 🟢 關鍵修復：防止鬼畜卡死！
        if (vAxis.currentCmdPos < vAxis.finalTargetPos) {
            vAxis.inPosition = false;
        }

        // 🔴 【時光機 Step 3：向後跨節 (倒退嚕)】
        if (vAxis.currentCmdPos < 0.0)
        {
            if (m_Group.enableHistory && !m_Group.historyQueue.empty())
            {
                // 🟢 監視器 2：時光機啟動 (直接轉 int，不乘 1000)
                RtPrintf("[LOG 2 - HIST_TRIG] vPos: %d, vVel: %d, Offset: %d\n",
                    (int)vAxis.currentCmdPos,
                    (int)vAxis.currentCmdVel,
                    (int)jm.currentOffset);

                m_Group.cmdQueue.push_front(m_Group.currentCmd);
                m_Group.currentCmd = m_Group.historyQueue.back();
                m_Group.historyQueue.pop_back();

                // 恢復大腦的幾何狀態
                m_Group.mode = m_Group.currentCmd.mode;
                m_Group.axisCount = m_Group.currentCmd.axisCount;
                for (int i = 0; i < 8; i++) {
                    m_Group.startPos[i] = m_Group.currentCmd.mem_startPos[i];
                    m_Group.ratio[i] = m_Group.currentCmd.mem_ratio[i];
                    m_Group.axisIndices[i] = m_Group.currentCmd.axisIndices[i];
                }
                m_Group.radius = m_Group.currentCmd.mem_radius;
                m_Group.startAngle = m_Group.currentCmd.mem_startAngle;
                m_Group.centerX = m_Group.currentCmd.mem_centerX;
                m_Group.centerY = m_Group.currentCmd.mem_centerY;

                // 🟢 恢復 3D 幾何與旋轉矩陣！
                m_Group.totalDist3D = m_Group.currentCmd.mem_totalDist;
                m_Group.totalAngle = m_Group.currentCmd.mem_totalAngle;

                m_Group.enableTransform = m_Group.currentCmd.mem_enableTransform;
                for (int i = 0; i < 3; ++i) {
                    m_Group.transformOrigin[i] = m_Group.currentCmd.mem_transformOrigin[i];
                    for (int j = 0; j < 3; ++j) {
                        m_Group.transformMatrix[i][j] = m_Group.currentCmd.mem_transformMatrix[i][j];
                    }
                }
            }
            else
            {
                vAxis.currentCmdPos = 0.0;
                vAxis.currentCmdVel = 0.0;
            }
        }
        else if (vAxis.currentCmdPos >= vAxis.finalTargetPos)
        {
            vAxis.currentCmdPos = vAxis.finalTargetPos;
            vAxis.inPosition = true;
        }

        vCmd.instantCmdPos = vAxis.currentCmdPos;
        vCmd.instantCmdVel = vAxis.currentCmdVel;
    }
    // ======================================================
    // 🌟 [司機 C] 正常加工模式 (優先權最低)
    // ======================================================
    else
    {
        // 拆包裹邏輯
        if ((!m_Group.isActive || vAxis.inPosition) && !m_Group.cmdQueue.empty()) {
            LoadNextCommand();
        }
        if (!m_Group.isActive) return;

        vAxis.cruiseVel_PPS = vAxis.maxVel_PPS * m_Group.feedrateOverride;

        if (vAxis.state == MotionState::MotionState_STOPPING) {
            Calc_Trajectory_Velocity(vAxis, vCmd);
        }
        else {
            Calc_Trajectory_Trapezoidal(vAxis, vCmd);
        }
    }



    // =========================================================
    // 🔴 獨立的時光機 (向後跨節)：必須放在司機分流的外面！
    // =========================================================
    if (vAxis.currentCmdPos < 0.0)
    {
        if (m_Group.enableHistory && !m_Group.historyQueue.empty())
        {
            m_Group.cmdQueue.push_front(m_Group.currentCmd);
            m_Group.currentCmd = m_Group.historyQueue.back();
            m_Group.historyQueue.pop_back();

            // 恢復大腦的幾何狀態
            m_Group.mode = m_Group.currentCmd.mode;
            m_Group.axisCount = m_Group.currentCmd.axisCount;
            for (int i = 0; i < 8; i++) {
                m_Group.startPos[i] = m_Group.currentCmd.mem_startPos[i];
                m_Group.ratio[i] = m_Group.currentCmd.mem_ratio[i];
                m_Group.axisIndices[i] = m_Group.currentCmd.axisIndices[i];
            }
            m_Group.radius = m_Group.currentCmd.mem_radius; m_Group.startAngle = m_Group.currentCmd.mem_startAngle;
            m_Group.centerX = m_Group.currentCmd.mem_centerX; m_Group.centerY = m_Group.currentCmd.mem_centerY;
            m_Group.totalDist3D = m_Group.currentCmd.mem_totalDist; m_Group.totalAngle = m_Group.currentCmd.mem_totalAngle;

            m_Group.enableTransform = m_Group.currentCmd.mem_enableTransform;
            for (int i = 0; i < 3; ++i) {
                m_Group.transformOrigin[i] = m_Group.currentCmd.mem_transformOrigin[i];
                for (int j = 0; j < 3; ++j) m_Group.transformMatrix[i][j] = m_Group.currentCmd.mem_transformMatrix[i][j];
            }

            // 🌟 終極修復：跨節後，必須把剩餘的負數距離，灌給新的線條長度！
            vAxis.currentCmdPos += m_Group.currentCmd.mem_totalDist;

            // 🌟 如果是 B2 模式，必須同步平移起點，才不會無限閃退！
            if (jm.state != JumpState::IDLE) jm.triggerPos += m_Group.currentCmd.mem_totalDist;
        }
        else
        {
            // 🌟 已經退到最最最起點 (歷史空了)，強制鎖死在 0.0！
            vAxis.currentCmdPos = 0.0;
            vAxis.currentCmdVel = 0.0;
        }
    }// =========================================================
    // 🟢 【時光機：向前跨節 (進刀回歸)】 - 解決暴衝的對齊核心
    // =========================================================
    else if (vAxis.currentCmdPos > m_Group.currentCmd.mem_totalDist)
    {
        Process_Forward_Crossing(); // 👈 身體換檔全交給副程式！
    }

    vCmd.instantCmdPos = vAxis.currentCmdPos;
    vCmd.instantCmdVel = vAxis.currentCmdVel;

   



    // ======================================================
    // 🌟 以下完全保留你原本的幾何分配與空間旋轉 (原封不動！)
    // ======================================================
    if (m_Group.mode == InterpolationMode::LINEAR)
    {
        for (int i = 0; i < m_Group.axisCount; ++i) {
            int idx = m_Group.axisIndices[i];
            AxisContext& realAxis = (*m_pContexts)[idx];
            realAxis.logicalCmdPos = m_Group.startPos[i] + (vCmd.instantCmdPos * m_Group.ratio[i]);
            realAxis.logicalCmdVel = vCmd.instantCmdVel * m_Group.ratio[i];
        }
    }
    else if (m_Group.mode == InterpolationMode::CIRCULAR_CW || m_Group.mode == InterpolationMode::CIRCULAR_CCW)
    {
        int idxX = m_Group.axisIndices[0];
        int idxY = m_Group.axisIndices[1];
        AxisContext& realX = (*m_pContexts)[idxX];
        AxisContext& realY = (*m_pContexts)[idxY];

        double progressRatio = vCmd.instantCmdPos / m_Group.totalDist3D;
        double current_angle = m_Group.startAngle + (progressRatio * m_Group.totalAngle);

        double r_start = m_Group.currentCmd.startRadius;
        double r_end = m_Group.currentCmd.endRadius;
        double dynamicRadius = r_start + (r_end - r_start) * progressRatio;

        realX.logicalCmdPos = m_Group.centerX + m_Group.radius * std::cos(current_angle);
        realY.logicalCmdPos = m_Group.centerY + m_Group.radius * std::sin(current_angle);

        if (m_Group.axisCount >= 3) {
            int idxZ = m_Group.axisIndices[2];
            AxisContext& realZ = (*m_pContexts)[idxZ];
            double totalDeltaZ = m_Group.currentCmd.targetPos[2] - m_Group.startPos[2];
            realZ.logicalCmdPos = m_Group.startPos[2] + (totalDeltaZ * progressRatio);
            realZ.logicalCmdVel = vCmd.instantCmdVel * (totalDeltaZ / m_Group.totalDist3D);
        }

        double xyVelRatio = (m_Group.radius * std::abs(m_Group.totalAngle)) / m_Group.totalDist3D;
        double planarVel = vCmd.instantCmdVel * xyVelRatio;
        if (m_Group.mode == InterpolationMode::CIRCULAR_CW) planarVel = -planarVel;

        realX.logicalCmdVel = -planarVel * std::sin(current_angle);
        realY.logicalCmdVel = planarVel * std::cos(current_angle);
    }


    // =====================================================================
      // 🌟 🟢 [全新架構：跳刀 3D 向量疊加層] 🟢 🌟
      // =====================================================================
    if (jm.state != JumpState::IDLE)
    {
        double offsetDist = std::abs(jm.currentOffset);
        double offsetVel = std::abs(jm.jumpVel);
        int sign = (jm.state == JumpState::RETRACTING) ? 1 : -1; // 退刀加，進刀減


        // 🌟 修改 2：加入復歸對齊引擎 (獨立運作，不干擾原本降落邏輯)
        if (jm.state == JumpState::RESUME_ALIGN_PRIMARY ||
            jm.state == JumpState::RESUME_ALIGN_OTHERS ||
            jm.state == JumpState::RESUME_ALIGN_ALL)
        {
            for (int i = 0; i < m_Group.axisCount; ++i) {

                bool isFirstStageAxis = (jm.firstStageMask & (1 << i)) != 0;
                bool shouldMove = false;
                if (jm.state == JumpState::RESUME_ALIGN_ALL) shouldMove = true;
                else if (jm.state == JumpState::RESUME_ALIGN_PRIMARY) shouldMove = isFirstStageAxis;
                else if (jm.state == JumpState::RESUME_ALIGN_OTHERS) shouldMove = !isFirstStageAxis;

                int axisIdx = m_Group.axisIndices[i];
                if (shouldMove && jm.alignDist > 1e-5) {
                    double delta = jm.apexPos[i] - jm.joggedStartPos[i];
                    (*m_pContexts)[axisIdx].logicalCmdPos = jm.joggedStartPos[i] + (delta * (jm.alignOffset / jm.alignDist));
                }
                else {
                    // 🌟 終極護盾：如果這軸這回合不動，或者根本沒有距離 (alignDist=0)，
                    // 必須強制把它鎖死在原位，防止被上方的 LINEAR 幾何破壞！
                    (*m_pContexts)[axisIdx].logicalCmdPos = jm.joggedStartPos[i];
                }
                // 🌟 對齊期間，速度交給引擎算，非移動軸強制為 0
                (*m_pContexts)[axisIdx].logicalCmdVel = 0.0;
            }

            if (jm.alignDist < 1e-5 || jm.alignOffset >= jm.alignDist - 1e-5)
            {
                if (jm.state == JumpState::RESUME_ALIGN_PRIMARY && jm.resumeAlignMode == 0)
                {
                    // 🟢 第一階段 (Primary) 結束：只把「有參與第一階段」的軸，起點更新為頂點
                    for (int i = 0; i < m_Group.axisCount; ++i) {
                        if ((jm.firstStageMask & (1 << i)) != 0) jm.joggedStartPos[i] = jm.apexPos[i];
                    }
                    jm.state = JumpState::RESUME_ALIGN_OTHERS;
                    jm.alignOffset = 0.0; jm.jumpVel = 0.0;

                    // 🌟 重新計算第二階段的總距離！
                    double sum_sq = 0.0;
                    for (int i = 0; i < m_Group.axisCount; ++i) {
                        double delta = jm.apexPos[i] - jm.joggedStartPos[i];
                        sum_sq += delta * delta;
                    }
                    jm.alignDist = std::sqrt(sum_sq);
                }
                else if (jm.state == JumpState::RESUME_ALIGN_OTHERS && jm.resumeAlignMode == 1)
                {
                    // 🟢 第一階段 (Others) 結束：只把「有參與 Others」的軸，起點更新為頂點
                    for (int i = 0; i < m_Group.axisCount; ++i) {
                        if ((jm.firstStageMask & (1 << i)) == 0) jm.joggedStartPos[i] = jm.apexPos[i];
                    }
                    jm.state = JumpState::RESUME_ALIGN_PRIMARY;
                    jm.alignOffset = 0.0; jm.jumpVel = 0.0;

                    // 🌟 重新計算第二階段的總距離！
                    double sum_sq = 0.0;
                    for (int i = 0; i < m_Group.axisCount; ++i) {
                        double delta = jm.apexPos[i] - jm.joggedStartPos[i];
                        sum_sq += delta * delta;
                    }
                    jm.alignDist = std::sqrt(sum_sq);
                }
                else
                {
                    // 🟢 所有對齊階段都結束了！完美收尾。
                    for (int i = 0; i < m_Group.axisCount; ++i) jm.joggedStartPos[i] = jm.apexPos[i];
                    jm.alignOffset = 0.0; jm.jumpVel = 0.0;

                    // =========================================================
                    // 🌟 完美銜接：根據模式把機台推回軌道
                    // =========================================================
                    if (jm.mode == JumpMode::B3_CENTER) jm.state = JumpState::B3_FROM_APEX;
                    else if (jm.mode == JumpMode::B4_ORBITAL_DIAGONAL) jm.state = JumpState::B4_FROM_APEX;
                    else jm.state = JumpState::APPROACHING;

                    jm.isPauseMode = false; jm.currentStepIdx = 0;

                    // ⚠️ 還原降落起點 (B2 絕對不能動！)
                    if (jm.mode == JumpMode::B0_REVERSE || jm.mode == JumpMode::B1_SPECIFIC_AXIS) {
                        double startOffset = 0.0;
                        for (const auto& s : jm.retractSteps) startOffset -= s.distance;
                        jm.currentOffset = startOffset;
                    }
                }
            }
            else {
                double acc = jm.alignVel * 5.0;
                jm.alignOffset = PlanTrapezoidal(jm.alignOffset, jm.alignDist, jm.alignVel, acc, acc, jm.jumpVel, dt);
            }
        }
        else if (jm.mode == JumpMode::B1_SPECIFIC_AXIS)
        {
            int targetIdx = jm.b1_AxisIndex;
            // 絕對鎖死：起點 + (現在的距離 * 向量)
            (*m_pContexts)[targetIdx].logicalCmdPos = jm.frozenPos[targetIdx] + (offsetDist * jm.b1_dir);
            (*m_pContexts)[targetIdx].logicalCmdVel = offsetVel * jm.b1_dir * sign;


        }
        else if (jm.mode == JumpMode::B0_REVERSE)
        {
            for (int i = 0; i < m_Group.axisCount; ++i) {
                int axisIdx = m_Group.axisIndices[i];
                double vec_component = jm.b0_Vector[i];
                // 絕對鎖死：起點 + (現在的距離 * 向量)
                (*m_pContexts)[axisIdx].logicalCmdPos = jm.frozenPos[axisIdx] + (offsetDist * vec_component);
                (*m_pContexts)[axisIdx].logicalCmdVel = (offsetVel * sign) * vec_component;
            }
        }// ======================================================
        // 🌟 B3 專屬的 5 階段狀態機 (完美轉角煞停 + 內部融合)
        // ======================================================
        else if (jm.mode == JumpMode::B3_CENTER && (jm.state >= JumpState::B3_TO_CENTER && jm.state <= JumpState::B3_TO_WORKPIECE || jm.state == JumpState::PAUSED_HOLD))
        {
            std::vector<JumpSegment>* currentScript = nullptr;
            double finalTarget = 0.0;
            JumpState nextState = JumpState::IDLE;

            // 1. 根據當前狀態，選擇對應的腳本與目標
            if (jm.state == JumpState::B3_TO_CENTER) {
                currentScript = &jm.b3_toCenterSteps;
                finalTarget = jm.b3_distToCenter;
                nextState = JumpState::B3_TO_APEX;
            }
            else if (jm.state == JumpState::B3_TO_APEX) {
                currentScript = &jm.b3_toApexSteps;
                finalTarget = jm.b3_distToApex;
                nextState = JumpState::B3_DWELL;
            }
            else if (jm.state == JumpState::B3_FROM_APEX) {
                currentScript = &jm.b3_fromApexSteps;
                finalTarget = jm.b3_distToApex;
                nextState = JumpState::B3_TO_WORKPIECE;
            }
            else if (jm.state == JumpState::B3_TO_WORKPIECE) {
                currentScript = &jm.b3_toWorkpieceSteps;
                finalTarget = jm.b3_distToCenter;
                nextState = JumpState::IDLE;
            }

            // 2. 處理 DWELL 狀態
            if (jm.state == JumpState::B3_DWELL) {
                jm.dwellTimer += dt * 1000.0;
                if (jm.dwellTimer >= jm.dwellTimeTarget) {
                    jm.state = JumpState::B3_FROM_APEX;
                    jm.currentStepIdx = 0; jm.currentOffset = 0.0; jm.jumpVel = 0.0;
                }
            }
            // 3. 處理移動狀態 (執行規劃器與速度融合)
            else if (currentScript != nullptr && !currentScript->empty())
            {
                JumpSegment& seg = (*currentScript)[jm.currentStepIdx];
                double realAcc = (seg.accTime > 0.0001) ? (seg.velocity / seg.accTime) : 1e10;
                double realDec = (seg.decTime > 0.0001) ? (seg.velocity / seg.decTime) : 1e10;

                // 🌟 內部融合邊界判斷
                double stepBoundary = 0.0;
                for (int i = 0; i <= jm.currentStepIdx; ++i) stepBoundary += (*currentScript)[i].distance;

                // 呼叫規劃器 (往 finalTarget 前進)
                jm.currentOffset = PlanTrapezoidal(jm.currentOffset, finalTarget, seg.velocity, realAcc, realDec, jm.jumpVel, dt);

                // 跨越內部腳本，無縫換檔
                if (jm.currentOffset >= stepBoundary && jm.currentStepIdx < (int)currentScript->size() - 1) {
                    jm.currentStepIdx++;
                }
                // 🌟 【真・防爆衝換檔】：只要距離大於等於目標(容許微小誤差)，立刻強制斬斷！絕不讓規劃器發瘋反轉！
                if (jm.currentOffset >= finalTarget - 1e-5) {

                    jm.currentStepIdx = 0;
                    jm.currentOffset = 0.0; // 進入下一階段，里程碑強制歸零！
                    jm.jumpVel = 0.0;       // 速度強制歸零煞停！
                    jm.dwellTimer = 0.0;

                    // ==========================================================
                    // 🌟 [絕對關鍵：暫停空中攔截網]
                    // 如果剛走完 TO_APEX 抵達最高點，且是暫停模式，直接鎖死在 PAUSED_HOLD！
                    // ==========================================================
                    if (jm.state == JumpState::B3_TO_APEX && jm.isPauseMode) {
                        jm.state = JumpState::PAUSED_HOLD;

                        // 💥 破案核心：絕對不能用 logicalCmdPos 拍快照！那會有 1ms 的落後誤差！
                         // 必須直接用幾何數學，算出 100% 完美的實體頂點座標！
                        jm.apexPos[0] = jm.b3_centerPos[0] + jm.b3_distToApex * jm.b3_retractVector[0];
                        jm.apexPos[1] = jm.b3_centerPos[1] + jm.b3_distToApex * jm.b3_retractVector[1];
                        if (m_Group.axisCount >= 3) {
                            jm.apexPos[2] = jm.b3_centerPos[2] + jm.b3_distToApex * jm.b3_retractVector[2];
                        }
                        RtPrintf("[PAUSE_B3] Safely Holding at Math Apex.\n");
                    }
                    else {
                        // 正常的排渣模式，或者是 B3 的其他階段，就順順切換到下一個狀態
                        jm.state = nextState;
                    }

                    // ==========================================================
                    // 🌟 【神級修復：歸還放電控制權】
                    // 如果第四段走完，狀態變成 IDLE，必須立刻把鑰匙還給 PATH_SERVO！
                    // ==========================================================
                    if (jm.state == JumpState::IDLE) {
                        m_Group.pathMode = PathMode::PATH_SERVO; // 交還給上帝模式
                        jm.isRecovering = true; // 啟動軟著陸 (無縫接軌放電速度)

                        // 🛡️ 防呆保險：在最後一微秒，強制將座標鎖死在最初拍下的快照點！
                        int idxX = m_Group.axisIndices[0], idxY = m_Group.axisIndices[1];
                        int idxZ = (m_Group.axisCount >= 3) ? m_Group.axisIndices[2] : -1;

                        (*m_pContexts)[idxX].logicalCmdPos = jm.frozenPos[0];
                        (*m_pContexts)[idxY].logicalCmdPos = jm.frozenPos[1];
                        if (idxZ != -1) (*m_pContexts)[idxZ].logicalCmdPos = jm.frozenPos[2];
                    }
                }
            }

            // ======================================================
            // 4. 根據狀態，套用實體軸座標 (完美 3D 座標映射)
            // ======================================================
            if (jm.state != JumpState::IDLE)
            {
                double curX = jm.frozenPos[0], curY = jm.frozenPos[1], curZ = jm.frozenPos[2];
                double velX = 0, velY = 0, velZ = 0;

                int idxX = m_Group.axisIndices[0], idxY = m_Group.axisIndices[1];
                int idxZ = (m_Group.axisCount >= 3) ? m_Group.axisIndices[2] : -1;

                double offset = jm.currentOffset;
                double jVel = jm.jumpVel;  // 保留規劃器的正負號

                if (jm.state == JumpState::B3_TO_CENTER)
                {
                    if (offset > jm.b3_distToCenter) offset = jm.b3_distToCenter; // 🛡️ 絕對夾死
                    if (jm.b3_distToCenter > 1e-6) {
                        double dirX = (jm.b3_centerPos[0] - jm.frozenPos[0]) / jm.b3_distToCenter;
                        double dirY = (jm.b3_centerPos[1] - jm.frozenPos[1]) / jm.b3_distToCenter;
                        double dirZ = (jm.b3_centerPos[2] - jm.frozenPos[2]) / jm.b3_distToCenter;

                        curX = jm.frozenPos[0] + offset * dirX;
                        curY = jm.frozenPos[1] + offset * dirY;
                        curZ = jm.frozenPos[2] + offset * dirZ;
                        velX = jVel * dirX; velY = jVel * dirY; velZ = jVel * dirZ;
                    }
                }
                else if (jm.state == JumpState::B3_TO_APEX || jm.state == JumpState::B3_DWELL || jm.state == JumpState::PAUSED_HOLD)
                {
                 
                    if (offset > jm.b3_distToApex) offset = jm.b3_distToApex; // 🛡️ 絕對夾死

                    // 🛡️ DWELL 或 PAUSED_HOLD 期間強制鎖死在頂點，絕不跟著 offset 歸零亂跑！
                    double tempOffset = (jm.state == JumpState::B3_DWELL || jm.state == JumpState::PAUSED_HOLD) ? jm.b3_distToApex : offset;

                    curX = jm.b3_centerPos[0] + tempOffset * jm.b3_retractVector[0];
                    curY = jm.b3_centerPos[1] + tempOffset * jm.b3_retractVector[1];
                    curZ = jm.b3_centerPos[2] + tempOffset * jm.b3_retractVector[2];

                    // 速度強制為 0
                    if (jm.state == JumpState::B3_DWELL || jm.state == JumpState::PAUSED_HOLD) {
                        velX = 0; velY = 0; velZ = 0;
                    }
                    else {
                        velX = jVel * jm.b3_retractVector[0];
                        velY = jVel * jm.b3_retractVector[1];
                        velZ = jVel * jm.b3_retractVector[2];
                    }
                }
                else if (jm.state == JumpState::B3_FROM_APEX)
                {
                    if (offset > jm.b3_distToApex) offset = jm.b3_distToApex; // 🛡️ 絕對夾死
                    double dropOffset = jm.b3_distToApex - offset;

                    curX = jm.b3_centerPos[0] + dropOffset * jm.b3_retractVector[0];
                    curY = jm.b3_centerPos[1] + dropOffset * jm.b3_retractVector[1];
                    curZ = jm.b3_centerPos[2] + dropOffset * jm.b3_retractVector[2];

                    velX = -jVel * jm.b3_retractVector[0];
                    velY = -jVel * jm.b3_retractVector[1];
                    velZ = -jVel * jm.b3_retractVector[2];
                }
                else if (jm.state == JumpState::B3_TO_WORKPIECE)
                {
                    if (offset > jm.b3_distToCenter) offset = jm.b3_distToCenter; // 🛡️ 絕對夾死
                    if (jm.b3_distToCenter > 1e-6) {
                        double dirX = (jm.frozenPos[0] - jm.b3_centerPos[0]) / jm.b3_distToCenter;
                        double dirY = (jm.frozenPos[1] - jm.b3_centerPos[1]) / jm.b3_distToCenter;
                        double dirZ = (jm.frozenPos[2] - jm.b3_centerPos[2]) / jm.b3_distToCenter;




                        curX = jm.b3_centerPos[0] + offset * dirX;
                        curY = jm.b3_centerPos[1] + offset * dirY;
                        curZ = jm.b3_centerPos[2] + offset * dirZ;
                        velX = jVel * dirX; velY = jVel * dirY; velZ = jVel * dirZ;
                    }
                }

                // 🌟 寫入實體軸 (完全不變)
                (*m_pContexts)[idxX].logicalCmdPos = curX;
                (*m_pContexts)[idxX].logicalCmdVel = velX;
                (*m_pContexts)[idxY].logicalCmdPos = curY;
                (*m_pContexts)[idxY].logicalCmdVel = velY;
                if (idxZ != -1) {
                    (*m_pContexts)[idxZ].logicalCmdPos = curZ;
                    (*m_pContexts)[idxZ].logicalCmdVel = velZ;
                }

                vAxis.currentCmdPos = jm.triggerPos;
                vAxis.currentCmdVel = 0.0;
            }
            else
            {
                // B3 結束！完美交接回放電點
                m_Group.pathMode = PathMode::PATH_SERVO;
            }


          
        }// ======================================================
        // 🌟 B4 搖動/行星加工：專屬 5 階段狀態機與座標映射
        // ======================================================
      else if (jm.mode == JumpMode::B4_ORBITAL_DIAGONAL &&
      ((jm.state >= JumpState::B4_TO_UPPER_CENTER && jm.state <= JumpState::B4_TO_WORKPIECE) || jm.state == JumpState::PAUSED_HOLD))
        {
        // --------------------------------------------------
        // 【第一部分：大腦 (狀態機與速度規劃)】
        // --------------------------------------------------
        std::vector<JumpSegment>* currentScript = nullptr;
        double finalTarget = 0.0;
        JumpState nextState = JumpState::IDLE;

        // 1. 選擇腳本與目標
        if (jm.state == JumpState::B4_TO_UPPER_CENTER) {
            currentScript = &jm.b4_toUpperSteps;
            finalTarget = jm.b4_distToUpper;
            nextState = JumpState::B4_TO_APEX;
        }
        else if (jm.state == JumpState::B4_TO_APEX) {
            currentScript = &jm.b4_toApexSteps;
            finalTarget = jm.b4_distToApex;
            nextState = JumpState::B4_DWELL;
        }
        else if (jm.state == JumpState::B4_FROM_APEX) {
            currentScript = &jm.b4_fromApexSteps;
            finalTarget = jm.b4_distToApex;
            nextState = JumpState::B4_TO_WORKPIECE;
        }
        else if (jm.state == JumpState::B4_TO_WORKPIECE) {
            currentScript = &jm.b4_toWorkpieceSteps;
            finalTarget = jm.b4_distToUpper;
            nextState = JumpState::IDLE;
        }

        // 2. 執行 DWELL 或 移動
        if (jm.state == JumpState::B4_DWELL) {
            jm.dwellTimer += dt * 1000.0;
            if (jm.dwellTimer >= jm.dwellTimeTarget) {
                jm.state = JumpState::B4_FROM_APEX;
                jm.currentStepIdx = 0; jm.currentOffset = 0.0; jm.jumpVel = 0.0;
            }
        }
        else if (currentScript != nullptr && !currentScript->empty())
        {
            JumpSegment& seg = (*currentScript)[jm.currentStepIdx];
            double realAcc = (seg.accTime > 0.0001) ? (seg.velocity / seg.accTime) : 1e10;
            double realDec = (seg.decTime > 0.0001) ? (seg.velocity / seg.decTime) : 1e10;

            double stepBoundary = 0.0;
            for (int i = 0; i <= jm.currentStepIdx; ++i) stepBoundary += (*currentScript)[i].distance;

            // 呼叫梯形規劃器
            jm.currentOffset = PlanTrapezoidal(jm.currentOffset, finalTarget, seg.velocity, realAcc, realDec, jm.jumpVel, dt);

            // 內部換檔
            if (jm.currentOffset >= stepBoundary && jm.currentStepIdx < (int)currentScript->size() - 1) {
                jm.currentStepIdx++;
            }

            if (jm.currentOffset >= finalTarget - 1e-5) {

                jm.currentStepIdx = 0;
                jm.currentOffset = 0.0;
                jm.jumpVel = 0.0;
                jm.dwellTimer = 0.0;

                // ==========================================================
                // 🌟 [絕對關鍵：B4 暫停空中攔截網 (純數學真頂點)]
                // ==========================================================
                if (jm.state == JumpState::B4_TO_APEX && jm.isPauseMode) {
                    jm.state = JumpState::PAUSED_HOLD;

                    jm.apexPos[0] = jm.b4_upperCenterPos[0] + jm.b4_distToApex * jm.b4_retractVector[0];
                    jm.apexPos[1] = jm.b4_upperCenterPos[1] + jm.b4_distToApex * jm.b4_retractVector[1];
                    if (m_Group.axisCount >= 3) {
                        jm.apexPos[2] = jm.b4_upperCenterPos[2] + jm.b4_distToApex * jm.b4_retractVector[2];
                    }
                    RtPrintf("[PAUSE_B4] Safely Holding at Math Apex.\n");
                }
                else {
                    jm.state = nextState;
                }

                // 🚨 PATH_SERVO 交接！
                if (jm.state == JumpState::IDLE) {
                    m_Group.pathMode = PathMode::PATH_SERVO;
                    jm.isRecovering = true; // 啟動軟著陸

                    int idxX = m_Group.axisIndices[0], idxY = m_Group.axisIndices[1];
                    int idxZ = (m_Group.axisCount >= 3) ? m_Group.axisIndices[2] : -1;
                    (*m_pContexts)[idxX].logicalCmdPos = jm.frozenPos[0];
                    (*m_pContexts)[idxY].logicalCmdPos = jm.frozenPos[1];
                    if (idxZ != -1) (*m_pContexts)[idxZ].logicalCmdPos = jm.frozenPos[2];
                }
            }
        }

        // --------------------------------------------------
        // 【第二部分：手腳 (3D 座標映射與無敵夾鉗)】
        // --------------------------------------------------
        if (jm.state != JumpState::IDLE)
        {
            double curX = jm.frozenPos[0], curY = jm.frozenPos[1], curZ = jm.frozenPos[2];
            double velX = 0, velY = 0, velZ = 0;
            double offset = jm.currentOffset;
            double jVel = jm.jumpVel;

            int idxX = m_Group.axisIndices[0], idxY = m_Group.axisIndices[1];
            int idxZ = (m_Group.axisCount >= 3) ? m_Group.axisIndices[2] : -1;

            if (jm.state == JumpState::B4_TO_UPPER_CENTER)
            {
                if (offset > jm.b4_distToUpper) offset = jm.b4_distToUpper; // 🛡️ 絕對夾死
                if (jm.b4_distToUpper > 1e-6) {
                    double dirX = (jm.b4_upperCenterPos[0] - jm.frozenPos[0]) / jm.b4_distToUpper;
                    double dirY = (jm.b4_upperCenterPos[1] - jm.frozenPos[1]) / jm.b4_distToUpper;
                    double dirZ = (jm.b4_upperCenterPos[2] - jm.frozenPos[2]) / jm.b4_distToUpper;

                    curX = jm.frozenPos[0] + offset * dirX;
                    curY = jm.frozenPos[1] + offset * dirY;
                    curZ = jm.frozenPos[2] + offset * dirZ;
                    velX = jVel * dirX; velY = jVel * dirY; velZ = jVel * dirZ;
                }
            }
            else if (jm.state == JumpState::B4_TO_APEX || jm.state == JumpState::B4_DWELL || jm.state == JumpState::PAUSED_HOLD)
            {
                if (offset > jm.b4_distToApex) offset = jm.b4_distToApex; // 🛡️ 絕對夾死
                double tempOffset = (jm.state == JumpState::B4_DWELL || jm.state == JumpState::PAUSED_HOLD) ? jm.b4_distToApex : offset;

                curX = jm.b4_upperCenterPos[0] + tempOffset * jm.b4_retractVector[0];
                curY = jm.b4_upperCenterPos[1] + tempOffset * jm.b4_retractVector[1];
                curZ = jm.b4_upperCenterPos[2] + tempOffset * jm.b4_retractVector[2];

                if (jm.state == JumpState::B4_DWELL || jm.state == JumpState::PAUSED_HOLD) {
                    velX = 0; velY = 0; velZ = 0;
                }
                else {
                    velX = jVel * jm.b4_retractVector[0];
                    velY = jVel * jm.b4_retractVector[1];
                    velZ = jVel * jm.b4_retractVector[2];
                }
            }
            else if (jm.state == JumpState::B4_FROM_APEX)
            {
                if (offset > jm.b4_distToApex) offset = jm.b4_distToApex; // 🛡️ 絕對夾死
                double dropOffset = jm.b4_distToApex - offset;

                curX = jm.b4_upperCenterPos[0] + dropOffset * jm.b4_retractVector[0];
                curY = jm.b4_upperCenterPos[1] + dropOffset * jm.b4_retractVector[1];
                curZ = jm.b4_upperCenterPos[2] + dropOffset * jm.b4_retractVector[2];

                velX = -jVel * jm.b4_retractVector[0];
                velY = -jVel * jm.b4_retractVector[1];
                velZ = -jVel * jm.b4_retractVector[2];
            }
            else if (jm.state == JumpState::B4_TO_WORKPIECE)
            {
                if (offset > jm.b4_distToUpper) offset = jm.b4_distToUpper; // 🛡️ 絕對夾死
                if (jm.b4_distToUpper > 1e-6) {
                    // 方向：從「上中心」指回「放電點」
                    double dirX = (jm.frozenPos[0] - jm.b4_upperCenterPos[0]) / jm.b4_distToUpper;
                    double dirY = (jm.frozenPos[1] - jm.b4_upperCenterPos[1]) / jm.b4_distToUpper;
                    double dirZ = (jm.frozenPos[2] - jm.b4_upperCenterPos[2]) / jm.b4_distToUpper;

                    curX = jm.b4_upperCenterPos[0] + offset * dirX;
                    curY = jm.b4_upperCenterPos[1] + offset * dirY;
                    curZ = jm.b4_upperCenterPos[2] + offset * dirZ;
                    velX = jVel * dirX; velY = jVel * dirY; velZ = jVel * dirZ;
                }
            }

            // 🌟 寫入實體軸
            (*m_pContexts)[idxX].logicalCmdPos = curX;
            (*m_pContexts)[idxX].logicalCmdVel = velX;
            (*m_pContexts)[idxY].logicalCmdPos = curY;
            (*m_pContexts)[idxY].logicalCmdVel = velY;
            if (idxZ != -1) {
                (*m_pContexts)[idxZ].logicalCmdPos = curZ;
                (*m_pContexts)[idxZ].logicalCmdVel = velZ;
            }

            // 凍結虛擬主軸 (保持放電進度)
            vAxis.currentCmdPos = jm.triggerPos;
            vAxis.currentCmdVel = 0.0;
        }
        }

      

       
    }







    // =====================================================================
    // 🌟 🟢 [空間座標轉換過濾器] 🟢 🌟
    // =====================================================================
    int idxX = m_Group.axisIndices[0];
    int idxY = m_Group.axisIndices[1];
    int idxZ = (m_Group.axisCount >= 3) ? m_Group.axisIndices[2] : -1;
    AxisContext& realX = (*m_pContexts)[idxX];
    AxisContext& realY = (*m_pContexts)[idxY];

    if (m_Group.enableTransform && m_Group.axisCount >= 2)
    {
        double logP[3] = { realX.logicalCmdPos, realY.logicalCmdPos, 0.0 };
        double logV[3] = { realX.logicalCmdVel, realY.logicalCmdVel, 0.0 };
        if (idxZ != -1) {
            logP[2] = (*m_pContexts)[idxZ].logicalCmdPos;
            logV[2] = (*m_pContexts)[idxZ].logicalCmdVel;
        }

        double physP[3] = { 0,0,0 };
        double physV[3] = { 0,0,0 };

        for (int r = 0; r < 3; ++r) {
            physP[r] = m_Group.transformOrigin[r];
            physV[r] = 0.0;
            for (int c = 0; c < 3; ++c) {
                physP[r] += m_Group.transformMatrix[r][c] * (logP[c] - m_Group.transformOrigin[c]);
                physV[r] += m_Group.transformMatrix[r][c] * logV[c];
            }
        }

        realX.currentCmdPos = physP[0];
        realX.currentCmdVel = physV[0];
        realY.currentCmdPos = physP[1];
        realY.currentCmdVel = physV[1];
        if (idxZ != -1) {
            (*m_pContexts)[idxZ].currentCmdPos = physP[2];
            (*m_pContexts)[idxZ].currentCmdVel = physV[2];
        }
    }
    else
    {
        realX.currentCmdPos = realX.logicalCmdPos;
        realX.currentCmdVel = realX.logicalCmdVel;
        realY.currentCmdPos = realY.logicalCmdPos;
        realY.currentCmdVel = realY.logicalCmdVel;
        if (idxZ != -1) {
            (*m_pContexts)[idxZ].currentCmdPos = (*m_pContexts)[idxZ].logicalCmdPos;
            (*m_pContexts)[idxZ].currentCmdVel = (*m_pContexts)[idxZ].logicalCmdVel;
        }
    }

    // 3. 結束檢查
    if (vAxis.state == MotionState::MotionState_IDLE) {
        m_Group.isActive = false;
        for (int i = 0; i < m_Group.axisCount; ++i) {
            int idx = m_Group.axisIndices[i];
            (*m_pContexts)[idx].state = MotionState::MotionState_IDLE;
            (*m_pContexts)[idx].currentCmdVel = 0.0;
        }
    }

    // 🟢 監視器 3：物理突波偵測 (放在最尾巴)
    static double lastRealVelX = 0;
    double currentRealVelX = (*m_pContexts)[m_Group.axisIndices[0]].logicalCmdVel;

   
    lastRealVelX = currentRealVelX;





    static double last_log_VelX = 0;
    double current_log_VelX = (*m_pContexts)[m_Group.axisIndices[0]].logicalCmdVel;

    // 🌟 偵測門檻：一毫秒內速度變化超過 500,000 PPS (你可以視情況調整)
    if (std::abs(current_log_VelX - last_log_VelX) > 500000.0)
    {
        // 在這行下斷點 (F9)，程式就會停在速度驟降的那一瞬間
        RtPrintf(">>> [HIT] Velocity Spike Detected! Before: %d | After: %d | Gap: %d\n",
            (int)last_log_VelX, (int)current_log_VelX, (int)(current_log_VelX - last_log_VelX));
    }
    last_log_VelX = current_log_VelX;
}




// ==========================================
// [樣板實例化] (Explicit Instantiation)
// ==========================================
// 這是為了讓編譯器知道要為你的 ENI_ServoDrive 產生代碼
// 請確保這裡 include 了你的驅動器定義檔，例如: 
// #include "EtherCatDefinitions.h" 

// 假設你的結構叫 ENI_ServoDrive (請根據你的專案修改)

template void MotionCore::UpdateMotion<ENI_ServoDrive>(ENI_ServoDrive&, AxisContext&);
template void MotionCore::Run_Servo_Loop<ENI_ServoDrive>(ENI_ServoDrive&, AxisContext&, const AxisCommand&);
