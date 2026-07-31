// 檔案：EtherCatMaster_Run.cpp
#include "EtherCatMaster.h"
#include "GlobalConfig.h" // 如果你有用到 DEBUG_PRINT 等功能
#include <windows.h> 
#include <rtapi.h> 
#include <rtssapi.h> 
#include <stdio.h>

void EtherCatMaster::RunRealTimeCycle_EXAMPLE_MODE()//主要程式迴圈執行 測試模式
{

    int WK = 0;

    //WKC 判斷-----------------------------------------------
    const auto& slaves = m_pEni->GetSlaves();
    int total_slaves = (int)slaves.size();
    for (int i = 0; i < total_slaves; i++)
    {
        int slaveWKC = 0;

        // 檢查是否有 Input (TxPDO)
        if (slaves[i].inputBitLength > 0)
        {
            slaveWKC += 1;
        }

        // 檢查是否有 Output (RxPDO)
        if (slaves[i].outputBitLength > 0)
        {
            slaveWKC += 2;
        }

        // 累加到總分
        EXPECTED_WKC_PDO += slaveWKC;

        // 
        DEBUG_PRINT("Slave %d: %s (In: %d bytes, Out: %d bytes) -> WKC contribution: %d\n", i, slaves[i].name, slaves[i].inputBitLength, slaves[i].outputBitLength, slaveWKC);
    }





    //PDO 中斷宣告---------------------------------------------------------------
    HANDLE hTimer_PDO = NULL;
    LARGE_INTEGER liPeriod_PDO;
    liPeriod_PDO.QuadPart = 2500; // 250us
    hTimer_PDO = RtCreateTimer(NULL, 0, GlobalTimerHandler_PDO, this, 64, CLOCK_2);//PSECURITY_ATTRIBUTES,StackSize,pRoutine,Context,Priority (請填入一個優先權數值，0~127),Clock
    if (hTimer_PDO == NULL)
    {
        DEBUG_PRINT("GlobalTimerHandler_PDO Error>>%d\n", GetLastError());
        return;
    }
    else
    {
        if (!RtSetTimerRelative(hTimer_PDO, &liPeriod_PDO, &liPeriod_PDO))
        {
            DEBUG_PRINT("GlobalTimerHandler_PDO Error>>RtSetTimerRelative\n");
            return;
        }
    }

    //PLC 中斷宣告---------------------------------------------------------------
    HANDLE hTimer_PLC = NULL;// 用來存放計時器的 Handle
    LARGE_INTEGER liPeriod_PLC;
    liPeriod_PLC.QuadPart = 10000; // 1ms
    hTimer_PLC = RtCreateTimer(NULL, 0, GlobalTimerHandler_PLC, this, 63, CLOCK_2);//PSECURITY_ATTRIBUTES,StackSize,pRoutine,Context,Priority (請填入一個優先權數值，0~127),Clock

    if (hTimer_PLC == NULL)
    {
        DEBUG_PRINT("GlobalTimerHandler_PLC Error>>%d\n", GetLastError());
        return;
    }
    else
    {
        if (!RtSetTimerRelative(hTimer_PLC, &liPeriod_PLC, &liPeriod_PLC))
        {
            DEBUG_PRINT("GlobalTimerHandler_PLC Error>>RtSetTimerRelative\n");
        }
    }



    m_ServoList[0].pOutput->ControlWord = 0x0080;
    m_ServoList[1].pOutput->ControlWord = 0x0080;



    bool isSuccess = PDO_SendCommandAndWait
    (
        EcatCmdType::CMD_SET_STATE, // 1. 指令類型 (使用 enum class)
        0x0000,                 // 2. 站號: 0x0000 代表廣播 (Broadcast)
        0x0000,                 // 3. Index: 切換狀態用不到，填 0
        0x00,                   // 4. Sub: 用不到，填 0
        0x0008,                 // 5. 數值: 0x0008 (Request OP State)
        2,                      // 6. 長度: [關鍵] AL Control 是 2 Bytes
        1000                    // 7. 超時: 給它 1秒鐘的時間
    );

    if (isSuccess)
    {
        DEBUG_PRINT("[System] System is now in OP Mode. (Success)\n");
    }
    else
    {
        DEBUG_PRINT("[Error] Failed to switch to OP Mode! (Timeout or Error)\n");
    }



    //Motion---------------------------------------------------


    // 1. [關鍵] 根據馬達數量，調整軸參數陣列的大小
    m_Motion.Link(&m_ServoList, &m_Axes);

    size_t motorCount = m_ServoList.size();

    if (motorCount > 0)
    {
        // 自動產生 N 個 AxisContext 物件
        m_Axes.resize(motorCount);

        // 2. 使用迴圈初始化每一軸
        for (size_t i = 0; i < motorCount; ++i)
        {
            // ==========================================
            // A. 基礎初始化 (Base Initialization)
            // ==========================================
            // 這裡設定該軸的解析度 (台達 B3/A2 預設 16,777,216)
            // 如果你有不同廠牌混用，可以用 switch(i) 來區分
            m_Motion.InitAxis(m_Axes[i], 16777216.0);

            // ==========================================
            // B. 針對第 1 軸 (Index 0) 設定測試參數
            // ==========================================
            if (i == 0)
            {
                // --- [1] 安全限速設定 ---
                // 測試時先不要跑 3000 RPM，改跑 600 RPM 就好
                double safeSpeedRPM = 3000.0;
                m_Axes[i].maxVel_PPS = MotionCore::RpmToPps(safeSpeedRPM, m_Axes[i].resolution_PPR);

                // --- [2] 柔和加減速 ---
                // 設定 1.0 秒才加速到 600 RPM (原本是 0.5 秒到 3000，太猛了)
                // 這樣你看得清楚它在加速，不會 "咚" 一聲就到位
                m_Axes[i].acc_PPS2 = m_Axes[i].maxVel_PPS * 1.0;
                m_Axes[i].dec_PPS2 = m_Axes[i].maxVel_PPS * 1.0;

                // --- [3] PID 剛性設定 (最重要!) ---
                // Kp: 20.0 (建議起始值，若馬達震動改小，若馬達不動改大)
                // Ki: 0.0  (測試絕對設 0，避免積分暴衝)
                // Kd: 0.0  (通常不需要)
                m_Axes[i].pid.Kp = 20.0;
                m_Axes[i].pid.Ki = 10.0;//10.5
                m_Axes[i].pid.Kd = 0.0;

                // --- [4] 保護設定 ---
                // 允許跟隨誤差 (Lag) 大約 2 度 (100000 pulse)
                // 測試初期設寬一點，避免 PID 還沒調好一直跳機
                m_Axes[i].pid.EnableLagCheck = true;
                m_Axes[i].pid.MaxLag = 100000000.0;

                DEBUG_PRINT(">> Axis 0 Configured: Speed=%.0f RPM, Kp=%.1f\n", safeSpeedRPM, m_Axes[i].pid.Kp);
            }

            // ==========================================
            // C. 針對第 2 軸 (如果有)
            // ==========================================
            else if (i == 1)
            {
                // --- [1] 安全限速設定 ---
                // 測試時先不要跑 3000 RPM，改跑 600 RPM 就好
                double safeSpeedRPM = 3000.0;
                m_Axes[i].maxVel_PPS = MotionCore::RpmToPps(safeSpeedRPM, m_Axes[i].resolution_PPR);

                // --- [2] 柔和加減速 ---
                // 設定 1.0 秒才加速到 600 RPM (原本是 0.5 秒到 3000，太猛了)
                // 這樣你看得清楚它在加速，不會 "咚" 一聲就到位
                m_Axes[i].acc_PPS2 = m_Axes[i].maxVel_PPS * 1.0;
                m_Axes[i].dec_PPS2 = m_Axes[i].maxVel_PPS * 1.0;

                // --- [3] PID 剛性設定 (最重要!) ---
                // Kp: 20.0 (建議起始值，若馬達震動改小，若馬達不動改大)
                // Ki: 0.0  (測試絕對設 0，避免積分暴衝)
                // Kd: 0.0  (通常不需要)
                m_Axes[i].pid.EnableLagCheck = true;
                m_Axes[i].pid.Kp = 20.0;
                m_Axes[i].pid.Ki = 10.0;//10.5
                m_Axes[i].pid.Kd = 0.0;

                // --- [4] 保護設定 ---
                // 允許跟隨誤差 (Lag) 大約 2 度 (100000 pulse)
                // 測試初期設寬一點，避免 PID 還沒調好一直跳機
                m_Axes[i].pid.MaxLag = 100000000.0;

                DEBUG_PRINT(">> Axis 0 Configured: Speed=%.0f RPM, Kp=%.1f\n", safeSpeedRPM, m_Axes[i].pid.Kp);
            }

            else if (i == 2)
            {
                // --- [1] 安全限速設定 ---
                // 測試時先不要跑 3000 RPM，改跑 600 RPM 就好
                double safeSpeedRPM = 3000.0;
                m_Axes[i].maxVel_PPS = MotionCore::RpmToPps(safeSpeedRPM, m_Axes[i].resolution_PPR);

                // --- [2] 柔和加減速 ---
                // 設定 1.0 秒才加速到 600 RPM (原本是 0.5 秒到 3000，太猛了)
                // 這樣你看得清楚它在加速，不會 "咚" 一聲就到位
                m_Axes[i].acc_PPS2 = m_Axes[i].maxVel_PPS * 1.0;
                m_Axes[i].dec_PPS2 = m_Axes[i].maxVel_PPS * 1.0;

                // --- [3] PID 剛性設定 (最重要!) ---
                // Kp: 20.0 (建議起始值，若馬達震動改小，若馬達不動改大)
                // Ki: 0.0  (測試絕對設 0，避免積分暴衝)
                // Kd: 0.0  (通常不需要)
                m_Axes[i].pid.EnableLagCheck = true;
                m_Axes[i].pid.Kp = 20.0;
                m_Axes[i].pid.Ki = 10.0;//10.5
                m_Axes[i].pid.Kd = 0.0;

                // --- [4] 保護設定 ---
                // 允許跟隨誤差 (Lag) 大約 2 度 (100000 pulse)
                // 測試初期設寬一點，避免 PID 還沒調好一直跳機
                m_Axes[i].pid.MaxLag = 100000000.0;

                DEBUG_PRINT(">> Axis 0 Configured: Speed=%.0f RPM, Kp=%.1f\n", safeSpeedRPM, m_Axes[i].pid.Kp);
            }






        }

        DEBUG_PRINT("Initialized %d Axes with Motion Parameters.\n", (int)motorCount);
    }




    //主控迴圈-------------------------------------------------------------

    while (1)
    {

        // 1. 睡 10ms (更省資源)
        RtSleep(10);

        // 2. 補償 Tick
        // 10ms = 10,000us = 40 * 250us
        // 所以一次加 40
        tickCount_RunRealTimeCycle += 40;


        timer_10ms += 10;
        timer_100ms += 10;
        timer_1000ms += 10;
        Debug_test_timer += 10; // 測試專用時間軸

        // 3. 判斷 1 秒 (4000 ticks)
        // 注意：因為每次加 40，所以一定會整除 4000，邏輯成立
        if (tickCount_RunRealTimeCycle % 4000 == 0)
        {


            if (!m_ServoList.empty())
            {

                for (int i = 0; i < m_ServoList.size(); i++)
                {
                    ENI_ServoDrive& servo = m_ServoList[i]; // 注意這裡你是操作第2顆馬達(Index 1)

                    //DEBUG_PRINT("ActualPosition>>%d\n", servo.pInput->ActualPosition);
                    //DEBUG_PRINT("Object_2510>>%d\n", servo.pInput->Object_2510);

                    if (servo.pInput != nullptr && servo.pOutput != nullptr)
                    {
                        uint16_t statusWord = servo.pInput->StatusWord;

                     
                        if ((statusWord & 0x006F) == 0x0027)
                        {
                            // 已經激磁成功，維持 0x000F
                            servo.pOutput->ControlWord = 0x000F;

                            // 這裡可以開始給速度了 (小心測試)
                            //servo.pOutput->TargetVelocity = -16777216;

                            // DEBUG_PRINT 太多會影響效能，成功後建議少印一點
                            //DEBUG_PRINT("State: Operation Enabled (Servo On!)\n");

                            if (m_Axes.size() > 0)
                            {
                                if (m_Axes[0].state != MotionState::MotionState_IDLE &&
                                    m_Axes[0].state != MotionState::MotionState_ERROR)
                                {

                                }
                                else
                                {



                                    if (test_timer == 0)
                                    {
                                        // Jerk 大 (很衝) = 緩衝區很小 (平均時間短)
                                        //Jerk 小(很柔) = 緩衝區很大(平均時間長)
                                        m_Motion.InitSmoothBuffer(m_Axes[0], 100 * 1);//加加速度來開啟 100ms 的平滑功能
                                        m_Motion.InitSmoothBuffer(m_Axes[1], 100 * 1);//加加速度來開啟 100ms 的平滑功能
                                        m_Motion.InitSmoothBuffer(m_Axes[2], 100 * 1);//加加速度來開啟 100ms 的平滑功能
                                        m_Motion.InitVirtualAxisSmooth(100 * 1);

                                        m_Axes[0].isServoOn = true;
                                        m_Axes[1].isServoOn = true;
                                        m_Axes[2].isServoOn = true;
                                        
                                        //選項 1
                                        //優點：定位最準，反應最快，絕不過衝。
                                        //缺點：起步和煞車會比較硬 (Jerk 無限大)。

                                        //選項 2
                                        //大幅縮短平滑時間 (如果你堅持要有一點軟啟動)
                                    }


                                }





                                int COM_Index = 0;
                                double ONE_REV = 16777216.0;
                                double targetPos = 16777216.0 * 1; // 
                                double targetVel = 8388608.0 * 10;  // 用 0.5 圈/秒 的速度跑 (30 RPM)


                                double accTime = 0.2; // 希望 0.2 秒加速完畢 (標準)
                                double decTime = 0.2; // 希望 0.2 秒減速完畢




                                double realPos = m_Axes[COM_Index].currentActPos;

                                //MoveToPosition---------------------------------------------------------


                                if (m_Axes[COM_Index].inPosition == true && m_Axes[COM_Index].state == MotionState::MotionState_IDLE)// 命令已完成，馬達已到位
                                {

                                    if (test_timer == 2)
                                    {
                                        //m_Motion.LineMove({ COM_Index }, { targetPos * 0 }, targetVel, accTime, decTime, BufferMode::BUFFERED);
                                        //m_Motion.MoveToPosition(m_Axes[COM_Index], targetPos * 0, targetVel, accTime, decTime);
                                    }

                                    if (test_timer == 5)
                                    {
                                        //m_Motion.MoveToPosition(m_Axes[COM_Index], targetPos * 10, targetVel, accTime, decTime);
                                    }


                                    if (realPos < 0)
                                    {
                                        //m_Motion.MoveToPosition(m_Axes[COM_Index], targetPos, targetVel, accTime, decTime);
                                    }
                                    else
                                    {
                                        //m_Motion.MoveToPosition(m_Axes[COM_Index], 0-targetPos, targetVel, accTime, decTime);
                                    }


                                }





                                //VelocityMove---------------------------------------------------------

                                 /*
                                 // 1. 計數器累加
                                 test_timer++;
                                 accTime = 0.1;
                                 // 2. 每 300ms 切換一次方向 (假設 Cycle Time = 1ms)
                                 if (test_timer >= 2)
                                 {
                                     test_timer = 0;      // 歸零，防止溢位
                                     test_dir = test_dir *-1;      // 乘以 -1 來切換方向 (1 -> -1 -> 1...)

                                 }
                                 m_Motion.VelocityMove(m_Axes[COM_Index], targetVel* test_dir, accTime);
                                 */

                                 //StopMove---------------------------------------------------------

                                  /*
                                  accTime = 0.1;
                                  decTime = 0.1;
                                  targetPos = 16777216.0 * 100; //
                                  targetVel = 8388608.0 * 1;  // 用 0.5 圈/秒 的速度跑 (30 RPM)
                                  if (test_timer == 0)
                                  {
                                      m_Motion.MoveToPosition(m_Axes[COM_Index], targetPos, targetVel, accTime, decTime);
                                      //m_Motion.VelocityMove(m_Axes[COM_Index], targetVel* test_dir, accTime);
                                  }

                                  if (test_timer == 5)
                                  {
                                      m_Motion.StopMove(m_Axes[COM_Index], decTime);
                                  }*/




                                  //EmergencyStop---------------------------------------------------------
                                  /*
                                  accTime = 0.1;
                                  decTime = 0.1;
                                  targetPos = 16777216.0 * 100; //
                                  if (test_timer == 0)
                                  {
                                      //m_Motion.MoveToPosition(m_Axes[COM_Index], targetPos, targetVel, accTime, decTime);
                                      m_Motion.VelocityMove(m_Axes[COM_Index], targetVel* test_dir, accTime);
                                  }

                                  if (test_timer == 5)
                                  {
                                      m_Motion.EmergencyStop(m_Axes[COM_Index]);
                                  }
                                  test_timer++;
                                  */


                                  //LineMove---------------------------------------------------------




                                if (m_Axes[0].inPosition == true && m_Axes[0].state == MotionState::MotionState_IDLE &&
                                    m_Axes[1].inPosition == true && m_Axes[1].state == MotionState::MotionState_IDLE)// 命令已完成，馬達已到位
                                {
                                    /*
                                    if (realPos < 0)
                                    {
                                        m_Motion.LineMove({ 0, 1 }, { targetPos * 1, targetPos * 1 }, targetVel, accTime, decTime,BufferMode::ABORTING);
                                    }
                                    else
                                    {
                                        m_Motion.LineMove({ 0, 1 }, { 0-targetPos * 1, 0-targetPos * 1 }, targetVel, accTime, decTime, BufferMode::ABORTING);
                                    }*/

                                    /*
                                    if (realPos < 0)
                                    {
                                        m_Motion.LineMove({ 0, 1 ,2}, { targetPos * 1, targetPos * 1 ,targetPos * 1 }, targetVel, accTime, decTime, BufferMode::ABORTING);
                                    }
                                    else
                                    {
                                        m_Motion.LineMove({ 0, 1 ,2}, { 0 - targetPos * 1, 0 - targetPos * 1 , 0 - targetPos * 1 }, targetVel, accTime, decTime, BufferMode::ABORTING);
                                    }*/

                                }


                                //StopGroup---------------------------------------------------------
                                /*
                                targetPos = 16777216.0 * 100 *1; //
                                targetVel = 8388608.0  *10;  // 用 0.5 圈/秒 的速度跑 (30 RPM)
                                accTime = 0.1;
                                decTime = 0.1;
                                if (test_timer == 0)
                                {
                                    m_Motion.LineMove({ 0, 1 }, { targetPos * 1, targetPos * 1 }, targetVel, accTime, decTime);
                                }


                                if (test_timer == 5)
                                {
                                    m_Motion.StopGroup(m_Axes, decTime);
                                }*/





                                //EmergencyStopGroup---------------------------------------------------------
                                /*
                                targetPos = 16777216.0 * 100 * 1; //
                                targetVel = 8388608.0 * 10;  // 用 0.5 圈/秒 的速度跑 (30 RPM)
                                accTime = 0.1;
                                decTime = 0.1;
                                if (test_timer == 0)
                                {
                                   m_Motion.LineMove({ 0, 1 }, { targetPos * 1, targetPos * 1 }, targetVel, accTime, decTime);
                                }


                                if (test_timer == 5)
                                {
                                    m_Motion.EmergencyStopGroup();
                                }
                                */







                                //ArcMove---------------------------------------------------------
                                /*
                                std::vector<int> activeAxes = { 0, 1}; // X 和 Y 軸

                                // 假設目前 X=0, Y=0。
                                // 我們要畫一個圓心在 (10000, 0) 的半圓，跑到終點 (20000, 0)
                                std::vector<double> centerPos_Arc = { ONE_REV * 1.0, 0.0 };
                                std::vector<double> targetPos_Arc = { ONE_REV * 2.0, 0.0 };

                                // 方向 1 (CCW)，目標速度 5000，加減速 0.2 秒
                                int dir = 1;
                                if (test_timer == 0)
                                {

                                    if (m_Axes[0].inPosition == true && m_Axes[0].state == MotionState::MotionState_IDLE &&
                                        m_Axes[1].inPosition == true && m_Axes[1].state == MotionState::MotionState_IDLE)// 命令已完成，馬達已到位
                                    {
                                        m_Motion.SetGroupFeedrateOverride(1);
                                        m_Motion.LineMove({ 0, 1 ,2}, { 0, 0,0}, targetVel, accTime, decTime, BufferMode::ABORTING);
                                    }
                                }


                                if (test_timer ==5)
                                {
                                    accTime = 0.2; // 希望 0.2 秒加速完畢 (標準)
                                    decTime = 0.2; // 希望 0.2 秒減速完畢
                                    if (m_Axes[0].inPosition == true && m_Axes[0].state == MotionState::MotionState_IDLE &&
                                        m_Axes[1].inPosition == true && m_Axes[1].state == MotionState::MotionState_IDLE)// 命令已完成，馬達已到位
                                    {
                                        m_Motion.SetGroupFeedrateOverride(0.01);
                                        m_Motion.ArcMove(activeAxes, targetPos_Arc, centerPos_Arc, dir, targetVel, accTime, decTime, BufferMode::ABORTING);
                                    }


                                }*/




                                //LineMove ArcMove Queue ---------------------------------------------------------


                                accTime = 0.2; // 希望 0.2 秒加速完畢 (標準)
                                decTime = 0.2; // 希望 0.2 秒減速完畢

                                if (m_Axes[0].inPosition == true && m_Axes[0].state == MotionState::MotionState_IDLE &&
                                    m_Axes[1].inPosition == true && m_Axes[1].state == MotionState::MotionState_IDLE)// 命令已完成，馬達已到位
                                {
                                    if (test_timer == 0)
                                    {

                                        /*
                                        m_Motion.LineMove({ 0, 1 }, { 0, 0 }, targetVel, accTime, decTime, BufferMode::BUFFERED);
                                        m_Motion.LineMove({ 0, 1 }, { ONE_REV * 5.0, ONE_REV * 5.0 }, targetVel, accTime, decTime, BufferMode::BUFFERED);
                                        m_Motion.LineMove({ 0, 1 }, { ONE_REV * -5.0, ONE_REV * -5.0 }, targetVel, accTime, decTime, BufferMode::BUFFERED);
                                        */


                                        /*
                                        //m_Motion.SetGroupPathMode(PathMode::EXACT_STOP);// 精確停止
                                        m_Motion.SetGroupPathMode(PathMode::CONTINUOUS);// 連續軌跡
                                        m_Motion.LineMove({ 0, 1 }, { 0, 0 }, targetVel, accTime, decTime, BufferMode::BUFFERED);
                                        m_Motion.LineMove({ 0, 1 }, { ONE_REV * 1.0, 0.0 }, targetVel, accTime, decTime, BufferMode::BUFFERED);
                                        m_Motion.ArcMove({ 0, 1 }, { ONE_REV * 2.0, 0.0 }, { ONE_REV * 1.5, 0.0 }, 1, targetVel, accTime, decTime, BufferMode::BUFFERED);
                                        m_Motion.LineMove({ 0, 1 }, { ONE_REV * 3.0, 0.0 }, targetVel, accTime, decTime, BufferMode::BUFFERED);
                                        */


                                        /*
                                        //m_Motion.SetGroupPathMode(PathMode::EXACT_STOP);// 精確停止
                                        m_Motion.SetGroupPathMode(PathMode::CONTINUOUS);// 連續軌跡

                                        m_Motion.LineMove({ 0, 1 }, { 0, 0 }, targetVel, accTime, decTime, BufferMode::BUFFERED);
                                        targetVel = 8388608.0 * 5;  // 用 0.5 圈/秒 的速度跑 (30 RPM)
                                        m_Motion.LineMove({ 0, 1 }, { ONE_REV * 2.0, ONE_REV * 0.0 }, targetVel, accTime, decTime, BufferMode::BUFFERED);
                                        targetVel = 8388608.0 * 10;  // 用 0.5 圈/秒 的速度跑 (30 RPM)
                                        m_Motion.LineMove({ 0, 1 }, { ONE_REV * 5.0, ONE_REV * 0.0 }, targetVel, accTime, decTime, BufferMode::BUFFERED);
                                        */


                                        /*
                                        //m_Motion.SetGroupPathMode(PathMode::EXACT_STOP);// 精確停止
                                       m_Motion.SetGroupPathMode(PathMode::CONTINUOUS);// 連續軌跡

                                        m_Motion.LineMove({ 0, 1 }, { 0, 0 }, targetVel, accTime, decTime, BufferMode::BUFFERED);
                                        targetVel = 8388608.0 * 5;  // 用 0.5 圈/秒 的速度跑 (30 RPM)
                                        m_Motion.LineMove({ 0, 1 }, { ONE_REV * 5.0, ONE_REV * 5.0 }, targetVel, accTime, decTime, BufferMode::BUFFERED);
                                        targetVel = 8388608.0 * 10;  // 用 0.5 圈/秒 的速度跑 (30 RPM)
                                        m_Motion.LineMove({ 0, 1 }, { ONE_REV * 0.0, ONE_REV * 0.0 }, targetVel, accTime, decTime, BufferMode::BUFFERED);
                                        */



                                        // 設定基礎參數
                                       /*
                                       m_Motion.SetGroupPathMode(PathMode::CONTINUOUS);// 連續軌跡
                                       double rev = 8388608.0;
                                       double targetVel = rev * 5.0; // 300 RPM 巡航速度
                                       double accTime = 0.5;
                                       double decTime = 0.5;

                                       // 0. 先把機台歸零準備
                                       m_Motion.LineMove({ 0, 1 }, { 0.0, 0.0 }, targetVel, accTime, decTime, BufferMode::BUFFERED);

                                       // 1. 直線：往右衝 (+X方向)
                                       m_Motion.LineMove({ 0, 1 }, { rev * 5.0, 0.0 }, targetVel, accTime, decTime, BufferMode::BUFFERED);

                                       // 2. 圓弧：逆時針 90度轉角
                                       // 起點 (5,0) 的切線剛好是 +X，完美對齊上一段！
                                       // 終點 (7,2) 的切線剛好是 +Y，完美對齊下一段！
                                       // 參數 0 通常代表 CCW (逆時針)，如果不對請改成 1
                                       m_Motion.ArcMove({ 0, 1 }, { rev * 7.0, rev * 2.0 }, { rev * 5.0, rev * 2.0 }, 0, targetVel, accTime, decTime, BufferMode::BUFFERED);

                                       // 3. 直線：往上衝 (+Y方向)
                                       m_Motion.LineMove({ 0, 1 }, { rev * 7.0, rev * 7.0 }, targetVel, accTime, decTime, BufferMode::BUFFERED);
                                       */
                                    }

                                }






                                //UpdatePathServoVelocity ---------------------------------------------------------


                                // 基礎參數
                                /*
                                targetVel = 8388608.0 * 10.0; // 2 圈/秒 (120 RPM)
                                accTime = 0.2;
                                decTime = 0.2;

                                // --- 階段 1：歸零準備 (0 ~ 1 秒) ---
                                if (test_timer == 0)
                                {
                                    m_Motion.SetGroupPathMode(PathMode::EXACT_STOP);

                                    m_Motion.LineMove({ 0, 1 }, { 0.0, 0.0 }, targetVel*5, accTime, decTime, BufferMode::ABORTING);
                                    //m_Motion.ArcMove({ 0, 1 }, { ONE_REV * 20.0, 0.0 }, { ONE_REV * 10.5, 0.0 }, 1, targetVel, accTime, decTime, BufferMode::ABORTING);
                                }

                                // --- 階段 2：啟動 Path Servo 並下達長距離指令 (第 2 秒) ---
                                if (test_timer == 5) // 1.0秒處 (4000 * 0.25ms)
                                {
                                    m_Motion.SetGroupPathMode(PathMode::PATH_SERVO);
                                    m_Motion.UpdatePathServoVelocity(targetVel * 0.5);
                                    // 下達一個很長的距離，確保我們有足夠的空間來回抽動
                                    m_Motion.LineMove({ 0, 1 }, { ONE_REV * 20.0, ONE_REV * 20.0 }, targetVel*0.5, accTime, decTime, BufferMode::ABORTING);
                                    //m_Motion.ArcMove({ 0, 1 }, { ONE_REV * 20.0, 0.0 }, { ONE_REV * 10.5, 0.0 }, 1, targetVel, accTime, decTime, BufferMode::ABORTING);
                                }
                                */






                                //原路徑回退 ---------------------------------------------------------
                                /*
                                if (test_timer == 0)
                                {
                                    m_Motion.SetGroupPathMode(PathMode::EXACT_STOP);

                                    m_Motion.LineMove({ 0, 1 }, { 0.0, 0.0 }, targetVel * 5, accTime, decTime, BufferMode::ABORTING);

                                }


                                if (test_timer == 7)
                                {
                                    // 開啟時光機引擎
                                    m_Motion.EnableHistoryBuffer(true);

                                    // 切換到 PATH_SERVO 模式 (上帝模式，由外部給定正負速度)
                                    m_Motion.SetGroupPathMode(PathMode::PATH_SERVO);
                                    m_Motion.UpdatePathServoVelocity(targetVel * 0.4);
                                    ONE_REV = 16777216.0;

                                    accTime = 0.2;
                                    decTime = 0.2;

                                    // 🟢 給它「真正有距離」的指令！
                                    // 第一段：X軸往右走 10 圈
                                    m_Motion.LineMove({ 0, 1 }, { ONE_REV * 8.0, 0.0 }, targetVel, accTime, decTime, BufferMode::BUFFERED);

                                    // 第二段：逆時針畫個半圓
                                    m_Motion.ArcMove({ 0, 1 }, { ONE_REV * 20.0, ONE_REV * 10.0 }, { ONE_REV * 10.0, ONE_REV * 10.0 }, 1, targetVel, accTime, decTime, BufferMode::BUFFERED);
                                }


                                if (test_timer == 25)
                                {
                                    m_Motion.UpdatePathServoVelocity(targetVel * -0.4);
                                }

                                if (test_timer == 28)
                                {
                                    m_Motion.UpdatePathServoVelocity(targetVel * 0.4);
                                }
                              */







                              //ArcMove 螺旋差補---------------------------------------------------------

                              /*
                              targetVel = 8388608.0 * 1.0; // 2 圈/秒 (120 RPM)
                              accTime = 0.2;
                              decTime = 0.2;

                              if (test_timer == 0)
                              {


                                  m_Motion.LineMove({ 0, 1 ,2}, { 0.0, 0.0 ,0.0}, targetVel * 10, accTime, decTime, BufferMode::ABORTING);

                              }

                              // 1. 宣告參與連動的軸：X(0), Y(1), Z(2)
                              std::vector<int> activeAxes = { 0, 1, 2 };

                              // 2. 設定圓心 (2D 平面)：假設 X 圓心在 1 圈，Y 圓心在 0
                              std::vector<double> centerPos_Arc = { ONE_REV * 1.0, 0.0 };

                              // 3. 設定終點 (3D 空間)：X 跑到 2 圈，Y 回到 0，Z 軸往下鑽 5 圈！
                              std::vector<double> targetPos_Arc = { ONE_REV * 2.0, 0.0, ONE_REV * -5.0 *0};

                              // 方向 1 (CCW 逆時針)
                              int dir = 1;

                              // -----------------------------------------------------------------
                              // 歸零前置作業 (回到 X=0, Y=0, Z=0)
                              // -----------------------------------------------------------------
                              if (test_timer == 0)
                              {

                                  //m_Motion.LineMove({ 0, 1 ,2 }, { 0.0, 0.0 ,0.0 }, targetVel * 10, accTime, decTime, BufferMode::ABORTING);

                              }

                              // -----------------------------------------------------------------
                              // 觸發 3D 螺旋插補
                              // -----------------------------------------------------------------
                              if (test_timer == 7)
                              {
                                  accTime = 0.2;
                                  decTime = 0.2;

                                  // 同樣要確保 X, Y, Z 三軸都已經歸零完畢，乖乖在原地等
                                  if (m_Axes[0].inPosition && m_Axes[0].state == MotionState::MotionState_IDLE &&
                                      m_Axes[1].inPosition && m_Axes[1].state == MotionState::MotionState_IDLE &&
                                      m_Axes[2].inPosition && m_Axes[2].state == MotionState::MotionState_IDLE)
                                  {


                                      // 發射！啟動 3D 螺旋插補

                                      //m_Motion.SetCoordinateTransform(true, 0, 0, 0, 45.0, 0.0, 0.0);
                                     // m_Motion.LineMove({ 0, 1 ,2 }, { ONE_REV * 5.0, 0.0 ,0.0 }, targetVel * 10, accTime, decTime, BufferMode::ABORTING);


                                       //m_Motion.SetCoordinateTransform(true, 0, 0, 0, 0.0, 0.0, 45.0);
                                      m_Motion.ArcMove(activeAxes, targetPos_Arc, centerPos_Arc, dir, targetVel, accTime, decTime, BufferMode::ABORTING);
                                  }
                              }
                              */


                              //Jump 跳躍排渣---------------------------------------------------------


                                targetVel = 8388608.0 * 1;  // 用 0.5 圈/秒 的速度跑 (30 RPM)
                                accTime = 0.2;
                                decTime = 0.2;

                                if (test_timer == 0)
                                {


                                    m_Motion.LineMove({ 0,1,2 }, { 0.0 ,0.0,0.0 }, targetVel * 15, accTime, decTime, BufferMode::ABORTING);

                                }




                                if (test_timer == 10)
                                {
                                    // 開啟時光機引擎
                                    m_Motion.EnableHistoryBuffer(true);

                                    // 切換到 PATH_SERVO 模式 (上帝模式，由外部給定正負速度)
                                    targetVel = 8388608.0 * 1;  // 用 0.5 圈/秒 的速度跑 (30 RPM)
                                    m_Motion.SetGroupPathMode(PathMode::PATH_SERVO);
                                    m_Motion.UpdatePathServoVelocity(targetVel * 2);
                                    ONE_REV = 16777216.0;

                                    accTime = 0.2;
                                    decTime = 0.2;

                                    // 🟢 給它「真正有距離」的指令！
                                    // 第一段：X軸往右走 10 圈
                                    //m_Motion.LineMove({ 0}, { ONE_REV * 10.0 }, targetVel, accTime, decTime, BufferMode::BUFFERED);




                                    // LineMove排渣----------------------------------------------------
                                    // 2. 設定一個緩慢的放電前進速度 (例如 10 RPM)
                                    /*
                                    m_Motion.UpdatePathServoVelocity(ONE_REV * (10.0 / 60.0));

                                    // 3. 下達三軸同動指令 (假設你的軸編號是 0:X, 1:Y, 2:Z)
                                    std::vector<int> axes = { 0, 1, 2 };
                                    std::vector<double> targets = { ONE_REV * 10.0, ONE_REV * 10.0, -ONE_REV * 5.0 };

                                    // 發送指令 (加減速 0.2s)
                                    m_Motion.LineMove(axes, targets, ONE_REV, 0.2, 0.2, BufferMode::BUFFERED);
                                    */


                                    // ArcMove排渣----------------------------------------------------
                                    /*
                                    std::vector<int> axes = { 0, 1 }; // 聯動 X 軸與 Y 軸

                                    // 中心點設在 X: 5圈, Y: 0
                                    std::vector<double> centerPos = { ONE_REV * 5.0, 0.0 };

                                    // 目標點：畫一個完美的半圓，跑到 X: 10圈, Y: 0 的位置
                                    // (用半圓測試最安全，波形特徵也最明顯)
                                    std::vector<double> targetPos = { ONE_REV * 10.0, 0.0 };

                                    // 4. 下達圓弧指令！
                                    m_Motion.ArcMove(
                                        axes,
                                        targetPos,
                                        centerPos,
                                        1,             // dir: 1 代表 CCW (逆時針)
                                        ONE_REV,       // 速度 (在 PATH_SERVO 模式下不會用到，但需填入合法值)
                                        0.2, 0.2,      // 加減速時間 0.2s
                                        BufferMode::BUFFERED
                                    );*/


                                    // 多段排渣----------------------------------------------------
                                    /*
                                    std::vector<int> axesXY = { 0, 1 }; // X, Y 軸

                                    // =========================================================
                                    // 第 1 節：X 軸往正向走 3 圈 (純直線)
                                    // =========================================================
                                    m_Motion.LineMove(axesXY, { ONE_REV * 3.0, 0.0 }, ONE_REV, 0.2, 0.2, BufferMode::BUFFERED);

                                    // =========================================================
                                    // 第 2 節：畫一個四分之一圓弧 (轉彎)
                                    // 假設從 X:3 繼續走，畫一個半徑 2 圈的順時針圓弧
                                    // =========================================================
                                    m_Motion.ArcMove(axesXY,
                                        { ONE_REV * 5.0, -ONE_REV * 2.0 }, // 目標點
                                        { 0.0, -ONE_REV * 2.0 },           // 中心點 (相對起點)
                                        0,                                 // 0 代表 CW 順時針
                                        ONE_REV, 0.2, 0.2, BufferMode::BUFFERED);

                                    // =========================================================
                                    // 第 3 節：Y 軸往負向繼續走 3 圈 (純直線)
                                    // =========================================================
                                    m_Motion.LineMove(axesXY, { ONE_REV * 5.0, -ONE_REV * 5.0 }, ONE_REV, 0.2, 0.2, BufferMode::BUFFERED);
                                    */




                                    // B0 B1排渣----------------------------------------------------
                                    // B0 B1加工暫停----------------------------------------------------
                                    // B3排渣----------------------------------------------------
                                    // B3加工暫停----------------------------------------------------
                                    // 2. 讓 X, Y, Z 三軸一起走斜線

                                    //std::vector<int> axes = { 0, 1, 2 };
                                    //m_Motion.LineMove(axes, { ONE_REV * 5.0, ONE_REV * 5.0, ONE_REV * 5.0 }, ONE_REV, 0.2, 0.2, BufferMode::BUFFERED);

                                    std::vector<int> axes = { 2 };
                                    m_Motion.LineMove(axes, { ONE_REV * 5.0 }, ONE_REV, 0.2, 0.2, BufferMode::BUFFERED);










                                    //B2 加工暫停
                                    /*
                                    m_Motion.LineMove(axes, { ONE_REV * 1.0, ONE_REV * 1.0, ONE_REV * 1.0 }, ONE_REV, 0.2, 0.2, BufferMode::BUFFERED);
                                    m_Motion.LineMove(axes, { ONE_REV * 0.0, ONE_REV * 0.0, ONE_REV * 0.0 }, ONE_REV, 0.2, 0.2, BufferMode::BUFFERED);
                                    m_Motion.LineMove(axes, { ONE_REV * 2.0, ONE_REV * 2.0, ONE_REV * 2.0 }, ONE_REV, 0.2, 0.2, BufferMode::BUFFERED);
                                    m_Motion.LineMove(axes, { ONE_REV * -1.0, ONE_REV * -1.0, ONE_REV * -1.0 }, ONE_REV, 0.2, 0.2, BufferMode::BUFFERED);
                                    m_Motion.LineMove(axes, { ONE_REV * 5.0, ONE_REV * 5.0, ONE_REV * 5.0 }, ONE_REV, 0.2, 0.2, BufferMode::BUFFERED);
                                    */
                                }


                                if (test_timer == 15)
                                {
                                    // LineMove排渣----------------------------------------------------
                                    // ArcMove排渣----------------------------------------------------
                                    /*
                                    // ==========================================
                                    // 1. 設定【退刀】序列 (總共退 3.5 圈)
                                    // 參數: { 距離(正值), 速度, 加速時間(s), 減速時間(s) }
                                    // ==========================================
                                    std::vector<JumpSegment> testRetract =
                                    {
                                        // 第 1 段：慢速拔出 (克服積碳吸力)
                                        { ONE_REV * 0.5,  ONE_REV * 0.5,  0.1, 0.1 },

                                        // 第 2 段：高速大跳躍 (產生強大水流沖走碳渣)
                                        { ONE_REV * 3.0,  ONE_REV * 2.0,  0.2, 0.2 }
                                    };

                                    // ==========================================
                                    // 2. 設定【進刀】序列 (總共也必須是 3.5 圈)
                                    // ==========================================
                                    std::vector<JumpSegment> testApproach =
                                    {
                                        // 第 1 段：高速降落 (節省時間，回到安全高度)
                                        { ONE_REV * 2.5,  ONE_REV * 2.0,  0.2, 0.2 },

                                        // 第 2 段：中速靠近 (準備進入狹窄的加工孔)
                                        { ONE_REV * 0.8,  ONE_REV * 0.5,  0.1, 0.1 },

                                        // 第 3 段：超慢速軟著陸 (最後 0.2 圈，避免電極撞擊工件)
                                        { ONE_REV * 0.2,  ONE_REV * 0.1,  0.1, 0.1 }
                                    };

                                    // 3. 觸發跳刀！(在頂端停留 800 毫秒)
                                    m_Motion.TriggerPathJump(JumpMode::B2_PATH_REVERSE,testRetract, testApproach, 800.0,2);
                                    */


                                    // 多段排渣----------------------------------------------------
                                    // 1. 設定單段「退刀」參數 (一口氣退 6 圈！強迫跨節)
                                    /*
                                    std::vector<JumpSegment> testRetract = {
                                        { ONE_REV * 6.0,  ONE_REV * 2.0,  0.2,  0.2 }
                                    };

                                    // 2. 設定單段「進刀」參數 (乖乖鑽回 6 圈)
                                    std::vector<JumpSegment> testApproach = {
                                        { ONE_REV * 6.0,  ONE_REV * 1.0,  0.2,  0.2 }
                                    };

                                    // 3. 觸發跳刀！(在頂端停留 800 毫秒)
                                    m_Motion.TriggerPathJump(JumpMode::B2_PATH_REVERSE, testRetract, testApproach, 800.0,2);
                                    */



                                    // B0 B1排渣----------------------------------------------------


                                     /*
                                    std::vector<JumpSegment> testRetract = {
                                        { ONE_REV * 1.0, ONE_REV * 2.0, 0.2, 0.2 },
                                      { ONE_REV * 1.0, ONE_REV * 3.0, 0.2, 0.2 }
                                    };

                                    std::vector<JumpSegment> testApproach = {
                                        { ONE_REV * 1.0, ONE_REV * 3.0, 0.2, 0.2 } ,
                                           { ONE_REV * 1.0, ONE_REV * 1.0, 0.2, 0.2 }

                                    };

                                    // 🟢 觸發跳刀！第一個參數改成 JumpMode::B1_SPECIFIC_AXIS，最後一個參數 2 代表 Z 軸
                                    //m_Motion.TriggerPathJump(JumpMode::B1_SPECIFIC_AXIS, testRetract, testApproach, 500.0, 2);
                                    m_Motion.TriggerPathJump(JumpMode::B0_REVERSE, testRetract, testApproach, 500.0, 2);
                                   */


                                }


                                //B3 排渣--------------------------------------------------------------------------------
                                /*
                                if (test_timer == 20)
                                {

                                    // 【腳本 1：放電點 -> 中心】 (橫移退刀)

                                    std::vector<JumpSegment> b3_toCenter =
                                    {
                                        // 距離寫 0.0 就好！反正底層會用 b3_distToCenter 強制蓋掉它
                                        { ONE_REV * 0.5, ONE_REV * 1.0, 0.2, 0.2 },  // 慢速橫移 (0.5圈/秒)，避免刮傷側壁
                                        { ONE_REV*0.5, ONE_REV * 1.5, 0.2, 0.2 }  // 慢速橫移 (0.5圈/秒)，避免刮傷側壁
                                    };

                                    // 【腳本 2：中心 -> 頂點】 (拔高排渣)
                                    std::vector<JumpSegment> b3_toApex =
                                    {
                                        // 這是真正的拔高距離！假設我們要在安全中心往上抽 5 圈
                                        { ONE_REV * 1.0, ONE_REV * 3, 0.2, 0.2 },  // 高速拔起 (3.0圈/秒)，產生強大抽吸力

                                        // 這是真正的拔高距離！假設我們要在安全中心往上抽 5 圈
                                        { ONE_REV * 2.0, ONE_REV * 5, 0.2, 0.2 }  // 高速拔起 (3.0圈/秒)，產生強大抽吸力
                                    };

                                    // 【腳本 3：頂點 -> 中心】 (高速降落)
                                    std::vector<JumpSegment> b3_fromApex =
                                    {
                                        // 降落距離跟拔高一樣
                                        { ONE_REV * 2.0, ONE_REV * 5, 0.2, 0.2 },  // 高速降落 (3.0圈/秒)，節省時間
                                         { ONE_REV * 1.0, ONE_REV * 3, 0.2, 0.2 }
                                    };

                                    // 【腳本 4：中心 -> 放電點】 (橫移進刀)
                                    std::vector<JumpSegment> b3_toWorkpiece =
                                    {
                                        // 距離一樣寫 0.0 就好！底層會自動算好剩下的帳
                                        {  ONE_REV * 0.1, ONE_REV * 2.5, 0.2, 0.2 } , // 極慢速摸回工件 (0.2圈/秒)，準備重新放電
                                            { ONE_REV * 0.1, ONE_REV * 1.5, 0.2, 0.2 }
                                    };

                                    // 2. 觸發全新 B3 API！
                                    m_Motion.TriggerCenterJump_B3
                                    (
                                        0.0, 0.0, 0.0,  // 🌟 [指定圓心]：假設你的螺旋下刀圓心在 X=0, Y=0, Z=0
                                        0.0, 0.0, 1.0*-1,  // 🌟 [拔高向量]：0, 0, 1 代表純 Z 軸垂直拔高 (若要斜向排渣可改 1,0,1)
                                        b3_toCenter,    // 塞入腳本 1
                                        b3_toApex,      // 塞入腳本 2
                                        b3_fromApex,    // 塞入腳本 3
                                        b3_toWorkpiece, // 塞入腳本 4
                                        100.0           // Dwell 停留時間：在最高點停留 500 毫秒沖油
                                    );
                                }*/


                                //B4 排渣--------------------------------------------------------------------------------
                                /*
                                if (test_timer == 20)
                                {
                                    // ====================================================================
                                    // 📐 [參數設定區]：在這裡指定你的 B4 搖動跳刀特徵！
                                    // ====================================================================
                                    double jumpHeight_Rev = 5.0;      // Z 軸要爬升幾圈 (高度)
                                    double tiltAngle_deg = 45.0;      // 搖動斜角 (例如 30度、45度、60度)
                                    double dirAngle_deg = 90.0;       // 退刀方向角 (0度=向+X退，90度=向+Y退，180度=向-X退)

                                    // ====================================================================
                                    // 🧠 [C# / 測試端 數學換算]：把角度變成 XYZ 座標
                                    // ====================================================================
                                    double PI = 3.14159265359;
                                    double tiltRad = tiltAngle_deg * PI / 180.0;
                                    double dirRad = dirAngle_deg * PI / 180.0;

                                    // 算出 XY 平面要退多少圈 (三角函數: 鄰邊 = 對邊 / tan(角度))
                                    double xyDist_Rev = jumpHeight_Rev / std::tan(tiltRad);

                                    // 🌟 直接給定當前放電點座標 (測試用)
                                    // 假設目前放電點在 X=0 圈, Y=0 圈, Z=-10 圈 (代表已經往下加工了一段距離)
                                    double startX = 0.0;
                                    double startY = 0.0;
                                    double startZ = -10.0 * ONE_REV ;

                                    // 計算出完美的「上中心點」(Upper Center)
                                    double upperCx = startX + (xyDist_Rev * ONE_REV * std::cos(dirRad));
                                    double upperCy = startY + (xyDist_Rev * ONE_REV * std::sin(dirRad));
                                    // 假設 Z 軸往上退刀是正向加回去 (請依你的機台座標系調整符號)
                                    double upperCz = startZ + (jumpHeight_Rev * ONE_REV);

                                    // ====================================================================
                                    // 🎬 [腳本設定區]：跟 B3 一模一樣！
                                    // ====================================================================
                                    // 【腳本 1：放電點 -> 上中心】 (斜角退刀)
                                    std::vector<JumpSegment> b4_toUpper = {
                                        // 距離寫 0.0，底層會自動算出斜邊長度並覆寫！
                                        {ONE_REV * 0.5, ONE_REV * 1.0, 0.2, 0.2 },
                                        { ONE_REV * 0.5, ONE_REV * 2.0, 0.2, 0.2 }
                                    };

                                    // 【腳本 2：上中心 -> 頂點】 (垂直拔高排渣)
                                    std::vector<JumpSegment> b4_toApex = {
                                        // 這是真正的垂直拔高距離！(從上中心再往上抽 3 圈)
                                        { ONE_REV * 1.0, ONE_REV * 3.0, 0.2, 0.2 },
                                        { ONE_REV * 2.0, ONE_REV * 5.0, 0.2, 0.2 }
                                    };

                                    // 【腳本 3：頂點 -> 上中心】 (垂直降落)
                                    std::vector<JumpSegment> b4_fromApex = {
                                        // 降落距離跟拔高一樣 (3圈)
                                        { ONE_REV * 2.0, ONE_REV * 5.0, 0.2, 0.2 },
                                        { ONE_REV * 1.0, ONE_REV * 3.0, 0.2, 0.2 }
                                    };

                                    // 【腳本 4：上中心 -> 放電點】 (斜角進刀)
                                    std::vector<JumpSegment> b4_toWorkpiece = {
                                        // 距離寫 0.0，底層會自動算好斜邊帳！
                                        { 0.0, ONE_REV * 2.0, 0.2, 0.2 },
                                        { 0.0, ONE_REV * 0.5, 0.2, 0.2 }  // 極慢速摸回工件
                                    };

                                    // ====================================================================
                                    // 🚀 [觸發全新 B4 API]
                                    // ====================================================================
                                    m_Motion.TriggerOrbitalJump_B4
                                    (
                                        upperCx, upperCy, upperCz, // 🌟 傳入算好的「上中心點」絕對座標
                                        0.0, 0.0, 1.0,             // 🌟 拔高向量：到了上中心後，純 Z 軸往上抽 (0,0,1)
                                        b4_toUpper,
                                        b4_toApex,
                                        b4_fromApex,
                                        b4_toWorkpiece,
                                        500.0                      // Dwell 停留 100ms
                                    );
                                }*/


                                //B0 加工暫停--------------------------------------------------------------------------------
                                /*
                                if (test_timer == 15)
                                {
                                    // 1. 定義退刀腳本 (例如：先慢速破真空，再高速拔高)
                                    std::vector<JumpSegment> b0_retract = {
                                       { ONE_REV * 1.0, ONE_REV * 2.0, 0.2, 0.2 }, // 第 1 段：慢速破真空
                                       { ONE_REV * 1.0, ONE_REV * 3.0, 0.2, 0.2 }, // 第 2 段：中速退離表面
                                       { ONE_REV * 2.0, ONE_REV * 5.0, 0.2, 0.2 } // 第 3 段：🌟 額外安全距離 (高速往上抽 10 圈！)
                                    };

                                    // 2. 定義進刀腳本 (例如：高速降落，最後再慢速尋邊)
                                    std::vector<JumpSegment> b0_approach = {
                                      { ONE_REV * 2.0, ONE_REV * 5.0, 0.2, 0.2 }, // 第 1 段：從高空高速降落
                                      { ONE_REV * 1.0,  ONE_REV * 3.0, 0.2, 0.2 }, // 第 2 段：中速接近
                                      { ONE_REV * 1.0,  ONE_REV * 1.0, 0.2, 0.2 }  // 第 3 段：極慢速尋邊摸回火花區
                                    };

                                    m_Motion.TriggerPause_B0(b0_retract, b0_approach);


                                }
                                if (test_timer == 30)
                                {
                                    // 🌟 呼叫極簡版對齊引擎 API！
                                    // 參數 1 (alignMode) : 0 = 先動 Z 軸，再動 XY 軸 (最安全的防呆模式)
                                    // 0>>名單內的先走。（走完後，名單外的才走）
                                    // 1>>名單外的先走。（走完後，名單內的才走）
                                    // 2>>全軸同動
                                    // 參數 2 (alignVel)  : 對齊回到頂點的速度 (例如 1 圈/秒)
                                    // 參數 3 (primaryIdx): 優先軸是誰？ 2 代表 Z 軸

                                    // 參數1為0(名單內的先動)，參數3為3(X和Y)
                                   m_Motion.TriggerPauseResume(0, ONE_REV * 1.0, 3);
                                }*/







                                //B0 加工暫停--------------------------------------------------------------------------------
                                /*
                                if (test_timer == 15)
                                {
                                    // 1. 定義退刀腳本 (例如：先慢速破真空，再高速拔高)
                                    std::vector<JumpSegment> b1_retract = {
                                       { ONE_REV * 1.0, ONE_REV * 2.0, 0.2, 0.2 }, // 第 1 段：慢速破真空
                                       { ONE_REV * 1.0, ONE_REV * 3.0, 0.2, 0.2 }, // 第 2 段：中速退離表面
                                       { ONE_REV * 2.0, ONE_REV * 5.0, 0.2, 0.2 } // 第 3 段：🌟 額外安全距離 (高速往上抽 10 圈！)
                                    };

                                    // 2. 定義進刀腳本 (例如：高速降落，最後再慢速尋邊)
                                    std::vector<JumpSegment> b1_approach = {
                                      { ONE_REV * 2.0, ONE_REV * 5.0, 0.2, 0.2 }, // 第 1 段：從高空高速降落
                                      { ONE_REV * 1.0,  ONE_REV * 3.0, 0.2, 0.2 }, // 第 2 段：中速接近
                                      { ONE_REV * 1.0,  ONE_REV * 1.0, 0.2, 0.2 }  // 第 3 段：極慢速尋邊摸回火花區
                                    };
                                    // 🌟 呼叫 B1 API！
                                    // 參數 3: 2 代表 Z 軸 (X=0, Y=1, Z=2)
                                    // 參數 4: 1.0 代表正方向往上退
                                    m_Motion.TriggerPause_B1(b1_retract, b1_approach, 2, 1.0);


                                }
                                if (test_timer == 30)
                                {
                                    // 🌟 呼叫極簡版對齊引擎 API！
                                    // 參數 1 (alignMode) : 0 = 先動 Z 軸，再動 XY 軸 (最安全的防呆模式)
                                    // 0>>名單內的先走。（走完後，名單外的才走）
                                    // 1>>名單外的先走。（走完後，名單內的才走）
                                    // 2>>全軸同動
                                    // 參數 2 (alignVel)  : 對齊回到頂點的速度 (例如 1 圈/秒)
                                    // 參數 3 (primaryIdx): 優先軸是誰？ 2 代表 Z 軸

                                    // 參數1為0(名單內的先動)，參數3為3(X和Y)
                                    m_Motion.TriggerPauseResume(0, ONE_REV * 1.0, 3);
                                }*/


                                //B0 加工暫停--------------------------------------------------------------------------------
                                /*
                                if (test_timer == 30)
                                {
                                    std::vector<JumpSegment> b2_retract = {
                                        { ONE_REV * 1.0, ONE_REV * 1.0, 0.2, 0.2 }, // 慢速破真空
                                        { ONE_REV * 100.0*1, ONE_REV * 6, 0.2, 0.2} // 🌟 直接給超大距離，讓它無腦沿原路退到起點！
                                    };


                                    std::vector<JumpSegment> b2_approach = {
                                        { ONE_REV * 0.0, ONE_REV * 6.0, 0.2, 0.2 },// 高速沿原路回去
                                        { ONE_REV * 1.0, ONE_REV * 1.0, 0.2, 0.2 }   // 慢速尋邊
                                    };


                                    m_Motion.TriggerPause_B2(b2_retract, b2_approach);
                                }


                                if (test_timer == 50)
                                {
                                    // 🌟 呼叫極簡版對齊引擎 API！
                                    // 參數 1 (alignMode) : 0 = 先動 Z 軸，再動 XY 軸 (最安全的防呆模式)
                                    // 0>>名單內的先走。（走完後，名單外的才走）
                                    // 1>>名單外的先走。（走完後，名單內的才走）
                                    // 2>>全軸同動
                                    // 參數 2 (alignVel)  : 對齊回到頂點的速度 (例如 1 圈/秒)
                                    // 參數 3 (primaryIdx): 優先軸是誰？ 2 代表 Z 軸

                                    // 參數1為0(名單內的先動)，參數3為3(X和Y)
                                  // B2 一樣共用對齊引擎！全同動(2) 或者 先Z後XY(0) 隨你高興
                                    m_Motion.TriggerPauseResume(0, ONE_REV * 1.0, 4);
                                }*/



                                //B3 加工暫停--------------------------------------------------------------------------------
                                /*
                                if (test_timer == 20) // 假設這是你的暫停測試按鈕或條件
                                {
                                    // 【腳本 1：放電點 -> 中心】 (橫移退刀)
                                    std::vector<JumpSegment> b3_toCenter =
                                    {
                                        { ONE_REV * 0.5, ONE_REV * 1.0, 0.2, 0.2 },
                                        { ONE_REV * 0.5, ONE_REV * 1.5, 0.2, 0.2 }
                                    };

                                    // 【腳本 2：中心 -> 頂點】 (拔高退刀)
                                    std::vector<JumpSegment> b3_toApex =
                                    {
                                        { ONE_REV * 1.0, ONE_REV * 3.0, 0.2, 0.2 },
                                        { ONE_REV * 2.0, ONE_REV * 5.0, 0.2, 0.2 }
                                    };

                                    // 【腳本 3：頂點 -> 中心】 (高速降落 - ⚠️ 暫停期間不會執行，會存在大腦裡等 Resume)
                                    std::vector<JumpSegment> b3_fromApex =
                                    {
                                        { ONE_REV * 2.0, ONE_REV * 5.0, 0.2, 0.2 },
                                        { ONE_REV * 1.0, ONE_REV * 3.0, 0.2, 0.2 }
                                    };

                                    // 【腳本 4：中心 -> 放電點】 (橫移進刀 - ⚠️ 暫停期間不會執行，會存在大腦裡等 Resume)
                                    std::vector<JumpSegment> b3_toWorkpiece =
                                    {
                                        { ONE_REV * 0.1, ONE_REV * 2.5, 0.2, 0.2 },
                                        { ONE_REV * 0.1, ONE_REV * 1.5, 0.2, 0.2 }
                                    };

                                    // 🌟 2. 觸發全新的 B3 暫停 API！

                                    m_Motion.TriggerPause_B3
                                    (
                                        0.0, 0.0, 0.0,     // 🌟 [指定圓心]：假設你的下刀圓心在 X=0, Y=0, Z=0
                                        0.0, 0.0, 1.0 * -1,  // 🌟 [拔高向量]：0, 0, -1 代表純 Z 軸拔高
                                        b3_toCenter,       // 塞入腳本 1
                                        b3_toApex,         // 塞入腳本 2
                                        b3_fromApex,       // 塞入腳本 3
                                        b3_toWorkpiece     // 塞入腳本 4
                                        // 🛑 移除了 Dwell Time，因為暫停是停在空中無限期等待！
                                    );
                                }

                                if (test_timer == 35)
                                {
                                    // 🌟 呼叫極簡版對齊引擎 API！
                                    // 參數 1 (alignMode) : 0 = 先動 Z 軸，再動 XY 軸 (最安全的防呆模式)
                                    // 0>>名單內的先走。（走完後，名單外的才走）
                                    // 1>>名單外的先走。（走完後，名單內的才走）
                                    // 2>>全軸同動
                                    // 參數 2 (alignVel)  : 對齊回到頂點的速度 (例如 1 圈/秒)
                                    // 參數 3 (primaryIdx): 優先軸是誰？ 2 代表 Z 軸

                                    // 參數1為0(名單內的先動)，參數3為3(X和Y)
                                    m_Motion.TriggerPauseResume(0, ONE_REV * 1.0, 3);

                                }*/



                                //B4 加工暫停--------------------------------------------------------------------------------

                                /*
                                if (test_timer == 20) // 觸發 B4 加工暫停測試
                                {
                                    // ====================================================================
                                    // 📐 [參數設定區]：在這裡指定你的 B4 搖動跳刀特徵！
                                    // ====================================================================
                                    double jumpHeight_Rev = 5.0;      // Z 軸要爬升幾圈 (高度)
                                    double tiltAngle_deg = 45.0;      // 搖動斜角 (例如 30度、45度、60度)
                                    double dirAngle_deg = 90.0;       // 退刀方向角 (0度=向+X退，90度=向+Y退，180度=向-X退)

                                    // ====================================================================
                                    // 🧠 [C# / 測試端 數學換算]：把角度變成 XYZ 座標
                                    // ====================================================================
                                    double PI = 3.14159265359;
                                    double tiltRad = tiltAngle_deg * PI / 180.0;
                                    double dirRad = dirAngle_deg * PI / 180.0;

                                    // 算出 XY 平面要退多少圈 (三角函數: 鄰邊 = 對邊 / tan(角度))
                                    double xyDist_Rev = jumpHeight_Rev / std::tan(tiltRad);

                                    // 🌟 直接給定當前放電點座標 (測試用)
                                    // 假設目前放電點在 X=0 圈, Y=0 圈, Z=-10 圈 (代表已經往下加工了一段距離)
                                    double startX = 0.0;
                                    double startY = 0.0;
                                    double startZ = -10.0 * ONE_REV;

                                    // 計算出完美的「上中心點」(Upper Center)
                                    double upperCx = startX + (xyDist_Rev * ONE_REV * std::cos(dirRad));
                                    double upperCy = startY + (xyDist_Rev * ONE_REV * std::sin(dirRad));
                                    // 假設 Z 軸往上退刀是正向加回去 (請依你的機台座標系調整符號)
                                    double upperCz = startZ + (jumpHeight_Rev * ONE_REV);

                                    // ====================================================================
                                    // 🎬 [腳本設定區]：跟 B3 一模一樣！
                                    // ====================================================================
                                    // 【腳本 1：放電點 -> 上中心】 (斜角退刀)
                                    std::vector<JumpSegment> b4_toUpper = {
                                        // 距離寫 0.0，底層會自動算出斜邊長度並覆寫！
                                        {ONE_REV * 0.5, ONE_REV * 1.0, 0.2, 0.2 },
                                        { ONE_REV * 0.5, ONE_REV * 2.0, 0.2, 0.2 }
                                    };

                                    // 【腳本 2：上中心 -> 頂點】 (垂直拔高排渣)
                                    std::vector<JumpSegment> b4_toApex = {
                                        // 這是真正的垂直拔高距離！(從上中心再往上抽 3 圈)
                                        { ONE_REV * 1.0, ONE_REV * 3.0, 0.2, 0.2 },
                                        { ONE_REV * 2.0, ONE_REV * 5.0, 0.2, 0.2 }
                                    };

                                    // 【腳本 3：頂點 -> 上中心】 (垂直降落)
                                    std::vector<JumpSegment> b4_fromApex = {
                                        // 降落距離跟拔高一樣 (3圈)
                                        { ONE_REV * 2.0, ONE_REV * 5.0, 0.2, 0.2 },
                                        { ONE_REV * 1.0, ONE_REV * 3.0, 0.2, 0.2 }
                                    };

                                    // 【腳本 4：上中心 -> 放電點】 (斜角進刀)
                                    std::vector<JumpSegment> b4_toWorkpiece = {
                                        // 距離寫 0.0，底層會自動算好斜邊帳！
                                        { 0.0, ONE_REV * 2.0, 0.2, 0.2 },
                                        { 0.0, ONE_REV * 0.5, 0.2, 0.2 }  // 極慢速摸回工件
                                    };

                                    // ====================================================================
                                    // 🚀 [呼叫 B4 暫停 API]
                                    // ====================================================================
                                    m_Motion.TriggerPause_B4
                                    (
                                        upperCx * 0, upperCy * 0, upperCz * 0, // 傳入算好的 3D 中繼點
                                        0.0, 0.0, 1.0,             // 拔高向量：垂直向上 (0,0,1)
                                        b4_toUpper,
                                        b4_toApex,
                                        b4_fromApex,
                                        b4_toWorkpiece
                                        // 🛑 無 dwellTime 參數，會自動進入 PAUSED_HOLD
                                    );
                                }

                                if (test_timer == 35)
                                {
                                    // 🌟 呼叫極簡版對齊引擎 API！
                                    // 參數 1 (alignMode) : 0 = 先動 Z 軸，再動 XY 軸 (最安全的防呆模式)
                                    // 0>>名單內的先走。（走完後，名單外的才走）
                                    // 1>>名單外的先走。（走完後，名單內的才走）
                                    // 2>>全軸同動
                                    // 參數 2 (alignVel)  : 對齊回到頂點的速度 (例如 1 圈/秒)
                                    // 參數 3 (primaryIdx): 優先軸是誰？ 2 代表 Z 軸

                                    // 參數1為0(名單內的先動)，參數3為3(X和Y)
                                    m_Motion.TriggerPauseResume(0, ONE_REV * 1.0, 3);

                                }*/


                                test_timer++;




                            }






                        }
                        else
                        {
                            // 未知狀態或正在切換中，維持目前指令或設為 0
                            // servo.pOutput->ControlWord = 0x0000;
                        }

                        // 列印除錯用
                        // DEBUG_PRINT("Status: 0x%04X, Control: 0x%04X\n", statusWord, servo.pOutput->ControlWord);
                    }
                }

            }

            // 讀取目前的 AD 值顯示
            int16_t adVal = 0;
            if (!m_AdList.empty())
            {
                adVal = *(int16_t*)m_AdList[0].pInputLoc;
            }
            // 印出診斷資訊
            // Jitter 單位是 100ns (例如 50 代表 5us)

            DEBUG_PRINT(">>> [1s] WKC:%d | Timeouts:%d | Err:%d | AD:%d\n", wkc_PDO, timeout_count_PDO, wkc_error_count_PDO, adVal);

            // 假設您是要讀取 Slave 2 的 16 個 bits
            DEBUG_PRINT("Slave 2 Output: ");

            // 我們從 15 倒數回 0，這樣印出來才是符合人類閱讀習慣的二進制 (MSB -> LSB)

            for (int bit = 15; bit >= 0; bit--)
            {
                bool status = Get_O(2, bit);
                DEBUG_PRINT("%d", status);

                if (bit == 8) DEBUG_PRINT(" "); // 中間加個空格方便閱讀
            }
            DEBUG_PRINT("\n");

        }


        if (timer_10ms_Count % 10 == 0)
        {
            //UpdatePathServoVelocity ---------------------------------------------------------

            /*
            double targetVel = 8388608.0 * 10;
            // --- 階段 3：開始鐘擺震盪 (第 2 秒以後) ---
            if (test_timer > 8)
            {
                if (debug_EDM == 0)
                {

                    m_Motion.UpdatePathServoVelocity(targetVel * debug_EDM);
                }
                else
                {

                    m_Motion.UpdatePathServoVelocity(targetVel * debug_EDM);
                }
                debug_EDM = -debug_EDM;
            }*/
        }


        //10ms
        if (timer_10ms >= 10)
        {
            timer_10ms = 0; // 執行完立刻歸零
            timer_10ms_Count += 1;
            //DEBUG_PRINT("10ms\n");
        }

        //100ms
        if (timer_100ms >= 100)
        {
            timer_100ms = 0; // 執行完立刻歸零
            timer_100ms_Count += 1;
            //DEBUG_PRINT("100ms\n");
        }

        //1000ms
        if (timer_1000ms >= 1000)
        {
            timer_1000ms = 0; // 執行完立刻歸零
            timer_1000ms_Count += 1;
            //DEBUG_PRINT("1000ms\n");
        }


        if (Debug_test_timer >= 1000)
        {
            Debug_test_timer = 0; // 執行完立刻歸零
            Debug_test_timer_Count += 1;
            //DEBUG_PRINT("Debug_test_timer ms\n");
        }







    }


}