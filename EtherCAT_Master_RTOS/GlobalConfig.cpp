#include "GlobalConfig.h"
#include "ConfigReader.h"
#include "MotionCore.h" // 🌟 必須引入，才能操作 axes 和 motion
#include <algorithm>
#include <windows.h> 
#include <rtapi.h>


bool GlobalConfig::LoadAxisConfig(const std::string& filePath, std::vector<AxisContext>& axes, MotionCore& motion)
{
    //讀取參數確定軸數量-----------------------------------------------------------------
    int axisCount = (int)ConfigUtil::ReadParam(filePath, "AxisCount", 0.0);
    DEBUG_PRINT("LoadAxisConfig Axis Count>>%d\n", axisCount);
    if (axisCount == 0) 
    {
        
    }
    else
    {
        axes.resize(axisCount);//調整 m_Axes 陣列的大小

       
        for (int i = 0; i < axisCount; i++)//開始填入軸參數
        {
            std::string prefix = std::to_string(i) + "_"; // 產生 "0_", "1_" 等前綴

           
              
            // ----------------------------------------------------
            //基礎初始化
            // ----------------------------------------------------
            double res = ConfigUtil::ReadParam(filePath, prefix + "Resolution", 16777216.0);//解析度
            motion.InitAxis(axes[i], res); // InitAxis 會幫你把 currentCmdPos 等動態變數歸零

            // ----------------------------------------------------
            //讀取物理與運動參數
            // ----------------------------------------------------
            double rpm = ConfigUtil::ReadParam(filePath, prefix + "MaxVel_RPM", 3000.0);
            axes[i].maxVel_PPS = MotionCore::RpmToPps(rpm, axes[i].resolution_PPR);      // 最高轉速 (Pulse/sec)

            // 讀取加減速所需的時間(秒)，然後換算成加速度(PPS2)
            // 加速度 = 目標速度 / 加速時間
            double accTime = ConfigUtil::ReadParam(filePath, prefix + "Acc_Time", 1);// 加速到滿速所需時間(秒)
            double decTime = ConfigUtil::ReadParam(filePath, prefix + "Dec_Time", 1);// 減速所需時間(秒)

            // 防呆：避免除以零
            axes[i].acc_PPS2 = (accTime > 0.0) ? (axes[i].maxVel_PPS / accTime) : (axes[i].maxVel_PPS * 2.0);// 加速到滿速所需時間(秒)
            axes[i].dec_PPS2 = (decTime > 0.0) ? (axes[i].maxVel_PPS / decTime) : (axes[i].maxVel_PPS * 2.0);

            double smoothTime = ConfigUtil::ReadParam(filePath, prefix + "SmoothTime", 100.0);//平滑時間(ms)

            motion.InitSmoothBuffer(axes[i], smoothTime);//加加速度來開啟 100ms 的平滑功能
            motion.InitVirtualAxisSmooth(smoothTime);

            // ----------------------------------------------------
            // 雙閉環參數
            // ----------------------------------------------------
            int fbVal = (int)ConfigUtil::ReadParam(filePath, prefix + "FbMode", 0.0);// 0=馬達編碼器, 1=外部光學尺
            axes[i].fbMode = (fbVal == 1) ? FeedbackSource::LINEAR_SCALE : FeedbackSource::MOTOR_ENCODER;

            axes[i].scaleToMotorRatio = ConfigUtil::ReadParam(filePath, prefix + "ScaleRatio", 1.0);// 光學尺與馬達的解析度比例
            axes[i].maxDeviation = ConfigUtil::ReadParam(filePath, prefix + "MaxDev", 1677721.6);// 雙閉環最大容許偏差

            // ----------------------------------------------------
            // D. PID 參數與保護
            // ----------------------------------------------------
            axes[i].pid.Kp = ConfigUtil::ReadParam(filePath, prefix + "Kp", 20.0);// 比例增益 (剛性)
            axes[i].pid.Ki = ConfigUtil::ReadParam(filePath, prefix + "Ki", 10.0);// 積分增益 (消除靜差)
            axes[i].pid.Kd = ConfigUtil::ReadParam(filePath, prefix + "Kd", 0.0);// 微分增益 (阻尼)

            axes[i].pid.EnableLagCheck = (ConfigUtil::ReadParam(filePath, prefix + "EnableLagCheck", 1.0) == 1.0);//跟隨誤差保護 0不啟用 1啟用
            axes[i].pid.MaxLag = ConfigUtil::ReadParam(filePath, prefix + "MaxLag", 100000.0);// 最大允許跟隨誤差

           
        }
    }


    return true;
}
void GlobalConfig::LoadFromFile(const std::string& filePath)
{
    //----------系統模式
    std::string modeStr = ConfigUtil::ReadConfigString(filePath, "System_Mode", "UNKNOWN_MODE");
    std::transform(modeStr.begin(), modeStr.end(), modeStr.begin(), ::toupper);

    if (modeStr == "EDM_SINKER_MODE")
    {
        systemMode = SystemMode::EDM_SINKER_MODE;
    }
    else  if (modeStr == "EXAMPLE_MODE")
    {
        systemMode = SystemMode::EXAMPLE_MODE;
    }
    else
    {
        systemMode = SystemMode::UNKNOWN_MODE;
    }

    // 2. 讀取 Debug 開關 (預設給 0)
    Debug_ShowMessage = (int)ConfigUtil::ReadParam(filePath, "Debug_ShowMessage", 0.0);



    switch (systemMode)
    {
    case SystemMode::EDM_SINKER_MODE:
        DEBUG_PRINT("systemMode:EDM_SINKER_MODE\n");
        break;
    case SystemMode::EXAMPLE_MODE:
        DEBUG_PRINT("systemMode:EXAMPLE_MODE\n");
        break;
    case SystemMode::UNKNOWN_MODE:
        DEBUG_PRINT("systemMode:UNKNOWN_MODE\n");
        break;

    }

    DEBUG_PRINT("Debug_ShowMes:%d\n", Debug_ShowMessage);
   
   

   
   
}