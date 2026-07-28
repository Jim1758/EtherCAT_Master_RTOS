#include "GlobalConfig.h"
#include "ConfigReader.h"
#include "MotionCore.h" // 🌟 必須引入，才能操作 axes 和 motion
#include <algorithm>
#include <windows.h> 
#include <rtapi.h>
#include "CompensationEngine.h" // 必須引入引擎結構
#include <fstream>
#include <sstream>
#include <vector>
bool GlobalConfig::LoadAxisConfig(const std::string& filePath, std::vector<AxisContext>& axes, MotionCore& motion)
{
    //讀取參數確定軸數量-----------------------------------------------------------------
    int axisCount = (int)ConfigUtil::ReadParam(filePath, "AxisCount", 0.0);
    System_axisCount = axisCount;
    DEBUG_PRINT("LoadAxisConfig Axis Count>>%d\n", axisCount);
    if (axisCount == 0) 
    {
        
    }
    else
    {
        axes.resize(axisCount);//調整 m_Axes 陣列的大小

       
        for (int i = 0; i < axisCount; i++)//開始填入軸參數
        {
            std::string prefix = std::to_string(i) + "_";

            axes[i].axisIndex = (int)ConfigUtil::ReadParam(filePath, prefix + "axisIndex", i);

            // ----------------------------------------------------
            // 1. 基礎初始化
            // ----------------------------------------------------
            double res = ConfigUtil::ReadParam(filePath, prefix + "Resolution", 16777216.0);
            motion.InitAxis(axes[i], res);

            // ----------------------------------------------------
            // 2. 🌟 [新增] 機械機構參數 (由參數檔讀取)
            // ----------------------------------------------------
            axes[i].reduction_MotorSide = ConfigUtil::ReadParam(filePath, prefix + "Reduction_MotorSide", 1.0);
            axes[i].reduction_LoadSide = ConfigUtil::ReadParam(filePath, prefix + "Reduction_LoadSide", 1.0);
            axes[i].mechanicalPitch = ConfigUtil::ReadParam(filePath, prefix + "MechanicalPitch", 10.0);
            axes[i].isReverse = (ConfigUtil::ReadParam(filePath, prefix + "IsReverse", 0.0) == 1.0);

            // 🌟 自動計算最終導程 (Final Lead)
            // 確保 LoadSide 不為 0 以防除以零錯誤
            if (axes[i].reduction_LoadSide > 0.0001) {
                axes[i].finalLead = (axes[i].mechanicalPitch * axes[i].reduction_MotorSide) / axes[i].reduction_LoadSide;
            }
            else {
                axes[i].finalLead = axes[i].mechanicalPitch; // 防呆
            }

            // ----------------------------------------------------
            // 3. 讀取物理與運動參數
            // ----------------------------------------------------
           

            double accTime = ConfigUtil::ReadParam(filePath, prefix + "Acc_Time", 1);
            double decTime = ConfigUtil::ReadParam(filePath, prefix + "Dec_Time", 1);

            axes[i].acc_PPS2 = (accTime > 0.0) ? (axes[i].maxVel_PPS / accTime) : (axes[i].maxVel_PPS * 2.0);
            axes[i].dec_PPS2 = (decTime > 0.0) ? (axes[i].maxVel_PPS / decTime) : (axes[i].maxVel_PPS * 2.0);

            double smoothTime = ConfigUtil::ReadParam(filePath, prefix + "SmoothTime", 100.0);
            motion.InitSmoothBuffer(axes[i], smoothTime);
            motion.InitVirtualAxisSmooth(smoothTime);

            // ----------------------------------------------------
            // 4. 雙閉環與 PID 參數
            // ----------------------------------------------------
            int fbVal = (int)ConfigUtil::ReadParam(filePath, prefix + "FbMode", 0.0);
            axes[i].fbMode = (fbVal == 1) ? FeedbackSource::LINEAR_SCALE : FeedbackSource::MOTOR_ENCODER;

            axes[i].scaleToMotorRatio = ConfigUtil::ReadParam(filePath, prefix + "ScaleRatio", 1.0);
            axes[i].maxDeviation = ConfigUtil::ReadParam(filePath, prefix + "MaxDev", 1677721.6);

            
            axes[i].pid.EnableLagCheck = (ConfigUtil::ReadParam(filePath, prefix + "EnableLagCheck", 1.0) == 1.0);
            axes[i].pid.MaxLag = ConfigUtil::ReadParam(filePath, prefix + "MaxLag", 100000.0);

            // ----------------------------------------------------
            // 5. 軸型態與補償參數
            // ----------------------------------------------------
            int typeVal = (int)ConfigUtil::ReadParam(filePath, prefix + "AxisType", 0.0);
            axes[i].axisType = (typeVal == 1) ? AxisType::ROTARY : (typeVal == 2 ? AxisType::ROTARY_CONTINUOUS : AxisType::LINEAR);
            axes[i].rotaryModulo = ConfigUtil::ReadParam(filePath, prefix + "RotaryModulo", 360.0);
            axes[i].useShortestPath = (ConfigUtil::ReadParam(filePath, prefix + "ShortestPath", 0.0) == 1.0);

            axes[i].enableBacklash = (ConfigUtil::ReadParam(filePath, prefix + "EnableBacklash", 0.0) == 1.0);
            axes[i].backlashAmount_mm = ConfigUtil::ReadParam(filePath, prefix + "BacklashAmount", 0.0);
            axes[i].enablePitch = (ConfigUtil::ReadParam(filePath, prefix + "EnablePitch", 0.0) == 1.0);
            axes[i].pitchStartPos_mm = ConfigUtil::ReadParam(filePath, prefix + "PitchStartPos", 0.0);
            axes[i].pitchStep_mm = ConfigUtil::ReadParam(filePath, prefix + "PitchStep", 10.0);

            motion.m_CompEngine.InitAxisCompensation(i, axes[i].enableBacklash, axes[i].backlashAmount_mm, axes[i].enablePitch, axes[i].pitchStep_mm);
        }
    }


    return true;
}


bool GlobalConfig::LoadPIDConfig(const std::string& filePath, std::vector<AxisContext>& axes, MotionCore& motion)
{
    //讀取參數確定軸數量-----------------------------------------------------------------
    int axisCount = System_axisCount;
    DEBUG_PRINT("LoadPIDConfig Axis Count>>%d\n", axisCount);
    if (axisCount == 0)
    {

    }
    else
    {
        axes.resize(axisCount);//調整 m_Axes 陣列的大小


        for (int i = 0; i < axisCount; i++)//開始填入軸參數
        {
            std::string prefix = std::to_string(i) + "_";

            axes[i].pid.Kp = ConfigUtil::ReadParam(filePath, prefix + "Kp_IDLE", 20.0);
            axes[i].pid.Ki = ConfigUtil::ReadParam(filePath, prefix + "Ki_IDLE", 10.0);
            axes[i].pid.Kd = ConfigUtil::ReadParam(filePath, prefix + "Kd_IDLE", 0.0);
     
            //閒置時PID---------------------------
            axes[i].Pid_IDLE.Kp = ConfigUtil::ReadParam(filePath, prefix + "Kp_IDLE", 20.0);
            axes[i].Pid_IDLE.Ki = ConfigUtil::ReadParam(filePath, prefix + "Ki_IDLE", 10.0);
            axes[i].Pid_IDLE.Kd = ConfigUtil::ReadParam(filePath, prefix + "Kd_IDLE", 0.0);
            //G00時PID---------------------------
            axes[i].Pid_IDLE.Kp = ConfigUtil::ReadParam(filePath, prefix + "Kp_G00", 20.0);
            axes[i].Pid_IDLE.Ki = ConfigUtil::ReadParam(filePath, prefix + "Ki_G00", 10.0);
            axes[i].Pid_IDLE.Kd = ConfigUtil::ReadParam(filePath, prefix + "Kd_G00", 0.0);
        }
    }


    return true;
}


bool GlobalConfig::LoadSpeedConfig(const std::string& filePath, std::vector<AxisContext>& axes, MotionCore& motion)
{
    //讀取參數確定軸數量-----------------------------------------------------------------
    int axisCount = System_axisCount;
    DEBUG_PRINT("LoadPIDConfig Axis Count>>%d\n", axisCount);
    if (axisCount == 0)
    {

    }
    else
    {
        axes.resize(axisCount);//調整 m_Axes 陣列的大小


        for (int i = 0; i < axisCount; i++)//開始填入軸參數
        {
            std::string prefix = std::to_string(i) + "_";

            double Max_speed = ConfigUtil::ReadParam(filePath, prefix + "MAX_Speed", 5000.0);
            axes[i].maxVel_PPS = MotionCore::UnitPerMinToPps(Max_speed, axes[i].resolution_PPR, axes[i].finalLead);

            double g00_speed_user = ConfigUtil::ReadParam(filePath, prefix + "G00_Speed", 5000.0);
            axes[i].G00_PPS = MotionCore::UnitPerMinToPps(g00_speed_user, axes[i].resolution_PPR, axes[i].finalLead);
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
bool GlobalConfig::LoadPitchTable(const std::string& filePath, CompensationEngine& compEngine)
{
    std::ifstream in(filePath);
    if (!in.is_open()) {
        DEBUG_PRINT("[WARN] PITCH_TABLE.txt not found at: %s\n", filePath.c_str());
        return false;
    }

    std::string line;
    // 建立 8 個暫存陣列，用來收集 8 軸各自的整排資料
    std::vector<double> pitchData[8];

    while (std::getline(in, line)) {
        // 過濾空白行與註解 (分號或雙斜線開頭)
        if (line.empty() || line[0] == ';' || line[0] == '/' || line[0] == '#') {
            continue;
        }

        std::stringstream ss(line);
        double val;

        // 橫向讀取這一行的 8 個數值
        for (int i = 0; i < 8; ++i) {
            if (ss >> val) {
                pitchData[i].push_back(val);
            }
            else {
                pitchData[i].push_back(0.0); // 防呆：如果檔案少寫欄位，自動補 0
            }
        }
    }
    in.close();

    // 將收集好的直向陣列，一軸一軸餵給 CompensationEngine
    for (int i = 0; i < 8; ++i) {
        if (!pitchData[i].empty()) {
            compEngine.SetPitchTable(i, pitchData[i]);
            // DEBUG_PRINT("[INFO] Axis %d Loaded Pitch Table, Size: %d\n", i, (int)pitchData[i].size());
        }
    }

    return true;
}

