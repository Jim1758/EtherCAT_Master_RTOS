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

          
          

            // ----------------------------------------------------
            // 1. 基礎初始化
            // ----------------------------------------------------
            double res = ConfigUtil::ReadParam(filePath, prefix + "Resolution", 16777216.0);
            motion.InitAxis(axes[i], res);


            axes[i].axisIndex = (int)ConfigUtil::ReadParam(filePath, prefix + "axisIndex", i);


            axes[i].isExist = (ConfigUtil::ReadParam(filePath, prefix + "isExist", 1.0) == 1.0);

            // ----------------------------------------------------
            // 2. 🌟 [新增] 機械機構參數 (由參數檔讀取)
            // ----------------------------------------------------
            axes[i].reduction_MotorSide = ConfigUtil::ReadParam(filePath, prefix + "Reduction_MotorSide", 1.0);
            axes[i].reduction_LoadSide = ConfigUtil::ReadParam(filePath, prefix + "Reduction_LoadSide", 1.0);
            axes[i].mechanicalPitch = ConfigUtil::ReadParam(filePath, prefix + "MechanicalPitch", 10.0);
            axes[i].isReverse = (ConfigUtil::ReadParam(filePath, prefix + "isReverse", 0.0) == 1.0);
            axes[i].Axis_Reverse = (ConfigUtil::ReadParam(filePath, prefix + "Axis_Reverse", 0.0) == 1.0);

       

            // 🌟 自動計算最終導程 (Final Lead)
            // 確保 LoadSide 不為 0 以防除以零錯誤
            if (axes[i].reduction_LoadSide > 0.0001) {
                axes[i].finalLead = (axes[i].mechanicalPitch * axes[i].reduction_MotorSide) / axes[i].reduction_LoadSide;
            }
            else {
                axes[i].finalLead = axes[i].mechanicalPitch; // 防呆
            }

            double pulsePerUnit = axes[i].resolution_PPR / axes[i].finalLead;

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
            double maxLag_mm = ConfigUtil::ReadParam(filePath, prefix + "maxLag_mm", 2.0);
            axes[i].maxLag_mm = maxLag_mm;
            axes[i].pid.MaxLag = axes[i].maxLag_mm * pulsePerUnit;
          

            //到位視窗判定
            double  inPositionWindow_mm= ConfigUtil::ReadParam(filePath, prefix + "inPositionWindow_mm", 0.005);
            axes[i].inPositionWindow_mm = inPositionWindow_mm;
            axes[i].inPositionWindow_Pulse= axes[i].inPositionWindow_mm * pulsePerUnit;
       

            // ----------------------------------------------------
            // 5. 軸型態與補償參數
            // ----------------------------------------------------
            int typeVal = (int)ConfigUtil::ReadParam(filePath, prefix + "AxisType", 0.0);
            axes[i].axisType = (typeVal == 1) ? AxisType::ROTARY : (typeVal == 2 ? AxisType::ROTARY_CONTINUOUS : AxisType::LINEAR);
            axes[i].rotaryModulo = ConfigUtil::ReadParam(filePath, prefix + "RotaryModulo", 360.0);
            axes[i].useShortestPath = (ConfigUtil::ReadParam(filePath, prefix + "ShortestPath", 0.0) == 1.0);

            axes[i].enableBacklash = (ConfigUtil::ReadParam(filePath, prefix + "EnableBacklash", 0.0) == 1.0);
            axes[i].backlashAmount_Pos_mm = ConfigUtil::ReadParam(filePath, prefix + "backlashAmount_Pos_mm", 0.0);
            axes[i].backlashAmount_Neg_mm = ConfigUtil::ReadParam(filePath, prefix + "backlashAmount_Neg_mm", 0.0);
            axes[i].backlashSpeed = ConfigUtil::ReadParam(filePath, prefix + "backlashSpeed", 3);


            axes[i].enablePitch = (ConfigUtil::ReadParam(filePath, prefix + "EnablePitch", 0.0) == 1.0);
            axes[i].pitchStartPos_mm = ConfigUtil::ReadParam(filePath, prefix + "PitchStartPos", 0.0);
            axes[i].pitchStep_mm = ConfigUtil::ReadParam(filePath, prefix + "PitchStep", 10.0);
            axes[i].pitchSpeed_mm_s = ConfigUtil::ReadParam(filePath, prefix + "PitchSpeed_mm_s", 3);
           
           

            motion.m_CompEngine.InitAxisCompensation(i, axes[i].enableBacklash, axes[i].backlashAmount_Pos_mm, axes[i].backlashAmount_Neg_mm,  axes[i].backlashSpeed, axes[i].enablePitch, axes[i].pitchStartPos_mm,axes[i].pitchStep_mm, axes[i].pitchSpeed_mm_s);





        
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
            axes[i].Pid_G00.Kp = ConfigUtil::ReadParam(filePath, prefix + "Kp_G00", 20.0);
            axes[i].Pid_G00.Ki = ConfigUtil::ReadParam(filePath, prefix + "Ki_G00", 10.0);
            axes[i].Pid_G00.Kd = ConfigUtil::ReadParam(filePath, prefix + "Kd_G00", 0.0);
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

            double Max_speed = ConfigUtil::ReadParam(filePath, prefix + "MAX_Speed", 0);
            axes[i].maxVel_PPS = MotionCore::UnitPerMinToPps(Max_speed, axes[i].resolution_PPR, axes[i].finalLead);

            double g00_speed_user = ConfigUtil::ReadParam(filePath, prefix + "G00_Speed", 0);
            axes[i].G00_PPS = MotionCore::UnitPerMinToPps(g00_speed_user, axes[i].resolution_PPR, axes[i].finalLead);

            double G00_acc_time= ConfigUtil::ReadParam(filePath, prefix + "G00_acc_time", 0);
            axes[i].G00_acc_time = G00_acc_time;
           
            double G00_dec_time = ConfigUtil::ReadParam(filePath, prefix + "G00_dec_time", 0);
            axes[i].G00_dec_time = G00_acc_time;


            double g07_speed_user = ConfigUtil::ReadParam(filePath, prefix + "G07_Speed", 0);
            axes[i].G07_PPS = MotionCore::UnitPerMinToPps(g00_speed_user, axes[i].resolution_PPR, axes[i].finalLead);

            double G07_acc_time = ConfigUtil::ReadParam(filePath, prefix + "G07_acc_time", 0);
            axes[i].G07_acc_time = G07_acc_time;

            double G07_dec_time = ConfigUtil::ReadParam(filePath, prefix + "G07_dec_time", 0);
            axes[i].G07_dec_time = G07_acc_time;

            double g161_speed_user = ConfigUtil::ReadParam(filePath, prefix + "G161_Speed", 0);
            axes[i].G161_PPS = MotionCore::UnitPerMinToPps(g161_speed_user, axes[i].resolution_PPR, axes[i].finalLead);

            double G161_acc_time = ConfigUtil::ReadParam(filePath, prefix + "G161_acc_time", 0);
            axes[i].G161_acc_time = G161_acc_time;

            double G161_dec_time = ConfigUtil::ReadParam(filePath, prefix + "G161_dec_time", 0);
            axes[i].G161_dec_time = G161_acc_time;


            double Stop_dec_time = ConfigUtil::ReadParam(filePath, prefix + "Stop_dec_time", 0);
            axes[i].Stop_dec_time = Stop_dec_time;




            double g28_speed_user = ConfigUtil::ReadParam(filePath, prefix + "G28_Speed", 500.0);
            axes[i].G28_PPS = MotionCore::UnitPerMinToPps(g28_speed_user, axes[i].resolution_PPR, axes[i].finalLead);

            double G28_acc_time = ConfigUtil::ReadParam(filePath, prefix + "G28_acc_time", 0);
            axes[i].G28_acc_time = G28_acc_time;

            double G28_dec_time = ConfigUtil::ReadParam(filePath, prefix + "G28_dec_time", 0);
            axes[i].G28_dec_time = G28_acc_time;



            double g30_speed_user = ConfigUtil::ReadParam(filePath, prefix + "G30_Speed", 0);
            axes[i].G30_PPS = MotionCore::UnitPerMinToPps(g30_speed_user, axes[i].resolution_PPR, axes[i].finalLead);

            double G30_acc_time = ConfigUtil::ReadParam(filePath, prefix + "G30_acc_time", 0);
            axes[i].G30_acc_time = G30_acc_time;

            double G30_dec_time = ConfigUtil::ReadParam(filePath, prefix + "G30_dec_time", 0);
            axes[i].G30_dec_time = G30_acc_time;


            double g32_speed_user = ConfigUtil::ReadParam(filePath, prefix + "G32_Speed", 0);
            axes[i].G32_PPS = MotionCore::UnitPerMinToPps(g32_speed_user, axes[i].resolution_PPR, axes[i].finalLead);

            double G32_acc_time = ConfigUtil::ReadParam(filePath, prefix + "G32_acc_time", 0);
            axes[i].G32_acc_time = G32_acc_time;

            double G32_dec_time = ConfigUtil::ReadParam(filePath, prefix + "G32_dec_time", 0);
            axes[i].G32_dec_time = G32_acc_time;



            double g53_speed_user = ConfigUtil::ReadParam(filePath, prefix + "G53_Speed", 0);
            axes[i].G53_PPS = MotionCore::UnitPerMinToPps(g53_speed_user, axes[i].resolution_PPR, axes[i].finalLead);

            double G53_acc_time = ConfigUtil::ReadParam(filePath, prefix + "G53_acc_time", 0);
            axes[i].G53_acc_time = G53_acc_time;

            double G53_dec_time = ConfigUtil::ReadParam(filePath, prefix + "G53_dec_time", 0);
            axes[i].G53_dec_time = G53_acc_time;
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
// 🌟 增加一個 bool isPositive 參數
bool GlobalConfig::LoadPitchTable(const std::string& filePath, CompensationEngine& compEngine, bool isPositive)
{
    std::ifstream in(filePath);
    if (!in.is_open()) {
        DEBUG_PRINT("[WARN] File not found: %s\n", filePath.c_str());
        return false;
    }

    std::string line;
    std::vector<double> pitchData[8]; // 暫存 8 軸的資料

    while (std::getline(in, line)) {
        if (line.empty() || line[0] == ';' || line[0] == '/' || line[0] == '#') continue;

        std::stringstream ss(line);
        double val;
        for (int i = 0; i < 8; ++i) {
            if (ss >> val) pitchData[i].push_back(val);
            else pitchData[i].push_back(0.0);
        }
    }
    in.close();

    // 🌟 根據 isPositive 決定呼叫哪一個 Setter
    for (int i = 0; i < 8; ++i) {
        if (!pitchData[i].empty()) {
            if (isPositive) compEngine.SetPitchTablePos(i, pitchData[i]);
            else compEngine.SetPitchTableNeg(i, pitchData[i]);
        }
    }

    return true;
}

