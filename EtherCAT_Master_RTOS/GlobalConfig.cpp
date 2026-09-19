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
#include "EtherCatMaster.h" // 🌟 必須在這裡引入，才能操作 master 的成員
#include "SHMManager.h" 
#include "NCManager.h" 
#include "HomePersistenceManager.h"
#include "NCElectrodeRotationConfig.h"
#include "AlarmManager.h"
#include <cmath>

bool GlobalConfig::LoadAxisConfig(const std::string& filePath, std::vector<AxisContext>& axis, MotionCore& motion)
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
        axis.resize(axisCount);//調整 m_Axes 陣列的大小


        for (int i = 0; i < axisCount; i++)//開始填入軸參數
        {
            std::string prefix = std::to_string(i) + "_";




            // ----------------------------------------------------
            // 1. 基礎初始化
            // ----------------------------------------------------
            double res = ConfigUtil::ReadParam(filePath, prefix + "Resolution", 16777216.0);
            motion.InitAxis(axis[i], res);


            axis[i].axisIndex = (int)ConfigUtil::ReadParam(filePath, prefix + "axisIndex", i);


            axis[i].isExist = (ConfigUtil::ReadParam(filePath, prefix + "isExist", 1.0) == 1.0);

            // ----------------------------------------------------
            // 2. 🌟 [新增] 機械機構參數 (由參數檔讀取)
            // ----------------------------------------------------
            axis[i].reduction_MotorSide = ConfigUtil::ReadParam(filePath, prefix + "Reduction_MotorSide", 1.0);
            axis[i].reduction_LoadSide = ConfigUtil::ReadParam(filePath, prefix + "Reduction_LoadSide", 1.0);
            axis[i].mechanicalPitch = ConfigUtil::ReadParam(filePath, prefix + "MechanicalPitch", 10.0);
            axis[i].isReverse = (ConfigUtil::ReadParam(filePath, prefix + "isReverse", 0.0) == 1.0);
            axis[i].Axis_Reverse = (ConfigUtil::ReadParam(filePath, prefix + "Axis_Reverse", 0.0) == 1.0);



            // 🌟 自動計算最終導程 (Final Lead)
            // 確保 LoadSide 不為 0 以防除以零錯誤
            if (axis[i].reduction_LoadSide > 0.0001) {
                axis[i].finalLead = (axis[i].mechanicalPitch * axis[i].reduction_MotorSide) / axis[i].reduction_LoadSide;
            }
            else {
                axis[i].finalLead = axis[i].mechanicalPitch; // 防呆
            }

            double pulsePerUnit = axis[i].resolution_PPR / axis[i].finalLead;

            // ----------------------------------------------------
            // 3. 讀取物理與運動參數
            // ----------------------------------------------------


            double accTime = ConfigUtil::ReadParam(filePath, prefix + "Acc_Time", 1);
            double decTime = ConfigUtil::ReadParam(filePath, prefix + "Dec_Time", 1);

            axis[i].acc_PPS2 = (accTime > 0.0) ? (axis[i].maxVel_PPS / accTime) : (axis[i].maxVel_PPS * 2.0);
            axis[i].dec_PPS2 = (decTime > 0.0) ? (axis[i].maxVel_PPS / decTime) : (axis[i].maxVel_PPS * 2.0);

            double smoothTime = ConfigUtil::ReadParam(filePath, prefix + "SmoothTime", 100.0);
            motion.InitSmoothBuffer(axis[i], smoothTime);
            motion.InitVirtualAxisSmooth(smoothTime);

            // ----------------------------------------------------
            // 4. 雙閉環與 PID 參數
            // ----------------------------------------------------
            int fbVal = (int)ConfigUtil::ReadParam(filePath, prefix + "FbMode", 0.0);
            axis[i].fbMode = (fbVal == 1) ? FeedbackSource::LINEAR_SCALE : FeedbackSource::MOTOR_ENCODER;

            axis[i].scaleToMotorRatio = ConfigUtil::ReadParam(filePath, prefix + "ScaleRatio", 1.0);
            axis[i].maxDeviation = ConfigUtil::ReadParam(filePath, prefix + "MaxDev", 1677721.6);


            axis[i].pid.EnableLagCheck = (ConfigUtil::ReadParam(filePath, prefix + "EnableLagCheck", 1.0) == 1.0);
            double maxLag_mm = ConfigUtil::ReadParam(filePath, prefix + "maxLag_mm", 2.0);
            axis[i].maxLag_mm = maxLag_mm;
            axis[i].pid.MaxLag = axis[i].maxLag_mm * pulsePerUnit;


            //到位視窗判定
            double  inPositionWindow_mm = ConfigUtil::ReadParam(filePath, prefix + "inPositionWindow_mm", 0.005);
            axis[i].inPositionWindow_mm = inPositionWindow_mm;
            axis[i].inPositionWindow_Pulse = axis[i].inPositionWindow_mm * pulsePerUnit;


            // ----------------------------------------------------
            // 5. 軸型態與補償參數
            // ----------------------------------------------------
            int typeVal = (int)ConfigUtil::ReadParam(filePath, prefix + "AxisType", 0.0);
            axis[i].axisType = (typeVal == 1) ? AxisType::ROTARY : (typeVal == 2 ? AxisType::ROTARY_CONTINUOUS : AxisType::LINEAR);
            axis[i].rotaryModulo = ConfigUtil::ReadParam(filePath, prefix + "RotaryModulo", 360.0);
            axis[i].useShortestPath = (ConfigUtil::ReadParam(filePath, prefix + "ShortestPath", 0.0) == 1.0);

            axis[i].enableBacklash = (ConfigUtil::ReadParam(filePath, prefix + "EnableBacklash", 0.0) == 1.0);
            axis[i].backlashAmount_Pos_mm = ConfigUtil::ReadParam(filePath, prefix + "backlashAmount_Pos_mm", 0.0);
            axis[i].backlashAmount_Neg_mm = ConfigUtil::ReadParam(filePath, prefix + "backlashAmount_Neg_mm", 0.0);
            axis[i].backlashSpeed = ConfigUtil::ReadParam(filePath, prefix + "backlashSpeed", 3);


            axis[i].enablePitch = (ConfigUtil::ReadParam(filePath, prefix + "EnablePitch", 0.0) == 1.0);
            axis[i].pitchStartPos_mm = ConfigUtil::ReadParam(filePath, prefix + "PitchStartPos", 0.0);
            axis[i].pitchStep_mm = ConfigUtil::ReadParam(filePath, prefix + "PitchStep", 10.0);
            axis[i].pitchSpeed_mm_s = ConfigUtil::ReadParam(filePath, prefix + "PitchSpeed_mm_s", 3);



            motion.m_CompEngine.InitAxisCompensation(i, axis[i].enableBacklash, axis[i].backlashAmount_Pos_mm, axis[i].backlashAmount_Neg_mm, axis[i].backlashSpeed, axis[i].enablePitch, axis[i].pitchStartPos_mm, axis[i].pitchStep_mm, axis[i].pitchSpeed_mm_s);



            //極限設定-------------------------------------------------------------

            // 第 1 組軟體行程 G22 / G23 -------------------------------------------------------------

            axis[i].travelLimit1Enable = (ConfigUtil::ReadParam(filePath, prefix + "TravelLimit1Enable", 0.0) == 1.0);
            axis[i].travelLimit1Positive_unit = ConfigUtil::ReadParam(filePath, prefix + "TravelLimit1Positive", 0.0);
            axis[i].travelLimit1Negative_unit = ConfigUtil::ReadParam(filePath, prefix + "TravelLimit1Negative", 0.0);

            // Unit -> Pulse
            axis[i].travelLimit1Positive_Pulse = axis[i].travelLimit1Positive_unit * pulsePerUnit;
            axis[i].travelLimit1Negative_Pulse = axis[i].travelLimit1Negative_unit * pulsePerUnit;


            // 第 2 組軟體行程 系統參數決定是否開啟-------------------------------------------------------------
            axis[i].travelLimit2Enable = (ConfigUtil::ReadParam(filePath, prefix + "TravelLimit2Enable", 0.0) == 1.0);
            axis[i].travelLimit2Positive_unit = ConfigUtil::ReadParam(filePath, prefix + "TravelLimit2Positive", 0.0);
            axis[i].travelLimit2Negative_unit = ConfigUtil::ReadParam(filePath, prefix + "TravelLimit2Negative", 0.0);

            // Unit -> Pulse
            axis[i].travelLimit2Positive_Pulse = axis[i].travelLimit2Positive_unit * pulsePerUnit;
            axis[i].travelLimit2Negative_Pulse = axis[i].travelLimit2Negative_unit * pulsePerUnit;

            // 第 3 組軟體行程 系統參數決定是否開啟-------------------------------------------------------------

            axis[i].travelLimit3Enable = (ConfigUtil::ReadParam(filePath, prefix + "TravelLimit3Enable", 0.0) == 1.0);
            axis[i].travelLimit3Positive_unit = ConfigUtil::ReadParam(filePath, prefix + "TravelLimit3Positive", 0.0);
            axis[i].travelLimit3Negative_unit = ConfigUtil::ReadParam(filePath, prefix + "TravelLimit3Negative", 0.0);

            // Unit -> Pulse
            axis[i].travelLimit3Positive_Pulse = axis[i].travelLimit3Positive_unit * pulsePerUnit;
            axis[i].travelLimit3Negative_Pulse = axis[i].travelLimit3Negative_unit * pulsePerUnit;






        }
    }


    return true;
}


bool GlobalConfig::LoadPIDConfig(const std::string& filePath, std::vector<AxisContext>& axis, MotionCore& motion)
{
    //讀取參數確定軸數量-----------------------------------------------------------------
    int axisCount = System_axisCount;
    DEBUG_PRINT("LoadPIDConfig Axis Count>>%d\n", axisCount);
    if (axisCount == 0)
    {

    }
    else
    {
        axis.resize(axisCount);//調整 m_Axes 陣列的大小


        for (int i = 0; i < axisCount; i++)//開始填入軸參數
        {
            std::string prefix = std::to_string(i) + "_";

            axis[i].pid.Kp = ConfigUtil::ReadParam(filePath, prefix + "Kp_IDLE", 20.0);
            axis[i].pid.Ki = ConfigUtil::ReadParam(filePath, prefix + "Ki_IDLE", 10.0);
            axis[i].pid.Kd = ConfigUtil::ReadParam(filePath, prefix + "Kd_IDLE", 0.0);
            axis[i].pid.Kvff = ConfigUtil::ReadParam(filePath, prefix + "Kvff_IDLE", 1.0);

            //閒置靜止時PID---------------------------
            axis[i].Pid_IDLE.Kp = ConfigUtil::ReadParam(filePath, prefix + "Kp_IDLE", 20.0);
            //axis[i].Pid_IDLE.Kp = 200;//測試用
            axis[i].Pid_IDLE.Ki = ConfigUtil::ReadParam(filePath, prefix + "Ki_IDLE", 10.0);
            axis[i].Pid_IDLE.Kd = ConfigUtil::ReadParam(filePath, prefix + "Kd_IDLE", 0.0);
            axis[i].Pid_IDLE.Kvff = ConfigUtil::ReadParam(filePath, prefix + "Kvff_IDLE", 0.0);
            //G00時PID---------------------------
            axis[i].Pid_G00.Kp = ConfigUtil::ReadParam(filePath, prefix + "Kp_G00", 20.0);
            //axis[i].Pid_G00.Kp = 20;//測試用
            axis[i].Pid_G00.Ki = ConfigUtil::ReadParam(filePath, prefix + "Ki_G00", 10.0);
            //axis[i].Pid_G00.Ki = 10;
            axis[i].Pid_G00.Kd = ConfigUtil::ReadParam(filePath, prefix + "Kd_G00", 0.0);
            axis[i].Pid_G00.Kvff = ConfigUtil::ReadParam(filePath, prefix + "Kvff_G00", 1.0);
        }
    }


    return true;
}


bool GlobalConfig::LoadSpeedConfig(const std::string& filePath, std::vector<AxisContext>& axis, MotionCore& motion)
{
    //讀取參數確定軸數量-----------------------------------------------------------------
    int axisCount = System_axisCount;
    DEBUG_PRINT("LoadPIDConfig Axis Count>>%d\n", axisCount);
    if (axisCount == 0)
    {

    }
    else
    {
        axis.resize(axisCount);//調整 m_Axes 陣列的大小


        for (int i = 0; i < axisCount; i++)//開始填入軸參數
        {
            std::string prefix = std::to_string(i) + "_";

            double Max_speed = ConfigUtil::ReadParam(filePath, prefix + "MAX_Speed", 0);
            axis[i].maxVel_PPS = MotionCore::UnitPerMinToPps(Max_speed, axis[i].resolution_PPR, axis[i].finalLead);



            double Stop_dec_time = ConfigUtil::ReadParam(filePath, prefix + "Stop_dec_time", 0);
            axis[i].Stop_dec_time = Stop_dec_time;


            double Jog_speed_user = ConfigUtil::ReadParam(filePath, prefix + "JOG_MAX_PPS", 0);
            axis[i].JOG_MAX_PPS = MotionCore::UnitPerMinToPps(Jog_speed_user, axis[i].resolution_PPR, axis[i].finalLead);

            double Jog_acc_time = ConfigUtil::ReadParam(filePath, prefix + "JOG_acc_time", 0);
            axis[i].JOG_acc_time = Jog_acc_time;

            double Jog_dec_time = ConfigUtil::ReadParam(filePath, prefix + "JOG_dec_time", 0);
            axis[i].JOG_dec_time = Jog_acc_time;


            double FINE_JOG_0001_PPS_user = ConfigUtil::ReadParam(filePath, prefix + "FINE_JOG_0001_PPS", 0);
            axis[i].FINE_JOG_0001_PPS = MotionCore::UnitPerMinToPps(FINE_JOG_0001_PPS_user, axis[i].resolution_PPR, axis[i].finalLead);

            double FINE_JOG_0010_PPS_user = ConfigUtil::ReadParam(filePath, prefix + "FINE_JOG_0010_PPS", 0);
            axis[i].FINE_JOG_0010_PPS = MotionCore::UnitPerMinToPps(FINE_JOG_0010_PPS_user, axis[i].resolution_PPR, axis[i].finalLead);
            double FINE_JOG_0100_PPS_user = ConfigUtil::ReadParam(filePath, prefix + "FINE_JOG_0100_PPS", 0);
            axis[i].FINE_JOG_0100_PPS = MotionCore::UnitPerMinToPps(FINE_JOG_0100_PPS_user, axis[i].resolution_PPR, axis[i].finalLead);
            double FINE_JOG_1000_PPS_user = ConfigUtil::ReadParam(filePath, prefix + "FINE_JOG_1000_PPS", 0);
            axis[i].FINE_JOG_1000_PPS = MotionCore::UnitPerMinToPps(FINE_JOG_1000_PPS_user, axis[i].resolution_PPR, axis[i].finalLead);


            double MPG_BASE_DISTANCE = ConfigUtil::ReadParam(filePath, prefix + "MPG_BASE_DISTANCE", 0);
            axis[i].MPG_BASE_DISTANCE = MPG_BASE_DISTANCE;


            double MPG_MAX_PPS = ConfigUtil::ReadParam(filePath, prefix + "MPG_MAX_PPS", 0);
            axis[i].MPG_MAX_PPS = MotionCore::UnitPerMinToPps(MPG_MAX_PPS, axis[i].resolution_PPR, axis[i].finalLead);




            double INCH_0001_DISTANCE_user = ConfigUtil::ReadParam(filePath, prefix + "INCH_0001_DISTANCE", 0);
            axis[i].INCH_0001_DISTANCE = INCH_0001_DISTANCE_user;

            double INCH_0010_DISTANCE_user = ConfigUtil::ReadParam(filePath, prefix + "INCH_0010_DISTANCE", 0);
            axis[i].INCH_0010_DISTANCE = INCH_0010_DISTANCE_user;

            double INCH_0100_DISTANCE_user = ConfigUtil::ReadParam(filePath, prefix + "INCH_0100_DISTANCE", 0);
            axis[i].INCH_0100_DISTANCE = INCH_0100_DISTANCE_user;

            double INCH_1000_DISTANCE_user = ConfigUtil::ReadParam(filePath, prefix + "INCH_1000_DISTANCE", 0);
            axis[i].INCH_1000_DISTANCE = INCH_1000_DISTANCE_user;


            double INCH_JOG_PPS_user = ConfigUtil::ReadParam(filePath, prefix + "INCH_JOG_PPS", 0);
            axis[i].INCH_JOG_PPS = MotionCore::UnitPerMinToPps(INCH_JOG_PPS_user, axis[i].resolution_PPR, axis[i].finalLead);

            double INCH_acc_time = ConfigUtil::ReadParam(filePath, prefix + "INCH_acc_time", 0);
            axis[i].INCH_acc_time = INCH_acc_time;

            double INCH_dec_time = ConfigUtil::ReadParam(filePath, prefix + "INCH_dec_time", 0);
            axis[i].INCH_dec_time = INCH_dec_time;


            double g00_speed_user = ConfigUtil::ReadParam(filePath, prefix + "G00_Speed", 0);
            axis[i].G00_PPS = MotionCore::UnitPerMinToPps(g00_speed_user, axis[i].resolution_PPR, axis[i].finalLead);

            double G00_acc_time = ConfigUtil::ReadParam(filePath, prefix + "G00_acc_time", 0);
            axis[i].G00_acc_time = G00_acc_time;

            double G00_dec_time = ConfigUtil::ReadParam(filePath, prefix + "G00_dec_time", 0);
            axis[i].G00_dec_time = G00_acc_time;


            double g07_speed_user = ConfigUtil::ReadParam(filePath, prefix + "G07_Speed", 0);
            axis[i].G07_PPS = MotionCore::UnitPerMinToPps(g00_speed_user, axis[i].resolution_PPR, axis[i].finalLead);

            double G07_acc_time = ConfigUtil::ReadParam(filePath, prefix + "G07_acc_time", 0);
            axis[i].G07_acc_time = G07_acc_time;

            double G07_dec_time = ConfigUtil::ReadParam(filePath, prefix + "G07_dec_time", 0);
            axis[i].G07_dec_time = G07_acc_time;

            double g161_speed_user = ConfigUtil::ReadParam(filePath, prefix + "G161_Speed", 0);
            axis[i].G161_PPS = MotionCore::UnitPerMinToPps(g161_speed_user, axis[i].resolution_PPR, axis[i].finalLead);

            double G161_acc_time = ConfigUtil::ReadParam(filePath, prefix + "G161_acc_time", 0);
            axis[i].G161_acc_time = G161_acc_time;

            double G161_dec_time = ConfigUtil::ReadParam(filePath, prefix + "G161_dec_time", 0);
            axis[i].G161_dec_time = G161_acc_time;






            double g28_speed_user = ConfigUtil::ReadParam(filePath, prefix + "G28_Speed", 500.0);
            axis[i].G28_PPS = MotionCore::UnitPerMinToPps(g28_speed_user, axis[i].resolution_PPR, axis[i].finalLead);

            double G28_acc_time = ConfigUtil::ReadParam(filePath, prefix + "G28_acc_time", 0);
            axis[i].G28_acc_time = G28_acc_time;

            double G28_dec_time = ConfigUtil::ReadParam(filePath, prefix + "G28_dec_time", 0);
            axis[i].G28_dec_time = G28_acc_time;



            double g30_speed_user = ConfigUtil::ReadParam(filePath, prefix + "G30_Speed", 0);
            axis[i].G30_PPS = MotionCore::UnitPerMinToPps(g30_speed_user, axis[i].resolution_PPR, axis[i].finalLead);

            double G30_acc_time = ConfigUtil::ReadParam(filePath, prefix + "G30_acc_time", 0);
            axis[i].G30_acc_time = G30_acc_time;

            double G30_dec_time = ConfigUtil::ReadParam(filePath, prefix + "G30_dec_time", 0);
            axis[i].G30_dec_time = G30_acc_time;


            double g32_speed_user = ConfigUtil::ReadParam(filePath, prefix + "G32_Speed", 0);
            axis[i].G32_PPS = MotionCore::UnitPerMinToPps(g32_speed_user, axis[i].resolution_PPR, axis[i].finalLead);

            double G32_acc_time = ConfigUtil::ReadParam(filePath, prefix + "G32_acc_time", 0);
            axis[i].G32_acc_time = G32_acc_time;

            double G32_dec_time = ConfigUtil::ReadParam(filePath, prefix + "G32_dec_time", 0);
            axis[i].G32_dec_time = G32_acc_time;



            double g53_speed_user = ConfigUtil::ReadParam(filePath, prefix + "G53_Speed", 0);
            axis[i].G53_PPS = MotionCore::UnitPerMinToPps(g53_speed_user, axis[i].resolution_PPR, axis[i].finalLead);

            double G53_acc_time = ConfigUtil::ReadParam(filePath, prefix + "G53_acc_time", 0);
            axis[i].G53_acc_time = G53_acc_time;

            double G53_dec_time = ConfigUtil::ReadParam(filePath, prefix + "G53_dec_time", 0);
            axis[i].G53_dec_time = G53_acc_time;





        }
    }


    return true;
}


bool GlobalConfig::LoadNCConfig(const std::string& filePath, std::vector<AxisContext>& axis, MotionCore& motion, NCManager* nc)
{
    // Legacy absent-file defaults remain safe: no electrode axis is inferred.
    // An opened file must be read and validated completely before any write.
    int electrodeAxis = 0;
    std::ifstream config(filePath);
    NCElectrodeRotationConfig::ParseError error = NCElectrodeRotationConfig::ParseError::None;
    if (config.is_open() && !NCElectrodeRotationConfig::Read(config, electrodeAxis, error))
    {
        RtPrintf("[NC-CONFIG][REJECT] key=ElectrodeRotationAxis reason=%s\n",
            NCElectrodeRotationConfig::ErrorText(error));
        if (nc != nullptr)
            AlarmManager::GetInstance().Trigger(AlarmManager::G_Code_Invalid_parameter);
        return false;
    }
    if (motion.m_pCoordMgr == nullptr)
    {
        RtPrintf("[NC-CONFIG][REJECT] key=ElectrodeRotationAxis reason=COORDINATE_MANAGER_MISSING\n");
        if (nc != nullptr)
            AlarmManager::GetInstance().Trigger(AlarmManager::G_Code_Invalid_parameter);
        return false;
    }
    if (!motion.m_pCoordMgr->ConfigureElectrodeRotationAxis(electrodeAxis, axis, nc))
        return false; // The coordinate mutation guard supplies the rejection.
    RtPrintf("[NC-CONFIG][ELECTRODE-ROLE] axis=%d enabled=%d\n", electrodeAxis,
        motion.m_pCoordMgr->isCAxisOffsetRotationEnabled ? 1 : 0);

    //第一軟體極限保護G22 G23 啟動時預設 0為G23 1為G22
    bool programmableTravelLimitEnabled = (ConfigUtil::ReadParam(filePath, "ProgrammableTravelLimitEnabled", 0.0) == 1.0);
    motion.m_pCoordMgr->SetProgrammableTravelLimitEnabled(programmableTravelLimitEnabled, nc);





    return true;
}

bool GlobalConfig::LoadHomeConfig(const std::string& filePath, std::vector<AxisContext>& axis, MotionCore& motion, NCManager* nc)
{


    //讀取參數確定軸數量-----------------------------------------------------------------
    int axisCount = System_axisCount;
    DEBUG_PRINT("LoadHomeConfig Axis Count>>%d\n", axisCount);
    if (axisCount == 0)
    {

    }
    else
    {
        axis.resize(axisCount);//調整 m_Axes 陣列的大小


        for (int i = 0; i < axisCount; i++)//開始填入軸參數
        {
            std::string prefix = std::to_string(i) + "_";


            //尋原點---------------------------------------------------------------
            axis[i].home.enabled = (ConfigUtil::ReadParam(filePath, prefix + "HomeEnable", 0.0) == 1.0);//該軸是否允許執行尋原點 0不啟用 1啟用


            // 尋找原點模式 ---------------------------------------------------------
            //
            // HomeMethod：
            // 決定此軸使用哪一種方式建立機械原點。
            //
            // 注意：
            // INDEX 訊號來源另外由 HomeReferenceSource 設定，
            // 可以選擇馬達編碼器、光學尺、外部 IO 或絕對式位置。
            //
            // 0 = DOG_INDEX
            //     先依照設定方向尋找 HOME DOG。
            //     DOG 觸發後減速停止，再反向退出 DOG，
            //     最後以低速尋找 INDEX，並使用 INDEX 位置建立機械原點。
            //
            // 1 = LIMIT_INDEX
            //     以正向或負向硬體極限開關作為第一階段尋找訊號。
            //     碰到預期方向的硬體極限後減速停止並反向退出，
            //     最後以低速尋找 INDEX，並使用 INDEX 位置建立機械原點。
            //
            // 2 = DOG_ONLY
            //     只尋找 HOME DOG，不再尋找 INDEX。
            //     DOG 觸發、減速停止並完成反向退出後，
            //     直接依照 DOG 位置與 HomeOffset 建立機械原點。
            //
            // 3 = LIMIT_ONLY
            //     只尋找指定方向的硬體極限，不再尋找 INDEX。
            //     碰到極限、減速停止並完成反向退出後，
            //     直接依照極限位置與 HomeOffset 建立機械原點。
            //
            // 4 = INDEX_ONLY
            //     不尋找 HOME DOG，也不尋找硬體極限。
            //     直接依照設定方向與低速尋找 INDEX，
            //     捕捉到 INDEX 後減速停止並建立機械原點。
            //
            // 5 = ABSOLUTE_REFERENCE
            //     使用絕對式馬達編碼器或絕對式光學尺的位置，
            //     不需要執行 DOG、極限或 INDEX 搜尋動作。
            //     讀取有效的絕對位置後直接建立機械原點。
            //
            // 6 = CURRENT_POSITION
            //     將目前所在位置直接設定為機械原點。
            //     主要用於安裝、校機、測試或特殊設備，
            //     正式機台使用時必須由權限與參數保護。
            //
            // 7 = MECHANICAL_STOP
            //     預留的機械端點尋原點方式。
            //     未來可依馬達扭矩、追隨誤差或堵轉狀態，
            //     判斷軸已接觸機械止擋並建立機械原點。
            //     第一版暫不實作。
            //
            // ---------------------------------------------------------

            int homeMethod = static_cast<int>(ConfigUtil::ReadParam(filePath, prefix + "HomeMethod", 0.0));

            if (homeMethod < 0 || homeMethod > 7)
            {
                homeMethod = 0;
            }

            axis[i].home.method = static_cast<HomeMethod>(homeMethod);

            //原點訊號來源模式 ---------------------------------------------------------
            // ---------------------------------------------------------
            // HOME Reference Source
            //
            // 0 = NONE
            // 1 = MOTOR_ENCODER_INDEX 馬達編碼器索引訊號
            // 2 = LINEAR_SCALE_INDEX_DRIVE 線性標尺索引驅動
            // 3 = EXTERNAL_IO_INDEX 外部 I/O 索引
            // 4 = ABSOLUTE_MOTOR_ENCODER 絕對式馬達編碼器
            // 5 = ABSOLUTE_LINEAR_SCALE 絕對線性比例
            // ---------------------------------------------------------

            int homeReferenceSource = static_cast<int>(ConfigUtil::ReadParam(filePath, prefix + "HomeReferenceSource", 1.0));

            if (homeReferenceSource < 0 || homeReferenceSource > 5)
            {
                homeReferenceSource = 1;
            }

            axis[i].home.referenceSource = static_cast<HomeReferenceSource>(homeReferenceSource);



            // 捕捉HOME 點方式---------------------------------------------------------
            // ---------------------------------------------------------
            // HOME Reference Capture Mode
            //
            // 0 = DRIVE_HARDWARE_LATCH>>驅動硬體鎖存
            // 1 = EXTERNAL_HARDWARE_LATCH>>外部硬體鎖存器
            // 2 = SOFTWARE_SAMPLE >>軟體
            // ---------------------------------------------------------

            int homeCaptureMode = static_cast<int>(ConfigUtil::ReadParam(filePath, prefix + "HomeReferenceCaptureMode", 0.0));

            if (homeCaptureMode < 0 || homeCaptureMode > 2)
            {
                homeCaptureMode = 0;
            }

            axis[i].home.captureMode = static_cast<HomeReferenceCaptureMode>(homeCaptureMode);

            // 尋HOME方向---------------------------------------------------------
            // ---------------------------------------------------------
            // HOME Direction
            //
            // -1 = Machine Negative Direction
            // +1 = Machine Positive Direction
            //
            // 其他數值一律視為 -1。
            // ---------------------------------------------------------

            int homeDirection = static_cast<int>(ConfigUtil::ReadParam(filePath, prefix + "HomeDirection", -1.0));
            axis[i].home.direction = (homeDirection == 1) ? 1 : -1;

            // 尋HOME順序---------------------------------------------------------
            // ---------------------------------------------------------
            // HOME Order
            //
            // G81 P1 時使用。
            //
            // Order 小的群組先執行；
            // 相同 Order 的軸同時執行。
            // ---------------------------------------------------------

            int homeOrder = static_cast<int>(ConfigUtil::ReadParam(filePath, prefix + "HomeOrder", 0.0));

            if (homeOrder < 0)
            {
                homeOrder = 0;
            }
            axis[i].home.order = homeOrder;

            // DOG  訊號極性---------------------------------------------------------
            // ---------------------------------------------------------
            // DOG Input Polarity
            //
            // 0 = Active Low
            // 1 = Active High
            // ---------------------------------------------------------

            axis[i].home.dogActiveHigh = (ConfigUtil::ReadParam(filePath, prefix + "HomeDogActiveHigh", 1.0) == 1.0);

            // INDEX  訊號極性---------------------------------------------------------
            // ---------------------------------------------------------
            // Reference / INDEX Input Polarity
            //
            // 0 = Active Low
            // 1 = Active High
            // ---------------------------------------------------------

            axis[i].home.referenceActiveHigh = (ConfigUtil::ReadParam(filePath, prefix + "HomeReferenceActiveHigh", 1.0) == 1.0);


            //外部 IO 參考 C 點---------------------------------------------------------
            // ---------------------------------------------------------
            // External IO Reference C Point
            //
            // referenceSource == EXTERNAL_IO_INDEX 時使用。
            //
            // -1：
            //     使用 NCPLC::C::HOME_INDEX_BASE + AxisIndex
            //
            // >= 0：
            //     使用指定的 PLC C Point。
            // ---------------------------------------------------------

            int externalReferenceCPoint = static_cast<int>(ConfigUtil::ReadParam(filePath, prefix + "HomeExternalReferenceCPoint", -1.0));

            if (externalReferenceCPoint < -1)
            {
                externalReferenceCPoint = -1;
            }

            axis[i].home.externalReferenceCPoint = externalReferenceCPoint;

            // =====================================================
            // Drive Touch Probe / Motor Encoder INDEX
            //
            // 使用範圍：
            //
            // HomeReferenceSource = MOTOR_ENCODER_INDEX
            // HomeReferenceCaptureMode = DRIVE_HARDWARE_LATCH
            //
            // 台達 A3-E 目前實測：
            //
            // 60B8 = 0x0015
            // 60B9 = 0x0041
            // 60BA = Motor Z Hardware Capture Position
            //
            // ConfigUtil::ReadParam() 使用十進位：
            //
            // 0x0015 = 21
            // 0x0001 = 1
            // 0x0002 = 2
            // 0x0040 = 64
            // =====================================================

            // -----------------------------------------------------
            // 16-bit HOME Probe 參數讀取防呆
            // -----------------------------------------------------

            auto ReadUInt16HomeParam =
                [&](const std::string& key,
                    uint16_t defaultValue) -> uint16_t
            {
                double rawValue =
                    ConfigUtil::ReadParam(
                        filePath,
                        key,
                        static_cast<double>(defaultValue));

                if (!std::isfinite(rawValue))
                {
                    return defaultValue;
                }

                if (rawValue < 0.0)
                {
                    rawValue = 0.0;
                }

                if (rawValue > 65535.0)
                {
                    rawValue = 65535.0;
                }

                return static_cast<uint16_t>(
                    rawValue);
            };


            // -----------------------------------------------------
            // Probe Arm Mode
            //
            // 0 = CONTROLLER_60B8
            // 1 = DRIVE_AUTO_ARM
            // -----------------------------------------------------

            int driveProbeArmMode =
                static_cast<int>(
                    ConfigUtil::ReadParam(
                        filePath,
                        prefix + "HomeDriveProbeArmMode",
                        0.0));

            if (driveProbeArmMode < 0 ||
                driveProbeArmMode > 1)
            {
                driveProbeArmMode = 0;
            }

            axis[i].home.driveProbeArmMode =
                static_cast<HomeDriveProbeArmMode>(
                    driveProbeArmMode);


            // -----------------------------------------------------
            // 0x60B8 Raw Function Value
            // -----------------------------------------------------

            axis[i].home.driveProbeDisarmValue =
                ReadUInt16HomeParam(
                    prefix + "HomeDriveProbeDisarmValue",
                    0x0000);

            axis[i].home.driveProbeArmValue =
                ReadUInt16HomeParam(
                    prefix + "HomeDriveProbeArmValue",
                    0x0015);


            // -----------------------------------------------------
            // 0x60B9 Status Masks
            // -----------------------------------------------------

            axis[i].home.driveProbeArmedMask =
                ReadUInt16HomeParam(
                    prefix + "HomeDriveProbeArmedMask",
                    0x0001);

            axis[i].home.driveProbeCapturedMask =
                ReadUInt16HomeParam(
                    prefix + "HomeDriveProbeCapturedMask",
                    0x0002);

            axis[i].home.driveProbeCaptureToggleMask =
                ReadUInt16HomeParam(
                    prefix + "HomeDriveProbeCaptureToggleMask",
                    0x0000);


            // -----------------------------------------------------
            // 0x60B9 Source Status 驗證
            //
            // SourceMask = 0：
            //     不驗證來源。
            // -----------------------------------------------------

            axis[i].home.driveProbeSourceMask =
                ReadUInt16HomeParam(
                    prefix + "HomeDriveProbeSourceMask",
                    0x0000);

            axis[i].home.driveProbeExpectedSourceValue =
                ReadUInt16HomeParam(
                    prefix + "HomeDriveProbeExpectedSourceValue",
                    0x0000);


            // -----------------------------------------------------
            // Clear / Arm Timeout
            // -----------------------------------------------------

            axis[i].home.driveProbeClearTimeoutSec =
                ConfigUtil::ReadParam(
                    filePath,
                    prefix + "HomeDriveProbeClearTimeout",
                    2.0);

            axis[i].home.driveProbeArmTimeoutSec =
                ConfigUtil::ReadParam(
                    filePath,
                    prefix + "HomeDriveProbeArmTimeout",
                    2.0);


            // -----------------------------------------------------
            // Drive Probe Policy
            // -----------------------------------------------------

            axis[i].home.driveProbeRequireArmedStatus =
                (ConfigUtil::ReadParam(
                    filePath,
                    prefix + "HomeDriveProbeRequireArmedStatus",
                    1.0) == 1.0);

            axis[i].home.driveProbeDisarmAfterCapture =
                (ConfigUtil::ReadParam(
                    filePath,
                    prefix + "HomeDriveProbeDisarmAfterCapture",
                    1.0) == 1.0);

            axis[i].home.driveProbeRequireNewCapture =
                (ConfigUtil::ReadParam(
                    filePath,
                    prefix + "HomeDriveProbeRequireNewCapture",
                    1.0) == 1.0);

            axis[i].home.driveProbeAllowPositionChangeDetection =
                (ConfigUtil::ReadParam(
                    filePath,
                    prefix + "HomeDriveProbeAllowPositionChangeDetection",
                    0.0) == 1.0);





            //尋找DOG --------------------------------------------------------------------
            double searchSpeedUser = ConfigUtil::ReadParam(filePath, prefix + "HomeSearchSpeed", 0.0);//速度 
            axis[i].home.searchSpeed_PPS = MotionCore::UnitPerMinToPps(searchSpeedUser, axis[i].resolution_PPR, axis[i].finalLead);
            axis[i].home.searchAccTime = ConfigUtil::ReadParam(filePath, prefix + "HomeSearchAccTime", 0.2);//加速度
            axis[i].home.searchDecTime = ConfigUtil::ReadParam(filePath, prefix + "HomeSearchDecTime", 0.2);//減速度
            axis[i].home.searchMaxDistance_unit = ConfigUtil::ReadParam(filePath, prefix + "HomeSearchMaxDistance", 0.0);//尋找距離保護 0不保護
            axis[i].home.searchTimeoutSec = ConfigUtil::ReadParam(filePath, prefix + "HomeSearchTimeout", 0.0);//尋找距離時間保護 0不保護




            //找到DOG滑行停止 --------------------------------------------------------------------
            axis[i].home.switchStopDecTime = ConfigUtil::ReadParam(filePath, prefix + "HomeSwitchStopDecTime", 0.2);//減速度
            axis[i].home.switchStopMaxDistance_unit = ConfigUtil::ReadParam(filePath, prefix + "HomeSwitchStopMaxDistance", 0.0);//滑行停止最大距離保護


            // =====================================================
            //  Backoff Mode
            //
            // 0 = FIXED_DISTANCE
            //
            //     固定反方向移動指定距離。
            //
            // 1 = UNTIL_DOG_OFF_PLUS_DISTANCE
            //
            //     先反向移動直到 DOG OFF，
            //     再額外離開指定距離。
            //
            // 建議正式機台使用模式 1。
            // =====================================================

            int backoffMode = static_cast<int>(ConfigUtil::ReadParam(filePath, prefix + "HomeBackoffMode", 1.0));

            if (backoffMode < 0 || backoffMode > 1)
            {
                backoffMode = 1;
            }

            axis[i].home.backoffMode = static_cast<HomeBackoffMode>(backoffMode);
            axis[i].home.backoffDistance_unit = ConfigUtil::ReadParam(filePath, prefix + "HomeBackoffDistance", 0.0);
            axis[i].home.backoffExtraDistance_unit = ConfigUtil::ReadParam(filePath, prefix + "HomeBackoffExtraDistance", 0.0);

            double backoffSpeedUser = ConfigUtil::ReadParam(filePath, prefix + "HomeBackoffSpeed", 0.0);

            axis[i].home.backoffSpeed_PPS = MotionCore::UnitPerMinToPps(backoffSpeedUser, axis[i].resolution_PPR, axis[i].finalLead);
            axis[i].home.backoffAccTime = ConfigUtil::ReadParam(filePath, prefix + "HomeBackoffAccTime", 0.2);
            axis[i].home.backoffDecTime = ConfigUtil::ReadParam(filePath, prefix + "HomeBackoffDecTime", 0.2);
            axis[i].home.backoffMaxDistance_unit = ConfigUtil::ReadParam(filePath, prefix + "HomeBackoffMaxDistance", 0.0);
            axis[i].home.backoffTimeoutSec = ConfigUtil::ReadParam(filePath, prefix + "HomeBackoffTimeout", 0.0);


            // =====================================================
            //  Backoff 完成後安全檢查
            //
            // 預設都必須檢查。
            //
            // 避免軸仍停在 DOG / Hard Limit 區域，
            // 卻開始尋找 INDEX。
            // =====================================================

            axis[i].home.alarmIfDogNotReleasedBeforeIndex = (ConfigUtil::ReadParam(filePath, prefix + "HomeAlarmIfDogNotReleasedBeforeIndex", 1.0) == 1.0);
            axis[i].home.alarmIfHardLimitNotReleasedBeforeIndex = (ConfigUtil::ReadParam(filePath, prefix + "HomeAlarmIfHardLimitNotReleasedBeforeIndex", 1.0) == 1.0);


            // =====================================================
            //  INDEX Search
            // =====================================================

            double indexSpeedUser = ConfigUtil::ReadParam(filePath, prefix + "HomeIndexSearchSpeed", 0.0);
            axis[i].home.indexSearchSpeed_PPS = MotionCore::UnitPerMinToPps(indexSpeedUser, axis[i].resolution_PPR, axis[i].finalLead);
            axis[i].home.indexSearchAccTime = ConfigUtil::ReadParam(filePath, prefix + "HomeIndexSearchAccTime", 0.2);

            // INDEX 找到後：
            //
            // Capture Reference Position
            // ↓
            // Controlled Deceleration
            // ↓
            // MotionState_IDLE
            //
            // 不可以直接急停。
            axis[i].home.indexStopDecTime = ConfigUtil::ReadParam(filePath, prefix + "HomeIndexStopDecTime", 0.2);


            // 最大 INDEX 搜尋距離。
            //
            // 超過仍找不到：
            //
            // HOME INDEX NOT FOUND Alarm
            axis[i].home.indexMaxDistance_unit = ConfigUtil::ReadParam(filePath, prefix + "HomeIndexMaxDistance", 0.0);
            axis[i].home.indexTimeoutSec = ConfigUtil::ReadParam(filePath, prefix + "HomeIndexTimeout", 0.0);


            // =====================================================
            //  Home Offset
            // =====================================================

            axis[i].home.homeOffset_unit = ConfigUtil::ReadParam(filePath, prefix + "HomeOffset", 0.0);


            // =====================================================
            //  HOME 完成後是否定位 Machine Zero
            // =====================================================

            axis[i].home.moveToZero = (ConfigUtil::ReadParam(filePath, prefix + "HomeMoveToZero", 0.0) == 1.0);
            double moveZeroSpeedUser = ConfigUtil::ReadParam(filePath, prefix + "HomeMoveToZeroSpeed", 0.0);
            axis[i].home.moveToZeroSpeed_PPS = MotionCore::UnitPerMinToPps(moveZeroSpeedUser, axis[i].resolution_PPR, axis[i].finalLead);
            axis[i].home.moveToZeroAccTime = ConfigUtil::ReadParam(filePath, prefix + "HomeMoveToZeroAccTime", 0.2);
            axis[i].home.moveToZeroDecTime = ConfigUtil::ReadParam(filePath, prefix + "HomeMoveToZeroDecTime", 0.2);


            // =====================================================
            //  HOME Search 專用 Gain
            //
            // 使用階段：
            //
            // SEARCH_SWITCH
            // SWITCH_DECEL_STOP
            // BACK_OFF
            // =====================================================

            axis[i].home.searchGain.Kp = ConfigUtil::ReadParam(filePath, prefix + "HomeSearch_Kp", 20.0);
            axis[i].home.searchGain.Ki = ConfigUtil::ReadParam(filePath, prefix + "HomeSearch_Ki", 0.0);
            axis[i].home.searchGain.Kd = ConfigUtil::ReadParam(filePath, prefix + "HomeSearch_Kd", 0.0);
            axis[i].home.searchGain.Kvff = ConfigUtil::ReadParam(filePath, prefix + "HomeSearch_Kvff", 1.0);

            // =====================================================
            //  HOME INDEX 專用 Gain
            //
            // 使用階段：
            //
            // SEARCH_INDEX
            // INDEX_DECEL_STOP
            // =====================================================

            axis[i].home.indexGain.Kp = ConfigUtil::ReadParam(filePath, prefix + "HomeIndex_Kp", 20.0);
            axis[i].home.indexGain.Ki = ConfigUtil::ReadParam(filePath, prefix + "HomeIndex_Ki", 0.0);
            axis[i].home.indexGain.Kd = ConfigUtil::ReadParam(filePath, prefix + "HomeIndex_Kd", 0.0);
            axis[i].home.indexGain.Kvff = ConfigUtil::ReadParam(filePath, prefix + "HomeIndex_Kvff", 1.0);


            // =====================================================
            // HOME 速度安全 Clamp
            //
            // HOME 不允許任何階段超過該軸 MAX Speed。
            // =====================================================

            if (axis[i].maxVel_PPS > 0.0)
            {
                if (axis[i].home.searchSpeed_PPS > axis[i].maxVel_PPS)
                {
                    axis[i].home.searchSpeed_PPS = axis[i].maxVel_PPS;
                }

                if (axis[i].home.backoffSpeed_PPS > axis[i].maxVel_PPS)
                {
                    axis[i].home.backoffSpeed_PPS = axis[i].maxVel_PPS;
                }

                if (axis[i].home.indexSearchSpeed_PPS > axis[i].maxVel_PPS)
                {
                    axis[i].home.indexSearchSpeed_PPS = axis[i].maxVel_PPS;
                }

                if (axis[i].home.moveToZeroSpeed_PPS > axis[i].maxVel_PPS)
                {
                    axis[i].home.moveToZeroSpeed_PPS = axis[i].maxVel_PPS;
                }
            }
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

bool GlobalConfig::InitSystemParameters(EtherCatMaster& master)
{
    DEBUG_PRINT("========== Starting system parameter loading and initialization ==========\n");

    // 1. 綁定硬體指標 (必須最先做，後面的參數載入才會寫入正確的實體)
    master.m_Motion.Link(&master.m_ServoList, &master.m_Axes);

    // 2. 🌟 一鍵載入參數並初始化所有軸！
    std::string axisConfigPath = GlobalConfig::GetInstance().ParameterDir + "AxisConfig.txt";
    if (!GlobalConfig::GetInstance().LoadAxisConfig(axisConfigPath, master.m_Axes, master.m_Motion))
    {
        DEBUG_PRINT("LoadConfig Error！>>AxisConfig.txt\n");
        return -1;
    }


    std::string pidConfigPath = GlobalConfig::GetInstance().ParameterDir + "PIDConfig.txt";
    if (!GlobalConfig::GetInstance().LoadPIDConfig(pidConfigPath, master.m_Axes, master.m_Motion))
    {
        DEBUG_PRINT("LoadConfig Error！>>PIDConfig.txt\n");
        return -1;
    }

    std::string speedConfigPath = GlobalConfig::GetInstance().ParameterDir + "SpeedConfig.txt";
    if (!GlobalConfig::GetInstance().LoadSpeedConfig(speedConfigPath, master.m_Axes, master.m_Motion))
    {
        DEBUG_PRINT("LoadConfigPathConfig Error！>>SpeedConfig.txt\n");
        return -1;
    }

    // 2. 🌟 讀取螺距誤差表，並寫入 CompensationEngine
    GlobalConfig::LoadPitchTable(GlobalConfig::GetInstance().ParameterDir + "PITCH_TABLE_Pos.txt", master.m_Motion.m_CompEngine, true);

    // 2. 🌟 讀取螺距誤差表，並寫入 CompensationEngine
    GlobalConfig::LoadPitchTable(GlobalConfig::GetInstance().ParameterDir + "PITCH_TABLE_Neg.txt", master.m_Motion.m_CompEngine, false);


    //坐標系初始化
    master.m_NC->CoordSys.SetWCS(master.m_NC->CoordSys.GetCurrentWCSGCode(), master.m_NC);
    master.pCoordMgr = &(master.m_NC->CoordSys); // 指向目前的座標系
    master.m_Motion.LinkCoordinateManager(&(master.m_NC->CoordSys));


    //載入NC設定
    std::string NCConfigPath = GlobalConfig::GetInstance().ParameterDir + "NCConfig.txt";
    if (!GlobalConfig::GetInstance().LoadNCConfig(NCConfigPath, master.m_Axes, master.m_Motion, master.m_NC))
    {
        DEBUG_PRINT("LoadConfig Error！>>NCConfig.txt\n");
        return false;
    }

    //載入Home設定
    std::string HomeConfigPath = GlobalConfig::GetInstance().ParameterDir + "HomeConfig.txt";
    if (!GlobalConfig::GetInstance().LoadHomeConfig(HomeConfigPath, master.m_Axes, master.m_Motion, master.m_NC))
    {
        DEBUG_PRINT("LoadConfig Error！>>HomeConfig.txt\n");
        return -1;
    }


    // =========================================================
    // HOME Persistence
    //
    // Snapshot / History 固定放在：
    //
    // GlobalConfig::NCDataDir
    // = D:\EtherCAT_Master_Data\Data\
    //
    // 啟動時會載入「上一次成功 HOME」供診斷，
    // 但一定強制所有軸 isHomed=false。
    //
    // 因此每次 Core / 機台重新啟動後仍必須重新 G81。
    // =========================================================

    HomePersistenceManager::GetInstance().Initialize(
        GlobalConfig::GetInstance().NCDataDir,
        master.m_Axes,
        master.m_NC);


    //載入初始NC檔案---------------------------------------------------------------------
    std::string Initial_NcPath = GlobalConfig::GetInstance().NCProgramDir + "Null.nc";
    master.m_NC->LoadProgram(Initial_NcPath);

    return true;
}
