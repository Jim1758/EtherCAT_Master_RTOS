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
#include "NCEccentricCSelfCheck.h"
#include "NCEccentricCRuntimeSelfCheck.h"
#include "NCEccentricCProfileSelfCheck.h"
#include "NCEccentricCExecutorSelfCheck.h"
#include "NCEccentricCTransportSelfCheck.h"
#include "NCEccentricCPipelineSelfCheck.h"
#include "NCEccentricCBindingSelfCheck.h"
#include "NCEccentricCTransactionSelfCheck.h"
#include "CNCStartupStackDiagnostic.h"
#include "AlarmManager.h"
#include <cmath>
#include "CompensationConfigIO.h"
#include "EDMVoltageConfigIO.h"
#include "EDMGapInputConfigIO.h"
#include "EDMRecipeConfigIO.h"
#include "EDMConditionStartupIO.h"
#include "EDMProcessConfigIO.h"


namespace
{
    bool PbcBootFailure(int alarm, const pbc::Diagnostic& d, const char* source)
    {
        DEBUG_PRINT("[PBC-1][AL%d] source=%s axis=%d reason=%s lineOrRow=%u column=%u\n",
            alarm, source, d.axis, pbc::ErrorName(d.error),
            static_cast<unsigned>(d.line), static_cast<unsigned>(d.column));
        AlarmManager::GetInstance().Trigger(alarm, 0, d.axis);
        return false;
    }

    bool ReadPbcAxisParameters(const pbc::ParameterMap& values, int index,
        AxisContext& axis, CompensationEngine& engine, pbc::Diagnostic& d)
    {
        d = pbc::Diagnostic{};
        d.axis = index;
        const std::string key = std::to_string(index) + "_";
        pbc::Config config{};
        config.policy.periodic = axis.axisType == AxisType::ROTARY ||
            axis.axisType == AxisType::ROTARY_CONTINUOUS;
        if (!pbc::Flag(values, key + "EnableBacklash", false, config.backlash, d) ||
            !pbc::Flag(values, key + "EnablePitch", false, config.pitch, d) ||
            !pbc::Number(values, key + "backlashAmount_Pos_mm", 0.0, config.backlashPositive, d) ||
            !pbc::Number(values, key + "backlashAmount_Neg_mm", 0.0, config.backlashNegative, d) ||
            !pbc::Number(values, key + "backlashSpeed", 3.0, config.backlashSpeed, d) ||
            !pbc::Number(values, key + "PitchStartPos", 0.0, config.pitchStart, d) ||
            !pbc::Number(values, key + "PitchStep", 10.0, config.pitchStep, d) ||
            !pbc::Number(values, key + "PitchSpeed_mm_s", 3.0, config.pitchSpeed, d) ||
            !pbc::Number(values, key + "CompMaxAbsOffset_unit", 0.1, config.policy.maxAbsOffset, d) ||
            !pbc::Number(values, key + "CompMaxAbsSlope", 0.01, config.policy.maxAbsSlope, d) ||
            !pbc::Flag(values, key + "PitchPeriodic", config.policy.periodic, config.policy.periodic, d) ||
            !pbc::Flag(values, key + "CompAllowDirectionalPitchWithBacklash", false,
                config.policy.allowDirectionalPitchWithBacklash, d)) return false;
        if (config.backlashPositive < 0.0 || config.backlashNegative < 0.0 ||
            config.backlashSpeed <= 0.0 || config.backlashSpeed > 100.0 ||
            config.pitchStep <= 0.0 || config.pitchSpeed <= 0.0 || config.pitchSpeed > 100.0 ||
            config.policy.maxAbsOffset <= 0.0 || config.policy.maxAbsOffset > 1.0 ||
            config.policy.maxAbsSlope <= 0.0 || config.policy.maxAbsSlope > 0.1)
        { d.error = pbc::Error::ParameterRange; return false; }
        if (!engine.InitAxisCompensation(index, config.backlash, config.backlashPositive,
                config.backlashNegative, config.backlashSpeed, config.pitch,
                config.pitchStart, config.pitchStep, config.pitchSpeed) ||
            !engine.SetAxisPolicy(index, config.policy))
        { d.error = pbc::Error::ConfigurationSealed; return false; }
        axis.enableBacklash = config.backlash;
        axis.backlashAmount_Pos_mm = config.backlashPositive;
        axis.backlashAmount_Neg_mm = config.backlashNegative;
        axis.backlashSpeed = config.backlashSpeed;
        axis.enablePitch = config.pitch;
        axis.pitchStartPos_mm = config.pitchStart;
        axis.pitchStep_mm = config.pitchStep;
        axis.pitchSpeed_mm_s = config.pitchSpeed;
        return true;
    }

    bool StagePbcPitchPair(const std::string& directory, CompensationEngine& engine)
    {
        if (engine.IsSealed())
        {
            pbc::Diagnostic d{}; d.error = pbc::Error::ConfigurationSealed;
            return PbcBootFailure(AlarmManager::MECHANICAL_COMPENSATION_CONFIG_INVALID, d, "TABLE_PAIR");
        }
        pbc::PitchColumns positive{}, negative{};
        pbc::Diagnostic posDiagnostic{}, negDiagnostic{};
        const bool posOk = pbc::ReadPitchFile(directory + "PITCH_TABLE_Pos.txt", positive, posDiagnostic);
        const bool negOk = pbc::ReadPitchFile(directory + "PITCH_TABLE_Neg.txt", negative, negDiagnostic);
        bool pairOk = posOk && negOk;
        pbc::Diagnostic problem = !posOk ? posDiagnostic : negDiagnostic;
        const char* source = !posOk ? "PITCH_TABLE_Pos.txt" : "PITCH_TABLE_Neg.txt";
        if (pairOk && positive[0].size() != negative[0].size())
        { pairOk = false; problem.error = pbc::Error::TablePair; source = "TABLE_PAIR_ROWS"; }
        if (!pairOk)
        {
            if (engine.HasEnabledPitch())
                return PbcBootFailure(AlarmManager::MECHANICAL_COMPENSATION_TABLE_INVALID, problem, source);
            DEBUG_PRINT("[PBC-1] UNUSED_TABLE_REJECT source=%s reason=%s line=%u; no pitch axis enabled\n",
                source, pbc::ErrorName(problem.error), static_cast<unsigned>(problem.line));
            // Neither direction is committed. No mixed new/old table state.
            positive = pbc::PitchColumns{};
            negative = pbc::PitchColumns{};
        }
        for (std::size_t i = 0U; i < pbc::AxisCount; ++i)
        {
            if (!engine.SetPitchTables(static_cast<int>(i), positive[i], negative[i]))
            {
                pbc::Diagnostic d{}; d.error = pbc::Error::Allocation; d.axis = static_cast<int>(i);
                return PbcBootFailure(AlarmManager::MECHANICAL_COMPENSATION_CONFIG_INVALID, d, "TABLE_PAIR_STAGE");
            }
        }
        if (pairOk)
            DEBUG_PRINT("[PBC-1] TABLE_PAIR=PARSED rows=%u columns=8; dormant data is not calibration acceptance\n",
                static_cast<unsigned>(positive[0].size()));
        return true;
    }
}

bool GlobalConfig::LoadAxisConfig(const std::string& filePath, std::vector<AxisContext>& axis, MotionCore& motion)
{
    // PBC-1: parsing/validation happens before resize or compensation mutation.
    if (motion.m_CompEngine.IsSealed())
    {
        pbc::Diagnostic d{}; d.error = pbc::Error::ConfigurationSealed;
        return PbcBootFailure(AlarmManager::MECHANICAL_COMPENSATION_CONFIG_INVALID, d, "AxisConfig.txt");
    }
    pbc::ParameterMap pbcParameters;
    pbc::Diagnostic pbcDiagnostic{};
    if (!pbc::ReadParameterFile(filePath, pbcParameters, pbcDiagnostic))
        return PbcBootFailure(AlarmManager::MECHANICAL_COMPENSATION_CONFIG_INVALID, pbcDiagnostic, "AxisConfig.txt");
    double requestedAxisCount = 0.0;
    if (!pbc::Number(pbcParameters, "AxisCount", 0.0, requestedAxisCount, pbcDiagnostic) ||
        requestedAxisCount < 1.0 || requestedAxisCount > 8.0 ||
        std::floor(requestedAxisCount) != requestedAxisCount)
    {
        if (pbcDiagnostic.error == pbc::Error::None) pbcDiagnostic.error = pbc::Error::AxisIndex;
        return PbcBootFailure(AlarmManager::MECHANICAL_COMPENSATION_CONFIG_INVALID, pbcDiagnostic, "AxisConfig.txt");
    }
    int axisCount = static_cast<int>(requestedAxisCount);
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
            PrintCNCStartupStackCheckpoint("VIRTUAL_INIT_ENTER", i);
            motion.InitVirtualAxisSmooth(smoothTime);
            PrintCNCStartupStackCheckpoint("VIRTUAL_INIT_EXIT", i);

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

            // Native units are mm for linear axes, degrees for rotary axes.
            // Legacy key spelling is retained; no automatic unit conversion.
            if (!ReadPbcAxisParameters(pbcParameters, i, axis[i], motion.m_CompEngine, pbcDiagnostic))
                return PbcBootFailure(AlarmManager::MECHANICAL_COMPENSATION_CONFIG_INVALID,
                    pbcDiagnostic, "AxisConfig.txt/compensation");


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
    DEBUG_PRINT("[SPEED-CONFIG][BASE40] AxisCount=%d parameterMapping=OWN_KEY\n", axisCount);
    DEBUG_PRINT("[POSITIONING][BASE48] exactStopProfiles=7,28,30,32,161 referenceAtomicPair=1 homeWholeBlock=1 homeRequestPreflight=1 homePendingHandoffGuard=1 homeZeroTravelPreflight=1 homeZeroResolvedTarget=1 homeBackoffDirected=1 homeBackoffBudget=1 homeMoveAckGate=1 homeControlStopAckGate=1 g53VelocityCeiling=1 positioningDynamicsPreflight=1 cornerConsumerPreflight=1\n");
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
            axis[i].JOG_dec_time = Jog_dec_time;


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
            axis[i].G00_dec_time = G00_dec_time;


            double g07_speed_user = ConfigUtil::ReadParam(filePath, prefix + "G07_Speed", 0);
            axis[i].G07_PPS = MotionCore::UnitPerMinToPps(g07_speed_user, axis[i].resolution_PPR, axis[i].finalLead);

            double G07_acc_time = ConfigUtil::ReadParam(filePath, prefix + "G07_acc_time", 0);
            axis[i].G07_acc_time = G07_acc_time;

            double G07_dec_time = ConfigUtil::ReadParam(filePath, prefix + "G07_dec_time", 0);
            axis[i].G07_dec_time = G07_dec_time;

            double g161_speed_user = ConfigUtil::ReadParam(filePath, prefix + "G161_Speed", 0);
            axis[i].G161_PPS = MotionCore::UnitPerMinToPps(g161_speed_user, axis[i].resolution_PPR, axis[i].finalLead);

            double G161_acc_time = ConfigUtil::ReadParam(filePath, prefix + "G161_acc_time", 0);
            axis[i].G161_acc_time = G161_acc_time;

            double G161_dec_time = ConfigUtil::ReadParam(filePath, prefix + "G161_dec_time", 0);
            axis[i].G161_dec_time = G161_dec_time;






            double g28_speed_user = ConfigUtil::ReadParam(filePath, prefix + "G28_Speed", 500.0);
            axis[i].G28_PPS = MotionCore::UnitPerMinToPps(g28_speed_user, axis[i].resolution_PPR, axis[i].finalLead);

            double G28_acc_time = ConfigUtil::ReadParam(filePath, prefix + "G28_acc_time", 0);
            axis[i].G28_acc_time = G28_acc_time;

            double G28_dec_time = ConfigUtil::ReadParam(filePath, prefix + "G28_dec_time", 0);
            axis[i].G28_dec_time = G28_dec_time;



            double g30_speed_user = ConfigUtil::ReadParam(filePath, prefix + "G30_Speed", 0);
            axis[i].G30_PPS = MotionCore::UnitPerMinToPps(g30_speed_user, axis[i].resolution_PPR, axis[i].finalLead);

            double G30_acc_time = ConfigUtil::ReadParam(filePath, prefix + "G30_acc_time", 0);
            axis[i].G30_acc_time = G30_acc_time;

            double G30_dec_time = ConfigUtil::ReadParam(filePath, prefix + "G30_dec_time", 0);
            axis[i].G30_dec_time = G30_dec_time;


            double g32_speed_user = ConfigUtil::ReadParam(filePath, prefix + "G32_Speed", 0);
            axis[i].G32_PPS = MotionCore::UnitPerMinToPps(g32_speed_user, axis[i].resolution_PPR, axis[i].finalLead);

            double G32_acc_time = ConfigUtil::ReadParam(filePath, prefix + "G32_acc_time", 0);
            axis[i].G32_acc_time = G32_acc_time;

            double G32_dec_time = ConfigUtil::ReadParam(filePath, prefix + "G32_dec_time", 0);
            axis[i].G32_dec_time = G32_dec_time;



            double g53_speed_user = ConfigUtil::ReadParam(filePath, prefix + "G53_Speed", 0);
            axis[i].G53_PPS = MotionCore::UnitPerMinToPps(g53_speed_user, axis[i].resolution_PPR, axis[i].finalLead);

            double G53_acc_time = ConfigUtil::ReadParam(filePath, prefix + "G53_acc_time", 0);
            axis[i].G53_acc_time = G53_acc_time;

            double G53_dec_time = ConfigUtil::ReadParam(filePath, prefix + "G53_dec_time", 0);
            axis[i].G53_dec_time = G53_dec_time;





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
    // BASE76 verifies only a synthetic fixed-Z geometry model at startup.
    // The startup-only live role is reported, never inferred or changed here.
    // Existing G162 Motion admission remains closed until its dedicated curved
    // consumer and stopping envelope are implemented and verified separately.
    PrintCNCStartupStackCheckpoint("STARTUP_CORE_ENTER");
    const NCEccentricCSelfCheckResult eccentricCore = RunNCEccentricCSelfCheck();
    RtPrintf("[BASE76][ECC-CORE] selfcheck=%s checks=%u failed=%u role=%d roleAssigned=%u dynamicMotion=1 restrictedNc=1\n",
        eccentricCore.passed ? "PASS" : "FAIL",
        static_cast<unsigned>(eccentricCore.checks), static_cast<unsigned>(eccentricCore.failedCheck),
        electrodeAxis, electrodeAxis != 0 ? 1U : 0U);
    if (!eccentricCore.passed)
    {
        AlarmManager::GetInstance().Trigger(AlarmManager::PATH_GEOMETRY_INVALID);
        return false;
    }
    // BASE77 is a synthetic prepared-pulse/stop-envelope model. The source
    // is local to the check; no live axis or table is altered or authorized.
    PrintCNCStartupStackCheckpoint("STARTUP_RUNTIME_ENTER");
    const NCEccentricCRuntimeSelfCheckResult eccentricRuntime = RunNCEccentricCRuntimeSelfCheck();
    RtPrintf("[BASE77][ECC-RUNTIME] selfcheck=%s checks=%u failed=%u role=%d roleAssigned=%u cycleUs=250 modelOnly=1 dynamicMotion=1 restrictedNc=1 phase=STARTUP\n",
        eccentricRuntime.passed ? "PASS" : "FAIL",
        static_cast<unsigned>(eccentricRuntime.checks), static_cast<unsigned>(eccentricRuntime.failedCheck),
        electrodeAxis, electrodeAxis != 0 ? 1U : 0U);
    if (!eccentricRuntime.passed)
    {
        AlarmManager::GetInstance().Trigger(AlarmManager::PATH_GEOMETRY_INVALID);
        return false;
    }
    // BASE78 checks a dedicated sampled scalar profile and controlled STOP.
    // Synthetic only: these samples bypass the legacy FIR and have no live owner.
    PrintCNCStartupStackCheckpoint("STARTUP_PROFILE_ENTER");
    const NCEccentricCProfileSelfCheckResult eccentricProfile = RunNCEccentricCProfileSelfCheck();
    RtPrintf("[BASE78][ECC-PROFILE] selfcheck=%s checks=%u failed=%u role=%d roleAssigned=%u cycleUs=250 modelOnly=1 dynamicMotion=1 restrictedNc=1 legacyFir=0 phase=STARTUP\n",
        eccentricProfile.passed ? "PASS" : "FAIL",
        static_cast<unsigned>(eccentricProfile.checks), static_cast<unsigned>(eccentricProfile.failedCheck),
        electrodeAxis, electrodeAxis != 0 ? 1U : 0U);
    if (!eccentricProfile.passed)
    {
        AlarmManager::GetInstance().Trigger(AlarmManager::PATH_GEOMETRY_INVALID);
        return false;
    }

    // BASE79A: synthetic owned-executor diagnostics; no axis output is made by this check.
    // No new alarm/admission decision is made by this isolated test result.
    PrintCNCStartupStackCheckpoint("STARTUP_EXEC_ENTER");
    const NCEccentricCExecutorSelfCheckResult eccentricExecutor = RunNCEccentricCExecutorSelfCheck();
    PrintCNCStartupStackCheckpoint("STARTUP_EXEC_EXIT");
    RtPrintf("[BASE79A][ECC-EXECUTOR] selfcheck=%s checks=%u failed=%u configuredRole=%d isolated=1 diagnosticOnly=1 dynamicMotion=1 restrictedNc=1 phase=STARTUP\n",
        eccentricExecutor.passed ? "PASS" : "FAIL",
        static_cast<unsigned>(eccentricExecutor.checks), static_cast<unsigned>(eccentricExecutor.failedCheck),
        electrodeAxis);

    // BASE79B_FIX1: heap-backed transport diagnostics; live NC admission remains closed.
    PrintCNCStartupStackCheckpoint("STARTUP_TRANSPORT_ENTER");
    const NCEccentricCTransportSelfCheckResult eccentricTransport = RunNCEccentricCTransportSelfCheck();
    PrintCNCStartupStackCheckpoint("STARTUP_TRANSPORT_EXIT");
    RtPrintf("[BASE79B-FIX1][ECC-TRANSPORT] selfcheck=%s checks=%u failed=%u configuredRole=%d heapScratch=1 diagnosticOnly=1 dynamicMotion=1 restrictedNc=1 phase=STARTUP\n",
        eccentricTransport.passed ? "PASS" : "FAIL",
        static_cast<unsigned>(eccentricTransport.checks), static_cast<unsigned>(eccentricTransport.failedCheck),
        electrodeAxis);

    // BASE79C: copied packet -> decoded plan -> isolated executor diagnostics.
    // Keep live NC admission closed and the machine role unchanged.
    PrintCNCStartupStackCheckpoint("STARTUP_PIPELINE_ENTER");
    const NCEccentricCPipelineSelfCheckResult eccentricPipeline = RunNCEccentricCPipelineSelfCheck();
    PrintCNCStartupStackCheckpoint("STARTUP_PIPELINE_EXIT");
    RtPrintf("[BASE79C][ECC-PIPELINE] selfcheck=%s checks=%u failed=%u configuredRole=%d heapScratch=1 diagnosticOnly=1 dynamicMotion=1 restrictedNc=1 phase=STARTUP\n",
        eccentricPipeline.passed ? "PASS" : "FAIL",
        static_cast<unsigned>(eccentricPipeline.checks), static_cast<unsigned>(eccentricPipeline.failedCheck),
        electrodeAxis);

    // BASE79D: synthetic checks of the read-only consumer start binding.
    PrintCNCStartupStackCheckpoint("STARTUP_BINDING_ENTER");
    const NCEccentricCBindingSelfCheckResult eccentricBinding = RunNCEccentricCBindingSelfCheck();
    PrintCNCStartupStackCheckpoint("STARTUP_BINDING_EXIT");
    RtPrintf("[BASE79D][ECC-BINDING] selfcheck=%s checks=%u failed=%u configuredRole=%d bindingOnly=1 heapScratch=1 diagnosticOnly=1 dynamicMotion=1 restrictedNc=1 phase=STARTUP\n",
        eccentricBinding.passed ? "PASS" : "FAIL",
        static_cast<unsigned>(eccentricBinding.checks), static_cast<unsigned>(eccentricBinding.failedCheck),
        electrodeAxis);

    // BASE79E: private candidate/commit diagnostics; this check makes no axis output.
    PrintCNCStartupStackCheckpoint("STARTUP_TRANSACTION_ENTER");
    const NCEccentricCTransactionSelfCheckResult eccentricTransaction = RunNCEccentricCTransactionSelfCheck();
    PrintCNCStartupStackCheckpoint("STARTUP_TRANSACTION_EXIT");
    RtPrintf("[BASE79E][ECC-TRANSACTION] selfcheck=%s checks=%u failed=%u configuredRole=%d stagedCommit=1 heapScratch=1 diagnosticOnly=1 dynamicMotion=1 restrictedNc=1 phase=STARTUP\n",
        eccentricTransaction.passed ? "PASS" : "FAIL",
        static_cast<unsigned>(eccentricTransaction.checks), static_cast<unsigned>(eccentricTransaction.failedCheck),
        electrodeAxis);


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
    // Compatibility staging API only. Runtime uses sealed, paired data;
    // this function never commits one direction into a running model.
    if (compEngine.IsSealed()) return false;
    pbc::PitchColumns columns{};
    pbc::Diagnostic d{};
    if (!pbc::ReadPitchFile(filePath, columns, d))
    {
        DEBUG_PRINT("[PBC-1] TABLE_REJECT source=%s reason=%s line=%u column=%u\n",
            filePath.c_str(), pbc::ErrorName(d.error),
            static_cast<unsigned>(d.line), static_cast<unsigned>(d.column));
        return false;
    }
    for (std::size_t i = 0U; i < pbc::AxisCount; ++i)
    {
        const bool staged = isPositive ? compEngine.SetPitchTablePos(static_cast<int>(i), columns[i]) :
            compEngine.SetPitchTableNeg(static_cast<int>(i), columns[i]);
        if (!staged) return false;
    }
    return true;
}

bool GlobalConfig::InitSystemParameters(EtherCatMaster& master)
{
    PrintCNCStartupStackCheckpoint("SYS_INIT_ENTER");
    RtPrintf("[BASE79N][BUILD] base=BASE79M_FIX1 nativeXYZCEndpoint=1 resolvedRotaryTail=1 lifecycleRegression=1 notReadyDiag=1 absoluteC=1 incrementalC=1 incrementalZC=1 absoluteZC=1 incrementalXYZC=1 absoluteXYZC=1 nominalXYTargets=1 zFeedMMMin=1 xyzFeedMMMin=1 holdResume=1 heldResetCurved=1 lateResumeReject=2014 motionConsumer=1 heapScratch=1 dynamicMotion=1 restrictedNc=1 packetBytes=%u motionCoreBytes=%u groupBytes=%u\n",
        static_cast<unsigned>(sizeof(MotionCommand)), static_cast<unsigned>(sizeof(MotionCore)),
        static_cast<unsigned>(sizeof(InterpolationGroup)));
    DEBUG_PRINT("========== Starting system parameter loading and initialization ==========\n");

    // 1. 綁定硬體指標 (必須最先做，後面的參數載入才會寫入正確的實體)
    master.m_Motion.Link(&master.m_ServoList, &master.m_Axes);

    // BASE79F: reserve the sole consumer workspace before configuration or
    // cyclic threads can use Motion. Runtime and NC LOAD never allocate it.
    PrintCNCStartupStackCheckpoint("ECC_STORAGE_ENTER");
    const bool eccentricStorageReady = master.m_Motion.InitializeEccentricCConsumer();
    PrintCNCStartupStackCheckpoint("ECC_STORAGE_EXIT");
    RtPrintf("[BASE79K][ECC-STORAGE] ready=%u storageBytes=%u motionConsumer=1 heapStorage=1 dynamicMotion=1 restrictedNc=1 phase=STARTUP\n",
        eccentricStorageReady ? 1U : 0U,
        static_cast<unsigned>(master.m_Motion.EccentricCConsumerStorageBytes()));
    if (!eccentricStorageReady) return false;
    PrintCNCStartupStackCheckpoint("ECC_PRODUCER_STORAGE_ENTER");
    const bool eccentricProducerReady = master.m_NC && master.m_NC->InitializeEccentricCProducer();
    PrintCNCStartupStackCheckpoint("ECC_PRODUCER_STORAGE_EXIT");
    RtPrintf("[BASE79K][ECC-PRODUCER-STORAGE] ready=%u storageBytes=%u restrictedNc=1 phase=STARTUP\n",
        eccentricProducerReady ? 1U : 0U,
        master.m_NC ? static_cast<unsigned>(master.m_NC->EccentricCProducerStorageBytes()) : 0U);
    if (!eccentricProducerReady) return false;

    // 2. 🌟 一鍵載入參數並初始化所有軸！
    std::string axisConfigPath = GlobalConfig::GetInstance().ParameterDir + "AxisConfig.txt";
    if (!GlobalConfig::GetInstance().LoadAxisConfig(axisConfigPath, master.m_Axes, master.m_Motion))
    {
        DEBUG_PRINT("LoadConfig Error！>>AxisConfig.txt\n");
        return false; // PBC-1: bool(-1) is true, never use it for failure.
    }


    std::string pidConfigPath = GlobalConfig::GetInstance().ParameterDir + "PIDConfig.txt";
    if (!GlobalConfig::GetInstance().LoadPIDConfig(pidConfigPath, master.m_Axes, master.m_Motion))
    {
        DEBUG_PRINT("LoadConfig Error！>>PIDConfig.txt\n");
        return false; // PBC-1: bool(-1) is true, never use it for failure.
    }

    std::string speedConfigPath = GlobalConfig::GetInstance().ParameterDir + "SpeedConfig.txt";
    if (!GlobalConfig::GetInstance().LoadSpeedConfig(speedConfigPath, master.m_Axes, master.m_Motion))
    {
        DEBUG_PRINT("LoadConfigPathConfig Error！>>SpeedConfig.txt\n");
        return false; // PBC-1: bool(-1) is true, never use it for failure.
    }

    // PBC-1: validate a complete pair and every axis before sealing the model.
    if (!StagePbcPitchPair(GlobalConfig::GetInstance().ParameterDir, master.m_Motion.m_CompEngine))
        return false;
    pbc::Diagnostic compensationDiagnostic{};
    if (!master.m_Motion.m_CompEngine.FinalizeConfiguration(master.m_Axes, compensationDiagnostic))
    {
        const int code = compensationDiagnostic.error == pbc::Error::IntegrationPending ?
            AlarmManager::MECHANICAL_COMPENSATION_NOT_INTEGRATED :
            AlarmManager::MECHANICAL_COMPENSATION_CONFIG_INVALID;
        return PbcBootFailure(code, compensationDiagnostic, "COMPENSATION_FINALIZE");
    }
    DEBUG_PRINT("[PBC-1] CONFIG=PASS axes=%u enabled=%u GENERAL_NONZERO=LOCKED\n",
        static_cast<unsigned>(master.m_Axes.size()), master.m_Motion.m_CompEngine.EnabledAxisCount());
    DEBUG_PRINT("[PBC-2] COORDINATE_CONTRACT=PASS checks=%u FEEDBACK=NOMINAL BEFORE_WCS=1 GENERAL_NONZERO=LOCKED\n",
        master.m_Motion.m_CompEngine.CoordinateContractChecks());
    DEBUG_PRINT("[PBC-3B] LIFECYCLE_MODEL=PASS checks=%u TX_SEAM=NIC_API_RESULT GENERAL_NONZERO=LOCKED\n",
        master.m_Motion.m_CompEngine.LifecycleContractChecks());

    unsigned sendContractChecks = 0U;
    if (!pbc::CheckSendContract(sendContractChecks))
    {
        compensationDiagnostic.error = pbc::Error::LifecycleContract;
        return PbcBootFailure(AlarmManager::MECHANICAL_COMPENSATION_CONFIG_INVALID,
            compensationDiagnostic, "X_SEND_CONTRACT");
    }
    DEBUG_PRINT("[PBC-3C] X_SEND_CONTRACT=PASS checks=%u mode=SHADOW_ZERO GENERAL_NONZERO=LOCKED\n",
        sendContractChecks);
    DEBUG_PRINT("[PBC-3D] X_REFERENCE_CONTRACT=PASS checks=%u LIVE_COMMIT=HOME_RESET SNAP=NUMERIC_ONLY GENERAL_NONZERO=LOCKED\n",
        master.m_Motion.m_CompEngine.XReferenceChecks());
    DEBUG_PRINT("[PBC-3E] X_CONTROL_CONTRACT=PASS checks=%u LIVE_CYCLE=OFF_OR_X_ZERO_OR_K_PITCH_OR_L_BACKLASH_OR_M_COMBINED_OR_N_ASYMMETRIC_OR_O_DIRECTIONAL_PITCH_OR_P_DIRECTIONAL_COMBINED MODEL_COMMIT=NIC_API_RESULT ZERO_CANDIDATE=RESERVED GENERAL_NONZERO=LOCKED\n",
        master.m_Motion.m_CompEngine.XControlChecks());
    DEBUG_PRINT("[PBC-3E-FIX1] STOP_SEND_RESERVATION=EXACT GENERAL_NONZERO=LOCKED\n");
    DEBUG_PRINT("[PBC-3F] FROZEN_STOP=COMMITTED_ONLY RESET=RETAIN_KNOWN_OFFSET INVALID_ACTIVE=FENCED GENERAL_NONZERO=LOCKED\n");
    DEBUG_PRINT("[PBC-3G] KNOWN_RECOVERY=RESET_RETAIN LOSS_SNAP=INHIBITED UNKNOWN_OUTPUT=QUARANTINED ACTIVE_HOME=X_ZERO_OR_STAGED_FIRST_INDEX GENERAL_NONZERO=LOCKED\n");
    unsigned homeStopProofChecks = 0U;
    if (!CheckMotionPbcHomeStopProof(homeStopProofChecks))
    {
        compensationDiagnostic.error = pbc::Error::LifecycleContract;
        return PbcBootFailure(AlarmManager::MECHANICAL_COMPENSATION_CONFIG_INVALID,
            compensationDiagnostic, "X_HOME_RAW_STOP_PROOF");
    }
    DEBUG_PRINT("[PBC-3H] X_HOME_RAW_STOP=PASS checks=%u samples=%u STOP_PROOF_ONLY=1 GENERAL_NONZERO=LOCKED\n",
        homeStopProofChecks, MotionPbcHomeStopProof::RequiredSamples);
    unsigned homeCaptureProofChecks = 0U;
    if (!CheckMotionPbcHomeCaptureProof(homeCaptureProofChecks))
    {
        compensationDiagnostic.error = pbc::Error::LifecycleContract;
        return PbcBootFailure(AlarmManager::MECHANICAL_COMPENSATION_CONFIG_INVALID,
            compensationDiagnostic, "X_HOME_CAPTURE_PROOF");
    }
    DEBUG_PRINT("[PBC-3I] X_CAPTURE_CONTRACT=PASS checks=%u TX=SERIALIZED_NIC INPUT=RT_CURRENT HOME=TOKEN_REQUIRED UNKNOWN_RECOVERY=LOCKED GENERAL_NONZERO=LOCKED\n",
        homeCaptureProofChecks);

    // Public FinalizeConfiguration has already validated the numerical boot
    // frame before any CoordSys/NC consumer. This does not establish HOME.
    const AxisContext* zeroOnlyX = master.m_Axes.empty() ? nullptr : &master.m_Axes[0];
    DEBUG_PRINT("[PBC-3J] X_ZERO_ONLY=%u PITCH=%u BACKLASH=%u BOOT_FRAME=VALIDATED HOME=REQUIRED UNKNOWN_RECOVERY=LOCKED GENERAL_NONZERO=LOCKED\n",
        zeroOnlyX && master.m_Motion.m_CompEngine.IsXZeroOnlyConfiguration(*zeroOnlyX) ? 1U : 0U,
        zeroOnlyX && zeroOnlyX->enablePitch ? 1U : 0U,
        zeroOnlyX && zeroOnlyX->enableBacklash ? 1U : 0U);

    // PBC-3K releases only the sealed, bounded, equal-direction X pitch profile.
    // Legacy low-frequency NONZERO=LOCKED labels refer to general integration.
    DEBUG_PRINT("[PBC-3K] X_PITCH_ONLY=%u PITCH=%u BACKLASH=%u MAX_OFFSET_NM=10000 MAX_SLOPE_PPM=10000 FIRST_HOME=PHYSICAL_INDEX REHOME=REJECTED RESET=RETAIN_KNOWN UNKNOWN_RECOVERY=LOCKED GENERAL_BACKLASH=LOCKED OTHER_AXES=LOCKED\n",
        zeroOnlyX && master.m_Motion.m_CompEngine.IsXPitchOnlyConfiguration(*zeroOnlyX) ? 1U : 0U,
        zeroOnlyX && zeroOnlyX->enablePitch ? 1U : 0U,
        zeroOnlyX && zeroOnlyX->enableBacklash ? 1U : 0U);

    // PBC-3L is a separate backlash-only profile; pitch mixing stays locked.
    DEBUG_PRINT("[PBC-3L] X_BACKLASH_ONLY=%u PITCH=%u BACKLASH=%u MAX_BRANCH_NM=10000 DIRECTION=POS_PLUS_NEG_MINUS FIRST_HOME=PHYSICAL_INDEX REHOME=REJECTED RESET=RETAIN_CANCEL_TARGET UNKNOWN_RECOVERY=LOCKED GENERAL_PITCH_MIX=LOCKED OTHER_AXES=LOCKED\n",
        zeroOnlyX && master.m_Motion.m_CompEngine.IsXBacklashOnlyConfiguration(*zeroOnlyX) ? 1U : 0U,
        zeroOnlyX && zeroOnlyX->enablePitch ? 1U : 0U,
        zeroOnlyX && zeroOnlyX->enableBacklash ? 1U : 0U);

    // PBC-3M combines bounded same-direction pitch with symmetric X backlash.
    DEBUG_PRINT("[PBC-3M] X_COMBINED=%u PITCH=%u BACKLASH=%u MAX_ABS_SUM_NM=10000 PITCH_PAIR=SAME BRANCHES=SYMMETRIC FIRST_HOME=PHYSICAL_INDEX REHOME=REJECTED RESET=RETAIN_BOTH_CANCEL_TARGETS UNKNOWN_RECOVERY=LOCKED OTHER_AXES=LOCKED\n",
        zeroOnlyX && master.m_Motion.m_CompEngine.IsXCombinedConfiguration(*zeroOnlyX) ? 1U : 0U,
        zeroOnlyX && zeroOnlyX->enablePitch ? 1U : 0U,
        zeroOnlyX && zeroOnlyX->enableBacklash ? 1U : 0U);

    // PBC-3N retains paired pitch and admits distinct positive/negative backlash magnitudes.
    DEBUG_PRINT("[PBC-3N] X_ASYMMETRIC_COMBINED=%u PITCH=%u BACKLASH=%u MAX_ABS_SUM_NM=10000 PITCH_PAIR=SAME BRANCHES=ASYMMETRIC_POS_PLUS_NEG_MINUS FIRST_HOME=PHYSICAL_INDEX REHOME=REJECTED RESET=RETAIN_BOTH_CANCEL_TARGETS UNKNOWN_RECOVERY=LOCKED OTHER_AXES=LOCKED\n",
        zeroOnlyX && master.m_Motion.m_CompEngine.IsXAsymmetricCombinedConfiguration(*zeroOnlyX) ? 1U : 0U,
        zeroOnlyX && zeroOnlyX->enablePitch ? 1U : 0U,
        zeroOnlyX && zeroOnlyX->enableBacklash ? 1U : 0U);

    // PBC-3O releases signed, distinct PN pitch tables with backlash OFF/zero.
    DEBUG_PRINT("[PBC-3O] X_DIRECTIONAL_PITCH_ONLY=%u PITCH=%u BACKLASH=%u MAX_OFFSET_NM=10000 MAX_SLOPE_PPM=10000 PITCH_PAIR=DIRECTIONAL_SIGNED FIRST_HOME=PHYSICAL_INDEX REHOME=REJECTED RESET=RETAIN_CANCEL_TARGET DIRECTIONAL_MIX=LOCKED UNKNOWN_RECOVERY=LOCKED OTHER_AXES=LOCKED\n",
        zeroOnlyX && master.m_Motion.m_CompEngine.IsXDirectionalPitchOnlyConfiguration(*zeroOnlyX) ? 1U : 0U,
        zeroOnlyX && zeroOnlyX->enablePitch ? 1U : 0U,
        zeroOnlyX && zeroOnlyX->enableBacklash ? 1U : 0U);

    // PBC-3P requires an explicit directional-mixing policy and asymmetric backlash.
    DEBUG_PRINT("[PBC-3P] X_DIRECTIONAL_COMBINED=%u PITCH=%u BACKLASH=%u POLICY=EXPLICIT_ACK_REQUIRED MAX_ABS_SUM_NM=10000 MAX_SLOPE_PPM=10000 PITCH_PAIR=DIRECTIONAL_SIGNED BRANCHES=ASYMMETRIC FIRST_HOME=PHYSICAL_INDEX REHOME=REJECTED RESET=RETAIN_BOTH_CANCEL_TARGETS UNKNOWN_RECOVERY=LOCKED OTHER_AXES=LOCKED\n",
        zeroOnlyX && master.m_Motion.m_CompEngine.IsXDirectionalCombinedConfiguration(*zeroOnlyX) ? 1U : 0U,
        zeroOnlyX && zeroOnlyX->enablePitch ? 1U : 0U,
        zeroOnlyX && zeroOnlyX->enableBacklash ? 1U : 0U);

    //坐標系初始化
    master.m_NC->CoordSys.SetWCS(master.m_NC->CoordSys.GetCurrentWCSGCode(), master.m_NC);
    master.pCoordMgr = &(master.m_NC->CoordSys); // 指向目前的座標系
    master.m_Motion.LinkCoordinateManager(&(master.m_NC->CoordSys));


    //載入NC設定
    std::string NCConfigPath = GlobalConfig::GetInstance().ParameterDir + "NCConfig.txt";
    PrintCNCStartupStackCheckpoint("NC_CONFIG_ENTER");
    if (!GlobalConfig::GetInstance().LoadNCConfig(NCConfigPath, master.m_Axes, master.m_Motion, master.m_NC))
    {
        DEBUG_PRINT("LoadConfig Error！>>NCConfig.txt\n");
        return false;
    }

    // EDM18 selected ADC/simulation input. Missing/invalid configuration disables
    // the new observation channel only; no implicit fallback to simulation.
    {
        const std::string gapPath = GlobalConfig::GetInstance().ParameterDir + "EDMGapInputConfig.txt";
        EDMGapInput::Profile gapProfile{};
        EDMGapInputConfigIO::Diagnostic gapDiagnostic{};
        const bool loaded = EDMGapInputConfigIO::LoadFile(gapPath.c_str(), gapProfile, gapDiagnostic);
        const bool installed = master.m_NC->InstallEDMGapInputBeforeStart(gapProfile, &master);
        master.ConfigureGapAnalogInputBeforeStart(loaded ? gapProfile.adIndex : 0U,
            loaded ? gapProfile.expectedDeviceId : 0U);
        DEBUG_PRINT("[EDM18] event=PROFILE result=%s reason=%s key=%s line=%u source=%s AD=%u expectedDevice=%u gainMillionths=%lld offsetMv=%d calibrated=%u physicalPermit=0 discharge=0\n",
            loaded && installed ? "READY" : "DISABLED", loaded ? (installed ? "NONE" : "INSTALL") : EDMGapInputConfigIO::ErrorName(gapDiagnostic.error),
            gapDiagnostic.key, static_cast<unsigned>(gapDiagnostic.line), EDMGap::SourceName(gapProfile.source),
            static_cast<unsigned>(gapProfile.adIndex), static_cast<unsigned>(gapProfile.expectedDeviceId),
            static_cast<long long>(gapProfile.voltageGainMillionths), static_cast<int>(gapProfile.voltageOffsetMv),
            gapProfile.calibrationConfirmed ? 1U : 0U);
        if (loaded && installed)
            DEBUG_PRINT("[EDM18-CAL2] schema=%u profile=%u calibration=%u rawValid=%d:%d calRaw=%d:%d boardMv=%d:%d zeroClampBoardUv=%u gainMillionths=%lld offsetMv=%d\n",
                static_cast<unsigned>(gapProfile.schemaVersion), static_cast<unsigned>(gapProfile.profileRevision),
                static_cast<unsigned>(gapProfile.calibrationRevision), static_cast<int>(gapProfile.rawMin), static_cast<int>(gapProfile.rawMax),
                static_cast<int>(gapProfile.schemaVersion == 2U ? gapProfile.calibrationRawMin : gapProfile.rawMin),
                static_cast<int>(gapProfile.schemaVersion == 2U ? gapProfile.calibrationRawMax : gapProfile.rawMax),
                static_cast<int>(gapProfile.boardMvAtMin), static_cast<int>(gapProfile.boardMvAtMax),
                static_cast<unsigned>(gapProfile.zeroClampBoardUv), static_cast<long long>(gapProfile.voltageGainMillionths),
                static_cast<int>(gapProfile.voltageOffsetMv));
    }

    // EDM15 optional boot profile: strict parsing, no fallback and no runtime I/O.
    // Invalid/missing profile disables only G180 P5; existing CNC remains available.
    {
        const std::string voltagePath = GlobalConfig::GetInstance().ParameterDir + "EDMVoltageConfig.txt";
        EDMVoltage::Profile voltageProfile{};
        EDMVoltageConfigIO::Diagnostic voltageDiagnostic{};
        const bool voltageLoaded = EDMVoltageConfigIO::LoadFile(voltagePath.c_str(),
            voltageProfile, voltageDiagnostic);
        const bool voltageInstalled = master.m_NC->InstallEDMVoltageProfileBeforeStart(voltageProfile);
        if (!voltageLoaded || !voltageInstalled)
            DEBUG_PRINT("[EDM15] event=PROFILE result=DISABLED reason=%s key=%s line=%u detail=%s syntheticOnly=1 physicalPermit=0 discharge=0\n",
                voltageLoaded ? "INSTALL" : EDMVoltageConfigIO::ErrorName(voltageDiagnostic.error),
                voltageDiagnostic.key, static_cast<unsigned int>(voltageDiagnostic.line),
                EDMVoltage::ProfileErrorName(voltageDiagnostic.profileError));
        else
            DEBUG_PRINT("[EDM15] event=PROFILE result=READY profile=%u calibration=%u rawMin=%d rawMax=%d mvMin=%d mvMax=%d syntheticOnly=1 physicalPermit=0 discharge=0\n",
                static_cast<unsigned int>(voltageProfile.profileRevision),
                static_cast<unsigned int>(voltageProfile.calibrationRevision),
                static_cast<int>(voltageProfile.rawMin), static_cast<int>(voltageProfile.rawMax),
                static_cast<int>(voltageProfile.mvAtMin), static_cast<int>(voltageProfile.mvAtMax));
    }

    // EDM19 FIX4: all creation/validation and journal reads occur before NC starts.
    // A recovery display is explicit and remains read-only even after alarm RESET.
    {
        const std::string recipeDirectory = GlobalConfig::GetInstance().BaseDataDir + "Data\\EDM\\";
        std::unique_ptr<EDMRecipe::Catalog> recipeCatalog;
        EDMConditionStartupIO::StartupResult startup{};
        const bool recipeLoaded = EDMConditionStartupIO::LoadForStartup(recipeDirectory.c_str(), recipeCatalog, startup);
        if (!recipeLoaded)
        {
            startup.emergencyDisplay = true; startup.allowPersistence = false;
            if (startup.tableId == 0U) startup.tableId = 1U;
            try { recipeCatalog = EDMConditionDefaults::MakeEmergencyCatalog(startup.tableId); }
            catch (...) { recipeCatalog.reset(); }
        }
        const bool recipeInstalled = master.m_NC->InstallEDMRecipeCatalogBeforeStart(std::move(recipeCatalog));
        const bool recipeSelected = recipeInstalled && master.m_NC->SelectEDMRecipeAtStartupBeforeStart(startup.tableId, startup.emergencyDisplay);
        if (!recipeLoaded || !recipeInstalled || !recipeSelected || startup.emergencyDisplay)
        {
            AlarmManager::GetInstance().Trigger(AlarmManager::EDM_CONDITION_CATALOG_INVALID);
            DEBUG_PRINT("[EDM19_FIX4] event=STARTUP_COND result=RECOVERY_DISPLAY table=%u E=1 readonly=1 reason=%s file=%s key=%s physicalPermit=0 discharge=0\n",
                static_cast<unsigned>(startup.tableId), EDMRecipeConfigIO::ErrorName(startup.diagnostic.error), startup.diagnostic.file, startup.diagnostic.key);
        }
        else
        {
            if (startup.recovered || startup.generated)
                AlarmManager::GetInstance().Trigger(AlarmManager::EDM_CONDITION_STARTUP_RECOVERED);
            if (startup.saveFailed)
                AlarmManager::GetInstance().Trigger(AlarmManager::EDM_CONDITION_SELECTION_SAVE_FAILED);
            DEBUG_PRINT("[EDM19_FIX4] event=STARTUP_COND result=READY table=%u E=1 generated=%u recovered=%u saveFailed=%u mode=SHADOW_ONLY physicalPermit=0 discharge=0\n",
                static_cast<unsigned>(startup.tableId), startup.generated ? 1U : 0U, startup.recovered ? 1U : 0U, startup.saveFailed ? 1U : 0U);
        }
    }

    // EDM20 boot loads a complete bundle. NC-owner live tuning replaces only
    // the validated runtime copy. Missing/invalid startup configuration
    // raises AL4020; no default write and no physical EDM enable are performed.
    {
        const std::string processDirectory = GlobalConfig::GetInstance().ParameterDir;
        const std::string processPath = processDirectory + "EDMProcessConfig.ini";
        PrintCNCStartupStackCheckpoint("EDM20_PROCESS_LOAD_ENTER");
        RtPrintf("[EDM20_FIX1][BOOT] phase=PROCESS_BUNDLE_ENTER directory=%.400s\n", processDirectory.c_str());
        EDM20::ProcessProfile processProfile{};
        EDMProcessConfigIO::Diagnostic processDiagnostic{};
        bool processLoaded = false;
        try { processLoaded = EDMProcessConfigIO::LoadBundle(processDirectory, processProfile, processDiagnostic); }
        catch (...) { processDiagnostic.reason = "exception while loading process profile"; }
        RtPrintf("[EDM20_FIX1][BOOT] phase=PROCESS_BUNDLE_EXIT ok=%u file=%.200s line=%u key=%.64s reason=%.128s\n",
            processLoaded ? 1U : 0U, processDiagnostic.path.c_str(), static_cast<unsigned>(processDiagnostic.line),
            processDiagnostic.key.c_str(), processDiagnostic.reason.c_str());
        PrintCNCStartupStackCheckpoint("EDM20_PROCESS_LOAD_EXIT");
        RtPrintf("[EDM20_FIX1][BOOT] phase=INSTALL_ENTER axes=%u\n", static_cast<unsigned>(master.m_Axes.size()));
        const bool processInstalled = processLoaded && master.m_NC->InstallEDMProcessProfileBeforeStart(
            processProfile, master.m_Axes.empty() ? nullptr : &master.m_Axes[0], master.m_Axes.size());
        RtPrintf("[EDM20_FIX1][BOOT] phase=INSTALL_EXIT ready=%u\n", processInstalled ? 1U : 0U);
        if (!processLoaded || !processInstalled)
        {
            AlarmManager::GetInstance().Trigger(AlarmManager::EDM_PROCESS_CONFIG_INVALID);
            DEBUG_PRINT("[EDM20] event=PROFILE result=REJECT alarm=4020 file=%.160s line=%u key=%.64s reason=%.112s mode=SHADOW_ONLY physicalPermit=0 discharge=0\n",
                processDiagnostic.path.empty() ? processPath.c_str() : processDiagnostic.path.c_str(), static_cast<unsigned>(processDiagnostic.line), processDiagnostic.key.c_str(),
                processLoaded ? "INSTALL_OR_AXIS_GAINS" : processDiagnostic.reason.c_str());
        }
        else
            DEBUG_PRINT("[EDM20] event=PROFILE result=READY revision=%u machine=%u axisCount=%u gainApplied=0 mode=SHADOW_ONLY cadence=NC_10MS physicalPermit=0 discharge=0\n",
                static_cast<unsigned>(processProfile.revision), static_cast<unsigned>(processProfile.machineProfileId),
                static_cast<unsigned>(master.m_Axes.size()));
    }

    PrintCNCStartupStackCheckpoint("NC_CONFIG_EXIT");
    //載入Home設定
    std::string HomeConfigPath = GlobalConfig::GetInstance().ParameterDir + "HomeConfig.txt";
    if (!GlobalConfig::GetInstance().LoadHomeConfig(HomeConfigPath, master.m_Axes, master.m_Motion, master.m_NC))
    {
        DEBUG_PRINT("LoadConfig Error！>>HomeConfig.txt\n");
        return false; // PBC-1: bool(-1) is true, never use it for failure.
    }


    // =========================================================
    // HOME Persistence
    //
    // Snapshot / History 固定放在：
    //
    // GlobalConfig::NCDataDir
    // = D:\EtherCAT_Master_Data\Data\ (directory)
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
    PrintCNCStartupStackCheckpoint("BOOT_NC_LOAD_ENTER");
    master.m_NC->LoadProgram(Initial_NcPath);
    PrintCNCStartupStackCheckpoint("SYS_INIT_EXIT");

    return true;
}
