#pragma once

#ifndef EDM_GLOBAL_CONFIG_H
#define EDM_GLOBAL_CONFIG_H

#include <string>
#include <vector>

#include "CompensationEngine.h"


// ============================================================
// Forward Declarations
//
// GlobalConfig.h ぃ钡 include MotionCore.h / EtherCatMaster.h /
// NCManager.h磷 Header ぇ丁Θ碻吏 include
// 痷タ惠璶Ч俱よパ GlobalConfig.cpp include
// ============================================================

struct AxisContext;

class MotionCore;
class EtherCatMaster;
class NCManager;


// ============================================================
// ╰参家Α
// ============================================================

enum class SystemMode
{
    UNKNOWN_MODE,
    EDM_SINKER_MODE,
    EXAMPLE_MODE
};


// ============================================================
// GlobalConfig
// ============================================================

class GlobalConfig
{
public:
    // --------------------------------------------------------
    // Singleton
    // --------------------------------------------------------

    static GlobalConfig& GetInstance()
    {
        static GlobalConfig instance;

        return instance;
    }


    // ========================================================
    // 办砞﹚
    // ========================================================

    SystemMode systemMode =
        SystemMode::UNKNOWN_MODE;

    int Debug_ShowMessage =
        0;

    double Global_Override =
        1.0;


    // ========================================================
    // ╰参禸计
    //
    // LoadAxisConfig() Θ穝
    // LoadHomeConfig() 单ㄤ把计更瑈祘ノ
    // ========================================================

    int System_axisCount =
        0;


    // ========================================================
    // Data Paths
    // ========================================================

    std::string BaseDataDir =
        "D:\\EtherCAT_Master_Data\\";

    std::string ParameterDir =
        "D:\\EtherCAT_Master_Data\\Data\\Parameter\\";

    std::string NCProgramDir =
        BaseDataDir +
        "NC_Program\\";

    std::string NCMacroProgramDir =
        BaseDataDir +
        "NC_Macro\\";

    // HOME Snapshot / HOME History ㄏノヘ魁
    //
    // D:\EtherCAT_Master_Data\Data\
    //
    // HomeSnapshot.txt
    // HomeSnapshot.bak.txt
    // HomeHistory.csv
    std::string NCDataDir =
        BaseDataDir +
        "Data\\";

    std::string PLC_Dir =
        "D:\\EtherCAT_Master_Data\\PLC\\";


    // ========================================================
    // General Config
    // ========================================================

    void LoadFromFile(
        const std::string& filePath);


    // ========================================================
    // Axis Config
    // ========================================================

    bool LoadAxisConfig(
        const std::string& filePath,
        std::vector<AxisContext>& axis,
        MotionCore& motion);


    // ========================================================
    // PID Config
    // ========================================================

    bool LoadPIDConfig(
        const std::string& filePath,
        std::vector<AxisContext>& axis,
        MotionCore& motion);


    // ========================================================
    // Speed Config
    // ========================================================

    bool LoadSpeedConfig(
        const std::string& filePath,
        std::vector<AxisContext>& axis,
        MotionCore& motion);


    // ========================================================
    // Pitch Compensation Table
    // ========================================================

    static bool LoadPitchTable(
        const std::string& filePath,
        CompensationEngine& compEngine,
        bool isPositive);


    // ========================================================
    // System Parameter Initialization
    // ========================================================

    bool InitSystemParameters(
        EtherCatMaster& master);


    // ========================================================
    // NC Config
    // ========================================================

    bool LoadNCConfig(
        const std::string& filePath,
        std::vector<AxisContext>& axis,
        MotionCore& motion,
        NCManager* nc);


    // ========================================================
    // HOME Config
    //
    // 硂ゲ斗籔 GlobalConfig.cpp Ч璓
    // ========================================================

    bool LoadHomeConfig(
        const std::string& filePath,
        std::vector<AxisContext>& axis,
        MotionCore& motion,
        NCManager* nc);


private:
    // ========================================================
    // Singleton Protection
    // ========================================================

    GlobalConfig()
    {
    }

    GlobalConfig(
        const GlobalConfig&) =
        delete;

    GlobalConfig& operator=(
        const GlobalConfig&) =
        delete;
};


// ============================================================
// Debug Print
// ============================================================

#define DEBUG_PRINT(fmt, ...) \
    do \
    { \
        if (GlobalConfig::GetInstance().Debug_ShowMessage == 1) \
        { \
            RtPrintf(fmt, ##__VA_ARGS__); \
        } \
    } while (0)


#endif // EDM_GLOBAL_CONFIG_H
