#pragma once
#include <string>
#include <vector>



enum class SystemMode //╰参家Α
{
    UNKNOWN_MODE,//ゼ家Α
    EDM_SINKER_MODE,//EDM 繨縤家Α
    EXAMPLE_MODE//絛ㄒ家Α

};


struct AxisContext;
class MotionCore;


class GlobalConfig
{
public:
    //  虫ㄒ家Αみ玂靡╰参穦Τ GlobalConfig 龟ㄒ
    static GlobalConfig& GetInstance() 
    {
        static GlobalConfig instance;
        return instance;
    }

    // ==========================================
    //  硂柑┮Τ办跑计
    // ==========================================
    SystemMode systemMode = SystemMode::UNKNOWN_MODE;
    int Debug_ShowMessage = 0;       // 箇砞闽超
    double Global_Override = 1.0; // ㄒ办秈倒瞯


    std::string BaseDataDir = "D:\\EtherCAT_Master_Data\\";
    std::string NCProgramDir = BaseDataDir + "NC_Program\\";
    std::string NCMacroProgramDir = BaseDataDir + "NC_Macro\\";
    std::string NCDataDir = BaseDataDir + "Data\\";
    // ==========================================
    // 更ㄧ计璽砫弄 txt 郎恶骸跑计
    // ==========================================
    void LoadFromFile(const std::string& filePath);
    bool LoadAxisConfig(const std::string& filePath, std::vector<AxisContext>& axes, MotionCore& motion);//更禸把计

private:
    // 玛篶窽ゎ new GlobalConfig()
    GlobalConfig() {}

    // 玛ī絋玂荡癸Τだō
    GlobalConfig(const GlobalConfig&) = delete;
    GlobalConfig& operator=(const GlobalConfig&) = delete;
};

#define DEBUG_PRINT(fmt, ...) \
    do { \
        if (GlobalConfig::GetInstance().Debug_ShowMessage == 1) { \
            RtPrintf(fmt, ##__VA_ARGS__); \
        } \
    } while(0)