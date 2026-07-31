#pragma once
#include "NC_Types.h"  
#include "NCManager.h" 

namespace GCodeHandlers 
{

    // 🌟 更改回傳型別：如果不需等待就回傳 nullptr，需要等待就回傳「專屬檢查函式」
    WaitConditionFunc Handle_G00(const NCBlock& block, NCManager* nc);
    WaitConditionFunc Handle_G53(const NCBlock& block, NCManager* nc);
    WaitConditionFunc Handle_G04(const NCBlock& block, NCManager* nc);
    WaitConditionFunc Handle_G68(const NCBlock& block, NCManager* nc);
    WaitConditionFunc Handle_G69(const NCBlock& block, NCManager* nc);
    WaitConditionFunc Handle_G168(const NCBlock& block, NCManager* nc);
    WaitConditionFunc Handle_G169(const NCBlock& block, NCManager* nc);

    WaitConditionFunc Handle_G10(const NCBlock& block, NCManager* nc);
    WaitConditionFunc Handle_G160(const NCBlock& block, NCManager* nc);

    WaitConditionFunc Handle_G51(const NCBlock& block, NCManager* nc);
    WaitConditionFunc Handle_G50(const NCBlock& block, NCManager* nc);

    WaitConditionFunc Handle_G151(const NCBlock& block, NCManager* nc);
    WaitConditionFunc Handle_G150(const NCBlock& block, NCManager* nc);

    WaitConditionFunc Handle_G15(const NCBlock& block, NCManager* nc);
    WaitConditionFunc Handle_G16(const NCBlock& block, NCManager* nc);

    WaitConditionFunc Handle_G40(const NCBlock& block, NCManager* nc);
    WaitConditionFunc Handle_G41(const NCBlock& block, NCManager* nc);
    WaitConditionFunc Handle_G42(const NCBlock& block, NCManager* nc);

    void Reset_G04(NCManager* nc); // 🌟 新增：G04 專用的重置函式
    WaitConditionFunc Handle_GCode(const NCBlock& block, NCManager* nc);


    // 🌟 新增：M 碼專屬處理器
    WaitConditionFunc Handle_MCode(const NCBlock& block, NCManager* nc);

}