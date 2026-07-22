#pragma once
#include "NC_Types.h"  
#include "NCManager.h" 

namespace GCodeHandlers 
{

    // 🌟 更改回傳型別：如果不需等待就回傳 nullptr，需要等待就回傳「專屬檢查函式」
    WaitConditionFunc Handle_G00(const NCBlock& block, NCManager* nc);
    WaitConditionFunc Handle_G04(const NCBlock& block, NCManager* nc);
    WaitConditionFunc Handle_GCode(const NCBlock& block, NCManager* nc);


    // 🌟 新增：M 碼專屬處理器
    WaitConditionFunc Handle_MCode(const NCBlock& block, NCManager* nc);

}