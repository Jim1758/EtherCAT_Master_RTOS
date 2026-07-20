#pragma once
#include "NC_Types.h"
#include "MacroParser.h"
#include <string>

class GCodeParser {
public:
    GCodeParser(MacroParser& macroParser);

    // 🌟 核心功能：讀入一行字串，回傳打包好的 NCBlock
    NCBlock ParseLine(const std::string& line);

private:
    MacroParser& m_macroParser;

    // 判斷某個字母是不是數學函數的開頭 (避免把 SIN 的 S 當成主軸 S 碼)
    bool IsMathKeyword(const std::string& str, size_t pos);
};