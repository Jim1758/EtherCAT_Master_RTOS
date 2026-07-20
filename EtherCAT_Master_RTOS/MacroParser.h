#pragma once
#include "MacroEngine.h"
#include <string>
#include <cmath>

class MacroParser {
public:
    MacroParser(MacroEngine& engine);

    // 🌟 核心功能 1：清除字串中的註解 (; 與 /* */) 與空白
    static std::string CleanExpression(const std::string& input);

    // 🌟 核心功能 2：計算數學/邏輯算式，回傳結果
    // 支援: +, -, *, /, ==, !=, >, <, &&(AND), ||(OR), SIN(), COS() 等
    double Evaluate(const std::string& expression);

    // 🌟 核心功能 3：執行變數指派 (例如 "#1 = 100 + 50")
    bool ExecuteAssignment(const std::string& expression);

private:
    MacroEngine& m_engine;
    const char* m_p; // 當前解析的字元指標

    // 遞迴下降解析法 (由低優先權到高優先權)
    double parseExpression(); // 處理 ||, OR
    double parseLogicalAnd(); // 處理 &&, AND
    double parseEquality();   // 處理 ==, !=
    double parseRelational(); // 處理 >, <, >=, <=
    double parseAddSub();     // 處理 +, -
    double parseMulDiv();     // 處理 *, /
    double parseUnary();      // 處理正負號 (+, -), 以及 NOT (!)
    double parsePrimary();    // 處理數字、變數 (#, @, $)、括號 ()、函數 (SIN, COS...)

    void skipWhitespace();
    bool match(const char* str); // 比對並吃掉指定字串
};