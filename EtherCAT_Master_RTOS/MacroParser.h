#pragma once
#include "MacroEngine.h"
#include <cstdint>
#include <string>

enum class MacroEvalError : std::uint8_t
{
    NONE = 0,
    EMPTY_EXPRESSION,
    SYNTAX_ERROR,
    VARIABLE_INDEX_OUT_OF_RANGE,
    DIVIDE_BY_ZERO,
    DOMAIN_ERROR,
    NON_FINITE_RESULT,
    COMPLEXITY_LIMIT
};

class MacroParser
{
public:
    explicit MacroParser(MacroEngine& engine);

    // 清除 ; / /* */ 註解、空白並轉大寫。
    static std::string CleanExpression(const std::string& input);

    // 相容舊 API：錯誤時回傳 0.0。新 NC 執行管線應使用 TryEvaluate。
    double Evaluate(const std::string& expression);

    // 完整消耗運算式並回報錯誤；只讀 MacroEngine，不寫入變數。
    bool TryEvaluate(
        const std::string& expression,
        double& result,
        MacroEvalError* error = nullptr);

    // 相容舊 API：只有外部明確呼叫時才會 Commit 指派。
    // GCodeParser 不再呼叫此函式。
    bool ExecuteAssignment(const std::string& expression);

private:
    MacroEngine& m_engine;
    const char* m_p = nullptr;
    MacroEvalError m_error = MacroEvalError::NONE;
    std::uint32_t m_recursionDepth = 0;

    double parseExpression();
    double parseLogicalAnd();
    double parseEquality();
    double parseRelational();
    double parseAddSub();
    double parseMulDiv();
    double parseUnary();
    double parsePrimary();

    bool parseDelimitedExpression(double& value);
    bool match(const char* token);
    void skipWhitespace();
    void setError(MacroEvalError error) noexcept;
};
