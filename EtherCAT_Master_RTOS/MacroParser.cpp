#include "MacroParser.h"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>

namespace
{
    constexpr double PI = 3.14159265358979323846;
    constexpr std::size_t MAX_MACRO_EXPRESSION_LENGTH = 4096u;
    constexpr std::uint32_t MAX_MACRO_RECURSION_DEPTH = 128u;

    // BASE55: every evaluated operand must be finite before an enclosing
    // comparison, logical operator or function can turn it into a finite value.
    // Preserve the first error and the existing TryEvaluate failure result.
    double RequireFiniteMacroValue(double value, MacroEvalError& error) noexcept
    {
        if (error != MacroEvalError::NONE)
        {
            return 0.0;
        }
        if (!std::isfinite(value))
        {
            error = MacroEvalError::NON_FINITE_RESULT;
            return 0.0;
        }
        return value;
    }

    bool IsVariablePrefix(char value) noexcept
    {
        return value == '#' || value == '@' || value == '$';
    }

    std::size_t FindAssignmentSeparator(
        const std::string& text) noexcept
    {
        int depth = 0;
        char stack[64] = { 0 };

        for (std::size_t i = 0; i < text.size(); ++i)
        {
            const char c = text[i];
            if (c == '(' || c == '[')
            {
                if (depth < static_cast<int>(sizeof(stack)))
                {
                    stack[depth++] = c;
                }
                continue;
            }
            if (c == ')' || c == ']')
            {
                if (depth > 0)
                {
                    --depth;
                }
                continue;
            }
            if (depth != 0 || c != '=')
            {
                continue;
            }

            const char previous = i > 0 ? text[i - 1] : '\0';
            const char next = i + 1 < text.size() ? text[i + 1] : '\0';
            if (previous == '=' || previous == '>' ||
                previous == '<' || previous == '!' ||
                next == '=')
            {
                continue;
            }
            return i;
        }
        return std::string::npos;
    }

    bool IsIntegralIndex(double value, int& index) noexcept
    {
        if (!std::isfinite(value))
        {
            return false;
        }

        const double rounded = std::round(value);
        if (std::fabs(value - rounded) > 1.0e-9 ||
            rounded < static_cast<double>(std::numeric_limits<int>::min()) ||
            rounded > static_cast<double>(std::numeric_limits<int>::max()))
        {
            return false;
        }

        index = static_cast<int>(rounded);
        return true;
    }
}

MacroParser::MacroParser(MacroEngine& engine)
    : m_engine(engine)
{
}

std::string MacroParser::CleanExpression(const std::string& input)
{
    std::string result;
    result.reserve(input.size());
    bool inComment = false;

    for (std::size_t i = 0; i < input.size(); ++i)
    {
        const char c = input[i];
        if (!inComment)
        {
            if (c == '/' && i + 1u < input.size() && input[i + 1u] == '*')
            {
                inComment = true;
                ++i;
                continue;
            }

            if (c == ';')
            {
                break;
            }

            const unsigned char u = static_cast<unsigned char>(c);
            if (std::isspace(u) == 0)
            {
                result += static_cast<char>(std::toupper(u));
            }
        }
        else if (c == '*' && i + 1u < input.size() && input[i + 1u] == '/')
        {
            inComment = false;
            ++i;
        }
    }

    return result;
}

bool MacroParser::ExecuteAssignment(const std::string& expression)
{
    const std::string clean = CleanExpression(expression);
    if (clean.size() < 4u || !IsVariablePrefix(clean[0]))
    {
        return false;
    }

    const std::size_t separator = FindAssignmentSeparator(clean);
    if (separator == std::string::npos || separator <= 1u ||
        separator + 1u >= clean.size())
    {
        return false;
    }

    double indexValue = 0.0;
    double assignedValue = 0.0;
    if (!TryEvaluate(clean.substr(1u, separator - 1u), indexValue) ||
        !TryEvaluate(clean.substr(separator + 1u), assignedValue))
    {
        return false;
    }

    int index = 0;
    if (!IsIntegralIndex(indexValue, index))
    {
        return false;
    }

    const char prefix = clean[0];
    const bool valid =
        MacroVariableRules::IsValidIndex(prefix, index);
    if (!valid)
    {
        return false;
    }

    m_engine.SetVar(prefix, index, assignedValue);
    return true;
}

double MacroParser::Evaluate(const std::string& expression)
{
    double result = 0.0;
    if (!TryEvaluate(expression, result))
    {
        return 0.0;
    }
    return result;
}

bool MacroParser::TryEvaluate(
    const std::string& expression,
    double& result,
    MacroEvalError* error)
{
    const std::string clean = CleanExpression(expression);
    if (clean.empty())
    {
        if (error != nullptr)
        {
            *error = MacroEvalError::EMPTY_EXPRESSION;
        }
        result = 0.0;
        return false;
    }

    if (clean.size() > MAX_MACRO_EXPRESSION_LENGTH)
    {
        if (error != nullptr)
        {
            *error = MacroEvalError::COMPLEXITY_LIMIT;
        }
        result = 0.0;
        return false;
    }

    m_p = clean.c_str();
    m_error = MacroEvalError::NONE;
    m_recursionDepth = 0u;
    const double value = parseExpression();
    skipWhitespace();

    if (m_error == MacroEvalError::NONE && *m_p != '\0')
    {
        setError(MacroEvalError::SYNTAX_ERROR);
    }

    if (m_error == MacroEvalError::NONE && !std::isfinite(value))
    {
        setError(MacroEvalError::NON_FINITE_RESULT);
    }

    if (error != nullptr)
    {
        *error = m_error;
    }

    if (m_error != MacroEvalError::NONE)
    {
        result = 0.0;
        return false;
    }

    result = value;
    return true;
}

void MacroParser::setError(MacroEvalError error) noexcept
{
    if (m_error == MacroEvalError::NONE)
    {
        m_error = error;
    }
}

void MacroParser::skipWhitespace()
{
    while (m_p != nullptr &&
        std::isspace(static_cast<unsigned char>(*m_p)) != 0)
    {
        ++m_p;
    }
}

bool MacroParser::match(const char* token)
{
    if (m_error != MacroEvalError::NONE || token == nullptr)
    {
        return false;
    }

    skipWhitespace();
    const std::size_t length = std::strlen(token);
    if (std::strncmp(m_p, token, length) != 0)
    {
        return false;
    }

    m_p += length;
    return true;
}

double MacroParser::parseExpression()
{
    double left = parseLogicalAnd();
    while (m_error == MacroEvalError::NONE)
    {
        bool hasOperator = false;
        if (match("||"))
        {
            hasOperator = true;
        }
        else if (match("OR"))
        {
            hasOperator = true;
        }

        if (!hasOperator)
        {
            break;
        }

        // 一定要解析 RHS；不能用 C++ short-circuit，否則 Parser 指標不會前進。
        const double right = parseLogicalAnd();
        left = (left != 0.0 || right != 0.0) ? 1.0 : 0.0;
    }
    return left;
}

double MacroParser::parseLogicalAnd()
{
    double left = parseEquality();
    while (m_error == MacroEvalError::NONE)
    {
        bool hasOperator = false;
        if (match("&&"))
        {
            hasOperator = true;
        }
        else if (match("AND"))
        {
            hasOperator = true;
        }

        if (!hasOperator)
        {
            break;
        }

        const double right = parseEquality();
        left = (left != 0.0 && right != 0.0) ? 1.0 : 0.0;
    }
    return left;
}

double MacroParser::parseEquality()
{
    double left = parseRelational();
    while (m_error == MacroEvalError::NONE)
    {
        if (match("==") || match("EQ"))
        {
            const double right = parseRelational();
            left = left == right ? 1.0 : 0.0;
        }
        else if (match("!=") || match("NE"))
        {
            const double right = parseRelational();
            left = left != right ? 1.0 : 0.0;
        }
        else
        {
            break;
        }
    }
    return left;
}

double MacroParser::parseRelational()
{
    double left = parseAddSub();
    while (m_error == MacroEvalError::NONE)
    {
        if (match(">=") || match("GE"))
        {
            const double right = parseAddSub();
            left = left >= right ? 1.0 : 0.0;
        }
        else if (match("<=") || match("LE"))
        {
            const double right = parseAddSub();
            left = left <= right ? 1.0 : 0.0;
        }
        else if (match(">") || match("GT"))
        {
            const double right = parseAddSub();
            left = left > right ? 1.0 : 0.0;
        }
        else if (match("<") || match("LT"))
        {
            const double right = parseAddSub();
            left = left < right ? 1.0 : 0.0;
        }
        else
        {
            break;
        }
    }
    return left;
}

double MacroParser::parseAddSub()
{
    double left = parseMulDiv();
    while (m_error == MacroEvalError::NONE)
    {
        if (match("+"))
        {
            left += parseMulDiv();
        }
        else if (match("-"))
        {
            left -= parseMulDiv();
        }
        else
        {
            break;
        }

        // Check each step, not only the final result of a chained expression.
        left = RequireFiniteMacroValue(left, m_error);
    }
    return left;
}

double MacroParser::parseMulDiv()
{
    double left = parseUnary();
    while (m_error == MacroEvalError::NONE)
    {
        if (match("*"))
        {
            left *= parseUnary();
        }
        else if (match("/"))
        {
            const double right = parseUnary();
            if (right == 0.0)
            {
                setError(MacroEvalError::DIVIDE_BY_ZERO);
                return 0.0;
            }
            left /= right;
        }
        else
        {
            break;
        }

        // Check each step, not only the final result of a chained expression.
        left = RequireFiniteMacroValue(left, m_error);
    }
    return left;
}

double MacroParser::parseUnary()
{
    if (m_recursionDepth >= MAX_MACRO_RECURSION_DEPTH)
    {
        setError(MacroEvalError::COMPLEXITY_LIMIT);
        return 0.0;
    }

    ++m_recursionDepth;
    double result = 0.0;

    if (match("+"))
    {
        result = parseUnary();
    }
    else if (match("-"))
    {
        result = -parseUnary();
    }
    else if (match("!") || match("NOT"))
    {
        result = parseUnary() == 0.0 ? 1.0 : 0.0;
    }
    else
    {
        result = parsePrimary();
    }

    --m_recursionDepth;
    // Covers literals, #/@/$ reads, grouped values, function results and unary
    // operands. Recursive unary evaluation also prevents NOT from hiding NaN/Inf.
    return RequireFiniteMacroValue(result, m_error);
}

bool MacroParser::parseDelimitedExpression(double& value)
{
    char closing = '\0';
    if (match("("))
    {
        closing = ')';
    }
    else if (match("["))
    {
        closing = ']';
    }
    else
    {
        setError(MacroEvalError::SYNTAX_ERROR);
        return false;
    }

    value = parseExpression();
    if (m_error != MacroEvalError::NONE)
    {
        return false;
    }

    const char token[2] = { closing, '\0' };
    if (!match(token))
    {
        setError(MacroEvalError::SYNTAX_ERROR);
        return false;
    }
    return true;
}

double MacroParser::parsePrimary()
{
    if (m_error != MacroEvalError::NONE)
    {
        return 0.0;
    }

    skipWhitespace();

    if (*m_p == '(' || *m_p == '[')
    {
        double value = 0.0;
        parseDelimitedExpression(value);
        return value;
    }

    if (IsVariablePrefix(*m_p))
    {
        const char prefix = *m_p++;
        const double indexValue = parseUnary();
        if (m_error != MacroEvalError::NONE)
        {
            return 0.0;
        }

        int index = 0;
        if (!IsIntegralIndex(indexValue, index))
        {
            setError(MacroEvalError::SYNTAX_ERROR);
            return 0.0;
        }

        if (!MacroVariableRules::IsValidIndex(prefix, index))
        {
            setError(MacroEvalError::VARIABLE_INDEX_OUT_OF_RANGE);
            return 0.0;
        }

        return m_engine.GetVar(prefix, index);
    }

    enum class Function
    {
        NONE,
        SIN,
        COS,
        TAN,
        ATAN,
        SQRT,
        ABS,
        FIX,
        ROUND
    };

    Function function = Function::NONE;
    if (match("ATAN")) function = Function::ATAN;
    else if (match("ROUND")) function = Function::ROUND;
    else if (match("SQRT")) function = Function::SQRT;
    else if (match("SIN")) function = Function::SIN;
    else if (match("COS")) function = Function::COS;
    else if (match("TAN")) function = Function::TAN;
    else if (match("ABS")) function = Function::ABS;
    else if (match("FIX")) function = Function::FIX;

    if (function != Function::NONE)
    {
        double argument = 0.0;
        if (!parseDelimitedExpression(argument))
        {
            return 0.0;
        }

        switch (function)
        {
        case Function::SIN:
            return std::sin(argument * PI / 180.0);
        case Function::COS:
            return std::cos(argument * PI / 180.0);
        case Function::TAN:
            return std::tan(argument * PI / 180.0);
        case Function::ATAN:
            return std::atan(argument) * 180.0 / PI;
        case Function::SQRT:
            if (argument < 0.0)
            {
                setError(MacroEvalError::DOMAIN_ERROR);
                return 0.0;
            }
            return std::sqrt(argument);
        case Function::ABS:
            return std::fabs(argument);
        case Function::FIX:
            return std::floor(argument);
        case Function::ROUND:
            return std::round(argument);
        case Function::NONE:
        default:
            break;
        }
    }

    char* next = nullptr;
    const double value = std::strtod(m_p, &next);
    if (next == m_p)
    {
        setError(MacroEvalError::SYNTAX_ERROR);
        return 0.0;
    }

    m_p = next;
    return value;
}
