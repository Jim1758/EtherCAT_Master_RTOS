#include "GCodeParser.h"
#include "MacroParser.h"
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <limits>

namespace
{
    bool StartsWith(
        const std::string& text,
        std::size_t position,
        const char* token) noexcept
    {
        if (token == nullptr)
        {
            return false;
        }

        std::size_t i = 0;
        while (token[i] != '\0')
        {
            if (position + i >= text.size() ||
                text[position + i] != token[i])
            {
                return false;
            }
            ++i;
        }
        return true;
    }

    bool IsVariablePrefix(char value) noexcept
    {
        return value == '#' || value == '@' || value == '$';
    }

    bool IsExpressionValueStart(char value) noexcept
    {
        const unsigned char u = static_cast<unsigned char>(value);
        return
            std::isdigit(u) != 0 ||
            value == '+' ||
            value == '-' ||
            value == '.' ||
            value == '[' ||
            value == '(' ||
            IsVariablePrefix(value);
    }

    bool ExpressionDependsOnMacroState(
        const std::string& expression) noexcept
    {
        return
            expression.find_first_of("#@$") !=
            std::string::npos;
    }

    std::size_t FindTopLevelKeyword(
        const std::string& text,
        const char* keyword,
        std::size_t start) noexcept
    {
        char stack[64] = { 0 };
        int depth = 0;

        for (std::size_t i = start; i < text.size(); ++i)
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

            if (depth == 0 && StartsWith(text, i, keyword))
            {
                return i;
            }
        }

        return std::string::npos;
    }

    std::size_t FindAssignmentSeparator(
        const std::string& text,
        std::size_t start) noexcept
    {
        char stack[64] = { 0 };
        int depth = 0;

        for (std::size_t i = start; i < text.size(); ++i)
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

            // ==、>=、<=、!= 都是運算子，不是指派分隔符。
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

    bool TryParseStrictIntegerLiteral(
        const std::string& expression,
        int& value) noexcept
    {
        if (expression.empty())
        {
            return false;
        }

        char* end = nullptr;
        const double parsed = std::strtod(expression.c_str(), &end);
        if (end == expression.c_str() ||
            end == nullptr ||
            *end != '\0' ||
            !std::isfinite(parsed))
        {
            return false;
        }

        const double rounded = std::round(parsed);
        if (std::fabs(parsed - rounded) > 1.0e-9 ||
            rounded < static_cast<double>(std::numeric_limits<int>::min()) ||
            rounded > static_cast<double>(std::numeric_limits<int>::max()))
        {
            return false;
        }

        value = static_cast<int>(rounded);
        return true;
    }

    // =========================================================================
    // Stage NC-0.2G.2 - Standard NC parenthesis-comment compatibility
    //
    // MacroParser::CleanExpression() intentionally preserves '(' / ')' because
    // Macro expressions use them for grouping and functions (for example
    // X(#1+2), SIN(30), IF(...)).  NC source files also use the standard
    // Fanuc-style comment form '(COMMENT)'.  This lexer removes only
    // parenthesis groups that are comments, while preserving expression
    // parentheses.
    // =========================================================================
    bool IsExpressionParenthesisStart(
        const std::string& normalizedPrefix) noexcept
    {
        if (normalizedPrefix.empty())
        {
            return false;
        }

        const char previous = normalizedPrefix.back();
        const unsigned char u =
            static_cast<unsigned char>(previous);

        // An address or expression keyword immediately followed by '(' starts
        // an expression: X(...), IF(...), SIN(...), GOTO(...), etc.
        if (std::isalpha(u) != 0)
        {
            return true;
        }

        switch (previous)
        {
        case '#':
        case '@':
        case '$':
        case '=':
        case '+':
        case '-':
        case '*':
        case '/':
        case '%':
        case '^':
        case '[':
        case '(':
        case ',':
        case '<':
        case '>':
        case '!':
        case '&':
        case '|':
            return true;

        default:
            return false;
        }
    }

    bool CleanGCodeSourceLine(
        const std::string& input,
        std::string& output)
    {
        // Preserve the established handling of whitespace, case conversion,
        // ';' end-of-line comments and C-style /* ... */ comments.
        const std::string normalized =
            MacroParser::CleanExpression(input);

        output.clear();
        output.reserve(normalized.size());

        int expressionParenDepth = 0;
        int bracketDepth = 0;
        int commentDepth = 0;

        for (char c : normalized)
        {
            if (commentDepth > 0)
            {
                if (c == '(')
                {
                    ++commentDepth;
                }
                else if (c == ')')
                {
                    --commentDepth;
                }
                continue;
            }

            if (c == '[')
            {
                ++bracketDepth;
                output += c;
                continue;
            }

            if (c == ']')
            {
                if (bracketDepth > 0)
                {
                    --bracketDepth;
                }
                output += c;
                continue;
            }

            if (c == '(')
            {
                const bool expressionParenthesis =
                    expressionParenDepth > 0 ||
                    bracketDepth > 0 ||
                    IsExpressionParenthesisStart(output);

                if (expressionParenthesis)
                {
                    ++expressionParenDepth;
                    output += c;
                }
                else
                {
                    commentDepth = 1;
                }
                continue;
            }

            if (c == ')')
            {
                if (expressionParenDepth <= 0)
                {
                    // Unmatched ')' is neither a valid expression delimiter
                    // nor a complete NC comment.
                    return false;
                }

                --expressionParenDepth;
                output += c;
                continue;
            }

            output += c;
        }

        // An unterminated '(COMMENT' is a syntax error.  Expression delimiter
        // balance remains the responsibility of the normal parser below.
        return commentDepth == 0;
    }
}

std::size_t GCodeParser::GetExpressionKeywordLength(
    const std::string& text,
    std::size_t position) noexcept
{
    struct Keyword
    {
        const char* text;
        std::size_t length;
        bool requiresDelimiter;
    };

    static const Keyword keywords[] =
    {
        { "ROUND", 5u, true },
        { "SQRT", 4u, true },
        { "ATAN", 4u, true },
        { "SIN", 3u, true },
        { "COS", 3u, true },
        { "TAN", 3u, true },
        { "ABS", 3u, true },
        { "FIX", 3u, true },
        { "AND", 3u, false },
        { "NOT", 3u, false },
        { "OR", 2u, false },
        { "EQ", 2u, false },
        { "NE", 2u, false },
        { "GT", 2u, false },
        { "LT", 2u, false },
        { "GE", 2u, false },
        { "LE", 2u, false }
    };

    for (const Keyword& keyword : keywords)
    {
        if (!StartsWith(text, position, keyword.text))
        {
            continue;
        }

        if (keyword.requiresDelimiter)
        {
            const std::size_t next = position + keyword.length;
            if (next >= text.size() ||
                (text[next] != '(' && text[next] != '['))
            {
                continue;
            }
        }

        return keyword.length;
    }

    return 0u;
}

NCParsedBlock GCodeParser::ParseLine(
    const std::string& line) const
{
    NCParsedBlock block{};
    std::string clean;
    if (!CleanGCodeSourceLine(line, clean))
    {
        block.error = NCParseError::UNBALANCED_DELIMITER;
        return block;
    }

    if (clean.empty())
    {
        return block;
    }

    std::size_t position = 0u;
    if (clean[position] == '/')
    {
        block.isBlockSkip = true;
        ++position;
        if (position >= clean.size())
        {
            return block;
        }
    }

    // 可選的前置 N Sequence Word。先抽出後再辨識 IF/GOTO，支援：
    // N100 IF[#1 GT 0] GOTO 200
    if (position + 1u < clean.size() &&
        clean[position] == 'N' &&
        IsExpressionValueStart(clean[position + 1u]))
    {
        const std::size_t valueStart = position + 1u;
        std::size_t i = valueStart;
        char stack[64] = { 0 };
        int depth = 0;

        for (; i < clean.size(); ++i)
        {
            const char c = clean[i];
            if (c == '(' || c == '[')
            {
                if (depth >= static_cast<int>(sizeof(stack)))
                {
                    block.error = NCParseError::UNBALANCED_DELIMITER;
                    return block;
                }
                stack[depth++] = c;
                continue;
            }

            if (c == ')' || c == ']')
            {
                if (depth <= 0)
                {
                    block.error = NCParseError::UNBALANCED_DELIMITER;
                    return block;
                }

                const char open = stack[depth - 1];
                if ((c == ')' && open != '(') ||
                    (c == ']' && open != '['))
                {
                    block.error = NCParseError::UNBALANCED_DELIMITER;
                    return block;
                }
                --depth;
                continue;
            }

            if (depth == 0 &&
                i > valueStart &&
                IsVariablePrefix(c))
            {
                // 支援 N100#1=5：N100 是 Label，後面的 #1=5 是
                // 同一行 Commit Statement。複合動態 N 請寫成 N[#1+#2]。
                break;
            }

            if (depth == 0 &&
                std::isalpha(static_cast<unsigned char>(c)) != 0)
            {
                const std::size_t keywordLength =
                    GetExpressionKeywordLength(clean, i);
                if (keywordLength > 0u)
                {
                    i += keywordLength - 1u;
                    continue;
                }
                break;
            }
        }

        if (depth != 0 || i == valueStart)
        {
            block.error =
                depth != 0
                ? NCParseError::UNBALANCED_DELIMITER
                : NCParseError::MISSING_ADDRESS_VALUE;
            return block;
        }

        block.hasParam[static_cast<std::size_t>('N' - 'A')] = true;
        block.paramExpressions[static_cast<std::size_t>('N' - 'A')] =
            clean.substr(valueStart, i - valueStart);
        block.dependsOnMacroState =
            ExpressionDependsOnMacroState(
                block.paramExpressions[static_cast<std::size_t>('N' - 'A')]);
        block.isEmpty = false;
        position = i;
    }

    if (position >= clean.size())
    {
        return block;
    }

    const std::string statement = clean.substr(position);

    // 純 Macro Assignment。只保存語法，不執行。
    if (!statement.empty() && IsVariablePrefix(statement[0]))
    {
        const std::size_t separator =
            FindAssignmentSeparator(statement, 1u);
        if (separator != std::string::npos)
        {
            const std::string indexExpression =
                statement.substr(1u, separator - 1u);
            const std::string valueExpression =
                statement.substr(separator + 1u);

            if (indexExpression.empty() || valueExpression.empty())
            {
                block.error = NCParseError::INVALID_ASSIGNMENT;
                return block;
            }

            block.controlType = NCParsedControlType::ASSIGNMENT;
            block.dependsOnMacroState = true;
            block.assignment.prefix = statement[0];
            block.assignment.indexExpression = indexExpression;
            block.assignment.valueExpression = valueExpression;
            block.isEmpty = false;
            return block;
        }
    }

    // IF[...]GOTO... 或 GOTO...。條件與目標都延後到 Commit 時才求值。
    if (StartsWith(statement, 0u, "IF"))
    {
        const std::size_t gotoPosition =
            FindTopLevelKeyword(statement, "GOTO", 2u);
        if (gotoPosition == std::string::npos)
        {
            block.error = NCParseError::INVALID_GOTO;
            return block;
        }

        const std::string condition =
            statement.substr(2u, gotoPosition - 2u);
        const std::string target =
            statement.substr(gotoPosition + 4u);

        if (condition.empty() || target.empty())
        {
            block.error = NCParseError::INVALID_GOTO;
            return block;
        }

        block.controlType = NCParsedControlType::GOTO;
        block.dependsOnMacroState = true;
        block.gotoStatement.conditional = true;
        block.gotoStatement.conditionExpression = condition;
        block.gotoStatement.targetExpression = target;
        block.isEmpty = false;
        return block;
    }

    if (StartsWith(statement, 0u, "GOTO"))
    {
        const std::string target = statement.substr(4u);
        if (target.empty())
        {
            block.error = NCParseError::INVALID_GOTO;
            return block;
        }

        block.controlType = NCParsedControlType::GOTO;
        block.dependsOnMacroState = true;
        block.gotoStatement.conditional = false;
        block.gotoStatement.targetExpression = target;
        block.isEmpty = false;
        return block;
    }

    if (FindTopLevelKeyword(statement, "GOTO", 0u) != std::string::npos)
    {
        block.error = NCParseError::INVALID_GOTO;
        return block;
    }

    char currentAddress = 0;
    std::string currentValue;
    char delimiterStack[64] = { 0 };
    int delimiterDepth = 0;

    auto processWord = [&]() -> bool
    {
        if (currentAddress == 0 && currentValue.empty())
        {
            return true;
        }

        if (currentAddress == 0)
        {
            block.error = NCParseError::ORPHAN_VALUE;
            return false;
        }

        if (currentValue.empty())
        {
            block.error = NCParseError::MISSING_ADDRESS_VALUE;
            return false;
        }

        block.isEmpty = false;
        block.dependsOnMacroState =
            block.dependsOnMacroState ||
            ExpressionDependsOnMacroState(currentValue);

        if (currentAddress == 'G')
        {
            if (block.gCount < NC_MAX_G_CODES_PER_BLOCK)
            {
                block.gExpressions[static_cast<std::size_t>(block.gCount)] =
                    currentValue;
            }
            ++block.gCount;
        }
        else if (currentAddress == 'M')
        {
            if (block.mCount < NC_MAX_M_CODES_PER_BLOCK)
            {
                block.mExpressions[static_cast<std::size_t>(block.mCount)] =
                    currentValue;
            }
            ++block.mCount;
        }
        else if (currentAddress >= 'A' && currentAddress <= 'Z')
        {
            const std::size_t index =
                static_cast<std::size_t>(currentAddress - 'A');
            block.hasParam[index] = true;
            block.paramExpressions[index] = currentValue;
        }

        currentAddress = 0;
        currentValue.clear();
        return true;
    };

    for (std::size_t i = position; i < clean.size(); ++i)
    {
        const char c = clean[i];

        if (c == '(' || c == '[')
        {
            if (currentAddress == 0 ||
                delimiterDepth >= static_cast<int>(sizeof(delimiterStack)))
            {
                block.error =
                    currentAddress == 0
                    ? NCParseError::ORPHAN_VALUE
                    : NCParseError::UNBALANCED_DELIMITER;
                return block;
            }

            delimiterStack[delimiterDepth++] = c;
            currentValue += c;
            continue;
        }

        if (c == ')' || c == ']')
        {
            if (currentAddress == 0 || delimiterDepth <= 0)
            {
                block.error = NCParseError::UNBALANCED_DELIMITER;
                return block;
            }

            const char open = delimiterStack[delimiterDepth - 1];
            if ((c == ')' && open != '(') ||
                (c == ']' && open != '['))
            {
                block.error = NCParseError::UNBALANCED_DELIMITER;
                return block;
            }

            --delimiterDepth;
            currentValue += c;
            continue;
        }

        if (delimiterDepth == 0 &&
            std::isalpha(static_cast<unsigned char>(c)) != 0)
        {
            if (currentAddress != 0)
            {
                const std::size_t keywordLength =
                    GetExpressionKeywordLength(clean, i);
                if (keywordLength > 0u)
                {
                    currentValue.append(clean, i, keywordLength);
                    i += keywordLength - 1u;
                    continue;
                }
            }

            if (!processWord())
            {
                return block;
            }

            currentAddress = static_cast<char>(
                std::toupper(static_cast<unsigned char>(c)));
            continue;
        }

        if (currentAddress != 0)
        {
            currentValue += c;
            continue;
        }

        // '%' 是常見程式邊界符號，保持相容並略過。
        if (c == '%')
        {
            continue;
        }

        block.error = NCParseError::ORPHAN_VALUE;
        return block;
    }

    if (delimiterDepth != 0)
    {
        block.error = NCParseError::UNBALANCED_DELIMITER;
        return block;
    }

    processWord();
    return block;
}

bool GCodeParser::TryExtractLiteralSequenceNumber(
    const NCParsedBlock& block,
    int& sequenceNumber)
{
    if (block.error != NCParseError::NONE || !block.has('N'))
    {
        return false;
    }

    return TryParseStrictIntegerLiteral(
        block.expression('N'),
        sequenceNumber);
}

bool GCodeParser::TryExtractLiteralSequenceNumber(
    const std::string& line,
    int& sequenceNumber)
{
    const GCodeParser parser;
    const NCParsedBlock block = parser.ParseLine(line);
    return TryExtractLiteralSequenceNumber(
        block,
        sequenceNumber);
}
