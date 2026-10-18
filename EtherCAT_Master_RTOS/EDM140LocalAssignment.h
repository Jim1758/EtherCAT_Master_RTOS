#pragma once

#include "GCodeParser.h"
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

// EDM140: a syntax-only, fixed-size description of the explicitly admitted
// local bookkeeping rows. It neither reads macros nor grants motion authority.
namespace EDM140
{
    constexpr std::size_t MaxExpressionLength = 64U;
    constexpr unsigned MaxExpressionDepth = 8U;
    constexpr unsigned AssignmentCapacity = 8U;
    constexpr double MaximumAbsValue = 1000000.0;

    struct Specification
    {
        std::array<char, MaxExpressionLength + 1U> expression{};
        std::uint8_t length = 0U;
        std::uint8_t target = 0U;
        std::uint8_t readMask = 0U; // bit 0: #1; bit 1: #2.
    };
    static_assert(sizeof(Specification) <= 72U,
        "EDM140 local syntax snapshot must remain fixed and bounded.");

    namespace Detail
    {
        // This recognizes only arithmetic syntax. The existing MacroParser is
        // still the sole evaluator at the actual stopped program boundary.
        class Arithmetic
        {
        public:
            explicit Arithmetic(const std::string& input) noexcept : text(input) {}

            bool Parse(std::uint8_t& reads) noexcept
            {
                reads = 0U;
                if (text.empty() || text.size() > MaxExpressionLength ||
                    !Expression(0U) || position != text.size()) return false;
                reads = readMask;
                return true;
            }

        private:
            bool Expression(unsigned depth) noexcept
            {
                if (!Term(depth)) return false;
                while (position < text.size() &&
                    (text[position] == '+' || text[position] == '-'))
                {
                    ++position;
                    if (!Term(depth)) return false;
                }
                return true;
            }

            bool Term(unsigned depth) noexcept
            {
                if (!Factor(depth)) return false;
                while (position < text.size() &&
                    (text[position] == '*' || text[position] == '/'))
                {
                    ++position;
                    if (!Factor(depth)) return false;
                }
                return true;
            }

            bool Factor(unsigned depth) noexcept
            {
                if (position >= text.size() || depth > MaxExpressionDepth) return false;
                const char current = text[position];
                if (current == '+' || current == '-')
                {
                    ++position;
                    return Factor(depth + 1U);
                }
                if (current == '[' || current == '(')
                {
                    ++position;
                    const char close = current == '[' ? ']' : ')';
                    if (!Expression(depth + 1U) || position >= text.size() ||
                        text[position] != close) return false;
                    ++position;
                    return true;
                }
                if (current == '#')
                {
                    ++position;
                    if (position >= text.size() ||
                        (text[position] != '1' && text[position] != '2')) return false;
                    readMask |= static_cast<std::uint8_t>(1U << (text[position] - '1'));
                    ++position;
                    return true;
                }
                bool digit = false;
                while (position < text.size() && text[position] >= '0' && text[position] <= '9')
                { digit = true; ++position; }
                if (position < text.size() && text[position] == '.')
                {
                    ++position;
                    while (position < text.size() && text[position] >= '0' && text[position] <= '9')
                    { digit = true; ++position; }
                }
                return digit;
            }

            const std::string& text;
            std::size_t position = 0U;
            std::uint8_t readMask = 0U;
        };
    }

    inline bool Parse(const NCParsedBlock& block, Specification& output) noexcept
    {
        output = Specification{};
        if (block.error != NCParseError::NONE || block.isEmpty || block.isBlockSkip ||
            !block.dependsOnMacroState || block.controlType != NCParsedControlType::ASSIGNMENT ||
            block.gCount != 0 || block.mCount != 0 || block.duplicateAddressMask != 0U ||
            block.assignment.prefix != '#' || block.assignment.indexExpression.size() != 1U ||
            (block.assignment.indexExpression[0] != '1' && block.assignment.indexExpression[0] != '2') ||
            block.gotoStatement.conditional || !block.gotoStatement.conditionExpression.empty() ||
            !block.gotoStatement.targetExpression.empty()) return false;
        // No mixed address, G/M expression, label or hidden unused word can
        // accompany a row that is exempted from the frozen geometry guard.
        for (const auto& expression : block.gExpressions) if (!expression.empty()) return false;
        for (const auto& expression : block.mExpressions) if (!expression.empty()) return false;
        for (std::size_t word = 0U; word < block.hasParam.size(); ++word)
            if (block.hasParam[word] || !block.paramExpressions[word].empty()) return false;
        std::uint8_t reads = 0U;
        Detail::Arithmetic grammar(block.assignment.valueExpression);
        if (!grammar.Parse(reads)) return false;
        output.target = static_cast<std::uint8_t>(block.assignment.indexExpression[0] - '0');
        output.length = static_cast<std::uint8_t>(block.assignment.valueExpression.size());
        output.readMask = reads;
        for (std::size_t i = 0U; i < output.length; ++i)
            output.expression[i] = block.assignment.valueExpression[i];
        return true;
    }

    inline bool Same(const Specification& left, const Specification& right) noexcept
    {
        if (left.target < 1U || left.target > 2U || !left.length ||
            left.length > MaxExpressionLength || (left.readMask & ~3U) != 0U ||
            left.target != right.target || left.length != right.length ||
            left.readMask != right.readMask) return false;
        return left.expression == right.expression;
    }

    inline bool Matches(const NCParsedBlock& block, const Specification& expected) noexcept
    {
        Specification actual{};
        return Parse(block, actual) && Same(actual, expected);
    }

    inline bool ValidResult(const Specification& expected,
        char prefix, int index, double value) noexcept
    {
        return expected.target >= 1U && expected.target <= 2U &&
            prefix == '#' && index == expected.target &&
            std::isfinite(value) && std::fabs(value) <= MaximumAbsValue;
    }
}
