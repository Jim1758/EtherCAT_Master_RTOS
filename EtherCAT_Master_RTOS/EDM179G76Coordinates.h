#pragma once
#include "GCodeParser.h"
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>

// NC-only provenance for G76 P11 absolute coordinate expressions. This grammar
// grants no frame, motion or arm authority. The caller must retain the G90
// frozen-frame and physical travel gates and use the real runtime resolver.
namespace EDM179
{
    // Append K to preserve the established coordinate/literal word indices.
    // K rotates the orbit's local shape plane in degrees; it is never an axis
    // expression and does not grant coordinate or motion authority.
    constexpr unsigned WordCount = 9U;
    constexpr char Words[] = "XYZPRFBNK";
    static_assert(sizeof(Words) == WordCount + 1U, "EDM179 source word mask is complete.");
    constexpr unsigned MaximumExpressionLength = 48U;
    constexpr double MaximumOffsetMm = 0.005;

    struct Expression
    {
        std::array<char, MaximumExpressionLength + 1U> text{};
        std::uint8_t length = 0U;
    };
    struct Specification
    {
        std::array<Expression, WordCount> expressions{};
        Expression gExpression{};
        std::array<double, WordCount> literalValues{};
        std::array<double, 3U> magnitudes{};
        // 0 reads the variable directly, 1 adds, 2 subtracts. Keeping the
        // operation reproduces the resolver's signed-zero arithmetic exactly.
        std::array<std::uint8_t, 3U> operations{};
        unsigned presentMask = 0U;
        unsigned expressionMask = 0U;
        bool valid = false;
    };
    struct Row
    {
        Specification source{};
        std::array<double, 3U> programStart{};
        std::array<double, WordCount> values{};
        int pc = -1;
        bool valid = false;
    };
    static_assert(sizeof(Specification) <= 640U, "EDM179 bounded NC source provenance.");
    static_assert(sizeof(Row) <= 768U, "EDM179 bounded heap-owned NC coordinate receipt.");

    inline bool SameDouble(double a, double b) noexcept
    { return std::memcmp(&a, &b, sizeof(double)) == 0; }

    namespace Detail
    {
        inline bool Store(const std::string& input, Expression& output) noexcept
        {
            output = Expression{};
            if (input.empty() || input.size() > MaximumExpressionLength) return false;
            output.length = static_cast<std::uint8_t>(input.size());
            for (std::size_t i = 0U; i < input.size(); ++i) output.text[i] = input[i];
            return true;
        }
        inline bool SameExpression(const Expression& a, const Expression& b) noexcept
        { return a.length == b.length && a.text == b.text; }

        inline bool Unsigned(const std::string& text, std::uint32_t& value) noexcept
        {
            value = 0U;
            if (text.empty() || text.size() > 32U) return false;
            const auto maximum = (std::numeric_limits<std::uint32_t>::max)();
            for (char c : text)
            {
                if (c < '0' || c > '9') return false;
                const unsigned digit = static_cast<unsigned>(c - '0');
                if (value > maximum / 10U || (value == maximum / 10U && digit > maximum % 10U)) return false;
                value = value * 10U + digit;
            }
            return true;
        }

        inline bool Decimal(const char* begin, std::size_t length, bool signedValue, double& value) noexcept
        {
            value = 0.0;
            if (length == 0U || length > 32U) return false;
            char text[33]{};
            unsigned digits = 0U, dots = 0U;
            for (std::size_t i = 0U; i < length; ++i)
            {
                const char c = begin[i]; text[i] = c;
                if (i == 0U && signedValue && (c == '+' || c == '-')) continue;
                if (c == '.' && ++dots == 1U) continue;
                if (c < '0' || c > '9') return false;
                ++digits;
            }
            if (!digits) return false;
            char* end = nullptr;
            const double candidate = std::strtod(text, &end);
            if (end != text + length || !std::isfinite(candidate)) return false;
            value = candidate; return true;
        }

        inline bool AxisExpression(const std::string& input, unsigned axis,
            std::uint8_t& operation, double& magnitude) noexcept
        {
            operation = 0U; magnitude = 0.0;
            if (axis >= 3U || input.empty() || input.size() > MaximumExpressionLength) return false;
            std::size_t begin = 0U, end = input.size();
            if (input[0] == '[')
            {
                if (end < 2U || input[end - 1U] != ']') return false;
                ++begin; --end;
            }
            if (end - begin < 4U || input[begin] != '$' || input[begin + 1U] != '1' ||
                input[begin + 2U] != '0' || input[begin + 3U] != static_cast<char>('0' + axis)) return false;
            begin += 4U;
            if (begin == end) return true;
            const char sign = input[begin++];
            if (sign != '+' && sign != '-') return false;
            double parsed = 0.0;
            if (!Decimal(input.data() + begin, end - begin, false, parsed) ||
                parsed < 0.0 || parsed > MaximumOffsetMm) return false;
            operation = sign == '+' ? 1U : 2U; magnitude = parsed;
            return true;
        }

        inline bool SameSpecification(const Specification& a, const Specification& b) noexcept
        {
            if (!a.valid || !b.valid || a.presentMask != b.presentMask ||
                a.expressionMask != b.expressionMask || a.operations != b.operations ||
                !SameExpression(a.gExpression, b.gExpression)) return false;
            for (unsigned word = 0U; word < WordCount; ++word)
                if (!SameExpression(a.expressions[word], b.expressions[word]) ||
                    !SameDouble(a.literalValues[word], b.literalValues[word])) return false;
            for (unsigned axis = 0U; axis < 3U; ++axis)
                if (!SameDouble(a.magnitudes[axis], b.magnitudes[axis])) return false;
            return true;
        }

        inline bool Expected(const Specification& source, const std::array<double, 3U>& start,
            unsigned word, double& value) noexcept
        {
            if (word >= WordCount) return false;
            value = source.literalValues[word];
            if (word < 3U && (source.expressionMask & (1U << word)))
            {
                if (!std::isfinite(start[word])) return false;
                if (source.operations[word] == 0U) value = start[word];
                else if (source.operations[word] == 1U) value = start[word] + source.magnitudes[word];
                else if (source.operations[word] == 2U) value = start[word] - source.magnitudes[word];
                else return false;
            }
            return std::isfinite(value);
        }

        inline bool ResolvedShape(const NCParsedBlock& parsed, const NCBlock& resolved) noexcept
        {
            if (resolved.isEmpty || resolved.isBlockSkip || resolved.isGoto || resolved.gotoTarget != -1 ||
                !resolved.hasG || resolved.gCount != 1 || resolved.gCode != 76 || resolved.gCodes[0] != 76 ||
                resolved.mCount != 0) return false;
            for (unsigned i = 1U; i < NC_MAX_G_CODES_PER_BLOCK; ++i) if (resolved.gCodes[i] != 0) return false;
            for (int code : resolved.mCode) if (code != -1) return false;
            for (char word = 'A'; word <= 'Z'; ++word)
                if (parsed.has(word) != resolved.has(word) ||
                    (!resolved.has(word) && !SameDouble(resolved.val(word), 0.0))) return false;
            return true;
        }
    }

    // Parse only this expression route. A literal-only row deliberately returns
    // false so that its established literal admission remains authoritative.
    // shapeOnly contains zero for expression axes solely for shape validation;
    // it must never be dispatched or used as a resolved machining instruction.
    inline bool Parse(const NCParsedBlock& block, Specification& output, NCBlock& shapeOnly) noexcept
    {
        output = Specification{}; shapeOnly = NCBlock{};
        if (block.error != NCParseError::NONE || block.isEmpty || block.isBlockSkip ||
            !block.dependsOnMacroState || block.controlType != NCParsedControlType::NONE ||
            block.gCount != 1 || block.mCount != 0 || block.duplicateAddressMask != 0U ||
            block.assignment.prefix || !block.assignment.indexExpression.empty() ||
            !block.assignment.valueExpression.empty() || block.gotoStatement.conditional ||
            !block.gotoStatement.conditionExpression.empty() || !block.gotoStatement.targetExpression.empty()) return false;
        std::uint32_t gcode = 0U;
        if (!Detail::Unsigned(block.gExpressions[0], gcode) || gcode != 76U) return false;
        for (std::size_t i = 1U; i < block.gExpressions.size(); ++i) if (!block.gExpressions[i].empty()) return false;
        for (const auto& expression : block.mExpressions) if (!expression.empty()) return false;
        Specification source{}; NCBlock shape{};
        if (!Detail::Store(block.gExpressions[0], source.gExpression)) return false;
        shape.isEmpty = false; shape.hasG = true; shape.gCount = 1; shape.gCode = shape.gCodes[0] = 76;
        for (char address = 'A'; address <= 'Z'; ++address)
        {
            const auto& expression = block.expression(address);
            if (!block.has(address)) { if (!expression.empty()) return false; continue; }
            unsigned word = 0U; while (word < WordCount && Words[word] != address) ++word;
            if (word == WordCount || !Detail::Store(expression, source.expressions[word])) return false;
            double literal = 0.0;
            if (word < 3U && Detail::AxisExpression(expression, word, source.operations[word], source.magnitudes[word]))
                source.expressionMask |= 1U << word;
            else if (!Detail::Decimal(expression.data(), expression.size(), true, literal)) return false;
            if (address == 'K' && (literal < -180.0 || literal > 180.0)) return false;
            if (address == 'N')
            {
                std::uint32_t sequence = 0U;
                if (!Detail::Unsigned(expression, sequence)) return false;
            }
            source.literalValues[word] = literal; source.presentMask |= 1U << word;
            shape.hasParam[address - 'A'] = true; shape.param[address - 'A'] = literal;
        }
        if (!source.expressionMask || !shape.has('P') || shape.val('P') != 11.0 ||
            !shape.has('R') || shape.val('R') <= 0.0 || shape.val('R') > MaximumOffsetMm ||
            !shape.has('F') || shape.val('F') != 0.12 ||
            (shape.has('B') && (shape.val('B') != 0.0 || std::signbit(shape.val('B'))) &&
                shape.val('B') != 1.0 && shape.val('B') != 2.0 && shape.val('B') != 3.0 && shape.val('B') != 4.0)) return false;
        // EDM181 preserves an omitted B in the source/resolved receipts.
        // Only the live NC profile may select its effective B4; this parser
        // neither injects a B word nor grants a profile or modal authority.
        source.valid = true; output = source; shapeOnly = shape; return true;
    }

    inline bool Shape(const NCParsedBlock& parsed) noexcept
    { Specification source{}; NCBlock shape{}; return Parse(parsed, source, shape); }

    inline bool MatchesSource(const NCParsedBlock& parsed, const Specification& source) noexcept
    {
        Specification current{}; NCBlock shape{};
        return Parse(parsed, current, shape) && Detail::SameSpecification(current, source);
    }

    inline bool Capture(const NCParsedBlock& parsed, const NCBlock& resolved,
        const std::array<double, 3U>& programStart, int pc, Row& output) noexcept
    {
        const auto start = programStart; // Capture may reuse output.programStart.
        output = Row{};
        Specification source{}; NCBlock shape{};
        if (pc < 0 || !Parse(parsed, source, shape) || !Detail::ResolvedShape(parsed, resolved)) return false;
        for (double value : start) if (!std::isfinite(value)) return false;
        std::array<double, WordCount> values{};
        for (unsigned word = 0U; word < WordCount; ++word)
        {
            if (!(source.presentMask & (1U << word))) continue;
            double expected = 0.0;
            if (!Detail::Expected(source, start, word, expected) ||
                !SameDouble(resolved.val(Words[word]), expected)) return false;
            values[word] = resolved.val(Words[word]);
        }
        output.source = source; output.programStart = start; output.values = values;
        output.pc = pc; output.valid = true; return true;
    }

    inline bool Matches(const NCParsedBlock& parsed, const Row& row) noexcept
    {
        if (!row.valid || row.pc < 0 || !MatchesSource(parsed, row.source)) return false;
        for (double value : row.programStart) if (!std::isfinite(value)) return false;
        for (unsigned word = 0U; word < WordCount; ++word)
        {
            double expected = 0.0;
            if (row.source.presentMask & (1U << word))
            { if (!Detail::Expected(row.source, row.programStart, word, expected)) return false; }
            if (!SameDouble(row.values[word], expected)) return false;
        }
        return true;
    }

    inline bool MatchesResolved(const NCParsedBlock& parsed, const NCBlock& resolved, const Row& row) noexcept
    {
        if (!Matches(parsed, row) || !Detail::ResolvedShape(parsed, resolved)) return false;
        for (unsigned word = 0U; word < WordCount; ++word)
            if ((row.source.presentMask & (1U << word)) &&
                !SameDouble(resolved.val(Words[word]), row.values[word])) return false;
        return true;
    }
}
