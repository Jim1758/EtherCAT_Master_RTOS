#pragma once
#include "EDM140LocalAssignment.h"
#include "NC_Types.h"
#include <cstdio>
#include <cstdlib>

// NC-only provenance for an authored arithmetic motion row. The frozen
// numeric result is still checked by the existing bounded physical geometry.
namespace EDM141
{
    constexpr unsigned RowCapacity = 16U;
    constexpr unsigned WordCount = 11U;
    constexpr char Words[] = "XYZCUVIJFBN";
    struct Expression
    {
        std::array<char, EDM140::MaxExpressionLength + 1U> text{};
        std::uint8_t length = 0U;
    };
    struct Row
    {
        std::array<Expression, WordCount> expressions{};
        Expression gExpression{};
        std::array<double, WordCount> values{};
        std::array<double, 2U> localValues{};
        std::uint32_t presentMask = 0U;
        unsigned assignmentCount = 0U, definedMask = 0U, readMask = 0U;
        int pc = -1, gcode = 0;
        bool valid = false;
    };
    static_assert(sizeof(Row) <= 960U, "EDM141 bounded frozen authored motion row.");
    inline bool Store(const std::string& input, Expression& output) noexcept
    {
        output = Expression{};
        if (input.empty() || input.size() > EDM140::MaxExpressionLength) return false;
        output.length = static_cast<std::uint8_t>(input.size());
        for (std::size_t i = 0U; i < input.size(); ++i) output.text[i] = input[i];
        return true;
    }
    inline bool Same(const std::string& input, const Expression& frozen) noexcept
    {
        if (!frozen.length || frozen.length > EDM140::MaxExpressionLength || input.size() != frozen.length) return false;
        for (std::size_t i = 0U; i < input.size(); ++i) if (input[i] != frozen.text[i]) return false;
        for (std::size_t i = frozen.length; i < frozen.text.size(); ++i) if (frozen.text[i]) return false;
        return true;
    }
    inline bool Literal(const std::string& input, double& value) noexcept
    {
        if (input.empty() || input.size() > 32U) return false;
        unsigned digits = 0U, dots = 0U;
        for (std::size_t i = 0U; i < input.size(); ++i)
        {
            const char c = input[i];
            if (!i && (c == '+' || c == '-')) continue;
            if (c == '.' && ++dots == 1U) continue;
            if (c < '0' || c > '9') return false;
            ++digits;
        }
        char* end = nullptr; value = std::strtod(input.c_str(), &end);
        return digits && end == input.c_str() + input.size() && std::isfinite(value);
    }
    inline bool HasMacroSyntax(const NCParsedBlock& block) noexcept
    {
        // Do not trust the cached dependency hint after a source drift.
        for (const auto& expression : block.gExpressions)
            if (expression.find_first_of("#@$") != std::string::npos) return true;
        for (const auto& expression : block.mExpressions)
            if (expression.find_first_of("#@$") != std::string::npos) return true;
        for (const auto& expression : block.paramExpressions)
            if (expression.find_first_of("#@$") != std::string::npos) return true;
        return false;
    }
    inline bool Shape(const NCParsedBlock& block) noexcept
    {
        if (block.error != NCParseError::NONE || block.isEmpty || block.isBlockSkip ||
            block.controlType != NCParsedControlType::NONE || block.gCount != 1 || block.mCount ||
            block.duplicateAddressMask || block.assignment.prefix || !block.assignment.indexExpression.empty() ||
            !block.assignment.valueExpression.empty() || block.gotoStatement.conditional ||
            !block.gotoStatement.conditionExpression.empty() || !block.gotoStatement.targetExpression.empty()) return false;
        double code = 0.0;
        if (!Literal(block.gExpressions[0], code) || (code != 1.0 && code != 2.0 && code != 3.0)) return false;
        for (std::size_t i = 1U; i < block.gExpressions.size(); ++i) if (!block.gExpressions[i].empty()) return false;
        for (const auto& expression : block.mExpressions) if (!expression.empty()) return false;
        bool macro = false;
        for (char a = 'A'; a <= 'Z'; ++a)
        {
            const auto& expression = block.expression(a);
            if (!block.has(a)) { if (!expression.empty()) return false; continue; }
            unsigned word = 0U; while (word < WordCount && Words[word] != a) ++word;
            if (word == WordCount) return false;
            std::uint8_t reads = 0U;
            if (word < 9U)
            { EDM140::Detail::Arithmetic grammar(expression); if (!grammar.Parse(reads)) return false; }
            else { double literal = 0.0; if (!Literal(expression, literal)) return false; }
            macro = macro || reads != 0U;
        }
        return block.dependsOnMacroState == macro;
    }
    inline bool Capture(const NCParsedBlock& parsed, const NCBlock& resolved,
        const std::array<double, 2U>& locals, unsigned defined, unsigned assignments, int pc, Row& output) noexcept
    {
        output = Row{};
        if (!Shape(parsed) || pc < 0 || assignments > EDM140::AssignmentCapacity || (defined & ~3U) ||
            resolved.isEmpty || resolved.isBlockSkip || resolved.isGoto || resolved.gCount != 1 || resolved.mCount ||
            !Store(parsed.gExpressions[0], output.gExpression)) return false;
        double code = 0.0;
        if (!Literal(parsed.gExpressions[0], code) || resolved.gCode != code || resolved.gCodes[0] != code) return false;
        output.gcode = static_cast<int>(code); output.pc = pc;
        output.assignmentCount = assignments; output.definedMask = defined; output.localValues = locals;
        for (unsigned word = 0U; word < WordCount; ++word)
        {
            const char a = Words[word];
            if (parsed.has(a) != resolved.has(a)) return false;
            if (!parsed.has(a)) continue;
            std::uint8_t reads = 0U;
            if (word < 9U) { EDM140::Detail::Arithmetic grammar(parsed.expression(a)); if (!grammar.Parse(reads)) return false; }
            if ((reads & ~defined) || !Store(parsed.expression(a), output.expressions[word]) ||
                !std::isfinite(resolved.val(a)) || std::fabs(resolved.val(a)) > EDM140::MaximumAbsValue) return false;
            output.presentMask |= 1U << word; output.readMask |= reads; output.values[word] = resolved.val(a);
        }
        for (char a = 'A'; a <= 'Z'; ++a) if (parsed.has(a) != resolved.has(a)) return false;
        output.valid = true; return true;
    }
    inline bool Matches(const NCParsedBlock& parsed, const Row& frozen) noexcept
    {
        if (!frozen.valid || frozen.pc < 0 || frozen.assignmentCount > EDM140::AssignmentCapacity ||
            (frozen.definedMask & ~3U) || (frozen.readMask & ~frozen.definedMask) ||
            !Shape(parsed) || !Same(parsed.gExpressions[0], frozen.gExpression)) return false;
        unsigned mask = 0U, reads = 0U; double code = 0.0;
        if (!Literal(parsed.gExpressions[0], code) || frozen.gcode != code) return false;
        for (unsigned word = 0U; word < WordCount; ++word)
        {
            const char a = Words[word];
            if (!parsed.has(a)) continue;
            if (!Same(parsed.expression(a), frozen.expressions[word])) return false;
            std::uint8_t wordReads = 0U;
            if (word < 9U) { EDM140::Detail::Arithmetic grammar(parsed.expression(a)); if (!grammar.Parse(wordReads)) return false; }
            mask |= 1U << word; reads |= wordReads;
        }
        return mask == frozen.presentMask && reads == frozen.readMask;
    }
    inline bool MatchesResolved(const NCParsedBlock& parsed, const NCBlock& resolved, const Row& frozen) noexcept
    {
        if (!Matches(parsed, frozen) || resolved.isEmpty || resolved.isBlockSkip || resolved.isGoto ||
            !resolved.hasG || resolved.gCount != 1 || resolved.mCount || resolved.gCode != frozen.gcode ||
            resolved.gCodes[0] != frozen.gcode) return false;
        for (char a = 'A'; a <= 'Z'; ++a) if (parsed.has(a) != resolved.has(a)) return false;
        for (unsigned word = 0U; word < WordCount; ++word)
            if ((frozen.presentMask & (1U << word)) && (!std::isfinite(frozen.values[word]) ||
                resolved.val(Words[word]) != frozen.values[word])) return false;
        return true;
    }
    inline bool MakeLiteral(const NCParsedBlock& parsed, const Row& frozen, NCParsedBlock& output) noexcept
    {
        if (!Matches(parsed, frozen)) return false;
        output = parsed; output.dependsOnMacroState = false;
        for (unsigned word = 0U; word < 9U; ++word)
        {
            if (!(frozen.presentMask & (1U << word))) continue;
            char text[64]{};
            const int length = std::snprintf(text, sizeof(text), "%.17f", frozen.values[word]);
            if (length <= 0 || static_cast<std::size_t>(length) >= sizeof(text)) return false;
            std::string numeric(text); double roundtrip = 0.0;
            if (!Literal(numeric, roundtrip) || roundtrip != frozen.values[word]) return false;
            output.paramExpressions[Words[word] - 'A'] = numeric;
        }
        return true;
    }
}
