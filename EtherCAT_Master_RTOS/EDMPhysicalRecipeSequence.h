#pragma once

#include "GCodeParser.h"
#include "EDMRecipe.h"
#include <cstdint>
#include <limits>
#include <string>

// EDM77 explicitly opts into stopped recipe sequencing with G180 P61/P62 Q1.
// EDM79 adds Q2 for a separately supervised, pinned recipe plan. One sequence
// must use the same mode throughout; this grammar grants neither mode authority.
// This is a pure syntax admission gate, not a recipe executor or motion gate.
// Every fixture must independently capture a valid recipe after the preceding
// fixture has completed its existing Stop/Disarm/fresh released-origin proof.
namespace EDM77
{
    constexpr unsigned MaximumPhysicalLines = 128U;
    constexpr unsigned MaximumFixtures = 8U; // Including the current fixture.
    constexpr unsigned MaximumOperationsPerGap = 16U;

    enum class Error : std::uint8_t
    {
        None, InvalidCurrentFixture, MissingTerminal, InvalidBlock,
        TooManyFixtures, TooManyOperations, MissingMutation, DanglingOperation,
        MixedSequenceMode
    };

    struct Result
    {
        bool valid = false;
        Error error = Error::None;
        unsigned futureFixtures = 0U;
        unsigned physicalLineCount = 0U;
        unsigned terminalOffset = 0U;
        unsigned errorOffset = 0U;
    };

    namespace Detail
    {
        // Match G39's authored unsigned-literal contract without conversion,
        // allocation, locale, rounding, or macro evaluation.
        inline bool UnsignedLiteral(const std::string& text,
            std::uint32_t maximum, std::uint32_t& value) noexcept
        {
            value = 0U;
            if (text.empty() || text.size() > 32U) return false;
            for (char c : text)
            {
                if (c < '0' || c > '9') return false;
                const std::uint32_t digit = static_cast<std::uint32_t>(c - '0');
                if (value > maximum / 10U ||
                    (value == maximum / 10U && digit > maximum % 10U)) return false;
                value = value * 10U + digit;
            }
            return true;
        }

        inline bool EqualLiteral(const std::string& text, std::uint32_t expected) noexcept
        {
            std::uint32_t value = 0U;
            return UnsignedLiteral(text, expected, value) && value == expected;
        }

        inline bool Clean(const NCParsedBlock& block) noexcept
        {
            // These flags must be checked before isEmpty: the real parser
            // marks malformed comments and a bare block-skip slash as empty.
            if (block.error != NCParseError::NONE || block.isBlockSkip ||
                block.dependsOnMacroState || block.controlType != NCParsedControlType::NONE ||
                block.duplicateAddressMask != 0U || block.gCount < 0 || block.mCount < 0 ||
                block.gCount > NC_MAX_G_CODES_PER_BLOCK || block.mCount > NC_MAX_M_CODES_PER_BLOCK)
                return false;
            std::uint32_t sequence = 0U;
            return !block.has('N') || UnsignedLiteral(block.expression('N'),
                (std::numeric_limits<std::uint32_t>::max)(), sequence);
        }

        inline bool OnlyAddresses(const NCParsedBlock& block, const char* allowed) noexcept
        {
            for (char letter = 'A'; letter <= 'Z'; ++letter)
            {
                if (!block.has(letter)) continue;
                bool found = false;
                for (const char* p = allowed; *p != '\0'; ++p)
                    if (*p == letter) { found = true; break; }
                if (!found) return false;
            }
            return true;
        }

        inline bool Empty(const NCParsedBlock& block) noexcept
        {
            return block.isEmpty && block.gCount == 0 && block.mCount == 0 &&
                OnlyAddresses(block, "");
        }

        inline bool Terminal(const NCParsedBlock& block) noexcept
        {
            return !block.isEmpty && block.gCount == 0 && block.mCount == 1 &&
                EqualLiteral(block.mExpressions[0], 30U) && OnlyAddresses(block, "N");
        }

        inline bool AllowedField(std::uint32_t field) noexcept
        {
            return field == 5U || field == 6U || field == 7U || field == 8U ||
                field == 9U || field == 21U || field == 22U;
        }

        inline bool Operation(const NCParsedBlock& block, bool& mutation) noexcept
        {
            mutation = false;
            if (block.isEmpty || block.mCount != 0) return false;
            std::uint32_t value = 0U;
            if (block.gCount == 0 && block.has('E') && OnlyAddresses(block, "NE") &&
                UnsignedLiteral(block.expression('E'), static_cast<std::uint32_t>(EDMRecipe::RowsPerTable), value) &&
                value != 0U)
            {
                mutation = true;
                return true;
            }
            if (block.gCount != 1 || !block.has('P')) return false;
            if (EqualLiteral(block.gExpressions[0], 38U))
            {
                if (!OnlyAddresses(block, "NP") ||
                    !UnsignedLiteral(block.expression('P'), EDMRecipe::MaximumConditionTableId, value) ||
                    value == 0U) return false;
                mutation = true;
                return true;
            }
            if (!EqualLiteral(block.gExpressions[0], 39U) || !block.has('Q') ||
                !OnlyAddresses(block, "NPQR") ||
                !UnsignedLiteral(block.expression('P'), 65535U, value) || !AllowedField(value)) return false;
            std::uint32_t operation = 0U;
            if (!UnsignedLiteral(block.expression('Q'), 4U, operation) ||
                block.has('R') != (operation >= 1U && operation <= 3U)) return false;
            if (block.has('R'))
            {
                const std::string& argument = block.expression('R');
                if (argument.empty() || argument.size() > 32U || argument.find('\0') != std::string::npos)
                    return false;
                if (operation == 1U)
                {
                    if (!UnsignedLiteral(argument, 65535U, value) || value == 0U) return false;
                }
                else if (operation == 2U)
                {
                    if (argument != "1" && argument != "-1") return false;
                }
                else
                {
                    EDMRecipe::Value fixed = 0LL;
                    if (!EDMRecipe::ParseValue(argument.c_str(), fixed)) return false;
                }
            }
            mutation = operation != 0U;
            return true;
        }
    }

    inline unsigned SequenceMode(const NCParsedBlock& block) noexcept
    {
        if (!Detail::Clean(block) || block.isEmpty || block.gCount != 1 || block.mCount != 0 ||
            !Detail::EqualLiteral(block.gExpressions[0], 180U) ||
            !block.has('P') || !block.has('Q') || !Detail::OnlyAddresses(block, "NPQ") ||
            (!Detail::EqualLiteral(block.expression('Q'), 1U) &&
                !Detail::EqualLiteral(block.expression('Q'), 2U))) return 0U;
        std::uint32_t profile = 0U, mode = 0U;
        if (!Detail::UnsignedLiteral(block.expression('P'), 62U, profile) ||
            (profile != 61U && profile != 62U) ||
            !Detail::UnsignedLiteral(block.expression('Q'), 2U, mode)) return 0U;
        return static_cast<unsigned>(mode);
    }

    inline bool IsSequenceFixture(const NCParsedBlock& block) noexcept
    { return SequenceMode(block) != 0U; }

    inline bool NumericFixtureMatches(const NCParsedBlock& parsed, const NCBlock& block) noexcept
    {
        if (!IsSequenceFixture(parsed) || block.isEmpty || block.isBlockSkip || block.isGoto ||
            !block.hasG || block.gCode != 180 || block.gCount != 1 || block.gCodes[0] != 180 ||
            block.mCount != 0) return false;
        for (char letter = 'A'; letter <= 'Z'; ++letter)
        {
            if (block.has(letter) != parsed.has(letter)) return false;
            if (!block.has(letter)) continue;
            std::uint32_t value = 0U;
            if (!Detail::UnsignedLiteral(parsed.expression(letter),
                (std::numeric_limits<std::uint32_t>::max)(), value) ||
                block.val(letter) != static_cast<double>(value)) return false;
        }
        return true;
    }

    // get(offset) returns a stable parsed source block or nullptr when missing.
    // Offsets 1..128 count every physical suffix line, including comments. The
    // current fixture is not counted in that suffix bound. No source is copied,
    // resolved, mutated, or executed. The caller supplies an allocation-free,
    // non-throwing getter and performs admission before any physical Arm.
    template<class Getter>
    Result ValidateSuffix(const NCParsedBlock& current, Getter get) noexcept
    {
        Result result{};
        const unsigned mode = SequenceMode(current);
        if (mode == 0U)
        {
            result.error = Error::InvalidCurrentFixture;
            return result;
        }
        unsigned operations = 0U;
        bool hasMutation = false;
        for (unsigned offset = 1U; offset <= MaximumPhysicalLines; ++offset)
        {
            const NCParsedBlock* block = get(offset);
            result.errorOffset = offset;
            if (block == nullptr)
            {
                result.error = Error::MissingTerminal;
                return result;
            }
            result.physicalLineCount = offset;
            if (!Detail::Clean(*block))
            {
                result.error = Error::InvalidBlock;
                return result;
            }
            if (Detail::Empty(*block)) continue;
            if (Detail::Terminal(*block))
            {
                if (operations != 0U)
                {
                    result.error = Error::DanglingOperation;
                    return result;
                }
                result.valid = true;
                result.error = Error::None;
                result.terminalOffset = offset;
                result.errorOffset = 0U;
                return result;
            }
            if (IsSequenceFixture(*block))
            {
                if (SequenceMode(*block) != mode)
                {
                    result.error = Error::MixedSequenceMode;
                    return result;
                }
                if (result.futureFixtures + 1U >= MaximumFixtures)
                {
                    result.error = Error::TooManyFixtures;
                    return result;
                }
                if (!hasMutation)
                {
                    result.error = Error::MissingMutation;
                    return result;
                }
                ++result.futureFixtures;
                operations = 0U;
                hasMutation = false;
                continue;
            }
            bool mutation = false;
            if (!Detail::Operation(*block, mutation))
            {
                result.error = Error::InvalidBlock;
                return result;
            }
            if (++operations > MaximumOperationsPerGap)
            {
                result.error = Error::TooManyOperations;
                return result;
            }
            hasMutation = hasMutation || mutation;
        }
        result.error = Error::MissingTerminal;
        return result;
    }
}
