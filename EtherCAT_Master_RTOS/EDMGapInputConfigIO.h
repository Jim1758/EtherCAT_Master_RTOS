#pragma once

#include "EDMGapInput.h"
#include <cstddef>
#include <cstring>
#include <fstream>

// EDM18 boot-thread-only file loading. Never call LoadFile from NC/RT service.
// Parsing has bounded fixed storage and commits the complete profile only once
// every required key and all semantic constraints have passed. No defaults.
namespace EDMGapInputConfigIO
{
    constexpr std::size_t MaximumFileBytes = 4096U;
    constexpr std::size_t MaximumLineBytes = 256U;
    constexpr std::size_t MaximumLines = 128U;
    constexpr std::size_t Schema1KeyCount = 22U;
    constexpr std::size_t KeyCount = 25U;

    enum class Error : std::uint8_t
    {
        None, FileOpen, FileRead, FileTooLarge, TooManyLines, LineTooLong,
        InvalidCharacter, Format, UnknownKey, DuplicateKey, MissingKey, Number, Mode, Profile
    };
    struct Diagnostic
    {
        Error error = Error::None;
        std::size_t line = 0U;
        char key[40]{};
    };

    inline const char* ErrorName(Error error) noexcept
    {
        switch (error)
        {
        case Error::None: return "NONE";
        case Error::FileOpen: return "FILE_OPEN";
        case Error::FileRead: return "FILE_READ";
        case Error::FileTooLarge: return "FILE_TOO_LARGE";
        case Error::TooManyLines: return "TOO_MANY_LINES";
        case Error::LineTooLong: return "LINE_TOO_LONG";
        case Error::InvalidCharacter: return "INVALID_CHARACTER";
        case Error::Format: return "FORMAT";
        case Error::UnknownKey: return "UNKNOWN_KEY";
        case Error::DuplicateKey: return "DUPLICATE_KEY";
        case Error::MissingKey: return "MISSING_KEY";
        case Error::Number: return "NUMBER";
        case Error::Mode: return "MODE";
        case Error::Profile: return "PROFILE";
        }
        return "UNKNOWN";
    }

    namespace Detail
    {
        inline const char* Key(std::size_t index) noexcept
        {
            static const char* const names[KeyCount] = {
                "SchemaVersion", "ProfileRevision", "Source", "AdIndex", "ExpectedDeviceId", "CalibrationRevision",
                "CalibrationConfirmed", "RawMin", "RawMax", "BoardMvAtMin", "BoardMvAtMax", "VoltageGain",
                "VoltageOffsetMv", "MinMv", "MaxMv", "LowEnterMv", "LowExitMv", "HighExitMv", "HighEnterMv",
                "MaxAgeMs", "DwellMs", "SimulationMv", "CalibrationRawMin", "CalibrationRawMax", "ZeroClampBoardUv"
            };
            return index < KeyCount ? names[index] : "";
        }
        inline bool Fail(Diagnostic& d, Error error, std::size_t line, const char* key = "") noexcept
        {
            d.error = error;
            d.line = line;
            std::size_t i = 0U;
            for (; key[i] != '\0' && i + 1U < sizeof(d.key); ++i) d.key[i] = key[i];
            d.key[i] = '\0';
            return false;
        }
        inline char* Trim(char* text) noexcept
        {
            while (*text == ' ' || *text == '\t') ++text;
            std::size_t length = std::strlen(text);
            while (length > 0U && (text[length - 1U] == ' ' || text[length - 1U] == '\t'))
                text[--length] = '\0';
            return text;
        }
        inline bool Decimal(const char* token, bool signedRaw, std::uint64_t maximum,
            std::int64_t& value) noexcept
        {
            bool negative = false;
            if (*token == '-' && signedRaw) { negative = true; ++token; }
            if (*token == '\0') return false;
            const std::uint64_t limit = negative ? 2147483648ULL : maximum;
            std::uint64_t magnitude = 0ULL;
            for (; *token != '\0'; ++token)
            {
                if (*token < '0' || *token > '9') return false;
                const std::uint64_t digit = static_cast<std::uint64_t>(*token - '0');
                if (magnitude > (limit - digit) / 10ULL) return false;
                magnitude = magnitude * 10ULL + digit;
            }
            value = negative ? -static_cast<std::int64_t>(magnitude) : static_cast<std::int64_t>(magnitude);
            return true;
        }
        inline bool Gain(const char* text, std::int64_t& result) noexcept
        {
            std::uint64_t whole = 0U, fraction = 0U, digits = 0U;
            if (*text < '0' || *text > '9') return false;
            while (*text >= '0' && *text <= '9') {
                whole = whole * 10U + static_cast<unsigned>(*text++ - '0');
                if (whole > 1000U) return false;
            }
            if (*text == '.') {
                ++text;
                while (*text >= '0' && *text <= '9') {
                    if (++digits > 6U) return false;
                    fraction = fraction * 10U + static_cast<unsigned>(*text++ - '0');
                }
                if (digits == 0U) return false;
            }
            if (*text != '\0') return false;
            while (digits++ < 6U) fraction *= 10U;
            result = static_cast<std::int64_t>(whole * 1000000U + fraction);
            return result >= 1LL && result <= 1000000000LL;
        }
        inline void Assign(EDMGapInput::Profile& p, std::size_t index, std::int64_t n) noexcept
        {
            switch (index)
            {
            case 0U: p.schemaVersion = static_cast<std::uint32_t>(n); break;
            case 1U: p.profileRevision = static_cast<std::uint32_t>(n); break;
            case 3U: p.adIndex = static_cast<std::uint32_t>(n); break;
            case 4U: p.expectedDeviceId = static_cast<std::uint32_t>(n); break;
            case 5U: p.calibrationRevision = static_cast<std::uint32_t>(n); break;
            case 6U: p.calibrationConfirmed = n == 1; break;
            case 7U: p.rawMin = static_cast<std::int32_t>(n); break;
            case 8U: p.rawMax = static_cast<std::int32_t>(n); break;
            case 9U: p.boardMvAtMin = static_cast<std::int32_t>(n); break;
            case 10U: p.boardMvAtMax = static_cast<std::int32_t>(n); break;
            case 12U: p.voltageOffsetMv = static_cast<std::int32_t>(n); break;
            case 13U: p.gap.minMv = static_cast<std::int32_t>(n); break;
            case 14U: p.gap.maxMv = static_cast<std::int32_t>(n); break;
            case 15U: p.gap.lowEnterMv = static_cast<std::int32_t>(n); break;
            case 16U: p.gap.lowExitMv = static_cast<std::int32_t>(n); break;
            case 17U: p.gap.highExitMv = static_cast<std::int32_t>(n); break;
            case 18U: p.gap.highEnterMv = static_cast<std::int32_t>(n); break;
            case 19U: p.gap.maxAgeMs = static_cast<std::uint64_t>(n); break;
            case 20U: p.gap.dwellMs = static_cast<std::uint64_t>(n); break;
            case 21U: p.simulationMv = static_cast<std::int32_t>(n); break;
            case 22U: p.calibrationRawMin = static_cast<std::int32_t>(n); break;
            case 23U: p.calibrationRawMax = static_cast<std::int32_t>(n); break;
            case 24U: p.zeroClampBoardUv = static_cast<std::uint32_t>(n); break;
            default: break;
            }
        }

    }

    inline bool Parse(const char* bytes, std::size_t length,
        EDMGapInput::Profile& result, Diagnostic& diagnostic) noexcept
    {
        diagnostic = Diagnostic{};
        if (length > MaximumFileBytes) return Detail::Fail(diagnostic, Error::FileTooLarge, 0U);
        if (bytes == nullptr && length != 0U) return Detail::Fail(diagnostic, Error::Format, 0U);
        EDMGapInput::Profile candidate{};
        std::uint32_t seen = 0U;
        std::size_t keyLines[KeyCount]{};
        std::size_t offset = 0U, lineNumber = 0U;
        if (length >= 3U && static_cast<unsigned char>(bytes[0]) == 0xEFU &&
            static_cast<unsigned char>(bytes[1]) == 0xBBU && static_cast<unsigned char>(bytes[2]) == 0xBFU) offset = 3U;
        while (offset < length)
        {
            ++lineNumber;
            if (lineNumber > MaximumLines) return Detail::Fail(diagnostic, Error::TooManyLines, lineNumber);
            char line[MaximumLineBytes + 1U]{};
            std::size_t used = 0U;
            while (offset < length && bytes[offset] != '\n' && bytes[offset] != '\r')
            {
                if (used >= MaximumLineBytes) return Detail::Fail(diagnostic, Error::LineTooLong, lineNumber);
                if (bytes[offset] == '\0') return Detail::Fail(diagnostic, Error::InvalidCharacter, lineNumber);
                line[used++] = bytes[offset++];
            }
            if (offset < length && bytes[offset] == '\r') ++offset;
            if (offset < length && bytes[offset] == '\n') ++offset;
            // UTF-8 comments are permitted, while keys/values are ASCII only.
            for (std::size_t i = 0U; i < used; ++i)
            {
                if (line[i] == '#' || line[i] == ';' || (line[i] == '/' && i + 1U < used && line[i + 1U] == '/'))
                { line[i] = '\0'; break; }
                const unsigned char c = static_cast<unsigned char>(line[i]);
                if ((c < 32U && c != 9U) || c >= 127U)
                    return Detail::Fail(diagnostic, Error::InvalidCharacter, lineNumber);
            }
            char* key = Detail::Trim(line);
            if (*key == '\0') continue;
            char* equal = std::strchr(key, '=');
            if (equal == nullptr) return Detail::Fail(diagnostic, Error::Format, lineNumber);
            *equal = '\0';
            key = Detail::Trim(key);
            char* value = Detail::Trim(equal + 1);
            if (*key == '\0' || *value == '\0' || std::strchr(value, '=') != nullptr)
                return Detail::Fail(diagnostic, Error::Format, lineNumber, key);
            std::size_t index = 0U;
            for (; index < KeyCount && std::strcmp(key, Detail::Key(index)) != 0; ++index) {}
            if (index == KeyCount) return Detail::Fail(diagnostic, Error::UnknownKey, lineNumber, key);
            const std::uint32_t bit = 1U << static_cast<unsigned int>(index);
            if ((seen & bit) != 0U) return Detail::Fail(diagnostic, Error::DuplicateKey, lineNumber, key);
            seen |= bit;
            keyLines[index] = lineNumber;
            if (index == 2U)
            {
                if (std::strcmp(value, "SIMULATION") == 0) candidate.source = EDMGap::Source::SIMULATED;
                else if (std::strcmp(value, "AD") == 0) candidate.source = EDMGap::Source::PHYSICAL;
                else return Detail::Fail(diagnostic, Error::Mode, lineNumber, key);
            }
            else if (index == 11U)
            {
                if (!Detail::Gain(value, candidate.voltageGainMillionths))
                    return Detail::Fail(diagnostic, Error::Number, lineNumber, key);
            }
            else
            {
                const bool signedValue = index == 7U || index == 8U || index == 12U ||
                    index == 22U || index == 23U;
                std::int64_t numeric = 0;
                const std::uint64_t limit = index <= 5U ? 4294967295ULL : 2147483647ULL;
                if (!Detail::Decimal(value, signedValue, limit, numeric) ||
                    (index == 6U && numeric != 0 && numeric != 1))
                    return Detail::Fail(diagnostic, Error::Number, lineNumber, key);
                Detail::Assign(candidate, index, numeric);
            }
        }
        // Defer schema-specific key checks until all lines have been read so
        // SchemaVersion may appear anywhere without changing acceptance.
        for (std::size_t i = 0U; i < Schema1KeyCount; ++i)
            if ((seen & (1U << static_cast<unsigned int>(i))) == 0U)
                return Detail::Fail(diagnostic, Error::MissingKey, 0U, Detail::Key(i));
        if (candidate.schemaVersion != 1U && candidate.schemaVersion != 2U)
            return Detail::Fail(diagnostic, Error::Profile, 0U, "PROFILE");
        for (std::size_t i = Schema1KeyCount; i < KeyCount; ++i)
        {
            const bool present = (seen & (1U << static_cast<unsigned int>(i))) != 0U;
            if (candidate.schemaVersion == 1U && present)
                return Detail::Fail(diagnostic, Error::UnknownKey, keyLines[i], Detail::Key(i));
            if (candidate.schemaVersion == 2U && !present)
                return Detail::Fail(diagnostic, Error::MissingKey, 0U, Detail::Key(i));
        }
        if (!EDMGapInput::Validate(candidate))
            return Detail::Fail(diagnostic, Error::Profile, 0U, "PROFILE");

        result = candidate;
        return true;
    }

    inline bool LoadFile(const char* path, EDMGapInput::Profile& result, Diagnostic& diagnostic) noexcept
    {
        diagnostic = Diagnostic{};
        if (path == nullptr || *path == '\0') return Detail::Fail(diagnostic, Error::FileOpen, 0U);
        try
        {
            std::ifstream input(path, std::ios::binary);
            if (!input.is_open()) return Detail::Fail(diagnostic, Error::FileOpen, 0U);
            char bytes[MaximumFileBytes + 1U]{};
            input.read(bytes, static_cast<std::streamsize>(sizeof(bytes)));
            const std::streamsize count = input.gcount();
            if (input.bad() || (input.fail() && !input.eof()) || count < 0)
                return Detail::Fail(diagnostic, Error::FileRead, 0U);
            if (static_cast<std::size_t>(count) > MaximumFileBytes)
                return Detail::Fail(diagnostic, Error::FileTooLarge, 0U);
            return Parse(bytes, static_cast<std::size_t>(count), result, diagnostic);
        }
        catch (...) { return Detail::Fail(diagnostic, Error::FileRead, 0U); }
    }
}
