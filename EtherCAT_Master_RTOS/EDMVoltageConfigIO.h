#pragma once

#include "EDMVoltage.h"
#include <cstddef>
#include <cstring>
#include <fstream>

// EDM15 boot-thread-only file loading. Never call LoadFile from NC/RT service.
// Parsing has bounded fixed storage and commits the complete profile only once
// every required key and all semantic constraints have passed. No defaults.
namespace EDMVoltageConfigIO
{
    constexpr std::size_t MaximumFileBytes = 4096U;
    constexpr std::size_t MaximumLineBytes = 256U;
    constexpr std::size_t MaximumLines = 128U;
    constexpr std::size_t KeyCount = 18U;

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
        EDMVoltage::ProfileError profileError = EDMVoltage::ProfileError::None;
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
                "SchemaVersion", "ProfileRevision", "Mode", "DeviceId", "ChannelId", "CalibrationRevision",
                "RawMin", "RawMax", "MvAtMin", "MvAtMax", "MinMv", "MaxMv", "LowEnterMv", "LowExitMv",
                "HighExitMv", "HighEnterMv", "MaxAgeMs", "DwellMs"
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
        inline void Assign(EDMVoltage::Profile& p, std::size_t index, std::int64_t v) noexcept
        {
            switch (index)
            {
            case 0U: p.schemaVersion = static_cast<std::uint32_t>(v); break;
            case 1U: p.profileRevision = static_cast<std::uint32_t>(v); break;
            case 3U: p.deviceId = static_cast<std::uint32_t>(v); break;
            case 4U: p.channelId = static_cast<std::uint32_t>(v); break;
            case 5U: p.calibrationRevision = static_cast<std::uint32_t>(v); break;
            case 6U: p.rawMin = static_cast<std::int32_t>(v); break;
            case 7U: p.rawMax = static_cast<std::int32_t>(v); break;
            case 8U: p.mvAtMin = static_cast<std::int32_t>(v); break;
            case 9U: p.mvAtMax = static_cast<std::int32_t>(v); break;
            case 10U: p.gap.minMv = static_cast<std::int32_t>(v); break;
            case 11U: p.gap.maxMv = static_cast<std::int32_t>(v); break;
            case 12U: p.gap.lowEnterMv = static_cast<std::int32_t>(v); break;
            case 13U: p.gap.lowExitMv = static_cast<std::int32_t>(v); break;
            case 14U: p.gap.highExitMv = static_cast<std::int32_t>(v); break;
            case 15U: p.gap.highEnterMv = static_cast<std::int32_t>(v); break;
            case 16U: p.gap.maxAgeMs = static_cast<std::uint64_t>(v); break;
            case 17U: p.gap.dwellMs = static_cast<std::uint64_t>(v); break;
            default: break;
            }
        }
        inline std::size_t ErrorKey(EDMVoltage::ProfileError error, const EDMVoltage::Profile& p) noexcept
        {
            using E = EDMVoltage::ProfileError;
            switch (error)
            {
            case E::Schema: return 0U;
            case E::Revision: return p.profileRevision == 0U ? 1U : 5U;
            case E::Mode: return 2U;
            case E::Identity: return 3U;
            case E::Calibration: return p.rawMin >= p.rawMax ? 7U : 9U;
            case E::VoltageRange:
                if (p.gap.minMv < 0 || p.gap.minMv >= p.gap.maxMv) return 10U;
                if (p.gap.maxMv > 1000000) return 11U;
                return p.mvAtMin < p.gap.minMv ? 8U : 9U;
            case E::Thresholds:
                if (p.gap.lowEnterMv < p.gap.minMv) return 12U;
                if (p.gap.lowExitMv <= p.gap.lowEnterMv) return 13U;
                if (p.gap.highExitMv < p.gap.lowExitMv) return 14U;
                return 15U;
            case E::Timing: return p.gap.maxAgeMs != 100ULL ? 16U : 17U;
            default: return 0U;
            }
        }
    }

    inline bool Parse(const char* bytes, std::size_t length,
        EDMVoltage::Profile& result, Diagnostic& diagnostic) noexcept
    {
        diagnostic = Diagnostic{};
        if (length > MaximumFileBytes) return Detail::Fail(diagnostic, Error::FileTooLarge, 0U);
        if (bytes == nullptr && length != 0U) return Detail::Fail(diagnostic, Error::Format, 0U);
        EDMVoltage::Profile candidate{};
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
                if (std::strcmp(value, "SYNTHETIC_SHADOW") != 0)
                    return Detail::Fail(diagnostic, Error::Mode, lineNumber, key);
            }
            else
            {
                const bool raw = index == 6U || index == 7U;
                const std::uint64_t maximum = raw || (index >= 8U && index <= 15U)
                    ? 2147483647ULL : 4294967295ULL;
                std::int64_t numeric = 0;
                if (!Detail::Decimal(value, raw, maximum, numeric))
                    return Detail::Fail(diagnostic, Error::Number, lineNumber, key);
                Detail::Assign(candidate, index, numeric);
            }
        }
        for (std::size_t i = 0U; i < KeyCount; ++i)
            if ((seen & (1U << static_cast<unsigned int>(i))) == 0U)
                return Detail::Fail(diagnostic, Error::MissingKey, 0U, Detail::Key(i));
        diagnostic.profileError = EDMVoltage::ValidateProfile(candidate);
        if (diagnostic.profileError != EDMVoltage::ProfileError::None)
        {
            const std::size_t index = Detail::ErrorKey(diagnostic.profileError, candidate);
            return Detail::Fail(diagnostic, Error::Profile, keyLines[index], Detail::Key(index));
        }
        result = candidate;
        return true;
    }

    inline bool LoadFile(const char* path, EDMVoltage::Profile& result, Diagnostic& diagnostic) noexcept
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
