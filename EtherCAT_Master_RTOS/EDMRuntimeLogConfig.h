#pragma once

#include "EDMRuntimeLogPolicy.h"
#include <cstddef>
#include <cstdint>

// EDM378: bounded, allocation-free parsing only. Opening/reading the file is
// performed once by the startup caller, never by this header or an RT callback.
namespace EDMRuntimeLogPolicy
{
    static constexpr std::size_t MaximumLineBytes = 256U;
    static constexpr std::size_t MaximumFileBytes = 16384U;
    static constexpr std::uint32_t MaximumLines = 128U;

    enum class ConfigError : std::uint32_t
    {
        None = 0U,
        Syntax,
        UnknownKey,
        DuplicateKey,
        InvalidValue,
        LineTooLong,
        TooManyLines,
        FileTooLarge,
        IoError
    };

    inline const char* ConfigErrorName(ConfigError error) noexcept
    {
        switch (error)
        {
        case ConfigError::None: return "NONE";
        case ConfigError::Syntax: return "SYNTAX";
        case ConfigError::UnknownKey: return "UNKNOWN_KEY";
        case ConfigError::DuplicateKey: return "DUPLICATE_KEY";
        case ConfigError::InvalidValue: return "INVALID_VALUE";
        case ConfigError::LineTooLong: return "LINE_TOO_LONG";
        case ConfigError::TooManyLines: return "TOO_MANY_LINES";
        case ConfigError::FileTooLarge: return "FILE_TOO_LARGE";
        case ConfigError::IoError: return "IO_ERROR";
        }
        return "UNKNOWN";
    }

    struct ParseResult
    {
        Config config{};
        ConfigError error = ConfigError::None;
        std::uint32_t line = 0U;
        std::uint32_t keys = 0U;
        bool Valid() const noexcept { return error == ConfigError::None; }
    };

    class ConfigParser
    {
    public:
        bool AddLine(const char* text, std::size_t length,
            std::uint32_t lineNumber) noexcept
        {
            if (error_ != ConfigError::None) return false;
            if (lineNumber == 0U || lineNumber != lastLine_ + 1U)
                return Fail(ConfigError::Syntax, lineNumber);
            if (lineNumber > MaximumLines)
                return Fail(ConfigError::TooManyLines, lineNumber);
            if (length > MaximumLineBytes)
                return Fail(ConfigError::LineTooLong, lineNumber);
            if (text == nullptr && length != 0U)
                return Fail(ConfigError::Syntax, lineNumber);
            // Count one separator per supplied line. The loader additionally
            // bounds the actual byte count including its final line separator.
            if (length + 1U > MaximumFileBytes - bytes_)
                return Fail(ConfigError::FileTooLarge, lineNumber);
            bytes_ += length + 1U;
            lastLine_ = lineNumber;
            if (length == 0U) return true;
            for (std::size_t i = 0U; i < length; ++i)
                if (text[i] == '\0' || text[i] == '\n')
                    return Fail(ConfigError::Syntax, lineNumber);

            std::size_t first = 0U;
            if (lineNumber == 1U && length >= 3U &&
                static_cast<unsigned char>(text[0]) == 0xEFU &&
                static_cast<unsigned char>(text[1]) == 0xBBU &&
                static_cast<unsigned char>(text[2]) == 0xBFU)
                first = 3U;
            std::size_t last = length;
            for (std::size_t i = first; i < last; ++i)
            {
                if (text[i] == '#' || text[i] == ';' ||
                    (text[i] == '/' && i + 1U < last && text[i + 1U] == '/'))
                {
                    last = i;
                    break;
                }
            }
            Trim(text, first, last);
            if (first == last) return true;
            std::size_t equals = first;
            while (equals < last && text[equals] != '=') ++equals;
            if (equals == last) return Fail(ConfigError::Syntax, lineNumber);
            std::size_t keyFirst = first, keyLast = equals;
            std::size_t valueFirst = equals + 1U, valueLast = last;
            Trim(text, keyFirst, keyLast);
            Trim(text, valueFirst, valueLast);
            if (keyFirst == keyLast || valueFirst == valueLast)
                return Fail(ConfigError::Syntax, lineNumber);

            const std::uint32_t key = Key(text + keyFirst, keyLast - keyFirst);
            if (key == 0U) return Fail(ConfigError::UnknownKey, lineNumber);
            if ((seenKeys_ & key) != 0U)
                return Fail(ConfigError::DuplicateKey, lineNumber);
            std::uint32_t value = 0U;
            if (!Unsigned(text + valueFirst, valueLast - valueFirst, value))
                return Fail(ConfigError::InvalidValue, lineNumber);
            const bool interval = key == HealthInterval || key == DetailInterval;
            if (!interval && (valueLast - valueFirst != 1U || value > 1U))
                return Fail(ConfigError::InvalidValue, lineNumber);
            if ((key == HealthInterval && (value < 10U || value > 3600U)) ||
                (key == DetailInterval && (value < 1U || value > 3600U)))
                return Fail(ConfigError::InvalidValue, lineNumber);

            switch (key)
            {
            case Health: config_.HealthSummary = value != 0U; break;
            case HealthInterval: config_.HealthSummaryIntervalSec = value; break;
            case Dc: config_.DcDetail = value != 0U; break;
            case Timing: config_.TimingDetail = value != 0U; break;
            case Gap: config_.GapSamples = value != 0U; break;
            case Preview: config_.ProcessPreview = value != 0U; break;
            case Shadow: config_.ShadowDetail = value != 0U; break;
            case DetailInterval: config_.DetailIntervalSec = value; break;
            default: return Fail(ConfigError::UnknownKey, lineNumber);
            }
            seenKeys_ |= key;
            ++keys_;
            return true;
        }

        void Reject(ConfigError error, std::uint32_t lineNumber) noexcept
        {
            if (error != ConfigError::None) (void)Fail(error, lineNumber);
        }

        ParseResult Finish() const noexcept
        {
            ParseResult result{};
            if (error_ == ConfigError::None) result.config = config_;
            result.error = error_;
            result.line = errorLine_;
            result.keys = keys_;
            return result;
        }

    private:
        enum KeyBit : std::uint32_t
        {
            Health = 1U, HealthInterval = 2U, Dc = 4U, Timing = 8U,
            Gap = 16U, Preview = 32U, Shadow = 64U, DetailInterval = 128U
        };

        static bool Space(char ch) noexcept
        { return ch == ' ' || ch == '\t' || ch == '\r'; }

        static void Trim(const char* text, std::size_t& first,
            std::size_t& last) noexcept
        {
            while (first < last && Space(text[first])) ++first;
            while (last > first && Space(text[last - 1U])) --last;
        }

        template<std::size_t N>
        static bool Equals(const char* text, std::size_t length,
            const char (&expected)[N]) noexcept
        {
            if (length != N - 1U) return false;
            for (std::size_t i = 0U; i < length; ++i)
                if (text[i] != expected[i]) return false;
            return true;
        }

        static std::uint32_t Key(const char* text, std::size_t length) noexcept
        {
            if (Equals(text, length, "HealthSummary")) return Health;
            if (Equals(text, length, "HealthSummaryIntervalSec")) return HealthInterval;
            if (Equals(text, length, "DcDetail")) return Dc;
            if (Equals(text, length, "TimingDetail")) return Timing;
            if (Equals(text, length, "GapSamples")) return Gap;
            if (Equals(text, length, "ProcessPreview")) return Preview;
            if (Equals(text, length, "ShadowDetail")) return Shadow;
            if (Equals(text, length, "DetailIntervalSec")) return DetailInterval;
            return 0U;
        }

        static bool Unsigned(const char* text, std::size_t length,
            std::uint32_t& output) noexcept
        {
            if (length == 0U) return false;
            std::uint32_t value = 0U;
            for (std::size_t i = 0U; i < length; ++i)
            {
                if (text[i] < '0' || text[i] > '9') return false;
                const std::uint32_t digit = static_cast<std::uint32_t>(text[i] - '0');
                if (value > (0xFFFFFFFFU - digit) / 10U) return false;
                value = value * 10U + digit;
            }
            output = value;
            return true;
        }

        bool Fail(ConfigError error, std::uint32_t lineNumber) noexcept
        {
            if (error_ == ConfigError::None)
            {
                error_ = error;
                errorLine_ = lineNumber;
            }
            return false;
        }

        Config config_{};
        ConfigError error_ = ConfigError::None;
        std::uint32_t errorLine_ = 0U, lastLine_ = 0U, keys_ = 0U, seenKeys_ = 0U;
        std::size_t bytes_ = 0U;
    };
}
