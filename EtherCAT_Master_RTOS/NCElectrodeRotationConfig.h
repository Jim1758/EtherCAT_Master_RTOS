#pragma once

#include <istream>
#include <string>

// Startup-only parsing. Axis letters and numeric axis-type settings do not
// imply an electrode role. The selected physical axis is validated separately
// by CoordinateManager before any NC configuration is committed.
namespace NCElectrodeRotationConfig
{
    enum class ParseError
    {
        None,
        StreamFailure,
        MalformedEntry,
        DuplicateKey,
        InvalidValue
    };

    inline const char* ErrorText(ParseError error) noexcept
    {
        switch (error)
        {
        case ParseError::None: return "NONE";
        case ParseError::StreamFailure: return "READ_FAILED";
        case ParseError::MalformedEntry: return "MALFORMED_KEY";
        case ParseError::DuplicateKey: return "DUPLICATE_KEY";
        case ParseError::InvalidValue: return "INVALID_AXIS_VALUE";
        default: return "UNKNOWN";
        }
    }

    inline std::string Trim(const std::string& value)
    {
        const std::string::size_type first = value.find_first_not_of(" \t\r\n\v\f");
        if (first == std::string::npos) return std::string();
        const std::string::size_type last = value.find_last_not_of(" \t\r\n\v\f");
        return value.substr(first, last - first + 1U);
    }

    inline bool IsIdentifierByte(char value) noexcept
    {
        return (value >= 'A' && value <= 'Z') ||
            (value >= 'a' && value <= 'z') ||
            (value >= '0' && value <= '9') || value == '_';
    }

    // On failure oneBasedAxis is unchanged. A missing key commits 0 (unbound).
    // Only literal 0 or 4..8 is accepted: no floating conversion, truncation,
    // signs, exponent notation, or trailing numeric/text data.
    inline bool Read(std::istream& input, int& oneBasedAxis, ParseError& error)
    {
        error = ParseError::None;
        if (!input.good())
        {
            error = ParseError::StreamFailure;
            return false;
        }

        const std::string key("ElectrodeRotationAxis");
        bool firstLine = true;
        bool found = false;
        int candidate = 0;
        for (;;)
        {
            std::string line;
            bool haveLine = false;
            try
            {
                haveLine = static_cast<bool>(std::getline(input, line));
            }
            catch (...)
            {
                // A stream may throw on a normal EOF. Its final unterminated
                // line must still be checked; buffer/read failures fail closed.
                if (!input.eof() || input.bad())
                {
                    error = ParseError::StreamFailure;
                    return false;
                }
                haveLine = !line.empty();
            }
            if (input.bad() || (!haveLine && !input.eof()))
            {
                error = ParseError::StreamFailure;
                return false;
            }
            if (!haveLine) break;

            line = Trim(line);
            if (firstLine && line.compare(0U, 3U, "\xEF\xBB\xBF") == 0)
                line = Trim(line.substr(3U));
            firstLine = false;
            const std::string::size_type comment = line.find("//");
            if (comment != std::string::npos) line = Trim(line.substr(0U, comment));

            if (line.compare(0U, key.size(), key) == 0 &&
                (line.size() == key.size() || !IsIdentifierByte(line[key.size()])))
            {
                const std::string::size_type equals = line.find('=');
                if (equals == std::string::npos || Trim(line.substr(0U, equals)) != key)
                {
                    error = ParseError::MalformedEntry;
                    return false;
                }
                if (found)
                {
                    error = ParseError::DuplicateKey;
                    return false;
                }
                const std::string value = Trim(line.substr(equals + 1U));
                if (value.size() != 1U ||
                    !(value[0] == '0' || (value[0] >= '4' && value[0] <= '8')))
                {
                    error = ParseError::InvalidValue;
                    return false;
                }
                candidate = value[0] - '0';
                found = true;
            }
            if (input.eof()) break;
        }
        oneBasedAxis = candidate;
        return true;
    }
}
