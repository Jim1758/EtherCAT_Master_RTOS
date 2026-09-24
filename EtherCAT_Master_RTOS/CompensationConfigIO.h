#pragma once
// PBC-1. BOOT-THREAD ONLY: strict candidate parsing; never called by 250 us RT.
#include "MechanicalCompensationModel.h"
#include <fstream>
#include <locale>
#include <map>
#include <sstream>
#include <string>

namespace pbc
{
using PitchColumns = std::array<std::vector<double>, AxisCount>;
struct ParameterValue { std::string text; std::size_t line = 0U; };
using ParameterMap = std::map<std::string, ParameterValue>;

inline std::string Trim(const std::string& s)
{
    const std::size_t first = s.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return std::string();
    return s.substr(first, s.find_last_not_of(" \t\r\n") - first + 1U);
}
inline std::string DataPart(std::string line, bool firstLine)
{
    if (firstLine && line.compare(0U, 3U, "\xEF\xBB\xBF") == 0) line.erase(0U, 3U);
    std::size_t cut = line.size();
    for (const char* marker : { "//", ";", "#" })
    {
        const std::size_t found = line.find(marker);
        if (found != std::string::npos && found < cut) cut = found;
    }
    return Trim(line.substr(0U, cut));
}
inline bool ParseFinite(const std::string& token, double& result)
{
    std::istringstream input(token);
    input.imbue(std::locale::classic());
    double candidate = 0.0;
    if (!(input >> candidate) || !std::isfinite(candidate)) return false;
    input >> std::ws;
    if (!input.eof()) return false;
    result = candidate;
    return true;
}
inline bool ReadPitchTable(std::istream& input, PitchColumns& result, Diagnostic& d)
{
    d = Diagnostic{};
    PitchColumns candidate{};
    std::string line;
    std::size_t sourceLine = 0U;
    try
    {
        while (std::getline(input, line))
        {
            ++sourceLine;
            d.line = sourceLine;
            if (line.size() > 4096U) { d.error = Error::LineTooLong; return false; }
            line = DataPart(line, sourceLine == 1U);
            if (line.empty()) continue;
            if (candidate[0].size() >= MaxTableRows) { d.error = Error::TooManyRows; return false; }
            std::istringstream row(line);
            row.imbue(std::locale::classic());
            std::array<double, AxisCount> values{};
            for (std::size_t axis = 0U; axis < AxisCount; ++axis)
            {
                std::string token;
                d.column = axis + 1U;
                if (!(row >> token)) { d.error = Error::TableColumns; return false; }
                if (!ParseFinite(token, values[axis])) { d.error = Error::TableNumber; return false; }
            }
            std::string extra;
            if (row >> extra) { d.column = AxisCount + 1U; d.error = Error::TableColumns; return false; }
            for (std::size_t axis = 0U; axis < AxisCount; ++axis) candidate[axis].push_back(values[axis]);
        }
        if (input.bad() || (input.fail() && !input.eof())) { d.error = Error::FileRead; return false; }
        if (candidate[0].size() < 2U) { d.error = Error::EmptyTable; return false; }
        result.swap(candidate);
        d = Diagnostic{};
        return true;
    }
    catch (...) { d.error = Error::Allocation; return false; }
}
inline bool ReadPitchFile(const std::string& path, PitchColumns& result, Diagnostic& d)
{
    std::ifstream input(path, std::ios::binary);
    if (!input.is_open()) { d = Diagnostic{}; d.error = Error::FileOpen; return false; }
    return ReadPitchTable(input, result, d);
}
inline bool ReadParameters(std::istream& input, ParameterMap& result, Diagnostic& d)
{
    d = Diagnostic{};
    ParameterMap candidate;
    std::string line;
    std::size_t sourceLine = 0U;
    try
    {
        while (std::getline(input, line))
        {
            ++sourceLine;
            d.line = sourceLine;
            if (line.size() > 4096U) { d.error = Error::LineTooLong; return false; }
            line = DataPart(line, sourceLine == 1U);
            if (line.empty()) continue;
            const std::size_t equal = line.find('=');
            if (equal == std::string::npos || equal == 0U)
            { d.error = Error::ParameterFormat; return false; }
            const std::string key = Trim(line.substr(0U, equal));
            const std::string value = Trim(line.substr(equal + 1U));
            if (key.empty() || value.empty()) { d.error = Error::ParameterFormat; return false; }
            if (!candidate.emplace(key, ParameterValue{ value, sourceLine }).second)
            { d.error = Error::DuplicateParameter; return false; }
        }
        if (input.bad() || (input.fail() && !input.eof())) { d.error = Error::FileRead; return false; }
        result.swap(candidate);
        d = Diagnostic{};
        return true;
    }
    catch (...) { d.error = Error::Allocation; return false; }
}
inline bool ReadParameterFile(const std::string& path, ParameterMap& result, Diagnostic& d)
{
    std::ifstream input(path, std::ios::binary);
    if (!input.is_open()) { d = Diagnostic{}; d.error = Error::FileOpen; return false; }
    return ReadParameters(input, result, d);
}
inline bool Number(const ParameterMap& parameters, const std::string& key,
    double fallback, double& result, Diagnostic& d)
{
    const auto found = parameters.find(key);
    if (found == parameters.end()) { result = fallback; return true; }
    if (!ParseFinite(found->second.text, result))
    { d.error = Error::ParameterNumber; d.line = found->second.line; return false; }
    return true;
}
inline bool Flag(const ParameterMap& parameters, const std::string& key,
    bool fallback, bool& result, Diagnostic& d)
{
    double value = 0.0;
    if (!Number(parameters, key, fallback ? 1.0 : 0.0, value, d)) return false;
    if (value != 0.0 && value != 1.0)
    {
        d.error = Error::ParameterRange;
        const auto found = parameters.find(key);
        if (found != parameters.end()) d.line = found->second.line;
        return false;
    }
    result = (value == 1.0);
    return true;
}
}
