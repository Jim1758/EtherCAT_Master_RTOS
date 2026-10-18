#pragma once

#include "EDM165FlushPidConfig.h"
#include <array>
#include <cmath>
#include <fstream>
#include <string>

// Boot-only configuration. These are physical feed speeds, not PID gains.
// Missing keys keep the EDM170 contract: inherit the authored physical F.
// NC freezes both values for the confirmed run/cache/owner before admission.
namespace EDM171Config
{
constexpr double MaximumCenterReturnMmMin = 30.0;

struct G76CenterReturnSpeeds
{
    double b3MmMin = 0.0;
    double b4MmMin = 0.0;
};

inline bool ValidCenterReturnSpeeds(const G76CenterReturnSpeeds& speeds) noexcept
{
    const auto valid = [](double value) noexcept
    {
        return std::isfinite(value) && value >= 0.0 &&
            value <= MaximumCenterReturnMmMin;
    };
    return valid(speeds.b3MmMin) && valid(speeds.b4MmMin);
}

inline bool LoadG76CenterReturnSpeeds(const std::string& path,
    G76CenterReturnSpeeds& output, std::string& errorKey)
{
    const std::array<std::string, 2U> keys{{
        "G76_B3_CENTER_RETURN_MM_MIN", "G76_B4_CENTER_RETURN_MM_MIN"}};
    G76CenterReturnSpeeds candidate{};
    const std::array<double*, 2U> targets{{&candidate.b3MmMin, &candidate.b4MmMin}};
    std::array<bool, 2U> seen{{false, false}};
    errorKey = "G76_CENTER_RETURN_CONFIG";
    std::ifstream file(path);
    // Match the existing PID file's absent-file/absent-field defaults.
    if (!file.is_open()) { output = candidate; errorKey.clear(); return true; }
    std::string line;
    while (std::getline(file, line))
    {
        const std::string trimmed = EDM165Config::Trim(line);
        if (trimmed.empty() || trimmed.compare(0U, 2U, "//") == 0) continue;
        const std::size_t equal = trimmed.find('=');
        const std::string key = EDM165Config::Trim(trimmed.substr(0U, equal));
        for (std::size_t field = 0U; field < keys.size(); ++field)
        {
            if (key != keys[field]) continue;
            errorKey = keys[field];
            if (seen[field] || equal == std::string::npos ||
                !EDM165Config::ParseGain(trimmed.substr(equal + 1U), *targets[field]) ||
                !(*targets[field] > 0.0) || *targets[field] > MaximumCenterReturnMmMin)
                return false;
            seen[field] = true;
            break;
        }
    }
    if (file.bad()) { errorKey = "G76_CENTER_RETURN_READ"; return false; }
    output = candidate;
    errorKey.clear();
    return true;
}
}
