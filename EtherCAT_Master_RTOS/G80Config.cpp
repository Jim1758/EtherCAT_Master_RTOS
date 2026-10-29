#include "G80Config.h"

#include <cerrno>
#include <cmath>
#include <fstream>
#include <limits>
#include <locale>
#include <set>
#include <sstream>

namespace G80
{
namespace
{
constexpr std::size_t MaximumFileBytes = 32768U;
constexpr std::size_t MaximumLineBytes = 1024U;

struct RealField { const char* key; double Config::*member; };
const RealField RealFields[] = {
    {"ContactVoltageV", &Config::contactVoltageV},
    {"ReleaseHysteresisV", &Config::releaseHysteresisV},
    {"SlowSpeedMmMin", &Config::slowSpeedMmMin},
    {"FastSpeedMmMin", &Config::fastSpeedMmMin},
    {"SlowSpeedDegMin", &Config::slowSpeedDegMin},
    {"FastSpeedDegMin", &Config::fastSpeedDegMin},
    {"RetreatMm", &Config::retreatMm},
    {"RetreatDeg", &Config::retreatDeg},
    {"MaxMeanErrorMm", &Config::maxMeanErrorMm},
    {"MaxMeanErrorDeg", &Config::maxMeanErrorDeg},
    {"MaxSearchMm", &Config::maxSearchMm},
    {"MaxSearchDeg", &Config::maxSearchDeg},
    {"MaxCycleSeconds", &Config::maxCycleSeconds},
    {"ReleaseTimeoutSeconds", &Config::releaseTimeoutSeconds},
    {"SettleTimeoutSeconds", &Config::settleTimeoutSeconds},
    {"AccelerationSeconds", &Config::accelerationSeconds},
    {"DecelerationSeconds", &Config::decelerationSeconds},
    {"SettleWindowMm", &Config::settleWindowMm},
    {"SettleWindowDeg", &Config::settleWindowDeg},
    {"SettleHoldSeconds", &Config::settleHoldSeconds},
    {"DFilterSeconds", &Config::derivativeFilterSeconds}
};
struct CountField { const char* key; std::uint32_t Config::*member; };
const RealField SimulationRealFields[] = {
    {"SimulationEdgeDistanceMm", &Config::simulationEdgeDistanceMm},
    {"SimulationEdgeDistanceDeg", &Config::simulationEdgeDistanceDeg},
    {"SimulationSpreadMm", &Config::simulationSpreadMm},
    {"SimulationSpreadDeg", &Config::simulationSpreadDeg},
    {"SimulationLossDistanceMm", &Config::simulationLossDistanceMm},
    {"SimulationLossDistanceDeg", &Config::simulationLossDistanceDeg}
};
const CountField CountFields[] = {
    {"SlowTouches", &Config::slowTouches},
    {"ContactStableSamples", &Config::contactStableSamples},
    {"ReleaseStableSamples", &Config::releaseStableSamples}
};
struct GainField { const char* key; double AxisGains::*member; };
const GainField GainFields[] = {
    {"Kp", &AxisGains::kp}, {"Ki", &AxisGains::ki},
    {"Kd", &AxisGains::kd}, {"Kvff", &AxisGains::kvff},
    {"MaxIntegral", &AxisGains::maxIntegral},
    {"FollowingLimitNative", &AxisGains::followingLimitNative}
};

bool White(char ch) noexcept { return ch == ' ' || ch == '\t' || ch == '\r' || ch == '\n'; }
bool Digit(char ch) noexcept { return ch >= '0' && ch <= '9'; }
std::string Trim(const std::string& value)
{
    std::size_t begin = 0U, end = value.size();
    while (begin < end && White(value[begin])) ++begin;
    while (end > begin && White(value[end - 1U])) --end;
    return value.substr(begin, end - begin);
}
bool Ascii(const std::string& value) noexcept
{
    for (const unsigned char ch : value) if (ch < 0x20U || ch > 0x7eU) return false;
    return true;
}
bool Real(const std::string& value, double& result)
{
    std::size_t pos = 0U;
    if (pos < value.size() && (value[pos] == '+' || value[pos] == '-')) ++pos;
    bool digits = false;
    while (pos < value.size() && Digit(value[pos])) { digits = true; ++pos; }
    if (pos < value.size() && value[pos] == '.') {
        ++pos;
        while (pos < value.size() && Digit(value[pos])) { digits = true; ++pos; }
    }
    if (!digits) return false;
    if (pos < value.size() && (value[pos] == 'e' || value[pos] == 'E')) {
        ++pos;
        if (pos < value.size() && (value[pos] == '+' || value[pos] == '-')) ++pos;
        const std::size_t first = pos;
        while (pos < value.size() && Digit(value[pos])) ++pos;
        if (pos == first) return false;
    }
    if (pos != value.size()) return false;
    std::istringstream stream(value);
    stream.imbue(std::locale::classic());
    stream >> result;
    return !stream.fail() && stream.eof() && std::isfinite(result);
}
bool Count(const std::string& value, std::uint32_t& result) noexcept
{
    if (value.empty()) return false;
    std::uint32_t number = 0U;
    const auto maximum = (std::numeric_limits<std::uint32_t>::max)();
    for (char ch : value) {
        if (!Digit(ch)) return false;
        const std::uint32_t digit = static_cast<std::uint32_t>(ch - '0');
        if (number > (maximum - digit) / 10U) return false;
        number = number * 10U + digit;
    }
    result = number; return true;
}
enum class ParseResult { Valid, Unknown, Invalid };
ParseResult Assign(Config& config, const std::string& key, const std::string& value)
{
    if (key == "Enabled") {
        if (value != "0" && value != "1") return ParseResult::Invalid;
        config.enabled = value == "1"; return ParseResult::Valid;
    }
    for (const auto& field : RealFields)
        if (key == field.key) return Real(value, config.*(field.member)) ? ParseResult::Valid : ParseResult::Invalid;
    for (const auto& field : CountFields)
        if (key == field.key) return Count(value, config.*(field.member)) ? ParseResult::Valid : ParseResult::Invalid;
    if (key == "SimulationScenario")
        return Count(value, config.simulationScenario) ? ParseResult::Valid : ParseResult::Invalid;
    for (const auto& field : SimulationRealFields)
        if (key == field.key) return Real(value, config.*(field.member)) ? ParseResult::Valid : ParseResult::Invalid;
    if (key.size() > 2U && key[1] == '_') {
        const std::string axes = "XYZCUV";
        const auto axis = axes.find(key[0]);
        if (axis != std::string::npos) {
            const std::string suffix = key.substr(2U);
            for (const auto& field : GainFields)
                if (suffix == field.key) return Real(value, config.gains[axis].*(field.member)) ? ParseResult::Valid : ParseResult::Invalid;
        }
    }
    return ParseResult::Unknown;
}
bool Reject(std::string& error, const std::string& reason)
{ error = reason; return false; }
bool Positive(double value) noexcept { return std::isfinite(value) && value > 0.0; }
bool Nonnegative(double value) noexcept { return std::isfinite(value) && value >= 0.0; }
}

bool BuildNativeRampCaps(const Config& config, double maximumVelocityNative,
    double& accelerationNative, double& decelerationNative) noexcept
{
    // Do not take AxisContext or any G00 values: both admission paths use this
    // same process-specific calculation before freezing the RT request.
    if (!Positive(maximumVelocityNative) || !Positive(config.maxCycleSeconds) ||
        !Positive(config.accelerationSeconds) || !Positive(config.decelerationSeconds) ||
        config.accelerationSeconds < SampleSeconds || config.decelerationSeconds < SampleSeconds ||
        config.accelerationSeconds > config.maxCycleSeconds ||
        config.decelerationSeconds > config.maxCycleSeconds) return false;
    const double acceleration = maximumVelocityNative / config.accelerationSeconds;
    const double deceleration = maximumVelocityNative / config.decelerationSeconds;
    if (!Positive(acceleration) || !Positive(deceleration)) return false;
    accelerationNative = acceleration;
    decelerationNative = deceleration;
    return true;
}

bool ValidateConfig(const Config& config, std::string& error) noexcept
{
    try {
        error.clear();
        if (config.slowTouches == 0U || config.slowTouches > MaximumSlowTouches)
            return Reject(error, "SLOW_TOUCHES_RANGE");
        if (!config.contactStableSamples || config.contactStableSamples > MaximumStableSamples ||
            !config.releaseStableSamples || config.releaseStableSamples > MaximumStableSamples)
            return Reject(error, "STABLE_SAMPLES_RANGE");
        if (!Positive(config.contactVoltageV) || !Positive(config.releaseHysteresisV) ||
            config.contactVoltageV > 1000.0 || config.releaseHysteresisV > 1000.0 - config.contactVoltageV)
            return Reject(error, "CONTACT_VOLTAGE_RANGE");
        if (!Positive(config.slowSpeedMmMin) || !Positive(config.fastSpeedMmMin) ||
            !Positive(config.slowSpeedDegMin) || !Positive(config.fastSpeedDegMin) ||
            config.fastSpeedMmMin < config.slowSpeedMmMin || config.fastSpeedDegMin < config.slowSpeedDegMin)
            return Reject(error, "SPEED_RANGE");
        if (!Positive(config.retreatMm) || !Positive(config.retreatDeg) ||
            !Positive(config.maxSearchMm) || !Positive(config.maxSearchDeg) ||
            config.retreatMm > config.maxSearchMm || config.retreatDeg > config.maxSearchDeg)
            return Reject(error, "SEARCH_OR_RETREAT_RANGE");
        if (!Nonnegative(config.maxMeanErrorMm) || !Nonnegative(config.maxMeanErrorDeg))
            return Reject(error, "MEAN_ERROR_RANGE");
        if (!Positive(config.maxCycleSeconds) || config.maxCycleSeconds > 3600.0 ||
            !Positive(config.releaseTimeoutSeconds) || config.releaseTimeoutSeconds > config.maxCycleSeconds ||
            !Positive(config.settleTimeoutSeconds) || config.settleTimeoutSeconds > config.maxCycleSeconds)
            return Reject(error, "TIMEOUT_RANGE");
        if (!Positive(config.accelerationSeconds) || !Positive(config.decelerationSeconds) ||
            config.accelerationSeconds < SampleSeconds || config.decelerationSeconds < SampleSeconds ||
            config.accelerationSeconds > config.maxCycleSeconds || config.decelerationSeconds > config.maxCycleSeconds)
            return Reject(error, "RAMP_TIME_RANGE");
        if (!Positive(config.settleWindowMm) || !Positive(config.settleWindowDeg) ||
            !Positive(config.settleHoldSeconds) || config.settleHoldSeconds < SampleSeconds ||
            config.settleHoldSeconds > config.settleTimeoutSeconds)
            return Reject(error, "SETTLE_RANGE");
        if (!Positive(config.derivativeFilterSeconds))
            return Reject(error, "D_FILTER_RANGE");
        if (config.simulationScenario > 4U ||
            !Positive(config.simulationEdgeDistanceMm) || !Positive(config.simulationEdgeDistanceDeg) ||
            !Nonnegative(config.simulationSpreadMm) || !Nonnegative(config.simulationSpreadDeg) ||
            !Positive(config.simulationLossDistanceMm) || !Positive(config.simulationLossDistanceDeg))
            return Reject(error, "SIMULATION_RANGE");
        for (unsigned axis = 0U; axis < AxisCount; ++axis) {
            const auto& gain = config.gains[axis];
            if (!Positive(gain.kp) || !Nonnegative(gain.ki) || !Nonnegative(gain.kd) ||
                !Nonnegative(gain.kvff) || !Nonnegative(gain.maxIntegral) || !Positive(gain.followingLimitNative))
                return Reject(error, std::string(1U, "XYZCUV"[axis]) + "_GAIN_RANGE");
            const double window = axis < 3U ? config.settleWindowMm : config.settleWindowDeg;
            if (window > gain.followingLimitNative)
                return Reject(error, std::string(1U, "XYZCUV"[axis]) + "_SETTLE_EXCEEDS_FOLLOWING");
        }
        return true;
    } catch (...) {
        // Keep noexcept even under allocation failure; no partial config becomes live.
        error.clear(); return false;
    }
}

ConfigLoadStatus LoadConfig(const std::string& path, Config& config, std::string& error) noexcept
{
    config = Config{};
    try {
        error.clear(); errno = 0;
        std::ifstream file(path, std::ios::binary);
        if (!file.is_open()) {
            if (errno == ENOENT) return ConfigLoadStatus::MissingDisabled;
            error = "CONFIG_OPEN_FAILED"; return ConfigLoadStatus::Invalid;
        }
        // Bound the read before line parsing, including a maliciously long line.
        // Allocation happens only in this NC/startup loader, never in RT.
        std::string contents(MaximumFileBytes + 1U, '\0');
        file.read(&contents[0U], static_cast<std::streamsize>(contents.size()));
        const auto readBytes = file.gcount();
        if (file.bad() || (file.fail() && !file.eof()) || readBytes < 0) {
            error = "CONFIG_READ_FAILED"; return ConfigLoadStatus::Invalid;
        }
        if (static_cast<std::size_t>(readBytes) > MaximumFileBytes) {
            error = "CONFIG_TOO_LARGE"; return ConfigLoadStatus::Invalid;
        }
        contents.resize(static_cast<std::size_t>(readBytes));
        std::istringstream input(contents);
        Config candidate{}; std::set<std::string> seen;
        std::string line; std::size_t lineNumber = 0U, bytes = 0U;
        while (std::getline(input, line)) {
            ++lineNumber; bytes += line.size() + 1U;
            if (line.size() > MaximumLineBytes || bytes > MaximumFileBytes) {
                error = "CONFIG_TOO_LARGE"; return ConfigLoadStatus::Invalid;
            }
            if (lineNumber == 1U && line.size() >= 3U &&
                static_cast<unsigned char>(line[0]) == 0xefU &&
                static_cast<unsigned char>(line[1]) == 0xbbU &&
                static_cast<unsigned char>(line[2]) == 0xbfU) line.erase(0U, 3U);
            const auto comment = line.find("//");
            if (comment != std::string::npos) line.resize(comment);
            const auto content = Trim(line); if (content.empty()) continue;
            const auto prefix = "LINE_" + std::to_string(lineNumber) + "_";
            const auto equal = content.find('=');
            if (equal == std::string::npos || content.find('=', equal + 1U) != std::string::npos) {
                error = prefix + "EXPECTED_KEY_VALUE"; return ConfigLoadStatus::Invalid;
            }
            const auto key = Trim(content.substr(0U, equal));
            const auto value = Trim(content.substr(equal + 1U));
            if (key.empty() || value.empty() || !Ascii(key) || !Ascii(value)) {
                error = prefix + "NON_ASCII_OR_EMPTY_KEY_VALUE"; return ConfigLoadStatus::Invalid;
            }
            if (!seen.insert(key).second) {
                error = prefix + "DUPLICATE_KEY_" + key; return ConfigLoadStatus::Invalid;
            }
            const auto parsed = Assign(candidate, key, value);
            if (parsed != ParseResult::Valid) {
                error = prefix + (parsed == ParseResult::Unknown ? "UNKNOWN_KEY_" : "INVALID_VALUE_") + key;
                return ConfigLoadStatus::Invalid;
            }
        }
        if (!input.eof() || input.bad()) {
            error = "CONFIG_READ_FAILED"; return ConfigLoadStatus::Invalid;
        }
        const auto required = [&seen, &error](const std::string& key) {
            if (seen.count(key)) return true;
            error = "MISSING_KEY_" + key; return false;
        };
        if (!required("Enabled")) return ConfigLoadStatus::Invalid;
        for (const auto& field : RealFields) if (!required(field.key)) return ConfigLoadStatus::Invalid;
        for (const auto& field : CountFields) if (!required(field.key)) return ConfigLoadStatus::Invalid;
        // The old physical profile remains valid. An explicitly supplied
        // simulation extension must be complete, never a partial default mix.
        bool hasSimulation = seen.count("SimulationScenario") != 0U;
        for (const auto& field : SimulationRealFields) hasSimulation = hasSimulation || seen.count(field.key) != 0U;
        if (hasSimulation) {
            if (!required("SimulationScenario")) return ConfigLoadStatus::Invalid;
            for (const auto& field : SimulationRealFields) if (!required(field.key)) return ConfigLoadStatus::Invalid;
        }
        for (unsigned axis = 0U; axis < AxisCount; ++axis)
            for (const auto& field : GainFields)
                if (!required(std::string(1U, "XYZCUV"[axis]) + "_" + field.key)) return ConfigLoadStatus::Invalid;
        if (!ValidateConfig(candidate, error)) return ConfigLoadStatus::Invalid;
        config = candidate; return ConfigLoadStatus::Loaded;
    } catch (...) {
        config = Config{}; error.clear(); return ConfigLoadStatus::Invalid;
    }
}
}
