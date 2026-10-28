#include "PositionAutoTuneConfig.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <fstream>
#include <limits>
#include <locale>
#include <set>
#include <sstream>

namespace PositionAutoTune
{
namespace
{
bool White(char value) noexcept
{
    return value == ' ' || value == '\t' || value == '\r' || value == '\n';
}

std::string Trim(const std::string& value)
{
    std::size_t first = 0U;
    while (first < value.size() && White(value[first])) ++first;
    std::size_t last = value.size();
    while (last > first && White(value[last - 1U])) --last;
    return value.substr(first, last - first);
}

bool Ascii(const std::string& value) noexcept
{
    for (const unsigned char ch : value)
        if (ch < 0x20U || ch > 0x7eU) return false;
    return true;
}

bool Digit(char value) noexcept { return value >= '0' && value <= '9'; }

bool ParseDouble(const std::string& value, double& result)
{
    // Decimal grammar deliberately excludes NaN, infinity, hex and locale commas.
    std::size_t offset = 0U;
    if (offset < value.size() && (value[offset] == '+' || value[offset] == '-')) ++offset;
    bool digits = false;
    while (offset < value.size() && Digit(value[offset])) { digits = true; ++offset; }
    if (offset < value.size() && value[offset] == '.')
    {
        ++offset;
        while (offset < value.size() && Digit(value[offset])) { digits = true; ++offset; }
    }
    if (!digits) return false;
    if (offset < value.size() && (value[offset] == 'e' || value[offset] == 'E'))
    {
        ++offset;
        if (offset < value.size() && (value[offset] == '+' || value[offset] == '-')) ++offset;
        const std::size_t exponentStart = offset;
        while (offset < value.size() && Digit(value[offset])) ++offset;
        if (offset == exponentStart) return false;
    }
    if (offset != value.size()) return false;
    std::istringstream stream(value);
    stream.imbue(std::locale::classic());
    stream >> result;
    return !stream.fail() && stream.eof() && std::isfinite(result);
}

bool ParseUnsigned(const std::string& value, std::uint32_t& result) noexcept
{
    if (value.empty()) return false;
    std::uint32_t number = 0U;
    const std::uint32_t maximum = (std::numeric_limits<std::uint32_t>::max)();
    for (const char ch : value)
    {
        if (!Digit(ch)) return false;
        const std::uint32_t digit = static_cast<std::uint32_t>(ch - '0');
        if (number > (maximum - digit) / 10U) return false;
        number = number * 10U + digit;
    }
    result = number;
    return true;
}

bool ParseBool(const std::string& value, bool& result) noexcept
{
    if (value == "0") { result = false; return true; }
    if (value == "1") { result = true; return true; }
    return false;
}

enum class ParseStatus { Parsed, Unknown, Invalid };

ParseStatus ParseKey(Config& config, const std::string& key, const std::string& value)
{
    if (key == "Enabled") return ParseBool(value, config.enabled) ? ParseStatus::Parsed : ParseStatus::Invalid;
    if (key == "MachineId") { config.machineId = value; return ParseStatus::Parsed; }
    if (key == "MachineProfile")
    {
        if (value == "BENCH") config.machineProfile = MachineProfile::Bench;
        else if (value == "MACHINE") config.machineProfile = MachineProfile::Machine;
        else return ParseStatus::Invalid;
        return ParseStatus::Parsed;
    }
    if (key == "MaxSamples") return ParseUnsigned(value, config.maxSamples) ? ParseStatus::Parsed : ParseStatus::Invalid;
    double* destination = nullptr;
    if (key == "MaxDurationSeconds") destination = &config.maxDurationSeconds;
    else if (key == "TwoPointMaxDurationSeconds") destination = &config.twoPointMaxDurationSeconds;
    else if (key == "ModelFitMinimum") destination = &config.modelFitMinimum;
    else if (key == "MinimumVelocityMmS") destination = &config.minimumVelocityMmS;
    else if (key == "CandidateResponseFactor") destination = &config.candidateResponseFactor;
    else if (key == "MaxKpIncreaseRatio") destination = &config.maxKpIncreaseRatio;
    else if (key.size() > 2U && key[0] >= '0' && key[0] <= '5' && key[1] == '_')
    {
        AxisConfig& axis = config.axis[static_cast<std::size_t>(key[0] - '0')];
        const std::string field = key.substr(2U);
        if (field == "Enabled") return ParseBool(value, axis.enabled) ? ParseStatus::Parsed : ParseStatus::Invalid;
        if (field == "IdleEnabled") return ParseBool(value, axis.idleEnabled) ? ParseStatus::Parsed : ParseStatus::Invalid;
        if (field == "RangeEnabled") return ParseBool(value, axis.rangeEnabled) ? ParseStatus::Parsed : ParseStatus::Invalid;
        if (field == "Cycles") return ParseUnsigned(value, axis.cycles) ? ParseStatus::Parsed : ParseStatus::Invalid;
        if (field == "TuningStrategy") return ParseUnsigned(value, axis.tuningStrategy) ? ParseStatus::Parsed : ParseStatus::Invalid;
        if (field == "ExcursionMm") destination = &axis.excursionMm;
        else if (field == "TargetPositionMm") destination = &axis.targetPositionMm;
        else if (field == "CruiseLagTargetMm") destination = &axis.cruiseLagTargetMm;
        else if (field == "SpeedMmMin") destination = &axis.speedMmMin;
        else if (field == "AccelerationSeconds") destination = &axis.accelerationSeconds;
        else if (field == "DecelerationSeconds") destination = &axis.decelerationSeconds;
        else if (field == "DwellSeconds") destination = &axis.dwellSeconds;
        else if (field == "FollowingLimitMm") destination = &axis.followingLimitMm;
        else if (field == "SettleWindowMm") destination = &axis.settleWindowMm;
        else if (field == "SettleHoldSeconds") destination = &axis.settleHoldSeconds;
        else if (field == "SettleTimeoutSeconds") destination = &axis.settleTimeoutSeconds;
        else if (field == "KpMin") destination = &axis.kpMin;
        else if (field == "KpMax") destination = &axis.kpMax;
        else if (field == "KvffMin") destination = &axis.kvffMin;
        else if (field == "KvffMax") destination = &axis.kvffMax;
        else if (field == "MaxOvershootMm") destination = &axis.maxOvershootMm;
        else if (field == "MaxRmsErrorMm") destination = &axis.maxRmsErrorMm;
        else if (field == "RangeMinMCS") destination = &axis.rangeMinMCS;
        else if (field == "RangeMaxMCS") destination = &axis.rangeMaxMCS;
        else if (field == "EndPadding") destination = &axis.endPadding;
        else if (field == "IdleKpMin") destination = &axis.idleKpMin;
        else if (field == "IdleKpMax") destination = &axis.idleKpMax;
        else if (field == "IdleKiMin") destination = &axis.idleKiMin;
        else if (field == "IdleKiMax") destination = &axis.idleKiMax;
        else if (field == "IdleIntegralTimeSeconds") destination = &axis.idleIntegralTimeSeconds;
        else if (field == "IdleObservationSeconds") destination = &axis.idleObservationSeconds;
        else if (field == "IdleMaxRmsErrorMm") destination = &axis.idleMaxRmsErrorMm;
        else if (field == "IdleMaxJitterMm") destination = &axis.idleMaxJitterMm;
    }
    if (destination == nullptr) return ParseStatus::Unknown;
    return ParseDouble(value, *destination) ? ParseStatus::Parsed : ParseStatus::Invalid;
}

bool Positive(double value) noexcept { return std::isfinite(value) && value > 0.0; }
bool Nonnegative(double value) noexcept { return std::isfinite(value) && value >= 0.0; }

bool Reject(std::string& error, const std::string& reason)
{
    error = reason;
    return false;
}

void ExceptionError(std::string& error) noexcept
{
    try { error = "CONFIG_EXCEPTION"; } catch (...) {}
}
}

double EffectiveTwoPointDurationSeconds(const Config& config, double predictedSeconds) noexcept
{
    if (!Nonnegative(predictedSeconds) || !Positive(config.twoPointMaxDurationSeconds) ||
        config.twoPointMaxDurationSeconds > kTwoPointMaximumDurationSeconds)
        return (std::numeric_limits<double>::infinity)();
    // Clamp before multiplication: a huge finite request still receives a
    // finite one-hour ceiling and is rejected by the caller's time comparison.
    const double reserve = predictedSeconds >= kTwoPointMaximumDurationSeconds ?
        kTwoPointMaximumDurationSeconds : std::ceil(predictedSeconds * 1.25 + 5.0);
    return (std::min)(kTwoPointMaximumDurationSeconds,
        (std::max)(config.twoPointMaxDurationSeconds, reserve));
}

double EffectiveCandidateResponseFactor(const Config& config, const AxisConfig& axis) noexcept
{
    switch (axis.tuningStrategy)
    {
    case 0U: return config.candidateResponseFactor;
    case 1U: return 0.12;
    case 2U: return 0.20;
    case 3U: return 0.28;
    default: return (std::numeric_limits<double>::quiet_NaN)();
    }
}

double EffectiveMaxKpIncreaseRatio(const Config& config, const AxisConfig& axis) noexcept
{
    switch (axis.tuningStrategy)
    {
    case 0U: return config.maxKpIncreaseRatio;
    case 1U: return (std::min)(config.maxKpIncreaseRatio, 1.10);
    case 2U: case 3U: return (std::min)(config.maxKpIncreaseRatio, 1.25);
    default: return (std::numeric_limits<double>::quiet_NaN)();
    }
}

const char* TuningStrategyName(std::uint32_t strategy) noexcept
{
    switch (strategy)
    {
    case 0U: return "CUSTOM";
    case 1U: return "STABLE";
    case 2U: return "STANDARD";
    case 3U: return "FAST";
    default: return "UNKNOWN";
    }
}

double PredictAxisDurationSeconds(const AxisConfig& axis) noexcept
{
    if (!Positive(axis.excursionMm) || !Positive(axis.speedMmMin) ||
        !Positive(axis.accelerationSeconds) || !Positive(axis.decelerationSeconds) ||
        !Nonnegative(axis.dwellSeconds) || !Positive(axis.settleHoldSeconds) || axis.cycles == 0U ||
        (axis.idleEnabled && !Positive(axis.idleObservationSeconds)))
        return (std::numeric_limits<double>::infinity)();
    const double speed = axis.speedMmMin / 60.0;
    const double rampTime = axis.accelerationSeconds + axis.decelerationSeconds;
    if (!Positive(speed) || !Positive(rampTime)) return (std::numeric_limits<double>::infinity)();
    const double rampDistance = speed * rampTime * 0.5;
    double legSeconds = 0.0;
    if (axis.excursionMm >= rampDistance)
        legSeconds = axis.excursionMm / speed + rampTime * 0.5;
    else
        legSeconds = std::sqrt(2.0 * axis.excursionMm / speed) * std::sqrt(rampTime);
    const double dwell = (std::max)(axis.dwellSeconds,axis.idleEnabled?axis.idleObservationSeconds:0.0);
    const double duration = 4.0 * static_cast<double>(axis.cycles) *
        (legSeconds + dwell + axis.settleHoldSeconds);
    return Positive(duration) ? duration : (std::numeric_limits<double>::infinity)();
}

bool ValidateConfig(const Config& config, std::string& error) noexcept
{
    try
    {
        error.clear();
        if (config.machineProfile != MachineProfile::Bench && config.machineProfile != MachineProfile::Machine)
            return Reject(error, "INVALID_MACHINE_PROFILE");
        if (config.machineId.empty() || config.machineId.size() > 48U)
            return Reject(error, "INVALID_MACHINE_ID");
        for (const char ch : config.machineId)
            if (!(Digit(ch) || (ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z') || ch == '_' || ch == '-'))
                return Reject(error, "INVALID_MACHINE_ID");
        if (config.maxSamples != kMaxSamples) return Reject(error, "MAX_SAMPLES_MUST_BE_131072");
        if (!Positive(config.maxDurationSeconds) ||
            config.maxDurationSeconds > static_cast<double>(kMaxSamples - 1U) * kSamplePeriodSeconds)
            return Reject(error, "DURATION_EXCEEDS_FIXED_SAMPLE_CAPACITY");
        if (!Positive(config.twoPointMaxDurationSeconds) || config.twoPointMaxDurationSeconds > kTwoPointMaximumDurationSeconds)
            return Reject(error, "INVALID_TWO_POINT_MAX_DURATION");
        if (!Positive(config.modelFitMinimum) || config.modelFitMinimum >= 1.0)
            return Reject(error, "INVALID_MODEL_FIT_MINIMUM");
        if (!Positive(config.minimumVelocityMmS)) return Reject(error, "INVALID_MINIMUM_VELOCITY");
        if (!Positive(config.candidateResponseFactor) || config.candidateResponseFactor > 1.0)
            return Reject(error, "INVALID_CANDIDATE_RESPONSE_FACTOR");
        if (!std::isfinite(config.maxKpIncreaseRatio) || config.maxKpIncreaseRatio < 1.0)
            return Reject(error, "INVALID_MAX_KP_INCREASE_RATIO");
        bool anyEnabled = false;
        for (std::size_t index = 0U; index < kAxisCount; ++index)
        {
            const AxisConfig& axis = config.axis[index];
            const std::string prefix = "AXIS_" + std::to_string(index) + "_";
            if (axis.tuningStrategy > 3U) return Reject(error, prefix + "INVALID_TUNING_STRATEGY");
            if (!Nonnegative(axis.targetPositionMm)) return Reject(error, prefix + "INVALID_TARGET_POSITION");
            if (!Nonnegative(axis.cruiseLagTargetMm)) return Reject(error, prefix + "INVALID_CRUISE_LAG_TARGET");
            if (!Positive(axis.excursionMm) || !Positive(axis.speedMmMin) ||
                !Positive(axis.accelerationSeconds) || !Positive(axis.decelerationSeconds) ||
                !Nonnegative(axis.dwellSeconds) || axis.cycles == 0U || axis.cycles > 32U)
                return Reject(error, prefix + "INVALID_MOTION");
            if (!Positive(axis.followingLimitMm) || !Positive(axis.settleWindowMm) ||
                axis.settleWindowMm > axis.followingLimitMm)
                return Reject(error, prefix + "INVALID_FOLLOWING_OR_SETTLE_WINDOW");
            if (!Positive(axis.settleHoldSeconds) || !Positive(axis.settleTimeoutSeconds) ||
                axis.settleHoldSeconds < kSamplePeriodSeconds ||
                axis.settleTimeoutSeconds < axis.settleHoldSeconds)
                return Reject(error, prefix + "INVALID_SETTLE_TIMES");
            if (!Positive(axis.kpMin) || !Positive(axis.kpMax) || axis.kpMax < axis.kpMin ||
                !Positive(axis.kvffMin) || !Positive(axis.kvffMax) || axis.kvffMax < axis.kvffMin)
                return Reject(error, prefix + "INVALID_CANDIDATE_BOUNDS");
            if (!Nonnegative(axis.maxOvershootMm) || !Positive(axis.maxRmsErrorMm))
                return Reject(error, prefix + "INVALID_ACCEPTANCE_LIMITS");
            if (!std::isfinite(axis.rangeMinMCS) || !std::isfinite(axis.rangeMaxMCS) ||
                !std::isfinite(axis.endPadding))
                return Reject(error, prefix + "RANGE_NONFINITE");
            if (axis.endPadding < 0.0) return Reject(error, prefix + "RANGE_PADDING");
            if (axis.rangeEnabled)
            {
                if (axis.rangeMinMCS >= axis.rangeMaxMCS) return Reject(error, prefix + "RANGE_ORDER");
                const double width = axis.rangeMaxMCS - axis.rangeMinMCS;
                if (!std::isfinite(width))
                    return Reject(error, prefix + "RANGE_NONFINITE");
                // Q1 uses the full taught stroke. Legacy NC preflight still
                // checks excursion/padding and the complete origin envelope.
            }
            if (!Positive(axis.idleKpMin) || !Positive(axis.idleKpMax) || axis.idleKpMax < axis.idleKpMin ||
                !Nonnegative(axis.idleKiMin) || !Nonnegative(axis.idleKiMax) || axis.idleKiMax < axis.idleKiMin ||
                !Positive(axis.idleIntegralTimeSeconds))
                return Reject(error, prefix + "INVALID_IDLE_CANDIDATE_BOUNDS");
            if (!Positive(axis.idleObservationSeconds) || axis.idleObservationSeconds < 0.05)
                return Reject(error, prefix + "INVALID_IDLE_OBSERVATION");
            if (!Positive(axis.idleMaxRmsErrorMm) || !Nonnegative(axis.idleMaxJitterMm))
                return Reject(error, prefix + "INVALID_IDLE_ACCEPTANCE_LIMITS");
            // Reject overflow/underflow even for disabled axes. Mode-specific
            // NC preflight applies the legacy or Q1 time budget before motion.
            const double travel = 4.0 * axis.excursionMm * static_cast<double>(axis.cycles);
            const double duration = PredictAxisDurationSeconds(axis);
            if (!Positive(travel) || !Positive(duration)) return Reject(error, prefix + "INVALID_PREDICTED_MOTION");
            if (!axis.enabled) continue;
            anyEnabled = true;
            if (axis.speedMmMin / 60.0 <= config.minimumVelocityMmS)
                return Reject(error, prefix + "SPEED_BELOW_IDENTIFICATION_MINIMUM");
        }
        if (config.enabled && !anyEnabled) return Reject(error, "ENABLED_WITH_NO_AXES");
        return true;
    }
    catch (...)
    {
        ExceptionError(error);
        return false;
    }
}

ConfigLoadStatus LoadConfig(const std::string& path, Config& config,
    std::string& error) noexcept
{
    config.enabled = false;
    try
    {
        config = Config{};
        error.clear();
        errno = 0;
        std::ifstream input(path, std::ios::in | std::ios::binary);
        if (!input.is_open())
        {
            if (errno == ENOENT) return ConfigLoadStatus::MissingDisabled;
            error = "CONFIG_OPEN_FAILED";
            return ConfigLoadStatus::Invalid;
        }
        Config candidate{};
        std::set<std::string> seen;
        std::string line;
        std::size_t lineNumber = 0U;
        std::size_t totalBytes = 0U;
        while (std::getline(input, line))
        {
            ++lineNumber;
            totalBytes += line.size() + 1U;
            if (line.size() > 8192U || totalBytes > 262144U)
            {
                error = "CONFIG_TOO_LARGE";
                return ConfigLoadStatus::Invalid;
            }
            if (lineNumber == 1U && line.size() >= 3U &&
                static_cast<unsigned char>(line[0]) == 0xefU &&
                static_cast<unsigned char>(line[1]) == 0xbbU &&
                static_cast<unsigned char>(line[2]) == 0xbfU) line.erase(0U, 3U);
            const std::size_t comment = line.find("//");
            if (comment != std::string::npos) line.resize(comment);
            const std::string content = Trim(line);
            if (content.empty()) continue;
            const std::string location = "LINE_" + std::to_string(lineNumber) + "_";
            const std::size_t equal = content.find('=');
            if (equal == std::string::npos || content.find('=', equal + 1U) != std::string::npos)
            {
                error = location + "EXPECTED_KEY_VALUE";
                return ConfigLoadStatus::Invalid;
            }
            const std::string key = Trim(content.substr(0U, equal));
            const std::string value = Trim(content.substr(equal + 1U));
            if (key.empty() || value.empty() || !Ascii(key) || !Ascii(value))
            {
                error = location + "NON_ASCII_OR_EMPTY_KEY_VALUE";
                return ConfigLoadStatus::Invalid;
            }
            if (!seen.insert(key).second)
            {
                error = location + "DUPLICATE_KEY_" + key;
                return ConfigLoadStatus::Invalid;
            }
            const ParseStatus status = ParseKey(candidate, key, value);
            if (status != ParseStatus::Parsed)
            {
                error = location + (status == ParseStatus::Unknown ? "UNKNOWN_KEY_" : "INVALID_VALUE_") + key;
                return ConfigLoadStatus::Invalid;
            }
        }
        if (!input.eof() || input.bad())
        {
            error = "CONFIG_READ_FAILED";
            return ConfigLoadStatus::Invalid;
        }
        for (std::size_t index = 0U; index < kAxisCount; ++index)
        {
            if (!candidate.axis[index].rangeEnabled) continue;
            const std::string prefix = std::to_string(index) + "_";
            if (!seen.count(prefix + "RangeEnabled") || !seen.count(prefix + "RangeMinMCS") ||
                !seen.count(prefix + "RangeMaxMCS") || !seen.count(prefix + "EndPadding"))
            {
                error = "AXIS_" + std::to_string(index) + "_RANGE_KEYS_REQUIRED";
                return ConfigLoadStatus::Invalid;
            }
        }
        if (!ValidateConfig(candidate, error)) return ConfigLoadStatus::Invalid;
        config = candidate;
        return ConfigLoadStatus::Loaded;
    }
    catch (...)
    {
        config.enabled = false;
        ExceptionError(error);
        return ConfigLoadStatus::Invalid;
    }
}
}
