#pragma once

#include "EDMGapServo.h"
#include <cstring>
#include <type_traits>

// EDM97: immutable NC tuning for the bounded simulated six-axis path servo.
// Values describe the voltage curve only; they confer no output permission.
namespace EDM97
{
constexpr double ReferenceVoltageV = 35.0;
constexpr double CuttingScaleMmPerVoltMin = 0.001;
constexpr double MaximumFeedMmMin = 0.12;
constexpr double MaximumRetreatMmMin = 0.03;
constexpr std::uint64_t ScriptPhaseUs = 300000ULL;

struct Tuning
{
    double positiveGain = 0.0;
    double negativeGain = 0.0;
    double deadbandV = 0.0;
};
static_assert(std::is_trivially_copyable<Tuning>::value, "EDM97 tuning is copied by value");
static_assert(sizeof(Tuning) == 3U * sizeof(double), "EDM97 tuning has three double values");

inline bool ValidTuning(const Tuning& tuning) noexcept
{
    return std::isfinite(tuning.positiveGain) && tuning.positiveGain >= 6.0 && tuning.positiveGain <= 10.0 &&
        std::isfinite(tuning.negativeGain) && tuning.negativeGain >= 1.0 && tuning.negativeGain <= 2.0 &&
        std::isfinite(tuning.deadbandV) && tuning.deadbandV >= 0.0 && tuning.deadbandV <= 2.0 &&
        !std::signbit(tuning.deadbandV);
}

inline std::uint64_t DoubleBits(double value) noexcept
{
    static_assert(sizeof(double) == sizeof(std::uint64_t), "EDM97 uses 64-bit double values");
    std::uint64_t bits = 0ULL;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

inline bool SameTuning(const Tuning& left, const Tuning& right) noexcept
{
    return DoubleBits(left.positiveGain) == DoubleBits(right.positiveGain) &&
        DoubleBits(left.negativeGain) == DoubleBits(right.negativeGain) &&
        DoubleBits(left.deadbandV) == DoubleBits(right.deadbandV);
}

inline bool EmptyTuning(const Tuning& tuning) noexcept
{ return SameTuning(tuning, Tuning{}); }

// Diagnostic identity only. Authorizations must compare every value exactly.
inline std::uint64_t ConfigHash(const Tuning& tuning) noexcept
{
    const std::uint64_t words[3U] = { DoubleBits(tuning.positiveGain),
        DoubleBits(tuning.negativeGain), DoubleBits(tuning.deadbandV) };
    std::uint64_t hash = 14695981039346656037ULL;
    for (const auto word : words)
        for (unsigned byte = 0U; byte < 8U; ++byte) {
            hash ^= (word >> (8U * byte)) & 255ULL;
            hash *= 1099511628211ULL;
        }
    return hash;
}

inline EDMGapServo::Profile MakeProfile(const Tuning& tuning) noexcept
{
    EDMGapServo::Profile profile{};
    for (unsigned i = 0U; i < 3U; ++i) {
        profile.positiveGain[i] = tuning.positiveGain;
        profile.negativeGain[i] = tuning.negativeGain;
    }
    profile.cuttingScaleMmPerVoltMin = CuttingScaleMmPerVoltMin;
    profile.maxFeedMmPerMin = MaximumFeedMmMin;
    profile.maxRetreatMmPerMin = MaximumRetreatMmMin;
    profile.deadbandV = tuning.deadbandV;
    profile.shortMachiningV = ReferenceVoltageV;
    profile.shortHysteresisV = 5.0;
    profile.shortEnterMs = profile.shortExitMs = 20U;
    profile.shortRetreatMmPerMin = 0.0;
    return profile;
}
}
