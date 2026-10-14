#pragma once

#include "EDMSixAxisGapTuning.h"

// EDM98: deterministic deadband-noise exercise for the existing scalar lane.
// This script and its command evidence confer no physical stop/output permission.
namespace EDM98
{
constexpr std::uint64_t ForwardPhaseUs = 600000ULL;
constexpr std::uint64_t QuietPhaseUs = 800000ULL;
constexpr std::uint64_t QuietProofUs = 200000ULL;
constexpr std::uint64_t QuietProofTicks = 800ULL;

inline bool ValidTuning(const EDM97::Tuning& tuning) noexcept
{ return EDM97::ValidTuning(tuning) && tuning.deadbandV >= 1.0; }

inline unsigned Phase(std::uint64_t elapsedUs) noexcept
{
    return elapsedUs < ForwardPhaseUs ? 1U :
        elapsedUs < ForwardPhaseUs + QuietPhaseUs ? 2U :
        elapsedUs < 2ULL * ForwardPhaseUs + QuietPhaseUs ? 3U :
        elapsedUs < 2ULL * (ForwardPhaseUs + QuietPhaseUs) ? 4U : 5U;
}

inline bool IsQuietPhase(unsigned phase) noexcept
{ return phase == 2U || phase == 4U; }

inline double Voltage(std::uint64_t elapsedUs, std::uint64_t sequence) noexcept
{
    const unsigned phase = Phase(elapsedUs);
    return IsQuietPhase(phase) ? ((sequence & 1ULL) ? 34.5 : 35.5) :
        phase == 3U ? 20.0 : 50.0;
}
}
