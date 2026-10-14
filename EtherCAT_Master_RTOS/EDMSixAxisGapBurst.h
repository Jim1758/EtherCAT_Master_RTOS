#pragma once

#include "EDMSixAxisGapTuning.h"

// EDM99: immutable simulated pulse suppression and two sustained retreat windows.
// Capture-time script evidence describes scalar commands, never physical permits.
namespace EDM99
{
constexpr std::uint64_t PulseStartUs = 200000ULL;
constexpr std::uint64_t PulseWindowUs = 50000ULL;
constexpr std::uint64_t PulseLowUs = 5000ULL;
constexpr unsigned PulseWindows = 4U;
constexpr std::uint64_t PulseEndUs = PulseStartUs + PulseWindows * PulseWindowUs;
constexpr std::uint64_t FirstRetreatStartUs = 600000ULL;
constexpr std::uint64_t FirstRetreatEndUs = 1300000ULL;
constexpr std::uint64_t SecondRetreatStartUs = 1600000ULL;
constexpr std::uint64_t SecondRetreatEndUs = 2300000ULL;

inline bool ValidTuning(const EDM97::Tuning& tuning) noexcept
{ return EDM97::ValidTuning(tuning); }

inline unsigned Phase(std::uint64_t elapsedUs) noexcept
{
    return elapsedUs < PulseStartUs ? 1U : elapsedUs < PulseEndUs ? 2U :
        elapsedUs < FirstRetreatStartUs ? 3U : elapsedUs < FirstRetreatEndUs ? 4U :
        elapsedUs < SecondRetreatStartUs ? 5U : elapsedUs < SecondRetreatEndUs ? 6U : 7U;
}

inline unsigned PulseWindow(std::uint64_t elapsedUs) noexcept
{
    return elapsedUs >= PulseStartUs && elapsedUs < PulseEndUs ?
        static_cast<unsigned>((elapsedUs - PulseStartUs) / PulseWindowUs) + 1U : 0U;
}

inline bool IsPulseLow(std::uint64_t elapsedUs) noexcept
{
    return PulseWindow(elapsedUs) && (elapsedUs - PulseStartUs) % PulseWindowUs < PulseLowUs;
}

inline double Voltage(std::uint64_t elapsedUs) noexcept
{
    const unsigned phase = Phase(elapsedUs);
    return IsPulseLow(elapsedUs) || phase == 4U || phase == 6U ? 20.0 : 50.0;
}
}
