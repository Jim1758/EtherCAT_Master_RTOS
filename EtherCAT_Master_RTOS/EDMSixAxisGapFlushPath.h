#pragma once

#include "EDMSixAxisGapTuning.h"

// EDM105: one bounded command-path flush on each outward paired segment.
// These fixed ratios use the admitted scalar pulse metric. Capture and dwell
// prove commanded position only; physical guards retain their own authority.
namespace EDM105
{
constexpr unsigned MaximumSteps = 16U;
constexpr unsigned ExpectedFlushCycles = 8U;
constexpr double TriggerProgress = 0.5;
constexpr double StrokeRatio = 0.1;
constexpr double SpeedRatio = 0.5;
constexpr double NominalAllowanceSeconds = 2.5;
constexpr std::uint64_t CommandDwellUs = 50000ULL;
constexpr std::uint64_t CommandDwellTicks = 200ULL;

enum class Phase : std::uint8_t
{ None, AwaitTrigger, Brake, Retreat, DwellRetreat, Return, DwellReturn, Complete };
inline const char* PhaseName(Phase phase) noexcept
{
    switch (phase) {
    case Phase::None: return "NONE"; case Phase::AwaitTrigger: return "AWAIT_TRIGGER";
    case Phase::Brake: return "BRAKE"; case Phase::Retreat: return "RETREAT";
    case Phase::DwellRetreat: return "DWELL_RETREAT"; case Phase::Return: return "RETURN";
    case Phase::DwellReturn: return "DWELL_RETURN"; case Phase::Complete: return "COMPLETE";
    } return "UNKNOWN";
}
inline bool ValidTuning(const EDM97::Tuning& tuning) noexcept
{ return EDM97::ValidTuning(tuning); }
}
