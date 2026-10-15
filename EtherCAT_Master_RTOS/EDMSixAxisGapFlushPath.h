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

// EDM107: one frozen, bounded command-path profile for the admitted flush run.
// NC I/J are scalar percentages; K is an integral dwell in milliseconds.
// Defaults preserve the EDM105 command path bit for bit.
namespace EDM107
{
struct FlushProfile
{
    double strokeRatio = EDM105::StrokeRatio;
    double speedRatio = EDM105::SpeedRatio;
    std::uint64_t dwellUs = EDM105::CommandDwellUs;
};
static_assert(std::is_trivially_copyable<FlushProfile>::value,
    "EDM107 flush profile is copied by value");
static_assert(sizeof(FlushProfile) == 24U, "EDM107 stores three bounded profile values");

inline bool ValidFlushProfile(const FlushProfile& profile) noexcept
{
    return std::isfinite(profile.strokeRatio) && profile.strokeRatio >= 0.05 && profile.strokeRatio <= 0.1 &&
        std::isfinite(profile.speedRatio) && profile.speedRatio >= 0.25 && profile.speedRatio <= 0.5 &&
        profile.dwellUs >= 20000ULL && profile.dwellUs <= 100000ULL && profile.dwellUs % 1000ULL == 0ULL;
}

inline bool SameFlushProfile(const FlushProfile& left, const FlushProfile& right) noexcept
{
    return EDM97::DoubleBits(left.strokeRatio) == EDM97::DoubleBits(right.strokeRatio) &&
        EDM97::DoubleBits(left.speedRatio) == EDM97::DoubleBits(right.speedRatio) &&
        left.dwellUs == right.dwellUs;
}

inline bool IsDefaultFlushProfile(const FlushProfile& profile) noexcept
{ return SameFlushProfile(profile, FlushProfile{}); }

inline std::uint64_t FlushDwellTicks(const FlushProfile& profile) noexcept
{ return profile.dwellUs / 250ULL; }

inline bool ParseFlushProfile(double strokePercent, double speedPercent, double dwellMs,
    FlushProfile& profile) noexcept
{
    if (!std::isfinite(strokePercent) || strokePercent < 5.0 || strokePercent > 10.0 ||
        !std::isfinite(speedPercent) || speedPercent < 25.0 || speedPercent > 50.0 ||
        !std::isfinite(dwellMs) || dwellMs < 20.0 || dwellMs > 100.0 || std::floor(dwellMs) != dwellMs)
        return false;
    const FlushProfile parsed{ strokePercent / 100.0, speedPercent / 100.0,
        static_cast<std::uint64_t>(dwellMs) * 1000ULL };
    if (!ValidFlushProfile(parsed)) return false;
    profile = parsed;
    return true;
}
}

// EDM106: first interrupted planner state, retained for NC-thread diagnostics.
// This is neither an axis command receipt nor physical stop/encoder evidence.
namespace EDM106
{
struct Interruption
{
    // fault identifies Segment::Fail only; guard/transport faults have separate evidence.
    bool captured = false, fault = false;
    EDM105::Phase phase = EDM105::Phase::None;
    std::uint64_t sourceTick = 0ULL, plannerTick = 0ULL, nowUs = 0ULL;
    double q = 0.0, v = 0.0;
};
static_assert(std::is_trivially_copyable<Interruption>::value,
    "EDM106 interruption is a bounded value snapshot");
static_assert(sizeof(Interruption) <= 48U, "EDM106 stores no trajectory");
}
