#pragma once
#include <cstdint>
#include "EDMZGapCurveFixture.h"

// EDM40 SIM-only finite-position interruption script. A changed voltage is
// sampled during a proved applied move; the NC bridge then requests priority
// Stop and waits for a fresh physical Held proof before same-origin re-arm.
// This pure contract grants neither motion nor physical discharge authority.
namespace EDM40
{
    constexpr double TargetHalfMm=.04;
    constexpr double TriggerProgressMm=.002,TriggerRemainingMm=.004;
    constexpr std::uint64_t TriggerMinimumUs=20000U,TriggerMinimumTicks=80U;
    constexpr std::uint64_t ZeroDwellUs=500000U,ZeroDwellTicks=2000U;
    constexpr unsigned CycleCount=6U,LaunchCount=24U,InterruptCount=18U;
    constexpr unsigned ReturnCount=6U,ZeroCount=12U;
    struct ScriptLeg
    {
        double voltage=0.0,targetMm=0.0,triggerVoltage=0.0;
    };
    inline bool TryLeg(unsigned step,ScriptLeg&leg) noexcept
    {
        leg=ScriptLeg{};
        switch(step)
        {
        case 0U:leg.voltage=70.0;leg.targetMm=-TargetHalfMm;leg.triggerVoltage=60.0;return true;
        case 1U:leg.voltage=80.0;leg.targetMm=-TargetHalfMm;leg.triggerVoltage=40.0;return true;
        case 2U:leg.voltage=40.0;leg.targetMm=0.0;leg.triggerVoltage=50.0;return true;
        case 3U:leg.voltage=50.0;leg.targetMm=0.0;return true;
        default:return false;
        }
    }
}
