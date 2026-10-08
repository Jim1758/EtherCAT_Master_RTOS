#pragma once
#include <cstdint>
#include "EDMZGapReplanFixture.h"

// EDM41 SIM-only short-circuit interruption script. The NC bridge owns the
// fresh short-entry/clear observation windows and the persistent-short alarm.
// The RT policy separately validates every finite Position source and direction.
// This pure contract grants neither motion nor physical discharge authority.
namespace EDM41
{
    constexpr std::uint64_t ShortEntryUs=2000U,ShortEntryTicks=8U;
    constexpr std::uint64_t ShortClearUs=5000U,ShortClearTicks=20U;
    constexpr unsigned ShortEntryCount=6U,ShortClearCount=6U;
    inline bool IsScriptVoltage(double voltage) noexcept
    {return voltage==20.0||EDM39::IsScriptVoltage(voltage);}
    inline bool TryLeg(unsigned step,bool persistent,EDM40::ScriptLeg&leg) noexcept
    {
        leg=EDM40::ScriptLeg{};
        switch(step)
        {
        case 0U:leg.voltage=70.0;leg.targetMm=-EDM40::TargetHalfMm;leg.triggerVoltage=60.0;return true;
        case 1U:leg.voltage=80.0;leg.targetMm=-EDM40::TargetHalfMm;leg.triggerVoltage=20.0;return true;
        case 2U:leg.voltage=20.0;leg.targetMm=0.0;leg.triggerVoltage=persistent?0.0:50.0;return true;
        case 3U:
            if(persistent)return false;
            leg.voltage=50.0;leg.targetMm=0.0;return true;
        default:return false;
        }
    }
}
