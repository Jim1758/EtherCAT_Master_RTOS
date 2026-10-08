#pragma once
#include "EDMGapServo.h"

// EDM39 fixed SIM-only voltage curve. This pure model grants no motion or
// discharge authority; the scoped RT policy validates every finite segment.
namespace EDM39
{
    constexpr double ReferenceVoltage=60.0;
    inline EDMGapServo::Profile FixedProfile() noexcept
    {
        EDMGapServo::Profile profile{};
        for(unsigned i=0U;i<3U;++i)
            profile.positiveGain[i]=profile.negativeGain[i]=100.0;
        profile.cuttingScaleMmPerVoltMin=.01;
        profile.maxFeedMmPerMin=profile.maxRetreatMmPerMin=20.0;
        profile.deadbandV=0.0;
        return profile;
    }
    inline bool IsScriptVoltage(double voltage) noexcept
    {return voltage==40.0||voltage==50.0||voltage==60.0||voltage==70.0||voltage==80.0;}
    inline EDMGapServo::Result Evaluate(double voltage) noexcept
    {return EDMGapServo::Evaluate(FixedProfile(),voltage,ReferenceVoltage,100.0);}
}
