#pragma once
#include <cstdint>
#include "EDMGapServo.h"

// EDM43 SIM-only finite rapid forward/retreat contract. This model grants
// neither motion nor discharge authority; RT independently validates its source.
namespace EDM43
{
    constexpr double ReferenceVoltage=60.0;
    constexpr double FeedMmMin=30.0,PdoCapMmS=.5,TargetHalfMm=.25,OuterHalfMm=.4;
    constexpr double AccelerationTimeSec=.3,DecelerationTimeSec=.3;
    constexpr double AdvanceProgressMm=.13,RetreatProgressMm=.045,TriggerRemainingMm=.06;
    constexpr double SpeedMinimumMmS=.4;
    constexpr std::uint64_t SpeedWindowMinUs=20000U,SpeedWindowMinTicks=80U;
    constexpr std::uint64_t SpeedWindowMaxUs=40000U,SpeedWindowMaxTicks=160U;
    constexpr std::uint64_t ZeroDwellUs=100000U,ZeroDwellTicks=400U;
    constexpr std::uint64_t ShortEntryUs=2000U,ShortEntryTicks=8U;
    constexpr std::uint64_t ShortClearUs=5000U,ShortClearTicks=20U;
    constexpr unsigned CycleCount=6U,LaunchCount=18U,InterruptCount=12U;
    constexpr unsigned ReturnCount=6U,ZeroCount=6U,ShortEntryCount=6U,ShortClearCount=6U;
    constexpr unsigned AdvanceSpeedCount=6U,RetreatSpeedCount=6U;
    struct Leg
    {
        double voltage=0.0,targetMm=0.0,triggerVoltage=0.0,progressMm=0.0;
    };
    inline EDMGapServo::Profile FixedProfile() noexcept
    {
        EDMGapServo::Profile profile{};
        for(unsigned i=0U;i<3U;++i)
            profile.positiveGain[i]=profile.negativeGain[i]=100.0;
        profile.cuttingScaleMmPerVoltMin=.015;
        profile.maxFeedMmPerMin=profile.maxRetreatMmPerMin=FeedMmMin;
        profile.deadbandV=0.0;
        return profile;
    }
    inline bool IsScriptVoltage(double voltage) noexcept
    {return voltage==20.0||voltage==40.0||voltage==50.0||voltage==60.0||voltage==70.0||voltage==80.0;}
    inline EDMGapServo::Result Evaluate(double voltage) noexcept
    {return EDMGapServo::Evaluate(FixedProfile(),voltage,ReferenceVoltage,100.0);}
    inline bool TryLeg(unsigned step,bool persistent,Leg&leg) noexcept
    {
        leg=Leg{};
        switch(step)
        {
        case 0U:
            leg.voltage=80.0;leg.targetMm=-TargetHalfMm;leg.triggerVoltage=20.0;
            leg.progressMm=AdvanceProgressMm;return true;
        case 1U:
            leg.voltage=20.0;leg.targetMm=0.0;leg.triggerVoltage=persistent?0.0:50.0;
            leg.progressMm=RetreatProgressMm;return true;
        case 2U:
            if(persistent)return false;
            leg.voltage=50.0;leg.targetMm=0.0;return true;
        default:return false;
        }
    }
}
