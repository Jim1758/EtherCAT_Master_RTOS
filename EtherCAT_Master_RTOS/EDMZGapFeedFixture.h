#pragma once
#include <cstdint>
#include "EDMZGapShortReplanFixture.h"

// EDM42 SIM-only finite-endpoint feed updates. Changing feed does not create
// a new trajectory, change the first-Arm origin, or authorize physical discharge.
namespace EDM42
{
    constexpr unsigned FeedUpdatesPerLeg=2U,RequiredFeedUpdates=36U,PersistentFeedUpdates=4U;
    constexpr std::uint64_t FeedUpdateMinTicks=80U,FeedUpdateMinUs=20000U;
    constexpr double FeedUpdateMinProgressMm=.0005,FeedUpdateMinRemainingMm=.006;
    constexpr double RtFeedUpdateMinRemainingMm=.004;
    inline bool IsScriptVoltage(double voltage) noexcept
    {return EDM41::IsScriptVoltage(voltage);}
    inline bool TryLeg(unsigned step,bool persistent,EDM40::ScriptLeg&leg) noexcept
    {return EDM41::TryLeg(step,persistent,leg);}
    constexpr bool HasFeedUpdates(unsigned step) noexcept
    {return step==0U||step==1U||step==3U;}
    inline bool TryFeedUpdateVoltage(unsigned step,unsigned index,double&voltage) noexcept
    {
        voltage=0.0;
        if(index>=FeedUpdatesPerLeg)return false;
        switch(step)
        {
        case 0U:voltage=index==0U?80.0:70.0;return true;
        case 1U:voltage=index==0U?70.0:80.0;return true;
        case 3U:voltage=index==0U?40.0:50.0;return true;
        default:return false;
        }
    }
    constexpr bool IsSameDirectionFeedPair(double previous,double next) noexcept
    {return (previous==70.0&&next==80.0)||(previous==80.0&&next==70.0)||
        (previous==40.0&&next==50.0)||(previous==50.0&&next==40.0);}
}
