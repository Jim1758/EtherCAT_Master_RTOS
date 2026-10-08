#pragma once
#include <cstdint>
#include "EDMZGapRapidFixture.h"

// EDM44 changes only the explicit diagnostic run length. Motion geometry,
// SIM voltage curve, speed, dwell and proof rules remain the EDM43 contract.
// P31 is 100 uninterrupted normal cycles. P32 is the first-cycle persistent
// negative control; it cannot claim a completed 100-cycle normal run.
namespace EDM44
{
    constexpr unsigned CycleCount=100U,LaunchCount=3U*CycleCount,InterruptCount=2U*CycleCount;
    constexpr unsigned ReturnCount=CycleCount,ZeroCount=CycleCount;
    constexpr unsigned ShortEntryCount=CycleCount,ShortClearCount=CycleCount;
    constexpr unsigned AdvanceSpeedCount=CycleCount,RetreatSpeedCount=CycleCount;
    constexpr std::uint64_t WindowTimeoutMs=600000U;
}
