#pragma once

#include "EDMSixAxisGapTuning.h"

// EDM100: an explicitly armed, bounded eight-segment burst session. Each
// segment retains the EDM99 capture-time script, dwell and scalar protections.
namespace EDM100
{
constexpr unsigned MaximumSteps = 8U;

inline bool ValidTuning(const EDM97::Tuning& tuning) noexcept
{ return EDM97::ValidTuning(tuning); }
}
