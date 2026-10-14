#pragma once

#include "EDMSixAxisGapTuning.h"

// EDM102: an explicitly armed, bounded sixteen-segment burst session. Each
// segment retains the EDM99 capture-time script, dwell and scalar protections.
namespace EDM102
{
constexpr unsigned MaximumSteps = 16U;

inline bool ValidTuning(const EDM97::Tuning& tuning) noexcept
{ return EDM97::ValidTuning(tuning); }
}
