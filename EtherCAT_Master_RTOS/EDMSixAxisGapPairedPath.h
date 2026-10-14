#pragma once

#include "EDMSixAxisGapTuning.h"

// EDM104: sixteen authored segments grouped into eight independent outward /
// exact-return pairs. Each segment retains the EDM99 capture-time script,
// dwell and scalar protections; only the paired profile admits a new vector
// at the beginning of each pair.
namespace EDM104
{
constexpr unsigned MaximumSteps = 16U;

inline bool ValidTuning(const EDM97::Tuning& tuning) noexcept
{ return EDM97::ValidTuning(tuning); }
}
