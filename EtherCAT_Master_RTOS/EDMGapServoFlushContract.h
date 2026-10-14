#pragma once
#include "EDMFlushPhysicalRuntime.h"
#include "EDMGapServoRecipeBinding.h"
#include <cmath>

// EDM60: the two frozen recipes must describe one admission snapshot. This
// contract grants no motion, stop receipt, PID configuration or discharge.
namespace EDM60
{
constexpr double InitialDepthMm = .125;
constexpr double WorkUpperMm = .249;
constexpr double CaptureAllowanceMm = .001;
constexpr double InitialAllowanceMm = .00075;
constexpr double CycleSeconds = .00025;
constexpr double AccelerationMmS2 = .5 / .3;
inline double WorkFloorMm(const EDM57::Recipe& recipe) noexcept
{ return EDM57::MaximumLiftHeight(recipe) + .002; }

inline bool SameAdmission(const EDM57::RecipeIdentity& a,
    const EDM59::RecipeIdentity& b) noexcept
{
    if (a.catalogRevision != b.catalogRevision ||
        a.definitionRevision != b.definitionRevision || a.profileId != b.profileId ||
        a.tableId != b.tableId || a.tableRevision != b.tableRevision ||
        a.eCode != b.eCode || a.generation != b.generation) return false;
    // Each recipe already checks its own IDs. The seven fields must also
    // remain distinct across the flush/servo boundary.
    for (unsigned i = 0; i < 5U; ++i)
        for (unsigned j = 0; j < 2U; ++j)
            if (a.fieldIds[i] == b.fieldIds[j]) return false;
    return true;
}

inline double MaximumWorkDepthMm(const EDM57::Recipe& flush,
    const EDM59::Recipe& servo) noexcept
{
    // The installed curve cap is applied after E8 by the existing evaluator.
    // Use that cap conservatively, without applying the override a second time.
    const double speed = servo.curve.maxFeedMmPerMin / 60.0;
    const double workSeconds = static_cast<double>(flush.workTimeUs) / 1000000.0;
    const double coast = speed * speed / (2.0 * AccelerationMmS2) + speed * CycleSeconds;
    return InitialDepthMm + InitialAllowanceMm + static_cast<double>(flush.cycleCount) *
        (speed * workSeconds + coast + CaptureAllowanceMm);
}

template<class C> inline bool ValidateConfig(const C& config) noexcept
{
    const unsigned profile = static_cast<unsigned>(config.profile);
    if (profile < 49U || profile > 52U ||
        !EDM57::ValidateRecipeForProfile(config.flush, profile) ||
        !EDM59::ValidateRecipe(config.servoRecipe) ||
        config.flush.workDepthMm != InitialDepthMm ||
        !SameAdmission(config.flush.binding, config.servoRecipe.binding)) return false;
    const double floor = WorkFloorMm(config.flush);
    const double maximum = MaximumWorkDepthMm(config.flush, config.servoRecipe);
    return std::isfinite(floor) && floor > 0.0 && floor < InitialDepthMm &&
        std::isfinite(maximum) && maximum <= WorkUpperMm;
}
}
