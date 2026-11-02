#pragma once

#include "EDMGapServo.h"
#include "EDMFlushPlan.h"
#include "EDMAxisGainProfiles.h"
#include <cmath>
#include <cstdint>

namespace EDM20
{
    // Schema 2 intentionally has no mode that authorizes physical output.
    enum class ProcessMode : std::uint32_t { ShadowOnly = 0U };

    struct ProcessProfile
    {
        std::uint32_t schemaVersion = 2U;
        std::uint32_t revision = 1U;
        std::uint32_t machineProfileId = 1U;
        ProcessMode mode = ProcessMode::ShadowOnly;
        EDMGapServo::Profile servo{};
        // EDM339: selected only by an admitted orbit machining role. Keeping
        // this separate leaves the ordinary cutting/short-retreat curve intact.
        double orbitScaleMmPerVoltMin = 0.010;
        FlushProfile flush{};
        AxisGainProfiles axisGains{};
    };

    inline bool ValidateProcessProfile(const ProcessProfile& profile) noexcept
    {
        return profile.schemaVersion == 2U && profile.revision > 0U &&
            profile.machineProfileId > 0U && profile.mode == ProcessMode::ShadowOnly &&
            profile.servo.revision == profile.revision &&
            EDMGapServo::ValidateProfile(profile.servo) &&
            std::isfinite(profile.orbitScaleMmPerVoltMin) && profile.orbitScaleMmPerVoltMin >= 0.001 &&
            ValidateFlushProfile(profile.flush) && ValidateAxisGainProfiles(profile.axisGains);
    }
}
