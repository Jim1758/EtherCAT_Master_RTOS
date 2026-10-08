#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

// EDM20 data/resolve contract only. This file neither switches MotionCore PID
// banks nor changes position windows, MaxLag, axis limits, or motion authority.
namespace EDM20
{
    constexpr std::size_t AxisGainSlotCount = 8U;

    struct AxisGains
    {
        double Kp = 0.0;
        double Ki = 0.0;
        double Kd = 0.0;
        double Kvff = 1.0;
    };

    struct AxisGainProfile
    {
        // Inherit the caller's already validated CNC gains until dedicated
        // discharge/flush gains have been commissioned for this physical axis.
        bool inheritCnc = true;
        AxisGains custom{};
    };

    struct AxisGainProfiles
    {
        // Stable zero-based physical axis slots, not X/Y/Z name assumptions.
        std::array<AxisGainProfile, AxisGainSlotCount> discharge{};
        std::array<AxisGainProfile, AxisGainSlotCount> flush{};
    };

    enum class GainBank : std::uint8_t { Discharge = 0, Flush = 1 };
    enum class GainResolveStatus : std::uint8_t
    {
        Ready = 0, InvalidAxis, InvalidBank, InvalidProfile, InvalidCncGains
    };

    struct GainResolution
    {
        GainResolveStatus status = GainResolveStatus::InvalidProfile;
        AxisGains gains{};
        bool inheritedCnc = false;
        constexpr bool AppliedToMotion() const noexcept { return false; }
    };

    inline bool ValidateAxisGains(const AxisGains& value) noexcept
    {
        return std::isfinite(value.Kp) && value.Kp >= 0.0 &&
            std::isfinite(value.Ki) && value.Ki >= 0.0 &&
            std::isfinite(value.Kd) && value.Kd >= 0.0 &&
            std::isfinite(value.Kvff) && value.Kvff >= 0.0;
    }

    inline bool ValidateAxisGainProfiles(const AxisGainProfiles& value) noexcept
    {
        for (std::size_t axis = 0U; axis < AxisGainSlotCount; ++axis)
        {
            // Dormant custom entries must still be valid data. This prevents
            // an inherit-to-custom switch exposing previously unchecked NaN.
            if (!ValidateAxisGains(value.discharge[axis].custom) ||
                !ValidateAxisGains(value.flush[axis].custom)) return false;
        }
        return true;
    }

    inline GainResolution ResolveAxisGains(const AxisGainProfiles& profiles,
        GainBank bank, std::size_t axis, const AxisGains& cncGains) noexcept
    {
        GainResolution result{};
        if (axis >= AxisGainSlotCount)
        { result.status = GainResolveStatus::InvalidAxis; return result; }
        if (bank != GainBank::Discharge && bank != GainBank::Flush)
        { result.status = GainResolveStatus::InvalidBank; return result; }
        if (!ValidateAxisGainProfiles(profiles)) return result;
        const AxisGainProfile& selected = bank == GainBank::Discharge ?
            profiles.discharge[axis] : profiles.flush[axis];
        if (selected.inheritCnc && !ValidateAxisGains(cncGains))
        { result.status = GainResolveStatus::InvalidCncGains; return result; }
        result.gains = selected.inheritCnc ? cncGains : selected.custom;
        result.inheritedCnc = selected.inheritCnc;
        result.status = GainResolveStatus::Ready;
        return result;
    }
}
