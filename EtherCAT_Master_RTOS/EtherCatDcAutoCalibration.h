#pragma once

#include "EtherCatMaster_DC_Tuning.h"
#include <cstdint>

// EDM376: shared, bounded AUTO calibration decision and diagnostic vocabulary.
// This helper has no platform calls, allocation, clocks or control authority.
namespace EDM376DcCal
{
    static constexpr std::uint32_t InputRejected = 0x0001U;
    static constexpr std::uint32_t RobustUnlocked = 0x0002U;
    static constexpr std::uint32_t BufferWarmup = 0x0004U;
    static constexpr std::uint32_t RangeLow = 0x0008U;
    static constexpr std::uint32_t RangeHigh = 0x0010U;
    static constexpr std::uint32_t MadHigh = 0x0020U;
    static constexpr std::uint32_t RawMedianFar = 0x0040U;
    static constexpr std::uint32_t CycleUnclean = 0x0080U;
    static constexpr std::uint32_t NcBusy = 0x0100U;
    static constexpr std::uint32_t MotionBusy = 0x0200U;
    static constexpr std::uint32_t HardTrip = 0x0400U;
    static constexpr std::uint32_t PublicationBusy = 0x0800U;
    static constexpr std::uint32_t UnstableWindows = 0x1000U;
    static constexpr std::uint32_t IdleIdentityChanged = 0x2000U;

    // Unsigned subtraction avoids signed overflow even for malformed snapshots.
    inline std::uint64_t AbsoluteDifference(
        std::int64_t left, std::int64_t right) noexcept
    {
        return left >= right
            ? static_cast<std::uint64_t>(left) - static_cast<std::uint64_t>(right)
            : static_cast<std::uint64_t>(right) - static_cast<std::uint64_t>(left);
    }

    inline std::uint32_t QualityRejectMask(
        std::int64_t rawPpb,
        std::int64_t medianPpb,
        std::int64_t madPpb,
        bool currentAccepted,
        bool robustLocked,
        std::uint32_t bufferCount,
        bool cycleClean) noexcept
    {
        std::uint32_t mask = 0U;
        if (!currentAccepted) mask |= InputRejected;
        if (!robustLocked) mask |= RobustUnlocked;
        if (bufferCount < 9U) mask |= BufferWarmup;
        if (medianPpb < EtherCatDcTuning::DriftCalibrationMinimumPpb ||
            rawPpb < -EtherCatDcTuning::RobustMaximumAbsoluteDriftPpb) mask |= RangeLow;
        if (medianPpb > EtherCatDcTuning::DriftCalibrationMaximumPpb ||
            rawPpb > EtherCatDcTuning::RobustMaximumAbsoluteDriftPpb) mask |= RangeHigh;
        if (madPpb < 0 || madPpb > EtherCatDcTuning::DriftCalibrationMaximumMadPpb)
            mask |= MadHigh;
        if (AbsoluteDifference(rawPpb, medianPpb) >
            static_cast<std::uint64_t>(
                EtherCatDcTuning::DriftCalibrationMaximumRawMedianDeviationPpb))
            mask |= RawMedianFar;
        if (!cycleClean) mask |= CycleUnclean;
        return mask;
    }

    // The full mask remains in the log; this is only its most useful first tag.
    inline const char* PrimaryReason(std::uint32_t mask) noexcept
    {
        if ((mask & HardTrip) != 0U) return "HARD_TRIP";
        if ((mask & NcBusy) != 0U) return "NC_BUSY";
        if ((mask & MotionBusy) != 0U) return "MOTION_BUSY";
        if ((mask & PublicationBusy) != 0U) return "COMMIT_BUSY";
        if ((mask & IdleIdentityChanged) != 0U) return "IDLE_CHANGED";
        if ((mask & UnstableWindows) != 0U) return "UNSTABLE_WINDOWS";
        if ((mask & RangeLow) != 0U) return "RANGE_LOW";
        if ((mask & RangeHigh) != 0U) return "RANGE_HIGH";
        if ((mask & MadHigh) != 0U) return "MAD_HIGH";
        if ((mask & RawMedianFar) != 0U) return "RAW_MEDIAN_FAR";
        if ((mask & CycleUnclean) != 0U) return "CYCLE_UNCLEAN";
        if ((mask & InputRejected) != 0U) return "INPUT_REJECTED";
        if ((mask & RobustUnlocked) != 0U) return "ROBUST_UNLOCKED";
        if ((mask & BufferWarmup) != 0U) return "BUFFER_WARMUP";
        return mask == 0U ? "NONE" : "UNKNOWN";
    }
}
