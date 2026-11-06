#pragma once

#include <cstdint>

// EDM378: loaded once before cyclic runtime starts. Only the existing
// supervisory/NC diagnostic callers read these settings. Alarm, control,
// collection, HMI publication and explicit operation results are independent.
namespace EDMRuntimeLogPolicy
{
    struct Config
    {
        bool HealthSummary = true;
        std::uint32_t HealthSummaryIntervalSec = 30U;
        bool DcDetail = false;
        bool TimingDetail = false;
        bool GapSamples = false;
        bool ProcessPreview = false;
        bool ShadowDetail = false;
        std::uint32_t DetailIntervalSec = 5U;
    };

    // Defined in the existing supervisory translation unit; no new CPP item.
    // The returned configuration is never reloaded while runtime is active.
    const Config& Current() noexcept;

    inline std::uint64_t DetailIntervalMs() noexcept
    {
        return static_cast<std::uint64_t>(Current().DetailIntervalSec) * 1000ULL;
    }

    // Caller-owned, single-thread rate limiter. The caller supplies an existing
    // monotonic observation; this helper reads no clock and performs no I/O.
    struct RateGate
    {
        bool seen = false;
        std::uint64_t lastMs = 0ULL;

        bool Due(std::uint64_t nowMs, std::uint32_t intervalSec) noexcept
        {
            if (!seen)
            {
                seen = true;
                lastMs = nowMs;
                return true;
            }
            if (nowMs < lastMs)
            {
                // Rebind after a clock reset without immediately printing twice.
                lastMs = nowMs;
                return false;
            }
            const std::uint64_t intervalMs = static_cast<std::uint64_t>(
                intervalSec != 0U ? intervalSec : 1U) * 1000ULL;
            if (nowMs - lastMs < intervalMs) return false;
            lastMs = nowMs;
            return true;
        }
    };
}
