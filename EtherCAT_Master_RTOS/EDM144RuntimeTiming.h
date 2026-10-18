#pragma once

#include <cstdint>
#include <limits>

// Diagnostic-only, bounded callback sampling. All durations are nanoseconds.
namespace EDM144
{
    static const std::uint32_t Stride = 65u;
    static const std::uint32_t SamplesPerWindow = 64u;
    static const std::uint32_t PhaseCount = 7u;
    static const std::uint32_t MetricCount = 8u;
    static const std::uint32_t BucketCount = 5u;

    enum Phase
    {
        PreWire = 0,       // callback entry -> transaction envelope
        Wire,              // LRW/FRMW plus existing before/after QPC captures
        PostRx,            // after transaction envelope -> before observation
        Observe,           // before observation -> before interpolation
        Interpolation,     // before interpolation -> after interpolation
        Axes,              // after interpolation -> after axes
        Tail,              // after axes -> callback end
        Total
    };

    enum Bucket
    {
        Idle = 0,
        Auto,
        Other,
        OwnerChange,
        InvalidPDO
    };

    struct Sample
    {
        std::uint64_t at[MetricCount];
        std::uint64_t cycle;
        std::uint32_t ownerBegin;
        std::uint32_t ownerEnd;
        bool processValid;
    };

    struct Stats
    {
        std::uint32_t count;
        std::uint64_t sumNs[MetricCount];
        std::uint64_t maxNs[MetricCount];
        std::uint32_t over250;
    };

    struct Snapshot
    {
        std::uint64_t windowId;
        std::uint64_t firstCycle;
        std::uint64_t lastCycle;
        std::uint32_t attempted;
        std::uint32_t accepted;
        std::uint32_t rejected;
        Stats buckets[BucketCount];
    };

    inline const char* BucketName(unsigned bucket) noexcept
    {
        switch (bucket)
        {
        case Idle: return "IDLE";
        case Auto: return "AUTO";
        case Other: return "OTHER";
        case OwnerChange: return "OWNER_CHANGE";
        case InvalidPDO: return "INVALID_PDO";
        default: return "UNKNOWN";
        }
    }

    class Collector
    {
    public:
        Collector() noexcept : _countdown(0u), _nextWindowId(1u), _window{} {}

        // The first callback is sampled; 65 is coprime with four PLC slots.
        bool Due() noexcept
        {
            if (_countdown == 0u)
            {
                _countdown = Stride - 1u;
                return true;
            }
            --_countdown;
            return false;
        }

        // Output changes only when a complete window of attempted samples ends.
        bool Record(const Sample& sample, Snapshot& output) noexcept
        {
            if (_window.attempted == 0u)
            {
                _window.windowId = _nextWindowId;
                _window.firstCycle = sample.cycle;
            }
            _window.lastCycle = sample.cycle;
            ++_window.attempted;

            bool valid = sample.at[0] != 0u && sample.cycle != 0u;
            for (std::uint32_t phase = 0u; phase < PhaseCount; ++phase)
            {
                if (sample.at[phase + 1u] < sample.at[phase])
                    valid = false;
            }

            // Monotonic timestamps make subtraction safe. Bounding each total
            // also bounds every phase and the sum of up to 64 accepted samples.
            std::uint64_t total = 0u;
            if (valid)
            {
                total = sample.at[PhaseCount] - sample.at[0];
                if (total > (std::numeric_limits<std::uint64_t>::max)() / SamplesPerWindow)
                    valid = false;
            }

            if (valid)
            {
                ++_window.accepted;
                const unsigned bucket = !sample.processValid ? InvalidPDO :
                    sample.ownerBegin != sample.ownerEnd ? OwnerChange :
                    sample.ownerBegin == 0u ? Idle :
                    sample.ownerBegin == 1u ? Auto : Other;
                Stats& stats = _window.buckets[bucket];
                ++stats.count;
                for (std::uint32_t metric = 0u; metric < MetricCount; ++metric)
                {
                    const std::uint64_t duration = metric == Total ? total :
                        sample.at[metric + 1u] - sample.at[metric];
                    stats.sumNs[metric] += duration;
                    if (duration > stats.maxNs[metric])
                        stats.maxNs[metric] = duration;
                }
                if (total > 250000u)
                    ++stats.over250;
            }
            else
            {
                ++_window.rejected;
            }

            if (_window.attempted != SamplesPerWindow)
                return false;

            output = _window;
            _window = Snapshot{};
            ++_nextWindowId;
            if (_nextWindowId == 0u)
                _nextWindowId = 1u;
            return true;
        }

    private:
        std::uint32_t _countdown;
        std::uint64_t _nextWindowId;
        Snapshot _window;
    };

    // Supplied by the runtime's existing snapshot publication channel.
    bool ReadRuntimeTiming(Snapshot& output, std::uint64_t& publication) noexcept;
}
