#pragma once

#include <cstdint>

// CG observes GAP input only. These defaults are simulation values, not machine
// protection settings. One owning thread supplies an explicit source session
// and a monotonic millisecond clock; this class performs no I/O or motion.
namespace EDMGap
{
    enum class Source : std::uint8_t { NONE, SIMULATED, PHYSICAL };
    enum class Quality : std::uint8_t
    {
        NO_SAMPLE, VALID, INVALID, STALE, OUT_OF_RANGE, SEQUENCE_ERROR,
        CLOCK_ERROR, SOURCE_MISMATCH, CONFIG_ERROR
    };
    enum class Band : std::uint8_t { UNKNOWN, LOW, NORMAL, HIGH };

    struct Sample
    {
        std::int32_t voltageMv = 0;
        std::uint64_t sequence = 0;
        std::uint64_t sampledAtMs = 0;
        Source source = Source::NONE;
        bool valid = false;
    };

    struct Config
    {
        std::int32_t minMv = 0;
        std::int32_t maxMv = 200000;
        std::int32_t lowEnterMv = 30000;
        std::int32_t lowExitMv = 35000;
        std::int32_t highExitMv = 75000;
        std::int32_t highEnterMv = 80000;
        std::uint64_t maxAgeMs = 100;
        std::uint64_t dwellMs = 30;
    };

    struct Snapshot
    {
        Quality quality = Quality::NO_SAMPLE;
        Band band = Band::UNKNOWN;
        Band pendingBand = Band::UNKNOWN;
        Source source = Source::NONE;
        std::int32_t voltageMv = 0;
        std::uint64_t sequence = 0;
        std::uint64_t sampledAtMs = 0;
        std::uint64_t observedAtMs = 0;
        std::uint64_t pendingSinceMs = 0;
        bool configured = false;
    };

    inline const char* SourceName(Source value)
    {
        switch (value)
        {
        case Source::NONE: return "NONE";
        case Source::SIMULATED: return "SIMULATED";
        case Source::PHYSICAL: return "PHYSICAL";
        }
        return "UNKNOWN";
    }

    inline const char* QualityName(Quality value)
    {
        switch (value)
        {
        case Quality::NO_SAMPLE: return "NO_SAMPLE";
        case Quality::VALID: return "VALID";
        case Quality::INVALID: return "INVALID";
        case Quality::STALE: return "STALE";
        case Quality::OUT_OF_RANGE: return "OUT_OF_RANGE";
        case Quality::SEQUENCE_ERROR: return "SEQUENCE_ERROR";
        case Quality::CLOCK_ERROR: return "CLOCK_ERROR";
        case Quality::SOURCE_MISMATCH: return "SOURCE_MISMATCH";
        case Quality::CONFIG_ERROR: return "CONFIG_ERROR";
        }
        return "UNKNOWN";
    }

    inline const char* BandName(Band value)
    {
        switch (value)
        {
        case Band::UNKNOWN: return "UNKNOWN";
        case Band::LOW: return "LOW";
        case Band::NORMAL: return "NORMAL";
        case Band::HIGH: return "HIGH";
        }
        return "UNKNOWN";
    }

    class Monitor
    {
    public:
        bool Configure(const Config& config, Source source)
        {
            m_config = config;
            m_expectedSource = source;
            m_configured = (source == Source::SIMULATED || source == Source::PHYSICAL)
                && config.minMv <= config.lowEnterMv
                && config.lowEnterMv < config.lowExitMv
                && config.lowExitMv <= config.highExitMv
                && config.highExitMv < config.highEnterMv
                && config.highEnterMv <= config.maxMv
                && config.maxAgeMs > 0;
            Reset();
            return m_configured;
        }

        // Reset starts a new sequence/clock session and preserves configuration.
        // Invalid configuration remains closed until a successful Configure.
        void Reset()
        {
            m_snapshot = Snapshot{};
            m_snapshot.configured = m_configured;
            if (!m_configured) m_snapshot.quality = Quality::CONFIG_ERROR;
            m_latestSample = Sample{};
            m_lastAcquiredAtMs = 0;
            m_lastNowMs = 0;
            m_pendingSequence = 0;
            m_haveSample = false;
            m_haveAcquisitionTime = false;
            m_haveLocalTime = false;
            m_clockFault = false;
        }

        const Snapshot& Current() const { return m_snapshot; }

        const Snapshot& Poll(std::uint64_t nowMs)
        {
            if (ObserveClock(nowMs)) ExpirePrior(nowMs);
            return m_snapshot;
        }

        // Sequence zero is reserved. Identical duplicates never refresh acquisition
        // time or finish dwell. Every non-clock-latched error needs a strictly newer
        // acceptable sample to recover; Poll and duplicates cannot restore validity.
        const Snapshot& Publish(const Sample& sample, std::uint64_t nowMs)
        {
            if (!ObserveClock(nowMs)) return m_snapshot;
            ExpirePrior(nowMs);
            m_snapshot.source = sample.source;
            m_snapshot.voltageMv = sample.voltageMv;
            m_snapshot.sequence = sample.sequence;
            m_snapshot.sampledAtMs = sample.sampledAtMs;
            if (sample.source != m_expectedSource)
                return Fail(Quality::SOURCE_MISMATCH);
            if (sample.sequence == 0 || (m_haveSample && sample.sequence < m_latestSample.sequence))
                return Fail(Quality::SEQUENCE_ERROR);
            if (m_haveSample && sample.sequence == m_latestSample.sequence)
            {
                if (!SameSample(sample, m_latestSample)) return Fail(Quality::SEQUENCE_ERROR);
                return m_snapshot;
            }
            m_latestSample = sample;
            m_haveSample = true;
            if (sample.sampledAtMs > nowMs
                || (m_haveAcquisitionTime && sample.sampledAtMs < m_lastAcquiredAtMs))
                return Fail(Quality::CLOCK_ERROR);
            m_lastAcquiredAtMs = sample.sampledAtMs;
            m_haveAcquisitionTime = true;
            if (!sample.valid) return Fail(Quality::INVALID);
            if (sample.voltageMv < m_config.minMv || sample.voltageMv > m_config.maxMv)
                return Fail(Quality::OUT_OF_RANGE);
            if (nowMs - sample.sampledAtMs > m_config.maxAgeMs) return Fail(Quality::STALE);

            m_snapshot.quality = Quality::VALID;
            const Band target = Classify(sample.voltageMv);
            if (target == m_snapshot.band)
            {
                ClearPending();
            }
            else if (target != m_snapshot.pendingBand)
            {
                m_snapshot.pendingBand = target;
                m_snapshot.pendingSinceMs = sample.sampledAtMs;
                m_pendingSequence = sample.sequence;
            }
            else if (sample.sequence > m_pendingSequence
                && sample.sampledAtMs - m_snapshot.pendingSinceMs >= m_config.dwellMs)
            {
                m_snapshot.band = target;
                ClearPending();
            }
            return m_snapshot;
        }

    private:
        bool ObserveClock(std::uint64_t nowMs)
        {
            m_snapshot.observedAtMs = nowMs;
            if (!m_configured)
            {
                Fail(Quality::CONFIG_ERROR);
                return false;
            }
            if (m_clockFault || (m_haveLocalTime && nowMs < m_lastNowMs))
            {
                m_clockFault = true;
                Fail(Quality::CLOCK_ERROR);
                return false;
            }
            m_lastNowMs = nowMs;
            m_haveLocalTime = true;
            return true;
        }

        void ExpirePrior(std::uint64_t nowMs)
        {
            if (m_snapshot.quality == Quality::VALID
                && nowMs - m_latestSample.sampledAtMs > m_config.maxAgeMs)
                Fail(Quality::STALE);
        }

        const Snapshot& Fail(Quality quality)
        {
            m_snapshot.quality = quality;
            m_snapshot.band = Band::UNKNOWN;
            ClearPending();
            return m_snapshot;
        }

        void ClearPending()
        {
            m_snapshot.pendingBand = Band::UNKNOWN;
            m_snapshot.pendingSinceMs = 0;
            m_pendingSequence = 0;
        }

        Band Classify(std::int32_t voltageMv) const
        {
            if (m_snapshot.band == Band::LOW && voltageMv < m_config.lowExitMv) return Band::LOW;
            if (m_snapshot.band == Band::HIGH && voltageMv > m_config.highExitMv) return Band::HIGH;
            if (voltageMv <= m_config.lowEnterMv) return Band::LOW;
            if (voltageMv >= m_config.highEnterMv) return Band::HIGH;
            return Band::NORMAL;
        }

        static bool SameSample(const Sample& a, const Sample& b)
        {
            return a.voltageMv == b.voltageMv && a.sequence == b.sequence
                && a.sampledAtMs == b.sampledAtMs && a.source == b.source && a.valid == b.valid;
        }

        Config m_config{};
        Snapshot m_snapshot{};
        Sample m_latestSample{};
        std::uint64_t m_lastAcquiredAtMs = 0;
        std::uint64_t m_lastNowMs = 0;
        std::uint64_t m_pendingSequence = 0;
        Source m_expectedSource = Source::NONE;
        bool m_configured = false;
        bool m_haveSample = false;
        bool m_haveAcquisitionTime = false;
        bool m_haveLocalTime = false;
        bool m_clockFault = false;
    };

    static_assert(sizeof(Monitor) <= 256, "GAP monitor exceeds its fixed storage budget");
}
