#pragma once

#include "EDMGapSignal.h"
#include <cstdint>
#include <limits>

// Adapter-neutral PHYSICAL-classified shadow only. This object performs no I/O,
// clock reads, allocation, Motion or discharge control. A driver must explicitly
// map its device counter and timestamp into the declared adapter stream/domain.
namespace EDMGapAcquisition
{
    enum class Reason : std::uint8_t
    {
        None, AwaitSession, Configuration, Session, Identity, ClockDomain,
        Sequence, AcquisitionClock, ObserverClock, Invalid, RawRange, Stale, Revoked
    };

    struct Identity
    {
        std::uint32_t schemaVersion = 1U, deviceId = 0U, channelId = 0U;
        std::uint32_t calibrationRevision = 0U, clockDomain = 0U;
        std::uint64_t producerBoot = 0ULL, consumerSession = 0ULL;
        bool Matches(const Identity& other) const noexcept
        {
            return schemaVersion == other.schemaVersion && deviceId == other.deviceId &&
                channelId == other.channelId && calibrationRevision == other.calibrationRevision &&
                clockDomain == other.clockDomain && producerBoot == other.producerBoot &&
                consumerSession == other.consumerSession;
        }
    };

    struct Descriptor
    {
        Identity identity{};
        std::int32_t rawMin = 0, rawMax = 0, mvAtMin = 0, mvAtMax = 0;
    };

    struct Frame
    {
        Identity identity{};
        std::uint64_t sequence = 0ULL, capturedAtMs = 0ULL;
        std::int32_t rawCode = 0;
        EDMGap::Source source = EDMGap::Source::PHYSICAL;
        bool valid = false;
    };

    struct Snapshot
    {
        EDMGap::Snapshot gap{};
        Reason reason = Reason::AwaitSession;
        std::uint64_t consumerSession = 0ULL;
        bool faulted = true;
        constexpr bool PhysicalDischargeEnabled() const noexcept { return false; }
    };

    inline const char* ReasonName(Reason reason) noexcept
    {
        switch (reason)
        {
        case Reason::None: return "NONE";
        case Reason::AwaitSession: return "AWAIT_SESSION";
        case Reason::Configuration: return "CONFIGURATION";
        case Reason::Session: return "SESSION";
        case Reason::Identity: return "IDENTITY";
        case Reason::ClockDomain: return "CLOCK_DOMAIN";
        case Reason::Sequence: return "SEQUENCE";
        case Reason::AcquisitionClock: return "ACQUISITION_CLOCK";
        case Reason::ObserverClock: return "OBSERVER_CLOCK";
        case Reason::Invalid: return "INVALID";
        case Reason::RawRange: return "RAW_RANGE";
        case Reason::Stale: return "STALE";
        case Reason::Revoked: return "REVOKED";
        }
        return "UNKNOWN";
    }

    class Ingress
    {
    public:
        static constexpr std::uint64_t MaximumAgeMs = 100ULL;

        // No Reset API: attempted consumer generations survive faults/revocation.
        // Every newer nonzero attempt reserves its generation before validation;
        // a failed descriptor cannot be reinterpreted under the same frame scope.
        bool Configure(const Descriptor& descriptor, const EDMGap::Config& config,
            std::uint64_t armedAtMs) noexcept
        {
            if (descriptor.identity.consumerSession == 0ULL ||
                descriptor.identity.consumerSession <= m_sessionHighWater)
            {
                Fail(Reason::Session, EDMGap::Quality::SEQUENCE_ERROR);
                return false;
            }
            m_sessionHighWater = descriptor.identity.consumerSession;
            const Identity& id = descriptor.identity;
            EDMGap::Monitor candidate{};
            if (id.schemaVersion != 1U || id.deviceId == 0U ||
                id.calibrationRevision == 0U || id.producerBoot == 0ULL || id.clockDomain == 0U ||
                descriptor.rawMin >= descriptor.rawMax || descriptor.mvAtMin >= descriptor.mvAtMax ||
                descriptor.mvAtMin < config.minMv || descriptor.mvAtMax > config.maxMv ||
                config.maxAgeMs != MaximumAgeMs ||
                !candidate.Configure(config, EDMGap::Source::PHYSICAL))
            {
                Fail(Reason::Configuration, EDMGap::Quality::CONFIG_ERROR);
                return false;
            }
            if (m_haveClock && id.clockDomain == m_descriptor.identity.clockDomain && armedAtMs < m_lastNowMs)
            {
                Fail(Reason::ObserverClock, EDMGap::Quality::CLOCK_ERROR);
                return false;
            }
            m_descriptor = descriptor;
            m_monitor = candidate;
            m_armedAtMs = m_lastNowMs = armedAtMs;
            m_haveClock = true;
            m_haveFrame = false;
            m_lastFrame = Frame{};
            m_current = Snapshot{};
            m_current.gap = m_monitor.Poll(armedAtMs);
            m_current.consumerSession = id.consumerSession;
            m_current.reason = Reason::None;
            m_current.faulted = false;
            return true;
        }

        const Snapshot& Current() const noexcept { return m_current; }
        std::uint64_t SessionHighWater() const noexcept { return m_sessionHighWater; } // Highest attempted.

        void Revoke() noexcept { Fail(Reason::Revoked, EDMGap::Quality::INVALID); }

        const Snapshot& Poll(std::uint64_t nowMs) noexcept
        {
            if (m_current.faulted) return m_current;
            if (nowMs < m_lastNowMs) return Fail(Reason::ObserverClock, EDMGap::Quality::CLOCK_ERROR);
            m_lastNowMs = nowMs;
            m_current.gap = m_monitor.Poll(nowMs);
            const std::uint64_t origin = m_haveFrame ? m_lastFrame.capturedAtMs : m_armedAtMs;
            if (nowMs - origin > MaximumAgeMs) return Fail(Reason::Stale, EDMGap::Quality::STALE);
            return m_current;
        }

        const Snapshot& Publish(const Frame& frame, std::uint64_t nowMs) noexcept
        {
            // A newly arrived frame cannot hide an expired previously accepted
            // sample, nor an expired no-sample admission window.
            Poll(nowMs);
            if (m_current.faulted) return m_current;
            if (frame.identity.clockDomain != m_descriptor.identity.clockDomain)
                return Fail(Reason::ClockDomain, EDMGap::Quality::CLOCK_ERROR);
            if (!frame.identity.Matches(m_descriptor.identity) || frame.source != EDMGap::Source::PHYSICAL)
                return Fail(Reason::Identity, EDMGap::Quality::SOURCE_MISMATCH);
            if (m_haveFrame && frame.sequence == m_lastFrame.sequence)
            {
                if (!SameFrame(frame, m_lastFrame)) return Fail(Reason::Sequence, EDMGap::Quality::SEQUENCE_ERROR);
                return m_current; // Poll ages the original acquisition; no dwell progress.
            }
            if (frame.sequence == 0ULL || (!m_haveFrame && frame.sequence != 1ULL) ||
                (m_haveFrame && (m_lastFrame.sequence == (std::numeric_limits<std::uint64_t>::max)() ||
                    frame.sequence != m_lastFrame.sequence + 1ULL)))
                return Fail(Reason::Sequence, EDMGap::Quality::SEQUENCE_ERROR);
            if (frame.capturedAtMs < m_armedAtMs || frame.capturedAtMs > nowMs ||
                (m_haveFrame && frame.capturedAtMs < m_lastFrame.capturedAtMs))
                return Fail(Reason::AcquisitionClock, EDMGap::Quality::CLOCK_ERROR);
            if (nowMs - frame.capturedAtMs > MaximumAgeMs) return Fail(Reason::Stale, EDMGap::Quality::STALE);
            if (!frame.valid) return Fail(Reason::Invalid, EDMGap::Quality::INVALID);
            if (frame.rawCode < m_descriptor.rawMin || frame.rawCode > m_descriptor.rawMax)
                return Fail(Reason::RawRange, EDMGap::Quality::OUT_OF_RANGE);

            const std::uint64_t offset = static_cast<std::uint64_t>(
                static_cast<std::int64_t>(frame.rawCode) - m_descriptor.rawMin);
            const std::uint64_t rawSpan = static_cast<std::uint64_t>(
                static_cast<std::int64_t>(m_descriptor.rawMax) - m_descriptor.rawMin);
            const std::uint64_t mvSpan = static_cast<std::uint64_t>(
                static_cast<std::int64_t>(m_descriptor.mvAtMax) - m_descriptor.mvAtMin);
            // Both factors <= UINT32_MAX. Their product fits uint64, including
            // the full signed-int32 ranges. Unsigned division defines floor.
            const std::int64_t mv = static_cast<std::int64_t>(m_descriptor.mvAtMin) +
                static_cast<std::int64_t>((offset * mvSpan) / rawSpan);
            EDMGap::Sample sample{};
            sample.voltageMv = static_cast<std::int32_t>(mv);
            sample.sequence = frame.sequence;
            sample.sampledAtMs = frame.capturedAtMs;
            sample.source = EDMGap::Source::PHYSICAL;
            sample.valid = true;
            m_current.gap = m_monitor.Publish(sample, nowMs);
            m_lastFrame = frame;
            m_haveFrame = true;
            return m_current;
        }

    private:
        const Snapshot& Fail(Reason reason, EDMGap::Quality quality) noexcept
        {
            // Preserve the first fault until an explicit newer valid session.
            if (!m_current.faulted || m_current.reason == Reason::AwaitSession)
            {
                m_current.reason = reason;
                m_current.gap.quality = quality;
            }
            m_current.faulted = true;
            m_current.gap.band = m_current.gap.pendingBand = EDMGap::Band::UNKNOWN;
            m_current.gap.pendingSinceMs = 0ULL;
            return m_current;
        }

        static bool SameFrame(const Frame& a, const Frame& b) noexcept
        {
            return a.identity.Matches(b.identity) && a.sequence == b.sequence &&
                a.capturedAtMs == b.capturedAtMs && a.rawCode == b.rawCode &&
                a.source == b.source && a.valid == b.valid;
        }

        Descriptor m_descriptor{};
        Frame m_lastFrame{};
        EDMGap::Monitor m_monitor{};
        Snapshot m_current{};
        std::uint64_t m_sessionHighWater = 0ULL, m_armedAtMs = 0ULL, m_lastNowMs = 0ULL;
        bool m_haveClock = false, m_haveFrame = false;
    };

    static_assert(sizeof(Ingress) <= 512U, "ADC shadow ingress has fixed storage");
}
