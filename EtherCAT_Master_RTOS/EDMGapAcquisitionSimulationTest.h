#pragma once

#include "EDMGapAcquisition.h"
#include <cstdint>
#include <limits>

// Deterministic tests of the production shadow adapter. Virtual acquisition
// clocks and synthetic raw values are not measurements from a physical device.
namespace EDMGapAcquisitionSimulationTest
{
    constexpr std::uint32_t CaseCount = 28U;
    inline const char* CaseName(std::uint32_t index) noexcept
    {
        static const char* const names[CaseCount] = {
            "ENDPOINTS", "SIGNED_FLOOR", "RAW_INT32_RANGE", "MV_INT32_RANGE",
            "LOW_NORMAL_HIGH", "BAD_CALIBRATION", "BAD_DESCRIPTOR", "RAW_RANGE",
            "INVALID_FRAME", "DEVICE_CHANNEL", "SCHEMA_CAL_REV", "PRODUCER_BOOT",
            "CONSUMER_ABA", "CLOCK_DOMAIN", "CAPTURE_FLOOR_FUTURE", "CAPTURE_REGRESSION",
            "OBSERVER_REGRESSION", "AGE_100_101", "LATE_FRESH", "DUPLICATE",
            "SEQUENCE_FAULTS", "FAULT_LATCH", "REVOKE_REARM", "SOURCE_MISMATCH",
            "MONITOR_CONFIG", "NO_SAMPLE_DEADLINE", "PRODUCER_RESTART", "SESSION_EXHAUSTION"
        };
        return index < CaseCount ? names[index] : "UNKNOWN";
    }

    struct CaseResult
    {
        EDMGapAcquisition::Snapshot actual{};
        std::uint32_t checks = 0U;
        bool passed = false;
    };

    class Scenario
    {
    public:
        void Restart() noexcept { m_ingress.Revoke(); m_nextCase = 0U; m_live = true; }
        void Revoke() noexcept { m_ingress.Revoke(); m_live = false; }
        const EDMGapAcquisition::Snapshot& Current() const noexcept { return m_ingress.Current(); }

        CaseResult Step(std::uint32_t index) noexcept
        {
            using namespace EDMGapAcquisition;
            CaseResult result{};
            result.passed = m_live && index == m_nextCase && index < CaseCount;
            const auto check = [&result](bool condition) { ++result.checks; result.passed = result.passed && condition; };
            const auto fault = [this, &check](Reason reason) {
                check(m_ingress.Current().faulted && m_ingress.Current().reason == reason &&
                    m_ingress.Current().gap.band == EDMGap::Band::UNKNOWN &&
                    !m_ingress.Current().PhysicalDischargeEnabled());
            };
            if (!result.passed) { result.actual = m_ingress.Current(); return result; }
            check(Begin());
            switch (index)
            {
            case 0U:
                check(Push(0, 0).gap.voltageMv == 0);
                check(Push(2000, 30).gap.voltageMv == 200000);
                break;
            case 1U:
                m_descriptor.rawMax = 3; m_descriptor.mvAtMin = -100; m_descriptor.mvAtMax = 100;
                m_config.minMv = -100;
                check(Reconfigure());
                check(Push(1, 0).gap.voltageMv == -34);
                check(Push(2, 30).gap.voltageMv == 33);
                break;
            case 2U:
                m_descriptor.rawMin = (std::numeric_limits<std::int32_t>::min)();
                m_descriptor.rawMax = (std::numeric_limits<std::int32_t>::max)();
                check(Reconfigure());
                check(Push(m_descriptor.rawMin, 0).gap.voltageMv == 0);
                check(Push(0, 30).gap.voltageMv == 100000);
                check(Push(m_descriptor.rawMax, 60).gap.voltageMv == 200000);
                break;
            case 3U:
                m_descriptor.rawMin = m_descriptor.mvAtMin = (std::numeric_limits<std::int32_t>::min)();
                m_descriptor.rawMax = m_descriptor.mvAtMax = (std::numeric_limits<std::int32_t>::max)();
                m_config.minMv = m_descriptor.mvAtMin; m_config.maxMv = m_descriptor.mvAtMax;
                check(Reconfigure());
                check(Push(m_descriptor.rawMin, 0).gap.voltageMv == m_descriptor.mvAtMin);
                check(Push(0, 30).gap.voltageMv == 0);
                check(Push(m_descriptor.rawMax, 60).gap.voltageMv == m_descriptor.mvAtMax);
                break;
            case 4U:
                Push(200, 0); check(Push(200, 30).gap.band == EDMGap::Band::LOW);
                Push(500, 60); check(Push(500, 90).gap.band == EDMGap::Band::NORMAL);
                Push(900, 120); check(Push(900, 150).gap.band == EDMGap::Band::HIGH);
                check(!m_ingress.Current().PhysicalDischargeEnabled());
                break;
            case 5U:
                for (unsigned int mutation = 0; mutation < 5U; ++mutation)
                {
                    check(Begin());
                    if (mutation == 0U) m_descriptor.rawMax = m_descriptor.rawMin;
                    if (mutation == 1U) m_descriptor.rawMin = m_descriptor.rawMax + 1;
                    if (mutation == 2U) m_descriptor.mvAtMax = m_descriptor.mvAtMin;
                    if (mutation == 3U) m_descriptor.mvAtMin = m_descriptor.mvAtMax + 1;
                    if (mutation == 4U) m_descriptor.mvAtMax = m_config.maxMv + 1;
                    check(!Reconfigure()); fault(Reason::Configuration);
                }
                break;
            case 6U:
                for (unsigned int mutation = 0; mutation < 5U; ++mutation)
                {
                    check(Begin());
                    if (mutation == 0U) m_descriptor.identity.schemaVersion = 2U;
                    if (mutation == 1U) m_descriptor.identity.deviceId = 0U;
                    if (mutation == 2U) m_descriptor.identity.calibrationRevision = 0U;
                    if (mutation == 3U) m_descriptor.identity.producerBoot = 0ULL;
                    if (mutation == 4U) m_descriptor.identity.clockDomain = 0U;
                    check(!Reconfigure()); fault(Reason::Configuration);
                }
                break;
            case 7U:
                Push(-1, 0); fault(Reason::RawRange);
                check(Begin()); Push(2001, 0); fault(Reason::RawRange);
                break;
            case 8U:
            {
                Frame frame = MakeFrame(500, 0); frame.valid = false;
                m_ingress.Publish(frame, m_baseMs); fault(Reason::Invalid);
                break;
            }
            case 9U:
            case 10U:
                for (unsigned int mutation = 0; mutation < 2U; ++mutation)
                {
                    check(Begin()); Frame frame = MakeFrame(500, 0);
                    if (index == 9U && mutation == 0U) ++frame.identity.deviceId;
                    if (index == 9U && mutation == 1U) ++frame.identity.channelId;
                    if (index == 10U && mutation == 0U) ++frame.identity.schemaVersion;
                    if (index == 10U && mutation == 1U) ++frame.identity.calibrationRevision;
                    m_ingress.Publish(frame, m_baseMs); fault(Reason::Identity);
                }
                break;
            case 11U:
            case 13U:
            case 23U:
            {
                Frame frame = MakeFrame(500, 0);
                if (index == 11U) ++frame.identity.producerBoot;
                if (index == 13U) ++frame.identity.clockDomain;
                if (index == 23U) frame.source = EDMGap::Source::SIMULATED;
                m_ingress.Publish(frame, m_baseMs);
                fault(index == 13U ? Reason::ClockDomain : Reason::Identity);
                break;
            }
            case 12U:
            {
                Frame old = MakeFrame(500, 0); m_ingress.Publish(old, m_baseMs);
                m_ingress.Revoke();
                check(!m_ingress.Configure(m_descriptor, m_config, m_baseMs));
                check(m_ingress.Current().faulted);
                check(Reconfigure());
                m_ingress.Publish(old, m_baseMs); fault(Reason::Identity);
                break;
            }
            case 14U:
                for (unsigned int mutation = 0; mutation < 2U; ++mutation)
                {
                    check(Begin()); Frame frame = MakeFrame(500, 0);
                    frame.capturedAtMs = mutation == 0U ? m_baseMs - 1ULL : m_baseMs + 1ULL;
                    m_ingress.Publish(frame, m_baseMs); fault(Reason::AcquisitionClock);
                }
                break;
            case 15U:
                Push(500, 30); Push(500, 20, 40); fault(Reason::AcquisitionClock);
                break;
            case 16U:
                Push(500, 30); m_ingress.Poll(m_baseMs + 29ULL); fault(Reason::ObserverClock);
                break;
            case 17U:
                Push(500, 0); check(!m_ingress.Poll(m_baseMs + 100ULL).faulted);
                m_ingress.Poll(m_baseMs + 101ULL); fault(Reason::Stale);
                break;
            case 18U:
                Push(500, 0); Push(500, 101); fault(Reason::Stale);
                break;
            case 19U:
            {
                Frame frame = MakeFrame(500, 0); m_ingress.Publish(frame, m_baseMs);
                const auto& duplicate = m_ingress.Publish(frame, m_baseMs + 100ULL);
                check(!duplicate.faulted && duplicate.gap.band == EDMGap::Band::UNKNOWN &&
                    duplicate.gap.sampledAtMs == m_baseMs && duplicate.gap.sequence == 1ULL);
                m_ingress.Publish(frame, m_baseMs + 101ULL); fault(Reason::Stale);
                break;
            }
            case 20U:
                for (unsigned int mutation = 0; mutation < 4U; ++mutation)
                {
                    check(Begin()); Frame frame = MakeFrame(500, 0);
                    if (mutation == 0U) frame.sequence = 0ULL;
                    if (mutation == 1U) frame.sequence = 2ULL;
                    if (mutation >= 2U) { m_ingress.Publish(frame, m_baseMs); }
                    if (mutation == 2U) ++frame.rawCode;
                    if (mutation == 3U) frame.sequence = 3ULL;
                    m_ingress.Publish(frame, m_baseMs + 10ULL); fault(Reason::Sequence);
                }
                break;
            case 21U:
                Push(-1, 0); fault(Reason::RawRange);
                Push(500, 30); fault(Reason::RawRange);
                check(Reconfigure()); Push(500, 0);
                check(Push(500, 30).gap.band == EDMGap::Band::NORMAL && !m_ingress.Current().faulted);
                break;
            case 22U:
                Push(500, 0); m_ingress.Revoke(); Push(500, 30); fault(Reason::Revoked);
                check(Reconfigure()); check(!Push(500, 0).faulted);
                break;
            case 24U:
                m_config.maxAgeMs = 101ULL;
                check(!Reconfigure()); fault(Reason::Configuration);
                check(Begin()); m_config.lowExitMv = m_config.lowEnterMv;
                check(!Reconfigure()); fault(Reason::Configuration);
                break;
            case 25U:
                check(m_ingress.Poll(m_baseMs + 100ULL).gap.quality == EDMGap::Quality::NO_SAMPLE);
                m_ingress.Poll(m_baseMs + 101ULL); fault(Reason::Stale);
                check(Begin()); Push(500, 101); fault(Reason::Stale);
                break;
            case 26U:
            {
                Frame old = MakeFrame(500, 0); m_ingress.Publish(old, m_baseMs);
                ++m_descriptor.identity.producerBoot; check(Reconfigure());
                check(!Push(500, 0).faulted);
                m_ingress.Publish(old, m_baseMs + 10ULL); fault(Reason::Identity);
                break;
            }
            case 27U:
            {
                // Separate bounded fixture: exhausting its lifetime must not
                // make the reusable main scenario impossible to restart.
                Ingress exhausted{}; Descriptor descriptor = m_descriptor;
                descriptor.identity.consumerSession = (std::numeric_limits<std::uint64_t>::max)();
                check(exhausted.Configure(descriptor, m_config, m_baseMs));
                exhausted.Revoke();
                check(!exhausted.Configure(descriptor, m_config, m_baseMs));
                descriptor.identity.consumerSession = 1ULL;
                check(!exhausted.Configure(descriptor, m_config, m_baseMs));
                check(exhausted.SessionHighWater() == (std::numeric_limits<std::uint64_t>::max)());
                break;
            }
            default: check(false); break;
            }
            check(!m_ingress.Current().PhysicalDischargeEnabled());
            result.actual = m_ingress.Current();
            ++m_nextCase;
            return result;
        }

    private:
        bool Reconfigure() noexcept
        {
            if (m_serial == (std::numeric_limits<std::uint64_t>::max)() ||
                m_baseMs > (std::numeric_limits<std::uint64_t>::max)() - 1000ULL) return false;
            m_descriptor.identity.consumerSession = ++m_serial;
            m_baseMs += 1000ULL;
            m_sequence = 0ULL;
            return m_ingress.Configure(m_descriptor, m_config, m_baseMs);
        }
        bool Begin() noexcept
        {
            m_descriptor = EDMGapAcquisition::Descriptor{};
            m_descriptor.identity.deviceId = 1U;
            m_descriptor.identity.channelId = 0U;
            m_descriptor.identity.calibrationRevision = 1U;
            m_descriptor.identity.producerBoot = 1ULL;
            m_descriptor.identity.clockDomain = 1U;
            m_descriptor.rawMax = 2000; m_descriptor.mvAtMax = 200000;
            m_config = EDMGap::Config{};
            return Reconfigure();
        }
        EDMGapAcquisition::Frame MakeFrame(std::int32_t raw, std::uint64_t offset) noexcept
        {
            EDMGapAcquisition::Frame frame{};
            frame.identity = m_descriptor.identity;
            frame.sequence = ++m_sequence;
            frame.capturedAtMs = m_baseMs + offset;
            frame.rawCode = raw; frame.valid = true;
            return frame;
        }
        const EDMGapAcquisition::Snapshot& Push(std::int32_t raw, std::uint64_t offset,
            std::uint64_t observedOffset = 0ULL) noexcept
        {
            return m_ingress.Publish(MakeFrame(raw, offset),
                m_baseMs + (observedOffset == 0ULL ? offset : observedOffset));
        }
        EDMGapAcquisition::Ingress m_ingress{};
        EDMGapAcquisition::Descriptor m_descriptor{};
        EDMGap::Config m_config{};
        std::uint64_t m_serial = 0ULL, m_baseMs = 0ULL, m_sequence = 0ULL;
        std::uint32_t m_nextCase = 0U;
        bool m_live = false;
    };
    static_assert(sizeof(Scenario) <= 768U, "ADC shadow scenario has fixed storage");
}
