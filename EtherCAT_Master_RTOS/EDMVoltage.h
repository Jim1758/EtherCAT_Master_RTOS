#pragma once

#include "EDMGapAcquisition.h"
#include <cstdint>
#include <limits>

// EDM15: owner-thread voltage observation, explicitly SYNTHETIC_SHADOW only.
// No hardware I/O, Motion, discharge output, allocation or implicit clock reads.
namespace EDMVoltage
{
    enum class Provenance : std::uint8_t { SyntheticShadow };
    enum class ProfileError : std::uint8_t
    { None, Schema, Revision, Mode, Identity, Calibration, VoltageRange, Thresholds, Timing };

    struct Profile
    {
        std::uint32_t schemaVersion = 1U, profileRevision = 0U;
        std::uint32_t deviceId = 0U, channelId = 0U, calibrationRevision = 0U;
        std::int32_t rawMin = 0, rawMax = 0, mvAtMin = 0, mvAtMax = 0;
        EDMGap::Config gap{};
        Provenance provenance = Provenance::SyntheticShadow;
    };

    inline const char* ProfileErrorName(ProfileError error) noexcept
    {
        switch (error)
        {
        case ProfileError::None: return "NONE";
        case ProfileError::Schema: return "SCHEMA";
        case ProfileError::Revision: return "REVISION";
        case ProfileError::Mode: return "MODE";
        case ProfileError::Identity: return "IDENTITY";
        case ProfileError::Calibration: return "CALIBRATION";
        case ProfileError::VoltageRange: return "VOLTAGE_RANGE";
        case ProfileError::Thresholds: return "THRESHOLDS";
        case ProfileError::Timing: return "TIMING";
        }
        return "UNKNOWN";
    }

    inline ProfileError ValidateProfile(const Profile& p) noexcept
    {
        if (p.schemaVersion != 1U) return ProfileError::Schema;
        if (p.profileRevision == 0U || p.calibrationRevision == 0U) return ProfileError::Revision;
        if (p.provenance != Provenance::SyntheticShadow) return ProfileError::Mode;
        if (p.deviceId == 0U) return ProfileError::Identity;
        if (p.rawMin >= p.rawMax || p.mvAtMin >= p.mvAtMax) return ProfileError::Calibration;
        // Software schema bound only; this is NOT a device voltage rating.
        if (p.gap.minMv < 0 || p.gap.maxMv > 1000000 || p.gap.minMv >= p.gap.maxMv ||
            p.mvAtMin < p.gap.minMv || p.mvAtMax > p.gap.maxMv) return ProfileError::VoltageRange;
        if (!(p.gap.minMv <= p.gap.lowEnterMv && p.gap.lowEnterMv < p.gap.lowExitMv &&
            p.gap.lowExitMv <= p.gap.highExitMv && p.gap.highExitMv < p.gap.highEnterMv &&
            p.gap.highEnterMv <= p.gap.maxMv)) return ProfileError::Thresholds;
        if (p.gap.maxAgeMs != EDMGapAcquisition::Ingress::MaximumAgeMs ||
            p.gap.dwellMs == 0ULL || p.gap.dwellMs > 1000ULL) return ProfileError::Timing;
        return ProfileError::None;
    }

    struct Snapshot
    {
        Provenance provenance = Provenance::SyntheticShadow;
        EDMGap::Source source = EDMGap::Source::NONE;
        EDMGap::Quality quality = EDMGap::Quality::NO_SAMPLE;
        EDMGap::Band band = EDMGap::Band::UNKNOWN, pendingBand = EDMGap::Band::UNKNOWN;
        EDMGapAcquisition::Reason reason = EDMGapAcquisition::Reason::AwaitSession;
        std::uint32_t profileRevision = 0U, calibrationRevision = 0U;
        std::uint32_t deviceId = 0U, channelId = 0U, clockDomain = 0U;
        std::uint64_t producerBoot = 0ULL, consumerSession = 0ULL;
        std::uint64_t sequence = 0ULL, capturedAtMs = 0ULL, observedAtMs = 0ULL, ageMs = 0ULL;
        std::int32_t rawCode = 0, voltageMv = 0, lastGoodRawCode = 0, lastGoodVoltageMv = 0;
        std::uint64_t lastGoodSequence = 0ULL, lastGoodCapturedAtMs = 0ULL;
        bool profileValid = false, armed = false, liveVoltageValid = false;
        bool lastGoodAvailable = false, ageKnown = false;
        constexpr bool PhysicalDischargeEnabled() const noexcept { return false; }
    };

    class Channel
    {
    public:
        Channel() noexcept = default;
        Channel(const Channel&) = delete;
        Channel& operator=(const Channel&) = delete;
        Channel(Channel&&) = delete;
        Channel& operator=(Channel&&) = delete;

        // Exactly one boot-time installation attempt. A second attempt closes
        // the channel rather than replacing the profile under existing frames.
        bool Initialize(const Profile& profile) noexcept
        {
            if (m_initialized)
            {
                m_ready = false;
                Revoke();
                return false;
            }
            m_initialized = true;
            m_ready = ValidateProfile(profile) == ProfileError::None;
            if (m_ready) m_profile = profile;
            Refresh(0ULL);
            return m_ready;
        }

        bool ProfileReady() const noexcept { return m_ready; }
        const Profile& GetProfile() const noexcept { return m_profile; }
        std::uint64_t SessionHighWater() const noexcept { return m_ingress.SessionHighWater(); }
        EDMGapAcquisition::Identity Identity() const noexcept { return m_identity; }

        // Sessions increase across HOLD/RESET/completion; no reset API exists.
        // The guarantee is within this retained Channel object's lifetime.
        bool Start(std::uint64_t producerBoot, std::uint32_t clockDomain,
            std::uint64_t nowMs) noexcept
        {
            m_started = false;
            m_haveLastGood = false;
            m_lastGoodRaw = m_lastGoodMv = 0;
            m_lastGoodSequence = m_lastGoodCapture = 0ULL;
            EDMGapAcquisition::Descriptor descriptor{};
            descriptor.identity.schemaVersion = m_ready ? m_profile.schemaVersion : 0U;
            descriptor.identity.deviceId = m_profile.deviceId;
            descriptor.identity.channelId = m_profile.channelId;
            descriptor.identity.calibrationRevision = m_profile.calibrationRevision;
            descriptor.identity.producerBoot = producerBoot;
            descriptor.identity.clockDomain = clockDomain;
            const std::uint64_t highWater = m_ingress.SessionHighWater();
            descriptor.identity.consumerSession = highWater == (std::numeric_limits<std::uint64_t>::max)()
                ? highWater : highWater + 1ULL;
            descriptor.rawMin = m_profile.rawMin;
            descriptor.rawMax = m_profile.rawMax;
            descriptor.mvAtMin = m_profile.mvAtMin;
            descriptor.mvAtMax = m_profile.mvAtMax;
            m_identity = descriptor.identity;
            m_started = m_ingress.Configure(descriptor, m_profile.gap, nowMs);
            Refresh(nowMs);
            return m_started;
        }

        Snapshot Publish(const EDMGapAcquisition::Frame& frame, std::uint64_t nowMs) noexcept
        {
            const EDMGapAcquisition::Snapshot& input = m_ingress.Publish(frame, nowMs);
            if (m_ready && m_started && !input.faulted && input.gap.quality == EDMGap::Quality::VALID &&
                frame.identity.Matches(m_identity) && frame.sequence == input.gap.sequence)
            {
                m_haveLastGood = true;
                m_lastGoodRaw = frame.rawCode;
                m_lastGoodMv = input.gap.voltageMv;
                m_lastGoodSequence = input.gap.sequence;
                m_lastGoodCapture = input.gap.sampledAtMs;
            }
            Refresh(nowMs);
            return m_current;
        }

        // Poll before every time-sensitive use. Current() is only the previous
        // observation, not a promise of freshness at a caller's later clock.
        Snapshot Poll(std::uint64_t nowMs) noexcept
        {
            m_ingress.Poll(nowMs);
            Refresh(nowMs);
            return m_current;
        }
        Snapshot Current() const noexcept { return m_current; }

        void Revoke() noexcept
        {
            m_started = false;
            m_ingress.Revoke();
            Refresh(m_current.observedAtMs);
        }

    private:
        void Refresh(std::uint64_t nowMs) noexcept
        {
            const EDMGapAcquisition::Snapshot& input = m_ingress.Current();
            Snapshot result{};
            result.profileValid = m_ready;
            result.profileRevision = m_profile.profileRevision;
            result.calibrationRevision = m_profile.calibrationRevision;
            result.deviceId = m_profile.deviceId;
            result.channelId = m_profile.channelId;
            result.clockDomain = m_identity.clockDomain;
            result.producerBoot = m_identity.producerBoot;
            result.consumerSession = m_identity.consumerSession;
            result.observedAtMs = nowMs;
            result.source = m_started || m_haveLastGood ? input.gap.source : EDMGap::Source::NONE;
            result.reason = input.reason;
            result.quality = input.gap.quality;
            if (!m_ready)
            {
                result.reason = EDMGapAcquisition::Reason::Configuration;
                result.quality = EDMGap::Quality::CONFIG_ERROR;
            }
            result.armed = m_ready && m_started && !input.faulted;
            result.liveVoltageValid = result.armed && m_haveLastGood &&
                input.gap.quality == EDMGap::Quality::VALID;
            result.lastGoodAvailable = m_haveLastGood;
            result.lastGoodRawCode = m_lastGoodRaw;
            result.lastGoodVoltageMv = m_lastGoodMv;
            result.lastGoodSequence = m_lastGoodSequence;
            result.lastGoodCapturedAtMs = m_lastGoodCapture;
            result.sequence = m_haveLastGood ? m_lastGoodSequence : 0ULL;
            result.capturedAtMs = m_haveLastGood ? m_lastGoodCapture : 0ULL;
            result.ageKnown = m_haveLastGood && nowMs >= m_lastGoodCapture &&
                input.reason != EDMGapAcquisition::Reason::ObserverClock;
            if (result.ageKnown) result.ageMs = nowMs - m_lastGoodCapture;
            if (result.liveVoltageValid)
            {
                result.rawCode = m_lastGoodRaw;
                result.voltageMv = m_lastGoodMv;
                result.band = input.gap.band;
                result.pendingBand = input.gap.pendingBand;
            }
            m_current = result;
        }

        Profile m_profile{};
        EDMGapAcquisition::Ingress m_ingress{};
        EDMGapAcquisition::Identity m_identity{};
        Snapshot m_current{};
        std::uint64_t m_lastGoodSequence = 0ULL, m_lastGoodCapture = 0ULL;
        std::int32_t m_lastGoodRaw = 0, m_lastGoodMv = 0;
        bool m_initialized = false, m_ready = false, m_started = false, m_haveLastGood = false;
    };

    static_assert(sizeof(Channel) <= 768U, "EDM15 voltage channel has bounded owner-thread storage");
}
