#pragma once

#include "EDMVoltage.h"
#include <cstdint>
#include <limits>

// EDM15: profile-dependent synthetic ADC cases, never hardware acquisition.
// Real NC time only paces Step(); all capture/age times below are virtual.
namespace EDMVoltageSimulationTest
{
    constexpr std::uint32_t CaseCount = 16U;
    inline const char* CaseName(std::uint32_t index) noexcept
    {
        static const char* const names[CaseCount] = {
            "PROFILE_MIN", "PROFILE_QUARTER", "PROFILE_HALF", "PROFILE_THREE_QUARTER",
            "PROFILE_MAX", "INVALID_FRAME", "RAW_RANGE", "STALE_BOUNDARY",
            "DUPLICATE_NO_REFRESH", "SEQUENCE_GAP", "IDENTITY_REJECT", "CAPTURE_FUTURE",
            "REVOKE_REARM", "OLD_SESSION", "CONFIG_REJECT", "FINAL_RECOVER"
        };
        return index < CaseCount ? names[index] : "UNKNOWN";
    }
    struct CaseResult
    {
        EDMVoltage::Snapshot actual{};
        std::uint32_t checks = 0U;
        bool passed = false;
    };
    class Scenario
    {
    public:
        bool Install(const EDMVoltage::Profile& profile) noexcept
        {
            m_live = false;
            return m_channel.Initialize(profile);
        }
        bool Ready() const noexcept { return m_channel.ProfileReady(); }
        const EDMVoltage::Profile& Profile() const noexcept { return m_channel.GetProfile(); }
        bool Restart() noexcept
        {
            Revoke();
            m_nextCase = 0U;
            m_live = m_channel.ProfileReady();
            return m_live;
        }
        void Revoke() noexcept { m_channel.Revoke(); m_live = false; }
        EDMVoltage::Snapshot Current() const noexcept { return m_channel.Current(); }
        std::uint64_t SessionHighWater() const noexcept { return m_channel.SessionHighWater(); }
        CaseResult Step(std::uint32_t index) noexcept
        {
            using namespace EDMGapAcquisition;
            CaseResult result{};
            result.passed = m_live && index == m_nextCase && index < CaseCount;
            const auto check = [&result](bool ok) { ++result.checks; result.passed = result.passed && ok; };
            const auto fault = [this, &check](Reason reason) {
                const auto s = m_channel.Current();
                check(!s.liveVoltageValid && !s.armed && s.reason == reason &&
                    s.voltageMv == 0 && s.rawCode == 0 && !s.PhysicalDischargeEnabled());
            };
            if (!result.passed) { result.actual = Current(); return result; }
            check(Begin());
            if (!result.passed) { result.actual = Current(); return result; }
            const auto& profile = Profile();
            if (index <= 4U)
            {
                const std::int32_t raw = RawFraction(index);
                const auto value = Push(raw, 0ULL);
                check(value.liveVoltageValid && value.rawCode == raw &&
                    value.voltageMv == ExpectedMv(raw));
                check(value.profileRevision == profile.profileRevision &&
                    value.calibrationRevision == profile.calibrationRevision &&
                    value.deviceId == profile.deviceId && value.channelId == profile.channelId &&
                    value.ageKnown && value.ageMs == 0ULL && value.lastGoodAvailable);
            }
            else switch (index)
            {
            case 5U:
            {
                const auto good = Push(RawFraction(2U), 0ULL);
                Frame frame = MakeFrame(RawFraction(2U), 1ULL); frame.valid = false;
                m_channel.Publish(frame, m_baseMs + 1ULL); fault(Reason::Invalid);
                check(Current().lastGoodAvailable && Current().lastGoodVoltageMv == good.voltageMv);
                break;
            }
            case 6U:
                // A full int32 raw domain has no representable outside value.
                // In that profile verify both extrema instead of overflowing a test value.
                if (profile.rawMin != (std::numeric_limits<std::int32_t>::min)())
                {
                    Push(profile.rawMin - 1, 0ULL); fault(Reason::RawRange);
                }
                else if (profile.rawMax != (std::numeric_limits<std::int32_t>::max)())
                {
                    Push(profile.rawMax + 1, 0ULL); fault(Reason::RawRange);
                }
                else
                {
                    check(Push(profile.rawMin, 0ULL).voltageMv == profile.mvAtMin);
                    check(Push(profile.rawMax, 1ULL).voltageMv == profile.mvAtMax);
                    check(Current().liveVoltageValid);
                }
                break;
            case 7U:
                Push(RawFraction(2U), 0ULL);
                check(m_channel.Poll(m_baseMs + 100ULL).liveVoltageValid && Current().ageMs == 100ULL);
                m_channel.Poll(m_baseMs + 101ULL); fault(Reason::Stale);
                check(Current().lastGoodAvailable && Current().ageMs == 101ULL);
                break;
            case 8U:
            {
                const Frame same = MakeFrame(RawFraction(2U), 0ULL);
                m_channel.Publish(same, m_baseMs);
                check(m_channel.Publish(same, m_baseMs + 100ULL).liveVoltageValid);
                check(Current().sequence == same.sequence && Current().capturedAtMs == m_baseMs &&
                    Current().ageMs == 100ULL);
                m_channel.Publish(same, m_baseMs + 101ULL); fault(Reason::Stale);
                break;
            }
            case 9U:
            {
                Push(RawFraction(2U), 0ULL);
                Frame frame = MakeFrame(RawFraction(2U), 1ULL); ++frame.sequence;
                m_channel.Publish(frame, m_baseMs + 1ULL); fault(Reason::Sequence);
                break;
            }
            case 10U:
            {
                Frame frame = MakeFrame(RawFraction(2U), 0ULL);
                frame.identity.calibrationRevision ^= 1U;
                m_channel.Publish(frame, m_baseMs); fault(Reason::Identity);
                break;
            }
            case 11U:
            {
                Frame frame = MakeFrame(RawFraction(2U), 1ULL);
                m_channel.Publish(frame, m_baseMs); fault(Reason::AcquisitionClock);
                break;
            }
            case 12U:
            {
                const auto prior = Push(RawFraction(2U), 0ULL);
                m_channel.Revoke(); fault(Reason::Revoked);
                check(Current().lastGoodAvailable);
                check(Begin());
                check(!Current().lastGoodAvailable && !Current().liveVoltageValid &&
                    Current().consumerSession > prior.consumerSession);
                check(Push(RawFraction(2U), 0ULL).liveVoltageValid);
                break;
            }
            case 13U:
            {
                const Frame old = MakeFrame(RawFraction(2U), 0ULL);
                m_channel.Publish(old, m_baseMs);
                check(Begin());
                m_channel.Publish(old, m_baseMs); fault(Reason::Identity);
                check(!Current().lastGoodAvailable);
                break;
            }
            case 14U:
            {
                EDMVoltage::Profile bad = profile;
                bad.rawMax = bad.rawMin;
                EDMVoltage::Channel rejected{};
                check(EDMVoltage::ValidateProfile(bad) != EDMVoltage::ProfileError::None);
                check(!rejected.Initialize(bad) && !rejected.ProfileReady());
                check(!rejected.Start(1ULL, 1U, m_baseMs));
                check(!rejected.Current().liveVoltageValid && !rejected.Current().PhysicalDischargeEnabled());
                // Verify a rejected independent profile never alters the installed one.
                check(Push(RawFraction(2U), 0ULL).voltageMv == ExpectedMv(RawFraction(2U)));
                break;
            }
            case 15U:
            {
                const auto value = Push(RawFraction(2U), 0ULL);
                check(value.liveVoltageValid && value.quality == EDMGap::Quality::VALID &&
                    value.voltageMv == ExpectedMv(RawFraction(2U)));
                check(value.consumerSession == SessionHighWater() && value.sequence == 1ULL &&
                    value.provenance == EDMVoltage::Provenance::SyntheticShadow);
                break;
            }
            default: check(false); break;
            }
            result.actual = Current();
            check(!result.actual.PhysicalDischargeEnabled() &&
                result.actual.provenance == EDMVoltage::Provenance::SyntheticShadow);
            if (result.passed) ++m_nextCase;
            else Revoke();
            return result;
        }
    private:
        bool Begin() noexcept
        {
            if (m_baseMs > (std::numeric_limits<std::uint64_t>::max)() - 2048ULL) return false;
            m_baseMs += 1024ULL;
            m_sequence = 0ULL;
            return m_channel.Start(1ULL, 1U, m_baseMs);
        }
        std::int32_t RawFraction(std::uint32_t quarters) const noexcept
        {
            const auto& p = Profile();
            const std::int64_t span = static_cast<std::int64_t>(p.rawMax) - p.rawMin;
            return static_cast<std::int32_t>(static_cast<std::int64_t>(p.rawMin) + span * quarters / 4);
        }
        std::int32_t ExpectedMv(std::int32_t raw) const noexcept
        {
            const auto& p = Profile();
            // Independently arranged exact rational interpolation (bounded mV span <=1e6).
            const std::int64_t rawSpan = static_cast<std::int64_t>(p.rawMax) - p.rawMin;
            const std::int64_t numerator = (static_cast<std::int64_t>(raw) - p.rawMin) *
                (static_cast<std::int64_t>(p.mvAtMax) - p.mvAtMin);
            return static_cast<std::int32_t>(p.mvAtMin + numerator / rawSpan);
        }
        EDMGapAcquisition::Frame MakeFrame(std::int32_t raw, std::uint64_t offset) noexcept
        {
            EDMGapAcquisition::Frame frame{};
            frame.identity = m_channel.Identity();
            frame.rawCode = raw; frame.sequence = ++m_sequence;
            frame.capturedAtMs = m_baseMs + offset; frame.valid = true;
            return frame;
        }
        EDMVoltage::Snapshot Push(std::int32_t raw, std::uint64_t offset) noexcept
        {
            return m_channel.Publish(MakeFrame(raw, offset), m_baseMs + offset);
        }
        EDMVoltage::Channel m_channel{};
        std::uint64_t m_baseMs = 0ULL, m_sequence = 0ULL;
        std::uint32_t m_nextCase = 0U;
        bool m_live = false;
    };
    static_assert(sizeof(Scenario) <= 2048U, "EDM15 fixed simulation storage budget.");
}
