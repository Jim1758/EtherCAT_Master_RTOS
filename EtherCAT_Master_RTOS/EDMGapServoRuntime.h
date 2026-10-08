#pragma once

#include "EDMGapServo.h"
#include <cmath>
#include <cstdint>
#include <cstring>

// EDM55 source-neutral continuous GAP algorithm. One serialized RT owner only.
// This model has no clock reads, allocation, I/O, machine axis, Motion calls or
// discharge outputs. Its signed speed is advisory: +advance / -retreat along
// the caller's separately defined EDM path. A zero result or stopRequired is
// NOT evidence that any physical axis has stopped.
namespace EDM55
{
    constexpr bool PhysicalMotion = false;
    constexpr bool Discharge = false;

    enum class SourceKind : std::uint8_t { Simulation, Physical };

    struct Identity
    {
        SourceKind source = SourceKind::Simulation;
        std::uint64_t sourceId = 0ULL;
        std::uint64_t sourceGeneration = 0ULL;
        std::uint64_t calibrationRevision = 0ULL;
        std::uint64_t session = 0ULL;
    };

    struct Sample
    {
        bool valid = false;
        Identity identity{};
        std::uint64_t sequence = 0ULL;
        // Timestamp of the actual source conversion, never the current poll.
        std::uint64_t capturedAtUs = 0ULL;
        double voltageV = 0.0;
    };

    struct Config
    {
        std::uint64_t revision = 1ULL;
        // Reuses only the established static P/N curve and short-retreat speed.
        // The curve's millisecond detector settings do not control this model.
        EDMGapServo::Profile curve{};
        double referenceV = 60.0;
        double overridePercent = 100.0;
        double minVoltageV = 0.0;
        double maxVoltageV = 300.0;
        std::uint64_t maxAgeUs = 5000ULL;
        std::uint64_t maxTickGapUs = 2000ULL;
        std::uint64_t filterTauUs = 2000ULL;
        double shortThresholdV = 25.0;
        double shortHysteresisV = 5.0;
        std::uint64_t shortEnterUs = 2000ULL;
        std::uint64_t shortExitUs = 5000ULL;
        double accelerationMmPerMinPerSecond = 120.0;
        double decelerationMmPerMinPerSecond = 240.0;
        std::uint64_t reversalDwellUs = 5000ULL;

        Config() noexcept
        {
            for (unsigned i = 0U; i != 3U; ++i)
            {
                curve.positiveGain[i] = 100.0;
                curve.negativeGain[i] = 100.0;
            }
            curve.cuttingScaleMmPerVoltMin = 0.1;
            curve.maxFeedMmPerMin = 30.0;
            curve.maxRetreatMmPerMin = 30.0;
            curve.shortRetreatMmPerMin = 30.0;
            curve.deadbandV = 1.0;
        }
    };

    enum class Fault : std::uint8_t
    {
        None, Configuration, ConfigurationChange, Session, Identity, Sample,
        Sequence, SourceClock, Clock, Stale, Arithmetic
    };

    struct Output
    {
        bool valid = false;
        bool running = false;
        bool fresh = false;
        Fault fault = Fault::None;
        double rawVoltageV = 0.0;
        double filteredVoltageV = 0.0;
        double targetMmPerMin = 0.0;
        double speedMmPerMin = 0.0;
        bool feedInhibited = true;
        bool shortActive = false;
        bool stopRequired = false;
        std::uint64_t sequence = 0ULL;
        std::uint64_t capturedAtUs = 0ULL;
        std::uint64_t nowUs = 0ULL;
        std::uint64_t tick = 0ULL;
    };

    inline bool ValidIdentity(const Identity& value) noexcept
    {
        return (value.source == SourceKind::Simulation || value.source == SourceKind::Physical) &&
            value.sourceId != 0ULL && value.sourceGeneration != 0ULL &&
            value.calibrationRevision != 0ULL && value.session != 0ULL;
    }

    inline bool SameIdentity(const Identity& a, const Identity& b) noexcept
    {
        return a.source == b.source && a.sourceId == b.sourceId &&
            a.sourceGeneration == b.sourceGeneration &&
            a.calibrationRevision == b.calibrationRevision && a.session == b.session;
    }

    inline bool Validate(const Config& value) noexcept
    {
        if (value.revision == 0ULL || !EDMGapServo::ValidateProfile(value.curve)) return false;
        if (!std::isfinite(value.referenceV) || !std::isfinite(value.overridePercent) ||
            value.overridePercent < 0.0 || value.overridePercent > 100.0 ||
            !std::isfinite(value.minVoltageV) || !std::isfinite(value.maxVoltageV) ||
            value.minVoltageV < 0.0 || value.maxVoltageV <= value.minVoltageV ||
            value.referenceV <= value.minVoltageV || value.referenceV >= value.maxVoltageV)
            return false;
        if (!std::isfinite(value.shortThresholdV) || !std::isfinite(value.shortHysteresisV) ||
            value.shortThresholdV < value.minVoltageV || value.shortHysteresisV <= 0.0 ||
            !std::isfinite(value.shortThresholdV + value.shortHysteresisV) ||
            value.shortThresholdV + value.shortHysteresisV >= value.referenceV - value.curve.deadbandV ||
            value.curve.shortRetreatMmPerMin <= 0.0 ||
            value.curve.shortRetreatMmPerMin > value.curve.maxRetreatMmPerMin)
            return false;
        if (value.maxAgeUs == 0ULL || value.maxTickGapUs == 0ULL ||
            value.maxTickGapUs > value.maxAgeUs || value.filterTauUs == 0ULL ||
            value.shortEnterUs == 0ULL || value.shortExitUs < value.shortEnterUs ||
            value.reversalDwellUs == 0ULL)
            return false;
        if (!std::isfinite(value.accelerationMmPerMinPerSecond) ||
            !std::isfinite(value.decelerationMmPerMinPerSecond) ||
            value.accelerationMmPerMinPerSecond <= 0.0 ||
            value.decelerationMmPerMinPerSecond < value.accelerationMmPerMinPerSecond)
            return false;
        // Catch unusable curve arithmetic at both permitted voltage extremes,
        // independently of a zero override hiding excessive configured gains.
        if (!EDMGapServo::Evaluate(value.curve, value.minVoltageV, value.referenceV, 100.0).valid ||
            !EDMGapServo::Evaluate(value.curve, value.maxVoltageV, value.referenceV, 100.0).valid)
            return false;
        const double largestStep = value.decelerationMmPerMinPerSecond *
            (static_cast<double>(value.maxTickGapUs) / 1000000.0);
        return std::isfinite(largestStep) && largestStep > 0.0;
    }

    class Runtime
    {
    public:
        // A configuration attempt while running is a fault, even when equal.
        // Configure does not clear a sticky fault or reset the session fence.
        bool Configure(const Config& value) noexcept
        {
            if (m_running) { Fail(Fault::ConfigurationChange); return false; }
            if (!Validate(value))
            {
                m_configured = false;
                Fail(Fault::Configuration);
                return false;
            }
            m_config = value;
            m_configured = true;
            return true;
        }

        // Restart/HOLD resume requires a genuinely new session identity. A
        // successful newer Start clears the previous session's sticky fault,
        // but output stays invalid/zero until a validated fresh sample seeds it.
        // Never wrap/reuse session numbers; exhaustion cannot recover here.
        bool Start(const Identity& identity) noexcept
        {
            if (m_running) { Fail(Fault::Session); return false; }
            if (!m_configured) { Fail(Fault::Configuration); return false; }
            if (!ValidIdentity(identity) || identity.session <= m_lastSession)
            { Fail(Fault::Session); return false; }
            m_identity = identity;
            m_lastSession = identity.session;
            m_output = Output{};
            m_output.running = true;
            m_running = true;
            m_haveSample = false;
            m_haveClock = false;
            m_short = ShortState::Clear;
            m_shortSinceUs = 0ULL;
            m_lastDirection = 0;
            m_zeroSinceUs = 0ULL;
            m_zeroTracked = false;
            m_lastSample = Sample{};
            return true;
        }

        // Cancellation is neutral; an earlier fault remains sticky. There is
        // no physical stop acknowledgement and no authority to resume here.
        void Cancel() noexcept
        {
            m_running = false;
            if (m_output.fault != Fault::None) { Fail(m_output.fault); return; }
            m_output = Output{};
        }

        bool Running() const noexcept { return m_running; }
        Output Current() const noexcept { return m_output; }

        // Call once per real RT tick with the retained source sample. Duplicate
        // source samples may drive the continuous ramp, but cannot refresh age,
        // filter state or short-circuit source-time dwell.
        Output Step(const Sample& sample, std::uint64_t nowUs, std::uint64_t tick) noexcept
        {
            if (!m_running) return m_output;
            m_output.fresh = false;
            m_output.stopRequired = false;
            if (nowUs == 0ULL || tick == 0ULL || (m_haveClock &&
                (nowUs <= m_output.nowUs || tick <= m_output.tick ||
                    nowUs - m_output.nowUs > m_config.maxTickGapUs)))
                return Fail(Fault::Clock);
            const std::uint64_t dtUs = m_haveClock ? nowUs - m_output.nowUs : 0ULL;
            m_output.nowUs = nowUs;
            m_output.tick = tick;
            m_haveClock = true;
            // Check the retained source first: a fresh late arrival cannot
            // retroactively repair an interval that already lost input validity.
            if (m_haveSample && nowUs - m_lastSample.capturedAtUs > m_config.maxAgeUs)
                return Fail(Fault::Stale);
            if (!sample.valid || !std::isfinite(sample.voltageV) ||
                sample.voltageV < m_config.minVoltageV || sample.voltageV > m_config.maxVoltageV)
                return Fail(Fault::Sample);
            if (!ValidIdentity(sample.identity) || !SameIdentity(sample.identity, m_identity))
                return Fail(Fault::Identity);
            if (sample.sequence == 0ULL || (m_haveSample && sample.sequence < m_lastSample.sequence))
                return Fail(Fault::Sequence);
            if (sample.capturedAtUs == 0ULL || sample.capturedAtUs > nowUs)
                return Fail(Fault::SourceClock);
            if (nowUs - sample.capturedAtUs > m_config.maxAgeUs) return Fail(Fault::Stale);

            const bool first = !m_haveSample;
            const bool fresh = first || sample.sequence > m_lastSample.sequence;
            if (!fresh)
            {
                if (sample.capturedAtUs != m_lastSample.capturedAtUs ||
                    !SameDouble(sample.voltageV, m_lastSample.voltageV))
                    return Fail(Fault::Sequence);
            }
            else
            {
                if (!first && sample.capturedAtUs <= m_lastSample.capturedAtUs)
                    return Fail(Fault::SourceClock);
                double filtered = sample.voltageV;
                if (!first)
                {
                    const double sourceDtUs = static_cast<double>(sample.capturedAtUs - m_lastSample.capturedAtUs);
                    const double alpha = sourceDtUs / (static_cast<double>(m_config.filterTauUs) + sourceDtUs);
                    filtered = m_output.filteredVoltageV + alpha * (sample.voltageV - m_output.filteredVoltageV);
                }
                if (!std::isfinite(filtered)) return Fail(Fault::Arithmetic);
                m_output.rawVoltageV = sample.voltageV;
                m_output.filteredVoltageV = filtered;
                m_output.sequence = sample.sequence;
                m_output.capturedAtUs = sample.capturedAtUs;
                UpdateShort(sample);
                m_lastSample = sample;
                m_haveSample = true;
                m_output.fresh = true;
            }

            m_output.feedInhibited = m_short != ShortState::Clear;
            m_output.shortActive = m_short == ShortState::Active || m_short == ShortState::Exiting;
            const EDMGapServo::Result curve = EDMGapServo::Evaluate(m_config.curve,
                m_output.filteredVoltageV, m_config.referenceV, m_config.overridePercent);
            if (!curve.valid) return Fail(Fault::Arithmetic);
            double target = curve.limitedSpeedMmPerMin;
            if (m_output.shortActive)
                target = -m_config.curve.shortRetreatMmPerMin * (m_config.overridePercent / 100.0);
            else if (m_output.feedInhibited && target > 0.0) target = 0.0;
            if (!std::isfinite(target)) return Fail(Fault::Arithmetic);
            m_output.targetMmPerMin = target;
            m_output.valid = true;
            m_output.running = true;

            // Initial acquisition never launches a speed. Short detection is
            // seeded from this same raw sample; the filter is seeded once only.
            if (first)
            {
                m_output.speedMmPerMin = 0.0;
                m_zeroSinceUs = nowUs;
                m_zeroTracked = true;
                return m_output;
            }
            // Raw short admission bypasses the filtered curve and normal ramp.
            // The adapter must arrange a real stop through its existing safety
            // path; this immediate zero is only an advisory safety result.
            if (m_output.feedInhibited && m_output.speedMmPerMin > 0.0)
            {
                m_output.speedMmPerMin = 0.0;
                m_output.stopRequired = true;
                m_zeroSinceUs = nowUs;
                m_zeroTracked = true;
                return m_output;
            }
            if (!Ramp(target, dtUs, nowUs)) return Fail(Fault::Arithmetic);
            return m_output;
        }

    private:
        enum class ShortState : std::uint8_t { Clear, Entering, Active, Exiting };

        static bool SameDouble(double a, double b) noexcept
        {
            static_assert(sizeof(double) == sizeof(std::uint64_t), "EDM55 requires 64-bit double");
            std::uint64_t aBits = 0ULL;
            std::uint64_t bBits = 0ULL;
            std::memcpy(&aBits, &a, sizeof(aBits));
            std::memcpy(&bBits, &b, sizeof(bBits));
            return aBits == bBits;
        }

        Output Fail(Fault fault) noexcept
        {
            // Retain the first fault and last genuine source observations.
            if (m_output.fault == Fault::None) m_output.fault = fault;
            m_running = false;
            m_output.running = false;
            m_output.valid = false;
            m_output.fresh = false;
            m_output.speedMmPerMin = 0.0;
            m_output.targetMmPerMin = 0.0;
            m_output.feedInhibited = true;
            m_output.stopRequired = true;
            return m_output;
        }

        void UpdateShort(const Sample& sample) noexcept
        {
            if (m_short == ShortState::Clear || m_short == ShortState::Entering)
            {
                if (sample.voltageV <= m_config.shortThresholdV)
                {
                    if (m_short != ShortState::Entering) m_shortSinceUs = sample.capturedAtUs;
                    m_short = sample.capturedAtUs - m_shortSinceUs >= m_config.shortEnterUs ?
                        ShortState::Active : ShortState::Entering;
                }
                else m_short = ShortState::Clear;
            }
            else
            {
                if (sample.voltageV > m_config.shortThresholdV + m_config.shortHysteresisV)
                {
                    if (m_short != ShortState::Exiting) m_shortSinceUs = sample.capturedAtUs;
                    m_short = sample.capturedAtUs - m_shortSinceUs >= m_config.shortExitUs ?
                        ShortState::Clear : ShortState::Exiting;
                }
                else m_short = ShortState::Active;
            }
        }

        bool Ramp(double target, std::uint64_t dtUs, std::uint64_t nowUs) noexcept
        {
            const double oldSpeed = m_output.speedMmPerMin;
            const int targetDirection = target > 0.0 ? 1 : (target < 0.0 ? -1 : 0);
            if (oldSpeed == 0.0 && targetDirection != 0 && m_lastDirection != 0 &&
                targetDirection != m_lastDirection)
            {
                if (!m_zeroTracked) { m_zeroSinceUs = nowUs; m_zeroTracked = true; }
                if (nowUs - m_zeroSinceUs < m_config.reversalDwellUs) return true;
            }
            // No step may cross through zero. First decelerate to an exact zero;
            // a later tick may accelerate the opposite way after the dwell.
            if ((oldSpeed > 0.0 && target < 0.0) || (oldSpeed < 0.0 && target > 0.0)) target = 0.0;
            const bool slowing = std::fabs(target) < std::fabs(oldSpeed);
            const double rate = slowing ? m_config.decelerationMmPerMinPerSecond :
                m_config.accelerationMmPerMinPerSecond;
            const double delta = rate * (static_cast<double>(dtUs) / 1000000.0);
            if (!std::isfinite(delta) || delta < 0.0) return false;
            double speed = oldSpeed;
            if (target > oldSpeed) speed = target - oldSpeed <= delta ? target : oldSpeed + delta;
            else if (target < oldSpeed) speed = oldSpeed - target <= delta ? target : oldSpeed - delta;
            if (!std::isfinite(speed) || speed > m_config.curve.maxFeedMmPerMin ||
                speed < -m_config.curve.maxRetreatMmPerMin) return false;
            if (speed == 0.0)
            {
                speed = 0.0; // Canonical positive zero; preserve a prior zero dwell.
                if (oldSpeed != 0.0 || !m_zeroTracked) { m_zeroSinceUs = nowUs; m_zeroTracked = true; }
            }
            else
            {
                m_lastDirection = speed > 0.0 ? 1 : -1;
                m_zeroTracked = false;
            }
            m_output.speedMmPerMin = speed;
            return true;
        }

        Config m_config{};
        Identity m_identity{};
        Sample m_lastSample{};
        Output m_output{};
        std::uint64_t m_lastSession = 0ULL;
        std::uint64_t m_shortSinceUs = 0ULL;
        std::uint64_t m_zeroSinceUs = 0ULL;
        int m_lastDirection = 0;
        ShortState m_short = ShortState::Clear;
        bool m_configured = false;
        bool m_running = false;
        bool m_haveSample = false;
        bool m_haveClock = false;
        bool m_zeroTracked = false;
    };
}
