#pragma once

#include <cmath>
#include <cstdint>

// EDM20 deterministic SHADOW domain model. No I/O, allocation, clock reads,
// Motion calls or discharge outputs. Positive speed means machining advance;
// negative speed means retreat along the caller's separately defined EDM path.
// Numeric validity does not authorize a physical speed or select a machine axis.
namespace EDMGapServo
{
    struct Profile
    {
        std::uint32_t revision = 1U;
        double positiveGain[3] = { 25.0, 50.0, 50.0 };
        double negativeGain[3] = { 25.0, 50.0, 50.0 };
        // Positive magnitudes measured from referenceV, not absolute voltages.
        // The provisional second boundary preserves the previous second slope.
        double positiveBreakV[2] = { 10.0, 20.0 };
        double negativeBreakV[2] = { 10.0, 20.0 };
        double cuttingScaleMmPerVoltMin = 0.010;
        double maxFeedMmPerMin = 5.0;
        double maxRetreatMmPerMin = 5.0;
        bool limitsConfirmed = false;
        double deadbandV = 0.0;
        double shortIdleV = 10.0;
        double shortMachiningV = 0.0;
        // Zero volts is a real threshold. Only these flags disable detection.
        bool shortIdleEnabled = true;
        bool shortMachiningEnabled = true;
        std::uint32_t shortEnterMs = 2U;
        std::uint32_t shortExitMs = 5U;
        double shortHysteresisV = 1.0;
        // Zero uses only an already-negative current servo-curve speed.
        double shortRetreatMmPerMin = 0.5;
    };

    enum class ValidationError : std::uint8_t
    {
        None, Revision, Gain, BreakVoltage, CuttingScale, SpeedLimit, Deadband,
        ShortThreshold, ShortHysteresis, ShortRetreatSpeed
    };

    inline bool Finite(double value) noexcept { return std::isfinite(value); }

    inline bool ValidateProfile(const Profile& profile,
        ValidationError* error = nullptr) noexcept
    {
        ValidationError found = ValidationError::None;
        if (profile.revision == 0U) found = ValidationError::Revision;
        for (unsigned i = 0; i < 3U && found == ValidationError::None; ++i)
            if (!Finite(profile.positiveGain[i]) || profile.positiveGain[i] < 0.0 ||
                !Finite(profile.negativeGain[i]) || profile.negativeGain[i] < 0.0)
                found = ValidationError::Gain;
        if (found == ValidationError::None &&
            (!Finite(profile.positiveBreakV[0]) || !Finite(profile.positiveBreakV[1]) ||
                !Finite(profile.negativeBreakV[0]) || !Finite(profile.negativeBreakV[1]) ||
                profile.positiveBreakV[0] <= 0.0 ||
                profile.positiveBreakV[1] <= profile.positiveBreakV[0] ||
                profile.negativeBreakV[0] <= 0.0 ||
                profile.negativeBreakV[1] <= profile.negativeBreakV[0]))
            found = ValidationError::BreakVoltage;
        if (found == ValidationError::None &&
            (!Finite(profile.cuttingScaleMmPerVoltMin) || profile.cuttingScaleMmPerVoltMin < 0.001))
            found = ValidationError::CuttingScale;
        if (found == ValidationError::None &&
            (!Finite(profile.maxFeedMmPerMin) || profile.maxFeedMmPerMin <= 0.0 ||
                !Finite(profile.maxRetreatMmPerMin) || profile.maxRetreatMmPerMin <= 0.0))
            found = ValidationError::SpeedLimit;
        if (found == ValidationError::None &&
            (!Finite(profile.deadbandV) || profile.deadbandV < 0.0 ||
                profile.deadbandV >= profile.positiveBreakV[0] ||
                profile.deadbandV >= profile.negativeBreakV[0]))
            found = ValidationError::Deadband;
        if (found == ValidationError::None &&
            (!Finite(profile.shortIdleV) || profile.shortIdleV < 0.0 ||
                !Finite(profile.shortMachiningV) || profile.shortMachiningV < 0.0))
            found = ValidationError::ShortThreshold;
        if (found == ValidationError::None &&
            (!Finite(profile.shortHysteresisV) || profile.shortHysteresisV < 0.0 ||
                !Finite(profile.shortIdleV + profile.shortHysteresisV) ||
                !Finite(profile.shortMachiningV + profile.shortHysteresisV)))
            found = ValidationError::ShortHysteresis;
        if (found == ValidationError::None &&
            (!Finite(profile.shortRetreatMmPerMin) || profile.shortRetreatMmPerMin < 0.0))
            found = ValidationError::ShortRetreatSpeed;
        if (error) *error = found;
        return found == ValidationError::None;
    }

    enum class Error : std::uint8_t
    {
        None, Profile, Voltage, Override, Arithmetic, Sample, Sequence, Clock
    };

    struct Result
    {
        bool valid = false;
        Error error = Error::None;
        double errorV = 0.0;
        // Both speeds already include E8 override exactly once.
        double rawSpeedMmPerMin = 0.0;
        double limitedSpeedMmPerMin = 0.0;
        std::uint8_t segment = 0U; // 0=deadband; 1,2,3=selected P/N interval.
        bool saturated = false;
        bool limitsConfirmed = false; // Information only; never a run permit.
    };

    inline Result Evaluate(const Profile& profile, double actualV,
        double referenceV, double overridePercent) noexcept
    {
        Result result{};
        result.limitsConfirmed = profile.limitsConfirmed;
        if (!ValidateProfile(profile)) { result.error = Error::Profile; return result; }
        if (!Finite(actualV) || !Finite(referenceV)) { result.error = Error::Voltage; return result; }
        if (!Finite(overridePercent) || overridePercent < 0.0 || overridePercent > 100.0)
        { result.error = Error::Override; return result; }
        const double errorV = actualV - referenceV;
        if (!Finite(errorV)) { result.error = Error::Arithmetic; return result; }
        result.errorV = errorV;
        const double magnitude = std::fabs(errorV);
        if (magnitude <= profile.deadbandV) { result.valid = true; return result; }
        const bool positive = errorV > 0.0;
        const double* gain = positive ? profile.positiveGain : profile.negativeGain;
        const double* edge = positive ? profile.positiveBreakV : profile.negativeBreakV;
        const double firstEnd = magnitude < edge[0] ? magnitude : edge[0];
        double integral = (firstEnd - profile.deadbandV) * gain[0];
        result.segment = 1U;
        if (magnitude > edge[0])
        {
            const double secondEnd = magnitude < edge[1] ? magnitude : edge[1];
            integral += (secondEnd - edge[0]) * gain[1];
            result.segment = 2U;
        }
        if (magnitude > edge[1])
        {
            integral += (magnitude - edge[1]) * gain[2];
            result.segment = 3U;
        }
        // Cutting scale is already mm/(V*min); output is mm/min.
        const double raw = integral * profile.cuttingScaleMmPerVoltMin *
            (overridePercent / 100.0);
        if (!Finite(integral) || !Finite(raw))
        { result.error = Error::Arithmetic; result.segment = 0U; return result; }
        const double limit = positive ? profile.maxFeedMmPerMin : profile.maxRetreatMmPerMin;
        const double limited = raw > limit ? limit : raw;
        result.rawSpeedMmPerMin = positive ? raw : -raw;
        result.limitedSpeedMmPerMin = positive ? limited : -limited;
        result.saturated = raw > limit;
        result.valid = true;
        return result;
    }

    struct RetreatResult
    {
        bool valid = false;
        double signedSpeedMmPerMin = 0.0;
        bool usingCurve = false;
        bool saturated = false;
    };

    inline RetreatResult EvaluateShortRetreat(const Profile& profile,
        const Result& currentCurve) noexcept
    {
        RetreatResult result{};
        if (!ValidateProfile(profile) || !currentCurve.valid ||
            !Finite(currentCurve.limitedSpeedMmPerMin)) return result;
        result.valid = true;
        if (profile.shortRetreatMmPerMin == 0.0)
        {
            result.usingCurve = true;
            const double requested = currentCurve.limitedSpeedMmPerMin < 0.0 ?
                -currentCurve.limitedSpeedMmPerMin : 0.0;
            const double limited = requested > profile.maxRetreatMmPerMin ?
                profile.maxRetreatMmPerMin : requested;
            result.signedSpeedMmPerMin = -limited;
            result.saturated = requested > profile.maxRetreatMmPerMin ||
                (currentCurve.limitedSpeedMmPerMin < 0.0 && currentCurve.saturated);
        }
        else
        {
            result.saturated = profile.shortRetreatMmPerMin > profile.maxRetreatMmPerMin;
            result.signedSpeedMmPerMin = -(result.saturated ? profile.maxRetreatMmPerMin :
                profile.shortRetreatMmPerMin);
        }
        return result;
    }

    enum class ShortState : std::uint8_t { Clear, Entering, Active, Exiting, Invalid };

    struct Sample
    {
        bool valid = false;
        double voltage = 0.0;
        std::uint64_t nowMs = 0ULL;
        std::uint64_t sequence = 0ULL;
    };

    struct ShortResult
    {
        bool valid = false;
        Error error = Error::Sample;
        bool enabled = false;
        ShortState state = ShortState::Invalid;
        bool feedInhibited = true;
        bool shortActive = false;
        double thresholdV = 0.0;
    };

    class ShortDetector
    {
    public:
        // Caller must reset on source, calibration, session or path epoch change.
        // Step accepts fresh, quality-checked samples only. Sample.nowMs is the
        // SOURCE sample timestamp, not the observation/poll clock. The caller
        // checks age/cadence; repeating a sequence never accumulates dwell time.
        void Reset() noexcept { *this = ShortDetector{}; }

        ShortResult Step(const Profile& profile, const Sample& sample, bool machining) noexcept
        {
            const bool enabled = machining ? profile.shortMachiningEnabled : profile.shortIdleEnabled;
            const double threshold = machining ? profile.shortMachiningV : profile.shortIdleV;
            if (!ValidateProfile(profile)) return Fail(Error::Profile, enabled, threshold);
            if (!sample.valid || !Finite(sample.voltage)) return Fail(Error::Sample, enabled, threshold);
            if (sample.sequence == 0ULL || (m_haveSample && sample.sequence <= m_lastSequence))
                return Fail(Error::Sequence, enabled, threshold);
            if (m_haveSample && sample.nowMs < m_lastNowMs)
                return Fail(Error::Clock, enabled, threshold);

            const bool changed = !m_haveConfig || m_revision != profile.revision ||
                m_machining != machining || m_enabled != enabled || m_threshold != threshold ||
                m_hysteresis != profile.shortHysteresisV || m_enterMs != profile.shortEnterMs ||
                m_exitMs != profile.shortExitMs;
            if (changed) m_state = ShortState::Clear;
            m_haveConfig = true;
            m_revision = profile.revision;
            m_machining = machining;
            m_enabled = enabled;
            m_threshold = threshold;
            m_hysteresis = profile.shortHysteresisV;
            m_enterMs = profile.shortEnterMs;
            m_exitMs = profile.shortExitMs;
            m_haveSample = true;
            m_lastSequence = sample.sequence;
            m_lastNowMs = sample.nowMs;
            if (!enabled) m_state = ShortState::Clear;
            else if (m_state == ShortState::Clear || m_state == ShortState::Entering)
            {
                if (sample.voltage <= threshold)
                {
                    if (m_state != ShortState::Entering) m_pendingSinceMs = sample.nowMs;
                    m_state = (sample.nowMs - m_pendingSinceMs >= profile.shortEnterMs) ?
                        ShortState::Active : ShortState::Entering;
                }
                else m_state = ShortState::Clear;
            }
            else // Active or Exiting. Exit is strictly above threshold+hysteresis.
            {
                if (sample.voltage > threshold + profile.shortHysteresisV)
                {
                    if (m_state != ShortState::Exiting) m_pendingSinceMs = sample.nowMs;
                    m_state = (sample.nowMs - m_pendingSinceMs >= profile.shortExitMs) ?
                        ShortState::Clear : ShortState::Exiting;
                }
                else m_state = ShortState::Active;
            }
            ShortResult result{};
            result.valid = true;
            result.error = Error::None;
            result.enabled = enabled;
            result.state = m_state;
            result.feedInhibited = enabled && m_state != ShortState::Clear;
            result.shortActive = enabled && (m_state == ShortState::Active || m_state == ShortState::Exiting);
            result.thresholdV = threshold;
            return result;
        }

    private:
        ShortResult Fail(Error error, bool enabled, double threshold) noexcept
        {
            // Faults remove dwell evidence while preserving accepted sequence
            // and clock high-water marks. Only explicit Reset opens a new stream.
            // No retreat/advance is inferred from an invalid sample.
            m_state = ShortState::Clear;
            m_haveConfig = false;
            m_pendingSinceMs = 0ULL;
            ShortResult result{};
            result.error = error;
            result.enabled = enabled;
            result.thresholdV = Finite(threshold) ? threshold : 0.0;
            return result;
        }

        bool m_haveSample = false, m_haveConfig = false;
        bool m_machining = false, m_enabled = false;
        std::uint32_t m_revision = 0U, m_enterMs = 0U, m_exitMs = 0U;
        double m_threshold = 0.0, m_hysteresis = 0.0;
        std::uint64_t m_lastSequence = 0ULL, m_lastNowMs = 0ULL, m_pendingSinceMs = 0ULL;
        ShortState m_state = ShortState::Clear;
    };
}
