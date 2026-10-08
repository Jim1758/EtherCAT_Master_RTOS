#pragma once

#include "EDMGapSignal.h"
#include "EDMAnalogInput.h"
#include <cstdint>
#include <limits>

// EDM18: selected PDO ADC or explicit simulation. No IO, allocation, Motion or
// discharge writes. All clocks are supplied in the RT QPC millisecond domain.
namespace EDMGapInput
{
    constexpr std::int64_t GainScale = 1000000LL;
    enum class InputStatus : std::uint8_t
    {
        NotConfigured, ConfigurationError, BindingMissing, UnsupportedLayout,
        IdentityMismatch, AwaitSample, TransportInvalid, ClockInvalid,
        SnapshotBusy, ReadError, RawOutOfRange, CalibrationRequired, Stale, Valid
    };
    inline const char* StatusName(InputStatus value) noexcept
    {
        switch (value)
        {
        case InputStatus::NotConfigured: return "NOT_CONFIGURED";
        case InputStatus::ConfigurationError: return "CONFIGURATION_ERROR";
        case InputStatus::BindingMissing: return "BINDING_MISSING";
        case InputStatus::UnsupportedLayout: return "UNSUPPORTED_LAYOUT";
        case InputStatus::IdentityMismatch: return "IDENTITY_MISMATCH";
        case InputStatus::AwaitSample: return "AWAIT_SAMPLE";
        case InputStatus::TransportInvalid: return "TRANSPORT_INVALID";
        case InputStatus::ClockInvalid: return "CLOCK_INVALID";
        case InputStatus::SnapshotBusy: return "SNAPSHOT_BUSY";
        case InputStatus::ReadError: return "READ_ERROR";
        case InputStatus::RawOutOfRange: return "RAW_OUT_OF_RANGE";
        case InputStatus::CalibrationRequired: return "CALIBRATION_REQUIRED";
        case InputStatus::Stale: return "STALE";
        case InputStatus::Valid: return "VALID";
        }
        return "UNKNOWN";
    }
    struct Profile
    {
        std::uint32_t schemaVersion = 1U, profileRevision = 0U;
        EDMGap::Source source = EDMGap::Source::NONE;
        std::uint32_t adIndex = 0U, expectedDeviceId = 0U, calibrationRevision = 0U;
        bool calibrationConfirmed = false;
        std::int32_t rawMin = 0, rawMax = 0, boardMvAtMin = 0, boardMvAtMax = 0;
        // Schema 2 separates transport bounds from the two board calibration
        // points. Schema 1 retains its original raw-bound interpolation.
        std::int32_t calibrationRawMin = 0, calibrationRawMax = 0;
        std::uint32_t zeroClampBoardUv = 0U;
        std::int64_t voltageGainMillionths = 0LL;
        std::int32_t voltageOffsetMv = 0, simulationMv = 50000;
        EDMGap::Config gap{};
    };
    inline std::int64_t ScaleBoardMv(const Profile& p, std::int64_t boardMv) noexcept
    { return boardMv * p.voltageGainMillionths / GainScale + p.voltageOffsetMv; }
    namespace Detail
    {
        inline std::int64_t RoundSigned(std::int64_t numerator, std::int64_t denominator) noexcept
        {
            const std::int64_t whole = numerator / denominator;
            const std::int64_t remainder = numerator % denominator;
            if (remainder > 0 && remainder * 2 >= denominator) return whole + 1;
            if (remainder < 0 && -remainder * 2 >= denominator) return whole - 1;
            return whole;
        }
        inline std::int64_t BoardNumerator(const Profile& p, std::int32_t raw,
            std::int64_t& denominator) noexcept
        {
            denominator = static_cast<std::int64_t>(p.calibrationRawMax) - p.calibrationRawMin;
            return static_cast<std::int64_t>(p.boardMvAtMin) * denominator +
                (static_cast<std::int64_t>(raw) - p.calibrationRawMin) *
                (static_cast<std::int64_t>(p.boardMvAtMax) - p.boardMvAtMin);
        }
        // Call only after checking denominator > 0 and the board rational is
        // within +/-1,000,000 mV. Split before multiplying to prevent overflow.
        inline std::int64_t GapFromRational(const Profile& p, std::int64_t numerator,
            std::int64_t denominator) noexcept
        {
            const std::int64_t whole = numerator / denominator;
            const std::int64_t remainder = numerator % denominator;
            const std::int64_t remainderScaled = remainder * p.voltageGainMillionths;
            const std::int64_t scaled = whole * p.voltageGainMillionths +
                remainderScaled / denominator + static_cast<std::int64_t>(p.voltageOffsetMv) * GainScale;
            std::int64_t gapWhole = scaled / GainScale;
            std::int64_t gapRemainder = (scaled % GainScale) * denominator + remainderScaled % denominator;
            const std::int64_t gapDenominator = GainScale * denominator;
            // Offset can cross zero: round the complete signed affine value,
            // retaining the fractional tail through the nearest-mV decision.
            if (gapWhole > 0 && gapRemainder < 0) { --gapWhole; gapRemainder += gapDenominator; }
            else if (gapWhole < 0 && gapRemainder > 0) { ++gapWhole; gapRemainder -= gapDenominator; }
            return gapWhole + RoundSigned(gapRemainder, gapDenominator);
        }
        inline bool BoardInEnvelope(std::int64_t numerator, std::int64_t denominator) noexcept
        {
            return denominator > 0 && numerator >= -1000000LL * denominator &&
                numerator <= 1000000LL * denominator;
        }
        inline bool GapInRange(const Profile& p, std::int64_t numerator,
            std::int64_t denominator, std::int32_t minimumMv, std::int32_t maximumMv) noexcept
        {
            const std::int64_t whole = numerator / denominator;
            const std::int64_t remainderScaled = (numerator % denominator) * p.voltageGainMillionths;
            const std::int64_t wholeScaled = whole * p.voltageGainMillionths +
                remainderScaled / denominator + static_cast<std::int64_t>(p.voltageOffsetMv) * GainScale;
            const std::int64_t tail = remainderScaled % denominator;
            const std::int64_t minimum = static_cast<std::int64_t>(minimumMv) * GainScale;
            const std::int64_t maximum = static_cast<std::int64_t>(maximumMv) * GainScale;
            // Reject even sub-millivolt overshoot without constructing the
            // overflowing full numerator * gain product.
            return wholeScaled >= minimum && wholeScaled <= maximum &&
                !(wholeScaled == minimum && tail < 0) && !(wholeScaled == maximum && tail > 0);
        }
        inline std::int32_t GapAtRawBound(const Profile& p, std::int32_t raw) noexcept
        {
            std::int64_t denominator = 0;
            const std::int64_t numerator = BoardNumerator(p, raw, denominator);
            // Invalid/uninitialized profiles may still be observed for diagnostics.
            if (!BoardInEnvelope(numerator, denominator)) return 0;
            return static_cast<std::int32_t>(GapFromRational(p, numerator, denominator));
        }
    }
    inline bool Validate(const Profile& p) noexcept
    {
        if ((p.schemaVersion != 1U && p.schemaVersion != 2U) || p.profileRevision == 0U || p.calibrationRevision == 0U ||
            (p.source != EDMGap::Source::SIMULATED && p.source != EDMGap::Source::PHYSICAL) ||
            p.adIndex == 0U || p.adIndex > 64U || p.expectedDeviceId > 65535U ||
            p.rawMin < -32768 || p.rawMax > 65535 || p.rawMin >= p.rawMax ||
            p.boardMvAtMin < 0 || p.boardMvAtMax > 1000000 || p.boardMvAtMin >= p.boardMvAtMax ||
            p.voltageGainMillionths < 1LL || p.voltageGainMillionths > 1000LL * GainScale ||
            p.voltageOffsetMv < -1000000 || p.voltageOffsetMv > 1000000 ||
            p.gap.minMv < 0 || p.gap.maxMv > 1000000 || p.gap.minMv >= p.gap.maxMv ||
            p.gap.maxAgeMs == 0ULL || p.gap.maxAgeMs > 1000ULL ||
            p.gap.dwellMs == 0ULL || p.gap.dwellMs > 1000ULL ||
            p.simulationMv < p.gap.minMv || p.simulationMv > p.gap.maxMv) return false;
        std::int64_t a = 0, b = 0;
        if (p.schemaVersion == 1U)
        {
            a = ScaleBoardMv(p, p.boardMvAtMin); b = ScaleBoardMv(p, p.boardMvAtMax);
        }
        else
        {
            if (p.calibrationRawMin < p.rawMin || p.calibrationRawMax > p.rawMax ||
                p.calibrationRawMin >= p.calibrationRawMax || p.zeroClampBoardUv > 10000U ||
                (p.zeroClampBoardUv != 0U && p.boardMvAtMin != 0)) return false;
            std::int64_t denominator = 0;
            const std::int64_t low = Detail::BoardNumerator(p, p.rawMin, denominator);
            const std::int64_t high = Detail::BoardNumerator(p, p.rawMax, denominator);
            if (!Detail::BoardInEnvelope(low, denominator) ||
                !Detail::BoardInEnvelope(high, denominator)) return false;
            if (!Detail::GapInRange(p, low, denominator, -1000000, 1000000) ||
                !Detail::GapInRange(p, high, denominator, -1000000, 1000000)) return false;
            a = Detail::GapFromRational(p, low, denominator);
            b = Detail::GapFromRational(p, high, denominator);
        }
        if (a < -1000000LL || b > 1000000LL || a >= b) return false;
        EDMGap::Monitor check{};
        return check.Configure(p.gap, p.source);
    }
    struct Snapshot
    {
        EDMGap::Source configuredSource = EDMGap::Source::NONE;
        EDMGap::Snapshot gap{};
        InputStatus inputStatus = InputStatus::NotConfigured;
        bool configValid = false, liveVoltageValid = false, rawAvailable = false;
        bool lastGoodAvailable = false, ageKnown = false, calibrationConfirmed = false, boardVoltageValid = false;
        bool ownerClockValid = false;
        bool zeroClamped = false; // Local diagnostic only; no shared-memory ABI change.
        std::uint32_t adIndex = 0U, deviceId = 0U, channelId = 0U;
        std::uint32_t profileRevision = 0U, calibrationRevision = 0U, pdoOffset = 0U;
        std::int32_t rawCode = 0, voltageMv = 0, boardVoltageMv = 0;
        std::int32_t lastGoodRawCode = 0, lastGoodVoltageMv = 0;
        std::int32_t rawMin = 0, rawMax = 0, mvAtMin = 0, mvAtMax = 0;
        std::uint64_t sampleSequence = 0ULL, sampledAtMs = 0ULL, observedAtMs = 0ULL, ageMs = 0ULL;
        std::uint64_t lastGoodSequence = 0ULL, lastGoodAtMs = 0ULL;
        std::uint32_t maxAgeMs = 0U, dwellMs = 0U;
        constexpr bool PhysicalDischargeEnabled() const noexcept { return false; }
    };
    class Channel
    {
    public:
        bool Initialize(const Profile& profile) noexcept
        {
            if (m_attempted) { m_ready = false; return false; }
            m_attempted = true;
            m_ready = Validate(profile);
            if (m_ready) { m_profile = profile; m_monitor.Configure(profile.gap, profile.source); }
            return m_ready;
        }
        const Profile& GetProfile() const noexcept { return m_profile; }
        Snapshot Current() const noexcept { return m_current; }
        Snapshot Observe(const EDMAnalogInput::Snapshot& input, std::uint64_t nowMs, bool clockValid = true) noexcept
        {
            Snapshot result{};
            result.configValid = m_ready;
            result.configuredSource = m_profile.source;
            result.adIndex = m_profile.adIndex;
            result.profileRevision = m_profile.profileRevision;
            result.calibrationRevision = m_profile.calibrationRevision;
            result.calibrationConfirmed = m_profile.calibrationConfirmed;
            result.rawMin = m_profile.rawMin; result.rawMax = m_profile.rawMax;
            if (m_ready && m_profile.schemaVersion == 2U)
            {
                // These are extrapolated range endpoints, not calibration points.
                result.mvAtMin = Detail::GapAtRawBound(m_profile, m_profile.rawMin);
                result.mvAtMax = Detail::GapAtRawBound(m_profile, m_profile.rawMax);
            }
            else
            {
                result.mvAtMin = static_cast<std::int32_t>(ScaleBoardMv(m_profile, m_profile.boardMvAtMin));
                result.mvAtMax = static_cast<std::int32_t>(ScaleBoardMv(m_profile, m_profile.boardMvAtMax));
            }
            result.maxAgeMs = static_cast<std::uint32_t>(m_profile.gap.maxAgeMs);
            result.dwellMs = static_cast<std::uint32_t>(m_profile.gap.dwellMs);
            result.observedAtMs = nowMs;
            result.deviceId = input.deviceId; result.channelId = input.channelId;
            result.pdoOffset = input.inputBitOffset < 0 ? 0U : static_cast<std::uint32_t>(input.inputBitOffset / 8);
            result.rawAvailable = input.hasSample;
            result.rawCode = input.rawCode;
            result.sampleSequence = input.sequence; result.sampledAtMs = input.capturedAtMs;
            if (!clockValid || (m_haveClock && nowMs < m_lastNowMs)) m_clockFault = true;
            m_haveClock = true; m_lastNowMs = nowMs;
            result.ownerClockValid = !m_clockFault;
            result.inputStatus = InputStatus::AwaitSample;
            bool convertedGapInRange = true;
            const bool adcTime = input.hasSample && input.clockValid && input.capturedAtMs <= nowMs;
            if (adcTime)
            {
                result.ageKnown = true; result.ageMs = nowMs - input.capturedAtMs;
            }
            if (!m_ready) { result.inputStatus = InputStatus::ConfigurationError; return FinishInvalid(result, EDMGap::Quality::CONFIG_ERROR); }
            if (!input.configured) result.inputStatus = MapReason(input.reason);
            else if (input.adIndex != m_profile.adIndex ||
                (m_profile.expectedDeviceId != 0U && input.deviceId != m_profile.expectedDeviceId))
                result.inputStatus = InputStatus::IdentityMismatch;
            else if (!input.transportValid) {
                result.inputStatus = MapReason(input.reason);
                if (result.inputStatus == InputStatus::Valid) result.inputStatus = InputStatus::TransportInvalid;
            }
            else if (input.reason != EDMAnalogInput::Reason::Valid) result.inputStatus = MapReason(input.reason);
            else if (!input.hasSample) result.inputStatus = InputStatus::AwaitSample;
            else if (!adcTime) result.inputStatus = InputStatus::ClockInvalid;
            else if (result.ageMs > m_profile.gap.maxAgeMs) result.inputStatus = InputStatus::Stale;
            else if (input.rawCode < m_profile.rawMin || input.rawCode > m_profile.rawMax)
                result.inputStatus = InputStatus::RawOutOfRange;
            else
            {
                if (m_profile.schemaVersion == 1U)
                {
                    const std::int64_t numerator = (static_cast<std::int64_t>(input.rawCode) - m_profile.rawMin) *
                        (m_profile.boardMvAtMax - m_profile.boardMvAtMin);
                    const std::int64_t denominator = static_cast<std::int64_t>(m_profile.rawMax) - m_profile.rawMin;
                    result.boardVoltageMv = m_profile.boardMvAtMin + static_cast<std::int32_t>(numerator / denominator);
                    // Preserve schema 1 truncation exactly, including fractional gain.
                    const std::int64_t boardWhole = m_profile.boardMvAtMin + numerator / denominator;
                    const std::int64_t boardRemainder = numerator % denominator;
                    const std::int64_t scaled = boardWhole * m_profile.voltageGainMillionths +
                        boardRemainder * m_profile.voltageGainMillionths / denominator;
                    result.voltageMv = static_cast<std::int32_t>(scaled / GainScale + m_profile.voltageOffsetMv);
                }
                else
                {
                    std::int64_t denominator = 0;
                    std::int64_t numerator = Detail::BoardNumerator(m_profile, input.rawCode, denominator);
                    // Compare the unrounded rational in microvolts. Only the
                    // configured narrow negative-zero window may be clamped.
                    if (numerator < 0 && m_profile.zeroClampBoardUv != 0U &&
                        -numerator * 1000LL <= static_cast<std::int64_t>(m_profile.zeroClampBoardUv) * denominator)
                    {
                        numerator = 0; result.zeroClamped = true;
                    }
                    result.boardVoltageMv = static_cast<std::int32_t>(Detail::RoundSigned(numerator, denominator));
                    result.voltageMv = static_cast<std::int32_t>(Detail::GapFromRational(m_profile, numerator, denominator));
                    // Rounding must not silently convert an out-of-range value
                    // (including an unclamped negative fraction) into valid zero.
                    convertedGapInRange = Detail::GapInRange(m_profile, numerator, denominator,
                        m_profile.gap.minMv, m_profile.gap.maxMv);
                }
                result.boardVoltageValid = true;
                result.inputStatus = m_profile.calibrationConfirmed ? InputStatus::Valid : InputStatus::CalibrationRequired;
            }
            if (m_clockFault) { result.inputStatus = InputStatus::ClockInvalid; return FinishInvalid(result, EDMGap::Quality::CLOCK_ERROR); }
            if (m_profile.source == EDMGap::Source::SIMULATED)
            {
                if (m_simSequence == (std::numeric_limits<std::uint64_t>::max)())
                    return FinishInvalid(result, EDMGap::Quality::SEQUENCE_ERROR);
                EDMGap::Sample sample{};
                sample.source = EDMGap::Source::SIMULATED; sample.valid = true;
                sample.voltageMv = m_profile.simulationMv; sample.sequence = ++m_simSequence; sample.sampledAtMs = nowMs;
                result.gap = m_monitor.Publish(sample, nowMs);
                result.voltageMv = result.gap.voltageMv;
                // ADC diagnostics are independent; simulation freshness is its own clock.
                result.ageKnown = true; result.ageMs = 0ULL;
            }
            else
            {
                if (result.inputStatus != InputStatus::Valid)
                {
                    // A bad transport/config/range must not recover from replaying its cached sample.
                    if (input.sequence > m_rawHighWater) m_rawHighWater = input.sequence;
                    const auto q = result.inputStatus == InputStatus::Stale ? EDMGap::Quality::STALE :
                        (result.inputStatus == InputStatus::ClockInvalid ? EDMGap::Quality::CLOCK_ERROR :
                        (result.inputStatus == InputStatus::RawOutOfRange ? EDMGap::Quality::OUT_OF_RANGE : EDMGap::Quality::INVALID));
                    return FinishInvalid(result, q);
                }
                if (input.sequence < m_rawHighWater || input.sequence == 0ULL)
                    return FinishInvalid(result, EDMGap::Quality::SEQUENCE_ERROR);
                if (m_haveAdPayload && input.capturedAtMs < m_lastAdCaptured)
                {
                    if (input.sequence > m_rawHighWater) m_rawHighWater = input.sequence;
                    result.inputStatus = InputStatus::ClockInvalid;
                    return FinishInvalid(result, EDMGap::Quality::CLOCK_ERROR);
                }
                if (m_haveAdPayload && input.sequence == m_lastAdSequence &&
                    (input.rawCode != m_lastAdRaw || input.capturedAtMs != m_lastAdCaptured))
                    return FinishInvalid(result, EDMGap::Quality::SEQUENCE_ERROR);
                if (!convertedGapInRange)
                {
                    // Hardware payload is valid even though calibrated voltage
                    // is outside range. Keep its sequence/time floor so a later
                    // replay or timestamp rollback cannot restore validity.
                    if (input.sequence > m_rawHighWater)
                    {
                        m_rawHighWater = input.sequence;
                        m_haveAdPayload = true; m_lastAdRaw = input.rawCode;
                        m_lastAdCaptured = input.capturedAtMs; m_lastAdSequence = input.sequence;
                    }
                    return FinishInvalid(result, EDMGap::Quality::OUT_OF_RANGE);
                }
                if (input.sequence > m_rawHighWater)
                {
                    EDMGap::Sample sample{};
                    sample.source = EDMGap::Source::PHYSICAL; sample.valid = true;
                    sample.voltageMv = result.voltageMv; sample.sequence = input.sequence; sample.sampledAtMs = input.capturedAtMs;
                    result.gap = m_monitor.Publish(sample, nowMs);
                    m_rawHighWater = input.sequence;
                    m_haveAdPayload = true; m_lastAdRaw = input.rawCode;
                    m_lastAdCaptured = input.capturedAtMs; m_lastAdSequence = input.sequence;
                }
                else result.gap = m_monitor.Poll(nowMs);
            }
            result.liveVoltageValid = result.gap.quality == EDMGap::Quality::VALID;
            result.sampleSequence = result.gap.sequence; result.sampledAtMs = result.gap.sampledAtMs;
            if (result.liveVoltageValid)
            {
                m_lastGoodAvailable = true; m_lastGoodRaw = result.rawCode; m_lastGoodMv = result.voltageMv;
                m_lastGoodSequence = result.sampleSequence; m_lastGoodAtMs = result.sampledAtMs;
            }
            return Commit(result);
        }
    private:
        static InputStatus MapReason(EDMAnalogInput::Reason reason) noexcept
        {
            switch (reason)
            {
            case EDMAnalogInput::Reason::NotConfigured: return InputStatus::NotConfigured;
            case EDMAnalogInput::Reason::BindingMissing: return InputStatus::BindingMissing;
            case EDMAnalogInput::Reason::UnsupportedLayout: return InputStatus::UnsupportedLayout;
            case EDMAnalogInput::Reason::ContractInvalid: return InputStatus::UnsupportedLayout;
            case EDMAnalogInput::Reason::IdentityMismatch: return InputStatus::IdentityMismatch;
            case EDMAnalogInput::Reason::AwaitSample: return InputStatus::AwaitSample;
            case EDMAnalogInput::Reason::TransportInvalid: return InputStatus::TransportInvalid;
            case EDMAnalogInput::Reason::ClockInvalid: return InputStatus::ClockInvalid;
            case EDMAnalogInput::Reason::SnapshotBusy: return InputStatus::SnapshotBusy;
            case EDMAnalogInput::Reason::ReadError: return InputStatus::ReadError;
            case EDMAnalogInput::Reason::Valid: return InputStatus::Valid;
            }
            return InputStatus::ReadError;
        }
        Snapshot FinishInvalid(Snapshot& result, EDMGap::Quality quality) noexcept
        {
            m_monitor.Reset();
            result.gap.source = m_profile.source; result.gap.quality = quality;
            result.gap.observedAtMs = result.observedAtMs;
            result.boardVoltageValid = result.boardVoltageValid && quality != EDMGap::Quality::CLOCK_ERROR;
            return Commit(result);
        }
        Snapshot Commit(Snapshot& result) noexcept
        {
            result.lastGoodAvailable = m_lastGoodAvailable;
            result.lastGoodRawCode = m_lastGoodRaw; result.lastGoodVoltageMv = m_lastGoodMv;
            result.lastGoodSequence = m_lastGoodSequence; result.lastGoodAtMs = m_lastGoodAtMs;
            m_current = result; return result;
        }
        Profile m_profile{};
        EDMGap::Monitor m_monitor{};
        Snapshot m_current{};
        std::uint64_t m_rawHighWater = 0ULL, m_simSequence = 0ULL, m_lastNowMs = 0ULL;
        std::uint64_t m_lastGoodSequence = 0ULL, m_lastGoodAtMs = 0ULL;
        std::uint64_t m_lastAdCaptured = 0ULL, m_lastAdSequence = 0ULL;
        std::int32_t m_lastAdRaw = 0;
        std::int32_t m_lastGoodRaw = 0, m_lastGoodMv = 0;
        bool m_attempted = false, m_ready = false, m_haveClock = false, m_clockFault = false, m_lastGoodAvailable = false;
        bool m_haveAdPayload = false;
    };
}
