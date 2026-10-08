#pragma once
// PBC-3J: RT-local physical X motor INDEX capture chain; OFF or sealed zero-only.
// This is capture provenance, never permission to clear unknown output or move.
#include "HomeTypes.h"
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

struct MotionPbcHomeCaptureKey
{
    std::uint64_t ownerGeneration = 0ULL;
    std::uint64_t epoch = 0ULL;
    std::uint64_t referenceGeneration = 0ULL;
    std::uint64_t incidentGeneration = 0ULL;
    std::uint64_t sourceIdentity = 0ULL;
    std::uint64_t inputIdentity = 0ULL;
    std::uint64_t outputIdentity = 0ULL;
    std::uint64_t frameSafetyGeneration = 0ULL;
    std::uint64_t alarmSafetyGeneration = 0ULL;
    std::uint64_t drainGeneration = 0ULL;
    std::uint32_t alarmUpdateCount = 0U;
    int axisIndex = 0;
    int slaveIndex = -1;
    double resolutionPPR = 0.0;
    double finalLead = 0.0;
    double machineOffsetPulse = 0.0;
    // PBC-3J: even zero-valued enable modes belong to the proof identity.
    bool enablePitch = false;
    bool enableBacklash = false;
    bool isReverse = false;
    bool axisReverse = false;
    HomeConfig home{};
};

struct MotionPbcHomeCaptureSample
{
    std::uint64_t tick = 0ULL;
    std::uint32_t rawPosition = 0U;
    std::uint32_t capturedRawPosition = 0U;
    double unwrappedPulse = 0.0;
    std::uint16_t probeStatus = 0U;
    bool valid = false;
    bool contiguous = false;
    // Adapter supplies current HOME lease, OFF/sealed zero identity and healthy
    // enabled RT input and no active output uncertainty. Search/decel allowed.
    bool eligible = false;
};

enum class MotionPbcHomeCapturePhase : std::uint32_t
{
    Empty, DisarmPending, WaitClear, ClearReady, ArmPending,
    WaitArmed, Armed, Captured, Failed, Consumed
};

enum class MotionPbcHomeCaptureFailure : std::uint32_t
{
    None, Invalidated, InvalidKey, KeyChanged, InvalidSample, TickGap,
    TickReplay, InvalidRaw, RawDiscontinuity, SourceStatus, CommandReplay,
    CommandOrder, SendMismatch, SendRejected, StaleCapture, ArmedLost,
    CaptureRange, GenerationExhausted, Consumed
};

struct MotionPbcHomeCaptureSnapshot
{
    MotionPbcHomeCapturePhase phase = MotionPbcHomeCapturePhase::Empty;
    MotionPbcHomeCaptureFailure reason = MotionPbcHomeCaptureFailure::None;
    std::uint64_t ownerGeneration = 0ULL;
    std::uint64_t epoch = 0ULL;
    std::uint64_t token = 0ULL;
    std::uint64_t disarmSequence = 0ULL;
    std::uint64_t armSequence = 0ULL;
    std::uint64_t lastCommandSeq = 0ULL;
    std::uint64_t lastSampleTick = 0ULL;
    std::uint64_t lastSendTick = 0ULL;
    std::uint64_t captureTick = 0ULL;
    std::uint16_t lastStatus = 0U;
    std::uint32_t capturedRawPosition = 0U;
    double capturedReferencePulse = 0.0;
    bool clearReady = false;
    bool armed = false;
    bool captured = false;
    bool failed = false;
};

class MotionPbcHomeCaptureProof
{
public:
    // Single RT writer. No heap, lock, external I/O or unbounded retry.
    // Request is a locally applied command, NOT a successful send receipt.
    bool Request(const MotionPbcHomeCaptureKey& key, std::uint64_t tick,
        std::uint64_t commandSeq, std::uint16_t value) noexcept
    {
        if (!ValidKey(key)) return Fail(MotionPbcHomeCaptureFailure::InvalidKey);
        if (tick == 0ULL || tick < lastObservedTick_ ||
            (lastObservedTick_ != 0ULL && tick - lastObservedTick_ > 1ULL))
            return Fail(MotionPbcHomeCaptureFailure::TickGap);
        if (commandSeq == 0ULL || commandSeq <= highCommandSequence_)
            return Fail(MotionPbcHomeCaptureFailure::CommandReplay);
        highCommandSequence_ = commandSeq;
        if (Active() && !SameKey(key_, key))
            return Fail(MotionPbcHomeCaptureFailure::KeyChanged);
        if (pendingSequence_ != 0ULL)
            return Fail(MotionPbcHomeCaptureFailure::CommandOrder);

        if (value == key.home.driveProbeDisarmValue)
        {
            if (state_.phase == MotionPbcHomeCapturePhase::Captured)
            {
                // Cleanup output must not erase the immutable capture receipt.
                pendingCleanup_ = true;
                cleanupRequested_ = true;
            }
            else
            {
                if (Active()) return Fail(MotionPbcHomeCaptureFailure::CommandOrder);
                if (highToken_ == (std::numeric_limits<std::uint64_t>::max)())
                    return Fail(MotionPbcHomeCaptureFailure::GenerationExhausted);
                ++highToken_;
                key_ = key;
                state_ = MotionPbcHomeCaptureSnapshot{};
                state_.ownerGeneration = key.ownerGeneration;
                state_.epoch = key.epoch;
                state_.token = highToken_;
                state_.disarmSequence = commandSeq;
                state_.phase = MotionPbcHomeCapturePhase::DisarmPending;
                rawValid_ = false;
                pendingCleanup_ = false;
                cleanupRequested_ = false;
            }
        }
        else if (value == key.home.driveProbeArmValue &&
            state_.phase == MotionPbcHomeCapturePhase::ClearReady)
        {
            state_.armSequence = commandSeq;
            state_.phase = MotionPbcHomeCapturePhase::ArmPending;
        }
        else return Fail(MotionPbcHomeCaptureFailure::CommandOrder);
        pendingSequence_ = commandSeq;
        pendingValue_ = value;
        requestTick_ = tick;
        state_.lastCommandSeq = commandSeq;
        return true;
    }

    // Call exactly once for the tracked request, after canonical output bytes
    // and the actual NIC send result are known. Repeated unchanged PDO frames
    // are not additional receipts. Local APPLIED status is insufficient.
    bool CompleteSend(const MotionPbcHomeCaptureKey& key, std::uint64_t tick,
        std::uint64_t commandSeq, std::uint16_t value,
        bool serializedExact, bool nicSent) noexcept
    {
        if (!Active() || !ValidKey(key) || !SameKey(key_, key))
            return Fail(MotionPbcHomeCaptureFailure::KeyChanged);
        if (!serializedExact || !nicSent)
            return Fail(MotionPbcHomeCaptureFailure::SendRejected);
        if (pendingSequence_ == 0ULL || commandSeq != pendingSequence_ ||
            value != pendingValue_ || tick < requestTick_ ||
            tick != lastObservedTick_ || state_.lastSampleTick != tick)
            return Fail(MotionPbcHomeCaptureFailure::SendMismatch);
        if (pendingCleanup_)
        {
            if (state_.phase != MotionPbcHomeCapturePhase::Captured)
                return Fail(MotionPbcHomeCaptureFailure::CommandOrder);
        }
        else if (state_.phase == MotionPbcHomeCapturePhase::DisarmPending)
            state_.phase = MotionPbcHomeCapturePhase::WaitClear;
        else if (state_.phase == MotionPbcHomeCapturePhase::ArmPending)
            state_.phase = MotionPbcHomeCapturePhase::WaitArmed;
        else return Fail(MotionPbcHomeCaptureFailure::CommandOrder);
        state_.lastSendTick = tick;
        pendingSequence_ = 0ULL;
        pendingCleanup_ = false;
        return true;
    }

    // Observe only the selected current RT snapshot, never supervisory getters.
    // State transitions use samples strictly later than the successful send.
    bool Observe(const MotionPbcHomeCaptureKey& key,
        const MotionPbcHomeCaptureSample& sample) noexcept
    {
        const std::uint64_t previousTick = lastObservedTick_;
        if (sample.tick > lastObservedTick_) lastObservedTick_ = sample.tick;
        if (sample.tick == 0ULL || sample.tick <= previousTick)
            return Fail(MotionPbcHomeCaptureFailure::TickReplay);
        if (!sample.valid || !sample.eligible)
            return Fail(MotionPbcHomeCaptureFailure::InvalidSample);
        if (!sample.contiguous || (previousTick != 0ULL && sample.tick - previousTick != 1ULL))
            return Fail(MotionPbcHomeCaptureFailure::TickGap);
        if (!ValidKey(key)) return Fail(MotionPbcHomeCaptureFailure::InvalidKey);
        if (Active() && !SameKey(key_, key))
            return Fail(MotionPbcHomeCaptureFailure::KeyChanged);
        if (!ValidRaw(sample.rawPosition, sample.unwrappedPulse))
            return Fail(MotionPbcHomeCaptureFailure::InvalidRaw);
        if (rawValid_)
        {
            std::int64_t delta = 0;
            if (!RawDelta(sample.rawPosition, lastRaw_, delta) ||
                sample.unwrappedPulse - lastUnwrapped_ != static_cast<double>(delta))
                return Fail(MotionPbcHomeCaptureFailure::RawDiscontinuity);
        }
        lastRaw_ = sample.rawPosition;
        lastUnwrapped_ = sample.unwrappedPulse;
        rawValid_ = true;
        state_.lastSampleTick = sample.tick;
        state_.lastStatus = sample.probeStatus;
        if (!Active()) return false;
        const HomeConfig& h = key_.home;
        const bool sourceValid = (sample.probeStatus & h.driveProbeSourceMask) ==
            h.driveProbeExpectedSourceValue;
        const bool armed = (sample.probeStatus & h.driveProbeArmedMask) == h.driveProbeArmedMask;
        const bool captured = (sample.probeStatus & h.driveProbeCapturedMask) != 0U;
        // Source selection may be absent while disarmed. It must be proven
        // together with armed status, throughout search and at capture. An
        // authorized cleanup disarm may clear it without erasing the receipt.
        if (!sourceValid && (state_.phase == MotionPbcHomeCapturePhase::Armed ||
            (state_.phase == MotionPbcHomeCapturePhase::WaitArmed && armed) ||
            (state_.phase == MotionPbcHomeCapturePhase::Captured && !cleanupRequested_)))
            return Fail(MotionPbcHomeCaptureFailure::SourceStatus);
        switch (state_.phase)
        {
        case MotionPbcHomeCapturePhase::WaitClear:
            if (sample.tick > state_.lastSendTick && !captured &&
                (sample.probeStatus & h.driveProbeArmedMask) == 0U)
                state_.phase = MotionPbcHomeCapturePhase::ClearReady;
            break;
        case MotionPbcHomeCapturePhase::ClearReady:
        case MotionPbcHomeCapturePhase::ArmPending:
            if (captured) return Fail(MotionPbcHomeCaptureFailure::StaleCapture);
            break;
        case MotionPbcHomeCapturePhase::WaitArmed:
            if (sample.tick > state_.lastSendTick)
            {
                if (captured) return Fail(MotionPbcHomeCaptureFailure::StaleCapture);
                if (armed) state_.phase = MotionPbcHomeCapturePhase::Armed;
            }
            break;
        case MotionPbcHomeCapturePhase::Armed:
            if (captured)
            {
                std::int64_t delta = 0;
                if (!RawDelta(sample.capturedRawPosition, sample.rawPosition, delta))
                    return Fail(MotionPbcHomeCaptureFailure::CaptureRange);
                double pulse = sample.unwrappedPulse + static_cast<double>(delta);
                if (!ValidRaw(sample.capturedRawPosition, pulse))
                    return Fail(MotionPbcHomeCaptureFailure::CaptureRange);
                if (key_.isReverse) pulse = -pulse;
                if (key_.axisReverse) pulse = -pulse;
                state_.capturedRawPosition = sample.capturedRawPosition;
                state_.capturedReferencePulse = pulse;
                state_.captureTick = sample.tick;
                state_.phase = MotionPbcHomeCapturePhase::Captured;
            }
            else if (!armed) return Fail(MotionPbcHomeCaptureFailure::ArmedLost);
            break;
        default: break; // Captured remains immutable through decel and disarm.
        }
        return true;
    }

    MotionPbcHomeCaptureSnapshot Snapshot() const noexcept
    {
        MotionPbcHomeCaptureSnapshot result = state_;
        result.clearReady = state_.phase == MotionPbcHomeCapturePhase::ClearReady;
        result.armed = state_.phase == MotionPbcHomeCapturePhase::Armed;
        result.captured = state_.phase == MotionPbcHomeCapturePhase::Captured;
        result.failed = state_.phase == MotionPbcHomeCapturePhase::Failed;
        return result;
    }

    // Mailbox apply precedes this cycle's input update. The receipt needs the
    // previous adjacent healthy sample; the independent H stopped proof and
    // exact reservation must also pass. No capture conversion is repeated.
    bool IsCaptureReady(std::uint64_t currentTick,
        const MotionPbcHomeCaptureKey& key, std::uint64_t token,
        double capturedPulse, double homeOffset) const noexcept
    {
        return state_.phase == MotionPbcHomeCapturePhase::Captured &&
            token != 0ULL && token == state_.token && pendingSequence_ == 0ULL &&
            state_.lastSampleTick != (std::numeric_limits<std::uint64_t>::max)() &&
            state_.lastSampleTick + 1ULL == currentTick &&
            lastObservedTick_ == state_.lastSampleTick &&
            ValidKey(key) && SameKey(key_, key) &&
            SameDouble(capturedPulse, state_.capturedReferencePulse) &&
            SameDouble(homeOffset, key_.home.homeOffset_unit);
    }

    bool Consume(std::uint64_t token) noexcept
    {
        if (state_.phase != MotionPbcHomeCapturePhase::Captured ||
            token == 0ULL || token != state_.token) return false;
        state_.phase = MotionPbcHomeCapturePhase::Consumed;
        state_.reason = MotionPbcHomeCaptureFailure::Consumed;
        pendingSequence_ = 0ULL;
        pendingCleanup_ = false;
        rawValid_ = false;
        return true;
    }

    void Invalidate(MotionPbcHomeCaptureFailure reason =
        MotionPbcHomeCaptureFailure::Invalidated) noexcept { (void)Fail(reason); }

    // Current released capture-provenance profile: Delta A3 motor Z, probe 1.
    static bool IsStrictConfiguration(const HomeConfig& h) noexcept
    {
        return h.method == HomeMethod::INDEX_ONLY &&
            h.referenceSource == HomeReferenceSource::MOTOR_ENCODER_INDEX &&
            h.captureMode == HomeReferenceCaptureMode::DRIVE_HARDWARE_LATCH &&
            h.driveProbeArmMode == HomeDriveProbeArmMode::CONTROLLER_60B8 &&
            h.driveProbeRequireArmedStatus && h.driveProbeRequireNewCapture &&
            !h.driveProbeAllowPositionChangeDetection && h.driveProbeCaptureToggleMask == 0U &&
            h.driveProbeDisarmValue == 0x0000U && h.driveProbeArmValue == 0x0015U &&
            h.driveProbeArmedMask == 0x0001U && h.driveProbeCapturedMask == 0x0002U &&
            h.driveProbeSourceMask == 0x0040U && h.driveProbeExpectedSourceValue == 0x0040U;
    }

    static bool IsConfigurationValid(const MotionPbcHomeCaptureKey& key) noexcept
    { return ValidKey(key); }
    static bool SameIdentityKey(const MotionPbcHomeCaptureKey& a,
        const MotionPbcHomeCaptureKey& b) noexcept { return SameKey(a, b); }

private:
    MotionPbcHomeCaptureKey key_{};
    MotionPbcHomeCaptureSnapshot state_{};
    std::uint64_t highToken_ = 0ULL;
    std::uint64_t highCommandSequence_ = 0ULL;
    std::uint64_t lastObservedTick_ = 0ULL;
    std::uint64_t pendingSequence_ = 0ULL;
    std::uint64_t requestTick_ = 0ULL;
    std::uint16_t pendingValue_ = 0U;
    bool pendingCleanup_ = false;
    bool cleanupRequested_ = false;
    bool rawValid_ = false;
    std::uint32_t lastRaw_ = 0U;
    double lastUnwrapped_ = 0.0;

    bool Active() const noexcept
    {
        return state_.phase != MotionPbcHomeCapturePhase::Empty &&
            state_.phase != MotionPbcHomeCapturePhase::Failed &&
            state_.phase != MotionPbcHomeCapturePhase::Consumed;
    }
    bool Fail(MotionPbcHomeCaptureFailure reason) noexcept
    {
        state_.phase = MotionPbcHomeCapturePhase::Failed;
        state_.reason = reason;
        pendingSequence_ = 0ULL;
        pendingCleanup_ = false;
        rawValid_ = false;
        // Monotonic ticks, tokens and command sequences deliberately survive.
        return false;
    }
    static bool SameDouble(double a, double b) noexcept
    { return std::memcmp(&a, &b, sizeof(double)) == 0; }
    static bool ValidRaw(std::uint32_t raw, double unwrapped) noexcept
    {
        constexpr double MaximumExactPulse = 9007199254740991.0;
        return std::isfinite(unwrapped) && std::abs(unwrapped) <= MaximumExactPulse &&
            std::trunc(unwrapped) == unwrapped &&
            static_cast<std::uint32_t>(static_cast<std::int64_t>(unwrapped)) == raw;
    }
    static bool RawDelta(std::uint32_t value, std::uint32_t base,
        std::int64_t& delta) noexcept
    {
        const std::uint32_t difference = value - base;
        if (difference == 0x80000000U) return false;
        delta = difference <= 0x7fffffffU ? static_cast<std::int64_t>(difference) :
            -static_cast<std::int64_t>(0x100000000ULL - difference);
        return true;
    }
    static bool ValidKey(const MotionPbcHomeCaptureKey& key) noexcept
    {
        const HomeConfig& h = key.home;
        if (key.ownerGeneration == 0ULL || key.epoch == 0ULL || key.sourceIdentity == 0ULL ||
            key.axisIndex != 0 || key.slaveIndex < 0 ||
            !std::isfinite(key.resolutionPPR) || key.resolutionPPR <= 0.0 ||
            !std::isfinite(key.finalLead) || std::abs(key.finalLead) < 1.0e-12 ||
            !std::isfinite(key.machineOffsetPulse) ||
            !IsStrictConfiguration(h))
            return false;
        if (!std::isfinite(h.searchSpeed_PPS)) return false;
        if (!std::isfinite(h.searchAccTime)) return false;
        if (!std::isfinite(h.searchDecTime)) return false;
        if (!std::isfinite(h.searchMaxDistance_unit)) return false;
        if (!std::isfinite(h.searchTimeoutSec)) return false;
        if (!std::isfinite(h.switchStopDecTime)) return false;
        if (!std::isfinite(h.switchStopMaxDistance_unit)) return false;
        if (!std::isfinite(h.backoffDistance_unit)) return false;
        if (!std::isfinite(h.backoffExtraDistance_unit)) return false;
        if (!std::isfinite(h.backoffSpeed_PPS)) return false;
        if (!std::isfinite(h.backoffAccTime)) return false;
        if (!std::isfinite(h.backoffDecTime)) return false;
        if (!std::isfinite(h.backoffMaxDistance_unit)) return false;
        if (!std::isfinite(h.backoffTimeoutSec)) return false;
        if (!std::isfinite(h.driveProbeClearTimeoutSec)) return false;
        if (!std::isfinite(h.driveProbeArmTimeoutSec)) return false;
        if (!std::isfinite(h.indexSearchSpeed_PPS)) return false;
        if (!std::isfinite(h.indexSearchAccTime)) return false;
        if (!std::isfinite(h.indexStopDecTime)) return false;
        if (!std::isfinite(h.indexMaxDistance_unit)) return false;
        if (!std::isfinite(h.indexTimeoutSec)) return false;
        if (!std::isfinite(h.homeOffset_unit)) return false;
        if (!std::isfinite(h.moveToZeroSpeed_PPS)) return false;
        if (!std::isfinite(h.moveToZeroAccTime)) return false;
        if (!std::isfinite(h.moveToZeroDecTime)) return false;
        if (!std::isfinite(h.searchGain.Kp)) return false;
        if (!std::isfinite(h.searchGain.Ki)) return false;
        if (!std::isfinite(h.searchGain.Kd)) return false;
        if (!std::isfinite(h.searchGain.Kvff)) return false;
        if (!std::isfinite(h.indexGain.Kp)) return false;
        if (!std::isfinite(h.indexGain.Ki)) return false;
        if (!std::isfinite(h.indexGain.Kd)) return false;
        if (!std::isfinite(h.indexGain.Kvff)) return false;
        return true;
    }
    static bool SameHome(const HomeConfig& a, const HomeConfig& b) noexcept
    {
        return
            a.enabled == b.enabled &&
            a.method == b.method &&
            a.referenceSource == b.referenceSource &&
            a.captureMode == b.captureMode &&
            a.direction == b.direction &&
            a.order == b.order &&
            a.dogActiveHigh == b.dogActiveHigh &&
            a.referenceActiveHigh == b.referenceActiveHigh &&
            a.externalReferenceCPoint == b.externalReferenceCPoint &&
            SameDouble(a.searchSpeed_PPS, b.searchSpeed_PPS) &&
            SameDouble(a.searchAccTime, b.searchAccTime) &&
            SameDouble(a.searchDecTime, b.searchDecTime) &&
            SameDouble(a.searchMaxDistance_unit, b.searchMaxDistance_unit) &&
            SameDouble(a.searchTimeoutSec, b.searchTimeoutSec) &&
            SameDouble(a.switchStopDecTime, b.switchStopDecTime) &&
            SameDouble(a.switchStopMaxDistance_unit, b.switchStopMaxDistance_unit) &&
            a.backoffMode == b.backoffMode &&
            SameDouble(a.backoffDistance_unit, b.backoffDistance_unit) &&
            SameDouble(a.backoffExtraDistance_unit, b.backoffExtraDistance_unit) &&
            SameDouble(a.backoffSpeed_PPS, b.backoffSpeed_PPS) &&
            SameDouble(a.backoffAccTime, b.backoffAccTime) &&
            SameDouble(a.backoffDecTime, b.backoffDecTime) &&
            SameDouble(a.backoffMaxDistance_unit, b.backoffMaxDistance_unit) &&
            SameDouble(a.backoffTimeoutSec, b.backoffTimeoutSec) &&
            a.alarmIfDogNotReleasedBeforeIndex == b.alarmIfDogNotReleasedBeforeIndex &&
            a.alarmIfHardLimitNotReleasedBeforeIndex == b.alarmIfHardLimitNotReleasedBeforeIndex &&
            a.driveProbeArmMode == b.driveProbeArmMode &&
            a.driveProbeDisarmValue == b.driveProbeDisarmValue &&
            a.driveProbeArmValue == b.driveProbeArmValue &&
            a.driveProbeArmedMask == b.driveProbeArmedMask &&
            a.driveProbeCapturedMask == b.driveProbeCapturedMask &&
            a.driveProbeCaptureToggleMask == b.driveProbeCaptureToggleMask &&
            a.driveProbeSourceMask == b.driveProbeSourceMask &&
            a.driveProbeExpectedSourceValue == b.driveProbeExpectedSourceValue &&
            SameDouble(a.driveProbeClearTimeoutSec, b.driveProbeClearTimeoutSec) &&
            SameDouble(a.driveProbeArmTimeoutSec, b.driveProbeArmTimeoutSec) &&
            a.driveProbeRequireArmedStatus == b.driveProbeRequireArmedStatus &&
            a.driveProbeDisarmAfterCapture == b.driveProbeDisarmAfterCapture &&
            a.driveProbeRequireNewCapture == b.driveProbeRequireNewCapture &&
            a.driveProbeAllowPositionChangeDetection == b.driveProbeAllowPositionChangeDetection &&
            SameDouble(a.indexSearchSpeed_PPS, b.indexSearchSpeed_PPS) &&
            SameDouble(a.indexSearchAccTime, b.indexSearchAccTime) &&
            SameDouble(a.indexStopDecTime, b.indexStopDecTime) &&
            SameDouble(a.indexMaxDistance_unit, b.indexMaxDistance_unit) &&
            SameDouble(a.indexTimeoutSec, b.indexTimeoutSec) &&
            SameDouble(a.homeOffset_unit, b.homeOffset_unit) &&
            a.moveToZero == b.moveToZero &&
            SameDouble(a.moveToZeroSpeed_PPS, b.moveToZeroSpeed_PPS) &&
            SameDouble(a.moveToZeroAccTime, b.moveToZeroAccTime) &&
            SameDouble(a.moveToZeroDecTime, b.moveToZeroDecTime) &&
            SameDouble(a.searchGain.Kp, b.searchGain.Kp) &&
            SameDouble(a.searchGain.Ki, b.searchGain.Ki) &&
            SameDouble(a.searchGain.Kd, b.searchGain.Kd) &&
            SameDouble(a.searchGain.Kvff, b.searchGain.Kvff) &&
            SameDouble(a.indexGain.Kp, b.indexGain.Kp) &&
            SameDouble(a.indexGain.Ki, b.indexGain.Ki) &&
            SameDouble(a.indexGain.Kd, b.indexGain.Kd) &&
            SameDouble(a.indexGain.Kvff, b.indexGain.Kvff);
    }
    static bool SameKey(const MotionPbcHomeCaptureKey& a,
        const MotionPbcHomeCaptureKey& b) noexcept
    {
        return a.ownerGeneration == b.ownerGeneration && a.epoch == b.epoch &&
            a.referenceGeneration == b.referenceGeneration &&
            a.incidentGeneration == b.incidentGeneration && a.sourceIdentity == b.sourceIdentity &&
            a.inputIdentity == b.inputIdentity && a.outputIdentity == b.outputIdentity &&
            a.frameSafetyGeneration == b.frameSafetyGeneration &&
            a.alarmSafetyGeneration == b.alarmSafetyGeneration &&
            a.drainGeneration == b.drainGeneration && a.alarmUpdateCount == b.alarmUpdateCount &&
            a.axisIndex == b.axisIndex && a.slaveIndex == b.slaveIndex &&
            SameDouble(a.resolutionPPR, b.resolutionPPR) && SameDouble(a.finalLead, b.finalLead) &&
            SameDouble(a.machineOffsetPulse, b.machineOffsetPulse) &&
            a.enablePitch == b.enablePitch && a.enableBacklash == b.enableBacklash &&
            a.isReverse == b.isReverse && a.axisReverse == b.axisReverse && SameHome(a.home, b.home);
    }
};

// Boot-only bounded local checks: never writes an axis, reference or PDO.
inline bool CheckMotionPbcHomeCaptureProof(unsigned& checks) noexcept
{
    checks = 0U;
    bool pass = true;
    const auto check = [&checks, &pass](bool result) noexcept
    { ++checks; pass = pass && result; };
    MotionPbcHomeCaptureKey key{};
    key.ownerGeneration = key.epoch = key.sourceIdentity = 1ULL;
    key.slaveIndex = 0; key.resolutionPPR = 1000.0; key.finalLead = 1.0;
    key.home.method = HomeMethod::INDEX_ONLY;
    key.home.driveProbeSourceMask = key.home.driveProbeExpectedSourceValue = 0x0040U;
    MotionPbcHomeCaptureSample sample{};
    sample.valid = sample.contiguous = sample.eligible = true;
    sample.rawPosition = 100U; sample.unwrappedPulse = 100.0;
    sample.probeStatus = 0x0040U;
    MotionPbcHomeCaptureProof proof;
    check(proof.Request(key, 1ULL, 1ULL, key.home.driveProbeDisarmValue));
    sample.tick = 1ULL;
    check(proof.Observe(key, sample) && !proof.Snapshot().clearReady);
    check(proof.CompleteSend(key, 1ULL, 1ULL, key.home.driveProbeDisarmValue, true, true));
    sample.tick = 2ULL;
    check(proof.Observe(key, sample) && proof.Snapshot().clearReady);
    check(proof.Request(key, 3ULL, 2ULL, key.home.driveProbeArmValue));
    sample.tick = 3ULL;
    check(proof.Observe(key, sample));
    check(proof.CompleteSend(key, 3ULL, 2ULL, key.home.driveProbeArmValue, true, true));
    sample.tick = 4ULL; sample.probeStatus = 0x0040U | key.home.driveProbeArmedMask;
    check(proof.Observe(key, sample) && proof.Snapshot().armed);
    sample.tick = 5ULL; sample.probeStatus |= key.home.driveProbeCapturedMask;
    sample.capturedRawPosition = 99U;
    check(proof.Observe(key, sample) && proof.Snapshot().captured);
    const auto receipt = proof.Snapshot();
    check(receipt.capturedReferencePulse == 99.0 && receipt.captureTick == 5ULL);
    check(proof.IsCaptureReady(6ULL, key, receipt.token, 99.0, 0.0));
    check(!proof.IsCaptureReady(7ULL, key, receipt.token, 99.0, 0.0));
    check(!proof.IsCaptureReady(6ULL, key, receipt.token + 1ULL, 99.0, 0.0));
    check(proof.Consume(receipt.token) && !proof.Snapshot().captured);
    check(!proof.Consume(receipt.token));
    check(!proof.Observe(key, sample) && proof.Snapshot().failed);
    check(proof.Request(key, 6ULL, 3ULL, key.home.driveProbeDisarmValue));
    check(proof.Snapshot().token > receipt.token);
    return pass;
}
