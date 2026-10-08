#pragma once

#include "EDMAutomaticFlushCycle.h"
#include "EDMGapInput.h"
#include <limits>

// EDM24: real owner-supplied GAP observations gate an entirely virtual cycle.
// This class owns no AD, clock, axis, PID, discharge permission, I/O or allocation.
namespace EDM24
{
    enum class LiveAutomaticFlushState : std::uint8_t
    { Idle, Running, AwaitingFresh, Blocked, Cancelled, Fault };
    enum class LiveAutomaticFlushReason : std::uint8_t
    {
        None, AwaitFreshSample, DuplicateSample, ShadowBlocked, InterlockBlocked,
        InvalidGap, StaleGap, InvalidInitialGap, InvalidConfig, InvalidLifecycle,
        ConfigRevisionChanged, RecipeGenerationChanged, SourceChanged,
        CalibrationChanged, SampleSequenceRegressed, SampleTimeRegressed,
        ContradictorySample, ClockRollback, ServiceGap, CycleFault,
        CounterOverflow, ShortActive, MachiningBlocked
    };

    struct LiveSourceIdentity
    {
        EDMGap::Source source = EDMGap::Source::NONE;
        std::uint32_t profileRevision = 0U, calibrationRevision = 0U;
        std::uint32_t adIndex = 0U, deviceId = 0U, channelId = 0U, pdoOffset = 0U;
        std::uint32_t maxAgeMs = 0U;
        bool calibrationConfirmed = false;
    };

    struct LiveObservation
    {
        std::uint64_t nowMs = 0U, configRevision = 0U, recipeGeneration = 0U;
        EDMGapInput::Snapshot gap{};
        bool shadowReady = false, machiningAllowed = false, shortActive = false;
        bool returnAllowed = false, interlockReady = false;
    };

    struct LiveAutomaticFlushSnapshot
    {
        LiveAutomaticFlushState state = LiveAutomaticFlushState::Idle;
        LiveAutomaticFlushReason reason = LiveAutomaticFlushReason::None;
        EDM23::AutomaticFlushCycleSnapshot cycle{};
        LiveSourceIdentity sourceIdentity{};
        std::uint64_t freshSamples = 0U, duplicateSamples = 0U, blockedSamples = 0U;
        std::uint64_t lastSampleSeq = 0U, lastSampleTimeMs = 0U, lastObservedMs = 0U;
        double requestedSpeedMmPerMin = 0.0;
        bool acceptedFreshSample = false; // This Observe accepted freshness AND its shadow gate.
        constexpr bool PhysicalMotionEnabled() const noexcept { return false; }
        constexpr bool PhysicalDischargeEnabled() const noexcept { return false; }
    };

    class LiveAutomaticFlushSession
    {
    public:
        const LiveAutomaticFlushSnapshot& Snapshot() const noexcept { return snapshot_; }

        void Reset() noexcept
        {
            cycle_.Reset();
            snapshot_ = LiveAutomaticFlushSnapshot{};
            previousNowMs_ = 0U;
            havePayload_ = false;
            voltageKnown_ = false;
            lastRawCode_ = lastVoltageMv_ = 0;
        }

        void Revoke() noexcept
        {
            cycle_.Revoke();
            CopyCycle();
            snapshot_.state = LiveAutomaticFlushState::Cancelled;
            snapshot_.reason = LiveAutomaticFlushReason::None;
            snapshot_.acceptedFreshSample = false;
            snapshot_.requestedSpeedMmPerMin = 0.0;
        }

        bool Start(const EDM23::AutomaticFlushCycleConfig& config,
            const EDMGapInput::Snapshot& initialGap, std::uint64_t nowMs) noexcept
        {
            if (snapshot_.state != LiveAutomaticFlushState::Idle)
            { Fail(LiveAutomaticFlushReason::InvalidLifecycle); return false; }
            const bool physical = initialGap.configuredSource == EDMGap::Source::PHYSICAL;
            if (!initialGap.configValid || !initialGap.ownerClockValid ||
                (!physical && initialGap.configuredSource != EDMGap::Source::SIMULATED) ||
                initialGap.profileRevision == 0U || initialGap.maxAgeMs == 0U ||
                initialGap.maxAgeMs > 1000U || initialGap.observedAtMs > nowMs ||
                (physical && (!initialGap.calibrationConfirmed ||
                    initialGap.calibrationRevision == 0U || initialGap.adIndex == 0U)))
            { Fail(LiveAutomaticFlushReason::InvalidInitialGap); return false; }

            snapshot_.sourceIdentity.source = initialGap.configuredSource;
            snapshot_.sourceIdentity.profileRevision = initialGap.profileRevision;
            snapshot_.sourceIdentity.maxAgeMs = initialGap.maxAgeMs;
            if (physical)
            {
                snapshot_.sourceIdentity.calibrationRevision = initialGap.calibrationRevision;
                snapshot_.sourceIdentity.calibrationConfirmed = initialGap.calibrationConfirmed;
                snapshot_.sourceIdentity.adIndex = initialGap.adIndex;
                snapshot_.sourceIdentity.deviceId = initialGap.deviceId;
                snapshot_.sourceIdentity.channelId = initialGap.channelId;
                snapshot_.sourceIdentity.pdoOffset = initialGap.pdoOffset;
            }
            if (initialGap.liveVoltageValid && !ValidGap(initialGap, nowMs))
            { Fail(LiveAutomaticFlushReason::InvalidInitialGap); return false; }
            if (!cycle_.Start(config, nowMs))
            { Fail(LiveAutomaticFlushReason::InvalidConfig); return false; }
            previousNowMs_ = nowMs;
            snapshot_.lastObservedMs = nowMs;
            snapshot_.state = LiveAutomaticFlushState::AwaitingFresh;
            snapshot_.reason = LiveAutomaticFlushReason::AwaitFreshSample;
            // Freeze the initial high-water mark. A cached initial sample cannot
            // give work/flush credit. Invalid available physical payloads also
            // raise this mark, so validity toggles cannot replay them as fresh.
            const std::uint64_t sequence = SampleSequence(initialGap);
            if (sequence != 0U)
                RememberPayload(initialGap, sequence, initialGap.liveVoltageValid);
            CopyCycle();
            return true;
        }

        const LiveAutomaticFlushSnapshot& Observe(const LiveObservation& observation) noexcept
        {
            snapshot_.acceptedFreshSample = false;
            if (!Active()) return snapshot_;
            if (observation.nowMs < previousNowMs_)
                return Fail(LiveAutomaticFlushReason::ClockRollback);
            const std::uint64_t elapsedMs = observation.nowMs - previousNowMs_;
            // Check active service continuity BEFORE the current gate can turn
            // it into a HOLD. The long-pause exemption applies only after a
            // previously observed duplicate/invalid/shadow pause established it.
            if (snapshot_.state == LiveAutomaticFlushState::Running &&
                elapsedMs > EDM22::MaximumFlushStepMs)
                return Fail(LiveAutomaticFlushReason::ServiceGap);
            previousNowMs_ = observation.nowMs;
            snapshot_.lastObservedMs = observation.nowMs;
            if (observation.configRevision != snapshot_.cycle.configRevision)
                return Fail(LiveAutomaticFlushReason::ConfigRevisionChanged);
            if (observation.recipeGeneration != snapshot_.cycle.recipeGeneration)
                return Fail(LiveAutomaticFlushReason::RecipeGenerationChanged);
            if (!observation.interlockReady)
                return Fail(LiveAutomaticFlushReason::InterlockBlocked);
            const auto& gap = observation.gap;
            const auto& identity = snapshot_.sourceIdentity;
            if (gap.configuredSource != identity.source ||
                gap.profileRevision != identity.profileRevision || gap.maxAgeMs != identity.maxAgeMs ||
                (gap.gap.source != EDMGap::Source::NONE && gap.gap.source != identity.source))
                return Fail(LiveAutomaticFlushReason::SourceChanged);
            if (identity.source == EDMGap::Source::PHYSICAL)
            {
                if (gap.calibrationRevision != identity.calibrationRevision ||
                    gap.calibrationConfirmed != identity.calibrationConfirmed)
                    return Fail(LiveAutomaticFlushReason::CalibrationChanged);
                if (gap.adIndex != identity.adIndex || gap.deviceId != identity.deviceId ||
                    gap.channelId != identity.channelId || gap.pdoOffset != identity.pdoOffset)
                    return Fail(LiveAutomaticFlushReason::SourceChanged);
            }

            const bool valid = ValidGap(gap, observation.nowMs);
            const std::uint64_t sequence = SampleSequence(gap);
            if (sequence != 0U && havePayload_)
            {
                if (sequence < snapshot_.lastSampleSeq)
                    return Fail(LiveAutomaticFlushReason::SampleSequenceRegressed);
                if (gap.sampledAtMs < snapshot_.lastSampleTimeMs)
                    return Fail(LiveAutomaticFlushReason::SampleTimeRegressed);
                if (sequence == snapshot_.lastSampleSeq &&
                    (gap.sampledAtMs != snapshot_.lastSampleTimeMs ||
                    (identity.source == EDMGap::Source::PHYSICAL && gap.rawAvailable && gap.rawCode != lastRawCode_) ||
                    (valid && voltageKnown_ && gap.voltageMv != lastVoltageMv_)))
                    return Fail(LiveAutomaticFlushReason::ContradictorySample);
            }
            // A claimed-valid sample whose mirrored identity disagrees is a
            // contradictory payload, rather than a recoverable transport loss.
            if (gap.liveVoltageValid &&
                (gap.sampleSequence != gap.gap.sequence || gap.sampledAtMs != gap.gap.sampledAtMs ||
                 gap.voltageMv != gap.gap.voltageMv))
                return Fail(LiveAutomaticFlushReason::ContradictorySample);

            if (!valid)
            {
                if (sequence > snapshot_.lastSampleSeq)
                    RememberPayload(gap, sequence, false);
                const bool stale = gap.inputStatus == EDMGapInput::InputStatus::Stale ||
                    gap.gap.quality == EDMGap::Quality::STALE ||
                    (sequence != 0U && gap.sampledAtMs <= observation.nowMs &&
                        observation.nowMs - gap.sampledAtMs > identity.maxAgeMs);
                return Hold(observation, LiveAutomaticFlushState::Blocked,
                    stale ? LiveAutomaticFlushReason::StaleGap : LiveAutomaticFlushReason::InvalidGap, false);
            }
            if (sequence <= snapshot_.lastSampleSeq ||
                (havePayload_ && gap.sampledAtMs == snapshot_.lastSampleTimeMs))
            {
                // A renumbered cached acquisition is still a duplicate. Consume
                // its sequence floor so it cannot recover by replay, but require
                // BOTH a newer sequence and a newer acquisition timestamp.
                if (sequence > snapshot_.lastSampleSeq)
                    RememberPayload(gap, sequence, true);
                return Hold(observation, LiveAutomaticFlushState::AwaitingFresh,
                    LiveAutomaticFlushReason::DuplicateSample, true);
            }

            RememberPayload(gap, sequence, true);
            if (!Increment(snapshot_.freshSamples)) return snapshot_;
            if (!observation.shadowReady)
                return Hold(observation, LiveAutomaticFlushState::Blocked,
                    LiveAutomaticFlushReason::ShadowBlocked, false);

            auto input = CycleInput(observation);
            if (cycle_.Snapshot().state == EDM23::AutomaticFlushCycleState::Hold)
            {
                input.command = EDM23::AutomaticFlushCycleCommand::Resume;
                cycle_.Step(input); // Release is explicit and consumes no paused time.
                // A machining release may establish its allowed endpoint at
                // this SAME timestamp. Flush release stays at speed/distance 0.
                if (cycle_.Snapshot().state == EDM23::AutomaticFlushCycleState::Machining)
                {
                    input.command = EDM23::AutomaticFlushCycleCommand::Tick;
                    cycle_.Step(input);
                }
            }
            else if (snapshot_.state != LiveAutomaticFlushState::Running)
            {
                // Initial first-new sample cannot consume time since Start.
                input.machiningAllowed = false;
                cycle_.Step(input);
                if (cycle_.Snapshot().state == EDM23::AutomaticFlushCycleState::Machining)
                {
                    input.machiningAllowed = observation.machiningAllowed;
                    cycle_.Step(input); // Same-time endpoint establishes gate only.
                }
            }
            else cycle_.Step(input);
            CopyCycle();
            if (snapshot_.cycle.state == EDM23::AutomaticFlushCycleState::Fault)
                return Fail(LiveAutomaticFlushReason::CycleFault);
            snapshot_.state = LiveAutomaticFlushState::Running;
            snapshot_.reason = observation.shortActive ? LiveAutomaticFlushReason::ShortActive :
                (!observation.machiningAllowed ? LiveAutomaticFlushReason::MachiningBlocked : LiveAutomaticFlushReason::None);
            snapshot_.acceptedFreshSample = true;
            return snapshot_;
        }

    private:
        EDM23::AutomaticFlushCycle cycle_{};
        LiveAutomaticFlushSnapshot snapshot_{};
        std::uint64_t previousNowMs_ = 0U;
        std::int32_t lastRawCode_ = 0, lastVoltageMv_ = 0;
        bool havePayload_ = false, voltageKnown_ = false;

        bool Active() const noexcept
        {
            return snapshot_.state == LiveAutomaticFlushState::Running ||
                snapshot_.state == LiveAutomaticFlushState::AwaitingFresh ||
                snapshot_.state == LiveAutomaticFlushState::Blocked;
        }

        std::uint64_t SampleSequence(const EDMGapInput::Snapshot& gap) const noexcept
        {
            // SIM sequences belong to the simulation publisher; ADC payloads
            // visible in its diagnostics must not move the simulation floor.
            if (snapshot_.sourceIdentity.source == EDMGap::Source::SIMULATED)
                return gap.gap.source == EDMGap::Source::SIMULATED ? gap.gap.sequence : 0U;
            return gap.rawAvailable ? gap.sampleSequence : 0U;
        }

        bool ValidGap(const EDMGapInput::Snapshot& gap, std::uint64_t nowMs) const noexcept
        {
            const auto source = snapshot_.sourceIdentity.source;
            if (!gap.configValid || !gap.ownerClockValid || !gap.liveVoltageValid ||
                !gap.gap.configured || gap.gap.quality != EDMGap::Quality::VALID ||
                gap.gap.source != source || !gap.ageKnown ||
                gap.sampleSequence == 0U || gap.sampleSequence != gap.gap.sequence ||
                gap.sampledAtMs != gap.gap.sampledAtMs || gap.voltageMv != gap.gap.voltageMv ||
                gap.observedAtMs > nowMs || gap.gap.observedAtMs > nowMs ||
                gap.sampledAtMs > gap.observedAtMs || gap.sampledAtMs > gap.gap.observedAtMs ||
                gap.sampledAtMs > nowMs || nowMs - gap.sampledAtMs > snapshot_.sourceIdentity.maxAgeMs ||
                nowMs - gap.observedAtMs > snapshot_.sourceIdentity.maxAgeMs ||
                gap.ageMs > snapshot_.sourceIdentity.maxAgeMs) return false;
            return source == EDMGap::Source::SIMULATED ||
                (gap.rawAvailable && gap.boardVoltageValid && gap.calibrationConfirmed &&
                    gap.inputStatus == EDMGapInput::InputStatus::Valid);
        }

        void RememberPayload(const EDMGapInput::Snapshot& gap,
            std::uint64_t sequence, bool voltageKnown) noexcept
        {
            snapshot_.lastSampleSeq = sequence;
            snapshot_.lastSampleTimeMs = gap.sampledAtMs;
            lastRawCode_ = gap.rawCode;
            lastVoltageMv_ = gap.voltageMv;
            havePayload_ = true;
            voltageKnown_ = voltageKnown;
        }

        EDM23::AutomaticFlushCycleInput CycleInput(const LiveObservation& observation) const noexcept
        {
            EDM23::AutomaticFlushCycleInput result{};
            result.nowMs = observation.nowMs;
            result.configRevision = observation.configRevision;
            result.recipeGeneration = observation.recipeGeneration;
            result.interlockReady = true; // Outer loss is terminal before this method.
            result.machiningAllowed = observation.machiningAllowed;
            result.shortActive = observation.shortActive;
            result.returnAllowed = observation.returnAllowed;
            return result;
        }

        void CopyCycle() noexcept
        {
            snapshot_.cycle = cycle_.Snapshot();
            snapshot_.requestedSpeedMmPerMin = snapshot_.cycle.requestedSpeedMmPerMin;
        }

        bool Increment(std::uint64_t& counter) noexcept
        {
            if (counter == (std::numeric_limits<std::uint64_t>::max)())
            { Fail(LiveAutomaticFlushReason::CounterOverflow); return false; }
            ++counter;
            return true;
        }

        const LiveAutomaticFlushSnapshot& Hold(const LiveObservation& observation,
            LiveAutomaticFlushState state, LiveAutomaticFlushReason reason, bool duplicate) noexcept
        {
            if (duplicate && !Increment(snapshot_.duplicateSamples)) return snapshot_;
            if (!Increment(snapshot_.blockedSamples)) return snapshot_;
            auto input = CycleInput(observation);
            input.command = EDM23::AutomaticFlushCycleCommand::Hold;
            cycle_.Step(input);
            CopyCycle();
            if (snapshot_.cycle.state == EDM23::AutomaticFlushCycleState::Fault)
                return Fail(LiveAutomaticFlushReason::CycleFault);
            snapshot_.state = state;
            snapshot_.reason = reason;
            snapshot_.requestedSpeedMmPerMin = 0.0;
            return snapshot_;
        }

        const LiveAutomaticFlushSnapshot& Fail(LiveAutomaticFlushReason reason) noexcept
        {
            cycle_.Revoke();
            CopyCycle();
            snapshot_.state = LiveAutomaticFlushState::Fault;
            snapshot_.reason = reason;
            snapshot_.requestedSpeedMmPerMin = 0.0;
            snapshot_.acceptedFreshSample = false;
            return snapshot_;
        }
    };

    inline const char* LiveAutomaticFlushStateName(LiveAutomaticFlushState value) noexcept
    {
        switch (value)
        {
        case LiveAutomaticFlushState::Idle: return "IDLE";
        case LiveAutomaticFlushState::Running: return "RUNNING";
        case LiveAutomaticFlushState::AwaitingFresh: return "AWAITING_FRESH";
        case LiveAutomaticFlushState::Blocked: return "BLOCKED";
        case LiveAutomaticFlushState::Cancelled: return "CANCELLED";
        case LiveAutomaticFlushState::Fault: return "FAULT";
        }
        return "UNKNOWN";
    }

    inline const char* LiveAutomaticFlushReasonName(LiveAutomaticFlushReason value) noexcept
    {
        switch (value)
        {
        case LiveAutomaticFlushReason::None: return "NONE";
        case LiveAutomaticFlushReason::AwaitFreshSample: return "AWAIT_FRESH_SAMPLE";
        case LiveAutomaticFlushReason::DuplicateSample: return "DUPLICATE_SAMPLE";
        case LiveAutomaticFlushReason::ShadowBlocked: return "SHADOW_BLOCKED";
        case LiveAutomaticFlushReason::InterlockBlocked: return "INTERLOCK_BLOCKED";
        case LiveAutomaticFlushReason::InvalidGap: return "INVALID_GAP";
        case LiveAutomaticFlushReason::StaleGap: return "STALE_GAP";
        case LiveAutomaticFlushReason::InvalidInitialGap: return "INVALID_INITIAL_GAP";
        case LiveAutomaticFlushReason::InvalidConfig: return "INVALID_CONFIG";
        case LiveAutomaticFlushReason::InvalidLifecycle: return "INVALID_LIFECYCLE";
        case LiveAutomaticFlushReason::ConfigRevisionChanged: return "CONFIG_REVISION_CHANGED";
        case LiveAutomaticFlushReason::RecipeGenerationChanged: return "RECIPE_GENERATION_CHANGED";
        case LiveAutomaticFlushReason::SourceChanged: return "SOURCE_CHANGED";
        case LiveAutomaticFlushReason::CalibrationChanged: return "CALIBRATION_CHANGED";
        case LiveAutomaticFlushReason::SampleSequenceRegressed: return "SAMPLE_SEQUENCE_REGRESSED";
        case LiveAutomaticFlushReason::SampleTimeRegressed: return "SAMPLE_TIME_REGRESSED";
        case LiveAutomaticFlushReason::ContradictorySample: return "CONTRADICTORY_SAMPLE";
        case LiveAutomaticFlushReason::ClockRollback: return "CLOCK_ROLLBACK";
        case LiveAutomaticFlushReason::ServiceGap: return "SERVICE_GAP";
        case LiveAutomaticFlushReason::CycleFault: return "CYCLE_FAULT";
        case LiveAutomaticFlushReason::CounterOverflow: return "COUNTER_OVERFLOW";
        case LiveAutomaticFlushReason::ShortActive: return "SHORT_ACTIVE";
        case LiveAutomaticFlushReason::MachiningBlocked: return "MACHINING_BLOCKED";
        }
        return "UNKNOWN";
    }
}
