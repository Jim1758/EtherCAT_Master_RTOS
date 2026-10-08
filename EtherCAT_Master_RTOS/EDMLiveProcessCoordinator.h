#pragma once

#include "EDMProcessCoordinator.h"
#include "EDMProcessProfile.h"
#include "EDMAutomaticFlushRecipe.h"
#include "EDMLiveAutomaticFlush.h"

// EDM26 owner-thread adapter. Only EDM25::ProcessCoordinator evolves virtual
// state. No clock reads, allocations, Motion/PID calls, I/O or physical permits.
namespace EDM26
{
    using LiveProcessState = EDM24::LiveAutomaticFlushState;
    using LiveProcessReason = EDM24::LiveAutomaticFlushReason;
    using LiveObservation = EDM24::LiveObservation;
    using LiveSourceIdentity = EDM24::LiveSourceIdentity;

    struct ProcessRecipeBinding
    {
        bool ready = false;
        EDM23::RecipeBindingError error = EDM23::RecipeBindingError::NotReady;
        std::uint16_t failedFieldId = 0U, eCode = 0U;
        std::uint32_t tableId = 0U;
        std::uint64_t generation = 0U;
        EDM25::ProcessConfig config{};
        constexpr bool PhysicalMotionEnabled() const noexcept { return false; }
        constexpr bool PhysicalDischargeEnabled() const noexcept { return false; }
    };

    inline ProcessRecipeBinding BindProcessRecipe(const EDMRecipe::Controller& controller,
        const EDM20::ProcessProfile& installedProfile, const EDMGapInput::Snapshot& gap,
        const EDM20::FlushRequest& explicitVirtualRequest) noexcept
    {
        ProcessRecipeBinding output{};
        auto binding = EDM23::BindRecipe(controller, installedProfile.machineProfileId);
        output.tableId = binding.tableId; output.eCode = binding.eCode;
        output.generation = binding.generation; output.error = binding.error;
        output.failedFieldId = binding.failedFieldId;
        if (!binding.ready) return output;
        if (!EDM20::ValidateProcessProfile(installedProfile))
        { output.error = EDM23::RecipeBindingError::InvalidIdentity; return output; }
        EDMRecipe::Value overrideValue = 0LL, referenceValue = 0LL;
        const auto* catalog = controller.GetCatalog();
        if (catalog == nullptr ||
            !EDM23::RecipeBindingDetail::Read(controller, *catalog, "MachiningServoOverride", "%", overrideValue, binding) ||
            !EDM23::RecipeBindingDetail::Read(controller, *catalog, "GapVoltageSetpoint", "V", referenceValue, binding))
        { output.error = binding.error; output.failedFieldId = binding.failedFieldId; return output; }
        if (overrideValue < 0LL || overrideValue > 100LL * EDMRecipe::Scale ||
            referenceValue < 0LL || referenceValue > 1000LL * EDMRecipe::Scale ||
            gap.maxAgeMs == 0U || gap.maxAgeMs > EDM22::MaximumFlushStepMs)
        { output.error = EDM23::RecipeBindingError::EngineeringRange; return output; }
        EDM25::ProcessConfig candidate{};
        // This helper admits frozen base/deep plans only. It does not Step a
        // second executor, and the live session owns exactly one evolving engine.
        if (!EDM23::MakeAutomaticFlushCycleConfig(binding, installedProfile.flush,
            installedProfile.revision, explicitVirtualRequest, candidate.flush))
        { output.error = EDM23::RecipeBindingError::EngineeringRange; return output; }
        candidate.servo = installedProfile.servo;
        candidate.machiningOverridePercent = static_cast<double>(overrideValue) / static_cast<double>(EDMRecipe::Scale);
        candidate.referenceV = static_cast<double>(referenceValue) / static_cast<double>(EDMRecipe::Scale);
        candidate.maximumSampleAgeMs = gap.maxAgeMs;
        candidate.sourceGeneration = 1U; // Session identity fences are checked below.
        output.config = candidate; output.ready = true;
        output.error = EDM23::RecipeBindingError::None; output.failedFieldId = 0U; return output;
    }

    struct LiveProcessSnapshot
    {
        LiveProcessState state = LiveProcessState::Idle;
        LiveProcessReason reason = LiveProcessReason::None;
        EDM25::ProcessSnapshot process{};
        EDM23::AutomaticFlushCycleSnapshot cycle{};
        LiveSourceIdentity sourceIdentity{};
        std::uint64_t freshSamples = 0U, duplicateSamples = 0U, blockedSamples = 0U, coalescedSamples = 0U;
        std::uint64_t lastSampleSeq = 0U, lastSampleTimeMs = 0U, lastObservedMs = 0U;
        double requestedSpeedMmPerMin = 0.0;
        bool acceptedFreshSample = false;
        constexpr bool PhysicalMotionEnabled() const noexcept { return false; }
        constexpr bool PhysicalDischargeEnabled() const noexcept { return false; }
    };

    class LiveProcessSession
    {
    public:
        const LiveProcessSnapshot& Snapshot() const noexcept { return snapshot_; }
        void Reset() noexcept
        {
            engine_.Reset(); snapshot_ = LiveProcessSnapshot{};
            coalescedServo_ = EDMGapServo::Profile{}; coalescedReferenceV_ = coalescedOverridePercent_ = 0.0;
            lastModelCurveSpeed_ = 0.0; haveModelCurve_ = false;
            previousNowMs_ = sourceGeneration_ = 0U; lastRawCode_ = lastVoltageMv_ = 0;
            havePayload_ = voltageKnown_ = false;
        }
        void Revoke() noexcept
        {
            engine_.Revoke(); CopyEngine(); snapshot_.state = LiveProcessState::Cancelled;
            snapshot_.reason = LiveProcessReason::None; Neutralize();
        }
        bool Start(const EDM25::ProcessConfig& config, const EDMGapInput::Snapshot& initialGap,
            std::uint64_t nowMs) noexcept
        {
            if (snapshot_.state != LiveProcessState::Idle)
            { Fail(LiveProcessReason::InvalidLifecycle); return false; }
            const bool physical = initialGap.configuredSource == EDMGap::Source::PHYSICAL;
            if (!initialGap.configValid || !initialGap.ownerClockValid ||
                (!physical && initialGap.configuredSource != EDMGap::Source::SIMULATED) ||
                initialGap.profileRevision == 0U || initialGap.maxAgeMs == 0U ||
                initialGap.maxAgeMs > EDM22::MaximumFlushStepMs || initialGap.observedAtMs > nowMs ||
                (physical && (!initialGap.calibrationConfirmed || initialGap.calibrationRevision == 0U || initialGap.adIndex == 0U)))
            { Fail(LiveProcessReason::InvalidInitialGap); return false; }
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
            { Fail(LiveProcessReason::InvalidInitialGap); return false; }
            if (config.maximumSampleAgeMs != initialGap.maxAgeMs || !engine_.Start(config, nowMs))
            { Fail(LiveProcessReason::InvalidConfig); return false; }
            sourceGeneration_ = config.sourceGeneration; previousNowMs_ = nowMs;
            coalescedServo_ = config.servo; coalescedReferenceV_ = config.referenceV;
            coalescedOverridePercent_ = config.machiningOverridePercent;
            snapshot_.lastObservedMs = nowMs;
            snapshot_.state = LiveProcessState::AwaitingFresh; snapshot_.reason = LiveProcessReason::AwaitFreshSample;
            const std::uint64_t sequence = SampleSequence(initialGap);
            if (sequence != 0U) RememberPayload(initialGap, sequence, initialGap.liveVoltageValid);
            CopyEngine(); Neutralize(); return true;
        }

        const LiveProcessSnapshot& Observe(const LiveObservation& observation) noexcept
        {
            snapshot_.acceptedFreshSample = false;
            if (!Active()) return snapshot_;
            if (observation.nowMs < previousNowMs_) return Fail(LiveProcessReason::ClockRollback);
            const std::uint64_t elapsedMs = observation.nowMs - previousNowMs_;
            if (snapshot_.state == LiveProcessState::Running && elapsedMs > EDM22::MaximumFlushStepMs)
                return Fail(LiveProcessReason::ServiceGap);
            previousNowMs_ = observation.nowMs; snapshot_.lastObservedMs = observation.nowMs;
            if (observation.configRevision != snapshot_.cycle.configRevision)
                return Fail(LiveProcessReason::ConfigRevisionChanged);
            if (observation.recipeGeneration != snapshot_.cycle.recipeGeneration)
                return Fail(LiveProcessReason::RecipeGenerationChanged);
            if (!observation.interlockReady) return Fail(LiveProcessReason::InterlockBlocked);
            const auto& gap = observation.gap; const auto& identity = snapshot_.sourceIdentity;
            if (gap.configuredSource != identity.source || gap.profileRevision != identity.profileRevision ||
                gap.maxAgeMs != identity.maxAgeMs ||
                (gap.gap.source != EDMGap::Source::NONE && gap.gap.source != identity.source))
                return Fail(LiveProcessReason::SourceChanged);
            if (identity.source == EDMGap::Source::PHYSICAL)
            {
                if (gap.calibrationRevision != identity.calibrationRevision || gap.calibrationConfirmed != identity.calibrationConfirmed)
                    return Fail(LiveProcessReason::CalibrationChanged);
                if (gap.adIndex != identity.adIndex || gap.deviceId != identity.deviceId ||
                    gap.channelId != identity.channelId || gap.pdoOffset != identity.pdoOffset)
                    return Fail(LiveProcessReason::SourceChanged);
            }
            const bool valid = ValidGap(gap, observation.nowMs);
            const std::uint64_t sequence = SampleSequence(gap);
            if (sequence != 0U && havePayload_)
            {
                if (sequence < snapshot_.lastSampleSeq) return Fail(LiveProcessReason::SampleSequenceRegressed);
                if (gap.sampledAtMs < snapshot_.lastSampleTimeMs) return Fail(LiveProcessReason::SampleTimeRegressed);
                if (sequence == snapshot_.lastSampleSeq &&
                    (gap.sampledAtMs != snapshot_.lastSampleTimeMs ||
                    (identity.source == EDMGap::Source::PHYSICAL && gap.rawAvailable && gap.rawCode != lastRawCode_) ||
                    (valid && voltageKnown_ && gap.voltageMv != lastVoltageMv_)))
                    return Fail(LiveProcessReason::ContradictorySample);
            }
            if (gap.liveVoltageValid &&
                (gap.sampleSequence != gap.gap.sequence || gap.sampledAtMs != gap.gap.sampledAtMs || gap.voltageMv != gap.gap.voltageMv))
                return Fail(LiveProcessReason::ContradictorySample);
            const bool newSourceTime = !havePayload_ || gap.sampledAtMs > snapshot_.lastSampleTimeMs;
            const bool newSequence = sequence > snapshot_.lastSampleSeq;
            if (!valid)
            {
                if (newSequence) RememberPayload(gap, sequence, false);
                const bool stale = gap.inputStatus == EDMGapInput::InputStatus::Stale || gap.gap.quality == EDMGap::Quality::STALE ||
                    (sequence != 0U && gap.sampledAtMs <= observation.nowMs && observation.nowMs - gap.sampledAtMs > identity.maxAgeMs);
                return Block(observation, LiveProcessState::Blocked, stale ? LiveProcessReason::StaleGap : LiveProcessReason::InvalidGap, false);
            }
            if (newSequence) RememberPayload(gap, sequence, true);
            if (!observation.shadowReady)
                return Block(observation, LiveProcessState::Blocked, LiveProcessReason::ShadowBlocked, false);
            if (!newSequence || !newSourceTime)
            {
                // A 250-us producer can be phase-shifted from the ms observer:
                // age 0/1 polls are a coarse-clock wait, not a service pause.
                // Only a genuinely newer source timestamp advances the model,
                // even when it arrives later within the same owner millisecond.
                if (observation.nowMs - gap.sampledAtMs > 1U)
                    return Block(observation, LiveProcessState::AwaitingFresh, LiveProcessReason::DuplicateSample, true);
                if (sequence != 0U)
                {
                    const double voltage = static_cast<double>(gap.voltageMv) / 1000.0;
                    auto safety = EngineInput(observation); safety.processAllowed = true;
                    safety.sample.valid = true; safety.sample.voltage = voltage;
                    safety.sample.nowMs = gap.sampledAtMs; safety.sample.sequence = sequence;
                    engine_.ObserveSafety(safety); CopyEngine();
                    if (snapshot_.process.state == EDM25::ProcessState::Fault)
                        return Fail(LiveProcessReason::CycleFault);
                    if (snapshot_.process.state == EDM25::ProcessState::AwaitingFresh)
                        return Block(observation, newSequence ? LiveProcessState::Blocked : LiveProcessState::AwaitingFresh,
                            !newSequence ? LiveProcessReason::DuplicateSample :
                            (snapshot_.process.shortState != EDMGapServo::ShortState::Clear ? LiveProcessReason::ShortActive : LiveProcessReason::MachiningBlocked), !newSequence);
                    const auto shortState = snapshot_.process.shortState;
                    const auto curve = EDMGapServo::Evaluate(coalescedServo_, voltage,
                        coalescedReferenceV_, coalescedOverridePercent_);
                    const auto cycleState = engine_.CycleSnapshot().state;
                    const bool changedSign = haveModelCurve_ &&
                        ((curve.limitedSpeedMmPerMin > 0.0) != (lastModelCurveSpeed_ > 0.0) ||
                         (curve.limitedSpeedMmPerMin < 0.0) != (lastModelCurveSpeed_ < 0.0));
                    if (!curve.valid || (cycleState == EDM23::AutomaticFlushCycleState::Machining &&
                        shortState == EDMGapServo::ShortState::Clear && changedSign))
                        return Block(observation, LiveProcessState::Blocked, LiveProcessReason::MachiningBlocked, false);
                }
                const auto& cycle = engine_.CycleSnapshot();
                // A gate closure inside the ms still invalidates that action's
                // endpoint. Ignore neither a signed-machining gate nor a
                // return gate while the virtual executor is on its return leg.
                if (!observation.machiningAllowed && cycle.state == EDM23::AutomaticFlushCycleState::Machining)
                    return Block(observation, LiveProcessState::Blocked, LiveProcessReason::MachiningBlocked, false);
                if (!observation.returnAllowed &&
                    (cycle.state == EDM23::AutomaticFlushCycleState::Flushing || cycle.state == EDM23::AutomaticFlushCycleState::ReturnBlocked) &&
                    cycle.flushExecution.leg == EDM20::FlushLeg::Return)
                    return Block(observation, LiveProcessState::Blocked, LiveProcessReason::ShadowBlocked, false);
                // EtherCAT can publish four different acquisitions per ms.
                // Consume their observation floors without replacing/pausing
                // EDM25's separately retained last accepted model endpoint.
                if (!Increment(snapshot_.coalescedSamples)) return snapshot_;
                CopyEngine(); Neutralize(); return snapshot_;
            }
            EDM25::ProcessInput input = EngineInput(observation);
            input.processAllowed = true;
            input.sample.valid = true; input.sample.voltage = static_cast<double>(gap.voltageMv) / 1000.0;
            input.sample.nowMs = gap.sampledAtMs; input.sample.sequence = sequence;
            engine_.Step(input); CopyEngine();
            if (snapshot_.process.state == EDM25::ProcessState::Fault)
                return Fail(snapshot_.process.reason == EDM25::ProcessReason::ServiceGap ? LiveProcessReason::ServiceGap : LiveProcessReason::CycleFault);
            // Recovery publishes speed zero, but it still established this
            // model endpoint's curve/gate. Never infer it from neutralized LOG
            // fields or from the newer coalesced observation high-water mark.
            lastModelCurveSpeed_ = EDMGapServo::Evaluate(coalescedServo_, input.sample.voltage,
                coalescedReferenceV_, coalescedOverridePercent_).limitedSpeedMmPerMin;
            haveModelCurve_ = true;
            if (!Increment(snapshot_.freshSamples)) return snapshot_;
            snapshot_.state = LiveProcessState::Running; snapshot_.acceptedFreshSample = true;
            snapshot_.reason = snapshot_.process.shortState != EDMGapServo::ShortState::Clear ? LiveProcessReason::ShortActive :
                (!observation.machiningAllowed ? LiveProcessReason::MachiningBlocked : LiveProcessReason::None);
            return snapshot_;
        }

    private:
        EDM25::ProcessCoordinator engine_{};
        LiveProcessSnapshot snapshot_{};
        EDMGapServo::Profile coalescedServo_{};
        double coalescedReferenceV_ = 0.0, coalescedOverridePercent_ = 0.0;
        double lastModelCurveSpeed_ = 0.0;
        bool haveModelCurve_ = false;
        std::uint64_t previousNowMs_ = 0U, sourceGeneration_ = 0U;
        std::int32_t lastRawCode_ = 0, lastVoltageMv_ = 0;
        bool havePayload_ = false, voltageKnown_ = false;
        bool Active() const noexcept
        { return snapshot_.state == LiveProcessState::Running || snapshot_.state == LiveProcessState::AwaitingFresh || snapshot_.state == LiveProcessState::Blocked; }
        std::uint64_t SampleSequence(const EDMGapInput::Snapshot& gap) const noexcept
        {
            if (snapshot_.sourceIdentity.source == EDMGap::Source::SIMULATED)
                return gap.gap.source == EDMGap::Source::SIMULATED ? gap.gap.sequence : 0U;
            return gap.rawAvailable ? gap.sampleSequence : 0U;
        }
        bool ValidGap(const EDMGapInput::Snapshot& gap, std::uint64_t nowMs) const noexcept
        {
            const auto source = snapshot_.sourceIdentity.source;
            if (!gap.configValid || !gap.ownerClockValid || !gap.liveVoltageValid || !gap.gap.configured ||
                gap.gap.quality != EDMGap::Quality::VALID || gap.gap.source != source || !gap.ageKnown ||
                gap.sampleSequence == 0U || gap.sampleSequence != gap.gap.sequence || gap.sampledAtMs != gap.gap.sampledAtMs ||
                gap.voltageMv != gap.gap.voltageMv || gap.observedAtMs > nowMs || gap.gap.observedAtMs > nowMs ||
                gap.sampledAtMs > gap.observedAtMs || gap.sampledAtMs > gap.gap.observedAtMs || gap.sampledAtMs > nowMs ||
                gap.ageMs != gap.observedAtMs - gap.sampledAtMs ||
                nowMs - gap.sampledAtMs > snapshot_.sourceIdentity.maxAgeMs || nowMs - gap.observedAtMs > snapshot_.sourceIdentity.maxAgeMs ||
                gap.ageMs > snapshot_.sourceIdentity.maxAgeMs) return false;
            return source == EDMGap::Source::SIMULATED ||
                (gap.rawAvailable && gap.boardVoltageValid && gap.calibrationConfirmed && gap.inputStatus == EDMGapInput::InputStatus::Valid);
        }
        void RememberPayload(const EDMGapInput::Snapshot& gap, std::uint64_t sequence, bool voltageKnown) noexcept
        {
            snapshot_.lastSampleSeq = sequence; snapshot_.lastSampleTimeMs = gap.sampledAtMs;
            lastRawCode_ = gap.rawCode; lastVoltageMv_ = gap.voltageMv; havePayload_ = true; voltageKnown_ = voltageKnown;
        }
        EDM25::ProcessInput EngineInput(const LiveObservation& observation) const noexcept
        {
            EDM25::ProcessInput input{}; input.nowMs = observation.nowMs;
            input.configRevision = observation.configRevision; input.recipeGeneration = observation.recipeGeneration;
            input.sourceGeneration = sourceGeneration_; input.interlockReady = true;
            input.machiningAllowed = observation.machiningAllowed; input.returnAllowed = observation.returnAllowed; return input;
        }
        void CopyEngine() noexcept
        {
            snapshot_.process = engine_.Snapshot(); snapshot_.cycle = engine_.CycleSnapshot();
            snapshot_.requestedSpeedMmPerMin = snapshot_.process.speedMmMin;
        }
        void Neutralize() noexcept
        {
            snapshot_.acceptedFreshSample = false; snapshot_.requestedSpeedMmPerMin = snapshot_.process.speedMmMin = 0.0;
            snapshot_.cycle.requestedSpeedMmPerMin = snapshot_.cycle.flushExecution.requestedSpeedMmPerMin = 0.0;
        }
        bool Increment(std::uint64_t& counter) noexcept
        {
            if (counter == (std::numeric_limits<std::uint64_t>::max)())
            { Fail(LiveProcessReason::CounterOverflow); return false; }
            ++counter; return true;
        }
        const LiveProcessSnapshot& Block(const LiveObservation& observation, LiveProcessState state,
            LiveProcessReason reason, bool duplicate) noexcept
        {
            if (duplicate && !Increment(snapshot_.duplicateSamples)) return snapshot_;
            if (!Increment(snapshot_.blockedSamples)) return snapshot_;
            auto input = EngineInput(observation); input.processAllowed = false; engine_.Step(input); CopyEngine();
            if (snapshot_.process.state == EDM25::ProcessState::Fault) return Fail(LiveProcessReason::CycleFault);
            snapshot_.state = state; snapshot_.reason = reason; Neutralize(); return snapshot_;
        }
        const LiveProcessSnapshot& Fail(LiveProcessReason reason) noexcept
        {
            engine_.Revoke(); CopyEngine(); snapshot_.state = LiveProcessState::Fault;
            snapshot_.reason = reason; Neutralize(); return snapshot_;
        }
    };
    inline const char* LiveProcessStateName(LiveProcessState state) noexcept { return EDM24::LiveAutomaticFlushStateName(state); }
    inline const char* LiveProcessReasonName(LiveProcessReason reason) noexcept { return EDM24::LiveAutomaticFlushReasonName(reason); }
}
