#pragma once

#include "EDMGapServo.h"
#include "EDMAutomaticFlushCycle.h"

// EDM25: selection and integration of one virtual EDM action. It has no axis,
// clock, PID, I/O, allocation or physical output. B0 speeds are signed in the
// machining frame; B1 flush speeds are signed in the machine-Z frame.
namespace EDM25
{
    enum class ProcessState : std::uint8_t { Idle, Running, AwaitingFresh, Hold, Cancelled, Fault };
    enum class ProcessAction : std::uint8_t
    { Wait, ServoAdvance, ServoRetreat, ShortRetreat, FlushRetreat, FlushReturn, ReturnBlocked };
    enum class ProcessReason : std::uint8_t
    {
        None, AwaitFresh, DuplicateSample, Held, ProcessBlocked, MachiningBlocked,
        ShortEntering, ShortActive, ReturnBlocked, ZeroOverride, Deadband,
        InvalidConfig, InvalidLifecycle, InvalidCommand, InterlockLost,
        InvalidSample, StaleSample, SourceChanged, RevisionChanged,
        GenerationChanged, SequenceRegressed, SourceClockRegressed,
        ContradictorySample, ClockRollback, ServiceGap, CurveFault,
        ShortFault, CycleFault, NumericalFailure
    };
    enum class ProcessCommand : std::uint8_t { Tick, Hold, Resume };

    struct ProcessConfig
    {
        EDMGapServo::Profile servo{};
        EDM23::AutomaticFlushCycleConfig flush{};
        double referenceV = 50.0;
        double machiningOverridePercent = 100.0;
        std::uint64_t sourceGeneration = 1U;
        std::uint32_t maximumSampleAgeMs = 50U;
    };

    struct ProcessInput
    {
        std::uint64_t nowMs = 0U;
        EDMGapServo::Sample sample{};
        std::uint64_t sourceGeneration = 0U, configRevision = 0U, recipeGeneration = 0U;
        bool interlockReady = false, processAllowed = false;
        bool machiningAllowed = false, returnAllowed = false;
        ProcessCommand command = ProcessCommand::Tick;
    };

    struct ProcessSnapshot
    {
        ProcessState state = ProcessState::Idle;
        ProcessReason reason = ProcessReason::None;
        ProcessAction action = ProcessAction::Wait;
        EDM20::FlushDirection direction = EDM20::FlushDirection::AlongMachining;
        EDMGapServo::ShortState shortState = EDMGapServo::ShortState::Invalid;
        std::uint64_t completedCycles = 0U, workElapsedMs = 0U;
        // These are separate frames/origins. Never add a B1 offset to machining.
        double machiningOffsetMm = 0.0, flushOffsetMm = 0.0, speedMmMin = 0.0;
        bool deepCycle = false;
        constexpr bool PhysicalMotionEnabled() const noexcept { return false; }
        constexpr bool PhysicalDischargeEnabled() const noexcept { return false; }
    };

    class ProcessCoordinator
    {
    public:
        const ProcessSnapshot& Snapshot() const noexcept { return snapshot_; }
        const EDM23::AutomaticFlushCycleSnapshot& CycleSnapshot() const noexcept { return cycle_.Snapshot(); }
        void Reset() noexcept
        {
            cycle_.Reset(); detector_.Reset(); config_ = ProcessConfig{};
            snapshot_ = ProcessSnapshot{}; previousNowMs_ = lastSourceMs_ = lastSequence_ = 0U;
            lastVoltage_ = previousSpeed_ = 0.0; previousAction_ = ProcessAction::Wait;
            haveSample_ = pausedCycle_ = recoveryInhibit_ = recoveryExitPending_ = false;
            recoveryExitSinceMs_ = 0U;
        }
        void Revoke() noexcept
        {
            cycle_.Revoke(); CopyCycle(); snapshot_.state = ProcessState::Cancelled;
            snapshot_.reason = ProcessReason::None; Stop();
        }
        bool Start(const ProcessConfig& config, std::uint64_t nowMs = 0U) noexcept
        {
            if (snapshot_.state != ProcessState::Idle)
            { Fail(ProcessReason::InvalidLifecycle); return false; }
            if (!EDMGapServo::ValidateProfile(config.servo) ||
                !std::isfinite(config.referenceV) ||
                !EDM20::FlushPlanDetail::Percent(config.machiningOverridePercent) ||
                config.sourceGeneration == 0U || config.maximumSampleAgeMs == 0U ||
                config.maximumSampleAgeMs > EDM22::MaximumFlushStepMs ||
                !cycle_.Start(config.flush, 0U))
            { Fail(ProcessReason::InvalidConfig); return false; }
            config_ = config; previousNowMs_ = nowMs;
            snapshot_.state = ProcessState::AwaitingFresh;
            snapshot_.reason = ProcessReason::AwaitFresh; CopyCycle(); return true;
        }

        // EDM26: inspect every acquired short-circuit observation, including
        // multiple new sequences in one source millisecond. This explicit API
        // does not integrate work, servo/flush distance or cycle completions.
        // Step retains its stricter same-timestamp payload contract.
        const ProcessSnapshot& ObserveSafety(const ProcessInput& input) noexcept
        {
            if (snapshot_.state == ProcessState::Idle || snapshot_.state == ProcessState::Cancelled ||
                snapshot_.state == ProcessState::Fault) return snapshot_;
            if (input.command != ProcessCommand::Tick) return Fail(ProcessReason::InvalidCommand);
            if (!input.interlockReady) return Fail(ProcessReason::InterlockLost);
            if (input.configRevision != config_.flush.configRevision) return Fail(ProcessReason::RevisionChanged);
            if (input.recipeGeneration != config_.flush.recipeGeneration) return Fail(ProcessReason::GenerationChanged);
            if (input.sourceGeneration != config_.sourceGeneration) return Fail(ProcessReason::SourceChanged);
            if (input.nowMs < previousNowMs_) return Fail(ProcessReason::ClockRollback);
            const bool ownerAdvanced = input.nowMs > previousNowMs_;
            if (snapshot_.state == ProcessState::Running && input.nowMs - previousNowMs_ > EDM22::MaximumFlushStepMs)
                return Fail(ProcessReason::ServiceGap);
            previousNowMs_ = input.nowMs;
            if (!input.processAllowed) return Pause(input, ProcessState::AwaitingFresh, ProcessReason::ProcessBlocked);
            const auto& sample = input.sample;
            if (!sample.valid || !std::isfinite(sample.voltage) || sample.sequence == 0U || sample.nowMs > input.nowMs)
                return Fail(ProcessReason::InvalidSample);
            if (input.nowMs - sample.nowMs > config_.maximumSampleAgeMs) return Fail(ProcessReason::StaleSample);
            if (haveSample_ && sample.sequence < lastSequence_) return Fail(ProcessReason::SequenceRegressed);
            if (haveSample_ && sample.nowMs < lastSourceMs_) return Fail(ProcessReason::SourceClockRegressed);
            if (haveSample_ && sample.sequence == lastSequence_)
            {
                if (sample.nowMs != lastSourceMs_ || sample.voltage != lastVoltage_)
                    return Fail(ProcessReason::ContradictorySample);
                if (ownerAdvanced && input.nowMs - sample.nowMs > 1U)
                    return Pause(input, ProcessState::AwaitingFresh, ProcessReason::DuplicateSample);
                snapshot_.speedMmMin = 0.0; return snapshot_;
            }
            const bool sourceAdvanced = !haveSample_ || sample.nowMs > lastSourceMs_;
            if (snapshot_.state == ProcessState::Hold)
            {
                // HOLD can consume source high-water marks for validation,
                // but paused time is never entry/exit debounce evidence.
                lastSourceMs_ = sample.nowMs; lastSequence_ = sample.sequence;
                lastVoltage_ = sample.voltage; haveSample_ = true;
                snapshot_.speedMmMin = 0.0; return snapshot_;
            }
            const auto previousShort = snapshot_.shortState;
            auto shortResult = detector_.Step(config_.servo, sample, true);
            if (!shortResult.valid) return Fail(ProcessReason::ShortFault);
            PreserveShortLatch(sample, shortResult);
            snapshot_.shortState = shortResult.state;
            lastSourceMs_ = sample.nowMs; lastSequence_ = sample.sequence;
            lastVoltage_ = sample.voltage; haveSample_ = true;
            const auto& cycle = cycle_.Snapshot();
            const bool gateClosed = (!input.machiningAllowed && cycle.state == EDM23::AutomaticFlushCycleState::Machining) ||
                (!input.returnAllowed &&
                    (cycle.state == EDM23::AutomaticFlushCycleState::Flushing || cycle.state == EDM23::AutomaticFlushCycleState::ReturnBlocked) &&
                    cycle.flushExecution.leg == EDM20::FlushLeg::Return);
            if ((ownerAdvanced && input.nowMs - sample.nowMs > 1U) || sourceAdvanced || gateClosed || previousShort != shortResult.state)
                return Pause(input, ProcessState::AwaitingFresh,
                    shortResult.feedInhibited ? ProcessReason::ShortActive : ProcessReason::ProcessBlocked);
            // Stable sub-ms evidence leaves the accepted motion endpoint in
            // place. Public selected speed is zero; the private previous-speed
            // endpoint remains available for the next ordinary fresh Step.
            snapshot_.speedMmMin = 0.0; return snapshot_;
        }

        const ProcessSnapshot& Step(const ProcessInput& input) noexcept
        {
            if (snapshot_.state == ProcessState::Idle || snapshot_.state == ProcessState::Cancelled ||
                snapshot_.state == ProcessState::Fault) return snapshot_;
            if (static_cast<unsigned>(input.command) > static_cast<unsigned>(ProcessCommand::Resume))
                return Fail(ProcessReason::InvalidCommand);
            if (!input.interlockReady) return Fail(ProcessReason::InterlockLost);
            if (input.configRevision != config_.flush.configRevision) return Fail(ProcessReason::RevisionChanged);
            if (input.recipeGeneration != config_.flush.recipeGeneration) return Fail(ProcessReason::GenerationChanged);
            if (input.sourceGeneration != config_.sourceGeneration) return Fail(ProcessReason::SourceChanged);
            if (input.nowMs < previousNowMs_) return Fail(ProcessReason::ClockRollback);
            const std::uint64_t ownerDelta = input.nowMs - previousNowMs_;
            previousNowMs_ = input.nowMs;
            if (input.command == ProcessCommand::Hold)
                return Pause(input, ProcessState::Hold, ProcessReason::Held);
            const bool explicitResume = snapshot_.state == ProcessState::Hold && input.command == ProcessCommand::Resume;
            if (snapshot_.state == ProcessState::Hold && !explicitResume)
            { Stop(); snapshot_.reason = ProcessReason::Held; return snapshot_; }
            if (input.command == ProcessCommand::Resume && !explicitResume)
                return Fail(ProcessReason::InvalidCommand);
            if (!input.processAllowed)
                return Pause(input, explicitResume ? ProcessState::Hold : ProcessState::AwaitingFresh,
                    ProcessReason::ProcessBlocked);
            const auto& sample = input.sample;
            if (!sample.valid || !std::isfinite(sample.voltage) || sample.sequence == 0U || sample.nowMs > input.nowMs)
                return Fail(ProcessReason::InvalidSample);
            if (input.nowMs - sample.nowMs > config_.maximumSampleAgeMs)
                return Fail(ProcessReason::StaleSample);
            if (haveSample_ && sample.sequence < lastSequence_) return Fail(ProcessReason::SequenceRegressed);
            if (haveSample_ && sample.nowMs < lastSourceMs_) return Fail(ProcessReason::SourceClockRegressed);
            if (haveSample_ && sample.sequence == lastSequence_)
            {
                if (sample.nowMs != lastSourceMs_ || sample.voltage != lastVoltage_)
                    return Fail(ProcessReason::ContradictorySample);
                return Pause(input, explicitResume ? ProcessState::Hold : ProcessState::AwaitingFresh,
                    ProcessReason::DuplicateSample);
            }
            if (haveSample_ && sample.nowMs == lastSourceMs_)
            {
                // Sequence alone cannot manufacture elapsed sample time.
                if (sample.voltage != lastVoltage_) return Fail(ProcessReason::ContradictorySample);
                lastSequence_ = sample.sequence; lastVoltage_ = sample.voltage;
                return Pause(input, explicitResume ? ProcessState::Hold : ProcessState::AwaitingFresh,
                    ProcessReason::DuplicateSample);
            }
            const bool recovery = pausedCycle_ || !haveSample_;
            const std::uint64_t sourceDelta = haveSample_ ? sample.nowMs - lastSourceMs_ : 0U;
            if (!recovery && (ownerDelta > EDM22::MaximumFlushStepMs || sourceDelta > EDM22::MaximumFlushStepMs))
                return Fail(ProcessReason::ServiceGap);
            if (recovery)
            {
                // Reset detector dwell but preserve an already active short
                // latch. The gap in observations cannot supply exit evidence.
                recoveryInhibit_ = recoveryInhibit_ || snapshot_.shortState == EDMGapServo::ShortState::Active ||
                    snapshot_.shortState == EDMGapServo::ShortState::Exiting;
                recoveryExitPending_ = false;
                detector_.Reset();
            }
            auto shortResult = detector_.Step(config_.servo, sample, true);
            if (!shortResult.valid) return Fail(ProcessReason::ShortFault);
            PreserveShortLatch(sample, shortResult);
            const auto curve = EDMGapServo::Evaluate(config_.servo, sample.voltage,
                config_.referenceV, config_.machiningOverridePercent);
            if (!curve.valid) return Fail(ProcessReason::CurveFault);
            snapshot_.shortState = shortResult.state;
            lastSourceMs_ = sample.nowMs; lastSequence_ = sample.sequence;
            lastVoltage_ = sample.voltage; haveSample_ = true;
            EDM23::AutomaticFlushCycleInput cycleInput = CycleInput(sample.nowMs);
            cycleInput.machiningAllowed = input.machiningAllowed && !shortResult.feedInhibited &&
                curve.limitedSpeedMmPerMin > 0.0;
            cycleInput.shortActive = shortResult.feedInhibited;
            cycleInput.returnAllowed = input.returnAllowed && !shortResult.feedInhibited;
            if (pausedCycle_) cycleInput.command = EDM23::AutomaticFlushCycleCommand::Resume;
            cycle_.Step(cycleInput); pausedCycle_ = false; CopyCycle();
            if (cycle_.Snapshot().state == EDM23::AutomaticFlushCycleState::Fault)
                return Fail(ProcessReason::CycleFault);
            snapshot_.state = ProcessState::Running; snapshot_.reason = ProcessReason::None;
            Select(input, shortResult, curve);
            if (recovery)
            {
                // Start/resume establishes a fresh endpoint without consuming
                // elapsed virtual work or issuing a nonzero selected speed.
                if (cycle_.Snapshot().state == EDM23::AutomaticFlushCycleState::Machining)
                {
                    cycleInput.command = EDM23::AutomaticFlushCycleCommand::Tick;
                    cycle_.Step(cycleInput); CopyCycle();
                }
                Stop(); return snapshot_;
            }
            const bool servoAction = snapshot_.action == ProcessAction::ServoAdvance ||
                snapshot_.action == ProcessAction::ServoRetreat || snapshot_.action == ProcessAction::ShortRetreat;
            if (servoAction && previousAction_ == snapshot_.action && previousSpeed_ != 0.0 && snapshot_.speedMmMin != 0.0)
            {
                // Source time and both endpoint gates govern the sole machining
                // integration. The flush executor integrates its own offset.
                const double speed = (previousSpeed_ + snapshot_.speedMmMin) * 0.5;
                const double next = snapshot_.machiningOffsetMm + speed * (static_cast<double>(sourceDelta) / 60000.0);
                if (!std::isfinite(next) || std::fabs(next) > 1000000.0)
                    return Fail(ProcessReason::NumericalFailure);
                snapshot_.machiningOffsetMm = next;
            }
            previousAction_ = snapshot_.action; previousSpeed_ = snapshot_.speedMmMin;
            return snapshot_;
        }

    private:
        ProcessConfig config_{};
        ProcessSnapshot snapshot_{};
        EDM23::AutomaticFlushCycle cycle_{};
        EDMGapServo::ShortDetector detector_{};
        std::uint64_t previousNowMs_ = 0U, lastSourceMs_ = 0U, lastSequence_ = 0U;
        double lastVoltage_ = 0.0, previousSpeed_ = 0.0;
        ProcessAction previousAction_ = ProcessAction::Wait;
        bool haveSample_ = false, pausedCycle_ = false, recoveryInhibit_ = false, recoveryExitPending_ = false;
        std::uint64_t recoveryExitSinceMs_ = 0U;

        void PreserveShortLatch(const EDMGapServo::Sample& sample, EDMGapServo::ShortResult& result) noexcept
        {
            if (!recoveryInhibit_) return;
            if (!config_.servo.shortMachiningEnabled) { recoveryInhibit_ = false; return; }
            if (sample.voltage > config_.servo.shortMachiningV + config_.servo.shortHysteresisV)
            {
                if (!recoveryExitPending_)
                { recoveryExitPending_ = true; recoveryExitSinceMs_ = sample.nowMs; }
                if (sample.nowMs - recoveryExitSinceMs_ >= config_.servo.shortExitMs)
                { recoveryInhibit_ = false; recoveryExitPending_ = false; return; }
                result.state = EDMGapServo::ShortState::Exiting;
            }
            else
            { recoveryExitPending_ = false; result.state = EDMGapServo::ShortState::Active; }
            result.feedInhibited = result.shortActive = true;
        }

        EDM23::AutomaticFlushCycleInput CycleInput(std::uint64_t sourceMs) const noexcept
        {
            EDM23::AutomaticFlushCycleInput result{}; result.nowMs = sourceMs;
            result.configRevision = config_.flush.configRevision;
            result.recipeGeneration = config_.flush.recipeGeneration; result.interlockReady = true;
            return result;
        }
        void CopyCycle() noexcept
        {
            const auto& value = cycle_.Snapshot(); snapshot_.completedCycles = value.completedCycles;
            snapshot_.workElapsedMs = value.workElapsedMs; snapshot_.flushOffsetMm = value.virtualOffsetMm;
            snapshot_.deepCycle = value.deepCycle;
        }
        void Stop() noexcept
        {
            snapshot_.action = ProcessAction::Wait; snapshot_.speedMmMin = 0.0;
            previousAction_ = ProcessAction::Wait; previousSpeed_ = 0.0;
        }
        const ProcessSnapshot& Pause(const ProcessInput&, ProcessState state, ProcessReason reason) noexcept
        {
            // Freeze at the last accepted SOURCE timestamp. Observation time
            // never advances either virtual position while a gate is closed.
            if (!pausedCycle_)
            {
                auto input = CycleInput(haveSample_ ? lastSourceMs_ : 0U);
                input.command = EDM23::AutomaticFlushCycleCommand::Hold; cycle_.Step(input);
                pausedCycle_ = true; CopyCycle();
            }
            snapshot_.state = state; snapshot_.reason = reason; Stop(); return snapshot_;
        }
        const ProcessSnapshot& Fail(ProcessReason reason) noexcept
        {
            cycle_.Revoke(); CopyCycle(); snapshot_.state = ProcessState::Fault;
            snapshot_.reason = reason; Stop(); return snapshot_;
        }
        void Select(const ProcessInput& input, const EDMGapServo::ShortResult& shortResult,
            const EDMGapServo::Result& curve) noexcept
        {
            const auto& cycle = cycle_.Snapshot();
            snapshot_.action = ProcessAction::Wait; snapshot_.speedMmMin = 0.0;
            if (cycle.state == EDM23::AutomaticFlushCycleState::ReturnBlocked)
            {
                snapshot_.direction = cycle.flushExecution.direction;
                snapshot_.action = ProcessAction::ReturnBlocked; snapshot_.reason = ProcessReason::ReturnBlocked; return;
            }
            if (cycle.state == EDM23::AutomaticFlushCycleState::Flushing)
            {
                const auto& execution = cycle.flushExecution;
                snapshot_.direction = execution.direction;
                snapshot_.action = execution.leg == EDM20::FlushLeg::Retreat ?
                    ProcessAction::FlushRetreat : ProcessAction::FlushReturn;
                const bool negative = execution.direction == EDM20::FlushDirection::OppositeMachining ||
                    execution.direction == EDM20::FlushDirection::MachineNegativeZ;
                snapshot_.speedMmMin = negative ? -cycle.requestedSpeedMmPerMin : cycle.requestedSpeedMmPerMin;
                return; // Short never adds a second retreat to a frozen flush.
            }
            snapshot_.direction = EDM20::FlushDirection::AlongMachining;
            if (!input.machiningAllowed)
            { snapshot_.reason = ProcessReason::MachiningBlocked; return; }
            if (shortResult.state == EDMGapServo::ShortState::Entering)
            { snapshot_.reason = ProcessReason::ShortEntering; return; }
            if (shortResult.shortActive)
            {
                const auto retreat = EDMGapServo::EvaluateShortRetreat(config_.servo, curve);
                snapshot_.action = ProcessAction::ShortRetreat;
                snapshot_.direction = EDM20::FlushDirection::OppositeMachining;
                // Fixed retreat is native mm/min; EvaluateShortRetreat owns
                // its bound. Curve fallback already contains E8 exactly once.
                snapshot_.speedMmMin = retreat.signedSpeedMmPerMin;
                snapshot_.reason = ProcessReason::ShortActive; return;
            }
            if (config_.machiningOverridePercent == 0.0)
            { snapshot_.reason = ProcessReason::ZeroOverride; return; }
            snapshot_.speedMmMin = curve.limitedSpeedMmPerMin;
            if (snapshot_.speedMmMin > 0.0) snapshot_.action = ProcessAction::ServoAdvance;
            else if (snapshot_.speedMmMin < 0.0)
            { snapshot_.action = ProcessAction::ServoRetreat; snapshot_.direction = EDM20::FlushDirection::OppositeMachining; }
            else snapshot_.reason = ProcessReason::Deadband;
        }
    };

    inline const char* ProcessStateName(ProcessState state) noexcept
    {
        switch (state)
        {
        case ProcessState::Idle: return "IDLE"; case ProcessState::Running: return "RUNNING";
        case ProcessState::AwaitingFresh: return "AWAITING_FRESH"; case ProcessState::Hold: return "HOLD";
        case ProcessState::Cancelled: return "CANCELLED"; case ProcessState::Fault: return "FAULT";
        } return "UNKNOWN";
    }
    inline const char* ProcessActionName(ProcessAction action) noexcept
    {
        switch (action)
        {
        case ProcessAction::Wait: return "WAIT"; case ProcessAction::ServoAdvance: return "SERVO_ADVANCE";
        case ProcessAction::ServoRetreat: return "SERVO_RETREAT"; case ProcessAction::ShortRetreat: return "SHORT_RETREAT";
        case ProcessAction::FlushRetreat: return "FLUSH_RETREAT"; case ProcessAction::FlushReturn: return "FLUSH_RETURN";
        case ProcessAction::ReturnBlocked: return "RETURN_BLOCKED";
        } return "UNKNOWN";
    }
    inline const char* ProcessReasonName(ProcessReason reason) noexcept
    {
        switch (reason)
        {
        case ProcessReason::None: return "NONE"; case ProcessReason::AwaitFresh: return "AWAIT_FRESH";
        case ProcessReason::DuplicateSample: return "DUPLICATE_SAMPLE"; case ProcessReason::Held: return "HELD";
        case ProcessReason::ProcessBlocked: return "PROCESS_BLOCKED"; case ProcessReason::MachiningBlocked: return "MACHINING_BLOCKED";
        case ProcessReason::ShortEntering: return "SHORT_ENTERING"; case ProcessReason::ShortActive: return "SHORT_ACTIVE";
        case ProcessReason::ReturnBlocked: return "RETURN_BLOCKED"; case ProcessReason::ZeroOverride: return "ZERO_OVERRIDE";
        case ProcessReason::Deadband: return "DEADBAND"; case ProcessReason::InvalidConfig: return "INVALID_CONFIG";
        case ProcessReason::InvalidLifecycle: return "INVALID_LIFECYCLE"; case ProcessReason::InvalidCommand: return "INVALID_COMMAND";
        case ProcessReason::InterlockLost: return "INTERLOCK_LOST"; case ProcessReason::InvalidSample: return "INVALID_SAMPLE";
        case ProcessReason::StaleSample: return "STALE_SAMPLE"; case ProcessReason::SourceChanged: return "SOURCE_CHANGED";
        case ProcessReason::RevisionChanged: return "REVISION_CHANGED"; case ProcessReason::GenerationChanged: return "GENERATION_CHANGED";
        case ProcessReason::SequenceRegressed: return "SEQUENCE_REGRESSED"; case ProcessReason::SourceClockRegressed: return "SOURCE_CLOCK_REGRESSED";
        case ProcessReason::ContradictorySample: return "CONTRADICTORY_SAMPLE"; case ProcessReason::ClockRollback: return "CLOCK_ROLLBACK";
        case ProcessReason::ServiceGap: return "SERVICE_GAP"; case ProcessReason::CurveFault: return "CURVE_FAULT";
        case ProcessReason::ShortFault: return "SHORT_FAULT"; case ProcessReason::CycleFault: return "CYCLE_FAULT";
        case ProcessReason::NumericalFailure: return "NUMERICAL_FAILURE";
        } return "UNKNOWN";
    }
}
