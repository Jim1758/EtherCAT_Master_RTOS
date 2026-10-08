#pragma once

#include "EDMProcessCoordinator.h"
#include <limits>

// EDM25 G180 P10 local fixtures. No installed COND/AD or machine state is used.
namespace EDMProcessCoordinatorSimulationTest
{
    constexpr std::uint32_t CaseCount = 28U;
    constexpr std::uint32_t MaximumModelCallsPerCase = 256U;
    inline const char* CaseName(std::uint32_t index) noexcept
    {
        static const char* const names[CaseCount] = {
            "POSITIVE_SERVO", "NEGATIVE_SERVO", "DEADBAND", "ZERO_MACHINING_OVERRIDE",
            "SHORT_ENTERING_INHIBIT", "SHORT_ACTIVE_BOUNDED_RETREAT", "SHORT_EXIT_DEBOUNCE",
            "SHORT_CURVE_FALLBACK_ZERO", "FLUSH_PREEMPTS_SERVO", "SHORT_DURING_FLUSH_RETREAT",
            "SHORT_BLOCKS_FLUSH_RETURN", "RETURN_GATE_NO_CATCHUP", "DEEP_NTH_CYCLE", "B1_SIGNED_FRAME",
            "HOLD_WORK_EXPLICIT_RESUME", "HOLD_FLUSH_EXPLICIT_RESUME", "DUPLICATE_SAMPLE_NO_PROGRESS",
            "SOURCE_TIME_REQUIRED", "SEQUENCE_REGRESSION", "SOURCE_CLOCK_REGRESSION",
            "SOURCE_IDENTITY_FENCE", "REVISION_GENERATION_FENCES", "INVALID_NONFINITE_CONFIG",
            "INVALID_STALE_SAMPLE", "ACTIVE_SERVICE_GAP", "INTERLOCK_FAULT_LATCH",
            "REVOKE_RESET_REARM", "ZERO_FLUSH_OVERRIDE"
        };
        return index < CaseCount ? names[index] : "INVALID_CASE";
    }
    struct Snapshot
    {
        std::uint32_t state = 0U, reason = 0U, action = 0U, direction = 0U, shortState = 0U, modelCalls = 0U;
        std::uint64_t completedCycles = 0U, workElapsedMs = 0U;
        double machiningOffsetMm = 0.0, flushOffsetMm = 0.0, speedMmMin = 0.0;
        bool deepCycle = false;
        constexpr bool PhysicalMotionEnabled() const noexcept { return false; }
        constexpr bool PhysicalDischargeEnabled() const noexcept { return false; }
    };
    struct CaseResult { bool passed = false; std::uint32_t checks = 0U; Snapshot actual{}; };
    class Scenario
    {
    public:
        void Reset() noexcept { next_ = 0U; revoked_ = false; Prepare(); }
        void Revoke() noexcept { revoked_ = true; model_.Revoke(); Capture(); actual_.speedMmMin = 0.0; }
        const Snapshot& Current() const noexcept { return actual_; }
        CaseResult Step(std::uint32_t index) noexcept
        {
            using namespace EDM25;
            CaseResult result{};
            if (revoked_ || index != next_ || index >= CaseCount)
            { Revoke(); result.actual = Current(); return result; }
            ++next_; Prepare();
            switch (index)
            {
            case 0U:
                Begin(); Tick(10U); Action(ProcessAction::ServoAdvance); Near(S().speedMmMin, 2.5);
                Tick(10U); Near(S().machiningOffsetMm, 2.5 / 6000.0); Work(20U); break;
            case 1U:
                input_.sample.voltage = 40.0; Begin(); Tick(10U); Action(ProcessAction::ServoRetreat);
                Tick(10U); Near(S().machiningOffsetMm, -2.5 / 6000.0); Work(0U); break;
            case 2U:
                config_.servo.deadbandV = 1.0; input_.sample.voltage = 50.5; Begin(); Tick(10U);
                Action(ProcessAction::Wait); Reason(ProcessReason::Deadband); Work(0U); Near(S().speedMmMin, 0.0); break;
            case 3U:
                config_.machiningOverridePercent = 0.0;
                Begin(); Tick(10U); Tick(10U); Near(S().speedMmMin, 0.0); Work(0U);
                Near(S().machiningOffsetMm, 0.0); Reason(ProcessReason::ZeroOverride); break;
            case 4U:
                config_.servo.shortMachiningV = 100.0; Begin(); Tick(1U);
                Require(S().shortState == EDMGapServo::ShortState::Entering);
                Action(ProcessAction::Wait); Reason(ProcessReason::ShortEntering); Work(0U); break;
            case 5U:
                config_.servo.shortMachiningV = 100.0; config_.servo.shortRetreatMmPerMin = 100.0;
                config_.machiningOverridePercent = 20.0;
                Begin(); Tick(10U); Action(ProcessAction::ShortRetreat); Near(S().speedMmMin, -5.0);
                Tick(10U); Near(S().machiningOffsetMm, -5.0 / 6000.0); Work(0U);
                Prepare(false); config_.servo.shortMachiningV = 100.0;
                config_.machiningOverridePercent = 0.0; Begin(); Tick(10U);
                Action(ProcessAction::ShortRetreat); Near(S().speedMmMin, -0.5); break;
            case 6U:
                config_.servo.shortMachiningV = 45.0; input_.sample.voltage = 40.0;
                Begin(); Tick(10U); Require(S().shortState == EDMGapServo::ShortState::Active);
                input_.sample.voltage = 60.0; Tick(1U); Require(S().shortState == EDMGapServo::ShortState::Exiting);
                Action(ProcessAction::ShortRetreat); Tick(4U); Require(S().shortState == EDMGapServo::ShortState::Exiting);
                Tick(1U); Require(S().shortState == EDMGapServo::ShortState::Clear); Action(ProcessAction::ServoAdvance);
                Work(0U); Tick(10U); Work(10U); break;
            case 7U:
                config_.servo.shortRetreatMmPerMin = 0.0; config_.servo.shortMachiningV = 100.0;
                Begin(); Tick(10U); Action(ProcessAction::ShortRetreat); Near(S().speedMmMin, 0.0);
                input_.sample.voltage = 40.0; Tick(10U); Near(S().speedMmMin, -2.5);
                Prepare(false); config_.servo.shortRetreatMmPerMin = 0.0;
                config_.machiningOverridePercent = 0.0; input_.sample.voltage = 0.0;
                Begin(); Tick(10U); Action(ProcessAction::ShortRetreat); Near(S().speedMmMin, 0.0); break;
            case 8U:
                FastFlush(); Begin(); Trigger(); Tick(1U); Action(ProcessAction::FlushRetreat);
                Near(S().speedMmMin, -60.0); Near(S().flushOffsetMm, 0.001);
                constMachining(); Drain(); Require(S().completedCycles == 1U); constMachining(); break;
            case 9U:
                FastFlush(); config_.flush.baseJumpHeightMm = 0.010; Begin(); Trigger();
                input_.sample.voltage = 0.0; Tick(3U); Action(ProcessAction::FlushRetreat);
                Tick(3U); Require(S().shortState == EDMGapServo::ShortState::Active);
                Action(ProcessAction::FlushRetreat); Near(S().flushOffsetMm, 0.006); constMachining(); break;
            case 10U:
                FastFlush(); Begin(); Trigger(); ToReturn(); input_.sample.voltage = 0.0;
                Tick(1U); Action(ProcessAction::ReturnBlocked); Near(S().flushOffsetMm, 0.003);
                Tick(3U); Near(S().flushOffsetMm, 0.003); input_.sample.voltage = 60.0;
                Tick(1U); Action(ProcessAction::ReturnBlocked); Tick(5U); Near(S().flushOffsetMm, 0.003);
                Drain(); Require(S().completedCycles == 1U); break;
            case 11U:
                FastFlush(); Begin(); Trigger(); ToReturn(); input_.returnAllowed = false; Tick(1U);
                Action(ProcessAction::ReturnBlocked); Tick(100U); Near(S().flushOffsetMm, 0.003);
                input_.returnAllowed = true; Tick(100U); Near(S().flushOffsetMm, 0.003); Drain(); break;
            case 12U:
                FastFlush(); config_.flush.deepFlushCycleInterval = 3U;
                config_.flush.deepFlushHeightMultiplier = 2.0; Begin();
                for (std::uint32_t cycle = 1U; cycle <= 3U; ++cycle)
                {
                    Trigger(); Require(S().deepCycle == (cycle == 3U)); ToReturn();
                    Near(S().flushOffsetMm, cycle == 3U ? 0.006 : 0.003);
                    Drain(); Require(S().completedCycles == cycle);
                } break;
            case 13U:
                FastFlush(); config_.flush.flushRequest.mode = EDM20::FlushMode::B1;
                config_.flush.flushProfile.initialSlowDistanceMm = 0.001;
                config_.flush.flushProfile.initialSlowMmPerMin = 30.0; Begin(); Trigger(); Tick(1U);
                Require(S().direction == EDM20::FlushDirection::MachinePositiveZ);
                Near(S().speedMmMin, 30.0); Near(S().flushOffsetMm, 0.0005); ToReturn(); Tick(1U);
                Require(S().direction == EDM20::FlushDirection::MachineNegativeZ);
                Near(S().speedMmMin, -60.0); Drain(); break;
            case 14U:
            {
                Begin(); Tick(10U); Tick(10U); const double beforeWork = S().workElapsedMs;
                const double beforeOffset = S().machiningOffsetMm;
                input_.command = ProcessCommand::Hold; Tick(1U); State(ProcessState::Hold);
                input_.command = ProcessCommand::Tick; Tick(10000U); State(ProcessState::Hold);
                Near(S().machiningOffsetMm, beforeOffset); Near(static_cast<double>(S().workElapsedMs), beforeWork);
                input_.command = ProcessCommand::Resume; Tick(10000U); State(ProcessState::Running);
                Near(S().machiningOffsetMm, beforeOffset); Near(S().speedMmMin, 0.0);
                input_.command = ProcessCommand::Tick; Tick(10U); Work(30U); break;
            }
            case 15U:
                FastFlush(); Begin(); Trigger(); Tick(1U); Near(S().flushOffsetMm, 0.001);
                input_.command = ProcessCommand::Hold; Tick(1U); input_.command = ProcessCommand::Tick;
                Tick(10000U); Near(S().flushOffsetMm, 0.001); input_.command = ProcessCommand::Resume;
                Tick(10000U); Near(S().flushOffsetMm, 0.001); Near(S().speedMmMin, 0.0);
                input_.command = ProcessCommand::Tick; Tick(1U); Near(S().flushOffsetMm, 0.002); Drain();
                Prepare(false); FastFlush(); Begin(); Trigger(); input_.sample.voltage = 0.0;
                Tick(3U); Tick(3U); Action(ProcessAction::ReturnBlocked);
                input_.command = ProcessCommand::Hold; Tick(1U);
                input_.command = ProcessCommand::Resume; input_.sample.voltage = 0.5; Tick(10000U);
                Require(S().shortState == EDMGapServo::ShortState::Active); Near(S().flushOffsetMm, 0.003);
                input_.command = ProcessCommand::Tick; Tick(1U); Action(ProcessAction::ReturnBlocked);
                input_.sample.voltage = 60.0; Tick(1U); Require(S().shortState == EDMGapServo::ShortState::Exiting);
                Tick(5U); Near(S().flushOffsetMm, 0.003); Drain(); break;
            case 16U:
            {
                Begin(); Tick(10U); Tick(10U); const double before = S().machiningOffsetMm;
                input_.nowMs += 10U; Run(); State(ProcessState::AwaitingFresh);
                Reason(ProcessReason::DuplicateSample); Work(20U); Near(S().machiningOffsetMm, before);
                Tick(10000U); State(ProcessState::Running); Work(20U); Near(S().machiningOffsetMm, before);
                Tick(10U); Work(30U);
                Prepare(false); config_.servo.shortMachiningV = 45.0;
                input_.sample.voltage = 40.0; Begin(); Tick(10U);
                input_.nowMs += 1U; Run(); State(ProcessState::AwaitingFresh);
                input_.sample.voltage = 45.5; Tick(10000U); Require(S().shortState == EDMGapServo::ShortState::Active);
                Tick(1U); Action(ProcessAction::ShortRetreat);
                input_.sample.voltage = 60.0; Tick(1U); Require(S().shortState == EDMGapServo::ShortState::Exiting);
                input_.nowMs += 1U; Run(); Tick(10000U); Require(S().shortState == EDMGapServo::ShortState::Exiting);
                Tick(4U); Require(S().shortState == EDMGapServo::ShortState::Exiting);
                Tick(1U); Require(S().shortState == EDMGapServo::ShortState::Clear); Action(ProcessAction::ServoAdvance); break;
            }
            case 17U:
                Begin(); Tick(10U); ++input_.sample.sequence; input_.nowMs += 1U; Run();
                State(ProcessState::AwaitingFresh); Reason(ProcessReason::DuplicateSample); Work(10U);
                Tick(10U); Work(10U); Near(S().speedMmMin, 0.0); break;
            case 18U:
                Begin(); Tick(10U); --input_.sample.sequence; Run(); Fault(ProcessReason::SequenceRegressed); break;
            case 19U:
                Begin(); Tick(10U); ++input_.sample.sequence; --input_.sample.nowMs; Run();
                Fault(ProcessReason::SourceClockRegressed); break;
            case 20U:
                Begin(); ++input_.sourceGeneration; Tick(1U); Fault(ProcessReason::SourceChanged);
                --input_.sourceGeneration; Tick(1U); State(ProcessState::Fault); Near(S().speedMmMin, 0.0); break;
            case 21U:
                Begin(); ++input_.configRevision; Tick(1U); Fault(ProcessReason::RevisionChanged);
                Prepare(false); Begin(); ++input_.recipeGeneration; Tick(1U); Fault(ProcessReason::GenerationChanged); break;
            case 22U:
                config_.servo.positiveGain[0] = (std::numeric_limits<double>::quiet_NaN)();
                Require(!Start()); Fault(ProcessReason::InvalidConfig);
                Prepare(false); config_.referenceV = (std::numeric_limits<double>::infinity)();
                Require(!Start()); Fault(ProcessReason::InvalidConfig);
                Prepare(false); config_.flush.workTimeMs = 0U; Require(!Start()); Fault(ProcessReason::InvalidConfig); break;
            case 23U:
                Begin(); input_.sample.valid = false; Tick(1U); Fault(ProcessReason::InvalidSample);
                Prepare(false); Begin(); input_.nowMs += 51U; Run(); Fault(ProcessReason::StaleSample); break;
            case 24U:
                Begin(); Tick(251U); Fault(ProcessReason::ServiceGap); Near(S().machiningOffsetMm, 0.0);
                Prepare(false); config_.maximumSampleAgeMs = 250U; Begin(); input_.nowMs += 251U;
                input_.sample.nowMs += 1U; ++input_.sample.sequence;
                Run(); Fault(ProcessReason::ServiceGap); break;
            case 25U:
                Begin(); Tick(10U); input_.interlockReady = false; Tick(1U); Fault(ProcessReason::InterlockLost);
                input_.interlockReady = true; Tick(10U); State(ProcessState::Fault); Near(S().speedMmMin, 0.0); break;
            case 26U:
                Begin(); model_.Revoke(); Count(); State(ProcessState::Cancelled); Tick(10U);
                State(ProcessState::Cancelled); Require(!Start()); model_.Reset(); Count();
                Require(Start()); State(ProcessState::AwaitingFresh); Fresh(1U); Run();
                State(ProcessState::Running); Near(S().machiningOffsetMm, 0.0); break;
            case 27U:
                FastFlush(); config_.flush.jumpSpeedOverridePercent = 0.0; Begin(); Tick(20U);
                Work(20U); Require(S().completedCycles == 0U); Tick(100U); Work(20U);
                Require(S().completedCycles == 0U); Near(S().flushOffsetMm, 0.0); break;
            default: Require(false); break;
            }
            Capture(); Require(actual_.modelCalls <= MaximumModelCallsPerCase);
            Require(std::isfinite(actual_.machiningOffsetMm) && std::isfinite(actual_.flushOffsetMm) && std::isfinite(actual_.speedMmMin));
            Require(!actual_.PhysicalMotionEnabled() && !actual_.PhysicalDischargeEnabled());
            result.passed = passed_; result.checks = checks_; result.actual = actual_; return result;
        }
    private:
        EDM25::ProcessCoordinator model_{};
        EDM25::ProcessConfig config_{};
        EDM25::ProcessInput input_{};
        Snapshot actual_{};
        std::uint32_t next_ = 0U, calls_ = 0U, checks_ = 0U;
        bool revoked_ = false, passed_ = true;
        double machiningBeforeFlush_ = 0.0;
        const EDM25::ProcessSnapshot& S() const noexcept { return model_.Snapshot(); }
        void Prepare(bool resetCounters = true) noexcept
        {
            model_.Reset(); config_ = EDM25::ProcessConfig{};
            config_.flush.workTimeMs = 1000U; config_.flush.baseJumpHeightMm = 0.003;
            config_.flush.configRevision = config_.flush.recipeGeneration = 1U;
            config_.flush.flushRequest.jumpHeightConfirmed = true;
            config_.flush.flushRequest.frame.machiningDirectionConfirmed = true;
            config_.flush.flushRequest.frame.machineZDirectionConfirmed = true;
            config_.flush.flushProfile.mainRetractMmPerMin = 60.0;
            config_.flush.flushProfile.mainReturnMmPerMin = 60.0;
            config_.flush.flushProfile.finalApproachDistanceMm = 0.0;
            input_ = EDM25::ProcessInput{}; input_.configRevision = input_.recipeGeneration = input_.sourceGeneration = 1U;
            input_.interlockReady = input_.processAllowed = input_.machiningAllowed = input_.returnAllowed = true;
            input_.sample.valid = true; input_.sample.voltage = 60.0;
            input_.sample.sequence = 1U; input_.sample.nowMs = input_.nowMs = 100U;
            if (resetCounters) { calls_ = checks_ = 0U; passed_ = true; }
            actual_ = Snapshot{}; machiningBeforeFlush_ = 0.0;
        }
        void Require(bool value) noexcept { ++checks_; passed_ = passed_ && value; }
        void Near(double actual, double expected) noexcept { Require(std::fabs(actual - expected) <= 1e-10); }
        void Count() noexcept { ++calls_; Require(calls_ <= MaximumModelCallsPerCase); }
        bool Start() noexcept { Count(); return model_.Start(config_, input_.nowMs); }
        void Begin() noexcept { Require(Start()); Run(); State(EDM25::ProcessState::Running); Near(S().speedMmMin, 0.0); }
        void Fresh(std::uint64_t delta) noexcept { input_.nowMs += delta; input_.sample.nowMs = input_.nowMs; ++input_.sample.sequence; }
        void Run() noexcept { Count(); (void)model_.Step(input_); }
        void Tick(std::uint64_t delta) noexcept { Fresh(delta); Run(); }
        void State(EDM25::ProcessState value) noexcept { Require(S().state == value); }
        void Reason(EDM25::ProcessReason value) noexcept { Require(S().reason == value); }
        void Action(EDM25::ProcessAction value) noexcept { Require(S().action == value); }
        void Work(std::uint64_t value) noexcept { Require(S().workElapsedMs == value); }
        void Fault(EDM25::ProcessReason value) noexcept { State(EDM25::ProcessState::Fault); Reason(value); Near(S().speedMmMin, 0.0); }
        void FastFlush() noexcept { config_.flush.workTimeMs = 20U; }
        void Trigger() noexcept
        {
            for (std::uint32_t i = 0U; i < 8U && model_.CycleSnapshot().state == EDM23::AutomaticFlushCycleState::Machining; ++i) Tick(10U);
            Require(model_.CycleSnapshot().state == EDM23::AutomaticFlushCycleState::Flushing);
            Near(S().speedMmMin, 0.0); machiningBeforeFlush_ = S().machiningOffsetMm;
        }
        void ToReturn() noexcept
        {
            for (std::uint32_t i = 0U; i < 32U && model_.CycleSnapshot().flushExecution.leg != EDM20::FlushLeg::Return; ++i) Tick(1U);
            Require(model_.CycleSnapshot().flushExecution.leg == EDM20::FlushLeg::Return);
            Near(S().speedMmMin, 0.0);
        }
        void Drain() noexcept
        {
            for (std::uint32_t i = 0U; i < 32U && model_.CycleSnapshot().state != EDM23::AutomaticFlushCycleState::Machining; ++i) Tick(1U);
            Require(model_.CycleSnapshot().state == EDM23::AutomaticFlushCycleState::Machining);
            Near(S().flushOffsetMm, 0.0);
        }
        void constMachining() noexcept { Near(S().machiningOffsetMm, machiningBeforeFlush_); }
        void Capture() noexcept
        {
            actual_.state = static_cast<std::uint32_t>(S().state); actual_.reason = static_cast<std::uint32_t>(S().reason);
            actual_.action = static_cast<std::uint32_t>(S().action); actual_.direction = static_cast<std::uint32_t>(S().direction);
            actual_.shortState = static_cast<std::uint32_t>(S().shortState); actual_.modelCalls = calls_;
            actual_.completedCycles = S().completedCycles; actual_.workElapsedMs = S().workElapsedMs;
            actual_.machiningOffsetMm = S().machiningOffsetMm; actual_.flushOffsetMm = S().flushOffsetMm;
            actual_.speedMmMin = S().speedMmMin; actual_.deepCycle = S().deepCycle;
        }
    };
}
