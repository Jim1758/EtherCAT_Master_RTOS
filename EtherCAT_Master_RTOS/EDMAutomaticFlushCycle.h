#pragma once

#include "EDMFlushExecution.h"
#include <limits>

// EDM23: one frozen recipe drives a bounded virtual work/flush cycle. There is
// no physical axis owner, discharge permission, I/O, allocation, or clock here.
namespace EDM23
{
    enum class AutomaticFlushCycleState : std::uint8_t
    { Idle, Machining, Flushing, Hold, ReturnBlocked, Cancelled, Fault };
    enum class AutomaticFlushCycleCommand : std::uint8_t { Tick, Hold, Resume };
    enum class AutomaticFlushCycleError : std::uint8_t
    {
        None, InvalidConfig, InvalidLifecycle, InvalidCommand,
        ConfigRevisionChanged, RecipeGenerationChanged, ClockRollback,
        ServiceGap, InterlockLost, InvalidPlan, ExecutorFault, CounterOverflow
    };

    struct AutomaticFlushCycleConfig
    {
        std::uint64_t workTimeMs = 0U; // E5, already checked/converted from s.
        double baseJumpHeightMm = 0.0; // E6, confirmed mm only.
        double jumpSpeedOverridePercent = 100.0; // E7.
        std::uint32_t deepFlushCycleInterval = 0U; // E21; zero disables it.
        double deepFlushHeightMultiplier = 0.0; // E22; positive if E21 > 0.
        EDM20::FlushProfile flushProfile{};
        // Direction/frame and explicit E11 policy come from the caller. Height
        // and E7 in this request are overwritten by the frozen values above.
        EDM20::FlushRequest flushRequest{};
        std::uint64_t configRevision = 0U;
        std::uint64_t recipeGeneration = 0U;
    };

    struct AutomaticFlushCycleInput
    {
        std::uint64_t nowMs = 0U;
        std::uint64_t configRevision = 0U;
        std::uint64_t recipeGeneration = 0U;
        bool interlockReady = false;
        bool machiningAllowed = false;
        bool shortActive = false;
        bool returnAllowed = false;
        AutomaticFlushCycleCommand command = AutomaticFlushCycleCommand::Tick;
    };

    struct AutomaticFlushCycleSnapshot
    {
        AutomaticFlushCycleState state = AutomaticFlushCycleState::Idle;
        AutomaticFlushCycleError error = AutomaticFlushCycleError::None;
        std::uint64_t completedCycles = 0U;
        std::uint64_t workElapsedMs = 0U;
        double effectiveHeightMm = 0.0;
        bool deepCycle = false;
        double requestedSpeedMmPerMin = 0.0; // Flush magnitude; direction in executor.
        double virtualOffsetMm = 0.0;
        EDM22::FlushExecutionSnapshot flushExecution{};
        EDM20::FlushPlanStatus planStatus = EDM20::FlushPlanStatus::InvalidRequest;
        std::uint64_t configRevision = 0U;
        std::uint64_t recipeGeneration = 0U;
        constexpr bool PhysicalMotionEnabled() const noexcept { return false; }
        constexpr bool PhysicalDischargeEnabled() const noexcept { return false; }
    };

    namespace AutomaticFlushCycleDetail
    {
        inline bool AcceptedPlan(EDM20::FlushPlanStatus status) noexcept
        {
            return status == EDM20::FlushPlanStatus::ReadyLogicalOnly ||
                status == EDM20::FlushPlanStatus::PausedByOverride;
        }

        inline bool Validate(const AutomaticFlushCycleConfig& config) noexcept
        {
            return config.workTimeMs > 0U && config.workTimeMs <= 10000U &&
                config.configRevision != 0U && config.recipeGeneration != 0U &&
                std::isfinite(config.baseJumpHeightMm) &&
                config.baseJumpHeightMm >= 0.0 && config.baseJumpHeightMm <= 100.0 &&
                EDM20::FlushPlanDetail::Percent(config.jumpSpeedOverridePercent) &&
                config.deepFlushCycleInterval <= 20U &&
                std::isfinite(config.deepFlushHeightMultiplier) &&
                config.deepFlushHeightMultiplier >= 0.0 &&
                config.deepFlushHeightMultiplier <= 20.0 &&
                (config.deepFlushCycleInterval == 0U || config.deepFlushHeightMultiplier > 0.0) &&
                static_cast<unsigned>(config.flushRequest.mode) <=
                    static_cast<unsigned>(EDM20::FlushMode::B1) &&
                config.flushRequest.jumpHeightConfirmed &&
                EDM20::ValidateFlushProfile(config.flushProfile);
        }
    }

    class AutomaticFlushCycle
    {
    public:
        const AutomaticFlushCycleSnapshot& Snapshot() const noexcept { return snapshot_; }

        void Reset() noexcept
        {
            config_ = AutomaticFlushCycleConfig{};
            basePlan_ = EDM20::FlushPlanResult{};
            deepPlan_ = EDM20::FlushPlanResult{};
            executor_.Reset();
            snapshot_ = AutomaticFlushCycleSnapshot{};
            previousMs_ = 0U;
            previousMachiningAllowed_ = false;
            heldState_ = AutomaticFlushCycleState::Machining;
        }

        void Revoke() noexcept
        {
            if (snapshot_.state == AutomaticFlushCycleState::Flushing ||
                snapshot_.state == AutomaticFlushCycleState::ReturnBlocked ||
                snapshot_.state == AutomaticFlushCycleState::Hold)
                executor_.Revoke();
            CopyExecution();
            snapshot_.state = AutomaticFlushCycleState::Cancelled;
            snapshot_.requestedSpeedMmPerMin = 0.0;
            previousMachiningAllowed_ = false;
        }

        bool Start(const AutomaticFlushCycleConfig& config, std::uint64_t nowMs) noexcept
        {
            if (snapshot_.state != AutomaticFlushCycleState::Idle)
            { Fail(AutomaticFlushCycleError::InvalidLifecycle); return false; }
            if (!AutomaticFlushCycleDetail::Validate(config))
            { Fail(AutomaticFlushCycleError::InvalidConfig); return false; }

            EDM20::FlushRequest request = config.flushRequest;
            request.jumpHeightMm = config.baseJumpHeightMm;
            request.speedOverridePercent = config.jumpSpeedOverridePercent;
            auto basePlan = EDM20::BuildFlushPlan(config.flushProfile, request);
            if (!AutomaticFlushCycleDetail::AcceptedPlan(basePlan.status))
            { Fail(AutomaticFlushCycleError::InvalidPlan); return false; }
            auto deepPlan = basePlan;
            if (config.deepFlushCycleInterval > 0U)
            {
                request.jumpHeightMm = config.baseJumpHeightMm * config.deepFlushHeightMultiplier;
                if (!std::isfinite(request.jumpHeightMm) || request.jumpHeightMm > 2000.0 ||
                    (config.baseJumpHeightMm > 0.0 && request.jumpHeightMm == 0.0))
                { Fail(AutomaticFlushCycleError::InvalidConfig); return false; }
                deepPlan = EDM20::BuildFlushPlan(config.flushProfile, request);
                if (!AutomaticFlushCycleDetail::AcceptedPlan(deepPlan.status))
                { Fail(AutomaticFlushCycleError::InvalidPlan); return false; }
            }
            // E7=0 suspends automatic flushing even for a logical zero-height
            // recipe; no completed-cycle/deep-cycle credit is manufactured.
            if (config.jumpSpeedOverridePercent == 0.0)
                basePlan.status = deepPlan.status = EDM20::FlushPlanStatus::PausedByOverride;

            config_ = config; // Value copy; caller edits cannot alter the cycle.
            basePlan_ = basePlan;
            deepPlan_ = deepPlan;
            previousMs_ = nowMs;
            previousMachiningAllowed_ = false;
            snapshot_.state = AutomaticFlushCycleState::Machining;
            snapshot_.configRevision = config.configRevision;
            snapshot_.recipeGeneration = config.recipeGeneration;
            snapshot_.effectiveHeightMm = config.baseJumpHeightMm;
            snapshot_.planStatus = basePlan.status;
            return true;
        }

        const AutomaticFlushCycleSnapshot& Step(const AutomaticFlushCycleInput& input) noexcept
        {
            const bool active = snapshot_.state == AutomaticFlushCycleState::Machining ||
                snapshot_.state == AutomaticFlushCycleState::Flushing ||
                snapshot_.state == AutomaticFlushCycleState::ReturnBlocked ||
                snapshot_.state == AutomaticFlushCycleState::Hold;
            if (!active) return snapshot_; // Terminal states never restart themselves.
            if (static_cast<unsigned>(input.command) >
                static_cast<unsigned>(AutomaticFlushCycleCommand::Resume))
                return Fail(AutomaticFlushCycleError::InvalidCommand);
            if (!input.interlockReady) return Fail(AutomaticFlushCycleError::InterlockLost);
            if (input.configRevision != snapshot_.configRevision)
                return Fail(AutomaticFlushCycleError::ConfigRevisionChanged);
            if (input.recipeGeneration != snapshot_.recipeGeneration)
                return Fail(AutomaticFlushCycleError::RecipeGenerationChanged);
            if (input.nowMs < previousMs_) return Fail(AutomaticFlushCycleError::ClockRollback);
            const std::uint64_t elapsedMs = input.nowMs - previousMs_;
            previousMs_ = input.nowMs;

            if (input.command == AutomaticFlushCycleCommand::Hold)
            {
                if (snapshot_.state != AutomaticFlushCycleState::Hold)
                {
                    heldState_ = snapshot_.state;
                    if (heldState_ != AutomaticFlushCycleState::Machining)
                        StepExecution(input, EDM22::FlushExecutionCommand::Hold);
                    snapshot_.state = AutomaticFlushCycleState::Hold;
                }
                snapshot_.requestedSpeedMmPerMin = 0.0;
                previousMachiningAllowed_ = false;
                return snapshot_;
            }
            if (snapshot_.state == AutomaticFlushCycleState::Hold)
            {
                if (input.command == AutomaticFlushCycleCommand::Resume)
                {
                    snapshot_.state = heldState_;
                    if (heldState_ != AutomaticFlushCycleState::Machining)
                        StepExecution(input, EDM22::FlushExecutionCommand::Resume);
                }
                snapshot_.requestedSpeedMmPerMin = 0.0;
                previousMachiningAllowed_ = false;
                return snapshot_; // Long HOLD and its release consume no work/distance.
            }
            if (input.command == AutomaticFlushCycleCommand::Resume)
                return Fail(AutomaticFlushCycleError::InvalidCommand);

            if (snapshot_.state != AutomaticFlushCycleState::Machining)
            {
                StepExecution(input, EDM22::FlushExecutionCommand::Tick);
                return snapshot_;
            }

            const bool allowed = input.machiningAllowed && !input.shortActive;
            // Both ends must permit machining. A closed gate neither consumes
            // time nor creates credit for its first recovery sample.
            if (allowed && previousMachiningAllowed_)
            {
                if (elapsedMs > EDM22::MaximumFlushStepMs)
                    return Fail(AutomaticFlushCycleError::ServiceGap);
                const std::uint64_t remaining = config_.workTimeMs - snapshot_.workElapsedMs;
                snapshot_.workElapsedMs += elapsedMs < remaining ? elapsedMs : remaining;
            }
            previousMachiningAllowed_ = allowed;
            if (!allowed || snapshot_.workElapsedMs < config_.workTimeMs) return snapshot_;
            if (snapshot_.completedCycles == (std::numeric_limits<std::uint64_t>::max)())
                return Fail(AutomaticFlushCycleError::CounterOverflow);

            const std::uint64_t nextCycle = snapshot_.completedCycles + 1U;
            snapshot_.deepCycle = config_.deepFlushCycleInterval > 0U &&
                nextCycle % config_.deepFlushCycleInterval == 0U;
            snapshot_.effectiveHeightMm = snapshot_.deepCycle ?
                config_.baseJumpHeightMm * config_.deepFlushHeightMultiplier : config_.baseJumpHeightMm;
            const auto& plan = snapshot_.deepCycle ? deepPlan_ : basePlan_;
            snapshot_.planStatus = plan.status;
            if (plan.status == EDM20::FlushPlanStatus::PausedByOverride)
                return snapshot_; // E7=0: saturated timer, no executor/count busy loop.
            executor_.Reset();
            if (!executor_.Start(plan.plan, config_.configRevision, input.nowMs,
                input.returnAllowed && !input.shortActive, input.interlockReady))
                return Fail(AutomaticFlushCycleError::ExecutorFault);
            CopyExecution();
            snapshot_.state = AutomaticFlushCycleState::Flushing;
            snapshot_.requestedSpeedMmPerMin = 0.0;
            previousMachiningAllowed_ = false;
            if (executor_.Snapshot().state == EDM22::FlushExecutionState::Complete)
                CompleteCycle(); // Zero height: one logical cycle, never a loop.
            return snapshot_; // A trigger tick advances no virtual distance.
        }

    private:
        AutomaticFlushCycleConfig config_{};
        EDM20::FlushPlanResult basePlan_{};
        EDM20::FlushPlanResult deepPlan_{};
        EDM22::FlushExecutor executor_{};
        AutomaticFlushCycleSnapshot snapshot_{};
        std::uint64_t previousMs_ = 0U;
        bool previousMachiningAllowed_ = false;
        AutomaticFlushCycleState heldState_ = AutomaticFlushCycleState::Machining;

        void CopyExecution() noexcept
        {
            snapshot_.flushExecution = executor_.Snapshot();
            snapshot_.virtualOffsetMm = snapshot_.flushExecution.offsetMm;
            snapshot_.requestedSpeedMmPerMin = snapshot_.flushExecution.requestedSpeedMmPerMin;
        }

        void CompleteCycle() noexcept
        {
            if (snapshot_.completedCycles == (std::numeric_limits<std::uint64_t>::max)())
            { Fail(AutomaticFlushCycleError::CounterOverflow); return; }
            ++snapshot_.completedCycles;
            snapshot_.workElapsedMs = 0U;
            snapshot_.state = AutomaticFlushCycleState::Machining;
            snapshot_.requestedSpeedMmPerMin = 0.0;
            previousMachiningAllowed_ = false; // Completion's remainder is not machining time.
        }

        void StepExecution(const AutomaticFlushCycleInput& input,
            EDM22::FlushExecutionCommand command) noexcept
        {
            EDM22::FlushExecutionInput execution{};
            execution.nowMs = input.nowMs;
            execution.configRevision = input.configRevision;
            execution.returnAllowed = input.returnAllowed && !input.shortActive;
            execution.interlockReady = input.interlockReady;
            execution.command = command;
            executor_.Step(execution);
            CopyExecution();
            switch (snapshot_.flushExecution.state)
            {
            case EDM22::FlushExecutionState::Running:
                snapshot_.state = AutomaticFlushCycleState::Flushing; break;
            case EDM22::FlushExecutionState::Hold:
                snapshot_.state = AutomaticFlushCycleState::Hold; break;
            case EDM22::FlushExecutionState::ReturnBlocked:
                snapshot_.state = AutomaticFlushCycleState::ReturnBlocked; break;
            case EDM22::FlushExecutionState::Complete:
                CompleteCycle(); break;
            case EDM22::FlushExecutionState::Fault:
                Fail(snapshot_.flushExecution.error == EDM22::FlushExecutionError::ServiceGap ?
                    AutomaticFlushCycleError::ServiceGap : AutomaticFlushCycleError::ExecutorFault); break;
            default:
                Fail(AutomaticFlushCycleError::ExecutorFault); break;
            }
        }

        const AutomaticFlushCycleSnapshot& Fail(AutomaticFlushCycleError error) noexcept
        {
            if (executor_.Snapshot().state == EDM22::FlushExecutionState::Running ||
                executor_.Snapshot().state == EDM22::FlushExecutionState::ReturnBlocked ||
                executor_.Snapshot().state == EDM22::FlushExecutionState::Hold)
                executor_.Revoke();
            CopyExecution();
            snapshot_.state = AutomaticFlushCycleState::Fault;
            snapshot_.error = error;
            snapshot_.requestedSpeedMmPerMin = 0.0;
            previousMachiningAllowed_ = false;
            return snapshot_;
        }
    };

    inline const char* AutomaticFlushCycleStateName(AutomaticFlushCycleState value) noexcept
    {
        switch (value)
        {
        case AutomaticFlushCycleState::Idle: return "IDLE";
        case AutomaticFlushCycleState::Machining: return "MACHINING";
        case AutomaticFlushCycleState::Flushing: return "FLUSHING";
        case AutomaticFlushCycleState::Hold: return "HOLD";
        case AutomaticFlushCycleState::ReturnBlocked: return "RETURN_BLOCKED";
        case AutomaticFlushCycleState::Cancelled: return "CANCELLED";
        case AutomaticFlushCycleState::Fault: return "FAULT";
        }
        return "UNKNOWN";
    }

    inline const char* AutomaticFlushCycleErrorName(AutomaticFlushCycleError value) noexcept
    {
        switch (value)
        {
        case AutomaticFlushCycleError::None: return "NONE";
        case AutomaticFlushCycleError::InvalidConfig: return "INVALID_CONFIG";
        case AutomaticFlushCycleError::InvalidLifecycle: return "INVALID_LIFECYCLE";
        case AutomaticFlushCycleError::InvalidCommand: return "INVALID_COMMAND";
        case AutomaticFlushCycleError::ConfigRevisionChanged: return "CONFIG_REVISION_CHANGED";
        case AutomaticFlushCycleError::RecipeGenerationChanged: return "RECIPE_GENERATION_CHANGED";
        case AutomaticFlushCycleError::ClockRollback: return "CLOCK_ROLLBACK";
        case AutomaticFlushCycleError::ServiceGap: return "SERVICE_GAP";
        case AutomaticFlushCycleError::InterlockLost: return "INTERLOCK_LOST";
        case AutomaticFlushCycleError::InvalidPlan: return "INVALID_PLAN";
        case AutomaticFlushCycleError::ExecutorFault: return "EXECUTOR_FAULT";
        case AutomaticFlushCycleError::CounterOverflow: return "COUNTER_OVERFLOW";
        }
        return "UNKNOWN";
    }
}
