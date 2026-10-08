#pragma once

#include "EDMLiveProcessCoordinator.h"
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

// EDM46: owner-thread, logical-only bridge from the existing live process.
// These are instantaneous linear velocity intents, never positions, planner
// commands, drive acknowledgements, stopping evidence or physical permits.
// Machining-path and machine-Z coordinates retain their separate origins.
namespace EDM46
{
    constexpr std::size_t AxisSlots = 8U;
    enum class IntentState : std::uint8_t { Idle, AwaitingFresh, Ready, Blocked, Revoked, Fault };
    enum class IntentFrame : std::uint8_t { None, MachiningPath, MachineZ };
    enum class IntentReason : std::uint8_t
    {
        None, AwaitFresh, NonFresh, DuplicateSample, InvalidConfig, InvalidScope,
        InvalidLifecycle, RetiredSession, ScopeChanged, SourceChanged, ConfigChanged,
        RecipeChanged, AuthorityLost, ClockRollback, StaleSample, SequenceRegressed,
        SampleTimeRegressed, InvalidLiveState, InvalidAction, InvalidDirection,
        InvalidSpeed, SpeedLimit, CounterOverflow
    };
    struct Scope
    {
        std::uint64_t session = 0U, run = 0U, cache = 0U, dispatch = 0U;
        std::uint64_t executionEpoch = 0U, ownerLease = 0U;
        std::uint64_t configRevision = 0U, recipeGeneration = 0U, frameGeneration = 0U;
    };
    inline bool ValidScope(const Scope& s) noexcept
    {
        return s.session && s.run && s.cache && s.dispatch && s.executionEpoch &&
            s.ownerLease && s.configRevision && s.recipeGeneration && s.frameGeneration;
    }
    inline bool SameScope(const Scope& a, const Scope& b) noexcept
    {
        return a.session == b.session && a.run == b.run && a.cache == b.cache &&
            a.dispatch == b.dispatch && a.executionEpoch == b.executionEpoch &&
            a.ownerLease == b.ownerLease && a.configRevision == b.configRevision &&
            a.recipeGeneration == b.recipeGeneration && a.frameGeneration == b.frameGeneration;
    }
    struct LinearFrame
    {
        // Selected axes must be confirmed installed linear axes. A rotary
        // channel cannot be interpreted as mm merely because a slot exists.
        std::uint32_t selectedAxisMask = 0U, installedLinearAxisMask = 0U;
        std::array<double, AxisSlots> machiningUnit{};
        std::uint32_t machineZAxis = 2U;
        bool machineZConfirmed = false, virtualFrameConfirmed = false;
        std::uint64_t frameGeneration = 0U;
    };
    struct Config
    {
        LinearFrame frame{};
        // Logical engineering bounds, not a certified machine envelope.
        double maximumScalarMmMin = 1000.0;
        std::array<double, AxisSlots> maximumAxisMmMin{};
    };
    struct IntentSnapshot
    {
        IntentState state = IntentState::Idle;
        IntentReason reason = IntentReason::None;
        Scope scope{};
        EDM26::LiveSourceIdentity sourceIdentity{};
        std::uint64_t sequence = 0U, sourceSequence = 0U, sourceTimeMs = 0U, observedAtMs = 0U;
        std::uint64_t readyObservations = 0U, nonzeroObservations = 0U;
        std::uint32_t actionMask = 0U, nonzeroActionMask = 0U;
        EDM25::ProcessAction action = EDM25::ProcessAction::Wait;
        IntentFrame frame = IntentFrame::None;
        double signedSpeedMmMin = 0.0;
        std::array<double, AxisSlots> axisSpeedMmMin{};
        bool ready = false, acceptedFreshSample = false;
        constexpr bool PhysicalMotionEnabled() const noexcept { return false; }
        constexpr bool PhysicalDischargeEnabled() const noexcept { return false; }
    };
    inline bool AllSpeedsZero(const IntentSnapshot& value) noexcept
    {
        if (value.signedSpeedMmMin != 0.0) return false;
        for (double speed : value.axisSpeedMmMin) if (speed != 0.0) return false;
        return true;
    }
    inline bool SameSource(const EDM26::LiveSourceIdentity& a,
        const EDM26::LiveSourceIdentity& b) noexcept
    {
        return a.source == b.source && a.profileRevision == b.profileRevision &&
            a.calibrationRevision == b.calibrationRevision && a.adIndex == b.adIndex &&
            a.deviceId == b.deviceId && a.channelId == b.channelId && a.pdoOffset == b.pdoOffset &&
            a.maxAgeMs == b.maxAgeMs && a.calibrationConfirmed == b.calibrationConfirmed;
    }
    inline bool ValidateConfig(const Config& config) noexcept
    {
        const auto& frame = config.frame;
        if (!frame.virtualFrameConfirmed || frame.frameGeneration == 0U ||
            frame.selectedAxisMask == 0U || (frame.selectedAxisMask & ~0xffU) != 0U ||
            (frame.installedLinearAxisMask & ~0xffU) != 0U ||
            (frame.selectedAxisMask & ~frame.installedLinearAxisMask) != 0U ||
            !std::isfinite(config.maximumScalarMmMin) || config.maximumScalarMmMin <= 0.0 ||
            frame.machineZAxis >= AxisSlots) return false;
        if (frame.machineZConfirmed &&
            (frame.machineZAxis != 2U || (frame.selectedAxisMask & (1U << frame.machineZAxis)) == 0U))
            return false;
        double normSquared = 0.0;
        for (std::size_t axis = 0U; axis < AxisSlots; ++axis)
        {
            const double unit = frame.machiningUnit[axis], limit = config.maximumAxisMmMin[axis];
            const bool selected = (frame.selectedAxisMask & (1U << axis)) != 0U;
            if (!std::isfinite(unit) || std::fabs(unit) > 1.0 ||
                !std::isfinite(limit) || limit < 0.0 || (selected && limit == 0.0) ||
                (!selected && unit != 0.0)) return false;
            normSquared += unit * unit;
        }
        // The caller provides an explicit unit direction; never silently
        // normalize a malformed coordinate conversion or change its scale.
        return std::isfinite(normSquared) && std::fabs(normSquared - 1.0) <= 1e-9;
    }

    class IntentAdapter
    {
    public:
        const IntentSnapshot& Snapshot() const noexcept { return snapshot_; }
        void Reset() noexcept
        {
            // Reset cannot make a previously admitted session identity new.
            const std::uint64_t retired = lastSession_;
            *this = IntentAdapter{};
            lastSession_ = retired;
        }
        void Revoke() noexcept
        {
            Neutralize();
            snapshot_.state = IntentState::Revoked;
            snapshot_.reason = IntentReason::None;
        }
        bool Start(const Config& config, const Scope& scope,
            const EDM26::LiveProcessSnapshot& initialLive, std::uint64_t nowMs) noexcept
        {
            if (snapshot_.state != IntentState::Idle) return FailBool(IntentReason::InvalidLifecycle);
            if (!ValidScope(scope)) return FailBool(IntentReason::InvalidScope);
            if (scope.session <= lastSession_) return FailBool(IntentReason::RetiredSession);
            if (!ValidateConfig(config) || scope.frameGeneration != config.frame.frameGeneration)
                return FailBool(IntentReason::InvalidConfig);
            const auto& source = initialLive.sourceIdentity;
            const bool physical = source.source == EDMGap::Source::PHYSICAL;
            if ((!physical && source.source != EDMGap::Source::SIMULATED) ||
                source.profileRevision == 0U || source.maxAgeMs == 0U ||
                (physical && (!source.calibrationConfirmed || source.calibrationRevision == 0U || source.adIndex == 0U)))
                return FailBool(IntentReason::SourceChanged);
            if (initialLive.state != EDM26::LiveProcessState::AwaitingFresh ||
                initialLive.process.state != EDM25::ProcessState::AwaitingFresh ||
                initialLive.acceptedFreshSample || initialLive.requestedSpeedMmPerMin != 0.0 ||
                initialLive.process.speedMmMin != 0.0 || initialLive.lastObservedMs != nowMs ||
                initialLive.lastSampleTimeMs > nowMs ||
                (initialLive.lastSampleSeq == 0U && initialLive.lastSampleTimeMs != 0U))
                return FailBool(IntentReason::InvalidLiveState);
            if (initialLive.cycle.configRevision != scope.configRevision)
                return FailBool(IntentReason::ConfigChanged);
            if (initialLive.cycle.recipeGeneration != scope.recipeGeneration)
                return FailBool(IntentReason::RecipeChanged);
            config_ = config; snapshot_.scope = scope; snapshot_.sourceIdentity = source;
            snapshot_.sourceSequence = initialLive.lastSampleSeq;
            snapshot_.sourceTimeMs = initialLive.lastSampleTimeMs;
            snapshot_.observedAtMs = previousNowMs_ = nowMs;
            lastSession_ = scope.session;
            snapshot_.state = IntentState::AwaitingFresh;
            snapshot_.reason = IntentReason::AwaitFresh;
            return true;
        }
        const IntentSnapshot& Observe(const Scope& scope,
            const EDM26::LiveProcessSnapshot& live, bool authorityCurrent, std::uint64_t nowMs) noexcept
        {
            Neutralize();
            if (snapshot_.state == IntentState::Fault || snapshot_.state == IntentState::Revoked)
                return snapshot_;
            if (snapshot_.state == IntentState::Idle) return Fail(IntentReason::InvalidLifecycle);
            if (!SameScope(scope, snapshot_.scope)) return Fail(IntentReason::ScopeChanged);
            if (!authorityCurrent) return Fail(IntentReason::AuthorityLost);
            if (!SameSource(live.sourceIdentity, snapshot_.sourceIdentity)) return Fail(IntentReason::SourceChanged);
            if (live.cycle.configRevision != scope.configRevision) return Fail(IntentReason::ConfigChanged);
            if (live.cycle.recipeGeneration != scope.recipeGeneration) return Fail(IntentReason::RecipeChanged);
            if (nowMs < previousNowMs_ || live.lastObservedMs < snapshot_.observedAtMs ||
                live.lastObservedMs > nowMs || live.lastSampleTimeMs > live.lastObservedMs)
                return Fail(IntentReason::ClockRollback);
            if (live.lastSampleSeq < snapshot_.sourceSequence) return Fail(IntentReason::SequenceRegressed);
            if (live.lastSampleTimeMs < snapshot_.sourceTimeMs) return Fail(IntentReason::SampleTimeRegressed);
            if (live.lastSampleSeq == snapshot_.sourceSequence && live.lastSampleTimeMs != snapshot_.sourceTimeMs)
                return Fail(IntentReason::SampleTimeRegressed);
            const bool newerSequence = live.lastSampleSeq > snapshot_.sourceSequence;
            const bool newerTime = live.lastSampleTimeMs > snapshot_.sourceTimeMs;
            previousNowMs_ = nowMs; snapshot_.observedAtMs = live.lastObservedMs;
            snapshot_.sourceSequence = live.lastSampleSeq; snapshot_.sourceTimeMs = live.lastSampleTimeMs;
            if (static_cast<unsigned>(live.state) > static_cast<unsigned>(EDM26::LiveProcessState::Fault) ||
                live.state == EDM26::LiveProcessState::Idle || live.state == EDM26::LiveProcessState::Fault)
                return Fail(IntentReason::InvalidLiveState);
            if (live.state == EDM26::LiveProcessState::Cancelled)
            { Revoke(); return snapshot_; }
            if (!live.acceptedFreshSample)
                return Block(IntentReason::NonFresh);
            if (live.state != EDM26::LiveProcessState::Running ||
                live.process.state != EDM25::ProcessState::Running || live.lastSampleSeq == 0U)
                return Fail(IntentReason::InvalidLiveState);
            if (!newerSequence || !newerTime) return Block(IntentReason::DuplicateSample);
            if (nowMs - live.lastObservedMs > snapshot_.sourceIdentity.maxAgeMs ||
                nowMs - live.lastSampleTimeMs > snapshot_.sourceIdentity.maxAgeMs)
                return Fail(IntentReason::StaleSample);
            const double speed = live.process.speedMmMin;
            if (!std::isfinite(speed) || live.requestedSpeedMmPerMin != speed)
                return Fail(IntentReason::InvalidSpeed);
            if (std::fabs(speed) > config_.maximumScalarMmMin) return Fail(IntentReason::SpeedLimit);
            IntentFrame mappedFrame = IntentFrame::None;
            if (!MapAction(live, mappedFrame)) return snapshot_;
            std::array<double, AxisSlots> speeds{};
            if (mappedFrame == IntentFrame::MachiningPath)
                for (std::size_t axis = 0U; axis < AxisSlots; ++axis)
                {
                    speeds[axis] = speed * config_.frame.machiningUnit[axis];
                    if (speed != 0.0 && config_.frame.machiningUnit[axis] != 0.0 && speeds[axis] == 0.0)
                        return Fail(IntentReason::InvalidSpeed);
                }
            else if (mappedFrame == IntentFrame::MachineZ)
            {
                if (!config_.frame.machineZConfirmed) return Fail(IntentReason::InvalidDirection);
                speeds[config_.frame.machineZAxis] = speed;
            }
            bool nonzero = false;
            for (std::size_t axis = 0U; axis < AxisSlots; ++axis)
            {
                if (!std::isfinite(speeds[axis])) return Fail(IntentReason::InvalidSpeed);
                if (std::fabs(speeds[axis]) > config_.maximumAxisMmMin[axis]) return Fail(IntentReason::SpeedLimit);
                nonzero = nonzero || speeds[axis] != 0.0;
            }
            const auto maximum = (std::numeric_limits<std::uint64_t>::max)();
            if (snapshot_.sequence == maximum || snapshot_.readyObservations == maximum ||
                (nonzero && snapshot_.nonzeroObservations == maximum)) return Fail(IntentReason::CounterOverflow);
            ++snapshot_.sequence; ++snapshot_.readyObservations;
            if (nonzero) ++snapshot_.nonzeroObservations;
            snapshot_.actionMask |= 1U << static_cast<unsigned>(live.process.action);
            if (nonzero) snapshot_.nonzeroActionMask |= 1U << static_cast<unsigned>(live.process.action);
            snapshot_.state = IntentState::Ready; snapshot_.reason = IntentReason::None;
            snapshot_.action = live.process.action; snapshot_.frame = mappedFrame;
            snapshot_.signedSpeedMmMin = speed; snapshot_.axisSpeedMmMin = speeds;
            snapshot_.ready = snapshot_.acceptedFreshSample = true;
            return snapshot_;
        }
    private:
        Config config_{};
        IntentSnapshot snapshot_{};
        std::uint64_t previousNowMs_ = 0U, lastSession_ = 0U;
        void Neutralize() noexcept
        {
            snapshot_.ready = snapshot_.acceptedFreshSample = false;
            snapshot_.action = EDM25::ProcessAction::Wait; snapshot_.frame = IntentFrame::None;
            snapshot_.signedSpeedMmMin = 0.0; snapshot_.axisSpeedMmMin.fill(0.0);
        }
        const IntentSnapshot& Fail(IntentReason reason) noexcept
        { Neutralize(); snapshot_.state = IntentState::Fault; snapshot_.reason = reason; return snapshot_; }
        bool FailBool(IntentReason reason) noexcept { Fail(reason); return false; }
        const IntentSnapshot& Block(IntentReason reason) noexcept
        { snapshot_.state = IntentState::Blocked; snapshot_.reason = reason; return snapshot_; }
        bool MapAction(const EDM26::LiveProcessSnapshot& live, IntentFrame& frame) noexcept
        {
            using Action = EDM25::ProcessAction;
            using Direction = EDM20::FlushDirection;
            using Cycle = EDM23::AutomaticFlushCycleState;
            const Action action = live.process.action;
            const Direction direction = live.process.direction;
            const double speed = live.process.speedMmMin;
            if (static_cast<unsigned>(action) > static_cast<unsigned>(Action::ReturnBlocked))
                return FailBool(IntentReason::InvalidAction);
            if (static_cast<unsigned>(direction) > static_cast<unsigned>(Direction::MachineNegativeZ))
                return FailBool(IntentReason::InvalidDirection);
            if (live.cycle.state != Cycle::Machining && live.cycle.state != Cycle::Flushing &&
                live.cycle.state != Cycle::ReturnBlocked)
                return FailBool(IntentReason::InvalidLiveState);
            if (static_cast<unsigned>(live.process.shortState) >=
                static_cast<unsigned>(EDMGapServo::ShortState::Invalid))
                return FailBool(IntentReason::InvalidLiveState);
            if (action == Action::Wait)
                return speed == 0.0 ? true : FailBool(IntentReason::InvalidSpeed);
            if (action == Action::ReturnBlocked)
            {
                if (speed != 0.0 || live.cycle.state != Cycle::ReturnBlocked)
                    return FailBool(IntentReason::InvalidAction);
                if (live.cycle.flushExecution.leg != EDM20::FlushLeg::Return ||
                    live.cycle.flushExecution.direction != direction ||
                    live.cycle.flushExecution.state != EDM22::FlushExecutionState::ReturnBlocked ||
                    live.cycle.requestedSpeedMmPerMin != 0.0 ||
                    live.cycle.flushExecution.requestedSpeedMmPerMin != 0.0)
                    return FailBool(IntentReason::InvalidAction);
                return direction == Direction::AlongMachining || direction == Direction::MachineNegativeZ ?
                    true : FailBool(IntentReason::InvalidDirection);
            }
            if (action == Action::ServoAdvance || action == Action::ServoRetreat || action == Action::ShortRetreat)
            {
                if (live.cycle.state != Cycle::Machining) return FailBool(IntentReason::InvalidAction);
                const auto shortState = live.process.shortState;
                if (action == Action::ShortRetreat ?
                    (shortState != EDMGapServo::ShortState::Active && shortState != EDMGapServo::ShortState::Exiting) :
                    shortState != EDMGapServo::ShortState::Clear)
                    return FailBool(IntentReason::InvalidAction);
                const bool advance = action == Action::ServoAdvance;
                if (direction != (advance ? Direction::AlongMachining : Direction::OppositeMachining))
                    return FailBool(IntentReason::InvalidDirection);
                if (advance ? speed <= 0.0 : (action == Action::ServoRetreat ? speed >= 0.0 : speed > 0.0))
                    return FailBool(IntentReason::InvalidSpeed);
                frame = IntentFrame::MachiningPath; return true;
            }
            if (live.cycle.state != Cycle::Flushing) return FailBool(IntentReason::InvalidAction);
            const bool retreat = action == Action::FlushRetreat;
            if (live.cycle.flushExecution.leg != (retreat ? EDM20::FlushLeg::Retreat : EDM20::FlushLeg::Return) ||
                live.cycle.flushExecution.direction != direction ||
                live.cycle.flushExecution.state != EDM22::FlushExecutionState::Running ||
                live.cycle.requestedSpeedMmPerMin != std::fabs(speed) ||
                !std::isfinite(live.cycle.flushExecution.requestedSpeedMmPerMin) ||
                live.cycle.flushExecution.requestedSpeedMmPerMin < 0.0 ||
                (speed != 0.0 && live.cycle.flushExecution.requestedSpeedMmPerMin != std::fabs(speed)))
                return FailBool(IntentReason::InvalidAction);
            // Starting a flush preloads the executor's first segment rate,
            // while the cycle intentionally exposes speed zero at that
            // boundary. Only a nonzero selected speed must equal that rate.
            const bool path = direction == (retreat ? Direction::OppositeMachining : Direction::AlongMachining);
            const bool machineZ = direction == (retreat ? Direction::MachinePositiveZ : Direction::MachineNegativeZ);
            if (!path && !machineZ) return FailBool(IntentReason::InvalidDirection);
            const bool negative = direction == Direction::OppositeMachining || direction == Direction::MachineNegativeZ;
            if (negative ? speed > 0.0 : speed < 0.0) return FailBool(IntentReason::InvalidSpeed);
            frame = path ? IntentFrame::MachiningPath : IntentFrame::MachineZ; return true;
        }
    };
    static_assert(sizeof(IntentAdapter) <= 1024U, "EDM46 intent storage must remain fixed and bounded.");
    inline const char* IntentStateName(IntentState state) noexcept
    {
        switch (state)
        {
        case IntentState::Idle: return "IDLE";
        case IntentState::AwaitingFresh: return "AWAITING_FRESH";
        case IntentState::Ready: return "READY";
        case IntentState::Blocked: return "BLOCKED";
        case IntentState::Revoked: return "REVOKED";
        case IntentState::Fault: return "FAULT";
        }
        return "UNKNOWN";
    }
    inline const char* IntentReasonName(IntentReason reason) noexcept
    {
        switch (reason)
        {
#define EDM46_REASON(name, label) case IntentReason::name: return label
        EDM46_REASON(None, "NONE"); EDM46_REASON(AwaitFresh, "AWAIT_FRESH");
        EDM46_REASON(NonFresh, "NON_FRESH"); EDM46_REASON(DuplicateSample, "DUPLICATE_SAMPLE");
        EDM46_REASON(InvalidConfig, "INVALID_CONFIG"); EDM46_REASON(InvalidScope, "INVALID_SCOPE");
        EDM46_REASON(InvalidLifecycle, "INVALID_LIFECYCLE"); EDM46_REASON(RetiredSession, "RETIRED_SESSION");
        EDM46_REASON(ScopeChanged, "SCOPE_CHANGED"); EDM46_REASON(SourceChanged, "SOURCE_CHANGED");
        EDM46_REASON(ConfigChanged, "CONFIG_CHANGED"); EDM46_REASON(RecipeChanged, "RECIPE_CHANGED");
        EDM46_REASON(AuthorityLost, "AUTHORITY_LOST"); EDM46_REASON(ClockRollback, "CLOCK_ROLLBACK");
        EDM46_REASON(StaleSample, "STALE_SAMPLE"); EDM46_REASON(SequenceRegressed, "SEQUENCE_REGRESSED");
        EDM46_REASON(SampleTimeRegressed, "SAMPLE_TIME_REGRESSED"); EDM46_REASON(InvalidLiveState, "INVALID_LIVE_STATE");
        EDM46_REASON(InvalidAction, "INVALID_ACTION"); EDM46_REASON(InvalidDirection, "INVALID_DIRECTION");
        EDM46_REASON(InvalidSpeed, "INVALID_SPEED"); EDM46_REASON(SpeedLimit, "SPEED_LIMIT");
        EDM46_REASON(CounterOverflow, "COUNTER_OVERFLOW");
#undef EDM46_REASON
        }
        return "UNKNOWN";
    }
}
