#pragma once

#include "EDMFlushPlan.h"
#include <limits>

// EDM22: bounded virtual execution of an EDM20 B0/B1 logical plan. This class
// does not own an axis, clock, I/O permit, feedback sample, or physical output.
namespace EDM22
{
    enum class FlushExecutionState : std::uint8_t
    { Idle, Running, Hold, ReturnBlocked, Complete, Cancelled, Fault };
    enum class FlushExecutionCommand : std::uint8_t { Tick, Hold, Resume };
    enum class FlushExecutionError : std::uint8_t
    {
        None, InvalidPlan, InvalidLifecycle, InvalidCommand,
        ConfigRevisionChanged, ClockRollback, ServiceGap,
        NumericalFailure, InterlockLost, InvalidConfigRevision
    };

    constexpr std::uint64_t MaximumFlushStepMs = 250U;

    struct FlushExecutionInput
    {
        std::uint64_t nowMs = 0U;
        std::uint64_t configRevision = 0U;
        // Caller-owned prerequisites; true does not prove physical admission.
        bool returnAllowed = false;
        bool interlockReady = false;
        FlushExecutionCommand command = FlushExecutionCommand::Tick;
    };

    struct FlushExecutionSnapshot
    {
        FlushExecutionState state = FlushExecutionState::Idle;
        FlushExecutionError error = FlushExecutionError::None;
        std::size_t segmentIndex = 0U;
        std::size_t segmentCount = 0U;
        EDM20::FlushSegmentKind kind = EDM20::FlushSegmentKind::MainRetreat;
        EDM20::FlushLeg leg = EDM20::FlushLeg::Retreat;
        EDM20::FlushDirection direction = EDM20::FlushDirection::OppositeMachining;
        double segmentTravelledMm = 0.0;
        double retreatTravelledMm = 0.0;
        double returnTravelledMm = 0.0;
        double travelledMm = 0.0;
        double offsetMm = 0.0; // Nonnegative scalar distance from the virtual origin.
        double requestedSpeedMmPerMin = 0.0; // Magnitude; direction is separate.
        std::uint64_t configRevision = 0U;
        // One-sample marker: retreat completed, return has not advanced yet.
        bool legBoundary = false;
        constexpr bool PhysicalMotionEnabled() const noexcept { return false; }
        constexpr bool PhysicalDischargeEnabled() const noexcept { return false; }
    };

    namespace FlushExecutionDetail
    {
        inline bool EqualDistance(double left, double right) noexcept
        {
            if (left == right) return true;
            const double scale = left > right ? left : right;
            // Relative-only tolerance: no fixed mm floor that could accept an
            // unbalanced microscopic jump. Four additions need at most a few ulps.
            return std::fabs(left - right) <=
                scale * (16.0 * std::numeric_limits<double>::epsilon());
        }

        inline bool Validate(const EDM20::FlushPlan& plan) noexcept
        {
            if (plan.count > EDM20::MaximumFlushSegments ||
                !std::isfinite(plan.totalRetreatMm) ||
                !std::isfinite(plan.totalReturnMm) ||
                plan.totalRetreatMm < 0.0 || plan.totalReturnMm < 0.0)
                return false;
            if (plan.count == 0U)
                return plan.totalRetreatMm == 0.0 && plan.totalReturnMm == 0.0;
            if (plan.totalRetreatMm <= 0.0 || plan.totalReturnMm <= 0.0 ||
                !std::isfinite(plan.totalRetreatMm + plan.totalReturnMm))
                return false;
            const auto retreatDirection = plan.segments[0].direction;
            const bool isB1 = retreatDirection == EDM20::FlushDirection::MachinePositiveZ;
            if (!isB1 && retreatDirection != EDM20::FlushDirection::OppositeMachining)
                return false;
            const auto returnDirection = isB1 ? EDM20::FlushDirection::MachineNegativeZ :
                EDM20::FlushDirection::AlongMachining;
            double retreat = 0.0, returning = 0.0;
            int previousKind = -1;
            for (std::size_t i = 0U; i < plan.count; ++i)
            {
                const auto& segment = plan.segments[i];
                const int kind = static_cast<int>(segment.kind);
                if (kind < 0 || kind > 3 || kind <= previousKind ||
                    (!isB1 && kind == 0) || segment.requiresGapServo ||
                    !std::isfinite(segment.distanceMm) || segment.distanceMm <= 0.0 ||
                    !std::isfinite(segment.speedMmPerMin) || segment.speedMmPerMin <= 0.0)
                    return false;
                previousKind = kind;
                const bool isRetreat = kind < 2;
                if (segment.leg != (isRetreat ? EDM20::FlushLeg::Retreat : EDM20::FlushLeg::Return) ||
                    segment.direction != (isRetreat ? retreatDirection : returnDirection))
                    return false;
                if (isRetreat) retreat += segment.distanceMm;
                else returning += segment.distanceMm;
            }
            return retreat > 0.0 && returning > 0.0 &&
                std::isfinite(retreat) && std::isfinite(returning) &&
                std::isfinite(retreat + returning) &&
                EqualDistance(retreat, plan.totalRetreatMm) &&
                EqualDistance(returning, plan.totalReturnMm) &&
                EqualDistance(retreat, returning) &&
                EqualDistance(plan.totalRetreatMm, plan.totalReturnMm);
        }
    }

    class FlushExecutor
    {
    public:
        const FlushExecutionSnapshot& Snapshot() const noexcept { return snapshot_; }

        void Reset() noexcept
        {
            plan_ = EDM20::FlushPlan{};
            snapshot_ = FlushExecutionSnapshot{};
            previousMs_ = 0U;
        }

        void Revoke() noexcept
        {
            snapshot_.state = FlushExecutionState::Cancelled;
            snapshot_.requestedSpeedMmPerMin = 0.0;
            snapshot_.legBoundary = false;
        }

        bool Start(const EDM20::FlushPlan& plan, std::uint64_t configRevision,
            std::uint64_t nowMs, bool returnAllowed = false,
            bool interlockReady = false) noexcept
        {
            (void)returnAllowed; // Every nonempty accepted plan starts in retreat.
            if (snapshot_.state != FlushExecutionState::Idle)
            { Fail(FlushExecutionError::InvalidLifecycle); return false; }
            if (!interlockReady)
            { Fail(FlushExecutionError::InterlockLost); return false; }
            if (configRevision == 0U)
            { Fail(FlushExecutionError::InvalidConfigRevision); return false; }
            if (!FlushExecutionDetail::Validate(plan))
            { Fail(FlushExecutionError::InvalidPlan); return false; }
            plan_ = plan; // Frozen value copy: subsequent caller edits cannot change execution.
            previousMs_ = nowMs;
            snapshot_.configRevision = configRevision;
            snapshot_.segmentCount = plan.count;
            snapshot_.state = plan.count ? FlushExecutionState::Running : FlushExecutionState::Complete;
            if (plan.count) SelectSegment();
            return true;
        }

        const FlushExecutionSnapshot& Step(const FlushExecutionInput& input) noexcept
        {
            snapshot_.legBoundary = false;
            const bool active = snapshot_.state == FlushExecutionState::Running ||
                snapshot_.state == FlushExecutionState::Hold ||
                snapshot_.state == FlushExecutionState::ReturnBlocked;
            if (!active) return snapshot_; // Terminal states never restart themselves.
            if (static_cast<unsigned>(input.command) >
                static_cast<unsigned>(FlushExecutionCommand::Resume))
                return Fail(FlushExecutionError::InvalidCommand);
            if (!input.interlockReady) return Fail(FlushExecutionError::InterlockLost);
            if (input.configRevision != snapshot_.configRevision)
                return Fail(FlushExecutionError::ConfigRevisionChanged);
            if (input.nowMs < previousMs_) return Fail(FlushExecutionError::ClockRollback);
            const std::uint64_t elapsedMs = input.nowMs - previousMs_;
            previousMs_ = input.nowMs;
            if (input.command == FlushExecutionCommand::Hold)
            {
                snapshot_.state = FlushExecutionState::Hold;
                snapshot_.requestedSpeedMmPerMin = 0.0;
                return snapshot_;
            }
            if (snapshot_.state == FlushExecutionState::Hold)
            {
                if (input.command == FlushExecutionCommand::Resume)
                    snapshot_.state = snapshot_.leg == EDM20::FlushLeg::Return && !input.returnAllowed ?
                        FlushExecutionState::ReturnBlocked : FlushExecutionState::Running;
                snapshot_.requestedSpeedMmPerMin = 0.0;
                return snapshot_; // HOLD and its explicit resume consume no distance.
            }
            if (input.command == FlushExecutionCommand::Resume)
                return Fail(FlushExecutionError::InvalidCommand);
            if (snapshot_.state == FlushExecutionState::ReturnBlocked)
            {
                if (input.returnAllowed) snapshot_.state = FlushExecutionState::Running;
                snapshot_.requestedSpeedMmPerMin = 0.0;
                return snapshot_; // Gate release rebases rather than catching up.
            }
            if (snapshot_.leg == EDM20::FlushLeg::Return && !input.returnAllowed)
            {
                snapshot_.state = FlushExecutionState::ReturnBlocked;
                snapshot_.requestedSpeedMmPerMin = 0.0;
                return snapshot_;
            }
            if (elapsedMs > MaximumFlushStepMs) return Fail(FlushExecutionError::ServiceGap);
            if (elapsedMs == 0U) return snapshot_;
            double remainingMs = static_cast<double>(elapsedMs);
            for (std::size_t bound = 0U; bound < EDM20::MaximumFlushSegments; ++bound)
            {
                const auto& segment = plan_.segments[snapshot_.segmentIndex];
                const double remainingMm = segment.distanceMm - snapshot_.segmentTravelledMm;
                const double availableMm = segment.speedMmPerMin * (remainingMs / 60000.0);
                if (!std::isfinite(availableMm) || availableMm <= 0.0)
                    return Fail(FlushExecutionError::NumericalFailure);
                const bool finish = availableMm >= remainingMm;
                const double advanceMm = finish ? remainingMm : availableMm;
                const double previousTravelled = snapshot_.segmentTravelledMm;
                snapshot_.segmentTravelledMm = finish ? segment.distanceMm : previousTravelled + advanceMm;
                if (!finish && snapshot_.segmentTravelledMm == previousTravelled)
                    return Fail(FlushExecutionError::NumericalFailure);
                if (segment.leg == EDM20::FlushLeg::Retreat) snapshot_.retreatTravelledMm += advanceMm;
                else snapshot_.returnTravelledMm += advanceMm;
                UpdateDistances();
                if (!finish)
                {
                    snapshot_.requestedSpeedMmPerMin = segment.speedMmPerMin;
                    return snapshot_;
                }
                // availableMm comparison proved this segment fits; cancellation
                // error in the consumed-time arithmetic must not make time negative.
                const double consumedMs = (remainingMm / segment.speedMmPerMin) * 60000.0;
                remainingMs = consumedMs < remainingMs ? remainingMs - consumedMs : 0.0;
                ++snapshot_.segmentIndex;
                if (snapshot_.segmentIndex == plan_.count)
                {
                    snapshot_.retreatTravelledMm = plan_.totalRetreatMm;
                    snapshot_.returnTravelledMm = plan_.totalReturnMm;
                    UpdateDistances();
                    snapshot_.offsetMm = 0.0;
                    snapshot_.state = FlushExecutionState::Complete;
                    snapshot_.requestedSpeedMmPerMin = 0.0;
                    return snapshot_;
                }
                const bool reversal = segment.leg != plan_.segments[snapshot_.segmentIndex].leg;
                SelectSegment();
                if (reversal)
                {
                    snapshot_.retreatTravelledMm = plan_.totalRetreatMm;
                    UpdateDistances();
                    snapshot_.legBoundary = true;
                    snapshot_.requestedSpeedMmPerMin = 0.0;
                    if (!input.returnAllowed) snapshot_.state = FlushExecutionState::ReturnBlocked;
                    return snapshot_; // Observable reversal; discard this tick's unused time.
                }
                if (remainingMs <= 0.0) return snapshot_;
            }
            return Fail(FlushExecutionError::NumericalFailure); // Defensive fixed work bound.
        }

    private:
        EDM20::FlushPlan plan_{};
        FlushExecutionSnapshot snapshot_{};
        std::uint64_t previousMs_ = 0U;

        const FlushExecutionSnapshot& Fail(FlushExecutionError error) noexcept
        {
            snapshot_.state = FlushExecutionState::Fault;
            snapshot_.error = error;
            snapshot_.requestedSpeedMmPerMin = 0.0;
            snapshot_.legBoundary = false;
            return snapshot_;
        }

        void SelectSegment() noexcept
        {
            const auto& segment = plan_.segments[snapshot_.segmentIndex];
            snapshot_.kind = segment.kind;
            snapshot_.leg = segment.leg;
            snapshot_.direction = segment.direction;
            snapshot_.segmentTravelledMm = 0.0;
            snapshot_.requestedSpeedMmPerMin = segment.speedMmPerMin;
        }

        void UpdateDistances() noexcept
        {
            // Clamp only accumulated roundoff at the validated leg totals.
            if (snapshot_.retreatTravelledMm > plan_.totalRetreatMm)
                snapshot_.retreatTravelledMm = plan_.totalRetreatMm;
            if (snapshot_.returnTravelledMm > plan_.totalReturnMm)
                snapshot_.returnTravelledMm = plan_.totalReturnMm;
            snapshot_.travelledMm = snapshot_.retreatTravelledMm + snapshot_.returnTravelledMm;
            const double offset = snapshot_.retreatTravelledMm - snapshot_.returnTravelledMm;
            snapshot_.offsetMm = offset > 0.0 ? offset : 0.0;
        }
    };

    inline const char* FlushExecutionStateName(FlushExecutionState state) noexcept
    {
        switch (state)
        {
        case FlushExecutionState::Idle: return "IDLE";
        case FlushExecutionState::Running: return "RUNNING";
        case FlushExecutionState::Hold: return "HOLD";
        case FlushExecutionState::ReturnBlocked: return "RETURN_BLOCKED";
        case FlushExecutionState::Complete: return "COMPLETE";
        case FlushExecutionState::Cancelled: return "CANCELLED";
        case FlushExecutionState::Fault: return "FAULT";
        }
        return "UNKNOWN";
    }

    inline const char* FlushExecutionErrorName(FlushExecutionError error) noexcept
    {
        switch (error)
        {
        case FlushExecutionError::None: return "NONE";
        case FlushExecutionError::InvalidPlan: return "INVALID_PLAN";
        case FlushExecutionError::InvalidLifecycle: return "INVALID_LIFECYCLE";
        case FlushExecutionError::InvalidCommand: return "INVALID_COMMAND";
        case FlushExecutionError::ConfigRevisionChanged: return "CONFIG_REVISION_CHANGED";
        case FlushExecutionError::ClockRollback: return "CLOCK_ROLLBACK";
        case FlushExecutionError::ServiceGap: return "SERVICE_GAP";
        case FlushExecutionError::NumericalFailure: return "NUMERICAL_FAILURE";
        case FlushExecutionError::InterlockLost: return "INTERLOCK_LOST";
        case FlushExecutionError::InvalidConfigRevision: return "INVALID_CONFIG_REVISION";
        }
        return "UNKNOWN";
    }
}
