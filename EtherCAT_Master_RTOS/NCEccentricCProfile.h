#pragma once

#include "NCEccentricCRuntime.h"
#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>

// BASE78: a local, fixed-size scalar model, NOT a Motion driver or permission.
// Every segment starts/stops at an integer 250 us boundary. Velocity is linear
// on each ramp, constant on cruise, and continuous at all boundaries. Position
// is its analytic integral; no iterative position integration or endpoint
// velocity snap is used. Acceleration can change abruptly: jerk is not bounded.
//
// These samples REQUIRE zero inherited filter/history and NO additional FIR.
// Runtime.FirSamples() only describes its already validated conservative stop
// enclosure. It is not a request to filter this profile. Live transport, owner
// binding, reset/hold policy and 250 us execution-time qualification remain
// separate work. NCEccentricCDynamicMotionAdmission remains false.

enum class NCEccentricCProfileCode : std::uint8_t
{
    NOT_PREPARED = 0U, PREPARED = 1U, INVALID_RUNTIME = 2U, NUMERIC_RANGE = 3U
};

enum class NCEccentricCProfileState : std::uint8_t
{
    NOT_STARTED = 0U,
    AUTHORED_ACTIVE = 1U,
    CONTROLLED_STOP_ACTIVE = 2U,
    AUTHORED_COMPLETE = 3U,
    STOPPED_BEFORE_END = 4U,
    STOPPED_AT_OR_BEYOND_END = 5U
};

struct NCEccentricCProfilePoint
{
    NCEccentricCRuntimePoint runtime{};
    double scalarPulse = 0.0;
    double velocityPulsePerSec = 0.0;
    double accelerationPulsePerSec2 = 0.0;
    std::uint64_t sequence = 0ULL;
    NCEccentricCProfileState state = NCEccentricCProfileState::NOT_STARTED;
    bool valid = false;
    void Clear() noexcept { *this = NCEccentricCProfilePoint{}; }
};

namespace NCEccentricCProfileDetail
{
    constexpr double Dt = 0.00025;
    // Guard scalar speed arithmetic, without relaxing any BASE77 rate check.
    // Ramp rates receive an additional count-dependent guard below.
    constexpr double Headroom = 1.0 - 4096.0 * std::numeric_limits<double>::epsilon();
    constexpr std::uint64_t MaximumTicks = 2147483647ULL;

    struct Segment
    {
        double start = 0.0, end = 0.0, peak = 0.0, acceleration = 0.0, deceleration = 0.0;
        std::uint64_t up = 0ULL, cruise = 0ULL, down = 0ULL;
        bool stopping = false;
        std::uint64_t Ticks() const noexcept { return up + cruise + down; }
    };

    inline bool Positive(double x) noexcept { return std::isfinite(x) && x > 0.0; }
    inline double GuardedRampRate(double ceiling, double velocity) noexcept
    {
        if (!Positive(ceiling) || !Positive(velocity)) return 0.0;
        const double nominalTicks = (velocity / ceiling) / Dt;
        if (!Positive(nominalTicks) || nominalTicks > double(MaximumTicks)) return 0.0;
        // Computing adjacent v=peak*(tick/rampTicks) loses relative accuracy
        // proportional to rampTicks when those velocities are subtracted.
        // The eventual rounded ramp has at most 2*nominalTicks+4 ticks, since
        // the guard below is always >0.9999. 64 eps per such tick bounds the
        // quotient, multiply, subtraction and dt division with ample margin.
        // This lowers the planned derivative; it never widens the acceptance
        // ceiling. STOP uses its own short ramp bound (not a long authored one).
        const double countBound = (std::fmin)(double(MaximumTicks), 2.0 * nominalTicks + 4.0);
        const double guard = 1.0 - (4096.0 + 64.0 * countBound) *
            std::numeric_limits<double>::epsilon();
        return ceiling * guard;
    }
    inline bool Ticks(double value, std::uint64_t& ticks) noexcept
    {
        if (!std::isfinite(value) || value < 0.0 || value > double(MaximumTicks)) return false;
        const double rounded = std::ceil(value);
        if (rounded > double(MaximumTicks)) return false;
        ticks = static_cast<std::uint64_t>(rounded);
        return true;
    }

    // Rest-to-rest triangle/trapezoid. First find a continuous candidate peak;
    // round each ramp duration UP to ticks, then add any necessary cruise ticks.
    // Recompute peak from exact authored length and those three durations.
    // Longer rounded durations can only lower the candidate rate/acceleration.
    inline bool BuildAuthored(const NCEccentricCRuntimeValue& runtime,
        double start, Segment& output) noexcept
    {
        output = Segment{};
        const double end = runtime.AuthoredScalarPulse();
        const double distance = end - start;
        const double velocity = runtime.MaximumScalarVelocityPulsePerSec() * Headroom;
        const double acceleration = GuardedRampRate(runtime.MaximumScalarAccelerationPulsePerSec2(), velocity);
        const double deceleration = GuardedRampRate(runtime.MaximumScalarDecelerationPulsePerSec2(), velocity);
        if (!std::isfinite(start) || start < 0.0 || !Positive(distance) ||
            !Positive(velocity) || !Positive(acceleration) || !Positive(deceleration)) return false;
        const double distanceTicks = (distance / velocity) / Dt;
        const double upTicks = (velocity / acceleration) / Dt;
        const double downTicks = (velocity / deceleration) / Dt;
        if (!Positive(distanceTicks) || !Positive(upTicks) || !Positive(downTicks) ||
            distanceTicks > double(MaximumTicks) || upTicks > double(MaximumTicks) ||
            downTicks > double(MaximumTicks)) return false;
        const double halfRamps = 0.5 * upTicks + 0.5 * downTicks;
        const double fraction = distanceTicks < halfRamps ? std::sqrt(distanceTicks / halfRamps) : 1.0;
        if (!Positive(fraction)) return false;
        Segment segment{};
        segment.start = start; segment.end = end;
        if (!Ticks(upTicks * fraction, segment.up) || !Ticks(downTicks * fraction, segment.down) ||
            segment.up == 0ULL || segment.down == 0ULL) return false;
        const double remaining = distanceTicks - (0.5 * double(segment.up) + 0.5 * double(segment.down));
        if (remaining > 0.0 && !Ticks(remaining, segment.cruise)) return false;
        if (segment.Ticks() > MaximumTicks) return false;
        const double weightedTicks = 0.5 * double(segment.up) + double(segment.cruise) +
            0.5 * double(segment.down);
        segment.peak = distance / (weightedTicks * Dt);
        segment.acceleration = segment.peak / (double(segment.up) * Dt);
        segment.deceleration = segment.peak / (double(segment.down) * Dt);
        if (!Positive(segment.peak) || !Positive(segment.acceleration) || !Positive(segment.deceleration) ||
            segment.peak > runtime.MaximumScalarVelocityPulsePerSec() ||
            segment.acceleration > runtime.MaximumScalarAccelerationPulsePerSec2() ||
            segment.deceleration > runtime.MaximumScalarDecelerationPulsePerSec2()) return false;
        output = segment;
        return true;
    }

    inline bool BuildStop(const NCEccentricCRuntimeValue& runtime, double start,
        double velocity, Segment& output) noexcept
    {
        output = Segment{};
        if (!std::isfinite(start) || start < 0.0 || start > runtime.MaximumScalarPulse() ||
            !std::isfinite(velocity) || velocity < 0.0 ||
            velocity > runtime.MaximumScalarVelocityPulsePerSec()) return false;
        Segment segment{};
        segment.start = segment.end = start; segment.peak = velocity; segment.stopping = true;
        if (velocity > 0.0)
        {
            const double deceleration = GuardedRampRate(runtime.MaximumScalarStopDecelerationPulsePerSec2(), velocity);
            if (!Positive(deceleration) || !Ticks((velocity / deceleration) / Dt, segment.down) ||
                segment.down == 0ULL) return false;
            segment.deceleration = velocity / (double(segment.down) * Dt);
            const double distance = (0.5 * velocity) * (double(segment.down) * Dt);
            segment.end = start + distance;
            if (!Positive(distance) || !Positive(segment.deceleration) ||
                segment.deceleration > runtime.MaximumScalarStopDecelerationPulsePerSec2() ||
                !std::isfinite(segment.end) || segment.end <= start ||
                segment.end > runtime.MaximumScalarPulse()) return false;
        }
        output = segment;
        return true;
    }

    inline bool Sample(const Segment& segment, std::uint64_t tick,
        double& position, double& velocity, double& acceleration) noexcept
    {
        if (tick > segment.Ticks()) return false;
        if (tick == 0ULL)
        {
            position = segment.start;
            velocity = segment.stopping ? segment.peak : 0.0;
            acceleration = segment.stopping ? -segment.deceleration : segment.acceleration;
        }
        else if (tick == segment.Ticks())
        {
            position = segment.end; velocity = acceleration = 0.0;
        }
        else if (!segment.stopping && tick < segment.up)
        {
            const double ratio = double(tick) / double(segment.up);
            velocity = segment.peak * ratio;
            position = segment.start + (0.5 * velocity) * (double(tick) * Dt);
            acceleration = segment.acceleration;
        }
        else if (!segment.stopping && tick < segment.up + segment.cruise)
        {
            velocity = segment.peak;
            position = segment.start + segment.peak *
                ((0.5 * double(segment.up) + double(tick - segment.up)) * Dt);
            acceleration = 0.0;
        }
        else
        {
            const std::uint64_t remaining = segment.Ticks() - tick;
            if (segment.down == 0ULL) return false;
            velocity = segment.peak * (double(remaining) / double(segment.down));
            const std::uint64_t elapsed = segment.down - remaining;
            // Near the start of a long stop, end-minus-tail catastrophically
            // cancels. Integrate forward for the first half and backward from
            // the exact endpoint for the second half. Both are the same curve
            // to the explicit evaluation roundoff budget.
            if (elapsed < remaining)
            {
                const double decelStart = segment.start + segment.peak *
                    ((0.5 * double(segment.up) + double(segment.cruise)) * Dt);
                position = decelStart + segment.peak * (double(elapsed) * Dt) *
                    (1.0 - 0.5 * (double(elapsed) / double(segment.down)));
            }
            else position = segment.end - (0.5 * velocity) * (double(remaining) * Dt);
            acceleration = -segment.deceleration;
        }
        return std::isfinite(position) && position >= segment.start && position <= segment.end &&
            std::isfinite(velocity) && velocity >= 0.0 && velocity <= segment.peak &&
            std::isfinite(acceleration);
    }

    inline bool ContinuousSample(const NCEccentricCRuntimeValue& runtime, const Segment& segment,
        double beforePosition, double beforeVelocity, double position, double velocity) noexcept
    {
        const double displacement = position - beforePosition;
        const double meanDistance = (0.5 * beforeVelocity + 0.5 * velocity) * Dt;
        // Position subtraction can lose low bits at a large scalar anchor.
        // This bound is ONLY a floating evaluation/integral consistency check;
        // neither the scalar speed cap nor acceleration ceilings are relaxed.
        const double roundoff = 256.0 * std::numeric_limits<double>::epsilon() *
            ((std::fmax)(std::fabs(position), std::fabs(beforePosition)) +
                std::fabs(displacement) + std::fabs(meanDistance)) +
            8.0 * (std::numeric_limits<double>::denorm_min)();
        const double change = velocity - beforeVelocity;
        const double acceleration = std::fabs(change) / Dt;
        const double ceiling = change >= 0.0 ? runtime.MaximumScalarAccelerationPulsePerSec2() :
            segment.stopping ? runtime.MaximumScalarStopDecelerationPulsePerSec2() :
            runtime.MaximumScalarDecelerationPulsePerSec2();
        return std::isfinite(displacement) && displacement >= 0.0 && std::isfinite(roundoff) &&
            std::fabs(displacement - meanDistance) <= roundoff && std::isfinite(acceleration) &&
            acceleration <= ceiling && velocity <= runtime.MaximumScalarVelocityPulsePerSec();
    }
}

class NCEccentricCProfileValue;
class NCEccentricCProfileCursor;
inline NCEccentricCProfileCode PrepareNCEccentricCProfile(const NCEccentricCRuntimeValue&,
    NCEccentricCProfileValue&) noexcept;
inline bool BeginNCEccentricCProfile(const NCEccentricCProfileValue&, NCEccentricCProfileCursor&,
    NCEccentricCProfilePoint&) noexcept;
inline bool ReadNCEccentricCProfile(const NCEccentricCProfileCursor&, NCEccentricCProfilePoint&) noexcept;
inline bool AdvanceNCEccentricCProfile(NCEccentricCProfileCursor&, NCEccentricCProfilePoint&) noexcept;
inline bool RequestNCEccentricCProfileStop(NCEccentricCProfileCursor&, NCEccentricCProfilePoint&) noexcept;
inline bool ResumeNCEccentricCProfile(NCEccentricCProfileCursor&, NCEccentricCProfilePoint&) noexcept;

class NCEccentricCProfileValue
{
public:
    NCEccentricCProfileValue() noexcept = default;
    void Clear() noexcept { *this = NCEccentricCProfileValue{}; }
    bool IsValid() const noexcept { return valid_; }
    static constexpr bool RequiresUnfilteredSamples() noexcept { return true; }
    const NCEccentricCRuntimeValue& Runtime() const noexcept { return runtime_; }
    std::uint64_t TotalTicks() const noexcept { return segment_.Ticks(); }
    std::uint64_t AccelerationTicks() const noexcept { return segment_.up; }
    std::uint64_t CruiseTicks() const noexcept { return segment_.cruise; }
    std::uint64_t DecelerationTicks() const noexcept { return segment_.down; }
    double PeakVelocityPulsePerSec() const noexcept { return segment_.peak; }
    double AccelerationPulsePerSec2() const noexcept { return segment_.acceleration; }
    double DecelerationPulsePerSec2() const noexcept { return segment_.deceleration; }
private:
    NCEccentricCRuntimeValue runtime_{};
    NCEccentricCProfileDetail::Segment segment_{};
    bool valid_ = false;
    friend NCEccentricCProfileCode PrepareNCEccentricCProfile(const NCEccentricCRuntimeValue&,
        NCEccentricCProfileValue&) noexcept;
    friend bool BeginNCEccentricCProfile(const NCEccentricCProfileValue&, NCEccentricCProfileCursor&,
        NCEccentricCProfilePoint&) noexcept;
};

// Private cursor fields admit no independently writable position/velocity or
// external plan pointer. Copying a cursor copies its exact frozen source/plan;
// callers still need independent ownership/identity at a future Motion seam.
class NCEccentricCProfileCursor
{
public:
    NCEccentricCProfileCursor() noexcept = default;
    void Clear() noexcept { *this = NCEccentricCProfileCursor{}; }
    bool IsValid() const noexcept { return valid_; }
    NCEccentricCProfileState State() const noexcept { return state_; }
    double ScalarPulse() const noexcept { return position_; }
    double VelocityPulsePerSec() const noexcept { return velocity_; }
    std::uint64_t Sequence() const noexcept { return sequence_; }
    std::uint64_t SegmentTick() const noexcept { return tick_; }
    std::uint64_t SegmentTicks() const noexcept { return segment_.Ticks(); }
    double SegmentStartPulse() const noexcept { return segment_.start; }
    double SegmentEndPulse() const noexcept { return segment_.end; }
    double SegmentPeakVelocityPulsePerSec() const noexcept { return segment_.peak; }
    std::uint64_t SegmentAccelerationTicks() const noexcept { return segment_.up; }
    std::uint64_t SegmentCruiseTicks() const noexcept { return segment_.cruise; }
    std::uint64_t SegmentDecelerationTicks() const noexcept { return segment_.down; }
    const NCEccentricCRuntimeValue& Runtime() const noexcept { return plan_.Runtime(); }
private:
    NCEccentricCProfileValue plan_{};
    NCEccentricCProfileDetail::Segment segment_{};
    double position_ = 0.0, velocity_ = 0.0, acceleration_ = 0.0;
    std::uint64_t tick_ = 0ULL, sequence_ = 0ULL;
    NCEccentricCProfileState state_ = NCEccentricCProfileState::NOT_STARTED;
    bool valid_ = false;
    friend bool BeginNCEccentricCProfile(const NCEccentricCProfileValue&, NCEccentricCProfileCursor&,
        NCEccentricCProfilePoint&) noexcept;
    friend bool ReadNCEccentricCProfile(const NCEccentricCProfileCursor&, NCEccentricCProfilePoint&) noexcept;
    friend bool AdvanceNCEccentricCProfile(NCEccentricCProfileCursor&, NCEccentricCProfilePoint&) noexcept;
    friend bool RequestNCEccentricCProfileStop(NCEccentricCProfileCursor&, NCEccentricCProfilePoint&) noexcept;
    friend bool ResumeNCEccentricCProfile(NCEccentricCProfileCursor&, NCEccentricCProfilePoint&) noexcept;
};

inline NCEccentricCProfileCode PrepareNCEccentricCProfile(const NCEccentricCRuntimeValue& runtime,
    NCEccentricCProfileValue& output) noexcept
{
    if (!runtime.IsValid() || runtime.CycleSeconds() != NCEccentricCProfileDetail::Dt)
    {
        output.Clear();
        return NCEccentricCProfileCode::INVALID_RUNTIME;
    }
    // Snapshot first: Prepare(plan.Runtime(), plan) is a permitted alias.
    NCEccentricCProfileValue prepared{};
    prepared.runtime_ = runtime;
    output.Clear();
    const auto& frozen = prepared.runtime_;
    if (!NCEccentricCProfileDetail::BuildAuthored(frozen, 0.0, prepared.segment_))
        return NCEccentricCProfileCode::NUMERIC_RANGE;
    NCEccentricCRuntimePoint endpoint{};
    if (!EvaluateNCEccentricCRuntime(frozen, 0.0, 0.0, endpoint) ||
        !EvaluateNCEccentricCRuntime(frozen, frozen.AuthoredScalarPulse(), 0.0, endpoint))
        return NCEccentricCProfileCode::NUMERIC_RANGE;
    prepared.valid_ = true;
    output = prepared;
    return NCEccentricCProfileCode::PREPARED;
}

inline bool ReadNCEccentricCProfile(const NCEccentricCProfileCursor& cursor,
    NCEccentricCProfilePoint& output) noexcept
{
    output.Clear();
    if (!cursor.valid_ || !cursor.plan_.IsValid() || !std::isfinite(cursor.acceleration_)) return false;
    const auto& runtime = cursor.Runtime();
    const double accelerationCeiling = cursor.acceleration_ >= 0.0 ?
        runtime.MaximumScalarAccelerationPulsePerSec2() : cursor.segment_.stopping ?
        runtime.MaximumScalarStopDecelerationPulsePerSec2() : runtime.MaximumScalarDecelerationPulsePerSec2();
    if (std::fabs(cursor.acceleration_) > accelerationCeiling) return false;
    NCEccentricCProfilePoint point{};
    const auto phase = cursor.segment_.stopping ? NCEccentricCRuntimePhase::CONTROLLED_STOP :
        NCEccentricCRuntimePhase::AUTHORED;
    if (!EvaluateNCEccentricCRuntime(runtime, cursor.position_, cursor.velocity_, point.runtime, phase)) return false;
    point.scalarPulse = cursor.position_; point.velocityPulsePerSec = cursor.velocity_;
    point.accelerationPulsePerSec2 = cursor.acceleration_; point.sequence = cursor.sequence_;
    point.state = cursor.state_; point.valid = true;
    output = point;
    return true;
}

inline bool BeginNCEccentricCProfile(const NCEccentricCProfileValue& plan,
    NCEccentricCProfileCursor& cursor, NCEccentricCProfilePoint& output) noexcept
{
    output.Clear();
    if (!plan.IsValid() || cursor.valid_) return false;
    NCEccentricCProfileCursor candidate{};
    candidate.plan_ = plan; candidate.segment_ = plan.segment_;
    candidate.state_ = NCEccentricCProfileState::AUTHORED_ACTIVE; candidate.valid_ = true;
    if (!NCEccentricCProfileDetail::Sample(candidate.segment_, 0ULL, candidate.position_,
        candidate.velocity_, candidate.acceleration_) || !ReadNCEccentricCProfile(candidate, output)) return false;
    cursor = candidate;
    return true;
}

inline bool AdvanceNCEccentricCProfile(NCEccentricCProfileCursor& cursor,
    NCEccentricCProfilePoint& output) noexcept
{
    output.Clear();
    if (!cursor.valid_) return false;
    if (cursor.state_ == NCEccentricCProfileState::AUTHORED_COMPLETE ||
        cursor.state_ == NCEccentricCProfileState::STOPPED_BEFORE_END ||
        cursor.state_ == NCEccentricCProfileState::STOPPED_AT_OR_BEYOND_END)
        return ReadNCEccentricCProfile(cursor, output);
    if ((cursor.state_ != NCEccentricCProfileState::AUTHORED_ACTIVE &&
            cursor.state_ != NCEccentricCProfileState::CONTROLLED_STOP_ACTIVE) ||
        cursor.tick_ >= cursor.segment_.Ticks() ||
        cursor.sequence_ == (std::numeric_limits<std::uint64_t>::max)()) return false;
    NCEccentricCProfileCursor candidate = cursor;
    ++candidate.tick_; ++candidate.sequence_;
    if (!NCEccentricCProfileDetail::Sample(candidate.segment_, candidate.tick_, candidate.position_,
        candidate.velocity_, candidate.acceleration_) ||
        !NCEccentricCProfileDetail::ContinuousSample(candidate.Runtime(), candidate.segment_,
            cursor.position_, cursor.velocity_, candidate.position_, candidate.velocity_)) return false;
    if (candidate.tick_ == candidate.segment_.Ticks())
        candidate.state_ = !candidate.segment_.stopping ? NCEccentricCProfileState::AUTHORED_COMPLETE :
            candidate.position_ < candidate.Runtime().AuthoredScalarPulse() ?
            NCEccentricCProfileState::STOPPED_BEFORE_END : NCEccentricCProfileState::STOPPED_AT_OR_BEYOND_END;
    if (!ReadNCEccentricCProfile(candidate, output)) return false;
    cursor = candidate;
    return true;
}

inline bool RequestNCEccentricCProfileStop(NCEccentricCProfileCursor& cursor,
    NCEccentricCProfilePoint& output) noexcept
{
    output.Clear();
    if (!cursor.valid_) return false;
    // Repeated requests preserve the original stop anchor/target and tick.
    if (cursor.state_ == NCEccentricCProfileState::CONTROLLED_STOP_ACTIVE ||
        cursor.state_ == NCEccentricCProfileState::STOPPED_BEFORE_END ||
        cursor.state_ == NCEccentricCProfileState::STOPPED_AT_OR_BEYOND_END)
        return ReadNCEccentricCProfile(cursor, output);
    if (cursor.state_ != NCEccentricCProfileState::AUTHORED_ACTIVE) return false;
    NCEccentricCProfileCursor candidate = cursor;
    if (!NCEccentricCProfileDetail::BuildStop(candidate.Runtime(), cursor.position_, cursor.velocity_,
        candidate.segment_)) return false;
    candidate.tick_ = 0ULL;
    candidate.acceleration_ = -candidate.segment_.deceleration;
    candidate.state_ = candidate.segment_.Ticks() != 0ULL ? NCEccentricCProfileState::CONTROLLED_STOP_ACTIVE :
        candidate.position_ < candidate.Runtime().AuthoredScalarPulse() ?
        NCEccentricCProfileState::STOPPED_BEFORE_END : NCEccentricCProfileState::STOPPED_AT_OR_BEYOND_END;
    if (!ReadNCEccentricCProfile(candidate, output)) return false;
    cursor = candidate;
    return true;
}

inline bool ResumeNCEccentricCProfile(NCEccentricCProfileCursor& cursor,
    NCEccentricCProfilePoint& output) noexcept
{
    output.Clear();
    if (!cursor.valid_ || cursor.state_ != NCEccentricCProfileState::STOPPED_BEFORE_END ||
        cursor.velocity_ != 0.0 || cursor.position_ >= cursor.Runtime().AuthoredScalarPulse()) return false;
    NCEccentricCProfileCursor candidate = cursor;
    if (!NCEccentricCProfileDetail::BuildAuthored(candidate.Runtime(), cursor.position_, candidate.segment_)) return false;
    candidate.tick_ = 0ULL; candidate.acceleration_ = candidate.segment_.acceleration;
    candidate.state_ = NCEccentricCProfileState::AUTHORED_ACTIVE;
    if (!ReadNCEccentricCProfile(candidate, output)) return false;
    cursor = candidate;
    return true;
}

static_assert(std::is_trivially_copyable<NCEccentricCProfileValue>::value &&
    std::is_trivially_copyable<NCEccentricCProfileCursor>::value &&
    std::is_trivially_copyable<NCEccentricCProfilePoint>::value,
    "BASE78 profile must remain fixed, trivially-copyable model values.");
static_assert(sizeof(NCEccentricCProfileValue) <= 4096U &&
    sizeof(NCEccentricCProfileCursor) <= 4352U && sizeof(NCEccentricCProfilePoint) <= 192U,
    "BASE78 profile model storage must remain bounded.");
