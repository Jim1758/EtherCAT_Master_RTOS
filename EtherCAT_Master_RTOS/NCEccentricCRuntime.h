#pragma once

#include "NCEccentricCPath.h"
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>

// BASE77: prepared, allocation-free command-coordinate evaluation only.
// This is NOT a Motion packet, owner lease, ingress permission, stop driver,
// or 250 us execution-time qualification. BASE76 admission remains false.
// Prepare on a stopped, immutable source and copy the prepared value to its
// sole consumer. A later Motion lane must independently prepare/validate its
// transported source, bind the exact execution identity, and enforce the
// scalar-profile prerequisites below before using this evaluator.
// An instantaneous point cannot establish temporal continuity or acceleration.
// The caller must independently bind successive scalar positions to its
// velocities and enforce the exposed acceleration/deceleration ceilings; the
// current legacy trapezoid's terminal snap/clamp does not establish that proof.
//
// Scalar: nonnegative forward travel in the selected C role's pulse units.
// The prepared direction supplies the signed native C sweep. No mixed-unit
// XYZ/C norm, Actual-C feedback, shortest-path decision, or live H lookup is
// performed here. All stationary pulse anchors are preserved bit for bit.
//
// Stop/FIR envelope proof for a future monotone, bounded scalar profile:
//   0 <= every raw/filtered/history velocity <= V, positive box FIR of N
//   samples, dt=250 us, zero inherited velocity, no reverse/replay/blending.
// The raw/filter displacement lag is at most N*V*dt. Permit one further
// V*dt step before stop consumption. StopMove(MOVING) chooses either the
// filtered or the raw planning anchor plus v*T/2, and v<=V. Therefore a
// curve extended by V*(T/2+(N+1)*dt) beyond its authored endpoint contains
// that target and its positive-FIR output while the target clamp is honored.
// This is a geometric bound, NOT proof of the existing planner's acceleration:
// its terminal snap/clamp must separately satisfy the acceleration ceiling.
// The later lane must also preserve that same prepared T and N, bound all
// raw/filter history samples, and reject discontinuities before axis writes.

enum class NCEccentricCRuntimeCode : std::uint8_t
{
    NOT_PREPARED = 0U,
    PREPARED = 1U,
    INVALID_GEOMETRY = 2U,
    INVALID_TIMING = 3U,
    INVALID_PULSE_MAPPING = 4U,
    INVALID_TRAVEL_BOUNDS = 5U,
    STOP_ENVELOPE_REJECTED = 6U,
    NUMERIC_RANGE = 7U
};

enum class NCEccentricCRuntimePhase : std::uint8_t
{
    AUTHORED = 0U,
    CONTROLLED_STOP = 1U
};

struct NCEccentricCRuntimeInput
{
    NCEccentricCPathInput geometry{};
    std::array<double, 8U> startPulse{}, pulsePerNative{};
    std::array<double, 8U> travelMinNative{}, travelMaxNative{};
    std::uint32_t travelMask = 0U;
    std::uint32_t firSamples = 0U;
    double cycleSeconds = 0.00025;
    double stopSeconds = 0.0;
};

struct NCEccentricCRuntimePoint
{
    std::array<double, 8U> positionPulse{}, velocityPulse{};
    bool valid = false;
    void Clear() noexcept { *this = NCEccentricCRuntimePoint{}; }
};

class NCEccentricCRuntimeValue;
inline NCEccentricCRuntimeCode PrepareNCEccentricCRuntime(
    const NCEccentricCRuntimeInput&, NCEccentricCRuntimeValue&) noexcept;
inline bool EvaluateNCEccentricCRuntime(const NCEccentricCRuntimeValue&,
    double, double, NCEccentricCRuntimePoint&,
    NCEccentricCRuntimePhase = NCEccentricCRuntimePhase::AUTHORED) noexcept;

// Private prepared members prevent accidental assembly of a partly valid
// runtime object. This is not a wire format or a checksum against arbitrary
// memory corruption. Normal use is the complete result of Prepare; there is
// no public mutable valid bit or mutable geometry accessor.
class NCEccentricCRuntimeValue
{
public:
    NCEccentricCRuntimeValue() noexcept = default;
    void Clear() noexcept { *this = NCEccentricCRuntimeValue{}; }
    bool IsValid() const noexcept { return valid_; }
    const NCEccentricCPathValue& AuthoredPath() const noexcept { return authored_; }
    const NCEccentricCPathValue& ExtendedPath() const noexcept { return extended_; }
    const std::array<double, 8U>& StartPulse() const noexcept { return startPulse_; }
    const std::array<double, 8U>& EndPulse() const noexcept { return endPulse_; }
    const std::array<double, 8U>& MinimumPulse() const noexcept { return minimumPulse_; }
    const std::array<double, 8U>& MaximumPulse() const noexcept { return maximumPulse_; }
    const std::array<double, 8U>& PulsePerNative() const noexcept { return pulsePerNative_; }
    double AuthoredScalarPulse() const noexcept { return authoredScalarPulse_; }
    double MaximumScalarPulse() const noexcept { return maximumScalarPulse_; }
    double MaximumScalarVelocityPulsePerSec() const noexcept { return maxScalarVelocity_; }
    double MaximumScalarAccelerationPulsePerSec2() const noexcept { return maxScalarAcceleration_; }
    double MaximumScalarDecelerationPulsePerSec2() const noexcept { return maxScalarDeceleration_; }
    double MaximumScalarStopDecelerationPulsePerSec2() const noexcept { return maxScalarStopDeceleration_; }
    double StopReserveDegrees() const noexcept { return stopReserveDegrees_; }
    double StopSeconds() const noexcept { return stopSeconds_; }
    double CycleSeconds() const noexcept { return cycleSeconds_; }
    std::uint32_t FirSamples() const noexcept { return firSamples_; }

private:
    NCEccentricCPathValue authored_{}, extended_{};
    std::array<double, 8U> startPulse_{}, endPulse_{}, pulsePerNative_{};
    std::array<double, 8U> minimumPulse_{}, maximumPulse_{}, maximumVelocityPulse_{};
    std::array<double, 2U> startOffset_{}, signedH_{};
    double authoredScalarPulse_ = 0.0, maximumScalarPulse_ = 0.0;
    double maxScalarVelocity_ = 0.0, stopReserveDegrees_ = 0.0;
    double maxScalarAcceleration_ = 0.0, maxScalarDeceleration_ = 0.0;
    double maxScalarStopDeceleration_ = 0.0;
    double stopSeconds_ = 0.0, cycleSeconds_ = 0.0, direction_ = 0.0;
    std::uint32_t firSamples_ = 0U, role_ = 8U;
    bool valid_ = false;

    friend NCEccentricCRuntimeCode PrepareNCEccentricCRuntime(
        const NCEccentricCRuntimeInput&, NCEccentricCRuntimeValue&) noexcept;
    friend bool EvaluateNCEccentricCRuntime(const NCEccentricCRuntimeValue&,
        double, double, NCEccentricCRuntimePoint&, NCEccentricCRuntimePhase) noexcept;
};

namespace NCEccentricCRuntimeDetail
{
    inline bool HasNativePulsePrecision(double pulse, double pulsePerNative) noexcept
    {
        using namespace NCEccentricCDetail;
        if (!std::isfinite(pulse) || !Positive(pulsePerNative)) return false;
        const double infinity = (std::numeric_limits<double>::infinity)();
        const double upperStep = std::nextafter(pulse, infinity) - pulse;
        const double lowerStep = pulse - std::nextafter(pulse, -infinity);
        // Explicit ULP conversion is necessary for subnormal pulse scales;
        // epsilon*abs(pulse) alone can underflow to zero there.
        const double nativeError = 64.0 * Epsilon * std::fabs(pulse) / pulsePerNative +
            Max(upperStep, lowerStep) / pulsePerNative;
        return std::isfinite(nativeError) && nativeError <= NativeRoundoffBudget;
    }
}

inline NCEccentricCRuntimeCode PrepareNCEccentricCRuntime(
    const NCEccentricCRuntimeInput& input, NCEccentricCRuntimeValue& output) noexcept
{
    using namespace NCEccentricCDetail;
    output.Clear();
    NCEccentricCRuntimeValue prepared{};
    if (BuildNCEccentricCPath(input.geometry, prepared.authored_) !=
        NCEccentricCPathCode::BUILT_PATH)
        return NCEccentricCRuntimeCode::INVALID_GEOMETRY;
    // Match StopMove's real >=1ms branch explicitly; never inherit its hidden
    // 0.2s fallback. The 60s ceiling bounds this initial prepared contract.
    if (input.cycleSeconds != 0.00025 || !std::isfinite(input.stopSeconds) ||
        input.stopSeconds < 0.001 || input.stopSeconds > 60.0 ||
        input.firSamples < 1U || input.firSamples > 4096U)
        return NCEccentricCRuntimeCode::INVALID_TIMING;
    const NCEccentricCPathValue& path = prepared.authored_;
    const bool authoredXY = path.xDeltaMM != 0.0 || path.yDeltaMM != 0.0;
    prepared.role_ = path.electrodeAxis;
    prepared.direction_ = path.sweepDeg > 0.0 ? 1.0 : -1.0;
    prepared.startPulse_ = prepared.endPulse_ = input.startPulse;
    prepared.pulsePerNative_ = input.pulsePerNative;
    prepared.stopSeconds_ = input.stopSeconds;
    prepared.cycleSeconds_ = input.cycleSeconds;
    prepared.firSamples_ = input.firSamples;
    const double infinity = (std::numeric_limits<double>::infinity)();
    const double outward = 1.0 + 64.0 * Epsilon;
    for (unsigned a = 0U; a < 8U; ++a)
    {
        // Supply a positive conversion for held/unused slots as well.
        // Exact division binds the native geometry to the frozen pulse anchor;
        // all native starts, including C, are continuous unwrapped mechanical
        // coordinates. No approximate start rebase or modulo offset is accepted.
        if (!std::isfinite(input.startPulse[a]) || !Positive(input.pulsePerNative[a]) ||
            input.startPulse[a] / input.pulsePerNative[a] != path.startMCS[a])
            return NCEccentricCRuntimeCode::INVALID_PULSE_MAPPING;
        if (!std::isfinite(input.travelMinNative[a]) || !std::isfinite(input.travelMaxNative[a]) ||
            input.travelMinNative[a] > input.travelMaxNative[a])
            return NCEccentricCRuntimeCode::INVALID_TRAVEL_BOUNDS;
        if (!NCEccentricCRuntimeDetail::HasNativePulsePrecision(input.startPulse[a], input.pulsePerNative[a]))
            return NCEccentricCRuntimeCode::NUMERIC_RANGE;
    }
    if (input.travelMask == 0U || (input.travelMask & ~255U) != 0U ||
        (input.travelMask & path.groupMask) != path.groupMask)
        return NCEccentricCRuntimeCode::INVALID_TRAVEL_BOUNDS;

    const double speedDegrees = path.feedDegMin / 60.0;
    const double historyAndReactionSeconds = (double(input.firSamples) + 1.0) * input.cycleSeconds;
    const double reserve = speedDegrees * (0.5 * input.stopSeconds + historyAndReactionSeconds);
    prepared.stopReserveDegrees_ = std::nextafter(reserve * outward, infinity);
    const double extendedSweep = std::nextafter(
        (std::fabs(path.sweepDeg) + prepared.stopReserveDegrees_) * outward, infinity);
    if (!Positive(prepared.stopReserveDegrees_) || !Positive(extendedSweep) ||
        extendedSweep <= std::fabs(path.sweepDeg))
        return NCEccentricCRuntimeCode::NUMERIC_RANGE;
    NCEccentricCPathInput extendedInput = input.geometry;
    extendedInput.sweepDeg = prepared.direction_ * extendedSweep;
    // BASE79J: stopping continues the same Z/C pitch beyond the authored
    // endpoint. Extending C alone would bend the commanded Z trajectory.
    if (path.zDeltaMM != 0.0)
    {
        extendedInput.zDeltaMM = path.zDeltaMM * (extendedSweep / std::fabs(path.sweepDeg));
        if (!std::isfinite(extendedInput.zDeltaMM) ||
            std::fabs(extendedInput.zDeltaMM) <= std::fabs(path.zDeltaMM))
            return NCEccentricCRuntimeCode::NUMERIC_RANGE;
    }
    if (authoredXY)
    {
        double* extendedDelta[2] = { &extendedInput.xDeltaMM, &extendedInput.yDeltaMM };
        const double delta[2] = { path.xDeltaMM, path.yDeltaMM };
        for (unsigned a = 0U; a < 2U; ++a)
        {
            if (delta[a] == 0.0) continue;
            *extendedDelta[a] = delta[a] * (extendedSweep / std::fabs(path.sweepDeg));
            if (!std::isfinite(*extendedDelta[a]) || std::fabs(*extendedDelta[a]) <= std::fabs(delta[a]))
                return NCEccentricCRuntimeCode::NUMERIC_RANGE;
        }
    }
    // Both ordinary deceleration and the faster configured controlled stop
    // must pass the XY tangential+centripetal and native C rate limits.
    extendedInput.decTime = Min(path.decTime, input.stopSeconds);
    if (BuildNCEccentricCPath(extendedInput, prepared.extended_) !=
        NCEccentricCPathCode::BUILT_PATH)
        return NCEccentricCRuntimeCode::STOP_ENVELOPE_REJECTED;
    if (!IsNCEccentricCPathWithinBounds(prepared.extended_, input.travelMinNative,
        input.travelMaxNative, input.travelMask))
        return NCEccentricCRuntimeCode::STOP_ENVELOPE_REJECTED;

    const double rolePPU = input.pulsePerNative[prepared.role_];
    prepared.authoredScalarPulse_ = std::fabs(path.sweepDeg) * rolePPU;
    prepared.maximumScalarPulse_ = extendedSweep * rolePPU;
    prepared.maxScalarVelocity_ = speedDegrees * rolePPU;
    prepared.maxScalarAcceleration_ = prepared.maxScalarVelocity_ / path.accTime;
    prepared.maxScalarDeceleration_ = prepared.maxScalarVelocity_ / path.decTime;
    prepared.maxScalarStopDeceleration_ = prepared.maxScalarVelocity_ / input.stopSeconds;
    if (!Positive(prepared.authoredScalarPulse_) || !Positive(prepared.maximumScalarPulse_) ||
        prepared.maximumScalarPulse_ <= prepared.authoredScalarPulse_ ||
        !Positive(prepared.maxScalarVelocity_) || !Positive(prepared.maxScalarAcceleration_) ||
        !Positive(prepared.maxScalarDeceleration_) || !Positive(prepared.maxScalarStopDeceleration_))
        return NCEccentricCRuntimeCode::NUMERIC_RANGE;
    if (path.zDeltaMM != 0.0)
    {
        // Prove the exact evaluation order used below, including quotients.
        // A mathematically finite Z rate can otherwise hide overflow in
        // scalarVelocity/authoredScalar for a subnormal authored C sweep.
        const double progress = prepared.maximumScalarPulse_ / prepared.authoredScalarPulse_;
        const double progressRate = prepared.maxScalarVelocity_ / prepared.authoredScalarPulse_;
        const double zPulse = (path.zDeltaMM * progress) * input.pulsePerNative[2];
        const double zVelocity = (path.zDeltaMM * progressRate) * input.pulsePerNative[2];
        if (!Positive(progress) || !Positive(progressRate) ||
            !Positive(std::fabs(zPulse)) || !Positive(std::fabs(zVelocity)))
            return NCEccentricCRuntimeCode::NUMERIC_RANGE;
    }
    if (authoredXY)
    {
        const double delta[2] = { path.xDeltaMM, path.yDeltaMM };
        const double progress = prepared.maximumScalarPulse_ / prepared.authoredScalarPulse_;
        const double progressRate = prepared.maxScalarVelocity_ / prepared.authoredScalarPulse_;
        if (!Positive(progress) || !Positive(progressRate)) return NCEccentricCRuntimeCode::NUMERIC_RANGE;
        for (unsigned a = 0U; a < 2U; ++a)
        {
            if (delta[a] == 0.0) continue;
            const double ppu = input.pulsePerNative[a];
            const double pulse = (delta[a] * progress) * ppu;
            const double velocity = (delta[a] * progressRate) * ppu;
            const double authoredPulse = delta[a] * ppu;
            const double represented = (input.startPulse[a] + authoredPulse) - input.startPulse[a];
            const double error = std::fabs(represented - authoredPulse);
            if (!Positive(std::fabs(pulse)) || !Positive(std::fabs(velocity)) ||
                !std::isfinite(represented) || represented == 0.0 ||
                std::signbit(represented) != std::signbit(authoredPulse) ||
                error / ppu > NativeRoundoffBudget || error > std::fabs(authoredPulse) * RelativeSweepRoundoff)
                return NCEccentricCRuntimeCode::NUMERIC_RANGE;
        }
    }
    std::array<double, 3U> startOffset{}, endOffset{};
    if (!Offset(path.source, path.startMCS[prepared.role_], startOffset))
        return NCEccentricCRuntimeCode::NUMERIC_RANGE;
    if (authoredXY && !Offset(path.source, path.endMCS[prepared.role_], endOffset))
        return NCEccentricCRuntimeCode::NUMERIC_RANGE;
    if (authoredXY && std::remainder(path.sweepDeg, 360.0) == 0.0) endOffset = startOffset;
    const double sign = path.source.toolLengthMode == 43 ? 1.0 : -1.0;
    for (unsigned a = 0U; a < 2U; ++a)
    {
        prepared.startOffset_[a] = startOffset[a];
        prepared.signedH_[a] = sign * path.source.toolOffsetMM[a];
    }
    for (unsigned a = 0U; a < 8U; ++a)
    {
        const bool moving = (path.groupMask & (1U << a)) != 0U;
        const double ppu = input.pulsePerNative[a];
        if (!moving)
        {
            prepared.minimumPulse_[a] = prepared.maximumPulse_[a] = input.startPulse[a];
            continue;
        }
        const double endpointDelta = a == prepared.role_ ? prepared.direction_ *
            prepared.authoredScalarPulse_ : a == 2U ? path.zDeltaMM * ppu :
            authoredXY && a < 2U ? ((a == 0U ? path.xDeltaMM : path.yDeltaMM) +
                (endOffset[a] - startOffset[a])) * ppu :
            (path.endMCS[a] - path.startMCS[a]) * ppu;
        prepared.endPulse_[a] = input.startPulse[a] + endpointDelta;
        const double low = input.startPulse[a] +
            (prepared.extended_.minMCS[a] - path.startMCS[a]) * ppu;
        const double high = input.startPulse[a] +
            (prepared.extended_.maxMCS[a] - path.startMCS[a]) * ppu;
        const double pulseGuard = 64.0 * Epsilon *
            Max(std::fabs(input.startPulse[a]), Max(std::fabs(low), std::fabs(high)));
        prepared.minimumPulse_[a] = std::nextafter(low - pulseGuard, -infinity);
        prepared.maximumPulse_[a] = std::nextafter(high + pulseGuard, infinity);
        prepared.maximumVelocityPulse_[a] = std::nextafter(
            prepared.extended_.requiredVelocityNative[a] * ppu * outward, infinity);
        if (!std::isfinite(endpointDelta) || !std::isfinite(prepared.endPulse_[a]) ||
            !std::isfinite(prepared.minimumPulse_[a]) || !std::isfinite(prepared.maximumPulse_[a]) ||
            !Positive(prepared.maximumVelocityPulse_[a]) || !std::isfinite(pulseGuard) ||
            pulseGuard / ppu > NativeRoundoffBudget ||
            !NCEccentricCRuntimeDetail::HasNativePulsePrecision(prepared.minimumPulse_[a], ppu) ||
            !NCEccentricCRuntimeDetail::HasNativePulsePrecision(prepared.maximumPulse_[a], ppu) ||
            prepared.minimumPulse_[a] > input.startPulse[a] ||
            prepared.maximumPulse_[a] < input.startPulse[a] ||
            prepared.endPulse_[a] < prepared.minimumPulse_[a] ||
            prepared.endPulse_[a] > prepared.maximumPulse_[a])
            return NCEccentricCRuntimeCode::NUMERIC_RANGE;
        if (a == prepared.role_ || a == 2U)
        {
            const double actualDelta = prepared.endPulse_[a] - input.startPulse[a];
            const double error = std::fabs(actualDelta - endpointDelta);
            if (actualDelta == 0.0 || std::signbit(actualDelta) != std::signbit(endpointDelta) ||
                error / ppu > NativeRoundoffBudget ||
                error > std::fabs(endpointDelta) * RelativeSweepRoundoff)
                return NCEccentricCRuntimeCode::INVALID_PULSE_MAPPING;
        }
        // The native arc enclosure alone does not include the final affine
        // pulse conversion's rounding. Cover that extra guard in live travel
        // space too; an exactly tight supplied limit may therefore be rejected.
        if ((input.travelMask & (1U << a)) != 0U &&
            (prepared.minimumPulse_[a] / ppu < input.travelMinNative[a] ||
                prepared.maximumPulse_[a] / ppu > input.travelMaxNative[a]))
            return NCEccentricCRuntimeCode::STOP_ENVELOPE_REJECTED;
    }
    prepared.valid_ = true;
    output = prepared;
    return NCEccentricCRuntimeCode::PREPARED;
}

// Fixed work: one physical-angle reduction + sin/cos, two generated positions
// and velocities, and eight output checks. No Build, source table read, memory
// allocation, profile integration, unbounded loop, or queue operation occurs.
// CONTROLLED_STOP selects the already-proved geometric interval; its enum is
// not a safety owner/lease authorization and must never be used as one.
inline bool EvaluateNCEccentricCRuntime(const NCEccentricCRuntimeValue& prepared,
    double scalarPulse, double scalarVelocityPulsePerSec,
    NCEccentricCRuntimePoint& output, NCEccentricCRuntimePhase phase) noexcept
{
    using namespace NCEccentricCDetail;
    output.Clear();
    if (!prepared.valid_ || prepared.role_ >= 8U ||
        (phase != NCEccentricCRuntimePhase::AUTHORED && phase != NCEccentricCRuntimePhase::CONTROLLED_STOP) ||
        !std::isfinite(scalarPulse) || scalarPulse < 0.0 ||
        !std::isfinite(scalarVelocityPulsePerSec) || scalarVelocityPulsePerSec < 0.0 ||
        !Positive(prepared.authoredScalarPulse_) || !Positive(prepared.maximumScalarPulse_) ||
        !Positive(prepared.maxScalarVelocity_) ||
        scalarVelocityPulsePerSec > prepared.maxScalarVelocity_ ||
        scalarPulse > (phase == NCEccentricCRuntimePhase::AUTHORED ?
            prepared.authoredScalarPulse_ : prepared.maximumScalarPulse_)) return false;
    const unsigned role = prepared.role_;
    const double rolePPU = prepared.pulsePerNative_[role];
    if (!Positive(rolePPU) || (prepared.direction_ != 1.0 && prepared.direction_ != -1.0)) return false;
    const double angle = scalarPulse == prepared.authoredScalarPulse_ ? prepared.authored_.endMCS[role] :
        scalarPulse == prepared.maximumScalarPulse_ ? prepared.extended_.endMCS[role] :
        prepared.authored_.startMCS[role] + prepared.direction_ * (scalarPulse / rolePPU);
    double sine = 0.0, cosine = 0.0;
    if (!SinCos(angle, sine, cosine)) return false;
    const double offsetX = prepared.signedH_[0] * cosine - prepared.signedH_[1] * sine;
    const double offsetY = prepared.signedH_[0] * sine + prepared.signedH_[1] * cosine;
    const double omega = prepared.direction_ * (scalarVelocityPulsePerSec / rolePPU) * RadPerDeg;
    NCEccentricCRuntimePoint point{};
    point.positionPulse = prepared.startPulse_;
    if (prepared.authored_.generatedMask != 0U)
    {
        point.positionPulse[0] += (offsetX - prepared.startOffset_[0]) * prepared.pulsePerNative_[0];
        point.positionPulse[1] += (offsetY - prepared.startOffset_[1]) * prepared.pulsePerNative_[1];
        point.velocityPulse[0] = -offsetY * omega * prepared.pulsePerNative_[0];
        point.velocityPulse[1] = offsetX * omega * prepared.pulsePerNative_[1];
    }
    if (prepared.authored_.xDeltaMM != 0.0 || prepared.authored_.yDeltaMM != 0.0)
    {
        const double progress = scalarPulse / prepared.authoredScalarPulse_;
        const double progressRate = scalarVelocityPulsePerSec / prepared.authoredScalarPulse_;
        const double delta[2] = { prepared.authored_.xDeltaMM, prepared.authored_.yDeltaMM };
        const double offset[2] = { offsetX, offsetY };
        for (unsigned a = 0U; a < 2U; ++a)
        {
            if (delta[a] == 0.0) continue;
            point.positionPulse[a] = prepared.startPulse_[a] +
                (delta[a] * progress + (offset[a] - prepared.startOffset_[a])) * prepared.pulsePerNative_[a];
            point.velocityPulse[a] += (delta[a] * progressRate) * prepared.pulsePerNative_[a];
        }
    }
    if (prepared.authored_.zDeltaMM != 0.0)
    {
        const double progress = scalarPulse / prepared.authoredScalarPulse_;
        point.positionPulse[2] += (prepared.authored_.zDeltaMM * progress) * prepared.pulsePerNative_[2];
        point.velocityPulse[2] = (prepared.authored_.zDeltaMM *
            (scalarVelocityPulsePerSec / prepared.authoredScalarPulse_)) * prepared.pulsePerNative_[2];
    }
    point.positionPulse[role] += prepared.direction_ * scalarPulse;
    point.velocityPulse[role] = prepared.direction_ * scalarVelocityPulsePerSec;
    if (scalarPulse == 0.0) point.positionPulse = prepared.startPulse_;
    else if (scalarPulse == prepared.authoredScalarPulse_) point.positionPulse = prepared.endPulse_;
    for (unsigned a = 0U; a < 8U; ++a)
        if (!std::isfinite(point.positionPulse[a]) || !std::isfinite(point.velocityPulse[a]) ||
            !std::isfinite(prepared.minimumPulse_[a]) || !std::isfinite(prepared.maximumPulse_[a]) ||
            !std::isfinite(prepared.maximumVelocityPulse_[a]) ||
            point.positionPulse[a] < prepared.minimumPulse_[a] ||
            point.positionPulse[a] > prepared.maximumPulse_[a] ||
            std::fabs(point.velocityPulse[a]) > prepared.maximumVelocityPulse_[a]) return false;
    point.valid = true;
    output = point;
    return true;
}

static_assert(std::is_trivially_copyable<NCEccentricCRuntimeInput>::value &&
    std::is_trivially_copyable<NCEccentricCRuntimeValue>::value &&
    std::is_trivially_copyable<NCEccentricCRuntimePoint>::value,
    "BASE77 runtime preparation must remain fixed, trivially copyable values.");
static_assert(sizeof(NCEccentricCRuntimeInput) <= 1536U &&
    sizeof(NCEccentricCRuntimeValue) <= 4096U && sizeof(NCEccentricCRuntimePoint) <= 144U,
    "BASE77 runtime values must remain bounded.");
