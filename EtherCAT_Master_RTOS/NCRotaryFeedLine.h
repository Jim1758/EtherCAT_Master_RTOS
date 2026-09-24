#pragma once

#include "NCAxisIdentitySnapshot.h"
#include "NCRotaryFeedTarget.h"
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <type_traits>

// BASE69: a SINGLE positional rotary canonical line in native degrees and unwrapped
// logical pulses. This is immutable geometry/rate data, not Motion permission.
// The NC/Motion callers separately prove G90/G91, explicit F, source/currentness,
// travel limits, ownership, capture, and the exact-stop consumer contract.
//
// Only physical slots 3..7 may move, selected by one axisMask bit. The selected
// configured identity must be ROTARY (not continuous) with degree units. All
// eight endpoints are finite; unselected native/pulse endpoints retain bits.
// The selected native start may be modulo-reported. Its end MUST be start plus
// the resolved signed sweep, without wrapping the end. Pulse endpoints are unwrapped.
// Thus their DIFFERENCES agree through pulsePerDegree; absolute native/pulse
// values need not agree because the two frames may differ by whole revolutions.
// A signed sweep is never a millimetre length or a mixed mm/degree norm.
//
// No epsilon turns a nonzero segment into a point. Nonzero movement below the
// consumer's 1e-5-pulse floor, or speed below 1 pulse/s, is explicitly rejected.
// Delta consistency uses a binary64 subtraction/product roundoff allowance,
// capped at 1e-9 of the displacement. It is NOT a servo in-position window.
// Excessively coarse coordinates reject rather than hiding a different move.
// No allocation, I/O, platform call, shortest-path choice, or modulo is here.
#if defined(__FAST_MATH__) || defined(_M_FP_FAST)
#error BASE68_rotary_feed_requires_precise_floating_point_semantics
#endif
#if defined(__FINITE_MATH_ONLY__) && (__FINITE_MATH_ONLY__ > 0)
#error BASE68_rotary_feed_requires_nonfinite_checks
#endif

enum class NCRotaryFeedLineCode : std::uint8_t
{
    NOT_BUILT = 0U,
    BUILT_LINE = 1U,
    BUILT_POINT = 2U,
    INVALID_AXIS_IDENTITY = 3U,
    INVALID_AXIS_MASK = 4U,
    INVALID_FEED = 5U,
    NONFINITE_COORDINATE = 6U,
    OUTSIDE_AXIS_CHANGED = 7U,
    INVALID_AXIS_CONFIGURATION = 8U,
    NONFINITE_GEOMETRY = 9U,
    ZERO_LENGTH_MISMATCH = 10U,
    DIRECTION_MISMATCH = 11U,
    DELTA_CONVERSION_MISMATCH = 12U,
    BELOW_CONSUMER_RESOLUTION = 13U,
    INVALID_VELOCITY = 14U,
    AXIS_VELOCITY_LIMIT = 15U
};

struct NCRotaryFeedLineInput
{
    std::array<double, 8U> startMCS{}, endMCS{}, startPulse{}, endPulse{};
    NCAxisIdentitySnapshot axisIdentity{};
    std::uint32_t axisMask = 0U;
    double feedDegMin = 0.0;
    double pulsePerDegree = 0.0;
    double maxVelocityPPS = 0.0;
};

struct NCRotaryFeedLineValue
{
    std::array<double, 8U> startMCS{}, endMCS{}, startPulse{}, endPulse{};
    std::uint32_t axisMask = 0U;
    double feedDegMin = 0.0;
    double pulsePerDegree = 0.0;
    double sweepDeg = 0.0; // Signed authored native difference; never wrapped.
    double lengthPulse = 0.0;
    double velocityPPS = 0.0;
    // BASE69 G90 proof. Producer binds these after canonical geometry builds;
    // Motion independently proves WCS conversion and one-time target resolution.
    // G91 leaves all four fields zero; they never grant motion by themselves.
    double absoluteTargetWCS = 0.0, absoluteTargetMCS = 0.0, rotaryModulo = 0.0;
    bool rotaryShortestPath = false;
    bool point = false;
    bool valid = false;

    void Clear() noexcept
    {
        startMCS.fill(0.0); endMCS.fill(0.0);
        startPulse.fill(0.0); endPulse.fill(0.0);
        axisMask = 0U;
        feedDegMin = pulsePerDegree = sweepDeg = lengthPulse = velocityPPS = 0.0;
        absoluteTargetWCS = absoluteTargetMCS = rotaryModulo = 0.0;
        rotaryShortestPath = false;
        point = valid = false;
    }
};

namespace NCRotaryFeedDetail
{
    inline bool SameBits(double a, double b) noexcept
    {
        std::uint64_t aa = 0ULL, bb = 0ULL;
        std::memcpy(&aa, &a, sizeof(aa));
        std::memcpy(&bb, &b, sizeof(bb));
        return aa == bb;
    }

    // Positive finite operands; scale before multiplying so F*PPD/60 does
    // not overflow when its final result is representable.
    inline double Product(double a, double b, bool perMinute = false) noexcept
    {
        if (a == 0.0 || b == 0.0) return 0.0;
        int ea = 0, eb = 0;
        double m = std::frexp(a, &ea) * std::frexp(b, &eb);
        if (perMinute) m /= 60.0;
        return std::ldexp(m, ea + eb);
    }

    inline double Maximum(double a, double b) noexcept { return a > b ? a : b; }

    inline bool DeltaAgrees(double sweepDeg, double deltaPulse,
        double startDeg, double endDeg, double startPulse, double endPulse,
        double pulsePerDegree) noexcept
    {
        const double expected = Product(std::fabs(sweepDeg), pulsePerDegree);
        const double actual = std::fabs(deltaPulse);
        if (!std::isfinite(expected) || expected <= 0.0 || actual <= 0.0) return false;
        if (expected == actual) return true;
        const double relativeError = std::fabs(expected / actual - 1.0);
        if (!std::isfinite(relativeError) || relativeError > 1.0e-9) return false;
        const double nativeScale = Product(Maximum(std::fabs(startDeg), std::fabs(endDeg)),
            pulsePerDegree);
        const double scale = Maximum(Maximum(std::fabs(startPulse), std::fabs(endPulse)),
            Maximum(nativeScale, Maximum(expected, actual)));
        // An infinite endpoint scale is possible only for a harmless native
        // whole-turn offset product; the independent relative cap still holds.
        const double relativeBudget = 64.0 * (std::numeric_limits<double>::epsilon)() * (scale / actual);
        return relativeError <= relativeBudget;
    }
}

// Input/output are disjoint caller-owned objects and immutable during the call.
// Every rejection clears the output. Only BUILT_LINE/BUILT_POINT permit use.
inline NCRotaryFeedLineCode BuildNCRotaryFeedLine(
    const NCRotaryFeedLineInput& input, NCRotaryFeedLineValue& output) noexcept
{
    using Code = NCRotaryFeedLineCode;
    output.Clear();
    if (!IsNCAxisIdentitySnapshotValid(input.axisIdentity)) return Code::INVALID_AXIS_IDENTITY;
    const std::uint32_t mask = input.axisMask;
    if (mask == 0U || (mask & ~0xf8U) != 0U || (mask & (mask - 1U)) != 0U)
        return Code::INVALID_AXIS_MASK;
    if (!std::isfinite(input.feedDegMin) || input.feedDegMin <= 0.0 || input.feedDegMin > 100.0)
        return Code::INVALID_FEED;
    if (!std::isfinite(input.pulsePerDegree) || input.pulsePerDegree <= 0.0 ||
        !std::isfinite(input.maxVelocityPPS) || input.maxVelocityPPS <= 0.0)
        return Code::INVALID_AXIS_CONFIGURATION;
    unsigned selected = 8U;
    for (unsigned axis = 0U; axis < 8U; ++axis)
    {
        if (!std::isfinite(input.startMCS[axis]) || !std::isfinite(input.endMCS[axis]) ||
            !std::isfinite(input.startPulse[axis]) || !std::isfinite(input.endPulse[axis]))
            return Code::NONFINITE_COORDINATE;
        if ((mask & (1U << axis)) == 0U)
        {
            if (!NCRotaryFeedDetail::SameBits(input.startMCS[axis], input.endMCS[axis]) ||
                !NCRotaryFeedDetail::SameBits(input.startPulse[axis], input.endPulse[axis]))
                return Code::OUTSIDE_AXIS_CHANGED;
        }
        else
        {
            selected = axis;
            if (input.axisIdentity.exists[axis] != 1U || input.axisIdentity.axisType[axis] != 1U ||
                input.axisIdentity.nativeUnit[axis] != 2U)
                return Code::INVALID_AXIS_IDENTITY;
        }
    }
    if (selected >= 8U) return Code::INVALID_AXIS_MASK;
    const double sweepDeg = input.endMCS[selected] - input.startMCS[selected];
    const double deltaPulse = input.endPulse[selected] - input.startPulse[selected];
    if (!std::isfinite(sweepDeg) || !std::isfinite(deltaPulse)) return Code::NONFINITE_GEOMETRY;
    if ((sweepDeg == 0.0) != (deltaPulse == 0.0)) return Code::ZERO_LENGTH_MISMATCH;
    const bool point = sweepDeg == 0.0;
    const double lengthPulse = std::fabs(deltaPulse);
    double velocityPPS = 0.0;
    if (!point)
    {
        if ((sweepDeg < 0.0) != (deltaPulse < 0.0)) return Code::DIRECTION_MISMATCH;
        if (!NCRotaryFeedDetail::DeltaAgrees(sweepDeg, deltaPulse,
            input.startMCS[selected], input.endMCS[selected], input.startPulse[selected],
            input.endPulse[selected], input.pulsePerDegree)) return Code::DELTA_CONVERSION_MISMATCH;
        if (lengthPulse < 1.0e-5) return Code::BELOW_CONSUMER_RESOLUTION;
        velocityPPS = NCRotaryFeedDetail::Product(input.feedDegMin, input.pulsePerDegree, true);
        if (!std::isfinite(velocityPPS) || velocityPPS < 1.0) return Code::INVALID_VELOCITY;
        if (velocityPPS > input.maxVelocityPPS) return Code::AXIS_VELOCITY_LIMIT;
    }
    output.startMCS = input.startMCS; output.endMCS = input.endMCS;
    output.startPulse = input.startPulse; output.endPulse = input.endPulse;
    output.axisMask = mask;
    output.feedDegMin = input.feedDegMin; output.pulsePerDegree = input.pulsePerDegree;
    output.sweepDeg = sweepDeg; output.lengthPulse = lengthPulse; output.velocityPPS = velocityPPS;
    output.point = point; output.valid = true;
    return point ? Code::BUILT_POINT : Code::BUILT_LINE;
}

static_assert(std::numeric_limits<double>::is_iec559 && sizeof(double) == 8U,
    "BASE68 rotary feed requires IEEE binary64.");
static_assert(std::is_standard_layout<NCRotaryFeedLineValue>::value &&
    std::is_trivially_copyable<NCRotaryFeedLineValue>::value,
    "BASE68 rotary geometry must remain a simple bounded value.");
