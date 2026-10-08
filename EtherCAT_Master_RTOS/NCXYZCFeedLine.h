#pragma once

#include "NCAxisIdentitySnapshot.h"
#include "NCXYZCAbsoluteFeedTarget.h"
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <type_traits>

// BASE75: independent G90/G91 XYZ+C immutable geometry. F is the XYZ linear path
// rate in mm/min; degrees never enter that path length. All four effective native
// displacements must move. One pulse-space scalar and one acceleration profile
// give every selected axis the same progress. Building this value grants no
// source, ownership, travel, capture, replay, or live Motion permission.
// G91 retains the full authored C increment without a modulo/shortest choice.
#if defined(__FAST_MATH__) || defined(_M_FP_FAST)
#error BASE75_XYZC_feed_requires_precise_floating_point_semantics
#endif
#if defined(__FINITE_MATH_ONLY__) && (__FINITE_MATH_ONLY__ > 0)
#error BASE75_XYZC_feed_requires_nonfinite_checks
#endif

enum class NCXYZCFeedLineCode : std::uint8_t
{
    NOT_BUILT = 0U,
    BUILT_LINE = 1U,
    INVALID_AXIS_IDENTITY = 2U,
    INVALID_AXIS_MASK = 3U,
    INVALID_FEED = 4U,
    NONFINITE_COORDINATE = 5U,
    OUTSIDE_AXIS_CHANGED = 6U,
    INVALID_AXIS_CONFIGURATION = 7U,
    INVALID_AUTHORED_DELTA = 8U,
    ENDPOINT_MISMATCH = 9U,
    NONFINITE_GEOMETRY = 10U,
    DIRECTION_MISMATCH = 11U,
    DELTA_CONVERSION_MISMATCH = 12U,
    BELOW_CONSUMER_RESOLUTION = 13U,
    INVALID_VELOCITY = 14U,
    AXIS_VELOCITY_LIMIT = 15U,
    ROTARY_FEED_LIMIT = 16U,
    INVALID_ABSOLUTE_TARGET = 17U
};

struct NCXYZCFeedLineInput
{
    std::array<double, 8U> startMCS{}, endMCS{}, startPulse{}, endPulse{};
    NCAxisIdentitySnapshot axisIdentity{};
    std::uint32_t axisMask = 0U;
    double feedMMMin = 0.0;
    std::array<double, 4U> deltaNative{}, pulsePerUnit{}, maxVelocityPPS{};
    std::array<double, 4U> axisAccTime{}, axisDecTime{};
    std::array<double, 4U> absoluteTargetMCS{};
    double rotaryModulo = 0.0;
    bool rotaryShortestPath = false, absolute = false;
};

struct NCXYZCFeedLineValue
{
    std::array<double, 8U> startMCS{}, endMCS{}, startPulse{}, endPulse{};
    std::uint32_t axisMask = 0U;
    double feedMMMin = 0.0;
    std::array<double, 4U> deltaNative{}, pulsePerUnit{}, axisVelocityPPS{};
    double linearLengthMM = 0.0, nominalSeconds = 0.0;
    double lengthPulse = 0.0, velocityPPS = 0.0;
    double rotaryFeedDegMin = 0.0, accTime = 0.0, decTime = 0.0;
    // Raw WCS is bound against frozen source offsets by the producer.
    std::array<double, 4U> absoluteTargetWCS{}, absoluteTargetMCS{};
    double rotaryModulo = 0.0;
    bool rotaryShortestPath = false, absolute = false;
    bool valid = false;

    void Clear() noexcept
    {
        startMCS.fill(0.0); endMCS.fill(0.0);
        startPulse.fill(0.0); endPulse.fill(0.0);
        axisMask = 0U; feedMMMin = 0.0;
        deltaNative.fill(0.0); pulsePerUnit.fill(0.0); axisVelocityPPS.fill(0.0);
        linearLengthMM = nominalSeconds = lengthPulse = velocityPPS = 0.0;
        rotaryFeedDegMin = accTime = decTime = 0.0;
        absoluteTargetWCS.fill(0.0); absoluteTargetMCS.fill(0.0);
        rotaryModulo = 0.0; rotaryShortestPath = false; absolute = false;
        valid = false;
    }
};

namespace NCXYZCFeedDetail
{
    inline bool SameBits(double a, double b) noexcept
    {
        std::uint64_t aa = 0ULL, bb = 0ULL;
        std::memcpy(&aa, &a, sizeof(aa)); std::memcpy(&bb, &b, sizeof(bb));
        return aa == bb;
    }
    inline double Maximum(double a, double b) noexcept { return a > b ? a : b; }

    // Scale finite positive operands before products/quotients. A representable
    // common duration must not fail merely because the naive 60*length overflows.
    inline double ProductRatio(double a, double b, double c) noexcept
    {
        int ea = 0, eb = 0, ec = 0;
        const double ma = std::frexp(a, &ea), mb = std::frexp(b, &eb);
        const double mc = std::frexp(c, &ec);
        return std::ldexp((ma * mb) / mc, ea + eb - ec);
    }

    // Local binary64 subtraction/product roundoff, bounded independently by one
    // part per billion of this axis's authored move. Never a servo window.
    inline bool DeltaAgrees(double authored, double actual, double start,
        double end, double pulsePerUnit) noexcept
    {
        const double expected = std::fabs(authored) * pulsePerUnit;
        actual = std::fabs(actual);
        if (!std::isfinite(expected) || expected <= 0.0 || actual <= 0.0) return false;
        if (expected == actual) return true;
        const double error = std::fabs(expected / actual - 1.0);
        if (!std::isfinite(error) || error > 1.0e-9) return false;
        const double scale = Maximum(Maximum(std::fabs(start), std::fabs(end)),
            Maximum(expected, actual));
        const double budget = 64.0 * (std::numeric_limits<double>::epsilon)() * (scale / actual);
        return error <= budget;
    }
    // G90's effective sweep is itself a represented native end-start. The
    // native rotary frame may be modulo-reported near a full turn while physical
    // pulses are near zero. Both independent subtractions therefore contribute
    // roundoff; budgeting only the pulse frame falsely rejects valid motion.
    // The 1e-9 relative ceiling is identical to G91 and never a servo window.
    inline bool AbsoluteDeltaAgrees(double sweep, double pulseDelta,
        double nativeStart, double nativeEnd, double pulseStart, double pulseEnd,
        double pulsePerUnit) noexcept
    {
        const double expected = std::fabs(sweep) * pulsePerUnit;
        const double actual = std::fabs(pulseDelta);
        if (!std::isfinite(expected) || expected <= 0.0 || actual <= 0.0) return false;
        if (expected == actual) return true;
        const double error = std::fabs(expected / actual - 1.0);
        if (!std::isfinite(error) || error > 1.0e-9) return false;
        const double nativeScale = Maximum(Maximum(std::fabs(nativeStart),
            std::fabs(nativeEnd)), std::fabs(sweep));
        const double pulseScale = Maximum(Maximum(std::fabs(pulseStart),
            std::fabs(pulseEnd)), Maximum(expected, actual));
        const double budget = 64.0 * (std::numeric_limits<double>::epsilon)() *
            (nativeScale / std::fabs(sweep) + pulseScale / actual);
        return error <= budget;
    }

    inline bool ValidTime(double value) noexcept
    {
        return std::isfinite(value) && value >= 0.0;
    }
    inline double EffectiveTime(double value) noexcept { return value < 0.001 ? 0.2 : value; }
}

// Disjoint caller-owned input/output; every rejection clears the entire value.
// No point form, sparse-axis form, or allocation is provided.
inline NCXYZCFeedLineCode BuildNCXYZCFeedLine(
    const NCXYZCFeedLineInput& input, NCXYZCFeedLineValue& output) noexcept
{
    using Code = NCXYZCFeedLineCode;
    output.Clear();
    const NCAxisIdentitySnapshot& identity = input.axisIdentity;
    if (!IsNCAxisIdentitySnapshotValid(identity)) return Code::INVALID_AXIS_IDENTITY;
    for (unsigned axis = 0U; axis < 4U; ++axis)
        if (identity.exists[axis] != 1U || identity.axisType[axis] != (axis == 3U ? 1U : 0U) ||
            identity.nativeUnit[axis] != (axis == 3U ? 2U : 1U) || identity.address[axis] != "XYZC"[axis])
            return Code::INVALID_AXIS_IDENTITY;
    if (input.axisMask != 0x0fU) return Code::INVALID_AXIS_MASK;
    if (!std::isfinite(input.feedMMMin) || input.feedMMMin <= 0.0 || input.feedMMMin > 100.0)
        return Code::INVALID_FEED;
    for (unsigned axis = 0U; axis < 4U; ++axis)
    {
        if (!std::isfinite(input.deltaNative[axis]) || input.deltaNative[axis] == 0.0)
            return Code::INVALID_AUTHORED_DELTA;
        if (!std::isfinite(input.pulsePerUnit[axis]) || input.pulsePerUnit[axis] <= 0.0 ||
            !std::isfinite(input.maxVelocityPPS[axis]) || input.maxVelocityPPS[axis] <= 0.0 ||
            !NCXYZCFeedDetail::ValidTime(input.axisAccTime[axis]) ||
            !NCXYZCFeedDetail::ValidTime(input.axisDecTime[axis])) return Code::INVALID_AXIS_CONFIGURATION;
    }
    NCXYZCAbsoluteFeedTarget resolved{};
    if (input.absolute)
    {
        std::array<double, 4U> startMCS{}, startPulse{};
        for (unsigned axis = 0U; axis < 4U; ++axis)
        { startMCS[axis] = input.startMCS[axis]; startPulse[axis] = input.startPulse[axis]; }
        if (!TryResolveNCXYZCAbsoluteFeedTarget(startMCS, startPulse,
            input.absoluteTargetMCS, input.pulsePerUnit, input.rotaryShortestPath,
            input.rotaryModulo, resolved)) return Code::INVALID_ABSOLUTE_TARGET;
        for (unsigned axis = 0U; axis < 4U; ++axis)
            if (!NCXYZCFeedDetail::SameBits(input.deltaNative[axis], resolved.deltaNative[axis]))
                return Code::INVALID_ABSOLUTE_TARGET;
    }
    else
    {
        // G91 cannot carry stale G90 provenance or a rotary selection policy.
        for (unsigned axis = 0U; axis < 4U; ++axis)
            if (!NCXYZCFeedDetail::SameBits(input.absoluteTargetMCS[axis], 0.0))
                return Code::INVALID_ABSOLUTE_TARGET;
        if (!NCXYZCFeedDetail::SameBits(input.rotaryModulo, 0.0) ||
            input.rotaryShortestPath) return Code::INVALID_ABSOLUTE_TARGET;
    }
    std::array<double, 4U> deltaPulse{};
    for (unsigned axis = 0U; axis < 8U; ++axis)
    {
        if (!std::isfinite(input.startMCS[axis]) || !std::isfinite(input.endMCS[axis]) ||
            !std::isfinite(input.startPulse[axis]) || !std::isfinite(input.endPulse[axis]))
            return Code::NONFINITE_COORDINATE;
        if (axis >= 4U)
        {
            if (!NCXYZCFeedDetail::SameBits(input.startMCS[axis], input.endMCS[axis]) ||
                !NCXYZCFeedDetail::SameBits(input.startPulse[axis], input.endPulse[axis]))
                return Code::OUTSIDE_AXIS_CHANGED;
            continue;
        }
        const double authored = input.deltaNative[axis], ppu = input.pulsePerUnit[axis];
        const double authoredPulse = authored * ppu;
        const double expectedNative = input.absolute ? resolved.endMCS[axis] : input.startMCS[axis] + authored;
        const double expectedPulse = input.absolute ? resolved.endPulse[axis] : input.startPulse[axis] + authoredPulse;
        if (!std::isfinite(authoredPulse) || !std::isfinite(expectedNative) || !std::isfinite(expectedPulse))
            return Code::NONFINITE_GEOMETRY;
        if (!NCXYZCFeedDetail::SameBits(input.endMCS[axis], expectedNative) ||
            !NCXYZCFeedDetail::SameBits(input.endPulse[axis], expectedPulse)) return Code::ENDPOINT_MISMATCH;
        const double deltaNative = input.endMCS[axis] - input.startMCS[axis];
        deltaPulse[axis] = input.endPulse[axis] - input.startPulse[axis];
        if (!std::isfinite(deltaNative) || !std::isfinite(deltaPulse[axis])) return Code::NONFINITE_GEOMETRY;
        if (deltaNative == 0.0 || std::fabs(deltaPulse[axis]) < 1.0e-5)
            return Code::BELOW_CONSUMER_RESOLUTION;
        if ((authored < 0.0) != (deltaNative < 0.0) || (authored < 0.0) != (deltaPulse[axis] < 0.0))
            return Code::DIRECTION_MISMATCH;
        if (!NCXYZCFeedDetail::DeltaAgrees(authored, deltaNative, input.startMCS[axis], input.endMCS[axis], 1.0) ||
            !(input.absolute ?
                NCXYZCFeedDetail::AbsoluteDeltaAgrees(authored, deltaPulse[axis],
                    input.startMCS[axis], input.endMCS[axis],
                    input.startPulse[axis], input.endPulse[axis], ppu) :
                NCXYZCFeedDetail::DeltaAgrees(authored, deltaPulse[axis], input.startPulse[axis], input.endPulse[axis], ppu)))
            return Code::DELTA_CONVERSION_MISMATCH;
    }
    const double linearLength = std::hypot(std::hypot(input.deltaNative[0], input.deltaNative[1]), input.deltaNative[2]);
    if (!std::isfinite(linearLength) || linearLength <= 0.0) return Code::NONFINITE_GEOMETRY;
    const double seconds = NCXYZCFeedDetail::ProductRatio(linearLength, 60.0, input.feedMMMin);
    if (!std::isfinite(seconds) || seconds <= 0.0) return Code::NONFINITE_GEOMETRY;
    // Order is part of the canonical packet: match LoadNext's iterative norm,
    // rather than regrouping the same sum into a different binary64 result.
    double length = 0.0;
    for (unsigned axis = 0U; axis < 4U; ++axis) length = std::hypot(length, deltaPulse[axis]);
    if (!std::isfinite(length) || length <= 0.0) return Code::NONFINITE_GEOMETRY;
    const double velocity = NCXYZCFeedDetail::ProductRatio(length, 1.0, seconds);
    const double rotaryFeed = NCXYZCFeedDetail::ProductRatio(std::fabs(input.deltaNative[3]), 60.0, seconds);
    if (!std::isfinite(velocity) || velocity < 1.0 || !std::isfinite(rotaryFeed) || rotaryFeed <= 0.0)
        return Code::INVALID_VELOCITY;
    if (rotaryFeed > 100.0) return Code::ROTARY_FEED_LIMIT;
    std::array<double, 4U> projectedVelocity{};
    for (unsigned axis = 0U; axis < 4U; ++axis)
    {
        const double directVelocity = NCXYZCFeedDetail::ProductRatio(std::fabs(deltaPulse[axis]), 1.0, seconds);
        const double ratio = deltaPulse[axis] / length;
        projectedVelocity[axis] = std::fabs(velocity * ratio);
        if (!std::isfinite(directVelocity) || directVelocity <= 0.0 ||
            !std::isfinite(ratio) || ratio == 0.0 ||
            !std::isfinite(projectedVelocity[axis]) || projectedVelocity[axis] <= 0.0)
            return Code::INVALID_VELOCITY;
        if (directVelocity > input.maxVelocityPPS[axis] || projectedVelocity[axis] > input.maxVelocityPPS[axis])
            return Code::AXIS_VELOCITY_LIMIT;
    }
    const double mappedRotaryFeed = NCXYZCFeedDetail::ProductRatio(projectedVelocity[3], 60.0, input.pulsePerUnit[3]);
    if (!std::isfinite(mappedRotaryFeed) || mappedRotaryFeed <= 0.0) return Code::INVALID_VELOCITY;
    const double eps64 = 64.0 * (std::numeric_limits<double>::epsilon)();
    const double pulseScale = NCXYZCFeedDetail::Maximum(std::fabs(deltaPulse[3]),
        NCXYZCFeedDetail::Maximum(std::fabs(input.startPulse[3]), std::fabs(input.endPulse[3])));
    double subtractionBudget = eps64 * (pulseScale / std::fabs(deltaPulse[3]));
    if (input.absolute)
    {
        const double nativeScale = NCXYZCFeedDetail::Maximum(std::fabs(input.deltaNative[3]),
            NCXYZCFeedDetail::Maximum(std::fabs(input.startMCS[3]), std::fabs(input.endMCS[3])));
        subtractionBudget += eps64 * (nativeScale / std::fabs(input.deltaNative[3]));
    }
    const double relativeBudget = eps64 + (subtractionBudget < 1.0e-9 ? subtractionBudget : 1.0e-9);
    // The authored rate test is strict. Only independently bounded pulse
    // subtraction/projection roundoff is allowed here; no velocity is clamped.
    if (mappedRotaryFeed > 100.0 && mappedRotaryFeed - 100.0 > 100.0 * relativeBudget)
        return Code::ROTARY_FEED_LIMIT;
    double accTime = 0.0, decTime = 0.0;
    for (unsigned axis = 0U; axis < 4U; ++axis)
    {
        accTime = NCXYZCFeedDetail::Maximum(accTime, NCXYZCFeedDetail::EffectiveTime(input.axisAccTime[axis]));
        decTime = NCXYZCFeedDetail::Maximum(decTime, NCXYZCFeedDetail::EffectiveTime(input.axisDecTime[axis]));
    }
    const double acceleration = velocity / accTime, deceleration = velocity / decTime;
    if (!std::isfinite(acceleration) || acceleration <= 0.0 || !std::isfinite(deceleration) || deceleration <= 0.0)
        return Code::INVALID_VELOCITY;
    output.startMCS = input.startMCS; output.endMCS = input.endMCS;
    output.startPulse = input.startPulse; output.endPulse = input.endPulse;
    output.axisMask = input.axisMask; output.feedMMMin = input.feedMMMin;
    output.deltaNative = input.deltaNative; output.pulsePerUnit = input.pulsePerUnit;
    output.axisVelocityPPS = projectedVelocity;
    output.linearLengthMM = linearLength; output.nominalSeconds = seconds;
    output.lengthPulse = length; output.velocityPPS = velocity;
    output.rotaryFeedDegMin = rotaryFeed; output.accTime = accTime; output.decTime = decTime;
    if (input.absolute)
    {
        output.absolute = true;
        output.absoluteTargetMCS = input.absoluteTargetMCS;
        output.rotaryModulo = input.rotaryModulo;
        output.rotaryShortestPath = input.rotaryShortestPath;
    }
    output.valid = true;
    return Code::BUILT_LINE;
}

static_assert(std::numeric_limits<double>::is_iec559 && sizeof(double) == 8U,
    "BASE75 XYZC feed requires IEEE binary64.");
static_assert(sizeof(NCXYZCFeedLineValue) <= 512U &&
    std::is_standard_layout<NCXYZCFeedLineValue>::value &&
    std::is_trivially_copyable<NCXYZCFeedLineValue>::value,
    "BASE75 XYZC geometry must remain a bounded value.");
