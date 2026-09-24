#pragma once

#include "NCAxisIdentitySnapshot.h"
#include "NCZCAbsoluteFeedTarget.h"
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <type_traits>

// BASE70/71: an immutable G90/G91 Z+C line in separate native units. F is Z mm/min,
// never the Euclidean norm of millimetres and degrees. Both selected axes use
// T = 60*abs(effective Z)/F; pulse-space length and scalar speed only parameterize
// that common time. The consumer applies one shared acceleration/deceleration
// profile, so T is the nominal constant-feed time, not total elapsed run time.
// Physical slots 2/3 are respectively linear Z and positional rotary C. C keeps
// the full signed authored increment in G91. G90 independently verifies one
// canonical absolute resolution against its raw targets and rotary policy.
// Source, ownership, travel, exact-stop and capture permission remain callers'
// responsibility. Building geometry grants no Motion authority.
#if defined(__FAST_MATH__) || defined(_M_FP_FAST)
#error BASE70_ZC_feed_requires_precise_floating_point_semantics
#endif
#if defined(__FINITE_MATH_ONLY__) && (__FINITE_MATH_ONLY__ > 0)
#error BASE70_ZC_feed_requires_nonfinite_checks
#endif

enum class NCZCFeedLineCode : std::uint8_t
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

struct NCZCFeedLineInput
{
    std::array<double, 8U> startMCS{}, endMCS{}, startPulse{}, endPulse{};
    NCAxisIdentitySnapshot axisIdentity{};
    std::uint32_t axisMask = 0U;
    double feedMMMin = 0.0;
    double deltaZMM = 0.0, deltaCDeg = 0.0; // G91 authored / G90 resolved signed sweeps.
    double pulsePerMM = 0.0, pulsePerDegree = 0.0;
    double maxLinearVelocityPPS = 0.0, maxRotaryVelocityPPS = 0.0;
    double linearAccTime = 0.0, rotaryAccTime = 0.0;
    double linearDecTime = 0.0, rotaryDecTime = 0.0;
    double absoluteTargetZMCS = 0.0, absoluteTargetCMCS = 0.0, rotaryModulo = 0.0;
    bool absolute = false, rotaryShortestPath = false;
};

struct NCZCFeedLineValue
{
    std::array<double, 8U> startMCS{}, endMCS{}, startPulse{}, endPulse{};
    std::uint32_t axisMask = 0U;
    double feedMMMin = 0.0;
    double pulsePerMM = 0.0, pulsePerDegree = 0.0;
    double deltaZMM = 0.0, deltaCDeg = 0.0; // G91 authored / G90 resolved signed sweeps.
    double nominalSeconds = 0.0;
    double lengthPulse = 0.0, velocityPPS = 0.0;
    double linearVelocityPPS = 0.0, rotaryVelocityPPS = 0.0; // Actual scalar*ratio projection.
    double rotaryFeedDegMin = 0.0;
    double accTime = 0.0, decTime = 0.0;
    // Absolute WCS provenance is bound by the producer after geometry builds;
    // callers independently prove it against the frozen translation offsets.
    double absoluteTargetZWCS = 0.0, absoluteTargetCWCS = 0.0;
    double absoluteTargetZMCS = 0.0, absoluteTargetCMCS = 0.0, rotaryModulo = 0.0;
    bool absolute = false, rotaryShortestPath = false;
    bool valid = false;

    void Clear() noexcept
    {
        startMCS.fill(0.0); endMCS.fill(0.0);
        startPulse.fill(0.0); endPulse.fill(0.0);
        axisMask = 0U;
        feedMMMin = pulsePerMM = pulsePerDegree = deltaZMM = deltaCDeg = 0.0;
        nominalSeconds = lengthPulse = velocityPPS = 0.0;
        linearVelocityPPS = rotaryVelocityPPS = rotaryFeedDegMin = 0.0;
        accTime = decTime = 0.0;
        absoluteTargetZWCS = absoluteTargetCWCS = 0.0;
        absoluteTargetZMCS = absoluteTargetCMCS = rotaryModulo = 0.0;
        absolute = rotaryShortestPath = false;
        valid = false;
    }
};

namespace NCZCFeedDetail
{
    inline bool SameBits(double a, double b) noexcept
    {
        std::uint64_t aa = 0ULL, bb = 0ULL;
        std::memcpy(&aa, &a, sizeof(aa));
        std::memcpy(&bb, &b, sizeof(bb));
        return aa == bb;
    }

    inline double Maximum(double a, double b) noexcept { return a > b ? a : b; }

    // Positive finite operands. Scaling before products/quotients prevents an
    // intermediate overflow/underflow when the final result is representable.
    inline double ProductRatio(double a, double b, double c) noexcept
    {
        int ea = 0, eb = 0, ec = 0;
        const double ma = std::frexp(a, &ea), mb = std::frexp(b, &eb);
        const double mc = std::frexp(c, &ec);
        return std::ldexp((ma * mb) / mc, ea + eb - ec);
    }

    // Rounding allowance, not a servo in-position tolerance. The independent
    // relative ceiling rejects coordinates too coarse to preserve the move.
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
    // native C frame may be modulo-reported near 360 degrees while physical
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

// Disjoint caller-owned input/output. Every rejection clears the complete
// output; there is intentionally no point/no-op form of a mixed Z+C command.
inline NCZCFeedLineCode BuildNCZCFeedLine(
    const NCZCFeedLineInput& input, NCZCFeedLineValue& output) noexcept
{
    using Code = NCZCFeedLineCode;
    output.Clear();
    const NCAxisIdentitySnapshot& identity = input.axisIdentity;
    if (!IsNCAxisIdentitySnapshotValid(identity) ||
        identity.exists[2] != 1U || identity.axisType[2] != 0U ||
        identity.nativeUnit[2] != 1U || identity.address[2] != 'Z' ||
        identity.exists[3] != 1U || identity.axisType[3] != 1U ||
        identity.nativeUnit[3] != 2U || identity.address[3] != 'C')
        return Code::INVALID_AXIS_IDENTITY;
    if (input.axisMask != 0x0cU) return Code::INVALID_AXIS_MASK;
    if (!std::isfinite(input.feedMMMin) || input.feedMMMin <= 0.0 || input.feedMMMin > 100.0)
        return Code::INVALID_FEED;
    if (!std::isfinite(input.deltaZMM) || input.deltaZMM == 0.0 ||
        !std::isfinite(input.deltaCDeg) || input.deltaCDeg == 0.0)
        return Code::INVALID_AUTHORED_DELTA;
    if (!std::isfinite(input.pulsePerMM) || input.pulsePerMM <= 0.0 ||
        !std::isfinite(input.pulsePerDegree) || input.pulsePerDegree <= 0.0 ||
        !std::isfinite(input.maxLinearVelocityPPS) || input.maxLinearVelocityPPS <= 0.0 ||
        !std::isfinite(input.maxRotaryVelocityPPS) || input.maxRotaryVelocityPPS <= 0.0 ||
        !NCZCFeedDetail::ValidTime(input.linearAccTime) ||
        !NCZCFeedDetail::ValidTime(input.rotaryAccTime) ||
        !NCZCFeedDetail::ValidTime(input.linearDecTime) ||
        !NCZCFeedDetail::ValidTime(input.rotaryDecTime))
        return Code::INVALID_AXIS_CONFIGURATION;
    NCZCAbsoluteFeedTarget resolved{};
    if (input.absolute &&
        (!TryResolveNCZCAbsoluteFeedTarget(input.startMCS[2], input.startPulse[2],
            input.startMCS[3], input.startPulse[3], input.absoluteTargetZMCS,
            input.absoluteTargetCMCS, input.pulsePerMM, input.pulsePerDegree,
            input.rotaryShortestPath, input.rotaryModulo, resolved) ||
         !NCZCFeedDetail::SameBits(input.deltaZMM, resolved.deltaZMM) ||
         !NCZCFeedDetail::SameBits(input.deltaCDeg, resolved.deltaCDeg)))
        return Code::INVALID_ABSOLUTE_TARGET;
    for (unsigned axis = 0U; axis < 8U; ++axis)
    {
        if (!std::isfinite(input.startMCS[axis]) || !std::isfinite(input.endMCS[axis]) ||
            !std::isfinite(input.startPulse[axis]) || !std::isfinite(input.endPulse[axis]))
            return Code::NONFINITE_COORDINATE;
        if (axis != 2U && axis != 3U)
        {
            if (!NCZCFeedDetail::SameBits(input.startMCS[axis], input.endMCS[axis]) ||
                !NCZCFeedDetail::SameBits(input.startPulse[axis], input.endPulse[axis]))
                return Code::OUTSIDE_AXIS_CHANGED;
            continue;
        }
        const double authored = axis == 2U ? input.deltaZMM : input.deltaCDeg;
        const double ppu = axis == 2U ? input.pulsePerMM : input.pulsePerDegree;
        const double authoredPulse = authored * ppu;
        const double expectedNative = input.absolute ?
            (axis == 2U ? resolved.endZMCS : resolved.endCMCS) : input.startMCS[axis] + authored;
        const double expectedPulse = input.absolute ?
            (axis == 2U ? resolved.endZPulse : resolved.endCPulse) : input.startPulse[axis] + authoredPulse;
        if (!std::isfinite(authoredPulse) || !std::isfinite(expectedNative) ||
            !std::isfinite(expectedPulse)) return Code::NONFINITE_GEOMETRY;
        if (!NCZCFeedDetail::SameBits(input.endMCS[axis], expectedNative) ||
            !NCZCFeedDetail::SameBits(input.endPulse[axis], expectedPulse))
            return Code::ENDPOINT_MISMATCH;
        const double deltaNative = input.endMCS[axis] - input.startMCS[axis];
        const double deltaPulse = input.endPulse[axis] - input.startPulse[axis];
        if (!std::isfinite(deltaNative) || !std::isfinite(deltaPulse)) return Code::NONFINITE_GEOMETRY;
        if (deltaNative == 0.0 || std::fabs(deltaPulse) < 1.0e-5)
            return Code::BELOW_CONSUMER_RESOLUTION;
        if ((authored < 0.0) != (deltaNative < 0.0) ||
            (authored < 0.0) != (deltaPulse < 0.0)) return Code::DIRECTION_MISMATCH;
        if (!NCZCFeedDetail::DeltaAgrees(authored, deltaNative,
                input.startMCS[axis], input.endMCS[axis], 1.0) ||
            !(input.absolute ?
                NCZCFeedDetail::AbsoluteDeltaAgrees(authored, deltaPulse,
                    input.startMCS[axis], input.endMCS[axis],
                    input.startPulse[axis], input.endPulse[axis], ppu) :
                NCZCFeedDetail::DeltaAgrees(authored, deltaPulse,
                    input.startPulse[axis], input.endPulse[axis], ppu)))
            return Code::DELTA_CONVERSION_MISMATCH;
    }
    const double deltaZPulse = input.endPulse[2] - input.startPulse[2];
    const double deltaCPulse = input.endPulse[3] - input.startPulse[3];
    const double seconds = NCZCFeedDetail::ProductRatio(std::fabs(input.deltaZMM), 60.0, input.feedMMMin);
    if (!std::isfinite(seconds) || seconds <= 0.0) return Code::NONFINITE_GEOMETRY;
    const double length = std::hypot(deltaZPulse, deltaCPulse);
    if (!std::isfinite(length) || length <= 0.0) return Code::NONFINITE_GEOMETRY;
    const double velocity = NCZCFeedDetail::ProductRatio(length, 1.0, seconds);
    const double linearVelocity = NCZCFeedDetail::ProductRatio(std::fabs(deltaZPulse), 1.0, seconds);
    const double rotaryVelocity = NCZCFeedDetail::ProductRatio(std::fabs(deltaCPulse), 1.0, seconds);
    const double rotaryFeed = NCZCFeedDetail::ProductRatio(std::fabs(input.deltaCDeg), 60.0, seconds);
    if (!std::isfinite(velocity) || velocity < 1.0 ||
        !std::isfinite(linearVelocity) || linearVelocity <= 0.0 ||
        !std::isfinite(rotaryVelocity) || rotaryVelocity <= 0.0 ||
        !std::isfinite(rotaryFeed) || rotaryFeed <= 0.0) return Code::INVALID_VELOCITY;
    if (linearVelocity > input.maxLinearVelocityPPS ||
        rotaryVelocity > input.maxRotaryVelocityPPS) return Code::AXIS_VELOCITY_LIMIT;
    if (rotaryFeed > 100.0) return Code::ROTARY_FEED_LIMIT;
    // Reproduce the runtime's normalized interpolation, independently of the
    // equivalent delta/T equations. Extreme scale ratios can otherwise vanish,
    // and a multiply can round above a physical speed limit at its boundary.
    const double zRatio = deltaZPulse / length, cRatio = deltaCPulse / length;
    const double mappedLinearVelocity = std::fabs(velocity * zRatio);
    const double mappedRotaryVelocity = std::fabs(velocity * cRatio);
    if (!std::isfinite(zRatio) || zRatio == 0.0 ||
        !std::isfinite(cRatio) || cRatio == 0.0 ||
        !std::isfinite(mappedLinearVelocity) || mappedLinearVelocity <= 0.0 ||
        !std::isfinite(mappedRotaryVelocity) || mappedRotaryVelocity <= 0.0)
        return Code::INVALID_VELOCITY;
    if (mappedLinearVelocity > input.maxLinearVelocityPPS ||
        mappedRotaryVelocity > input.maxRotaryVelocityPPS) return Code::AXIS_VELOCITY_LIMIT;
    const double mappedRotaryFeed = NCZCFeedDetail::ProductRatio(mappedRotaryVelocity,
        60.0, input.pulsePerDegree);
    if (!std::isfinite(mappedRotaryFeed) || mappedRotaryFeed <= 0.0)
        return Code::INVALID_VELOCITY;
    const double eps64 = 64.0 * (std::numeric_limits<double>::epsilon)();
    const double pulseScale = NCZCFeedDetail::Maximum(std::fabs(deltaCPulse),
        NCZCFeedDetail::Maximum(std::fabs(input.startPulse[3]), std::fabs(input.endPulse[3])));
    const double nativeCScale = NCZCFeedDetail::Maximum(std::fabs(input.startMCS[3]),
        std::fabs(input.endMCS[3]));
    const double subtractionBudget = eps64 * (pulseScale / std::fabs(deltaCPulse)) +
        (input.absolute ? eps64 * (nativeCScale / std::fabs(input.deltaCDeg)) : 0.0);
    const double relativeBudget = eps64 + (subtractionBudget < 1.0e-9 ? subtractionBudget : 1.0e-9);
    // The authored degree/min limit above is exact. Only binary64 endpoint
    // subtraction and scalar projection rounding are allowed here, bounded by
    // the same 1e-9 relative conversion ceiling; no speed is silently clamped.
    if (mappedRotaryFeed > 100.0 && mappedRotaryFeed - 100.0 > 100.0 * relativeBudget)
        return Code::ROTARY_FEED_LIMIT;
    const double accTime = NCZCFeedDetail::Maximum(NCZCFeedDetail::EffectiveTime(input.linearAccTime),
        NCZCFeedDetail::EffectiveTime(input.rotaryAccTime));
    const double decTime = NCZCFeedDetail::Maximum(NCZCFeedDetail::EffectiveTime(input.linearDecTime),
        NCZCFeedDetail::EffectiveTime(input.rotaryDecTime));
    const double acceleration = velocity / accTime, deceleration = velocity / decTime;
    if (!std::isfinite(acceleration) || acceleration <= 0.0 ||
        !std::isfinite(deceleration) || deceleration <= 0.0) return Code::INVALID_VELOCITY;
    output.startMCS = input.startMCS; output.endMCS = input.endMCS;
    output.startPulse = input.startPulse; output.endPulse = input.endPulse;
    output.axisMask = input.axisMask; output.feedMMMin = input.feedMMMin;
    output.pulsePerMM = input.pulsePerMM; output.pulsePerDegree = input.pulsePerDegree;
    output.deltaZMM = input.deltaZMM; output.deltaCDeg = input.deltaCDeg;
    output.nominalSeconds = seconds; output.lengthPulse = length; output.velocityPPS = velocity;
    output.linearVelocityPPS = mappedLinearVelocity; output.rotaryVelocityPPS = mappedRotaryVelocity;
    output.rotaryFeedDegMin = rotaryFeed;
    output.accTime = accTime; output.decTime = decTime;
    if (input.absolute)
    {
        output.absolute = true;
        output.absoluteTargetZMCS = input.absoluteTargetZMCS;
        output.absoluteTargetCMCS = input.absoluteTargetCMCS;
        output.rotaryModulo = input.rotaryModulo;
        output.rotaryShortestPath = input.rotaryShortestPath;
    }
    output.valid = true;
    return Code::BUILT_LINE;
}

static_assert(std::numeric_limits<double>::is_iec559 && sizeof(double) == 8U,
    "BASE70 ZC feed requires IEEE binary64.");
static_assert(sizeof(NCZCFeedLineValue) <= 432U &&
    std::is_standard_layout<NCZCFeedLineValue>::value &&
    std::is_trivially_copyable<NCZCFeedLineValue>::value,
    "BASE70 ZC geometry must remain a bounded value.");
