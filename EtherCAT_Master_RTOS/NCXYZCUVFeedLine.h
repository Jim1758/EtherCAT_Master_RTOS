#pragma once
#include "EDM149VoltageMotion.h"

#include "NCAxisIdentitySnapshot.h"
#include "NCXYZCUVAbsoluteFeedTarget.h"
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <type_traits>

// EDM384: optional mixed-native G90/G91 XYZ+C+U+V immutable geometry. F is
// the configured linear-axis path rate in mm/min; degrees never enter length.
// Fixed present and absent axes are legal. One pulse-space scalar and one profile
// give every selected axis the same progress. Building this value grants no
// source, ownership, travel, capture, replay, or live Motion permission.
// G91 retains full authored C/U/V increments. G90 independently verifies one
// canonical absolute resolution per axis from raw MCS targets and rotary policy.
#if defined(__FAST_MATH__) || defined(_M_FP_FAST)
#error BASE74_XYZCUV_feed_requires_precise_floating_point_semantics
#endif
#if defined(__FINITE_MATH_ONLY__) && (__FINITE_MATH_ONLY__ > 0)
#error BASE74_XYZCUV_feed_requires_nonfinite_checks
#endif

enum class NCXYZCUVFeedLineCode : std::uint8_t
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

struct NCXYZCUVFeedLineInput
{
    std::array<double, 8U> startMCS{}, endMCS{}, startPulse{}, endPulse{};
    NCAxisIdentitySnapshot axisIdentity{};
    std::uint32_t axisMask = 0U;
    double feedMMMin = 0.0;
    std::array<double, 6U> deltaNative{}, pulsePerUnit{}, maxVelocityPPS{};
    std::array<double, 6U> axisAccTime{}, axisDecTime{};
    std::array<double, 6U> absoluteTargetMCS{};
    std::array<double, 3U> rotaryModulo{};
    std::array<bool, 3U> rotaryShortestPath{};
    bool absolute = false;
    bool zOnlyCommissioning = false;
    bool xyzOnlyCommissioning = false;
    bool sparseCommissioning = false;
    // 0xffffffff retains the original all-present-addresses contract. New
    // sources freeze their actual authored native axis words independently.
    std::uint32_t authoredAxisMask=0xffffffffU;
};

struct NCXYZCUVFeedLineValue
{
    std::array<double, 8U> startMCS{}, endMCS{}, startPulse{}, endPulse{};
    std::uint32_t axisMask = 0U;
    double feedMMMin = 0.0;
    std::array<double, 6U> deltaNative{}, pulsePerUnit{}, axisVelocityPPS{};
    double linearLengthMM = 0.0, nominalSeconds = 0.0;
    double lengthPulse = 0.0, velocityPPS = 0.0;
    std::array<double, 3U> rotaryFeedDegMin{}; // C, U, V respectively.
    double accTime = 0.0, decTime = 0.0;
    // WCS provenance is bound by the producer against frozen source offsets.
    std::array<double, 6U> absoluteTargetWCS{}, absoluteTargetMCS{};
    std::array<double, 3U> rotaryModulo{};
    std::array<bool, 3U> rotaryShortestPath{};
    bool absolute = false;
    bool zOnlyCommissioning = false;
    bool xyzOnlyCommissioning = false;
    bool sparseCommissioning = false;
    bool valid = false;
    std::uint32_t authoredAxisMask=0U;

    void Clear() noexcept
    {
        startMCS.fill(0.0); endMCS.fill(0.0);
        startPulse.fill(0.0); endPulse.fill(0.0);
        axisMask = 0U; feedMMMin = 0.0;
        deltaNative.fill(0.0); pulsePerUnit.fill(0.0); axisVelocityPPS.fill(0.0);
        linearLengthMM = nominalSeconds = lengthPulse = velocityPPS = 0.0;
        rotaryFeedDegMin.fill(0.0); accTime = decTime = 0.0;
        absoluteTargetWCS.fill(0.0); absoluteTargetMCS.fill(0.0);
        rotaryModulo.fill(0.0); rotaryShortestPath.fill(false); absolute = false;
        zOnlyCommissioning = false;
        xyzOnlyCommissioning = false;
        sparseCommissioning = false;
        valid = false; authoredAxisMask=0U;
    }
};

namespace NCXYZCUVFeedDetail
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
// Sparse references are admitted only by a disjoint dry commissioning flag.
// Normal G90/G91 permits configured sparse/fixed axes; the named dry
// commissioning patterns retain their separate strict contracts. No allocation.
inline NCXYZCUVFeedLineCode BuildNCXYZCUVFeedLine(
    const NCXYZCUVFeedLineInput& input, NCXYZCUVFeedLineValue& output) noexcept
{
    using Code = NCXYZCUVFeedLineCode;
    output.Clear();
    const NCAxisIdentitySnapshot& identity = input.axisIdentity;
    if (!IsNCAxisIdentitySnapshotValid(identity)) return Code::INVALID_AXIS_IDENTITY;
    if(identity.exists[6]||identity.exists[7])return Code::INVALID_AXIS_IDENTITY;
    std::uint32_t presentMask=0U;
    const bool commissioning=input.zOnlyCommissioning||input.xyzOnlyCommissioning||input.sparseCommissioning;
    for(unsigned axis=0U;axis<6U;++axis){
        if(axis<3U&&(identity.exists[axis]!=1U||identity.axisType[axis]!=0U))return Code::INVALID_AXIS_IDENTITY;
        if(identity.exists[axis]){
            if(identity.address[axis]!="XYZCUV"[axis])return Code::INVALID_AXIS_IDENTITY;
            presentMask|=1U<<axis;
        }
        if(commissioning&&(identity.exists[axis]!=1U||identity.axisType[axis]!=(axis<3U?0U:1U)))
            return Code::INVALID_AXIS_IDENTITY;
    }
    if(input.axisMask!=presentMask)return Code::INVALID_AXIS_MASK;
    const std::uint32_t authoredMask=input.authoredAxisMask==0xffffffffU?presentMask:input.authoredAxisMask;
    if((authoredMask&~presentMask)!=0U)return Code::INVALID_AXIS_MASK;
    if ((static_cast<unsigned>(input.zOnlyCommissioning) + static_cast<unsigned>(input.xyzOnlyCommissioning) +
        static_cast<unsigned>(input.sparseCommissioning) > 1U) ||
        (input.sparseCommissioning && (input.absolute ||
            !EDM149::ValidSparseIncrement(input.deltaNative,input.feedMMMin))) ||
        (input.xyzOnlyCommissioning && (input.absolute ||
            !EDM149::ValidXYZIncrement(input.deltaNative,input.feedMMMin)))) return Code::INVALID_AUTHORED_DELTA;
    if (input.zOnlyCommissioning && (input.absolute ||
        !EDM149::ValidZIncrement(input.deltaNative,input.feedMMMin))) return Code::INVALID_AUTHORED_DELTA;
    if (!std::isfinite(input.feedMMMin) || input.feedMMMin <= 0.0 || input.feedMMMin > 100.0)
        return Code::INVALID_FEED;
    for (unsigned axis = 0U; axis < 6U; ++axis)
    {
        if(!std::isfinite(input.deltaNative[axis])||
           (!(authoredMask&(1U<<axis))&&input.deltaNative[axis]!=0.0))return Code::INVALID_AUTHORED_DELTA;
        if(!identity.exists[axis]){if(input.deltaNative[axis]!=0.0)return Code::OUTSIDE_AXIS_CHANGED;continue;}
        if (!std::isfinite(input.pulsePerUnit[axis]) || input.pulsePerUnit[axis] <= 0.0 ||
            !std::isfinite(input.maxVelocityPPS[axis]) || input.maxVelocityPPS[axis] <= 0.0 ||
            !NCXYZCUVFeedDetail::ValidTime(input.axisAccTime[axis]) ||
            !NCXYZCUVFeedDetail::ValidTime(input.axisDecTime[axis])) return Code::INVALID_AXIS_CONFIGURATION;
    }
    NCXYZCUVAbsoluteFeedTarget resolved{};
    if (input.absolute)
    {
        std::array<double, 6U> startMCS{}, startPulse{};
        for (unsigned axis = 0U; axis < 6U; ++axis)
        { startMCS[axis] = input.startMCS[axis]; startPulse[axis] = input.startPulse[axis]; }
        if (!TryResolveNCXYZCUVAbsoluteFeedTarget(identity,startMCS, startPulse,
            input.absoluteTargetMCS, input.pulsePerUnit, input.rotaryShortestPath,
            input.rotaryModulo, resolved,authoredMask)) return Code::INVALID_ABSOLUTE_TARGET;
        for (unsigned axis = 0U; axis < 6U; ++axis)
            if (!NCXYZCUVFeedDetail::SameBits(input.deltaNative[axis], resolved.deltaNative[axis]))
                return Code::INVALID_ABSOLUTE_TARGET;
    }
    else
    {
        // G91 cannot carry stale G90 provenance or a rotary selection policy.
        for (unsigned axis = 0U; axis < 6U; ++axis)
            if (!NCXYZCUVFeedDetail::SameBits(input.absoluteTargetMCS[axis], 0.0))
                return Code::INVALID_ABSOLUTE_TARGET;
        for (unsigned rotary = 0U; rotary < 3U; ++rotary)
            if (!NCXYZCUVFeedDetail::SameBits(input.rotaryModulo[rotary], 0.0) ||
                input.rotaryShortestPath[rotary]) return Code::INVALID_ABSOLUTE_TARGET;
    }
    std::array<double, 6U> deltaPulse{};
    for (unsigned axis = 0U; axis < 8U; ++axis)
    {
        if (!std::isfinite(input.startMCS[axis]) || !std::isfinite(input.endMCS[axis]) ||
            !std::isfinite(input.startPulse[axis]) || !std::isfinite(input.endPulse[axis]))
            return Code::NONFINITE_COORDINATE;
        if(axis>=6U||!identity.exists[axis])
        {
            if (!NCXYZCUVFeedDetail::SameBits(input.startMCS[axis], input.endMCS[axis]) ||
                !NCXYZCUVFeedDetail::SameBits(input.startPulse[axis], input.endPulse[axis]))
                return Code::OUTSIDE_AXIS_CHANGED;
            continue;
        }
        const double authored = input.deltaNative[axis], ppu = input.pulsePerUnit[axis];
        const double authoredPulse = authored * ppu;
        const bool staticAxis=input.deltaNative[axis]==0.0||
            EDM149::StaticAxis(input.zOnlyCommissioning,input.xyzOnlyCommissioning,axis);
        const double expectedNative = staticAxis ? input.startMCS[axis] :
            (input.absolute ? resolved.endMCS[axis] : input.startMCS[axis] + authored);
        const double expectedPulse = staticAxis ? input.startPulse[axis] :
            (input.absolute ? resolved.endPulse[axis] : input.startPulse[axis] + authoredPulse);
        if (!std::isfinite(authoredPulse) || !std::isfinite(expectedNative) || !std::isfinite(expectedPulse))
            return Code::NONFINITE_GEOMETRY;
        if (!NCXYZCUVFeedDetail::SameBits(input.endMCS[axis], expectedNative) ||
            !NCXYZCUVFeedDetail::SameBits(input.endPulse[axis], expectedPulse)) return Code::ENDPOINT_MISMATCH;
        const double deltaNative = input.endMCS[axis] - input.startMCS[axis];
        deltaPulse[axis] = input.endPulse[axis] - input.startPulse[axis];
        if (!std::isfinite(deltaNative) || !std::isfinite(deltaPulse[axis])) return Code::NONFINITE_GEOMETRY;
        if(staticAxis)
        {
            if (authored != 0.0 || deltaNative != 0.0 || deltaPulse[axis] != 0.0 ||
                !NCXYZCUVFeedDetail::SameBits(input.startMCS[axis],input.endMCS[axis]) ||
                !NCXYZCUVFeedDetail::SameBits(input.startPulse[axis],input.endPulse[axis])) return Code::OUTSIDE_AXIS_CHANGED;
            continue;
        }
        if (deltaNative == 0.0 || std::fabs(deltaPulse[axis]) < 1.0e-5)
            return Code::BELOW_CONSUMER_RESOLUTION;
        if ((authored < 0.0) != (deltaNative < 0.0) || (authored < 0.0) != (deltaPulse[axis] < 0.0))
            return Code::DIRECTION_MISMATCH;
        if (!NCXYZCUVFeedDetail::DeltaAgrees(authored, deltaNative, input.startMCS[axis], input.endMCS[axis], 1.0) ||
            !(input.absolute ?
                NCXYZCUVFeedDetail::AbsoluteDeltaAgrees(authored, deltaPulse[axis],
                    input.startMCS[axis], input.endMCS[axis],
                    input.startPulse[axis], input.endPulse[axis], ppu) :
                NCXYZCUVFeedDetail::DeltaAgrees(authored, deltaPulse[axis], input.startPulse[axis], input.endPulse[axis], ppu)))
            return Code::DELTA_CONVERSION_MISMATCH;
    }
    double linearLength=0.0;
    // Millimetres come only from configured linear axes. A degree is never
    // added to the linear metric, even when its address is C, U or V.
    for(unsigned axis=0U;axis<6U;++axis)if(identity.exists[axis]&&identity.nativeUnit[axis]==1U)
        linearLength=std::hypot(linearLength,input.deltaNative[axis]);
    if (!std::isfinite(linearLength) || linearLength <= 0.0) return Code::NONFINITE_GEOMETRY;
    const double seconds = NCXYZCUVFeedDetail::ProductRatio(linearLength, 60.0, input.feedMMMin);
    if (!std::isfinite(seconds) || seconds <= 0.0) return Code::NONFINITE_GEOMETRY;
    // Order is part of the canonical packet: match LoadNext's iterative norm,
    // rather than regrouping the same sum into a different binary64 result.
    double length = 0.0;
    for (unsigned axis = 0U; axis < 6U; ++axis) length = std::hypot(length, deltaPulse[axis]);
    if (!std::isfinite(length) || length <= 0.0) return Code::NONFINITE_GEOMETRY;
    const double velocity = NCXYZCUVFeedDetail::ProductRatio(length, 1.0, seconds);
    if (!std::isfinite(velocity) || velocity < 1.0) return Code::INVALID_VELOCITY;
    std::array<double, 3U> rotaryFeed{};
    for (unsigned rotary = 0U; rotary < 3U; ++rotary)
    {
        const unsigned axis=rotary+3U;
        if(!identity.exists[axis]||identity.nativeUnit[axis]!=2U||input.deltaNative[axis]==0.0)continue;
        rotaryFeed[rotary] = NCXYZCUVFeedDetail::ProductRatio(
            std::fabs(input.deltaNative[rotary + 3U]), 60.0, seconds);
        if (!std::isfinite(rotaryFeed[rotary]) || rotaryFeed[rotary] <= 0.0)
            return Code::INVALID_VELOCITY;
        if (rotaryFeed[rotary] > 100.0) return Code::ROTARY_FEED_LIMIT;
    }
    std::array<double, 6U> projectedVelocity{};
    for (unsigned axis = 0U; axis < 6U; ++axis)
    {
        if(!identity.exists[axis]||input.deltaNative[axis]==0.0)continue;
        const double directVelocity = NCXYZCUVFeedDetail::ProductRatio(std::fabs(deltaPulse[axis]), 1.0, seconds);
        const double ratio = deltaPulse[axis] / length;
        projectedVelocity[axis] = std::fabs(velocity * ratio);
        if (!std::isfinite(directVelocity) || directVelocity <= 0.0 ||
            !std::isfinite(ratio) || ratio == 0.0 ||
            !std::isfinite(projectedVelocity[axis]) || projectedVelocity[axis] <= 0.0)
            return Code::INVALID_VELOCITY;
        if (directVelocity > input.maxVelocityPPS[axis] || projectedVelocity[axis] > input.maxVelocityPPS[axis])
            return Code::AXIS_VELOCITY_LIMIT;
    }
    const double eps64 = 64.0 * (std::numeric_limits<double>::epsilon)();
    for (unsigned axis = 3U; axis < 6U; ++axis)
    {
        if(!identity.exists[axis]||identity.nativeUnit[axis]!=2U||input.deltaNative[axis]==0.0)continue;
        const double mappedRotaryFeed = NCXYZCUVFeedDetail::ProductRatio(
            projectedVelocity[axis], 60.0, input.pulsePerUnit[axis]);
        if (!std::isfinite(mappedRotaryFeed) || mappedRotaryFeed <= 0.0)
            return Code::INVALID_VELOCITY;
        const double pulseScale = NCXYZCUVFeedDetail::Maximum(std::fabs(deltaPulse[axis]),
            NCXYZCUVFeedDetail::Maximum(std::fabs(input.startPulse[axis]), std::fabs(input.endPulse[axis])));
        const double nativeScale = NCXYZCUVFeedDetail::Maximum(std::fabs(input.startMCS[axis]),
            std::fabs(input.endMCS[axis]));
        const double subtractionBudget = eps64 * (pulseScale / std::fabs(deltaPulse[axis])) +
            (input.absolute ? eps64 * (nativeScale / std::fabs(input.deltaNative[axis])) : 0.0);
        const double relativeBudget = eps64 + (subtractionBudget < 1.0e-9 ? subtractionBudget : 1.0e-9);
        // Each authored rate test is strict. Only independently bounded pulse
        // subtraction/projection roundoff is allowed here; no velocity is clamped.
        if (mappedRotaryFeed > 100.0 && mappedRotaryFeed - 100.0 > 100.0 * relativeBudget)
            return Code::ROTARY_FEED_LIMIT;
    }
    double accTime = 0.0, decTime = 0.0;
    for (unsigned axis = 0U; axis < 6U; ++axis)
    {
        if(!identity.exists[axis]||input.deltaNative[axis]==0.0)continue;
        accTime = NCXYZCUVFeedDetail::Maximum(accTime, NCXYZCUVFeedDetail::EffectiveTime(input.axisAccTime[axis]));
        decTime = NCXYZCUVFeedDetail::Maximum(decTime, NCXYZCUVFeedDetail::EffectiveTime(input.axisDecTime[axis]));
    }
    const double acceleration = velocity / accTime, deceleration = velocity / decTime;
    if (!std::isfinite(acceleration) || acceleration <= 0.0 || !std::isfinite(deceleration) || deceleration <= 0.0)
        return Code::INVALID_VELOCITY;
    output.startMCS = input.startMCS; output.endMCS = input.endMCS;
    output.startPulse = input.startPulse; output.endPulse = input.endPulse;
    output.axisMask = input.axisMask; output.feedMMMin = input.feedMMMin;
    output.authoredAxisMask=authoredMask;
    output.zOnlyCommissioning = input.zOnlyCommissioning;
    output.xyzOnlyCommissioning = input.xyzOnlyCommissioning;
    output.sparseCommissioning = input.sparseCommissioning;
    output.deltaNative = input.deltaNative; output.pulsePerUnit = input.pulsePerUnit;
    for(unsigned axis=0U;axis<6U;++axis)if(!identity.exists[axis])output.pulsePerUnit[axis]=0.0;
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

// Full value comparison without interpreting padding as packet identity.
inline bool SameNCXYZCUVFeedLineValue(const NCXYZCUVFeedLineValue& a,const NCXYZCUVFeedLineValue& b) noexcept
{
    using NCXYZCUVFeedDetail::SameBits;
    if(a.axisMask!=b.axisMask||a.authoredAxisMask!=b.authoredAxisMask||a.absolute!=b.absolute||a.zOnlyCommissioning!=b.zOnlyCommissioning||
       a.xyzOnlyCommissioning!=b.xyzOnlyCommissioning||a.sparseCommissioning!=b.sparseCommissioning||a.valid!=b.valid)return false;
    if(!SameBits(a.feedMMMin,b.feedMMMin)||!SameBits(a.linearLengthMM,b.linearLengthMM)||
       !SameBits(a.nominalSeconds,b.nominalSeconds)||!SameBits(a.lengthPulse,b.lengthPulse)||
       !SameBits(a.velocityPPS,b.velocityPPS)||!SameBits(a.accTime,b.accTime)||!SameBits(a.decTime,b.decTime))return false;
    return std::memcmp(a.startMCS.data(),b.startMCS.data(),sizeof(a.startMCS))==0&&
        std::memcmp(a.endMCS.data(),b.endMCS.data(),sizeof(a.endMCS))==0&&
        std::memcmp(a.startPulse.data(),b.startPulse.data(),sizeof(a.startPulse))==0&&
        std::memcmp(a.endPulse.data(),b.endPulse.data(),sizeof(a.endPulse))==0&&
        std::memcmp(a.deltaNative.data(),b.deltaNative.data(),sizeof(a.deltaNative))==0&&
        std::memcmp(a.pulsePerUnit.data(),b.pulsePerUnit.data(),sizeof(a.pulsePerUnit))==0&&
        std::memcmp(a.axisVelocityPPS.data(),b.axisVelocityPPS.data(),sizeof(a.axisVelocityPPS))==0&&
        std::memcmp(a.rotaryFeedDegMin.data(),b.rotaryFeedDegMin.data(),sizeof(a.rotaryFeedDegMin))==0&&
        std::memcmp(a.absoluteTargetWCS.data(),b.absoluteTargetWCS.data(),sizeof(a.absoluteTargetWCS))==0&&
        std::memcmp(a.absoluteTargetMCS.data(),b.absoluteTargetMCS.data(),sizeof(a.absoluteTargetMCS))==0&&
        std::memcmp(a.rotaryModulo.data(),b.rotaryModulo.data(),sizeof(a.rotaryModulo))==0&&
        a.rotaryShortestPath==b.rotaryShortestPath;
}

static_assert(std::numeric_limits<double>::is_iec559 && sizeof(double) == 8U,
    "BASE74 XYZCUV feed requires IEEE binary64.");
static_assert(sizeof(NCXYZCUVFeedLineValue) <= 624U &&
    std::is_standard_layout<NCXYZCUVFeedLineValue>::value &&
    std::is_trivially_copyable<NCXYZCUVFeedLineValue>::value,
    "BASE74 XYZCUV geometry must remain a bounded value.");
