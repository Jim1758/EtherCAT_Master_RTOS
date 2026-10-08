#pragma once

#include "NCTranslationSnapshot.h"
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <type_traits>

// BASE76: immutable, native-unit geometry for one electrode role rotating about
// the fixed mechanical Z direction. This value grants NO Motion admission,
// ownership, queue, replay, servo, or stopped-transition permission. In
// particular, its bounds describe the authored sweep, NOT RESET stopping/FIR
// overrun. The later runtime lane must prove those additional bounds separately.
// H keeps its existing meaning: an XYZ millimetre vector, signed by G43/G44.
// EXT/WCS change nominal representation; neither changes the physical angle.
#if defined(__FAST_MATH__) || defined(_M_FP_FAST)
#error BASE76_eccentric_geometry_requires_precise_floating_point_semantics
#endif
#if defined(__FINITE_MATH_ONLY__) && (__FINITE_MATH_ONLY__ > 0)
#error BASE76_eccentric_geometry_requires_nonfinite_checks
#endif

constexpr bool NCEccentricCDynamicMotionAdmission = false;

enum class NCEccentricCPathCode : std::uint8_t
{
    NOT_BUILT = 0U,
    BUILT_PATH = 1U,
    INVALID_SOURCE = 2U,
    INVALID_ELECTRODE_ROLE = 3U,
    INVALID_AXIS_CONFIGURATION = 4U,
    NONFINITE_COORDINATE = 5U,
    INVALID_SWEEP = 6U,
    INVALID_FEED = 7U,
    INVALID_RAMP_TIME = 8U,
    NUMERIC_RANGE = 9U,
    BELOW_RESOLUTION = 10U,
    AXIS_VELOCITY_LIMIT = 11U,
    AXIS_ACCELERATION_LIMIT = 12U,
    AXIS_DECELERATION_LIMIT = 13U
};

struct NCEccentricCPathInput
{
    NCTranslationSnapshot source{};
    std::array<double, 8U> startMCS{};
    // Native units/s and native units/s^2; no mixed mm/degree norm.
    std::array<double, 8U> maxVelocityNative{};
    std::array<double, 8U> maxAccelerationNative{};
    std::array<double, 8U> maxDecelerationNative{};
    // Already resolved continuous physical sweep. No shortest-path/modulo
    // decision is made here. F remains degrees/minute for the authored role.
    double sweepDeg = 0.0, feedDegMin = 0.0;
    // BASE79K: resolved G90/G91 Z travel, synchronized to the same C
    // scalar. Zero preserves the accepted C-only geometry exactly.
    double zDeltaMM = 0.0;
    // BASE79M: resolved G90/G91 nominal XY translation. The producer solves
    // absolute targets; zero retains the accepted C-only/Z+C lane.
    double xDeltaMM = 0.0, yDeltaMM = 0.0;
    double accTime = 0.0, decTime = 0.0;
};

struct NCEccentricCPathValue
{
    NCTranslationSnapshot source{};
    std::array<double, 8U> startMCS{}, endMCS{}, minMCS{}, maxMCS{};
    std::array<double, 8U> requiredVelocityNative{};
    std::array<double, 8U> requiredAccelerationNative{};
    std::array<double, 8U> requiredDecelerationNative{};
    std::array<double, 3U> nominalMCS{}, nominalWCS{};
    std::uint32_t authoredMask = 0U, generatedMask = 0U, groupMask = 0U;
    std::uint32_t electrodeAxis = 8U;
    double sweepDeg = 0.0, feedDegMin = 0.0, accTime = 0.0, decTime = 0.0;
    double zDeltaMM = 0.0;
    // BASE79M: resolved G90/G91 nominal XY translation. The producer solves
    // absolute targets; zero retains the accepted C-only/Z+C lane.
    double xDeltaMM = 0.0, yDeltaMM = 0.0;
    double radiusMM = 0.0, omegaRadPerSec = 0.0;
    double alphaAccRadPerSec2 = 0.0, alphaDecRadPerSec2 = 0.0;
    bool valid = false;

    void Clear() noexcept { *this = NCEccentricCPathValue{}; }
};

struct NCEccentricCPoint
{
    std::array<double, 8U> positionMCS{}, derivativePerU{};
    bool valid = false;
    void Clear() noexcept { *this = NCEccentricCPoint{}; }
};

using NCEccentricCPathPoint = NCEccentricCPoint;

namespace NCEccentricCDetail
{
    constexpr double Pi = 3.141592653589793238462643383279502884;
    constexpr double RadPerDeg = Pi / 180.0;
    // Explicit numerical contract, unrelated to servo in-position tolerance.
    // 1e7 degrees bounds phase conversion error; the scale-dependent native
    // roundoff test below may impose a much tighter bound for a large radius.
    constexpr double MaxAbsAngleDeg = 1.0e7;
    constexpr double NativeRoundoffBudget = 5.0e-8;
    // Native endpoint addition must preserve the resolved sweep to 0.01 ppm.
    // An absolute-only budget would admit a materially wrong tiny sweep at a
    // large multi-turn start, particularly when the XY eccentricity is zero.
    constexpr double RelativeSweepRoundoff = 1.0e-8;
    constexpr double Epsilon = std::numeric_limits<double>::epsilon();

    inline bool Positive(double x) noexcept { return std::isfinite(x) && x > 0.0; }
    inline double Max(double a, double b) noexcept { return a > b ? a : b; }
    inline double Min(double a, double b) noexcept { return a < b ? a : b; }

    inline bool Same(double a, double b) noexcept
    {
        return std::memcmp(&a, &b, sizeof(double)) == 0;
    }

    template <std::size_t Count>
    inline bool Same(const std::array<double, Count>& a,
        const std::array<double, Count>& b) noexcept
    {
        return std::memcmp(a.data(), b.data(), Count * sizeof(double)) == 0;
    }

    inline bool Scope(const NCTranslationSnapshot& s) noexcept
    {
        return IsNCTranslationSnapshotValid(s) && s.axisIdentity.systemMode == 1U &&
            s.axisIdentity.eccentricEnabled == 1U && s.unitsMode == 21 &&
            s.rotationPlane == 17 && s.polarMode == 15 && s.cutterMode == 40 &&
            (s.toolLengthMode == 43 || s.toolLengthMode == 44) &&
            s.toolHCode >= 1 && s.toolHCode <= 100 && s.rotationMode == 69 &&
            s.workMode == 169 && s.scalingMode == 50 && s.mirrorMask == 0U;
    }

    inline bool Role(const NCTranslationSnapshot& s, unsigned& role) noexcept
    {
        role = 8U;
        const NCAxisIdentitySnapshot& id = s.axisIdentity;
        if (!IsNCAxisIdentitySnapshotValid(id) || id.electrodeAxisPlusOne < 4U ||
            id.electrodeAxisPlusOne > 8U) return false;
        const unsigned selected = id.electrodeAxisPlusOne - 1U;
        if (id.exists[selected] != 1U || id.axisType[selected] != 1U ||
            id.nativeUnit[selected] != 2U) return false;
        for (unsigned a = 0U; a < 3U; ++a)
            if (id.exists[a] != 1U || id.axisType[a] != 0U || id.nativeUnit[a] != 1U)
                return false;
        role = selected;
        return true;
    }

    // Trig has a fixed physical 360-degree period, independent of any NC
    // shortest-path modulo. Cardinal values are exact, including full turns.
    inline bool SinCos(double degrees, double& sine, double& cosine) noexcept
    {
        sine = cosine = 0.0;
        if (!std::isfinite(degrees) || std::fabs(degrees) > MaxAbsAngleDeg) return false;
        const double phase = std::remainder(degrees, 360.0);
        if (phase == 0.0) { sine = 0.0; cosine = 1.0; }
        else if (phase == 90.0) { sine = 1.0; cosine = 0.0; }
        else if (phase == -90.0) { sine = -1.0; cosine = 0.0; }
        else if (std::fabs(phase) == 180.0) { sine = 0.0; cosine = -1.0; }
        else { const double rad = phase * RadPerDeg; sine = std::sin(rad); cosine = std::cos(rad); }
        return std::isfinite(sine) && std::isfinite(cosine);
    }

    inline bool Offset(const NCTranslationSnapshot& s, double degrees,
        std::array<double, 3U>& offset) noexcept
    {
        offset.fill(0.0);
        double sine = 0.0, cosine = 0.0;
        if (!SinCos(degrees, sine, cosine)) return false;
        const double sign = s.toolLengthMode == 43 ? 1.0 : -1.0;
        offset[0] = sign * (s.toolOffsetMM[0] * cosine - s.toolOffsetMM[1] * sine);
        offset[1] = sign * (s.toolOffsetMM[0] * sine + s.toolOffsetMM[1] * cosine);
        offset[2] = sign * s.toolOffsetMM[2];
        return std::isfinite(offset[0]) && std::isfinite(offset[1]) && std::isfinite(offset[2]);
    }

    inline double PositivePhase(double degrees) noexcept
    {
        double result = std::fmod(degrees, 360.0);
        if (result < 0.0) result += 360.0;
        return result;
    }

    // A constant number of analytic critical-angle tests, never a sampling
    // approximation or a loop whose duration depends on the number of turns.
    inline bool ContainsPhase(double start, double sweep, double target,
        double angleGuard) noexcept
    {
        if (std::fabs(sweep) >= 360.0) return true;
        const double distance = sweep > 0.0 ? PositivePhase(target - start) :
            PositivePhase(start - target);
        return distance <= std::fabs(sweep) + angleGuard || distance >= 360.0 - angleGuard;
    }

    inline bool ValueShape(const NCEccentricCPathValue& v) noexcept
    {
        unsigned role = 8U;
        if (!v.valid || !Scope(v.source) || !Role(v.source, role) || role != v.electrodeAxis ||
            v.authoredMask != ((1U << role) | (v.zDeltaMM != 0.0 ? 4U : 0U) |
                (v.xDeltaMM != 0.0 ? 1U : 0U) | (v.yDeltaMM != 0.0 ? 2U : 0U)) ||
            !std::isfinite(v.xDeltaMM) || !std::isfinite(v.yDeltaMM) ||
            ((v.xDeltaMM != 0.0 || v.yDeltaMM != 0.0) &&
                v.source.distanceMode != 90 && v.source.distanceMode != 91) ||
            !std::isfinite(v.zDeltaMM) || (v.zDeltaMM != 0.0 && v.source.distanceMode != 90 && v.source.distanceMode != 91) ||
            v.generatedMask != (v.radiusMM > 0.0 ? 3U : 0U) ||
            v.groupMask != (v.authoredMask | v.generatedMask) ||
            !std::isfinite(v.radiusMM) || v.radiusMM < 0.0 ||
            !std::isfinite(v.sweepDeg) || v.sweepDeg == 0.0 ||
            std::fabs(v.sweepDeg) > MaxAbsAngleDeg || !Positive(v.feedDegMin) ||
            v.feedDegMin > 100.0 || !Positive(v.accTime) || !Positive(v.decTime)) return false;
        if (!Positive(v.omegaRadPerSec) || !Positive(v.alphaAccRadPerSec2) ||
            !Positive(v.alphaDecRadPerSec2)) return false;
        for (unsigned a = 0U; a < 8U; ++a)
            if (!std::isfinite(v.startMCS[a]) || !std::isfinite(v.endMCS[a]) ||
                !std::isfinite(v.minMCS[a]) || !std::isfinite(v.maxMCS[a]) ||
                v.minMCS[a] > v.maxMCS[a] || v.startMCS[a] < v.minMCS[a] ||
                v.startMCS[a] > v.maxMCS[a] || v.endMCS[a] < v.minMCS[a] ||
                v.endMCS[a] > v.maxMCS[a]) return false;
        for (unsigned a = 0U; a < 3U; ++a)
            if (!std::isfinite(v.nominalMCS[a]) || !std::isfinite(v.nominalWCS[a])) return false;
        return true;
    }
}

inline NCEccentricCPathCode BuildNCEccentricCPath(const NCEccentricCPathInput& input,
    NCEccentricCPathValue& output) noexcept
{
    using namespace NCEccentricCDetail;
    output.Clear();
    if (!Scope(input.source) || (input.zDeltaMM != 0.0 && input.source.distanceMode != 90 && input.source.distanceMode != 91))
        return NCEccentricCPathCode::INVALID_SOURCE;
    if (!std::isfinite(input.zDeltaMM) || !std::isfinite(input.xDeltaMM) ||
        !std::isfinite(input.yDeltaMM)) return NCEccentricCPathCode::NONFINITE_COORDINATE;
    const bool authoredXY = input.xDeltaMM != 0.0 || input.yDeltaMM != 0.0;
    if (authoredXY && input.source.distanceMode != 90 && input.source.distanceMode != 91) return NCEccentricCPathCode::INVALID_SOURCE;
    unsigned role = 8U;
    if (!Role(input.source, role)) return NCEccentricCPathCode::INVALID_ELECTRODE_ROLE;
    for (double coordinate : input.startMCS)
        if (!std::isfinite(coordinate)) return NCEccentricCPathCode::NONFINITE_COORDINATE;
    if (!std::isfinite(input.sweepDeg) || input.sweepDeg == 0.0)
        return NCEccentricCPathCode::INVALID_SWEEP;
    if (!Positive(input.feedDegMin) || input.feedDegMin > 100.0)
        return NCEccentricCPathCode::INVALID_FEED;
    if (!Positive(input.accTime) || !Positive(input.decTime))
        return NCEccentricCPathCode::INVALID_RAMP_TIME;
    if (std::fabs(input.sweepDeg) > MaxAbsAngleDeg ||
        std::fabs(input.startMCS[role]) > MaxAbsAngleDeg)
        return NCEccentricCPathCode::NUMERIC_RANGE;

    NCEccentricCPathValue v{};
    v.source = input.source;
    v.startMCS = v.endMCS = v.minMCS = v.maxMCS = input.startMCS;
    v.electrodeAxis = role;
    v.zDeltaMM = input.zDeltaMM;
    v.xDeltaMM = input.xDeltaMM; v.yDeltaMM = input.yDeltaMM;
    v.authoredMask = (1U << role) | (v.zDeltaMM != 0.0 ? 4U : 0U) |
        (v.xDeltaMM != 0.0 ? 1U : 0U) | (v.yDeltaMM != 0.0 ? 2U : 0U);
    v.sweepDeg = input.sweepDeg;
    v.feedDegMin = input.feedDegMin;
    v.accTime = input.accTime;
    v.decTime = input.decTime;
    v.radiusMM = std::hypot(v.source.toolOffsetMM[0], v.source.toolOffsetMM[1]);
    if (!std::isfinite(v.radiusMM)) return NCEccentricCPathCode::NUMERIC_RANGE;
    v.generatedMask = v.radiusMM > 0.0 ? 3U : 0U;
    v.groupMask = v.authoredMask | v.generatedMask;
    v.endMCS[role] = v.startMCS[role] + v.sweepDeg;
    if (!std::isfinite(v.endMCS[role]) || std::fabs(v.endMCS[role]) > MaxAbsAngleDeg)
        return NCEccentricCPathCode::NUMERIC_RANGE;
    if (v.endMCS[role] == v.startMCS[role]) return NCEccentricCPathCode::BELOW_RESOLUTION;
    const double resolvedSweep = v.endMCS[role] - v.startMCS[role];
    if (!std::isfinite(resolvedSweep) || std::signbit(resolvedSweep) != std::signbit(v.sweepDeg))
        return NCEccentricCPathCode::BELOW_RESOLUTION;
    const double sweepError = std::fabs(resolvedSweep - v.sweepDeg);
    if (sweepError > NativeRoundoffBudget ||
        sweepError > std::fabs(v.sweepDeg) * RelativeSweepRoundoff)
        return NCEccentricCPathCode::BELOW_RESOLUTION;

    std::array<double, 3U> startOffset{}, endOffset{};
    if (!Offset(v.source, v.startMCS[role], startOffset) ||
        !Offset(v.source, v.endMCS[role], endOffset))
        return NCEccentricCPathCode::NUMERIC_RANGE;
    const bool fullTurns = std::remainder(v.sweepDeg, 360.0) == 0.0;
    if (fullTurns) endOffset = startOffset;
    const double angleScale = Max(360.0, Max(std::fabs(v.startMCS[role]),
        Max(std::fabs(v.endMCS[role]), std::fabs(v.sweepDeg))));
    const double angleGuard = 64.0 * Epsilon * angleScale;
    const double phaseErrorMM = v.radiusMM * (angleGuard * RadPerDeg);
    if (!std::isfinite(phaseErrorMM) || phaseErrorMM > NativeRoundoffBudget)
        return NCEccentricCPathCode::NUMERIC_RANGE;
    std::array<double, 2U> positionGuard{};
    for (unsigned a = 0U; a < 3U; ++a)
    {
        v.nominalMCS[a] = v.startMCS[a] - startOffset[a];
        const double sourceOffset = v.source.extOffsetMM[a] + v.source.wcsOffsetMM[a];
        v.nominalWCS[a] = v.nominalMCS[a] - sourceOffset;
        const double scale = Max(std::fabs(v.startMCS[a]), Max(std::fabs(startOffset[a]),
            Max(std::fabs(v.nominalMCS[a]), Max(std::fabs(v.nominalWCS[a]),
                Max(std::fabs(v.source.extOffsetMM[a]), std::fabs(v.source.wcsOffsetMM[a]))))));
        const double arithmeticGuard = 64.0 * Epsilon * Max(scale, v.radiusMM);
        if (!std::isfinite(v.nominalMCS[a]) || !std::isfinite(v.nominalWCS[a]) ||
            !std::isfinite(arithmeticGuard) || arithmeticGuard > NativeRoundoffBudget ||
            (a < 2U && arithmeticGuard + phaseErrorMM > NativeRoundoffBudget))
            return NCEccentricCPathCode::NUMERIC_RANGE;
        if (a < 2U)
        {
            positionGuard[a] = arithmeticGuard + phaseErrorMM;
            v.endMCS[a] = fullTurns || v.radiusMM == 0.0 ? v.startMCS[a] :
                v.startMCS[a] + (endOffset[a] - startOffset[a]);
            if (!std::isfinite(v.endMCS[a])) return NCEccentricCPathCode::NUMERIC_RANGE;
        }
    }

    if (v.zDeltaMM != 0.0)
    {
        v.endMCS[2] = v.startMCS[2] + v.zDeltaMM;
        const double represented = v.endMCS[2] - v.startMCS[2];
        const double error = std::fabs(represented - v.zDeltaMM);
        if (!std::isfinite(v.endMCS[2]) || !std::isfinite(represented) ||
            represented == 0.0 || std::signbit(represented) != std::signbit(v.zDeltaMM) ||
            error > NativeRoundoffBudget || error > std::fabs(v.zDeltaMM) * RelativeSweepRoundoff)
            return NCEccentricCPathCode::BELOW_RESOLUTION;
        // The affine Z path is monotone. Enclose its endpoints plus the
        // rounding of intermediate multiply/add; H Z remains a fixed offset.
        const double guard = 64.0 * Epsilon * Max(std::fabs(v.zDeltaMM),
            Max(std::fabs(v.startMCS[2]), std::fabs(v.endMCS[2])));
        if (!std::isfinite(guard) || guard > NativeRoundoffBudget)
            return NCEccentricCPathCode::NUMERIC_RANGE;
        const double infinity = (std::numeric_limits<double>::infinity)();
        v.minMCS[2] = std::nextafter(Min(v.startMCS[2], v.endMCS[2]) - guard, -infinity);
        v.maxMCS[2] = std::nextafter(Max(v.startMCS[2], v.endMCS[2]) + guard, infinity);
        if (!std::isfinite(v.minMCS[2]) || !std::isfinite(v.maxMCS[2]))
            return NCEccentricCPathCode::NUMERIC_RANGE;
    }

    const double sweepRad = std::fabs(v.sweepDeg) * RadPerDeg;
    const double xyArcLength = v.radiusMM * sweepRad;
    if (!std::isfinite(xyArcLength)) return NCEccentricCPathCode::NUMERIC_RANGE;
    if (v.radiusMM > 0.0 && (xyArcLength == 0.0 ||
        (xyArcLength <= Max(positionGuard[0], positionGuard[1]) && !fullTurns)))
        return NCEccentricCPathCode::BELOW_RESOLUTION;

    v.minMCS[role] = Min(v.startMCS[role], v.endMCS[role]);
    v.maxMCS[role] = Max(v.startMCS[role], v.endMCS[role]);
    if (v.radiusMM > 0.0)
    {
        // The signed rotated H at the start determines physical phase. G44
        // therefore shifts phase by 180 degrees without reversing the sweep.
        const double startPhase = std::atan2(startOffset[1], startOffset[0]) / RadPerDeg;
        for (unsigned a = 0U; a < 2U; ++a)
        {
            double low = Min(v.startMCS[a], v.endMCS[a]);
            double high = Max(v.startMCS[a], v.endMCS[a]);
            const double maximumPhase = a == 0U ? 0.0 : 90.0;
            const double minimumPhase = a == 0U ? 180.0 : 270.0;
            if (ContainsPhase(startPhase, v.sweepDeg, maximumPhase, angleGuard))
                high = Max(high, v.nominalMCS[a] + v.radiusMM);
            if (ContainsPhase(startPhase, v.sweepDeg, minimumPhase, angleGuard))
                low = Min(low, v.nominalMCS[a] - v.radiusMM);
            v.minMCS[a] = std::nextafter(low - positionGuard[a],
                -(std::numeric_limits<double>::infinity)());
            v.maxMCS[a] = std::nextafter(high + positionGuard[a],
                (std::numeric_limits<double>::infinity)());
            if (!std::isfinite(v.minMCS[a]) || !std::isfinite(v.maxMCS[a]))
                return NCEccentricCPathCode::NUMERIC_RANGE;
        }
    }

    // BASE79L: the pure rotating-H enclosure above is independent of the
    // affine nominal line. Its Minkowski sum with each translation interval
    // encloses every interior extremum without sampling or an unbounded solve.
    if (authoredXY)
    {
        const double delta[2] = { v.xDeltaMM, v.yDeltaMM };
        const double infinity = (std::numeric_limits<double>::infinity)();
        for (unsigned a = 0U; a < 2U; ++a)
        {
            if (delta[a] == 0.0) continue;
            const double authoredEnd = v.startMCS[a] + delta[a];
            const double represented = authoredEnd - v.startMCS[a];
            const double error = std::fabs(represented - delta[a]);
            if (!std::isfinite(authoredEnd) || !std::isfinite(represented) ||
                represented == 0.0 || std::signbit(represented) != std::signbit(delta[a]) ||
                error > NativeRoundoffBudget || error > std::fabs(delta[a]) * RelativeSweepRoundoff)
                return NCEccentricCPathCode::BELOW_RESOLUTION;
            const double rotatingDelta = fullTurns || v.radiusMM == 0.0 ? 0.0 :
                endOffset[a] - startOffset[a];
            const double combinedDelta = delta[a] + rotatingDelta;
            v.endMCS[a] = v.startMCS[a] + combinedDelta;
            const double scale = Max(std::fabs(delta[a]), Max(std::fabs(rotatingDelta),
                Max(std::fabs(v.startMCS[a]), std::fabs(v.endMCS[a]))));
            const double guard = 64.0 * Epsilon * scale;
            if (!std::isfinite(combinedDelta) || !std::isfinite(v.endMCS[a]) ||
                !std::isfinite(guard) || guard + positionGuard[a] > NativeRoundoffBudget)
                return NCEccentricCPathCode::NUMERIC_RANGE;
            // A zero physical XY endpoint delta is valid when authored travel
            // cancels the change in H. Only the authored delta must resolve.
            v.minMCS[a] = std::nextafter(v.minMCS[a] + Min(0.0, delta[a]) - guard, -infinity);
            v.maxMCS[a] = std::nextafter(v.maxMCS[a] + Max(0.0, delta[a]) + guard, infinity);
            if (!std::isfinite(v.minMCS[a]) || !std::isfinite(v.maxMCS[a]))
                return NCEccentricCPathCode::NUMERIC_RANGE;
        }
    }

    const double degreesPerSec = v.feedDegMin / 60.0;
    v.omegaRadPerSec = degreesPerSec * RadPerDeg;
    v.alphaAccRadPerSec2 = v.omegaRadPerSec / v.accTime;
    v.alphaDecRadPerSec2 = v.omegaRadPerSec / v.decTime;
    if (!Positive(degreesPerSec) || !Positive(v.omegaRadPerSec) ||
        !Positive(v.alphaAccRadPerSec2) || !Positive(v.alphaDecRadPerSec2))
        return NCEccentricCPathCode::NUMERIC_RANGE;
    v.requiredVelocityNative[role] = degreesPerSec;
    v.requiredAccelerationNative[role] = degreesPerSec / v.accTime;
    v.requiredDecelerationNative[role] = degreesPerSec / v.decTime;
    if (v.zDeltaMM != 0.0)
    {
        const double velocity = std::fabs(v.zDeltaMM / v.sweepDeg) * degreesPerSec;
        v.requiredVelocityNative[2] = velocity;
        v.requiredAccelerationNative[2] = velocity / v.accTime;
        v.requiredDecelerationNative[2] = velocity / v.decTime;
    }
    if (v.radiusMM > 0.0)
    {
        const double velocity = v.radiusMM * v.omegaRadPerSec;
        const double centripetal = velocity * v.omegaRadPerSec;
        const double acceleration = v.radiusMM * v.alphaAccRadPerSec2 + centripetal;
        const double deceleration = v.radiusMM * v.alphaDecRadPerSec2 + centripetal;
        if (!Positive(velocity) || !std::isfinite(centripetal) ||
            !Positive(acceleration) || !Positive(deceleration))
            return NCEccentricCPathCode::NUMERIC_RANGE;
        v.requiredVelocityNative[0] = v.requiredVelocityNative[1] = velocity;
        v.requiredAccelerationNative[0] = v.requiredAccelerationNative[1] = acceleration;
        v.requiredDecelerationNative[0] = v.requiredDecelerationNative[1] = deceleration;
    }
    if (authoredXY)
    {
        const double delta[2] = { v.xDeltaMM, v.yDeltaMM };
        for (unsigned a = 0U; a < 2U; ++a)
        {
            if (delta[a] == 0.0) continue;
            const double velocity = std::fabs(delta[a] / v.sweepDeg) * degreesPerSec;
            if (!Positive(velocity)) return NCEccentricCPathCode::NUMERIC_RANGE;
            v.requiredVelocityNative[a] += velocity;
            v.requiredAccelerationNative[a] += velocity / v.accTime;
            v.requiredDecelerationNative[a] += velocity / v.decTime;
        }
    }
    for (unsigned a = 0U; a < 8U; ++a)
    {
        if ((v.groupMask & (1U << a)) == 0U) continue;
        // Round physical requirements outward as well as positions. The
        // conservative 32-epsilon margin covers binary64 hypot/product/sum
        // rounding; an exact supplied limit must cover the reported bound.
        const double up = 1.0 + 32.0 * Epsilon;
        const double infinity = (std::numeric_limits<double>::infinity)();
        v.requiredVelocityNative[a] = std::nextafter(v.requiredVelocityNative[a] * up, infinity);
        v.requiredAccelerationNative[a] = std::nextafter(v.requiredAccelerationNative[a] * up, infinity);
        v.requiredDecelerationNative[a] = std::nextafter(v.requiredDecelerationNative[a] * up, infinity);
        if (!Positive(input.maxVelocityNative[a]) || !Positive(input.maxAccelerationNative[a]) ||
            !Positive(input.maxDecelerationNative[a]))
            return NCEccentricCPathCode::INVALID_AXIS_CONFIGURATION;
        if (!Positive(v.requiredVelocityNative[a]) || !Positive(v.requiredAccelerationNative[a]) ||
            !Positive(v.requiredDecelerationNative[a])) return NCEccentricCPathCode::NUMERIC_RANGE;
        if (v.requiredVelocityNative[a] > input.maxVelocityNative[a])
            return NCEccentricCPathCode::AXIS_VELOCITY_LIMIT;
        if (v.requiredAccelerationNative[a] > input.maxAccelerationNative[a])
            return NCEccentricCPathCode::AXIS_ACCELERATION_LIMIT;
        if (v.requiredDecelerationNative[a] > input.maxDecelerationNative[a])
            return NCEccentricCPathCode::AXIS_DECELERATION_LIMIT;
    }
    v.valid = true;
    if (!ValueShape(v)) return NCEccentricCPathCode::NUMERIC_RANGE;
    output = v;
    return NCEccentricCPathCode::BUILT_PATH;
}

// Public geometry helpers accept only the complete deterministic Build result,
// not merely a caller-set valid bit. Rebuilding here is bounded and allocation
// free. This is a reference/validation API, not a claim that its validation cost
// has been qualified for a 250 us runtime loop.
inline bool IsNCEccentricCPathValueValid(const NCEccentricCPathValue& value) noexcept
{
    using NCEccentricCDetail::Same;
    if (!NCEccentricCDetail::ValueShape(value)) return false;
    NCEccentricCPathInput input{};
    input.source = value.source;
    input.startMCS = value.startMCS;
    input.sweepDeg = value.sweepDeg;
    input.zDeltaMM = value.zDeltaMM;
    input.xDeltaMM = value.xDeltaMM; input.yDeltaMM = value.yDeltaMM;
    input.feedDegMin = value.feedDegMin;
    input.accTime = value.accTime;
    input.decTime = value.decTime;
    input.maxVelocityNative.fill((std::numeric_limits<double>::max)());
    input.maxAccelerationNative.fill((std::numeric_limits<double>::max)());
    input.maxDecelerationNative.fill((std::numeric_limits<double>::max)());
    NCEccentricCPathValue rebuilt{};
    if (BuildNCEccentricCPath(input, rebuilt) != NCEccentricCPathCode::BUILT_PATH) return false;
    return Same(value.endMCS, rebuilt.endMCS) && Same(value.minMCS, rebuilt.minMCS) &&
        Same(value.maxMCS, rebuilt.maxMCS) && Same(value.nominalMCS, rebuilt.nominalMCS) &&
        Same(value.nominalWCS, rebuilt.nominalWCS) &&
        Same(value.requiredVelocityNative, rebuilt.requiredVelocityNative) &&
        Same(value.requiredAccelerationNative, rebuilt.requiredAccelerationNative) &&
        Same(value.requiredDecelerationNative, rebuilt.requiredDecelerationNative) &&
        value.electrodeAxis == rebuilt.electrodeAxis && value.authoredMask == rebuilt.authoredMask &&
        value.generatedMask == rebuilt.generatedMask && value.groupMask == rebuilt.groupMask &&
        Same(value.radiusMM, rebuilt.radiusMM) && Same(value.omegaRadPerSec, rebuilt.omegaRadPerSec) &&
        Same(value.alphaAccRadPerSec2, rebuilt.alphaAccRadPerSec2) &&
        Same(value.alphaDecRadPerSec2, rebuilt.alphaDecRadPerSec2);
}

// Only use a successful, immutable Build value. A future Motion consumer must
// independently rebuild its packet before entering a runtime evaluator.
inline bool EvaluateNCEccentricCPath(const NCEccentricCPathValue& value, double u,
    NCEccentricCPoint& output) noexcept
{
    using namespace NCEccentricCDetail;
    output.Clear();
    if (!IsNCEccentricCPathValueValid(value) || !std::isfinite(u) || u < 0.0 || u > 1.0) return false;
    NCEccentricCPoint point{};
    point.positionMCS = value.startMCS;
    const unsigned role = value.electrodeAxis;
    const double angle = u == 0.0 ? value.startMCS[role] : u == 1.0 ? value.endMCS[role] :
        value.startMCS[role] + value.sweepDeg * u;
    std::array<double, 3U> startOffset{}, offset{};
    if (!Offset(value.source, value.startMCS[role], startOffset) ||
        !Offset(value.source, angle, offset)) return false;
    point.positionMCS[role] = angle;
    if (value.zDeltaMM != 0.0) point.positionMCS[2] += value.zDeltaMM * u;
    if (value.generatedMask != 0U)
        for (unsigned a = 0U; a < 2U; ++a)
            point.positionMCS[a] = value.startMCS[a] + (offset[a] - startOffset[a]);
    if (value.xDeltaMM != 0.0 || value.yDeltaMM != 0.0)
    {
        const double delta[2] = { value.xDeltaMM, value.yDeltaMM };
        for (unsigned a = 0U; a < 2U; ++a)
            if (delta[a] != 0.0)
                point.positionMCS[a] = value.startMCS[a] +
                    (delta[a] * u + (offset[a] - startOffset[a]));
    }
    if (u == 0.0) point.positionMCS = value.startMCS;
    else if (u == 1.0) point.positionMCS = value.endMCS;
    const double sweepRad = value.sweepDeg * RadPerDeg;
    if (value.generatedMask != 0U)
    {
        point.derivativePerU[0] = -offset[1] * sweepRad;
        point.derivativePerU[1] = offset[0] * sweepRad;
    }
    if (value.xDeltaMM != 0.0) point.derivativePerU[0] += value.xDeltaMM;
    if (value.yDeltaMM != 0.0) point.derivativePerU[1] += value.yDeltaMM;
    point.derivativePerU[role] = value.sweepDeg;
    point.derivativePerU[2] = value.zDeltaMM;
    for (unsigned a = 0U; a < 8U; ++a)
        if (!std::isfinite(point.positionMCS[a]) || !std::isfinite(point.derivativePerU[a]) ||
            point.positionMCS[a] < value.minMCS[a] || point.positionMCS[a] > value.maxMCS[a])
            return false;
    point.valid = true;
    output = point;
    return true;
}

// Actual points may use Actual C here; commanded points use their same-cycle
// commanded C. The caller must never feed Actual C back into commanded XY.
inline bool TryInverseNCEccentricCPoint(const NCEccentricCPathValue& value,
    const std::array<double, 8U>& physicalMCS,
    std::array<double, 3U>& nominalMCS, std::array<double, 3U>& nominalWCS) noexcept
{
    using namespace NCEccentricCDetail;
    nominalMCS.fill(0.0); nominalWCS.fill(0.0);
    if (&nominalMCS == &nominalWCS || !IsNCEccentricCPathValueValid(value)) return false;
    for (double coordinate : physicalMCS)
        if (!std::isfinite(coordinate)) return false;
    std::array<double, 3U> offset{}, mcs{}, wcs{};
    if (!Offset(value.source, physicalMCS[value.electrodeAxis], offset)) return false;
    for (unsigned a = 0U; a < 3U; ++a)
    {
        mcs[a] = physicalMCS[a] - offset[a];
        wcs[a] = mcs[a] - (value.source.extOffsetMM[a] + value.source.wcsOffsetMM[a]);
        const double scale = Max(std::fabs(physicalMCS[a]), Max(std::fabs(offset[a]),
            Max(std::fabs(mcs[a]), Max(std::fabs(wcs[a]),
                Max(std::fabs(value.source.extOffsetMM[a]), std::fabs(value.source.wcsOffsetMM[a]))))));
        const double inverseGuard = 64.0 * Epsilon * scale;
        if (!std::isfinite(mcs[a]) || !std::isfinite(wcs[a]) ||
            !std::isfinite(inverseGuard) || inverseGuard > NativeRoundoffBudget) return false;
    }
    nominalMCS = mcs; nominalWCS = wcs;
    return true;
}

// This is a geometric envelope check only. The mask must cover all generated
// and authored axes; it may additionally request stationary existing axes.
inline bool IsNCEccentricCPathWithinBounds(const NCEccentricCPathValue& value,
    const std::array<double, 8U>& minimum, const std::array<double, 8U>& maximum,
    std::uint32_t mask) noexcept
{
    if (!IsNCEccentricCPathValueValid(value) || mask == 0U || (mask & ~255U) != 0U ||
        (mask & value.groupMask) != value.groupMask) return false;
    for (unsigned a = 0U; a < 8U; ++a)
    {
        if ((mask & (1U << a)) == 0U) continue;
        if (value.source.axisIdentity.exists[a] != 1U || !std::isfinite(minimum[a]) ||
            !std::isfinite(maximum[a]) || minimum[a] > maximum[a] ||
            value.minMCS[a] < minimum[a] || value.maxMCS[a] > maximum[a]) return false;
    }
    return true;
}

static_assert(std::is_trivially_copyable<NCEccentricCPathInput>::value &&
    std::is_trivially_copyable<NCEccentricCPathValue>::value &&
    std::is_trivially_copyable<NCEccentricCPoint>::value,
    "BASE76 geometry must remain fixed, trivially copyable values.");
static_assert(sizeof(NCEccentricCPathInput) <= 1024U &&
    sizeof(NCEccentricCPathValue) <= 1280U && sizeof(NCEccentricCPoint) <= 144U,
    "BASE76 pure geometry must remain bounded; it is not a Motion transport.");
