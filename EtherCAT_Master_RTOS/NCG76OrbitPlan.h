#pragma once

#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <cstring>
#include "NCTranslationSnapshot.h"

// G76 P11 prototype geometry only. These points are inputs to the existing
// CNC line producer; this header owns no motion, GAP, power or I/O authority.
// G90 inputs are absolute program XYZ. G91 uses a zero program origin and
// literal canonical XYZ displacements, preserving small moves at large origins.
// The growth is a polygonal approximation in program G17 XY, not an exact arc.
#if defined(__FAST_MATH__) || defined(_M_FP_FAST)
#error G76_orbit_requires_precise_floating_point_semantics
#endif
#if defined(__FINITE_MATH_ONLY__) && (__FINITE_MATH_ONLY__ > 0)
#error G76_orbit_requires_nonfinite_checks
#endif

namespace NCG76
{
using Point3 = std::array<double, 3U>;
using Point8 = std::array<double, 8U>;
constexpr std::uint32_t Version = 1U;
constexpr std::uint32_t ModeP11 = 11U;
constexpr unsigned MaximumPoints = 32U;
constexpr unsigned GrowSegments = 24U;
constexpr unsigned FlushSample = 12U;
constexpr unsigned NoIndex = 0xffffffffU;
constexpr double MaximumRadiusMm = 1.0;
constexpr double MaximumCenterTravelMm = 2.0;
constexpr double MaximumFeedMmMin = 30.0;
constexpr double MaximumFlushHeightMm = 2.0;

struct PlanInput
{
    Point3 start{}, end{};
    double radiusMm = 0.0;
    double feedMmMin = 0.0;
    unsigned flushMode = 0U; // 0: none; 2: mixed retained proof required; 3/4: geometric DEMO.
    double heightMm = 0.0;  // Total height above sample-12 center, not an addition.
    double centerReturnFeedMmMin = 0.0; // Physical mm/min; zero inherits F.
};

enum class Phase : std::uint8_t
{
    Grow = 0U,
    ToCenter = 1U,
    Lift = 2U,
    ReturnCenter = 3U,
    Resume = 4U,
    FinishCenter = 5U
};

struct Endpoint
{
    Point3 position{}, center{}, orbit{};
    Phase phase = Phase::Grow;
    unsigned growIndex = 0U;
    double radiusMm = 0.0;
    double feedMmMin = 0.0;
};

struct ProgramPlan
{
    std::uint32_t version = 0U, modeP = 0U;
    PlanInput input{};
    std::array<Endpoint, MaximumPoints> points{};
    unsigned count = 0U, growCount = 0U;
    unsigned flushBeginIndex = 0U, flushEndIndex = 0U;
    std::array<unsigned, 4U> flushLegIndices{};
    unsigned flushLegCount = 0U;
    double chordErrorBoundMm = 0.0;
    Point3 boundsMin{}, boundsMax{};
};
using Plan = ProgramPlan;

// NC freezes all eight machine-coordinate slots and its translation generation
// before asking the normal transactional CNC producer to accept this batch.
struct NativePlan
{
    ProgramPlan program{};
    NCTranslationSnapshot translation{};
    std::uint32_t version = Version, modeP = ModeP11, count = 0U;
    Point8 startMCS{};
    std::array<Point8, MaximumPoints> pointsMCS{};
    std::array<double, MaximumPoints> feedMmMin{};
    std::uint64_t translationGeneration = 0ULL;
};

namespace Detail
{
inline bool Finite(double v) noexcept { return std::isfinite(v) != 0; }
inline bool FinitePoint(const Point3& p) noexcept
{
    return Finite(p[0]) && Finite(p[1]) && Finite(p[2]);
}
inline bool SamePoint(const Point3& a, const Point3& b) noexcept
{
    for (unsigned j = 0U; j < 3U; ++j)
        if (std::memcmp(&a[j],&b[j],sizeof(double)) != 0) return false;
    return true;
}
inline bool NonzeroFiniteLeg(const Point3& a, const Point3& b) noexcept
{
    if (!FinitePoint(a) || !FinitePoint(b)) return false;
    double sum = 0.0;
    bool nonzero = false;
    for (unsigned j = 0U; j < 3U; ++j)
    {
        const double d = b[j] - a[j];
        if (!Finite(d)) return false;
        nonzero = nonzero || d != 0.0;
        sum = std::hypot(sum, d);
    }
    return nonzero && Finite(sum) && sum > 0.0;
}
inline bool SameInput(const PlanInput& a, const PlanInput& b) noexcept
{
    return SamePoint(a.start,b.start) && SamePoint(a.end,b.end) &&
        a.radiusMm == b.radiusMm && a.feedMmMin == b.feedMmMin &&
        a.flushMode == b.flushMode && a.heightMm == b.heightMm &&
        a.centerReturnFeedMmMin == b.centerReturnFeedMmMin;
}
inline bool SameEndpoint(const Endpoint& a, const Endpoint& b) noexcept
{
    return SamePoint(a.position,b.position) && SamePoint(a.center,b.center) &&
        SamePoint(a.orbit,b.orbit) && a.phase == b.phase &&
        a.growIndex == b.growIndex && a.radiusMm == b.radiusMm && a.feedMmMin == b.feedMmMin;
}
inline bool Append(ProgramPlan& p, const Endpoint& point, bool flush) noexcept
{
    if (p.count >= MaximumPoints || !FinitePoint(point.position) ||
        !FinitePoint(point.center) || !FinitePoint(point.orbit) ||
        !Finite(point.radiusMm) || !Finite(point.feedMmMin) ||
        !(point.feedMmMin > 0.0) || point.feedMmMin > MaximumFeedMmMin) return false;
    const Point3& previous = p.count == 0U ? p.input.start : p.points[p.count-1U].position;
    if (!NonzeroFiniteLeg(previous,point.position)) return false;
    if (flush)
    {
        if (p.flushLegCount >= p.flushLegIndices.size()) return false;
        if (p.flushLegCount == 0U) p.flushBeginIndex = p.count;
        p.flushEndIndex = p.count;
        p.flushLegIndices[p.flushLegCount++] = p.count;
    }
    p.points[p.count++] = point;
    for (unsigned j = 0U; j < 3U; ++j)
    {
        p.boundsMin[j] = (std::min)(p.boundsMin[j],point.position[j]);
        p.boundsMax[j] = (std::max)(p.boundsMax[j],point.position[j]);
    }
    return true;
}
inline bool ValidInput(const PlanInput& in, Point3& delta, double& coordinateScale) noexcept
{
    if (!FinitePoint(in.start) || !FinitePoint(in.end) ||
        !Finite(in.radiusMm) || !(in.radiusMm > 0.0) || in.radiusMm > MaximumRadiusMm ||
        !Finite(in.feedMmMin) || !(in.feedMmMin > 0.0) || in.feedMmMin > MaximumFeedMmMin ||
        !Finite(in.heightMm) || !Finite(in.centerReturnFeedMmMin) ||
        in.centerReturnFeedMmMin < 0.0 || in.centerReturnFeedMmMin > MaximumFeedMmMin ||
        (in.flushMode != 0U && in.flushMode != 2U && in.flushMode != 3U && in.flushMode != 4U))
        return false;
    if (in.flushMode == 0U || in.flushMode == 2U)
    {
        if (in.heightMm != 0.0 || in.centerReturnFeedMmMin != 0.0) return false;
    }
    else if (!(in.heightMm > 0.0) || in.heightMm > MaximumFlushHeightMm ||
             (in.flushMode == 4U && in.heightMm < in.radiusMm*0.5)) return false;
    double travel = 0.0;
    coordinateScale = 1.0;
    for (unsigned j = 0U; j < 3U; ++j)
    {
        delta[j] = in.end[j] - in.start[j];
        if (!Finite(delta[j])) return false;
        travel = std::hypot(travel,delta[j]);
        // A conservative envelope of the continuous growing curve and all
        // DEMO legs must remain finite, even between the generated endpoints.
        const double low = (std::min)(in.start[j],in.end[j]) - (j < 2U ? in.radiusMm : 0.0);
        const double high = (std::max)(in.start[j],in.end[j]) +
            (j < 2U ? in.radiusMm : in.heightMm);
        if (!Finite(low) || !Finite(high) || !Finite(high-low)) return false;
        coordinateScale = (std::max)(coordinateScale,(std::max)(std::fabs(low),std::fabs(high)));
    }
    return Finite(travel) && travel <= MaximumCenterTravelMm;
}
} // namespace Detail

// Failure replaces out with an empty plan. Do not concurrently change the
// borrowed input/output while this allocation-free routine is running.
inline bool Build(const PlanInput& in, ProgramPlan& out) noexcept
{
    // Copy first, so a caller may safely rebuild directly from out.input.
    const PlanInput frozen = in;
    out = ProgramPlan{};
    Point3 delta{};
    double coordinateScale = 0.0;
    if (!Detail::ValidInput(frozen,delta,coordinateScale)) return false;
    ProgramPlan p{};
    p.version = Version; p.modeP = ModeP11; p.input = frozen;
    p.boundsMin = frozen.start; p.boundsMax = frozen.start;
    p.flushBeginIndex = NoIndex; p.flushEndIndex = NoIndex;
    p.flushLegIndices.fill(NoIndex);
    constexpr double pi = 3.141592653589793238462643383279502884;
    for (unsigned k = 1U; k <= GrowSegments; ++k)
    {
        const double t = static_cast<double>(k)/static_cast<double>(GrowSegments);
        Endpoint e{};
        e.phase = Phase::Grow; e.growIndex = k; e.radiusMm = frozen.radiusMm*t;
        e.feedMmMin = frozen.feedMmMin;
        for (unsigned j = 0U; j < 3U; ++j)
            e.center[j] = k == GrowSegments ? frozen.end[j] : frozen.start[j] + delta[j]*t;
        double cosine = 0.0, sine = 0.0;
        // Exact cardinal samples avoid artificial residual motion at the
        // half-turn, closure and the B4 45-degree construction.
        if (k == 6U) sine = 1.0;
        else if (k == 12U) cosine = -1.0;
        else if (k == 18U) sine = -1.0;
        else if (k == 24U) cosine = 1.0;
        else
        {
            const double angle = (2.0*pi)*t;
            cosine = std::cos(angle); sine = std::sin(angle);
        }
        e.orbit = {{e.radiusMm*cosine,e.radiusMm*sine,0.0}};
        for (unsigned j = 0U; j < 3U; ++j) e.position[j] = e.center[j]+e.orbit[j];
        if (!Detail::Append(p,e,false)) return false;
        ++p.growCount;
        if (k == FlushSample && (frozen.flushMode == 3U || frozen.flushMode == 4U))
        {
            const Endpoint saved = e; // Resume uses these exact stored bits.
            Endpoint leg = saved;
            leg.position = saved.center;
            leg.phase = Phase::ToCenter;
            leg.feedMmMin = frozen.centerReturnFeedMmMin > 0.0 ? frozen.centerReturnFeedMmMin : frozen.feedMmMin;
            if (frozen.flushMode == 4U) leg.position[2] += saved.radiusMm;
            if (!Detail::Append(p,leg,true)) return false;
            const Endpoint upper = leg;
            leg.phase = Phase::Lift;
            leg.feedMmMin = frozen.feedMmMin;
            leg.position = saved.center;
            leg.position[2] += frozen.heightMm;
            // H==r is valid B4. Coalesce the two zero-length apex legs;
            // metadata describes only actual nonzero CNC segments.
            const bool apexDistinct = !Detail::SamePoint(upper.position,leg.position);
            if (apexDistinct)
            {
                if (!Detail::Append(p,leg,true)) return false;
                leg = upper; leg.phase = Phase::ReturnCenter; leg.feedMmMin = frozen.feedMmMin;
                if (!Detail::Append(p,leg,true)) return false;
            }
            else if (frozen.flushMode != 4U) return false;
            leg = saved; leg.phase = Phase::Resume;
            if (!Detail::Append(p,leg,true)) return false;
        }
    }
    Endpoint finish{};
    finish.position = frozen.end; finish.center = frozen.end;
    finish.phase = Phase::FinishCenter; finish.growIndex = GrowSegments;
    finish.feedMmMin = frozen.centerReturnFeedMmMin > 0.0 ? frozen.centerReturnFeedMmMin : frozen.feedMmMin;
    if (!Detail::Append(p,finish,false)) return false;
    // For q(t)=C(t)+Rt(cos(2*pi*t),sin(2*pi*t),0), C''=0 and
    // |q''| <= R*((2*pi)^2+4*pi) < 53R (pi<22/7).
    // Linear interpolation on h=1/24 has error <= max|q''|*h*h/8.
    // The additional epsilon budget covers endpoint arithmetic at this frame
    // scale; this is not a servo tracking or transformed-machine error bound.
    const double geometric = frozen.radiusMm*(53.0/(8.0*GrowSegments*GrowSegments));
    const double arithmetic = 64.0*std::numeric_limits<double>::epsilon()*coordinateScale;
    p.chordErrorBoundMm = std::nextafter(geometric+arithmetic,std::numeric_limits<double>::infinity());
    if (!Detail::Finite(p.chordErrorBoundMm)) return false;
    out = p;
    return true;
}

// Rebuild and compare every public field, including unused slots, so tampered
// phase, resume, bounds or geometry metadata cannot look like a valid plan.
inline bool Valid(const ProgramPlan& p) noexcept
{
    ProgramPlan expected{};
    if (!Build(p.input,expected) || p.version != expected.version || p.modeP != expected.modeP ||
        !Detail::SameInput(p.input,expected.input) || p.count != expected.count ||
        p.growCount != expected.growCount || p.flushBeginIndex != expected.flushBeginIndex ||
        p.flushEndIndex != expected.flushEndIndex || p.flushLegIndices != expected.flushLegIndices ||
        p.flushLegCount != expected.flushLegCount || p.chordErrorBoundMm != expected.chordErrorBoundMm ||
        !Detail::SamePoint(p.boundsMin,expected.boundsMin) || !Detail::SamePoint(p.boundsMax,expected.boundsMax))
        return false;
    for (unsigned i = 0U; i < MaximumPoints; ++i)
        if (!Detail::SameEndpoint(p.points[i],expected.points[i])) return false;
    return true;
}

inline bool FrameSupported(const NCTranslationSnapshot& s) noexcept
{
    if (!(IsNCTranslationSnapshotValid(s) && s.rotationPlane == 17 &&
        s.cutterMode == 40 && s.polarMode == 15 && s.workMode == 169 &&
        s.workWCode == 0 && s.axisIdentity.eccentricEnabled == 0U)) return false;
    for (unsigned a = 0U; a < 3U; ++a)
        if (s.axisIdentity.exists[a] != 1U || s.axisIdentity.axisType[a] != 0U ||
            s.axisIdentity.nativeUnit[a] != 1U) return false;
    return true;
}

// Geometry-only B2 staging. It grants no permission for direct G76 execution;
// NCG76Mixed owns the required retained-path reverse/return proof.
inline bool BuildNativeBase(const ProgramPlan& program, const NCTranslationSnapshot& translation,
    const Point8& startMCS, NativePlan& out) noexcept
{
    // Locals also permit safe rebuild from fields of out itself.
    const ProgramPlan frozenProgram = program;
    const NCTranslationSnapshot frozenTranslation = translation;
    const Point8 frozenStart = startMCS;
    out = NativePlan{};
    if (!Valid(frozenProgram) || !FrameSupported(frozenTranslation) ||
        ((frozenProgram.input.flushMode == 3U || frozenProgram.input.flushMode == 4U) &&
            (frozenTranslation.mirrorMask & 4U) != 0U)) return false;
    for (unsigned a = 0U; a < 8U; ++a)
        if (!Detail::Finite(frozenStart[a])) return false;
    const double scale = frozenTranslation.scalingMode == 51 ? frozenTranslation.scalingFactor : 1.0;
    double travel = 0.0;
    for (unsigned a = 0U; a < 3U; ++a)
        travel = std::hypot(travel,frozenProgram.input.end[a]-frozenProgram.input.start[a]);
    if (!Detail::Finite(scale) || !(scale > 0.0) ||
        !Detail::Finite(travel*scale) || travel*scale > MaximumCenterTravelMm ||
        !Detail::Finite(frozenProgram.input.radiusMm*scale) ||
        frozenProgram.input.radiusMm*scale > MaximumRadiusMm ||
        !Detail::Finite(frozenProgram.input.heightMm*scale) ||
        frozenProgram.input.heightMm*scale > MaximumFlushHeightMm) return false;
    double baselineProgram[8] = {};
    NCTranslationInversePoint(frozenTranslation,frozenStart.data(),baselineProgram);
    for (unsigned a = 0U; a < 8U; ++a)
        if (!Detail::Finite(baselineProgram[a])) return false;
    if (frozenTranslation.distanceMode == 90)
    {
        for (unsigned a = 0U; a < 3U; ++a)
            if (frozenProgram.input.start[a] != baselineProgram[a]) return false;
    }
    else
    {
        for (unsigned a = 0U; a < 3U; ++a)
            if (frozenProgram.input.start[a] != 0.0) return false;
    }
    NativePlan p{};
    p.program = frozenProgram; p.translation = frozenTranslation;
    p.startMCS = frozenStart; p.count = frozenProgram.count;
    p.translationGeneration = frozenTranslation.generation;
    const bool hasXYZ[8] = {true,true,true,false,false,false,false,false};
    Point3 previous{{frozenStart[0],frozenStart[1],frozenStart[2]}};
    for (unsigned i = 0U; i < p.count; ++i)
    {
        double author[8] = {}, machine[8] = {};
        if (frozenTranslation.distanceMode == 91)
        {
            for (unsigned a = 0U; a < 3U; ++a)
                author[a] = frozenProgram.points[i].position[a]-frozenProgram.input.start[a];
            if (!TryNCTranslationIncrementalTarget(frozenTranslation,frozenStart.data(),author,hasXYZ,machine))
                return false;
        }
        else
        {
            for (unsigned a = 0U; a < 8U; ++a) author[a] = baselineProgram[a];
            for (unsigned a = 0U; a < 3U; ++a) author[a] = frozenProgram.points[i].position[a];
            NCTranslationForwardPoint(frozenTranslation,author,machine);
        }
        for (unsigned a = 0U; a < 8U; ++a)
        {
            p.pointsMCS[i][a] = a < 3U ? machine[a] : frozenStart[a];
            if (!Detail::Finite(p.pointsMCS[i][a])) return false;
        }
        p.feedMmMin[i] = frozenProgram.points[i].feedMmMin;
        const Point3 next{{p.pointsMCS[i][0],p.pointsMCS[i][1],p.pointsMCS[i][2]}};
        if (!Detail::NonzeroFiniteLeg(previous,next)) return false;
        previous = next;
    }
    out = p;
    return true;
}

inline bool ValidNativeBase(const NativePlan& p) noexcept
{
    NativePlan expected{};
    if (!BuildNativeBase(p.program,p.translation,p.startMCS,expected) ||
        p.version != expected.version || p.modeP != expected.modeP || p.count != expected.count ||
        p.translationGeneration != expected.translationGeneration) return false;
    for (unsigned a = 0U; a < 8U; ++a)
        if (std::memcmp(&p.startMCS[a],&expected.startMCS[a],sizeof(double)) != 0) return false;
    for (unsigned i = 0U; i < MaximumPoints; ++i)
    {
        if (std::memcmp(&p.feedMmMin[i],&expected.feedMmMin[i],sizeof(double)) != 0) return false;
        for (unsigned a = 0U; a < 8U; ++a)
            if (std::memcmp(&p.pointsMCS[i][a],&expected.pointsMCS[i][a],sizeof(double)) != 0) return false;
    }
    return true;
}

inline bool BuildNative(const ProgramPlan& program, const NCTranslationSnapshot& translation,
    const Point8& startMCS, NativePlan& out) noexcept
{
    if (program.input.flushMode == 2U) { out = NativePlan{}; return false; }
    return BuildNativeBase(program,translation,startMCS,out);
}

inline bool ValidNative(const NativePlan& p) noexcept
{
    return p.program.input.flushMode != 2U && ValidNativeBase(p);
}


// EDM173 FIX2 opt-in validation storage. Allocate once on the owning caller's
// startup heap; never share it across concurrent NC/RT owners. Legacy overloads
// above deliberately retain their original behavior and implementation.
struct ValidationScratch
{
    ProgramPlan program{};
    NativePlan native{};
};
namespace Detail
{
inline bool ValidationStorageOverlaps(const void* a,std::size_t an,const void* b,std::size_t bn) noexcept
{
    const auto x=reinterpret_cast<std::uintptr_t>(a),y=reinterpret_cast<std::uintptr_t>(b);
    return x<=y ? y-x<an : x-y<bn;
}
inline void ClearValidationProgram(ProgramPlan& p) noexcept
{
    p.version=p.modeP=0U;p.input=PlanInput{};p.points.fill(Endpoint{});
    p.count=p.growCount=p.flushBeginIndex=p.flushEndIndex=p.flushLegCount=0U;
    p.flushLegIndices.fill(0U);p.chordErrorBoundMm=0.0;p.boundsMin.fill(0.0);p.boundsMax.fill(0.0);
}
inline void ClearValidationNative(NativePlan& p) noexcept
{
    ClearValidationProgram(p.program);p.translation=NCTranslationSnapshot{};
    p.version=Version;p.modeP=ModeP11;p.count=0U;p.startMCS.fill(0.0);
    p.pointsMCS.fill(Point8{});p.feedMmMin.fill(0.0);p.translationGeneration=0ULL;
}
inline bool ClearValidationProgramFalse(ProgramPlan& p) noexcept {ClearValidationProgram(p);return false;}
inline bool ClearValidationNativeFalse(NativePlan& p) noexcept {ClearValidationNative(p);return false;}
inline bool BuildValidationProgram(const PlanInput& in, ProgramPlan& out) noexcept
{
    // Copy first, so a caller may safely rebuild directly from out.input.
    const PlanInput frozen = in;
    ClearValidationProgram(out);
    Point3 delta{};
    double coordinateScale = 0.0;
    if (!Detail::ValidInput(frozen,delta,coordinateScale)) return ClearValidationProgramFalse(out);
    ProgramPlan& p = out;
    p.version = Version; p.modeP = ModeP11; p.input = frozen;
    p.boundsMin = frozen.start; p.boundsMax = frozen.start;
    p.flushBeginIndex = NoIndex; p.flushEndIndex = NoIndex;
    p.flushLegIndices.fill(NoIndex);
    constexpr double pi = 3.141592653589793238462643383279502884;
    for (unsigned k = 1U; k <= GrowSegments; ++k)
    {
        const double t = static_cast<double>(k)/static_cast<double>(GrowSegments);
        Endpoint e{};
        e.phase = Phase::Grow; e.growIndex = k; e.radiusMm = frozen.radiusMm*t;
        e.feedMmMin = frozen.feedMmMin;
        for (unsigned j = 0U; j < 3U; ++j)
            e.center[j] = k == GrowSegments ? frozen.end[j] : frozen.start[j] + delta[j]*t;
        double cosine = 0.0, sine = 0.0;
        // Exact cardinal samples avoid artificial residual motion at the
        // half-turn, closure and the B4 45-degree construction.
        if (k == 6U) sine = 1.0;
        else if (k == 12U) cosine = -1.0;
        else if (k == 18U) sine = -1.0;
        else if (k == 24U) cosine = 1.0;
        else
        {
            const double angle = (2.0*pi)*t;
            cosine = std::cos(angle); sine = std::sin(angle);
        }
        e.orbit = {{e.radiusMm*cosine,e.radiusMm*sine,0.0}};
        for (unsigned j = 0U; j < 3U; ++j) e.position[j] = e.center[j]+e.orbit[j];
        if (!Detail::Append(p,e,false)) return ClearValidationProgramFalse(out);
        ++p.growCount;
        if (k == FlushSample && (frozen.flushMode == 3U || frozen.flushMode == 4U))
        {
            const Endpoint saved = e; // Resume uses these exact stored bits.
            Endpoint leg = saved;
            leg.position = saved.center;
            leg.phase = Phase::ToCenter;
            leg.feedMmMin = frozen.centerReturnFeedMmMin > 0.0 ? frozen.centerReturnFeedMmMin : frozen.feedMmMin;
            if (frozen.flushMode == 4U) leg.position[2] += saved.radiusMm;
            if (!Detail::Append(p,leg,true)) return ClearValidationProgramFalse(out);
            const Endpoint upper = leg;
            leg.phase = Phase::Lift;
            leg.feedMmMin = frozen.feedMmMin;
            leg.position = saved.center;
            leg.position[2] += frozen.heightMm;
            // H==r is valid B4. Coalesce the two zero-length apex legs;
            // metadata describes only actual nonzero CNC segments.
            const bool apexDistinct = !Detail::SamePoint(upper.position,leg.position);
            if (apexDistinct)
            {
                if (!Detail::Append(p,leg,true)) return ClearValidationProgramFalse(out);
                leg = upper; leg.phase = Phase::ReturnCenter; leg.feedMmMin = frozen.feedMmMin;
                if (!Detail::Append(p,leg,true)) return ClearValidationProgramFalse(out);
            }
            else if (frozen.flushMode != 4U) return ClearValidationProgramFalse(out);
            leg = saved; leg.phase = Phase::Resume;
            if (!Detail::Append(p,leg,true)) return ClearValidationProgramFalse(out);
        }
    }
    Endpoint finish{};
    finish.position = frozen.end; finish.center = frozen.end;
    finish.phase = Phase::FinishCenter; finish.growIndex = GrowSegments;
    finish.feedMmMin = frozen.centerReturnFeedMmMin > 0.0 ? frozen.centerReturnFeedMmMin : frozen.feedMmMin;
    if (!Detail::Append(p,finish,false)) return ClearValidationProgramFalse(out);
    // For q(t)=C(t)+Rt(cos(2*pi*t),sin(2*pi*t),0), C''=0 and
    // |q''| <= R*((2*pi)^2+4*pi) < 53R (pi<22/7).
    // Linear interpolation on h=1/24 has error <= max|q''|*h*h/8.
    // The additional epsilon budget covers endpoint arithmetic at this frame
    // scale; this is not a servo tracking or transformed-machine error bound.
    const double geometric = frozen.radiusMm*(53.0/(8.0*GrowSegments*GrowSegments));
    const double arithmetic = 64.0*std::numeric_limits<double>::epsilon()*coordinateScale;
    p.chordErrorBoundMm = std::nextafter(geometric+arithmetic,std::numeric_limits<double>::infinity());
    if (!Detail::Finite(p.chordErrorBoundMm)) return ClearValidationProgramFalse(out);
    return true;
}

inline bool ValidateProgramWithStorage(const ProgramPlan& p,ProgramPlan& expected) noexcept
{
    if (!BuildValidationProgram(p.input,expected) || p.version != expected.version || p.modeP != expected.modeP ||
        !Detail::SameInput(p.input,expected.input) || p.count != expected.count ||
        p.growCount != expected.growCount || p.flushBeginIndex != expected.flushBeginIndex ||
        p.flushEndIndex != expected.flushEndIndex || p.flushLegIndices != expected.flushLegIndices ||
        p.flushLegCount != expected.flushLegCount || p.chordErrorBoundMm != expected.chordErrorBoundMm ||
        !Detail::SamePoint(p.boundsMin,expected.boundsMin) || !Detail::SamePoint(p.boundsMax,expected.boundsMax))
        return false;
    for (unsigned i = 0U; i < MaximumPoints; ++i)
        if (!Detail::SameEndpoint(p.points[i],expected.points[i])) return false;
    return true;
}

inline bool BuildValidationNative(const ProgramPlan& program, const NCTranslationSnapshot& translation,
    const Point8& startMCS, NativePlan& out) noexcept
{
    // Private validation builder: caller has rejected overlap and fully
    // validated program. Every native endpoint is regenerated independently.
    const ProgramPlan& frozenProgram = program;
    const NCTranslationSnapshot& frozenTranslation = translation;
    const Point8& frozenStart = startMCS;
    ClearValidationNative(out);
    if (!FrameSupported(frozenTranslation) ||
        ((frozenProgram.input.flushMode == 3U || frozenProgram.input.flushMode == 4U) &&
            (frozenTranslation.mirrorMask & 4U) != 0U)) return ClearValidationNativeFalse(out);
    for (unsigned a = 0U; a < 8U; ++a)
        if (!Detail::Finite(frozenStart[a])) return ClearValidationNativeFalse(out);
    const double scale = frozenTranslation.scalingMode == 51 ? frozenTranslation.scalingFactor : 1.0;
    double travel = 0.0;
    for (unsigned a = 0U; a < 3U; ++a)
        travel = std::hypot(travel,frozenProgram.input.end[a]-frozenProgram.input.start[a]);
    if (!Detail::Finite(scale) || !(scale > 0.0) ||
        !Detail::Finite(travel*scale) || travel*scale > MaximumCenterTravelMm ||
        !Detail::Finite(frozenProgram.input.radiusMm*scale) ||
        frozenProgram.input.radiusMm*scale > MaximumRadiusMm ||
        !Detail::Finite(frozenProgram.input.heightMm*scale) ||
        frozenProgram.input.heightMm*scale > MaximumFlushHeightMm) return ClearValidationNativeFalse(out);
    double baselineProgram[8] = {};
    NCTranslationInversePoint(frozenTranslation,frozenStart.data(),baselineProgram);
    for (unsigned a = 0U; a < 8U; ++a)
        if (!Detail::Finite(baselineProgram[a])) return ClearValidationNativeFalse(out);
    if (frozenTranslation.distanceMode == 90)
    {
        for (unsigned a = 0U; a < 3U; ++a)
            if (frozenProgram.input.start[a] != baselineProgram[a]) return ClearValidationNativeFalse(out);
    }
    else
    {
        for (unsigned a = 0U; a < 3U; ++a)
            if (frozenProgram.input.start[a] != 0.0) return ClearValidationNativeFalse(out);
    }
    NativePlan& p = out;
    p.program = frozenProgram; p.translation = frozenTranslation;
    p.startMCS = frozenStart; p.count = frozenProgram.count;
    p.translationGeneration = frozenTranslation.generation;
    const bool hasXYZ[8] = {true,true,true,false,false,false,false,false};
    Point3 previous{{frozenStart[0],frozenStart[1],frozenStart[2]}};
    for (unsigned i = 0U; i < p.count; ++i)
    {
        double author[8] = {}, machine[8] = {};
        if (frozenTranslation.distanceMode == 91)
        {
            for (unsigned a = 0U; a < 3U; ++a)
                author[a] = frozenProgram.points[i].position[a]-frozenProgram.input.start[a];
            if (!TryNCTranslationIncrementalTarget(frozenTranslation,frozenStart.data(),author,hasXYZ,machine))
                return ClearValidationNativeFalse(out);
        }
        else
        {
            for (unsigned a = 0U; a < 8U; ++a) author[a] = baselineProgram[a];
            for (unsigned a = 0U; a < 3U; ++a) author[a] = frozenProgram.points[i].position[a];
            NCTranslationForwardPoint(frozenTranslation,author,machine);
        }
        for (unsigned a = 0U; a < 8U; ++a)
        {
            p.pointsMCS[i][a] = a < 3U ? machine[a] : frozenStart[a];
            if (!Detail::Finite(p.pointsMCS[i][a])) return ClearValidationNativeFalse(out);
        }
        p.feedMmMin[i] = frozenProgram.points[i].feedMmMin;
        const Point3 next{{p.pointsMCS[i][0],p.pointsMCS[i][1],p.pointsMCS[i][2]}};
        if (!Detail::NonzeroFiniteLeg(previous,next)) return ClearValidationNativeFalse(out);
        previous = next;
    }
    return true;
}

inline bool ValidateNativeWithStorage(const NativePlan& p,NativePlan& expected) noexcept
{
    if (!BuildValidationNative(p.program,p.translation,p.startMCS,expected) ||
        p.version != expected.version || p.modeP != expected.modeP || p.count != expected.count ||
        p.translationGeneration != expected.translationGeneration) return false;
    for (unsigned a = 0U; a < 8U; ++a)
        if (std::memcmp(&p.startMCS[a],&expected.startMCS[a],sizeof(double)) != 0) return false;
    for (unsigned i = 0U; i < MaximumPoints; ++i)
    {
        if (std::memcmp(&p.feedMmMin[i],&expected.feedMmMin[i],sizeof(double)) != 0) return false;
        for (unsigned a = 0U; a < 8U; ++a)
            if (std::memcmp(&p.pointsMCS[i][a],&expected.pointsMCS[i][a],sizeof(double)) != 0) return false;
    }
    return true;
}
} // namespace Detail
inline void ClearValidationScratch(ValidationScratch& scratch) noexcept
{Detail::ClearValidationProgram(scratch.program);Detail::ClearValidationNative(scratch.native);}
inline bool ClearValidationScratchFalse(ValidationScratch& scratch) noexcept
{ClearValidationScratch(scratch);return false;}
inline bool Valid(const ProgramPlan& p,ValidationScratch& scratch) noexcept
{
    if(Detail::ValidationStorageOverlaps(&p,sizeof(p),&scratch,sizeof(scratch)))return false;
    ClearValidationScratch(scratch);
    return Detail::ValidateProgramWithStorage(p,scratch.program) || ClearValidationScratchFalse(scratch);
}
inline bool ValidNativeBase(const NativePlan& p,ValidationScratch& scratch) noexcept
{
    if(Detail::ValidationStorageOverlaps(&p,sizeof(p),&scratch,sizeof(scratch)))return false;
    ClearValidationScratch(scratch);
    if(!Detail::ValidateProgramWithStorage(p.program,scratch.program) ||
        !Detail::ValidateNativeWithStorage(p,scratch.native))return ClearValidationScratchFalse(scratch);
    return true;
}
inline bool ValidNative(const NativePlan& p,ValidationScratch& scratch) noexcept
{
    if(Detail::ValidationStorageOverlaps(&p,sizeof(p),&scratch,sizeof(scratch)))return false;
    if(p.program.input.flushMode==2U)return ClearValidationScratchFalse(scratch);
    return ValidNativeBase(p,scratch);
}
} // namespace NCG76
