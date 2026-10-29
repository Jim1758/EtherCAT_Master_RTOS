#pragma once

#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

// EDM285 P50 geometry only. This descriptor conveys no motion, GAP, power,
// process-B or I/O authority. Coordinates are in one caller-selected frozen
// Cartesian frame; NC uses program millimetres and maps through a frozen frame.
// K is an axial distance. Linear is the explicitly requested radius law; it
// describes a cone. TrueSphere is a separate, explicit hemisphere option.
// The 24-chord circle is a Phase-1 approximation, not a machining-accuracy
// promise. Each descriptor/layer exposes its sagitta bound. A future runtime
// may consume layer centers/radii as arcs or choose adequate subdivision.
#if defined(__FAST_MATH__) || defined(_M_FP_FAST)
#error G76_sphere_requires_precise_floating_point_semantics
#endif
#if defined(__FINITE_MATH_ONLY__) && (__FINITE_MATH_ONLY__ > 0)
#error G76_sphere_requires_nonfinite_checks
#endif

namespace NCG76Sphere
{
using Point3 = std::array<double, 3U>;
constexpr std::uint32_t Version = 1U;
constexpr unsigned ModeP = 50U;
constexpr unsigned MaximumPoints = 32U;
constexpr unsigned CircleSegments = 24U;
constexpr unsigned LayerSegments = CircleSegments + 3U;
constexpr unsigned MaximumLayers = 4096U;
constexpr double Pi = 3.14159265358979323846264338327950288;

enum class RadiusLaw : std::uint8_t { Linear = 0U, TrueSphere = 1U };

struct Input
{
    Point3 start{}, end{};
    double radiusMm = 0.0, layerStepMm = 0.0, feedMmMin = 0.0;
    RadiusLaw radiusLaw = RadiusLaw::Linear;
};

// Geometry admission limits, not machine commissioning or execution limits.
// Callers can tighten these. The compiled 4096-layer/32-point bounds cannot be
// raised by an injected Limits object. Coordinate error has a fixed mm ceiling;
// large origins never earn a larger acceptable physical error.
struct Limits
{
    double minimumRadiusMm = 1e-9;
    double minimumLayerStepMm = 1e-9;
    double minimumTravelMm = 1e-9;
    double minimumFeedMmMin = 1e-9;
    double maximumRadiusMm = 1000.0;
    double maximumLayerStepMm = 1000.0;
    double maximumTravelMm = 10000.0;
    double maximumFeedMmMin = 100000.0;
    double maximumCoordinateErrorMm = 1e-7;
    unsigned maximumLayers = MaximumLayers;
};

struct Descriptor
{
    Input input{};
    Limits limits{};
    Point3 axis{}, u{}, v{};
    double travelMm = 0.0;
    double chordErrorBoundMm = 0.0;
    double coordinateErrorBoundMm = 0.0;
    unsigned circleLayerCount = 0U; // Positive-radius rings; terminal is separate.
    std::uint32_t version = 0U;
    std::uint64_t fingerprint = 0ULL; // Identity hint only, never validation alone.
    bool valid = false;
};

struct Layer
{
    unsigned index = 0U;
    double depthMm = 0.0, radiusMm = 0.0, chordErrorBoundMm = 0.0;
    Point3 center{};
    bool terminal = false;
};

enum class StepKind : std::uint8_t
{
    InitialDepth, InitialRetreat, CircleEntry, CircleChord, ReturnCenter, AdvanceCenter
};

struct Cursor
{
    bool initial = false;
    unsigned layerIndex = 0U, segmentIndex = 0U;
};

struct Segment
{
    Point3 start{}, end{}, centerStart{}, centerEnd{};
    StepKind kind = StepKind::InitialDepth;
    unsigned layerIndex = 0U, segmentIndex = 0U;
};

struct Endpoint
{
    Point3 position{}, center{};
    StepKind kind = StepKind::InitialDepth;
    unsigned layerIndex = 0U, segmentIndex = 0U;
};

struct Chunk
{
    Point3 start{};
    std::array<Endpoint, MaximumPoints> points{};
    unsigned count = 0U, layerIndex = 0U;
    bool initial = false;
    double chordErrorBoundMm = 0.0;
};

namespace Detail
{
inline bool Finite(double x) noexcept { return std::isfinite(x) != 0; }
inline bool FinitePoint(const Point3& p) noexcept
{ return Finite(p[0]) && Finite(p[1]) && Finite(p[2]); }
inline bool SameDouble(double a, double b) noexcept
{ return std::memcmp(&a, &b, sizeof(double)) == 0; }
inline bool SamePoint(const Point3& a, const Point3& b) noexcept
{
    for (unsigned i = 0U; i < 3U; ++i) if (!SameDouble(a[i], b[i])) return false;
    return true;
}
inline bool SameInput(const Input& a, const Input& b) noexcept
{
    return SamePoint(a.start,b.start) && SamePoint(a.end,b.end) &&
        SameDouble(a.radiusMm,b.radiusMm) && SameDouble(a.layerStepMm,b.layerStepMm) &&
        SameDouble(a.feedMmMin,b.feedMmMin) && a.radiusLaw == b.radiusLaw;
}
inline bool SameLimits(const Limits& a, const Limits& b) noexcept
{
    return SameDouble(a.minimumRadiusMm,b.minimumRadiusMm) &&
        SameDouble(a.minimumLayerStepMm,b.minimumLayerStepMm) &&
        SameDouble(a.minimumTravelMm,b.minimumTravelMm) &&
        SameDouble(a.minimumFeedMmMin,b.minimumFeedMmMin) &&
        SameDouble(a.maximumRadiusMm,b.maximumRadiusMm) &&
        SameDouble(a.maximumLayerStepMm,b.maximumLayerStepMm) &&
        SameDouble(a.maximumTravelMm,b.maximumTravelMm) &&
        SameDouble(a.maximumFeedMmMin,b.maximumFeedMmMin) &&
        SameDouble(a.maximumCoordinateErrorMm,b.maximumCoordinateErrorMm) &&
        a.maximumLayers == b.maximumLayers;
}
inline bool PositiveRange(double low, double high) noexcept
{ return Finite(low) && Finite(high) && low > 0.0 && high >= low; }
inline bool ValidLimits(const Limits& p) noexcept
{
    return PositiveRange(p.minimumRadiusMm,p.maximumRadiusMm) &&
        PositiveRange(p.minimumLayerStepMm,p.maximumLayerStepMm) &&
        PositiveRange(p.minimumTravelMm,p.maximumTravelMm) &&
        PositiveRange(p.minimumFeedMmMin,p.maximumFeedMmMin) &&
        Finite(p.maximumCoordinateErrorMm) && p.maximumCoordinateErrorMm > 0.0 &&
        p.maximumLayers > 0U && p.maximumLayers <= MaximumLayers;
}
inline double Length(const Point3& p) noexcept
{ return std::hypot(std::hypot(p[0],p[1]),p[2]); }
inline bool Normalize(Point3& p) noexcept
{
    const double length = Length(p);
    if (!Finite(length) || length <= 0.0) return false;
    for (unsigned i = 0U; i < 3U; ++i) p[i] /= length;
    return FinitePoint(p);
}
inline Point3 Cross(const Point3& a, const Point3& b) noexcept
{
    return Point3{{a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]}};
}
inline bool NonzeroLeg(const Point3& a, const Point3& b) noexcept
{
    Point3 delta{};
    for (unsigned i = 0U; i < 3U; ++i) delta[i] = b[i]-a[i];
    const double length = Length(delta);
    return FinitePoint(a) && FinitePoint(b) && Finite(length) && length > 0.0;
}
inline bool OffsetPoint(const Point3& base, const Point3& delta,
    double errorBound, Point3& point) noexcept
{
    for (unsigned i = 0U; i < 3U; ++i)
    {
        point[i] = base[i] + delta[i];
        if (!Finite(point[i]) || !Finite(delta[i]) ||
            std::fabs((point[i]-base[i])-delta[i]) > errorBound) return false;
    }
    return true;
}
inline double RadiusAt(const Descriptor& p, double depth) noexcept
{
    if (depth == 0.0) return p.input.radiusMm;
    if (depth == p.input.radiusMm) return 0.0;
    if (p.input.radiusLaw == RadiusLaw::Linear) return p.input.radiusMm-depth;
    // R*sqrt((R-s)/R)*sqrt(1+s/R) avoids R^2 and preserves the small
    // complement near the terminal layer better than 1-s/R subtraction.
    return p.input.radiusMm * std::sqrt((p.input.radiusMm-depth)/p.input.radiusMm) *
        std::sqrt(1.0+depth/p.input.radiusMm);
}
inline bool LayerUnchecked(const Descriptor& p, unsigned index, Layer& out) noexcept
{
    out = Layer{};
    if (index > p.circleLayerCount) return false;
    out.index = index;
    out.terminal = index == p.circleLayerCount;
    if (out.terminal)
    {
        out.depthMm = p.input.radiusMm;
        out.center = p.input.end; // Exact authored endpoint, no reconstructed drift.
        return true;
    }
    out.depthMm = static_cast<double>(index)*p.input.layerStepMm;
    out.radiusMm = RadiusAt(p,out.depthMm);
    out.chordErrorBoundMm = out.radiusMm*(1.0-std::cos(Pi/CircleSegments));
    if (!Finite(out.depthMm) || out.depthMm < 0.0 || out.depthMm >= p.input.radiusMm ||
        !Finite(out.radiusMm) || out.radiusMm <= 0.0 || !Finite(out.chordErrorBoundMm)) return false;
    Point3 delta{};
    for (unsigned i = 0U; i < 3U; ++i)
        delta[i] = -(p.input.radiusMm-out.depthMm)*p.axis[i];
    return OffsetPoint(p.input.end,delta,p.coordinateErrorBoundMm,out.center);
}
inline bool CirclePoint(const Descriptor& p, const Layer& layer,
    unsigned sample, Point3& out) noexcept
{
    if (sample > CircleSegments) return false;
    // Exact cardinal coefficients and exact closure are contractual.
    double c = 0.0, s = 0.0;
    const unsigned k = sample % CircleSegments;
    if (k == 0U) c = 1.0;
    else if (k == CircleSegments/4U) s = 1.0;
    else if (k == CircleSegments/2U) c = -1.0;
    else if (k == 3U*CircleSegments/4U) s = -1.0;
    else
    {
        const double angle = 2.0*Pi*static_cast<double>(k)/CircleSegments;
        c = std::cos(angle); s = std::sin(angle);
    }
    Point3 delta{};
    for (unsigned i = 0U; i < 3U; ++i)
        delta[i] = layer.radiusMm*(c*p.u[i]+s*p.v[i]);
    return OffsetPoint(layer.center,delta,p.coordinateErrorBoundMm,out);
}
inline void HashWord(std::uint64_t& h, std::uint64_t word) noexcept
{
    for (unsigned i = 0U; i < 8U; ++i)
    { h ^= (word >> (8U*i)) & 0xffULL; h *= 1099511628211ULL; }
}
inline void HashDouble(std::uint64_t& h, double x) noexcept
{
    std::uint64_t bits = 0ULL;
    static_assert(sizeof(bits)==sizeof(x),"P50 requires 64-bit double");
    std::memcpy(&bits,&x,sizeof(bits)); HashWord(h,bits);
}
inline std::uint64_t Fingerprint(const Descriptor& p) noexcept
{
    std::uint64_t h = 1469598103934665603ULL;
    HashWord(h,0x4737365035303238ULL); HashWord(h,p.version);
    for (unsigned i=0U;i<3U;++i) { HashDouble(h,p.input.start[i]); HashDouble(h,p.input.end[i]); }
    HashDouble(h,p.input.radiusMm); HashDouble(h,p.input.layerStepMm); HashDouble(h,p.input.feedMmMin);
    HashWord(h,static_cast<std::uint8_t>(p.input.radiusLaw));
    HashDouble(h,p.limits.minimumRadiusMm); HashDouble(h,p.limits.minimumLayerStepMm);
    HashDouble(h,p.limits.minimumTravelMm); HashDouble(h,p.limits.minimumFeedMmMin);
    HashDouble(h,p.limits.maximumRadiusMm); HashDouble(h,p.limits.maximumLayerStepMm);
    HashDouble(h,p.limits.maximumTravelMm); HashDouble(h,p.limits.maximumFeedMmMin);
    HashDouble(h,p.limits.maximumCoordinateErrorMm); HashWord(h,p.limits.maximumLayers);
    for (unsigned i=0U;i<3U;++i) { HashDouble(h,p.axis[i]); HashDouble(h,p.u[i]); HashDouble(h,p.v[i]); }
    HashDouble(h,p.travelMm); HashDouble(h,p.chordErrorBoundMm);
    HashDouble(h,p.coordinateErrorBoundMm); HashWord(h,p.circleLayerCount);
    return h;
}
inline bool Derive(const Input& input, const Limits& limits, Descriptor& out) noexcept
{
    out.input = input; out.limits = limits;
    if (!ValidLimits(limits) || !FinitePoint(input.start) || !FinitePoint(input.end) ||
        !Finite(input.radiusMm) || input.radiusMm < limits.minimumRadiusMm || input.radiusMm > limits.maximumRadiusMm ||
        !Finite(input.layerStepMm) || input.layerStepMm < limits.minimumLayerStepMm || input.layerStepMm > limits.maximumLayerStepMm ||
        !Finite(input.feedMmMin) || input.feedMmMin < limits.minimumFeedMmMin || input.feedMmMin > limits.maximumFeedMmMin ||
        (input.radiusLaw != RadiusLaw::Linear && input.radiusLaw != RadiusLaw::TrueSphere)) return false;
    for (unsigned i = 0U; i < 3U; ++i) out.axis[i] = input.end[i]-input.start[i];
    out.travelMm = Length(out.axis);
    if (!FinitePoint(out.axis) || !Finite(out.travelMm) || out.travelMm < limits.minimumTravelMm ||
        out.travelMm > limits.maximumTravelMm || !Normalize(out.axis)) return false;

    // Keep the signed machining axis. Only the basis normal is canonicalized,
    // so changing +Z to -Z does not accidentally reverse the retreat direction.
    Point3 normal = out.axis;
    for (int i = 2; i >= 0; --i)
        if (normal[static_cast<unsigned>(i)] != 0.0)
        {
            if (normal[static_cast<unsigned>(i)] < 0.0)
                for (unsigned j=0U;j<3U;++j) normal[j] = -normal[j];
            break;
        }
    unsigned least = 0U;
    for (unsigned i = 1U; i < 3U; ++i)
        if (std::fabs(normal[i]) < std::fabs(normal[least])) least = i;
    Point3 reference{}; reference[least] = 1.0;
    out.u = Cross(reference,normal);
    if (!Normalize(out.u)) return false;
    out.v = Cross(normal,out.u);
    if (!Normalize(out.v)) return false;

    double ratio = input.radiusMm/input.layerStepMm;
    if (!Finite(ratio) || ratio <= 0.0) return false;
    // Decimal-looking R/K pairs can straddle an integer in binary. Within eight
    // double epsilons of the quotient scale, treat the terminal as that integer.
    // This rule only determines the count; all interior depths remain index*K.
    const double nearest = std::floor(ratio+0.5);
    const double tolerance = 8.0*std::numeric_limits<double>::epsilon()*(std::max)(1.0,ratio);
    if (nearest >= 1.0 && std::fabs(ratio-nearest) <= tolerance) ratio = nearest;
    const double count = std::ceil(ratio);
    if (!Finite(count) || count < 1.0 || count > limits.maximumLayers) return false;
    out.circleLayerCount = static_cast<unsigned>(count);
    const double lastDepth = static_cast<double>(out.circleLayerCount-1U)*input.layerStepMm;
    const double lastGap = input.radiusMm-lastDepth;
    if (!Finite(lastDepth) || !Finite(lastGap) || lastGap <= 0.0) return false;
    out.chordErrorBoundMm = input.radiusMm*(1.0-std::cos(Pi/CircleSegments));
    if (!Finite(out.chordErrorBoundMm) || out.chordErrorBoundMm <= 0.0) return false;

    // A conservative fixed-mm arithmetic envelope also prevents indistinct
    // tiny rings, chords and final short steps at very large absolute origins.
    double envelope = 0.0;
    for (unsigned i=0U;i<3U;++i)
    {
        const double extent = std::fabs(input.end[i])+2.0*input.radiusMm;
        if (!Finite(extent)) return false;
        envelope = (std::max)(envelope,(std::max)(std::fabs(input.start[i]),extent));
    }
    out.coordinateErrorBoundMm = envelope*(64.0*std::numeric_limits<double>::epsilon());
    const double smallestRadius = RadiusAt(out,lastDepth);
    const double smallestChord = 2.0*smallestRadius*std::sin(Pi/CircleSegments);
    const double feature = (std::min)(out.travelMm,(std::min)(input.radiusMm,
        (std::min)(input.layerStepMm,(std::min)(lastGap,smallestChord))));
    if (!Finite(out.coordinateErrorBoundMm) || out.coordinateErrorBoundMm <= 0.0 ||
        out.coordinateErrorBoundMm > limits.maximumCoordinateErrorMm ||
        !Finite(feature) || feature <= 0.0 || out.coordinateErrorBoundMm > feature/64.0) return false;
    out.version = Version; out.valid = true;
    out.fingerprint = Fingerprint(out);
    return true;
}
inline bool SegmentUnchecked(const Descriptor& p, const Cursor& cursor, Segment& out) noexcept
{
    out = Segment{};
    out.layerIndex = cursor.layerIndex; out.segmentIndex = cursor.segmentIndex;
    Layer layer{};
    if (cursor.initial)
    {
        if (cursor.layerIndex != 0U || cursor.segmentIndex > 1U || !LayerUnchecked(p,0U,layer)) return false;
        out.kind = cursor.segmentIndex == 0U ? StepKind::InitialDepth : StepKind::InitialRetreat;
        out.start = cursor.segmentIndex == 0U ? p.input.start : p.input.end;
        out.end = cursor.segmentIndex == 0U ? p.input.end : layer.center;
        out.centerStart = out.start; out.centerEnd = out.end;
    }
    else
    {
        if (cursor.layerIndex >= p.circleLayerCount || cursor.segmentIndex >= LayerSegments ||
            !LayerUnchecked(p,cursor.layerIndex,layer)) return false;
        out.centerStart = layer.center; out.centerEnd = layer.center;
        if (cursor.segmentIndex == 0U)
        {
            out.kind = StepKind::CircleEntry; out.start = layer.center;
            if (!CirclePoint(p,layer,0U,out.end)) return false;
        }
        else if (cursor.segmentIndex <= CircleSegments)
        {
            out.kind = StepKind::CircleChord;
            if (!CirclePoint(p,layer,cursor.segmentIndex-1U,out.start) ||
                !CirclePoint(p,layer,cursor.segmentIndex,out.end)) return false;
        }
        else if (cursor.segmentIndex == CircleSegments+1U)
        {
            out.kind = StepKind::ReturnCenter; out.end = layer.center;
            if (!CirclePoint(p,layer,0U,out.start)) return false;
        }
        else
        {
            Layer next{};
            if (!LayerUnchecked(p,cursor.layerIndex+1U,next)) return false;
            out.kind = StepKind::AdvanceCenter; out.start = layer.center; out.end = next.center;
            out.centerEnd = next.center;
        }
    }
    return NonzeroLeg(out.start,out.end);
}
inline void ClearChunk(Chunk& out) noexcept
{
    out.start = Point3{}; out.count = 0U; out.layerIndex = 0U; out.initial = false; out.chordErrorBoundMm = 0.0;
    for (unsigned i=0U;i<MaximumPoints;++i) out.points[i] = Endpoint{};
}
} // namespace Detail

inline bool SameDescriptor(const Descriptor& a, const Descriptor& b) noexcept
{
    return Detail::SameInput(a.input,b.input) && Detail::SameLimits(a.limits,b.limits) &&
        Detail::SamePoint(a.axis,b.axis) && Detail::SamePoint(a.u,b.u) && Detail::SamePoint(a.v,b.v) &&
        Detail::SameDouble(a.travelMm,b.travelMm) && Detail::SameDouble(a.chordErrorBoundMm,b.chordErrorBoundMm) &&
        Detail::SameDouble(a.coordinateErrorBoundMm,b.coordinateErrorBoundMm) &&
        a.circleLayerCount == b.circleLayerCount && a.version == b.version &&
        a.fingerprint == b.fingerprint && a.valid == b.valid;
}

inline bool Build(const Input& input, const Limits& limits, Descriptor& out) noexcept
{
    // The small inputs may legally be out.input/out.limits. Freeze before clear.
    const Input frozenInput = input;
    const Limits frozenLimits = limits;
    out = Descriptor{};
    if (!Detail::Derive(frozenInput,frozenLimits,out)) { out = Descriptor{}; return false; }
    return true;
}
inline bool Build(const Input& input, Descriptor& out) noexcept
{ return Build(input,Limits{},out); }

inline bool Valid(const Descriptor& p) noexcept
{
    if (!p.valid || p.version != Version) return false;
    Descriptor rebuilt{};
    return Detail::Derive(p.input,p.limits,rebuilt) && SameDescriptor(p,rebuilt);
}
inline bool ValidateDescriptor(const Descriptor& p) noexcept { return Valid(p); }

inline bool LayerAt(const Descriptor& p, unsigned index, Layer& out) noexcept
{
    out = Layer{};
    if (Valid(p) && Detail::LayerUnchecked(p,index,out)) return true;
    out = Layer{}; return false;
}
inline bool SegmentAt(const Descriptor& p, const Cursor& cursor, Segment& out) noexcept
{
    out = Segment{};
    if (Valid(p) && Detail::SegmentUnchecked(p,cursor,out)) return true;
    out = Segment{}; return false;
}

inline bool Evaluate(const Descriptor& p, const Cursor& cursor, double fraction,
    Point3& position, Point3& center) noexcept
{
    // Output aliases of descriptor fields, or of each other, are rejected
    // without mutation. Normal failure clears both caller-owned output points.
    const std::uintptr_t begin = reinterpret_cast<std::uintptr_t>(&p);
    const std::uintptr_t end = begin+sizeof(p);
    const std::uintptr_t pos = reinterpret_cast<std::uintptr_t>(&position);
    const std::uintptr_t cen = reinterpret_cast<std::uintptr_t>(&center);
    if (pos == cen || (pos < end && pos+sizeof(position) > begin) ||
        (cen < end && cen+sizeof(center) > begin)) return false;
    position = Point3{}; center = Point3{};
    if (!Detail::Finite(fraction) || fraction < 0.0 || fraction > 1.0) return false;
    Segment segment{};
    if (!SegmentAt(p,cursor,segment)) return false;
    if (fraction == 0.0) { position=segment.start; center=segment.centerStart; return true; }
    if (fraction == 1.0) { position=segment.end; center=segment.centerEnd; return true; }
    for (unsigned i=0U;i<3U;++i)
    {
        position[i] = segment.start[i]+fraction*(segment.end[i]-segment.start[i]);
        center[i] = segment.centerStart[i]+fraction*(segment.centerEnd[i]-segment.centerStart[i]);
    }
    if (!Detail::FinitePoint(position) || !Detail::FinitePoint(center))
    { position=Point3{}; center=Point3{}; return false; }
    return true;
}

inline bool BuildInitial(const Descriptor& p, Chunk& out) noexcept
{
    Detail::ClearChunk(out);
    if (!Valid(p)) return false;
    out.initial = true; out.start = p.input.start;
    for (unsigned i=0U;i<2U;++i)
    {
        Segment segment{};
        if (!Detail::SegmentUnchecked(p,Cursor{true,0U,i},segment))
        { Detail::ClearChunk(out); return false; }
        out.points[i].position=segment.end; out.points[i].center=segment.centerEnd;
        out.points[i].kind=segment.kind; out.points[i].segmentIndex=i;
    }
    out.count=2U;
    return true;
}
inline bool BuildLayer(const Descriptor& p, unsigned index, Chunk& out) noexcept
{
    Detail::ClearChunk(out);
    if (!Valid(p) || index >= p.circleLayerCount) return false;
    Layer layer{};
    if (!Detail::LayerUnchecked(p,index,layer)) return false;
    out.start=layer.center; out.layerIndex=index; out.chordErrorBoundMm=layer.chordErrorBoundMm;
    for (unsigned i=0U;i<LayerSegments;++i)
    {
        Segment segment{};
        if (!Detail::SegmentUnchecked(p,Cursor{false,index,i},segment))
        { Detail::ClearChunk(out); return false; }
        out.points[i].position=segment.end; out.points[i].center=segment.centerEnd;
        out.points[i].kind=segment.kind; out.points[i].layerIndex=index; out.points[i].segmentIndex=i;
    }
    out.count=LayerSegments;
    return true;
}
} // namespace NCG76Sphere
