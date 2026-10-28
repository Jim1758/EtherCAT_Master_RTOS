#pragma once

#include <cmath>
#include <cstdint>

// Pure G76 mode/shape selection. Recognition is not execution permission;
// each NC/Motion lane still admits its own exact modes and process scope.
// EDM273 adds the vector mode identities through the existing complete-contour
// geometry. Their target-axis/K/Q contract is shared with the corresponding
// shape; directional clipping and quadrant semantics are not implemented.
namespace NCG76Mode
{
enum class Family : std::uint8_t { Invalid, Taper, HoleExpansion, FixedRadius };
enum class Shape : std::uint8_t { Invalid, Circle, Square, Diamond, Polygon };
struct Definition
{
    std::uint32_t modeP=0U;
    Family family=Family::Invalid;
    Shape shape=Shape::Invalid;
    bool vector=false;
    bool implemented=false;
};
inline bool IsRecognized(std::uint32_t modeP) noexcept
{ return (modeP>=1U&&modeP<=7U)||(modeP>=11U&&modeP<=17U)||(modeP>=21U&&modeP<=27U); }
// NC words and provenance markers are doubles. Validate before narrowing so
// NaN, infinity and out-of-range values cannot become an implemented mode.
inline bool TryModeFromLiteral(double literal,std::uint32_t& modeP) noexcept
{
    modeP=0U;
    if(!std::isfinite(literal)||literal<1.0||literal>27.0||std::floor(literal)!=literal)return false;
    const auto candidate=static_cast<std::uint32_t>(literal);
    if(!IsRecognized(candidate))return false;
    modeP=candidate;return true;
}
inline bool IsVector(std::uint32_t modeP) noexcept
{ return IsRecognized(modeP)&&(modeP%10U==4U||modeP%10U==5U||modeP%10U==7U); }
inline bool IsHoleExpansion(std::uint32_t modeP) noexcept
{ return modeP>=21U&&modeP<=27U; }
inline bool IsImplementedFixedRadius(std::uint32_t modeP) noexcept
{ return modeP>=1U&&modeP<=7U; }
inline bool IsFixedRadius(std::uint32_t modeP) noexcept
{ return IsImplementedFixedRadius(modeP); }
inline bool IsTaper(std::uint32_t modeP) noexcept
{ return modeP>=11U&&modeP<=17U; }
inline bool IsPolygon(std::uint32_t modeP) noexcept
{ return IsRecognized(modeP)&&(modeP%10U==6U||modeP%10U==7U); }
inline bool IsImplemented(std::uint32_t modeP) noexcept
{ return IsRecognized(modeP); }
// Ordinary AIR retains legacy P11's program-XY contour. Every other mode
// uses the plane normal to the authored XYZ start-to-target vector.
inline bool IsTargetAxisMode(std::uint32_t modeP) noexcept
{ return IsImplemented(modeP)&&modeP!=11U; }
// Only normalized boundary evaluation uses this canonical shape number.
// ProgramPlan/NativePlan retain the original P identity throughout validation.
inline std::uint32_t CanonicalContourMode(std::uint32_t modeP) noexcept
{
    if(!IsRecognized(modeP))return 0U;
    const auto index=modeP%10U;
    return index==1U?11U:index==2U||index==4U?12U:index==3U||index==5U?13U:16U;
}
inline bool Lookup(std::uint32_t modeP,Definition& out) noexcept
{
    out=Definition{};
    if(!IsRecognized(modeP))return false;
    out.modeP=modeP;
    out.family=modeP<10U?Family::FixedRadius:modeP<20U?Family::Taper:Family::HoleExpansion;
    const unsigned index=modeP%10U;
    out.shape=index==1U?Shape::Circle:index==2U||index==4U?Shape::Square:
        index==3U||index==5U?Shape::Diamond:Shape::Polygon;
    out.vector=IsVector(modeP);
    out.implemented=IsImplemented(modeP);
    return true;
}
inline const char* ShapeName(std::uint32_t modeP) noexcept
{
    modeP=CanonicalContourMode(modeP);
    return modeP==11U?"CIRCLE":modeP==12U?"SQUARE":modeP==13U?"DIAMOND":modeP==16U?"POLYGON":"UNIMPLEMENTED";
}

inline bool ValidPolygonSides(unsigned sides) noexcept
{ return sides>=3U && sides<=12U; }
inline unsigned PolygonCornerSample(unsigned sides,unsigned corner) noexcept
{ return ValidPolygonSides(sides)&&corner<=sides?(24U*corner)/sides:25U; }
inline bool PolygonVertex(unsigned sides,unsigned vertex,double& x,double& y) noexcept
{
    x=y=0.0;
    if(!ValidPolygonSides(sides)||vertex>sides)return false;
    if(vertex==sides)vertex=0U;
    // Exact cardinal vertices also close the polygon without a trig residual.
    const unsigned quarter=4U*vertex;
    if(quarter%sides==0U)
    {
        switch(quarter/sides)
        {
        case 0U:x=1.0;break;case 1U:y=1.0;break;
        case 2U:x=-1.0;break;default:y=-1.0;break;
        }
    }
    else
    {
        constexpr double pi=3.141592653589793238462643383279502884;
        const double angle=(2.0*pi)*(static_cast<double>(vertex)/static_cast<double>(sides));
        x=std::cos(angle);y=std::sin(angle);
    }
    return true;
}

// One revolution has 24 intervals. Square R is its circumradius:
// K0 square edges are parallel to the local basis, with corners at 0/6/12/18/24.
// The perimeter is linear on each side, so no trigonometric square projection
// or unbounded corner curvature is hidden between samples.
// P13 is the confirmed equal-diagonal diamond: K0 vertices are +X,+Y,-X,-Y.
// R is its center-to-vertex circumradius, with the same six samples per side.
inline bool UnitBoundary(std::uint32_t modeP,unsigned sample,double& x,double& y,unsigned polygonSides=0U) noexcept
{
    // Families change the center/radius path, not the normalized contour.
    // EDM273 vector entries currently share this entire closed boundary.
    modeP=CanonicalContourMode(modeP);
    x=y=0.0;
    if(sample>24U||(modeP==16U?!ValidPolygonSides(polygonSides):polygonSides!=0U))return false;
    if(modeP==16U)
    {
        // Four vertices use the established diamond operations verbatim.
        if(polygonSides==4U)modeP=13U;
        else
        {
            if(sample==24U)return PolygonVertex(polygonSides,0U,x,y);
            for(unsigned side=0U;side<polygonSides;++side)
            {
                const unsigned begin=PolygonCornerSample(polygonSides,side);
                const unsigned end=PolygonCornerSample(polygonSides,side+1U);
                if(sample>=end)continue;
                if(sample==begin)return PolygonVertex(polygonSides,side,x,y);
                double ax=0.0,ay=0.0,bx=0.0,by=0.0;
                if(!PolygonVertex(polygonSides,side,ax,ay)||!PolygonVertex(polygonSides,side+1U,bx,by))return false;
                const double along=static_cast<double>(sample-begin)/static_cast<double>(end-begin);
                x=ax+(bx-ax)*along;y=ay+(by-ay)*along;
                return true;
            }
            return false;
        }
    }
    if(modeP==11U)
    {
        if(sample==0U||sample==24U)x=1.0;
        else if(sample==6U)y=1.0;
        else if(sample==12U)x=-1.0;
        else if(sample==18U)y=-1.0;
        else
        {
            constexpr double pi=3.141592653589793238462643383279502884;
            const double t=static_cast<double>(sample)/24.0;
            const double angle=(2.0*pi)*t;
            x=std::cos(angle);y=std::sin(angle);
        }
        return true;
    }
    if(modeP==13U)
    {
        if(sample==0U||sample==24U)x=1.0;
        else if(sample==6U)y=1.0;
        else if(sample==12U)x=-1.0;
        else if(sample==18U)y=-1.0;
        else
        {
            const unsigned side=sample/6U;
            const double along=static_cast<double>(sample%6U)/6.0;
            switch(side)
            {
            case 0U:x=1.0-along;y=along;break;
            case 1U:x=-along;y=1.0-along;break;
            case 2U:x=-1.0+along;y=-along;break;
            default:x=along;y=-1.0+along;break;
            }
        }
        return true;
    }
    if(modeP!=12U)return false;
    constexpr double half=0.707106781186547524400844362104849039;
    const unsigned side=sample==24U?0U:sample/6U;
    const double along=(2.0*half)*(static_cast<double>(sample%6U)/6.0);
    switch(side)
    {
    case 0U:x=half-along;y=half;break;
    case 1U:x=-half;y=half-along;break;
    case 2U:x=-half+along;y=-half;break;
    default:x=half;y=-half+along;break;
    }
    return true;
}
}
