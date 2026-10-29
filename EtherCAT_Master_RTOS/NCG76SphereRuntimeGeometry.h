#pragma once

#include "NCG76SphereCoordinates.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

// EDM286 analytic geometry only. Build/Valid run at admission, never per tick.
// LayerAt/CircleAt consume the caller's already validated immutable Geometry;
// their work is bounded O(1), with no frame transforms, allocation, publication,
// controller ownership or discharge authority. The original full frame and
// descriptor remain attached for producer/consumer/hold identity comparisons.
// C/U/V and absent slots retain their accepted native binary64 values.
// Pitch/backlash remain exclusively with the existing final send correction.
namespace NCG76SphereRuntimeGeometry
{
using Point3=NCG76Sphere::Point3;
using Point8=NCG76SphereCoordinates::Point8;
using RadiusLaw=NCG76Sphere::RadiusLaw;
constexpr unsigned Version=1U;
constexpr double Pi=NCG76Sphere::Pi,TwoPi=2.0*Pi;
constexpr double MaximumArithmeticErrorMm=1.e-9;
constexpr double OrthonormalTolerance=1.e-12;
struct Geometry
{
    NCG76SphereCoordinates::FrozenFrame frame{};
    NCG76Sphere::Descriptor descriptor{};
    Point8 startMCS{},endMCS{},boundsMinMCS{},boundsMaxMCS{};
    Point3 axis{},u{},v{},maxCircleTangentAbs{},maxCircleCurvatureAbs{};
    double radiusMm=0.0,layerStepMm=0.0,travelMm=0.0,feedMmMin=0.0;
    double minimumCircleRadiusMm=0.0,maximumCircleArcLengthMm=0.0;
    double arithmeticErrorBoundMm=0.0,coordinateErrorBoundMm=0.0;
    unsigned circleLayerCount=0U,version=0U;
    RadiusLaw radiusLaw=RadiusLaw::Linear;
    bool valid=false;
};
struct Layer
{
    Point8 center{};
    double depthMm=0.0,radiusMm=0.0,arcLengthMm=0.0;
    unsigned index=0U;
    bool terminal=false;
};
namespace Detail
{
inline bool Bits(double a,double b) noexcept {return std::memcmp(&a,&b,sizeof(double))==0;}
template<std::size_t N> inline bool Same(const std::array<double,N>& a,const std::array<double,N>& b) noexcept
{return std::memcmp(a.data(),b.data(),N*sizeof(double))==0;}
template<std::size_t N> inline bool Finite(const std::array<double,N>& p) noexcept
{for(double x:p)if(!std::isfinite(x))return false;return true;}
inline double Norm(const Point3& p) noexcept {return std::hypot(std::hypot(p[0],p[1]),p[2]);}
inline double Dot(const Point3& a,const Point3& b) noexcept {return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
inline bool Overlap(const void* a,std::size_t an,const void* b,std::size_t bn) noexcept
{
    const auto x=reinterpret_cast<std::uintptr_t>(a),y=reinterpret_cast<std::uintptr_t>(b);
    return x<y+bn&&y<x+an;
}
inline bool Positive(double x) noexcept {return std::isfinite(x)&&x>0.0;}
inline bool Basis(const Geometry& g) noexcept
{
    return Finite(g.axis)&&Finite(g.u)&&Finite(g.v)&&
        std::fabs(Norm(g.axis)-1.0)<=OrthonormalTolerance&&
        std::fabs(Norm(g.u)-1.0)<=OrthonormalTolerance&&std::fabs(Norm(g.v)-1.0)<=OrthonormalTolerance&&
        std::fabs(Dot(g.axis,g.u))<=OrthonormalTolerance&&std::fabs(Dot(g.axis,g.v))<=OrthonormalTolerance&&
        std::fabs(Dot(g.u,g.v))<=OrthonormalTolerance;
    // Deliberately no cross(u,v)==axis requirement: mirrors and the signed
    // machining direction may reverse the retained contour handedness.
}
inline bool Offset(const Point8& base,const Point3& delta,double errorBound,Point8& output) noexcept
{
    output=base;Point3 error{};
    for(unsigned a=0;a<3;++a){output[a]=base[a]+delta[a];error[a]=(output[a]-base[a])-delta[a];}
    return Finite(output)&&Finite(delta)&&Finite(error)&&Norm(error)<=errorBound;
}
inline double RadiusAt(const Geometry& g,double depth) noexcept
{
    if(depth==0.0)return g.radiusMm;
    if(depth==g.radiusMm)return 0.0;
    return g.radiusLaw==RadiusLaw::Linear?g.radiusMm-depth:
        g.radiusMm*std::sqrt((g.radiusMm-depth)/g.radiusMm)*std::sqrt(1.0+depth/g.radiusMm);
}
inline bool RawLayer(const Geometry& g,unsigned index,Layer& out) noexcept
{
    out=Layer{};
    if(index>g.circleLayerCount)return false;
    out.index=index;out.terminal=index==g.circleLayerCount;
    if(out.terminal){out.center=g.endMCS;out.depthMm=g.radiusMm;return true;}
    out.depthMm=static_cast<double>(index)*g.layerStepMm;
    if(!std::isfinite(out.depthMm)||out.depthMm<0.0||out.depthMm>=g.radiusMm)return false;
    out.radiusMm=RadiusAt(g,out.depthMm);out.arcLengthMm=TwoPi*out.radiusMm;
    if(!Positive(out.radiusMm)||!Positive(out.arcLengthMm))return false;
    Point3 delta{};const double retreat=out.depthMm-g.radiusMm;
    for(unsigned a=0;a<3;++a)delta[a]=g.axis[a]*retreat;
    return Offset(g.endMCS,delta,g.arithmeticErrorBoundMm,out.center);
}
inline bool ScalarShape(const Geometry& g) noexcept
{
    if(!Positive(g.radiusMm)||!Positive(g.layerStepMm)||!Positive(g.travelMm)||!Positive(g.feedMmMin)||
        !g.circleLayerCount||g.circleLayerCount>NCG76Sphere::MaximumLayers||
        (g.radiusLaw!=RadiusLaw::Linear&&g.radiusLaw!=RadiusLaw::TrueSphere)||!Basis(g)||
        !Finite(g.startMCS)||!Finite(g.endMCS))return false;
    for(unsigned a=3;a<8;++a)if(!Bits(g.startMCS[a],g.endMCS[a]))return false;
    Point3 delta{},expected{},error{};
    for(unsigned a=0;a<3;++a){delta[a]=g.endMCS[a]-g.startMCS[a];expected[a]=g.axis[a]*g.travelMm;error[a]=delta[a]-expected[a];}
    const double difference=Norm(error);
    if(!Finite(delta)||!Finite(expected)||!std::isfinite(difference)||!Positive(Norm(delta))||
        difference>MaximumArithmeticErrorMm||difference>g.travelMm/64.0)return false;
    // Keep the exact admitted layer count; never silently truncate or add a
    // tiny ring after converting R/K into native millimetres.
    double ratio=g.radiusMm/g.layerStepMm;
    if(!Positive(ratio))return false;
    const double nearest=std::floor(ratio+.5);
    if(nearest>=1.0&&std::fabs(ratio-nearest)<=8.0*std::numeric_limits<double>::epsilon()*(std::max)(1.0,ratio))ratio=nearest;
    return std::ceil(ratio)==static_cast<double>(g.circleLayerCount);
}
inline bool Derive(const NCG76SphereCoordinates::NativePlan& source,Geometry& output) noexcept
{
    Geometry g{};g.frame=source.frame;g.descriptor=source.descriptor;
    g.startMCS=source.frame.startMCS;g.endMCS=source.endMCS;g.axis=source.axis;g.u=source.u;g.v=source.v;
    g.radiusMm=source.radiusMm;g.layerStepMm=source.layerStepMm;g.travelMm=source.travelMm;
    g.feedMmMin=source.descriptor.input.feedMmMin;g.circleLayerCount=source.descriptor.circleLayerCount;
    g.radiusLaw=source.descriptor.input.radiusLaw;
    if(!ScalarShape(g))return false;
    // One physical-mm arithmetic budget, independent of how large the frame
    // offsets or origins are. It is separate from the admitted source bound.
    double envelope=0.0;
    for(unsigned a=0;a<3;++a){const double extent=std::fabs(g.endMCS[a])+2.0*g.radiusMm;
        if(!std::isfinite(extent))return false;
        envelope=(std::max)(envelope,(std::max)(std::fabs(g.startMCS[a]),extent));}
    g.arithmeticErrorBoundMm=std::nextafter(envelope*(64.0*std::numeric_limits<double>::epsilon()),
        std::numeric_limits<double>::infinity());
    if(!Positive(g.arithmeticErrorBoundMm)||g.arithmeticErrorBoundMm>MaximumArithmeticErrorMm)return false;
    Layer first{},last{};
    if(!RawLayer(g,0,first)||!RawLayer(g,g.circleLayerCount-1U,last))return false;
    g.minimumCircleRadiusMm=last.radiusMm;g.maximumCircleArcLengthMm=TwoPi*g.radiusMm;
    const double lastGap=g.radiusMm-last.depthMm;
    const double feature=(std::min)(g.travelMm,(std::min)(g.radiusMm,(std::min)(g.layerStepMm,lastGap)));
    if(!Positive(feature)||g.arithmeticErrorBoundMm>feature/64.0||
        g.arithmeticErrorBoundMm>g.minimumCircleRadiusMm/64.0||!Positive(1.0/g.minimumCircleRadiusMm)||
        !Positive(g.maximumCircleArcLengthMm))return false;
    // Analytic circles have no 24-chord sagitta term. Carry coordinate
    // rounding only, and expand the whole path envelope outwards.
    g.coordinateErrorBoundMm=source.coordinateErrorBoundMm+g.arithmeticErrorBoundMm;
    if(!Positive(g.coordinateErrorBoundMm))return false;
    g.boundsMinMCS=g.boundsMaxMCS=g.startMCS;
    for(unsigned a=0;a<3;++a){
        const double amplitude=std::hypot(g.u[a],g.v[a]);
        g.maxCircleTangentAbs[a]=amplitude==0.0?0.0:std::nextafter(amplitude*(1.0+16.0*std::numeric_limits<double>::epsilon()),
            std::numeric_limits<double>::infinity());
        g.maxCircleCurvatureAbs[a]=amplitude==0.0?0.0:std::nextafter(g.maxCircleTangentAbs[a]/g.minimumCircleRadiusMm,
            std::numeric_limits<double>::infinity());
        const double excursion=g.radiusMm*g.maxCircleTangentAbs[a];
        // The center segment and maximum orbit amplitude contain every ring,
        // initial depth/retreat and straight HOLD/resume leg. These are path
        // bounds, not a claim about machine following error or stop overrun.
        const double lower=(std::min)(g.startMCS[a],(std::min)(first.center[a],g.endMCS[a])-excursion);
        const double upper=(std::max)(g.startMCS[a],(std::max)(first.center[a],g.endMCS[a])+excursion);
        g.boundsMinMCS[a]=std::nextafter(lower-g.coordinateErrorBoundMm,-std::numeric_limits<double>::infinity());
        g.boundsMaxMCS[a]=std::nextafter(upper+g.coordinateErrorBoundMm,std::numeric_limits<double>::infinity());
        if(!std::isfinite(g.maxCircleTangentAbs[a])||!std::isfinite(g.maxCircleCurvatureAbs[a])||
            !std::isfinite(g.boundsMinMCS[a])||!std::isfinite(g.boundsMaxMCS[a]))return false;
    }
    g.version=Version;g.valid=true;output=g;return true;
}
}
inline bool SameGeometry(const Geometry& a,const Geometry& b) noexcept
{
    return a.valid==b.valid&&a.version==b.version&&a.circleLayerCount==b.circleLayerCount&&a.radiusLaw==b.radiusLaw&&
        NCG76SphereCoordinates::SameFrame(a.frame,b.frame)&&NCG76Sphere::SameDescriptor(a.descriptor,b.descriptor)&&
        Detail::Same(a.startMCS,b.startMCS)&&Detail::Same(a.endMCS,b.endMCS)&&
        Detail::Same(a.boundsMinMCS,b.boundsMinMCS)&&Detail::Same(a.boundsMaxMCS,b.boundsMaxMCS)&&
        Detail::Same(a.axis,b.axis)&&Detail::Same(a.u,b.u)&&Detail::Same(a.v,b.v)&&
        Detail::Same(a.maxCircleTangentAbs,b.maxCircleTangentAbs)&&Detail::Same(a.maxCircleCurvatureAbs,b.maxCircleCurvatureAbs)&&
        Detail::Bits(a.radiusMm,b.radiusMm)&&Detail::Bits(a.layerStepMm,b.layerStepMm)&&
        Detail::Bits(a.travelMm,b.travelMm)&&Detail::Bits(a.feedMmMin,b.feedMmMin)&&
        Detail::Bits(a.minimumCircleRadiusMm,b.minimumCircleRadiusMm)&&Detail::Bits(a.maximumCircleArcLengthMm,b.maximumCircleArcLengthMm)&&
        Detail::Bits(a.arithmeticErrorBoundMm,b.arithmeticErrorBoundMm)&&Detail::Bits(a.coordinateErrorBoundMm,b.coordinateErrorBoundMm);
}
inline bool Build(const NCG76SphereCoordinates::NativePlan& input,Geometry& output) noexcept
{
    const NCG76SphereCoordinates::NativePlan source=input;output=Geometry{};
    return NCG76SphereCoordinates::Valid(source)&&Detail::Derive(source,output);
}
// Full admission check. Never call this from the periodic interpolation path.
inline bool Valid(const Geometry& g) noexcept
{
    if(!g.valid||g.version!=Version)return false;
    NCG76SphereCoordinates::NativePlan source{};Geometry expected{};
    return NCG76SphereCoordinates::BuildNative(g.frame,g.descriptor,source)==NCG76SphereCoordinates::Status::Ok&&
        Detail::Derive(source,expected)&&SameGeometry(g,expected);
}
inline bool LayerAt(const Geometry& g,unsigned index,Layer& output) noexcept
{
    if(Detail::Overlap(&g,sizeof(g),&output,sizeof(output)))return false;
    output=Layer{};
    if(!g.valid||g.version!=Version||index>g.circleLayerCount)return false;
    Layer layer{};if(!Detail::RawLayer(g,index,layer))return false;
    output=layer;return true;
}
// Native analytic circle; tangent is dP/ds (unitless), curvature is dT/ds
// (1/mm). Thus velocity=tangent*speed and acceleration=tangent*scalarAccel
// +curvature*speed^2. Reflected u/v preserve the original contour direction.
inline bool CircleAt(const Geometry& g,unsigned layerIndex,double angleRad,Point8& position,
    Point3& tangent,Point3& curvature) noexcept
{
    if(Detail::Overlap(&g,sizeof(g),&position,sizeof(position))||Detail::Overlap(&g,sizeof(g),&tangent,sizeof(tangent))||
        Detail::Overlap(&g,sizeof(g),&curvature,sizeof(curvature))||
        Detail::Overlap(&position,sizeof(position),&tangent,sizeof(tangent))||
        Detail::Overlap(&position,sizeof(position),&curvature,sizeof(curvature))||
        Detail::Overlap(&tangent,sizeof(tangent),&curvature,sizeof(curvature)))return false;
    position=Point8{};tangent=Point3{};curvature=Point3{};
    if(!g.valid||g.version!=Version||!std::isfinite(angleRad)||angleRad<0.0||angleRad>TwoPi)return false;
    Layer layer{};if(!LayerAt(g,layerIndex,layer)||layer.terminal)return false;
    double sine=0.0,cosine=1.0;
    if(angleRad==Pi*.5){sine=1.0;cosine=0.0;}
    else if(angleRad==Pi){sine=0.0;cosine=-1.0;}
    else if(angleRad==Pi*1.5){sine=-1.0;cosine=0.0;}
    else if(angleRad!=0.0&&angleRad!=TwoPi){sine=std::sin(angleRad);cosine=std::cos(angleRad);}
    Point3 radial{},delta{},t{},k{};
    for(unsigned a=0;a<3;++a){radial[a]=g.u[a]*cosine+g.v[a]*sine;delta[a]=layer.radiusMm*radial[a];
        t[a]=-g.u[a]*sine+g.v[a]*cosine;k[a]=-radial[a]/layer.radiusMm;}
    Point8 point{};
    if(!Detail::Finite(t)||!Detail::Finite(k)||!Detail::Offset(layer.center,delta,g.arithmeticErrorBoundMm,point))return false;
    for(unsigned a=0;a<3;++a)if(point[a]<g.boundsMinMCS[a]||point[a]>g.boundsMaxMCS[a])return false;
    position=point;tangent=t;curvature=k;return true;
}
static_assert(std::is_trivially_copyable<Geometry>::value,"P50 analytic geometry is immutable value data");
}
