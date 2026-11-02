#pragma once

#include "NCG76SphereCoordinates.h"
#include "NCG76OrbitContourGeometry.h"
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
enum class Kind : std::uint8_t { Sphere, Line, Arc, OrbitContour };
constexpr unsigned Version=1U;
constexpr double Pi=NCG76Sphere::Pi,TwoPi=2.0*Pi;
constexpr double MaximumArithmeticErrorMm=1.e-9;
constexpr double OrthonormalTolerance=1.e-12;
struct Geometry
{
    NCG76SphereCoordinates::FrozenFrame frame{};
    union { NCG76Sphere::Descriptor descriptor{}; NCG76OrbitContour::Descriptor orbit; };
    Point8 startMCS{},endMCS{},boundsMinMCS{},boundsMaxMCS{};
    Point3 axis{},u{},v{},maxCircleTangentAbs{},maxCircleCurvatureAbs{};
    double radiusMm=0.0,layerStepMm=0.0,travelMm=0.0,feedMmMin=0.0;
    double minimumCircleRadiusMm=0.0,maximumCircleArcLengthMm=0.0;
    double arithmeticErrorBoundMm=0.0,coordinateErrorBoundMm=0.0;
    unsigned circleLayerCount=0U,version=0U;
    RadiusLaw radiusLaw=RadiusLaw::Linear;
    // EDM307: explicit primitive source, never a fabricated P50 descriptor.
    Point3 arcCenterMCS{},arcNormal{};
    double arcSweepRad=0.0;
    int arcDirection=0;
    Kind kind=Kind::Sphere;
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
    return a.kind==b.kind&&a.valid==b.valid&&a.version==b.version&&a.circleLayerCount==b.circleLayerCount&&a.radiusLaw==b.radiusLaw&&
        Detail::Same(a.arcCenterMCS,b.arcCenterMCS)&&Detail::Same(a.arcNormal,b.arcNormal)&&
        Detail::Bits(a.arcSweepRad,b.arcSweepRad)&&a.arcDirection==b.arcDirection&&
        NCG76SphereCoordinates::SameFrame(a.frame,b.frame)&&
        (a.kind==Kind::OrbitContour?NCG76OrbitContour::SameDescriptor(a.orbit,b.orbit):
            NCG76Sphere::SameDescriptor(a.descriptor,b.descriptor))&&
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
// Native G01/G02/G03 primitives share the cycle and condition configuration,
// while retaining their own canonical source and a single nominal station.
namespace Detail
{
inline Point3 Cross(const Point3& a,const Point3& b) noexcept
{return Point3{{a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]}};}
inline bool PrimitiveSource(const NCG76SphereCoordinates::FrozenFrame& frame,
    const Point8& target,double feed,Geometry& g) noexcept
{
    if(!NCG76SphereCoordinates::ValidFrame(frame)||!Finite(target)||!Positive(feed)||feed>100000.0)return false;
    for(unsigned a=3U;a<8U;++a)if(!Bits(frame.startMCS[a],target[a]))return false;
    g.frame=frame;g.startMCS=frame.startMCS;g.endMCS=target;g.feedMmMin=feed;
    return true;
}
inline bool PrimitiveBudget(Geometry& g,double feature,double envelope) noexcept
{
    g.arithmeticErrorBoundMm=std::nextafter(envelope*(64.0*std::numeric_limits<double>::epsilon()),
        std::numeric_limits<double>::infinity());
    if(!Positive(g.arithmeticErrorBoundMm)||g.arithmeticErrorBoundMm>MaximumArithmeticErrorMm||
       !Positive(feature)||g.arithmeticErrorBoundMm>feature/64.0)return false;
    g.coordinateErrorBoundMm=g.arithmeticErrorBoundMm;
    return true;
}
inline bool ArcMappingEnvelope(const NCG76SphereCoordinates::FrozenFrame& frame,
    double radius,double& envelope) noexcept
{
    // G90 endpoints pass through affine points, whereas IJK passes through
    // vectors. Small native endpoints therefore do not bound cancellation at
    // the point map's scale/mirror/rotation centres and fixed offsets. Trace
    // absolute operand magnitudes through those same active stages. The L1
    // envelope covers XYZ and the forward/inverse dependency paths remain
    // inside PrimitiveBudget's existing 64-epsilon arithmetic allowance.
    // G91 uses start + a vector; its local native envelope already applies.
    if(frame.translation.distanceMode!=90)return true;
    const auto& s=frame.translation;
    const double span=2.0*radius+MaximumArithmeticErrorMm+
        NCG76SphereCoordinates::MaximumMappingErrorMm;
    // An admitted endpoint is within a native diameter, plus the two existing
    // admission fences, of the accepted start. These fences bound the input
    // domain only; they are not added to the resulting arithmetic tolerance.
    Point3 magnitude{};
    for(unsigned a=0U;a<3U;++a){
        magnitude[a]=std::fabs(frame.programStart[a])+
            span/(frame.fixedEccentric?1.0:frame.nativeScale);
        if(frame.fixedEccentric){
            magnitude[a]+=std::fabs(s.extOffsetMM[a])+std::fabs(s.wcsOffsetMM[a])+
                std::fabs(frame.signedEccentricOffset[a]);
        }else{
            if(s.scalingMode==51&&s.scalingFactor!=1.0)
                magnitude[a]=std::fabs(s.scalingCenterMM[a])+
                    (magnitude[a]+std::fabs(s.scalingCenterMM[a]))*s.scalingFactor;
            if((s.mirrorMask&(1U<<a))!=0U)
                magnitude[a]+=2.0*std::fabs(s.mirrorCenterMM[a]);
        }
    }
    if(!frame.fixedEccentric){
        NCArcPlaneAxes plane{};if(!TryGetNCArcPlaneAxes(s.rotationPlane,plane))return false;
        double cosine=1.0,sine=0.0;
        NCTranslationRotationCoefficients(s,cosine,sine);
        if(cosine!=1.0||sine!=0.0){
            const double u=magnitude[plane.u]+std::fabs(s.rotationCenterMM[0]);
            const double v=magnitude[plane.v]+std::fabs(s.rotationCenterMM[1]);
            magnitude[plane.u]=std::fabs(cosine)*u+std::fabs(sine)*v+std::fabs(s.rotationCenterMM[0]);
            magnitude[plane.v]=std::fabs(sine)*u+std::fabs(cosine)*v+std::fabs(s.rotationCenterMM[1]);
        }
        for(unsigned a=0U;a<3U;++a)
            magnitude[a]+=std::fabs(s.extOffsetMM[a])+std::fabs(s.wcsOffsetMM[a])+
                std::fabs(NCTranslationToolOffsetMM(s,a))+(s.workMode==168?std::fabs(s.workOffset[a]):0.0);
        double angle=0.0;if(!TryGetNCTranslationWorkPlaneAngle(s,angle))return false;
        NCTranslationPlanarCoefficients(angle,cosine,sine);
        if(cosine!=1.0||sine!=0.0){
            const double u=magnitude[plane.u]+std::fabs(s.workRotationCenterMM[0]);
            const double v=magnitude[plane.v]+std::fabs(s.workRotationCenterMM[1]);
            magnitude[plane.u]=std::fabs(cosine)*u+std::fabs(sine)*v+std::fabs(s.workRotationCenterMM[0]);
            magnitude[plane.v]=std::fabs(sine)*u+std::fabs(cosine)*v+std::fabs(s.workRotationCenterMM[1]);
        }
    }
    const double mapped=magnitude[0]+magnitude[1]+magnitude[2];
    if(!Finite(magnitude)||!std::isfinite(mapped))return false;
    envelope=(std::max)(envelope,mapped);return true;
}
inline bool DeriveLine(const NCG76SphereCoordinates::FrozenFrame& frame,
    const Point8& target,double feed,Geometry& out) noexcept
{
    Geometry g{};g.kind=Kind::Line;
    if(!PrimitiveSource(frame,target,feed,g))return false;
    Point3 delta{};double envelope=0.0;
    for(unsigned a=0U;a<3U;++a){delta[a]=target[a]-frame.startMCS[a];
        envelope=(std::max)(envelope,(std::max)(std::fabs(target[a]),std::fabs(frame.startMCS[a])));}
    g.travelMm=Norm(delta);
    if(!Finite(delta)||!std::isfinite(g.travelMm))return false;
    if(g.travelMm==0.0){
        // An addressed G01 at the accepted endpoint is a real stationary
        // parent. Preserve that accepted spelling; no tangent or positive
        // path distance is fabricated and no flushing motion is generated.
        if(!PrimitiveBudget(g,1.0,(std::max)(1.0,envelope)))return false;
        g.endMCS=g.boundsMinMCS=g.boundsMaxMCS=g.startMCS;
        g.version=Version;g.valid=true;out=g;return true;
    }
    if(!Positive(g.travelMm)||g.travelMm>10000.0||!PrimitiveBudget(g,g.travelMm,envelope))return false;
    for(unsigned a=0U;a<3U;++a){g.axis[a]=delta[a]/g.travelMm;g.maxCircleTangentAbs[a]=std::fabs(g.axis[a]);}
    g.boundsMinMCS=g.boundsMaxMCS=g.startMCS;
    for(unsigned a=0U;a<3U;++a){
        g.boundsMinMCS[a]=std::nextafter((std::min)(g.startMCS[a],g.endMCS[a])-g.coordinateErrorBoundMm,
            -std::numeric_limits<double>::infinity());
        g.boundsMaxMCS[a]=std::nextafter((std::max)(g.startMCS[a],g.endMCS[a])+g.coordinateErrorBoundMm,
            std::numeric_limits<double>::infinity());}
    g.version=Version;g.valid=true;out=g;return true;
}
inline bool DeriveArc(const NCG76SphereCoordinates::FrozenFrame& frame,const Point8& target,
    const Point3& center,const Point3& normal,int direction,double feed,bool fullCircle,Geometry& out) noexcept
{
    Geometry g{};g.kind=Kind::Arc;
    if(!PrimitiveSource(frame,target,feed,g)||!Finite(center)||!Finite(normal)||(direction!=1&&direction!=-1))return false;
    const double normalLength=Norm(normal);
    if(!Positive(normalLength)||std::fabs(normalLength-1.0)>OrthonormalTolerance)return false;
    Point3 radial{},endRadial{},unitNormal{};double envelope=0.0;
    for(unsigned a=0U;a<3U;++a){radial[a]=g.startMCS[a]-center[a];endRadial[a]=g.endMCS[a]-center[a];unitNormal[a]=normal[a]/normalLength;}
    g.radiusMm=Norm(radial);
    if(!Finite(radial)||!Finite(endRadial)||!Positive(g.radiusMm)||g.radiusMm>10000.0||!Positive(1.0/g.radiusMm))return false;
    for(unsigned a=0U;a<3U;++a){g.u[a]=radial[a]/g.radiusMm;
        envelope=(std::max)(envelope,(std::max)(std::fabs(g.startMCS[a]),std::fabs(center[a])+g.radiusMm));}
    if(!fullCircle&&!ArcMappingEnvelope(frame,g.radiusMm,envelope))return false;
    if(!PrimitiveBudget(g,g.radiusMm,envelope))return false;
    if(std::fabs(Dot(radial,unitNormal))>g.arithmeticErrorBoundMm||
       std::fabs(Dot(endRadial,unitNormal))>g.arithmeticErrorBoundMm||
       std::fabs(Norm(endRadial)-g.radiusMm)>g.arithmeticErrorBoundMm)return false;
    g.v=Cross(unitNormal,g.u);for(double& x:g.v)x*=static_cast<double>(direction);
    if(std::fabs(Norm(g.v)-1.0)>OrthonormalTolerance||std::fabs(Dot(g.u,g.v))>OrthonormalTolerance)return false;
    bool same=true;for(unsigned a=0U;a<3U;++a)same=same&&Bits(g.startMCS[a],g.endMCS[a]);
    if(fullCircle!=same)return false;
    double angle=fullCircle?TwoPi:std::atan2(Dot(endRadial,g.v),Dot(endRadial,g.u));
    if(!fullCircle&&angle<=0.0)angle+=TwoPi;
    if(!Positive(angle)||angle>TwoPi)return false;
    g.arcSweepRad=angle;g.arcCenterMCS=center;g.arcNormal=normal;g.arcDirection=direction;g.axis=g.v;
    g.travelMm=g.radiusMm*angle;g.minimumCircleRadiusMm=g.radiusMm;g.maximumCircleArcLengthMm=g.travelMm;
    if(!Positive(g.travelMm)||g.travelMm>10000.0||g.arithmeticErrorBoundMm>g.travelMm/64.0)return false;
    g.boundsMinMCS=g.boundsMaxMCS=g.startMCS;
    for(unsigned a=0U;a<3U;++a){
        const double amplitude=std::hypot(g.u[a],g.v[a]);
        g.maxCircleTangentAbs[a]=amplitude==0.0?0.0:std::nextafter(amplitude*(1.0+16.0*std::numeric_limits<double>::epsilon()),
            std::numeric_limits<double>::infinity());
        g.maxCircleCurvatureAbs[a]=amplitude==0.0?0.0:std::nextafter(g.maxCircleTangentAbs[a]/g.radiusMm,
            std::numeric_limits<double>::infinity());
        double low=(std::min)(g.startMCS[a],g.endMCS[a]),high=(std::max)(g.startMCS[a],g.endMCS[a]);
        double extreme=std::atan2(g.v[a],g.u[a]);if(extreme<0.0)extreme+=TwoPi;
        for(unsigned k=0U;k<2U;++k){double q=extreme+static_cast<double>(k)*Pi;if(q>TwoPi)q-=TwoPi;
            if(q<=angle){const double value=center[a]+g.radiusMm*(g.u[a]*std::cos(q)+g.v[a]*std::sin(q));
                low=(std::min)(low,value);high=(std::max)(high,value);}}
        g.boundsMinMCS[a]=std::nextafter(low-g.coordinateErrorBoundMm,-std::numeric_limits<double>::infinity());
        g.boundsMaxMCS[a]=std::nextafter(high+g.coordinateErrorBoundMm,std::numeric_limits<double>::infinity());
        if(!std::isfinite(g.maxCircleCurvatureAbs[a])||!std::isfinite(g.boundsMinMCS[a])||!std::isfinite(g.boundsMaxMCS[a]))return false;
    }
    g.version=Version;g.valid=true;out=g;return true;
}
}
inline bool BuildLine(const NCG76SphereCoordinates::FrozenFrame& frame,const Point8& target,
    double feed,Geometry& output) noexcept
{
    const auto frozenFrame=frame;const Point8 frozenTarget=target;output=Geometry{};
    return Detail::DeriveLine(frozenFrame,frozenTarget,feed,output);
}
inline bool BuildArc(const NCG76SphereCoordinates::FrozenFrame& frame,const Point8& target,
    const Point3& center,const Point3& normal,int direction,double feed,bool fullCircle,Geometry& output) noexcept
{
    const auto frozenFrame=frame;const Point8 frozenTarget=target;const Point3 frozenCenter=center,frozenNormal=normal;output=Geometry{};
    return Detail::DeriveArc(frozenFrame,frozenTarget,frozenCenter,frozenNormal,direction,feed,fullCircle,output);
}
// Full admission check. Never call this from the periodic interpolation path.
inline bool Valid(const Geometry& g) noexcept
{
    if(!g.valid||g.version!=Version)return false;
    if(g.kind==Kind::OrbitContour){
        Geometry expected{};
        if(NCG76OrbitContour::ModeP(g)==15U)
            return NCG76OrbitContour::BuildTimedP15(g.frame,g.orbit.source.end,g.orbit.source.radiusMm,
                g.orbit.source.shapeAngleDeg,g.orbit.source.feedMmMin,g.orbit.finishEnabled,expected)&&SameGeometry(g,expected);
        if(NCG76OrbitContour::ModeP(g)==14U)
            return NCG76OrbitContour::BuildTimedP14(g.frame,g.orbit.source.end,g.orbit.source.radiusMm,
                g.orbit.source.shapeAngleDeg,g.orbit.source.feedMmMin,g.orbit.finishEnabled,expected)&&SameGeometry(g,expected);
        if(NCG76OrbitContour::ModeP(g)==13U)
            return NCG76OrbitContour::BuildTimedP13(g.frame,g.orbit.source.end,g.orbit.source.radiusMm,
                g.orbit.source.shapeAngleDeg,g.orbit.source.feedMmMin,g.orbit.finishEnabled,expected)&&SameGeometry(g,expected);
        if(NCG76OrbitContour::ModeP(g)==12U)
            return NCG76OrbitContour::BuildTimedP12(g.frame,g.orbit.source.end,g.orbit.source.radiusMm,
                g.orbit.source.shapeAngleDeg,g.orbit.source.feedMmMin,g.orbit.finishEnabled,expected)&&SameGeometry(g,expected);
        if(NCG76OrbitContour::ModeP(g)!=11U)return false;
        return NCG76OrbitContour::BuildTimedP11(g.frame,g.orbit.source.end,g.orbit.source.radiusMm,
            g.orbit.source.shapeAngleDeg,g.orbit.source.feedMmMin,g.orbit.finishEnabled,expected)&&SameGeometry(g,expected);
    }
    if(g.kind!=Kind::Sphere){
        Geometry expected{};
        if(g.kind==Kind::Line)return Detail::DeriveLine(g.frame,g.endMCS,g.feedMmMin,expected)&&SameGeometry(g,expected);
        if(g.kind!=Kind::Arc)return false;
        bool full=true;for(unsigned a=0U;a<3U;++a)full=full&&Detail::Bits(g.startMCS[a],g.endMCS[a]);
        return Detail::DeriveArc(g.frame,g.endMCS,g.arcCenterMCS,g.arcNormal,g.arcDirection,g.feedMmMin,full,expected)&&SameGeometry(g,expected);
    }
    NCG76SphereCoordinates::NativePlan source{};Geometry expected{};
    return NCG76SphereCoordinates::BuildNative(g.frame,g.descriptor,source)==NCG76SphereCoordinates::Status::Ok&&
        Detail::Derive(source,expected)&&SameGeometry(g,expected);
}
inline bool LayerAt(const Geometry& g,unsigned index,Layer& output) noexcept
{
    if(Detail::Overlap(&g,sizeof(g),&output,sizeof(output)))return false;
    output=Layer{};
    if(!g.valid||g.version!=Version||index>g.circleLayerCount)return false;
    if(g.kind==Kind::Line||g.kind==Kind::Arc){if(index!=0U)return false;output.center=g.endMCS;output.terminal=true;return true;}
    if(g.kind==Kind::OrbitContour){
        if(index!=0U||NCG76OrbitContour::ModeP(g)!=11U)return false;
        output.center=g.endMCS;output.radiusMm=g.radiusMm;output.arcLengthMm=TwoPi*g.radiusMm;
        return NCG76OrbitContour::FinishEnabled(g);
    }
    if(g.kind!=Kind::Sphere)return false;
    Layer layer{};if(!Detail::RawLayer(g,index,layer))return false;
    output=layer;return true;
}
inline bool ArcAt(const Geometry& g,double angleRad,Point8& position,Point3& tangent,Point3& curvature) noexcept
{
    if(Detail::Overlap(&g,sizeof(g),&position,sizeof(position))||Detail::Overlap(&g,sizeof(g),&tangent,sizeof(tangent))||
       Detail::Overlap(&g,sizeof(g),&curvature,sizeof(curvature))||Detail::Overlap(&position,sizeof(position),&tangent,sizeof(tangent))||
       Detail::Overlap(&position,sizeof(position),&curvature,sizeof(curvature))||Detail::Overlap(&tangent,sizeof(tangent),&curvature,sizeof(curvature)))return false;
    position=Point8{};tangent=Point3{};curvature=Point3{};
    if(!g.valid||g.version!=Version||g.kind!=Kind::Arc||!std::isfinite(angleRad)||angleRad<0.0||angleRad>g.arcSweepRad)return false;
    double sine=0.0,cosine=1.0;
    if(angleRad==Pi*.5){sine=1.0;cosine=0.0;}
    else if(angleRad==Pi){sine=0.0;cosine=-1.0;}
    else if(angleRad==Pi*1.5){sine=-1.0;cosine=0.0;}
    else if(angleRad!=0.0&&angleRad!=TwoPi){sine=std::sin(angleRad);cosine=std::cos(angleRad);}
    position=g.startMCS;
    for(unsigned a=0U;a<3U;++a){const double radial=g.u[a]*cosine+g.v[a]*sine;
        position[a]=g.arcCenterMCS[a]+g.radiusMm*radial;
        tangent[a]=-g.u[a]*sine+g.v[a]*cosine;curvature[a]=-radial/g.radiusMm;}
    if(angleRad==0.0)position=g.startMCS;else if(angleRad==g.arcSweepRad)position=g.endMCS;
    if(!Detail::Finite(position)||!Detail::Finite(tangent)||!Detail::Finite(curvature))return false;
    for(unsigned a=0U;a<3U;++a)if(position[a]<g.boundsMinMCS[a]||position[a]>g.boundsMaxMCS[a])return false;
    return true;
}
inline bool NominalAt(const Geometry& g,double fraction,Point8& position,Point3& tangent,Point3& curvature) noexcept
{
    if(Detail::Overlap(&g,sizeof(g),&position,sizeof(position))||Detail::Overlap(&g,sizeof(g),&tangent,sizeof(tangent))||
       Detail::Overlap(&g,sizeof(g),&curvature,sizeof(curvature))||Detail::Overlap(&position,sizeof(position),&tangent,sizeof(tangent))||
       Detail::Overlap(&position,sizeof(position),&curvature,sizeof(curvature))||Detail::Overlap(&tangent,sizeof(tangent),&curvature,sizeof(curvature)))return false;
    position=Point8{};tangent=Point3{};curvature=Point3{};
    if(!g.valid||g.version!=Version||!std::isfinite(fraction)||fraction<0.0||fraction>1.0)return false;
    if(g.kind==Kind::Arc)return ArcAt(g,fraction==1.0?g.arcSweepRad:fraction*g.arcSweepRad,position,tangent,curvature);
    if(g.kind!=Kind::Line||(g.travelMm==0.0&&fraction!=0.0))return false;
    position=fraction==1.0?g.endMCS:g.startMCS;tangent=g.axis;curvature=Point3{};
    if(fraction>0.0&&fraction<1.0)for(unsigned a=0U;a<3U;++a)position[a]=g.startMCS[a]+(g.endMCS[a]-g.startMCS[a])*fraction;
    return Detail::Finite(position)&&Detail::Finite(tangent);
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
    if(g.kind==Kind::Arc)return layerIndex==0U&&ArcAt(g,angleRad,position,tangent,curvature);
    if(g.kind!=Kind::Sphere&&g.kind!=Kind::OrbitContour)return false;
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

namespace NCG76OrbitContour
{
namespace Detail
{
inline unsigned RawMode(const Geometry& g) noexcept
{return g.orbit.contourMode==0U?11U:g.orbit.contourMode==12U?12U:g.orbit.contourMode==13U?13U:g.orbit.contourMode==14U?14U:g.orbit.contourMode==15U?15U:0U;}
inline bool Ready(const Geometry& g) noexcept
{
    return g.kind==NCG76SphereRuntimeGeometry::Kind::OrbitContour&&g.valid&&
        g.version==NCG76SphereRuntimeGeometry::Version&&g.orbit.valid&&g.orbit.version==Version&&
        NCG76SphereRuntimeGeometry::Detail::Positive(g.radiusMm)&&
        NCG76SphereRuntimeGeometry::Detail::Positive(g.orbit.nominalLengthMm)&&
        RawMode(g)!=0U&&g.orbit.source.targetAxis==(RawMode(g)!=11U);
}
inline bool SameFixed(const Point8& a,const Point8& b) noexcept
{for(unsigned i=3U;i<8U;++i)if(std::memcmp(&a[i],&b[i],sizeof(double))!=0)return false;return true;}
inline bool NativeVertex(const Geometry& g,unsigned vertex,Point8& output) noexcept
{
    if(vertex>NominalLegCount)return false;
    if(vertex==0U){output=g.startMCS;return true;}
    if(vertex==NominalLegCount){output=g.endMCS;return true;}
    double x=0.0,y=0.0;
    if(!NCG76Mode::UnitBoundary(RawMode(g),vertex,x,y))return false;
    const double t=static_cast<double>(vertex)/24.0;
    output=g.startMCS;
    for(unsigned a=0U;a<3U;++a){
        const double center=vertex==24U?g.orbit.centerEndMCS[a]:
            g.orbit.centerStartMCS[a]+(g.orbit.centerEndMCS[a]-g.orbit.centerStartMCS[a])*t;
        output[a]=center+(g.radiusMm*t)*(g.orbit.unitU[a]*x+g.orbit.unitV[a]*y);
    }
    return NCG76SphereRuntimeGeometry::Detail::Finite(output)&&SameFixed(output,g.startMCS);
}
inline double Distance(const Point8& a,const Point8& b) noexcept
{return std::hypot(std::hypot(b[0]-a[0],b[1]-a[1]),b[2]-a[2]);}
inline bool NativeFinishVertex(const Geometry& g,unsigned vertex,Point8& output) noexcept
{
    const unsigned mode=RawMode(g);
    if((mode!=12U&&mode!=13U&&mode!=14U&&mode!=15U)||vertex>4U)return false;
    // Do not independently spell the seam; it is the exact released final
    // grow endpoint, including coordinate cancellation and signed zero bits.
    if(vertex==0U||vertex==4U)return NativeVertex(g,24U,output);
    double x=0.0,y=0.0;
    if(!NCG76Mode::UnitBoundary(mode,vertex*6U,x,y))return false;
    output=g.startMCS;
    for(unsigned a=0U;a<3U;++a)
        output[a]=g.orbit.centerEndMCS[a]+g.radiusMm*(g.orbit.unitU[a]*x+g.orbit.unitV[a]*y);
    return NCG76SphereRuntimeGeometry::Detail::Finite(output)&&SameFixed(output,g.startMCS);
}
inline bool Within(const Geometry& g,const Point8& p) noexcept
{
    if(!NCG76SphereRuntimeGeometry::Detail::Finite(p)||!SameFixed(p,g.startMCS))return false;
    for(unsigned a=0U;a<3U;++a)if(p[a]<g.boundsMinMCS[a]||p[a]>g.boundsMaxMCS[a])return false;
    return true;
}
}

inline unsigned ModeP(const Geometry& g) noexcept
{return Detail::Ready(g)?Detail::RawMode(g):0U;}
inline bool IsFourSideContour(const Geometry& g) noexcept
{const unsigned mode=ModeP(g);return mode==12U||mode==13U||mode==14U||mode==15U;}
inline bool EndpointAt(const Geometry& g,unsigned vertex,Point8& output) noexcept
{
    if(NCG76SphereRuntimeGeometry::Detail::Overlap(&g,sizeof(g),&output,sizeof(output)))return false;
    output=Point8{};
    return Detail::Ready(g)&&Detail::NativeVertex(g,vertex,output)&&Detail::Within(g,output);
}
inline bool NominalCenterAt(const Geometry& g,unsigned index,double fraction,Point8& output) noexcept
{
    if(NCG76SphereRuntimeGeometry::Detail::Overlap(&g,sizeof(g),&output,sizeof(output)))return false;
    output=Point8{};
    if(!Detail::Ready(g)||index>=NominalLegCount||!std::isfinite(fraction)||fraction<0.0||fraction>1.0)return false;
    if(index==24U){output=g.endMCS;return true;}
    const double t=(static_cast<double>(index)+fraction)/24.0;
    output=g.startMCS;
    for(unsigned a=0U;a<3U;++a)output[a]=t==1.0?g.orbit.centerEndMCS[a]:
        g.orbit.centerStartMCS[a]+(g.orbit.centerEndMCS[a]-g.orbit.centerStartMCS[a])*t;
    return Detail::Within(g,output);
}
inline bool NominalLegAt(const Geometry& g,unsigned index,Point8& begin,Point8& end,
    Point8& center,double& length,double& prefix) noexcept
{
    using NCG76SphereRuntimeGeometry::Detail::Overlap;
    const void* pointers[]={&begin,&end,&center,&length,&prefix};
    const std::size_t sizes[]={sizeof(begin),sizeof(end),sizeof(center),sizeof(length),sizeof(prefix)};
    for(unsigned a=0U;a<5U;++a){
        if(Overlap(&g,sizeof(g),pointers[a],sizes[a]))return false;
        for(unsigned b=0U;b<a;++b)if(Overlap(pointers[a],sizes[a],pointers[b],sizes[b]))return false;
    }
    begin=end=center=Point8{};length=prefix=0.0;
    if(!Detail::Ready(g)||index>=NominalLegCount)return false;
    const unsigned block=index/CoarseStride;
    prefix=g.orbit.coarsePrefix[block];
    if(!std::isfinite(prefix)||prefix<0.0)return false;
    Point8 previous{},next{};
    if(!Detail::NativeVertex(g,block*CoarseStride,previous))return false;
    for(unsigned i=block*CoarseStride;i<=index;++i){
        if(!Detail::NativeVertex(g,i+1U,next))return false;
        const double span=Detail::Distance(previous,next);
        if(!std::isfinite(span)||span<=0.0)return false;
        if(i==index){begin=previous;end=next;length=span;break;}
        prefix+=span;previous=next;
    }
    return NominalCenterAt(g,index,1.0,center)&&Detail::Within(g,begin)&&Detail::Within(g,end)&&
        std::isfinite(prefix)&&prefix>=0.0&&prefix<g.orbit.nominalLengthMm&&
        std::isfinite(prefix+length)&&prefix+length<=std::nextafter(g.orbit.nominalLengthMm,
            std::numeric_limits<double>::infinity());
}
inline double NominalLength(const Geometry& g) noexcept
{return Detail::Ready(g)?g.orbit.nominalLengthMm:0.0;}
inline bool FinishEnabled(const Geometry& g) noexcept
{return Detail::Ready(g)&&g.orbit.finishEnabled;}
inline unsigned FinishLegCount(const Geometry& g) noexcept
{return IsFourSideContour(g)&&FinishEnabled(g)?4U:0U;}
inline bool FinishVertexAt(const Geometry& g,unsigned vertex,Point8& output) noexcept
{
    if(NCG76SphereRuntimeGeometry::Detail::Overlap(&g,sizeof(g),&output,sizeof(output)))return false;
    output=Point8{};
    return FinishLegCount(g)==4U&&Detail::NativeFinishVertex(g,vertex,output)&&Detail::Within(g,output);
}
inline bool FinishLegAt(const Geometry& g,unsigned side,Point8& begin,Point8& end,
    Point8& center,double& length,double& prefix) noexcept
{
    using NCG76SphereRuntimeGeometry::Detail::Overlap;
    const void* pointers[]={&begin,&end,&center,&length,&prefix};
    const std::size_t sizes[]={sizeof(begin),sizeof(end),sizeof(center),sizeof(length),sizeof(prefix)};
    for(unsigned a=0U;a<5U;++a){
        if(Overlap(&g,sizeof(g),pointers[a],sizes[a]))return false;
        for(unsigned b=0U;b<a;++b)if(Overlap(pointers[a],sizes[a],pointers[b],sizes[b]))return false;
    }
    begin=end=center=Point8{};length=prefix=0.0;
    if(FinishLegCount(g)!=4U||side>=4U)return false;
    Point8 previous{},next{};
    if(!Detail::NativeFinishVertex(g,0U,previous)||!Detail::Within(g,previous))return false;
    for(unsigned i=0U;i<=side;++i){
        if(!Detail::NativeFinishVertex(g,i+1U,next)||!Detail::Within(g,next))return false;
        const double span=Detail::Distance(previous,next);
        if(!std::isfinite(span)||span<=0.0)return false;
        if(i==side){begin=previous;end=next;length=span;center=g.endMCS;return true;}
        if(!std::isfinite(prefix+span)||prefix+span<=prefix)return false;
        prefix+=span;previous=next;
    }
    return false;
}
inline double FinishEntryLength(const Geometry& g) noexcept
{
    Point8 vertex{};
    if(!FinishVertexAt(g,0U,vertex))return 0.0;
    const double length=Detail::Distance(g.endMCS,vertex);
    return std::isfinite(length)&&length>0.0?length:0.0;
}
inline double FinishLength(const Geometry& g) noexcept
{
    Point8 begin{},end{},center{};double length=0.0,prefix=0.0;
    if(!FinishLegAt(g,3U,begin,end,center,length,prefix))return 0.0;
    const double total=prefix+length;
    return std::isfinite(total)&&total>prefix?total:0.0;
}

inline bool BuildTimedContour(unsigned modeP,const NCG76SphereCoordinates::FrozenFrame& inputFrame,const Point3& programTarget,
    double radiusMm,double shapeAngleDeg,double feedMmMin,bool finish,Geometry& output) noexcept
{
    const auto frame=inputFrame;const Point3 target=programTarget;
    output=Geometry{};
    if((modeP!=11U&&modeP!=12U&&modeP!=13U&&modeP!=14U&&modeP!=15U)||!NCG76SphereCoordinates::ValidFrame(frame)||frame.translation.rotationPlane!=17||
        !NCG76::ValidShapeAngle(shapeAngleDeg)||!std::isfinite(radiusMm)||radiusMm<=0.0||radiusMm>NCG76::MaximumRadiusMm||
        !std::isfinite(feedMmMin)||feedMmMin<=0.0||feedMmMin>NCG76::MaximumFeedMmMin||
        !NCG76SphereRuntimeGeometry::Detail::Finite(target))return false;
    NCG76::PlanInput source{};
    for(unsigned a=0U;a<3U;++a){source.start[a]=frame.programStart[a];source.end[a]=target[a];}
    source.radiusMm=radiusMm;source.feedMmMin=feedMmMin;source.shapeAngleDeg=shapeAngleDeg;
    source.targetAxis=modeP!=11U;
    Point3 sourceDelta{};double sourceEnvelope=0.0;
    if(!NCG76::Detail::ValidInput(source,sourceDelta,sourceEnvelope))return false;
    NCG76::Detail::OrbitBasis basis{};
    if(!NCG76::Detail::BuildOrbitBasis(source,sourceDelta,basis))return false;
    Geometry g{};g.kind=NCG76SphereRuntimeGeometry::Kind::OrbitContour;
    ::new(static_cast<void*>(&g.orbit)) Descriptor{};
    g.orbit.contourMode=modeP==11U?0U:static_cast<std::uint8_t>(modeP);
    g.frame=frame;g.startMCS=frame.startMCS;
    if(NCG76SphereCoordinates::MapPoint(frame,target,g.endMCS)!=NCG76SphereCoordinates::Status::Ok)return false;
    // G17's allowed affine frame keeps program Z separable. Preserve a
    // deliberately stationary accepted Z exactly instead of rounding its
    // inverse/forward spelling through fixed offsets a second time.
    if(sourceDelta[2]==0.0)g.endMCS[2]=g.startMCS[2];
    g.orbit.source=source;g.orbit.finishEnabled=finish;g.orbit.version=Version;
    const auto rotatedU=NCG76::Detail::OrbitOffset(Point3{{1.0,0.0,0.0}},shapeAngleDeg,basis);
    const auto rotatedV=NCG76::Detail::OrbitOffset(Point3{{0.0,1.0,0.0}},shapeAngleDeg,basis);
    if(NCG76SphereCoordinates::MapVector(frame,rotatedU,g.u)!=NCG76SphereCoordinates::Status::Ok||
       NCG76SphereCoordinates::MapVector(frame,rotatedV,g.v)!=NCG76SphereCoordinates::Status::Ok)return false;
    g.radiusMm=radiusMm*frame.nativeScale;g.feedMmMin=feedMmMin;
    if(!std::isfinite(g.radiusMm)||g.radiusMm<=0.0||g.radiusMm>NCG76::MaximumRadiusMm)return false;
    Point3 centerDelta{};
    for(unsigned a=0U;a<3U;++a){
        g.u[a]/=frame.nativeScale;g.v[a]/=frame.nativeScale;
        g.orbit.unitU[a]=g.u[a];g.orbit.unitV[a]=g.v[a];
        g.orbit.centerStartMCS[a]=g.startMCS[a];g.orbit.centerEndMCS[a]=g.endMCS[a];
        centerDelta[a]=g.endMCS[a]-g.startMCS[a];
    }
    const double centerTravel=NCG76SphereRuntimeGeometry::Detail::Norm(centerDelta);
    const double uu=NCG76SphereRuntimeGeometry::Detail::Norm(g.u),vv=NCG76SphereRuntimeGeometry::Detail::Norm(g.v);
    if(!std::isfinite(centerTravel)||centerTravel>NCG76::MaximumCenterTravelMm||
        std::fabs(uu-1.0)>NCG76SphereRuntimeGeometry::OrthonormalTolerance||
        std::fabs(vv-1.0)>NCG76SphereRuntimeGeometry::OrthonormalTolerance||
        std::fabs(NCG76SphereRuntimeGeometry::Detail::Dot(g.u,g.v))>NCG76SphereRuntimeGeometry::OrthonormalTolerance)return false;
    if(centerTravel>0.0)for(unsigned a=0U;a<3U;++a)g.axis[a]=centerDelta[a]/centerTravel;
    else g.axis=NCG76SphereRuntimeGeometry::Detail::Cross(g.u,g.v);
    double envelope=0.0;
    for(unsigned a=0U;a<3U;++a){
        const double amplitude=std::hypot(g.u[a],g.v[a]);
        envelope=(std::max)(envelope,(std::max)(std::fabs(g.startMCS[a]),std::fabs(g.endMCS[a]))+g.radiusMm*amplitude);
        g.maxCircleTangentAbs[a]=modeP!=11U||amplitude==0.0?0.0:std::nextafter(amplitude*(1.0+16.0*std::numeric_limits<double>::epsilon()),
            std::numeric_limits<double>::infinity());
        g.maxCircleCurvatureAbs[a]=modeP!=11U||amplitude==0.0?0.0:g.maxCircleTangentAbs[a]/g.radiusMm;
    }
    g.arithmeticErrorBoundMm=std::nextafter(envelope*(64.0*std::numeric_limits<double>::epsilon()),
        std::numeric_limits<double>::infinity());
    if(!std::isfinite(g.arithmeticErrorBoundMm)||g.arithmeticErrorBoundMm<=0.0||
        g.arithmeticErrorBoundMm>NCG76SphereRuntimeGeometry::MaximumArithmeticErrorMm)return false;
    g.orbit.arithmeticErrorMm=g.arithmeticErrorBoundMm;
    g.coordinateErrorBoundMm=std::nextafter(NCG76SphereCoordinates::MaximumMappingErrorMm+g.arithmeticErrorBoundMm,
        std::numeric_limits<double>::infinity());
    g.boundsMinMCS=g.boundsMaxMCS=g.startMCS;
    for(unsigned a=0U;a<3U;++a){
        const double excursion=g.radiusMm*std::hypot(g.u[a],g.v[a]);
        g.boundsMinMCS[a]=std::nextafter((std::min)(g.startMCS[a],g.endMCS[a])-excursion-g.coordinateErrorBoundMm,
            -std::numeric_limits<double>::infinity());
        g.boundsMaxMCS[a]=std::nextafter((std::max)(g.startMCS[a],g.endMCS[a])+excursion+g.coordinateErrorBoundMm,
            std::numeric_limits<double>::infinity());
        if(!std::isfinite(g.boundsMinMCS[a])||!std::isfinite(g.boundsMaxMCS[a])||
            !std::isfinite(g.maxCircleCurvatureAbs[a]))return false;
    }
    // Compare every procedural chord endpoint with the existing contour program
    // sample and frozen point mapping. No 6168-byte NativePlan is materialized
    // on the nested RT admission stack. Per-tick evaluation uses native data.
    Point8 previous=g.startMCS,current{},mapped{};double total=0.0,minimum=g.radiusMm;
    for(unsigned i=0U;i<NominalLegCount;++i){
        if(i%CoarseStride==0U)g.orbit.coarsePrefix[i/CoarseStride]=total;
        if(!Detail::NativeVertex(g,i+1U,current)||!Detail::Within(g,current))return false;
        Point3 program=target;
        if(i<24U){
            const unsigned k=i+1U;const double t=static_cast<double>(k)/24.0;double x=0.0,y=0.0;
            if(!NCG76Mode::UnitBoundary(modeP,k,x,y))return false;
            const auto orbit=NCG76::Detail::OrbitOffset(Point3{{radiusMm*t*x,radiusMm*t*y,0.0}},shapeAngleDeg,basis);
            for(unsigned a=0U;a<3U;++a)program[a]=(k==24U?target[a]:source.start[a]+sourceDelta[a]*t)+orbit[a];
        }
        if(NCG76SphereCoordinates::MapPoint(frame,program,mapped)!=NCG76SphereCoordinates::Status::Ok||
            !NCG76SphereCoordinates::Detail::WithinError(Point3{{current[0],current[1],current[2]}},
                Point3{{mapped[0],mapped[1],mapped[2]}})||!Detail::SameFixed(current,g.startMCS))return false;
        const double length=Detail::Distance(previous,current);
        if(!std::isfinite(length)||length<=0.0||!std::isfinite(total+length)||total+length<=total)return false;
        minimum=(std::min)(minimum,length);total+=length;previous=current;
    }
    if(g.coordinateErrorBoundMm>minimum/64.0)return false;
    g.travelMm=g.orbit.nominalLengthMm=total;g.orbit.valid=true;
    g.minimumCircleRadiusMm=modeP==11U?g.radiusMm:0.0;
    g.maximumCircleArcLengthMm=modeP==11U?NCG76SphereRuntimeGeometry::TwoPi*g.radiusMm:0.0;
    g.version=NCG76SphereRuntimeGeometry::Version;g.valid=true;
    if(modeP==11U&&(!std::isfinite(g.maximumCircleArcLengthMm)||g.maximumCircleArcLengthMm<=0.0))return false;
    if(modeP!=11U&&finish){
        const double entry=FinishEntryLength(g),perimeter=FinishLength(g);
        if(!(entry>0.0)||!(perimeter>0.0)||g.coordinateErrorBoundMm>entry/64.0)return false;
        for(unsigned side=0U;side<4U;++side){
            Point8 begin{},end{},center{};double length=0.0,prefix=0.0;
            if(!FinishLegAt(g,side,begin,end,center,length,prefix)||g.coordinateErrorBoundMm>length/64.0)return false;
        }
    }
    output=g;return true;
}
inline bool BuildTimedP11(const NCG76SphereCoordinates::FrozenFrame& frame,const Point3& target,
    double radiusMm,double shapeAngleDeg,double feedMmMin,bool finish,Geometry& output) noexcept
{return BuildTimedContour(11U,frame,target,radiusMm,shapeAngleDeg,feedMmMin,finish,output);}
inline bool BuildTimedP12(const NCG76SphereCoordinates::FrozenFrame& frame,const Point3& target,
    double radiusMm,double shapeAngleDeg,double feedMmMin,bool finish,Geometry& output) noexcept
{return BuildTimedContour(12U,frame,target,radiusMm,shapeAngleDeg,feedMmMin,finish,output);}
inline bool BuildTimedP13(const NCG76SphereCoordinates::FrozenFrame& frame,const Point3& target,
    double radiusMm,double shapeAngleDeg,double feedMmMin,bool finish,Geometry& output) noexcept
{return BuildTimedContour(13U,frame,target,radiusMm,shapeAngleDeg,feedMmMin,finish,output);}
inline bool BuildTimedP14(const NCG76SphereCoordinates::FrozenFrame& frame,const Point3& target,
    double radiusMm,double shapeAngleDeg,double feedMmMin,bool finish,Geometry& output) noexcept
{return BuildTimedContour(14U,frame,target,radiusMm,shapeAngleDeg,feedMmMin,finish,output);}
inline bool BuildTimedP15(const NCG76SphereCoordinates::FrozenFrame& frame,const Point3& target,
    double radiusMm,double shapeAngleDeg,double feedMmMin,bool finish,Geometry& output) noexcept
{return BuildTimedContour(15U,frame,target,radiusMm,shapeAngleDeg,feedMmMin,finish,output);}
}
