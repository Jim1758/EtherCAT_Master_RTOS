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
enum class Kind : std::uint8_t { Sphere, Line, Arc };
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
    if(g.kind!=Kind::Sphere)return false;
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
