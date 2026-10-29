#pragma once

#include "NCG76SpherePlan.h"
#include "NCTranslationSnapshot.h"
#include "NCXYZCUVFeedScope.h"
#include "NCEccentricCPath.h"
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

// EDM285: pure coordinate adapter; no queue, Motion, PDO or discharge admission.
// All lengths enter as canonical mm, F remains native mm/min (never scaled).
// Supported affine frame: G21 Cartesian G90/G91; G17/G18/G19; ext/WCS;
// G43/G44/G49 fixed XYZ H; G68; G168's one matching normal-axis rotation;
// positive uniform XYZ scale and XYZ mirrors. C/U/V and absent slots stay
// bitwise fixed at the accepted native start. Cross-plane WORK tilt, cutter
// compensation, polar notation and simultaneous rotary motion are not admitted.
// Eccentric H49 is an identity only when the untouched snapshot is valid.
// Eccentric H43/H44 additionally require the existing NCEccentricCDetail
// Scope/Role==physical C proof; its fixed signed Offset uses physical start C,
// not C's WCS representation. Other eccentric combinations are unsupported.
// Output is commanded native geometry BEFORE the existing final-send pitch /
// backlash correction. Neither correction is duplicated or claimed here.
namespace NCG76SphereCoordinates
{
using Point3=NCG76Sphere::Point3;
using Point8=std::array<double,8U>;
constexpr double MaximumMappingErrorMm=1.e-9;
enum class Status : unsigned
{
    Ok, InvalidSnapshot, UnsupportedUnits, UnsupportedAxisScope,
    UnsupportedEccentric, UnsupportedWorkRotation, UnsupportedCutter,
    UnsupportedPolar, InvalidStart, InvalidInput, InvalidDescriptor,
    NumericRange, PrecisionLoss, FrameMismatch, InvalidCursor
};
struct FrozenFrame
{
    NCTranslationSnapshot translation{};
    Point8 startMCS{},programStart{};
    Point3 signedEccentricOffset{};
    double nativeScale=0.0;
    bool fixedEccentric=false,valid=false;
};
struct NativePlan
{
    NCG76Sphere::Descriptor descriptor{};
    FrozenFrame frame{};
    double radiusMm=0.0,layerStepMm=0.0,travelMm=0.0;
    double chordErrorBoundMm=0.0,coordinateErrorBoundMm=0.0;
    Point3 axis{},u{},v{};
    Point8 endMCS{};
    bool valid=false;
};
namespace Detail
{
inline bool Bits(double a,double b) noexcept {return std::memcmp(&a,&b,sizeof(double))==0;}
template<std::size_t N> inline bool Same(const std::array<double,N>& a,const std::array<double,N>& b) noexcept
{return std::memcmp(a.data(),b.data(),N*sizeof(double))==0;}
template<std::size_t N> inline bool Finite(const std::array<double,N>& point) noexcept
{for(double value:point)if(!std::isfinite(value))return false;return true;}
inline double Norm(const Point3& vector) noexcept
{return std::hypot(std::hypot(vector[0],vector[1]),vector[2]);}
inline bool WithinError(const Point3& actual,const Point3& expected) noexcept
{
    Point3 difference{};for(unsigned a=0;a<3;++a)difference[a]=actual[a]-expected[a];
    const double error=Norm(difference);return std::isfinite(error)&&error<=MaximumMappingErrorMm;
}
inline bool PreservesVector(const Point3& actual,const Point3& expected) noexcept
{
    const double length=Norm(expected),actualLength=Norm(actual);
    Point3 difference{};for(unsigned a=0;a<3;++a)difference[a]=actual[a]-expected[a];
    const double error=Norm(difference);
    return std::isfinite(length)&&std::isfinite(actualLength)&&std::isfinite(error)&&
        error<=MaximumMappingErrorMm&&(length==0.0?actualLength==0.0:actualLength>0.0&&error<=length/64.0);
}
inline Status Supported(const NCTranslationSnapshot& s) noexcept
{
    if(s.axisIdentity.eccentricEnabled==1U&&s.toolLengthMode!=49){
        unsigned role=8U;
        if(!NCEccentricCDetail::Scope(s)||!NCEccentricCDetail::Role(s,role)||role!=3U||
            s.axisIdentity.address[0]!='X'||s.axisIdentity.address[1]!='Y'||
            s.axisIdentity.address[2]!='Z'||s.axisIdentity.address[3]!='C')return Status::UnsupportedEccentric;
    }
    if(s.unitsMode!=21)return Status::UnsupportedUnits;
    if(s.polarMode!=15)return Status::UnsupportedPolar;
    if(s.cutterMode!=40)return Status::UnsupportedCutter;
    double workAngle=0.0;
    if(!TryGetNCTranslationWorkPlaneAngle(s,workAngle))return Status::UnsupportedWorkRotation;
    if(!IsNCTranslationSnapshotValid(s))return Status::InvalidSnapshot;
    if(!IsNCXYZCUVFeedAxisScopeValid(s.axisIdentity)||s.axisIdentity.exists[6]!=0||s.axisIdentity.exists[7]!=0)
        return Status::UnsupportedAxisScope;
    return Status::Ok;
}
inline bool RawVector(const FrozenFrame& frame,const Point3& point,Point3& output) noexcept
{
    NCArcPlaneAxes plane{};const auto& source=frame.translation;
    if(!TryGetNCArcPlaneAxes(source.rotationPlane,plane))return false;
    if(!NCTranslationRotateArcVector(source,point[plane.u],point[plane.v],output[plane.u],output[plane.v]))return false;
    output[plane.normal]=NCTranslationScaleMirrorAxisVector(source,point[plane.normal],plane.normal);
    return Finite(output);
}
inline bool RawForward(const FrozenFrame& frame,const Point3& point,Point8& output) noexcept
{
    output=frame.startMCS;
    if(frame.translation.distanceMode==91){
        Point3 vector{};if(!RawVector(frame,point,vector))return false;
        for(unsigned a=0;a<3;++a)output[a]=frame.startMCS[a]+vector[a];
    }else if(frame.fixedEccentric){
        for(unsigned a=0;a<3;++a)output[a]=(point[a]+(frame.translation.extOffsetMM[a]+
            frame.translation.wcsOffsetMM[a]))+frame.signedEccentricOffset[a];
    }else{
        Point8 program=frame.programStart,translated{};
        for(unsigned a=0;a<3;++a)program[a]=point[a];
        NCTranslationForwardPoint(frame.translation,program.data(),translated.data());
        for(unsigned a=0;a<3;++a)output[a]=translated[a];
    }
    return Finite(output);
}
}
inline bool SameFrame(const FrozenFrame& a,const FrozenFrame& b) noexcept
{
    return a.valid==b.valid&&a.fixedEccentric==b.fixedEccentric&&
        SameNCTranslationSnapshot(a.translation,b.translation)&&Detail::Same(a.startMCS,b.startMCS)&&
        Detail::Same(a.programStart,b.programStart)&&Detail::Same(a.signedEccentricOffset,b.signedEccentricOffset)&&
        Detail::Bits(a.nativeScale,b.nativeScale);
}
inline Status BuildFrame(const NCTranslationSnapshot& source,const Point8& acceptedStart,FrozenFrame& output) noexcept
{
    // Copy inputs before clearing output, including callers rebuilding in place.
    const NCTranslationSnapshot frozen=source;const Point8 initial=acceptedStart;output=FrozenFrame{};
    const auto status=Detail::Supported(frozen);if(status!=Status::Ok)return status;
    if(!Detail::Finite(initial))return Status::InvalidStart;
    FrozenFrame frame{};frame.translation=frozen;frame.startMCS=initial;frame.nativeScale=frozen.scalingFactor;
    frame.fixedEccentric=frozen.axisIdentity.eccentricEnabled==1U&&frozen.toolLengthMode!=49;
    if(frame.fixedEccentric&&!NCEccentricCDetail::Offset(frozen,initial[3],frame.signedEccentricOffset))
        return Status::UnsupportedEccentric;
    if(frozen.distanceMode==90){
        if(frame.fixedEccentric){
            for(unsigned a=0;a<8;++a){
                const double nominal=a<3?initial[a]-frame.signedEccentricOffset[a]:initial[a];
                frame.programStart[a]=nominal-(frozen.extOffsetMM[a]+frozen.wcsOffsetMM[a]);
            }
        }else NCTranslationInversePoint(frozen,initial.data(),frame.programStart.data());
        if(!Detail::Finite(frame.programStart))return Status::NumericRange;
        Point8 reconstructed{};Point3 program{{frame.programStart[0],frame.programStart[1],frame.programStart[2]}};
        if(!Detail::RawForward(frame,program,reconstructed))return Status::NumericRange;
        if(!Detail::WithinError({{reconstructed[0],reconstructed[1],reconstructed[2]}},{{initial[0],initial[1],initial[2]}}))
            return Status::PrecisionLoss;
    }
    frame.valid=true;output=frame;return Status::Ok;
}
inline bool ValidFrame(const FrozenFrame& frame) noexcept
{
    FrozenFrame expected{};return frame.valid&&BuildFrame(frame.translation,frame.startMCS,expected)==Status::Ok&&SameFrame(frame,expected);
}
inline Status MapVector(const FrozenFrame& inputFrame,const Point3& vector,Point3& output) noexcept
{
    const FrozenFrame frame=inputFrame;const Point3 frozen=vector;output=Point3{};
    if(!ValidFrame(frame))return Status::FrameMismatch;
    if(!Detail::Finite(frozen))return Status::InvalidInput;
    Point3 native{};if(!Detail::RawVector(frame,frozen,native))return Status::NumericRange;
    const double authoredNorm=Detail::Norm(frozen),expectedNorm=authoredNorm*frame.nativeScale,nativeNorm=Detail::Norm(native);
    if(!std::isfinite(authoredNorm)||!std::isfinite(expectedNorm)||!std::isfinite(nativeNorm)||
        (authoredNorm>0.0&&(!(expectedNorm>0.0)||!(nativeNorm>0.0))))return Status::NumericRange;
    const double error=std::fabs(nativeNorm-expectedNorm);
    const double budget=128.0*std::numeric_limits<double>::epsilon()*expectedNorm;
    if(error>budget)return Status::PrecisionLoss;
    output=native;return Status::Ok;
}
inline Status MapPoint(const FrozenFrame& inputFrame,const Point3& programPoint,Point8& output) noexcept
{
    const FrozenFrame frame=inputFrame;const Point3 frozen=programPoint;output=Point8{};
    if(!ValidFrame(frame))return Status::FrameMismatch;
    if(!Detail::Finite(frozen))return Status::InvalidInput;
    Point3 delta{};for(unsigned a=0;a<3;++a)delta[a]=frozen[a]-frame.programStart[a];
    if(!Detail::Finite(delta))return Status::NumericRange;
    Point3 expectedVector{};const auto vectorStatus=MapVector(frame,delta,expectedVector);
    if(vectorStatus!=Status::Ok)return vectorStatus;
    if(delta[0]==0.0&&delta[1]==0.0&&delta[2]==0.0){output=frame.startMCS;return Status::Ok;}
    Point8 native{};if(!Detail::RawForward(frame,frozen,native))return Status::NumericRange;
    Point3 actualVector{},expectedPoint{},nativePoint{};
    for(unsigned a=0;a<3;++a){actualVector[a]=native[a]-frame.startMCS[a];
        expectedPoint[a]=frame.startMCS[a]+expectedVector[a];nativePoint[a]=native[a];}
    if(!Detail::Finite(actualVector)||!Detail::Finite(expectedPoint))return Status::NumericRange;
    if(!Detail::PreservesVector(actualVector,expectedVector)||
        !Detail::WithinError(nativePoint,expectedPoint))return Status::PrecisionLoss;
    output=native;return Status::Ok;
}
inline Status BuildInput(const FrozenFrame& frame,const Point3& authoredXYZ,unsigned authoredMask,
    double radiusMm,double layerStepMm,double feedMmMin,NCG76Sphere::RadiusLaw radiusLaw,NCG76Sphere::Input& output) noexcept
{
    const Point3 words=authoredXYZ;output=NCG76Sphere::Input{};
    if(!ValidFrame(frame))return Status::FrameMismatch;
    if(!authoredMask||(authoredMask&~7U)!=0U||!Detail::Finite(words)||
        !std::isfinite(radiusMm)||radiusMm<=0.0||!std::isfinite(layerStepMm)||layerStepMm<=0.0||
        !std::isfinite(feedMmMin)||feedMmMin<=0.0)return Status::InvalidInput;
    NCG76Sphere::Input input{};
    for(unsigned a=0;a<3;++a){input.start[a]=frame.programStart[a];
        input.end[a]=(authoredMask&(1U<<a))?words[a]:input.start[a];}
    input.radiusMm=radiusMm;input.layerStepMm=layerStepMm;input.feedMmMin=feedMmMin;input.radiusLaw=radiusLaw;
    Point8 endpoint{};const auto status=MapPoint(frame,input.end,endpoint);if(status!=Status::Ok)return status;
    output=input;return Status::Ok;
}
inline bool SameNativePlan(const NativePlan& a,const NativePlan& b) noexcept
{
    return a.valid==b.valid&&SameFrame(a.frame,b.frame)&&NCG76Sphere::SameDescriptor(a.descriptor,b.descriptor)&&
        Detail::Bits(a.radiusMm,b.radiusMm)&&Detail::Bits(a.layerStepMm,b.layerStepMm)&&Detail::Bits(a.travelMm,b.travelMm)&&
        Detail::Bits(a.chordErrorBoundMm,b.chordErrorBoundMm)&&Detail::Bits(a.coordinateErrorBoundMm,b.coordinateErrorBoundMm)&&
        Detail::Same(a.axis,b.axis)&&Detail::Same(a.u,b.u)&&Detail::Same(a.v,b.v)&&Detail::Same(a.endMCS,b.endMCS);
}
inline Status BuildNative(const FrozenFrame& frame,const NCG76Sphere::Descriptor& descriptor,NativePlan& output) noexcept
{
    const FrozenFrame frozenFrame=frame;const NCG76Sphere::Descriptor frozenDescriptor=descriptor;output=NativePlan{};
    if(!ValidFrame(frozenFrame))return Status::FrameMismatch;
    if(!NCG76Sphere::Valid(frozenDescriptor))return Status::InvalidDescriptor;
    for(unsigned a=0;a<3;++a)if(!Detail::Bits(frozenDescriptor.input.start[a],frozenFrame.programStart[a]))return Status::FrameMismatch;
    NativePlan plan{};plan.frame=frozenFrame;plan.descriptor=frozenDescriptor;
    plan.radiusMm=frozenDescriptor.input.radiusMm*frozenFrame.nativeScale;
    plan.layerStepMm=frozenDescriptor.input.layerStepMm*frozenFrame.nativeScale;
    plan.travelMm=frozenDescriptor.travelMm*frozenFrame.nativeScale;
    plan.chordErrorBoundMm=frozenDescriptor.chordErrorBoundMm*frozenFrame.nativeScale;
    plan.coordinateErrorBoundMm=frozenDescriptor.coordinateErrorBoundMm*frozenFrame.nativeScale;
    if(!std::isfinite(plan.radiusMm)||plan.radiusMm<=0.0||!std::isfinite(plan.layerStepMm)||plan.layerStepMm<=0.0||
        !std::isfinite(plan.travelMm)||plan.travelMm<=0.0||!std::isfinite(plan.chordErrorBoundMm)||plan.chordErrorBoundMm<=0.0||
        !std::isfinite(plan.coordinateErrorBoundMm)||plan.coordinateErrorBoundMm<=0.0)return Status::NumericRange;
    const auto endpointStatus=MapPoint(frozenFrame,frozenDescriptor.input.end,plan.endMCS);
    if(endpointStatus!=Status::Ok)return endpointStatus;
    // Authored geometry can be well resolved yet collapse after downscaling
    // at a large native origin. Bound native arithmetic against the smallest
    // ring/chord/step, rather than allowing an absolute tolerance to erase it.
    double nativeEnvelope=0.0;
    for(unsigned a=0;a<3;++a){
        const double extent=std::fabs(plan.endMCS[a])+2.0*plan.radiusMm;
        if(!std::isfinite(extent))return Status::NumericRange;
        nativeEnvelope=(std::max)(nativeEnvelope,(std::max)(std::fabs(frozenFrame.startMCS[a]),extent));
    }
    const double nativeResolutionError=plan.coordinateErrorBoundMm+
        nativeEnvelope*(64.0*std::numeric_limits<double>::epsilon());
    NCG76Sphere::Layer last{};
    if(!NCG76Sphere::LayerAt(frozenDescriptor,frozenDescriptor.circleLayerCount-1U,last))return Status::InvalidDescriptor;
    const double nativeLastGap=(frozenDescriptor.input.radiusMm-last.depthMm)*frozenFrame.nativeScale;
    const double nativeSmallestChord=(2.0*last.radiusMm*std::sin(NCG76Sphere::Pi/NCG76Sphere::CircleSegments))*frozenFrame.nativeScale;
    const double smallestFeature=(std::min)(plan.travelMm,(std::min)(plan.radiusMm,
        (std::min)(plan.layerStepMm,(std::min)(nativeLastGap,nativeSmallestChord))));
    if(!std::isfinite(nativeResolutionError)||!std::isfinite(smallestFeature)||smallestFeature<=0.0||
        nativeResolutionError>MaximumMappingErrorMm||nativeResolutionError>smallestFeature/64.0)
        return Status::PrecisionLoss;
    // Native-endpoint magnitudes do not bound intermediate G90 affine
    // operations around distant pivots. Include the explicit point-mapping
    // fence in the public error envelope; resolution checks remain separate.
    plan.coordinateErrorBoundMm=nativeResolutionError+MaximumMappingErrorMm;
    const Point3* authored[3]={&frozenDescriptor.axis,&frozenDescriptor.u,&frozenDescriptor.v};
    Point3* mapped[3]={&plan.axis,&plan.u,&plan.v};
    for(unsigned k=0;k<3;++k){const auto status=MapVector(frozenFrame,*authored[k],*mapped[k]);if(status!=Status::Ok)return status;
        for(double& value:*mapped[k])value/=frozenFrame.nativeScale;
        if(!Detail::Finite(*mapped[k]))return Status::NumericRange;}
    // Keep the mapped triad, including reflection handedness. Rebuilding a
    // native descriptor with a fresh cross-product would change contour order.
    plan.valid=true;output=plan;return Status::Ok;
}
inline bool Valid(const NativePlan& plan) noexcept
{
    NativePlan expected{};return plan.valid&&BuildNative(plan.frame,plan.descriptor,expected)==Status::Ok&&SameNativePlan(plan,expected);
}
inline Status EvaluateNative(const NativePlan& inputPlan,const NCG76Sphere::Cursor& cursor,double fraction,
    Point8& position,Point8& center) noexcept
{
    // Evaluation cannot mutate a frozen plan through output aliases. Reject
    // before clearing anything, matching the descriptor evaluator contract.
    const std::uintptr_t begin=reinterpret_cast<std::uintptr_t>(&inputPlan),end=begin+sizeof(inputPlan);
    const std::uintptr_t pos=reinterpret_cast<std::uintptr_t>(&position),cen=reinterpret_cast<std::uintptr_t>(&center);
    if(pos==cen||(pos<end&&pos+sizeof(position)>begin)||(cen<end&&cen+sizeof(center)>begin))return Status::InvalidInput;
    const NativePlan plan=inputPlan;
    position=Point8{};center=Point8{};
    if(!Valid(plan))return Status::InvalidDescriptor;
    Point3 programPosition{},programCenter{};
    if(!NCG76Sphere::Evaluate(plan.descriptor,cursor,fraction,programPosition,programCenter))return Status::InvalidCursor;
    Point8 mappedPosition{},mappedCenter{};
    const auto positionStatus=MapPoint(plan.frame,programPosition,mappedPosition);
    if(positionStatus!=Status::Ok)return positionStatus;
    const auto centerStatus=MapPoint(plan.frame,programCenter,mappedCenter);
    if(centerStatus!=Status::Ok)return centerStatus;
    NCG76Sphere::Segment segment{};
    if(!NCG76Sphere::SegmentAt(plan.descriptor,cursor,segment))return Status::InvalidCursor;
    Point8 segmentStart{},segmentEnd{};
    const auto startStatus=MapPoint(plan.frame,segment.start,segmentStart);
    if(startStatus!=Status::Ok)return startStatus;
    const auto endStatus=MapPoint(plan.frame,segment.end,segmentEnd);
    if(endStatus!=Status::Ok)return endStatus;
    Point3 authoredLeg{},nativeLeg{},expectedLeg{};
    for(unsigned a=0;a<3;++a){authoredLeg[a]=segment.end[a]-segment.start[a];nativeLeg[a]=segmentEnd[a]-segmentStart[a];}
    const auto legStatus=MapVector(plan.frame,authoredLeg,expectedLeg);
    if(legStatus!=Status::Ok)return legStatus;
    if(!Detail::PreservesVector(nativeLeg,expectedLeg))return Status::PrecisionLoss;
    // Norms include XYZ mm only; C/U/V degrees never enter a path length.
    Point3 authoredOrbit{},nativeOrbit{},expectedOrbit{};
    for(unsigned a=0;a<3;++a){authoredOrbit[a]=programPosition[a]-programCenter[a];nativeOrbit[a]=mappedPosition[a]-mappedCenter[a];}
    const auto orbitStatus=MapVector(plan.frame,authoredOrbit,expectedOrbit);
    if(orbitStatus!=Status::Ok)return orbitStatus;
    if(!Detail::PreservesVector(nativeOrbit,expectedOrbit))return Status::PrecisionLoss;
    position=mappedPosition;center=mappedCenter;return Status::Ok;
}
static_assert(std::is_trivially_copyable<FrozenFrame>::value,"P50 frozen frame is pure value data");
static_assert(std::is_trivially_copyable<NativePlan>::value,"P50 native plan grants no runtime authority");
}
