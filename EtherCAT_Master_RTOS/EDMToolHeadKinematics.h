#pragma once

#include "EDMMultiAxisKernel.h"
#include <array>
#include <cmath>
#include <limits>
#include <type_traits>

// EDM86 reference serial TOOL HEAD only. Given C/U/V angles, solve the
// Cartesian translation needed to preserve a programmed tool point. This is
// not a table/head topology guess, calibrated machine model or general IK.
// Column vectors: T = T0*T1*T2, Tj = [R(axis,sign*q[channel]+zero), pivot].
// Both joint axis and parent-to-pivot translation are in that joint's PARENT
// frame. The channel permutation explicitly determines rotation order.
namespace EDM86
{
using Vector3=std::array<double,3U>;
using Matrix3=std::array<Vector3,3U>;
constexpr double RadPerDeg=3.141592653589793238462643383279502884/180.0;
constexpr double MaximumAbsAngleDeg=1.0e7;
struct RevoluteJoint
{
    unsigned channel=3U;
    Vector3 axis{{0.0,0.0,1.0}};
    Vector3 parentToPivotMm{};
    int sign=1;
    double zeroAngleDeg=0.0;
};
struct HeadChain
{
    HeadChain() noexcept
    {
        joints[0].channel=3U; joints[0].axis={{0.0,0.0,1.0}};
        joints[1].channel=4U; joints[1].axis={{1.0,0.0,0.0}};
        joints[2].channel=5U; joints[2].axis={{0.0,1.0,0.0}};
    }
    std::array<RevoluteJoint,3U> joints{};
};
struct RigidTransform
{
    Matrix3 rotation{{{{1.0,0.0,0.0}},{{0.0,1.0,0.0}},{{0.0,0.0,1.0}}}};
    Vector3 translationMm{};
};
namespace Detail
{
inline bool FiniteVector(const Vector3& v) noexcept
{ for(const auto value:v) if(!EDM81::Finite(value)) return false; return true; }
inline double Norm(const Vector3& v) noexcept
{ return std::hypot(std::hypot(v[0],v[1]),v[2]); }
inline double AddUp(double a,double b) noexcept
{
    if(a==0.0) return b;
    if(b==0.0) return a;
    return std::nextafter(a+b,(std::numeric_limits<double>::infinity)());
}
inline double MultiplyUp(double a,double b) noexcept
{
    if(a==0.0 || b==0.0) return 0.0;
    return std::nextafter(a*b,(std::numeric_limits<double>::infinity)());
}
// L1 is deliberately conservative for Euclidean reach. Each addition is
// rounded outward, avoiding an unproven libm-hypot error allowance in limits.
inline double ReachUpper(const Vector3& v) noexcept
{ return AddUp(AddUp(std::fabs(v[0]),std::fabs(v[1])),std::fabs(v[2])); }
inline bool Unit(const Vector3& v) noexcept
{
    if(!FiniteVector(v)) return false;
    for(const auto value:v) if(std::fabs(value)>1.0) return false;
    const double norm=Norm(v);
    return EDM81::Finite(norm) && std::fabs(norm-1.0)<=5.0e-10;
}
inline bool SinCos(double degrees,double& sine,double& cosine) noexcept
{
    if(!EDM81::Finite(degrees) || std::fabs(degrees)>MaximumAbsAngleDeg) return false;
    const double phase=std::remainder(degrees,360.0);
    if(phase==0.0) { sine=0.0; cosine=1.0; }
    else if(phase==90.0) { sine=1.0; cosine=0.0; }
    else if(phase==-90.0) { sine=-1.0; cosine=0.0; }
    else if(std::fabs(phase)==180.0) { sine=0.0; cosine=-1.0; }
    else { sine=std::sin(phase*RadPerDeg); cosine=std::cos(phase*RadPerDeg); }
    return EDM81::Finite(sine) && EDM81::Finite(cosine);
}
inline bool Multiply(const Matrix3& matrix,const Vector3& v,Vector3& out) noexcept
{
    Vector3 result{};
    for(unsigned row=0U;row<3U;++row)
        result[row]=matrix[row][0]*v[0]+matrix[row][1]*v[1]+matrix[row][2]*v[2];
    if(!FiniteVector(result)) return false;
    out=result; return true;
}
inline bool Angle(const RevoluteJoint& joint,const EDM81::Pose& pose,double& angle) noexcept
{
    if(joint.channel<3U || joint.channel>5U || !EDM81::Finite(pose[joint.channel]) ||
        std::fabs(pose[joint.channel])>MaximumAbsAngleDeg) return false;
    angle=static_cast<double>(joint.sign)*pose[joint.channel]+joint.zeroAngleDeg;
    return EDM81::Finite(angle) && std::fabs(angle)<=MaximumAbsAngleDeg;
}
inline bool Rotation(const RevoluteJoint& joint,double angle,Matrix3& out) noexcept
{
    double sine=0.0,cosine=0.0;
    if(!SinCos(angle,sine,cosine)) return false;
    // Admission permits norm roundoff, never a scale factor in Rodrigues.
    const double norm=Norm(joint.axis);
    if(!EDM81::Finite(norm) || norm<=0.0) return false;
    const double x=joint.axis[0]/norm,y=joint.axis[1]/norm,z=joint.axis[2]/norm;
    const double t=1.0-cosine;
    out={{{{cosine+x*x*t,x*y*t-z*sine,x*z*t+y*sine}},
          {{y*x*t+z*sine,cosine+y*y*t,y*z*t-x*sine}},
          {{z*x*t-y*sine,z*y*t+x*sine,cosine+z*z*t}}}};
    for(const auto& row:out) if(!FiniteVector(row)) return false;
    return true;
}
}
inline bool ValidChain(const HeadChain& chain) noexcept
{
    unsigned channels=0U;
    for(const auto& joint:chain.joints)
    {
        if(joint.channel<3U || joint.channel>5U || (joint.sign!=1 && joint.sign!=-1) ||
            !Detail::Unit(joint.axis) || !Detail::FiniteVector(joint.parentToPivotMm) ||
            !EDM81::Finite(Detail::Norm(joint.parentToPivotMm)) ||
            !EDM81::Finite(joint.zeroAngleDeg) || std::fabs(joint.zeroAngleDeg)>MaximumAbsAngleDeg)
            return false;
        const unsigned bit=1U<<(joint.channel-3U);
        if(channels&bit) return false;
        channels|=bit;
    }
    return channels==7U;
}
inline bool ChainsEqual(const HeadChain& a,const HeadChain& b) noexcept
{
    if(!ValidChain(a) || !ValidChain(b)) return false;
    for(unsigned j=0U;j<3U;++j)
    {
        const auto& x=a.joints[j]; const auto& y=b.joints[j];
        if(x.channel!=y.channel || x.axis!=y.axis || x.parentToPivotMm!=y.parentToPivotMm ||
            x.sign!=y.sign || x.zeroAngleDeg!=y.zeroAngleDeg) return false;
    }
    return true;
}
inline bool ValidPoseAngles(const HeadChain& chain,const EDM81::Pose& pose) noexcept
{
    if(!ValidChain(chain)) return false;
    for(const auto value:pose) if(!EDM81::Finite(value)) return false;
    for(const auto& joint:chain.joints)
    { double angle=0.0; if(!Detail::Angle(joint,pose,angle)) return false; }
    return true;
}
inline bool ForwardTransform(const HeadChain& chain,const EDM81::Pose& pose,RigidTransform& out) noexcept
{
    out=RigidTransform{};
    if(!ValidPoseAngles(chain,pose)) return false;
    RigidTransform result{};
    for(const auto& joint:chain.joints)
    {
        double angle=0.0; Matrix3 rotation{},composed{}; Vector3 pivot{};
        if(!Detail::Angle(joint,pose,angle) || !Detail::Rotation(joint,angle,rotation) ||
            !Detail::Multiply(result.rotation,joint.parentToPivotMm,pivot)) return false;
        for(unsigned row=0U;row<3U;++row)
        {
            result.translationMm[row]+=pivot[row];
            if(!EDM81::Finite(result.translationMm[row])) return false;
            for(unsigned column=0U;column<3U;++column)
                composed[row][column]=result.rotation[row][0]*rotation[0][column]+
                    result.rotation[row][1]*rotation[1][column]+result.rotation[row][2]*rotation[2][column];
            if(!Detail::FiniteVector(composed[row])) return false;
        }
        result.rotation=composed;
    }
    out=result; return true;
}
inline bool ToolPointOffset(const HeadChain& chain,const EDM81::Pose& pose,const Vector3& tipMm,
    Vector3& out) noexcept
{
    if(!Detail::FiniteVector(tipMm)) { out=Vector3{}; return false; }
    RigidTransform transform{}; Vector3 result{};
    if(!ForwardTransform(chain,pose,transform) || !Detail::Multiply(transform.rotation,tipMm,result))
    { out=Vector3{}; return false; }
    for(unsigned axis=0U;axis<3U;++axis) result[axis]+=transform.translationMm[axis];
    if(!Detail::FiniteVector(result)) { out=Vector3{}; return false; }
    out=result; return true;
}
inline bool RotateDirection(const HeadChain& chain,const EDM81::Pose& pose,const Vector3& local,
    Vector3& out) noexcept
{
    if(!Detail::FiniteVector(local)) { out=Vector3{}; return false; }
    RigidTransform transform{};
    if(!ForwardTransform(chain,pose,transform) || !Detail::Multiply(transform.rotation,local,out))
    { out=Vector3{}; return false; }
    return true;
}
// A global envelope for EVERY rotary angle, not only sampled endpoints.
// p0 is fixed. The remaining pivot vectors and tip are bounded by their norms.
inline bool OffsetEnvelope(const HeadChain& chain,const Vector3& tipMm,Vector3& center,double& radius) noexcept
{
    if(!ValidChain(chain) || !Detail::FiniteVector(tipMm))
    { center=Vector3{}; radius=0.0; return false; }
    const double reach=Detail::AddUp(Detail::AddUp(Detail::ReachUpper(chain.joints[1].parentToPivotMm),
        Detail::ReachUpper(chain.joints[2].parentToPivotMm)),Detail::ReachUpper(tipMm));
    // Also cover the bounded three-rotation/addition evaluation roundoff. This
    // allowance concerns coordinate arithmetic, not unknown machine accuracy.
    const double scale=Detail::AddUp(Detail::ReachUpper(chain.joints[0].parentToPivotMm),reach);
    const double allowance=Detail::MultiplyUp(scale,64.0*(std::numeric_limits<double>::epsilon)());
    const double value=Detail::AddUp(reach,allowance);
    if(!EDM81::Finite(value)) { center=Vector3{}; radius=0.0; return false; }
    center=chain.joints[0].parentToPivotMm; radius=value; return true;
}
// A rotation about joint j can move the tip by at most its downstream lever
// arm times angular travel. Triangle inequality remains valid for every
// orientation/order/pivot and for an entire crossed canonical interval.
inline bool DerivativeEnvelope(const HeadChain& chain,const Vector3& tipMm,
    const EDM81::Pose& nativePerMm,double& extraCartesianPerMm) noexcept
{
    if(!ValidChain(chain) || !Detail::FiniteVector(tipMm)) { extraCartesianPerMm=0.0; return false; }
    for(const auto value:nativePerMm) if(!EDM81::Finite(value) || value<0.0)
    { extraCartesianPerMm=0.0; return false; }
    double lever=Detail::ReachUpper(tipMm),sum=0.0;
    if(!EDM81::Finite(lever)) { extraCartesianPerMm=0.0; return false; }
    for(int j=2;j>=0;--j)
    {
        const auto& joint=chain.joints[static_cast<unsigned>(j)];
        const double angleScale=std::nextafter(RadPerDeg,(std::numeric_limits<double>::infinity)());
        sum=Detail::AddUp(sum,Detail::MultiplyUp(Detail::MultiplyUp(lever,nativePerMm[joint.channel]),angleScale));
        if(!EDM81::Finite(sum)) { extraCartesianPerMm=0.0; return false; }
        if(j>0) lever=Detail::AddUp(lever,Detail::ReachUpper(joint.parentToPivotMm));
        if(!EDM81::Finite(lever)) { extraCartesianPerMm=0.0; return false; }
    }
    extraCartesianPerMm=sum; return true;
}
static_assert(std::is_trivially_copyable<HeadChain>::value &&
    std::is_nothrow_default_constructible<HeadChain>::value,"EDM86 chain is fixed no-throw value storage");
static_assert(sizeof(HeadChain)<=256U,"EDM86 chain exceeds fixed three-joint storage budget");
}
