#pragma once

#include "EDMSynchronizedPath.h"
#include "EDMToolHeadKinematics.h"

// EDM84 numerical reference model only. FixedZC explicitly means a tool offset
// rotating about the fixed machine Z axis; U/V have no assumed physical axes,
// centres, rotation order or inverse kinematics. nominal XYZ is the tool point,
// joint XYZ = nominal XYZ - Rz(C)*toolOffset + frozenDirection*offsetMm.
// Nothing here is a calibrated machine transform or an output authorization.
// EDM86 SerialHeadCUV adds an explicitly parameterized reference head chain.
// Given its prescribed C/U/V coordinates, the same chain supplies the tip
// offset and tool-frame direction. It does not infer a physical head/table
// arrangement or solve unspecified rotary angles.
namespace EDM84
{
enum class DirectionFrame : std::uint8_t { Machine, ElectrodeC, ElectrodeCUV };
struct DirectionMapping
{
    DirectionFrame frame=DirectionFrame::Machine;
    std::array<double,3U> unit{{0.0,0.0,1.0}};
    std::uint64_t revision=1U;
};
enum class MappingKind : std::uint8_t { Identity, FixedZC, SerialHeadCUV };
struct ToolMapping
{
    MappingKind kind=MappingKind::Identity;
    std::array<double,3U> toolOffsetMm{};
    std::uint64_t revision=1U;
    EDM86::HeadChain chain{}; // Used only by SerialHeadCUV; one shared direction/tip chain.
};
namespace MappingDetail
{
constexpr double RadPerDeg=3.141592653589793238462643383279502884/180.0;
constexpr double MaximumAbsAngleDeg=1.0e7;
inline bool Unit(const std::array<double,3U>& value) noexcept
{
    double norm=0.0;
    for(const double v:value)
    { if(!EDM81::Finite(v) || std::fabs(v)>1.0) return false; norm+=v*v; }
    return EDM81::Finite(norm) && std::fabs(norm-1.0)<=1e-9;
}
inline bool PoseFinite(const EDM81::Pose& pose) noexcept
{ for(const double v:pose) if(!EDM81::Finite(v)) return false; return true; }
inline bool SinCos(double degrees,double& sine,double& cosine) noexcept
{
    sine=cosine=0.0;
    if(!EDM81::Finite(degrees) || std::fabs(degrees)>MaximumAbsAngleDeg) return false;
    // Same physical 360-degree convention as NCEccentricCPath; no change to
    // the unwrapped rotary coordinate used for path, limits or velocities.
    const double phase=std::remainder(degrees,360.0);
    if(phase==0.0) { sine=0.0; cosine=1.0; }
    else if(phase==90.0) { sine=1.0; cosine=0.0; }
    else if(phase==-90.0) { sine=-1.0; cosine=0.0; }
    else if(std::fabs(phase)==180.0) { sine=0.0; cosine=-1.0; }
    else { sine=std::sin(phase*RadPerDeg); cosine=std::cos(phase*RadPerDeg); }
    return EDM81::Finite(sine) && EDM81::Finite(cosine);
}
inline bool RotatedOffset(const ToolMapping& mapping,double degrees,
    std::array<double,3U>& result) noexcept
{
    result={{0.0,0.0,0.0}};
    if(mapping.kind==MappingKind::Identity) return true;
    if(mapping.kind!=MappingKind::FixedZC) return false;
    double sine=0.0,cosine=0.0;
    if(!SinCos(degrees,sine,cosine)) return false;
    const auto& offset=mapping.toolOffsetMm;
    result={{offset[0]*cosine-offset[1]*sine,offset[0]*sine+offset[1]*cosine,offset[2]}};
    for(const double v:result) if(!EDM81::Finite(v)) return false;
    return true;
}
inline bool RotatedOffset(const ToolMapping& mapping,const EDM81::Pose& pose,
    std::array<double,3U>& result) noexcept
{
    if(mapping.kind==MappingKind::SerialHeadCUV)
        return EDM86::ToolPointOffset(mapping.chain,pose,mapping.toolOffsetMm,result);
    return RotatedOffset(mapping,pose[3U],result);
}
}
inline bool ValidDirection(const DirectionMapping& direction) noexcept
{
    return direction.revision &&
        (direction.frame==DirectionFrame::Machine || direction.frame==DirectionFrame::ElectrodeC ||
            direction.frame==DirectionFrame::ElectrodeCUV) &&
        MappingDetail::Unit(direction.unit);
}
inline bool ValidMapping(const ToolMapping& mapping) noexcept
{
    if(!mapping.revision || (mapping.kind!=MappingKind::Identity && mapping.kind!=MappingKind::FixedZC &&
        mapping.kind!=MappingKind::SerialHeadCUV)) return false;
    if(mapping.kind==MappingKind::SerialHeadCUV && (!EDM86::ValidChain(mapping.chain) ||
        !EDM81::Finite(EDM86::Detail::ReachUpper(mapping.toolOffsetMm)))) return false;
    for(const double v:mapping.toolOffsetMm)
        if(!EDM81::Finite(v) || (mapping.kind==MappingKind::Identity && v!=0.0)) return false;
    return EDM81::Finite(std::hypot(mapping.toolOffsetMm[0],mapping.toolOffsetMm[1]));
}
inline bool ResolveDirection(const DirectionMapping& direction,const EDM81::Pose& nominalPose,
    std::array<double,3U>& outUnit) noexcept
{
    if(!ValidDirection(direction) || direction.frame==DirectionFrame::ElectrodeCUV ||
        !MappingDetail::PoseFinite(nominalPose))
    { outUnit={{0.0,0.0,0.0}}; return false; }
    auto candidate=direction.unit;
    if(direction.frame==DirectionFrame::ElectrodeC)
    {
        double sine=0.0,cosine=0.0;
        if(!MappingDetail::SinCos(nominalPose[3U],sine,cosine))
        { outUnit={{0.0,0.0,0.0}}; return false; }
        candidate[0]=direction.unit[0]*cosine-direction.unit[1]*sine;
        candidate[1]=direction.unit[0]*sine+direction.unit[1]*cosine;
    }
    // Admission allows unit-norm roundoff, not a scale command. Normalize that
    // allowance after rotation so a valid near-unit vector cannot acquire a
    // component slightly greater than one at a cardinal direction. This also
    // preserves the conservative maxOffsetMm per-axis envelope below.
    const double norm=std::hypot(std::hypot(candidate[0],candidate[1]),candidate[2]);
    if(!EDM81::Finite(norm) || norm<=0.0) { outUnit={{0.0,0.0,0.0}}; return false; }
    for(double& value:candidate) value/=norm;
    if(!MappingDetail::Unit(candidate)) { outUnit={{0.0,0.0,0.0}}; return false; }
    outUnit=candidate; return true;
}
// ElectrodeCUV resolves from the exact same chain as the tool-point mapping.
// The legacy overload deliberately rejects this frame without its mapping.
inline bool ResolveDirection(const DirectionMapping& direction,const ToolMapping& mapping,
    const EDM81::Pose& nominalPose,std::array<double,3U>& outUnit) noexcept
{
    if(direction.frame!=DirectionFrame::ElectrodeCUV)
        return ResolveDirection(direction,nominalPose,outUnit);
    if(!ValidDirection(direction) || !ValidMapping(mapping) ||
        mapping.kind!=MappingKind::SerialHeadCUV || !MappingDetail::PoseFinite(nominalPose))
    { outUnit={{0.0,0.0,0.0}}; return false; }
    std::array<double,3U> candidate{};
    if(!EDM86::RotateDirection(mapping.chain,nominalPose,direction.unit,candidate))
    { outUnit={{0.0,0.0,0.0}}; return false; }
    const double norm=std::hypot(std::hypot(candidate[0],candidate[1]),candidate[2]);
    if(!EDM81::Finite(norm) || norm<=0.0) { outUnit={{0.0,0.0,0.0}}; return false; }
    for(double& value:candidate) value/=norm;
    if(!MappingDetail::Unit(candidate)) { outUnit={{0.0,0.0,0.0}}; return false; }
    outUnit=candidate; return true;
}
inline bool ComposePose(const ToolMapping& mapping,const EDM81::Pose& nominalPose,
    const std::array<double,3U>& frozenUnit,double offsetMm,EDM81::Pose& outJoint) noexcept
{
    if(!ValidMapping(mapping) || !MappingDetail::PoseFinite(nominalPose) ||
        !MappingDetail::Unit(frozenUnit) || !EDM81::Finite(offsetMm))
    { outJoint=EDM81::Pose{}; return false; }
    std::array<double,3U> toolOffset{};
    if(!MappingDetail::RotatedOffset(mapping,nominalPose,toolOffset))
    { outJoint=EDM81::Pose{}; return false; }
    EDM81::Pose candidate=nominalPose;
    for(unsigned axis=0U;axis<3U;++axis)
    {
        candidate[axis]=nominalPose[axis]-toolOffset[axis]+frozenUnit[axis]*offsetMm;
        if(!EDM81::Finite(candidate[axis])) { outJoint=EDM81::Pose{}; return false; }
    }
    outJoint=candidate; return true;
}
inline bool RecoverToolPoint(const ToolMapping& mapping,const EDM81::Pose& jointPose,
    std::array<double,3U>& outXYZ) noexcept
{
    if(!ValidMapping(mapping) || !MappingDetail::PoseFinite(jointPose))
    { outXYZ={{0.0,0.0,0.0}}; return false; }
    std::array<double,3U> offset{},candidate{};
    if(!MappingDetail::RotatedOffset(mapping,jointPose,offset))
    { outXYZ={{0.0,0.0,0.0}}; return false; }
    for(unsigned axis=0U;axis<3U;++axis)
    {
        candidate[axis]=jointPose[axis]+offset[axis];
        if(!EDM81::Finite(candidate[axis])) { outXYZ={{0.0,0.0,0.0}}; return false; }
    }
    outXYZ=candidate; return true;
}

// Conservative admission envelope, including intermediate rotary angles and
// all possible frozen unit directions. maxOffsetMm is the absolute excursion
// budget. The travel reference is the COMPOSED initial zero-offset pose.
inline EDM81::Reason ValidateMappedBounds(const SynchronizedPath& path,const EDM81::Config& config,
    const ToolMapping& mapping,double maxOffsetMm) noexcept
{
    if(!ValidMapping(mapping) || !EDM81::Finite(maxOffsetMm) || maxOffsetMm<0.0)
        return EDM81::Reason::Configuration;
    EDM81::Pose minimum{},maximum{},nominal{},origin{}; unsigned ordinal=0U; double u=0.0;
    const auto result=path.NativeBounds(minimum,maximum);
    if(result!=EDM81::Reason::None) return result;
    if(mapping.kind==MappingKind::FixedZC &&
        (minimum[3U]<-MappingDetail::MaximumAbsAngleDeg || maximum[3U]>MappingDetail::MaximumAbsAngleDeg))
        return EDM81::Reason::UnsupportedMapping;
    if(!path.Evaluate(0.0,nominal,ordinal,u) ||
        !ComposePose(mapping,nominal,{{0.0,0.0,1.0}},0.0,origin)) return EDM81::Reason::NonFinite;
    double radius=mapping.kind==MappingKind::FixedZC?
        std::hypot(mapping.toolOffsetMm[0],mapping.toolOffsetMm[1]):0.0;
    std::array<double,3U> envelopeCenter{};
    if(mapping.kind==MappingKind::SerialHeadCUV &&
        (!EDM86::ValidPoseAngles(mapping.chain,minimum) || !EDM86::ValidPoseAngles(mapping.chain,maximum) ||
         !EDM86::OffsetEnvelope(mapping.chain,mapping.toolOffsetMm,envelopeCenter,radius)))
        return EDM81::Reason::UnsupportedMapping;
    const double infinity=(std::numeric_limits<double>::infinity)();
    for(unsigned axis=0U;axis<EDM81::AxisCount;++axis)
    {
        const auto& b=config.axes[axis];
        if(!SynchronizedPath::ValidAxisBounds(b)) return EDM81::Reason::Configuration;
        double low=minimum[axis],high=maximum[axis];
        if(axis<3U)
        {
            const double pad=mapping.kind==MappingKind::SerialHeadCUV?
                EDM86::Detail::AddUp(maxOffsetMm,radius):maxOffsetMm+(axis<2U?radius:0.0);
            const double shift=mapping.kind==MappingKind::SerialHeadCUV?-envelopeCenter[axis]:
                axis==2U && mapping.kind==MappingKind::FixedZC?-mapping.toolOffsetMm[2U]:0.0;
            if(!EDM81::Finite(pad) || !EDM81::Finite(shift)) return EDM81::Reason::NonFinite;
            if(pad!=0.0 || shift!=0.0)
            {
                // Outward rounding of each operation keeps the coarse radius
                // envelope conservative at the supplied native axis limits.
                const double outwardPad=pad==0.0?0.0:std::nextafter(pad,infinity);
                low=std::nextafter(std::nextafter(low+shift,-infinity)-outwardPad,-infinity);
                high=std::nextafter(std::nextafter(high+shift,infinity)+outwardPad,infinity);
            }
        }
        if(!EDM81::Finite(low) || !EDM81::Finite(high)) return EDM81::Reason::NonFinite;
        if(low<b.minimum || high>b.maximum) return EDM81::Reason::AxisLimit;
        const double travel=EDM81::Maximum(std::fabs(low-origin[axis]),std::fabs(high-origin[axis]));
        if(!EDM81::Finite(travel)) return EDM81::Reason::NonFinite;
        if(travel>b.maxTravel) return EDM81::Reason::TravelLimit;
    }
    return EDM81::Reason::None;
}

// Shared by NC feed admission and the RT whole-interval velocity validator.
// Serial-head Cartesian bounds include all three downstream rotary lever arms.
inline bool MappedDerivativeBounds(const ToolMapping& mapping,const EDM81::Pose& nativePerMm,
    EDM81::Pose& outPerMm) noexcept
{
    if(!ValidMapping(mapping)) { outPerMm=EDM81::Pose{}; return false; }
    for(const auto value:nativePerMm) if(!EDM81::Finite(value) || value<0.0)
    { outPerMm=EDM81::Pose{}; return false; }
    auto result=nativePerMm;
    if(mapping.kind==MappingKind::FixedZC)
    {
        const double radius=std::hypot(mapping.toolOffsetMm[0],mapping.toolOffsetMm[1]);
        const double rotational=radius*nativePerMm[3U]*MappingDetail::RadPerDeg;
        if(!EDM81::Finite(rotational)) { outPerMm=EDM81::Pose{}; return false; }
        result[0]+=rotational; result[1]+=rotational;
    }
    else if(mapping.kind==MappingKind::SerialHeadCUV)
    {
        double rotational=0.0;
        if(!EDM86::DerivativeEnvelope(mapping.chain,mapping.toolOffsetMm,nativePerMm,rotational))
        { outPerMm=EDM81::Pose{}; return false; }
        for(unsigned axis=0U;axis<3U;++axis) result[axis]=EDM86::Detail::AddUp(result[axis],rotational);
    }
    for(const auto value:result) if(!EDM81::Finite(value)) { outPerMm=EDM81::Pose{}; return false; }
    outPerMm=result; return true;
}
// Frozen direction is constant during an excursion. The C-dependent tool
// offset is still evaluated at the same canonical sample as all native axes.
// Native derivative bounds cover complete crossed arcs/rotations, even if the
// composed endpoint chord is zero. No acceleration/stopping proof is implied.
inline EDM81::Reason ValidateMappedTraversal(const SynchronizedPath& path,const EDM81::Config& config,
    const ToolMapping& mapping,double fromCursor,double toCursor,double fromOffset,double toOffset,
    const std::array<double,3U>& frozenUnit) noexcept
{
    if(!ValidMapping(mapping) || !MappingDetail::Unit(frozenUnit) ||
        !EDM81::Finite(fromOffset) || !EDM81::Finite(toOffset) || config.periodUs!=250U)
        return EDM81::Reason::Configuration;
    EDM81::Pose derivative{};
    const auto result=path.DerivativeBounds(fromCursor,toCursor,derivative);
    if(result!=EDM81::Reason::None) return result;
    const double scale=60000000.0/static_cast<double>(config.periodUs);
    const double scalarRate=std::fabs(toCursor-fromCursor)*scale;
    const double offsetRate=std::fabs(toOffset-fromOffset)*scale;
    EDM81::Pose mappedDerivative{};
    if(!EDM81::Finite(scalarRate) || !EDM81::Finite(offsetRate) ||
        !MappedDerivativeBounds(mapping,derivative,mappedDerivative)) return EDM81::Reason::NonFinite;
    for(unsigned axis=0U;axis<EDM81::AxisCount;++axis)
    {
        if(!SynchronizedPath::ValidAxisBounds(config.axes[axis])) return EDM81::Reason::Configuration;
        const double rate=mapping.kind==MappingKind::SerialHeadCUV?
            EDM86::Detail::AddUp(EDM86::Detail::MultiplyUp(mappedDerivative[axis],scalarRate),
                axis<3U?EDM86::Detail::MultiplyUp(std::fabs(frozenUnit[axis]),offsetRate):0.0):
            mappedDerivative[axis]*scalarRate+(axis<3U?std::fabs(frozenUnit[axis])*offsetRate:0.0);
        if(!EDM81::Finite(rate)) return EDM81::Reason::NonFinite;
        if(rate>config.axes[axis].maxVelocityPerMin*(1.0+1e-12)) return EDM81::Reason::VelocityLimit;
    }
    return EDM81::Reason::None;
}
static_assert(std::is_trivially_copyable<DirectionMapping>::value &&
    std::is_trivially_copyable<ToolMapping>::value,"EDM84 mappings must be fixed copied values");
}
