#pragma once
#include "NCG76OrbitPlan.h"
#include "NCG76SphereCoordinates.h"
#include <array>
#include <cstdint>
#include <cstring>
#include <type_traits>

// Compact P01..P07/P11..P17/P21..P27 provenance and native evaluation cache. This occupies the
// existing 280-byte sphere descriptor slot; it does not enlarge the 4096-byte
// Motion request bank. RuntimeGeometry provides the complete inline API.
namespace NCG76SphereRuntimeGeometry { struct Geometry; }
namespace NCG76OrbitContour
{
using Point3=NCG76SphereCoordinates::Point3;
using Point8=NCG76SphereCoordinates::Point8;
using Geometry=NCG76SphereRuntimeGeometry::Geometry;
constexpr unsigned Version=1U,NominalLegCount=25U,CoarseStride=4U;
struct Descriptor
{
    NCG76::PlanInput source{};
    Point3 centerStartMCS{},centerEndMCS{},unitU{},unitV{};
    std::array<double,7U> coarsePrefix{};
    double nominalLengthMm=0.0,arithmeticErrorMm=0.0;
    std::uint32_t version=0U;
    bool finishEnabled=false,valid=false;
    // Zero retains the released P11 descriptor bytes. P12/P13/P14/P15 use one byte
    // of the old tail padding without enlarging the fixed Motion bank.
    std::uint8_t contourMode=0U;
};
static_assert(sizeof(Descriptor)<=sizeof(NCG76Sphere::Descriptor),"P11 metadata must not grow the motion bank");
static_assert(std::is_trivially_copyable<Descriptor>::value,"P11 descriptor is immutable value metadata");
inline bool SameDescriptor(const Descriptor& a,const Descriptor& b) noexcept
{
    return a.version==b.version&&a.finishEnabled==b.finishEnabled&&a.valid==b.valid&&a.contourMode==b.contourMode&&
        NCG76::Detail::SameInput(a.source,b.source)&&
        std::memcmp(a.centerStartMCS.data(),b.centerStartMCS.data(),sizeof(a.centerStartMCS))==0&&
        std::memcmp(a.centerEndMCS.data(),b.centerEndMCS.data(),sizeof(a.centerEndMCS))==0&&
        std::memcmp(a.unitU.data(),b.unitU.data(),sizeof(a.unitU))==0&&
        std::memcmp(a.unitV.data(),b.unitV.data(),sizeof(a.unitV))==0&&
        std::memcmp(a.coarsePrefix.data(),b.coarsePrefix.data(),sizeof(a.coarsePrefix))==0&&
        std::memcmp(&a.nominalLengthMm,&b.nominalLengthMm,sizeof(double))==0&&
        std::memcmp(&a.arithmeticErrorMm,&b.arithmeticErrorMm,sizeof(double))==0;
}
inline bool BuildTimedFixedRadius(unsigned modeP,const NCG76SphereCoordinates::FrozenFrame&,const Point3&,
    double radiusMm,double shapeAngleDeg,double feedMmMin,bool finish,Geometry&,unsigned polygonSides=0U) noexcept;
inline bool BuildTimedP11(const NCG76SphereCoordinates::FrozenFrame&,const Point3&,
    double radiusMm,double shapeAngleDeg,double feedMmMin,bool finish,Geometry&) noexcept;
inline bool BuildTimedP12(const NCG76SphereCoordinates::FrozenFrame&,const Point3&,
    double radiusMm,double shapeAngleDeg,double feedMmMin,bool finish,Geometry&) noexcept;
inline bool BuildTimedP13(const NCG76SphereCoordinates::FrozenFrame&,const Point3&,
    double radiusMm,double shapeAngleDeg,double feedMmMin,bool finish,Geometry&) noexcept;
inline bool BuildTimedP14(const NCG76SphereCoordinates::FrozenFrame&,const Point3&,
    double radiusMm,double shapeAngleDeg,double feedMmMin,bool finish,Geometry&) noexcept;
inline bool BuildTimedP15(const NCG76SphereCoordinates::FrozenFrame&,const Point3&,
    double radiusMm,double shapeAngleDeg,double feedMmMin,bool finish,Geometry&) noexcept;
inline bool BuildTimedP16(const NCG76SphereCoordinates::FrozenFrame&,const Point3&,
    double radiusMm,double shapeAngleDeg,double feedMmMin,bool finish,unsigned polygonSides,Geometry&) noexcept;
inline bool BuildTimedP17(const NCG76SphereCoordinates::FrozenFrame&,const Point3&,
    double radiusMm,double shapeAngleDeg,double feedMmMin,bool finish,unsigned polygonSides,Geometry&) noexcept;
inline bool BuildTimedP21(const NCG76SphereCoordinates::FrozenFrame&,const Point3&,
    double radiusMm,double shapeAngleDeg,double feedMmMin,bool finish,Geometry&) noexcept;
inline bool BuildTimedP22(const NCG76SphereCoordinates::FrozenFrame&,const Point3&,
    double radiusMm,double shapeAngleDeg,double feedMmMin,bool finish,Geometry&) noexcept;
inline bool BuildTimedP23(const NCG76SphereCoordinates::FrozenFrame&,const Point3&,
    double radiusMm,double shapeAngleDeg,double feedMmMin,bool finish,Geometry&) noexcept;
inline bool BuildTimedP24(const NCG76SphereCoordinates::FrozenFrame&,const Point3&,
    double radiusMm,double shapeAngleDeg,double feedMmMin,bool finish,Geometry&) noexcept;
inline bool BuildTimedP25(const NCG76SphereCoordinates::FrozenFrame&,const Point3&,
    double radiusMm,double shapeAngleDeg,double feedMmMin,bool finish,Geometry&) noexcept;
inline bool BuildTimedP26(const NCG76SphereCoordinates::FrozenFrame&,const Point3&,
    double radiusMm,double shapeAngleDeg,double feedMmMin,bool finish,unsigned polygonSides,Geometry&) noexcept;
inline bool BuildTimedP27(const NCG76SphereCoordinates::FrozenFrame&,const Point3&,
    double radiusMm,double shapeAngleDeg,double feedMmMin,bool finish,unsigned polygonSides,Geometry&) noexcept;
inline unsigned ModeP(const Geometry&) noexcept;
// P21/P22/P23/P24/P25/P26/P27 add the initial target-center approach to the released 25-leg contour.
inline unsigned NominalLegCountFor(const Geometry&) noexcept;
inline bool IsExpansionContour(const Geometry&) noexcept;
inline bool IsFixedRadiusContour(const Geometry&) noexcept;
inline bool IsCircleContour(const Geometry&) noexcept;
inline bool IsFourSideContour(const Geometry&) noexcept;
inline bool IsStraightSideContour(const Geometry&) noexcept;
inline bool EndpointAt(const Geometry&,unsigned vertex,Point8&) noexcept;
inline bool NominalCenterAt(const Geometry&,unsigned index,double fraction,Point8&) noexcept;
inline bool NominalLegAt(const Geometry&,unsigned index,Point8& begin,Point8& end,
    Point8& center,double& length,double& prefix) noexcept;
inline double NominalLength(const Geometry&) noexcept;
inline bool FinishEnabled(const Geometry&) noexcept;
// P12/P13/P14/P15/P22/P23/P24/P25 have four straight sides; P16/P17/P26/P27 have their authored Q sides.
// These are exact straight legs, not a synthetic circle or a P50 layer.
// The closed vertex FinishLegCount preserves
// vertex 0's exact bits, also equal to the final nominal grow vertex.
inline unsigned FinishLegCount(const Geometry&) noexcept;
inline bool FinishVertexAt(const Geometry&,unsigned vertex,Point8&) noexcept;
inline bool FinishLegAt(const Geometry&,unsigned side,Point8& begin,Point8& end,
    Point8& center,double& length,double& prefix) noexcept;
inline double FinishEntryLength(const Geometry&) noexcept;
inline double FinishLength(const Geometry&) noexcept;
}
