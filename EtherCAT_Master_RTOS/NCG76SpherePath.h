#pragma once

#include "NCG76SphereRuntimeGeometry.h"
#include "NCG76SphereRetreat.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <type_traits>

// EDM292: the canonical nominal P50 path, expressed as one distance station.
// This is NC-side value metadata only. It grants no Motion/GAP/history/PLC
// authority and releases no discharge execution. The released P50 law remains
// linear R-i*K; TrueSphere is deliberately not a second implicit NC mode.
//
// Prefix sums use the arithmetic series of the positive layer radii. Admission
// checks every (at most 4096) layer; station lookup uses bounded binary search.
// No heap allocation, mutable global state, process output or servo work occurs.
// Station boundaries are rounded once to binary64. Interior progress maps
// linearly inside that rounded span; admission bounds this metric error in
// fixed millimetres and rejects spans that cannot remain distinct.
// Keep an admitted Descriptor immutable. Valid performs full reconstruction;
// the other APIs consume that immutable descriptor with bounded shape checks.
namespace NCG76SpherePath
{
using Geometry=NCG76SphereRuntimeGeometry::Geometry;
using Point3=NCG76SphereRuntimeGeometry::Point3;
using Point8=NCG76SphereRuntimeGeometry::Point8;
constexpr unsigned Version=1U;
constexpr double MaximumStationErrorMm=1.e-7;
constexpr unsigned MaximumLayerSearchIterations=13U;

enum class Stage : std::uint8_t
{ InitialDepth, InitialRetreat, CircleEntry, Orbit, ReturnCenter, AdvanceCenter, Complete, ContourNominal, ContourFinish };
enum class Status : std::uint8_t
{ Ok, InvalidGeometry, UnsupportedRadiusLaw, NumericRange, InvalidDescriptor,
  InvalidStage, InvalidLayer, InvalidProgress, InvalidStation, InvalidCursor, PrecisionLoss };

struct Descriptor
{
    Geometry geometry{};
    double initialDepthLengthMm=0.0,initialRetreatLengthMm=0.0;
    double layeredStartMm=0.0,totalLengthMm=0.0,stationErrorBoundMm=0.0;
    unsigned layerCount=0U,version=0U;
    bool valid=false;
};
struct Span
{
    double startMm=0.0,endMm=0.0,lengthMm=0.0;
    Stage stage=Stage::InitialDepth;
    unsigned layerIndex=0U;
};
struct Cursor
{
    Span span{};
    double stationMm=0.0,localProgressMm=0.0,fraction=0.0;
    bool valid=false;
};
struct Sample
{
    Point8 position{},center{};
    // XYZ projection of d(native position)/d(metric linear distance). A
    // coordinated line may also move native C/U/V; its XYZ projection can be
    // shorter than one or zero for an auxiliary-linear-only metric path.
    Point3 tangent{};
    Cursor cursor{};
    bool terminal=false,valid=false;
};

namespace Detail
{
inline bool Positive(double x) noexcept {return std::isfinite(x)&&x>0.0;}
inline bool Bits(double a,double b) noexcept {return std::memcmp(&a,&b,sizeof(double))==0;}
inline bool SameSpan(const Span& a,const Span& b) noexcept
{return a.stage==b.stage&&a.layerIndex==b.layerIndex&&Bits(a.startMm,b.startMm)&&
    Bits(a.endMm,b.endMm)&&Bits(a.lengthMm,b.lengthMm);}
inline bool SameCursor(const Cursor& a,const Cursor& b) noexcept
{return a.valid==b.valid&&SameSpan(a.span,b.span)&&Bits(a.stationMm,b.stationMm)&&
    Bits(a.localProgressMm,b.localProgressMm)&&Bits(a.fraction,b.fraction);}
inline bool Overlap(const void* a,std::size_t an,const void* b,std::size_t bn) noexcept
{const auto x=reinterpret_cast<std::uintptr_t>(a),y=reinterpret_cast<std::uintptr_t>(b);return x<y+bn&&y<x+an;}
inline bool Shape(const Descriptor& p) noexcept
{
    if(!p.valid||p.version!=Version||!p.geometry.valid||
        p.geometry.version!=NCG76SphereRuntimeGeometry::Version)return false;
    if(p.geometry.kind==NCG76SphereRuntimeGeometry::Kind::OrbitContour){
        double total=0.0;
        return NCG76SphereRetreat::ContourTotal(p.geometry,total)&&p.layerCount==0U&&
            p.geometry.circleLayerCount==0U&&p.geometry.radiusLaw==NCG76Sphere::RadiusLaw::Linear&&
            Bits(p.initialDepthLengthMm,NCG76OrbitContour::NominalLength(p.geometry))&&
            p.initialRetreatLengthMm==0.0&&Bits(p.layeredStartMm,p.initialDepthLengthMm)&&
            Bits(p.totalLengthMm,total)&&Positive(p.stationErrorBoundMm)&&
            p.stationErrorBoundMm<=MaximumStationErrorMm&&p.totalLengthMm>p.stationErrorBoundMm*64.0;
    }
    if(p.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::Sphere)
        return (p.geometry.kind==NCG76SphereRuntimeGeometry::Kind::Line||
                p.geometry.kind==NCG76SphereRuntimeGeometry::Kind::Arc)&&
            p.geometry.radiusLaw==NCG76Sphere::RadiusLaw::Linear&&p.layerCount==0U&&p.geometry.circleLayerCount==0U&&
            Bits(p.initialDepthLengthMm,p.geometry.travelMm)&&p.initialRetreatLengthMm==0.0&&
            Bits(p.layeredStartMm,p.totalLengthMm)&&Bits(p.totalLengthMm,p.geometry.travelMm)&&
            (Positive(p.totalLengthMm)||(p.geometry.kind==NCG76SphereRuntimeGeometry::Kind::Line&&p.totalLengthMm==0.0))&&
            Positive(p.stationErrorBoundMm)&&p.stationErrorBoundMm<=MaximumStationErrorMm&&
            (p.totalLengthMm==0.0||p.totalLengthMm>p.stationErrorBoundMm*64.0);
    return p.geometry.radiusLaw==NCG76Sphere::RadiusLaw::Linear&&p.layerCount==p.geometry.circleLayerCount&&
        p.layerCount>0U&&p.layerCount<=NCG76Sphere::MaximumLayers&&
        Positive(p.initialDepthLengthMm)&&Positive(p.initialRetreatLengthMm)&&
        Positive(p.layeredStartMm)&&Positive(p.totalLengthMm)&&p.totalLengthMm>p.layeredStartMm&&
        Positive(p.stationErrorBoundMm)&&p.stationErrorBoundMm<=MaximumStationErrorMm;
}
// Sum r_0 .. r_(count-1). The terminal center advance sums to exactly R.
// long double is additional host precision when available, not an ABI promise:
// every converted stage boundary is separately fenced on binary64 targets too.
inline long double LayerPrefix(const Descriptor& p,unsigned count) noexcept
{
    if(count==0U)return 0.0L;
    const long double n=static_cast<long double>(count),r=p.geometry.radiusMm,k=p.geometry.layerStepMm;
    const long double last=r-static_cast<long double>(count-1U)*k;
    const long double radii=n*(r+last)*0.5L;
    const long double advance=count==p.layerCount?r:n*k;
    return (2.0L+static_cast<long double>(NCG76SphereRuntimeGeometry::TwoPi))*radii+advance;
}
inline long double LayerStart(const Descriptor& p,unsigned index) noexcept
{return static_cast<long double>(p.initialDepthLengthMm)+p.initialRetreatLengthMm+LayerPrefix(p,index);}
inline NCG76SphereRetreat::Stage ContourRetreatStage(Stage stage) noexcept
{
    return stage==Stage::ContourFinish?NCG76SphereRetreat::Stage::ContourFinish:
        stage==Stage::ContourNominal?NCG76SphereRetreat::Stage::ContourNominal:
        static_cast<NCG76SphereRetreat::Stage>(static_cast<unsigned>(stage));
}
inline Stage ContourPathStage(NCG76SphereRetreat::Stage stage) noexcept
{
    return stage==NCG76SphereRetreat::Stage::ContourFinish?Stage::ContourFinish:
        stage==NCG76SphereRetreat::Stage::ContourNominal?Stage::ContourNominal:
        static_cast<Stage>(static_cast<unsigned>(stage));
}
inline Status SpanUnchecked(const Descriptor& p,Stage stage,unsigned layer,Span& out) noexcept
{
    out=Span{};out.stage=stage;out.layerIndex=layer;
    if(p.geometry.kind==NCG76SphereRuntimeGeometry::Kind::OrbitContour){
        if(stage==Stage::Complete){
            if(layer!=0U)return Status::InvalidLayer;
            out.startMm=out.endMm=p.totalLengthMm;return Status::Ok;
        }
        NCG76SphereRetreat::Span span{};
        if(!NCG76SphereRetreat::ContourSpan(p.geometry,ContourRetreatStage(stage),layer,span))return Status::InvalidStage;
        out.startMm=span.start;out.endMm=span.end;out.lengthMm=span.length;return Status::Ok;
    }
    // A G01/G02/G03 parent has exactly one nominal analytic leg. It does
    // not acquire a synthetic sphere layer, center-return or retreat stage.
    if(p.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::Sphere){
        if(layer!=0U)return Status::InvalidLayer;
        if(stage==Stage::Complete){out.startMm=out.endMm=p.totalLengthMm;return Status::Ok;}
        if(stage!=Stage::InitialDepth)return Status::InvalidStage;
        out.endMm=p.totalLengthMm;out.lengthMm=p.initialDepthLengthMm;
        return (Positive(out.lengthMm)||(p.geometry.kind==NCG76SphereRuntimeGeometry::Kind::Line&&out.lengthMm==0.0))&&
            Bits(out.endMm,out.lengthMm)?Status::Ok:Status::PrecisionLoss;
    }
    long double begin=0.0L,end=0.0L;double length=0.0;
    if(stage==Stage::InitialDepth||stage==Stage::InitialRetreat){
        if(layer!=0U)return Status::InvalidLayer;
        begin=stage==Stage::InitialDepth?0.0L:static_cast<long double>(p.initialDepthLengthMm);
        length=stage==Stage::InitialDepth?p.initialDepthLengthMm:p.initialRetreatLengthMm;end=begin+length;
    }else if(stage==Stage::Complete){
        if(layer!=p.layerCount)return Status::InvalidLayer;
        out.startMm=out.endMm=p.totalLengthMm;return Status::Ok;
    }else{
        if(stage<Stage::CircleEntry||stage>Stage::AdvanceCenter)return Status::InvalidStage;
        if(layer>=p.layerCount)return Status::InvalidLayer;
        NCG76SphereRuntimeGeometry::Layer current{},next{};
        if(!NCG76SphereRuntimeGeometry::LayerAt(p.geometry,layer,current)||
            !NCG76SphereRuntimeGeometry::LayerAt(p.geometry,layer+1U,next))return Status::InvalidGeometry;
        const long double radius=current.radiusMm,arc=current.arcLengthMm;
        const long double base=LayerStart(p,layer);
        if(stage==Stage::CircleEntry){begin=base;length=current.radiusMm;end=base+radius;}
        else if(stage==Stage::Orbit){begin=base+radius;length=current.arcLengthMm;end=begin+arc;}
        else if(stage==Stage::ReturnCenter){begin=base+radius+arc;length=current.radiusMm;end=begin+radius;}
        else{begin=base+radius+arc+radius;length=next.depthMm-current.depthMm;end=LayerStart(p,layer+1U);}
    }
    out.startMm=static_cast<double>(begin);out.endMm=static_cast<double>(end);out.lengthMm=length;
    if(!std::isfinite(out.startMm)||!std::isfinite(out.endMm)||out.startMm<0.0||
        !Positive(length)||out.endMm<=out.startMm||out.endMm>p.totalLengthMm)return Status::PrecisionLoss;
    return Status::Ok;
}
inline Status CursorUnchecked(const Descriptor& p,const Span& span,double fraction,Cursor& out) noexcept
{
    out=Cursor{};
    if(!std::isfinite(fraction)||fraction<0.0||fraction>1.0)return Status::InvalidProgress;
    Cursor cursor{};cursor.span=span;cursor.fraction=fraction;
    if(span.stage==Stage::Complete){
        if(fraction!=0.0)return Status::InvalidProgress;
        cursor.stationMm=p.totalLengthMm;
    }else if(span.lengthMm==0.0){
        if(p.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::Line||span.stage!=Stage::InitialDepth||fraction!=0.0)
            return Status::InvalidProgress;
    }else{
        cursor.localProgressMm=fraction==1.0?span.lengthMm:fraction*span.lengthMm;
        cursor.stationMm=fraction==0.0?span.startMm:fraction==1.0?span.endMm:
            span.startMm+(span.endMm-span.startMm)*fraction;
        if(!std::isfinite(cursor.localProgressMm)||!std::isfinite(cursor.stationMm)||
            (fraction>0.0&&cursor.stationMm<=span.startMm)||
            (fraction<1.0&&cursor.stationMm>=span.endMm))return Status::PrecisionLoss;
    }
    cursor.valid=true;out=cursor;return Status::Ok;
}
inline bool SameDescriptor(const Descriptor& a,const Descriptor& b) noexcept
{
    return a.valid==b.valid&&a.version==b.version&&a.layerCount==b.layerCount&&
        Bits(a.initialDepthLengthMm,b.initialDepthLengthMm)&&Bits(a.initialRetreatLengthMm,b.initialRetreatLengthMm)&&
        Bits(a.layeredStartMm,b.layeredStartMm)&&Bits(a.totalLengthMm,b.totalLengthMm)&&
        Bits(a.stationErrorBoundMm,b.stationErrorBoundMm)&&NCG76SphereRuntimeGeometry::SameGeometry(a.geometry,b.geometry);
}
}

inline Status Build(const Geometry& source,Descriptor& output) noexcept
{
    // In-place rebuild from output.geometry is valid; freeze before clearing.
    const Geometry geometry=source;output=Descriptor{};
    if(!NCG76SphereRuntimeGeometry::Valid(geometry))return Status::InvalidGeometry;
    if(geometry.radiusLaw!=NCG76Sphere::RadiusLaw::Linear)return Status::UnsupportedRadiusLaw;
    Descriptor p{};p.geometry=geometry;p.layerCount=geometry.circleLayerCount;
    if(geometry.kind==NCG76SphereRuntimeGeometry::Kind::OrbitContour){
        p.layerCount=0U;p.initialDepthLengthMm=p.layeredStartMm=NCG76OrbitContour::NominalLength(geometry);
        if(!NCG76SphereRetreat::ContourTotal(geometry,p.totalLengthMm))return Status::NumericRange;
        const double ulp=std::nextafter(p.totalLengthMm,std::numeric_limits<double>::infinity())-p.totalLengthMm;
        p.stationErrorBoundMm=ulp*32.0;
        if(!Detail::Positive(p.stationErrorBoundMm)||p.stationErrorBoundMm>MaximumStationErrorMm)return Status::PrecisionLoss;
        double previousEnd=0.0;
        for(unsigned i=0U;i<NCG76OrbitContour::NominalLegCountFor(geometry);++i){
            Span span{};const auto status=Detail::SpanUnchecked(p,Stage::ContourNominal,i,span);
            if(status!=Status::Ok)return status;
            if(!Detail::Bits(span.startMm,previousEnd)||span.lengthMm<=p.stationErrorBoundMm*64.0||
               std::fabs((span.endMm-span.startMm)-span.lengthMm)>p.stationErrorBoundMm)return Status::PrecisionLoss;
            previousEnd=span.endMm;
        }
        if(NCG76OrbitContour::FinishEnabled(geometry)&&NCG76OrbitContour::IsStraightSideContour(geometry)){
            const unsigned count=NCG76OrbitContour::FinishLegCount(geometry);
            for(unsigned i=0U;i<count+2U;++i){
                const Stage stage=i==0U?Stage::CircleEntry:i==count+1U?Stage::ReturnCenter:Stage::ContourFinish;
                const unsigned side=stage==Stage::ContourFinish?i-1U:0U;
                Span span{};const auto status=Detail::SpanUnchecked(p,stage,side,span);
                if(status!=Status::Ok)return status;
                if(!Detail::Bits(span.startMm,previousEnd)||span.lengthMm<=p.stationErrorBoundMm*64.0||
                   std::fabs((span.endMm-span.startMm)-span.lengthMm)>p.stationErrorBoundMm)return Status::PrecisionLoss;
                previousEnd=span.endMm;
            }
        }else if(NCG76OrbitContour::FinishEnabled(geometry))for(Stage stage:{Stage::CircleEntry,Stage::Orbit,Stage::ReturnCenter}){
            Span span{};const auto status=Detail::SpanUnchecked(p,stage,0U,span);
            if(status!=Status::Ok)return status;
            if(!Detail::Bits(span.startMm,previousEnd)||span.lengthMm<=p.stationErrorBoundMm*64.0||
               std::fabs((span.endMm-span.startMm)-span.lengthMm)>p.stationErrorBoundMm)return Status::PrecisionLoss;
            previousEnd=span.endMm;
        }
        if(!Detail::Bits(previousEnd,p.totalLengthMm))return Status::PrecisionLoss;
        p.version=Version;p.valid=true;if(!Detail::Shape(p))return Status::PrecisionLoss;
        output=p;return Status::Ok;
    }
    if(geometry.kind!=NCG76SphereRuntimeGeometry::Kind::Sphere){
        p.initialDepthLengthMm=p.layeredStartMm=p.totalLengthMm=geometry.travelMm;
        const double ulp=std::nextafter(p.totalLengthMm,std::numeric_limits<double>::infinity())-p.totalLengthMm;
        p.stationErrorBoundMm=geometry.travelMm==0.0?geometry.coordinateErrorBoundMm:ulp*16.0;p.version=Version;p.valid=true;
        if(!Detail::Shape(p))return Status::PrecisionLoss;
        output=p;return Status::Ok;
    }
    p.initialDepthLengthMm=geometry.travelMm;p.initialRetreatLengthMm=geometry.radiusMm;
    p.layeredStartMm=static_cast<double>(static_cast<long double>(p.initialDepthLengthMm)+p.initialRetreatLengthMm);
    p.totalLengthMm=static_cast<double>(Detail::LayerStart(p,p.layerCount));
    if(!Detail::Positive(p.totalLengthMm)||p.totalLengthMm<=p.layeredStartMm)return Status::NumericRange;
    const double ulp=std::nextafter(p.totalLengthMm,std::numeric_limits<double>::infinity())-p.totalLengthMm;
    p.stationErrorBoundMm=ulp*16.0;
    if(!Detail::Positive(p.stationErrorBoundMm)||p.stationErrorBoundMm>MaximumStationErrorMm)return Status::PrecisionLoss;
    // Fixed-mm and relative fences reject an apparently positive but unresolved
    // last tiny ring/step instead of letting two distinct stages share station.
    for(unsigned i=0U;i<p.layerCount;++i){
        double previousEnd=static_cast<double>(Detail::LayerStart(p,i));
        for(Stage stage:{Stage::CircleEntry,Stage::Orbit,Stage::ReturnCenter,Stage::AdvanceCenter}){
            Span span{};const auto status=Detail::SpanUnchecked(p,stage,i,span);
            if(status!=Status::Ok)return status;
            if(!Detail::Bits(span.startMm,previousEnd)||span.lengthMm<=p.stationErrorBoundMm*64.0||
                std::fabs((span.endMm-span.startMm)-span.lengthMm)>p.stationErrorBoundMm)
                return Status::PrecisionLoss;
            previousEnd=span.endMm;
        }
    }
    for(Stage stage:{Stage::InitialDepth,Stage::InitialRetreat}){
        Span span{};const auto status=Detail::SpanUnchecked(p,stage,0U,span);
        if(status!=Status::Ok)return status;
        if(span.lengthMm<=p.stationErrorBoundMm*64.0)return Status::PrecisionLoss;
    }
    p.version=Version;p.valid=true;output=p;return Status::Ok;
}
inline bool Valid(const Descriptor& p) noexcept
{
    Descriptor expected{};return Detail::Shape(p)&&Build(p.geometry,expected)==Status::Ok&&Detail::SameDescriptor(p,expected);
}
inline Status GetSpan(const Descriptor& p,Stage stage,unsigned layer,Span& output) noexcept
{
    if(Detail::Overlap(&p,sizeof(p),&output,sizeof(output)))return Status::InvalidDescriptor;
    output=Span{};if(!Detail::Shape(p))return Status::InvalidDescriptor;
    Span span{};const auto status=Detail::SpanUnchecked(p,stage,layer,span);if(status==Status::Ok)output=span;return status;
}
// An explicit phase endpoint keeps that phase identity even though its station
// also starts the following phase. Use fractions when importing Cycle feedback:
// its actual native line norm can differ slightly from this analytic length.
inline Status MakeCursorByFraction(const Descriptor& p,Stage stage,unsigned layer,double fraction,Cursor& output) noexcept
{
    if(Detail::Overlap(&p,sizeof(p),&output,sizeof(output)))return Status::InvalidDescriptor;
    output=Cursor{};if(!Detail::Shape(p))return Status::InvalidDescriptor;
    Span span{};const auto status=Detail::SpanUnchecked(p,stage,layer,span);if(status!=Status::Ok)return status;
    return Detail::CursorUnchecked(p,span,fraction,output);
}
inline Status MakeCursor(const Descriptor& p,Stage stage,unsigned layer,double localMm,Cursor& output) noexcept
{
    if(Detail::Overlap(&p,sizeof(p),&output,sizeof(output)))return Status::InvalidDescriptor;
    output=Cursor{};if(!Detail::Shape(p))return Status::InvalidDescriptor;
    Span span{};const auto status=Detail::SpanUnchecked(p,stage,layer,span);if(status!=Status::Ok)return status;
    if(!std::isfinite(localMm)||localMm<0.0||localMm>span.lengthMm)return Status::InvalidProgress;
    const double fraction=span.lengthMm==0.0?0.0:localMm==span.lengthMm?1.0:localMm/span.lengthMm;
    if((localMm>0.0&&fraction<=0.0)||(localMm<span.lengthMm&&fraction>=1.0))return Status::PrecisionLoss;
    const auto result=Detail::CursorUnchecked(p,span,fraction,output);
    if(result==Status::Ok)output.localProgressMm=localMm;
    return result;
}
inline Status Station(const Descriptor& p,Stage stage,unsigned layer,double localMm,double& output) noexcept
{
    if(Detail::Overlap(&p,sizeof(p),&output,sizeof(output)))return Status::InvalidDescriptor;
    output=0.0;Cursor cursor{};const auto status=MakeCursor(p,stage,layer,localMm,cursor);
    if(status==Status::Ok)output=cursor.stationMm;return status;
}
// A station exactly at a boundary belongs to the following stage. Thus zero is
// INITIAL_DEPTH, a ring's closure is RETURN_CENTER, and total is COMPLETE.
// This convention is deterministic; explicit MakeCursor retains either side.
inline Status Locate(const Descriptor& p,double stationMm,Cursor& output) noexcept
{
    if(Detail::Overlap(&p,sizeof(p),&output,sizeof(output)))return Status::InvalidDescriptor;
    output=Cursor{};if(!Detail::Shape(p))return Status::InvalidDescriptor;
    if(!std::isfinite(stationMm)||stationMm<0.0||stationMm>p.totalLengthMm)return Status::InvalidStation;
    if(stationMm==p.totalLengthMm)return MakeCursorByFraction(p,Stage::Complete,p.layerCount,0.0,output);
    if(p.geometry.kind==NCG76SphereRuntimeGeometry::Kind::OrbitContour){
        NCG76SphereRetreat::Span span{};
        if(!NCG76SphereRetreat::ContourLocate(p.geometry,stationMm,false,span))return Status::InvalidStation;
        Span pathSpan{};const auto status=Detail::SpanUnchecked(p,Detail::ContourPathStage(span.stage),span.layer,pathSpan);
        if(status!=Status::Ok)return status;
        const double fraction=stationMm==span.start?0.0:(stationMm-span.start)/(span.end-span.start);
        const auto result=Detail::CursorUnchecked(p,pathSpan,fraction,output);
        if(result==Status::Ok)output.stationMm=stationMm;return result;
    }
    if(p.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::Sphere){
        const auto result=MakeCursorByFraction(p,Stage::InitialDepth,0U,
            stationMm==0.0?0.0:stationMm/p.totalLengthMm,output);
        if(result==Status::Ok)output.stationMm=stationMm;
        return result;
    }
    Span span{};
    if(stationMm<p.initialDepthLengthMm){
        const auto status=Detail::SpanUnchecked(p,Stage::InitialDepth,0U,span);if(status!=Status::Ok)return status;
    }else if(stationMm<p.layeredStartMm){
        const auto status=Detail::SpanUnchecked(p,Stage::InitialRetreat,0U,span);if(status!=Status::Ok)return status;
    }else{
        unsigned low=0U,high=p.layerCount,iterations=0U;
        while(low+1U<high){
            if(++iterations>MaximumLayerSearchIterations)return Status::InvalidDescriptor;
            const unsigned middle=low+(high-low)/2U;
            const double begin=static_cast<double>(Detail::LayerStart(p,middle));
            if(!std::isfinite(begin))return Status::InvalidDescriptor;
            if(stationMm<begin)high=middle;else low=middle;
        }
        bool found=false;
        for(Stage stage:{Stage::CircleEntry,Stage::Orbit,Stage::ReturnCenter,Stage::AdvanceCenter}){
            const auto status=Detail::SpanUnchecked(p,stage,low,span);if(status!=Status::Ok)return status;
            if(stationMm<span.endMm){found=true;break;}
        }
        if(!found)return Status::InvalidStation;
    }
    if(stationMm<span.startMm||stationMm>=span.endMm)return Status::InvalidStation;
    const double fraction=stationMm==span.startMm?0.0:(stationMm-span.startMm)/(span.endMm-span.startMm);
    const auto status=Detail::CursorUnchecked(p,span,fraction,output);
    if(status==Status::Ok)output.stationMm=stationMm;return status;
}
inline Status Evaluate(const Descriptor& p,const Cursor& cursor,Sample& output) noexcept
{
    if(Detail::Overlap(&p,sizeof(p),&output,sizeof(output))||Detail::Overlap(&cursor,sizeof(cursor),&output,sizeof(output)))
        return Status::InvalidCursor;
    output=Sample{};if(!Detail::Shape(p))return Status::InvalidDescriptor;
    if(!cursor.valid)return Status::InvalidCursor;
    Cursor expected{};const auto check=MakeCursorByFraction(p,cursor.span.stage,cursor.span.layerIndex,cursor.fraction,expected);
    // MakeCursor local and Locate station retain the caller's representable
    // value; both must still match the same admitted span and fraction.
    if(check!=Status::Ok||!Detail::SameSpan(cursor.span,expected.span)||
        !std::isfinite(cursor.stationMm)||!std::isfinite(cursor.localProgressMm)||
        cursor.stationMm<cursor.span.startMm||cursor.stationMm>cursor.span.endMm||
        cursor.localProgressMm<0.0||cursor.localProgressMm>cursor.span.lengthMm||
        std::fabs(cursor.stationMm-expected.stationMm)>p.stationErrorBoundMm||
        std::fabs(cursor.localProgressMm-expected.localProgressMm)>p.stationErrorBoundMm)return Status::InvalidCursor;
    Sample sample{};sample.cursor=cursor;const auto stage=cursor.span.stage;
    if(stage==Stage::Complete){sample.position=sample.center=p.geometry.endMCS;sample.terminal=true;}
    else if(p.geometry.kind==NCG76SphereRuntimeGeometry::Kind::OrbitContour){
        NCG76SphereRetreat::Span span{};const auto mapped=Detail::ContourRetreatStage(stage);
        if(!NCG76SphereRetreat::GetSpan(p.geometry,mapped,cursor.span.layerIndex,span)||
           !NCG76SphereRetreat::Evaluate(p.geometry,span,cursor.fraction,sample.position,sample.center))return Status::InvalidGeometry;
        if(stage==Stage::Orbit){
            Point8 point{};Point3 curvature{};
            if(!NCG76SphereRuntimeGeometry::CircleAt(p.geometry,0U,
                cursor.fraction==1.0?NCG76SphereRuntimeGeometry::TwoPi:cursor.fraction*NCG76SphereRuntimeGeometry::TwoPi,
                point,sample.tangent,curvature))return Status::InvalidGeometry;
        }else{
            Point8 begin{},end{},center{};double length=0.0,prefix=0.0;
            if(stage==Stage::ContourNominal){
                if(!NCG76OrbitContour::NominalLegAt(p.geometry,cursor.span.layerIndex,begin,end,center,length,prefix))return Status::InvalidGeometry;
            }else if(NCG76OrbitContour::IsStraightSideContour(p.geometry)){
                if(stage==Stage::ContourFinish){
                    if(!NCG76OrbitContour::FinishLegAt(p.geometry,cursor.span.layerIndex,begin,end,center,length,prefix))
                        return Status::InvalidGeometry;
                }else{
                    Point8 vertex{};
                    if(!NCG76OrbitContour::FinishVertexAt(p.geometry,0U,vertex))return Status::InvalidGeometry;
                    if(stage==Stage::CircleEntry){begin=p.geometry.endMCS;end=vertex;}
                    else if(stage==Stage::ReturnCenter){begin=vertex;end=p.geometry.endMCS;}
                    else return Status::InvalidStage;
                }
            }else{
                Point8 circle{};Point3 tangent{},curvature{};
                if(!NCG76SphereRuntimeGeometry::CircleAt(p.geometry,0U,0.0,circle,tangent,curvature))return Status::InvalidGeometry;
                if(stage==Stage::CircleEntry){begin=p.geometry.endMCS;end=circle;}
                else if(stage==Stage::ReturnCenter){begin=circle;end=p.geometry.endMCS;}
                else return Status::InvalidStage;
            }
            Point3 delta{};for(unsigned a=0U;a<3U;++a)delta[a]=end[a]-begin[a];
            const double norm=NCG76SphereRuntimeGeometry::Detail::Norm(delta);
            if(!Detail::Positive(norm))return Status::PrecisionLoss;
            for(unsigned a=0U;a<3U;++a)sample.tangent[a]=delta[a]/norm;
        }
    }
    else if(p.geometry.kind!=NCG76SphereRuntimeGeometry::Kind::Sphere){
        Point3 curvature{};
        if(stage!=Stage::InitialDepth||
            !NCG76SphereRuntimeGeometry::NominalAt(p.geometry,cursor.fraction,
                sample.position,sample.tangent,curvature))return Status::InvalidGeometry;
        sample.center=sample.position;
    }else{
        Point8 begin{},end{};Point3 tangent{};
        NCG76SphereRuntimeGeometry::Layer layer{},next{};Point8 circle{};Point3 curve{};
        if(stage==Stage::InitialDepth){begin=p.geometry.startMCS;end=p.geometry.endMCS;}
        else{
            if(!NCG76SphereRuntimeGeometry::LayerAt(p.geometry,cursor.span.layerIndex,layer)||
                !NCG76SphereRuntimeGeometry::CircleAt(p.geometry,cursor.span.layerIndex,0.0,circle,tangent,curve))
                return Status::InvalidGeometry;
            if(stage==Stage::InitialRetreat){begin=p.geometry.endMCS;end=layer.center;}
            else if(stage==Stage::CircleEntry){begin=layer.center;end=circle;}
            else if(stage==Stage::ReturnCenter){begin=circle;end=layer.center;}
            else if(stage==Stage::AdvanceCenter){
                if(!NCG76SphereRuntimeGeometry::LayerAt(p.geometry,cursor.span.layerIndex+1U,next))return Status::InvalidGeometry;
                begin=layer.center;end=next.center;
            }else if(stage!=Stage::Orbit)return Status::InvalidStage;
        }
        if(stage==Stage::Orbit){
            const double angle=cursor.fraction==1.0?NCG76SphereRuntimeGeometry::TwoPi:
                cursor.fraction*NCG76SphereRuntimeGeometry::TwoPi;
            if(!NCG76SphereRuntimeGeometry::CircleAt(p.geometry,cursor.span.layerIndex,angle,sample.position,sample.tangent,curve))
                return Status::InvalidGeometry;
            sample.center=layer.center;
        }else{
            sample.position=begin;
            if(cursor.fraction==1.0)sample.position=end;
            else if(cursor.fraction!=0.0)for(unsigned a=0U;a<3U;++a)
                sample.position[a]=begin[a]+(end[a]-begin[a])*cursor.fraction;
            Point3 delta{};for(unsigned a=0U;a<3U;++a)delta[a]=end[a]-begin[a];
            const double norm=NCG76SphereRuntimeGeometry::Detail::Norm(delta);
            if(!Detail::Positive(norm))return Status::PrecisionLoss;
            for(unsigned a=0U;a<3U;++a)sample.tangent[a]=delta[a]/norm;
            sample.center=(stage==Stage::CircleEntry||stage==Stage::ReturnCenter)?layer.center:sample.position;
        }
    }
    if(!NCG76SphereRuntimeGeometry::Detail::Finite(sample.position)||
        !NCG76SphereRuntimeGeometry::Detail::Finite(sample.center)||!NCG76SphereRuntimeGeometry::Detail::Finite(sample.tangent))
        return Status::NumericRange;
    for(unsigned a=0U;a<8U;++a){
        if(sample.position[a]<p.geometry.boundsMinMCS[a]||sample.position[a]>p.geometry.boundsMaxMCS[a]||
            sample.center[a]<p.geometry.boundsMinMCS[a]||sample.center[a]>p.geometry.boundsMaxMCS[a])return Status::NumericRange;
        const bool movingNativeLine=p.geometry.kind==NCG76SphereRuntimeGeometry::Kind::Line&&
            a<6U&&p.geometry.frame.translation.axisIdentity.exists[a]==1U;
        if(a>=3U&&!movingNativeLine&&(!Detail::Bits(sample.position[a],p.geometry.startMCS[a])||
            !Detail::Bits(sample.center[a],p.geometry.startMCS[a])))return Status::InvalidGeometry;
    }
    sample.valid=true;output=sample;return Status::Ok;
}
inline Status SampleAt(const Descriptor& p,double stationMm,Sample& output) noexcept
{
    if(Detail::Overlap(&p,sizeof(p),&output,sizeof(output)))return Status::InvalidDescriptor;
    output=Sample{};Cursor cursor{};const auto status=Locate(p,stationMm,cursor);
    return status==Status::Ok?Evaluate(p,cursor,output):status;
}
static_assert(std::is_trivially_copyable<Descriptor>::value,"P50 station descriptor is value metadata only");
static_assert(std::is_trivially_copyable<Cursor>::value,"P50 cursor grants no execution authority");
}
