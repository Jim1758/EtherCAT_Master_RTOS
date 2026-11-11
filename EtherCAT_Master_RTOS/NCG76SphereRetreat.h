#pragma once
#include "NCG76SphereRuntimeGeometry.h"
#include <algorithm>
#include <limits>

// EDM301: bounded geometry for one already-admitted P50 parent. This view
// borrows immutable Geometry; it never builds a layer array or grants motion.
namespace NCG76SphereRetreat
{
using Geometry=NCG76SphereRuntimeGeometry::Geometry;
using Point8=NCG76SphereRuntimeGeometry::Point8;
using Point3=NCG76SphereRuntimeGeometry::Point3;
enum class Stage : std::uint8_t
{InitialDepth,InitialRetreat,CircleEntry,Orbit,ReturnCenter,AdvanceCenter,ContourNominal,ContourFinish};
struct Span { double start=0.0,end=0.0,length=0.0;Stage stage=Stage::InitialDepth;unsigned layer=0U; };
inline bool ValidMode(unsigned mode) noexcept {return mode==0U||mode==2U||mode==3U||mode==4U;}
inline bool ValidMode(const Geometry& geometry,unsigned mode) noexcept
{
    if(geometry.kind==NCG76SphereRuntimeGeometry::Kind::OrbitContour)return mode==0U||mode==2U;
    if(geometry.kind==NCG76SphereRuntimeGeometry::Kind::Sphere)return ValidMode(mode);
    // A straight G01 has no radial orbit offset: its B3/B4 center is the
    // stopped machining anchor, so the zero center legs coalesce into B0's
    // same native coupled line. G76 center geometry remains unchanged.
    if(geometry.kind==NCG76SphereRuntimeGeometry::Kind::Line)return mode<=4U;
    return geometry.kind==NCG76SphereRuntimeGeometry::Kind::Arc&&(mode==0U||mode==1U||mode==2U);
}
// EDM341: one procedural 25-leg taper and one canonical terminal H loop.
// Repeated H loops borrow this same immutable station region, like P50 rings.
inline bool ContourTotal(const Geometry& g,double& total) noexcept
{
    total=0.0;
    if(g.kind!=NCG76SphereRuntimeGeometry::Kind::OrbitContour||!g.valid||
       g.version!=NCG76SphereRuntimeGeometry::Version)return false;
    const double base=NCG76OrbitContour::NominalLength(g);
    if(!std::isfinite(base)||base<=0.0)return false;
    total=base;
    if(NCG76OrbitContour::FinishEnabled(g)&&NCG76OrbitContour::IsStraightSideContour(g)){
        const double radial=NCG76OrbitContour::FinishEntryLength(g);
        const double entry=base+radial,perimeter=entry+NCG76OrbitContour::FinishLength(g);
        total=perimeter+radial;
        if(!std::isfinite(radial)||radial<=0.0||!std::isfinite(entry)||!std::isfinite(perimeter)||
           !std::isfinite(total)||entry<=base||perimeter<=entry||total<=perimeter)return false;
    }else if(NCG76OrbitContour::FinishEnabled(g)){
        const double entry=base+g.radiusMm,orbit=entry+NCG76SphereRuntimeGeometry::TwoPi*g.radiusMm;
        total=orbit+g.radiusMm;
        if(!std::isfinite(entry)||!std::isfinite(orbit)||!std::isfinite(total)||
           entry<=base||orbit<=entry||total<=orbit)return false;
    }
    return std::isfinite(total)&&total>0.0;
}
inline bool ContourSpan(const Geometry& g,Stage stage,unsigned index,Span& out) noexcept
{
    out=Span{};out.stage=stage;out.layer=index;
    double total=0.0;if(!ContourTotal(g,total))return false;
    const double base=NCG76OrbitContour::NominalLength(g);
    if(stage==Stage::ContourNominal){
        Point8 begin{},end{},center{};double length=0.0,prefix=0.0;
        if(index>=NCG76OrbitContour::NominalLegCountFor(g)||!NCG76OrbitContour::NominalLegAt(g,index,begin,end,center,length,prefix))return false;
        out.start=prefix;out.end=index+1U==NCG76OrbitContour::NominalLegCountFor(g)?base:prefix+length;out.length=length;
    }else if(NCG76OrbitContour::IsStraightSideContour(g)){
        if(!NCG76OrbitContour::FinishEnabled(g))return false;
        const double radial=NCG76OrbitContour::FinishEntryLength(g),entry=base+radial;
        const double perimeter=entry+NCG76OrbitContour::FinishLength(g);
        if(stage==Stage::ContourFinish){
            Point8 begin{},end{},center{};double length=0.0,prefix=0.0;
            if(!NCG76OrbitContour::FinishLegAt(g,index,begin,end,center,length,prefix))return false;
            out.start=entry+prefix;out.end=index+1U==NCG76OrbitContour::FinishLegCount(g)?perimeter:entry+(prefix+length);
            out.length=length;
        }else if(index!=0U)return false;
        else if(stage==Stage::CircleEntry){out.start=base;out.end=entry;out.length=radial;}
        else if(stage==Stage::ReturnCenter){out.start=perimeter;out.end=total;out.length=radial;}
        else return false;
    }else{
        if(index!=0U||!NCG76OrbitContour::FinishEnabled(g))return false;
        const double entry=base+g.radiusMm,orbit=entry+NCG76SphereRuntimeGeometry::TwoPi*g.radiusMm;
        if(stage==Stage::CircleEntry){out.start=base;out.end=entry;out.length=g.radiusMm;}
        else if(stage==Stage::Orbit){out.start=entry;out.end=orbit;out.length=NCG76SphereRuntimeGeometry::TwoPi*g.radiusMm;}
        else if(stage==Stage::ReturnCenter){out.start=orbit;out.end=total;out.length=g.radiusMm;}
        else return false;
    }
    return std::isfinite(out.start)&&std::isfinite(out.end)&&std::isfinite(out.length)&&
        out.start>=0.0&&out.end>out.start&&out.length>0.0&&out.end<=total;
}
inline bool ContourLocate(const Geometry& g,double station,bool reverse,Span& out) noexcept
{
    double total=0.0;
    if(!ContourTotal(g,total)||!std::isfinite(station)||station<0.0||station>total)return false;
    const double probe=reverse&&station>0.0?std::nextafter(station,-std::numeric_limits<double>::infinity()):station;
    if(probe<NCG76OrbitContour::NominalLength(g)){
        // Seven prefixes cover groups of four legs; at most three binary
        // decisions and four candidate legs, with no array materialization.
        unsigned low=0U,high=7U,iterations=0U;
        while(low+1U<high){
            if(++iterations>3U)return false;
            const unsigned middle=low+(high-low)/2U;
            if(probe<g.orbit.coarsePrefix[middle])high=middle;else low=middle;
        }
        const unsigned limit=(std::min)(NCG76OrbitContour::NominalLegCountFor(g),low*4U+4U);
        for(unsigned index=low*4U;index<limit;++index){
            Span span{};if(!ContourSpan(g,Stage::ContourNominal,index,span))return false;
            if(probe>=span.start&&probe<span.end){out=span;return station>=span.start&&station<=span.end;}
        }
        return false;
    }
    if(NCG76OrbitContour::IsStraightSideContour(g)){
        Span span{};
        if(!ContourSpan(g,Stage::CircleEntry,0U,span))return false;
        if(probe>=span.start&&probe<span.end){out=span;return station>=span.start&&station<=span.end;}
        for(unsigned side=0U;side<NCG76OrbitContour::FinishLegCount(g);++side){
            if(!ContourSpan(g,Stage::ContourFinish,side,span))return false;
            if(probe>=span.start&&probe<span.end){out=span;return station>=span.start&&station<=span.end;}
        }
        if(!ContourSpan(g,Stage::ReturnCenter,0U,span))return false;
        if(probe>=span.start&&probe<span.end){out=span;return station>=span.start&&station<=span.end;}
        return false;
    }
    for(Stage stage:{Stage::CircleEntry,Stage::Orbit,Stage::ReturnCenter}){
        Span span{};if(!ContourSpan(g,stage,0U,span))return false;
        if(probe>=span.start&&probe<span.end){out=span;return station>=span.start&&station<=span.end;}
    }
    return false;
}
inline long double LayerStart(const Geometry& g,unsigned count) noexcept
{
    const long double n=count,r=g.radiusMm,k=g.layerStepMm;
    const long double sum=count?n*(r+(r-static_cast<long double>(count-1U)*k))*0.5L:0.0L;
    const long double advance=count==g.circleLayerCount?r:n*k;
    return static_cast<long double>(g.travelMm)+r+
        (2.0L+static_cast<long double>(NCG76SphereRuntimeGeometry::TwoPi))*sum+advance;
}
inline bool GetSpan(const Geometry& g,Stage stage,unsigned layer,Span& out) noexcept
{
    out=Span{};out.stage=stage;out.layer=layer;
    if(!g.valid)return false;
    if(g.kind==NCG76SphereRuntimeGeometry::Kind::OrbitContour)return ContourSpan(g,stage,layer,out);
    if(g.kind!=NCG76SphereRuntimeGeometry::Kind::Sphere){
        if((g.kind!=NCG76SphereRuntimeGeometry::Kind::Line&&g.kind!=NCG76SphereRuntimeGeometry::Kind::Arc)||
            g.version!=NCG76SphereRuntimeGeometry::Version||g.circleLayerCount!=0U||
            stage!=Stage::InitialDepth||layer!=0U||!std::isfinite(g.travelMm)||g.travelMm<0.0||
            (g.travelMm==0.0&&g.kind!=NCG76SphereRuntimeGeometry::Kind::Line))return false;
        out.end=out.length=g.travelMm;return true;
    }
    if(!g.circleLayerCount||g.circleLayerCount>NCG76Sphere::MaximumLayers)return false;
    long double begin=0.0L,end=0.0L;
    if(stage==Stage::InitialDepth||stage==Stage::InitialRetreat){
        if(layer)return false;
        begin=stage==Stage::InitialDepth?0.0L:static_cast<long double>(g.travelMm);
        out.length=stage==Stage::InitialDepth?g.travelMm:g.radiusMm;end=begin+out.length;
    }else{
        if(layer>=g.circleLayerCount||stage<Stage::CircleEntry||stage>Stage::AdvanceCenter)return false;
        NCG76SphereRuntimeGeometry::Layer here{},next{};
        if(!NCG76SphereRuntimeGeometry::LayerAt(g,layer,here)||!NCG76SphereRuntimeGeometry::LayerAt(g,layer+1U,next))return false;
        const long double base=LayerStart(g,layer),r=here.radiusMm,arc=here.arcLengthMm;
        if(stage==Stage::CircleEntry){begin=base;end=base+r;out.length=here.radiusMm;}
        else if(stage==Stage::Orbit){begin=base+r;end=begin+arc;out.length=here.arcLengthMm;}
        else if(stage==Stage::ReturnCenter){begin=base+r+arc;end=begin+r;out.length=here.radiusMm;}
        else{begin=base+r+arc+r;end=LayerStart(g,layer+1U);out.length=next.depthMm-here.depthMm;}
    }
    out.start=static_cast<double>(begin);out.end=static_cast<double>(end);
    return std::isfinite(out.start)&&std::isfinite(out.end)&&std::isfinite(out.length)&&
        out.start>=0.0&&out.end>out.start&&out.length>0.0;
}
inline bool Station(const Geometry& g,Stage stage,unsigned layer,double fraction,double& station) noexcept
{
    station=0.0;Span span{};
    if(!GetSpan(g,stage,layer,span)||!std::isfinite(fraction)||fraction<0.0||fraction>1.0)return false;
    station=fraction==0.0?span.start:fraction==1.0?span.end:span.start+(span.end-span.start)*fraction;
    return std::isfinite(station)&&station>=span.start&&station<=span.end&&
        (fraction==0.0||station>span.start)&&(fraction==1.0||station<span.end);
}
inline bool MaximumFlushStation(const Geometry& g,double& station) noexcept
{
    Span span{};
    if(g.kind==NCG76SphereRuntimeGeometry::Kind::OrbitContour){
        if(NCG76OrbitContour::FinishEnabled(g)&&NCG76OrbitContour::IsStraightSideContour(g)){
            const unsigned count=NCG76OrbitContour::FinishLegCount(g);
            if(!count||!GetSpan(g,Stage::ContourFinish,count-1U,span))return false;
            station=span.end;return true;
        }
        if(!GetSpan(g,NCG76OrbitContour::FinishEnabled(g)?Stage::Orbit:Stage::ContourNominal,
            NCG76OrbitContour::FinishEnabled(g)?0U:NCG76OrbitContour::NominalLegCountFor(g)-2U,span))return false;
        station=span.end;return true;
    }
    const bool single=g.kind!=NCG76SphereRuntimeGeometry::Kind::Sphere;
    if(!GetSpan(g,single?Stage::InitialDepth:Stage::Orbit,single?0U:g.circleLayerCount-1U,span))return false;
    station=span.end;return true;
}
inline bool Locate(const Geometry& g,double station,bool reverse,Span& out) noexcept
{
    if(!std::isfinite(station)||station<0.0)return false;
    if(g.kind==NCG76SphereRuntimeGeometry::Kind::OrbitContour)return ContourLocate(g,station,reverse,out);
    if(g.kind!=NCG76SphereRuntimeGeometry::Kind::Sphere){
        if(g.travelMm==0.0||station>g.travelMm||(!reverse&&station==g.travelMm))return false;
        return GetSpan(g,Stage::InitialDepth,0U,out);
    }
    const double probe=reverse&&station>0.0?std::nextafter(station,-std::numeric_limits<double>::infinity()):station;
    if(probe<g.travelMm)return GetSpan(g,Stage::InitialDepth,0U,out);
    if(probe<static_cast<double>(static_cast<long double>(g.travelMm)+g.radiusMm))
        return GetSpan(g,Stage::InitialRetreat,0U,out);
    unsigned low=0U,high=g.circleLayerCount,iterations=0U;
    if(!high||high>NCG76Sphere::MaximumLayers)return false;
    while(low+1U<high){
        if(++iterations>13U)return false;
        const unsigned middle=low+(high-low)/2U;
        if(probe<static_cast<double>(LayerStart(g,middle)))high=middle;else low=middle;
    }
    for(Stage stage:{Stage::CircleEntry,Stage::Orbit,Stage::ReturnCenter,Stage::AdvanceCenter}){
        Span span{};if(!GetSpan(g,stage,low,span))return false;
        if(probe>=span.start&&probe<span.end){out=span;return station>=span.start&&station<=span.end;}
    }
    return false;
}
inline double Fraction(const Span& span,double station) noexcept
{return station==span.start?0.0:station==span.end?1.0:(station-span.start)/(span.end-span.start);}
inline bool Evaluate(const Geometry& g,const Span& span,double fraction,Point8& point,Point8& center) noexcept
{
    if(!std::isfinite(fraction)||fraction<0.0||fraction>1.0)return false;
    Point8 begin{},end{};Point3 tangent{},curvature{};
    if(g.kind==NCG76SphereRuntimeGeometry::Kind::OrbitContour){
        Span expected{};
        if(!ContourSpan(g,span.stage,span.layer,expected)||span.start!=expected.start||
           span.end!=expected.end||span.length!=expected.length)return false;
        if(span.stage==Stage::ContourNominal){
            double length=0.0,prefix=0.0;
            if(!NCG76OrbitContour::NominalLegAt(g,span.layer,begin,end,center,length,prefix)||
               !NCG76OrbitContour::NominalCenterAt(g,span.layer,fraction,center))return false;
        }else if(NCG76OrbitContour::IsStraightSideContour(g)){
            center=g.endMCS;
            if(span.stage==Stage::ContourFinish){
                double length=0.0,prefix=0.0;
                if(!NCG76OrbitContour::FinishLegAt(g,span.layer,begin,end,center,length,prefix))return false;
            }else{
                Point8 vertex{};
                if(!NCG76OrbitContour::FinishVertexAt(g,0U,vertex))return false;
                if(span.stage==Stage::CircleEntry){begin=g.endMCS;end=vertex;}
                else if(span.stage==Stage::ReturnCenter){begin=vertex;end=g.endMCS;}
                else return false;
            }
        }else{
            NCG76SphereRuntimeGeometry::Layer layer{};Point8 circle{};
            if(!NCG76SphereRuntimeGeometry::LayerAt(g,0U,layer)||
               !NCG76SphereRuntimeGeometry::CircleAt(g,0U,0.0,circle,tangent,curvature))return false;
            center=layer.center;
            if(span.stage==Stage::Orbit)return NCG76SphereRuntimeGeometry::CircleAt(g,0U,
                fraction==1.0?NCG76SphereRuntimeGeometry::TwoPi:fraction*NCG76SphereRuntimeGeometry::TwoPi,
                point,tangent,curvature);
            if(span.stage==Stage::CircleEntry){begin=layer.center;end=circle;}
            else if(span.stage==Stage::ReturnCenter){begin=circle;end=layer.center;}
            else return false;
        }
        point=fraction==0.0?begin:fraction==1.0?end:begin;
        if(fraction>0.0&&fraction<1.0)for(unsigned a=0U;a<3U;++a)point[a]=begin[a]+(end[a]-begin[a])*fraction;
        return NCG76SphereRuntimeGeometry::Detail::Finite(point)&&NCG76SphereRuntimeGeometry::Detail::Finite(center);
    }
    if(g.kind!=NCG76SphereRuntimeGeometry::Kind::Sphere){
        Span expected{};
        if(!GetSpan(g,span.stage,span.layer,expected)||span.start!=expected.start||
            span.end!=expected.end||span.length!=expected.length||
            !NCG76SphereRuntimeGeometry::NominalAt(g,fraction,point,tangent,curvature))return false;
        center=point;return true;
    }
    if(span.stage==Stage::InitialDepth){begin=g.startMCS;end=g.endMCS;}
    else{
        NCG76SphereRuntimeGeometry::Layer layer{},next{};
        Point8 circle{};
        if(!NCG76SphereRuntimeGeometry::LayerAt(g,span.layer,layer)||
            !NCG76SphereRuntimeGeometry::CircleAt(g,span.layer,0.0,circle,tangent,curvature))return false;
        if(span.stage==Stage::Orbit){
            if(!NCG76SphereRuntimeGeometry::CircleAt(g,span.layer,
                fraction*NCG76SphereRuntimeGeometry::TwoPi,point,tangent,curvature))return false;
            center=layer.center;return true;
        }
        if(span.stage==Stage::InitialRetreat){begin=g.endMCS;end=layer.center;}
        else if(span.stage==Stage::CircleEntry){begin=layer.center;end=circle;}
        else if(span.stage==Stage::ReturnCenter){begin=circle;end=layer.center;}
        else if(span.stage==Stage::AdvanceCenter){
            if(!NCG76SphereRuntimeGeometry::LayerAt(g,span.layer+1U,next))return false;
            begin=layer.center;end=next.center;
        }else return false;
        if(span.stage==Stage::CircleEntry||span.stage==Stage::ReturnCenter)center=layer.center;
    }
    point=fraction==0.0?begin:fraction==1.0?end:begin;
    if(fraction>0.0&&fraction<1.0)for(unsigned a=0U;a<3U;++a)point[a]=begin[a]+(end[a]-begin[a])*fraction;
    if(span.stage!=Stage::CircleEntry&&span.stage!=Stage::ReturnCenter)center=point;
    for(double x:point)if(!std::isfinite(x))return false;
    return true;
}
inline bool AxisEnvelope(const Geometry& g,unsigned mode,double height,unsigned axis,double& minimum,double& maximum) noexcept
{
    if(axis>=8U||!ValidMode(g,mode)||!std::isfinite(height)||height<0.0)return false;
    minimum=g.boundsMinMCS[axis];maximum=g.boundsMaxMCS[axis];
    if(g.kind==NCG76SphereRuntimeGeometry::Kind::Line&&g.travelMm==0.0)
        return std::isfinite(minimum)&&std::isfinite(maximum)&&minimum<=maximum;
    // A G01 B0 excursion follows the same native/mm ratio in every
    // participating axis. Rotary degrees are scaled by that ratio, not by an
    // XYZ tangent or an invented millimetre angular distance.
    if(g.kind==NCG76SphereRuntimeGeometry::Kind::Line&&(mode==0U||mode==3U||mode==4U)&&height>0.0&&axis<6U){
        const double ratio=(g.endMCS[axis]-g.startMCS[axis])/g.travelMm;
        const double offset=-ratio*height;
        if(!std::isfinite(ratio)||!std::isfinite(offset))return false;
        if(offset!=0.0){
            minimum=std::nextafter((std::min)(minimum,minimum+offset),-std::numeric_limits<double>::infinity());
            maximum=std::nextafter((std::max)(maximum,maximum+offset),std::numeric_limits<double>::infinity());
        }
        return std::isfinite(minimum)&&std::isfinite(maximum)&&minimum<=maximum;
    }
    // B0 freezes the current forward tangent at its stopped arc checkpoint.
    // Either tangent sign may occur over an arc, so admit the full physical
    // excursion independently of the particular sample that triggers E5.
    if(mode==0U&&g.kind==NCG76SphereRuntimeGeometry::Kind::OrbitContour&&height>0.0&&axis<3U){
        minimum=std::nextafter(minimum-height,-std::numeric_limits<double>::infinity());
        maximum=std::nextafter(maximum+height,std::numeric_limits<double>::infinity());
    }else if(mode==1U){
        if(axis==2U&&height>0.0)
            maximum=std::nextafter(maximum+height,std::numeric_limits<double>::infinity());
    }else if(mode==0U&&g.kind==NCG76SphereRuntimeGeometry::Kind::Arc&&height>0.0&&axis<3U){
        minimum=std::nextafter(minimum-height,-std::numeric_limits<double>::infinity());
        maximum=std::nextafter(maximum+height,std::numeric_limits<double>::infinity());
    }else if(mode!=2U&&height>0.0&&axis<3U){
        const double offset=-g.axis[axis]*height;
        minimum=std::nextafter((std::min)(minimum,minimum+offset),-std::numeric_limits<double>::infinity());
        maximum=std::nextafter((std::max)(maximum,maximum+offset),std::numeric_limits<double>::infinity());
    }
    return std::isfinite(minimum)&&std::isfinite(maximum)&&minimum<=maximum;
}
}
