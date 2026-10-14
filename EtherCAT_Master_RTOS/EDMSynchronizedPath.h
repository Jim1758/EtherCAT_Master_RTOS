#pragma once

#include "EDMCNCRetainedPath.h"

// EDM84: immutable XYZ canonical geometry plus three native rotary channels.
// The distance metric remains XYZ millimetres. C/U/V are unwrapped degrees,
// interpolated at the SAME canonical u; there is no shortest-path decision.
// This value has no pulse mapping, queue ownership or physical output permit.
namespace EDM84
{
using RotaryPoint=std::array<double,3U>;
class SynchronizedPath
{
public:
    // Fixed value storage; explicit noexcept keeps the FIX1 reset contract portable.
    SynchronizedPath() noexcept {}

    static constexpr unsigned Capacity=EDM82::RetainedPath::Capacity;
    bool Capture(const EDM82::RetainedPath& source,const RotaryPoint* boundaries,
        unsigned count,const EDM81::Scope& scope) noexcept
    {
        valid_=false; reason_=EDM82::PathReason::None;
        if(!EDM81::ValidScope(scope) || !EDM81::SameScope(scope,source.FrozenScope()))
            return Fail(EDM82::PathReason::Scope);
        if(!source.Valid() || !source.Count() || source.Count()>Capacity)
            return Fail(EDM82::PathReason::Geometry);
        if(!boundaries || count!=source.Count()+1U)
            return Fail(EDM82::PathReason::Capacity);
        for(unsigned p=0U;p<count;++p)
            for(unsigned axis=0U;axis<3U;++axis)
            {
                if(!EDM81::Finite(boundaries[p][axis])) return Fail(EDM82::PathReason::Geometry);
                if(p)
                {
                    const auto* row=source.Row(p-1U);
                    const double delta=boundaries[p][axis]-boundaries[p-1U][axis];
                    if(!row || row->lengthMM<=0.0 || !EDM81::Finite(delta) ||
                        !EDM81::Finite(delta/row->lengthMM)) return Fail(EDM82::PathReason::Metric);
                }
            }
        // No borrowed source pointer survives. Each seam uses one stored key,
        // so reverse traversal cannot introduce another rotary endpoint.
        path_=source;
        for(unsigned p=0U;p<count;++p) rotary_[p]=boundaries[p];
        valid_=true; return true;
    }
    bool Valid() const noexcept { return valid_; }
    EDM82::PathReason Failure() const noexcept { return reason_; }
    unsigned Count() const noexcept { return valid_?path_.Count():0U; }
    double LengthMm() const noexcept { return valid_?path_.LengthMm():0.0; }
    const EDM81::Scope& FrozenScope() const noexcept { return path_.FrozenScope(); }
    const NCPathCoreRetainedGeometry* Row(unsigned row) const noexcept
    { return valid_?path_.Row(row):nullptr; }
    double Station(unsigned boundary) const noexcept
    { return valid_?path_.Station(boundary):0.0; }

    bool Evaluate(double cursorMm,EDM81::Pose& pose,unsigned& ordinal,double& u) const noexcept
    {
        pose=EDM81::Pose{}; ordinal=0U; u=0.0;
        EDM81::Pose candidate{}; unsigned rowOrdinal=0U; double parameter=0.0;
        if(!valid_ || !path_.Evaluate(cursorMm,candidate,rowOrdinal,parameter) ||
            !rowOrdinal || rowOrdinal>Count()) return false;
        const unsigned row=rowOrdinal-1U;
        for(unsigned axis=0U;axis<3U;++axis)
        {
            const double a=rotary_[row][axis],b=rotary_[row+1U][axis];
            const double value=parameter==0.0?a:parameter==1.0?b:a+(b-a)*parameter;
            if(!EDM81::Finite(value) || value<EDM81::Minimum(a,b) || value>EDM81::Maximum(a,b))
                return false;
            candidate[axis+3U]=value;
        }
        pose=candidate; ordinal=rowOrdinal; u=parameter; return true;
    }

    EDM81::Reason NativeBounds(EDM81::Pose& minimum,EDM81::Pose& maximum) const noexcept
    {
        minimum=maximum=EDM81::Pose{};
        if(!valid_) return EDM81::Reason::Path;
        for(unsigned axis=0U;axis<3U;++axis)
        { minimum[axis]=maximum[axis]=path_.Row(0U)->startMCS[axis];
          minimum[axis+3U]=maximum[axis+3U]=rotary_[0U][axis]; }
        for(unsigned row=0U;row<Count();++row)
        {
            const auto& g=*path_.Row(row);
            for(unsigned axis=0U;axis<3U;++axis)
            {
                double low=EDM81::Minimum(g.startMCS[axis],g.endMCS[axis]);
                double high=EDM81::Maximum(g.startMCS[axis],g.endMCS[axis]);
                if(g.kind==NCPathCoreRetainedKind::ARC && axis<2U)
                { low=g.boundsMinMCS[axis]; high=g.boundsMaxMCS[axis]; }
                minimum[axis]=EDM81::Minimum(minimum[axis],low);
                maximum[axis]=EDM81::Maximum(maximum[axis],high);
                minimum[axis+3U]=EDM81::Minimum(minimum[axis+3U],rotary_[row+1U][axis]);
                maximum[axis+3U]=EDM81::Maximum(maximum[axis+3U],rotary_[row+1U][axis]);
            }
        }
        return EDM81::Reason::None;
    }
    EDM81::Reason ValidateBounds(const EDM81::Config& config) const noexcept
    {
        EDM81::Pose minimum{},maximum{},origin{}; unsigned ordinal=0U; double u=0.0;
        const auto result=NativeBounds(minimum,maximum);
        if(result!=EDM81::Reason::None || !Evaluate(0.0,origin,ordinal,u)) return EDM81::Reason::Path;
        for(unsigned axis=0U;axis<EDM81::AxisCount;++axis)
        {
            const auto& b=config.axes[axis];
            if(!ValidAxisBounds(b)) return EDM81::Reason::Configuration;
            if(minimum[axis]<b.minimum || maximum[axis]>b.maximum) return EDM81::Reason::AxisLimit;
            const double travel=EDM81::Maximum(std::fabs(minimum[axis]-origin[axis]),
                std::fabs(maximum[axis]-origin[axis]));
            if(!EDM81::Finite(travel)) return EDM81::Reason::NonFinite;
            if(travel>b.maxTravel) return EDM81::Reason::TravelLimit;
        }
        return EDM81::Reason::None;
    }

    // Bounds the ENTIRE interval, not just its endpoint chord. A canonical
    // XYZ seam allowance is included as seam travel / total cursor distance.
    EDM81::Reason DerivativeBounds(double from,double to,EDM81::Pose& perMm) const noexcept
    {
        perMm=EDM81::Pose{};
        if(!valid_ || !EDM81::Finite(from) || !EDM81::Finite(to) ||
            from<0.0 || to<0.0 || from>LengthMm() || to>LengthMm()) return EDM81::Reason::Path;
        if(from==to) return EDM81::Reason::None;
        const double low=EDM81::Minimum(from,to),high=EDM81::Maximum(from,to);
        const double distance=high-low;
        EDM81::Pose derivative{},seamTravel{};
        for(unsigned row=0U;row<Count();++row)
        {
            if(high<=Station(row) || low>=Station(row+1U)) continue;
            const auto& g=*Row(row);
            const double begin=EDM81::Maximum(0.0,(low-Station(row))/g.lengthMM);
            const double end=EDM81::Minimum(1.0,(high-Station(row))/g.lengthMM);
            for(unsigned axis=0U;axis<EDM81::AxisCount;++axis)
            {
                double d=0.0;
                if(axis>=3U) d=std::fabs(rotary_[row+1U][axis-3U]-rotary_[row][axis-3U])/g.lengthMM;
                else if(g.kind==NCPathCoreRetainedKind::LINE)
                    d=std::fabs(g.endMCS[axis]-g.startMCS[axis])/g.lengthMM;
                else if(axis<2U)
                    d=g.radiusMM*std::fabs(g.sweepRadians)/g.lengthMM*
                        MaximumAbsTrig(g.startAngle+begin*g.sweepRadians,
                            g.startAngle+end*g.sweepRadians,axis==0U);
                if(!EDM81::Finite(d)) return EDM81::Reason::NonFinite;
                derivative[axis]=EDM81::Maximum(derivative[axis],d);
            }
        }
        for(unsigned row=1U;row<Count();++row)
            if(low<=Station(row) && high>Station(row))
                for(unsigned axis=0U;axis<3U;++axis)
                    seamTravel[axis]+=std::fabs(Row(row)->startMCS[axis]-Row(row-1U)->endMCS[axis]);
        for(unsigned axis=0U;axis<EDM81::AxisCount;++axis)
        {
            derivative[axis]+=seamTravel[axis]/distance;
            if(!EDM81::Finite(derivative[axis])) return EDM81::Reason::NonFinite;
        }
        perMm=derivative; return EDM81::Reason::None;
    }
    EDM81::Reason ValidateTraversal(double from,double to,const EDM81::Config& config) const noexcept
    {
        if(config.periodUs!=250U) return EDM81::Reason::Configuration;
        EDM81::Pose derivative{};
        const auto result=DerivativeBounds(from,to,derivative);
        if(result!=EDM81::Reason::None) return result;
        const double speed=std::fabs(to-from)*(60000000.0/static_cast<double>(config.periodUs));
        if(!EDM81::Finite(speed)) return EDM81::Reason::NonFinite;
        for(unsigned axis=0U;axis<EDM81::AxisCount;++axis)
        {
            if(!ValidAxisBounds(config.axes[axis])) return EDM81::Reason::Configuration;
            const double rate=derivative[axis]*speed;
            if(!EDM81::Finite(rate)) return EDM81::Reason::NonFinite;
            if(rate>config.axes[axis].maxVelocityPerMin*(1.0+1e-12)) return EDM81::Reason::VelocityLimit;
        }
        return EDM81::Reason::None;
    }
    static bool ValidAxisBounds(const EDM81::AxisBounds& b) noexcept
    {
        return EDM81::Finite(b.minimum) && EDM81::Finite(b.maximum) && b.minimum<b.maximum &&
            EDM81::Finite(b.maxTravel) && b.maxTravel>0.0 &&
            EDM81::Finite(b.maxVelocityPerMin) && b.maxVelocityPerMin>0.0;
    }
private:
    static double MaximumAbsTrig(double a,double b,bool sine) noexcept
    {
        constexpr double pi=3.141592653589793238462643383279502884;
        const double low=EDM81::Minimum(a,b),high=EDM81::Maximum(a,b);
        double maximum=EDM81::Maximum(std::fabs(sine?std::sin(a):std::cos(a)),
            std::fabs(sine?std::sin(b):std::cos(b)));
        // Original validated canonical angles stay within [-3*pi,3*pi].
        for(int k=-4;k<=4;++k)
        {
            const double extremum=static_cast<double>(k)*pi+(sine?pi*0.5:0.0);
            if(extremum>=low && extremum<=high) maximum=1.0;
        }
        return maximum;
    }
    bool Fail(EDM82::PathReason reason) noexcept { reason_=reason; valid_=false; return false; }
    EDM82::RetainedPath path_{};
    std::array<RotaryPoint,Capacity+1U> rotary_{};
    EDM82::PathReason reason_=EDM82::PathReason::None;
    bool valid_=false;
};
static_assert(std::is_trivially_copyable<SynchronizedPath>::value,"EDM84 path must be copied by value");
static_assert(sizeof(SynchronizedPath)<7168U,"EDM84 fixed path budget changed");
}
