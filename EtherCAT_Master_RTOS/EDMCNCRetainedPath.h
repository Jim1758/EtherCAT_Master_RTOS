#pragma once

#include "NCPathCoreRetainedPath.h"
#include "EDMMultiAxisKernel.h"
#include <cstring>

// EDM82: copied, bounded, read-only CNC geometry for numerical simulation.
// The source supplies already-built canonical geometry and a caller-owned
// lifetime stamp. This is NOT a live CNC queue, owner lease or pulse-scale proof.
// Existing canonical MCS/pulse seam checks are preserved. No source cursor is
// changed, and no pointer into the source survives Capture().
// Capture is a same-thread operation: its source must remain immutable until
// the call returns. This object is not a concurrent publication mechanism.
namespace EDM82
{
enum class PathReason : std::uint8_t
{
    None, Lifecycle, Scope, AxisMask, Scale, Capacity, Geometry,
    UnsupportedGeometry, Discontinuity, Metric, Cursor
};

class RetainedPath
{
public:
    static constexpr unsigned Capacity=NCPathCoreRetainedPath::Capacity;
    bool Capture(const NCPathCoreRetainedPath& source,const EDM81::Scope& scope,
        const std::array<double,8U>& pulsePerNativeUnit) noexcept
    {
        valid_=false; count_=0U; lengthMm_=0.0; reason_=PathReason::None;
        if(!EDM81::ValidScope(scope)) return Fail(PathReason::Scope);
        if(source.Pending() || source.DistanceMode() ||
            source.State()!=NCPathCoreRetainedCursorState::IDLE ||
            source.Fault()!=NCPathCoreRetainedFault::NONE)
            return Fail(PathReason::Lifecycle);
        if(source.ValidAxisMask()!=0x3FU) return Fail(PathReason::AxisMask);
        if(!source.Count() || source.Count()>Capacity) return Fail(PathReason::Capacity);
        for(unsigned axis=0U;axis<8U;++axis)
            if(!EDM81::Finite(pulsePerNativeUnit[axis]) || pulsePerNativeUnit[axis]<=0.0)
                return Fail(PathReason::Scale);
        station_[0]=0.0;
        for(unsigned row=0U;row<source.Count();++row)
        {
            const auto* value=source.Get(row);
            if(!value || !IsNCPathCoreRetainedGeometryValid(*value))
                return Fail(PathReason::Geometry);
            if((value->kind!=NCPathCoreRetainedKind::LINE &&
                value->kind!=NCPathCoreRetainedKind::ARC) || value->point)
                return Fail(PathReason::UnsupportedGeometry);
            // Existing retained ARC is explicitly G17 XY, not the broader
            // G18/G19 FeedArc type. C/U/V and unsupported slots 6/7 stay held.
            if((value->kind==NCPathCoreRetainedKind::ARC && value->axisMask!=3U) ||
                (value->axisMask & ~7U)) return Fail(PathReason::UnsupportedGeometry);
            if(value->lengthMM<=0.0) return Fail(PathReason::Metric);
            if(row && !AreNCPathCoreRetainedEndpointsConnected(rows_[row-1U],*value,
                0x3FU,pulsePerNativeUnit)) return Fail(PathReason::Discontinuity);
            rows_[row]=*value;
            station_[row+1U]=station_[row]+value->lengthMM;
            if(!EDM81::Finite(station_[row+1U]) || station_[row+1U]<=station_[row])
                return Fail(PathReason::Metric);
        }
        scope_=scope; scale_=pulsePerNativeUnit;
        count_=source.Count(); lengthMm_=station_[count_]; valid_=true;
        return true;
    }
    bool Valid() const noexcept { return valid_; }
    PathReason Failure() const noexcept { return reason_; }
    unsigned Count() const noexcept { return count_; }
    double LengthMm() const noexcept { return lengthMm_; }
    const EDM81::Scope& FrozenScope() const noexcept { return scope_; }
    const NCPathCoreRetainedGeometry* Row(unsigned row) const noexcept
    { return valid_ && row<count_?&rows_[row]:nullptr; }
    double Station(unsigned boundary) const noexcept
    { return valid_ && boundary<=count_?station_[boundary]:0.0; }

    bool Evaluate(double cursorMm,EDM81::Pose& pose,unsigned& ordinal,double& u) const noexcept
    {
        pose=EDM81::Pose{}; ordinal=0U; u=0.0;
        if(!valid_ || !EDM81::Finite(cursorMm) || cursorMm<0.0 || cursorMm>lengthMm_)
            return false;
        // Seams deterministically belong to the preceding row's exact end.
        // Traversal direction does not rebase or reverse the original geometry.
        unsigned row=0U;
        if(cursorMm==lengthMm_) { row=count_-1U; u=1.0; }
        else
        {
            while(row+1U<count_ && cursorMm>station_[row+1U]) ++row;
            if(cursorMm==station_[row]) u=0.0;
            else if(cursorMm==station_[row+1U]) u=1.0;
            else u=(cursorMm-station_[row])/rows_[row].lengthMM;
        }
        for(unsigned axis=0U;axis<EDM81::AxisCount;++axis)
            if(!EvaluateNCPathCoreRetainedAxisCanonical(rows_[row],axis,u,pose[axis]))
                return false;
        // Native retained distance cursors use 1-based original ordinals.
        // Row(index) and Station(boundary) remain conventional 0-based access.
        ordinal=row+1U;
        return true;
    }

    EDM81::Reason ValidateBounds(const EDM81::Config& config) const noexcept
    {
        if(!valid_) return EDM81::Reason::Path;
        const auto& origin=rows_[0].startMCS;
        for(unsigned row=0U;row<count_;++row)
            for(unsigned axis=0U;axis<EDM81::AxisCount;++axis)
            {
                const auto& g=rows_[row]; const auto& b=config.axes[axis];
                double low=EDM81::Minimum(g.startMCS[axis],g.endMCS[axis]);
                double high=EDM81::Maximum(g.startMCS[axis],g.endMCS[axis]);
                if(g.kind==NCPathCoreRetainedKind::ARC && axis<2U)
                { low=g.boundsMinMCS[axis]; high=g.boundsMaxMCS[axis]; }
                if(low<b.minimum || high>b.maximum) return EDM81::Reason::AxisLimit;
                const double travel=EDM81::Maximum(std::fabs(low-origin[axis]),
                    std::fabs(high-origin[axis]));
                if(!EDM81::Finite(travel)) return EDM81::Reason::NonFinite;
                if(travel>b.maxTravel) return EDM81::Reason::TravelLimit;
            }
        return EDM81::Reason::None;
    }

    EDM81::Reason ValidateTraversal(double from,double to,const EDM81::Config& config) const noexcept
    {
        if(!valid_ || !EDM81::Finite(from) || !EDM81::Finite(to) ||
            from<0.0 || to<0.0 || from>lengthMm_ || to>lengthMm_)
            return EDM81::Reason::Path;
        if(from==to) return EDM81::Reason::None;
        const double low=EDM81::Minimum(from,to),high=EDM81::Maximum(from,to);
        const double scale=60000000.0/static_cast<double>(config.periodUs);
        const double speed=(high-low)*scale;
        if(!EDM81::Finite(speed)) return EDM81::Reason::NonFinite;
        EDM81::Pose derivative{},seamTravel{};
        for(unsigned row=0U;row<count_;++row)
        {
            if(high<=station_[row] || low>=station_[row+1U]) continue;
            const auto& g=rows_[row];
            const double begin=EDM81::Maximum(0.0,(low-station_[row])/g.lengthMM);
            const double end=EDM81::Minimum(1.0,(high-station_[row])/g.lengthMM);
            for(unsigned axis=0U;axis<EDM81::AxisCount;++axis)
            {
                double d=0.0;
                if(g.kind==NCPathCoreRetainedKind::LINE)
                    d=std::fabs(g.endMCS[axis]-g.startMCS[axis])/g.lengthMM;
                else if(axis<2U)
                {
                    // Bound the native derivative on the entire crossed arc
                    // span, including interior sine/cosine extrema. Endpoint
                    // chord averaging must never hide a high interior rate.
                    const double a=g.startAngle+begin*g.sweepRadians;
                    const double b=g.startAngle+end*g.sweepRadians;
                    d=g.radiusMM*std::fabs(g.sweepRadians)/g.lengthMM*
                        MaximumAbsTrig(a,b,axis==0U);
                }
                if(!EDM81::Finite(d)) return EDM81::Reason::NonFinite;
                derivative[axis]=EDM81::Maximum(derivative[axis],d);
            }
        }
        for(unsigned row=1U;row<count_;++row)
            if(low<=station_[row] && high>station_[row])
                for(unsigned axis=0U;axis<EDM81::AxisCount;++axis)
                    seamTravel[axis]+=std::fabs(rows_[row].startMCS[axis]-rows_[row-1U].endMCS[axis]);
        for(unsigned axis=0U;axis<EDM81::AxisCount;++axis)
        {
            const double rate=derivative[axis]*speed+seamTravel[axis]*scale;
            if(!EDM81::Finite(rate)) return EDM81::Reason::NonFinite;
            if(rate>config.axes[axis].maxVelocityPerMin*(1.0+1e-12))
                return EDM81::Reason::VelocityLimit;
        }
        return EDM81::Reason::None;
    }
private:
    static double MaximumAbsTrig(double a,double b,bool sine) noexcept
    {
        constexpr double pi=3.141592653589793238462643383279502884;
        const double low=EDM81::Minimum(a,b),high=EDM81::Maximum(a,b);
        double maximum=EDM81::Maximum(std::fabs(sine?std::sin(a):std::cos(a)),
            std::fabs(sine?std::sin(b):std::cos(b)));
        // All canonical angles are within [-3*pi,3*pi]. Fixed bounds keep the
        // numerical work independent of requested geometry or runtime input.
        for(int k=-4;k<=4;++k)
        {
            const double extremum=static_cast<double>(k)*pi+(sine?pi*0.5:0.0);
            if(extremum>=low && extremum<=high) maximum=1.0;
        }
        return maximum;
    }
    bool Fail(PathReason reason) noexcept
    { reason_=reason; valid_=false; count_=0U; lengthMm_=0.0; return false; }
    std::array<NCPathCoreRetainedGeometry,Capacity> rows_{};
    std::array<double,Capacity+1U> station_{};
    std::array<double,8U> scale_{};
    EDM81::Scope scope_{};
    double lengthMm_=0.0;
    unsigned count_=0U;
    PathReason reason_=PathReason::None;
    bool valid_=false;
};
static_assert(sizeof(RetainedPath)<6656U,"EDM82 retained geometry budget changed");
}
