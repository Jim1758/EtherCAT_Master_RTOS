#pragma once
#include "NCG76OrbitPlan.h"
#include "NCPathCoreRetainedPath.h"
#include "MotionExecutionContract.h"
#include "NCTranslationArcPrecision.h"
#include <array>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <limits>

// Bounded native canonical geometry and historical source evidence only.
// No allocation, command publication, owner acquisition, GAP or I/O occurs.
// Input and output must not overlap. Large Input/Plan values are caller-owned;
// production callers allocate them at startup, never on the RTSS stack.
namespace NCG76Mixed
{
using Point8 = NCG76::Point8;
using Geometry = NCPathCoreRetainedGeometry;
constexpr unsigned HistoryCapacity = 32U;
constexpr unsigned MaximumNominal = 32U;
constexpr unsigned MaximumLegs = 128U;
constexpr unsigned NoIndex = 0xffffffffU;
constexpr double MaximumRetreatMm = 2.0;

enum class Kind : std::uint8_t { LINE = 1U, ARC = 2U, G76 = 3U };
enum class Phase : std::uint8_t { Nominal = 0U, Retreat = 1U, Return = 2U };
enum class Status : std::uint8_t
{
    EMPTY = 0U, BUILT = 1U, INVALID_SCOPE = 2U, INVALID_HISTORY = 3U,
    INVALID_NOMINAL = 4U, INVALID_DISTANCE = 5U, INSUFFICIENT_HISTORY = 6U,
    DISCONTINUITY = 7U, CAPACITY = 8U, INVALID_FEED = 9U
};
struct Scope
{
    std::uint64_t run = 0ULL, cache = 0ULL, translationGeneration = 0ULL;
    MotionOwnerLease lease{};
};
struct HistoryRow
{
    Geometry geometry{};
    MotionExecutionIdentity identity{};
    MotionOwnerLease lease{};
    std::uint64_t run = 0ULL, cache = 0ULL, dispatch = 0ULL, commit = 0ULL,
        translationGeneration = 0ULL;
    int pc = -1, line = 0;
    unsigned childOrdinal = 0U;
};
struct HistorySnapshot
{
    std::array<HistoryRow, HistoryCapacity> rows{};
    unsigned count = 0U, validAxisMask = 0U;
    Point8 pulsePerUnit{};
};
struct Leg
{
    Geometry geometry{};
    double startU = 0.0, endU = 0.0, feedPhysicalMmMin = 0.0;
    Phase phase = Phase::Nominal;
    unsigned nominalOrdinal = 0U, originalSourceIndex = NoIndex;
};
struct NominalSource
{
    std::array<Geometry, MaximumNominal> geometry{};
    std::array<double, MaximumNominal> feedMmMin{};
    unsigned count = 0U, triggerAfter = 0U;
    Kind kind = Kind::LINE;
    NCG76::NativePlan orbit{};
    int pc = -1, line = 0;
};
struct Input
{
    Scope scope{};
    NCTranslationSnapshot translation{};
    HistorySnapshot history{};
    NominalSource nominal{};
    double retreatMm = 0.0, outFeedMmMin = 0.0, returnFeedMmMin = 0.0;
};
struct Plan
{
    Input input{};
    std::array<Leg, MaximumLegs> legs{};
    unsigned count = 0U, reverseCount = 0U;
    double availableMm = 0.0;
    Status status = Status::EMPTY;
    void Clear() noexcept;
};

bool SameGeometry(const Geometry&, const Geometry&) noexcept;
bool SameScope(const Scope&, const Scope&) noexcept;
bool FrameSupported(const NCTranslationSnapshot&) noexcept;
const char* StatusName(Status) noexcept;
bool BuildG76Nominal(const NCG76::NativePlan&,const Point8& startPulse,
    const Point8& pulsePerUnit,int pc,int line,NominalSource&) noexcept;
bool Build(const Input&,Plan&) noexcept;
bool Valid(const Plan&) noexcept;
bool Valid(const Plan&,Plan& rebuildScratch) noexcept;

class RollingHistory
{
public:
    void Clear() noexcept;
    bool Append(const HistoryRow&,unsigned validAxisMask,const Point8& pulsePerUnit) noexcept;
    bool Snapshot(HistorySnapshot&) const noexcept;
    unsigned Count() const noexcept { return m_rows.count; }
    const HistoryRow* Get(unsigned index) const noexcept;
    Status Fault() const noexcept { return m_fault; }
private:
    HistorySnapshot m_rows{};
    Status m_fault = Status::EMPTY;
};

namespace Detail
{
inline bool Finite(double v) noexcept { return std::isfinite(v) != 0; }
inline bool Feed(double v) noexcept { return Finite(v) && v > 0.0 && v <= NCG76::MaximumFeedMmMin; }
template<class T> inline bool Bits(const T& a,const T& b) noexcept
{ return std::memcmp(&a,&b,sizeof(T)) == 0; }
template<std::size_t N> inline bool ArrayBits(const std::array<double,N>& a,const std::array<double,N>& b) noexcept
{ for (std::size_t j=0U;j<N;++j) if (!Bits(a[j],b[j])) return false; return true; }
inline void ClearRow(HistoryRow& r) noexcept
{
    r.geometry.Clear(); r.identity=MotionExecutionIdentity{}; r.lease=MotionOwnerLease{};
    r.run=r.cache=r.dispatch=r.commit=r.translationGeneration=0ULL;
    r.pc=-1; r.line=0; r.childOrdinal=0U;
}
inline void ClearHistory(HistorySnapshot& h) noexcept
{
    for (auto& r:h.rows) ClearRow(r);
    h.count=h.validAxisMask=0U; h.pulsePerUnit.fill(0.0);
}
inline void ClearNominal(NominalSource& n) noexcept
{
    for (auto& g:n.geometry) g.Clear();
    n.feedMmMin.fill(0.0); n.count=n.triggerAfter=0U; n.kind=Kind::LINE;
    n.orbit=NCG76::NativePlan{}; n.pc=-1; n.line=0;
}
inline bool ScopeValid(const Scope& s) noexcept
{
    return s.run!=0ULL && s.cache!=0ULL && s.translationGeneration!=0ULL &&
        s.lease.IsValid() && s.lease.owner==MotionOwner::AUTO;
}
inline bool Unit(double pulse,double mm,double ppu) noexcept
{
    if (!Finite(pulse) || !Finite(mm) || !Finite(ppu) || ppu<=0.0) return false;
    const double converted=pulse/ppu;
    const double scale=(std::max)(1.0,(std::max)(std::fabs(mm),std::fabs(converted)));
    const double error=std::fabs(converted-mm);
    return Finite(converted) && error<=64.0*std::numeric_limits<double>::epsilon()*scale && error<=1e-7;
}
inline bool Calibration(unsigned mask,const Point8& ppu) noexcept
{
    if ((mask&7U)!=7U || (mask&~255U)!=0U) return false;
    for (unsigned a=0U;a<8U;++a)
        if (!Finite(ppu[a]) || ((mask&(1U<<a))!=0U ? ppu[a]<=0.0 : ppu[a]!=0.0)) return false;
    return true;
}
inline bool GeometryValid(const Geometry& g,unsigned mask,const Point8& ppu) noexcept
{
    if (!IsNCPathCoreRetainedGeometryValid(g) || g.point ||
        (g.kind!=NCPathCoreRetainedKind::LINE && g.kind!=NCPathCoreRetainedKind::ARC) ||
        !(g.lengthMM>0.0) || !(g.lengthPulse>0.0) || (g.axisMask&mask)!=g.axisMask) return false;
    for (unsigned a=0U;a<8U;++a)
        if ((mask&(1U<<a))!=0U &&
            (!Unit(g.startPulse[a],g.startMCS[a],ppu[a]) || !Unit(g.endPulse[a],g.endMCS[a],ppu[a]))) return false;
    return g.kind!=NCPathCoreRetainedKind::ARC || (ppu[0]==ppu[1] &&
        Unit(g.centerPulse[0],g.centerMCS[0],ppu[0]) && Unit(g.centerPulse[1],g.centerMCS[1],ppu[1]) &&
        Unit(g.radiusPulse,g.radiusMM,ppu[0]));
}
inline bool Allowance(const Geometry& g,const NCTranslationSnapshot& translation) noexcept
{
    if (g.sourceRoundoffMM==0.0F) return true;
    float expected=0.0F;
    return TryGetNCTranslationArcRoundoffMM(translation,expected,g.fullCircle) &&
        Bits(expected,g.sourceRoundoffMM);
}
inline bool Proof(const HistoryRow& r,const Scope& s) noexcept
{
    return ScopeValid(s) && r.run==s.run && r.cache==s.cache &&
        r.translationGeneration==s.translationGeneration && r.lease.Matches(s.lease) &&
        r.identity.IsAssigned() && r.identity.source==MotionCommandSource::NC_MEMORY &&
        r.identity.sourceBlockId==r.pc && r.pc>=0 && r.line>0 &&
        r.dispatch!=0ULL && r.commit!=0ULL && r.childOrdinal>0U;
}
inline bool Ordered(const HistoryRow& a,const HistoryRow& b) noexcept
{
    if (a.identity.segmentId==b.identity.segmentId) return false;
    if (b.dispatch==a.dispatch)
        return b.commit==a.commit && b.pc==a.pc && b.line==a.line &&
            b.identity.epoch==a.identity.epoch && b.childOrdinal>a.childOrdinal;
    return b.dispatch>a.dispatch && b.commit>a.commit;
}
inline const Geometry& Source(const Input& in,unsigned index) noexcept
{
    return index<in.history.count ? in.history.rows[index].geometry :
        in.nominal.geometry[index-in.history.count];
}
inline bool CanonicalOrbitRow(const NCG76::NativePlan& orbit,unsigned index,
    const Point8& initialPulse,const Point8& ppu,const Point8& previousPulse,Geometry& out) noexcept
{
    out.Clear();
    if (index>=orbit.count) return false;
    out.startMCS=index==0U?orbit.startMCS:orbit.pointsMCS[index-1U];
    out.endMCS=orbit.pointsMCS[index]; out.startPulse=previousPulse; out.endPulse=previousPulse;
    for (unsigned a=0U;a<3U;++a)
    {
        out.endPulse[a]=out.endMCS[a]==out.startMCS[a]?previousPulse[a]:
            out.endMCS[a]==orbit.startMCS[a]?initialPulse[a]:out.endMCS[a]*ppu[a];
        if (!Finite(out.endPulse[a])) return false;
        out.lengthMM=std::hypot(out.lengthMM,out.endMCS[a]-out.startMCS[a]);
        out.lengthPulse=std::hypot(out.lengthPulse,out.endPulse[a]-out.startPulse[a]);
    }
    for (unsigned a=0U;a<2U;++a)
    {
        out.boundsMinMCS[a]=(std::min)(out.startMCS[a],out.endMCS[a]);
        out.boundsMaxMCS[a]=(std::max)(out.startMCS[a],out.endMCS[a]);
    }
    out.axisMask=7U; out.kind=NCPathCoreRetainedKind::LINE; out.valid=true;
    return IsNCPathCoreRetainedGeometryValid(out) && !out.point && out.lengthMM>0.0 && out.lengthPulse>0.0;
}
inline bool SameLeg(const Leg& a,const Leg& b) noexcept
{
    return SameGeometry(a.geometry,b.geometry) && Bits(a.startU,b.startU) && Bits(a.endU,b.endU) &&
        Bits(a.feedPhysicalMmMin,b.feedPhysicalMmMin) && a.phase==b.phase &&
        a.nominalOrdinal==b.nominalOrdinal && a.originalSourceIndex==b.originalSourceIndex;
}
template<class Emit>
inline Status Generate(const Input& in,Emit emit,double& available,unsigned& reverseCount,unsigned& count) noexcept
{
    available=0.0; reverseCount=count=0U;
    if (!ScopeValid(in.scope) || !FrameSupported(in.translation) ||
        in.scope.run!=in.translation.runToken || in.scope.translationGeneration!=in.translation.generation ||
        !Calibration(in.history.validAxisMask,in.history.pulsePerUnit)) return Status::INVALID_SCOPE;
    unsigned expectedMask=0U;
    for (unsigned a=0U;a<8U;++a) if (in.translation.axisIdentity.exists[a]!=0U) expectedMask|=1U<<a;
    if (expectedMask!=in.history.validAxisMask) return Status::INVALID_SCOPE;
    if (!Finite(in.retreatMm) || in.retreatMm<=0.0 || in.retreatMm>MaximumRetreatMm) return Status::INVALID_DISTANCE;
    if (!Feed(in.outFeedMmMin) || !Feed(in.returnFeedMmMin)) return Status::INVALID_FEED;
    const auto& h=in.history; const auto& n=in.nominal;
    if (h.count>HistoryCapacity) return Status::CAPACITY;
    for (unsigned i=0U;i<h.count;++i)
    {
        if (!Proof(h.rows[i],in.scope) || !GeometryValid(h.rows[i].geometry,h.validAxisMask,h.pulsePerUnit) ||
            !Allowance(h.rows[i].geometry,in.translation))
            return Status::INVALID_HISTORY;
        for (unsigned j=0U;j<i;++j)
            if (h.rows[i].identity.segmentId==h.rows[j].identity.segmentId) return Status::INVALID_HISTORY;
        if (i!=0U && (!Ordered(h.rows[i-1U],h.rows[i]) ||
            !AreNCPathCoreRetainedEndpointsConnected(h.rows[i-1U].geometry,h.rows[i].geometry,h.validAxisMask,h.pulsePerUnit)))
            return Status::DISCONTINUITY;
    }
    if (n.count==0U || n.count>MaximumNominal || n.pc<0 || n.line<=0 || n.triggerAfter==0U || n.triggerAfter>n.count)
        return Status::INVALID_NOMINAL;
    if ((n.kind==Kind::LINE && (n.count!=1U || n.triggerAfter!=1U || n.geometry[0].kind!=NCPathCoreRetainedKind::LINE)) ||
        (n.kind==Kind::ARC && (n.count!=1U || n.triggerAfter!=1U || n.geometry[0].kind!=NCPathCoreRetainedKind::ARC)) ||
        (n.kind!=Kind::LINE && n.kind!=Kind::ARC && n.kind!=Kind::G76)) return Status::INVALID_NOMINAL;
    if (n.kind==Kind::G76 && (n.count!=25U || n.triggerAfter!=12U ||
        n.orbit.program.input.flushMode!=2U || !NCG76::ValidNativeBase(n.orbit) ||
        !SameNCTranslationSnapshot(n.orbit.translation,in.translation))) return Status::INVALID_NOMINAL;
    Point8 previousPulse=n.geometry[0].startPulse;
    const Point8 initialPulse=previousPulse;
    for (unsigned i=0U;i<n.count;++i)
    {
        if (!GeometryValid(n.geometry[i],h.validAxisMask,h.pulsePerUnit) || !Feed(n.feedMmMin[i]) ||
            !Allowance(n.geometry[i],in.translation)) return Status::INVALID_NOMINAL;
        if (i!=0U && !AreNCPathCoreRetainedEndpointsConnected(n.geometry[i-1U],n.geometry[i],h.validAxisMask,h.pulsePerUnit))
            return Status::DISCONTINUITY;
        if (n.kind==Kind::G76)
        {
            Geometry expected{};
            if (!CanonicalOrbitRow(n.orbit,i,initialPulse,h.pulsePerUnit,previousPulse,expected) ||
                !SameGeometry(expected,n.geometry[i]) || !Bits(n.feedMmMin[i],n.orbit.feedMmMin[i])) return Status::INVALID_NOMINAL;
            previousPulse=expected.endPulse;
        }
    }
    if (h.count!=0U && !AreNCPathCoreRetainedEndpointsConnected(h.rows[h.count-1U].geometry,n.geometry[0],h.validAxisMask,h.pulsePerUnit))
        return Status::DISCONTINUITY;
    const unsigned total=h.count+n.triggerAfter;
    const unsigned lower=total>HistoryCapacity?total-HistoryCapacity:0U;
    for (unsigned i=lower;i<total;++i)
    {
        available+=Source(in,i).lengthMM;
        if (!Finite(available)) return Status::INVALID_HISTORY;
    }
    if (in.retreatMm>available) return Status::INSUFFICIENT_HISTORY;
    struct Interval { unsigned source=0U; double start=0.0,end=0.0; };
    std::array<Interval,HistoryCapacity> reverse{};
    double remaining=in.retreatMm;
    for (unsigned cursor=total;cursor>lower && remaining>0.0;)
    {
        --cursor; const double length=Source(in,cursor).lengthMM;
        if (reverseCount>=HistoryCapacity) return Status::CAPACITY;
        Interval& step=reverse[reverseCount++]; step.source=cursor; step.start=1.0;
        if (remaining>=length)
        {
            step.end=0.0; remaining-=length;
            // Exact requested equality to the published full available length
            // means all retained rows, despite subtraction-order roundoff.
            if (cursor==lower && in.retreatMm==available) remaining=0.0;
        }
        else
        {
            step.end=1.0-remaining/length; remaining=0.0;
            if (!Finite(step.end) || step.end<0.0 || step.end>=1.0) return Status::INVALID_DISTANCE;
        }
    }
    if (remaining>0.0 || reverseCount==0U) return Status::INSUFFICIENT_HISTORY;
    if (n.count+2U*reverseCount>MaximumLegs) return Status::CAPACITY;
    Leg leg{};
    const auto output=[&](const Leg& item)->bool { return emit(item,count++); };
    for (unsigned i=0U;i<n.triggerAfter;++i)
    {
        leg.geometry=n.geometry[i]; leg.startU=0.0; leg.endU=1.0; leg.feedPhysicalMmMin=n.feedMmMin[i];
        leg.phase=Phase::Nominal; leg.nominalOrdinal=i+1U; leg.originalSourceIndex=h.count+i;
        if (!output(leg)) return Status::INVALID_NOMINAL;
    }
    for (unsigned i=0U;i<reverseCount;++i)
    {
        leg.geometry=Source(in,reverse[i].source); leg.startU=reverse[i].start; leg.endU=reverse[i].end;
        leg.feedPhysicalMmMin=in.outFeedMmMin; leg.phase=Phase::Retreat; leg.nominalOrdinal=0U;
        leg.originalSourceIndex=reverse[i].source;
        if (!output(leg)) return Status::INVALID_NOMINAL;
    }
    for (unsigned i=reverseCount;i>0U;--i)
    {
        leg.geometry=Source(in,reverse[i-1U].source); leg.startU=reverse[i-1U].end; leg.endU=reverse[i-1U].start;
        leg.feedPhysicalMmMin=in.returnFeedMmMin; leg.phase=Phase::Return; leg.nominalOrdinal=0U;
        leg.originalSourceIndex=reverse[i-1U].source;
        if (!output(leg)) return Status::INVALID_NOMINAL;
    }
    for (unsigned i=n.triggerAfter;i<n.count;++i)
    {
        leg.geometry=n.geometry[i]; leg.startU=0.0; leg.endU=1.0; leg.feedPhysicalMmMin=n.feedMmMin[i];
        leg.phase=Phase::Nominal; leg.nominalOrdinal=i+1U; leg.originalSourceIndex=h.count+i;
        if (!output(leg)) return Status::INVALID_NOMINAL;
    }
    return Status::BUILT;
}
} // namespace Detail

inline bool SameGeometry(const Geometry& a,const Geometry& b) noexcept
{
    return Detail::ArrayBits(a.startMCS,b.startMCS) && Detail::ArrayBits(a.endMCS,b.endMCS) &&
        Detail::ArrayBits(a.startPulse,b.startPulse) && Detail::ArrayBits(a.endPulse,b.endPulse) &&
        Detail::ArrayBits(a.centerMCS,b.centerMCS) && Detail::ArrayBits(a.centerPulse,b.centerPulse) &&
        Detail::ArrayBits(a.boundsMinMCS,b.boundsMinMCS) && Detail::ArrayBits(a.boundsMaxMCS,b.boundsMaxMCS) &&
        Detail::Bits(a.radiusMM,b.radiusMM) && Detail::Bits(a.radiusPulse,b.radiusPulse) &&
        Detail::Bits(a.startAngle,b.startAngle) && Detail::Bits(a.sweepRadians,b.sweepRadians) &&
        Detail::Bits(a.lengthMM,b.lengthMM) && Detail::Bits(a.lengthPulse,b.lengthPulse) &&
        a.axisMask==b.axisMask && a.direction==b.direction && a.kind==b.kind &&
        a.fullCircle==b.fullCircle && a.point==b.point && a.valid==b.valid && Detail::Bits(a.sourceRoundoffMM,b.sourceRoundoffMM);
}
inline bool SameScope(const Scope& a,const Scope& b) noexcept
{
    return a.run==b.run && a.cache==b.cache && a.translationGeneration==b.translationGeneration && a.lease.Matches(b.lease);
}
inline bool FrameSupported(const NCTranslationSnapshot& s) noexcept
{
    // Marker-2 admission is a separately qualified native lane. Its original
    // geometry/pulses are already frozen; no authored transform is applied a
    // second time. Legacy retained marker-0/1 frame gates remain unchanged.
    return NCG76::FrameSupported(s);
}
inline const char* StatusName(Status status) noexcept
{
    switch (status)
    {
    case Status::EMPTY:return "EMPTY"; case Status::BUILT:return "BUILT";
    case Status::INVALID_SCOPE:return "INVALID_SCOPE"; case Status::INVALID_HISTORY:return "INVALID_HISTORY";
    case Status::INVALID_NOMINAL:return "INVALID_NOMINAL"; case Status::INVALID_DISTANCE:return "INVALID_DISTANCE";
    case Status::INSUFFICIENT_HISTORY:return "INSUFFICIENT_HISTORY"; case Status::DISCONTINUITY:return "DISCONTINUITY";
    case Status::CAPACITY:return "CAPACITY"; case Status::INVALID_FEED:return "INVALID_FEED";
    default:return "UNKNOWN";
    }
}
inline void Plan::Clear() noexcept
{
    input.scope=Scope{}; input.translation=NCTranslationSnapshot{};
    Detail::ClearHistory(input.history); Detail::ClearNominal(input.nominal);
    input.retreatMm=input.outFeedMmMin=input.returnFeedMmMin=0.0;
    for (auto& leg:legs)
    {
        leg.geometry.Clear(); leg.startU=leg.endU=leg.feedPhysicalMmMin=0.0;
        leg.phase=Phase::Nominal; leg.nominalOrdinal=0U; leg.originalSourceIndex=NoIndex;
    }
    count=reverseCount=0U; availableMm=0.0; status=Status::EMPTY;
}
inline bool BuildG76Nominal(const NCG76::NativePlan& orbit,const Point8& startPulse,
    const Point8& ppu,int pc,int line,NominalSource& out) noexcept
{
    Detail::ClearNominal(out);
    if (!NCG76::ValidNativeBase(orbit) || orbit.program.input.flushMode!=2U || orbit.count!=25U ||
        !FrameSupported(orbit.translation) || pc<0 || line<=0) return false;
    Point8 previous=startPulse;
    for (unsigned a=0U;a<8U;++a)
        if (!Detail::Finite(startPulse[a]) || !Detail::Finite(ppu[a]) ||
            (orbit.translation.axisIdentity.exists[a]!=0U && ppu[a]<=0.0)) return false;
    for (unsigned i=0U;i<orbit.count;++i)
    {
        if (!Detail::CanonicalOrbitRow(orbit,i,startPulse,ppu,previous,out.geometry[i]))
        { Detail::ClearNominal(out); return false; }
        previous=out.geometry[i].endPulse; out.feedMmMin[i]=orbit.feedMmMin[i];
    }
    out.count=orbit.count; out.triggerAfter=12U; out.kind=Kind::G76; out.orbit=orbit; out.pc=pc; out.line=line;
    return true;
}
inline bool Build(const Input& in,Plan& out) noexcept
{
    if (&in==&out.input) { out.Clear(); return false; } // Never retain a stale usable plan on failure.
    out.Clear();
    const auto append=[&](const Leg& leg,unsigned index)->bool
    { if (index>=MaximumLegs) return false; out.legs[index]=leg; return true; };
    unsigned count=0U,reverse=0U; double available=0.0;
    const Status result=Detail::Generate(in,append,available,reverse,count);
    if (result!=Status::BUILT)
    {
        out.Clear(); out.status=result; out.availableMm=available; return false;
    }
    out.input=in; out.count=count; out.reverseCount=reverse; out.availableMm=available; out.status=result;
    return true;
}
inline bool Valid(const Plan& plan) noexcept
{
    if (plan.status!=Status::BUILT || plan.count==0U || plan.count>MaximumLegs) return false;
    const auto compare=[&](const Leg& leg,unsigned index)->bool
    { return index<plan.count && Detail::SameLeg(leg,plan.legs[index]); };
    unsigned count=0U,reverse=0U; double available=0.0;
    if (Detail::Generate(plan.input,compare,available,reverse,count)!=Status::BUILT ||
        count!=plan.count || reverse!=plan.reverseCount || !Detail::Bits(available,plan.availableMm)) return false;
    Leg empty{};
    for (unsigned i=count;i<MaximumLegs;++i) if (!Detail::SameLeg(empty,plan.legs[i])) return false;
    return true;
}
inline bool Valid(const Plan& plan,Plan& rebuildScratch) noexcept
{
    // Streaming validation needs no full-plan scratch. Keep this overload for
    // producers already owning startup-allocated revalidation workspace.
    (void)rebuildScratch; return Valid(plan);
}
inline void RollingHistory::Clear() noexcept
{ Detail::ClearHistory(m_rows); m_fault=Status::EMPTY; }
inline const HistoryRow* RollingHistory::Get(unsigned index) const noexcept
{ return m_fault==Status::EMPTY && index<m_rows.count?&m_rows.rows[index]:nullptr; }
inline bool RollingHistory::Snapshot(HistorySnapshot& out) const noexcept
{
    if (m_fault!=Status::EMPTY) { Detail::ClearHistory(out); return false; }
    out=m_rows; return true;
}
inline bool RollingHistory::Append(const HistoryRow& row,unsigned mask,const Point8& ppu) noexcept
{
    if (m_fault!=Status::EMPTY) return false;
    Scope scope{}; scope.run=row.run; scope.cache=row.cache; scope.translationGeneration=row.translationGeneration; scope.lease=row.lease;
    if (!Detail::Proof(row,scope) || !Detail::Calibration(mask,ppu) || !Detail::GeometryValid(row.geometry,mask,ppu))
    { m_fault=Status::INVALID_HISTORY; return false; }
    const unsigned count=m_rows.count;
    if (count!=0U)
    {
        const HistoryRow& last=m_rows.rows[count-1U];
        Scope old{}; old.run=last.run; old.cache=last.cache; old.translationGeneration=last.translationGeneration; old.lease=last.lease;
        if (!SameScope(scope,old) || mask!=m_rows.validAxisMask || !Detail::ArrayBits(ppu,m_rows.pulsePerUnit))
        { m_fault=Status::INVALID_SCOPE; return false; }
        if (!Detail::Ordered(last,row) || !AreNCPathCoreRetainedEndpointsConnected(last.geometry,row.geometry,mask,ppu))
        { m_fault=Status::DISCONTINUITY; return false; }
        for (unsigned i=0U;i<count;++i) if (m_rows.rows[i].identity.segmentId==row.identity.segmentId)
        { m_fault=Status::INVALID_HISTORY; return false; }
    }
    if (count==HistoryCapacity)
        for (unsigned i=1U;i<HistoryCapacity;++i) m_rows.rows[i-1U]=m_rows.rows[i];
    else ++m_rows.count;
    m_rows.rows[m_rows.count-1U]=row; m_rows.validAxisMask=mask; m_rows.pulsePerUnit=ppu;
    return true;
}
} // namespace NCG76Mixed
