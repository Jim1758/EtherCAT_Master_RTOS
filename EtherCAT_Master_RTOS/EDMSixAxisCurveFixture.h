#pragma once
#include "EDMSixAxisCommonCurve.h"
#include <limits>
#include <algorithm>
#include <cmath>

// EDM112: bounded NC-thread numerical evidence. No fixture value grants motion,
// discharge, ownership, completed-path authority or a physical ARC permission.
namespace EDM112
{
constexpr unsigned ExpectedFixtures = 37U;
constexpr unsigned ExpectedCompletedLines = 16U;
constexpr unsigned CompletedLineSamples = 7U;
constexpr unsigned ExpectedLineSamples = 7U;
constexpr unsigned ExpectedArcSamplesPerDirection = 72U;
constexpr unsigned ExpectedHelixRotaryCases = 24U;
constexpr unsigned ExpectedReverseSpans = 30U;
constexpr unsigned ExpectedB2Plans = 3U;
constexpr unsigned ExpectedEndpointChecks = 122U;
constexpr unsigned ExpectedDerivativeChecks = 259U;
constexpr unsigned ExpectedBoundsChecks = 25U;
constexpr unsigned ExpectedNegativeCases = 9U;

enum class Reason : unsigned
{ None, Build, Evaluate, Endpoint, Derivative, Bounds, Span, Plan, Negative, CompletedLine, Sequence };
inline const char* ReasonName(Reason reason) noexcept
{
    switch (reason) {
    case Reason::None: return "NONE"; case Reason::Build: return "BUILD";
    case Reason::Evaluate: return "EVALUATE"; case Reason::Endpoint: return "ENDPOINT";
    case Reason::Derivative: return "DERIVATIVE"; case Reason::Bounds: return "BOUNDS";
    case Reason::Span: return "SPAN"; case Reason::Plan: return "PLAN";
    case Reason::Negative: return "NEGATIVE"; case Reason::CompletedLine: return "COMPLETED_LINE";
    case Reason::Sequence: return "SEQUENCE";
    } return "INVALID";
}
struct Result
{
    unsigned cases = 0U, lineSamples = 0U, cwSamples = 0U, ccwSamples = 0U, planeMask = 0U;
    unsigned helixRotaryCases = 0U, reverseSpans = 0U, b2Plans = 0U, endpointChecks = 0U;
    unsigned derivativeChecks = 0U, boundsChecks = 0U, negativeCases = 0U;
    unsigned completedLines = 0U, completedLineSamples = 0U, failedCase = 0U;
    Reason reason = Reason::None;
    bool pass = false;
    bool physicalPermit = false;
};
struct Workspace
{
    Curve current{}, previous{}, scratch{};
    Span span{};
    B2Plan plan{};
    Bounds bounds{};
    EDM109::Line line{};
    Point start{}, end{}, scale{}, position{}, velocity{}, first{}, second{};
    Point expected{}, expectedFirst{}, expectedSecond{}, legacyPosition{}, legacyVelocity{};
    Point low{}, high{}, seam{};
};
inline bool IsFixtureResultCurrent(const Result& r) noexcept
{
    return r.pass && !r.physicalPermit && r.reason == Reason::None && r.failedCase == 0U &&
        r.cases == ExpectedFixtures && r.lineSamples == ExpectedLineSamples &&
        r.cwSamples == ExpectedArcSamplesPerDirection && r.ccwSamples == ExpectedArcSamplesPerDirection &&
        r.planeMask == 7U && r.helixRotaryCases == ExpectedHelixRotaryCases &&
        r.reverseSpans == ExpectedReverseSpans && r.b2Plans == ExpectedB2Plans &&
        r.endpointChecks == ExpectedEndpointChecks && r.derivativeChecks == ExpectedDerivativeChecks &&
        r.boundsChecks == ExpectedBoundsChecks && r.negativeCases == ExpectedNegativeCases &&
        r.completedLines <= ExpectedCompletedLines &&
        r.completedLineSamples == r.completedLines * CompletedLineSamples;
}
namespace FixtureDetail
{
constexpr double Pi = 3.141592653589793238462643383279502884;
inline bool Fail(Result& r, Reason reason) noexcept
{ r.reason = reason; r.failedCase = r.cases ? r.cases : 1U; return false; }
inline bool Near(double a, double b) noexcept
{
    return std::isfinite(a) && std::isfinite(b) &&
        std::fabs(a - b) <= 256.0 * (std::numeric_limits<double>::epsilon)() *
        (1.0 + (std::max)(std::fabs(a), std::fabs(b)));
}
inline bool NearPoint(const Point& a, const Point& b) noexcept
{ for (unsigned i=0U;i<6U;++i) if (!Near(a[i],b[i])) return false; return true; }
inline void Axes(unsigned plane, unsigned& u, unsigned& v, unsigned& normal) noexcept
{
    u = plane == 17U ? 0U : plane == 18U ? 2U : 1U;
    v = plane == 17U ? 1U : plane == 18U ? 0U : 2U;
    normal = plane == 17U ? 2U : plane == 18U ? 1U : 0U;
}
// Independent native-space trigonometry, deliberately not a call to any new
// production evaluator. The expected circle has radius 2 and authored angle.
inline void KnownArc(const Point& start, const Point& end, const Point& scale,
    unsigned plane, double startAngle, double sweep, double u,
    Point& p, Point& d, Point& dd) noexcept
{
    for (unsigned i=0U;i<6U;++i) {
        const double delta=end[i]-start[i]; p[i]=start[i]+delta*u; d[i]=delta; dd[i]=0.0;
    }
    unsigned a=0U,b=0U,n=0U; Axes(plane,a,b,n);
    const double angle=startAngle+sweep*u;
    p[a]=start[a]+2.0*(std::cos(angle)-std::cos(startAngle))*scale[a];
    p[b]=start[b]+2.0*(std::sin(angle)-std::sin(startAngle))*scale[b];
    d[a]=-2.0*std::sin(angle)*sweep*scale[a];
    d[b]= 2.0*std::cos(angle)*sweep*scale[b];
    dd[a]=-2.0*std::cos(angle)*sweep*sweep*scale[a];
    dd[b]=-2.0*std::sin(angle)*sweep*sweep*scale[b];
    if (u==0.0) p=start; else if(u==1.0) p=end;
}
inline void ScaleNative(Point& point, const Point& scale) noexcept
{ for(unsigned i=0U;i<6U;++i) point[i]*=scale[i]; }
inline bool CompareLine(const EDM109::Line& line, Workspace& w) noexcept
{
    if (!BuildLine(line,w.scratch)) return false;
    constexpr double samples[CompletedLineSamples]={0.0,0.125,0.25,0.5,0.75,0.875,1.0};
    for (unsigned j=0U;j<CompletedLineSamples;++j) {
        const double u=samples[j], s=u*line.lengthMm;
        const double speed=(j&1U) ? -0.001 : 0.001;
        if (!EvaluateMm(w.scratch,s,speed,w.position,w.velocity) ||
            !EDM109::EvaluateMm(line,s,w.legacyPosition) ||
            !EDM109::SamePulse(w.position,w.legacyPosition)) return false;
        for(unsigned i=0U;i<6U;++i)
            w.expectedFirst[i]=(line.endPulse[i]-line.startPulse[i])*speed/line.lengthMm;
        if(!NearPoint(w.velocity,w.expectedFirst)) return false;
        const double q=u*line.pulseLength, v=speed*line.pulseLength/line.lengthMm;
        if(!EvaluateLineScalar(w.scratch,q,v,w.position,w.velocity) ||
            !EDM109::EvaluateScalar(line,q,v,w.legacyPosition,w.legacyVelocity) ||
            !EDM109::SamePulse(w.position,w.legacyPosition) ||
            !EDM109::SamePulse(w.velocity,w.legacyVelocity)) return false;
    }
    return true;
}
inline bool MixedPlan(unsigned which, Workspace& w, Result& r) noexcept
{
    w.start={{-1.0,0.0,0.0,710.0,-400.0,1060.0}};
    w.end={{2.0,0.0,0.0,725.0,-410.0,1090.0}};
    ScaleNative(w.start,w.scale);ScaleNative(w.end,w.scale);
    if(which==0U) {
        if(!EDM109::BuildLine(w.start,w.end,w.scale,w.line)||!BuildLine(w.line,w.previous)) return Fail(r,Reason::Build);
    } else {
        w.start=w.end;w.start[0]=0.0;w.start[1]=(which==1U?2.0:-2.0)*w.scale[1];
        if(!BuildArc(w.start,w.end,w.scale,17U,{{0.0,which==1U?-2.0:2.0}},which==1U?-1:1,false,w.previous)) return Fail(r,Reason::Build);
    }
    w.start=w.end;w.end[2]+=0.5*w.scale[2];w.end[3]+=30.0*w.scale[3];
    w.end[4]-=20.0*w.scale[4];w.end[5]+=400.0*w.scale[5];
    if(which==1U) {
        w.end[0]+=w.scale[0];w.end[1]+=w.scale[1];
        if(!EDM109::BuildLine(w.start,w.end,w.scale,w.line)||!BuildLine(w.line,w.current)) return Fail(r,Reason::Build);
    } else {
        w.end[0]=0.0;w.end[1]=(which==0U?2.0:-2.0)*w.scale[1];
        if(!BuildArc(w.start,w.end,w.scale,17U,{{-2.0,0.0}},which==0U?1:-1,false,w.current)) return Fail(r,Reason::Build);
    }
    const double anchor=0.625*w.current.lengthMm, fraction=0.25;
    if(!BuildB2Plan(w.current,w.previous,anchor,fraction,w.plan)||!w.plan.valid||w.plan.count!=4U||
        !EDM109::SamePulse(w.plan.anchorPulse,w.plan.returnedPulse)) return Fail(r,Reason::Plan);
    for(unsigned part=0U;part<4U;++part) {
        const auto& span=w.plan.spans[part];const auto& original=(part==0U||part==3U)?w.current:w.previous;
        const double from=part==0U?anchor:part==1U?w.previous.lengthMm:part==2U?w.previous.lengthMm*(1.0-fraction):0.0;
        const double to=part==0U?0.0:part==1U?w.previous.lengthMm*(1.0-fraction):part==2U?w.previous.lengthMm:anchor;
        if(!span.valid||!span.stopRequired||span.startMm!=from||span.endMm!=to||
            !Near(span.lengthMm,std::fabs(to-from))||span.curve.kind!=original.kind) return Fail(r,Reason::Span);
        if(to<from)++r.reverseSpans;
        for(unsigned sample=0U;sample<3U;++sample) {
            const double t=sample*0.5, station=from+(to-from)*t, unit=station/original.lengthMm;
            if(!EvaluateSpan(span,t*span.lengthMm,0.001,w.position,w.velocity))return Fail(r,Reason::Evaluate);
            if(original.kind==Kind::Line) {
                for(unsigned i=0U;i<6U;++i) {
                    w.expected[i]=original.line.startPulse[i]+(original.line.endPulse[i]-original.line.startPulse[i])*unit;
                    w.expectedFirst[i]=original.line.endPulse[i]-original.line.startPulse[i];
                }
                if(unit==0.0)w.expected=original.line.startPulse;else if(unit==1.0)w.expected=original.line.endPulse;
            } else {
                const double angle=(&original==&w.previous)?(which==1U?Pi*0.5:-Pi*0.5):0.0;
                const int dir=(&original==&w.previous)?(which==1U?-1:1):(which==0U?1:-1);
                KnownArc(original.line.startPulse,original.line.endPulse,w.scale,17U,angle,dir*Pi*0.5,unit,w.expected,w.expectedFirst,w.expectedSecond);
            }
            for(unsigned i=0U;i<6U;++i)w.expectedFirst[i]*=(to<from?-0.001:0.001)/original.lengthMm;
            if(!NearPoint(w.position,w.expected))return Fail(r,Reason::Evaluate);
            if(!NearPoint(w.velocity,w.expectedFirst))return Fail(r,Reason::Derivative);
            ++r.derivativeChecks;
            if(sample==0U||sample==2U) {
                ++r.endpointChecks;
                if(sample==0U&&part&& !EDM109::SamePulse(w.position,w.seam))return Fail(r,Reason::Endpoint);
                if(sample==2U)w.seam=w.position;
            }
        }
    }
    if(!EDM109::SamePulse(w.seam,w.plan.anchorPulse))return Fail(r,Reason::Endpoint);
    ++r.b2Plans;return true;
}
}

inline bool RunFixture(Workspace& w, Result& r) noexcept
{
    r=Result{};
    w.scale={{1250.0,2300.0,3500.0,256.0,512.0,1024.0}};
    ++r.cases;
    w.start={{123.0,-234.0,345.0,185600.0,-209920.0,1116160.0}};
    w.end=w.start;for(unsigned i=0U;i<6U;++i)w.end[i]+=(i&1U?-1.0:1.0)*w.scale[i]*(i<3U?0.002:0.04);
    if(!EDM109::BuildLine(w.start,w.end,w.scale,w.line)||!FixtureDetail::CompareLine(w.line,w))return FixtureDetail::Fail(r,Reason::Build);
    r.lineSamples=ExpectedLineSamples;r.derivativeChecks=ExpectedLineSamples;r.endpointChecks=2U;
    constexpr double samples[6]={0.0,0.125,0.25,0.5,0.75,1.0};
    for(unsigned plane=17U;plane<=19U;++plane)for(int direction=-1;direction<=1;direction+=2)for(unsigned turn=1U;turn<=4U;++turn) {
        ++r.cases;unsigned a=0U,b=0U,n=0U;FixtureDetail::Axes(plane,a,b,n);
        w.start={{4.0,-3.0,2.0,725.0,-410.0,1090.0}};w.end=w.start;
        w.end[a]+=turn==1U||turn==3U?-2.0:turn==2U?-4.0:0.0;
        w.end[b]+=turn==1U?direction*2.0:turn==3U?-direction*2.0:0.0;
        w.end[n]+=1.0;w.end[3]+=15.0;w.end[4]-=25.0;w.end[5]+=370.0;
        FixtureDetail::ScaleNative(w.start,w.scale);FixtureDetail::ScaleNative(w.end,w.scale);
        if(!BuildArc(w.start,w.end,w.scale,plane,{{-2.0,0.0}},direction,turn==4U,w.current))return FixtureDetail::Fail(r,Reason::Build);
        const double sweep=static_cast<double>(direction)*static_cast<double>(turn)*FixtureDetail::Pi*0.5;
        const double length=std::hypot(2.0*std::fabs(sweep),1.0);
        if(!FixtureDetail::Near(w.current.lengthMm,length)||!FixtureDetail::Near(w.current.sweepRadians,sweep))return FixtureDetail::Fail(r,Reason::Build);
        for(unsigned j=0U;j<6U;++j) {
            const double u=samples[j],speed=(j&1U)?-0.001:0.001;
            FixtureDetail::KnownArc(w.start,w.end,w.scale,plane,0.0,sweep,u,w.expected,w.expectedFirst,w.expectedSecond);
            if(!EvaluateUnit(w.current,u,w.position,&w.first,&w.second)||!FixtureDetail::NearPoint(w.position,w.expected))return FixtureDetail::Fail(r,Reason::Evaluate);
            if(!FixtureDetail::NearPoint(w.first,w.expectedFirst)||!FixtureDetail::NearPoint(w.second,w.expectedSecond))return FixtureDetail::Fail(r,Reason::Derivative);
            if((j==0U||j==5U)&&!EDM109::SamePulse(w.position,j==0U?w.start:w.end))return FixtureDetail::Fail(r,Reason::Endpoint);
            if(j==0U||j==5U)++r.endpointChecks;
            if(!EvaluateMm(w.current,u*length,speed,w.position,w.velocity))return FixtureDetail::Fail(r,Reason::Evaluate);
            for(unsigned i=0U;i<6U;++i)w.expectedFirst[i]*=speed/length;
            if(!FixtureDetail::NearPoint(w.position,w.expected)||!FixtureDetail::NearPoint(w.velocity,w.expectedFirst))return FixtureDetail::Fail(r,Reason::Derivative);
            ++r.derivativeChecks;if(direction<0)++r.cwSamples;else ++r.ccwSamples;
        }
        if(!WholeCurveBounds(w.current,w.bounds))return FixtureDetail::Fail(r,Reason::Bounds);
        for(unsigned i=0U;i<6U;++i) {
            double low=(std::min)(w.start[i],w.end[i]),high=(std::max)(w.start[i],w.end[i]);
            if(i==a){low=w.start[i]-(turn==1U?2.0:4.0)*w.scale[i];high=w.start[i];}
            if(i==b){low=w.start[i]+(direction<0||turn>=3U?-2.0:0.0)*w.scale[i];high=w.start[i]+(direction>0||turn>=3U?2.0:0.0)*w.scale[i];}
            // The production envelope deliberately expands circular extrema
            // by its source precision budget. Require analytical containment
            // and bound that expansion; a zero extremum needs an absolute
            // allowance derived from the scale, not a relative-zero test.
            const double arithmetic = 256.0*(std::numeric_limits<double>::epsilon)()*
                (1.0+(std::max)(std::fabs(low),std::fabs(high)));
            const double allowance=(i==a||i==b?w.current.precisionBudgetMm*w.scale[i]:0.0)+arithmetic;
            if(w.bounds.minPulse[i]>low||w.bounds.maxPulse[i]<high||
                low-w.bounds.minPulse[i]>allowance||w.bounds.maxPulse[i]-high>allowance)return FixtureDetail::Fail(r,Reason::Bounds);
            if(!EDM109::SameDouble(w.bounds.minNative[i],w.bounds.minPulse[i]/w.scale[i])||
                !EDM109::SameDouble(w.bounds.maxNative[i],w.bounds.maxPulse[i]/w.scale[i]))return FixtureDetail::Fail(r,Reason::Bounds);
        }
        ++r.boundsChecks;
        if(!BuildSpan(w.current,0.75*length,0.25*length,w.span))return FixtureDetail::Fail(r,Reason::Span);
        for(unsigned j=0U;j<3U;++j) {
            const double u=0.75-0.25*j;
            FixtureDetail::KnownArc(w.start,w.end,w.scale,plane,0.0,sweep,u,w.expected,w.expectedFirst,w.expectedSecond);
            if(!EvaluateSpan(w.span,0.5*j*w.span.lengthMm,0.001,w.position,w.velocity))return FixtureDetail::Fail(r,Reason::Span);
            for(unsigned i=0U;i<6U;++i)w.expectedFirst[i]*=-0.001/length;
            if(!FixtureDetail::NearPoint(w.position,w.expected)||!FixtureDetail::NearPoint(w.velocity,w.expectedFirst))return FixtureDetail::Fail(r,Reason::Span);
            ++r.derivativeChecks;if(j==0U||j==2U)++r.endpointChecks;
        }
        ++r.reverseSpans;++r.helixRotaryCases;r.planeMask|=1U<<(plane-17U);
    }
    for(unsigned which=0U;which<3U;++which){++r.cases;if(!FixtureDetail::MixedPlan(which,w,r))return false;}
    // Eight rejected builders. Each starts from a previously valid output so
    // a failure must invalidate it, preventing stale-geometry reuse.
    w.start={{2500.0,0.0,0.0,185600.0,-209920.0,1116160.0}};
    w.end=w.start;w.end[0]=0.0;w.end[1]=4600.0;
    for(unsigned bad=0U;bad<8U;++bad) {
        ++r.cases;w.low=w.scale;w.high=w.end;int direction=1;unsigned plane=17U;bool full=false;
        std::array<double,2> center{{-2.0,0.0}};
        if(bad==0U)direction=0;
        if(bad==1U)w.low[2]=0.0;
        if(bad==2U)w.high[1]=6900.0;
        if(bad==3U)full=true;
        if(bad==4U)w.high[0]=(std::numeric_limits<double>::quiet_NaN)();
        if(bad==5U)center={{0.0,0.0}};
        if(bad==6U)plane=20U;
        if(bad==7U)w.low[4]=(std::numeric_limits<double>::infinity)();
        w.scratch=w.current;
        if(BuildArc(w.start,w.high,w.low,plane,center,direction,full,w.scratch)||w.scratch.valid)return FixtureDetail::Fail(r,Reason::Negative);
        ++r.negativeCases;
    }
    ++r.cases;
    w.end=w.start;w.end[0]=-w.start[0];
    if(!BuildArc(w.start,w.end,w.scale,17U,{{-2.0,0.0}},1,false,w.current)||!WholeCurveBounds(w.current,w.bounds))return FixtureDetail::Fail(r,Reason::Build);
    w.low=w.bounds.minNative;w.high=w.bounds.maxNative;w.high[1]=1.0;
    if(BoundsWithin(w.bounds,w.low,w.high))return FixtureDetail::Fail(r,Reason::Negative);
    ++r.boundsChecks;++r.negativeCases;
    r.pass=true;
    if(!IsFixtureResultCurrent(r))return FixtureDetail::Fail(r,Reason::Sequence);
    return true;
}
inline bool CompareCompletedLine(const EDM109::Line& line,Workspace& w,Result& r) noexcept
{
    if(!IsFixtureResultCurrent(r)||r.completedLines>=ExpectedCompletedLines)return FixtureDetail::Fail(r,Reason::Sequence);
    if(!FixtureDetail::CompareLine(line,w))return FixtureDetail::Fail(r,Reason::CompletedLine);
    ++r.completedLines;r.completedLineSamples+=CompletedLineSamples;
    return true;
}
static_assert(sizeof(Result)<=96U,"EDM112 fixture result must remain bounded");
static_assert(sizeof(Workspace)<=4096U,"EDM112 workspace belongs to persistent NC storage");
}
