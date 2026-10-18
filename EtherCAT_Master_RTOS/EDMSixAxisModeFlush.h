#pragma once
#include "EDMSixAxisCurveB2Excursion.h"
#include "EDMSixAxisGapFlushPath.h"

// EDM116: an explicit, frozen mode selection. This pure value owns neither
// output authority nor physical settlement; Motion supplies the latter.
namespace EDM116
{
using Point = EDM114::Point;
using Source = EDM109::Source;
using CompletionToken = EDM115::CompletionToken;
using Feedback = EDM115::Feedback;
using Phase = EDM115::Phase;
using Reason = EDM115::Reason;
constexpr unsigned ExpectedEvents = 8U, ExpectedDwells = 28U;
constexpr unsigned RequiredSettledCycles = EDM115::RequiredSettledCycles;
constexpr double SampleSeconds = EDM114::SampleSeconds;
constexpr std::uint64_t LocalDeadlineUs = 3000000ULL, LocalDeadlineTicks = 12000ULL;
enum class Mode : std::uint8_t { B0 = 0U, B1 = 1U, B2 = 2U, Invalid = 255U };
// EDM124 opts in to one additional authored arc pair at rows 4/5. Earlier
// fixtures retain their original geometry, B modes and token checkpoints.
enum class Fixture : std::uint8_t
{ Canonical116 = 0U, Complement117 = 1U, ArcPair124 = 2U, ArcPairComplement124 = 3U,
  Modal127 = 4U, ModalComplement127 = 5U, ModalZ149 = 6U, ModalXYZ150 = 7U, ModalSix151 = 8U, ModalArc152 = 9U, ModalOrder153 = 10U, ModalSparse154 = 11U, ModalSparseArc155 = 12U, ModalProgram156 = 13U, ModalPause157 = 14U, ModalVisibleFlush158 = 15U, ModalXYZVisibleFlush159 = 16U, ModalSixVisibleFlush167 = 17U, ModalG76Gap173 = 18U };
constexpr unsigned G76GapRows173 = 25U;
inline unsigned PolicyRows(Fixture fixture) noexcept
{ return fixture == Fixture::ModalG76Gap173 ? G76GapRows173 : EDM114::RequiredRows; }
inline bool IsModalFixture(Fixture fixture) noexcept
{ return fixture == Fixture::Modal127 || fixture == Fixture::ModalComplement127 || fixture == Fixture::ModalZ149 || fixture == Fixture::ModalXYZ150 || fixture == Fixture::ModalSix151 || fixture == Fixture::ModalArc152 || fixture == Fixture::ModalOrder153 || fixture == Fixture::ModalSparse154 || fixture == Fixture::ModalSparseArc155 || fixture == Fixture::ModalProgram156 || fixture == Fixture::ModalPause157 || fixture == Fixture::ModalVisibleFlush158 || fixture == Fixture::ModalXYZVisibleFlush159 || fixture == Fixture::ModalSixVisibleFlush167 || fixture == Fixture::ModalG76Gap173; }
inline bool IsMotionTestFixture(Fixture fixture) noexcept
{ return fixture == Fixture::ModalZ149 || fixture == Fixture::ModalXYZ150 || fixture == Fixture::ModalSix151 || fixture == Fixture::ModalArc152 || fixture == Fixture::ModalOrder153 || fixture == Fixture::ModalSparse154 || fixture == Fixture::ModalSparseArc155 || fixture == Fixture::ModalProgram156 || fixture == Fixture::ModalPause157 || fixture == Fixture::ModalVisibleFlush158 || fixture == Fixture::ModalXYZVisibleFlush159 || fixture == Fixture::ModalSixVisibleFlush167 || fixture == Fixture::ModalG76Gap173; }
inline bool IsAuthoredOrderFixture(Fixture fixture) noexcept
{ return fixture == Fixture::ModalOrder153 || fixture == Fixture::ModalSparse154 || fixture == Fixture::ModalSparseArc155 || fixture == Fixture::ModalProgram156 || fixture == Fixture::ModalPause157 || fixture == Fixture::ModalVisibleFlush158 || fixture == Fixture::ModalXYZVisibleFlush159 || fixture == Fixture::ModalSixVisibleFlush167; }
inline bool IsBoundedArcFixture(Fixture fixture) noexcept
{ return fixture == Fixture::ModalArc152 || IsAuthoredOrderFixture(fixture); }
inline bool ValidFixture(Fixture fixture) noexcept
{ return fixture == Fixture::Canonical116 || fixture == Fixture::Complement117 ||
    fixture == Fixture::ArcPair124 || fixture == Fixture::ArcPairComplement124 || IsModalFixture(fixture); }
inline bool IsArcPairFixture(Fixture fixture) noexcept
{ return fixture == Fixture::ArcPair124 || fixture == Fixture::ArcPairComplement124 ||
    (IsModalFixture(fixture) && fixture != Fixture::ModalG76Gap173); }
inline bool IsComplementFixture(Fixture fixture) noexcept
{ return fixture == Fixture::Complement117 || fixture == Fixture::ArcPairComplement124 ||
    fixture == Fixture::ModalComplement127; }
inline bool IsArcStep(unsigned step, Fixture fixture = Fixture::Canonical116, int authoredCode = 0) noexcept
{
    if (fixture == Fixture::ModalG76Gap173) return false;
    if (IsAuthoredOrderFixture(fixture)) return step < EDM114::RequiredRows && (authoredCode == 2 || authoredCode == 3);
    return ValidFixture(fixture) && fixture != Fixture::ModalZ149 && (EDM114::IsArcStep(step) ||
        (IsArcPairFixture(fixture) && (step == 4U || step == 5U)));
}
inline int ExpectedGCode(unsigned step, Fixture fixture = Fixture::Canonical116, int authoredCode = 0) noexcept
{
    if (!ValidFixture(fixture)) return 0;
    if (fixture == Fixture::ModalG76Gap173) return step < G76GapRows173 ? 1 : 0;
    // An authored-order row has no implicit fallback to a historical fixture.
    if (IsAuthoredOrderFixture(fixture)) return step < EDM114::RequiredRows &&
        authoredCode >= 1 && authoredCode <= 3 ? authoredCode : 0;
    if (fixture == Fixture::ModalZ149) return step < EDM114::RequiredRows ? 1 : 0;
    const int code = IsArcPairFixture(fixture) && (step == 4U || step == 5U) ?
        ((step & 1U) ? 3 : 2) : EDM114::ExpectedGCode(step);
    return IsComplementFixture(fixture) && IsArcStep(step,fixture) ? 5 - code : code;
}
inline bool ValidArcRecipe(int code, unsigned step, const Point& increment,
    const std::array<double, 2U>& center, double feed, Fixture fixture = Fixture::Canonical116) noexcept
{
    if (IsBoundedArcFixture(fixture)) {
        EDM112::Curve curve{};
        return IsArcStep(step,fixture,code) && code == ExpectedGCode(step,fixture,code) &&
            EDM114::BuildBoundedXYArc(code,increment,center,feed,curve);
    }
    const unsigned recipeStep = IsArcPairFixture(fixture) && (step == 4U || step == 5U) ? step - 2U : step;
    return ValidFixture(fixture) && IsArcStep(step,fixture) && code == ExpectedGCode(step, fixture) &&
        EDM114::ValidArcRecipe(EDM114::ExpectedGCode(recipeStep), recipeStep, increment, center, feed);
}
inline unsigned ExpectedArcFlushRows(unsigned completed, Fixture fixture = Fixture::Canonical116) noexcept
{
    if (!ValidFixture(fixture) || completed > EDM114::RequiredRows || fixture == Fixture::ModalZ149 ||
        fixture == Fixture::ModalG76Gap173) return 0U;
    return (completed + 1U) / 4U + (IsArcPairFixture(fixture) && completed >= 5U ? 1U : 0U);
}
inline bool ValidMode(Mode mode) noexcept { return mode == Mode::B0 || mode == Mode::B1 || mode == Mode::B2; }
inline const char* ModeName(Mode mode) noexcept
{ return mode == Mode::B0 ? "B0" : mode == Mode::B1 ? "B1" : mode == Mode::B2 ? "B2" : "INVALID"; }
inline const char* PhaseName(Phase phase) noexcept { return EDM115::PhaseName(phase); }
inline bool ParseMode(double value, Mode& mode) noexcept
{
    if (!std::isfinite(value) || value < 0.0 || value > 2.0 || std::floor(value) != value) return false;
    mode = static_cast<Mode>(static_cast<unsigned>(value)); return true;
}
inline Mode ExpectedMode(unsigned step) noexcept
{
    const Mode modes[8U] = { Mode::B0, Mode::B2, Mode::B2, Mode::B0, Mode::B1, Mode::B1, Mode::B0, Mode::B1 };
    return step < EDM114::RequiredRows ? modes[step / 2U] : Mode::Invalid;
}
inline bool IsTriggerStep(unsigned step) noexcept { return step < EDM114::RequiredRows && (step & 1U) == 0U; }
inline unsigned EventOrdinal(unsigned step) noexcept { return IsTriggerStep(step) ? step / 2U + 1U : 0U; }
inline unsigned DwellCount(Mode mode) noexcept { return mode == Mode::B2 ? 5U : ValidMode(mode) ? 3U : 0U; }
inline std::uint64_t DeadlineUs(Mode mode) noexcept { return mode == Mode::B2 ? EDM115::MaximumDurationUs : ValidMode(mode) ? LocalDeadlineUs : 0ULL; }
struct Policy
{
    Mode mode = Mode::Invalid;
    Fixture fixture = Fixture::Canonical116;
    EDM107::FlushProfile profile{};
    int selectorPC = -1;
    std::uint64_t selectorDispatch = 0ULL, revision = 0ULL;
    // EDM127: the modal origin survives omitted B words. selectorPC/Dispatch
    // bind this immutable policy to its own motion row; they are never a
    // fabricated B dispatch. Legacy explicit-P131 policies keep origin empty.
    int modeOriginPC = -1;
    std::uint64_t modeOriginDispatch = 0ULL, modeOriginRevision = 0ULL;
};
inline bool ValidPolicy(const Policy& policy) noexcept
{
    if ((policy.fixture == Fixture::ModalVisibleFlush158 && (policy.mode != Mode::B1 || policy.revision != 1ULL)) ||
        ((policy.fixture == Fixture::ModalXYZVisibleFlush159 || policy.fixture == Fixture::ModalSixVisibleFlush167) &&
            (policy.mode != Mode::B1 || policy.revision > 3ULL)) ||
        (policy.fixture == Fixture::ModalG76Gap173 && policy.mode != Mode::B1) ||
        !ValidMode(policy.mode) || !ValidFixture(policy.fixture) || !EDM107::ValidFlushProfile(policy.profile) || policy.selectorPC < 0 ||
        !policy.selectorDispatch || policy.revision < 1ULL || policy.revision > PolicyRows(policy.fixture)) return false;
    if (!IsModalFixture(policy.fixture)) return policy.modeOriginPC == -1 && !policy.modeOriginDispatch && !policy.modeOriginRevision;
    return policy.modeOriginPC >= 0 && policy.modeOriginDispatch && policy.modeOriginRevision &&
        policy.modeOriginPC <= policy.selectorPC && policy.modeOriginDispatch <= policy.selectorDispatch &&
        ((policy.modeOriginPC == policy.selectorPC) == (policy.modeOriginDispatch == policy.selectorDispatch));
}
inline bool SamePolicy(const Policy& a, const Policy& b) noexcept
{
    return a.mode == b.mode && a.fixture == b.fixture && EDM107::SameFlushProfile(a.profile,b.profile) && a.selectorPC == b.selectorPC &&
        a.selectorDispatch == b.selectorDispatch && a.revision == b.revision &&
        a.modeOriginPC == b.modeOriginPC && a.modeOriginDispatch == b.modeOriginDispatch && a.modeOriginRevision == b.modeOriginRevision;
}
inline bool PolicyMatchesBinding(const Policy& policy, unsigned step, int sourcePC, std::uint64_t dispatch) noexcept
{
    return ValidPolicy(policy) && step < PolicyRows(policy.fixture) && sourcePC >= 0 && dispatch &&
        policy.revision == static_cast<std::uint64_t>(step)+1ULL &&
        (IsModalFixture(policy.fixture) ? (policy.selectorPC == sourcePC && policy.selectorDispatch == dispatch) :
            (policy.selectorPC < sourcePC && policy.selectorDispatch < dispatch));
}
inline bool PolicyMatchesSource(const Policy& policy, const Source& source) noexcept
{
    const bool validSource = policy.fixture == Fixture::ModalG76Gap173 ?
        (EDM109::ValidScope(source.scope) && source.dispatch && source.epoch &&
            source.epoch <= (std::numeric_limits<std::uint32_t>::max)() && source.segment &&
            source.sourcePC >= 0 && source.step < G76GapRows173) : EDM114::ValidSource(source);
    return validSource && PolicyMatchesBinding(policy,source.step,source.sourcePC,source.dispatch);
}
inline bool PolicyFollowsCompleted(const Policy& policy, const Policy& previous, const Source& source) noexcept
{
    // A compound G76 retains one real NC dispatch. Only its canonical child
    // ordinal advances; neither the selector nor its modal origin is invented.
    if (policy.fixture == Fixture::ModalG76Gap173)
        return ValidPolicy(policy) && PolicyMatchesSource(previous,source) && policy.fixture == previous.fixture &&
            policy.revision == previous.revision+1ULL && policy.selectorPC == source.sourcePC &&
            policy.selectorDispatch == source.dispatch && policy.modeOriginPC == previous.modeOriginPC &&
            policy.modeOriginDispatch == previous.modeOriginDispatch && policy.modeOriginRevision == previous.modeOriginRevision &&
            EDM107::SameFlushProfile(policy.profile,previous.profile);
    if (!ValidPolicy(policy) || !PolicyMatchesSource(previous,source) || policy.fixture != previous.fixture ||
        policy.revision != previous.revision+1ULL || policy.selectorPC <= source.sourcePC ||
        policy.selectorDispatch <= source.dispatch) return false;
    if (!IsModalFixture(policy.fixture)) return true;
    const bool sameOrigin = policy.modeOriginPC == previous.modeOriginPC &&
        policy.modeOriginDispatch == previous.modeOriginDispatch && policy.modeOriginRevision == previous.modeOriginRevision;
    if (sameOrigin) return policy.mode == previous.mode;
    // A genuine later B block may be standalone or inline. More than one
    // standalone selection between rows advances the origin revision again.
    return policy.modeOriginPC > source.sourcePC && policy.modeOriginDispatch > source.dispatch &&
        policy.modeOriginRevision > previous.modeOriginRevision;
}
inline bool EmptyToken(const CompletionToken& token) noexcept { return EDM115::EmptyToken(token); }
inline bool SameToken(const CompletionToken& a,const CompletionToken& b) noexcept { return EDM115::SameToken(a,b); }
inline bool ValidToken(const CompletionToken& token,unsigned step) noexcept { return EDM115::ValidToken(token,step); }
inline double NativeLength(const EDM114::Mapping& mapping) noexcept
{
    const auto& g=mapping.GetGeometry(); if (!g.valid) return 0.0;
    if (g.kind==EDM114::Kind::Arc) return g.pulseLength/g.pulsePerNative[0];
    if (g.kind!=EDM114::Kind::Line) return 0.0;
    double length=0.0;
    for(unsigned i=0U;i<3U;++i) length=std::hypot(length,(g.endPulse[i]-g.startPulse[i])/g.pulsePerNative[i]);
    return length;
}


// The NC receipt and the pure executor share this exact arithmetic. Static
// axes retain the captured anchor bits; B0 has one frozen six-axis tangent.
inline bool ComputeRetreat(const EDM114::Geometry& g,const Point& anchor,double q,Mode mode,
    double strokeRatio,Point& retreat,double& strokeMm) noexcept
{
    if(!g.valid||!std::isfinite(q)||q<0.0||q>g.pulseLength||!std::isfinite(strokeRatio)||strokeRatio<0.05||strokeRatio>0.1)return false;
    Point tangent{},top=anchor;double nativeLength=0.0;
    if(g.kind==EDM114::Kind::Line)
    {
        tangent=g.ratio;
        for(unsigned i=0U;i<3U;++i)nativeLength=std::hypot(nativeLength,(g.endPulse[i]-g.startPulse[i])/g.pulsePerNative[i]);
    }
    else if(g.kind==EDM114::Kind::Arc)
    {
        nativeLength=g.pulseLength/g.pulsePerNative[0];
        const double angle=g.startAngle+(q/g.pulseLength)*g.sweepRadians;
        const double sign=g.sweepRadians>0.0?1.0:-1.0;
        tangent[0]=-std::sin(angle)*sign;tangent[1]=std::cos(angle)*sign;
    }
    else return false;
    if(!EDM114::Positive(nativeLength))return false;
    const double stroke=nativeLength*strokeRatio;
    if(mode==Mode::B0)
    {
        double nativeDerivative=0.0;
        for(unsigned i=0U;i<3U;++i)nativeDerivative=std::hypot(nativeDerivative,tangent[i]/g.pulsePerNative[i]);
        if(!EDM114::Positive(nativeDerivative))return false;
        const double travel=stroke/nativeDerivative;
        for(unsigned i=0U;i<6U;++i)if(tangent[i]!=0.0)top[i]=anchor[i]-travel*tangent[i];
    }
    else if(mode==Mode::B1)top[2]=anchor[2]+stroke*g.pulsePerNative[2];
    else return false;
    for(unsigned i=0U;i<6U;++i)if(!std::isfinite(top[i])||!EDM114::Positive(g.pulsePerNative[i]))return false;
    retreat=top;strokeMm=stroke;return true;
}
inline bool ComputeRetreat(const EDM114::Feedback& row,const Point& anchor,double q,Mode mode,
    double ratio,Point& retreat,double& strokeMm) noexcept
{
    EDM114::Geometry g{};g.startPulse=row.startPulse;g.endPulse=row.endPulse;g.pulsePerNative=row.pulsePerNative;
    g.centerPulse=row.centerPulse;g.radiusPulse=row.radiusPulse;g.startAngle=row.startAngle;g.sweepRadians=row.sweepRadians;
    g.pulseLength=row.pulseLength;g.kind=row.kind;g.valid=row.geometryValid;
    if(g.kind==EDM114::Kind::Line)for(unsigned i=0U;i<6U;++i)g.ratio[i]=(g.endPulse[i]-g.startPulse[i])/g.pulseLength;
    return ComputeRetreat(g,anchor,q,mode,ratio,retreat,strokeMm);
}

class LocalExcursion
{
public:
    const Feedback& Current() const noexcept { return feedback_; }
    const EDM114::Mapping& CurrentMapping() const noexcept { return lift_; }
    const EDM114::Mapping& OriginalMapping() const noexcept { return original_; }
    const Source& CurrentSource() const noexcept { return source_; }
    const Point& Anchor() const noexcept { return anchor_; }
    const Point& RetreatPulse() const noexcept { return top_; }
    double AnchorQ() const noexcept { return anchorQ_; }
    double StrokeNativeMm() const noexcept { return strokeMm_; }
    std::uint64_t TopDwellTicks() const noexcept { return dwellTicks_[0]; }
    std::uint64_t TopDwellUs() const noexcept { return dwellUs_[0]; }
    std::uint64_t ReturnDwellTicks() const noexcept { return dwellTicks_[1]; }
    std::uint64_t ReturnDwellUs() const noexcept { return dwellUs_[1]; }
    unsigned TopSettledCycles() const noexcept { return dwellSettled_[0]; }
    unsigned ReturnSettledCycles() const noexcept { return dwellSettled_[1]; }
    void Clear() noexcept;
    bool Begin(const EDM114::Mapping&,const Source&,const Policy&,const Point&,double,double,double,double,std::uint64_t,std::uint64_t) noexcept;
    bool Step(std::uint64_t,std::uint64_t,bool) noexcept;
    bool RequestStop(std::uint64_t=0ULL,std::uint64_t=0ULL) noexcept;
    bool StepStop(std::uint64_t,std::uint64_t) noexcept;
    bool Map(Point&,Point&,Point* = nullptr) const noexcept;
private:
    struct Span { double length=0.0,peak=0.0,acceleration=0.0,deceleration=0.0;std::uint64_t up=0ULL,cruise=0ULL,down=0ULL; } span_{};
    bool BuildLine(const EDM114::Mapping&,Mode,double,double,double,double) noexcept;
    bool AdvanceClock(std::uint64_t,std::uint64_t) noexcept;
    bool SetSample(double,double) noexcept;
    bool ValidSample() const noexcept;
    bool Fail(Reason reason) noexcept
    { if(!feedback_.faulted)feedback_.reason=reason;feedback_.faulted=true;feedback_.active=feedback_.complete=feedback_.moving=feedback_.endpointReady=false;feedback_.phase=Phase::Faulted;return false; }
    EDM114::Mapping original_{},lift_{};
    Source source_{};Policy policy_{};Point anchor_{},top_{};
    Feedback feedback_{};EDM114::StopScalar stop_{};
    double anchorQ_=0.0,strokeMm_=0.0;
    std::uint64_t phaseTick_=0ULL,phaseUs_=0ULL;
    std::array<std::uint64_t,2U> dwellTicks_{},dwellUs_{};
    std::array<unsigned,2U> dwellSettled_{};
};

class Excursion
{
public:
    const Feedback& Current() const noexcept { return policy_.mode==Mode::B2?cross_.Current():local_.Current(); }
    const Policy& FrozenPolicy() const noexcept { return policy_; }
    const EDM114::Mapping& CurrentMapping() const noexcept { return policy_.mode==Mode::B2?cross_.CurrentMapping():local_.CurrentMapping(); }
    const EDM114::Mapping& OriginalMapping() const noexcept { return policy_.mode==Mode::B2?cross_.OriginalMapping():local_.OriginalMapping(); }
    const EDM114::Mapping& PreviousMapping() const noexcept { return policy_.mode==Mode::B2?cross_.PreviousMapping():emptyMapping_; }
    const Source& CurrentSource() const noexcept { return policy_.mode==Mode::B2?cross_.CurrentSource():local_.CurrentSource(); }
    const CompletionToken& PreviousToken() const noexcept { return policy_.mode==Mode::B2?cross_.PreviousToken():emptyToken_; }
    const Point& Anchor() const noexcept { return policy_.mode==Mode::B2?cross_.Anchor():local_.Anchor(); }
    const Point& RetreatPulse() const noexcept { return policy_.mode==Mode::B2?crossRetreat_:local_.RetreatPulse(); }
    double AnchorQ() const noexcept { return policy_.mode==Mode::B2?cross_.AnchorQ():local_.AnchorQ(); }
    double StrokeNativeMm() const noexcept { return policy_.mode==Mode::B2?NativeLength(cross_.PreviousMapping())*EDM115::PreviousRetreatFraction:local_.StrokeNativeMm(); }
    std::uint64_t DeadlineUs() const noexcept { return EDM116::DeadlineUs(policy_.mode); }
    std::uint64_t TopDwellTicks() const noexcept { return policy_.mode==Mode::B2?0ULL:local_.TopDwellTicks(); }
    std::uint64_t TopDwellUs() const noexcept { return policy_.mode==Mode::B2?0ULL:local_.TopDwellUs(); }
    std::uint64_t ReturnDwellTicks() const noexcept { return policy_.mode==Mode::B2?0ULL:local_.ReturnDwellTicks(); }
    std::uint64_t ReturnDwellUs() const noexcept { return policy_.mode==Mode::B2?0ULL:local_.ReturnDwellUs(); }
    unsigned TopSettledCycles() const noexcept { return policy_.mode==Mode::B2?0U:local_.TopSettledCycles(); }
    unsigned ReturnSettledCycles() const noexcept { return policy_.mode==Mode::B2?0U:local_.ReturnSettledCycles(); }
    void Clear() noexcept { policy_=Policy{};local_.Clear();cross_.Clear();emptyMapping_.Clear();emptyToken_=CompletionToken{};crossRetreat_=Point{}; }
    bool Begin(const EDM114::Mapping& current,const EDM114::Mapping& previous,const Source& source,const CompletionToken& token,
        const Policy& policy,const Point& anchor,double q,double speed,double accel,double decel,std::uint64_t tick,std::uint64_t us) noexcept;
    bool Step(std::uint64_t tick,std::uint64_t us,bool settled) noexcept { return policy_.mode==Mode::B2?cross_.Step(tick,us,settled):local_.Step(tick,us,settled); }
    bool RequestStop(std::uint64_t tick=0ULL,std::uint64_t us=0ULL) noexcept { return policy_.mode==Mode::B2?cross_.RequestStop(tick,us):local_.RequestStop(tick,us); }
    bool StepStop(std::uint64_t tick,std::uint64_t us) noexcept { return policy_.mode==Mode::B2?cross_.StepStop(tick,us):local_.StepStop(tick,us); }
    bool Map(Point& p,Point& v,Point* a=nullptr) const noexcept { return policy_.mode==Mode::B2?cross_.Map(p,v,a):local_.Map(p,v,a); }
private:
    Policy policy_{};LocalExcursion local_{};EDM115::Excursion cross_{};
    EDM114::Mapping emptyMapping_{};CompletionToken emptyToken_{};Point crossRetreat_{};
};
}

namespace EDM116
{
inline void LocalExcursion::Clear() noexcept
{
    original_.Clear();lift_.Clear();source_=Source{};policy_=Policy{};anchor_=top_=Point{};
    feedback_=Feedback{};stop_.Clear();span_=Span{};anchorQ_=strokeMm_=0.0;phaseTick_=phaseUs_=0ULL;dwellTicks_={};dwellUs_={};dwellSettled_={};
}
inline bool LocalExcursion::BuildLine(const EDM114::Mapping& original,Mode mode,double stroke,double speed,double accel,double decel) noexcept
{
    const auto& g=original.GetGeometry();double plannedStroke=0.0;
    if(!ComputeRetreat(g,anchor_,anchorQ_,mode,policy_.profile.strokeRatio,top_,plannedStroke)||plannedStroke!=stroke)return false;
    EDM109::Line line{};
    if(!EDM109::BuildLine(anchor_,top_,g.pulsePerNative,line))return false;
    // Preserve software per-axis caps, and freeze scalar limits no stronger
    // than the original accepted path. The generated line is a flush leg,
    // never a replacement chord for its original ARC2 source.
    auto limits=original.FrozenLimits();
    limits.scalarMaxVelocity=(std::min)(speed*(line.pulseLength/line.lengthMm),limits.scalarMaxVelocity);
    limits.scalarAcceleration=(std::min)(limits.scalarMaxVelocity/accel,limits.scalarAcceleration);
    limits.scalarDeceleration=(std::min)(limits.scalarMaxVelocity/decel,limits.scalarDeceleration);
    for(unsigned i=0U;i<6U;++i)if(line.ratio[i]!=0.0)
    {
        const double direction=std::fabs(line.ratio[i]);
        limits.scalarMaxVelocity=(std::min)(limits.scalarMaxVelocity,limits.maxVelocityPps[i]/direction);
        limits.scalarAcceleration=(std::min)(limits.scalarAcceleration,limits.maxAccelerationPps2[i]/direction);
        limits.scalarDeceleration=(std::min)(limits.scalarDeceleration,limits.maxAccelerationPps2[i]/direction);
    }
    if(!lift_.BindLine(anchor_,top_,g.pulsePerNative,line.pulseLength,line.ratio,limits))return false;
    span_.length=line.pulseLength;
    const double peak=(std::min)(limits.scalarMaxVelocity,std::sqrt(2.0*span_.length/(1.0/limits.scalarAcceleration+1.0/limits.scalarDeceleration)));
    const double up=(std::max)(1.0,std::ceil((peak/limits.scalarAcceleration)/SampleSeconds));
    const double down=(std::max)(1.0,std::ceil((peak/limits.scalarDeceleration)/SampleSeconds));
    const double cruise=(std::max)(0.0,std::ceil((span_.length/peak)/SampleSeconds-0.5*(up+down)));
    if(!EDM114::Positive(peak)||!std::isfinite(up+down+cruise)||up+down+cruise>=static_cast<double>(LocalDeadlineTicks))return false;
    span_.up=static_cast<std::uint64_t>(up);span_.down=static_cast<std::uint64_t>(down);span_.cruise=static_cast<std::uint64_t>(cruise);
    span_.peak=span_.length/((0.5*(up+down)+cruise)*SampleSeconds);
    span_.acceleration=span_.peak/(up*SampleSeconds);span_.deceleration=span_.peak/(down*SampleSeconds);
    return EDM114::Within(span_.peak,limits.scalarMaxVelocity)&&EDM114::Within(span_.acceleration,limits.scalarAcceleration)&&
        EDM114::Within(span_.deceleration,limits.scalarDeceleration);
}
inline bool LocalExcursion::Begin(const EDM114::Mapping& original,const Source& source,const Policy& policy,
    const Point& anchor,double anchorQ,double speed,double accel,double decel,std::uint64_t tick,std::uint64_t us) noexcept
{
    if(feedback_.geometryValid||feedback_.faulted)return false;
    Clear();const auto& g=original.GetGeometry();
    if(!PolicyMatchesSource(policy,source)||!IsTriggerStep(source.step)||policy.mode==Mode::B2)return Fail(Reason::Source);
    const double nativeLength=NativeLength(original);
    if(!g.valid||!EDM114::Positive(nativeLength)||nativeLength>0.003464101617||!std::isfinite(anchorQ)||
        anchorQ<g.pulseLength*0.5||anchorQ>=g.pulseLength)return Fail(Reason::Geometry);
    Point p{},v{},a{};
    if(!original.Eval(anchorQ,0.0,0.0,p,v,a)||!EDM109::SamePulse(p,anchor))return Fail(Reason::Geometry);
    if(!std::isfinite(speed)||speed!=0.12*policy.profile.speedRatio/60.0||
        !std::isfinite(accel)||!std::isfinite(decel)||accel<0.05||decel<0.05||!tick||!us)return Fail(Reason::Dynamics);
    original_=original;source_=source;policy_=policy;anchor_=anchor;anchorQ_=anchorQ;
    strokeMm_=nativeLength*policy.profile.strokeRatio;
    if(!BuildLine(original,policy.mode,strokeMm_,speed,accel,decel))return Fail(Reason::Dynamics);
    const std::uint64_t endpointDwell=(std::max)(static_cast<std::uint64_t>(RequiredSettledCycles),EDM107::FlushDwellTicks(policy.profile));
    const std::uint64_t forecast=2ULL*(span_.up+span_.cruise+span_.down+1ULL)+RequiredSettledCycles+2ULL*endpointDwell;
    if(forecast>=LocalDeadlineTicks)return Fail(Reason::Deadline);
    feedback_.geometryValid=feedback_.active=feedback_.endpointReady=true;feedback_.started=1ULL;
    feedback_.tick=feedback_.startTick=phaseTick_=tick;feedback_.nowUs=feedback_.startUs=phaseUs_=us;
    feedback_.phase=Phase::InitialSettle;feedback_.span=0U;feedback_.rowKind=EDM114::Kind::Line;feedback_.direction=1;
    feedback_.length=span_.length;feedback_.maximumVelocity=span_.peak;feedback_.acceleration=span_.acceleration;feedback_.deceleration=span_.deceleration;
    return ValidSample()||Fail(Reason::Geometry);
}
inline bool LocalExcursion::AdvanceClock(std::uint64_t tick,std::uint64_t us) noexcept
{
    if(feedback_.tick==(std::numeric_limits<std::uint64_t>::max)()||tick!=feedback_.tick+1ULL||us<=feedback_.nowUs||us-feedback_.nowUs>1500ULL)return Fail(Reason::Clock);
    feedback_.tick=tick;feedback_.nowUs=us;return true;
}
inline bool LocalExcursion::ValidSample() const noexcept
{
    if(!feedback_.geometryValid||feedback_.span>=2U||!std::isfinite(feedback_.progress)||feedback_.progress<0.0||feedback_.progress>span_.length||
        !EDM114::Within(std::fabs(feedback_.v),span_.peak)||(feedback_.v!=0.0&&(feedback_.v>0.0?1:-1)!=feedback_.direction)||
        !EDM114::StopFits(feedback_.progress,std::fabs(feedback_.v),span_.length,span_.deceleration))return false;
    Point p{},v{},a{};return lift_.Eval(feedback_.q,feedback_.v,feedback_.a,p,v,a);
}
inline bool LocalExcursion::SetSample(double progress,double velocity) noexcept
{
    const double signedVelocity=feedback_.direction*velocity;
    feedback_.a=(signedVelocity-feedback_.v)/SampleSeconds;feedback_.progress=progress;feedback_.v=signedVelocity;
    feedback_.q=feedback_.span==0U?progress:progress==0.0?span_.length:progress==span_.length?0.0:span_.length-progress;
    return ValidSample()||Fail(Reason::Dynamics);
}
inline bool LocalExcursion::Step(std::uint64_t tick,std::uint64_t us,bool settled) noexcept
{
    if(!feedback_.active||feedback_.faulted||feedback_.stopRequested||!feedback_.geometryValid)return false;
    if(!AdvanceClock(tick,us))return false;
    if(tick-feedback_.startTick>LocalDeadlineTicks||us-feedback_.startUs>LocalDeadlineUs)return Fail(Reason::Deadline);
    if(feedback_.phase==Phase::InitialSettle||feedback_.phase==Phase::EndpointSettle)
    {
        if(tick-phaseTick_>EDM115::MaximumDwellTicks||us-phaseUs_>EDM115::MaximumDwellUs)return Fail(Reason::Dwell);
        if(feedback_.v!=0.0||!feedback_.endpointReady)return Fail(Reason::Lifecycle);
        feedback_.a=0.0;feedback_.settledCycles=settled?feedback_.settledCycles+1U:0U;
        const bool initial=feedback_.phase==Phase::InitialSettle;
        if(feedback_.settledCycles<RequiredSettledCycles||(!initial&&(tick-phaseTick_<EDM107::FlushDwellTicks(policy_.profile)||us-phaseUs_<policy_.profile.dwellUs)))return true;
        if(!initial){dwellTicks_[feedback_.span]=tick-phaseTick_;dwellUs_[feedback_.span]=us-phaseUs_;dwellSettled_[feedback_.span]=feedback_.settledCycles;}
        ++feedback_.completedDwells;
        if(!initial&&feedback_.span==1U)
        {
            Point p{},v{};
            if(feedback_.completedDwells!=3U||!Map(p,v)||!EDM109::SamePulse(p,anchor_))return Fail(Reason::Geometry);
            feedback_.completed=feedback_.anchorReturned=1ULL;feedback_.complete=true;feedback_.active=feedback_.moving=false;feedback_.phase=Phase::Complete;return true;
        }
        if(!initial){feedback_.span=1U;feedback_.direction=-1;feedback_.progress=0.0;feedback_.q=span_.length;}
        phaseTick_=tick;phaseUs_=us;feedback_.phase=Phase::Moving;feedback_.moving=true;feedback_.endpointReady=false;feedback_.settledCycles=0U;return true;
    }
    if(feedback_.phase!=Phase::Moving)return Fail(Reason::Lifecycle);
    const auto elapsed=tick-phaseTick_;
    if(elapsed>=span_.up+span_.cruise+span_.down)
    {
        if(!SetSample(span_.length,0.0))return false;
        feedback_.moving=false;feedback_.endpointReady=true;feedback_.settledCycles=0U;feedback_.phase=Phase::EndpointSettle;phaseTick_=tick;phaseUs_=us;return true;
    }
    double q=0.0,v=0.0;
    if(elapsed<=span_.up){const double t=static_cast<double>(elapsed)*SampleSeconds;q=0.5*span_.acceleration*t*t;v=span_.acceleration*t;}
    else if(elapsed<=span_.up+span_.cruise){q=0.5*span_.peak*static_cast<double>(span_.up)*SampleSeconds+span_.peak*static_cast<double>(elapsed-span_.up)*SampleSeconds;v=span_.peak;}
    else{const double t=static_cast<double>(span_.up+span_.cruise+span_.down-elapsed)*SampleSeconds;q=span_.length-0.5*span_.deceleration*t*t;v=span_.deceleration*t;}
    return SetSample(q,v);
}
inline bool LocalExcursion::RequestStop(std::uint64_t tick,std::uint64_t us) noexcept
{
    if(!feedback_.geometryValid||feedback_.faulted||feedback_.complete)return false;
    if(feedback_.stopRequested)return true;
    if((tick==0ULL)!=(us==0ULL)||(tick!=0ULL&&(tick<feedback_.tick||us<feedback_.nowUs||us<250ULL)))return Fail(Reason::Clock);
    if(!ValidSample()||!stop_.Begin(feedback_.progress,std::fabs(feedback_.v),span_.length,span_.deceleration))return Fail(Reason::StopEnvelope);
    feedback_.stopRequested=true;feedback_.active=feedback_.complete=feedback_.endpointReady=false;feedback_.settledCycles=0U;
    feedback_.stopped=stop_.Current().stopped;feedback_.moving=!feedback_.stopped;feedback_.phase=feedback_.stopped?Phase::Stopped:Phase::Stopping;
    if(feedback_.stopped)feedback_.a=0.0;
    if(tick!=0ULL){if(tick==feedback_.tick)feedback_.nowUs=us;else{feedback_.tick=tick-1ULL;feedback_.nowUs=us-250ULL;}}
    return true;
}
inline bool LocalExcursion::StepStop(std::uint64_t tick,std::uint64_t us) noexcept
{
    if(!feedback_.geometryValid||!feedback_.stopRequested||feedback_.faulted)return false;
    if(tick==feedback_.tick&&us==feedback_.nowUs)return true;
    if(!AdvanceClock(tick,us))return false;
    if(feedback_.stopped){feedback_.a=0.0;return true;}
    if(feedback_.phase!=Phase::Stopping||!stop_.Step())return Fail(Reason::StopEnvelope);
    if(!SetSample(stop_.Current().q,stop_.Current().v))return false;
    if(stop_.Current().stopped){feedback_.stopped=true;feedback_.moving=false;feedback_.phase=Phase::Stopped;}
    return true;
}
inline bool LocalExcursion::Map(Point& position,Point& velocity,Point* acceleration) const noexcept
{
    if(!feedback_.geometryValid||feedback_.faulted||!ValidSample()||&position==&velocity||
        (acceleration&&(acceleration==&position||acceleration==&velocity)))return false;
    Point p{},v{},a{};if(!lift_.Eval(feedback_.q,feedback_.v,feedback_.a,p,v,a))return false;
    position=p;velocity=v;if(acceleration)*acceleration=a;return true;
}
inline bool Excursion::Begin(const EDM114::Mapping& current,const EDM114::Mapping& previous,const Source& source,const CompletionToken& token,
    const Policy& policy,const Point& anchor,double q,double speed,double accel,double decel,std::uint64_t tick,std::uint64_t us) noexcept
{
    if(Current().geometryValid||Current().faulted||!PolicyMatchesSource(policy,source)||!IsTriggerStep(source.step))return false;
    if(policy.mode==Mode::B2)
    {
        if(!EDM116::ValidToken(token,source.step)||speed!=0.12*policy.profile.speedRatio/60.0)return false;
        policy_=policy;
        const EDM115::TraversalSchedule schedule = IsArcPairFixture(policy.fixture) ?
            EDM115::TraversalSchedule::ArcPair124 : EDM115::TraversalSchedule::Mixed115;
        if(!cross_.Begin(current,previous,source,token,anchor,q,speed,accel,decel,tick,us,schedule))return false;
        Point velocity{},acceleration{};
        return previous.Eval(previous.GetGeometry().pulseLength*(1.0-EDM115::PreviousRetreatFraction),0.0,0.0,crossRetreat_,velocity,acceleration);
    }
    if(!EDM116::EmptyToken(token))return false;
    policy_=policy;return local_.Begin(current,source,policy,anchor,q,speed,accel,decel,tick,us);
}
static_assert(std::is_trivially_copyable<Policy>::value,"EDM116 frozen policy value");
static_assert(std::is_trivially_copyable<Excursion>::value&&sizeof(Excursion)<=4096U,"EDM116 bounded mixed-mode executor");
}
