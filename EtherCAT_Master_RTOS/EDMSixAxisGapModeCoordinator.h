#pragma once
#include "EDMSixAxisGapTraversal.h"
#include "EDMSixAxisPhysicalGuard.h"
#include "EDMSixAxisModeFlush.h"
#include "EDM130WorkTimer.h"
#include "EDMG76AxisFlushGeometry.h"

// EDM128 coordinates one live GAP history and one bounded local line. The
// history is suspended during B0/B1: only the selected actual output image is
// proposed and committed. Visiting history never re-executes an NC instruction.
namespace EDM128
{
using Point=EDM126::Point;
using Mode=EDM116::Mode;
using Policy=EDM116::Policy;
using EDM116::IsCenterMode;
enum class Phase : std::uint8_t
{ Gap, Brake, AnchorDwell, Outbound, TopDwell, Returning, ReturnDwell, Resume, Stopping, Stopped, Fault, ToCenter, CenterDwell, CenterReturnDwell, ToAnchor };
inline const char* PhaseName(Phase p) noexcept
{
    switch(p) {
    case Phase::Gap:return "GAP";case Phase::Brake:return "BRAKE";case Phase::AnchorDwell:return "ANCHOR_DWELL";
    case Phase::Outbound:return "OUTBOUND";case Phase::TopDwell:return "TOP_DWELL";case Phase::Returning:return "RETURNING";
    case Phase::ReturnDwell:return "RETURN_DWELL";case Phase::Resume:return "RESUME";case Phase::Stopping:return "STOPPING";
    case Phase::Stopped:return "STOPPED";case Phase::Fault:return "FAULT";
    case Phase::ToCenter:return "TO_CENTER";case Phase::CenterDwell:return "CENTER_DWELL";
    case Phase::CenterReturnDwell:return "CENTER_RETURN_DWELL";case Phase::ToAnchor:return "TO_ANCHOR";
    } return "INVALID";
}
// EDM137 keeps the excursion state machine and its exact endpoints intact.
// A speed-stage receipt describes the command calculated from the preceding
// committed pose, before this sample is integrated and mapped to all six axes.
enum class SpeedStage : std::uint8_t
{ None, InitialRetract, NormalRetract, NormalReturn, ReturnDeceleration, FinalApproach };
inline const char* SpeedStageName(SpeedStage stage) noexcept
{
    switch(stage) {
    case SpeedStage::None:return "NONE";
    case SpeedStage::InitialRetract:return "INITIAL_RETRACT";
    case SpeedStage::NormalRetract:return "NORMAL_RETRACT";
    case SpeedStage::NormalReturn:return "NORMAL_RETURN";
    case SpeedStage::ReturnDeceleration:return "RETURN_DECELERATION";
    case SpeedStage::FinalApproach:return "FINAL_APPROACH";
    } return "INVALID";
}
// Conservative distance for discrete trapezoidal deceleration to a nonzero
// cap. One current-speed sample covers the fractional last deceleration
// sample. This smooth upper bound avoids ceil/fmod discontinuities at an
// integral number of braking samples, including floating-point roundoff.
inline double DistanceToSpeedCap(double speed,double cap,double deceleration) noexcept
{
    if(speed<=cap)return 0.0;
    return 0.5*(speed-cap)*((speed+cap)/deceleration)+speed*EDM126::SampleSeconds;
}
// Choose a target which the existing acceleration-limited integrator can
// reach this sample, while reserving enough distance to reach the final cap
// before entering E13. Endpoint braking remains owned by that integrator.
inline bool ApproachTarget(double speed,double nominal,double finalCap,double distance,
    double acceleration,double deceleration,double& target) noexcept
{
    if(!std::isfinite(speed)||speed<0.0||!EDM126::Positive(nominal)||!EDM126::Positive(finalCap)||
        finalCap>nominal||!std::isfinite(distance)||distance<0.0||
        !EDM126::Positive(acceleration)||!EDM126::Positive(deceleration))return false;
    target=nominal;
    const double step=EDM126::SampleSeconds;
    const double next=speed+(std::max)(-deceleration*step,(std::min)(acceleration*step,nominal-speed));
    if(next<=finalCap)return true;
    const auto travel=[&](double v) noexcept {
        return 0.5*(speed+v)*step+DistanceToSpeedCap(v,finalCap,deceleration);
    };
    if(travel(next)<=distance)return true;
    double low=(std::max)(finalCap,speed-deceleration*step),high=next;
    if(speed>finalCap&&travel(low)>distance+EDM126::Roundoff(distance))return false;
    // A candidate at the final cap can safely cross the boundary. Candidates
    // above it must complete all remaining discrete deceleration beforehand.
    for(unsigned i=0U;i<48U;++i) {
        const double middle=low+(high-low)*0.5;
        if(travel(middle)<=distance)low=middle;else high=middle;
    }
    target=low;return std::isfinite(target)&&target>0.0;
}
constexpr unsigned ExpectedEvents=8U, ExpectedDwells=24U;
inline unsigned ExpectedCrossings(unsigned completed) noexcept
{
    if(completed>16U)return 0U;
    unsigned n=0U;for(unsigned row=2U;row<completed;row+=2U)n+=row==2U||row==4U?2U:1U;return n;
}
inline unsigned ExpectedMaximumDepth(unsigned completed) noexcept { return completed>=3U&&completed<=16U?2U:0U; }
struct Feedback
{
    Policy policy{};
    EDM126::Feedback anchor{};
    Phase phase=Phase::Gap;
    unsigned eventCount=0U, modeEvents[EDM116::ModeCount]{}, qualifiedDwells=0U, stableCycles=0U;
    std::uint64_t stableStartUs=0ULL;
    bool rowEventComplete=false, detached=false, stopped=false, faulted=false, anchorCaptured=false, exactReturn=false;
    bool liveAnchor=false; // EDM129 explicit physical-source profile; immutable for all rows.
    EDM130::WorkEvidence work{}; // Nonzero duration is the independent EDM130 opt-in.
    // EDM132 is independently optional. All zero preserves prior profiles.
    double absoluteJumpHeightMm=0.0;
    bool targetCaptured=false,topCaptured=false;
    EDM126::Source targetSource{};
    double targetLocalMm=0.0,targetStationMm=0.0,actualJumpHeightMm=0.0;
    Point topPositionPulse{},tangentPulsePerMm{},heightPulsePerNative{};
    // EDM133: explicitly bound schedule; zero tail preserves EDM132.
    bool deepBound=false;
    std::uint32_t deepFlushCycleInterval=0U;
    double deepFlushHeightMultiplier=0.0;
    std::uint64_t eventIndex=0ULL,completedDeepEvents=0ULL;
    bool deepCycle=false;
    double effectiveJumpHeightMm=0.0;
    SpeedStage speedStage=SpeedStage::None;
    double stageDistanceMm=0.0,stageTargetMmSec=0.0;
    double initialStageDistanceMm=0.0,finalStageDistanceMm=0.0;
    // B3/B4 receipt: true orbit center, on-axis waypoint and four settled legs.
    // For these modes tangentPulsePerMm is the signed machining-axis vector.
    bool centerCaptured=false;
    Point centerPositionPulse{},nominalCenterPositionPulse{};
    double centerReturnMmMin=0.0;
    unsigned centerLegsCompleted=0U;
};
// EDM164 pauses the existing timed event, including its current backend.
// Stopping/fault and completed events cannot be made recoverable by HOLD.
inline bool IsFlushPhase(Phase phase) noexcept
{
    return phase==Phase::Brake||phase==Phase::AnchorDwell||phase==Phase::Outbound||
        phase==Phase::TopDwell||phase==Phase::Returning||phase==Phase::ReturnDwell||phase==Phase::Resume||
        phase==Phase::ToCenter||phase==Phase::CenterDwell||phase==Phase::CenterReturnDwell||phase==Phase::ToAnchor;
}
inline bool CanPauseTimedFlush(const Feedback& e) noexcept
{
    const bool mainOnly=e.policy.mode==Mode::B2||e.phase==Phase::Brake||e.phase==Phase::AnchorDwell;
    const bool backend=mainOnly?!e.detached:(e.phase==Phase::Outbound||e.detached);
    return !IsCenterMode(e.policy.mode)&&IsFlushPhase(e.phase)&&EDM116::ValidMode(e.policy.mode)&&backend&&
        !e.faulted&&!e.stopped&&e.liveAnchor&&EDM130::IsTimed(e.work)&&
        EDM130::ValidWorkEvidence(e.work)&&e.work.completedEvents<e.work.triggeredEvents&&
        e.work.triggeredEvents-e.work.completedEvents==1ULL;
}
inline bool Zero(const EDM126::Feedback& f) noexcept
{
    if(f.velocityMmSec!=0.0)return false;
    for(unsigned i=0U;i<6U;++i)if(f.velocityPps[i]!=0.0)return false;
    return true;
}
inline unsigned VisibleFlushProfile(const Policy& policy) noexcept
{return policy.fixture==EDM116::Fixture::ModalVisibleFlush158?1U:
    policy.fixture==EDM116::Fixture::ModalXYZVisibleFlush159?2U:
    policy.fixture==EDM116::Fixture::ModalSixVisibleFlush167?3U:
    policy.fixture==EDM116::Fixture::ModalG76Gap173?(policy.mode==Mode::B4?7U:policy.mode==Mode::B3?6U:policy.mode==Mode::B2?5U:policy.mode==Mode::B1?4U:policy.mode==Mode::B0?8U:0U):0U;}
// Profile 5 uses retained path distance and the ordinary motion envelope.
// It never grants the detached B1 visible-Z lift or its higher speed limit.
inline double CommissioningHeightMm(unsigned profile) noexcept
{return profile==8U?2.0:profile==5U?0.001:EDM94::VisibleFlushHeightMm(profile);}
inline bool ValidVisibleWorkTime(unsigned profile,std::uint64_t workTimeUs) noexcept
{return (profile==4U||profile==5U||profile==6U||profile==7U||profile==8U)?(workTimeUs>=250000ULL&&workTimeUs<=2000000ULL):
    (profile>=1U&&profile<=3U&&workTimeUs==500000ULL);}
inline bool VisibleFlush158(const Policy& policy) noexcept
{return VisibleFlushProfile(policy)==1U;}
// This commissioning lift uses a gentler scalar ramp than the ordinary
// axis cap, without changing configured PID gains or following protection.
// Reserve after both caps so finite differences remain strictly inside them.
inline double ReservedVisibleAcceleration(double effectiveMaxSpeedMmSec,double axisAccelerationMmSec2,double heightMm) noexcept
{
    if(!EDM126::Positive(effectiveMaxSpeedMmSec)||!EDM126::Positive(axisAccelerationMmSec2)||
        (heightMm!=5.0&&!EDM204::ValidCommissioningHeightMm(heightMm)))return 0.0;
    const double bound=(std::min)(0.04,axisAccelerationMmSec2);
    const double dt=EDM126::SampleSeconds;
    // Integrate's endpoint zero snap may consume up to Roundoff(length)/dt
    // of residual speed, hence Roundoff(length)/(dt*dt) of acceleration.
    // Profile admission separately binds the selected one-, two- or five-mm
    // stroke; this numerical reserve never grants a profile or height.
    const double reserve=(std::max)(2.0*EDM126::Roundoff(effectiveMaxSpeedMmSec)/dt,
        2.0*EDM126::Roundoff(heightMm)/(dt*dt));
    const double acceleration=bound-reserve;
    return EDM126::Positive(acceleration)?acceleration:0.0;
}
inline double ReservedVisibleAcceleration158(double effectiveMaxSpeedMmSec,double axisAccelerationMmSec2) noexcept
{return ReservedVisibleAcceleration(effectiveMaxSpeedMmSec,axisAccelerationMmSec2,5.0);}
inline double ReservedB3VisibleAcceleration(double effectiveMaxSpeedMmSec,double axisAccelerationMmSec2,
    double heightMm,double actualLengthMm) noexcept
{
    if(!EDM126::Positive(actualLengthMm))return 0.0;
    const double base=ReservedVisibleAcceleration(effectiveMaxSpeedMmSec,axisAccelerationMmSec2,heightMm);
    if(!EDM126::Positive(base))return 0.0;
    // Absolute pulse subtraction can make an axial leg slightly longer than
    // its authored span. A shorter B4 leg never raises the existing cap.
    const double extra=2.0*(EDM126::Roundoff((std::max)(heightMm,actualLengthMm))-
        EDM126::Roundoff(heightMm))/(EDM126::SampleSeconds*EDM126::SampleSeconds);
    const double acceleration=base-extra;
    return EDM126::Positive(acceleration)?acceleration:0.0;
}
inline bool ValidAbsoluteJumpHeightMm(double h,unsigned profile=0U) noexcept
{return std::isfinite(h)&&profile<=8U&&((profile==6U||profile==7U)?EDM204::ValidCommissioningHeightMm(h):
    profile?h==CommissioningHeightMm(profile):(h>=0.0001&&h<=0.005));}
inline bool HeightClose(double a,double b) noexcept
{return std::isfinite(a)&&std::isfinite(b)&&std::fabs(a-b)<=EDM126::Roundoff((std::max)(std::fabs(a),std::fabs(b)));}
// Local distance is measured by subtracting absolute pulse coordinates.
// Include their representational error in native mm, while retaining a
// precision cap 100 times smaller than the E6 one-nanometer fixed-point step.
inline bool LocalHeightClose(double measured,double expected,const Point& anchor,
    const Point& top,const Point& pulsePerNative,unsigned profile=0U) noexcept
{
    if(!EDM126::Positive(measured)||!ValidAbsoluteJumpHeightMm(expected,profile))return false;
    double tolerance=EDM126::Roundoff(expected);
    for(unsigned i=0U;i<3U;++i) {
        if(!std::isfinite(anchor[i])||!std::isfinite(top[i])||!EDM126::Positive(pulsePerNative[i]))return false;
        if(top[i]!=anchor[i]) {
            const double axisRoundoff=EDM126::Roundoff((std::max)(std::fabs(anchor[i]),std::fabs(top[i])))/pulsePerNative[i];
            if(!std::isfinite(axisRoundoff))return false;
            tolerance=std::hypot(tolerance,axisRoundoff);
        }
    }
    return tolerance<=1.0e-8&&std::fabs(measured-expected)<=tolerance;
}
inline bool EmptyB3Proof(const Feedback& e) noexcept
{return !e.centerCaptured&&e.centerReturnMmMin==0.0&&e.centerLegsCompleted==0U&&
    EDM109::SamePulse(e.centerPositionPulse,Point{})&&EDM109::SamePulse(e.nominalCenterPositionPulse,Point{});}
inline bool SameB3Proof(const Feedback& a,const Feedback& b) noexcept
{return a.centerCaptured==b.centerCaptured&&a.centerReturnMmMin==b.centerReturnMmMin&&
    a.centerLegsCompleted==b.centerLegsCompleted&&EDM109::SamePulse(a.centerPositionPulse,b.centerPositionPulse)&&
    EDM109::SamePulse(a.nominalCenterPositionPulse,b.nominalCenterPositionPulse);}
// E6 is measured opposite the signed compensated machining axis from the
// true orbit center plane. B4's diagonal join contributes to that retreat.
inline Point HeightOriginPulse(const Feedback& e) noexcept
{return IsCenterMode(e.policy.mode)?e.nominalCenterPositionPulse:e.anchor.positionPulse;}
constexpr double B3CenterRadiusLimitMm=EDM204::MaximumRadiusMm;
inline bool CenterAxisGeometry(const Feedback& e,EDM204::Geometry& geometry) noexcept
{
    geometry=EDM204::Geometry{};
    if(!IsCenterMode(e.policy.mode)||e.policy.fixture!=EDM116::Fixture::ModalG76Gap173||
        !e.centerCaptured||!e.anchorCaptured||
        !EDM204::Build(e.anchor.positionPulse,e.nominalCenterPositionPulse,e.tangentPulsePerMm,
            e.heightPulsePerNative,e.deepBound?e.effectiveJumpHeightMm:e.absoluteJumpHeightMm,
            static_cast<unsigned>(e.policy.mode),geometry)||
        !EDM109::SamePulse(e.centerPositionPulse,geometry.centerPulse)||
        (e.topCaptured&&!EDM109::SamePulse(e.topPositionPulse,geometry.topPulse)))return false;
    return true;
}
// Both center modes use their actual on-axis waypoint-to-top distance.
// Non-center modes retain their prior stage distance and E6 conventions.
inline bool TrySpeedStageSpanMm(const Feedback& e,double& span) noexcept
{
    span=e.targetCaptured?(e.deepBound?e.effectiveJumpHeightMm:e.absoluteJumpHeightMm):0.0;
    if(!std::isfinite(span)||span<0.0)return false;
    if(!e.targetCaptured||!IsCenterMode(e.policy.mode))return true;
    EDM204::Geometry geometry{};
    if(!CenterAxisGeometry(e,geometry))return false;
    span=geometry.axialSpanMm;return true;
}
inline bool ValidB3Proof(const Feedback& e) noexcept
{
    if(!IsCenterMode(e.policy.mode))return EmptyB3Proof(e);
    if(!e.targetCaptured)return EmptyB3Proof(e);
    EDM204::Geometry geometry{};
    if(!CenterAxisGeometry(e,geometry)||!EDM126::Positive(e.centerReturnMmMin)||e.centerReturnMmMin>30.0)return false;
    const unsigned n=e.centerLegsCompleted;
    if(n!=0U&&n!=1U&&n!=3U&&n!=7U&&n!=15U)return false;
    if(e.topCaptured!=(n>=3U))return false;
    switch(e.phase) {
    case Phase::ToCenter:case Phase::CenterDwell:return n==0U;
    case Phase::Outbound:case Phase::TopDwell:return n==1U;
    case Phase::Returning:case Phase::CenterReturnDwell:return n==3U;
    case Phase::ToAnchor:case Phase::ReturnDwell:return n==7U;
    case Phase::Resume:case Phase::Gap:return n==15U&&e.exactReturn;
    case Phase::Stopping:case Phase::Stopped:return true;
    default:return false;
    }
}
inline bool EmptyHeightTarget(const Feedback& e) noexcept
{
    return !e.targetCaptured&&!e.topCaptured&&EDM126::SameSource(e.targetSource,EDM126::Source{})&&
        e.targetLocalMm==0.0&&e.targetStationMm==0.0&&e.actualJumpHeightMm==0.0&&
        EDM109::SamePulse(e.topPositionPulse,Point{})&&EDM109::SamePulse(e.tangentPulsePerMm,Point{})&&
        EDM109::SamePulse(e.heightPulsePerNative,Point{});
}
inline bool ValidSpeedProof(const Feedback& e,const EDM130::FlushStages& stages) noexcept
{
    if(EDM130::EmptyFlushStages(stages))return e.speedStage==SpeedStage::None&&
        e.stageDistanceMm==0.0&&e.stageTargetMmSec==0.0&&
        e.initialStageDistanceMm==0.0&&e.finalStageDistanceMm==0.0;
    if(!EDM130::ValidFlushStages(stages))return false;
    double height=0.0;
    if(!TrySpeedStageSpanMm(e,height)||
        e.initialStageDistanceMm!=(std::min)(EDM130::FrozenInitialSlowDistanceMm(stages),height)||
        e.finalStageDistanceMm!=(std::min)(EDM130::FrozenApproachDistanceMm(stages),height))return false;
    if(e.speedStage==SpeedStage::None)return e.stageDistanceMm==0.0&&e.stageTargetMmSec==0.0;
    if(!e.targetCaptured||!EDM126::Positive(height)||!std::isfinite(e.stageDistanceMm)||
        e.stageDistanceMm<0.0||e.stageDistanceMm>height+EDM126::Roundoff(height)||
        !EDM126::Positive(e.stageTargetMmSec))return false;
    switch(e.speedStage) {
    case SpeedStage::InitialRetract:
        return (e.phase==Phase::Outbound||e.phase==Phase::TopDwell)&&e.initialStageDistanceMm>0.0&&
            (e.stageDistanceMm<e.initialStageDistanceMm||
                (e.initialStageDistanceMm==height&&e.stageDistanceMm==height));
    case SpeedStage::NormalRetract:
        return (e.phase==Phase::Outbound||e.phase==Phase::TopDwell)&&
            e.stageDistanceMm>=e.initialStageDistanceMm;
    case SpeedStage::NormalReturn:
    case SpeedStage::ReturnDeceleration:
        return (e.phase==Phase::Returning||e.phase==Phase::ReturnDwell||e.phase==Phase::CenterReturnDwell||e.phase==Phase::ToAnchor)&&
            (e.finalStageDistanceMm==0.0||e.stageDistanceMm>e.finalStageDistanceMm);
    case SpeedStage::FinalApproach:
        return (e.phase==Phase::Returning||e.phase==Phase::ReturnDwell||e.phase==Phase::CenterReturnDwell||e.phase==Phase::ToAnchor)&&e.finalStageDistanceMm>0.0&&
            e.stageDistanceMm<=e.finalStageDistanceMm;
    default:return false;
    }
}
inline bool SameSpeedProof(const Feedback& a,const Feedback& b) noexcept
{
    return a.speedStage==b.speedStage&&EDM109::SameDouble(a.stageDistanceMm,b.stageDistanceMm)&&
        EDM109::SameDouble(a.stageTargetMmSec,b.stageTargetMmSec)&&
        EDM109::SameDouble(a.initialStageDistanceMm,b.initialStageDistanceMm)&&
        EDM109::SameDouble(a.finalStageDistanceMm,b.finalStageDistanceMm);
}
// Value-only receipt validation. The live coordinator additionally compares
// this receipt with its separately retained geometry and captured targets.
inline bool ValidHeightProof(const Feedback& e,double expectedBaseH,std::uint32_t expectedN=0U,
    double expectedM=0.0,bool expectedDeepBound=false) noexcept
{
    const unsigned profile=VisibleFlushProfile(e.policy);const bool visible=profile!=0U;
    if(!ValidB3Proof(e))return false;
    if((visible&&(e.policy.mode!=(profile==8U?Mode::B0:profile==7U?Mode::B4:profile==6U?Mode::B3:profile==5U?Mode::B2:Mode::B1)||!ValidAbsoluteJumpHeightMm(expectedBaseH,profile)||expectedN||expectedM!=1.0||!expectedDeepBound))||
        ((profile==4U||profile==5U||profile==6U||profile==7U||profile==8U)&&!ValidVisibleWorkTime(profile,e.work.workTimeUs))||
        !std::isfinite(expectedBaseH)||e.absoluteJumpHeightMm!=expectedBaseH||
        !EDM130::ValidDeepFlushConfig(expectedBaseH,expectedN,expectedM,expectedDeepBound,profile)||
        e.deepBound!=expectedDeepBound||e.deepFlushCycleInterval!=expectedN||
        e.deepFlushHeightMultiplier!=expectedM)return false;
    double expectedH=expectedBaseH;
    if(!expectedDeepBound) {
        if(e.eventIndex||e.completedDeepEvents||e.deepCycle||e.effectiveJumpHeightMm!=0.0)return false;
    } else {
        if(!EDM130::ValidWorkEvidence(e.work)||!EDM130::IsTimed(e.work)||
            e.eventCount!=e.work.completedEvents||e.completedDeepEvents!=(expectedN?e.work.completedEvents/expectedN:0ULL))return false;
        if(!e.eventIndex) {
            if(e.deepCycle||e.effectiveJumpHeightMm!=0.0||e.work.triggeredEvents!=e.work.completedEvents||
                e.rowEventComplete||e.anchorCaptured||e.targetCaptured||e.topCaptured||
                (e.phase!=Phase::Gap&&e.phase!=Phase::Stopping&&e.phase!=Phase::Stopped))return false;
        } else {
            if(e.eventIndex!=e.work.triggeredEvents||
                e.deepCycle!=(expectedN&&e.eventIndex%expectedN==0ULL))return false;
            expectedH=EDM130::SelectedJumpHeightMm(expectedBaseH,expectedN,expectedM,e.eventIndex);
            if(!ValidAbsoluteJumpHeightMm(expectedH,profile)||e.effectiveJumpHeightMm!=expectedH)return false;
            if(e.work.triggeredEvents==e.work.completedEvents) {
                if(!e.rowEventComplete||!e.exactReturn||!e.targetCaptured||!e.topCaptured||
                    (e.phase!=Phase::Gap&&e.phase!=Phase::Stopping&&e.phase!=Phase::Stopped))return false;
            } else if(e.rowEventComplete||e.phase==Phase::Gap)return false;
        }
    }
    if(expectedH==0.0)return EmptyHeightTarget(e);
    if(!ValidAbsoluteJumpHeightMm(expectedH,profile)||!e.liveAnchor||!EDM130::IsTimed(e.work))return false;
    if(!e.targetCaptured) {
        if(!EmptyHeightTarget(e)||e.anchorCaptured)return false;
        return e.phase==Phase::Gap||e.phase==Phase::Brake||e.phase==Phase::AnchorDwell||
            e.phase==Phase::Stopping||e.phase==Phase::Stopped;
    }
    if(!e.anchorCaptured||!Zero(e.anchor)||!EDM126::ValidSource(e.anchor.cursorSource)||
        !EDM126::ValidSource(e.anchor.authorizedSource)||!std::isfinite(e.anchor.stationMm)||
        !std::isfinite(e.anchor.localMm)||e.anchor.localMm<0.0)return false;
    for(unsigned i=0U;i<6U;++i)if(!EDM126::Positive(e.heightPulsePerNative[i])||
        !std::isfinite(e.tangentPulsePerMm[i])||!std::isfinite(e.anchor.positionPulse[i]))return false;
    if(e.policy.mode==Mode::B2) {
        if(!EDM126::ValidSource(e.targetSource)||!EDM109::SameScope(e.targetSource.scope,e.anchor.cursorSource.scope)||
            e.targetSource.ordinal>e.anchor.cursorSource.ordinal||!std::isfinite(e.targetLocalMm)||e.targetLocalMm<0.0||
            !std::isfinite(e.targetStationMm)||e.targetStationMm<0.0||
            !HeightClose(e.anchor.stationMm-e.targetStationMm,expectedH)||
            !EDM109::SamePulse(e.tangentPulsePerMm,Point{}))return false;
    } else {
        if(!EDM126::SameSource(e.targetSource,EDM126::Source{})||e.targetLocalMm!=0.0||e.targetStationMm!=0.0)return false;
        if(e.policy.mode==Mode::B0) {
            double norm=0.0;
            for(unsigned i=0U;i<3U;++i)norm=std::hypot(norm,e.tangentPulsePerMm[i]/e.heightPulsePerNative[i]);
            if(!HeightClose(norm,1.0))return false;
        } else if(IsCenterMode(e.policy.mode)) {
            Point axis{};if(!EDM204::AxisNative(e.tangentPulsePerMm,e.heightPulsePerNative,axis))return false;
        } else if(e.policy.mode!=Mode::B1||!EDM109::SamePulse(e.tangentPulsePerMm,Point{}))return false;
    }
    if(!e.topCaptured) {
        if(e.actualJumpHeightMm!=0.0||!EDM109::SamePulse(e.topPositionPulse,Point{}))return false;
        return e.phase==Phase::Outbound||e.phase==Phase::TopDwell||e.phase==Phase::ToCenter||e.phase==Phase::CenterDwell||e.phase==Phase::Stopping||e.phase==Phase::Stopped;
    }
    const Point origin=HeightOriginPulse(e);
    if(e.policy.mode==Mode::B2?!HeightClose(e.actualJumpHeightMm,expectedH):
        !LocalHeightClose(e.actualJumpHeightMm,expectedH,origin,e.topPositionPulse,e.heightPulsePerNative,profile))return false;
    double measured=0.0;
    for(unsigned i=0U;i<6U;++i) {
        if(!std::isfinite(e.topPositionPulse[i]))return false;
        if(e.policy.mode==Mode::B0) {
            const double want=e.tangentPulsePerMm[i]==0.0?e.anchor.positionPulse[i]:
                e.anchor.positionPulse[i]-expectedH*e.tangentPulsePerMm[i];
            if(!EDM109::SameDouble(e.topPositionPulse[i],want))return false;
        } else if(IsCenterMode(e.policy.mode)) {
            const double want=e.tangentPulsePerMm[i]==0.0?origin[i]:origin[i]-expectedH*e.tangentPulsePerMm[i];
            if(!EDM109::SameDouble(e.topPositionPulse[i],want))return false;
        } else if(e.policy.mode==Mode::B1) {
            const double want=i==2U?origin[i]+expectedH*e.heightPulsePerNative[i]:origin[i];
            if(!EDM109::SameDouble(e.topPositionPulse[i],want))return false;
        }
        if(i<3U)measured=std::hypot(measured,(e.topPositionPulse[i]-origin[i])/e.heightPulsePerNative[i]);
    }
    if(e.policy.mode!=Mode::B2&&(!LocalHeightClose(measured,expectedH,origin,e.topPositionPulse,e.heightPulsePerNative,profile)||
        !HeightClose(measured,e.actualJumpHeightMm)))return false;
    if(e.phase==Phase::Gap)return e.rowEventComplete&&e.exactReturn;
    return e.phase==Phase::Returning||e.phase==Phase::ReturnDwell||e.phase==Phase::Resume||
        e.phase==Phase::CenterReturnDwell||e.phase==Phase::ToAnchor||
        e.phase==Phase::Stopping||e.phase==Phase::Stopped;
}
// This B3/B4 table is built from the sealed nominal G76 plan before live stepping.
// Index 0 is the initial center; index k+1 is child k's final center.
struct B3Orbit
{
    Point centers[EDM116::G76GapRows173+1U]{};
    Point machiningAxisNative{}; // Signed unit XYZ vector; C/U/V remain exactly zero.
    EDM109::Scope scope{};
    double centerReturnMmMin=0.0;
};
inline bool EmptyB3Orbit(const B3Orbit& b) noexcept
{
    if(!EDM109::SameScope(b.scope,EDM109::Scope{})||b.centerReturnMmMin!=0.0||
        !EDM109::SamePulse(b.machiningAxisNative,Point{}))return false;
    for(const auto& p:b.centers)if(!EDM109::SamePulse(p,Point{}))return false;
    return true;
}
inline bool SameB3Orbit(const B3Orbit& a,const B3Orbit& b) noexcept
{
    if(!EDM109::SameScope(a.scope,b.scope)||a.centerReturnMmMin!=b.centerReturnMmMin||
        !EDM109::SamePulse(a.machiningAxisNative,b.machiningAxisNative))return false;
    for(unsigned k=0U;k<=EDM116::G76GapRows173;++k)
        if(!EDM109::SamePulse(a.centers[k],b.centers[k]))return false;
    return true;
}
inline bool ValidB3Orbit(const B3Orbit& b,const EDM109::Scope& scope) noexcept
{
    if(!EDM109::ValidScope(scope)||!EDM109::SameScope(b.scope,scope)||
        !EDM126::Positive(b.centerReturnMmMin)||b.centerReturnMmMin>30.0)return false;
    double norm=0.0;
    for(unsigned a=0U;a<6U;++a) {
        if(!std::isfinite(b.machiningAxisNative[a])||(a>=3U&&b.machiningAxisNative[a]!=0.0))return false;
        if(a<3U)norm=std::hypot(norm,b.machiningAxisNative[a]);
    }
    if(!HeightClose(norm,1.0))return false;
    for(const auto& p:b.centers)for(unsigned a=0U;a<6U;++a)
        if(!std::isfinite(p[a])||(a>=3U&&p[a]!=b.centers[0U][a]))return false;
    return EDM109::SamePulse(b.centers[24U],b.centers[25U]);
}
// The signed vector must agree with the sealed center-line progression in
// physical units. Reject unresolved or reversed directions before movement.
inline bool ValidB3OrbitMapping(const B3Orbit& b,const EDM109::Scope& scope,const Point& scale) noexcept
{
    if(!ValidB3Orbit(b,scope))return false;
    for(unsigned a=0U;a<6U;++a)if(!EDM126::Positive(scale[a]))return false;
    double previous=0.0,lastBudget=0.0;
    for(unsigned k=1U;k<=EDM116::G76GapRows173;++k) {
        Point delta{};double projection=0.0,budget=EDM126::Roundoff(0.0);
        for(unsigned a=0U;a<3U;++a) {
            delta[a]=(b.centers[k][a]-b.centers[0U][a])/scale[a];
            projection+=delta[a]*b.machiningAxisNative[a];
            const double coordinate=EDM126::Roundoff((std::max)(std::fabs(b.centers[k][a]),
                std::fabs(b.centers[0U][a])))/scale[a];
            budget=std::hypot(budget,coordinate);
        }
        double residual=0.0;
        for(unsigned a=0U;a<3U;++a)residual=std::hypot(residual,
            delta[a]-projection*b.machiningAxisNative[a]);
        if(!std::isfinite(projection)||!std::isfinite(residual)||!std::isfinite(budget)||
            budget>1.0e-8||residual>budget||projection<previous-budget)return false;
        previous=projection;lastBudget=budget;
    }
    return previous>lastBudget;
}

class Coordinator
{
public:
    bool BeginRow(EDM126::Traversal& main,const Policy& policy,const Point& lowPulse,const Point& highPulse,bool liveAnchor=false,std::uint64_t workTimeUs=0ULL,double absoluteJumpHeightMm=0.0,
        std::uint32_t deepFlushCycleInterval=0U,double deepFlushHeightMultiplier=0.0,bool deepBound=false,
        const EDM130::FlushStages& flushStages=EDM130::FlushStages{},
        const B3Orbit* b3Orbit=nullptr) noexcept
    {
        const EDM126::Feedback& f=main.Committed();
        const unsigned profile=VisibleFlushProfile(policy);const bool visible=profile!=0U;
        if((IsCenterMode(policy.mode)?(!b3Orbit||!ValidB3OrbitMapping(*b3Orbit,f.authorizedSource.scope,main.CursorMapping().Geometry().line.pulsePerNative)):b3Orbit!=nullptr)||
            (rowValid_&&b3Orbit&&!SameB3Orbit(b3Orbit_,*b3Orbit))||
            (visible&&(policy.mode!=(profile==8U?Mode::B0:profile==7U?Mode::B4:profile==6U?Mode::B3:profile==5U?Mode::B2:Mode::B1)||!ValidAbsoluteJumpHeightMm(absoluteJumpHeightMm,profile)||!liveAnchor||!ValidVisibleWorkTime(profile,workTimeUs)||
                !deepBound||deepFlushCycleInterval||deepFlushHeightMultiplier!=1.0))||
            (!EDM130::EmptyFlushStages(flushStages)&&(!EDM130::ValidFlushStages(flushStages)||
                !absoluteJumpHeightMm||!liveAnchor||!workTimeUs))||
            (rowValid_&&!EDM130::SameFlushStages(flushStages_,flushStages))||
            !EDM130::ValidDeepFlushConfig(absoluteJumpHeightMm,deepFlushCycleInterval,deepFlushHeightMultiplier,deepBound,profile)||
            (deepBound&&(!absoluteJumpHeightMm||!liveAnchor||!workTimeUs))||
            (rowValid_&&(deepBound_!=deepBound||deepFlushCycleInterval_!=deepFlushCycleInterval||
                deepFlushHeightMultiplier_!=deepFlushHeightMultiplier))||
            !std::isfinite(absoluteJumpHeightMm)||(absoluteJumpHeightMm!=0.0&&
                (!ValidAbsoluteJumpHeightMm(absoluteJumpHeightMm,profile)||!liveAnchor||!workTimeUs))||
            (rowValid_&&absoluteJumpHeightMm_!=absoluteJumpHeightMm)||
            (workTimeUs&&(!liveAnchor||!EDM130::ValidWorkTime(workTimeUs)))||
            (rowValid_&&committedEvidence_.work.workTimeUs!=workTimeUs)||
            pending_||stopping_||evidence_.faulted||!f.active||f.faulted||!Zero(f)||
            !EDM116::IsModalFixture(policy.fixture)||!EDM116::PolicyMatchesBinding(policy,
                static_cast<unsigned>(f.authorizedSource.ordinal),f.authorizedSource.sourcePC,f.authorizedSource.dispatch)||
            (rowValid_&&(policy.fixture==EDM116::Fixture::ModalG76Gap173||evidence_.policy.fixture==EDM116::Fixture::ModalG76Gap173)&&
                (policy.fixture!=evidence_.policy.fixture||policy.mode!=evidence_.policy.mode||policy.selectorPC!=evidence_.policy.selectorPC||
                    policy.selectorDispatch!=evidence_.policy.selectorDispatch||policy.modeOriginPC!=evidence_.policy.modeOriginPC||
                    policy.modeOriginDispatch!=evidence_.policy.modeOriginDispatch||policy.modeOriginRevision!=evidence_.policy.modeOriginRevision||
                    !EDM107::SameFlushProfile(policy.profile,evidence_.policy.profile)))||
            (rowValid_&&(evidence_.liveAnchor!=liveAnchor||!committed_.completeReady||f.authorizedSource.ordinal!=committed_.authorizedSource.ordinal+1ULL)))return Fail();
        for(unsigned i=0U;i<6U;++i)if(!std::isfinite(lowPulse[i])||!std::isfinite(highPulse[i])||
            lowPulse[i]>=highPulse[i]||f.positionPulse[i]<lowPulse[i]||f.positionPulse[i]>highPulse[i])return Fail();
        EDM130::WorkEvidence work=committedEvidence_.work;
        if(workTimeUs&&!EDM130::BeginWork(work,workTimeUs))return Fail();
        const unsigned events=evidence_.eventCount,dwells=evidence_.qualifiedDwells;
        unsigned modes[EDM116::ModeCount]{};
        for(unsigned i=0U;i<EDM116::ModeCount;++i)modes[i]=evidence_.modeEvents[i];
        evidence_=Feedback{};evidence_.policy=policy;evidence_.eventCount=events;evidence_.qualifiedDwells=dwells;
        evidence_.liveAnchor=liveAnchor;evidence_.work=work;evidence_.absoluteJumpHeightMm=absoluteJumpHeightMm;
        evidence_.deepBound=deepBound;evidence_.deepFlushCycleInterval=deepFlushCycleInterval;
        evidence_.deepFlushHeightMultiplier=deepFlushHeightMultiplier;
        evidence_.completedDeepEvents=deepBound&&deepFlushCycleInterval?work.completedEvents/deepFlushCycleInterval:0ULL;
        absoluteJumpHeightMm_=absoluteJumpHeightMm;deepBound_=deepBound;
        deepFlushCycleInterval_=deepFlushCycleInterval;deepFlushHeightMultiplier_=deepFlushHeightMultiplier;
        flushStages_=flushStages;if(b3Orbit)b3Orbit_=*b3Orbit;
        ClearHeightCapture();
        for(unsigned i=0U;i<EDM116::ModeCount;++i)evidence_.modeEvents[i]=modes[i];
        committedEvidence_=evidence_;image_=committed_=f;low_=lowPulse;high_=highPulse;
        rowValid_=true;first_=true;localValid_=false;backend_=committedBackend_=Backend::Main;
        anchor_=topTarget_=EDM126::Cursor{};committedHeight_=height_;
        CaptureSpeedProof();committedSpeedProof_=speedProof_;local_.Clear();return true;
    }
    bool Step(EDM126::Traversal& main,std::uint64_t tick,std::uint64_t nowUs,double gapTarget,bool physicalSettled,bool workEligible=false) noexcept
    {
        if(!rowValid_||stopping_||pending_||evidence_.faulted||!std::isfinite(gapTarget)||!Clock(tick,nowUs))return Fail();
        evidence_=committedEvidence_;backend_=committedBackend_;
        ClearSpeedSample();
        Phase p=evidence_.phase;
        const bool timed=EDM130::IsTimed(evidence_.work);
        if(p==Phase::Gap&&(timed?TimedTrigger(main,workEligible&&gapTarget>0.0):Trigger(committed_))) {
            if(timed) {
                if(evidence_.eventCount>=(std::numeric_limits<unsigned>::max)()/3U||
                    evidence_.qualifiedDwells>(std::numeric_limits<unsigned>::max)()-(IsCenterMode(evidence_.policy.mode)?5U:3U)||
                    !EDM130::TriggerWork(evidence_.work,tick,nowUs))return Fail();
                // Every excursion gets its own captured pose and local backend.
                // This reset is a proposal; BeginStop restores committed output.
                evidence_.anchor=EDM126::Feedback{};evidence_.anchorCaptured=false;
                evidence_.exactReturn=evidence_.rowEventComplete=false;ClearHeightCapture();
                if(deepBound_&&!FreezeEventHeight())return Fail();
                anchor_=topTarget_=EDM126::Cursor{};local_.Clear();localValid_=false;
                localCurve_=EDM112::Curve{};localLimits_=EDM126::Limits{};localSpeed_=0.0;
            }
            evidence_.phase=p=Phase::Brake;ResetDwell();
        }
        if(p==Phase::Gap) {
            if(!main.Step(tick,nowUs,gapTarget,physicalSettled))return Fail();
            MainImage(main);
            if(timed&&!EDM130::AccrueWork(evidence_.work,nowUs-committed_.nowUs,
                workEligible&&gapTarget>0.0&&image_.velocityMmSec>0.0&&committed_.velocityMmSec>0.0&&
                !image_.atCurrentEnd&&!image_.awaitingSeam&&!committed_.atCurrentEnd&&!committed_.awaitingSeam))return Fail();
        } else if(p==Phase::Brake) {
            if(!main.Step(tick,nowUs,0.0,physicalSettled))return Fail();
            MainImage(main);
            if(Zero(image_)) {evidence_.phase=Phase::AnchorDwell;ResetDwell();}
        } else if(p==Phase::AnchorDwell) {
            if(!main.Step(tick,nowUs,0.0,physicalSettled))return Fail();
            MainImage(main);
            if(Dwell(physicalSettled,nowUs)) {
                evidence_.anchor=image_;evidence_.anchorCaptured=true;
                anchor_.source=image_.cursorSource;anchor_.localMm=image_.localMm;
                if(IsCenterMode(evidence_.policy.mode)) {if(!CaptureB3Center(main))return Fail();}
                else if(evidence_.policy.mode!=Mode::B2&&!BuildLocal(main))return Fail();
                if(absoluteJumpHeightMm_!=0.0&&!CaptureHeightTarget(main))return Fail();
                if(!EDM130::EmptyFlushStages(flushStages_)) {
                    double span=0.0;if(!TrySpeedStageSpanMm(evidence_,span))return Fail();
                    evidence_.initialStageDistanceMm=(std::min)(EDM130::FrozenInitialSlowDistanceMm(flushStages_),span);
                    evidence_.finalStageDistanceMm=(std::min)(EDM130::FrozenApproachDistanceMm(flushStages_),span);
                }
                evidence_.phase=IsCenterMode(evidence_.policy.mode)?Phase::ToCenter:Phase::Outbound;
            }
        } else if(IsCenterMode(evidence_.policy.mode)&&(p==Phase::ToCenter||p==Phase::CenterDwell||
            p==Phase::Outbound||p==Phase::TopDwell||p==Phase::Returning||p==Phase::CenterReturnDwell||
            p==Phase::ToAnchor||p==Phase::ReturnDwell)) {
            if(!StepB3(main,p,tick,nowUs,physicalSettled))return Fail();
        } else if(p==Phase::Outbound||p==Phase::Returning) {
            const bool outward=p==Phase::Outbound;
            if(evidence_.policy.mode==Mode::B2) {
                if(!EDM126::ValidSource(topTarget_.source)&&
                    !main.PreviousCursorTarget(1.0-evidence_.policy.profile.strokeRatio,topTarget_))return Fail();
                const EDM126::Cursor& target=outward?topTarget_:anchor_;
                double speed=Speed(main);
                if(!StageSpeed(outward,height_.anchorStationMm-main.Committed().stationMm,
                    main.Committed().velocityMmSec,main.CursorMapping(),speed)||
                    !main.StepTo(tick,nowUs,speed,target,physicalSettled))return Fail();
                MainImage(main);
                if(Zero(image_)&&EDM126::SameSource(image_.cursorSource,target.source)&&image_.localMm==target.localMm) {
                    evidence_.phase=outward?Phase::TopDwell:Phase::ReturnDwell;ResetDwell();
                }
            } else {
                if(!localValid_&&!StartLocal(main,tick,nowUs))return Fail();
                double speed=localSpeed_;
                if(!StageSpeed(outward,local_.Committed().localMm,local_.Committed().velocityMmSec,
                    local_.CursorMapping(),speed)||
                    !local_.Step(tick,nowUs,(outward?1.0:-1.0)*speed,physicalSettled))return Fail();
                LocalImage();
                if(Zero(image_)&&(outward?local_.Current().atCurrentEnd:local_.Current().localMm==0.0)) {
                    evidence_.phase=outward?Phase::TopDwell:Phase::ReturnDwell;ResetDwell();
                }
            }
        } else if(p==Phase::TopDwell||p==Phase::ReturnDwell) {
            if(evidence_.policy.mode==Mode::B2) {
                if(!main.Step(tick,nowUs,0.0,physicalSettled))return Fail();
            MainImage(main);
            } else {
                if(!local_.Step(tick,nowUs,0.0,physicalSettled))return Fail();
            LocalImage();
            }
            if(Dwell(physicalSettled,nowUs)) {
                if(p==Phase::TopDwell) {
                    if(absoluteJumpHeightMm_!=0.0&&!CaptureHeightTop())return Fail();
                    evidence_.phase=Phase::Returning;ResetDwell();
                }
                else {
                    if(!ExactAnchor(image_))return Fail();
                    evidence_.exactReturn=true;evidence_.phase=Phase::Resume;
                }
            }
        } else if(p==Phase::Resume) {
            if(!ExactAnchor(committed_)||!evidence_.exactReturn)return Fail();
            if(evidence_.policy.mode!=Mode::B2&&!main.ResumeAtRest(anchor_,committed_.positionPulse,
                evidence_.stableCycles,tick,nowUs))return Fail();
            if(!main.Step(tick,nowUs,0.0,physicalSettled))return Fail();
            MainImage(main);
            if(!ExactAnchor(image_)||(!timed&&evidence_.eventCount>=ExpectedEvents)||
                evidence_.eventCount==(std::numeric_limits<unsigned>::max)()||
                evidence_.modeEvents[static_cast<unsigned>(evidence_.policy.mode)]==(std::numeric_limits<unsigned>::max)())return Fail();
            if(timed&&!EDM130::CompleteWork(evidence_.work,tick,nowUs))return Fail();
            if(deepBound_) {
                evidence_.completedDeepEvents=deepFlushCycleInterval_?evidence_.work.completedEvents/deepFlushCycleInterval_:0ULL;
                height_.completedDeepEvents=evidence_.completedDeepEvents;
            }
            ++evidence_.eventCount;++evidence_.modeEvents[static_cast<unsigned>(evidence_.policy.mode)];
            evidence_.rowEventComplete=true;evidence_.phase=Phase::Gap;evidence_.detached=false;ResetDwell();
        } else return Fail();
        if(timed&&p!=Phase::Gap)evidence_.work.previousEligible=false;
        if(timed&&evidence_.phase!=Phase::Gap)image_.completeReady=false;
        if(!timed&&!evidence_.rowEventComplete&&(image_.authorizedSource.ordinal&1ULL)==0ULL)image_.completeReady=false;
        CaptureSpeedProof();return Propose();
    }
    // A completed NC row still needs fresh mapped zero receipts before its
    // physical endpoint may retire. This hold cannot accrue E5 work or flush.
    bool StepCompletedEndpoint(EDM126::Traversal& main,std::uint64_t tick,std::uint64_t nowUs) noexcept
    {
        if(!rowValid_||stopping_||pending_||evidence_.faulted||committedEvidence_.faulted||
            committedBackend_!=Backend::Main||committedEvidence_.phase!=Phase::Gap||
            !committed_.completeReady||!Zero(committed_)||committed_.historyDepth||
            !Clock(tick,nowUs))return Fail();
        evidence_=committedEvidence_;backend_=committedBackend_;ClearSpeedSample();
        if(!main.StepCompletedEndpoint(tick,nowUs))return Fail();
        MainImage(main);evidence_.work.previousEligible=false;
        CaptureSpeedProof();return Propose();
    }
    // A retained-path HOLD excursion advances the same live traversal and
    // commit protocol. It cannot trigger or complete an E5 flush, accrue work,
    // detach from authored geometry, or complete the active NC instruction.
    bool StepPausedPath(EDM126::Traversal& main,std::uint64_t tick,std::uint64_t nowUs,
        const EDM126::Cursor* target,double speedMmSec,bool physicalSettled) noexcept
    {
        const auto& path=main.Committed();
        if(!rowValid_||stopping_||pending_||evidence_.faulted||committedEvidence_.faulted||
            committedEvidence_.phase!=Phase::Gap||backend_!=Backend::Main||committedBackend_!=Backend::Main||
            committedEvidence_.detached||!committedEvidence_.liveAnchor||
            !EDM130::IsTimed(committedEvidence_.work)||
            committedEvidence_.work.triggeredEvents!=committedEvidence_.work.completedEvents||
            !path.active||path.faulted||path.stopRequested||path.stopped||main.Suspended()||
            !EDM126::SameSource(path.authorizedSource,committed_.authorizedSource)||
            !EDM126::SameSource(path.cursorSource,committed_.cursorSource)||
            path.tick!=committed_.tick||path.nowUs!=committed_.nowUs||
            path.localMm!=committed_.localMm||path.stationMm!=committed_.stationMm||
            path.positionPulse!=committed_.positionPulse||path.velocityPps!=committed_.velocityPps||
            path.velocityMmSec!=committed_.velocityMmSec||
            (target?!EDM126::Positive(speedMmSec):speedMmSec!=0.0)||!Clock(tick,nowUs))return Fail();
        evidence_=committedEvidence_;backend_=committedBackend_;ClearSpeedSample();
        if(target?!main.StepTo(tick,nowUs,speedMmSec,*target,physicalSettled):
            !main.Step(tick,nowUs,0.0,physicalSettled))return Fail();
        MainImage(main);evidence_.work.previousEligible=false;image_.completeReady=false;
        CaptureSpeedProof();return Propose();
    }
    // Freeze the E5 phase/anchor/target and brake the backend which already
    // owns this flush. The normal Step resumes this same event after START.
    bool StepPausedFlush(EDM126::Traversal& main,std::uint64_t tick,std::uint64_t nowUs,
        bool physicalSettled) noexcept
    {
        if(!rowValid_||stopping_||pending_||evidence_.faulted||
            !CanPauseTimedFlush(committedEvidence_)||backend_!=committedBackend_||
            (committedBackend_==Backend::Local?(!localValid_||!main.Suspended()):main.Suspended())||
            !Clock(tick,nowUs))return Fail();
        evidence_=committedEvidence_;backend_=committedBackend_;ClearSpeedSample();
        if(backend_==Backend::Main) {
            if(!main.Step(tick,nowUs,0.0,physicalSettled))return Fail();
            MainImage(main);
        } else {
            if(!local_.Step(tick,nowUs,0.0,physicalSettled))return Fail();
            LocalImage();
        }
        // Held wall time never satisfies an E5 dwell. Resume already has a
        // qualified exact return needed to reattach the suspended main path.
        if(evidence_.phase!=Phase::Resume)ResetDwell();
        evidence_.work.previousEligible=false;image_.completeReady=false;
        CaptureSpeedProof();return Propose();
    }
    bool CommitMapped(EDM126::Traversal& main,std::uint64_t tick) noexcept
    {
        if(!pending_||tick!=image_.tick||evidence_.faulted)return Fail();
        if(backend_==Backend::Main) {if(!main.CommitMapped(tick))return Fail();}
        else if(!local_.CommitMapped(tick))return Fail();
        committed_=image_;committedEvidence_=evidence_;committedBackend_=backend_;committedHeight_=height_;
        committedSpeedProof_=speedProof_;pending_=false;return true;
    }
    bool BeginStop(EDM126::Traversal& main,std::uint64_t tick,std::uint64_t nowUs) noexcept
    {
        if(!rowValid_||tick<committed_.tick||nowUs<committed_.nowUs||nowUs-committed_.nowUs>EDM126::MaximumSourceAgeUs)return Fail();
        image_=committed_;evidence_=committedEvidence_;backend_=committedBackend_;
        ClearSpeedSample();
        if(absoluteJumpHeightMm_!=0.0) {height_=committedHeight_;anchor_=height_.anchor;topTarget_=height_.target;}
        if(backend_==Backend::Main) {
            // The first detached proposal may not have been mapped. Restore
            // the preceding committed zero anchor before stopping that image.
            if(main.Suspended()&&(!ExactAnchor(committed_)||evidence_.stableCycles<EDM126::RequiredSeamStableCycles||
                !main.ResumeAtRest(anchor_,committed_.positionPulse,evidence_.stableCycles,tick,nowUs)))return Fail();
            if(!main.BeginStop(tick,nowUs))return Fail();
            MainImage(main);
        } else {
            if(!local_.BeginStop(tick,nowUs))return Fail();
            LocalImage();
        }
        evidence_.work.previousEligible=false;
        evidence_.phase=Phase::Stopping;evidence_.faulted=false;evidence_.stopped=Zero(image_);
        CaptureSpeedProof();pending_=false;stopping_=true;first_=true;return true;
    }
    bool StepStop(EDM126::Traversal& main,std::uint64_t tick,std::uint64_t nowUs) noexcept
    {
        if(!stopping_||pending_||evidence_.faulted)return Fail();
        if(backend_==Backend::Main) {if(!main.StepStop(tick,nowUs))return Fail();
            MainImage(main);}
        else {if(!local_.StepStop(tick,nowUs))return Fail();
            LocalImage();}
        evidence_.stopped=Zero(image_);evidence_.phase=evidence_.stopped?Phase::Stopped:Phase::Stopping;
        return Propose();
    }
    const EDM126::Feedback& Current() const noexcept{return image_;}
    const EDM126::Feedback& Committed() const noexcept{return committed_;}
    const Feedback& Evidence() const noexcept{return committedEvidence_;}
    const Feedback& ProposedEvidence() const noexcept{return evidence_;}
    bool IsHeightCurrent(double expectedH,std::uint32_t expectedN=0U,double expectedM=0.0,bool expectedDeepBound=false) const noexcept
    {
        return rowValid_&&absoluteJumpHeightMm_==expectedH&&deepBound_==expectedDeepBound&&
            deepFlushCycleInterval_==expectedN&&deepFlushHeightMultiplier_==expectedM&&HeightEvidenceCurrent(committedEvidence_,true)&&
            (!pending_||HeightEvidenceCurrent(evidence_));
    }
    bool IsFlushStagesCurrent(const EDM130::FlushStages& stages) const noexcept
    {
        return rowValid_&&EDM130::SameFlushStages(flushStages_,stages)&&
            SpeedProofCurrent(committedEvidence_,committedSpeedProof_)&&
            (!pending_||SpeedProofCurrent(evidence_,speedProof_));
    }
    bool Active() const noexcept{return rowValid_;}
    bool ExcursionActive() const noexcept{return rowValid_&&committedEvidence_.phase!=Phase::Gap;}
    void Clear() noexcept
    {
        local_.Clear();evidence_=committedEvidence_=Feedback{};image_=committed_=EDM126::Feedback{};
        anchor_=topTarget_=EDM126::Cursor{};localCurve_=EDM112::Curve{};localLimits_=EDM126::Limits{};
        low_=high_=Point{};localSpeed_=0.0;rowValid_=pending_=stopping_=first_=localValid_=false;
        backend_=committedBackend_=Backend::Main;absoluteJumpHeightMm_=0.0;
        deepBound_=false;deepFlushCycleInterval_=0U;deepFlushHeightMultiplier_=0.0;
        flushStages_=EDM130::FlushStages{};b3Orbit_=B3Orbit{};
        speedProof_=committedSpeedProof_=SpeedCapture{};
        ClearHeightCapture();committedHeight_=height_;
    }
private:
    enum class Backend:std::uint8_t {Main,Local};
    EDM126::BasicTraversal<1U> local_{};
    Feedback evidence_{},committedEvidence_{};
    EDM126::Feedback image_{},committed_{};
    EDM126::Cursor anchor_{},topTarget_{};
    EDM112::Curve localCurve_{};EDM126::Limits localLimits_{};
    Point low_{},high_{};double localSpeed_=0.0;
    struct HeightCapture {
        double anchorStationMm=0.0,targetStationMm=0.0;
        Point anchorPulse{},topPulse{},tangent{},scale{},centerPulse{},nominalCenterPulse{};
        double centerReturnMmMin=0.0;
        unsigned centerLegsCompleted=0U;
        EDM126::Cursor anchor{},target{};
        std::uint64_t eventIndex=0ULL,completedDeepEvents=0ULL;
        bool deepCycle=false;
        double effectiveJumpHeightMm=0.0;
    };
    double absoluteJumpHeightMm_=0.0;
    std::uint32_t deepFlushCycleInterval_=0U;
    double deepFlushHeightMultiplier_=0.0;
    bool deepBound_=false;
    EDM130::FlushStages flushStages_{};
    B3Orbit b3Orbit_{};
    Phase b3LocalLeg_=Phase::Gap;
    struct SpeedCapture {
        SpeedStage stage=SpeedStage::None;
        double distance=0.0,target=0.0,initialDistance=0.0,finalDistance=0.0;
    };
    SpeedCapture speedProof_{},committedSpeedProof_{};
    HeightCapture height_{},committedHeight_{};
    Backend backend_=Backend::Main,committedBackend_=Backend::Main;
    bool rowValid_=false,pending_=false,stopping_=false,first_=false,localValid_=false;
    bool Fail() noexcept
    {evidence_.faulted=true;evidence_.phase=Phase::Fault;image_.faulted=true;image_.reason=EDM126::Reason::State;image_.completeReady=false;return false;}
    bool Clock(std::uint64_t tick,std::uint64_t nowUs) noexcept
    {
        const bool seed=first_&&tick==committed_.tick&&nowUs==committed_.nowUs;
        first_=false;
        return seed||(committed_.tick!=(std::numeric_limits<std::uint64_t>::max)()&&tick==committed_.tick+1ULL&&
            nowUs>committed_.nowUs&&nowUs-committed_.nowUs<=EDM126::MaximumSourceAgeUs);
    }
    bool Trigger(const EDM126::Feedback& f) const noexcept
    {
        if(evidence_.rowEventComplete||(f.authorizedSource.ordinal&1ULL)!=0ULL)return false;
        return (evidence_.liveAnchor||f.authorizedSource.ordinal==0ULL)?(f.historyDepth==0U&&f.localMm>=f.lengthMm*0.5):
            (f.historyDepth==1U&&f.localMm<=f.lengthMm*0.8);
    }
    bool TimedTrigger(const EDM126::Traversal& main,bool eligible) const noexcept
    {
        if((VisibleFlushProfile(evidence_.policy)!=0U&&evidence_.work.completedEvents!=0ULL)||!eligible||!evidence_.work.due||committed_.velocityMmSec<=0.0||committed_.atCurrentEnd||
            committed_.awaitingSeam||committed_.localMm<=0.0||committed_.localMm>=committed_.lengthMm)return false;
        if(evidence_.policy.mode!=Mode::B2)return true;
        EDM126::Cursor target{};
        const double nextHeight=deepBound_?EDM130::SelectedJumpHeightMm(absoluteJumpHeightMm_,deepFlushCycleInterval_,
            deepFlushHeightMultiplier_,evidence_.work.completedEvents+1ULL):absoluteJumpHeightMm_;
        return absoluteJumpHeightMm_!=0.0?
            main.ResolveRetreatStation(committed_.stationMm-nextHeight,target):
            main.PreviousCursorTarget(1.0-evidence_.policy.profile.strokeRatio,target);
    }
    void ResetDwell() noexcept{evidence_.stableCycles=0U;evidence_.stableStartUs=0ULL;}
    bool Dwell(bool physical,std::uint64_t nowUs) noexcept
    {
        if(!physical||!Zero(committed_)||!Zero(image_)||!EDM109::SamePulse(committed_.positionPulse,image_.positionPulse)) {ResetDwell();return false;}
        if(!evidence_.stableCycles)evidence_.stableStartUs=nowUs;
        ++evidence_.stableCycles;
        if(evidence_.stableCycles<EDM126::RequiredSeamStableCycles||nowUs-evidence_.stableStartUs<evidence_.policy.profile.dwellUs)return false;
        if(evidence_.qualifiedDwells==(std::numeric_limits<unsigned>::max)())return Fail();
        ++evidence_.qualifiedDwells;return true;
    }
    double Speed(const EDM126::Traversal& main) const noexcept
    {return main.CursorMapping().MaximumSpeed()*evidence_.policy.profile.speedRatio;}
    void ClearSpeedSample() noexcept
    {evidence_.speedStage=SpeedStage::None;evidence_.stageDistanceMm=evidence_.stageTargetMmSec=0.0;}
    void CaptureSpeedProof() noexcept
    {
        speedProof_.stage=evidence_.speedStage;speedProof_.distance=evidence_.stageDistanceMm;
        speedProof_.target=evidence_.stageTargetMmSec;speedProof_.initialDistance=evidence_.initialStageDistanceMm;
        speedProof_.finalDistance=evidence_.finalStageDistanceMm;
    }
    bool SpeedProofCurrent(const Feedback& e,const SpeedCapture& proof) const noexcept
    {
        return ValidSpeedProof(e,flushStages_)&&e.speedStage==proof.stage&&
            EDM109::SameDouble(e.stageDistanceMm,proof.distance)&&EDM109::SameDouble(e.stageTargetMmSec,proof.target)&&
            EDM109::SameDouble(e.initialStageDistanceMm,proof.initialDistance)&&
            EDM109::SameDouble(e.finalStageDistanceMm,proof.finalDistance);
    }
    bool StageSpeed(bool outward,double distance,double velocity,const EDM126::CachedMapping& mapping,double& speed) noexcept
    {
        if(EDM130::EmptyFlushStages(flushStages_))return true;
        double height=0.0;if(!TrySpeedStageSpanMm(evidence_,height)||!EDM126::Positive(height))return false;
        double tolerance=EDM126::Roundoff(height);
        if(evidence_.policy.mode!=Mode::B2) {
            // The detached line uses absolute pulse endpoints. Include the
            // true center and waypoint in each axis's bounded subtraction
            // error; no global XYZ box or unbounded distance clipping.
            for(unsigned i=0U;i<3U;++i)if(IsCenterMode(evidence_.policy.mode)||height_.topPulse[i]!=height_.anchorPulse[i]) {
                double magnitude=(std::max)(std::fabs(height_.anchorPulse[i]),std::fabs(height_.topPulse[i]));
                if(IsCenterMode(evidence_.policy.mode)) {
                    magnitude=(std::max)(magnitude,std::fabs(height_.centerPulse[i]));
                    magnitude=(std::max)(magnitude,std::fabs(height_.nominalCenterPulse[i]));
                }
                const double axis=EDM126::Roundoff(magnitude)/height_.scale[i];
                tolerance=std::hypot(tolerance,axis);
            }
            if(!std::isfinite(tolerance)||tolerance>1.0e-8)return false;
        }
        if(!std::isfinite(distance)||distance < -tolerance||distance>height+tolerance)return false;
        distance=(std::max)(0.0,(std::min)(height,distance));
        const double nominal=(std::min)(speed,mapping.MaximumSpeed());
        // The legacy unloaded fixture encodes J50 * E7 in speedRatio.
        // J50 limits nominal travel; it must not halve native Pump speeds.
        const double ratio=2.0*evidence_.policy.profile.speedRatio;
        const double initial=(std::min)(nominal,EDM130::FrozenInitialSlowSpeedMmMin(flushStages_)*ratio/60.0);
        const double final=(std::min)(nominal,EDM130::FrozenFinalApproachSpeedMmMin(flushStages_)*ratio/60.0);
        if(!EDM126::Positive(initial)||!EDM126::Positive(final)||!EDM126::Positive(nominal))return false;
        speed=nominal;evidence_.stageDistanceMm=distance;
        if(outward) {
            const bool slow=evidence_.initialStageDistanceMm>0.0&&
                (distance<evidence_.initialStageDistanceMm||evidence_.initialStageDistanceMm==height);
            evidence_.speedStage=slow?SpeedStage::InitialRetract:SpeedStage::NormalRetract;
            if(slow)speed=initial;
        } else if(evidence_.finalStageDistanceMm>0.0&&distance<=evidence_.finalStageDistanceMm) {
            if(std::fabs(velocity)>final+EDM126::Roundoff(final))return false;
            speed=final;evidence_.speedStage=SpeedStage::FinalApproach;
        } else {
            evidence_.speedStage=SpeedStage::NormalReturn;
            if(evidence_.finalStageDistanceMm>0.0&&
                !ApproachTarget(std::fabs(velocity),nominal,final,distance-evidence_.finalStageDistanceMm,
                    mapping.Acceleration(),mapping.Deceleration(),speed))return false;
            if(speed<nominal)evidence_.speedStage=SpeedStage::ReturnDeceleration;
        }
        evidence_.stageTargetMmSec=speed;return true;
    }
    bool CaptureB3Center(const EDM126::Traversal& main) noexcept
    {
        const auto& a=evidence_.anchor;
        const auto& scale=main.CursorMapping().Geometry().line.pulsePerNative;
        if(!IsCenterMode(evidence_.policy.mode)||(VisibleFlushProfile(evidence_.policy)!=6U&&VisibleFlushProfile(evidence_.policy)!=7U)||
            !ValidB3OrbitMapping(b3Orbit_,a.cursorSource.scope,scale)||
            a.cursorSource.ordinal>=EDM116::G76GapRows173||!EDM126::Positive(a.lengthMm)||
            a.localMm<0.0||a.localMm>a.lengthMm||!Zero(a))return false;
        const unsigned k=static_cast<unsigned>(a.cursorSource.ordinal);
        const double u=a.localMm==a.lengthMm?1.0:a.localMm/a.lengthMm;
        Point nominal=a.positionPulse,axisPulse{};
        for(unsigned j=0U;j<6U;++j) {
            const double v=u==1.0?b3Orbit_.centers[k+1U][j]:
                b3Orbit_.centers[k][j]+(b3Orbit_.centers[k+1U][j]-b3Orbit_.centers[k][j])*u;
            if(!std::isfinite(v)||!EDM126::Positive(scale[j])||v<low_[j]||v>high_[j])return false;
            if(j<3U)nominal[j]=v;
            else if(v!=a.positionPulse[j])return false;
            axisPulse[j]=b3Orbit_.machiningAxisNative[j]==0.0?0.0:b3Orbit_.machiningAxisNative[j]*scale[j];
        }
        EDM204::Geometry geometry{};
        if(!EDM204::Build(a.positionPulse,nominal,axisPulse,scale,EventHeight(),
            static_cast<unsigned>(evidence_.policy.mode),geometry))return false;
        for(unsigned j=0U;j<6U;++j)if(geometry.centerPulse[j]<low_[j]||geometry.centerPulse[j]>high_[j]||
            geometry.topPulse[j]<low_[j]||geometry.topPulse[j]>high_[j])return false;
        height_.centerPulse=geometry.centerPulse;height_.nominalCenterPulse=nominal;
        height_.centerReturnMmMin=b3Orbit_.centerReturnMmMin;
        evidence_.centerCaptured=true;evidence_.centerPositionPulse=geometry.centerPulse;
        evidence_.nominalCenterPositionPulse=nominal;evidence_.tangentPulsePerMm=axisPulse;
        evidence_.centerReturnMmMin=b3Orbit_.centerReturnMmMin;
        evidence_.centerLegsCompleted=height_.centerLegsCompleted=0U;
        return true;
    }
    bool StartB3Leg(EDM126::Traversal& main,Phase leg) noexcept
    {
        if(!evidence_.centerCaptured||!evidence_.targetCaptured||!Zero(committed_)||
            (leg!=Phase::ToCenter&&leg!=Phase::Outbound&&leg!=Phase::ToAnchor))return false;
        const Point& from=leg==Phase::ToCenter?height_.anchorPulse:height_.centerPulse;
        const Point& to=leg==Phase::ToCenter?height_.centerPulse:leg==Phase::Outbound?height_.topPulse:height_.anchorPulse;
        if(!EDM109::SamePulse(committed_.positionPulse,from))return false;
        EDM109::Line line{};
        if(!EDM109::BuildLine(from,to,height_.scale,line)||!EDM112::BuildLine(line,localCurve_))return false;
        localLimits_=main.CursorMapping().FrozenLimits();
        if(leg==Phase::Outbound) {
            // The machining axis can point anywhere in XYZ. Intersect every
            // participating physical derivative before choosing the ramp.
            double speed=EDM94::VisibleFlushReferenceMmSec;
            double axisAcceleration=(std::numeric_limits<double>::max)();
            bool participating=false;
            for(unsigned i=0U;i<6U;++i) {
                const double derivative=std::fabs((to[i]-from[i])/localCurve_.lengthMm);
                if(!std::isfinite(derivative)||(i>=3U&&derivative!=0.0))return false;
                if(derivative==0.0)continue;
                participating=true;
                speed=(std::min)(speed,localLimits_.maxVelocityPps[i]/derivative);
                axisAcceleration=(std::min)(axisAcceleration,localLimits_.maxAccelerationPps2[i]/derivative);
            }
            if(!participating||!EDM126::Positive(speed)||!EDM126::Positive(axisAcceleration))return false;
            // E210: profile.speedRatio encodes J50 * E7. Apply E7 once to
            // the complete axial leg and both ramps; the independent radial
            // center feed below remains unchanged. Returning reuses this line.
            const double ratio=2.0*evidence_.policy.profile.speedRatio;
            if(!std::isfinite(ratio)||ratio<0.5||ratio>1.0)return false;
            speed*=ratio;
            // Scale the physical bound before reserving numerical snap error.
            // The full reserve is still required at a reduced E7 setting.
            axisAcceleration=(std::min)(0.04,axisAcceleration)*ratio;
            localLimits_.maxVelocityMmSec=speed;
            const double acc=ReservedB3VisibleAcceleration(speed,axisAcceleration,EventHeight(),localCurve_.lengthMm);
            if(!EDM126::Positive(acc))return false;
            localLimits_.accelerationMmSec2=localLimits_.decelerationMmSec2=acc;
        } else {
            // The independent center setting is bounded by the ordinary XYZ
            // commissioning reference and frozen per-axis limits. The visible
            // axial flush permission never raises the center-leg reference.
            localLimits_.maxVelocityMmSec=(std::min)(height_.centerReturnMmMin/60.0,
                EDM94::VisibleXYZMachiningReferenceMmSec);
            const double reserve=2.0*EDM126::Roundoff(localCurve_.lengthMm)/
                (EDM126::SampleSeconds*EDM126::SampleSeconds);
            localLimits_.accelerationMmSec2=(std::min)(0.04,localLimits_.accelerationMmSec2)-reserve;
            localLimits_.decelerationMmSec2=(std::min)(0.04,localLimits_.decelerationMmSec2)-reserve;
        }
        if(!EDM126::Positive(localLimits_.accelerationMmSec2)||!EDM126::Positive(localLimits_.decelerationMmSec2))return false;
        if(!main.Suspended()&&((leg!=Phase::ToCenter&&
            !(leg==Phase::Outbound&&EDM109::SamePulse(height_.anchorPulse,height_.centerPulse)))||
            !main.SuspendAtRest(anchor_)))return false;
        EDM126::Settings settings{};settings.maximumRetreatMm=localCurve_.lengthMm*2.0;
        local_.Clear();
        if(!local_.Begin(evidence_.anchor.authorizedSource.scope,settings)||
            !local_.BindCurrent(localCurve_,evidence_.anchor.authorizedSource,localLimits_,
                committed_.tick,committed_.nowUs))return false;
        localSpeed_=localLimits_.maxVelocityMmSec;localValid_=true;b3LocalLeg_=leg;return true;
    }
    bool StepB3(EDM126::Traversal& main,Phase p,std::uint64_t tick,std::uint64_t nowUs,bool physicalSettled) noexcept
    {
        const bool moving=p==Phase::ToCenter||p==Phase::Outbound||p==Phase::Returning||p==Phase::ToAnchor;
        if(moving) {
            // Braking can stop at the finish-center endpoint. A coincident
            // radial leg still needs physical settlement, but has no distance
            // to integrate. Use the stationary origin of the real axial backend;
            // never invent a tiny chord or claim movement for this leg.
            if((p==Phase::ToCenter||p==Phase::ToAnchor)&&
                EDM109::SamePulse(height_.anchorPulse,height_.centerPulse)) {
                if(!localValid_&&!StartB3Leg(main,Phase::Outbound))return false;
                if(!local_.Step(tick,nowUs,0.0,physicalSettled))return false;
                LocalImage();
                if(!Zero(image_)||!EDM109::SamePulse(image_.positionPulse,height_.centerPulse))return false;
                evidence_.phase=p==Phase::ToCenter?Phase::CenterDwell:Phase::ReturnDwell;
                ResetDwell();return true;
            }
            // A coincident center and apex has no axial leg. Retain the real
            // diagonal backend at its settled endpoint for both zero legs.
            if((p==Phase::Outbound||p==Phase::Returning)&&
                EDM109::SamePulse(height_.centerPulse,height_.topPulse)) {
                if(!localValid_||!local_.Step(tick,nowUs,0.0,physicalSettled))return false;
                LocalImage();
                if(!Zero(image_)||!EDM109::SamePulse(image_.positionPulse,height_.centerPulse))return false;
                evidence_.phase=p==Phase::Outbound?Phase::TopDwell:Phase::CenterReturnDwell;
                ResetDwell();return true;
            }
            const Phase leg=p==Phase::Returning?Phase::Outbound:p;
            if((!localValid_||b3LocalLeg_!=leg)&&!StartB3Leg(main,leg))return false;
            double speed=localSpeed_;
            if((p==Phase::Outbound||p==Phase::Returning)&&
                !StageSpeed(p==Phase::Outbound,local_.Committed().localMm,
                    local_.Committed().velocityMmSec,local_.CursorMapping(),speed))return false;
            // E11/E13 also apply to the last radial return to the actual
            // machining anchor; reaching the center is not final approach.
            if(p==Phase::ToAnchor&&!StageSpeed(false,
                localCurve_.lengthMm-local_.Committed().localMm,local_.Committed().velocityMmSec,
                local_.CursorMapping(),speed))return false;
            if(!local_.Step(tick,nowUs,(p==Phase::Returning?-1.0:1.0)*speed,physicalSettled))return false;
            LocalImage();
            if(Zero(image_)&&(p==Phase::Returning?local_.Current().localMm==0.0:local_.Current().atCurrentEnd)) {
                evidence_.phase=p==Phase::ToCenter?Phase::CenterDwell:p==Phase::Outbound?Phase::TopDwell:
                    p==Phase::Returning?Phase::CenterReturnDwell:Phase::ReturnDwell;
                ResetDwell();
            }
            return true;
        }
        if(!localValid_||!local_.Step(tick,nowUs,0.0,physicalSettled))return false;
        LocalImage();
        if(!Dwell(physicalSettled,nowUs))return !evidence_.faulted;
        const Point& want=p==Phase::TopDwell?height_.topPulse:p==Phase::ReturnDwell?height_.anchorPulse:height_.centerPulse;
        if(!Zero(image_)||!EDM109::SamePulse(image_.positionPulse,want))return false;
        if(p==Phase::CenterDwell) {
            evidence_.centerLegsCompleted=1U;evidence_.phase=Phase::Outbound;
        } else if(p==Phase::TopDwell) {
            if(!CaptureHeightTop())return false;
            evidence_.centerLegsCompleted=3U;evidence_.phase=Phase::Returning;
        } else if(p==Phase::CenterReturnDwell) {
            evidence_.centerLegsCompleted=7U;evidence_.phase=Phase::ToAnchor;
        } else if(p==Phase::ReturnDwell) {
            if(!ExactAnchor(image_))return false;
            evidence_.centerLegsCompleted=15U;evidence_.exactReturn=true;evidence_.phase=Phase::Resume;
        } else return false;
        height_.centerLegsCompleted=evidence_.centerLegsCompleted;
        if(p!=Phase::ReturnDwell)ResetDwell();
        return true;
    }
    bool BuildLocal(const EDM126::Traversal& main) noexcept
    {
        const auto& curve=main.CursorMapping().Geometry();Point top=evidence_.anchor.positionPulse;
        const double stroke=absoluteJumpHeightMm_!=0.0?EventHeight():curve.lengthMm*evidence_.policy.profile.strokeRatio;
        if(!EDM126::Positive(stroke))return false;
        if(evidence_.policy.mode==Mode::B0) {
            if(absoluteJumpHeightMm_!=0.0) {
                if(!main.CursorMapping().TangentPulsePerMm(evidence_.anchor.localMm,evidence_.tangentPulsePerMm))return false;
                for(unsigned i=0U;i<6U;++i)if(evidence_.tangentPulsePerMm[i]!=0.0)
                    top[i]-=stroke*evidence_.tangentPulsePerMm[i];
            } else {
                Point p{},d1{};if(!EDM112::EvaluateUnit(curve,evidence_.anchor.localMm/curve.lengthMm,p,&d1,nullptr))return false;
                double norm=0.0;for(unsigned i=0U;i<3U;++i)norm=std::hypot(norm,d1[i]/curve.line.pulsePerNative[i]);
                if(!EDM126::Positive(norm))return false;
                for(unsigned i=0U;i<6U;++i)if(d1[i]!=0.0)top[i]-=stroke*d1[i]/norm;
            }
        } else if(evidence_.policy.mode==Mode::B1)top[2]+=stroke*curve.line.pulsePerNative[2];else return false;
        for(unsigned i=0U;i<6U;++i)if(!std::isfinite(top[i])||top[i]<low_[i]||top[i]>high_[i])return false;
        EDM109::Line line{};
        if(!EDM109::BuildLine(evidence_.anchor.positionPulse,top,curve.line.pulsePerNative,line)||!EDM112::BuildLine(line,localCurve_))return false;
        localLimits_=main.CursorMapping().FrozenLimits();
        if(VisibleFlushProfile(evidence_.policy)!=0U) {
            localLimits_.maxVelocityMmSec=EDM94::VisibleFlushReferenceMmSec;
            double speed=localLimits_.maxVelocityMmSec,axisAcceleration=0.0;
            if(VisibleFlushProfile(evidence_.policy)==8U) {
                // Native B0 can be horizontal or oblique. Intersect every
                // participating XYZ derivative with its own frozen limits.
                bool participating=false;
                axisAcceleration=(std::numeric_limits<double>::max)();
                for(unsigned i=0U;i<6U;++i) {
                    const double d=std::fabs((localCurve_.line.endPulse[i]-localCurve_.line.startPulse[i])/localCurve_.lengthMm);
                    if(!std::isfinite(d)||(i>=3U&&d!=0.0))return false;
                    if(d==0.0)continue;
                    participating=true;
                    speed=(std::min)(speed,localLimits_.maxVelocityPps[i]/d);
                    axisAcceleration=(std::min)(axisAcceleration,localLimits_.maxAccelerationPps2[i]/d);
                }
                if(!participating||!EDM126::Positive(speed)||!EDM126::Positive(axisAcceleration))return false;
                localLimits_.maxVelocityMmSec=speed;
            } else {
                const double dz=std::fabs((localCurve_.line.endPulse[2]-localCurve_.line.startPulse[2])/localCurve_.lengthMm);
                if(!EDM126::Positive(dz))return false;
                speed=(std::min)(speed,localLimits_.maxVelocityPps[2]/dz);
                axisAcceleration=localLimits_.maxAccelerationPps2[2]/dz;
            }
            const double acceleration=ReservedVisibleAcceleration(speed,axisAcceleration,EventHeight());
            if(!EDM126::Positive(acceleration))return false;
            localLimits_.accelerationMmSec2=localLimits_.decelerationMmSec2=acceleration;
        } else localLimits_.maxVelocityMmSec*=evidence_.policy.profile.speedRatio;
        localSpeed_=localLimits_.maxVelocityMmSec;return true;
    }
    void ClearHeightCapture() noexcept
    {
        evidence_.targetCaptured=evidence_.topCaptured=false;evidence_.targetSource=EDM126::Source{};
        evidence_.targetLocalMm=evidence_.targetStationMm=evidence_.actualJumpHeightMm=0.0;
        evidence_.topPositionPulse=evidence_.tangentPulsePerMm=evidence_.heightPulsePerNative=Point{};
        evidence_.eventIndex=0ULL;evidence_.deepCycle=false;evidence_.effectiveJumpHeightMm=0.0;
        ClearSpeedSample();evidence_.initialStageDistanceMm=evidence_.finalStageDistanceMm=0.0;
        evidence_.centerCaptured=false;evidence_.centerPositionPulse=evidence_.nominalCenterPositionPulse=Point{};
        evidence_.centerReturnMmMin=0.0;evidence_.centerLegsCompleted=0U;b3LocalLeg_=Phase::Gap;
        height_=HeightCapture{};height_.completedDeepEvents=evidence_.completedDeepEvents;
    }
    double EventHeight() const noexcept
    {return deepBound_?evidence_.effectiveJumpHeightMm:absoluteJumpHeightMm_;}
    bool FreezeEventHeight() noexcept
    {
        if(!deepBound_||!evidence_.work.triggeredEvents||
            evidence_.work.triggeredEvents!=evidence_.work.completedEvents+1ULL)return false;
        evidence_.eventIndex=evidence_.work.triggeredEvents;
        evidence_.deepCycle=deepFlushCycleInterval_&&evidence_.eventIndex%deepFlushCycleInterval_==0ULL;
        evidence_.effectiveJumpHeightMm=EDM130::SelectedJumpHeightMm(absoluteJumpHeightMm_,deepFlushCycleInterval_,
            deepFlushHeightMultiplier_,evidence_.eventIndex);
        if(!ValidAbsoluteJumpHeightMm(evidence_.effectiveJumpHeightMm,VisibleFlushProfile(evidence_.policy)))return false;
        height_.eventIndex=evidence_.eventIndex;height_.deepCycle=evidence_.deepCycle;
        height_.effectiveJumpHeightMm=evidence_.effectiveJumpHeightMm;return true;
    }
    bool CaptureHeightTarget(const EDM126::Traversal& main) noexcept
    {
        height_.anchorPulse=evidence_.anchor.positionPulse;height_.anchorStationMm=evidence_.anchor.stationMm;
        height_.scale=main.CursorMapping().Geometry().line.pulsePerNative;
        evidence_.heightPulsePerNative=height_.scale;height_.tangent=evidence_.tangentPulsePerMm;
        if(evidence_.policy.mode==Mode::B2) {
            // Brake displacement belongs to GAP. H starts at the final rest
            // anchor, so never reuse the earlier trigger-eligibility target.
            if(!main.ResolveRetreatStation(height_.anchorStationMm-EventHeight(),topTarget_)||
                !main.EvaluateCursorAtRest(topTarget_,height_.topPulse,&height_.targetStationMm))return false;
            evidence_.targetSource=topTarget_.source;evidence_.targetLocalMm=topTarget_.localMm;
            evidence_.targetStationMm=height_.targetStationMm;
        } else if(IsCenterMode(evidence_.policy.mode)) {
            EDM204::Geometry geometry{};
            if(!CenterAxisGeometry(evidence_,geometry))return false;
            height_.topPulse=geometry.topPulse;
        } else height_.topPulse=localCurve_.line.endPulse;
        for(unsigned i=0U;i<6U;++i)if(!std::isfinite(height_.topPulse[i])||
            height_.topPulse[i]<low_[i]||height_.topPulse[i]>high_[i])return false;
        height_.anchor=anchor_;height_.target=topTarget_;evidence_.targetCaptured=true;return true;
    }
    bool CaptureHeightTop() noexcept
    {
        if(!evidence_.targetCaptured||!Zero(image_)||!EDM109::SamePulse(image_.positionPulse,height_.topPulse))return false;
        const Point origin=HeightOriginPulse(evidence_);
        double actual=0.0;
        if(evidence_.policy.mode==Mode::B2) {
            if(!EDM126::SameSource(image_.cursorSource,topTarget_.source)||image_.localMm!=topTarget_.localMm||
                image_.stationMm!=height_.targetStationMm)return false;
            actual=height_.anchorStationMm-image_.stationMm;
        } else for(unsigned i=0U;i<3U;++i)
            actual=std::hypot(actual,(image_.positionPulse[i]-origin[i])/height_.scale[i]);
        if(evidence_.policy.mode==Mode::B2?!HeightClose(actual,EventHeight()):
            !LocalHeightClose(actual,EventHeight(),origin,image_.positionPulse,height_.scale,VisibleFlushProfile(evidence_.policy)))return false;
        evidence_.topPositionPulse=image_.positionPulse;evidence_.actualJumpHeightMm=actual;
        evidence_.topCaptured=true;return true;
    }
    bool HeightEvidenceCurrent(const Feedback& e,bool committed=false) const noexcept
    {
        const HeightCapture& h=committed?committedHeight_:height_;
        if(!ValidHeightProof(e,absoluteJumpHeightMm_,deepFlushCycleInterval_,deepFlushHeightMultiplier_,deepBound_))return false;
        if(deepBound_&&(e.eventIndex!=h.eventIndex||e.deepCycle!=h.deepCycle||
            e.effectiveJumpHeightMm!=h.effectiveJumpHeightMm||e.completedDeepEvents!=h.completedDeepEvents))return false;
        if(!e.targetCaptured)return true;
        if(!EDM126::SameSource(e.anchor.cursorSource,h.anchor.source)||e.anchor.localMm!=h.anchor.localMm||
            !EDM126::SameSource(e.anchor.authorizedSource,committed_.authorizedSource)||
            e.anchor.stationMm!=h.anchorStationMm||!EDM109::SamePulse(e.anchor.positionPulse,h.anchorPulse)||
            !EDM109::SamePulse(e.heightPulsePerNative,h.scale)||!EDM109::SamePulse(e.tangentPulsePerMm,h.tangent))return false;
        if(IsCenterMode(e.policy.mode)&&(!e.centerCaptured||e.centerReturnMmMin!=h.centerReturnMmMin||
            e.centerLegsCompleted!=h.centerLegsCompleted||
            !EDM109::SamePulse(e.centerPositionPulse,h.centerPulse)||
            !EDM109::SamePulse(e.nominalCenterPositionPulse,h.nominalCenterPulse)))return false;
        if(e.policy.mode==Mode::B2&&(!EDM126::SameSource(e.targetSource,h.target.source)||
            e.targetLocalMm!=h.target.localMm||e.targetStationMm!=h.targetStationMm))return false;
        return !e.topCaptured||EDM109::SamePulse(e.topPositionPulse,h.topPulse);
    }
    bool StartLocal(EDM126::Traversal& main,std::uint64_t tick,std::uint64_t nowUs) noexcept
    {
        if(!main.SuspendAtRest(anchor_))return false;
        EDM126::Settings settings{};settings.maximumRetreatMm=localCurve_.lengthMm*2.0;
        local_.Clear();
        if(!local_.Begin(evidence_.anchor.authorizedSource.scope,settings)||
            !local_.BindCurrent(localCurve_,evidence_.anchor.authorizedSource,localLimits_,tick,nowUs))return false;
        localValid_=true;return true;
    }
    void MainImage(const EDM126::Traversal& main) noexcept
    {image_=main.Current();backend_=Backend::Main;evidence_.detached=false;}
    void LocalImage() noexcept
    {
        const auto& f=local_.Current();image_=evidence_.anchor;
        image_.tick=f.tick;image_.nowUs=f.nowUs;image_.positionPulse=f.positionPulse;
        image_.velocityPps=f.velocityPps;image_.accelerationPps2=f.accelerationPps2;
        image_.velocityMmSec=f.velocityMmSec;image_.accelerationMmSec2=f.accelerationMmSec2;
        image_.completeReady=image_.atCurrentEnd=image_.atRetreatLimit=image_.awaitingSeam=false;
        image_.seamStableCycles=0U;image_.stopRequested=f.stopRequested;image_.stopped=f.stopped;
        image_.faulted=f.faulted;image_.reason=f.reason;backend_=Backend::Local;evidence_.detached=true;
    }
    bool ExactAnchor(const EDM126::Feedback& f) const noexcept
    {return evidence_.anchorCaptured&&Zero(f)&&EDM126::SameSource(f.authorizedSource,evidence_.anchor.authorizedSource)&&
        EDM126::SameSource(f.cursorSource,anchor_.source)&&f.localMm==anchor_.localMm&&
        EDM109::SamePulse(f.positionPulse,evidence_.anchor.positionPulse);}
    bool Propose() noexcept
    {
        if(!EDM130::ValidWorkEvidence(evidence_.work)||!HeightEvidenceCurrent(evidence_)||
            !SpeedProofCurrent(evidence_,speedProof_)||image_.faulted||!EDM126::SameSource(image_.authorizedSource,committed_.authorizedSource))return Fail();
        for(unsigned i=0U;i<6U;++i)if(!std::isfinite(image_.positionPulse[i])||image_.positionPulse[i]<low_[i]||image_.positionPulse[i]>high_[i])return Fail();
        pending_=true;return true;
    }
};
static_assert(sizeof(Feedback)<=1216U,"EDM128 bounded immutable axis-center evidence");
}
