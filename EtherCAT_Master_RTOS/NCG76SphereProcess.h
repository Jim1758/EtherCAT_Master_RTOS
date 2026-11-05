#pragma once

#include "NCG76SphereCycle.h"
#include "NCG76SpherePath.h"
#include <cstdint>
#include <type_traits>

// EDM292: NC-side observation contract for a future P50 GAP/process adapter.
// A nominal station identifies the ORIGINAL machining path even while HOLD
// or resume moves elsewhere. Never substitute this point for actual feedback,
// a current motion command, a sealed history receipt, or permission to discharge.
// EDM301 observes SIM-only B2/B3/B4 retreat routes. B1 and physical
// discharge remain unreleased; this envelope never authorizes motion.
namespace NCG76SphereProcess
{
constexpr bool DischargeExecutionReleased=false;
enum class Role : std::uint8_t
{ Nominal, NominalSettle, NominalBrake, HoldReturn, HoldOrResumeSettle,
  HeldAtStart, Resume, ResumeBrake, Complete,
  FlushBrake, FlushSettle, Flush, FlushHoldBrake, GapHistorySettle, GapHistory };

struct Envelope
{
    NCG76SpherePath::Sample nominal{};
    NCG76SphereCycle::Phase phase=NCG76SphereCycle::Phase::Idle;
    Role role=Role::Nominal;
    std::uint64_t session=0ULL,tick=0ULL,holdSequence=0ULL,resumeSequence=0ULL;
    double layerRadiusMm=0.0,layerDepthMm=0.0;
    bool checkpointSaved=false,atNominalPath=false,valid=false,suspended=false;
};
inline bool IsSuspended(const NCG76SphereCycle::Feedback& feedback) noexcept
{return feedback.holdActive||feedback.held||feedback.flushActive||feedback.gapHistoryActive||feedback.finishActive;}

inline bool NominalStage(NCG76SphereCycle::Phase phase,NCG76SpherePath::Stage& stage) noexcept
{
    using P=NCG76SphereCycle::Phase;using S=NCG76SpherePath::Stage;
    switch(phase){
    case P::InitialDepth:stage=S::InitialDepth;return true;
    case P::ContourNominal:stage=S::ContourNominal;return true;
    case P::ContourFinish:stage=S::ContourFinish;return true;
    case P::InitialRetreat:stage=S::InitialRetreat;return true;
    case P::CircleEntry:stage=S::CircleEntry;return true;
    case P::Orbit:stage=S::Orbit;return true;
    case P::ReturnCenter:stage=S::ReturnCenter;return true;
    case P::AdvanceCenter:stage=S::AdvanceCenter;return true;
    default:return false;
    }
}

// Descriptor Build/Valid belongs to NC preparation. This bounded projection
// consumes one already scope-matched coherent RT feedback value; it writes no
// controller, output or history state. Ordinary failure withdraws the envelope;
// overlapping inputs/output are rejected untouched to preserve the inputs.
inline bool Observe(const NCG76SpherePath::Descriptor& path,
    const NCG76SphereCycle::Feedback& feedback,Envelope& output) noexcept
{
    // EDM359: the D-completion return has its own exact endpoint proof; it
    // is not the authored nominal end station used by this display envelope.

    if(NCG76SpherePath::Detail::Overlap(&path,sizeof(path),&output,sizeof(output))||
       NCG76SpherePath::Detail::Overlap(&feedback,sizeof(feedback),&output,sizeof(output)))return false;
    output=Envelope{};
    if(feedback.timeLimitReached||feedback.timedCompletion)return false;
    using P=NCG76SphereCycle::Phase;using S=NCG76SpherePath::Stage;
    if(!NCG76SpherePath::Detail::Shape(path)||!feedback.session||!feedback.tick||
       feedback.fault!=NCG76SphereCycle::Fault::None||feedback.completedLayers>path.layerCount)return false;
    const bool stationary=path.geometry.kind==NCG76SphereRuntimeGeometry::Kind::Line&&path.geometry.travelMm==0.0;
    const bool stationaryRecovery=stationary&&feedback.holdRecoveryEnabled&&feedback.holdActive&&
        (NCG76SphereCycle::Detail::IsRecovery(feedback.phase)||feedback.phase==P::HeldAtStart);
    if(stationary&&!stationaryRecovery&&(feedback.nominalPhase!=P::InitialDepth||feedback.nominalLengthMm!=0.0||
        feedback.nominalProgressMm!=0.0||feedback.speedMmS!=0.0||feedback.signedSpeedMmS!=0.0||
        feedback.flushActive||feedback.gapHistoryActive||
        (feedback.phase!=P::Settle&&feedback.phase!=P::HoldSettle&&
         feedback.phase!=P::HeldAtStart&&feedback.phase!=P::Complete)))return false;
    Envelope e{};e.phase=feedback.phase;e.session=feedback.session;e.tick=feedback.tick;
    e.suspended=IsSuspended(feedback);
    e.holdSequence=feedback.holdSequence;e.resumeSequence=feedback.resumeSequence;
    S stage{};
    if(NominalStage(feedback.phase,stage)){
        if(feedback.holdActive||feedback.held||feedback.nominalPhase!=feedback.phase)return false;
        e.role=Role::Nominal;e.atNominalPath=true;
    }else{
        switch(feedback.phase){
        case P::Settle:
            if(feedback.holdActive||feedback.held)return false;
            e.role=Role::NominalSettle;e.atNominalPath=true;break;
        case P::HoldBrake:e.role=Role::NominalBrake;e.atNominalPath=!feedback.gapHistoryActive;break;
        case P::HoldToCenter:case P::HoldToStart:e.role=Role::HoldReturn;break;
        case P::HoldSettle:e.role=Role::HoldOrResumeSettle;break;
        case P::RecoveryBrake:e.role=Role::NominalBrake;break;
        case P::RecoverySettle:e.role=Role::HoldOrResumeSettle;break;
        case P::RecoveryMove:case P::InspectionJog:case P::InspectionJogBrake:
        case P::InspectionJogReturn:e.role=Role::HoldReturn;break;
        case P::HeldAtStart:
            if(!feedback.held)return false;
            e.role=Role::HeldAtStart;break;
        case P::ResumeToCenter:case P::ResumeToAnchor:e.role=Role::Resume;break;
        case P::ResumeBrake:e.role=Role::ResumeBrake;break;
        case P::FlushBrake:
            if(!feedback.flushActive||feedback.holdActive||feedback.held)return false;
            e.role=Role::FlushBrake;e.atNominalPath=true;break;
        case P::FlushSettle:
            if(!feedback.flushActive||feedback.holdActive||feedback.held)return false;
            e.role=Role::FlushSettle;e.checkpointSaved=feedback.flushCheckpointSaved;
            e.atNominalPath=!feedback.flushCheckpointSaved;break;
        case P::FlushLift:case P::FlushReturn:case P::FlushToCenter:case P::FlushFromCenter:
        case P::FlushPathRetreat:case P::FlushPathReturn:
            if(!feedback.flushActive||!feedback.flushCheckpointSaved||feedback.holdActive||feedback.held)return false;
            e.role=Role::Flush;e.checkpointSaved=true;break;
        case P::FlushHoldBrake:
            if(!feedback.flushCheckpointSaved)return false;
            e.role=Role::FlushHoldBrake;break;
        case P::GapHistorySettle:case P::GapHistory:
            if(!feedback.gapHistoryActive||!feedback.gapMotionEnabled||feedback.holdActive||feedback.held||
               feedback.flushActive)return false;
            e.role=feedback.phase==P::GapHistorySettle?Role::GapHistorySettle:Role::GapHistory;
            e.checkpointSaved=true;break;
        case P::Complete:
            if(feedback.holdActive||feedback.held||feedback.completedLayers!=path.layerCount)return false;
            e.role=Role::Complete;e.atNominalPath=true;break;
        default:return false;
        }
        const bool flush=feedback.phase==P::FlushBrake||feedback.phase==P::FlushSettle||
            feedback.phase==P::FlushLift||feedback.phase==P::FlushReturn||feedback.phase==P::FlushToCenter||
            feedback.phase==P::FlushFromCenter||feedback.phase==P::FlushPathRetreat||feedback.phase==P::FlushPathReturn;
        const bool history=feedback.phase==P::GapHistorySettle||feedback.phase==P::GapHistory;
        if(!flush&&!history&&feedback.phase!=P::Settle&&feedback.phase!=P::Complete){
            if(!feedback.holdActive)return false;
            e.checkpointSaved=feedback.phase!=P::HoldBrake;
        }
    }
    if(feedback.gapHistoryActive){e.checkpointSaved=true;e.atNominalPath=false;}
    NCG76SpherePath::Cursor cursor{};
    if(feedback.phase==P::Complete){
        if(NCG76SpherePath::MakeCursorByFraction(path,S::Complete,path.layerCount,0.0,cursor)!=
            NCG76SpherePath::Status::Ok)return false;
        e.layerDepthMm=path.geometry.kind==NCG76SphereRuntimeGeometry::Kind::Sphere?path.geometry.radiusMm:0.0;
    }else{
        const bool sphere=path.geometry.kind==NCG76SphereRuntimeGeometry::Kind::Sphere;
        const bool contour=path.geometry.kind==NCG76SphereRuntimeGeometry::Kind::OrbitContour;
        if(!NominalStage(feedback.nominalPhase,stage))return false;
        const bool contourStage=stage==S::ContourNominal?
            feedback.layerIndex<NCG76OrbitContour::NominalLegCountFor(path.geometry):
            stage==S::ContourFinish?NCG76OrbitContour::IsStraightSideContour(path.geometry)&&
                feedback.layerIndex<NCG76OrbitContour::FinishLegCount(path.geometry):
            feedback.layerIndex==0U&&(stage==S::CircleEntry||stage==S::ReturnCenter||
                (NCG76OrbitContour::IsCircleContour(path.geometry)&&stage==S::Orbit));
        if((sphere?feedback.layerIndex>=path.layerCount:contour?
               (feedback.completedLayers!=0U||!contourStage):
               (feedback.layerIndex!=0U||stage!=S::InitialDepth||feedback.completedLayers!=0U))||
           !std::isfinite(feedback.nominalProgressMm)||!std::isfinite(feedback.nominalLengthMm)||
           (stationary?feedback.nominalLengthMm!=0.0:feedback.nominalLengthMm<=0.0)||feedback.nominalProgressMm<0.0||
           feedback.nominalProgressMm>feedback.nominalLengthMm)return false;
        const double fraction=stationary?0.0:feedback.nominalProgressMm==feedback.nominalLengthMm?1.0:
            feedback.nominalProgressMm/feedback.nominalLengthMm;
        const auto cursorStatus=NCG76SpherePath::MakeCursorByFraction(path,stage,feedback.layerIndex,fraction,cursor);
        if(cursorStatus!=NCG76SpherePath::Status::Ok){
            // A captured nominal stop retains its raw scalar while HOLD/JOG
            // traverses elsewhere, and on the first restored nominal tick.
            // Project only this read-only observation under the same admitted
            // station/arithmetic proof as recovery; do not rewrite feedback,
            // command, checkpoint or the strict canonical cursor APIs.
            const bool zeroSpeed=feedback.speedMmS==0.0&&feedback.signedSpeedMmS==0.0;
            const bool stoppedNominal=zeroSpeed&&!feedback.holdActive&&!feedback.held&&!feedback.flushActive&&
                (NCG76SphereCycle::Detail::IsNominal(feedback.phase)||feedback.phase==P::Settle);
            // E5 can stop one scalar ULP before an endpoint while the physical
            // settle proof is still pending. Observe that stopped nominal
            // point without treating it as an already captured checkpoint.
            const bool stoppedBeforeFlushCapture=zeroSpeed&&feedback.phase==P::FlushSettle&&
                feedback.flushActive&&!feedback.flushCheckpointSaved&&!feedback.holdActive&&!feedback.held&&
                feedback.phaseProgressMm==feedback.nominalProgressMm&&
                feedback.phaseLengthMm==feedback.nominalLengthMm;
            const bool holdPhase=feedback.phase==P::RecoveryBrake||feedback.phase==P::RecoverySettle||
                feedback.phase==P::RecoveryMove||feedback.phase==P::HeldAtStart||
                feedback.phase==P::InspectionJog||feedback.phase==P::InspectionJogBrake||
                feedback.phase==P::InspectionJogReturn;
            const bool capturedHold=feedback.holdRecoveryEnabled&&feedback.holdActive&&
                holdPhase&&((feedback.holdRecoveryPhase>=3U&&feedback.holdRecoveryPhase<=8U)||
                    feedback.holdRecoveryPaused||(feedback.phase==P::RecoverySettle&&zeroSpeed));
            const bool flushPhase=feedback.phase==P::FlushSettle||feedback.phase==P::FlushHoldBrake||
                NCG76SphereCycle::Detail::IsFlushMotionPhase(feedback.phase);
            const bool capturedFlush=feedback.flushActive&&feedback.flushCheckpointSaved&&flushPhase;
            if(cursorStatus!=NCG76SpherePath::Status::PrecisionLoss||feedback.gapHistoryActive||
               !std::isfinite(feedback.speedMmS)||!std::isfinite(feedback.signedSpeedMmS)||
               !(stoppedNominal||stoppedBeforeFlushCapture||capturedHold||capturedFlush))return false;
            const auto retreatStage=NCG76SphereCycle::RecoveryNominalStage(feedback.nominalPhase);
            NCG76SphereRetreat::Span span{};NCG76SpherePath::Point8 canonical{},center{};double station=0.0;
            if(!NCG76SphereRetreat::GetSpan(path.geometry,retreatStage,feedback.layerIndex,span)||
               !NCG76SphereRetreat::Evaluate(path.geometry,span,fraction,canonical,center)||
               !NCG76SphereCycle::ResolveStoppedRecoveryStation(path.geometry,retreatStage,
                   feedback.layerIndex,feedback.nominalProgressMm,feedback.nominalLengthMm,0.0,canonical,station))return false;
            const double projectedFraction=NCG76SphereRetreat::Fraction(span,station);
            if(NCG76SpherePath::MakeCursorByFraction(path,stage,feedback.layerIndex,projectedFraction,cursor)!=
                NCG76SpherePath::Status::Ok)return false;
        }
        if(contour&&stage!=S::ContourNominal)e.layerRadiusMm=path.geometry.radiusMm;
        if(sphere){
            NCG76SphereRuntimeGeometry::Layer layer{};
            if(!NCG76SphereRuntimeGeometry::LayerAt(path.geometry,feedback.layerIndex,layer))return false;
            e.layerRadiusMm=layer.radiusMm;e.layerDepthMm=layer.depthMm;
        }
    }
    if(NCG76SpherePath::Evaluate(path,cursor,e.nominal)!=NCG76SpherePath::Status::Ok)return false;
    e.valid=true;output=e;return true;
}

static_assert(std::is_trivially_copyable<Envelope>::value,"P50 process observation is value data only");
static_assert(!DischargeExecutionReleased,"P50 process observation grants no discharge authority");
}
