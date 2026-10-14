#pragma once
#include "EDMPhysicalServoFlushContract.h"
#include "EDMPhysicalBidirectionalContract.h"

// EDM75: a distinct frozen contract for signed physical-AD work plus timed
// fixture flushing. The complete deep lift fits .03 mm so a -.03 mm work
// point cannot place its top below -.06 mm. No discharge authority is granted.
namespace EDM75
{
using Recipe = EDM57::Recipe;
using Binding = EDM58::Binding;
using BindingError = EDM58::BindingError;
inline bool SameAdmission(const EDM57::RecipeIdentity& a,
    const EDM59::RecipeIdentity& b) noexcept
{ return EDM60::SameAdmission(a,b); }
inline bool SameAdmission(const Recipe& a,const EDM59::Recipe& b) noexcept
{ return EDM75::SameAdmission(a.binding,b.binding); }
inline double LiftHeight(const Recipe& r,bool deep) noexcept
{ return EDM71::LiftHeight(r,deep); }
inline double MaximumLiftHeight(const Recipe& r) noexcept
{ return EDM71::MaximumLiftHeight(r); }
inline bool ValidateRecipe(const Recipe& r) noexcept
{
    if(r.revision!=5U) return false;
    Recipe legacy=r; legacy.revision=4U;
    if(!EDM71::ValidateRecipe(legacy)) return false;
    std::uint64_t height=0,multiplier=0;
    return EDM57::CanonicalMillionths(r.baseHeightMm,height) &&
        EDM57::CanonicalMillionths(r.deepMultiplier,multiplier) &&
        (!r.deepInterval || height*multiplier<=30000000000ULL);
}
inline bool SameCompletion(const EDM57::Feedback& a,const EDM57::Feedback& b) noexcept
{ return EDM71::SameCompletion(a,b); }
inline bool CapturedGeometryValid(const EDM57::Feedback& f,const Recipe& r) noexcept
{
    return std::isfinite(f.workPointMm) && f.workPointMm>=-.03 && f.workPointMm<=.03 &&
        std::isfinite(f.liftHeightMm) && f.liftHeightMm>0.0 &&
        f.liftHeightMm<=EDM75::MaximumLiftHeight(r) &&
        (f.liftHeightMm==r.baseHeightMm ||
            (r.deepInterval && f.liftHeightMm==EDM75::LiftHeight(r,true))) &&
        std::isfinite(f.topMm) && f.topMm>=-.06 &&
        f.topMm==f.workPointMm-f.liftHeightMm;
}
inline bool CompletionValid(const EDM57::Feedback& f,const Recipe& r) noexcept
{
    // Only settled full returns earn cycles. Source expiry can leave at most
    // one additional dwell/approach; it must not credit that partial lift.
    return EDM75::ValidateRecipe(r) &&
        f.phase==static_cast<std::uint32_t>(EDM57::Phase::Terminal) &&
        f.complete && f.cycles>=2U && f.deepCycles==EDM57::ExpectedDeepCycles(r,f.cycles) &&
        f.topDwellCount>=f.cycles && f.topDwellCount-f.cycles<=1U &&
        f.slowApproachCount>=f.cycles && f.slowApproachCount-f.cycles<=1U &&
        f.slowApproachCount<=f.topDwellCount && !f.blockedCount && !f.resumedReturnCount &&
        !f.returnBlocked && !f.persistentLatched && !f.blockedSinceUs && !f.blockedSequence &&
        f.phaseSinceUs && f.destinationMm==0.0 && EDM75::CapturedGeometryValid(f,r);
}
inline bool HoldDestination(const EDM57::Feedback& f,const Recipe& r,double& destination) noexcept
{
    if(!EDM75::ValidateRecipe(r) || f.complete || !f.phaseSinceUs ||
        f.blockedCount || f.resumedReturnCount || f.returnBlocked || f.persistentLatched ||
        f.blockedSinceUs || f.blockedSequence || !std::isfinite(f.destinationMm)) return false;
    const auto phase=static_cast<EDM57::Phase>(f.phase);
    const bool captured=EDM75::CapturedGeometryValid(f,r);
    // Before the first capture there is no invented lift/top/work point.
    const bool empty=f.workPointMm==0.0 && f.topMm==0.0 && f.liftHeightMm==0.0 &&
        !f.cycles && !f.deepCycles && !f.topDwellCount && !f.slowApproachCount;
    bool held=false;
    switch(phase)
    {
    case EDM57::Phase::CaptureWorkPoint:
        held=(captured || empty) && f.destinationMm>=-.03 && f.destinationMm<=.03;
        break;
    case EDM57::Phase::TopSettle:
    case EDM57::Phase::TopDwell:
        held=captured && f.destinationMm==f.topMm;
        break;
    case EDM57::Phase::ApproachBoundarySettle:
        held=captured && f.destinationMm==f.workPointMm-r.finalApproachMm;
        break;
    case EDM57::Phase::ReturnSettle:
        held=captured && f.destinationMm==f.workPointMm;
        break;
    case EDM57::Phase::OriginSettle:
        held=(captured || empty) && f.destinationMm==0.0;
        break;
    default: break;
    }
    if(!held || f.destinationMm<-.06 || f.destinationMm>.03) return false;
    destination=f.destinationMm; return true;
}
inline Binding BindPhysicalBidirectionalFlushRecipe(const EDMRecipe::Controller& controller,
    std::uint32_t expectedProfileId) noexcept
{
    Binding out=EDM71::BindPhysicalFlushRecipe(controller,expectedProfileId);
    if(!out.ready) return out;
    out.recipe.revision=5U;
    std::uint64_t height=0,multiplier=0;
    if(!EDM57::CanonicalMillionths(out.recipe.baseHeightMm,height) ||
        !EDM57::CanonicalMillionths(out.recipe.deepMultiplier,multiplier))
    {out.ready=false;out.error=BindingError::InvalidRecipe;return out;}
    if(out.recipe.deepInterval && height*multiplier>30000000000ULL)
    {out.ready=false;out.error=BindingError::DeepHeightRange;
     out.failedFieldId=out.recipe.binding.fieldIds[4];return out;}
    if(!EDM75::ValidateRecipe(out.recipe))
    {out.ready=false;out.error=BindingError::InvalidRecipe;return out;}
    return out;
}
}
