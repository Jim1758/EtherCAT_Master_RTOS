#pragma once
#include "EDMFlushRecipeBinding.h"
#include "EDMGapServoFlushContract.h"
#include <cmath>
#include <cstdint>
#include <limits>

// EDM71: independent physical-AD fixture contract. It does not expand the
// accepted range of any legacy SIM/bound recipe, nor confer motion authority.
namespace EDM71
{
using Binding = EDM58::Binding;
using BindingError = EDM58::BindingError;
namespace BindingDetail = EDM58::BindingDetail;
inline bool SameAdmission(const EDM57::RecipeIdentity& a,
    const EDM59::RecipeIdentity& b) noexcept
{ return EDM60::SameAdmission(a,b); }
inline bool SameAdmission(const EDM57::Recipe& a,const EDM59::Recipe& b) noexcept
{ return EDM71::SameAdmission(a.binding,b.binding); }
inline double LiftHeight(const EDM57::Recipe& r,bool deep) noexcept
{
    if(!deep) return r.baseHeightMm;
    std::uint64_t height=0,multiplier=0;
    if(!EDM57::CanonicalMillionths(r.baseHeightMm,height) ||
        !EDM57::CanonicalMillionths(r.deepMultiplier,multiplier))
        return (std::numeric_limits<double>::quiet_NaN)();
    return static_cast<double>(height*multiplier)/1000000000000.0;
}
inline double MaximumLiftHeight(const EDM57::Recipe& r) noexcept
{
    if(!r.deepInterval) return r.baseHeightMm;
    const double deep=EDM71::LiftHeight(r,true);
    return !std::isfinite(deep) ? deep : (r.baseHeightMm>deep?r.baseHeightMm:deep);
}
inline bool ValidateRecipe(const EDM57::Recipe& r) noexcept
{
    if(r.revision!=4U || !EDM57::ValidateIdentity(r.binding) || r.cycleCount!=0U ||
        r.workTimeUs<50000ULL || r.workTimeUs>1000000ULL || r.topDwellUs!=100000ULL ||
        !std::isfinite(r.baseHeightMm) || r.baseHeightMm<.005 || r.baseHeightMm>.03 ||
        !std::isfinite(r.jumpSpeedOverridePercent) || r.jumpSpeedOverridePercent<50.0 ||
        r.jumpSpeedOverridePercent>100.0 || r.deepInterval>20U ||
        !std::isfinite(r.deepMultiplier) || r.deepMultiplier<(r.deepInterval?1.0:0.0) ||
        r.deepMultiplier>2.0 || r.finalApproachMm!=.001 || r.workDepthMm!=.03 ||
        r.liftMmMin!=5.0*r.jumpSpeedOverridePercent/100.0 ||
        r.returnMmMin!=5.0*r.jumpSpeedOverridePercent/100.0 ||
        r.finalApproachMmMin!=1.0*r.jumpSpeedOverridePercent/100.0) return false;
    std::uint64_t height=0,multiplier=0,speed=0;
    if(!EDM57::CanonicalMillionths(r.baseHeightMm,height) ||
        !EDM57::CanonicalMillionths(r.deepMultiplier,multiplier) ||
        !EDM57::CanonicalMillionths(r.jumpSpeedOverridePercent,speed)) return false;
    return !r.deepInterval || height*multiplier<=60000000000ULL;
}
inline bool SameCompletion(const EDM57::Feedback& a,const EDM57::Feedback& b) noexcept
{
    return a.phase==b.phase && a.cycles==b.cycles && a.deepCycles==b.deepCycles &&
        a.blockedCount==b.blockedCount && a.resumedReturnCount==b.resumedReturnCount &&
        a.slowApproachCount==b.slowApproachCount && a.topDwellCount==b.topDwellCount &&
        a.phaseSinceUs==b.phaseSinceUs && a.stableSinceUs==b.stableSinceUs &&
        a.blockedSinceUs==b.blockedSinceUs && a.blockedSequence==b.blockedSequence &&
        a.workPointMm==b.workPointMm && a.topMm==b.topMm &&
        a.destinationMm==b.destinationMm && a.liftHeightMm==b.liftHeightMm &&
        a.returnBlocked==b.returnBlocked && a.persistentLatched==b.persistentLatched &&
        a.complete==b.complete;
}
inline bool CompletionValid(const EDM57::Feedback& f,const EDM57::Recipe& r) noexcept
{
    // The ten-second window may interrupt one lift; only a settled return
    // credits a cycle. An unfinished dwell/approach must never credit PASS.
    return EDM71::ValidateRecipe(r) && f.phase==static_cast<std::uint32_t>(EDM57::Phase::Terminal) &&
        f.complete && f.cycles>0U && f.deepCycles==EDM57::ExpectedDeepCycles(r,f.cycles) &&
        f.topDwellCount>=f.cycles && f.topDwellCount-f.cycles<=1U &&
        f.slowApproachCount>=f.cycles && f.slowApproachCount-f.cycles<=1U &&
        f.slowApproachCount<=f.topDwellCount && !f.blockedCount && !f.resumedReturnCount &&
        !f.returnBlocked && !f.persistentLatched && !f.blockedSinceUs && !f.blockedSequence &&
        f.phaseSinceUs && std::isfinite(f.workPointMm) && f.workPointMm>=0.0 && f.workPointMm<=.03 &&
        std::isfinite(f.topMm) && f.topMm>=-.06 && f.topMm==f.workPointMm-f.liftHeightMm &&
        f.destinationMm==0.0 && std::isfinite(f.liftHeightMm) && f.liftHeightMm>0.0 &&
        f.liftHeightMm<=EDM71::MaximumLiftHeight(r) &&
        (f.liftHeightMm==r.baseHeightMm || (r.deepInterval && f.liftHeightMm==EDM71::LiftHeight(r,true)));
}
inline Binding BindPhysicalFlushRecipe(const EDMRecipe::Controller& controller,
    std::uint32_t expectedProfileId) noexcept
{
    static_assert(EDMRecipe::Scale==1000000LL,"EDM71 E5 fixed-point seconds must be exact microseconds.");
    static_assert(EDMRecipe::RowsPerTable==1000U,"EDM71 runtime E identity bound must match catalog.");
    static_assert(EDMRecipe::MaximumConditionTableId==99999999U,"EDM71 runtime table identity bound must match catalog.");
    Binding out{};
    const auto before=controller.Current();
    const auto* catalog=controller.GetCatalog();
    if(!catalog || !before.catalogReady) return out;
    if(catalog->schemaVersion!=2U || catalog->mode!=EDMRecipe::Mode::ShadowOnly)
    {out.error=BindingError::UnsupportedSchema;return out;}
    if(!expectedProfileId || catalog->profileId!=expectedProfileId || before.profileId!=expectedProfileId)
    {out.error=BindingError::ProfileMismatch;return out;}
    if(!before.tableSelected || !before.rowSelected)
    {out.error=BindingError::NoSelection;return out;}
    if(!before.generation || !before.tableId || before.tableId>EDMRecipe::MaximumConditionTableId ||
        !before.tableRevision || !before.eCode || before.eCode>EDMRecipe::RowsPerTable ||
        !catalog->catalogRevision || !catalog->definitionRevision ||
        before.catalogRevision!=catalog->catalogRevision || before.definitionRevision!=catalog->definitionRevision ||
        catalog->fields.empty() || catalog->fields.size()>EDMRecipe::MaximumFields ||
        before.fieldCount!=catalog->fields.size())
    {out.error=BindingError::InvalidIdentity;return out;}
    const EDMRecipe::Table* table=nullptr;
    for(const auto& candidate:catalog->tables)
        if(candidate.id==before.tableId)
        {
            if(table) {out.error=BindingError::InvalidIdentity;return out;}
            table=&candidate;
        }
    if(!table || table->revision!=before.tableRevision || !table->rowPresent[before.eCode-1U])
    {out.error=BindingError::InvalidIdentity;return out;}

    EDM57::Recipe candidate{};
    candidate.revision=4U; candidate.cycleCount=0U;
    candidate.topDwellUs=100000ULL; candidate.workDepthMm=.03;
    candidate.finalApproachMm=.001;
    auto& identity=candidate.binding;
    identity.catalogRevision=before.catalogRevision;identity.definitionRevision=before.definitionRevision;
    identity.profileId=before.profileId;identity.tableId=before.tableId;identity.tableRevision=before.tableRevision;
    identity.eCode=before.eCode;identity.generation=before.generation;
    const char* semantics[]={"DischargeWorkTime","JumpHeight","JumpSpeedOverride",
        "DeepFlushCycleInterval","DeepFlushHeightMultiplier"};
    const char* units[]={"s","mm","%","cycle","x"};
    EDMRecipe::Value values[5]{};
    for(std::size_t i=0;i<5U;++i)
        if(!BindingDetail::Read(controller,*catalog,semantics[i],units[i],values[i],identity.fieldIds[i],out)) return out;
    const EDMRecipe::Value minimum[]={50000LL,5000LL,50000000LL,0LL,values[3]?1000000LL:0LL};
    const EDMRecipe::Value maximum[]={1000000LL,30000LL,100000000LL,20000000LL,2000000LL};
    for(std::size_t i=0;i<5U;++i)
        if(values[i]<minimum[i] || values[i]>maximum[i] || (i==3U && values[i]%EDMRecipe::Scale!=0LL))
        {out.error=BindingError::EngineeringRange;out.failedFieldId=identity.fieldIds[i];return out;}
    // Checked fixed-point operands stay below 60e9. Reject oversized deep
    // lifts instead of clamping the selected height or expanding axis travel.
    if(values[3] && values[1]*values[4]>60000LL*EDMRecipe::Scale)
    {out.error=BindingError::DeepHeightRange;out.failedFieldId=identity.fieldIds[4];return out;}
    candidate.workTimeUs=static_cast<std::uint64_t>(values[0]);
    candidate.baseHeightMm=static_cast<double>(values[1])/static_cast<double>(EDMRecipe::Scale);
    candidate.jumpSpeedOverridePercent=static_cast<double>(values[2])/static_cast<double>(EDMRecipe::Scale);
    candidate.deepInterval=static_cast<std::uint32_t>(values[3]/EDMRecipe::Scale);
    candidate.deepMultiplier=static_cast<double>(values[4])/static_cast<double>(EDMRecipe::Scale);
    candidate.liftMmMin=candidate.returnMmMin=5.0*candidate.jumpSpeedOverridePercent/100.0;
    candidate.finalApproachMmMin=1.0*candidate.jumpSpeedOverridePercent/100.0;
    if(controller.GetCatalog()!=catalog || !BindingDetail::SameSelection(before,controller.Current()))
    {out.error=BindingError::CaptureChanged;return out;}
    if(!EDM71::ValidateRecipe(candidate)) {out.error=BindingError::InvalidRecipe;return out;}
    out.recipe=candidate;out.error=BindingError::None;out.ready=true;return out;
}
}
