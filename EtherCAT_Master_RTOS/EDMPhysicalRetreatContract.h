#pragma once

#include "EDMGapServoRecipeBinding.h"
#include <cmath>
#include <cstdint>

// EDM72: independent frozen binding for the unloaded physical-AD negative
// error fixture. A raised E9 makes the genuine input request bounded retreat;
// this is not short-circuit recovery or permission to enable discharge.
namespace EDM72
{
using Recipe = EDM59::Recipe;
using Binding = EDM59::Binding;
using BindingError = EDM59::BindingError;
constexpr double RetreatDepthMm=.03;
constexpr unsigned RetreatPhase=7U, HoldPhase=8U;
namespace BindingDetail
{
using EDM59::BindingDetail::CanonicalMillionths;
using EDM59::BindingDetail::Read;
using EDM59::BindingDetail::SameSelection;
inline BindingError CheckCurve(const Recipe& r) noexcept
{
    const auto& curve=r.curve;
    if(!EDMGapServo::ValidateProfile(curve) || curve.revision!=r.binding.processRevision ||
        curve.maxFeedMmPerMin>5.0 || curve.maxRetreatMmPerMin>5.0 ||
        curve.shortRetreatMmPerMin<=0.0 || curve.shortRetreatMmPerMin>curve.maxRetreatMmPerMin ||
        30.0>=r.referenceV-curve.deadbandV)
        return BindingError::CurveRange;
    if(!EDMGapServo::Evaluate(curve,0.0,r.referenceV,100.0).valid ||
        !EDMGapServo::Evaluate(curve,300.0,r.referenceV,100.0).valid)
        return BindingError::CurveRange;
    const auto advance=EDMGapServo::Evaluate(curve,100.0,r.referenceV,r.overridePercent);
    const auto returning=EDMGapServo::Evaluate(curve,75.0,r.referenceV,r.overridePercent);
    const auto low=EDMGapServo::Evaluate(curve,20.0,r.referenceV,r.overridePercent);
    const auto retreat=EDMGapServo::EvaluateShortRetreat(curve,low);
    const double shortSpeed=retreat.signedSpeedMmPerMin*(r.overridePercent/100.0);
    if(!advance.valid || !returning.valid || !low.valid || !retreat.valid ||
        advance.limitedSpeedMmPerMin<1.25 || returning.limitedSpeedMmPerMin>-.5 ||
        !std::isfinite(shortSpeed) || shortSpeed>-.25)
        return BindingError::CurveLiveness;
    return BindingError::None;
}
}
inline bool ValidateRecipe(const Recipe& r) noexcept
{
    return EDM59::ValidateIdentity(r.binding) && BindingDetail::CanonicalMillionths(r.referenceV) &&
        BindingDetail::CanonicalMillionths(r.overridePercent) && r.referenceV>=80.0 && r.referenceV<=85.0 &&
        r.overridePercent>=50.0 && r.overridePercent<=100.0 &&
        BindingDetail::CheckCurve(r)==BindingError::None;
}
inline bool NegativeInputAllowed(const Recipe& r,double voltage) noexcept
{
    return std::isfinite(voltage) && voltage>30.0 && voltage<r.referenceV-r.curve.deadbandV;
}
inline Binding BindPhysicalRetreatRecipe(const EDMRecipe::Controller& controller,
    const EDM20::ProcessProfile& installedProcessProfile,std::uint64_t tuningGeneration) noexcept
{
    static_assert(EDMRecipe::Scale==1000000LL,"EDM72 E8/E9 use fixed-point millionths.");
    Binding out{};
    const auto before=controller.Current();
    const auto* catalog=controller.GetCatalog();
    if(!catalog || !before.catalogReady) return out;
    if(catalog->schemaVersion!=2U || catalog->mode!=EDMRecipe::Mode::ShadowOnly)
    {out.error=BindingError::UnsupportedSchema;return out;}
    if(!EDM20::ValidateProcessProfile(installedProcessProfile) || !tuningGeneration)
    {out.error=BindingError::InvalidProcessProfile;return out;}
    if(catalog->profileId!=installedProcessProfile.machineProfileId ||
        before.profileId!=installedProcessProfile.machineProfileId)
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
    Recipe candidate{};
    candidate.curve=installedProcessProfile.servo;
    auto& identity=candidate.binding;
    identity.catalogRevision=before.catalogRevision;identity.definitionRevision=before.definitionRevision;
    identity.profileId=before.profileId;identity.tableId=before.tableId;identity.tableRevision=before.tableRevision;
    identity.eCode=before.eCode;identity.generation=before.generation;
    identity.processRevision=installedProcessProfile.revision;identity.tuningGeneration=tuningGeneration;
    EDMRecipe::Value overrideValue=0LL,referenceValue=0LL;
    if(!BindingDetail::Read(controller,*catalog,"MachiningServoOverride","%",overrideValue,identity.fieldIds[0],out) ||
        !BindingDetail::Read(controller,*catalog,"GapVoltageSetpoint","V",referenceValue,identity.fieldIds[1],out)) return out;
    if(overrideValue<50000000LL || overrideValue>100000000LL)
    {out.error=BindingError::EngineeringRange;out.failedFieldId=identity.fieldIds[0];return out;}
    if(referenceValue<80000000LL || referenceValue>85000000LL)
    {out.error=BindingError::EngineeringRange;out.failedFieldId=identity.fieldIds[1];return out;}
    candidate.overridePercent=static_cast<double>(overrideValue)/static_cast<double>(EDMRecipe::Scale);
    candidate.referenceV=static_cast<double>(referenceValue)/static_cast<double>(EDMRecipe::Scale);
    out.error=BindingDetail::CheckCurve(candidate);
    if(out.error!=BindingError::None) return out;
    if(controller.GetCatalog()!=catalog || !BindingDetail::SameSelection(before,controller.Current()))
    {out.error=BindingError::CaptureChanged;return out;}
    if(!EDM72::ValidateRecipe(candidate)) {out.error=BindingError::InvalidRecipe;return out;}
    out.recipe=candidate;out.error=BindingError::None;out.ready=true;return out;
}
}
