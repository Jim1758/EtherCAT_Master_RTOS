#pragma once

#include "EDMRecipe.h"
#include "EDMProcessProfile.h"
#include <cmath>
#include <cstdint>

// EDM59 owner-thread binding for the explicitly selected unloaded SIM fixture.
// These are immutable values, not permission to enable discharge or install
// axis PID gains. The RT adapter retains its own short-circuit safety policy.
namespace EDM59
{
struct RecipeIdentity
{
    std::uint32_t catalogRevision=0, definitionRevision=0, profileId=0;
    std::uint32_t tableId=0, tableRevision=0, processRevision=0;
    std::uint16_t eCode=0, fieldIds[2]{};
    std::uint64_t generation=0, tuningGeneration=0;
};
struct Recipe
{
    EDMGapServo::Profile curve{};
    double referenceV=0.0, overridePercent=0.0;
    RecipeIdentity binding{};
};
enum class BindingError : std::uint8_t
{
    None, NotReady, UnsupportedSchema, ProfileMismatch, NoSelection,
    InvalidIdentity, MissingSemantic, DuplicateSemantic, FieldDisabled,
    UnitUnconfirmed, UnitMismatch, Unconfigured, ValueRange, ValuePrecision,
    EngineeringRange, InvalidProcessProfile, CurveRange, CurveLiveness,
    CaptureChanged, InvalidRecipe
};
inline const char* BindingErrorName(BindingError e) noexcept
{
    switch(e)
    {
    case BindingError::None:return "NONE";
    case BindingError::NotReady:return "NOT_READY";
    case BindingError::UnsupportedSchema:return "UNSUPPORTED_SCHEMA";
    case BindingError::ProfileMismatch:return "PROFILE_MISMATCH";
    case BindingError::NoSelection:return "NO_SELECTION";
    case BindingError::InvalidIdentity:return "INVALID_IDENTITY";
    case BindingError::MissingSemantic:return "MISSING_SEMANTIC";
    case BindingError::DuplicateSemantic:return "DUPLICATE_SEMANTIC";
    case BindingError::FieldDisabled:return "FIELD_DISABLED";
    case BindingError::UnitUnconfirmed:return "UNIT_UNCONFIRMED";
    case BindingError::UnitMismatch:return "UNIT_MISMATCH";
    case BindingError::Unconfigured:return "UNCONFIGURED";
    case BindingError::ValueRange:return "VALUE_RANGE";
    case BindingError::ValuePrecision:return "VALUE_PRECISION";
    case BindingError::EngineeringRange:return "ENGINEERING_RANGE";
    case BindingError::InvalidProcessProfile:return "INVALID_PROCESS_PROFILE";
    case BindingError::CurveRange:return "CURVE_RANGE";
    case BindingError::CurveLiveness:return "CURVE_LIVENESS";
    case BindingError::CaptureChanged:return "CAPTURE_CHANGED";
    case BindingError::InvalidRecipe:return "INVALID_RECIPE";
    }
    return "UNKNOWN";
}
struct Binding
{
    Recipe recipe{};
    BindingError error=BindingError::NotReady;
    std::uint16_t failedFieldId=0U;
    bool ready=false;
};
inline bool SameIdentity(const RecipeIdentity&a,const RecipeIdentity&b) noexcept
{
    return a.catalogRevision==b.catalogRevision && a.definitionRevision==b.definitionRevision &&
        a.profileId==b.profileId && a.tableId==b.tableId && a.tableRevision==b.tableRevision &&
        a.processRevision==b.processRevision && a.eCode==b.eCode &&
        a.fieldIds[0]==b.fieldIds[0] && a.fieldIds[1]==b.fieldIds[1] &&
        a.generation==b.generation && a.tuningGeneration==b.tuningGeneration;
}
inline bool SameCurve(const EDMGapServo::Profile&a,const EDMGapServo::Profile&b) noexcept
{
    for(unsigned i=0;i<3U;++i)
        if(a.positiveGain[i]!=b.positiveGain[i] || a.negativeGain[i]!=b.negativeGain[i]) return false;
    for(unsigned i=0;i<2U;++i)
        if(a.positiveBreakV[i]!=b.positiveBreakV[i] || a.negativeBreakV[i]!=b.negativeBreakV[i]) return false;
    return a.revision==b.revision && a.cuttingScaleMmPerVoltMin==b.cuttingScaleMmPerVoltMin &&
        a.maxFeedMmPerMin==b.maxFeedMmPerMin && a.maxRetreatMmPerMin==b.maxRetreatMmPerMin &&
        a.limitsConfirmed==b.limitsConfirmed && a.deadbandV==b.deadbandV &&
        a.shortIdleV==b.shortIdleV && a.shortMachiningV==b.shortMachiningV &&
        a.shortIdleEnabled==b.shortIdleEnabled && a.shortMachiningEnabled==b.shortMachiningEnabled &&
        a.shortEnterMs==b.shortEnterMs && a.shortExitMs==b.shortExitMs &&
        a.shortHysteresisV==b.shortHysteresisV && a.shortRetreatMmPerMin==b.shortRetreatMmPerMin;
}
inline bool SameRecipe(const Recipe&a,const Recipe&b) noexcept
{
    return SameCurve(a.curve,b.curve) && a.referenceV==b.referenceV &&
        a.overridePercent==b.overridePercent && SameIdentity(a.binding,b.binding);
}
inline Recipe EmptyRecipe() noexcept { return Recipe{}; }
inline bool IsEmptyRecipe(const Recipe&r) noexcept { return SameRecipe(r,EmptyRecipe()); }
inline bool ValidateIdentity(const RecipeIdentity&i) noexcept
{
    return i.catalogRevision && i.definitionRevision && i.profileId && i.tableId &&
        i.tableId<=EDMRecipe::MaximumConditionTableId && i.tableRevision && i.processRevision &&
        i.eCode && i.eCode<=EDMRecipe::RowsPerTable && i.fieldIds[0] && i.fieldIds[1] &&
        i.fieldIds[0]!=i.fieldIds[1] && i.generation && i.tuningGeneration;
}
namespace BindingDetail
{
inline bool CanonicalMillionths(double value) noexcept
{
    if(!std::isfinite(value) || value<0.0 || value>100.0) return false;
    const auto units=static_cast<std::uint64_t>(std::floor(value*1000000.0+.5));
    return static_cast<double>(units)/1000000.0==value;
}
inline BindingError CheckCurve(const Recipe&r) noexcept
{
    const auto& curve=r.curve;
    if(!EDMGapServo::ValidateProfile(curve) || curve.revision!=r.binding.processRevision ||
        curve.maxFeedMmPerMin>5.0 || curve.maxRetreatMmPerMin>5.0 ||
        curve.shortRetreatMmPerMin<=0.0 || curve.shortRetreatMmPerMin>curve.maxRetreatMmPerMin ||
        // Fixed RT short threshold + hysteresis must lie below the deadband.
        30.0>=r.referenceV-curve.deadbandV)
        return BindingError::CurveRange;
    // Endpoint checks use full override so a lower E8 cannot conceal invalid
    // curve arithmetic. The sample source remains bounded to 0..300 V in RT.
    if(!EDMGapServo::Evaluate(curve,0.0,r.referenceV,100.0).valid ||
        !EDMGapServo::Evaluate(curve,300.0,r.referenceV,100.0).valid)
        return BindingError::CurveRange;
    const auto advance=EDMGapServo::Evaluate(curve,80.0,r.referenceV,r.overridePercent);
    const auto returning=EDMGapServo::Evaluate(curve,50.0,r.referenceV,r.overridePercent);
    const auto low=EDMGapServo::Evaluate(curve,20.0,r.referenceV,r.overridePercent);
    const auto retreat=EDMGapServo::EvaluateShortRetreat(curve,low);
    // The existing short-retreat evaluator returns the configured fixed speed;
    // EDM55 applies E8 exactly once to that branch. Match its actual demand.
    const double shortSpeed=retreat.signedSpeedMmPerMin*(r.overridePercent/100.0);
    if(!advance.valid || !returning.valid || !low.valid || !retreat.valid ||
        advance.limitedSpeedMmPerMin<1.25 || returning.limitedSpeedMmPerMin> -1.25 ||
        !std::isfinite(shortSpeed) || shortSpeed> -.25)
        return BindingError::CurveLiveness;
    return BindingError::None;
}
inline bool LabelEquals(const char* value,std::size_t size,const char* expected) noexcept
{
    for(std::size_t i=0;i<size;++i)
    {
        if(value[i]!=expected[i]) return false;
        if(value[i]=='\0') return true;
    }
    return false;
}
inline bool SameSelection(const EDMRecipe::Summary&a,const EDMRecipe::Summary&b) noexcept
{
    return a.catalogReady==b.catalogReady && a.tableSelected==b.tableSelected &&
        a.rowSelected==b.rowSelected && a.catalogRevision==b.catalogRevision &&
        a.definitionRevision==b.definitionRevision && a.profileId==b.profileId &&
        a.tableId==b.tableId && a.tableRevision==b.tableRevision && a.eCode==b.eCode &&
        a.generation==b.generation && a.fieldCount==b.fieldCount && a.configuredCount==b.configuredCount;
}
inline bool Read(const EDMRecipe::Controller& controller,const EDMRecipe::Catalog& catalog,
    const char* semantic,const char* unit,EDMRecipe::Value& value,std::uint16_t& fieldId,Binding& out) noexcept
{
    const EDMRecipe::FieldDefinition* match=nullptr;
    for(const auto& field:catalog.fields)
    {
        if(!LabelEquals(field.semanticKey,sizeof(field.semanticKey),semantic)) continue;
        if(match) {out.error=BindingError::DuplicateSemantic;out.failedFieldId=field.id;return false;}
        match=&field;
    }
    if(!match) {out.error=BindingError::MissingSemantic;return false;}
    out.failedFieldId=match->id;
    if(!match->id) {out.error=BindingError::InvalidIdentity;return false;}
    if(!match->enabled) {out.error=BindingError::FieldDisabled;return false;}
    if(!match->unitConfirmed) {out.error=BindingError::UnitUnconfirmed;return false;}
    if(!LabelEquals(match->unit,sizeof(match->unit),unit)) {out.error=BindingError::UnitMismatch;return false;}
    EDMRecipe::ParameterSnapshot p{};
    if(!controller.ReadField(match->id,p) || p.fieldId!=match->id || !p.configured ||
        (p.rowKind!=EDMRecipe::CellKind::Direct && p.rowKind!=EDMRecipe::CellKind::Stage))
    {out.error=BindingError::Unconfigured;return false;}
    value=p.actualValue;
    if(value<match->minimum || value>match->maximum) {out.error=BindingError::ValueRange;return false;}
    if(!EDMRecipe::ValueMatchesPrecision(value,match->decimals)) {out.error=BindingError::ValuePrecision;return false;}
    fieldId=match->id;out.failedFieldId=0U;return true;
}
}
inline bool ValidateRecipe(const Recipe&r) noexcept
{
    return ValidateIdentity(r.binding) && BindingDetail::CanonicalMillionths(r.referenceV) &&
        BindingDetail::CanonicalMillionths(r.overridePercent) && r.referenceV>=55.0 && r.referenceV<=65.0 &&
        r.overridePercent>=50.0 && r.overridePercent<=100.0 &&
        BindingDetail::CheckCurve(r)==BindingError::None;
}
inline Binding BindPhysicalServoRecipe(const EDMRecipe::Controller& controller,
    const EDM20::ProcessProfile& installedProcessProfile,std::uint64_t tuningGeneration) noexcept
{
    static_assert(EDMRecipe::Scale==1000000LL,"EDM59 E8/E9 use fixed-point millionths.");
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
    if(referenceValue<55000000LL || referenceValue>65000000LL)
    {out.error=BindingError::EngineeringRange;out.failedFieldId=identity.fieldIds[1];return out;}
    candidate.overridePercent=static_cast<double>(overrideValue)/static_cast<double>(EDMRecipe::Scale);
    candidate.referenceV=static_cast<double>(referenceValue)/static_cast<double>(EDMRecipe::Scale);
    out.error=BindingDetail::CheckCurve(candidate);
    if(out.error!=BindingError::None) return out;
    if(controller.GetCatalog()!=catalog || !BindingDetail::SameSelection(before,controller.Current()))
    {out.error=BindingError::CaptureChanged;return out;}
    if(!ValidateRecipe(candidate)) {out.error=BindingError::InvalidRecipe;return out;}
    out.recipe=candidate;out.error=BindingError::None;out.ready=true;return out;
}
static_assert(sizeof(Recipe)<=272U,"EDM59 frozen servo recipe must remain bounded.");
static_assert(sizeof(Binding)<=288U,"EDM59 owner-thread binding must remain bounded.");
}
