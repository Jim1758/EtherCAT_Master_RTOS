#pragma once

#include "EDMRecipe.h"
#include "EDMFlushPhysicalRuntime.h"
#include <cstdint>

// EDM58 owner-thread adapter. Resolve actual values by semantic identity, then
// publish only a frozen, bounded value object to the unloaded physical fixture.
// No catalog pointer, editing authority, axis command or discharge permission
// crosses this boundary. NC separately proves current G38/E run authority.
namespace EDM58
{
enum class BindingError : std::uint8_t
{
    None, NotReady, UnsupportedSchema, ProfileMismatch, NoSelection,
    InvalidIdentity, MissingSemantic, DuplicateSemantic, FieldDisabled,
    UnitUnconfirmed, UnitMismatch, Unconfigured, ValueRange, ValuePrecision,
    EngineeringRange, DeepHeightRange, CaptureChanged, InvalidRecipe
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
    case BindingError::DeepHeightRange:return "DEEP_HEIGHT_RANGE";
    case BindingError::CaptureChanged:return "CAPTURE_CHANGED";
    case BindingError::InvalidRecipe:return "INVALID_RECIPE";
    }
    return "UNKNOWN";
}
struct Binding
{
    EDM57::Recipe recipe{};
    BindingError error=BindingError::NotReady;
    std::uint16_t failedFieldId=0U;
    bool ready=false;
};
namespace BindingDetail
{
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
        if(match)
        { out.error=BindingError::DuplicateSemantic;out.failedFieldId=field.id;return false; }
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
inline Binding BindPhysicalFlushRecipe(const EDMRecipe::Controller& controller,
    std::uint32_t expectedProfileId) noexcept
{
    static_assert(EDMRecipe::Scale==1000000LL,"EDM58 E5 fixed-point seconds must be exact microseconds.");
    static_assert(EDMRecipe::RowsPerTable==1000U,"EDM58 runtime E identity bound must match catalog.");
    static_assert(EDMRecipe::MaximumConditionTableId==99999999U,"EDM58 runtime table identity bound must match catalog.");
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

    EDM57::Recipe candidate=EDM57::FixtureRecipe();
    candidate.revision=2U;
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
    const EDMRecipe::Value minimum[]={50000LL,30000LL,50000000LL,0LL,values[3]?1000000LL:0LL};
    const EDMRecipe::Value maximum[]={1000000LL,90000LL,100000000LL,20000000LL,2000000LL};
    for(std::size_t i=0;i<5U;++i)
        if(values[i]<minimum[i] || values[i]>maximum[i] || (i==3U && values[i]%EDMRecipe::Scale!=0LL))
        {out.error=BindingError::EngineeringRange;out.failedFieldId=identity.fieldIds[i];return out;}
    // Checked fixed-point operands stay below 180e9. Reject oversized deep
    // lifts instead of clamping the selected height or expanding axis travel.
    if(values[3] && values[1]*values[4]>120000LL*EDMRecipe::Scale)
    {out.error=BindingError::DeepHeightRange;out.failedFieldId=identity.fieldIds[4];return out;}
    candidate.workTimeUs=static_cast<std::uint64_t>(values[0]);
    candidate.baseHeightMm=static_cast<double>(values[1])/static_cast<double>(EDMRecipe::Scale);
    candidate.jumpSpeedOverridePercent=static_cast<double>(values[2])/static_cast<double>(EDMRecipe::Scale);
    candidate.deepInterval=static_cast<std::uint32_t>(values[3]/EDMRecipe::Scale);
    candidate.deepMultiplier=static_cast<double>(values[4])/static_cast<double>(EDMRecipe::Scale);
    candidate.liftMmMin=candidate.returnMmMin=30.0*candidate.jumpSpeedOverridePercent/100.0;
    candidate.finalApproachMmMin=6.0*candidate.jumpSpeedOverridePercent/100.0;
    if(controller.GetCatalog()!=catalog || !BindingDetail::SameSelection(before,controller.Current()))
    {out.error=BindingError::CaptureChanged;return out;}
    if(!EDM57::ValidateBoundRecipe(candidate)) {out.error=BindingError::InvalidRecipe;return out;}
    out.recipe=candidate;out.error=BindingError::None;out.ready=true;return out;
}
static_assert(sizeof(Binding)<=160U,"EDM58 owner-thread binding must remain compact.");
}
