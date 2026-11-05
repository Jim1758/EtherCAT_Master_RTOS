#pragma once
#include "EDMSpecialGapGainDefinition.h"
#include "EDMGapServo.h"
#include <array>
#include <cstring>

// EDM368 NC-owner capture only. No mutable catalog or process pointer crosses
// into RT; Apply changes only the six existing immutable gap-curve gains.
namespace EDMSpecialGapGainBinding
{
enum class Reason : std::uint8_t
{ None, NotReady, Identity, Definition, Unconfigured, ValueRange, ValuePrecision, CaptureChanged };
inline const char* ReasonName(Reason r) noexcept
{
    switch(r){case Reason::None:return "NONE";case Reason::NotReady:return "NOT_READY";
    case Reason::Identity:return "IDENTITY";case Reason::Definition:return "DEFINITION";
    case Reason::Unconfigured:return "UNCONFIGURED";case Reason::ValueRange:return "VALUE_RANGE";
    case Reason::ValuePrecision:return "VALUE_PRECISION";case Reason::CaptureChanged:return "CAPTURE_CHANGED";
    default:return "UNKNOWN";}
}
template<class Error> inline Error ErrorCode(Reason reason) noexcept
{
    switch(reason){case Reason::None:return Error::None;case Reason::NotReady:return Error::NotReady;
    case Reason::Unconfigured:return Error::Unconfigured;case Reason::ValueRange:return Error::ValueRange;
    case Reason::ValuePrecision:return Error::ValuePrecision;default:return Error::InvalidIdentity;}
}
struct Binding
{
    EDMRecipe::Summary selection{};
    const EDMRecipe::Catalog* catalog=nullptr; // NC owner only.
    std::array<EDMRecipe::ParameterSnapshot,EDMSpecialGapGainDefinition::FieldCount> snapshots{};
    std::array<EDMRecipe::Value,EDMSpecialGapGainDefinition::FieldCount> values{};
    EDMSpecialGapGainDefinition::Group group=EDMSpecialGapGainDefinition::Group::Invalid;
    Reason reason=Reason::NotReady;
    std::uint16_t failedFieldId=0U;
    bool ready=false,enabled=false,legacyOff=false;
};
inline bool SameSelection(const EDMRecipe::Summary& a,const EDMRecipe::Summary& b) noexcept
{
    return a.catalogReady==b.catalogReady&&a.tableSelected==b.tableSelected&&a.rowSelected==b.rowSelected&&
        a.catalogRevision==b.catalogRevision&&a.definitionRevision==b.definitionRevision&&a.profileId==b.profileId&&
        a.tableId==b.tableId&&a.tableRevision==b.tableRevision&&a.eCode==b.eCode&&
        a.generation==b.generation&&a.fieldCount==b.fieldCount&&a.configuredCount==b.configuredCount;
}
inline bool SameProfile(const EDMGapServo::Profile& a,const EDMGapServo::Profile& b) noexcept
{
    const auto same=[](double x,double y) noexcept {return std::memcmp(&x,&y,sizeof(double))==0;};
    if(a.revision!=b.revision||a.limitsConfirmed!=b.limitsConfirmed||
       a.shortIdleEnabled!=b.shortIdleEnabled||a.shortMachiningEnabled!=b.shortMachiningEnabled||
       a.shortEnterMs!=b.shortEnterMs||a.shortExitMs!=b.shortExitMs)return false;
    for(unsigned i=0U;i<3U;++i)if(!same(a.positiveGain[i],b.positiveGain[i])||!same(a.negativeGain[i],b.negativeGain[i]))return false;
    for(unsigned i=0U;i<2U;++i)if(!same(a.positiveBreakV[i],b.positiveBreakV[i])||!same(a.negativeBreakV[i],b.negativeBreakV[i]))return false;
    return same(a.cuttingScaleMmPerVoltMin,b.cuttingScaleMmPerVoltMin)&&same(a.maxFeedMmPerMin,b.maxFeedMmPerMin)&&
        same(a.maxRetreatMmPerMin,b.maxRetreatMmPerMin)&&same(a.deadbandV,b.deadbandV)&&
        same(a.shortIdleV,b.shortIdleV)&&same(a.shortMachiningV,b.shortMachiningV)&&
        same(a.shortHysteresisV,b.shortHysteresisV)&&same(a.shortRetreatMmPerMin,b.shortRetreatMmPerMin);
}
inline bool SameSnapshot(const EDMRecipe::ParameterSnapshot& a,const EDMRecipe::ParameterSnapshot& b) noexcept
{
    return a.fieldId==b.fieldId&&a.configured==b.configured&&a.rowKind==b.rowKind&&
        a.rowStageId==b.rowStageId&&a.rowBaseValue==b.rowBaseValue&&a.stageId==b.stageId&&
        a.baseValue==b.baseValue&&a.actualValue==b.actualValue&&a.overridePresent==b.overridePresent&&a.modified==b.modified;
}
inline bool Same(const Binding& a,const Binding& b) noexcept
{
    if(a.ready!=b.ready||a.enabled!=b.enabled||a.legacyOff!=b.legacyOff||a.reason!=b.reason||
       a.failedFieldId!=b.failedFieldId||a.catalog!=b.catalog||a.group!=b.group||a.values!=b.values||
       !SameSelection(a.selection,b.selection))return false;
    for(std::size_t i=0U;i<a.snapshots.size();++i)if(!SameSnapshot(a.snapshots[i],b.snapshots[i]))return false;
    return true;
}
inline Binding Capture(const EDMRecipe::Controller& controller,std::uint32_t expectedProfileId) noexcept
{
    namespace D=EDMSpecialGapGainDefinition;
    Binding out{};const auto before=controller.Current();const auto* catalog=controller.GetCatalog();
    if(!catalog||!before.catalogReady||!before.tableSelected||!before.rowSelected)return out;
    if(catalog->schemaVersion!=2U||!expectedProfileId||catalog->profileId!=expectedProfileId||
       before.profileId!=expectedProfileId||!before.generation||!before.catalogRevision||!before.definitionRevision||
       before.catalogRevision!=catalog->catalogRevision||before.definitionRevision!=catalog->definitionRevision||
       !before.tableId||before.tableId>EDMRecipe::MaximumConditionTableId||!before.tableRevision||
       !before.eCode||before.eCode>EDMRecipe::RowsPerTable||catalog->fields.empty()||
       catalog->fields.size()>EDMRecipe::MaximumFields||before.fieldCount!=catalog->fields.size())
    {out.reason=Reason::Identity;return out;}
    const EDMRecipe::Table* table=nullptr;
    for(const auto& candidate:catalog->tables)if(candidate.id==before.tableId){
        if(table){out.reason=Reason::Identity;return out;}table=&candidate;
    }
    if(!table||table->revision!=before.tableRevision||!table->rowPresent[before.eCode-1U])
    {out.reason=Reason::Identity;return out;}
    std::array<std::size_t,D::FieldCount> indices{};
    out.group=D::Inspect(*catalog,indices);
    if(out.group==D::Group::Invalid){out.reason=Reason::Definition;out.failedFieldId=D::SwitchFieldId;return out;}
    // A duplicate semantic elsewhere cannot masquerade as a different spare id.
    for(const auto& field:catalog->fields)for(std::uint16_t id=D::FirstFieldId;id<=D::LastFieldId;++id)
        if(field.id!=id&&std::strncmp(field.semanticKey,D::Semantic(id),sizeof(field.semanticKey))==0)
        {out.reason=Reason::Definition;out.failedFieldId=field.id;return out;}
    for(std::size_t i=0U;i<D::FieldCount;++i){
        if(indices[i]==catalog->fields.size())continue;
        const auto id=static_cast<std::uint16_t>(D::FirstFieldId+i);
        if(controller.ReadField(id,out.snapshots[i]))out.values[i]=out.snapshots[i].actualValue;
        if(out.group==D::Group::Legacy&&out.snapshots[i].configured&&
           (out.snapshots[i].rowKind!=EDMRecipe::CellKind::Direct||out.snapshots[i].actualValue!=0LL||
            out.snapshots[i].rowBaseValue!=0LL||out.snapshots[i].baseValue!=0LL||out.snapshots[i].rowStageId||
            out.snapshots[i].stageId||out.snapshots[i].overridePresent))
        {out.reason=Reason::ValueRange;out.failedFieldId=id;return out;}
    }
    out.legacyOff=out.group==D::Group::Missing||out.group==D::Group::Legacy;
    if(!out.legacyOff){
        const auto& sw=out.snapshots[0];out.failedFieldId=D::SwitchFieldId;
        if(!sw.configured||sw.rowKind!=EDMRecipe::CellKind::Direct){out.reason=Reason::Unconfigured;return out;}
        if(sw.actualValue<0LL||sw.actualValue>EDMRecipe::Scale){out.reason=Reason::ValueRange;return out;}
        if(sw.actualValue%EDMRecipe::Scale!=0LL){out.reason=Reason::ValuePrecision;return out;}
        out.enabled=sw.actualValue==EDMRecipe::Scale;
        if(out.enabled)for(std::size_t i=1U;i<D::FieldCount;++i){
            const auto& value=out.snapshots[i];out.failedFieldId=static_cast<std::uint16_t>(D::FirstFieldId+i);
            if(!value.configured||value.rowKind!=EDMRecipe::CellKind::Direct){out.reason=Reason::Unconfigured;return out;}
            if(value.actualValue<0LL||value.actualValue>D::Maximum(out.failedFieldId)){out.reason=Reason::ValueRange;return out;}
        }
    }
    if(controller.GetCatalog()!=catalog||!SameSelection(before,controller.Current()))
    {out.reason=Reason::CaptureChanged;return out;}
    out.catalog=catalog;out.selection=before;out.ready=true;out.reason=Reason::None;out.failedFieldId=0U;return out;
}
inline bool Apply(const Binding& binding,EDMGapServo::Profile& curve) noexcept
{
    if(!binding.ready)return false;
    if(binding.enabled){
        if(binding.group!=EDMSpecialGapGainDefinition::Group::Typed||binding.values[0]!=EDMRecipe::Scale)return false;
        for(unsigned i=1U;i<7U;++i)if(binding.values[i]<0LL||
            binding.values[i]>EDMSpecialGapGainDefinition::Maximum(static_cast<std::uint16_t>(30U+i)))return false;
    }
    if(binding.enabled)for(unsigned i=0U;i<3U;++i){
        curve.positiveGain[i]=static_cast<double>(binding.values[1U+i])/EDMRecipe::Scale;
        curve.negativeGain[i]=static_cast<double>(binding.values[4U+i])/EDMRecipe::Scale;
    }
    return true;
}
inline bool IsCurrent(const Binding& binding,const EDMRecipe::Controller& controller,std::uint32_t profileId) noexcept
{return binding.ready&&Same(binding,Capture(controller,profileId));}
}
