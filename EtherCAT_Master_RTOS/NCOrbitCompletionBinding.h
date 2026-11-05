#pragma once
#include "NCOrbitCompletionPolicy.h"
#include "EDMFlushRecipeBinding.h"
#include "EDMOrbitCompletionRetryDefinition.h"
#include <cstring>

// NC owner only. No catalog pointer or mutable E-condition selection crosses
// to RT; Request carries only the validated count and exact scaled percent.
namespace NCOrbitCompletionBinding
{
enum class RetryMode:std::uint8_t { Missing,LegacyZero,UnsetZero,Configured };
struct RetryBinding
{
    EDMRecipe::Value value=0LL,minimum=0LL,maximum=0LL;
    std::uint16_t field=0U;
    std::uint8_t decimals=0U;
    RetryMode mode=RetryMode::Missing;
    EDMRecipe::CellKind rowKind=EDMRecipe::CellKind::Unset;
    bool enabled=false,confirmed=false,configured=false;
    char unit[24]{},semantic[64]{};
};
inline bool SameRetry(const RetryBinding& a,const RetryBinding& b) noexcept
{
    return a.value==b.value&&a.minimum==b.minimum&&a.maximum==b.maximum&&a.field==b.field&&
        a.decimals==b.decimals&&a.mode==b.mode&&a.rowKind==b.rowKind&&a.enabled==b.enabled&&
        a.confirmed==b.confirmed&&a.configured==b.configured&&
        std::memcmp(a.unit,b.unit,sizeof(a.unit))==0&&
        std::memcmp(a.semantic,b.semantic,sizeof(a.semantic))==0;
}
inline bool CaptureRetry(const EDMRecipe::Controller& controller,const EDMRecipe::Catalog& catalog,
    const EDMRecipe::Summary& selection,RetryBinding& out,std::uint8_t& limit) noexcept
{
    namespace D=EDMOrbitCompletionRetryDefinition;
    out=RetryBinding{};limit=0U;
    const EDMRecipe::FieldDefinition* field=nullptr;
    const EDMRecipe::FieldDefinition* semantic=nullptr;
    std::size_t fieldIndex=0U;
    for(std::size_t i=0U;i<catalog.fields.size();++i){
        const auto& f=catalog.fields[i];
        if(D::IsSemantic(f)){if(semantic)return false;semantic=&f;}
        if(f.id==D::FieldId){if(field)return false;field=&f;fieldIndex=i;}
    }
    if(semantic&&semantic!=field)return false;
    if(!field)return true; // Older compact catalogs have no opt-in retry field.
    if(!D::Exact(*field)&&!D::Legacy(*field))return false;
    out.field=field->id;out.minimum=field->minimum;out.maximum=field->maximum;
    out.decimals=field->decimals;out.enabled=field->enabled;out.confirmed=field->unitConfirmed;
    std::memcpy(out.unit,field->unit,sizeof(out.unit));
    std::memcpy(out.semantic,field->semanticKey,sizeof(out.semantic));
    EDMRecipe::ParameterSnapshot actual{};
    if(controller.ReadField(field->id,actual)){
        if(actual.fieldId!=field->id||!actual.configured||
           (actual.rowKind!=EDMRecipe::CellKind::Direct&&actual.rowKind!=EDMRecipe::CellKind::Stage))return false;
        out.value=actual.actualValue;out.configured=true;out.rowKind=actual.rowKind;
        if(D::Legacy(*field)){
            if(out.value!=0LL)return false;
            out.mode=RetryMode::LegacyZero;return true;
        }
        if(out.value<0LL||out.value>D::MaximumRetries*EDMRecipe::Scale||
           out.value%EDMRecipe::Scale!=0LL)return false;
        out.mode=RetryMode::Configured;
        limit=static_cast<std::uint8_t>(out.value/EDMRecipe::Scale);return true;
    }
    // Unset is compatible only when the immutable selected cell is exactly
    // empty. Never turn a malformed configured value into unlimited retries.
    if(!selection.tableSelected||!selection.rowSelected||!selection.eCode||
       selection.eCode>EDMRecipe::RowsPerTable)return false;
    const EDMRecipe::Table* table=nullptr;
    for(const auto& t:catalog.tables)if(t.id==selection.tableId){if(table)return false;table=&t;}
    if(!table||table->cells.size()!=EDMRecipe::RowsPerTable*catalog.fields.size())return false;
    const auto& cell=table->cells[(selection.eCode-1U)*catalog.fields.size()+fieldIndex];
    if(cell.kind!=EDMRecipe::CellKind::Unset||cell.value!=0LL||cell.stageId||cell.overridePresent)return false;
    out.mode=D::Legacy(*field)?RetryMode::LegacyZero:RetryMode::UnsetZero;return true;
}
struct Binding
{
    EDMRecipe::Summary selection{};
    const EDMRecipe::Catalog* catalog=nullptr;
    EDMRecipe::Value value=0LL,minimum=0LL,maximum=0LL;
    std::uint16_t field=0U;
    std::uint8_t decimals=0U;
    char unit[24]{};
    bool enabled=false,confirmed=false,configured=false,ready=false;
    std::uint8_t retryLimit=0U;
    RetryBinding retry{};
};
inline Binding Capture(const EDMRecipe::Controller& controller,std::uint32_t required) noexcept
{
    Binding out{};if(!required){out.ready=true;return out;}
    if(required>NCOrbitCompletionPolicy::MaximumPasses)return out;
    const auto before=controller.Current();const auto* catalog=controller.GetCatalog();
    EDM58::Binding semantic{};
    if(!catalog||!before.catalogReady||catalog->schemaVersion!=2U||catalog->mode!=EDMRecipe::Mode::ShadowOnly||
       !catalog->profileId||catalog->profileId!=before.profileId||
       !EDM58::BindingDetail::Read(controller,*catalog,"OrbitCompletionRatio","%",out.value,out.field,semantic)||
       out.value<NCOrbitCompletionPolicy::MinimumPercentScaled||out.value>NCOrbitCompletionPolicy::MaximumPercentScaled)return Binding{};
    const EDMRecipe::FieldDefinition* definition=nullptr;
    for(const auto& field:catalog->fields)if(field.id==out.field){if(definition)return Binding{};definition=&field;}
    if(!definition)return Binding{};
    out.minimum=definition->minimum;out.maximum=definition->maximum;out.decimals=definition->decimals;
    out.enabled=definition->enabled;out.confirmed=definition->unitConfirmed;
    std::memcpy(out.unit,definition->unit,sizeof(out.unit));
    out.configured=true;
    if(!CaptureRetry(controller,*catalog,before,out.retry,out.retryLimit))return Binding{};
    if(controller.GetCatalog()!=catalog||!EDM58::BindingDetail::SameSelection(before,controller.Current()))return Binding{};
    out.selection=before;out.catalog=catalog;out.ready=true;return out;
}
inline bool IsCurrent(const Binding& frozen,const EDMRecipe::Controller& controller,std::uint32_t required) noexcept
{
    if(!required)return frozen.ready&&frozen.catalog==nullptr&&frozen.value==0LL&&frozen.field==0U&&
        frozen.retryLimit==0U&&SameRetry(frozen.retry,RetryBinding{});
    if(!frozen.ready||!frozen.catalog||controller.GetCatalog()!=frozen.catalog||
       !EDM58::BindingDetail::SameSelection(frozen.selection,controller.Current()))return false;
    const auto now=Capture(controller,required);
    return now.ready&&now.catalog==frozen.catalog&&now.field==frozen.field&&now.value==frozen.value&&
        now.minimum==frozen.minimum&&now.maximum==frozen.maximum&&now.decimals==frozen.decimals&&
        now.enabled==frozen.enabled&&now.confirmed==frozen.confirmed&&now.configured==frozen.configured&&
        std::memcmp(now.unit,frozen.unit,sizeof(now.unit))==0&&now.retryLimit==frozen.retryLimit&&
        SameRetry(now.retry,frozen.retry);
}
}
