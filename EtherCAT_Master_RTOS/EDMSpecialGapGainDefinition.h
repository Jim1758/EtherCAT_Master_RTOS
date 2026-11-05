#pragma once
#include "EDMRecipe.h"
#include <array>
#include <cstring>

// EDM368: optional E-specific gap-curve gains. Values retain the same direct
// gain units as EDMProcessConfig; these are neither axis PID nor percentages.
namespace EDMSpecialGapGainDefinition
{
constexpr std::uint16_t FirstFieldId=30U, LastFieldId=36U, SwitchFieldId=30U;
constexpr std::size_t FieldCount=7U;
constexpr unsigned MaximumGain=1000U;
inline bool InGroup(std::uint16_t id) noexcept {return id>=FirstFieldId&&id<=LastFieldId;}
inline const char* Semantic(std::uint16_t id) noexcept
{
    static const char* const names[FieldCount]={"SpecialGapGainEnabled","PositiveGain1","PositiveGain2",
        "PositiveGain3","NegativeGain1","NegativeGain2","NegativeGain3"};
    return InGroup(id)?names[id-FirstFieldId]:"";
}
inline const char* Name(std::uint16_t id) noexcept
{
    static const char* const names[FieldCount]={u8"\u958b\u555f\u7279\u5225P/N\u589e\u76ca",
        u8"\u6b63\u5411\u589e\u76ca1",u8"\u6b63\u5411\u589e\u76ca2",u8"\u6b63\u5411\u589e\u76ca3",
        u8"\u8ca0\u5411\u589e\u76ca1",u8"\u8ca0\u5411\u589e\u76ca2",u8"\u8ca0\u5411\u589e\u76ca3"};
    return InGroup(id)?names[id-FirstFieldId]:"";
}
inline const char* Unit(std::uint16_t id) noexcept {return id==SwitchFieldId?"code":"gain";}
inline std::uint8_t Decimals(std::uint16_t id) noexcept {return id==SwitchFieldId?0U:6U;}
inline EDMRecipe::Value Maximum(std::uint16_t id) noexcept
{return (id==SwitchFieldId?1LL:static_cast<EDMRecipe::Value>(MaximumGain))*EDMRecipe::Scale;}
inline EDMRecipe::Value DefaultValue(const EDMRecipe::FieldDefinition& f) noexcept
{
    if(!InGroup(f.id)||f.id==SwitchFieldId)return 0LL;
    return ((f.id==31U||f.id==34U)?25LL:50LL)*EDMRecipe::Scale;
}
inline bool Exact(const EDMRecipe::FieldDefinition& f) noexcept
{
    return InGroup(f.id)&&f.enabled&&f.unitConfirmed&&f.stages.empty()&&f.decimals==Decimals(f.id)&&
        f.minimum==0LL&&f.maximum==Maximum(f.id)&&
        std::strncmp(f.semanticKey,Semantic(f.id),sizeof(f.semanticKey))==0&&
        std::strncmp(f.unit,Unit(f.id),sizeof(f.unit))==0;
}
inline bool Legacy(const EDMRecipe::FieldDefinition& f) noexcept
{
    if(!InGroup(f.id)||f.enabled||f.unitConfirmed||f.decimals!=6U||f.minimum!=0LL||
       f.maximum!=0LL||!f.stages.empty()||std::strncmp(f.unit,"raw",sizeof(f.unit))!=0)return false;
    char reserved[]="Reserved030";reserved[10]=static_cast<char>('0'+f.id-FirstFieldId);
    return std::strncmp(f.semanticKey,reserved,sizeof(f.semanticKey))==0;
}
inline bool Normalize(std::uint32_t schema,std::uint32_t profile,EDMRecipe::FieldDefinition& f) noexcept
{
    if(schema!=2U||profile!=1U||!Legacy(f))return false;
    std::memset(f.semanticKey,0,sizeof(f.semanticKey));std::strcpy(f.semanticKey,Semantic(f.id));
    std::memset(f.name,0,sizeof(f.name));std::strcpy(f.name,Name(f.id));
    std::memset(f.unit,0,sizeof(f.unit));std::strcpy(f.unit,Unit(f.id));
    f.decimals=Decimals(f.id);f.minimum=0LL;f.maximum=Maximum(f.id);f.enabled=f.unitConfirmed=true;
    return true;
}
inline bool DefaultCell(const EDMRecipe::FieldDefinition& f,EDMRecipe::Cell& cell) noexcept
{
    if(!Exact(f)||cell.kind!=EDMRecipe::CellKind::Unset||cell.value!=0LL||cell.stageId||cell.overridePresent)return false;
    // Legacy blank and literal zero have one stable meaning before/after reboot.
    cell.kind=EDMRecipe::CellKind::Direct;return true;
}
enum class Group : std::uint8_t { Missing, Legacy, Typed, Invalid };
inline Group Inspect(const EDMRecipe::Catalog& catalog,std::array<std::size_t,FieldCount>& indices) noexcept
{
    indices.fill(catalog.fields.size());std::size_t count=0U,legacy=0U,typed=0U;
    for(std::size_t i=0U;i<catalog.fields.size();++i){
        const auto& f=catalog.fields[i];if(!InGroup(f.id))continue;
        if(indices[f.id-FirstFieldId]!=catalog.fields.size())return Group::Invalid;
        indices[f.id-FirstFieldId]=i;++count;legacy+=Legacy(f)?1U:0U;typed+=Exact(f)?1U:0U;
    }
    return !count?Group::Missing:count!=FieldCount?Group::Invalid:
        legacy==FieldCount?Group::Legacy:typed==FieldCount?Group::Typed:Group::Invalid;
}
}
