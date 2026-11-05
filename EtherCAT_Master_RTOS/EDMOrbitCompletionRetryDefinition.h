#pragma once
#include "EDMRecipe.h"
#include <cstring>

// EDM366: typed optional E29 definition. The exact former reserved zero-only
// seed may be interpreted at load time; external files are never overwritten.
namespace EDMOrbitCompletionRetryDefinition
{
constexpr std::uint16_t FieldId=29U;
constexpr unsigned MaximumRetries=99U;
constexpr char Semantic[]="OrbitCompletionRetryLimit";
constexpr char Unit[]="count";
inline bool IsSemantic(const EDMRecipe::FieldDefinition& f) noexcept
{return std::strncmp(f.semanticKey,Semantic,sizeof(f.semanticKey))==0;}
inline bool Exact(const EDMRecipe::FieldDefinition& f) noexcept
{
    return f.id==FieldId&&IsSemantic(f)&&f.enabled&&f.unitConfirmed&&f.decimals==0U&&
        f.minimum==0LL&&f.maximum==MaximumRetries*EDMRecipe::Scale&&
        std::strncmp(f.unit,Unit,sizeof(f.unit))==0;
}
inline bool Legacy(const EDMRecipe::FieldDefinition& f) noexcept
{
    return f.id==FieldId&&!f.enabled&&!f.unitConfirmed&&f.decimals==6U&&
        f.minimum==0LL&&f.maximum==0LL&&f.stages.empty()&&
        std::strncmp(f.semanticKey,"Reserved029",sizeof(f.semanticKey))==0&&
        std::strncmp(f.unit,"raw",sizeof(f.unit))==0;
}
inline bool Normalize(std::uint32_t schema,std::uint32_t profile,
    EDMRecipe::FieldDefinition& f) noexcept
{
    if(schema!=2U||profile!=1U||!Legacy(f))return false;
    std::memset(f.semanticKey,0,sizeof(f.semanticKey));std::strcpy(f.semanticKey,Semantic);
    std::memset(f.name,0,sizeof(f.name));std::strcpy(f.name,u8"\u6416\u52d5\u91cd\u8a66\u4e0a\u9650");
    std::memset(f.unit,0,sizeof(f.unit));std::strcpy(f.unit,Unit);
    f.decimals=0U;f.minimum=0LL;f.maximum=MaximumRetries*EDMRecipe::Scale;
    f.enabled=f.unitConfirmed=true;return true;
}
inline bool DefaultCell(const EDMRecipe::FieldDefinition& f,EDMRecipe::Cell& cell) noexcept
{
    if(!Exact(f)||cell.kind!=EDMRecipe::CellKind::Unset||cell.value!=0LL||
       cell.stageId!=0U||cell.overridePresent)return false;
    cell.kind=EDMRecipe::CellKind::Direct;return true;
}
}
