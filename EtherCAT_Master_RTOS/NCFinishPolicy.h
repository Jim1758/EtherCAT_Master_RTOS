#pragma once
#include "NC_Types.h"
#include "GCodeParser.h"
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>

// EDM335: L is a nonmodal finish-verification count ONLY on the admitted
// analytic EDM G01/G02/G03 lane. It owns no geometry or ordinary CNC meaning.
namespace NCFinishPolicy
{
constexpr char CountAddress='L';
constexpr std::uint32_t MaximumForcedFlushes=99U;
inline bool IsProcessCode(int code) noexcept {return code==1||code==2||code==3;}
inline bool LiteralNumber(const std::string& text,double& output) noexcept
{
    if(text.find('\0')!=std::string::npos)return false;
    const char* first=text.c_str();
    while(*first&&std::isspace(static_cast<unsigned char>(*first)))++first;
    errno=0;char* end=nullptr;const double candidate=std::strtod(first,&end);
    if(end==first||errno==ERANGE||!std::isfinite(candidate))return false;
    while(*end&&std::isspace(static_cast<unsigned char>(*end)))++end;
    if(*end)return false;
    output=candidate;return true;
}
inline bool Requests(const NCBlock& block) noexcept
{
    if(!block.has(CountAddress))return false;
    if(block.hasG&&IsProcessCode(block.gCode))return true;
    for(int i=0;i<block.gCount&&i<NC_MAX_G_CODES_PER_BLOCK;++i)
        if(IsProcessCode(block.gCodes[i]))return true;
    return false;
}
inline bool Requests(const NCParsedBlock& parsed) noexcept
{
    if(!parsed.has(CountAddress))return false;
    for(int i=0;i<parsed.gCount&&i<NC_MAX_G_CODES_PER_BLOCK;++i){
        double code=0.0;
        if(LiteralNumber(parsed.gExpressions[static_cast<std::size_t>(i)],code)&&
           (code==1.0||code==2.0||code==3.0))return true;
    }
    return false;
}
// This is the source/provenance contract. The caller still validates the
// existing G01 or plane-specific G02/G03 geometric address whitelist.
// Neither output changes on failure; absent L is exactly the disabled count.
inline bool Decode(const NCParsedBlock& parsed,const NCBlock& resolved,std::uint32_t& output) noexcept
{
    if(parsed.error!=NCParseError::NONE||parsed.isEmpty||parsed.isBlockSkip||parsed.dependsOnMacroState||
       parsed.duplicateAddressMask||parsed.controlType!=NCParsedControlType::NONE||parsed.gCount!=1||parsed.mCount!=0||
       parsed.assignment.prefix!=0||!parsed.assignment.indexExpression.empty()||!parsed.assignment.valueExpression.empty()||
       parsed.gotoStatement.conditional||!parsed.gotoStatement.conditionExpression.empty()||!parsed.gotoStatement.targetExpression.empty()||
       resolved.isEmpty||resolved.isBlockSkip||resolved.isGoto||!resolved.hasG||resolved.gCount!=1||
       !IsProcessCode(resolved.gCode)||resolved.gCodes[0]!=resolved.gCode||resolved.mCount!=0)return false;
    double code=0.0;
    if(!LiteralNumber(parsed.gExpressions[0],code)||code!=resolved.gCode)return false;
    for(std::size_t i=1U;i<parsed.gExpressions.size();++i)if(!parsed.gExpressions[i].empty())return false;
    for(const auto& expression:parsed.mExpressions)if(!expression.empty())return false;
    for(char address='A';address<='Z';++address){
        if(parsed.has(address)!=resolved.has(address))return false;
        if(!parsed.has(address)){
            if(!parsed.expression(address).empty())return false;
            continue;
        }
        double value=0.0;
        if(!LiteralNumber(parsed.expression(address),value)||!std::isfinite(resolved.val(address))||
           value!=resolved.val(address))return false;
        if(address==CountAddress&&std::signbit(value))return false;
    }
    const double count=resolved.has(CountAddress)?resolved.val(CountAddress):0.0;
    if(std::signbit(count)||count>MaximumForcedFlushes||std::floor(count)!=count)return false;
    output=static_cast<std::uint32_t>(count);return true;
}
inline bool StripValidated(const NCParsedBlock& parsed,const NCBlock& resolved,
    NCBlock& geometry,std::uint32_t& output) noexcept
{
    std::uint32_t count=0U;if(!Decode(parsed,resolved,count))return false;
    geometry=resolved;geometry.hasParam[CountAddress-'A']=false;geometry.param[CountAddress-'A']=0.0;
    output=count;return true;
}
}
