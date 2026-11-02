#pragma once
#include "NC_Types.h"
#include "GCodeParser.h"
#include "HoldConfig.h"
#include <cerrno>
#include <cctype>
#include <cstdint>
#include <cstdlib>

// EDM331 common declaration: zero means the current NC parent's start;
// n>0 means n previous completed NC parents in addition to the current one.
// A whole G76 NC command is one parent, never one internal layer or chord.
namespace NCHoldPolicy
{
constexpr int DeclarationCode=181;
constexpr unsigned MaximumPreviousParents=15U;
constexpr bool G76CommandIsOneParent=true;
struct State
{
    std::uint32_t previousSegments=0U;
    bool declared=false;
    void Clear() noexcept {previousSegments=0U;declared=false;}
    bool Commit(std::uint32_t count) noexcept
    {if(count>MaximumPreviousParents)return false;previousSegments=count;declared=true;return true;}
};
inline bool Decode(const NCBlock& b,std::uint32_t& output) noexcept
{
    if(b.isEmpty||b.isBlockSkip||b.isGoto||!b.hasG||b.gCode!=DeclarationCode||b.gCount!=1||
       b.gCodes[0]!=DeclarationCode||b.mCount!=0||!b.has('P'))return false;
    const double value=b.val('P');
    if(!std::isfinite(value)||value<0.0||value>MaximumPreviousParents||std::floor(value)!=value)return false;
    for(char address='A';address<='Z';++address)if(b.has(address)){
        if(address!='P'&&address!='N')return false;
        if(!std::isfinite(b.val(address)))return false;
        if(address=='N'&&(b.val(address)<0.0||std::floor(b.val(address))!=b.val(address)))return false;
    }
    output=static_cast<std::uint32_t>(value);return true;
}
inline bool LiteralNumber(const std::string& text,double& output) noexcept
{
    const char* first=text.c_str();while(*first&&std::isspace(static_cast<unsigned char>(*first)))++first;
    errno=0;char* end=nullptr;const double candidate=std::strtod(first,&end);
    if(end==first||errno==ERANGE||!std::isfinite(candidate))return false;
    while(*end&&std::isspace(static_cast<unsigned char>(*end)))++end;
    if(*end)return false;
    output=candidate;return true;
}
inline bool DecodeAuthored(const NCParsedBlock& parsed,const NCBlock& resolved,std::uint32_t& output) noexcept
{
    std::uint32_t count=0U;
    if(parsed.error!=NCParseError::NONE||parsed.controlType!=NCParsedControlType::NONE||parsed.isEmpty||
       parsed.isBlockSkip||parsed.dependsOnMacroState||parsed.duplicateAddressMask||parsed.gCount!=1||
       parsed.mCount!=0||!Decode(resolved,count))return false;
    double code=0.0;if(!LiteralNumber(parsed.gExpressions[0],code)||code!=DeclarationCode)return false;
    for(char address='A';address<='Z';++address){
        if(parsed.has(address)!=resolved.has(address))return false;
        if(!parsed.has(address))continue;
        double value=0.0;if(!LiteralNumber(parsed.expression(address),value)||value!=resolved.val(address))return false;
    }
    output=count;return true;
}
}
