#pragma once
#include "NCMachiningTime.h"
#include "NC_Types.h"
#include "GCodeParser.h"
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <cstdlib>
#include <string>

// EDM340: D/H are nonmodal minutes only for an independently admitted EDM
// G01/G02/G03, P50 or explicitly timed P11/P12/P13/P14/P15 parent. Pure syntax never grants
// that runtime authority; untimed P11/P12/P13/P14/P15 retain the legacy interpretation.
namespace NCMachiningTimePolicy
{
using Config=NCMachiningTime::Config;
constexpr char TotalAddress='D',AdditionalAddress='H';
constexpr std::uint64_t MillionthsPerMinute=1000000ULL,UsPerMillionthMinute=60ULL;
inline bool IsProcessCode(int code) noexcept {return code==1||code==2||code==3||code==76;}
inline bool AllowedAddress(int code,char a) noexcept
{
    if(a=='N'||a=='F'||a=='B'||a==TotalAddress||a==AdditionalAddress||a=='X'||a=='Y'||a=='Z')return true;
    return code==76?(a=='P'||a=='R'||a=='K'):
        (a=='L'||((code==2||code==3)&&(a=='I'||a=='J'||a=='K'||a=='R')));
}
inline bool LiteralNumber(const std::string& expression,double& output) noexcept
{
    if(expression.find('\0')!=std::string::npos)return false;
    const char* p=expression.c_str();while(*p==' '||*p=='\t'||*p=='\r'||*p=='\n')++p;
    errno=0;char* end=nullptr;const double value=std::strtod(p,&end);
    if(end==p||errno==ERANGE||!std::isfinite(value))return false;
    while(*end==' '||*end=='\t'||*end=='\r'||*end=='\n')++end;
    if(*end)return false;output=value;return true;
}
// Source identity for a newly opted-in mode cannot arise from binary64
// rounding or exponent syntax. Existing numeric policy outside P11 is retained.
inline bool ExactIntegerLiteral(const std::string& expression,unsigned expected) noexcept
{
    const auto space=[](char c) noexcept {return c==' '||c=='\t'||c=='\r'||c=='\n';};
    std::size_t i=0U;while(i<expression.size()&&space(expression[i]))++i;
    if(i<expression.size()&&expression[i]=='+')++i;
    unsigned value=0U;bool digits=false;
    while(i<expression.size()&&expression[i]>='0'&&expression[i]<='9'){
        digits=true;const unsigned digit=static_cast<unsigned>(expression[i++]-'0');
        if(value>expected/10U||(value==expected/10U&&digit>expected%10U))return false;
        value=value*10U+digit;
    }
    if(!digits||value!=expected)return false;
    if(i<expression.size()&&expression[i]=='.'){
        ++i;const std::size_t first=i;
        while(i<expression.size()&&expression[i]>='0'&&expression[i]<='9')if(expression[i++]!='0')return false;
        if(i==first)return false;
    }
    while(i<expression.size()&&space(expression[i]))++i;
    return i==expression.size();
}
inline bool Contains(const NCBlock& block,int code) noexcept
{
    if(block.hasG&&block.gCode==code)return true;
    for(int i=0;i<block.gCount&&i<NC_MAX_G_CODES_PER_BLOCK;++i)if(block.gCodes[i]==code)return true;
    return false;
}
inline bool Contains(const NCParsedBlock& parsed,int code) noexcept
{
    for(int i=0;i<parsed.gCount&&i<NC_MAX_G_CODES_PER_BLOCK;++i){double value=0.;
        if(LiteralNumber(parsed.gExpressions[static_cast<std::size_t>(i)],value)&&value==code)return true;}
    return false;
}
template<class Block> inline bool LegacyOwner(const Block& block,char address) noexcept
{
    if(Contains(block,65)||Contains(block,66))return true;
    return address==TotalAddress?(Contains(block,40)||Contains(block,41)||Contains(block,42)||
        Contains(block,176)||Contains(block,177)):
        (Contains(block,43)||Contains(block,44)||Contains(block,49)||Contains(block,178));
}
// EDM341: explicit timed-P11 intent owns its validation before a mixed
// tool/macro selector can hide D/H from the pre-side-effect admission guard.
// The full Decode below still rejects every mixed-G source.
inline bool TimedP11Intent(const NCBlock& block) noexcept
{
    return Contains(block,76)&&block.has('P')&&(block.val('P')==11.0||block.val('P')==12.0||block.val('P')==13.0||block.val('P')==14.0||block.val('P')==15.0)&&
        (block.has(TotalAddress)||block.has(AdditionalAddress));
}
inline bool TimedP11Intent(const NCParsedBlock& block) noexcept
{
    return Contains(block,76)&&block.has('P')&&(ExactIntegerLiteral(block.expression('P'),11U)||
        ExactIntegerLiteral(block.expression('P'),12U)||ExactIntegerLiteral(block.expression('P'),13U)||
        ExactIntegerLiteral(block.expression('P'),14U)||ExactIntegerLiteral(block.expression('P'),15U))&&
        (block.has(TotalAddress)||block.has(AdditionalAddress));
}
template<class Block> inline bool RequestsImpl(const Block& block) noexcept
{
    if(TimedP11Intent(block))return true;
    const bool words=(block.has(TotalAddress)&&!LegacyOwner(block,TotalAddress))||
        (block.has(AdditionalAddress)&&!LegacyOwner(block,AdditionalAddress));
    if(!words)return false;
    for(int code:{1,2,3,76})if(Contains(block,code))return true;
    return false;
}
inline bool Requests(const NCBlock& block) noexcept {return RequestsImpl(block);}
inline bool Requests(const NCParsedBlock& block) noexcept {return RequestsImpl(block);}
// Exact ordinary decimal minutes, at most six fractional digits. Conversion is
// integer-only: one millionth minute is 60 us. Exponents, signs (including -0),
// arithmetic and macro syntax never acquire time semantics.
inline bool ParseMinutes(const std::string& expression,std::uint64_t& output) noexcept
{
    const char* p=expression.c_str();while(*p==' '||*p=='\t'||*p=='\r'||*p=='\n')++p;
    std::uint64_t whole=0ULL,fraction=0ULL;unsigned places=0U;bool digit=false;
    while(*p>='0'&&*p<='9'){
        digit=true;const unsigned n=static_cast<unsigned>(*p++-'0');
        if(whole>1440ULL/10ULL||(whole==1440ULL/10ULL&&n>1440ULL%10ULL))return false;
        whole=whole*10ULL+n;
    }
    if(*p=='.'){
        ++p;const char* first=p;
        while(*p>='0'&&*p<='9'){
            if(places==6U)return false;digit=true;fraction=fraction*10ULL+static_cast<unsigned>(*p++-'0');++places;
        }
        if(p==first)return false;
    }
    if(!digit)return false;
    while(*p==' '||*p=='\t'||*p=='\r'||*p=='\n')++p;
    if(*p||static_cast<std::size_t>(p-expression.c_str())!=expression.size())return false;
    for(;places<6U;++places)fraction*=10ULL;
    const std::uint64_t units=whole*MillionthsPerMinute+fraction;
    if(units>1440ULL*MillionthsPerMinute)return false;
    const auto value=units*UsPerMillionthMinute;
    Config check{};check.totalLimitUs=value;
    if(!NCMachiningTime::ValidConfig(check))return false;
    output=value;return true;
}
inline bool CanonicalMinutes(double value,std::uint64_t& output) noexcept
{
    if(!std::isfinite(value)||std::signbit(value)||value>1440.0)return false;
    const auto units=static_cast<std::uint64_t>(std::floor(value*1000000.0+.5));
    if(static_cast<double>(units)/1000000.0!=value)return false;
    Config check{};check.totalLimitUs=units*UsPerMillionthMinute;
    if(!NCMachiningTime::ValidConfig(check))return false;
    output=check.totalLimitUs;return true;
}
// Shape-only owner classification for generic parameter guards/reducers. Full
// authored Decode and an exact EDM grant remain mandatory before dispatch.
inline bool ResolvedOwner(const NCBlock& block) noexcept
{
    if(block.isEmpty||block.isBlockSkip||block.isGoto||!block.hasG||block.gCount!=1||
       block.gCodes[0]!=block.gCode||!IsProcessCode(block.gCode)||block.mCount!=0||block.gotoTarget!=-1||
       (block.gCode==76&&(!block.has('P')||(block.val('P')!=50.0&&
           !((block.val('P')==11.0||block.val('P')==12.0||block.val('P')==13.0||block.val('P')==14.0||block.val('P')==15.0)&&
             (block.has(TotalAddress)||block.has(AdditionalAddress)))))))return false;
    for(std::size_t i=1U;i<NC_MAX_G_CODES_PER_BLOCK;++i)if(block.gCodes[i]!=0)return false;
    for(int value:block.mCode)if(value!=-1)return false;
    for(char a='A';a<='Z';++a){
        if(block.has(a)){if(!AllowedAddress(block.gCode,a)||!std::isfinite(block.val(a)))return false;}
        else if(block.val(a)!=0.0||std::signbit(block.val(a)))return false;
    }
    std::uint64_t ignored=0ULL;
    return (!block.has(TotalAddress)||CanonicalMinutes(block.val(TotalAddress),ignored))&&
        (!block.has(AdditionalAddress)||CanonicalMinutes(block.val(AdditionalAddress),ignored));
}
inline bool Decode(const NCParsedBlock& parsed,const NCBlock& block,Config& output) noexcept
{
    if(parsed.error!=NCParseError::NONE||parsed.isEmpty||parsed.isBlockSkip||parsed.dependsOnMacroState||
       parsed.duplicateAddressMask||parsed.controlType!=NCParsedControlType::NONE||parsed.gCount!=1||parsed.mCount!=0||
       parsed.assignment.prefix!=0||!parsed.assignment.indexExpression.empty()||!parsed.assignment.valueExpression.empty()||
       parsed.gotoStatement.conditional||!parsed.gotoStatement.conditionExpression.empty()||!parsed.gotoStatement.targetExpression.empty()||
       !ResolvedOwner(block)||block.gotoTarget!=-1)return false;
    for(std::size_t i=1U;i<NC_MAX_G_CODES_PER_BLOCK;++i)if(block.gCodes[i]!=0)return false;
    for(int value:block.mCode)if(value!=-1)return false;
    double code=0.;if(!LiteralNumber(parsed.gExpressions[0],code)||code!=block.gCode)return false;
    if(block.gCode==76&&(block.val('P')==11.0||block.val('P')==12.0||block.val('P')==13.0||block.val('P')==14.0||block.val('P')==15.0)&&
        (!ExactIntegerLiteral(parsed.gExpressions[0],76U)||
         !ExactIntegerLiteral(parsed.expression('P'),static_cast<unsigned>(block.val('P')))))return false;
    for(std::size_t i=1U;i<parsed.gExpressions.size();++i)if(!parsed.gExpressions[i].empty())return false;
    for(const auto& expression:parsed.mExpressions)if(!expression.empty())return false;
    for(char a='A';a<='Z';++a){
        if(parsed.has(a)!=block.has(a))return false;
        if(!parsed.has(a)){
            if(!parsed.expression(a).empty()||block.val(a)!=0.0||std::signbit(block.val(a)))return false;
            continue;
        }
        double value=0.;if(!LiteralNumber(parsed.expression(a),value)||!std::isfinite(block.val(a))||
            value!=block.val(a)||std::signbit(value)!=std::signbit(block.val(a)))return false;
    }
    Config result{};
    if((parsed.has(TotalAddress)&&!ParseMinutes(parsed.expression(TotalAddress),result.totalLimitUs))||
       (parsed.has(AdditionalAddress)&&!ParseMinutes(parsed.expression(AdditionalAddress),result.additionalUs)))return false;
    if(!NCMachiningTime::ValidConfig(result))return false;
    output=result;return true;
}
inline void StripResolved(NCBlock& geometry) noexcept
{
    for(char a:{TotalAddress,AdditionalAddress}){geometry.hasParam[a-'A']=false;geometry.param[a-'A']=0.0;}
}
inline bool StripValidated(const NCParsedBlock& parsed,const NCBlock& block,NCBlock& geometry,Config& output) noexcept
{
    Config candidate{};if(!Decode(parsed,block,candidate))return false;
    geometry=block;StripResolved(geometry);output=candidate;return true;
}
inline bool StripParsedValidated(const NCParsedBlock& parsed,const NCBlock& block,
    NCParsedBlock& geometryParsed,NCBlock& geometry,Config& output) noexcept
{
    Config candidate{};if(!Decode(parsed,block,candidate))return false;
    geometryParsed=parsed;geometry=block;StripResolved(geometry);
    for(char a:{TotalAddress,AdditionalAddress}){geometryParsed.hasParam[a-'A']=false;geometryParsed.paramExpressions[a-'A'].clear();}
    output=candidate;return true;
}
}
