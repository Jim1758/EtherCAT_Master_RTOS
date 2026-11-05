#pragma once
#include "NCPreparedBlockQueueShadow.h"
#include "NCMachiningTimePolicy.h"
#include <cmath>
#include <cstdint>

// EDM358: P01-P07 share the released source-local G76 L count of consecutive qualified complete
// inspection passes. It is independent of G01/G02/G03 L forced flushing.
namespace NCOrbitCompletionPolicy
{
constexpr char CountAddress='L';
constexpr std::uint32_t MaximumPasses=99U;
constexpr std::uint32_t PercentScale=1000000U,MinimumPercentScaled=10000000U,MaximumPercentScaled=100000000U;
inline bool SupportedMode(double p) noexcept
{return p==50.0||NCMachiningTimePolicy::TimedContourMode(p);}
inline bool Polygon(double p) noexcept {return NCMachiningTimePolicy::PolygonMode(p);}
inline bool Requests(const NCBlock& block) noexcept
{return block.has(CountAddress)&&NCMachiningTimePolicy::Contains(block,76);}
inline bool Requests(const NCParsedBlock& block) noexcept
{return block.has(CountAddress)&&NCMachiningTimePolicy::Contains(block,76);}
inline bool AllowedAddress(char a,double p) noexcept
{return a=='N'||a=='P'||a=='R'||a=='K'||a=='F'||a=='B'||a=='X'||a=='Y'||a=='Z'||a=='D'||a=='H'||a=='L'||(a=='Q'&&Polygon(p));}
inline bool Decode(const NCParsedBlock& parsed,const NCBlock& block,std::uint32_t& output) noexcept
{
    output=0U;NCBlock literal{};
    NCMachiningTime::Config timing{};
    // Reuse the complete hidden-field/literal owner contract, including the
    // independently optional D/H words. Literal resolution alone omits hidden
    // parser slots and duplicate/control metadata and cannot be authority.
    if(!NCMachiningTimePolicy::Decode(parsed,block,timing))return false;
    if(!NCPreparedBlockQueueShadow::TryBuildLiteralBlock(parsed,literal)||
       !block.hasG||block.gCode!=76||block.gCount!=1||block.gCodes[0]!=76||block.mCount||
       block.isEmpty||block.isGoto||block.isBlockSkip||!block.has('P')||!SupportedMode(block.val('P'))||
       !NCMachiningTimePolicy::ExactIntegerLiteral(parsed.gExpressions[0],76U)||
       !NCMachiningTimePolicy::ExactIntegerLiteral(parsed.expression('P'),static_cast<unsigned>(block.val('P')))||
       literal.hasG!=block.hasG||literal.gCode!=block.gCode||literal.gCount!=block.gCount||
       literal.mCount!=block.mCount||literal.isEmpty!=block.isEmpty||literal.isGoto!=block.isGoto||
       literal.isBlockSkip!=block.isBlockSkip||literal.gotoTarget!=block.gotoTarget)return false;
    for(int i=0;i<NC_MAX_G_CODES_PER_BLOCK;++i)if(literal.gCodes[i]!=block.gCodes[i])return false;
    for(int i=0;i<NC_MAX_M_CODES_PER_BLOCK;++i)if(literal.mCode[i]!=block.mCode[i])return false;
    for(char a='A';a<='Z';++a){
        if(literal.has(a)!=block.has(a)||literal.val(a)!=block.val(a)||
           std::signbit(literal.val(a))!=std::signbit(block.val(a))||
           (block.has(a)&&(!AllowedAddress(a,block.val('P'))||!std::isfinite(block.val(a)))))return false;
    }
    if(!block.has('L'))return true;
    const double value=block.val('L');
    if(std::signbit(value)||value>MaximumPasses||std::floor(value)!=value||
       !NCMachiningTimePolicy::ExactIntegerLiteral(parsed.expression('L'),static_cast<unsigned>(value)))return false;
    output=static_cast<std::uint32_t>(value);return true;
}
}
