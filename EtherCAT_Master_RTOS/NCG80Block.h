#pragma once
#include "NC_Types.h"
#include <array>
#include <cmath>
#include <cstdint>

// Pure whole-row decoder. Axis words are native direction components: never
// absolute/incremental endpoints, inch lengths or polar radius/angle words.
namespace NCG80
{
enum class Error : std::uint32_t {
    None, Shape, Duplicate, UnknownWord, NonFinite, Direction,
    Mode, Fast, Retreat, Origin, MissingOriginValue, OrphanValue, Feed, Source
};
struct Program {
    std::array<double,6U> direction{};
    std::uint32_t presentMask=0U,authoredMask=0U;
    unsigned p=0U,j=0U,k=0U;
    bool simulation=false; // Q1 is explicit for this one block only.
    int a=-1;
    double l=0.0,feedPercent=100.0;
};
inline bool Decode(const NCBlock& b,std::uint32_t duplicateMask,Program& out,Error& error) noexcept {
    out=Program{};error=Error::None;
    const int count=b.gCount>0?b.gCount:(b.hasG?1:0);
    if(b.isEmpty||b.isGoto||b.isBlockSkip||count!=1||!b.hasG||b.gCode!=80||
       (b.gCount>0&&b.gCodes[0]!=80)||b.mCount!=0){error=Error::Shape;return false;}
    if(duplicateMask){error=Error::Duplicate;return false;}
    for(char word='A';word<='Z';++word)if(b.has(word)){
        if(word!='N'&&word!='G'&&word!='X'&&word!='Y'&&word!='Z'&&word!='C'&&
           word!='U'&&word!='V'&&word!='P'&&word!='J'&&word!='K'&&word!='A'&&word!='L'&&word!='F'&&word!='Q')
            {error=Error::UnknownWord;return false;}
        if(!std::isfinite(b.val(word))){error=Error::NonFinite;return false;}
    }
    const char axes[6]={'X','Y','Z','C','U','V'};
    for(unsigned i=0U;i<6U;++i)if(b.has(axes[i])){
        out.presentMask|=1U<<i;out.direction[i]=b.val(axes[i]);
        if(out.direction[i]!=0.0)out.authoredMask|=1U<<i;
    }
    if(!out.authoredMask){error=Error::Direction;return false;}
    if(b.has('P')&&b.val('P')!=0.0&&b.val('P')!=1.0){error=Error::Mode;return false;}
    if(b.has('J')&&b.val('J')!=0.0&&b.val('J')!=1.0){error=Error::Fast;return false;}
    if(b.has('K')&&b.val('K')!=0.0&&b.val('K')!=1.0){error=Error::Retreat;return false;}
    if(b.has('Q')&&b.val('Q')!=0.0&&b.val('Q')!=1.0){error=Error::Source;return false;}
    out.p=b.has('P')?static_cast<unsigned>(b.val('P')):0U;
    out.j=b.has('J')?static_cast<unsigned>(b.val('J')):0U;
    out.k=b.has('K')?static_cast<unsigned>(b.val('K')):0U;
    out.simulation=b.has('Q')&&b.val('Q')==1.0;
    if(b.has('A')){
        if(b.val('A')!=0.0&&b.val('A')!=1.0&&b.val('A')!=2.0){error=Error::Origin;return false;}
        out.a=static_cast<int>(b.val('A'));
    }
    if(out.a==1&&!b.has('L')){error=Error::MissingOriginValue;return false;}
    if(b.has('L')&&out.a!=1){error=Error::OrphanValue;return false;}
    if(b.has('L'))out.l=b.val('L');
    if(b.has('F'))out.feedPercent=b.val('F');
    if(!(out.feedPercent>0.0)){error=Error::Feed;return false;}
    return true;
}
}
