#pragma once
#include "NC_Types.h"
#include "NCTranslationSnapshot.h"
#include <cmath>

// EDM384 source shape only. This is used inside the independently qualified
// EDM program grant. It does not relax ordinary CNC feed classifiers and never
// resolves axis identity, coordinates, feed provenance, or motion authority.
// B/D/H/L metadata must already have passed their own policies and been removed.
namespace NCEDMCoordinatedLine384
{
inline bool GeometryShape(const NCBlock& block,bool allowMissingFeed=true,int unitsMode=21) noexcept
{
    if((unitsMode!=20&&unitsMode!=21)||block.isEmpty||block.isBlockSkip||block.isGoto||
       block.gotoTarget!=-1||!block.hasG||block.gCode!=1||block.gCount!=1||block.gCodes[0]!=1||
       block.mCount!=0||(!block.has('F')&&!allowMissingFeed))return false;
    for(std::size_t i=1U;i<NC_MAX_G_CODES_PER_BLOCK;++i)if(block.gCodes[i]!=0)return false;
    for(int value:block.mCode)if(value!=-1)return false;
    bool axis=false;
    for(char address='A';address<='Z';++address){
        if(!block.has(address))continue;
        const bool nativeAxis=address=='X'||address=='Y'||address=='Z'||address=='C'||address=='U'||address=='V';
        if((address!='N'&&address!='F'&&!nativeAxis)||!std::isfinite(block.val(address)))return false;
        if(nativeAxis)axis=true;
    }
    if(!axis)return false;
    if(block.has('F')){
        const double feedMMMin=NCTranslationLengthToMM(block.val('F'),unitsMode);
        if(!std::isfinite(feedMMMin)||feedMMMin<=0.0||feedMMMin>100.0)return false;
    }
    return true;
}
}
