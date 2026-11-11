#pragma once

#include "NCXYZFeedScope.h"

// The original engineering fixtures retain their original full-six
// dimensional policy. Ordinary optional/mixed-axis admission below must not
// broaden an old commissioning grant merely because it shares this header.
inline bool IsNCLegacyXYZCUVFeedAxisScopeValid(const NCAxisIdentitySnapshot& identity) noexcept
{
    return IsNCXYZFeedAxisScopeValid(identity) &&
        identity.exists[3] == 1U && identity.axisType[3] == 1U &&
        identity.nativeUnit[3] == 2U && identity.address[3] == 'C' &&
        identity.exists[4] == 1U && identity.axisType[4] == 1U &&
        identity.nativeUnit[4] == 2U && identity.address[4] == 'U' &&
        identity.exists[5] == 1U && identity.axisType[5] == 1U &&
        identity.nativeUnit[5] == 2U && identity.address[5] == 'V';
}

inline bool IsNCLegacyXYZCUVFeedNeutralFrame(const NCTranslationSnapshot& source) noexcept
{
    return IsNCTranslationSnapshotValid(source) &&
        IsNCLegacyXYZCUVFeedAxisScopeValid(source.axisIdentity) &&
        source.rotationPlane == 17 && source.unitsMode == 21 &&
        (source.distanceMode == 90 || source.distanceMode == 91) && source.polarMode == 15 &&
        source.cutterMode == 40 && source.toolLengthMode == 49 &&
        source.rotationMode == 69 && source.workMode == 169 &&
        source.scalingMode == 50 && source.mirrorMask == 0U &&
        source.axisIdentity.eccentricEnabled == 0U;
}

// EDM384 admission: XYZ plus optional native C/U/V of any configured axis type.
// G90/G91 use a shared linear-native mm/min metric; fixed present axes are legal.
// This classifier never supplies geometry, modal feed, or Motion permission.
inline bool IsNCXYZCUVFeedAxisScopeValid(const NCAxisIdentitySnapshot& identity) noexcept
{
    if(!IsNCAxisIdentitySnapshotValid(identity)||identity.exists[6]||identity.exists[7])return false;
    for(unsigned axis=0U;axis<6U;++axis){
        if(axis<3U&&(identity.exists[axis]!=1U||identity.axisType[axis]!=0U))return false;
        if(identity.exists[axis]&&identity.address[axis]!="XYZCUV"[axis])return false;
    }
    return true;
}

inline bool IsNCXYZCUVFeedLineBlockAllowed(const NCAxisIdentitySnapshot& identity,
    const NCBlock& block,int distanceMode=91) noexcept
{
    if((distanceMode!=90&&distanceMode!=91)||!IsNCXYZCUVFeedAxisScopeValid(identity)||
       !IsNCXYZFeedSingleGBlock(block,1)||!block.has('F')||!std::isfinite(block.val('F'))||
       block.val('F')<=0.0||block.val('F')>100.0)return false;
    bool linearWord=false;
    for(char word='A';word<='Z';++word)if(block.has(word)){
        if((word!='N'&&word!='F'&&word!='X'&&word!='Y'&&word!='Z'&&word!='C'&&word!='U'&&word!='V')||
           !std::isfinite(block.val(word)))return false;
    }
    for(unsigned axis=0U;axis<6U;++axis)if(block.has("XYZCUV"[axis])){
        if(!identity.exists[axis])return false;
        if(identity.nativeUnit[axis]==1U&&(distanceMode==90||block.val("XYZCUV"[axis])!=0.0))linearWord=true;
    }
    // This lane has a native linear mm/min feed metric. Pure rotary moves
    // retain the existing separate ordinary rotary-motion source.
    return linearWord;
}

inline bool IsNCXYZCUVFeedNeutralFrame(const NCTranslationSnapshot& source) noexcept
{
    return IsNCTranslationSnapshotValid(source) &&
        IsNCXYZCUVFeedAxisScopeValid(source.axisIdentity) &&
        source.rotationPlane == 17 && source.unitsMode == 21 &&
        (source.distanceMode == 90 || source.distanceMode == 91) && source.polarMode == 15 &&
        source.cutterMode == 40 && source.toolLengthMode == 49 &&
        source.rotationMode == 69 && source.workMode == 169 &&
        source.scalingMode == 50 && source.mirrorMask == 0U &&
        source.axisIdentity.eccentricEnabled == 0U;
}

inline bool IsNCXYZCUVFeedBlockAllowed(const NCTranslationSnapshot& source,
    const NCBlock& block) noexcept
{
    return IsNCXYZCUVFeedNeutralFrame(source) && IsNCXYZCUVFeedLineBlockAllowed(source.axisIdentity, block, source.distanceMode);
}
