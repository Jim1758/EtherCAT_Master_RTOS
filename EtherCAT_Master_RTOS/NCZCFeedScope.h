#pragma once

#include "NCXYZFeedScope.h"

// BASE70/71 admission only: fixed physical Z+C, G90/G91, explicit words
// and explicit F in Z mm/min. G91 words must be nonzero; G90 raw zero is a
// valid target, with zero effective motion rejected by geometry. Neither source identity
// nor this classifier permits live motion or supplies modal feed state.
inline bool IsNCZCFeedAxisScopeValid(const NCAxisIdentitySnapshot& identity) noexcept
{
    return IsNCXYZFeedAxisScopeValid(identity) &&
        identity.exists[3] == 1U && identity.axisType[3] == 1U &&
        identity.nativeUnit[3] == 2U && identity.address[3] == 'C';
}

inline bool IsNCZCFeedLineBlockAllowed(const NCAxisIdentitySnapshot& identity,
    const NCBlock& block, int distanceMode = 91) noexcept
{
    if ((distanceMode != 90 && distanceMode != 91) ||
        !IsNCZCFeedAxisScopeValid(identity) || !IsNCXYZFeedSingleGBlock(block, 1) ||
        !block.has('Z') || !block.has('C') || !block.has('F') ||
        (distanceMode == 91 && (block.val('Z') == 0.0 || block.val('C') == 0.0)) ||
        block.val('F') <= 0.0 || block.val('F') > 100.0) return false;
    for (char word = 'A'; word <= 'Z'; ++word)
        if (block.has(word) && ((word != 'N' && word != 'Z' && word != 'C' && word != 'F') ||
            !std::isfinite(block.val(word)))) return false;
    return true;
}

inline bool IsNCZCFeedNeutralFrame(const NCTranslationSnapshot& source) noexcept
{
    return IsNCTranslationSnapshotValid(source) &&
        IsNCZCFeedAxisScopeValid(source.axisIdentity) &&
        source.rotationPlane == 17 && source.unitsMode == 21 &&
        (source.distanceMode == 90 || source.distanceMode == 91) && source.polarMode == 15 &&
        source.cutterMode == 40 && source.toolLengthMode == 49 &&
        source.rotationMode == 69 && source.workMode == 169 &&
        source.scalingMode == 50 && source.mirrorMask == 0U &&
        source.axisIdentity.eccentricEnabled == 0U;
}

inline bool IsNCZCFeedBlockAllowed(const NCTranslationSnapshot& source,
    const NCBlock& block) noexcept
{
    return IsNCZCFeedNeutralFrame(source) && IsNCZCFeedLineBlockAllowed(source.axisIdentity, block, source.distanceMode);
}
