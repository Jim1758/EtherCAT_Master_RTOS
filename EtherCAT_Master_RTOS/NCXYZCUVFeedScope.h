#pragma once

#include "NCXYZFeedScope.h"

// BASE73/74 admission: explicit XYZCUV and XYZ path F in G90/G91.
// G90 raw zero is valid; geometry rejects an effective zero on any selected axis.
// This classifier never supplies geometry, modal feed, or Motion permission.
inline bool IsNCXYZCUVFeedAxisScopeValid(const NCAxisIdentitySnapshot& identity) noexcept
{
    return IsNCXYZFeedAxisScopeValid(identity) &&
        identity.exists[3] == 1U && identity.axisType[3] == 1U &&
        identity.nativeUnit[3] == 2U && identity.address[3] == 'C' &&
        identity.exists[4] == 1U && identity.axisType[4] == 1U &&
        identity.nativeUnit[4] == 2U && identity.address[4] == 'U' &&
        identity.exists[5] == 1U && identity.axisType[5] == 1U &&
        identity.nativeUnit[5] == 2U && identity.address[5] == 'V';
}

inline bool IsNCXYZCUVFeedLineBlockAllowed(const NCAxisIdentitySnapshot& identity,
    const NCBlock& block, int distanceMode = 91) noexcept
{
    if ((distanceMode != 90 && distanceMode != 91) ||
        !IsNCXYZCUVFeedAxisScopeValid(identity) || !IsNCXYZFeedSingleGBlock(block, 1) ||
        !block.has('X') || !block.has('Y') || !block.has('Z') || !block.has('C') || !block.has('U') || !block.has('V') || !block.has('F') ||
        (distanceMode == 91 && (block.val('X') == 0.0 || block.val('Y') == 0.0 || block.val('Z') == 0.0 ||
            block.val('C') == 0.0 || block.val('U') == 0.0 || block.val('V') == 0.0)) ||
        block.val('F') <= 0.0 || block.val('F') > 100.0) return false;
    for (char word = 'A'; word <= 'Z'; ++word)
        if (block.has(word) && ((word != 'N' && word != 'X' && word != 'Y' && word != 'Z' && word != 'C' && word != 'U' && word != 'V' && word != 'F') ||
            !std::isfinite(block.val(word)))) return false;
    return true;
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
