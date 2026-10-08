#pragma once

#include "NCXYZFeedScope.h"

// BASE75: explicit XYZC and XYZ path F in G90/G91.
// Raw G90 zero is legal; geometry still requires four effective nonzero moves.
// This classifier never supplies geometry, modal feed, or Motion permission.
inline bool IsNCXYZCFeedAxisScopeValid(const NCAxisIdentitySnapshot& identity) noexcept
{
    return IsNCXYZFeedAxisScopeValid(identity) &&
        identity.exists[3] == 1U && identity.axisType[3] == 1U &&
        identity.nativeUnit[3] == 2U && identity.address[3] == 'C';
}

inline bool IsNCXYZCFeedLineBlockAllowed(const NCAxisIdentitySnapshot& identity,
    const NCBlock& block, int distanceMode = 91) noexcept
{
    if ((distanceMode != 90 && distanceMode != 91) ||
        !IsNCXYZCFeedAxisScopeValid(identity) || !IsNCXYZFeedSingleGBlock(block, 1) ||
        !block.has('X') || !block.has('Y') || !block.has('Z') || !block.has('C') || !block.has('F') ||
        (distanceMode == 91 && (block.val('X') == 0.0 || block.val('Y') == 0.0 || block.val('Z') == 0.0 || block.val('C') == 0.0)) ||
        block.val('F') <= 0.0 || block.val('F') > 100.0) return false;
    for (char word = 'A'; word <= 'Z'; ++word)
        if (block.has(word) && ((word != 'N' && word != 'X' && word != 'Y' && word != 'Z' && word != 'C' && word != 'F') ||
            !std::isfinite(block.val(word)))) return false;
    return true;
}

inline bool IsNCXYZCFeedNeutralFrame(const NCTranslationSnapshot& source) noexcept
{
    return IsNCTranslationSnapshotValid(source) &&
        IsNCXYZCFeedAxisScopeValid(source.axisIdentity) &&
        source.rotationPlane == 17 && source.unitsMode == 21 &&
        (source.distanceMode == 90 || source.distanceMode == 91) && source.polarMode == 15 &&
        source.cutterMode == 40 && source.toolLengthMode == 49 &&
        source.rotationMode == 69 && source.workMode == 169 &&
        source.scalingMode == 50 && source.mirrorMask == 0U &&
        source.axisIdentity.eccentricEnabled == 0U;
}

inline bool IsNCXYZCFeedBlockAllowed(const NCTranslationSnapshot& source,
    const NCBlock& block) noexcept
{
    return IsNCXYZCFeedNeutralFrame(source) && IsNCXYZCFeedLineBlockAllowed(source.axisIdentity, block, source.distanceMode);
}
