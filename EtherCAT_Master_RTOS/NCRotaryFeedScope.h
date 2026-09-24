#pragma once

#include "NCXYZFeedScope.h"

// BASE69: one positional rotary G01, G90 absolute or G91 delta / degree per minute.
// This classifier supplies no Motion authority, retained geometry or modal F.
// All callers retain source-currentness, owner, epoch and execution-drain gates.
inline bool TryGetNCRotaryFeedAxis(const NCAxisIdentitySnapshot& identity,
    const NCBlock& block, unsigned& axisIndex) noexcept
{
    if (!IsNCXYZFeedAxisScopeValid(identity) ||
        !IsNCXYZFeedSingleGBlock(block, 1) || !block.has('F') ||
        !std::isfinite(block.val('F')) || block.val('F') <= 0.0 ||
        block.val('F') > 100.0 || block.has('P') || block.has('Q')) return false;
    unsigned selected = 8U;
    for (unsigned axis = 3U; axis < 8U; ++axis)
    {
        const char letter = identity.address[axis];
        // G/M are parser-owned, N is a line identifier and F is angular feed.
        // P/Q retain their existing command meanings; never reinterpret them.
        if (identity.exists[axis] != 1U || identity.axisType[axis] != 1U ||
            identity.nativeUnit[axis] != 2U || letter == 'N' || letter == 'F' ||
            letter == 'G' || letter == 'M' || letter == 'P' || letter == 'Q' ||
            !block.has(letter)) continue;
        if (selected != 8U) return false;
        selected = axis;
    }
    if (selected == 8U) return false;
    for (char word = 'A'; word <= 'Z'; ++word)
        if (block.has(word) && ((word != 'N' && word != 'F' &&
            word != identity.address[selected]) || !std::isfinite(block.val(word))))
            return false;
    axisIndex = selected;
    return true;
}

inline bool IsNCRotaryFeedNeutralFrame(const NCTranslationSnapshot& source) noexcept
{
    return IsNCTranslationSnapshotValid(source) &&
        IsNCXYZFeedAxisScopeValid(source.axisIdentity) &&
        source.rotationPlane == 17 && source.unitsMode == 21 &&
        (source.distanceMode == 90 || source.distanceMode == 91) && source.polarMode == 15 &&
        source.cutterMode == 40 && source.toolLengthMode == 49 &&
        source.rotationMode == 69 && source.workMode == 169 &&
        source.scalingMode == 50 && source.mirrorMask == 0U &&
        source.axisIdentity.eccentricEnabled == 0U;
}

inline bool IsNCRotaryFeedBlockAllowed(const NCTranslationSnapshot& source,
    const NCBlock& block) noexcept
{
    unsigned axis = 8U;
    return IsNCRotaryFeedNeutralFrame(source) &&
        TryGetNCRotaryFeedAxis(source.axisIdentity, block, axis);
}
