#pragma once
#include "NC_Types.h"
#include <cmath>

// Pure syntax/axis-owner decoder for the bounded G07/G161 positioning lane.
// No coordinate, Motion, modal or output mutation occurs on rejection.
struct NCPositioningAxisWord
{
    char address = ' ';
    bool enabled = false;
    bool identityValid = false;
};

struct NCPositioningBlockTarget
{
    bool selected[8] = {};
    double authored[8] = {};
};

inline bool TryDecodeNCPositioningBlock(const NCBlock& block, int profileCode,
    const NCPositioningAxisWord (&axisWords)[8], bool frozenXYZOnly,
    NCPositioningBlockTarget& output, int& disabledAxis) noexcept
{
    disabledAxis = -1;
    const int count = block.gCount > 0 ? block.gCount : (block.hasG ? 1 : 0);
    if ((profileCode != 7 && profileCode != 161) || block.gCount < 0 ||
        !block.hasG || count != 1 || block.gCode != profileCode ||
        (block.gCount > 0 && block.gCodes[0] != profileCode) ||
        (block.has('G') && block.val('G') != static_cast<double>(profileCode)) ||
        block.mCount != 0 || block.isGoto || block.isBlockSkip) return false;

    int owners[26];
    for (int& owner : owners) owner = -1;
    for (int axis = 0; axis < 8; ++axis)
    {
        const NCPositioningAxisWord& native = axisWords[axis];
        const char letter = native.address;
        if (!native.enabled && (letter == ' ' || letter == '\0' || letter == 'N')) continue;
        const bool addressValid = letter >= 'A' && letter <= 'Z' &&
            letter != 'G' && letter != 'N' && letter != 'M' && letter != 'T' &&
            letter != 'H' && letter != 'D' && letter != 'F' && letter != 'P' &&
            letter != 'Q' && letter != 'L';
        if (!addressValid || (native.enabled && !native.identityValid) ||
            owners[letter - 'A'] >= 0) return false;
        owners[letter - 'A'] = axis;
    }
    NCPositioningBlockTarget candidate{};
    bool any = false;
    for (char letter = 'A'; letter <= 'Z'; ++letter)
    {
        if (!block.has(letter)) continue;
        const double value = block.val(letter);
        if (!std::isfinite(value)) return false;
        if (letter == 'N' || letter == 'G') continue;
        const int axis = owners[letter - 'A'];
        if (axis < 0) return false; // Includes unsupported F/P and every unowned word.
        if (!axisWords[axis].enabled) { disabledAxis = axis; return false; }
        if (frozenXYZOnly && axis > 2) return false;
        candidate.selected[axis] = true;
        candidate.authored[axis] = value;
        any = true;
    }
    if (!any) return false;
    output = candidate;
    return true;
}
