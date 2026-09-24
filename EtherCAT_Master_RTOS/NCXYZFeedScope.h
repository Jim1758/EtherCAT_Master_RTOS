#pragma once

#include "NC_Types.h"
#include "NCTranslationSnapshot.h"
#include "NCArcPlane.h"

// BASE58: fixed XYZ exact-stop feed while other positional axes are present.
// BASE59 adds native XYZ G91 and standalone G90/G91 frozen transitions.
// BASE60 adds neutral G17 G02/G03, including first-motion source freezing.
// BASE61 adds G18/G19 native XYZ lines and standalone G17/G18/G19 selection.
// BASE62 adds neutral G18/G19 arcs in the existing right-handed plane basis.
// BASE63 adds standalone selection of the existing sixty work-coordinate rows.
// BASE64 adds fixed native XYZ G43/G44 H offsets and standalone H/G49 selection.
// BASE65 adds fixed native XYZ G168 W offsets and standalone W/G169 selection.
// BASE66 adds fixed G68/G69 rotation in the selected G17/G18/G19 plane.
// BASE67 adds native XYZ G50/G51 scale and G150/G151 mirror selectors.
// Plane changes do not rotate native straight-line axis order.
// Auxiliary axes remain stationary native-coordinate slots. This policy never
// defines a mixed mm/degree path metric, rotary target, Motion owner or epoch.
// It is shared by NC whole-block admission and the later motion-entry check.
enum class NCXYZFeedScopeDecision : std::uint8_t
{
    UNCHANGED = 0U,
    ALLOWED = 1U,
    REJECTED = 2U
};

inline std::uint32_t NCXYZFeedPresentMask(const NCAxisIdentitySnapshot& identity) noexcept
{
    std::uint32_t mask = 0U;
    for (unsigned axis = 0U; axis < 8U; ++axis)
        if (identity.exists[axis] != 0U) mask |= 1U << axis;
    return mask;
}

inline bool IsNCXYZFeedAxisScopeValid(const NCAxisIdentitySnapshot& identity) noexcept
{
    if (!IsNCAxisIdentitySnapshotValid(identity) ||
        (NCXYZFeedPresentMask(identity) & ~7U) == 0U) return false;
    for (unsigned axis = 0U; axis < 8U; ++axis)
    {
        if (axis < 3U && (identity.exists[axis] != 1U ||
            identity.axisType[axis] != 0U || identity.address[axis] != "XYZ"[axis]))
            return false;
        if (axis >= 3U && identity.exists[axis] != 0U && identity.axisType[axis] > 1U)
            return false; // Continuous rotation has no BASE57 post-M30 hold.
    }
    return true;
}

inline bool IsNCXYZFeedNeutralFrame(const NCTranslationSnapshot& source) noexcept
{
    return IsNCTranslationSnapshotValid(source) &&
        IsNCXYZFeedAxisScopeValid(source.axisIdentity) &&
        IsNCArcPlaneCode(source.rotationPlane) && source.unitsMode == 21 &&
        (source.distanceMode == 90 || source.distanceMode == 91) &&
        source.polarMode == 15 && source.cutterMode == 40 &&
        (source.toolLengthMode == 49 || source.toolLengthMode == 43 || source.toolLengthMode == 44) &&
        (source.rotationMode == 69 || source.rotationMode == 68) &&
        (source.workMode == 169 || source.workMode == 168) &&
        // WORK fields 3..5 are yaw/pitch/roll, not auxiliary-axis offsets.
        // WORK remains XYZ translation only; validity also proves
        // finite row values, canonical cancellation and zero reserved fields.
        source.workOffset[3] == 0.0 && source.workOffset[4] == 0.0 && source.workOffset[5] == 0.0 &&
        source.workRotationCenterMM[0] == 0.0 && source.workRotationCenterMM[1] == 0.0 &&
        // BASE67: the snapshot validator above proves positive finite scale,
        // finite XYZ centres, XYZ-only mirror bits and canonical cancellation.
        // Auxiliary coordinates retain native units; no affine selector may
        // supply an auxiliary address or enable C-axis eccentric compensation.
        (source.scalingMode == 50 || source.scalingMode == 51) &&
        (source.mirrorMask & ~7U) == 0U && source.axisIdentity.eccentricEnabled == 0U;
}

inline bool IsNCXYZFeedSingleGBlock(const NCBlock& block, int code) noexcept
{
    return !block.isEmpty && !block.isGoto && !block.isBlockSkip &&
        block.hasG && block.gCount == 1 && block.gCode == code &&
        block.gCodes[0] == code && block.mCount == 0;
}

inline bool IsNCXYZFeedLineBlockAllowed(const NCBlock& block, int code) noexcept
{
    if ((code != 0 && code != 1) || !IsNCXYZFeedSingleGBlock(block, code) ||
        !(block.has('X') || block.has('Y') || block.has('Z')) ||
        (code == 1 && !block.has('F'))) return false;
    for (char word = 'A'; word <= 'Z'; ++word)
        if (block.has(word) && ((word != 'N' && word != 'X' && word != 'Y' &&
            word != 'Z' && word != 'F') || !std::isfinite(block.val(word)))) return false;
    // G01 is mm/min; optional G00 F retains the existing percent convention.
    return !block.has('F') || (block.val('F') > 0.0 && block.val('F') <= 100.0);
}

// BASE62: each plane uses its existing native millimetre arc geometry and
// right-handed u/v address order: G17 X/Y I/J, G18 Z/X K/I, G19 Y/Z J/K.
// IJK centers remain offsets from the accepted native start in both modes.
// R selects a partial arc; full circles require an in-plane center word and
// no endpoint words. The normal and all auxiliary axes remain stationary.
// Geometry, complete-sweep travel and source/owner proofs remain downstream.
inline bool IsNCXYZFeedArcBlockAllowed(const NCBlock& block, int planeCode = 17) noexcept
{
    NCArcPlaneAxes plane{};
    if (!TryGetNCArcPlaneAxes(planeCode, plane) ||
        (!IsNCXYZFeedSingleGBlock(block, 2) && !IsNCXYZFeedSingleGBlock(block, 3)) ||
        !block.has('F')) return false;
    for (char word = 'A'; word <= 'Z'; ++word)
        if (block.has(word) && ((word != 'N' && word != 'F' && word != 'R' &&
            word != plane.uAddress && word != plane.vAddress &&
            word != plane.uCenter && word != plane.vCenter) ||
            !std::isfinite(block.val(word)))) return false;
    if (block.val('F') <= 0.0 || block.val('F') > 100.0) return false;
    if (block.has('R'))
        return (block.has(plane.uAddress) || block.has(plane.vAddress)) &&
            block.val('R') != 0.0 && !block.has(plane.uCenter) && !block.has(plane.vCenter);
    return block.has(plane.uCenter) || block.has(plane.vCenter);
}

// Distance selection only admits the standalone spelling. The existing NC
// transition path still requires receipt/RT drain, reserves publication and
// changes the frozen source generation before the normal modal setter runs.
inline bool IsNCXYZFeedDistanceBlockAllowed(const NCBlock& block) noexcept
{
    if (!IsNCXYZFeedSingleGBlock(block, 90) && !IsNCXYZFeedSingleGBlock(block, 91))
        return false;
    for (char word = 'A'; word <= 'Z'; ++word)
        if (block.has(word) && (word != 'N' || !std::isfinite(block.val(word))))
            return false;
    return true;
}

// A plane selector is a complete standalone block. Existing NC transition
// code proves receipt/RT drain and atomically publishes the changed source.
inline bool IsNCXYZFeedPlaneBlockAllowed(const NCBlock& block) noexcept
{
    if (!IsNCXYZFeedSingleGBlock(block, 17) &&
        !IsNCXYZFeedSingleGBlock(block, 18) &&
        !IsNCXYZFeedSingleGBlock(block, 19)) return false;
    for (char word = 'A'; word <= 'Z'; ++word)
        if (block.has(word) && (word != 'N' || !std::isfinite(block.val(word))))
            return false;
    return true;
}

// BASE63: selecting a WCS changes interpretation, never a native target.
// Use the existing ten-bank decoder (G54..G59 through G954..G959); numeric
// gaps are not WCS selectors. The existing NC/Motion handoff still proves
// complete drain, publishes the new frozen source, then commits the selection.
inline bool IsNCXYZFeedWorkCoordinateBlockAllowed(const NCBlock& block) noexcept
{
    if (!IsNCWorkCoordinateCode(block.gCode) ||
        !IsNCXYZFeedSingleGBlock(block, block.gCode)) return false;
    for (char word = 'A'; word <= 'Z'; ++word)
        if (block.has(word) && (word != 'N' || !std::isfinite(block.val(word))))
            return false;
    return true;
}

// BASE64: H is a table row, not a motion address. Keep the existing finite
// integer H1..H100 and canonical cancellation decoder (G49 or G43/G44 H0).
// The validated source permits only XYZ tool offsets; auxiliary H slots must
// remain zero. Live row validation and the drained publication/commit still
// belong to CoordinateManager and the existing NC/Motion handoff.
inline bool IsNCXYZFeedToolLengthBlockAllowed(const NCBlock& block) noexcept
{
    if (!IsNCXYZFeedSingleGBlock(block, 43) &&
        !IsNCXYZFeedSingleGBlock(block, 44) &&
        !IsNCXYZFeedSingleGBlock(block, 49)) return false;
    for (char word = 'A'; word <= 'Z'; ++word)
        if (block.has(word) && ((word != 'N' && word != 'H') ||
            !std::isfinite(block.val(word)))) return false;
    int mode = 49, hCode = 0;
    return TryDecodeNCToolLengthSelection(block.gCode, block.has('H'),
        block.val('H'), mode, hCode);
}

// BASE65: fixed WORK selection has no centre or motion words. Keep the
// existing W1..W100 decoder: G169 (optionally W0) cancels; G168 W0 does not.
// Live row validation and changed-source drain/publication remain in the
// existing CoordinateManager/NC/Motion transition. Idempotent selection keeps
// the current descriptor and generation.
inline bool IsNCXYZFeedWorkpieceBlockAllowed(const NCBlock& block) noexcept
{
    if (!IsNCXYZFeedSingleGBlock(block, 168) &&
        !IsNCXYZFeedSingleGBlock(block, 169)) return false;
    for (char word = 'A'; word <= 'Z'; ++word)
        if (block.has(word) && ((word != 'N' && word != 'W') ||
            !std::isfinite(block.val(word)))) return false;
    int mode = 169, wCode = 0;
    return TryDecodeNCWorkSelection(block.gCode, block.has('W'),
        block.val('W'), mode, wCode);
}

// BASE66: a G68 centre is the complete selected plane pair in G90 WCS,
// followed by finite R within +/-360 degrees. No axis/M/other setting shares
// the block. G69 cancels independently, including in G91. A changed descriptor
// still goes through the existing drained Coordinate/Motion transaction.
inline bool IsNCXYZFeedRotationBlockAllowed(const NCBlock& block,
    int planeCode = 17, int distanceMode = 90) noexcept
{
    NCArcPlaneAxes plane{};
    if (!TryGetNCArcPlaneAxes(planeCode, plane) ||
        (!IsNCXYZFeedSingleGBlock(block, 68) &&
            !IsNCXYZFeedSingleGBlock(block, 69))) return false;
    const bool select = block.gCode == 68;
    if (select && (distanceMode != 90 || !block.has(plane.uAddress) ||
        !block.has(plane.vAddress) || !block.has('R') ||
        std::fabs(block.val('R')) > 360.0)) return false;
    for (char word = 'A'; word <= 'Z'; ++word)
        if (block.has(word) && ((word != 'N' &&
            !(select && (word == plane.uAddress || word == plane.vAddress || word == 'R'))) ||
            !std::isfinite(block.val(word)))) return false;
    return true;
}

// BASE67: a selector owns its entire row and changes only the existing XYZ
// affine descriptor. G51 has one explicit XYZ centre and positive factor;
// G151 selects one or more XYZ mirror centres. G150 cancels selected XYZ
// mirrors, or all mirrors when no axis is present; its finite axis values are
// selection words only. G50 never accepts a centre. The live decoder and the
// drained Coordinate/Motion transition still validate and publish the change.
inline bool IsNCXYZFeedScaleMirrorBlockAllowed(const NCBlock& block) noexcept
{
    const int code = block.gCode;
    if ((code != 50 && code != 51 && code != 150 && code != 151) ||
        !IsNCXYZFeedSingleGBlock(block, code)) return false;
    for (char word = 'A'; word <= 'Z'; ++word)
    {
        if (!block.has(word)) continue;
        const bool xyz = word == 'X' || word == 'Y' || word == 'Z';
        const bool allowed = word == 'N' || (code != 50 && xyz) ||
            (code == 51 && word == 'P');
        if (!allowed || !std::isfinite(block.val(word))) return false;
    }
    if (code == 51)
        return block.has('X') && block.has('Y') && block.has('Z') &&
            block.has('P') && block.val('P') > 0.0;
    if (code == 151)
        return block.has('X') || block.has('Y') || block.has('Z');
    return true;
}

inline bool IsNCXYZFeedFrozenBlockAllowed(const NCBlock& block, int plane = 17,
    int distanceMode = 90) noexcept
{
    if (IsNCXYZFeedLineBlockAllowed(block, 0) || IsNCXYZFeedLineBlockAllowed(block, 1))
        return true;
    if (IsNCXYZFeedArcBlockAllowed(block, plane)) return true;
    if (IsNCXYZFeedDistanceBlockAllowed(block)) return true;
    if (IsNCXYZFeedPlaneBlockAllowed(block)) return true;
    if (IsNCXYZFeedWorkCoordinateBlockAllowed(block)) return true;
    if (IsNCXYZFeedToolLengthBlockAllowed(block)) return true;
    if (IsNCXYZFeedWorkpieceBlockAllowed(block)) return true;
    if (IsNCXYZFeedRotationBlockAllowed(block, plane, distanceMode)) return true;
    if (IsNCXYZFeedScaleMirrorBlockAllowed(block)) return true;
    if (IsNCXYZFeedSingleGBlock(block, 4))
    {
        for (char word = 'A'; word <= 'Z'; ++word)
            if (block.has(word) && ((word != 'N' && word != 'X' && word != 'P') ||
                !std::isfinite(block.val(word)))) return false;
        // The existing BASE50 decoder still validates duration/tick range,
        // including X precedence and the no-argument/zero no-op contract.
        return true;
    }
    if (block.isGoto || block.isBlockSkip || block.hasG || block.gCount != 0) return false;
    if (block.mCount == 1 && !block.isEmpty &&
        (block.mCode[0] == 0 || block.mCode[0] == 30))
    {
        for (char word = 'A'; word <= 'Z'; ++word)
            if (block.has(word) && (word != 'N' || !std::isfinite(block.val(word)))) return false;
        return true;
    }
    if (!block.isEmpty || block.mCount != 0) return false;
    for (char word = 'A'; word <= 'Z'; ++word)
        if (block.has(word)) return false;
    return true;
}

inline NCXYZFeedScopeDecision EvaluateNCXYZFeedScope(
    const NCTranslationSnapshot& source, const NCBlock& block,
    std::uint32_t livePresentMask, bool frozen, bool memoryBound,
    bool runtimeClear) noexcept
{
    using Decision = NCXYZFeedScopeDecision;
    const std::uint32_t capturedMask = NCXYZFeedPresentMask(source.axisIdentity);
    if (((livePresentMask | capturedMask) & ~7U) == 0U) return Decision::UNCHANGED;
    if (!frozen)
    {
        // Before the first G01/G02/G03, preserve existing setup and neutral G00
        // fallback. In particular XYZ G00 must not freeze out a later CUV G00.
        if (block.gCount < 0 || block.gCount > NC_MAX_G_CODES_PER_BLOCK)
            return Decision::REJECTED;
        bool feedAction = block.hasG && block.gCode >= 1 && block.gCode <= 3;
        for (int index = 0; index < block.gCount; ++index)
            if (block.gCodes[index] >= 1 && block.gCodes[index] <= 3) feedAction = true;
        if (!feedAction) return Decision::UNCHANGED;
    }
    if (!memoryBound || !runtimeClear || livePresentMask != capturedMask ||
        !IsNCXYZFeedNeutralFrame(source)) return Decision::REJECTED;
    // An active G68 belongs to its captured plane. Cancel before changing
    // planes; a same-plane standalone selector remains idempotent.
    if (source.rotationMode == 68 && IsNCXYZFeedPlaneBlockAllowed(block) &&
        block.gCode != source.rotationPlane) return Decision::REJECTED;
    return (frozen ? IsNCXYZFeedFrozenBlockAllowed(block, source.rotationPlane, source.distanceMode) :
        (IsNCXYZFeedLineBlockAllowed(block, 1) ||
            IsNCXYZFeedArcBlockAllowed(block, source.rotationPlane))) ?
        Decision::ALLOWED : Decision::REJECTED;
}
