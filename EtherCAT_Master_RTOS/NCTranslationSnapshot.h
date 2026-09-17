#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

// Fixed XYZ translation and planar rotation source contract. Native MCS/pulse geometry remains
// authoritative after admission; this value never grants Motion ownership.
// generation is reserved at a fresh run, revision changes before first motion,
// and the complete value is frozen before that motion's first Preview.
struct NCTranslationSnapshot
{
    std::uint64_t runToken = 0ULL;
    std::uint64_t generation = 0ULL;
    std::uint64_t revision = 0ULL;
    std::int32_t wcsCode = 0;
    std::uint32_t schema = 8U;
    double extOffsetMM[8] = {};
    double wcsOffsetMM[8] = {};
    std::int32_t toolLengthMode = 49;
    std::int32_t toolHCode = 0;
    double toolOffsetMM[8] = {}; // Raw H row; sign is determined by G43/G44.
    std::int32_t workMode = 169;
    std::int32_t workWCode = 0;
    double workOffset[8] = {}; // XYZ mm, yaw/pitch/roll degrees, two reserved fields.
    std::int32_t rotationMode = 69;
    std::int32_t rotationPlane = 17;
    double rotationCenterMM[2] = {};
    double rotationAngleDeg = 0.0;
    double workRotationCenterMM[2] = {}; // Fixed G168 centre in native MCS millimetres.
    std::int32_t distanceMode = 90; // Fixed G90 point or G91 displacement semantics.
    std::int32_t unitsMode = 21; // Authored G20/G21; all stored geometry remains native mm/degrees.
    std::int32_t scalingMode = 50;
    std::uint32_t mirrorMask = 0U; // XYZ only; axes 4..8 never inherit geometric scaling.
    double scalingFactor = 1.0;
    double scalingCenterMM[3] = {};
    double mirrorCenterMM[3] = {};
};

inline bool TryDecodeNCWorkSelection(int gCode, bool hasW, double wValue,
    int& mode, int& wCode) noexcept
{
    if (gCode != 168 && gCode != 169) return false;
    if (gCode == 168 && !hasW) return false;
    if (hasW && (!std::isfinite(wValue) || wValue < 0.0 || wValue > 100.0 ||
        std::floor(wValue) != wValue)) return false;
    if ((gCode == 168 && wValue < 1.0) ||
        (gCode == 169 && hasW && wValue != 0.0)) return false;
    mode = gCode;
    wCode = gCode == 168 ? static_cast<int>(wValue) : 0;
    return true;
}

// Decode before any narrowing cast or modal write. H0 is cancellation only
// when it accompanies G43/G44; bare H words are not a command in this scope.
inline bool TryDecodeNCToolLengthSelection(int gCode, bool hasH, double hValue,
    int& mode, int& hCode) noexcept
{
    if (gCode != 43 && gCode != 44 && gCode != 49) return false;
    if (gCode != 49 && !hasH) return false;
    if (hasH && (!std::isfinite(hValue) || hValue < 0.0 || hValue > 100.0 ||
        std::floor(hValue) != hValue)) return false;
    if (gCode == 49 && hasH && hValue != 0.0) return false;
    const int selectedH = hasH ? static_cast<int>(hValue) : 0;
    mode = selectedH == 0 ? 49 : gCode;
    hCode = selectedH;
    return true;
}

inline double NCTranslationToolOffsetMM(const NCTranslationSnapshot& s,
    unsigned axis) noexcept
{
    return s.toolLengthMode == 43 ? s.toolOffsetMM[axis] :
        (s.toolLengthMode == 44 ? -s.toolOffsetMM[axis] : 0.0);
}

inline double NCTranslationAxisOffsetMM(const NCTranslationSnapshot& s,
    unsigned axis) noexcept
{
    // Preserve the accepted G49/no-work arithmetic order, including +0.0.
    return s.extOffsetMM[axis] + s.wcsOffsetMM[axis] +
        NCTranslationToolOffsetMM(s, axis) +
        (s.workMode == 168 && axis < 3U ? s.workOffset[axis] : 0.0);
}

inline bool IsNCTranslationSnapshotEmpty(const NCTranslationSnapshot& s) noexcept
{
    if (s.runToken != 0ULL || s.generation != 0ULL || s.revision != 0ULL ||
        s.wcsCode != 0 || s.schema != 8U || s.toolLengthMode != 49 ||
        s.toolHCode != 0 || s.workMode != 169 || s.workWCode != 0 ||
        s.rotationMode != 69 || s.rotationPlane != 17 ||
        s.rotationCenterMM[0] != 0.0 || s.rotationCenterMM[1] != 0.0 ||
        s.rotationAngleDeg != 0.0 || s.workRotationCenterMM[0] != 0.0 ||
        s.workRotationCenterMM[1] != 0.0 || s.distanceMode != 90 ||
        s.unitsMode != 21 || s.scalingMode != 50 || s.mirrorMask != 0U ||
        s.scalingFactor != 1.0) return false;
    for (unsigned axis = 0U; axis < 3U; ++axis)
        if (s.scalingCenterMM[axis] != 0.0 || std::signbit(s.scalingCenterMM[axis]) ||
            s.mirrorCenterMM[axis] != 0.0 || std::signbit(s.mirrorCenterMM[axis])) return false;
    for (unsigned i = 0U; i < 8U; ++i)
        if (s.extOffsetMM[i] != 0.0 || s.wcsOffsetMM[i] != 0.0 ||
            s.toolOffsetMM[i] != 0.0 || s.workOffset[i] != 0.0) return false;
    return true;
}

inline bool IsNCTranslationSnapshotValid(const NCTranslationSnapshot& s) noexcept
{
    if (s.runToken == 0ULL || s.generation == 0ULL || s.revision == 0ULL ||
        s.schema != 8U || s.wcsCode < 54 || s.wcsCode > 59 ||
        (s.distanceMode != 90 && s.distanceMode != 91) || (s.unitsMode != 20 && s.unitsMode != 21)) return false;
    if ((s.rotationMode != 68 && s.rotationMode != 69) || s.rotationPlane != 17 ||
        !std::isfinite(s.rotationCenterMM[0]) || !std::isfinite(s.rotationCenterMM[1]) ||
        !std::isfinite(s.rotationAngleDeg) || std::fabs(s.rotationAngleDeg) > 360.0 ||
        (s.rotationMode == 69 && (s.rotationCenterMM[0] != 0.0 ||
            s.rotationCenterMM[1] != 0.0 || s.rotationAngleDeg != 0.0))) return false;
    if ((s.scalingMode != 50 && s.scalingMode != 51) || s.mirrorMask > 7U ||
        !std::isfinite(s.scalingFactor) || s.scalingFactor <= 0.0 ||
        (s.scalingMode == 50 && s.scalingFactor != 1.0)) return false;
    for (unsigned axis = 0U; axis < 3U; ++axis)
    {
        if (!std::isfinite(s.scalingCenterMM[axis]) ||
            !std::isfinite(s.mirrorCenterMM[axis])) return false;
        if (s.scalingMode == 50 && (s.scalingCenterMM[axis] != 0.0 ||
            std::signbit(s.scalingCenterMM[axis]))) return false;
        if ((s.mirrorMask & (1U << axis)) == 0U &&
            (s.mirrorCenterMM[axis] != 0.0 || std::signbit(s.mirrorCenterMM[axis]))) return false;
    }
    const bool cancelled = s.toolLengthMode == 49;
    if (cancelled ? s.toolHCode != 0 :
        ((s.toolLengthMode != 43 && s.toolLengthMode != 44) ||
            s.toolHCode < 1 || s.toolHCode > 100)) return false;
    const bool workCancelled = s.workMode == 169;
    if (workCancelled ? s.workWCode != 0 :
        (s.workMode != 168 || s.workWCode < 1 || s.workWCode > 100)) return false;
    if (!std::isfinite(s.workRotationCenterMM[0]) ||
        !std::isfinite(s.workRotationCenterMM[1]) ||
        !std::isfinite(s.workOffset[3]) || std::fabs(s.workOffset[3]) > 360.0 ||
        ((workCancelled || s.workOffset[3] == 0.0) &&
            (s.workRotationCenterMM[0] != 0.0 || s.workRotationCenterMM[1] != 0.0))) return false;
    for (unsigned i = 0U; i < 8U; ++i)
    {
        if (!std::isfinite(s.extOffsetMM[i]) || !std::isfinite(s.wcsOffsetMM[i]) ||
            !std::isfinite(s.toolOffsetMM[i]) || !std::isfinite(s.workOffset[i]) ||
            !std::isfinite(s.extOffsetMM[i] + s.wcsOffsetMM[i]) ||
            !std::isfinite(NCTranslationAxisOffsetMM(s, i))) return false;
        if ((cancelled || i >= 3U) && s.toolOffsetMM[i] != 0.0) return false;
        if ((workCancelled || i >= 4U) && s.workOffset[i] != 0.0) return false;
    }
    return true;
}

inline bool SameNCTranslationSnapshot(const NCTranslationSnapshot& a,
    const NCTranslationSnapshot& b) noexcept
{
    return a.runToken == b.runToken && a.generation == b.generation &&
        a.revision == b.revision && a.wcsCode == b.wcsCode && a.schema == b.schema &&
        a.toolLengthMode == b.toolLengthMode && a.toolHCode == b.toolHCode &&
        a.workMode == b.workMode && a.workWCode == b.workWCode &&
        a.rotationMode == b.rotationMode && a.rotationPlane == b.rotationPlane &&
        a.distanceMode == b.distanceMode && a.unitsMode == b.unitsMode &&
        a.scalingMode == b.scalingMode && a.mirrorMask == b.mirrorMask &&
        std::memcmp(&a.scalingFactor, &b.scalingFactor, sizeof(a.scalingFactor)) == 0 &&
        std::memcmp(a.scalingCenterMM, b.scalingCenterMM, sizeof(a.scalingCenterMM)) == 0 &&
        std::memcmp(a.mirrorCenterMM, b.mirrorCenterMM, sizeof(a.mirrorCenterMM)) == 0 &&
        std::memcmp(a.rotationCenterMM, b.rotationCenterMM, sizeof(a.rotationCenterMM)) == 0 &&
        std::memcmp(&a.rotationAngleDeg, &b.rotationAngleDeg, sizeof(a.rotationAngleDeg)) == 0 &&
        std::memcmp(a.extOffsetMM, b.extOffsetMM, sizeof(a.extOffsetMM)) == 0 &&
        std::memcmp(a.wcsOffsetMM, b.wcsOffsetMM, sizeof(a.wcsOffsetMM)) == 0 &&
        std::memcmp(a.toolOffsetMM, b.toolOffsetMM, sizeof(a.toolOffsetMM)) == 0 &&
        std::memcmp(a.workOffset, b.workOffset, sizeof(a.workOffset)) == 0 &&
        std::memcmp(a.workRotationCenterMM, b.workRotationCenterMM,
            sizeof(a.workRotationCenterMM)) == 0;
}

inline bool IsNCTranslationSourceAllowed(int wcs,
    const NCTranslationSnapshot& snapshot) noexcept
{
    return IsNCTranslationSnapshotValid(snapshot) && wcs == snapshot.wcsCode;
}

inline bool IsNCTranslationToolModeAllowed(int mode,
    const NCTranslationSnapshot& snapshot) noexcept
{
    return IsNCTranslationSnapshotValid(snapshot) && mode == snapshot.toolLengthMode;
}

inline bool IsNCTranslationDistanceModeAllowed(bool absolute,
    const NCTranslationSnapshot& snapshot) noexcept
{
    return IsNCTranslationSnapshotValid(snapshot) &&
        snapshot.distanceMode == (absolute ? 90 : 91);
}

// Unit mode is source provenance only: transforms and Motion use native mm and degrees.
inline bool IsNCTranslationUnitModeAllowed(bool inch,
    const NCTranslationSnapshot& snapshot) noexcept
{
    return IsNCTranslationSnapshotValid(snapshot) && snapshot.unitsMode == (inch ? 20 : 21);
}

// Producer boundary only. The caller validates unitsMode and the finite result.
// Preserve G21 identity arithmetic rather than multiplying by 1.0.
inline double NCTranslationLengthToMM(double value, int unitsMode) noexcept
{
    return unitsMode == 20 ? value * 25.4 : value;
}

inline bool IsNCTranslationWorkModeAllowed(bool active, int wCode,
    const NCTranslationSnapshot& snapshot) noexcept
{
    return IsNCTranslationSnapshotValid(snapshot) &&
        (active ? 168 : 169) == snapshot.workMode && wCode == snapshot.workWCode;
}

// Match command source tags without trigonometry on the RT thread. The full
// descriptor comparison separately proves the centre and all fixed offsets.
inline bool IsNCTranslationRotationModeAllowed(bool active, double angle,
    int plane, const NCTranslationSnapshot& snapshot) noexcept
{
    return IsNCTranslationSnapshotValid(snapshot) &&
        (active ? 68 : 69) == snapshot.rotationMode && plane == snapshot.rotationPlane &&
        std::memcmp(&angle, &snapshot.rotationAngleDeg, sizeof(angle)) == 0;
}

inline bool NCTranslationHasScaleMirror(const NCTranslationSnapshot& s) noexcept
{
    return s.scalingMode == 51 || s.mirrorMask != 0U;
}

inline bool IsNCTranslationScaleMirrorModeAllowed(bool scaling, const bool* mirrors,
    const NCTranslationSnapshot& s) noexcept
{
    if (!mirrors || !IsNCTranslationSnapshotValid(s) ||
        s.scalingMode != (scaling ? 51 : 50)) return false;
    for (unsigned axis = 0U; axis < 8U; ++axis)
        if (mirrors[axis] != (axis < 3U && (s.mirrorMask & (1U << axis)) != 0U)) return false;
    return true;
}

// Authored native displacement -> signed/scaled displacement. Centers never
// participate in vectors; feed remains physical native distance per minute.
inline double NCTranslationScaleMirrorAxisVector(const NCTranslationSnapshot& s,
    double value, unsigned axis) noexcept
{
    if (axis >= 3U) return value;
    if (s.scalingMode == 51 && s.scalingFactor != 1.0) value *= s.scalingFactor;
    if ((s.mirrorMask & (1U << axis)) != 0U) value = -value;
    return value;
}

inline double NCTranslationScaleMirrorAxisPoint(const NCTranslationSnapshot& s,
    double value, unsigned axis) noexcept
{
    if (axis >= 3U) return value;
    if (s.scalingMode == 51 && s.scalingFactor != 1.0)
        value = s.scalingCenterMM[axis] + (value - s.scalingCenterMM[axis]) * s.scalingFactor;
    if ((s.mirrorMask & (1U << axis)) != 0U)
        value = s.mirrorCenterMM[axis] - (value - s.mirrorCenterMM[axis]);
    return value;
}

inline double NCTranslationInverseScaleMirrorAxisPoint(const NCTranslationSnapshot& s,
    double value, unsigned axis) noexcept
{
    if (axis >= 3U) return value;
    if ((s.mirrorMask & (1U << axis)) != 0U)
        value = s.mirrorCenterMM[axis] - (value - s.mirrorCenterMM[axis]);
    if (s.scalingMode == 51 && s.scalingFactor != 1.0)
        value = s.scalingCenterMM[axis] + (value - s.scalingCenterMM[axis]) / s.scalingFactor;
    return value;
}

// NC producer/display helpers only. Geometry is rotated in millimetres before
// independent axis pulse scaling. RT validation never calls these helpers.
inline bool NCTranslationHasPlanarRotation(const NCTranslationSnapshot& s) noexcept
{
    return s.rotationMode == 68 || (s.workMode == 168 && s.workOffset[3] != 0.0);
}

inline void NCTranslationPlanarCoefficients(double angle,
    double& cosine, double& sine) noexcept
{
    cosine = 1.0;
    sine = 0.0;
    if (angle == 0.0 || angle == 360.0 || angle == -360.0) return;
    if (angle == 90.0 || angle == -270.0)
    { cosine = 0.0; sine = 1.0; return; }
    if (angle == -90.0 || angle == 270.0)
    { cosine = 0.0; sine = -1.0; return; }
    if (angle == 180.0 || angle == -180.0)
    { cosine = -1.0; return; }
    const double radian = angle * (3.14159265358979323846 / 180.0);
    cosine = std::cos(radian);
    sine = std::sin(radian);
}

inline void NCTranslationRotationCoefficients(const NCTranslationSnapshot& s,
    double& cosine, double& sine) noexcept
{
    NCTranslationPlanarCoefficients(s.rotationMode == 68 ? s.rotationAngleDeg : 0.0,
        cosine, sine);
}

inline void NCTranslationRotatePlanarVector(double x, double y,
    double cosine, double sine, double& outputX, double& outputY) noexcept
{
    if (cosine == 1.0 && sine == 0.0)
    { outputX = x; outputY = y; return; }
    outputX = x * cosine - y * sine;
    outputY = x * sine + y * cosine;
}

inline void NCTranslationRotateXYVector(const NCTranslationSnapshot& s,
    double x, double y, double& outputX, double& outputY) noexcept
{
    x = NCTranslationScaleMirrorAxisVector(s, x, 0U);
    y = NCTranslationScaleMirrorAxisVector(s, y, 1U);
    double cosine = 1.0, sine = 0.0;
    NCTranslationRotationCoefficients(s, cosine, sine);
    double afterG68X = 0.0, afterG68Y = 0.0;
    NCTranslationRotatePlanarVector(x, y, cosine, sine, afterG68X, afterG68Y);
    NCTranslationPlanarCoefficients(s.workMode == 168 ? s.workOffset[3] : 0.0,
        cosine, sine);
    // I/J are vectors: neither rotation centre nor fixed offsets participate.
    NCTranslationRotatePlanarVector(afterG68X, afterG68Y, cosine, sine, outputX, outputY);
}

inline void NCTranslationForwardPoint(const NCTranslationSnapshot& s,
    const double* wcs, double* mcs) noexcept
{
    double x = NCTranslationScaleMirrorAxisPoint(s, wcs[0], 0U);
    double y = NCTranslationScaleMirrorAxisPoint(s, wcs[1], 1U);
    if (s.rotationMode == 68 && s.rotationAngleDeg != 0.0 &&
        s.rotationAngleDeg != 360.0 && s.rotationAngleDeg != -360.0)
    {
        double cosine = 1.0, sine = 0.0;
        NCTranslationRotationCoefficients(s, cosine, sine);
        NCTranslationRotatePlanarVector(x - s.rotationCenterMM[0],
            y - s.rotationCenterMM[1], cosine, sine, x, y);
        x += s.rotationCenterMM[0];
        y += s.rotationCenterMM[1];
    }
    // Preserve the accepted zero-yaw arithmetic. G68 precedes fixed offsets;
    // G168 rotates their complete result about its separate MCS centre.
    mcs[0] = x + NCTranslationAxisOffsetMM(s, 0U);
    mcs[1] = y + NCTranslationAxisOffsetMM(s, 1U);
    if (s.workMode == 168 && s.workOffset[3] != 0.0 &&
        s.workOffset[3] != 360.0 && s.workOffset[3] != -360.0)
    {
        double cosine = 1.0, sine = 0.0;
        NCTranslationPlanarCoefficients(s.workOffset[3], cosine, sine);
        NCTranslationRotatePlanarVector(mcs[0] - s.workRotationCenterMM[0],
            mcs[1] - s.workRotationCenterMM[1], cosine, sine, x, y);
        mcs[0] = s.workRotationCenterMM[0] + x;
        mcs[1] = s.workRotationCenterMM[1] + y;
    }
    for (unsigned axis = 2U; axis < 8U; ++axis)
        mcs[axis] = NCTranslationScaleMirrorAxisPoint(s, wcs[axis], axis) + NCTranslationAxisOffsetMM(s, axis);
}

inline void NCTranslationInversePoint(const NCTranslationSnapshot& s,
    const double* mcs, double* wcs) noexcept
{
    double unrotatedX = mcs[0], unrotatedY = mcs[1];
    if (s.workMode == 168 && s.workOffset[3] != 0.0 &&
        s.workOffset[3] != 360.0 && s.workOffset[3] != -360.0)
    {
        double cosine = 1.0, sine = 0.0;
        NCTranslationPlanarCoefficients(s.workOffset[3], cosine, sine);
        const double dx = mcs[0] - s.workRotationCenterMM[0];
        const double dy = mcs[1] - s.workRotationCenterMM[1];
        unrotatedX = s.workRotationCenterMM[0] + (dx * cosine + dy * sine);
        unrotatedY = s.workRotationCenterMM[1] + (-dx * sine + dy * cosine);
    }
    const double x = unrotatedX - NCTranslationAxisOffsetMM(s, 0U);
    const double y = unrotatedY - NCTranslationAxisOffsetMM(s, 1U);
    double cosine = 1.0, sine = 0.0;
    NCTranslationRotationCoefficients(s, cosine, sine);
    if (cosine == 1.0 && sine == 0.0)
    { wcs[0] = x; wcs[1] = y; }
    else
    {
        const double dx = x - s.rotationCenterMM[0];
        const double dy = y - s.rotationCenterMM[1];
        wcs[0] = s.rotationCenterMM[0] + (dx * cosine + dy * sine);
        wcs[1] = s.rotationCenterMM[1] + (-dx * sine + dy * cosine);
    }
    for (unsigned axis = 2U; axis < 8U; ++axis)
        wcs[axis] = mcs[axis] - NCTranslationAxisOffsetMM(s, axis);
    for (unsigned axis = 0U; axis < 3U; ++axis)
        wcs[axis] = NCTranslationInverseScaleMirrorAxisPoint(s, wcs[axis], axis);
}

// Fixed G91 uses displacement vectors, not points: neither rotation centre,
// EXT/WCS/H nor G168 translation is added a second time. The caller separately
// proves the sampled Motion baseline still matches this accepted native tail.
// Sparse rotated XY must first be expanded by the completion helper below.
inline bool TryNCTranslationIncrementalTarget(const NCTranslationSnapshot& s,
    const double* commandedMCS, const double* deltaWCS, const bool* hasAxis,
    double* outputMCS) noexcept
{
    if (!commandedMCS || !deltaWCS || !hasAxis || !outputMCS ||
        !IsNCTranslationSnapshotValid(s) || s.distanceMode != 91 ||
        (NCTranslationHasPlanarRotation(s) && hasAxis[0] != hasAxis[1])) return false;
    double delta[8] = {};
    double candidate[8] = {};
    for (unsigned axis = 0U; axis < 8U; ++axis)
    {
        if (!std::isfinite(commandedMCS[axis]) ||
            (hasAxis[axis] && !std::isfinite(deltaWCS[axis]))) return false;
        candidate[axis] = commandedMCS[axis];
        if (hasAxis[axis]) delta[axis] = deltaWCS[axis];
    }
    if (hasAxis[0] || hasAxis[1])
    {
        double rotatedX = 0.0, rotatedY = 0.0;
        NCTranslationRotateXYVector(s, delta[0], delta[1], rotatedX, rotatedY);
        if (!std::isfinite(rotatedX) || !std::isfinite(rotatedY)) return false;
        delta[0] = rotatedX;
        delta[1] = rotatedY;
    }
    delta[2] = NCTranslationScaleMirrorAxisVector(s, delta[2], 2U);
    for (unsigned axis = 0U; axis < 8U; ++axis)
    {
        if (hasAxis[axis]) candidate[axis] = commandedMCS[axis] + delta[axis];
        if (!std::isfinite(candidate[axis])) return false;
    }
    for (unsigned axis = 0U; axis < 8U; ++axis) outputMCS[axis] = candidate[axis];
    return true;
}

inline bool TryCompleteNCTranslationIncrementalEndpoint(const NCTranslationSnapshot& s,
    const double* commandedMCS, double* deltaWCS, bool* hasAxis) noexcept
{
    if (!commandedMCS || !deltaWCS || !hasAxis ||
        !IsNCTranslationSnapshotValid(s) || s.distanceMode != 91) return false;
    double candidateDelta[8] = {};
    bool candidateMask[8] = {};
    for (unsigned axis = 0U; axis < 8U; ++axis)
    {
        candidateDelta[axis] = deltaWCS[axis];
        candidateMask[axis] = hasAxis[axis];
    }
    if (NCTranslationHasPlanarRotation(s) && hasAxis[0] != hasAxis[1])
    {
        const unsigned omitted = hasAxis[0] ? 1U : 0U;
        candidateDelta[omitted] = 0.0;
        candidateMask[omitted] = true;
    }
    double candidateMCS[8] = {};
    if (!TryNCTranslationIncrementalTarget(s, commandedMCS, candidateDelta,
        candidateMask, candidateMCS)) return false;
    for (unsigned axis = 0U; axis < 8U; ++axis)
    {
        deltaWCS[axis] = candidateDelta[axis];
        hasAxis[axis] = candidateMask[axis];
    }
    return true;
}

// Complete only a sparse planar endpoint from the accepted native tail.
// The caller must separately prove that Motion starts from this same tail
// before admission. Original NCBlock presence bits never enter this helper.
inline bool TryCompleteNCTranslationPlanarEndpoint(const NCTranslationSnapshot& s,
    const double* commandedMCS, double* targetWCS, bool* hasAxis) noexcept
{
    if (!commandedMCS || !targetWCS || !hasAxis) return false;
    if (s.distanceMode == 91)
        return TryCompleteNCTranslationIncrementalEndpoint(s, commandedMCS, targetWCS, hasAxis);
    if (hasAxis[0] == hasAxis[1] || !NCTranslationHasPlanarRotation(s)) return true;
    if (!IsNCTranslationSnapshotValid(s) ||
        !std::isfinite(commandedMCS[0]) || !std::isfinite(commandedMCS[1])) return false;
    const unsigned present = hasAxis[0] ? 0U : 1U;
    const unsigned omitted = 1U - present;
    if (!std::isfinite(targetWCS[present])) return false;
    double currentWCS[8] = {};
    NCTranslationInversePoint(s, commandedMCS, currentWCS);
    if (!std::isfinite(currentWCS[0]) || !std::isfinite(currentWCS[1])) return false;
    double candidateWCS[8] = {};
    candidateWCS[present] = targetWCS[present];
    candidateWCS[omitted] = currentWCS[omitted];
    double candidateMCS[8] = {};
    NCTranslationForwardPoint(s, candidateWCS, candidateMCS);
    if (!std::isfinite(candidateMCS[0]) || !std::isfinite(candidateMCS[1])) return false;
    targetWCS[omitted] = candidateWCS[omitted];
    hasAxis[omitted] = true;
    return true;
}

static_assert(sizeof(NCTranslationSnapshot) == 424U &&
    alignof(NCTranslationSnapshot) == 8U,
    "Fixed translation descriptor layout must remain explicit.");
static_assert(std::is_trivially_copyable<NCTranslationSnapshot>::value &&
    std::is_standard_layout<NCTranslationSnapshot>::value,
    "Translation snapshots must be bounded transport values.");

static_assert(offsetof(NCTranslationSnapshot, runToken) == 0U &&
    offsetof(NCTranslationSnapshot, generation) == 8U &&
    offsetof(NCTranslationSnapshot, revision) == 16U &&
    offsetof(NCTranslationSnapshot, wcsCode) == 24U &&
    offsetof(NCTranslationSnapshot, schema) == 28U &&
    offsetof(NCTranslationSnapshot, extOffsetMM) == 32U &&
    offsetof(NCTranslationSnapshot, wcsOffsetMM) == 96U &&
    offsetof(NCTranslationSnapshot, toolLengthMode) == 160U &&
    offsetof(NCTranslationSnapshot, toolHCode) == 164U &&
    offsetof(NCTranslationSnapshot, toolOffsetMM) == 168U &&
    offsetof(NCTranslationSnapshot, workMode) == 232U &&
    offsetof(NCTranslationSnapshot, workWCode) == 236U &&
    offsetof(NCTranslationSnapshot, workOffset) == 240U &&
    offsetof(NCTranslationSnapshot, rotationMode) == 304U &&
    offsetof(NCTranslationSnapshot, rotationPlane) == 308U &&
    offsetof(NCTranslationSnapshot, rotationCenterMM) == 312U &&
    offsetof(NCTranslationSnapshot, rotationAngleDeg) == 328U &&
    offsetof(NCTranslationSnapshot, workRotationCenterMM) == 336U &&
    offsetof(NCTranslationSnapshot, distanceMode) == 352U &&
    offsetof(NCTranslationSnapshot, unitsMode) == 356U &&
    offsetof(NCTranslationSnapshot, scalingMode) == 360U &&
    offsetof(NCTranslationSnapshot, mirrorMask) == 364U &&
    offsetof(NCTranslationSnapshot, scalingFactor) == 368U &&
    offsetof(NCTranslationSnapshot, scalingCenterMM) == 376U &&
    offsetof(NCTranslationSnapshot, mirrorCenterMM) == 400U,
    "Atomic exact comparison requires a padding-free translation descriptor.");
