#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <type_traits>
#include "NCAxisIdentitySnapshot.h"
#include "NCWorkCoordinateCode.h"
#include "NCArcPlane.h"

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
    std::uint32_t schema = 48U;
    double extOffsetMM[8] = {};
    double wcsOffsetMM[8] = {};
    std::int32_t toolLengthMode = 49;
    std::int32_t toolHCode = 0;
    double toolOffsetMM[8] = {}; // Raw H row; sign is determined by G43/G44.
    std::int32_t workMode = 169;
    std::int32_t workWCode = 0;
    double workOffset[8] = {}; // XYZ mm, yaw/pitch/roll degrees, two reserved fields.
    std::int32_t rotationMode = 69;
    std::int32_t rotationPlane = 17; // Active arc/G68 plane. G18/G19 also admit fixed native XYZ H/WORK offsets.
    double rotationCenterMM[2] = {}; // Canonical (u,v): XY / ZX / YZ. Schema 17.
    double rotationAngleDeg = 0.0;
    // Canonical active-plane (u,v) centre in native MCS millimetres.
    // G17=(X,Y), G18=(Z,X), G19=(Y,Z).  BASE-PLANE-7 permits only the
    // matching normal-axis WORK rotation: yaw / pitch / roll respectively.
    double workRotationCenterMM[2] = {};
    std::int32_t distanceMode = 90; // Fixed G90 point or G91 displacement semantics.
    std::int32_t unitsMode = 21; // Authored G20/G21; all stored geometry remains native mm/degrees.
    std::int32_t scalingMode = 50;
    std::uint32_t mirrorMask = 0U; // XYZ only; axes 4..8 never inherit geometric scaling.
    double scalingFactor = 1.0;
    double scalingCenterMM[3] = {};
    double mirrorCenterMM[3] = {};
    std::int32_t polarMode = 15; // Authored endpoint notation; affine geometry stays Cartesian.
    std::int32_t storedStrokeMode = 23; // G22/G23 policy; native bounds remain axis configuration.
    std::int32_t cutterMode = 40;
    std::int32_t cutterD = 0;
    double cutterRadiusMM = 0.0; // Physical tool radius, independent of contour scaling.
    NCAxisIdentitySnapshot axisIdentity{}; // Frozen physical slots, dimensions and electrode role.
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

// D is a table index, never an axis value. Decode before narrowing or modal writes.
inline bool TryDecodeNCToolRadiusSelection(int gCode, bool hasD, double dValue,
    int& mode, int& dCode) noexcept
{
    if (gCode != 40 && gCode != 41 && gCode != 42) return false;
    if (gCode != 40 && !hasD) return false;
    if (hasD && (!std::isfinite(dValue) || dValue < 0.0 || dValue > 100.0 ||
        std::floor(dValue) != dValue)) return false;
    if ((gCode == 40 && hasD) ||
        (gCode != 40 && dValue < 1.0)) return false;
    mode = gCode;
    dCode = gCode == 40 ? 0 : static_cast<int>(dValue);
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

inline unsigned NCTranslationWorkPlaneAngleIndex(int plane) noexcept
{
    return plane == 18 ? 4U : (plane == 19 ? 5U : 3U);
}

inline bool TryGetNCTranslationWorkPlaneAngle(const NCTranslationSnapshot& s,
    double& angle) noexcept
{
    angle = 0.0;
    if (!IsNCArcPlaneCode(s.rotationPlane)) return false;
    if (s.workMode == 169) return s.workWCode == 0;
    if (s.workMode != 168 || s.workWCode < 1 || s.workWCode > 100) return false;
    const unsigned selected = NCTranslationWorkPlaneAngleIndex(s.rotationPlane);
    for (unsigned field = 3U; field <= 5U; ++field)
    {
        if (!std::isfinite(s.workOffset[field]) || std::fabs(s.workOffset[field]) > 360.0)
            return false;
        if (field != selected && s.workOffset[field] != 0.0) return false;
    }
    angle = s.workOffset[selected];
    return true;
}

inline bool NCTranslationHasWorkPlaneRotation(const NCTranslationSnapshot& s) noexcept
{
    double angle = 0.0;
    return TryGetNCTranslationWorkPlaneAngle(s, angle) && angle != 0.0;
}

inline bool IsNCTranslationSnapshotEmpty(const NCTranslationSnapshot& s) noexcept
{
    if (s.runToken != 0ULL || s.generation != 0ULL || s.revision != 0ULL ||
        s.wcsCode != 0 || s.schema != 48U || s.toolLengthMode != 49 ||
        s.toolHCode != 0 || s.workMode != 169 || s.workWCode != 0 ||
        s.rotationMode != 69 || s.rotationPlane != 17 ||
        s.rotationCenterMM[0] != 0.0 || s.rotationCenterMM[1] != 0.0 ||
        s.rotationAngleDeg != 0.0 || s.workRotationCenterMM[0] != 0.0 ||
        s.workRotationCenterMM[1] != 0.0 || s.distanceMode != 90 ||
        s.unitsMode != 21 || s.scalingMode != 50 || s.mirrorMask != 0U ||
        s.scalingFactor != 1.0 || s.polarMode != 15 || s.storedStrokeMode != 23 ||
        s.cutterMode != 40 || s.cutterD != 0 || s.cutterRadiusMM != 0.0 ||
        std::signbit(s.cutterRadiusMM) ||
        !IsNCAxisIdentitySnapshotEmpty(s.axisIdentity)) return false;
    for (unsigned axis = 0U; axis < 3U; ++axis)
        if (s.scalingCenterMM[axis] != 0.0 || std::signbit(s.scalingCenterMM[axis]) ||
            s.mirrorCenterMM[axis] != 0.0 || std::signbit(s.mirrorCenterMM[axis])) return false;
    for (unsigned i = 0U; i < 8U; ++i)
        if (s.extOffsetMM[i] != 0.0 || s.wcsOffsetMM[i] != 0.0 ||
            s.toolOffsetMM[i] != 0.0 || s.workOffset[i] != 0.0) return false;
    return true;
}

// BASE-PLANE-23: syntax opt-in only. All three planes admit G40 G90/G16
// radius-format partial arcs (complete or sparse endpoint). G17 polar cutter
// R is COMPLETE-pair only, enforced by IsCutterContourBlockShapeValid for both
// dispatch and immutable lookahead; G18/G19 retain their sparse contract.
// This predicate grants no ownership, frozen source, continuous motion,
// replay, sparse cutter endpoint or R full-circle permission by itself.
inline bool IsNCPolarRadiusArcNotationAllowed(int plane, bool absolute,
    bool polar, int cutterMode) noexcept
{
    return IsNCArcPlaneCode(plane) && absolute && polar &&
        (cutterMode == 40 || cutterMode == 41 || cutterMode == 42);
}

// BASE-PLANE-29: G17 Cartesian G91 keeps complete literal XY lines and
// partial arcs, and adds an NC-proved IJK single revolution with explicit
// X0/Y0 and tangent-line seams. G18/G19 retain their existing contours.
// Distance mode alone never grants a circle, ownership or continuity.
inline bool IsNCTranslationCutterDistanceModeAllowed(int plane, int mode) noexcept
{
    return IsNCArcPlaneCode(plane) &&
        (mode == 90 || mode == 91);
}

// G16 cutter notation is G90 only. G18/G19 retain nominal-tail sparse
// G01 chords, IJK/signed-R partial arcs and NC-proved IJK seam circles.
// BASE-PLANE-24 adds NC-proved complete-pair G17 G90/G16 IJK seam circles.
// BASE-PLANE-26 also admits sparse G17 polar partial arcs beside G01.
// Block shape, nominal-source identity and circle classification stay separate.
// This general notation predicate alone
// never grants a motion permit, a revolution, blending or replay.
inline bool IsNCTranslationCutterNotationAllowed(int plane, int distance, int polar) noexcept
{
    return IsNCTranslationCutterDistanceModeAllowed(plane, distance) &&
        (polar == 15 || (polar == 16 && distance == 90 && IsNCArcPlaneCode(plane)));
}

// BASE-PLANE-37: Cartesian G90 sparse G01 now also uses the accepted
// NOMINAL contour tail in G18 (Z/X) and G19 (Y/Z). At least one canonical
// plane word is required. G90 retains the omitted AUTHOR coordinate, whereas
// G91 supplies zero omitted delta. Both native plane endpoints are carried,
// including G40's physical offset removal. This grants neither sparse G90
// arcs/circles nor G16, ownership, nonzero-speed junctions or replay.
inline bool IsNCTranslationCutterSparseLineNotationAllowed(int plane,
    int distance, int polar) noexcept
{
    return polar == 15 && IsNCArcPlaneCode(plane) &&
        (distance == 90 || distance == 91);
}

// BASE-PLANE-36: Cartesian G91 PARTIAL G02/G03 may omit one canonical
// plane word in G18 (Z/X) or G19 (Y/Z), alongside the existing G17 lane.
// NC requires at least one in-plane literal and decodes the missing author
// delta as zero from the ACCEPTED NOMINAL contour tail, for both current
// dispatch and immutable lookahead. The producer still proves BOTH native
// plane endpoints, the fixed normal axis and the complete offset arc bounds.
// A sparse zero chord never grants a circle: the existing COMPLETE authored
// explicit-zero/repeated-pair proof and tangent-line seams remain mandatory.
// G17/G90 keeps its existing absolute decoder; G18/G19 G90, G16, ownership,
// nonzero-speed junctions, replay and multi-turn arcs are NOT relaxed.
inline bool IsNCTranslationCutterSparseArcNotationAllowed(int plane,
    int distance, int polar) noexcept
{
    return polar == 15 &&
        ((plane == 17 && (distance == 90 || distance == 91)) ||
         ((plane == 18 || plane == 19) && distance == 91));
}

// Shared NC lookahead / Motion producer / Motion consumer circle scope.
// BASE-PLANE-30 adds G17/G15/G90 IJK single revolutions. The immutable
// NC lookahead must prove the COMPLETE authored XY pair repeats bit-for-bit
// BEFORE affine rounding; current dispatch is bound to that nominal-source
// primitive and forward-tangent line seams. G91 retains explicit zero deltas
// and G90/G16 retains its positive-radius repeated-pair proof. No sparse
// coincidence, rounded chord, R circle, P request, arc entry, blending or
// replay is authorized by this syntax gate. Native packet guards still apply.
inline bool IsNCTranslationCutterArcNotationAllowed(int plane, int distance,
    int polar, bool fullCircle) noexcept
{
    return IsNCTranslationCutterNotationAllowed(plane, distance, polar) &&
        (!fullCircle || plane != 17 || (distance == 90 && (polar == 15 || polar == 16)) ||
            (distance == 91 && polar == 15));
}

inline bool IsNCTranslationBaseArcPlaneFrame(const NCTranslationSnapshot& s) noexcept
{
    double workPlaneAngle = 0.0;
    const bool workPlaneValid = TryGetNCTranslationWorkPlaneAngle(s, workPlaneAngle);
    return IsNCArcPlaneCode(s.rotationPlane) &&
        (s.rotationPlane == 17 || ((s.rotationMode == 68 || s.rotationMode == 69) &&
            // BASE-PLANE-7: H is a signed native XYZ vector. WORK may add a
            // plane-preserving normal-axis rotation only: G18 pitch or G19 roll.
            // Any cross-plane/multi-angle 3D tilt remains outside this lane.
            workPlaneValid &&
            (s.toolLengthMode == 49 || s.toolLengthMode == 43 || s.toolLengthMode == 44) &&
            // Cartesian G90/G91 and G90 polar-line notation are qualified here.
            // Cartesian partial/seam-circle
            // cutter contours. Payload/producer guards prove the canonical
            // plane, physical XYZ start and full offset-path travel bounds.
            (s.cutterMode == 40 || ((s.cutterMode == 41 || s.cutterMode == 42) &&
                IsNCTranslationCutterNotationAllowed(s.rotationPlane, s.distanceMode, s.polarMode))) &&
            (s.polarMode == 15 || (s.polarMode == 16 && s.distanceMode == 90)) &&
            s.axisIdentity.eccentricEnabled == 0U));
}

inline bool IsNCTranslationSnapshotValid(const NCTranslationSnapshot& s) noexcept
{
    if (s.runToken == 0ULL || s.generation == 0ULL || s.revision == 0ULL ||
        s.schema != 48U || !IsNCAxisIdentitySnapshotValid(s.axisIdentity) ||
        !IsNCWorkCoordinateCode(s.wcsCode) ||
        (s.distanceMode != 90 && s.distanceMode != 91) || (s.unitsMode != 20 && s.unitsMode != 21)) return false;
    if ((s.polarMode != 15 && s.polarMode != 16) || (s.storedStrokeMode != 22 && s.storedStrokeMode != 23) ||
        (s.polarMode == 16 && s.distanceMode != 90)) return false;
    if (s.cutterMode == 40)
    {
        if (s.cutterD != 0 || s.cutterRadiusMM != 0.0 ||
            std::signbit(s.cutterRadiusMM)) return false;
    }
    else if ((s.cutterMode != 41 && s.cutterMode != 42) ||
        s.cutterD < 1 || s.cutterD > 100 || !std::isfinite(s.cutterRadiusMM) ||
        s.cutterRadiusMM <= 0.0 || !IsNCTranslationCutterNotationAllowed(s.rotationPlane, s.distanceMode, s.polarMode)) return false;
    if ((s.rotationMode != 68 && s.rotationMode != 69) || !IsNCTranslationBaseArcPlaneFrame(s) ||
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
    double workPlaneAngle = 0.0;
    if (!TryGetNCTranslationWorkPlaneAngle(s, workPlaneAngle) ||
        !std::isfinite(s.workRotationCenterMM[0]) ||
        !std::isfinite(s.workRotationCenterMM[1]) ||
        ((workCancelled || workPlaneAngle == 0.0) &&
            (s.workRotationCenterMM[0] != 0.0 || s.workRotationCenterMM[1] != 0.0))) return false;
    for (unsigned i = 0U; i < 8U; ++i)
    {
        if (!std::isfinite(s.extOffsetMM[i]) || !std::isfinite(s.wcsOffsetMM[i]) ||
            !std::isfinite(s.toolOffsetMM[i]) || !std::isfinite(s.workOffset[i]) ||
            !std::isfinite(s.extOffsetMM[i] + s.wcsOffsetMM[i]) ||
            !std::isfinite(NCTranslationAxisOffsetMM(s, i))) return false;
        if ((cancelled || i >= 3U) && s.toolOffsetMM[i] != 0.0) return false;
        if ((workCancelled || i >= 6U) && s.workOffset[i] != 0.0) return false;
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
        a.polarMode == b.polarMode && a.storedStrokeMode == b.storedStrokeMode &&
        a.cutterMode == b.cutterMode && a.cutterD == b.cutterD &&
        SameNCAxisIdentitySnapshot(a.axisIdentity, b.axisIdentity) &&
        std::memcmp(&a.cutterRadiusMM, &b.cutterRadiusMM, sizeof(double)) == 0 &&
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

inline bool IsNCTranslationCutterModeAllowed(int mode, int dCode,
    const NCTranslationSnapshot& snapshot) noexcept
{
    return IsNCTranslationSnapshotValid(snapshot) &&
        mode == snapshot.cutterMode && dCode == snapshot.cutterD;
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

inline bool IsNCTranslationPolarModeAllowed(bool active,
    const NCTranslationSnapshot& snapshot) noexcept
{
    return IsNCTranslationSnapshotValid(snapshot) &&
        snapshot.polarMode == (active ? 16 : 15);
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
    return s.rotationMode == 68 || NCTranslationHasWorkPlaneRotation(s);
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
    double workAngle = 0.0;
    (void)TryGetNCTranslationWorkPlaneAngle(s, workAngle);
    NCTranslationPlanarCoefficients(workAngle, cosine, sine);
    // I/J are vectors: neither rotation centre nor fixed offsets participate.
    NCTranslationRotatePlanarVector(afterG68X, afterG68Y, cosine, sine, outputX, outputY);
}

// BASE-PLANE-4: signed uniform XYZ scale precedes the same right-handed G68
// rotation for endpoints and IJK vectors. Centers/offsets never enter vectors.
// Only reflections of the two circle axes reverse its direction (producer).
// G17 delegates to its unchanged scale/G68/WORK arithmetic.
inline bool NCTranslationRotateArcVector(const NCTranslationSnapshot& s,
    double u, double v, double& outputU, double& outputV) noexcept
{
    NCArcPlaneAxes plane{};
    if (!TryGetNCArcPlaneAxes(s.rotationPlane, plane)) return false;
    double a = 0.0, b = 0.0;
    if (s.rotationPlane == 17)
        NCTranslationRotateXYVector(s, u, v, a, b);
    else
    {
        double cosine = 1.0, sine = 0.0;
        NCTranslationRotationCoefficients(s, cosine, sine);
        u = NCTranslationScaleMirrorAxisVector(s, u, plane.u);
        v = NCTranslationScaleMirrorAxisVector(s, v, plane.v);
        NCTranslationRotatePlanarVector(u, v, cosine, sine, a, b);
        double workAngle = 0.0;
        if (!TryGetNCTranslationWorkPlaneAngle(s, workAngle)) return false;
        NCTranslationPlanarCoefficients(workAngle, cosine, sine);
        NCTranslationRotatePlanarVector(a, b, cosine, sine, u, v);
        a = u;
        b = v;
    }
    if (!std::isfinite(a) || !std::isfinite(b)) return false;
    outputU = a; outputV = b;
    return true;
}

inline bool NCTranslationHasSparseRotatedEndpoint(const NCTranslationSnapshot& s,
    const bool* hasAxis) noexcept
{
    NCArcPlaneAxes plane{};
    return hasAxis != nullptr && NCTranslationHasPlanarRotation(s) &&
        TryGetNCArcPlaneAxes(s.rotationPlane, plane) && hasAxis[plane.u] != hasAxis[plane.v];
}

inline void NCTranslationForwardPoint(const NCTranslationSnapshot& s,
    const double* wcs, double* mcs) noexcept
{
    // BASE-PLANE-7: XYZ scale -> XYZ mirror -> plane G68 -> fixed offsets ->
    // plane-preserving G168. The normal axis is scaled/mirrored and translated,
    // but the planar rotations never mix it. Keep the old G17 arithmetic order.
    if (s.rotationPlane == 18 || s.rotationPlane == 19)
    {
        NCArcPlaneAxes plane{};
        (void)TryGetNCArcPlaneAxes(s.rotationPlane, plane);
        double u = NCTranslationScaleMirrorAxisPoint(s, wcs[plane.u], plane.u);
        double v = NCTranslationScaleMirrorAxisPoint(s, wcs[plane.v], plane.v);
        double cosine = 1.0, sine = 0.0;
        NCTranslationRotationCoefficients(s, cosine, sine);
        if (cosine != 1.0 || sine != 0.0)
        {
            NCTranslationRotatePlanarVector(u - s.rotationCenterMM[0],
                v - s.rotationCenterMM[1], cosine, sine, u, v);
            u += s.rotationCenterMM[0]; v += s.rotationCenterMM[1];
        }
        for (unsigned axis = 0U; axis < 8U; ++axis)
            mcs[axis] = NCTranslationScaleMirrorAxisPoint(s, wcs[axis], axis) +
                NCTranslationAxisOffsetMM(s, axis);
        mcs[plane.u] = u + NCTranslationAxisOffsetMM(s, plane.u);
        mcs[plane.v] = v + NCTranslationAxisOffsetMM(s, plane.v);
        double workAngle = 0.0;
        (void)TryGetNCTranslationWorkPlaneAngle(s, workAngle);
        NCTranslationPlanarCoefficients(workAngle, cosine, sine);
        if (cosine != 1.0 || sine != 0.0)
        {
            NCTranslationRotatePlanarVector(
                mcs[plane.u] - s.workRotationCenterMM[0],
                mcs[plane.v] - s.workRotationCenterMM[1],
                cosine, sine, u, v);
            mcs[plane.u] = s.workRotationCenterMM[0] + u;
            mcs[plane.v] = s.workRotationCenterMM[1] + v;
        }
        return;
    }
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
    double workAngle = 0.0;
    (void)TryGetNCTranslationWorkPlaneAngle(s, workAngle);
    if (workAngle != 0.0 && workAngle != 360.0 && workAngle != -360.0)
    {
        double cosine = 1.0, sine = 0.0;
        NCTranslationPlanarCoefficients(workAngle, cosine, sine);
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
    if (s.rotationPlane == 18 || s.rotationPlane == 19)
    {
        NCArcPlaneAxes plane{};
        (void)TryGetNCArcPlaneAxes(s.rotationPlane, plane);
        double unrotated[8] = {};
        for (unsigned axis = 0U; axis < 8U; ++axis) unrotated[axis] = mcs[axis];
        double workAngle = 0.0;
        (void)TryGetNCTranslationWorkPlaneAngle(s, workAngle);
        double cosine = 1.0, sine = 0.0;
        NCTranslationPlanarCoefficients(workAngle, cosine, sine);
        if (cosine != 1.0 || sine != 0.0)
        {
            const double du = mcs[plane.u] - s.workRotationCenterMM[0];
            const double dv = mcs[plane.v] - s.workRotationCenterMM[1];
            unrotated[plane.u] = s.workRotationCenterMM[0] + (du * cosine + dv * sine);
            unrotated[plane.v] = s.workRotationCenterMM[1] + (-du * sine + dv * cosine);
        }
        for (unsigned axis = 0U; axis < 8U; ++axis)
            wcs[axis] = unrotated[axis] - NCTranslationAxisOffsetMM(s, axis);
        NCTranslationRotationCoefficients(s, cosine, sine);
        if (cosine != 1.0 || sine != 0.0)
        {
            const double du = wcs[plane.u] - s.rotationCenterMM[0];
            const double dv = wcs[plane.v] - s.rotationCenterMM[1];
            wcs[plane.u] = s.rotationCenterMM[0] + (du * cosine + dv * sine);
            wcs[plane.v] = s.rotationCenterMM[1] + (-du * sine + dv * cosine);
        }
        for (unsigned axis = 0U; axis < 3U; ++axis)
            wcs[axis] = NCTranslationInverseScaleMirrorAxisPoint(s, wcs[axis], axis);
        return;
    }
    double unrotatedX = mcs[0], unrotatedY = mcs[1];
    double workAngle = 0.0;
    (void)TryGetNCTranslationWorkPlaneAngle(s, workAngle);
    if (workAngle != 0.0 && workAngle != 360.0 && workAngle != -360.0)
    {
        double cosine = 1.0, sine = 0.0;
        NCTranslationPlanarCoefficients(workAngle, cosine, sine);
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

// Producer boundary only: canonical u is radius in native mm, v is degrees.
// G17 X/Y, G18 Z/X, G19 Y/Z. The normal axis remains a Cartesian length.
// The accepted native tail supplies omitted polar words through the Cartesian
// affine inverse. No modal radius/angle cache can diverge from the native tail.
// On success the selected plane pair is Cartesian; decode exactly once.
inline bool TryCompleteNCTranslationPolarEndpoint(const NCTranslationSnapshot& s,
    const double* baselineMCS, double* targetWCS, bool* hasAxis) noexcept
{
    if (!baselineMCS || !targetWCS || !hasAxis ||
        !IsNCTranslationSnapshotValid(s) || s.polarMode != 16 ||
        s.distanceMode != 90) return false;
    NCArcPlaneAxes plane{};
    if (!TryGetNCArcPlaneAxes(s.rotationPlane, plane)) return false;
    const unsigned u = plane.u, v = plane.v, normal = plane.normal;
    for (unsigned axis = 0U; axis < 8U; ++axis)
        if (!std::isfinite(baselineMCS[axis]) ||
            (hasAxis[axis] && (axis >= 3U || !std::isfinite(targetWCS[axis])))) return false;
    // A normal-only block does not manufacture plane presence or rewrite words.
    if (!hasAxis[u] && !hasAxis[v]) return true;
    double radius = hasAxis[u] ? targetWCS[u] : 0.0;
    double angle = hasAxis[v] ? targetWCS[v] : 0.0;
    if (!hasAxis[u] || !hasAxis[v])
    {
        double baselineWCS[8] = {};
        NCTranslationInversePoint(s, baselineMCS, baselineWCS);
        if (!std::isfinite(baselineWCS[u]) || !std::isfinite(baselineWCS[v])) return false;
        const double baselineRadius = std::hypot(baselineWCS[u], baselineWCS[v]);
        if (!std::isfinite(baselineRadius)) return false;
        if (!hasAxis[u]) radius = baselineRadius;
        if (!hasAxis[v])
        {
            // A transformed origin can inverse-map to a tiny nonzero residue.
            // Do not infer an arbitrary direction from cancellation/roundoff.
            double originWCS[8] = {}, originMCS[8] = {};
            NCTranslationForwardPoint(s, originWCS, originMCS);
            double magnitude = 1.0;
            for (unsigned component = 0U; component < 2U; ++component)
            {
                const unsigned axis = component == 0U ? u : v;
                const double scaledCenter = s.scalingCenterMM[axis] * s.scalingFactor;
                const double terms[] = { baselineMCS[axis], originMCS[axis],
                    s.extOffsetMM[axis], s.wcsOffsetMM[axis], s.toolOffsetMM[axis],
                    s.workOffset[axis], s.scalingCenterMM[axis], scaledCenter,
                    s.mirrorCenterMM[axis], s.rotationCenterMM[component],
                    s.workRotationCenterMM[component] };
                for (double term : terms)
                {
                    if (!std::isfinite(term)) return false;
                    if (std::fabs(term) > magnitude) magnitude = std::fabs(term);
                }
            }
            const double originBudget = 64.0 * (std::numeric_limits<double>::epsilon)() * magnitude;
            if (baselineRadius == 0.0 ||
                std::hypot(baselineMCS[u] - originMCS[u],
                    baselineMCS[v] - originMCS[v]) <= originBudget) return false;
            angle = std::atan2(baselineWCS[v], baselineWCS[u]) *
                (180.0 / 3.14159265358979323846);
        }
    }
    if (!std::isfinite(radius) || radius < 0.0 || !std::isfinite(angle)) return false;
    // Reduce finite multi-turn angles before multiplication, avoiding overflow.
    angle = std::fmod(angle, 360.0);
    double cosine = 1.0, sine = 0.0;
    NCTranslationPlanarCoefficients(angle, cosine, sine);
    const double x = radius == 0.0 ? 0.0 : radius * cosine;
    const double y = radius == 0.0 ? 0.0 : radius * sine;
    if (!std::isfinite(x) || !std::isfinite(y)) return false;
    double cartesian[8] = {};
    cartesian[u] = x;
    cartesian[v] = y;
    if (hasAxis[normal]) cartesian[normal] = targetWCS[normal];
    double native[8] = {};
    NCTranslationForwardPoint(s, cartesian, native);
    if (!std::isfinite(native[u]) || !std::isfinite(native[v]) ||
        (hasAxis[normal] && !std::isfinite(native[normal]))) return false;
    targetWCS[u] = x;
    targetWCS[v] = y;
    hasAxis[u] = true;
    hasAxis[v] = true;
    return true;
}

// BASE-PLANE-13 NC-only complete G90 polar cutter endpoint. Historical
// function name retained: the same point decoder serves a chord or partial arc. The caller
// supplies both radius (mm) and angle (degrees); no inverse or modal fallback
// may use the physical offset tail as a nominal polar baseline. Decode once,
// then apply the SAME immutable affine point transform as non-cutter G16.
// All untouched physical axes, including the normal, are preserved bitwise.
// On any failure outputMCS is unchanged. This helper grants no motion permit.
inline bool TryNCTranslationCutterPolarLineEndpoint(const NCTranslationSnapshot& s,
    const double* physicalMCS, double radiusMM, double angleDeg, double* outputMCS) noexcept
{
    NCArcPlaneAxes plane{};
    if (!physicalMCS || !outputMCS || !IsNCTranslationSnapshotValid(s) ||
        !IsNCTranslationCutterNotationAllowed(s.rotationPlane, s.distanceMode, s.polarMode) ||
        s.polarMode != 16 || !TryGetNCArcPlaneAxes(s.rotationPlane, plane)) return false;
    double decoded[8] = {}, transformed[8] = {}, candidate[8] = {};
    bool selected[8] = {};
    selected[plane.u] = selected[plane.v] = true;
    decoded[plane.u] = radiusMM; decoded[plane.v] = angleDeg;
    if (!TryCompleteNCTranslationPolarEndpoint(s, physicalMCS, decoded, selected)) return false;
    NCTranslationForwardPoint(s, decoded, transformed);
    for (unsigned axis = 0U; axis < 8U; ++axis)
    {
        candidate[axis] = selected[axis] ? transformed[axis] : physicalMCS[axis];
        if (!std::isfinite(candidate[axis])) return false;
    }
    std::memcpy(outputMCS, candidate, sizeof(candidate));
    return true;
}

// NC-only sparse G90 polar cutter endpoint. BASE-PLANE-26 uses the same
// nominal-source decoder for G17 G01 and partial G02/G03, as for G18/G19.
// This pure helper has no block/opcode and grants no arc/full-circle admission.
// Shared by a G01 chord/lead-out or a PARTIAL G02/G03 primitive. This helper
// decodes a point only and never authorizes a full circle or a Motion packet.
// Missing words come from the accepted NOMINAL contour, never the offset
// physical tool-centre tail. Physical unselected axes are copied unchanged.
// Authored radius is already in mm; an inferred radius is never converted twice.
// Both arrays remain separate even during G40 lead-out. This grants no permit;
// the NC caller proves run/cache/generation/plane and physical-tail continuity.
// Complete-pair calls retain the established decoder and numerical operation order.
// Failure is atomic, including aliased input/output storage.
inline bool TryNCTranslationCutterSparsePolarEndpoint(const NCTranslationSnapshot& s,
    const double* nominalMCS, const double* physicalMCS, double radiusMM,
    double angleDeg, std::uint32_t authoredEndpointMask, double* outputMCS) noexcept
{
    NCArcPlaneAxes plane{};
    if (!nominalMCS || !physicalMCS || !outputMCS ||
        !IsNCTranslationSnapshotValid(s) || s.polarMode != 16 ||
        !IsNCTranslationCutterNotationAllowed(s.rotationPlane, s.distanceMode, s.polarMode) ||
        !TryGetNCArcPlaneAxes(s.rotationPlane, plane) ||
        authoredEndpointMask == 0U || (authoredEndpointMask & ~plane.mask) != 0U) return false;
    for (unsigned axis = 0U; axis < 8U; ++axis)
    {
        if (!std::isfinite(nominalMCS[axis]) || !std::isfinite(physicalMCS[axis])) return false;
        if (axis != plane.u && axis != plane.v &&
            std::memcmp(&nominalMCS[axis], &physicalMCS[axis], sizeof(double)) != 0) return false;
    }
    if (authoredEndpointMask == plane.mask)
        return TryNCTranslationCutterPolarLineEndpoint(s, physicalMCS, radiusMM, angleDeg, outputMCS);
    double decoded[8] = {}, transformed[8] = {}, candidate[8] = {};
    bool selected[8] = {};
    selected[plane.u] = (authoredEndpointMask & (1U << plane.u)) != 0U;
    selected[plane.v] = (authoredEndpointMask & (1U << plane.v)) != 0U;
    if (selected[plane.u]) decoded[plane.u] = radiusMM;
    if (selected[plane.v]) decoded[plane.v] = angleDeg;
    if (!TryCompleteNCTranslationPolarEndpoint(s, nominalMCS, decoded, selected)) return false;
    NCTranslationForwardPoint(s, decoded, transformed);
    for (unsigned axis = 0U; axis < 8U; ++axis)
    {
        candidate[axis] = (axis == plane.u || axis == plane.v) ? transformed[axis] : physicalMCS[axis];
        if (!std::isfinite(candidate[axis])) return false;
    }
    std::memcpy(outputMCS, candidate, sizeof(candidate));
    return true;
}

// Preserve the BASE-PLANE-18 line-only entry name and numerical path for
// existing callers. Motion/arc classification remains owned by the NC caller.
inline bool TryNCTranslationCutterSparsePolarLineEndpoint(const NCTranslationSnapshot& s,
    const double* nominalMCS, const double* physicalMCS, double radiusMM,
    double angleDeg, std::uint32_t authoredEndpointMask, double* outputMCS) noexcept
{
    return TryNCTranslationCutterSparsePolarEndpoint(s, nominalMCS, physicalMCS,
        radiusMM, angleDeg, authoredEndpointMask, outputMCS);
}

// BASE-PLANE-33/34 NC-only G17/G15/G90 contour endpoint. The caller proves
// run/cache/generation/primitive identity and G01 or a permitted PARTIAL arc.
// G40 lead-out is still G01; full-circle proof remains a separate contract.
// Missing author coordinates come only from the accepted nominal contour.
// Full pairs retain the established forward-point arithmetic. Without planar
// rotations the omitted native NOMINAL axis is separable and copied bitwise;
// do not inject an inverse/forward rounding displacement on that fixed axis.
// With rotations, inverse the nominal point, replace only authored values,
// then forward both axes in the same frozen frame. D is not part of this map.
// All unselected physical axes are copied bitwise. No mutation on failure,
// even if output aliases either input. No circle or Motion permit is granted.
// BASE-PLANE-37: uMM/vMM are canonical plane values (XY / ZX / YZ).
// authoredMask retains native XYZ bit positions; never reinterpret G18 as X/Z.
inline bool TryNCTranslationCutterAbsoluteEndpoint(const NCTranslationSnapshot& s,
    const double* nominalMCS, const double* physicalMCS, double uMM, double vMM,
    std::uint32_t authoredMask, double* outputMCS) noexcept
{
    NCArcPlaneAxes plane{};
    if (!nominalMCS || !physicalMCS || !outputMCS ||
        !IsNCTranslationSnapshotValid(s) || !TryGetNCArcPlaneAxes(s.rotationPlane, plane) ||
        s.distanceMode != 90 || s.polarMode != 15 ||
        s.axisIdentity.eccentricEnabled != 0U ||
        !IsNCTranslationCutterSparseLineNotationAllowed(s.rotationPlane, s.distanceMode, s.polarMode) ||
        authoredMask == 0U || (authoredMask & ~plane.mask) != 0U) return false;
    for (unsigned axis = 0U; axis < 8U; ++axis)
    {
        if (!std::isfinite(nominalMCS[axis]) || !std::isfinite(physicalMCS[axis])) return false;
        if (axis != plane.u && axis != plane.v &&
            std::memcmp(&nominalMCS[axis], &physicalMCS[axis], sizeof(double)) != 0)
            return false;
    }
    if (((authoredMask & (1U << plane.u)) != 0U && !std::isfinite(uMM)) ||
        ((authoredMask & (1U << plane.v)) != 0U && !std::isfinite(vMM))) return false;
    double decoded[8] = {}, transformed[8] = {}, candidate[8] = {};
    const bool rotated = NCTranslationHasPlanarRotation(s);
    if (authoredMask != plane.mask && rotated)
    {
        NCTranslationInversePoint(s, nominalMCS, decoded);
        if (!std::isfinite(decoded[plane.u]) || !std::isfinite(decoded[plane.v])) return false;
    }
    if ((authoredMask & (1U << plane.u)) != 0U) decoded[plane.u] = uMM;
    if ((authoredMask & (1U << plane.v)) != 0U) decoded[plane.v] = vMM;
    NCTranslationForwardPoint(s, decoded, transformed);
    for (unsigned axis = 0U; axis < 8U; ++axis)
    {
        candidate[axis] = (axis == plane.u || axis == plane.v) ?
            ((!rotated && (authoredMask & (1U << axis)) == 0U) ? nominalMCS[axis] : transformed[axis]) :
            physicalMCS[axis];
        if (!std::isfinite(candidate[axis])) return false;
    }
    std::memcpy(outputMCS, candidate, sizeof(candidate));
    return true;
}

// Preserve the established line-only call site interface.
inline bool TryNCTranslationCutterAbsoluteLineEndpoint(const NCTranslationSnapshot& s,
    const double* nominalMCS, const double* physicalMCS, double uMM, double vMM,
    std::uint32_t authoredMask, double* outputMCS) noexcept
{
    return TryNCTranslationCutterAbsoluteEndpoint(s, nominalMCS, physicalMCS,
        uMM, vMM, authoredMask, outputMCS);
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
        NCTranslationHasSparseRotatedEndpoint(s, hasAxis)) return false;
    double delta[8] = {};
    double candidate[8] = {};
    for (unsigned axis = 0U; axis < 8U; ++axis)
    {
        if (!std::isfinite(commandedMCS[axis]) ||
            (hasAxis[axis] && !std::isfinite(deltaWCS[axis]))) return false;
        candidate[axis] = commandedMCS[axis];
        if (hasAxis[axis]) delta[axis] = deltaWCS[axis];
    }
    if (s.rotationPlane != 17)
    {
        NCArcPlaneAxes plane{};
        if (!TryGetNCArcPlaneAxes(s.rotationPlane, plane)) return false;
        if (hasAxis[plane.u] || hasAxis[plane.v])
        {
            double u = 0.0, v = 0.0;
            if (!NCTranslationRotateArcVector(s, delta[plane.u], delta[plane.v], u, v)) return false;
            delta[plane.u] = u; delta[plane.v] = v;
        }
        // A normal-axis LINEAR displacement still uses XYZ scaling/mirror.
        // No circle is permitted to move this axis in the current scope.
        delta[plane.normal] = NCTranslationScaleMirrorAxisVector(s, delta[plane.normal], plane.normal);
    }
    else
    {
        if (hasAxis[0] || hasAxis[1])
        {
            double rotatedX = 0.0, rotatedY = 0.0;
            NCTranslationRotateXYVector(s, delta[0], delta[1], rotatedX, rotatedY);
            if (!std::isfinite(rotatedX) || !std::isfinite(rotatedY)) return false;
            delta[0] = rotatedX;
            delta[1] = rotatedY;
        }
        delta[2] = NCTranslationScaleMirrorAxisVector(s, delta[2], 2U);
    }
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
    if (NCTranslationHasSparseRotatedEndpoint(s, hasAxis))
    {
        NCArcPlaneAxes plane{};
        if (!TryGetNCArcPlaneAxes(s.rotationPlane, plane)) return false;
        const unsigned omitted = hasAxis[plane.u] ? plane.v : plane.u;
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
    if (!NCTranslationHasSparseRotatedEndpoint(s, hasAxis)) return true;
    NCArcPlaneAxes plane{};
    if (!TryGetNCArcPlaneAxes(s.rotationPlane, plane)) return false;
    if (!IsNCTranslationSnapshotValid(s) ||
        !std::isfinite(commandedMCS[plane.u]) || !std::isfinite(commandedMCS[plane.v])) return false;
    const unsigned present = hasAxis[plane.u] ? plane.u : plane.v;
    const unsigned omitted = hasAxis[plane.u] ? plane.v : plane.u;
    if (!std::isfinite(targetWCS[present])) return false;
    double currentWCS[8] = {};
    NCTranslationInversePoint(s, commandedMCS, currentWCS);
    if (!std::isfinite(currentWCS[plane.u]) || !std::isfinite(currentWCS[plane.v])) return false;
    double candidateWCS[8] = {};
    candidateWCS[present] = targetWCS[present];
    candidateWCS[omitted] = currentWCS[omitted];
    double candidateMCS[8] = {};
    NCTranslationForwardPoint(s, candidateWCS, candidateMCS);
    if (!std::isfinite(candidateMCS[plane.u]) || !std::isfinite(candidateMCS[plane.v])) return false;
    targetWCS[omitted] = candidateWCS[omitted];
    hasAxis[omitted] = true;
    return true;
}

static_assert(sizeof(NCTranslationSnapshot) == 496U &&
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
    offsetof(NCTranslationSnapshot, mirrorCenterMM) == 400U &&
    offsetof(NCTranslationSnapshot, polarMode) == 424U &&
    offsetof(NCTranslationSnapshot, storedStrokeMode) == 428U &&
    offsetof(NCTranslationSnapshot, cutterMode) == 432U &&
    offsetof(NCTranslationSnapshot, cutterD) == 436U &&
    offsetof(NCTranslationSnapshot, cutterRadiusMM) == 440U &&
    offsetof(NCTranslationSnapshot, axisIdentity) == 448U,
    "Atomic exact comparison requires a padding-free translation descriptor.");
