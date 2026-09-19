#pragma once

#include "NCTranslationSnapshot.h"

#include <cmath>
#include <limits>

// Source-only arithmetic allowance for a planar circle whose native endpoints
// were produced by this frozen affine frame. Native coordinate/pulse arithmetic
// has its own allowance; callers must cap their COMBINED budget at 1e-7 mm.
// No geometry is changed, and this value grants no source or Motion ownership.
//
// The backward envelope bounds terms hidden by cancellation. Each planar
// rotation has L1 amplification <= 2; the final factors propagate earlier
// arithmetic through the two possible rotations without trigonometry on RT.
// Pre-scale terms are weighted by k so every term is in output millimetres.
// Only active XY terms participate. Identity transforms execute no centre
// arithmetic, and Z/auxiliary native units cannot enlarge an XY allowance.
inline bool TryGetNCTranslationArcRoundoffMM(const NCTranslationSnapshot& s,
    float& output, bool fullCircle = false) noexcept
{
    if (!IsNCTranslationSnapshotValid(s)) return false;
    NCArcPlaneAxes plane{};
    if (!TryGetNCArcPlaneAxes(s.rotationPlane, plane)) return false;
    const unsigned slots[2] = { plane.u, plane.v };
    // G91 adds only a transformed displacement to the accepted native tail.
    // A full circle preserves the accepted native start/end bits and only
    // transforms its I/J vector. Neither path evaluates point origins; their
    // vector arithmetic remains covered by the native geometry budget.
    if (s.distanceMode == 91 || fullCircle)
    {
        output = 0.0f;
        return true;
    }

    double workAngle = 0.0;
    if (!TryGetNCTranslationWorkPlaneAngle(s, workAngle)) return false;
    const bool workRotation = workAngle != 0.0 &&
        workAngle != 360.0 && workAngle != -360.0;
    const bool rotation = s.rotationMode == 68 && s.rotationAngleDeg != 0.0 &&
        s.rotationAngleDeg != 360.0 && s.rotationAngleDeg != -360.0;

    double envelope = 0.0;
    if (workRotation)
        envelope = 3.0 * (std::fabs(s.workRotationCenterMM[0]) +
            std::fabs(s.workRotationCenterMM[1]));
    // Add the absolute values separately: EXT/WCS/H/WORK may cancel each
    // other before a small native endpoint is finally obtained.
    for (unsigned component = 0U; component < 2U; ++component)
    {
        const unsigned axis = slots[component];
        envelope += std::fabs(s.extOffsetMM[axis]) + std::fabs(s.wcsOffsetMM[axis]) +
            std::fabs(NCTranslationToolOffsetMM(s, axis)) +
            (s.workMode == 168 ? std::fabs(s.workOffset[axis]) : 0.0);
    }
    if (rotation)
        envelope = 2.0 * envelope + 3.0 *
            (std::fabs(s.rotationCenterMM[0]) + std::fabs(s.rotationCenterMM[1]));
    for (unsigned component = 0U; component < 2U; ++component)
    {
        const unsigned axis = slots[component];
        if ((s.mirrorMask & (1U << axis)) != 0U)
            envelope += 2.0 * std::fabs(s.mirrorCenterMM[axis]);
        if (s.scalingMode == 51 && s.scalingFactor != 1.0)
            envelope += std::fabs(s.scalingCenterMM[axis]) +
                s.scalingFactor * std::fabs(s.scalingCenterMM[axis]);
    }
    if (rotation) envelope *= 2.0;
    if (workRotation) envelope *= 2.0;

    const double allowance = 64.0 * std::numeric_limits<double>::epsilon() * envelope;
    if (!std::isfinite(envelope) || !std::isfinite(allowance) || allowance < 0.0 ||
        allowance > 1.0e-7) return false;

    // Transport fits the existing four-byte geometry padding. Round upwards,
    // including a positive double that converts to float zero, so the stored
    // contract never understates the computed allowance. Oversize values are
    // rejected, never clamped to the cap. Failure leaves output untouched.
    float candidate = static_cast<float>(allowance);
    if (static_cast<double>(candidate) < allowance)
        candidate = std::nextafter(candidate, std::numeric_limits<float>::infinity());
    if (!std::isfinite(candidate) || static_cast<double>(candidate) > 1.0e-7)
        return false;
    output = candidate;
    return true;
}

static_assert(sizeof(float) == 4U && std::numeric_limits<float>::is_iec559,
    "Arc source allowance requires an IEEE binary32 transport field.");
