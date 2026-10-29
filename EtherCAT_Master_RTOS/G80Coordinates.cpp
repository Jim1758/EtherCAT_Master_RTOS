#include "G80Coordinates.h"
#include "NCEccentricCPath.h"
#include <cmath>
#include <limits>

namespace G80
{
namespace
{
bool Fail(CoordinateError reason, CoordinateError& error) noexcept
{
    error = reason;
    return false;
}

bool Finite(const Point6& point) noexcept
{
    for (double value : point) if (!std::isfinite(value)) return false;
    return true;
}

bool ValidateFrame(const NCTranslationSnapshot& frame, CoordinateError& error) noexcept
{
    if (!IsNCTranslationSnapshotValid(frame)) return Fail(CoordinateError::InvalidFrame, error);
    if (frame.cutterMode != 40) return Fail(CoordinateError::UnsupportedCutter, error);
    const auto& identity = frame.axisIdentity;
    for (unsigned axis = 0U; axis < 6U; ++axis)
        if (identity.exists[axis] && (identity.address[axis] != "XYZCUV"[axis] ||
            identity.nativeUnit[axis] != (axis < 3U ? 1U : 2U)))
            return Fail(CoordinateError::AxisIdentity, error);
    return true;
}

bool HasEccentricH(const NCTranslationSnapshot& frame) noexcept
{
    return frame.axisIdentity.eccentricEnabled != 0U && frame.toolLengthMode != 49;
}

bool EccentricRole(const NCTranslationSnapshot& frame, unsigned& role,
    CoordinateError& error) noexcept
{
    // Reuse the electrode-role and physical-Z signed-H model. The existing
    // path's Scope adds trajectory-specific restrictions which do not apply
    // to this single-point derivative/inverse (no eccentric arc is executed).
    if (!NCEccentricCDetail::Role(frame, role) || role >= 6U ||
        frame.axisIdentity.systemMode != 1U)
        return Fail(CoordinateError::EccentricRole, error);
    return true;
}

bool FrameAtAngle(const NCTranslationSnapshot& frame, const Point6& machine,
    NCTranslationSnapshot& evaluated, CoordinateError& error) noexcept
{
    evaluated = frame;
    if (!HasEccentricH(frame)) return true;
    unsigned role = 8U;
    if (!EccentricRole(frame, role, error)) return false;
    std::array<double, 3U> signedH{};
    if (!NCEccentricCDetail::Offset(frame, machine[role], signedH))
        return Fail(CoordinateError::NumericRange, error);
    const double sign = frame.toolLengthMode == 43 ? 1.0 : -1.0;
    // Generic affine helpers apply the G43/G44 sign themselves. H remains a
    // native vector inserted after G68/scale and before the fixed WORK rotation.
    for (unsigned axis = 0U; axis < 3U; ++axis)
        evaluated.toolOffsetMM[axis] = sign * signedH[axis];
    return true;
}

bool RotateWork(const NCTranslationSnapshot& frame, double* vector,
    bool inverse, CoordinateError& error) noexcept
{
    NCArcPlaneAxes plane{};
    double angle = 0.0;
    if (!TryGetNCArcPlaneAxes(frame.rotationPlane, plane) ||
        !TryGetNCTranslationWorkPlaneAngle(frame, angle))
        return Fail(CoordinateError::InvalidFrame, error);
    double cosine = 1.0, sine = 0.0;
    NCTranslationPlanarCoefficients(inverse ? -angle : angle, cosine, sine);
    double u = 0.0, v = 0.0;
    NCTranslationRotatePlanarVector(vector[plane.u], vector[plane.v], cosine, sine, u, v);
    if (!std::isfinite(u) || !std::isfinite(v)) return Fail(CoordinateError::NumericRange, error);
    vector[plane.u] = u;
    vector[plane.v] = v;
    return true;
}
}

bool ResolveDirection(const NCTranslationSnapshot& frame, const Point6& startMCS,
    const Point6& authoredDirection, bool mechanical, Point6& output,
    CoordinateError& error) noexcept
{
    error = CoordinateError::None;
    if (!ValidateFrame(frame, error)) return false;
    if (!Finite(startMCS) || !Finite(authoredDirection)) return Fail(CoordinateError::NonfiniteInput, error);
    bool any = false;
    for (unsigned axis = 0U; axis < 6U; ++axis)
    {
        if (authoredDirection[axis] == 0.0) continue;
        any = true;
        if (!frame.axisIdentity.exists[axis]) return Fail(CoordinateError::AbsentAxis, error);
    }
    if (!any) return Fail(CoordinateError::ZeroDirection, error);
    Point6 next = authoredDirection;
    if (!mechanical)
    {
        NCArcPlaneAxes plane{};
        if (!TryGetNCArcPlaneAxes(frame.rotationPlane, plane)) return Fail(CoordinateError::InvalidFrame, error);
        for (unsigned axis = 0U; axis < 3U; ++axis)
            next[axis] = NCTranslationScaleMirrorAxisVector(frame, authoredDirection[axis], axis);
        if (!NCTranslationRotateArcVector(frame, authoredDirection[plane.u],
            authoredDirection[plane.v], next[plane.u], next[plane.v]))
            return Fail(CoordinateError::NumericRange, error);
        if (HasEccentricH(frame))
        {
            unsigned role = 8U;
            if (!EccentricRole(frame, role, error)) return false;
            std::array<double, 3U> signedH{};
            if (!NCEccentricCDetail::Offset(frame, startMCS[role], signedH))
                return Fail(CoordinateError::NumericRange, error);
            const double angularDerivative = authoredDirection[role] * NCEccentricCDetail::RadPerDeg;
            double tangent[3] = {-signedH[1] * angularDerivative,
                signedH[0] * angularDerivative, 0.0};
            if (!RotateWork(frame, tangent, false, error)) return false;
            for (unsigned axis = 0U; axis < 3U; ++axis) next[axis] += tangent[axis];
        }
    }
    any = false;
    for (unsigned axis = 0U; axis < 6U; ++axis)
    {
        if (!std::isfinite(next[axis])) return Fail(CoordinateError::NumericRange, error);
        if (next[axis] != 0.0)
        {
            any = true;
            if (!frame.axisIdentity.exists[axis]) return Fail(CoordinateError::AbsentAxis, error);
        }
    }
    if (!any) return Fail(CoordinateError::ZeroDirection, error);
    output = next;
    return true;
}

bool MachineToWork(const NCTranslationSnapshot& frame, const Point6& machine,
    Point6& work, CoordinateError& error) noexcept
{
    error = CoordinateError::None;
    if (!ValidateFrame(frame, error)) return false;
    if (!Finite(machine)) return Fail(CoordinateError::NonfiniteInput, error);
    NCTranslationSnapshot evaluated{};
    if (!FrameAtAngle(frame, machine, evaluated, error)) return false;
    double input[8] = {}, result[8] = {};
    for (unsigned axis = 0U; axis < 6U; ++axis) input[axis] = machine[axis];
    NCTranslationInversePoint(evaluated, input, result);
    Point6 next{};
    for (unsigned axis = 0U; axis < 6U; ++axis)
    {
        if (!std::isfinite(result[axis])) return Fail(CoordinateError::NumericRange, error);
        next[axis] = result[axis];
    }
    work = next;
    return true;
}

bool BuildCoordinateViews(const NCTranslationSnapshot& frame, const Point6& machine,
    CoordinateViews& output, CoordinateError& error) noexcept
{
    error = CoordinateError::None;
    CoordinateViews next{};
    if (!MachineToWork(frame, machine, next.work, error)) return false;
    next.machine = machine;
    // Resolve eccentric H at the supplied physical point exactly once. All
    // comparison views retain this same physical-angle provenance, even when
    // coordinate-origin offsets make the work C value differ from machine C.
    NCTranslationSnapshot evaluated{};
    if (!FrameAtAngle(frame, machine, evaluated, error)) return false;
    const auto cancelFrameRotation = [](NCTranslationSnapshot& source) noexcept {
        source.rotationMode = 69;
        source.rotationAngleDeg = 0.0;
        source.rotationCenterMM[0] = source.rotationCenterMM[1] = 0.0;
        source.workOffset[3] = source.workOffset[4] = source.workOffset[5] = 0.0;
        source.workRotationCenterMM[0] = source.workRotationCenterMM[1] = 0.0;
    };
    const auto cancelToolOffset = [](NCTranslationSnapshot& source) noexcept {
        source.toolLengthMode = 49;
        source.toolHCode = 0;
        for (double& value : source.toolOffsetMM) value = 0.0;
    };
    const auto project = [&next, &error](const NCTranslationSnapshot& source,
        Point6& point) noexcept {
        if (!IsNCTranslationSnapshotValid(source)) return Fail(CoordinateError::InvalidFrame, error);
        double work[8] = {}, result[8] = {};
        for (unsigned axis = 0U; axis < 6U; ++axis) work[axis] = next.work[axis];
        NCTranslationForwardPoint(source, work, result);
        for (unsigned axis = 0U; axis < 6U; ++axis)
        {
            if (!std::isfinite(result[axis])) return Fail(CoordinateError::NumericRange, error);
            point[axis] = result[axis];
        }
        return true;
    };
    NCTranslationSnapshot withoutRotation = evaluated;
    cancelFrameRotation(withoutRotation);
    if (!project(withoutRotation, next.frameRotationRemovedMachine)) return false;
    NCTranslationSnapshot withoutTool = evaluated;
    cancelToolOffset(withoutTool);
    if (!project(withoutTool, next.toolOffsetRemovedMachine)) return false;
    cancelToolOffset(withoutRotation);
    if (!project(withoutRotation, next.frameRotationAndToolOffsetRemovedMachine)) return false;
    output = next;
    return true;
}

bool IsOriginSnapshotTransition(const NCTranslationSnapshot& previous,
    const NCTranslationSnapshot& candidate) noexcept
{
    if (!IsNCTranslationSnapshotValid(previous) || !IsNCTranslationSnapshotValid(candidate) ||
        previous.cutterMode != 40 ||
        previous.generation == (std::numeric_limits<std::uint64_t>::max)() ||
        previous.revision == (std::numeric_limits<std::uint64_t>::max)() ||
        candidate.generation != previous.generation + 1ULL ||
        candidate.revision != previous.revision + 1ULL) return false;
    NCTranslationSnapshot expected = previous;
    expected.generation = candidate.generation;
    expected.revision = candidate.revision;
    for (unsigned axis = 0U; axis < 6U; ++axis)
        expected.wcsOffsetMM[axis] = candidate.wcsOffsetMM[axis];
    return SameNCTranslationSnapshot(expected, candidate);
}

bool BuildOriginSnapshot(const NCTranslationSnapshot& previous,
    const Point6& meanMCS, std::uint32_t authoredMask, int action, double lValue,
    NCTranslationSnapshot& candidate, Point6& meanWCS, Point6& desiredWCS,
    CoordinateError& error) noexcept
{
    error = CoordinateError::None;
    if (!ValidateFrame(previous, error)) return false;
    if (action < 0 || action > 2 || (action == 1 && !std::isfinite(lValue)))
        return Fail(CoordinateError::InvalidOriginAction, error);
    if (!authoredMask || (authoredMask & ~63U)) return Fail(CoordinateError::InvalidOriginMask, error);
    for (unsigned axis = 0U; axis < 6U; ++axis)
        if ((authoredMask & (1U << axis)) && !previous.axisIdentity.exists[axis])
            return Fail(CoordinateError::AbsentAxis, error);
    if (previous.generation == (std::numeric_limits<std::uint64_t>::max)() ||
        previous.revision == (std::numeric_limits<std::uint64_t>::max)())
        return Fail(CoordinateError::CounterExhausted, error);
    Point6 mean{}, desired{};
    if (!MachineToWork(previous, meanMCS, mean, error)) return false;
    desired = mean;
    for (unsigned axis = 0U; axis < 6U; ++axis)
        if (authoredMask & (1U << axis))
            desired[axis] = action == 0 ? 0.0 : (action == 1 ? lValue : mean[axis] * 0.5);

    NCTranslationSnapshot evaluated{};
    if (!FrameAtAngle(previous, meanMCS, evaluated, error)) return false;
    double desired8[8] = {}, mapped[8] = {}, delta[6] = {};
    for (unsigned axis = 0U; axis < 6U; ++axis) desired8[axis] = desired[axis];
    NCTranslationForwardPoint(evaluated, desired8, mapped);
    for (unsigned axis = 0U; axis < 6U; ++axis)
    {
        delta[axis] = meanMCS[axis] - mapped[axis];
        if (!std::isfinite(delta[axis])) return Fail(CoordinateError::NumericRange, error);
    }
    if (!RotateWork(previous, delta, true, error)) return false;
    NCTranslationSnapshot next = previous;
    for (unsigned axis = 0U; axis < 6U; ++axis)
    {
        next.wcsOffsetMM[axis] += delta[axis];
        if (!std::isfinite(next.wcsOffsetMM[axis])) return Fail(CoordinateError::NumericRange, error);
    }
    ++next.generation;
    ++next.revision;
    if (!IsOriginSnapshotTransition(previous, next)) return Fail(CoordinateError::InvalidFrame, error);
    Point6 verified{};
    if (!MachineToWork(next, meanMCS, verified, error)) return false;
    for (unsigned axis = 0U; axis < 6U; ++axis)
        if (std::fabs(verified[axis] - desired[axis]) > NCEccentricCDetail::NativeRoundoffBudget)
            return Fail(CoordinateError::OriginRoundoff, error);
    candidate = next;
    meanWCS = mean;
    desiredWCS = desired;
    return true;
}
}
