#pragma once

#include "NCEccentricCPath.h"
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

// BASE76 startup-only synthetic model checks. No live axis, coordinate table,
// packet, ownership or admission state is read or changed. Passing this check
// is not permission to execute G162 compensation. Call once during NC startup.
struct NCEccentricCSelfCheckResult
{
    std::uint32_t checks = 0U;
    std::uint32_t failedCheck = 0U;
    bool passed = true;
};

inline NCEccentricCSelfCheckResult RunNCEccentricCSelfCheck() noexcept
{
    NCEccentricCSelfCheckResult result{};
    const auto check = [&](bool condition) noexcept -> bool
    {
        ++result.checks;
        if (!condition) { result.passed = false; result.failedCheck = result.checks; }
        return condition;
    };
    const auto close = [](double a, double b) noexcept -> bool
    {
        const double scale = (std::fmax)(1.0, (std::fmax)(std::fabs(a), std::fabs(b)));
        return std::isfinite(a) && std::isfinite(b) &&
            std::fabs(a - b) <= 128.0 * (std::numeric_limits<double>::epsilon)() * scale;
    };
    NCEccentricCPathInput input{};
    auto& source = input.source;
    source.runToken = source.generation = source.revision = 1ULL;
    source.wcsCode = 54;
    source.toolLengthMode = 43;
    source.toolHCode = 1;
    source.toolOffsetMM[0] = 2.0;
    source.toolOffsetMM[1] = 1.0;
    source.toolOffsetMM[2] = 3.0;
    source.extOffsetMM[0] = 10.0; source.extOffsetMM[1] = -20.0; source.extOffsetMM[2] = 30.0;
    source.wcsOffsetMM[0] = -5.0; source.wcsOffsetMM[1] = 15.0; source.wcsOffsetMM[2] = -25.0;
    auto& identity = source.axisIdentity;
    identity.bound = 1U; identity.systemMode = 1U;
    identity.electrodeAxisPlusOne = 4U; identity.eccentricEnabled = 1U;
    for (unsigned axis = 0U; axis < 6U; ++axis)
    {
        identity.exists[axis] = 1U;
        identity.axisType[axis] = axis < 3U ? 0U : 1U;
        identity.nativeUnit[axis] = axis < 3U ? 1U : 2U;
        identity.physicalIndexPlusOne[axis] = static_cast<std::uint8_t>(axis + 1U);
        identity.address[axis] = "XYZCUV"[axis];
    }
    input.startMCS = {100.0, 200.0, 300.0, 0.0, 20.0, -30.0, 0.0, 0.0};
    input.maxVelocityNative.fill(100.0);
    input.maxAccelerationNative.fill(1000.0);
    input.maxDecelerationNative.fill(1000.0);
    input.sweepDeg = 90.0; input.feedDegMin = 60.0;
    input.accTime = 0.5; input.decTime = 0.5;
    const NCEccentricCPathInput original = input;
    NCEccentricCPathValue path{};
    (void)BuildNCEccentricCPath(input, path);
    if (!check(path.valid && path.authoredMask == 8U && path.generatedMask == 3U && path.groupMask == 11U)) return result;
    if (!check(close(path.endMCS[0], 97.0) && close(path.endMCS[1], 201.0) &&
        path.endMCS[2] == 300.0 && path.endMCS[3] == 90.0)) return result;
    if (!check(close(path.nominalMCS[0], 98.0) && close(path.nominalMCS[1], 199.0) &&
        close(path.nominalMCS[2], 297.0) && close(path.nominalWCS[0], 93.0) &&
        close(path.nominalWCS[1], 204.0) && close(path.nominalWCS[2], 292.0))) return result;
    for (double u : {0.0, 0.25, 0.5, 0.75, 1.0})
    {
        NCEccentricCPathPoint point{};
        if (!check(EvaluateNCEccentricCPath(path, u, point) && point.valid)) return result;
        std::array<double, 3U> nominalMCS{}, nominalWCS{};
        if (!check(TryInverseNCEccentricCPoint(path, point.positionMCS, nominalMCS, nominalWCS))) return result;
        for (unsigned axis = 0U; axis < 3U; ++axis)
            if (!check(close(nominalMCS[axis], path.nominalMCS[axis]) &&
                close(nominalWCS[axis], path.nominalWCS[axis]))) return result;
        if (!check(point.positionMCS[2] == 300.0 && point.positionMCS[4] == 20.0 &&
            point.positionMCS[5] == -30.0 && point.derivativePerU[2] == 0.0 &&
            point.derivativePerU[3] == 90.0)) return result;
    }
    input.source.toolLengthMode = 44;
    (void)BuildNCEccentricCPath(input, path);
    if (!check(path.valid && close(path.endMCS[0], 103.0) && close(path.endMCS[1], 199.0))) return result;
    input = original;
    for (double sweep : {360.0, -360.0, 720.0})
    {
        input.sweepDeg = sweep;
        (void)BuildNCEccentricCPath(input, path);
        if (!check(path.valid && path.endMCS[0] == input.startMCS[0] &&
            path.endMCS[1] == input.startMCS[1] && path.generatedMask == 3U &&
            path.endMCS[3] == sweep)) return result;
        const double radius = std::sqrt(5.0);
        if (!check(path.minMCS[0] <= 98.0 - radius && path.maxMCS[0] >= 98.0 + radius &&
            path.minMCS[1] <= 199.0 - radius && path.maxMCS[1] >= 199.0 + radius)) return result;
    }
    input = original;
    input.source.toolOffsetMM[0] = 1.0; input.source.toolOffsetMM[1] = 0.0;
    input.startMCS[0] = input.startMCS[1] = 0.0; input.startMCS[3] = -45.0;
    (void)BuildNCEccentricCPath(input, path);
    if (!check(path.valid && close(path.startMCS[0], 0.0) && close(path.endMCS[0], 0.0) &&
        path.maxMCS[0] > 0.29)) return result;
    std::array<double, 8U> minimum{}, maximum{};
    minimum.fill(-1000.0); maximum.fill(1000.0); maximum[0] = 0.1;
    if (!check(!IsNCEccentricCPathWithinBounds(path, minimum, maximum, path.groupMask))) return result;
    maximum[0] = 1.0;
    if (!check(IsNCEccentricCPathWithinBounds(path, minimum, maximum, path.groupMask))) return result;
    input = original; input.source.toolOffsetMM[0] = input.source.toolOffsetMM[1] = 0.0;
    input.sweepDeg = 360.0;
    (void)BuildNCEccentricCPath(input, path);
    if (!check(path.valid && path.generatedMask == 0U && path.groupMask == 8U &&
        path.endMCS[0] == input.startMCS[0] && path.endMCS[1] == input.startMCS[1])) return result;
    input = original;
    (void)BuildNCEccentricCPath(input, path);
    if (!check(path.valid && path.requiredAccelerationNative[0] >
        path.radiusMM * path.alphaAccRadPerSec2)) return result; // Includes curvature.
    input.maxAccelerationNative[0] = path.radiusMM * path.alphaAccRadPerSec2;
    (void)BuildNCEccentricCPath(input, path);
    if (!check(!path.valid)) return result;
    input = original; input.maxVelocityNative[1] = 0.0001;
    (void)BuildNCEccentricCPath(input, path);
    if (!check(!path.valid)) return result;
    input = original; input.source.axisIdentity.electrodeAxisPlusOne = 0U;
    (void)BuildNCEccentricCPath(input, path);
    if (!check(!path.valid)) return result;
    input = original; input.source.axisIdentity.eccentricEnabled = 0U;
    (void)BuildNCEccentricCPath(input, path);
    if (!check(!path.valid)) return result;
    input = original; input.source.toolOffsetMM[3] = 1.0;
    (void)BuildNCEccentricCPath(input, path);
    if (!check(!path.valid)) return result;
    input = original; input.feedDegMin = (std::numeric_limits<double>::quiet_NaN)();
    (void)BuildNCEccentricCPath(input, path);
    if (!check(!path.valid && path.groupMask == 0U && path.endMCS[0] == 0.0)) return result;
    input = original;
    (void)BuildNCEccentricCPath(input, path);
    NCEccentricCPathPoint point{};
    if (!check(path.valid && !EvaluateNCEccentricCPath(path, -0.1, point) && !point.valid)) return result;
    return result;
}
