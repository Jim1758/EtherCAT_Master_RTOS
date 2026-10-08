#pragma once

#include "NCEccentricCRuntime.h"
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

// BASE77 startup / stopped NC-LOAD model check. This function uses only local
// synthetic values. It does not read/write live axes, H tables, Motion queues,
// settings or ownership. Passing it does not authorize dynamic compensation.
struct NCEccentricCRuntimeSelfCheckResult
{
    std::uint32_t checks = 0U;
    std::uint32_t failedCheck = 0U;
    bool passed = true;
};

inline NCEccentricCRuntimeSelfCheckResult RunNCEccentricCRuntimeSelfCheck() noexcept
{
    NCEccentricCRuntimeSelfCheckResult result{};
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
    if (!check(!NCEccentricCDynamicMotionAdmission)) return result;
    NCEccentricCRuntimeInput input{};
    auto& geometry = input.geometry;
    auto& source = geometry.source;
    source.runToken = source.generation = source.revision = 1ULL;
    source.wcsCode = 54; source.toolLengthMode = 43; source.toolHCode = 1;
    source.toolOffsetMM[0] = 2.0; source.toolOffsetMM[1] = 1.0; source.toolOffsetMM[2] = 3.0;
    source.extOffsetMM[0] = 10.0; source.wcsOffsetMM[1] = -20.0;
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
    geometry.startMCS = {100.0, 200.0, 300.0, 0.0, 20.0, -30.0, 0.0, 0.0};
    geometry.maxVelocityNative.fill(100.0);
    geometry.maxAccelerationNative.fill(1000.0);
    geometry.maxDecelerationNative.fill(1000.0);
    geometry.sweepDeg = 90.0; geometry.feedDegMin = 60.0;
    geometry.accTime = geometry.decTime = 0.5;
    input.pulsePerNative.fill(1000.0);
    for (unsigned a = 0U; a < 8U; ++a)
        input.startPulse[a] = geometry.startMCS[a] * input.pulsePerNative[a];
    input.travelMinNative.fill(-10000.0); input.travelMaxNative.fill(10000.0);
    input.travelMask = 63U; input.firSamples = 16U; input.stopSeconds = 0.2;
    const NCEccentricCRuntimeInput original = input;
    NCEccentricCRuntimeValue prepared{};
    NCEccentricCRuntimePoint point{};
    if (!check(PrepareNCEccentricCRuntime(input, prepared) == NCEccentricCRuntimeCode::PREPARED && prepared.IsValid())) return result;
    if (!check(prepared.AuthoredPath().authoredMask == 8U && prepared.AuthoredPath().generatedMask == 3U &&
        prepared.AuthoredPath().groupMask == 11U)) return result;
    if (!check(prepared.AuthoredScalarPulse() == 90000.0 && prepared.MaximumScalarVelocityPulsePerSec() == 1000.0 &&
        prepared.MaximumScalarPulse() > prepared.AuthoredScalarPulse() && prepared.StopReserveDegrees() >= 0.10425)) return result;
    if (!check(EvaluateNCEccentricCRuntime(prepared, 0.0, 0.0, point) && point.valid &&
        NCEccentricCDetail::Same(point.positionPulse, input.startPulse))) return result;
    if (!check(EvaluateNCEccentricCRuntime(prepared, prepared.AuthoredScalarPulse(), 0.0, point) &&
        NCEccentricCDetail::Same(point.positionPulse, prepared.EndPulse()) &&
        close(point.positionPulse[0], 97000.0) && close(point.positionPulse[1], 201000.0) &&
        point.positionPulse[3] == 90000.0)) return result;
    if (!check(EvaluateNCEccentricCRuntime(prepared, 45000.0, 1000.0, point))) return result;
    const double rootHalf = std::sqrt(0.5);
    if (!check(close(point.positionPulse[0], 100000.0 + (rootHalf - 2.0) * 1000.0) &&
        close(point.positionPulse[1], 200000.0 + (3.0 * rootHalf - 1.0) * 1000.0) &&
        close(point.velocityPulse[0], -3.0 * rootHalf * NCEccentricCDetail::RadPerDeg * 1000.0) &&
        close(point.velocityPulse[1], rootHalf * NCEccentricCDetail::RadPerDeg * 1000.0) &&
        point.velocityPulse[3] == 1000.0)) return result;
    for (unsigned a : {2U, 4U, 5U, 6U, 7U})
        if (!check(NCEccentricCDetail::Same(point.positionPulse[a], input.startPulse[a]) && point.velocityPulse[a] == 0.0)) return result;

    const double continuation = (prepared.AuthoredScalarPulse() + prepared.MaximumScalarPulse()) * 0.5;
    if (!check(!EvaluateNCEccentricCRuntime(prepared, continuation, 1000.0, point) && !point.valid &&
        point.positionPulse[0] == 0.0)) return result;
    if (!check(EvaluateNCEccentricCRuntime(prepared, continuation, 1000.0, point,
        NCEccentricCRuntimePhase::CONTROLLED_STOP) && point.positionPulse[3] > 90000.0)) return result;
    if (!check(EvaluateNCEccentricCRuntime(prepared, prepared.MaximumScalarPulse(), 0.0, point,
        NCEccentricCRuntimePhase::CONTROLLED_STOP))) return result;
    const double infinity = (std::numeric_limits<double>::infinity)();
    if (!check(!EvaluateNCEccentricCRuntime(prepared, std::nextafter(prepared.MaximumScalarPulse(), infinity), 0.0,
        point, NCEccentricCRuntimePhase::CONTROLLED_STOP) && !point.valid)) return result;
    if (!check(!EvaluateNCEccentricCRuntime(prepared, 1.0, std::nextafter(1000.0, infinity), point) && !point.valid)) return result;
    if (!check(!EvaluateNCEccentricCRuntime(prepared, -1.0, 0.0, point))) return result;
    if (!check(!EvaluateNCEccentricCRuntime(prepared, 0.0, -1.0, point))) return result;
    if (!check(!EvaluateNCEccentricCRuntime(prepared, 0.0, infinity, point))) return result;
    if (!check(!EvaluateNCEccentricCRuntime(prepared, 0.0, 0.0, point,
        static_cast<NCEccentricCRuntimePhase>(2U)))) return result;
    // Mutation of the input cannot change the prepared, frozen model.
    input.geometry.source.toolOffsetMM[0] = 1000.0;
    if (!check(EvaluateNCEccentricCRuntime(prepared, 90000.0, 0.0, point) && close(point.positionPulse[0], 97000.0))) return result;

    input = original; input.geometry.source.toolLengthMode = 44;
    if (!check(PrepareNCEccentricCRuntime(input, prepared) == NCEccentricCRuntimeCode::PREPARED &&
        EvaluateNCEccentricCRuntime(prepared, 90000.0, 0.0, point) &&
        close(point.positionPulse[0], 103000.0) && close(point.positionPulse[1], 199000.0))) return result;
    input = original; input.geometry.sweepDeg = -90.0;
    if (!check(PrepareNCEccentricCRuntime(input, prepared) == NCEccentricCRuntimeCode::PREPARED &&
        EvaluateNCEccentricCRuntime(prepared, 90000.0, 1000.0, point) &&
        close(point.positionPulse[0], 99000.0) && close(point.positionPulse[1], 197000.0) &&
        point.positionPulse[3] == -90000.0 && point.velocityPulse[3] == -1000.0)) return result;
    for (double sweep : {360.0, -360.0, 720.0})
    {
        input = original; input.geometry.sweepDeg = sweep;
        if (!check(PrepareNCEccentricCRuntime(input, prepared) == NCEccentricCRuntimeCode::PREPARED &&
            EvaluateNCEccentricCRuntime(prepared, prepared.AuthoredScalarPulse(), 0.0, point))) return result;
        if (!check(NCEccentricCDetail::Same(point.positionPulse[0], input.startPulse[0]) &&
            NCEccentricCDetail::Same(point.positionPulse[1], input.startPulse[1]) && point.positionPulse[3] == sweep * 1000.0)) return result;
    }
    input = original;
    input.geometry.source.toolOffsetMM[0] = input.geometry.source.toolOffsetMM[1] = 0.0;
    input.geometry.startMCS[0] = input.startPulse[0] = -0.0;
    if (!check(PrepareNCEccentricCRuntime(input, prepared) == NCEccentricCRuntimeCode::PREPARED &&
        prepared.AuthoredPath().generatedMask == 0U && EvaluateNCEccentricCRuntime(prepared, 45000.0, 1000.0, point) &&
        NCEccentricCDetail::Same(point.positionPulse[0], input.startPulse[0]) && point.velocityPulse[0] == 0.0)) return result;

    // An authored arc can fit while its curved STOP continuation does not.
    input = original;
    input.geometry.source.toolOffsetMM[0] = 1.0; input.geometry.source.toolOffsetMM[1] = 0.0;
    input.geometry.sweepDeg = 10.0; input.travelMaxNative[1] = 200.174;
    NCEccentricCPathValue authored{};
    if (!check(BuildNCEccentricCPath(input.geometry, authored) == NCEccentricCPathCode::BUILT_PATH &&
        IsNCEccentricCPathWithinBounds(authored, input.travelMinNative, input.travelMaxNative, input.travelMask))) return result;
    if (!check(PrepareNCEccentricCRuntime(input, prepared) == NCEccentricCRuntimeCode::STOP_ENVELOPE_REJECTED &&
        !prepared.IsValid())) return result;
    input = original; input.geometry.maxDecelerationNative[1] = 0.1;
    if (!check(BuildNCEccentricCPath(input.geometry, authored) == NCEccentricCPathCode::BUILT_PATH &&
        PrepareNCEccentricCRuntime(input, prepared) == NCEccentricCRuntimeCode::STOP_ENVELOPE_REJECTED)) return result;
    for (unsigned defect = 0U; defect < 8U; ++defect)
    {
        input = original;
        switch (defect)
        {
        case 0U: input.stopSeconds = 0.0009; break;
        case 1U: input.cycleSeconds = 0.001; break;
        case 2U: input.firSamples = 0U; break;
        case 3U: input.firSamples = 4097U; break;
        case 4U: input.pulsePerNative[0] = 0.0; break;
        case 5U: input.startPulse[0] += 1.0; break;
        case 6U: input.travelMask = 8U; break;
        default: input.geometry.source.axisIdentity.electrodeAxisPlusOne = 0U; break;
        }
        if (!check(PrepareNCEccentricCRuntime(input, prepared) != NCEccentricCRuntimeCode::PREPARED &&
            !prepared.IsValid() && !EvaluateNCEccentricCRuntime(prepared, 0.0, 0.0, point) && !point.valid)) return result;
    }
    input = original;
    if (!check(PrepareNCEccentricCRuntime(input, prepared) == NCEccentricCRuntimeCode::PREPARED)) return result;
    prepared.Clear();
    if (!check(!prepared.IsValid() && !EvaluateNCEccentricCRuntime(prepared, 0.0, 0.0, point))) return result;
    return result;
}
