#pragma once

#include "MotionEccentricCBinding.h"
#include "NCEccentricCProfileSelfCheck.h"
#include <memory>
#include <new>

struct NCEccentricCBindingSelfCheckResult
{
    std::uint32_t checks = 0U;
    std::uint32_t failedCheck = 0U;
    bool passed = true;
};

#if defined(_MSC_VER)
#define BASE79D_CHECK_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define BASE79D_CHECK_NOINLINE __attribute__((noinline))
#else
#define BASE79D_CHECK_NOINLINE
#endif

namespace NCEccentricCBindingSelfCheckDetail
{
    // Synthetic physical contexts only; never construct another MotionCore or
    // touch the live machine. Axis vectors/buffers remain empty. One allocation.
    struct Workspace
    {
        MotionEccentricCTransportWorkspace transport{};
        NCEccentricCRuntimeInput input = NCEccentricCProfileSelfCheckDetail::Input();
        MotionCommand command{}, bad{};
        NCEccentricCProfileValue decoded{};
        std::array<AxisContext, 8U> axes{};
        std::array<double, 8U> lower{}, upper{};
        MotionExecutionIdentity identity{};
        MotionOwnerLease owner{};
    };

    BASE79D_CHECK_NOINLINE inline void ResetAxes(Workspace& w) noexcept
    {
        for (unsigned index = 0U; index < 8U; ++index)
        {
            auto& axis = w.axes[index];
            axis.isExist = index < 6U; axis.axisIndex = static_cast<int>(index);
            axis.axisType = index < 3U ? AxisType::LINEAR : AxisType::ROTARY;
            axis.isServoOn = true; axis.isFault = false; axis.isLagAlarm = false; axis.isVirtualAxis = false;
            axis.state = MotionState::MotionState_IDLE;
            axis.resolution_PPR = w.input.pulsePerNative[index]; axis.finalLead = 1.0;
            axis.logicalCmdPos = axis.currentCmdPos = w.input.startPulse[index];
            axis.logicalCmdVel = axis.currentCmdVel = 0.0;
            axis.maxVel_PPS = w.input.geometry.maxVelocityNative[index] * w.input.pulsePerNative[index];
            axis.G00_acc_time = axis.G00_dec_time = 0.01; axis.Stop_dec_time = 0.04;
            w.lower[index] = -10000.0; w.upper[index] = 10000.0;
        }
    }

    BASE79D_CHECK_NOINLINE inline bool Prepare(Workspace& w) noexcept
    {
        return BuildMotionEccentricCCommand(w.input, w.identity, w.owner, w.command, w.transport) &&
            PrepareMotionEccentricCCommand(w.command, w.decoded, w.transport);
    }

    BASE79D_CHECK_NOINLINE inline bool Bound(const Workspace& w, const MotionCommand& command) noexcept
    {
        const auto& runtime = w.decoded.Runtime();
        if (!IsMotionEccentricCStartSourceBound(command, runtime)) return false;
        for (unsigned index = 0U; index < 8U; ++index)
        {
            const double minimum = runtime.MinimumPulse()[index] / runtime.PulsePerNative()[index];
            const double maximum = runtime.MaximumPulse()[index] / runtime.PulsePerNative()[index];
            const bool travelAllowed = runtime.ExtendedPath().minMCS[index] >= w.lower[index] &&
                runtime.ExtendedPath().maxMCS[index] <= w.upper[index] &&
                minimum >= w.lower[index] && maximum <= w.upper[index];
            if (!IsMotionEccentricCStartAxisBound(command, runtime, index, &w.axes[index], travelAllowed)) return false;
        }
        return true;
    }
}

BASE79D_CHECK_NOINLINE inline NCEccentricCBindingSelfCheckResult RunNCEccentricCBindingSelfCheck() noexcept
{
    using namespace NCEccentricCBindingSelfCheckDetail;
    NCEccentricCBindingSelfCheckResult result{};
    const auto check = [&](bool condition) noexcept -> bool
    {
        ++result.checks;
        if (!condition) { result.passed = false; result.failedCheck = result.checks; }
        return condition;
    };
    const std::unique_ptr<Workspace> workspace(new (std::nothrow) Workspace);
    if (!check(workspace && !NCEccentricCDynamicMotionAdmission)) return result;
    auto& w = *workspace;
    w.identity.epoch = 7U; w.identity.segmentId = 19ULL;
    w.identity.sourceBlockId = 23; w.identity.source = MotionCommandSource::NC_MEMORY;
    w.owner.owner = MotionOwner::AUTO; w.owner.generation = 11U;
    w.input.geometry.source.distanceMode = 91;
    for (int mode : {43, 44}) for (double direction : {1.0, -1.0})
    {
        w.input.geometry.source.toolLengthMode = mode; w.input.geometry.sweepDeg = direction * 0.01;
        ResetAxes(w);
        if (!check(Prepare(w) && IsMotionEccentricCStartSourceBound(w.command, w.decoded.Runtime()))) return result;
        if (!check(Bound(w, w.command))) return result;
        // PBC changes physical command coordinates downstream. Binding uses
        // the nominal logical anchor, never currentCmdPos or actual feedback.
        w.axes[0].currentCmdPos += 123.0; w.axes[3].currentActPos = -987.0;
        if (!check(Bound(w, w.command))) return result;
        w.axes[4].isServoOn = false;
        if (!check(Bound(w, w.command))) return result;
    }
    w.input.geometry.source.toolLengthMode = 43; w.input.geometry.sweepDeg = 0.01;
    ResetAxes(w);
    if (!check(Prepare(w) && Bound(w, w.command))) return result;
    const double nan = (std::numeric_limits<double>::quiet_NaN)();
    const double infinity = (std::numeric_limits<double>::infinity)();
    for (unsigned defect = 0U; defect < 20U; ++defect)
    {
        w.bad = w.command;
        switch (defect)
        {
        case 0U: w.bad.pathCoreEccentricCFeedExactStop = false; break;
        case 1U: w.bad.sourceG162Active = false; break;
        case 2U: ++w.bad.sourceTranslation.revision; break;
        case 3U: w.bad.targetPos[0] = std::nextafter(w.bad.targetPos[0], infinity); break;
        case 4U: w.bad.targetVel = std::nextafter(w.bad.targetVel, infinity); break;
        case 5U: w.bad.mem_startPos[4] = std::nextafter(w.bad.mem_startPos[4], infinity); break;
        case 6U: w.bad.mem_ratio[3] = std::nextafter(w.bad.mem_ratio[3], infinity); break;
        case 7U: w.bad.mem_centerX *= 2.0; break;
        case 8U: w.bad.mem_radius = -w.bad.mem_radius; break;
        case 9U: w.bad.mem_startAngle *= 2.0; break;
        case 10U: w.bad.accTime *= 2.0; break;
        case 11U: w.bad.decTime *= 2.0; break;
        case 12U: w.bad.mem_transformMatrix[0][0] = nan; break;
        case 13U: w.bad.mem_transformMatrix[1][1] = 0.0; break;
        case 14U: w.bad.mem_transformMatrix[2][2] = -1.0; break;
        case 15U: w.bad.mem_centerY = infinity; break;
        case 16U: w.bad.mem_totalDist = -10000.0; break;
        case 17U: w.bad.mem_transformOrigin[2] = w.input.geometry.startMCS[3]; break;
        case 18U: w.bad.axisIndices[1] = 3; break;
        default: w.bad.sourceTranslation.axisIdentity.electrodeAxisPlusOne = 5U; break;
        }
        if (!check(!Bound(w, w.bad) && Bound(w, w.command))) return result;
    }
    for (unsigned defect = 0U; defect < 26U; ++defect)
    {
        ResetAxes(w); auto& axis = w.axes[3];
        switch (defect)
        {
        case 0U: axis.axisIndex = 4; break;
        case 1U: axis.axisType = AxisType::LINEAR; break;
        case 2U: axis.isExist = false; break;
        case 3U: axis.logicalCmdPos = std::nextafter(w.input.startPulse[3], infinity); break;
        case 4U: axis.logicalCmdVel = 0.001; break;
        case 5U: axis.currentCmdVel = 0.001; break;
        case 6U: axis.isFault = true; break;
        case 7U: axis.isLagAlarm = true; break;
        case 8U: axis.isServoOn = false; break;
        case 9U: axis.resolution_PPR += 1.0; break;
        case 10U: axis.finalLead = 0.0; break;
        case 11U: axis.maxVel_PPS = 0.0; break;
        case 12U: axis.maxVel_PPS *= 0.5; break;
        case 13U: axis.G00_acc_time *= 2.0; break;
        case 14U: axis.G00_dec_time *= 2.0; break;
        case 15U: axis.Stop_dec_time *= 2.0; break;
        case 16U: axis.Stop_dec_time = 0.0; break;
        case 17U: axis.resolution_PPR = nan; break;
        case 18U: axis.logicalCmdVel = nan; break;
        case 19U: axis.currentCmdVel = nan; break;
        case 20U: axis.maxVel_PPS = nan; break;
        case 21U: axis.G00_acc_time = nan; break;
        case 22U: axis.G00_dec_time = nan; break;
        case 23U: axis.Stop_dec_time = nan; break;
        case 24U: axis.isVirtualAxis = true; break;
        default: axis.state = MotionState::MotionState_MOVING; break;
        }
        if (!check(!Bound(w, w.command))) return result;
    }
    // Presence and stationary anchors are checked for the entire frozen image.
    for (unsigned index = 0U; index < 8U; ++index)
    {
        ResetAxes(w); w.axes[index].isExist = !w.axes[index].isExist;
        if (!check(!Bound(w, w.command))) return result;
    }
    for (unsigned index = 0U; index < 6U; ++index)
    {
        ResetAxes(w); w.axes[index].logicalCmdPos = std::nextafter(w.input.startPulse[index], infinity);
        if (!check(!Bound(w, w.command))) return result;
        ResetAxes(w); w.axes[index].logicalCmdVel = 0.001;
        if (!check(!Bound(w, w.command))) return result;
    }
    // A limit that includes the authored C endpoint but excludes the stop
    // reserve must reject. Also cover lower/upper bounds on stationary axes.
    ResetAxes(w); w.upper[3] = w.decoded.Runtime().AuthoredPath().endMCS[3];
    if (!check(!Bound(w, w.command))) return result;
    for (unsigned index : {0U, 1U, 2U, 4U, 5U})
    {
        ResetAxes(w); w.lower[index] = w.decoded.Runtime().MaximumPulse()[index] / w.input.pulsePerNative[index] + 1.0;
        if (!check(!Bound(w, w.command))) return result;
        ResetAxes(w); w.upper[index] = w.decoded.Runtime().MinimumPulse()[index] / w.input.pulsePerNative[index] - 1.0;
        if (!check(!Bound(w, w.command))) return result;
    }
    ResetAxes(w);
    if (!check(!IsMotionEccentricCStartAxisBound(w.command, w.decoded.Runtime(), 8U, nullptr, false) &&
        !IsMotionEccentricCStartAxisBound(w.command, w.decoded.Runtime(), 0U, nullptr, true))) return result;
    // With no XY eccentricity, XY servo power/rate ceilings are not part of
    // the moving group, but their nominal anchors must still be stationary.
    w.input.geometry.source.toolOffsetMM[0] = w.input.geometry.source.toolOffsetMM[1] = 0.0;
    if (!check(Prepare(w) && w.command.axisCount == 1)) return result;
    ResetAxes(w); w.axes[0].isServoOn = w.axes[1].isServoOn = false;
    w.axes[0].maxVel_PPS = w.axes[1].maxVel_PPS = 0.0;
    if (!check(Bound(w, w.command))) return result;
    w.axes[1].logicalCmdVel = 0.001;
    if (!check(!Bound(w, w.command))) return result;
    w.decoded.Clear();
    if (!check(!Bound(w, w.command) && !NCEccentricCDynamicMotionAdmission)) return result;
    return result;
}

#undef BASE79D_CHECK_NOINLINE
