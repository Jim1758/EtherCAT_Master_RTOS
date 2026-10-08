#pragma once

#include "NCEccentricCProfile.h"
#include <cmath>
#include <cstdint>
#include <limits>

// BASE78: local synthetic model checks on startup / the non-RT NC LOAD thread.
// No live axes, H tables, configuration, ownership, queues or servo writes.
// Neither this result nor the profile grants dynamic Motion admission.
struct NCEccentricCProfileSelfCheckResult
{
    std::uint32_t checks = 0U;
    std::uint32_t failedCheck = 0U;
    bool passed = true;
};

namespace NCEccentricCProfileSelfCheckDetail
{
    inline NCEccentricCRuntimeInput Input() noexcept
    {
        NCEccentricCRuntimeInput input{};
        auto& geometry = input.geometry;
        auto& source = geometry.source;
        source.runToken = source.generation = source.revision = 1ULL;
        source.wcsCode = 54; source.toolLengthMode = 43; source.toolHCode = 1;
        source.toolOffsetMM[0] = 2.0; source.toolOffsetMM[1] = 1.0;
        source.toolOffsetMM[2] = 3.0;
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
        geometry.startMCS = {100.0, 200.0, 300.0, 0.0, 20.0, -30.0, -0.0, 0.0};
        geometry.maxVelocityNative.fill(100.0);
        geometry.maxAccelerationNative.fill(10000.0);
        geometry.maxDecelerationNative.fill(10000.0);
        geometry.sweepDeg = 0.01; geometry.feedDegMin = 60.0;
        geometry.accTime = 0.001; geometry.decTime = 0.002;
        input.pulsePerNative.fill(1000.0);
        for (unsigned axis = 0U; axis < 8U; ++axis)
            input.startPulse[axis] = geometry.startMCS[axis] * input.pulsePerNative[axis];
        input.travelMinNative.fill(-10000.0); input.travelMaxNative.fill(10000.0);
        input.travelMask = 63U; input.firSamples = 1U; input.stopSeconds = 0.04;
        return input;
    }

    inline bool Close(double a, double b) noexcept
    {
        const double scale = (std::fmax)(1.0, (std::fmax)(std::fabs(a), std::fabs(b)));
        return std::isfinite(a) && std::isfinite(b) &&
            std::fabs(a - b) <= 128.0 * (std::numeric_limits<double>::epsilon)() * scale;
    }

    // Fixed upper bound makes this a short non-RT check, never a live runner.
    inline bool Finish(NCEccentricCProfileCursor& cursor, NCEccentricCProfilePoint& point) noexcept
    {
        using State = NCEccentricCProfileState;
        for (unsigned sample = 0U; sample < 2048U; ++sample)
        {
            if (cursor.State() != State::AUTHORED_ACTIVE && cursor.State() != State::CONTROLLED_STOP_ACTIVE)
                return ReadNCEccentricCProfile(cursor, point) && point.velocityPulsePerSec == 0.0 &&
                    point.accelerationPulsePerSec2 == 0.0;
            const double previousPosition = cursor.ScalarPulse();
            const double previousVelocity = cursor.VelocityPulsePerSec();
            const auto previousSequence = cursor.Sequence();
            const bool stopping = cursor.State() == State::CONTROLLED_STOP_ACTIVE;
            if (!AdvanceNCEccentricCProfile(cursor, point) || !point.valid || !point.runtime.valid ||
                point.scalarPulse < previousPosition || point.velocityPulsePerSec < 0.0 ||
                point.velocityPulsePerSec > cursor.Runtime().MaximumScalarVelocityPulsePerSec() ||
                point.sequence != previousSequence + 1ULL) return false;
            const double change = point.velocityPulsePerSec - previousVelocity;
            const double ceiling = change >= 0.0 ? cursor.Runtime().MaximumScalarAccelerationPulsePerSec2() :
                stopping ? cursor.Runtime().MaximumScalarStopDecelerationPulsePerSec2() :
                cursor.Runtime().MaximumScalarDecelerationPulsePerSec2();
            if (std::fabs(change) / cursor.Runtime().CycleSeconds() > ceiling) return false;
            for (unsigned axis : {2U, 4U, 5U, 6U, 7U})
                if (!NCEccentricCDetail::Same(point.runtime.positionPulse[axis], cursor.Runtime().StartPulse()[axis]) ||
                    point.runtime.velocityPulse[axis] != 0.0) return false;
        }
        return false;
    }
}

inline NCEccentricCProfileSelfCheckResult RunNCEccentricCProfileSelfCheck() noexcept
{
    using namespace NCEccentricCProfileSelfCheckDetail;
    using State = NCEccentricCProfileState;
    NCEccentricCProfileSelfCheckResult result{};
    const auto check = [&](bool condition) noexcept -> bool
    {
        ++result.checks;
        if (!condition) { result.passed = false; result.failedCheck = result.checks; }
        return condition;
    };
    NCEccentricCRuntimeValue runtime{};
    NCEccentricCProfileValue plan{};
    NCEccentricCProfileCursor cursor{};
    NCEccentricCProfilePoint point{};
    if (!check(!NCEccentricCDynamicMotionAdmission && NCEccentricCProfileValue::RequiresUnfilteredSamples())) return result;
    if (!check(PrepareNCEccentricCProfile(runtime, plan) == NCEccentricCProfileCode::INVALID_RUNTIME && !plan.IsValid())) return result;
    if (!check(!BeginNCEccentricCProfile(plan, cursor, point) && !cursor.IsValid() && !point.valid)) return result;
    if (!check(!AdvanceNCEccentricCProfile(cursor, point) && !point.valid)) return result;
    if (!check(!RequestNCEccentricCProfileStop(cursor, point) && !point.valid)) return result;
    if (!check(!ResumeNCEccentricCProfile(cursor, point) && !point.valid)) return result;
    if (!check(!ReadNCEccentricCProfile(cursor, point) && !point.valid)) return result;

    // Both signs and both existing H modes use the same positive scalar.
    for (int toolMode : {43, 44})
        for (double direction : {1.0, -1.0})
        {
            auto input = Input();
            input.geometry.source.toolLengthMode = toolMode;
            input.geometry.sweepDeg *= direction;
            if (!check(PrepareNCEccentricCRuntime(input, runtime) == NCEccentricCRuntimeCode::PREPARED)) return result;
            if (!check(PrepareNCEccentricCProfile(runtime, plan) == NCEccentricCProfileCode::PREPARED && plan.IsValid() &&
                plan.TotalTicks() > 2ULL && plan.TotalTicks() < 2048ULL &&
                plan.PeakVelocityPulsePerSec() <= runtime.MaximumScalarVelocityPulsePerSec() &&
                plan.AccelerationPulsePerSec2() <= runtime.MaximumScalarAccelerationPulsePerSec2() &&
                plan.DecelerationPulsePerSec2() <= runtime.MaximumScalarDecelerationPulsePerSec2())) return result;
            cursor.Clear();
            if (!check(BeginNCEccentricCProfile(plan, cursor, point) && point.scalarPulse == 0.0 &&
                point.velocityPulsePerSec == 0.0 && NCEccentricCDetail::Same(point.runtime.positionPulse, input.startPulse))) return result;
            const auto ticks = plan.TotalTicks();
            // Mutating/clearing the caller's prepared objects cannot alter a cursor.
            plan.Clear(); runtime.Clear();
            if (!check(!plan.IsValid() && !runtime.IsValid() && cursor.Runtime().IsValid() && Finish(cursor, point))) return result;
            if (!check(point.state == State::AUTHORED_COMPLETE && point.scalarPulse == cursor.Runtime().AuthoredScalarPulse() &&
                point.sequence == ticks && point.velocityPulsePerSec == 0.0 &&
                NCEccentricCDetail::Same(point.runtime.positionPulse, cursor.Runtime().EndPulse()) &&
                Close(point.runtime.positionPulse[3], direction * 10.0))) return result;
            const auto terminal = point;
            if (!check(AdvanceNCEccentricCProfile(cursor, point) && point.sequence == terminal.sequence &&
                NCEccentricCDetail::Same(point.runtime.positionPulse, terminal.runtime.positionPulse))) return result;
            if (!check(!ResumeNCEccentricCProfile(cursor, point) && !point.valid && cursor.State() == State::AUTHORED_COMPLETE)) return result;
            if (!check(!RequestNCEccentricCProfileStop(cursor, point) && !point.valid && cursor.State() == State::AUTHORED_COMPLETE)) return result;
        }

    auto input = Input();
    if (!check(PrepareNCEccentricCRuntime(input, runtime) == NCEccentricCRuntimeCode::PREPARED &&
        PrepareNCEccentricCProfile(runtime, plan) == NCEccentricCProfileCode::PREPARED)) return result;
    cursor.Clear();
    if (!check(BeginNCEccentricCProfile(plan, cursor, point) && RequestNCEccentricCProfileStop(cursor, point) &&
        cursor.State() == State::STOPPED_BEFORE_END && cursor.ScalarPulse() == 0.0 && cursor.Sequence() == 0ULL)) return result;
    if (!check(RequestNCEccentricCProfileStop(cursor, point) && AdvanceNCEccentricCProfile(cursor, point) &&
        cursor.ScalarPulse() == 0.0 && cursor.Sequence() == 0ULL)) return result;
    if (!check(ResumeNCEccentricCProfile(cursor, point) && cursor.State() == State::AUTHORED_ACTIVE &&
        point.scalarPulse == 0.0 && point.velocityPulsePerSec == 0.0)) return result;
    if (!check(AdvanceNCEccentricCProfile(cursor, point) && point.scalarPulse > 0.0 && point.velocityPulsePerSec > 0.0)) return result;
    const auto moving = point;
    if (!check(!BeginNCEccentricCProfile(plan, cursor, point) && !point.valid &&
        cursor.ScalarPulse() == moving.scalarPulse && cursor.Sequence() == moving.sequence)) return result;
    if (!check(RequestNCEccentricCProfileStop(cursor, point) && cursor.State() == State::CONTROLLED_STOP_ACTIVE &&
        point.scalarPulse == moving.scalarPulse && point.velocityPulsePerSec == moving.velocityPulsePerSec &&
        point.sequence == moving.sequence)) return result;
    const double stopTarget = cursor.SegmentEndPulse();
    const auto stopTicks = cursor.SegmentTicks();
    if (!check(!ResumeNCEccentricCProfile(cursor, point) && !point.valid &&
        cursor.State() == State::CONTROLLED_STOP_ACTIVE && cursor.SegmentEndPulse() == stopTarget)) return result;
    if (!check(RequestNCEccentricCProfileStop(cursor, point) && cursor.SegmentEndPulse() == stopTarget &&
        cursor.SegmentTick() == 0ULL && cursor.SegmentTicks() == stopTicks)) return result;
    if (!check(Finish(cursor, point) && cursor.State() == State::STOPPED_BEFORE_END &&
        point.scalarPulse == stopTarget && point.velocityPulsePerSec == 0.0)) return result;
    const auto held = point;
    if (!check(AdvanceNCEccentricCProfile(cursor, point) && point.sequence == held.sequence &&
        NCEccentricCDetail::Same(point.runtime.positionPulse, held.runtime.positionPulse))) return result;
    if (!check(ResumeNCEccentricCProfile(cursor, point) && point.scalarPulse == held.scalarPulse &&
        point.velocityPulsePerSec == 0.0 && point.sequence == held.sequence &&
        NCEccentricCDetail::Same(point.runtime.positionPulse, held.runtime.positionPulse))) return result;
    if (!check(Finish(cursor, point) && point.state == State::AUTHORED_COMPLETE &&
        NCEccentricCDetail::Same(point.runtime.positionPulse, runtime.EndPulse()))) return result;

    // Slow controlled STOP can pass the authored endpoint; never clamp back.
    cursor.Clear();
    if (!check(BeginNCEccentricCProfile(plan, cursor, point))) return result;
    bool advanced = true;
    for (unsigned step = 0U; step < 8U; ++step)
        advanced = AdvanceNCEccentricCProfile(cursor, point) && advanced;
    if (!check(advanced && RequestNCEccentricCProfileStop(cursor, point) &&
        cursor.SegmentEndPulse() > runtime.AuthoredScalarPulse() &&
        cursor.SegmentEndPulse() <= runtime.MaximumScalarPulse())) return result;
    if (!check(Finish(cursor, point) && cursor.State() == State::STOPPED_AT_OR_BEYOND_END &&
        point.scalarPulse > runtime.AuthoredScalarPulse() && point.velocityPulsePerSec == 0.0)) return result;
    const auto overrun = point;
    if (!check(!ResumeNCEccentricCProfile(cursor, point) && !point.valid && cursor.ScalarPulse() == overrun.scalarPulse &&
        cursor.Sequence() == overrun.sequence && cursor.State() == State::STOPPED_AT_OR_BEYOND_END)) return result;
    if (!check(RequestNCEccentricCProfileStop(cursor, point) && AdvanceNCEccentricCProfile(cursor, point) &&
        point.sequence == overrun.sequence && NCEccentricCDetail::Same(point.runtime.positionPulse, overrun.runtime.positionPulse))) return result;

    input = Input(); input.geometry.sweepDeg = 1.0e-7;
    if (!check(PrepareNCEccentricCRuntime(input, runtime) == NCEccentricCRuntimeCode::PREPARED &&
        PrepareNCEccentricCProfile(runtime, plan) == NCEccentricCProfileCode::PREPARED && plan.CruiseTicks() == 0ULL)) return result;
    cursor.Clear();
    if (!check(BeginNCEccentricCProfile(plan, cursor, point) && Finish(cursor, point) &&
        point.state == State::AUTHORED_COMPLETE && point.scalarPulse == runtime.AuthoredScalarPulse() &&
        NCEccentricCDetail::Same(point.runtime.positionPulse, runtime.EndPulse()))) return result;

    // Regression for cancellation at the start of a long, endpoint-anchored STOP.
    // Probe only a bounded prefix here; exhaustive sweeps are HOST tests.
    input = Input(); input.stopSeconds = 60.0; input.geometry.feedDegMin = 100.0;
    input.geometry.accTime = 0.01011; input.geometry.decTime = 0.02013;
    input.pulsePerNative[3] = 16777216.0 / 360.0;
    if (!check(PrepareNCEccentricCRuntime(input, runtime) == NCEccentricCRuntimeCode::PREPARED &&
        PrepareNCEccentricCProfile(runtime, plan) == NCEccentricCProfileCode::PREPARED)) return result;
    cursor.Clear();
    if (!check(BeginNCEccentricCProfile(plan, cursor, point))) return result;
    advanced = true;
    for (unsigned step = 0U; step < 25U; ++step)
        advanced = AdvanceNCEccentricCProfile(cursor, point) && advanced;
    if (!check(advanced && RequestNCEccentricCProfileStop(cursor, point))) return result;
    advanced = true;
    for (unsigned step = 0U; step < 8U; ++step)
        advanced = AdvanceNCEccentricCProfile(cursor, point) && advanced;
    if (!check(advanced && point.state == State::CONTROLLED_STOP_ACTIVE && point.valid)) return result;

    // Failed preparation clears the destination and leaves an existing cursor intact.
    const auto savedScalar = cursor.ScalarPulse();
    const auto savedSequence = cursor.Sequence();
    runtime.Clear();
    if (!check(PrepareNCEccentricCProfile(runtime, plan) == NCEccentricCProfileCode::INVALID_RUNTIME && !plan.IsValid())) return result;
    if (!check(!BeginNCEccentricCProfile(plan, cursor, point) && !point.valid &&
        cursor.ScalarPulse() == savedScalar && cursor.Sequence() == savedSequence)) return result;
    cursor.Clear();
    if (!check(!cursor.IsValid() && !ReadNCEccentricCProfile(cursor, point) && !point.valid &&
        point.scalarPulse == 0.0 && point.runtime.positionPulse[0] == 0.0)) return result;
    return result;
}
