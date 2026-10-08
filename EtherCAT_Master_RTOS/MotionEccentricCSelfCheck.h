#pragma once

#include "MotionCore.h"
#include "NCEccentricCProfileSelfCheck.h"
#include <cstddef>
#include <limits>
#include <memory>
#include <new>

// BASE79: synthetic packet + owned consumer tests on startup/non-RT NC LOAD.
// No MotionCore instance, live axis, queue, owner publication or output write.
// Runtime integration is independently tested on HOST; this result grants no
// live NC admission and does not qualify 250 us wall-clock execution.
struct MotionEccentricCSelfCheckResult
{
    std::uint32_t checks = 0U;
    std::uint32_t failedCheck = 0U;
    bool passed = true;
};

namespace MotionEccentricCSelfCheckDetail
{
    // BASE79 FIX1: these test-owned values outlive nested profile/transport
    // calls. Keep them off the bounded RTSS startup/NC LOAD thread stack.
    // Each invocation owns its workspace; the live Motion consumer does not
    // use this allocation and no state is shared between concurrent checks.
    struct Workspace
    {
        MotionCommand command{}, original{};
        NCEccentricCProfileValue plan{};
        NCEccentricCProfilePoint point{}, terminal{}, moving{}, held{}, beforeStop{},
            safetyCurrent{}, successorCurrent{};
        MotionEccentricCExecutor executor{};
        MotionEccentricCContext context{}, bad{}, safety{}, wrongSafety{}, successor{};
        NCEccentricCRuntimeInput input = NCEccentricCProfileSelfCheckDetail::Input();
    };

    inline MotionEccentricCContext Context(const MotionCommand& command) noexcept
    {
        MotionEccentricCContext context{};
        context.execution = command.execution; context.sourceOwner = command.ownerLease;
        context.currentEpoch = command.execution.epoch; context.currentOwner = command.ownerLease;
        return context;
    }
    inline bool Finish(MotionEccentricCExecutor& executor, const MotionEccentricCContext& context,
        NCEccentricCProfilePoint& output) noexcept
    {
        using State = NCEccentricCProfileState;
        for (unsigned n = 0U; n < 2048U; ++n)
        {
            const double before = executor.Current().scalarPulse;
            if (!executor.Advance(context, output) || !output.valid || !output.runtime.valid ||
                output.scalarPulse < before || output.velocityPulsePerSec < 0.0 ||
                output.velocityPulsePerSec > executor.Runtime().MaximumScalarVelocityPulsePerSec()) return false;
            for (unsigned axis : {2U, 4U, 5U, 6U, 7U})
                if (!NCEccentricCDetail::Same(output.runtime.positionPulse[axis], executor.Runtime().StartPulse()[axis]) ||
                    output.runtime.velocityPulse[axis] != 0.0) return false;
            if (executor.State() != State::AUTHORED_ACTIVE && executor.State() != State::CONTROLLED_STOP_ACTIVE)
                return output.velocityPulsePerSec == 0.0;
        }
        return false;
    }
}

inline MotionEccentricCSelfCheckResult RunMotionEccentricCSelfCheck() noexcept
{
    using namespace MotionEccentricCSelfCheckDetail;
    using State = NCEccentricCProfileState;
    MotionEccentricCSelfCheckResult result{};
    const auto check = [&](bool condition) noexcept -> bool
    {
        ++result.checks;
        if (!condition) { result.passed = false; result.failedCheck = result.checks; }
        return condition;
    };
    const std::unique_ptr<Workspace> workspace(new (std::nothrow) Workspace);
    // Allocation failure is the first existing check: fail closed before any
    // workspace access and preserve the successful 78-check contract.
    if (!check(workspace && !NCEccentricCDynamicMotionAdmission && sizeof(MotionCommand) == 1064U &&
        offsetof(MotionCommand, pathCoreEccentricCFeedExactStop) == 385U)) return result;
    MotionExecutionIdentity identity{};
    identity.epoch = 1U; identity.segmentId = 1ULL; identity.sourceBlockId = 17;
    identity.source = MotionCommandSource::NC_MEMORY;
    MotionOwnerLease owner{}; owner.owner = MotionOwner::AUTO; owner.generation = 1U;
    auto& command = workspace->command;
    auto& plan = workspace->plan;
    auto& point = workspace->point;
    auto& executor = workspace->executor;
    auto& context = workspace->context;
    if (!check(!PrepareMotionEccentricCCommand(command, plan) && !plan.IsValid())) return result;
    if (!check(!executor.Advance(context, point) && !point.valid && !executor.IsValid())) return result;
    if (!check(!executor.RequestStop(context, point) && !point.valid)) return result;
    if (!check(!executor.Resume(context, point) && !point.valid)) return result;

    auto& input = workspace->input;
    input.geometry.source.distanceMode = 91;
    for (int mode : {43, 44})
        for (double direction : {1.0, -1.0})
        {
            input.geometry.source.toolLengthMode = mode;
            input.geometry.sweepDeg = direction * 0.01;
            if (!check(BuildMotionEccentricCCommand(input, identity, owner, command) &&
                PrepareMotionEccentricCCommand(command, plan) && plan.IsValid())) return result;
            if (!check(command.pathCoreEccentricCFeedExactStop && command.axisCount == 3 &&
                command.axisIndices[0] == 0 && command.axisIndices[1] == 1 && command.axisIndices[2] == 3 &&
                plan.Runtime().AuthoredPath().groupMask == 11U && plan.Runtime().FirSamples() == 1U)) return result;
            if (!check(IsMotionEccentricCFeedSourceAllowed(command) &&
                !IsMotionFixedTranslationSourceAllowed(command) && !IsMotionRotaryFeedSourceAllowed(command) &&
                !IsMotionXYZCUVFeedSourceAllowed(command))) return result;
            context = Context(command); executor.Clear();
            if (!check(executor.Begin(plan, command.execution, command.ownerLease, context, point) &&
                point.scalarPulse == 0.0 && point.velocityPulsePerSec == 0.0 &&
                NCEccentricCDetail::Same(point.runtime.positionPulse, input.startPulse))) return result;
            const auto totalTicks = plan.TotalTicks();
            plan.Clear();
            if (!check(Finish(executor, context, point) && executor.State() == State::AUTHORED_COMPLETE &&
                point.sequence == totalTicks && NCEccentricCDetail::Same(point.runtime.positionPulse, executor.Runtime().EndPulse()))) return result;
            auto& terminal = workspace->terminal; terminal = point;
            if (!check(executor.Advance(context, point) && point.sequence == terminal.sequence &&
                NCEccentricCDetail::Same(point.runtime.positionPulse, terminal.runtime.positionPulse))) return result;
        }

    input.geometry.source.toolLengthMode = 43; input.geometry.sweepDeg = 0.01;
    if (!check(BuildMotionEccentricCCommand(input, identity, owner, command) && PrepareMotionEccentricCCommand(command, plan))) return result;
    auto& original = workspace->original; original = command;
    const auto fingerprint = FoldMotionEccentricCTransportFingerprint(1469598103934665603ULL, command);
    if (!check(fingerprint == FoldMotionEccentricCTransportFingerprint(1469598103934665603ULL, original))) return result;
    // Changes to a frozen source or envelope are visible even if endpoints agree.
    command.sourceTranslation.extOffsetMM[0] += 1.0;
    if (!check(FoldMotionEccentricCTransportFingerprint(1469598103934665603ULL, command) != fingerprint)) return result;
    command = original; command.mem_transformMatrix[0][1] *= 2.0;
    if (!check(FoldMotionEccentricCTransportFingerprint(1469598103934665603ULL, command) != fingerprint)) return result;
    for (unsigned defect = 0U; defect < 16U; ++defect)
    {
        command = original;
        switch (defect)
        {
        case 0U: command.pathCoreEccentricCFeedExactStop = false; break;
        case 1U: command.pathCoreRotaryFeedExactStop = true; break;
        case 2U: command.replayTerminalAlreadyPublished = true; break;
        case 3U: command.cncFeedLookahead = true; break;
        case 4U: command.mem_enableTransform = true; break;
        case 5U: command.axisIndices[1] = 0; break;
        case 6U: command.targetPos[0] = std::nextafter(command.targetPos[0], (std::numeric_limits<double>::infinity)()); break;
        case 7U: command.mem_ratio[3] = 0.0; break;
        case 8U: command.mem_transformMatrix[0][0] = (std::numeric_limits<double>::quiet_NaN)(); break;
        case 9U: command.sourceTranslation.axisIdentity.electrodeAxisPlusOne = 0U; break;
        case 10U: command.sourceTranslation.distanceMode = 90; break;
        case 11U: command.execution.segmentId = 0ULL; break;
        case 12U: command.ownerLease.owner = MotionOwner::MDI; break;
        case 13U: command.sourceLinePC += 1; break;
        case 14U: command.targetPos[7] = 1.0; break;
        default: command.mem_centerX = 0.0; break;
        }
        if (!check(!PrepareMotionEccentricCCommand(command, plan) && !plan.IsValid())) return result;
    }
    command = original;
    if (!check(PrepareMotionEccentricCCommand(command, plan))) return result;
    context = Context(command); executor.Clear();
    if (!check(executor.Begin(plan, command.execution, command.ownerLease, context, point) &&
        executor.Advance(context, point) && point.velocityPulsePerSec > 0.0)) return result;
    auto& moving = workspace->moving; moving = point;
    for (unsigned defect = 0U; defect < 7U; ++defect)
    {
        auto& bad = workspace->bad; bad = context;
        switch (defect)
        {
        case 0U: ++bad.currentEpoch; break;
        case 1U: ++bad.currentOwner.generation; break;
        case 2U: ++bad.execution.segmentId; break;
        case 3U: ++bad.execution.sourceBlockId; break;
        case 4U: ++bad.sourceOwner.generation; break;
        case 5U: bad.fault = true; break;
        default: bad.safetyTicket = 99ULL; break;
        }
        if (!check(!executor.Advance(bad, point) && !point.valid &&
            executor.Current().scalarPulse == moving.scalarPulse && executor.Current().sequence == moving.sequence)) return result;
    }
    if (!check(executor.RequestStop(context, point) && point.scalarPulse == moving.scalarPulse &&
        point.velocityPulsePerSec == moving.velocityPulsePerSec && point.sequence == moving.sequence)) return result;
    if (!check(executor.RequestStop(context, point) && point.sequence == moving.sequence &&
        Finish(executor, context, point) && executor.State() == State::STOPPED_BEFORE_END)) return result;
    auto& held = workspace->held; held = point;
    if (!check(executor.Resume(context, point) && point.scalarPulse == held.scalarPulse &&
        point.velocityPulsePerSec == 0.0 && point.sequence == held.sequence)) return result;
    if (!check(Finish(executor, context, point) && executor.State() == State::AUTHORED_COMPLETE)) return result;

    // A published SAFETY request can bind a zero-time stop before its applied
    // proof exists. Advancing/axis output must wait for the applied proof.
    executor.Clear();
    if (!check(executor.Begin(plan, command.execution, command.ownerLease, context, point) && executor.Advance(context, point))) return result;
    auto& safety = workspace->safety; safety = context;
    safety.currentEpoch = 2U; safety.currentOwner.owner = MotionOwner::SAFETY;
    safety.currentOwner.generation = 2U; safety.safetyTicket = 19ULL;
    safety.safetyRequestAuthorized = true;
    auto& beforeStop = workspace->beforeStop; beforeStop = point;
    if (!check(executor.RequestStop(safety, point) && executor.IsSafetyStop() &&
        point.scalarPulse == beforeStop.scalarPulse && point.velocityPulsePerSec == beforeStop.velocityPulsePerSec &&
        point.sequence == beforeStop.sequence)) return result;
    if (!check(!executor.Advance(safety, point) && !point.valid && executor.Current().sequence == beforeStop.sequence)) return result;
    safety.safetyAuthorized = true;
    if (!check(executor.Advance(safety, point) && point.sequence == beforeStop.sequence + 1ULL)) return result;
    auto& safetyCurrent = workspace->safetyCurrent; safetyCurrent = point;
    auto& wrongSafety = workspace->wrongSafety; wrongSafety = safety; ++wrongSafety.safetyTicket;
    if (!check(!executor.Advance(wrongSafety, point) && !point.valid &&
        executor.Current().sequence == safetyCurrent.sequence)) return result;
    wrongSafety = safety; ++wrongSafety.currentOwner.generation;
    if (!check(!executor.Advance(wrongSafety, point) && !point.valid &&
        executor.Current().sequence == safetyCurrent.sequence)) return result;
    if (!check(!executor.Advance(context, point) && !point.valid &&
        !executor.Resume(context, point) && !point.valid)) return result;
    // Repeated real STOP publishes a successor token. Bind that token without
    // consuming time; an old request must never roll the binding backward.
    auto& successor = workspace->successor; successor = safety;
    successor.currentEpoch = 3U; successor.currentOwner.generation = 3U;
    successor.safetyTicket = 20ULL; successor.safetyAuthorized = false;
    if (!check(!executor.RequestStop(successor, point) && !point.valid &&
        executor.Current().sequence == safetyCurrent.sequence &&
        executor.Current().scalarPulse == safetyCurrent.scalarPulse)) return result;
    successor.previousSafetyEpoch = safety.currentEpoch;
    successor.previousSafetyOwner = safety.currentOwner;
    successor.previousSafetyTicket = safety.safetyTicket;
    if (!check(executor.RequestStop(successor, point) &&
        point.sequence == safetyCurrent.sequence && point.scalarPulse == safetyCurrent.scalarPulse &&
        point.velocityPulsePerSec == safetyCurrent.velocityPulsePerSec)) return result;
    if (!check(!executor.Advance(safety, point) && !point.valid &&
        executor.Current().sequence == safetyCurrent.sequence)) return result;
    if (!check(!executor.Advance(successor, point) && !point.valid &&
        executor.Current().sequence == safetyCurrent.sequence)) return result;
    successor.safetyAuthorized = true;
    if (!check(executor.Advance(successor, point) &&
        point.sequence == safetyCurrent.sequence + 1ULL)) return result;
    auto& successorCurrent = workspace->successorCurrent; successorCurrent = point;
    if (!check(!executor.RequestStop(safety, point) && !point.valid &&
        executor.Current().sequence == successorCurrent.sequence &&
        executor.Current().scalarPulse == successorCurrent.scalarPulse)) return result;
    successor.previousSafetyEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    successor.previousSafetyOwner = MotionOwnerLease{}; successor.previousSafetyTicket = 0ULL;
    if (!check(executor.RequestStop(successor, point) &&
        point.sequence == successorCurrent.sequence && point.scalarPulse == successorCurrent.scalarPulse &&
        point.velocityPulsePerSec == successorCurrent.velocityPulsePerSec)) return result;
    if (!check(Finish(executor, successor, point) && executor.State() == State::STOPPED_BEFORE_END &&
        !executor.Resume(successor, point) && !point.valid)) return result;
    executor.Clear();
    if (!check(!executor.IsValid() && !executor.Advance(context, point) && !point.valid)) return result;
    return result;
}
