#pragma once

#include "MotionEccentricCExecutor.h"
#include "NCEccentricCProfileSelfCheck.h"
#include <memory>
#include <new>

// BASE79A: isolated executor validation, called only during startup / NC LOAD.
// No MotionCore include, command packet, live instance, axis, queue or output.
// The synthetic C role below never changes the machine's configured role.
// This result is diagnostic only while dynamic NC admission remains closed.
struct NCEccentricCExecutorSelfCheckResult
{
    std::uint32_t checks = 0U;
    std::uint32_t failedCheck = 0U;
    bool passed = true;
};

namespace NCEccentricCExecutorSelfCheckDetail
{
    // One allocation per call, private to that invocation. Keep all long-lived
    // test objects off the bounded startup / NC LOAD thread stack.
    struct Workspace
    {
        NCEccentricCRuntimeInput input = NCEccentricCProfileSelfCheckDetail::Input();
        NCEccentricCRuntimeValue runtime{};
        NCEccentricCProfileValue plan{};
        MotionEccentricCExecutor executor{};
        NCEccentricCProfilePoint point{}, saved{}, terminal{};
        MotionEccentricCContext context{}, bad{}, safety{}, successor{};
        MotionExecutionIdentity identity{};
        MotionOwnerLease owner{};
    };

    inline MotionEccentricCContext Context(const MotionExecutionIdentity& identity,
        const MotionOwnerLease& owner) noexcept
    {
        MotionEccentricCContext context{};
        context.execution = identity; context.sourceOwner = owner;
        context.currentEpoch = identity.epoch; context.currentOwner = owner;
        return context;
    }

    inline bool Prepare(Workspace& w) noexcept
    {
        return PrepareNCEccentricCRuntime(w.input, w.runtime) == NCEccentricCRuntimeCode::PREPARED &&
            PrepareNCEccentricCProfile(w.runtime, w.plan) == NCEccentricCProfileCode::PREPARED;
    }

    inline bool SamePoint(const NCEccentricCProfilePoint& a,
        const NCEccentricCProfilePoint& b) noexcept
    {
        return a.valid == b.valid && a.state == b.state && a.sequence == b.sequence &&
            a.scalarPulse == b.scalarPulse && a.velocityPulsePerSec == b.velocityPulsePerSec &&
            a.accelerationPulsePerSec2 == b.accelerationPulsePerSec2 &&
            NCEccentricCDetail::Same(a.runtime.positionPulse, b.runtime.positionPulse) &&
            NCEccentricCDetail::Same(a.runtime.velocityPulse, b.runtime.velocityPulse);
    }

    inline bool Finish(MotionEccentricCExecutor& executor,
        const MotionEccentricCContext& context, NCEccentricCProfilePoint& output) noexcept
    {
        using State = NCEccentricCProfileState;
        for (unsigned n = 0U; n < 2048U; ++n)
        {
            const double previousPosition = executor.Current().scalarPulse;
            const double previousVelocity = executor.Current().velocityPulsePerSec;
            const auto previousSequence = executor.Current().sequence;
            const bool stopping = executor.State() == State::CONTROLLED_STOP_ACTIVE;
            if (!executor.Advance(context, output) || !output.valid || !output.runtime.valid ||
                output.scalarPulse < previousPosition || output.velocityPulsePerSec < 0.0 ||
                output.velocityPulsePerSec > executor.Runtime().MaximumScalarVelocityPulsePerSec() ||
                output.sequence != previousSequence + 1ULL) return false;
            const double change = output.velocityPulsePerSec - previousVelocity;
            const double ceiling = change >= 0.0 ? executor.Runtime().MaximumScalarAccelerationPulsePerSec2() :
                stopping ? executor.Runtime().MaximumScalarStopDecelerationPulsePerSec2() :
                executor.Runtime().MaximumScalarDecelerationPulsePerSec2();
            if (std::fabs(change) / executor.Runtime().CycleSeconds() > ceiling) return false;
            for (unsigned axis : {2U, 4U, 5U, 6U, 7U})
                if (!NCEccentricCDetail::Same(output.runtime.positionPulse[axis], executor.Runtime().StartPulse()[axis]) ||
                    output.runtime.velocityPulse[axis] != 0.0) return false;
            if (executor.State() != State::AUTHORED_ACTIVE && executor.State() != State::CONTROLLED_STOP_ACTIVE)
                return output.velocityPulsePerSec == 0.0 && output.accelerationPulsePerSec2 == 0.0;
        }
        return false;
    }
}

// Preserve this call boundary even in optimized builds: the caller must not
// inherit a large synthetic workspace or an inlined nested test call chain.
#if defined(_MSC_VER)
__declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
__attribute__((noinline))
#endif
inline NCEccentricCExecutorSelfCheckResult RunNCEccentricCExecutorSelfCheck() noexcept
{
    using namespace NCEccentricCExecutorSelfCheckDetail;
    using State = NCEccentricCProfileState;
    NCEccentricCExecutorSelfCheckResult result{};
    const auto check = [&](bool condition) noexcept -> bool
    {
        ++result.checks;
        if (!condition) { result.passed = false; result.failedCheck = result.checks; }
        return condition;
    };
    const std::unique_ptr<Workspace> workspace(new (std::nothrow) Workspace);
    if (!check(workspace && !NCEccentricCDynamicMotionAdmission &&
        NCEccentricCProfileValue::RequiresUnfilteredSamples())) return result;
    auto& w = *workspace;
    auto& executor = w.executor;
    auto& point = w.point;
    auto& context = w.context;
    w.identity.epoch = 1U; w.identity.segmentId = 1ULL;
    w.identity.sourceBlockId = 17; w.identity.source = MotionCommandSource::NC_MEMORY;
    w.owner.owner = MotionOwner::AUTO; w.owner.generation = 1U;
    context = Context(w.identity, w.owner);
    if (!check(!executor.Begin(w.plan, w.identity, w.owner, context, point) && !point.valid && !executor.IsValid())) return result;
    if (!check(!executor.Advance(context, point) && !point.valid && !executor.IsValid())) return result;
    if (!check(!executor.RequestStop(context, point) && !point.valid)) return result;
    if (!check(!executor.Resume(context, point) && !point.valid)) return result;

    // G43/G44 and both C directions: copied ownership, exact endpoint,
    // bounded derivatives and unchanged nonparticipating axes.
    w.input.geometry.source.distanceMode = 91;
    for (int mode : {43, 44})
        for (double direction : {1.0, -1.0})
        {
            w.input.geometry.source.toolLengthMode = mode;
            w.input.geometry.sweepDeg = direction * 0.01;
            if (!check(Prepare(w) && w.plan.IsValid())) return result;
            if (!check(w.plan.Runtime().AuthoredPath().groupMask == 11U &&
                w.plan.Runtime().AuthoredPath().electrodeAxis == 3U && w.plan.Runtime().FirSamples() == 1U)) return result;
            executor.Clear();
            if (!check(executor.Begin(w.plan, w.identity, w.owner, context, point) &&
                point.scalarPulse == 0.0 && point.velocityPulsePerSec == 0.0 &&
                NCEccentricCDetail::Same(point.runtime.positionPulse, w.input.startPulse))) return result;
            const auto totalTicks = w.plan.TotalTicks();
            w.plan.Clear(); w.runtime.Clear();
            if (!check(Finish(executor, context, point) && executor.State() == State::AUTHORED_COMPLETE &&
                point.sequence == totalTicks &&
                NCEccentricCDetail::Same(point.runtime.positionPulse, executor.Runtime().EndPulse()) &&
                NCEccentricCProfileSelfCheckDetail::Close(point.runtime.positionPulse[3], direction * 10.0))) return result;
            w.terminal = point;
            if (!check(executor.Advance(context, point) && SamePoint(point, w.terminal))) return result;
            if (!check(!executor.Resume(context, point) && !point.valid && SamePoint(executor.Current(), w.terminal))) return result;
            if (!check(!executor.RequestStop(context, point) && !point.valid && SamePoint(executor.Current(), w.terminal))) return result;
        }

    w.input.geometry.source.toolLengthMode = 43; w.input.geometry.sweepDeg = 0.01;
    if (!check(Prepare(w))) return result;
    for (unsigned defect = 0U; defect < 7U; ++defect)
    {
        w.bad = context;
        switch (defect)
        {
        case 0U: w.bad.execution.epoch = MOTION_EXECUTION_EPOCH_INVALID; break;
        case 1U: w.bad.execution.segmentId = MOTION_SEGMENT_ID_INVALID; break;
        case 2U: w.bad.execution.sourceBlockId = MOTION_SOURCE_BLOCK_ID_INVALID; break;
        case 3U: w.bad.execution.source = MotionCommandSource::NC_MDI; break;
        case 4U: w.bad.sourceOwner.owner = MotionOwner::NONE; break;
        case 5U: w.bad.sourceOwner.generation = MOTION_OWNER_GENERATION_INVALID; break;
        default: w.bad.sourceOwner.owner = MotionOwner::MDI; break;
        }
        w.bad.currentEpoch = w.bad.execution.epoch; w.bad.currentOwner = w.bad.sourceOwner;
        executor.Clear();
        if (!check(!executor.Begin(w.plan, w.bad.execution, w.bad.sourceOwner, w.bad, point) &&
            !point.valid && !executor.IsValid())) return result;
    }
    if (!check(executor.Begin(w.plan, w.identity, w.owner, context, point) &&
        executor.Advance(context, point) && point.velocityPulsePerSec > 0.0)) return result;
    w.saved = point;
    for (unsigned defect = 0U; defect < 15U; ++defect)
    {
        w.bad = context;
        switch (defect)
        {
        case 0U: ++w.bad.currentEpoch; break;
        case 1U: ++w.bad.currentOwner.generation; break;
        case 2U: ++w.bad.execution.segmentId; break;
        case 3U: ++w.bad.execution.sourceBlockId; break;
        case 4U: ++w.bad.sourceOwner.generation; break;
        case 5U: w.bad.fault = true; break;
        case 6U: w.bad.safetyTicket = 99ULL; break;
        case 7U: ++w.bad.execution.epoch; break;
        case 8U: w.bad.execution.source = MotionCommandSource::NC_MDI; break;
        case 9U: w.bad.sourceOwner.owner = MotionOwner::MDI; break;
        case 10U: w.bad.safetyRequestAuthorized = true; break;
        case 11U: w.bad.safetyAuthorized = true; break;
        case 12U: w.bad.previousSafetyEpoch = 2U; break;
        case 13U: w.bad.previousSafetyOwner = w.owner; break;
        default: w.bad.previousSafetyTicket = 99ULL; break;
        }
        if (!check(!executor.Advance(w.bad, point) && !point.valid && SamePoint(executor.Current(), w.saved) &&
            !executor.RequestStop(w.bad, point) && !point.valid && SamePoint(executor.Current(), w.saved))) return result;
    }
    if (!check(!executor.Begin(w.plan, w.identity, w.owner, context, point) && !point.valid &&
        SamePoint(executor.Current(), w.saved))) return result;
    if (!check(executor.RequestStop(context, point) && point.scalarPulse == w.saved.scalarPulse &&
        point.velocityPulsePerSec == w.saved.velocityPulsePerSec && point.sequence == w.saved.sequence)) return result;
    if (!check(executor.RequestStop(context, point) && point.sequence == w.saved.sequence &&
        Finish(executor, context, point) && executor.State() == State::STOPPED_BEFORE_END)) return result;
    w.saved = point;
    if (!check(executor.Advance(context, point) && SamePoint(point, w.saved))) return result;
    if (!check(executor.Resume(context, point) && point.scalarPulse == w.saved.scalarPulse &&
        point.velocityPulsePerSec == 0.0 && point.sequence == w.saved.sequence)) return result;
    if (!check(Finish(executor, context, point) && executor.State() == State::AUTHORED_COMPLETE)) return result;

    // A safety request may bind a zero-time stop before applied authority.
    // Samples must wait for the exact applied epoch, owner and ticket.
    executor.Clear();
    if (!check(executor.Begin(w.plan, w.identity, w.owner, context, point) && executor.Advance(context, point))) return result;
    w.safety = context;
    w.safety.currentEpoch = 2U; w.safety.currentOwner.owner = MotionOwner::SAFETY;
    w.safety.currentOwner.generation = 2U; w.safety.safetyTicket = 19ULL;
    w.safety.safetyRequestAuthorized = true;
    w.saved = point;
    if (!check(executor.RequestStop(w.safety, point) && executor.IsSafetyStop() &&
        point.scalarPulse == w.saved.scalarPulse && point.velocityPulsePerSec == w.saved.velocityPulsePerSec &&
        point.sequence == w.saved.sequence)) return result;
    if (!check(!executor.Advance(w.safety, point) && !point.valid && executor.Current().sequence == w.saved.sequence)) return result;
    w.safety.safetyAuthorized = true;
    if (!check(executor.Advance(w.safety, point) && point.sequence == w.saved.sequence + 1ULL)) return result;
    w.saved = point;
    for (unsigned defect = 0U; defect < 4U; ++defect)
    {
        w.bad = w.safety;
        switch (defect)
        {
        case 0U: ++w.bad.safetyTicket; break;
        case 1U: ++w.bad.currentOwner.generation; break;
        case 2U: ++w.bad.currentEpoch; break;
        default: w.bad.fault = true; break;
        }
        if (!check(!executor.Advance(w.bad, point) && !point.valid && SamePoint(executor.Current(), w.saved))) return result;
    }
    if (!check(!executor.Advance(context, point) && !point.valid &&
        !executor.Resume(context, point) && !point.valid && SamePoint(executor.Current(), w.saved))) return result;
    w.successor = w.safety;
    w.successor.currentEpoch = 3U; w.successor.currentOwner.generation = 3U;
    w.successor.safetyTicket = 20ULL; w.successor.safetyAuthorized = false;
    if (!check(!executor.RequestStop(w.successor, point) && !point.valid && SamePoint(executor.Current(), w.saved))) return result;
    w.successor.previousSafetyEpoch = w.safety.currentEpoch;
    w.successor.previousSafetyOwner = w.safety.currentOwner;
    w.successor.previousSafetyTicket = w.safety.safetyTicket;
    if (!check(executor.RequestStop(w.successor, point) && point.sequence == w.saved.sequence &&
        point.scalarPulse == w.saved.scalarPulse && point.velocityPulsePerSec == w.saved.velocityPulsePerSec)) return result;
    if (!check(!executor.Advance(w.safety, point) && !point.valid && SamePoint(executor.Current(), w.saved))) return result;
    if (!check(!executor.Advance(w.successor, point) && !point.valid && SamePoint(executor.Current(), w.saved))) return result;
    w.successor.safetyAuthorized = true;
    if (!check(executor.Advance(w.successor, point) && point.sequence == w.saved.sequence + 1ULL)) return result;
    w.saved = point;
    if (!check(!executor.RequestStop(w.safety, point) && !point.valid && SamePoint(executor.Current(), w.saved))) return result;
    w.successor.previousSafetyEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    w.successor.previousSafetyOwner = MotionOwnerLease{}; w.successor.previousSafetyTicket = 0ULL;
    if (!check(executor.RequestStop(w.successor, point) && point.sequence == w.saved.sequence &&
        point.scalarPulse == w.saved.scalarPulse && point.velocityPulsePerSec == w.saved.velocityPulsePerSec)) return result;
    if (!check(Finish(executor, w.successor, point) && executor.State() == State::STOPPED_BEFORE_END &&
        !executor.Resume(w.successor, point) && !point.valid)) return result;

    // Endpoint takeover must bind the same zero-velocity point, without a
    // duplicate time sample or an axis jump, and cannot resume authored motion.
    executor.Clear();
    if (!check(executor.Begin(w.plan, w.identity, w.owner, context, point) &&
        Finish(executor, context, point) && executor.State() == State::AUTHORED_COMPLETE)) return result;
    w.terminal = point;
    if (!check(executor.RequestStop(w.safety, point) && executor.IsSafetyStop() && SamePoint(point, w.terminal))) return result;
    if (!check(executor.Advance(w.safety, point) && SamePoint(point, w.terminal) &&
        !executor.Resume(context, point) && !point.valid)) return result;
    executor.Clear();
    if (!check(!executor.IsValid() && !executor.IsSafetyStop() &&
        executor.Identity().epoch == MOTION_EXECUTION_EPOCH_INVALID && !executor.Owner().IsValid() &&
        !executor.Advance(context, point) && !point.valid)) return result;
    return result;
}
