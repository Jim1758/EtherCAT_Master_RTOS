#pragma once

#include "NCEccentricCTransportSelfCheck.h"
#include "NCEccentricCExecutorSelfCheck.h"

// BASE79C: packet -> decoded profile -> isolated owned executor. Synthetic
// startup / NC LOAD diagnostics only; never a MotionCore permission or output.
struct NCEccentricCPipelineSelfCheckResult
{
    std::uint32_t checks = 0U;
    std::uint32_t failedCheck = 0U;
    bool passed = true;
};

#if defined(_MSC_VER)
#define BASE79C_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define BASE79C_NOINLINE __attribute__((noinline))
#else
#define BASE79C_NOINLINE
#endif

namespace NCEccentricCPipelineSelfCheckDetail
{
    // One private heap allocation per invocation, including transport scratch.
    // No static scratch, persistent instance, group member or nested allocation.
    struct Workspace
    {
        MotionEccentricCTransportWorkspace transport{};
        NCEccentricCRuntimeInput input = NCEccentricCProfileSelfCheckDetail::Input();
        MotionCommand packet{}, wire{};
        NCEccentricCProfileValue decoded{};
        MotionEccentricCExecutor executor{};
        NCEccentricCProfilePoint point{}, saved{}, terminal{};
        MotionEccentricCContext context{}, bad{}, safety{}, previous{};
        MotionExecutionIdentity identity{};
        MotionOwnerLease owner{};
        std::array<double, 8> expectedEnd{};
        std::uint64_t expectedTicks = 0ULL;
    };

    // Separate the transport and executor frames even under release inlining.
    BASE79C_NOINLINE inline bool Decode(Workspace& w) noexcept
    { return PrepareMotionEccentricCCommand(w.wire, w.decoded, w.transport); }

    BASE79C_NOINLINE inline bool Encode(Workspace& w) noexcept
    { return BuildMotionEccentricCCommand(w.input, w.identity, w.owner, w.packet, w.transport); }

    BASE79C_NOINLINE inline void ClearExecutor(Workspace& w) noexcept
    { w.executor.Clear(); }

    BASE79C_NOINLINE inline bool Begin(Workspace& w, const MotionEccentricCContext& context) noexcept
    { return w.executor.Begin(w.decoded, w.wire.execution, w.wire.ownerLease, context, w.point); }

    BASE79C_NOINLINE inline bool Advance(Workspace& w, const MotionEccentricCContext& context) noexcept
    { return w.executor.Advance(context, w.point); }

    BASE79C_NOINLINE inline bool Stop(Workspace& w, const MotionEccentricCContext& context) noexcept
    { return w.executor.RequestStop(context, w.point); }

    BASE79C_NOINLINE inline bool Resume(Workspace& w, const MotionEccentricCContext& context) noexcept
    { return w.executor.Resume(context, w.point); }

    BASE79C_NOINLINE inline bool Finish(Workspace& w, const MotionEccentricCContext& context) noexcept
    { return NCEccentricCExecutorSelfCheckDetail::Finish(w.executor, context, w.point); }

    inline bool SameIdentity(const MotionExecutionIdentity& a, const MotionExecutionIdentity& b) noexcept
    {
        return a.epoch == b.epoch && a.segmentId == b.segmentId &&
            a.sourceBlockId == b.sourceBlockId && a.source == b.source;
    }
}

BASE79C_NOINLINE inline NCEccentricCPipelineSelfCheckResult RunNCEccentricCPipelineSelfCheck() noexcept
{
    using namespace NCEccentricCPipelineSelfCheckDetail;
    using NCEccentricCExecutorSelfCheckDetail::Context;
    using NCEccentricCExecutorSelfCheckDetail::SamePoint;
    using NCEccentricCTransportSelfCheckDetail::Fingerprint;
    using NCEccentricCTransportSelfCheckDetail::RejectedByNativeRoutes;
    using State = NCEccentricCProfileState;
    NCEccentricCPipelineSelfCheckResult result{};
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
    w.context = Context(w.identity, w.owner);

    // Decode the actual copied packet, not a directly prepared test profile.
    for (int mode : {43, 44})
        for (double direction : {1.0, -1.0})
        {
            w.input.geometry.source.toolLengthMode = mode;
            w.input.geometry.sweepDeg = direction * 0.01;
            if (!check(Encode(w))) return result;
            const auto fingerprint = Fingerprint(w.packet);
            w.wire = w.packet;
            if (!check(Fingerprint(w.wire) == fingerprint && RejectedByNativeRoutes(w.wire) && Decode(w))) return result;
            w.expectedEnd = w.decoded.Runtime().EndPulse();
            w.expectedTicks = w.decoded.TotalTicks();
            ClearExecutor(w);
            if (!check(Begin(w, w.context) && SameIdentity(w.executor.Identity(), w.wire.execution) &&
                w.executor.Owner().Matches(w.wire.ownerLease) &&
                SameNCTranslationSnapshot(w.input.geometry.source, w.executor.Runtime().AuthoredPath().source) &&
                NCEccentricCDetail::Same(w.point.runtime.positionPulse, w.input.startPulse))) return result;

            // Late decode failure clears the shared output plan, without
            // changing the already committed executor's owned curve or point.
            w.saved = w.point;
            w.wire.mem_startPos[7] = std::numeric_limits<double>::quiet_NaN();
            if (!check(!Decode(w) && !w.decoded.IsValid() && SamePoint(w.executor.Current(), w.saved))) return result;

            // Reuse all transport storage for a different, valid curve. The
            // old executor must survive both invalidation and successful reuse.
            w.input.geometry.source.toolOffsetMM[0] += 3.0;
            ++w.input.geometry.source.revision;
            if (!check(Encode(w) && Fingerprint(w.packet) != fingerprint)) return result;
            w.wire = w.packet;
            if (!check(Decode(w) && !NCEccentricCDetail::Same(w.decoded.Runtime().EndPulse(), w.expectedEnd) &&
                Advance(w, w.context) && w.point.sequence == 1ULL)) return result;
            w.input.geometry.source.toolOffsetMM[0] -= 3.0;
            --w.input.geometry.source.revision;
            if (!check(Finish(w, w.context) && w.executor.State() == State::AUTHORED_COMPLETE &&
                w.point.sequence == w.expectedTicks &&
                NCEccentricCDetail::Same(w.point.runtime.positionPulse, w.expectedEnd) &&
                SameNCTranslationSnapshot(w.input.geometry.source, w.executor.Runtime().AuthoredPath().source))) return result;
            w.terminal = w.point;
            if (!check(Advance(w, w.context) && SamePoint(w.point, w.terminal) &&
                !Begin(w, w.context) && !w.point.valid && SamePoint(w.executor.Current(), w.terminal))) return result;
        }

    // A zero XY eccentricity still transports and executes the C role alone.
    w.input.geometry.source.toolLengthMode = 43; w.input.geometry.sweepDeg = 0.01;
    w.input.geometry.source.toolOffsetMM[0] = w.input.geometry.source.toolOffsetMM[1] = 0.0;
    if (!check(Encode(w) && w.packet.axisCount == 1 && w.packet.axisIndices[0] == 3)) return result;
    w.wire = w.packet;
    ClearExecutor(w);
    if (!check(Decode(w) && Begin(w, w.context) && Advance(w, w.context) &&
        NCEccentricCDetail::Same(w.point.runtime.positionPulse[0], w.input.startPulse[0]) &&
        NCEccentricCDetail::Same(w.point.runtime.positionPulse[1], w.input.startPulse[1]) &&
        w.point.runtime.velocityPulse[0] == 0.0 && w.point.runtime.velocityPulse[1] == 0.0)) return result;
    if (!check(Finish(w, w.context) && w.executor.State() == State::AUTHORED_COMPLETE &&
        NCEccentricCDetail::Same(w.point.runtime.positionPulse, w.executor.Runtime().EndPulse()))) return result;
    w.wire.mem_startPos[7] = std::numeric_limits<double>::quiet_NaN();
    ClearExecutor(w);
    if (!check(!Decode(w) && !w.decoded.IsValid() && !Begin(w, w.context) && !w.point.valid && !w.executor.IsValid())) return result;
    w.input.geometry.source.toolOffsetMM[0] = 2.0; w.input.geometry.source.toolOffsetMM[1] = 1.0;
    if (!check(Encode(w))) return result;
    w.wire = w.packet;
    if (!check(Decode(w))) return result;

    // A decodable packet is not current execution authority. Begin compares
    // the transported identity/owner against an independently supplied context.
    for (unsigned defect = 0U; defect < 6U; ++defect)
    {
        w.bad = w.context;
        switch (defect)
        {
        case 0U: ++w.bad.currentEpoch; break;
        case 1U: ++w.bad.currentOwner.generation; break;
        case 2U: ++w.bad.execution.segmentId; break;
        case 3U: ++w.bad.sourceOwner.generation; break;
        case 4U: ++w.bad.execution.sourceBlockId; break;
        default: w.bad.fault = true; break;
        }
        if (!check(!Begin(w, w.bad) && !w.point.valid && !w.executor.IsValid())) return result;
    }
    if (!check(Begin(w, w.context) && Advance(w, w.context) && w.point.velocityPulsePerSec > 0.0)) return result;
    w.saved = w.point;
    for (unsigned defect = 0U; defect < 10U; ++defect)
    {
        w.bad = w.context;
        switch (defect)
        {
        case 0U: ++w.bad.currentEpoch; break;
        case 1U: ++w.bad.currentOwner.generation; break;
        case 2U: ++w.bad.execution.segmentId; break;
        case 3U: ++w.bad.execution.sourceBlockId; break;
        case 4U: ++w.bad.sourceOwner.generation; break;
        case 5U: w.bad.execution.source = MotionCommandSource::NC_MDI; break;
        case 6U: w.bad.safetyTicket = 1ULL; break;
        case 7U: w.bad.safetyAuthorized = true; break;
        case 8U: w.bad.currentOwner.owner = MotionOwner::MDI; break;
        default: w.bad.fault = true; break;
        }
        if (!check(!Advance(w, w.bad) && !w.point.valid && SamePoint(w.executor.Current(), w.saved) &&
            !Stop(w, w.bad) && !w.point.valid && SamePoint(w.executor.Current(), w.saved))) return result;
    }

    // Normal controlled stop/resume acts on the decoded owned curve.
    if (!check(Stop(w, w.context) && w.executor.IsStopping() && w.point.sequence == w.saved.sequence &&
        w.point.scalarPulse == w.saved.scalarPulse && w.point.velocityPulsePerSec == w.saved.velocityPulsePerSec)) return result;
    if (!check(Finish(w, w.context) && w.executor.State() == State::STOPPED_BEFORE_END)) return result;
    w.saved = w.point;
    if (!check(Resume(w, w.context) && w.point.sequence == w.saved.sequence &&
        NCEccentricCDetail::Same(w.point.runtime.positionPulse, w.saved.runtime.positionPulse))) return result;
    if (!check(Finish(w, w.context) && w.executor.State() == State::AUTHORED_COMPLETE &&
        NCEccentricCDetail::Same(w.point.runtime.positionPulse, w.executor.Runtime().EndPulse()))) return result;

    // Synthetic safety takeover: binding a request cannot publish a sample
    // until the exact applied epoch/owner/ticket is also confirmed.
    ClearExecutor(w);
    if (!check(Begin(w, w.context) && Advance(w, w.context))) return result;
    w.saved = w.point; w.safety = w.context;
    w.safety.currentEpoch = 8U; w.safety.currentOwner.owner = MotionOwner::SAFETY;
    w.safety.currentOwner.generation = 12U; w.safety.safetyTicket = 29ULL;
    w.safety.safetyRequestAuthorized = true;
    if (!check(Stop(w, w.safety) && w.executor.IsSafetyStop() &&
        w.point.sequence == w.saved.sequence && w.point.scalarPulse == w.saved.scalarPulse &&
        w.point.velocityPulsePerSec == w.saved.velocityPulsePerSec)) return result;
    w.saved = w.point;
    if (!check(!Advance(w, w.safety) && !w.point.valid && SamePoint(w.executor.Current(), w.saved))) return result;
    w.safety.safetyAuthorized = true;
    if (!check(Advance(w, w.safety) && w.point.sequence == w.saved.sequence + 1ULL)) return result;
    w.saved = w.point;
    for (unsigned defect = 0U; defect < 5U; ++defect)
    {
        w.bad = w.safety;
        switch (defect)
        {
        case 0U: ++w.bad.currentEpoch; break;
        case 1U: ++w.bad.currentOwner.generation; break;
        case 2U: ++w.bad.safetyTicket; break;
        case 3U: ++w.bad.sourceOwner.generation; break;
        default: w.bad.fault = true; break;
        }
        if (!check(!Advance(w, w.bad) && !w.point.valid && SamePoint(w.executor.Current(), w.saved))) return result;
    }
    if (!check(!Advance(w, w.context) && !w.point.valid && !Resume(w, w.context) &&
        !w.point.valid && SamePoint(w.executor.Current(), w.saved))) return result;
    if (!check(Finish(w, w.safety) && w.executor.State() == State::STOPPED_BEFORE_END &&
        !Resume(w, w.safety) && !w.point.valid)) return result;

    // New command starts from the previous authored endpoint and uses a new
    // authority tuple. This is a fresh isolated case, not resuming safety motion.
    w.previous = w.context;
    w.input.startPulse = w.decoded.Runtime().EndPulse();
    for (unsigned axis = 0U; axis < 8U; ++axis)
        w.input.geometry.startMCS[axis] = w.input.startPulse[axis] / w.input.pulsePerNative[axis];
    w.input.geometry.sweepDeg = -0.01;
    ++w.identity.epoch; ++w.identity.segmentId; ++w.identity.sourceBlockId; ++w.owner.generation;
    w.context = Context(w.identity, w.owner);
    if (!check(Encode(w))) return result;
    w.wire = w.packet;
    if (!check(Decode(w) && NCEccentricCDetail::Same(w.decoded.Runtime().StartPulse(), w.input.startPulse))) return result;
    ClearExecutor(w);
    if (!check(!Begin(w, w.previous) && !w.point.valid && !w.executor.IsValid() && Begin(w, w.context) &&
        SameIdentity(w.executor.Identity(), w.identity) && w.executor.Owner().Matches(w.owner) &&
        NCEccentricCDetail::Same(w.point.runtime.positionPulse, w.input.startPulse))) return result;
    w.saved = w.point;
    if (!check(!Advance(w, w.previous) && !w.point.valid && SamePoint(w.executor.Current(), w.saved))) return result;
    if (!check(Finish(w, w.context) && w.executor.State() == State::AUTHORED_COMPLETE &&
        NCEccentricCDetail::Same(w.point.runtime.positionPulse, w.decoded.Runtime().EndPulse()) &&
        RejectedByNativeRoutes(w.wire) && !NCEccentricCDynamicMotionAdmission)) return result;
    return result;
}

#undef BASE79C_NOINLINE
