#pragma once

#include "MotionEccentricCTransaction.h"
#include "NCEccentricCExecutorSelfCheck.h"
#include <memory>
#include <new>

// BASE79E: synthetic output images only. This exercises private staging and
// callback ordering, not live Motion authority, hardware output or RT timing.
struct NCEccentricCTransactionSelfCheckResult
{
    std::uint32_t checks = 0U;
    std::uint32_t failedCheck = 0U;
    bool passed = true;
};

#if defined(_MSC_VER)
#define BASE79E_CHECK_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define BASE79E_CHECK_NOINLINE __attribute__((noinline))
#else
#define BASE79E_CHECK_NOINLINE
#endif

namespace NCEccentricCTransactionSelfCheckDetail
{
    using Operation = MotionEccentricCOperation;
    using State = NCEccentricCProfileState;
    using NCEccentricCExecutorSelfCheckDetail::SamePoint;

    // Exactly one private allocation per invocation. All large objects,
    // transport scratch and output copies stay off startup / NC LOAD stacks.
    struct Workspace
    {
        MotionEccentricCTransportWorkspace transport{};
        NCEccentricCRuntimeInput input = NCEccentricCProfileSelfCheckDetail::Input();
        MotionCommand packet{}, expected{};
        NCEccentricCProfileValue decoded{};
        MotionEccentricCTransaction transaction{}, other{};
        NCEccentricCProfilePoint image{}, saved{}, staged{}, terminal{};
        MotionEccentricCContext context{}, bad{}, safety{}, successor{};
        MotionExecutionIdentity identity{};
        MotionOwnerLease owner{};
        MotionEccentricCTicket ticket{}, stale{}, foreign{};
        std::uint64_t emitted = 0ULL, validated = 0ULL, reentryEmitted = 0ULL;
        Operation lastOperation = Operation::NONE;
        bool allow = true, reenterValidator = false, reenterSink = false;
        bool reentryPassed = true;
        unsigned lateContextChange = 0U;
    };

    BASE79E_CHECK_NOINLINE inline bool Encode(Workspace& w) noexcept
    {
        if (!BuildMotionEccentricCCommand(w.input, w.identity, w.owner, w.packet, w.transport)) return false;
        w.expected = w.packet;
        return true;
    }

    BASE79E_CHECK_NOINLINE inline bool Reset(Workspace& w) noexcept
    {
        if (!w.transaction.Clear() || !w.other.Clear()) return false;
        w.image.Clear(); w.allow = true; w.reenterValidator = w.reenterSink = false;
        w.reentryPassed = true; w.lastOperation = Operation::NONE;
        return Encode(w);
    }

    BASE79E_CHECK_NOINLINE inline bool Prepare(Workspace& w, Operation operation,
        const MotionEccentricCContext& context) noexcept
    {
        switch (operation)
        {
        case Operation::BEGIN: return w.transaction.PrepareBegin(w.packet, context, w.transport, w.decoded, w.ticket);
        case Operation::ADVANCE: return w.transaction.PrepareAdvance(context, w.ticket);
        case Operation::STOP: return w.transaction.PrepareStop(context, w.ticket);
        case Operation::RESUME: return w.transaction.PrepareResume(context, w.ticket);
        default: return false;
        }
    }

    // Every mutating entry must refuse callback reentry before touching the
    // original candidate, ticket or image. The nested callbacks must not run.
    BASE79E_CHECK_NOINLINE inline bool Reentry(Workspace& w) noexcept
    {
        const auto serial = w.transaction.CommittedSerial();
        const auto* pending = w.transaction.PendingPoint();
        const bool clear = w.transaction.Clear();
        const bool discard = w.transaction.Discard();
        const bool begin = w.transaction.PrepareBegin(w.packet, w.context, w.transport, w.decoded, w.ticket);
        const bool advance = w.transaction.PrepareAdvance(w.context, w.ticket);
        const bool stop = w.transaction.PrepareStop(w.context, w.ticket);
        const bool resume = w.transaction.PrepareResume(w.context, w.ticket);
        const bool commit = w.transaction.Commit(w.ticket, w.context,
            [](const MotionCommand&, const MotionEccentricCExecutor&,
                const MotionEccentricCExecutor&, Operation) noexcept -> bool { return true; },
            [&](const MotionCommand&, const NCEccentricCProfilePoint&, Operation) noexcept { ++w.reentryEmitted; });
        return !clear && !discard && !begin && !advance && !stop && !resume && !commit && w.reentryEmitted == 0ULL &&
            w.transaction.HasPending() && w.transaction.PendingPoint() == pending &&
            w.transaction.CommittedSerial() == serial;
    }

    BASE79E_CHECK_NOINLINE inline bool Validate(Workspace& w, const MotionCommand& command,
        const MotionEccentricCExecutor& previous, const MotionEccentricCExecutor& next,
        Operation operation) noexcept
    {
        ++w.validated;
        if (w.reenterValidator) w.reentryPassed = Reentry(w) && w.reentryPassed;
        if (!w.allow || !next.IsValid() || !SameNCTranslationSnapshot(command.sourceTranslation,
            w.expected.sourceTranslation) || !SameNCTranslationSnapshot(command.sourceTranslation,
            next.Runtime().AuthoredPath().source)) return false;
        if (operation == Operation::BEGIN)
        {
            if (previous.IsValid() || w.image.valid) return false;
            for (unsigned axis = 0U; axis < 8U; ++axis)
                if (!NCEccentricCDetail::Same(next.Current().runtime.positionPulse[axis],
                    w.expected.mem_startPos[axis])) return false;
        }
        else if (!previous.IsValid() || !w.image.valid || !SamePoint(previous.Current(), w.image)) return false;
        if (w.lateContextChange == 1U) ++w.context.currentEpoch;
        if (w.lateContextChange == 2U) ++w.context.safetyTicket;
        return w.transaction.PendingPoint() && SamePoint(next.Current(), *w.transaction.PendingPoint());
    }

    BASE79E_CHECK_NOINLINE inline bool Commit(Workspace& w,
        const MotionEccentricCTicket& ticket, const MotionEccentricCContext& context) noexcept
    {
        return w.transaction.Commit(ticket, context,
            [&](const MotionCommand& command, const MotionEccentricCExecutor& previous,
                const MotionEccentricCExecutor& next, Operation operation) noexcept -> bool
            { return Validate(w, command, previous, next, operation); },
            [&](const MotionCommand&, const NCEccentricCProfilePoint& point, Operation operation) noexcept
            {
                if (w.reenterSink) w.reentryPassed = Reentry(w) && w.reentryPassed;
                w.image = point; w.lastOperation = operation; ++w.emitted;
            });
    }

    BASE79E_CHECK_NOINLINE inline bool Step(Workspace& w, Operation operation,
        const MotionEccentricCContext& context) noexcept
    {
        return Prepare(w, operation, context) && Commit(w, w.ticket, context) &&
            SamePoint(w.image, w.transaction.Committed().Current()) &&
            !w.transaction.HasPending() && w.lastOperation == operation;
    }

    BASE79E_CHECK_NOINLINE inline bool Finish(Workspace& w,
        const MotionEccentricCContext& context) noexcept
    {
        for (unsigned tick = 0U; tick < 2048U; ++tick)
        {
            const double position = w.image.scalarPulse, velocity = w.image.velocityPulsePerSec;
            const auto sequence = w.image.sequence;
            const bool stopping = w.transaction.Committed().State() == State::CONTROLLED_STOP_ACTIVE;
            if (!Step(w, Operation::ADVANCE, context) || w.image.scalarPulse < position ||
                w.image.sequence != sequence + 1ULL || w.image.velocityPulsePerSec < 0.0) return false;
            const auto& runtime = w.transaction.Committed().Runtime();
            const double change = w.image.velocityPulsePerSec - velocity;
            const double ceiling = change >= 0.0 ? runtime.MaximumScalarAccelerationPulsePerSec2() :
                stopping ? runtime.MaximumScalarStopDecelerationPulsePerSec2() :
                runtime.MaximumScalarDecelerationPulsePerSec2();
            if (std::fabs(change) / runtime.CycleSeconds() > ceiling) return false;
            for (unsigned axis = 0U; axis < 8U; ++axis)
                if ((runtime.AuthoredPath().groupMask & (1U << axis)) == 0U &&
                    (!NCEccentricCDetail::Same(w.image.runtime.positionPulse[axis], runtime.StartPulse()[axis]) ||
                        w.image.runtime.velocityPulse[axis] != 0.0)) return false;
            const auto state = w.transaction.Committed().State();
            if (state != State::AUTHORED_ACTIVE && state != State::CONTROLLED_STOP_ACTIVE)
                return w.image.velocityPulsePerSec == 0.0 && w.image.accelerationPulsePerSec2 == 0.0;
        }
        return false;
    }

    BASE79E_CHECK_NOINLINE inline void ChangeContext(Workspace& w, unsigned field) noexcept
    {
        w.bad = w.context;
        switch (field)
        {
        case 0U: ++w.bad.execution.epoch; break;
        case 1U: ++w.bad.execution.segmentId; break;
        case 2U: ++w.bad.execution.sourceBlockId; break;
        case 3U: w.bad.execution.source = MotionCommandSource::NC_MDI; break;
        case 4U: w.bad.sourceOwner.owner = MotionOwner::MDI; break;
        case 5U: ++w.bad.sourceOwner.generation; break;
        case 6U: ++w.bad.currentEpoch; break;
        case 7U: w.bad.currentOwner.owner = MotionOwner::MDI; break;
        case 8U: ++w.bad.currentOwner.generation; break;
        case 9U: ++w.bad.safetyTicket; break;
        case 10U: ++w.bad.previousSafetyEpoch; break;
        case 11U: w.bad.previousSafetyOwner.owner = MotionOwner::SAFETY; break;
        case 12U: ++w.bad.previousSafetyOwner.generation; break;
        case 13U: ++w.bad.previousSafetyTicket; break;
        case 14U: w.bad.safetyRequestAuthorized = true; break;
        case 15U: w.bad.safetyAuthorized = true; break;
        default: w.bad.fault = true; break;
        }
    }
}

BASE79E_CHECK_NOINLINE inline NCEccentricCTransactionSelfCheckResult RunNCEccentricCTransactionSelfCheck() noexcept
{
    using namespace NCEccentricCTransactionSelfCheckDetail;
    NCEccentricCTransactionSelfCheckResult result{};
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
    w.context = NCEccentricCExecutorSelfCheckDetail::Context(w.identity, w.owner);
    w.input.geometry.source.distanceMode = 91;
    if (!check(!Prepare(w, Operation::ADVANCE, w.context) && !w.ticket.IsValid() &&
        !Prepare(w, Operation::STOP, w.context) && !Prepare(w, Operation::RESUME, w.context))) return result;

    // Four complete curves: Begin remains staged until the sink accepts; all
    // later caller packet/decoded/scratch edits leave the owned curve intact.
    for (int mode : {43, 44}) for (double direction : {1.0, -1.0})
    {
        w.input.geometry.source.toolLengthMode = mode; w.input.geometry.sweepDeg = direction * 0.01;
        if (!check(Reset(w))) return result;
        const auto emitted = w.emitted;
        if (!check(Prepare(w, Operation::BEGIN, w.context) && w.ticket.IsValid() &&
            w.transaction.HasPending() && !w.transaction.IsValid() && !w.image.valid && w.emitted == emitted)) return result;
        w.staged = *w.transaction.PendingPoint(); w.allow = false;
        if (!check(!Commit(w, w.ticket, w.context) && !w.transaction.HasPending() &&
            !w.transaction.IsValid() && !w.image.valid && w.emitted == emitted)) return result;
        w.allow = true;
        if (!check(Prepare(w, Operation::BEGIN, w.context) && SamePoint(w.staged, *w.transaction.PendingPoint()))) return result;
        const auto ticks = w.decoded.TotalTicks();
        w.packet.mem_startPos[7] = (std::numeric_limits<double>::quiet_NaN)();
        ++w.packet.sourceTranslation.revision; w.decoded.Clear();
        MotionEccentricCTransportDetail::ResetInPlace(w.transport);
        if (!check(Commit(w, w.ticket, w.context) && w.transaction.IsValid() &&
            w.emitted == emitted + 1ULL && SamePoint(w.image, w.staged) &&
            SamePoint(w.image, w.transaction.Committed().Current()))) return result;
        if (!check(Finish(w, w.context) && w.transaction.Committed().State() == State::AUTHORED_COMPLETE &&
            w.image.sequence == ticks && NCEccentricCDetail::Same(w.image.runtime.positionPulse,
                w.transaction.Committed().Runtime().EndPulse()) &&
            NCEccentricCProfileSelfCheckDetail::Close(w.image.runtime.positionPulse[3], direction * 10.0))) return result;
        w.terminal = w.image;
        if (!check(Step(w, Operation::ADVANCE, w.context) && SamePoint(w.image, w.terminal) &&
            !Prepare(w, Operation::RESUME, w.context) && !Prepare(w, Operation::STOP, w.context))) return result;
    }

    w.input.geometry.source.toolLengthMode = 43; w.input.geometry.sweepDeg = 0.01;
    if (!check(Reset(w) && Step(w, Operation::BEGIN, w.context))) return result;
    w.saved = w.image;
    if (!check(Prepare(w, Operation::ADVANCE, w.context))) return result;
    w.staged = *w.transaction.PendingPoint(); w.stale = w.ticket;
    if (!check(w.transaction.Discard() && !w.transaction.HasPending() &&
        SamePoint(w.transaction.Committed().Current(), w.saved) && SamePoint(w.image, w.saved))) return result;
    if (!check(Prepare(w, Operation::ADVANCE, w.context) && SamePoint(*w.transaction.PendingPoint(), w.staged))) return result;
    const auto firstEmitted = w.emitted;
    if (!check(!Commit(w, w.stale, w.context) && w.transaction.HasPending() &&
        w.emitted == firstEmitted && SamePoint(*w.transaction.PendingPoint(), w.staged))) return result;
    if (!check(w.other.PrepareBegin(w.packet, w.context, w.transport, w.decoded, w.foreign) &&
        !Commit(w, w.foreign, w.context) && w.transaction.HasPending() && w.emitted == firstEmitted)) return result;
    if (!check(Commit(w, w.ticket, w.context) && SamePoint(w.image, w.staged) &&
        w.image.sequence == w.saved.sequence + 1ULL && w.emitted == firstEmitted + 1ULL)) return result;
    w.stale = w.ticket;
    if (!check(!Commit(w, w.stale, w.context) && w.emitted == firstEmitted + 1ULL)) return result;
    if (!check(Prepare(w, Operation::ADVANCE, w.context) && !Commit(w, w.stale, w.context) &&
        w.transaction.HasPending() && w.transaction.Discard())) return result;
    if (!check(Prepare(w, Operation::ADVANCE, w.context))) return result;
    w.stale = w.ticket; w.bad = w.context; w.bad.fault = true;
    if (!check(!Prepare(w, Operation::ADVANCE, w.bad) && !w.ticket.IsValid() &&
        !w.transaction.HasPending() && !Commit(w, w.stale, w.context))) return result;
    if (!check(Prepare(w, Operation::ADVANCE, w.context) && !Commit(w, w.stale, w.context) &&
        w.transaction.HasPending() && w.transaction.Discard())) return result;

    // Prepare/commit compare every context field, including currently empty
    // predecessor facts. Refusal must happen before validator or sink callbacks.
    w.saved = w.image;
    for (unsigned field = 0U; field < 17U; ++field)
    {
        const auto emitted = w.emitted, validated = w.validated, serial = w.transaction.CommittedSerial();
        if (!check(Prepare(w, Operation::ADVANCE, w.context))) return result;
        w.staged = *w.transaction.PendingPoint(); ChangeContext(w, field);
        if (!check(!Commit(w, w.ticket, w.bad) && !w.transaction.HasPending() &&
            w.emitted == emitted && w.validated == validated && w.transaction.CommittedSerial() == serial &&
            SamePoint(w.image, w.saved) && SamePoint(w.transaction.Committed().Current(), w.saved))) return result;
        if (!check(Prepare(w, Operation::ADVANCE, w.context) &&
            SamePoint(*w.transaction.PendingPoint(), w.staged) && w.transaction.Discard())) return result;
    }

    // A callback may refresh the same context object passed into Commit. The
    // second context check must refuse those late changes before publication.
    for (unsigned field = 1U; field <= 2U; ++field)
    {
        const auto emitted = w.emitted, validated = w.validated, serial = w.transaction.CommittedSerial();
        w.lateContextChange = field;
        if (!check(Prepare(w, Operation::ADVANCE, w.context) && !Commit(w, w.ticket, w.context) &&
            !w.transaction.HasPending() && w.emitted == emitted && w.validated == validated + 1ULL &&
            w.transaction.CommittedSerial() == serial && SamePoint(w.image, w.saved) &&
            SamePoint(w.transaction.Committed().Current(), w.saved))) return result;
        w.lateContextChange = 0U;
        w.context = NCEccentricCExecutorSelfCheckDetail::Context(w.identity, w.owner);
    }

    // Validator refusals cover changed frozen source and all eight previous
    // position/velocity entries, including stationary and nonexistent axes.
    for (unsigned defect = 0U; defect < 20U; ++defect)
    {
        w.image = w.saved;
        if (defect < 8U) w.image.runtime.positionPulse[defect] += 1.0;
        else if (defect < 16U) w.image.runtime.velocityPulse[defect - 8U] += 1.0;
        else if (defect == 16U) w.image.scalarPulse += 1.0;
        else if (defect == 17U) ++w.image.sequence;
        else if (defect == 18U) ++w.expected.sourceTranslation.revision;
        else w.allow = false;
        w.terminal = w.image;
        const auto emitted = w.emitted, serial = w.transaction.CommittedSerial();
        if (!check(Prepare(w, Operation::ADVANCE, w.context) && !Commit(w, w.ticket, w.context) &&
            !w.transaction.HasPending() && w.emitted == emitted && w.transaction.CommittedSerial() == serial &&
            SamePoint(w.image, w.terminal) && SamePoint(w.transaction.Committed().Current(), w.saved))) return result;
        w.expected = w.packet; w.allow = true;
    }
    w.image = w.saved;

    // Neither a refused stop nor a refused resume changes time, state or image.
    w.allow = false;
    if (!check(Prepare(w, Operation::STOP, w.context) && !Commit(w, w.ticket, w.context) &&
        w.transaction.Committed().State() == State::AUTHORED_ACTIVE && SamePoint(w.image, w.saved))) return result;
    w.allow = true;
    if (!check(Step(w, Operation::STOP, w.context) && w.transaction.Committed().IsStopping() &&
        w.image.sequence == w.saved.sequence && w.image.scalarPulse == w.saved.scalarPulse &&
        w.image.velocityPulsePerSec == w.saved.velocityPulsePerSec)) return result;
    if (!check(Finish(w, w.context) && w.transaction.Committed().State() == State::STOPPED_BEFORE_END)) return result;
    w.saved = w.image; w.allow = false;
    if (!check(Prepare(w, Operation::RESUME, w.context) && !Commit(w, w.ticket, w.context) &&
        w.transaction.Committed().State() == State::STOPPED_BEFORE_END && SamePoint(w.image, w.saved))) return result;
    w.allow = true;
    if (!check(Step(w, Operation::RESUME, w.context) && w.image.sequence == w.saved.sequence &&
        w.image.scalarPulse == w.saved.scalarPulse && w.image.velocityPulsePerSec == 0.0 &&
        NCEccentricCDetail::Same(w.image.runtime.positionPulse, w.saved.runtime.positionPulse))) return result;
    if (!check(Finish(w, w.context) && w.transaction.Committed().State() == State::AUTHORED_COMPLETE)) return result;

    // Synthetic safety request and applied authority remain separate. Changing
    // the applied flag after staging requires reprepare, even for the same token.
    if (!check(Reset(w) && Step(w, Operation::BEGIN, w.context) && Step(w, Operation::ADVANCE, w.context))) return result;
    w.saved = w.image; w.safety = w.context;
    w.safety.currentEpoch = 8U; w.safety.currentOwner.owner = MotionOwner::SAFETY;
    w.safety.currentOwner.generation = 12U; w.safety.safetyTicket = 29ULL;
    w.safety.safetyRequestAuthorized = true;
    if (!check(Prepare(w, Operation::STOP, w.safety))) return result;
    w.bad = w.safety; w.bad.safetyAuthorized = true;
    if (!check(!Commit(w, w.ticket, w.bad) && !w.transaction.Committed().IsSafetyStop() && SamePoint(w.image, w.saved))) return result;
    if (!check(Step(w, Operation::STOP, w.safety) && w.transaction.Committed().IsSafetyStop() &&
        w.image.sequence == w.saved.sequence && w.image.scalarPulse == w.saved.scalarPulse &&
        w.image.velocityPulsePerSec == w.saved.velocityPulsePerSec)) return result;
    w.saved = w.image;
    if (!check(!Prepare(w, Operation::ADVANCE, w.safety) && SamePoint(w.image, w.saved))) return result;
    w.safety.safetyAuthorized = true;
    for (unsigned defect = 0U; defect < 3U; ++defect)
    {
        w.bad = w.safety;
        if (defect == 0U) ++w.bad.currentEpoch;
        else if (defect == 1U) ++w.bad.currentOwner.generation;
        else ++w.bad.safetyTicket;
        if (!check(!Prepare(w, Operation::ADVANCE, w.bad) && SamePoint(w.image, w.saved))) return result;
    }
    if (!check(Step(w, Operation::ADVANCE, w.safety) && w.image.sequence == w.saved.sequence + 1ULL)) return result;
    w.saved = w.image; w.successor = w.safety;
    w.successor.currentEpoch = 9U; w.successor.currentOwner.generation = 13U;
    w.successor.safetyTicket = 30ULL; w.successor.safetyAuthorized = false;
    if (!check(!Prepare(w, Operation::STOP, w.successor) && SamePoint(w.image, w.saved))) return result;
    w.successor.previousSafetyEpoch = w.safety.currentEpoch;
    w.successor.previousSafetyOwner = w.safety.currentOwner;
    w.successor.previousSafetyTicket = w.safety.safetyTicket;
    w.allow = false;
    if (!check(Prepare(w, Operation::STOP, w.successor) && !Commit(w, w.ticket, w.successor) &&
        SamePoint(w.image, w.saved) && SamePoint(w.transaction.Committed().Current(), w.saved))) return result;
    w.allow = true;
    if (!check(Step(w, Operation::ADVANCE, w.safety) && w.image.sequence == w.saved.sequence + 1ULL)) return result;
    w.saved = w.image;
    if (!check(Step(w, Operation::STOP, w.successor) && w.image.sequence == w.saved.sequence &&
        w.image.scalarPulse == w.saved.scalarPulse && w.image.velocityPulsePerSec == w.saved.velocityPulsePerSec)) return result;
    w.saved = w.image;
    if (!check(!Prepare(w, Operation::ADVANCE, w.safety) && !Prepare(w, Operation::ADVANCE, w.successor) &&
        !Prepare(w, Operation::RESUME, w.context) && SamePoint(w.image, w.saved))) return result;
    w.successor.safetyAuthorized = true;
    if (!check(Finish(w, w.successor) && w.transaction.Committed().State() == State::STOPPED_BEFORE_END &&
        !Prepare(w, Operation::RESUME, w.successor) && !Prepare(w, Operation::RESUME, w.context))) return result;

    // Retired pending tickets cannot affect a fresh begin, and all four
    // mutation routes reject reentry from both validator and sink callbacks.
    if (!check(Reset(w) && Prepare(w, Operation::BEGIN, w.context))) return result;
    w.stale = w.ticket;
    if (!check(w.transaction.Clear() && !w.transaction.IsValid() && !w.transaction.HasPending() &&
        !Commit(w, w.stale, w.context) && Prepare(w, Operation::BEGIN, w.context))) return result;
    if (!check(!Commit(w, w.stale, w.context) && w.transaction.HasPending())) return result;
    w.reenterValidator = w.reenterSink = true;
    if (!check(Commit(w, w.ticket, w.context) && w.reentryPassed && w.reentryEmitted == 0ULL &&
        SamePoint(w.image, w.transaction.Committed().Current()))) return result;
    if (!check(Step(w, Operation::ADVANCE, w.context) && w.reentryPassed && w.reentryEmitted == 0ULL)) return result;
    w.reenterValidator = w.reenterSink = false;

    // Zero eccentricity still advances the C role while preserving both XY
    // positions and zero XY velocity in every emitted image.
    w.input.geometry.source.toolOffsetMM[0] = w.input.geometry.source.toolOffsetMM[1] = 0.0;
    if (!check(Reset(w) && w.packet.axisCount == 1 && w.packet.axisIndices[0] == 3 &&
        Step(w, Operation::BEGIN, w.context) && Finish(w, w.context) &&
        w.transaction.Committed().State() == State::AUTHORED_COMPLETE &&
        NCEccentricCDetail::Same(w.image.runtime.positionPulse, w.transaction.Committed().Runtime().EndPulse()))) return result;
    if (!check(w.transaction.Clear() && !w.transaction.IsValid() && !w.transaction.HasPending() &&
        !NCEccentricCDynamicMotionAdmission)) return result;
    return result;
}

#undef BASE79E_CHECK_NOINLINE
