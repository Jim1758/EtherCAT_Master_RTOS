#pragma once

#include "MotionEccentricCBinding.h"
#include "MotionEccentricCExecutor.h"
#include <utility>

// BASE79E: single-consumer, caller-owned staging storage. Allocate this once
// outside a cyclic thread, or embed it in a caller's heap workspace. No method
// allocates. Neither this class, a ticket, nor a successful callback grants
// Motion authority. No live MotionCore caller is connected in this stage.
//
// A future live caller MUST hold its complete output/lifecycle reservation
// across Commit, supply current frozen-source / previous-output / axis / travel
// validation, and publish the whole logical image in the noexcept sink. The
// ordinary epoch-only reservation does not by itself freeze the owner word.
// The sink must be bounded, unable to fail, and must not publish partial output
// to other threads. Existing physical/PBC output remains a separate common pass.
// This is not a thread-safe or hardware-atomic axis output API.
#if defined(_MSC_VER)
#define BASE79E_TX_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define BASE79E_TX_NOINLINE __attribute__((noinline))
#else
#define BASE79E_TX_NOINLINE
#endif

enum class MotionEccentricCOperation : std::uint8_t
{
    NONE = 0U, BEGIN = 1U, ADVANCE = 2U, STOP = 3U, RESUME = 4U
};

class MotionEccentricCTransaction;

// Object-local correlation only, never output permission. Do not retain a
// ticket beyond this transaction object's lifetime. Clear/Discard do not reuse
// serial numbers. A different object, superseded prepare, or replay cannot
// commit a pending candidate. Serial exhaustion fails closed, without wrapping.
class MotionEccentricCTicket
{
public:
    bool IsValid() const noexcept { return transaction_ != nullptr && serial_ != 0ULL; }
private:
    const MotionEccentricCTransaction* transaction_ = nullptr;
    std::uint64_t serial_ = 0ULL;
    friend class MotionEccentricCTransaction;
};

class MotionEccentricCTransaction
{
public:
    MotionEccentricCTransaction() noexcept = default;
    MotionEccentricCTransaction(const MotionEccentricCTransaction&) = delete;
    MotionEccentricCTransaction& operator=(const MotionEccentricCTransaction&) = delete;
    bool IsValid() const noexcept { return committed_.IsValid(); }
    bool HasPending() const noexcept { return pending_; }
    std::uint64_t CommittedSerial() const noexcept { return committedSerial_; }
    // Borrowed views: copy values needed after a mutating call. PendingPoint
    // and callback references must not be retained by the sink or validator.
    const MotionEccentricCExecutor& Committed() const noexcept { return committed_; }
    const MotionCommand& Command() const noexcept { return command_; }
    const NCEccentricCProfilePoint* PendingPoint() const noexcept
    { return pending_ ? &point_ : nullptr; }

    // Clear retires private state only; stopping and retiring live axes would
    // be the Motion caller's responsibility. Callback reentry cannot mutate a
    // candidate or invalidate references while a commit is in progress.
    BASE79E_TX_NOINLINE bool Clear() noexcept
    {
        if (committing_) return false;
        InvalidatePending();
        MotionEccentricCTransportDetail::ResetInPlace(committed_);
        MotionEccentricCTransportDetail::ResetInPlace(candidate_);
        MotionEccentricCTransportDetail::ResetInPlace(command_);
        committedSerial_ = 0ULL;
        return true;
    }
    BASE79E_TX_NOINLINE bool Discard() noexcept
    {
        if (committing_) return false;
        InvalidatePending();
        return true;
    }

    BASE79E_TX_NOINLINE bool PrepareBegin(const MotionCommand& command,
        const MotionEccentricCContext& context, MotionEccentricCTransportWorkspace& scratch,
        NCEccentricCProfileValue& decoded, MotionEccentricCTicket& ticket) noexcept
    {
        if (committing_) return false;
        ticket = MotionEccentricCTicket{};
        InvalidatePending();
        if (IsValid() || serial_ == (std::numeric_limits<std::uint64_t>::max)()) return false;
        // Own this exact packet before decoding; subsequent caller edits or
        // scratch reuse cannot alter an active or pending transaction.
        command_ = command;
        MotionEccentricCTransportDetail::ResetInPlace(candidate_);
        if (!PrepareMotionEccentricCCommand(command_, decoded, scratch) ||
            !IsMotionEccentricCStartSourceBound(command_, decoded.Runtime()) ||
            !candidate_.Begin(decoded, command_.execution, command_.ownerLease, context, point_)) return false;
        return Stage(MotionEccentricCOperation::BEGIN, context, ticket);
    }
    BASE79E_TX_NOINLINE bool PrepareAdvance(const MotionEccentricCContext& context,
        MotionEccentricCTicket& ticket) noexcept
    { return PrepareOperation(MotionEccentricCOperation::ADVANCE, context, ticket); }
    BASE79E_TX_NOINLINE bool PrepareStop(const MotionEccentricCContext& context,
        MotionEccentricCTicket& ticket) noexcept
    { return PrepareOperation(MotionEccentricCOperation::STOP, context, ticket); }
    BASE79E_TX_NOINLINE bool PrepareResume(const MotionEccentricCContext& context,
        MotionEccentricCTicket& ticket) noexcept
    { return PrepareOperation(MotionEccentricCOperation::RESUME, context, ticket); }

    // Context equality rejects authority facts changed since prepare; it does
    // not prove those facts are current. Validator must obtain that proof and
    // check the complete previously emitted image and next image under the
    // caller's reservation. A refusal invalidates this matched pending ticket
    // but leaves the committed cursor and sink untouched. Reprepare for retry.
    // A wrong/stale ticket cannot cancel a different, valid pending operation.
    // A fresh ADVANCE at a terminal state may emit the same point (executor
    // idempotence). Motion must separately publish terminal feedback once;
    // a new transaction serial is not a new completed Motion segment.
    template<class Validator, class Sink>
    BASE79E_TX_NOINLINE bool Commit(const MotionEccentricCTicket& ticket,
        const MotionEccentricCContext& current, Validator&& validate, Sink&& emit) noexcept
    {
        static_assert(noexcept(std::declval<Validator&>()(std::declval<const MotionCommand&>(),
            std::declval<const MotionEccentricCExecutor&>(), std::declval<const MotionEccentricCExecutor&>(),
            MotionEccentricCOperation::NONE)), "Commit validation must be noexcept.");
        static_assert(std::is_same<decltype(std::declval<Validator&>()(std::declval<const MotionCommand&>(),
            std::declval<const MotionEccentricCExecutor&>(), std::declval<const MotionEccentricCExecutor&>(),
            MotionEccentricCOperation::NONE)), bool>::value, "Validator must return bool.");
        static_assert(noexcept(std::declval<Sink&>()(std::declval<const MotionCommand&>(),
            std::declval<const NCEccentricCProfilePoint&>(), MotionEccentricCOperation::NONE)),
            "Commit sink must be noexcept.");
        static_assert(std::is_same<decltype(std::declval<Sink&>()(std::declval<const MotionCommand&>(),
            std::declval<const NCEccentricCProfilePoint&>(), MotionEccentricCOperation::NONE)), void>::value,
            "Commit sink cannot fail after validation.");
        if (committing_ || !pending_ || ticket.transaction_ != this || ticket.serial_ != serial_) return false;
        committing_ = true;
        if (!SameContext(context_, current) || !ValidPoint() ||
            !validate(command_, committed_, candidate_, operation_) ||
            !SameContext(context_, current))
        {
            InvalidatePending(); committing_ = false; return false;
        }
        emit(command_, point_, operation_);
        committed_ = candidate_;
        committedSerial_ = serial_;
        InvalidatePending(); committing_ = false;
        return true;
    }

private:
    static bool SameContext(const MotionEccentricCContext& a,
        const MotionEccentricCContext& b) noexcept
    {
        return a.execution.epoch == b.execution.epoch && a.execution.segmentId == b.execution.segmentId &&
            a.execution.sourceBlockId == b.execution.sourceBlockId && a.execution.source == b.execution.source &&
            a.sourceOwner.owner == b.sourceOwner.owner && a.sourceOwner.generation == b.sourceOwner.generation &&
            a.currentEpoch == b.currentEpoch && a.currentOwner.owner == b.currentOwner.owner &&
            a.currentOwner.generation == b.currentOwner.generation && a.safetyTicket == b.safetyTicket &&
            a.previousSafetyEpoch == b.previousSafetyEpoch &&
            a.previousSafetyOwner.owner == b.previousSafetyOwner.owner &&
            a.previousSafetyOwner.generation == b.previousSafetyOwner.generation &&
            a.previousSafetyTicket == b.previousSafetyTicket &&
            a.safetyRequestAuthorized == b.safetyRequestAuthorized &&
            a.safetyAuthorized == b.safetyAuthorized && a.fault == b.fault;
    }
    void InvalidatePending() noexcept
    { pending_ = false; operation_ = MotionEccentricCOperation::NONE; point_.Clear(); }

    BASE79E_TX_NOINLINE bool ValidPoint() const noexcept
    {
        if (!candidate_.IsValid() || !point_.valid || !point_.runtime.valid ||
            !std::isfinite(point_.scalarPulse) || !std::isfinite(point_.velocityPulsePerSec) ||
            !std::isfinite(point_.accelerationPulsePerSec2)) return false;
        const auto& runtime = candidate_.Runtime();
        if (point_.scalarPulse < 0.0 || point_.scalarPulse > runtime.MaximumScalarPulse() ||
            point_.velocityPulsePerSec < 0.0 ||
            point_.velocityPulsePerSec > runtime.MaximumScalarVelocityPulsePerSec()) return false;
        for (unsigned axis = 0U; axis < 8U; ++axis)
        {
            const double position = point_.runtime.positionPulse[axis];
            if (!std::isfinite(position) || !std::isfinite(point_.runtime.velocityPulse[axis]) ||
                position < runtime.MinimumPulse()[axis] || position > runtime.MaximumPulse()[axis]) return false;
            if ((runtime.AuthoredPath().groupMask & (1U << axis)) == 0U &&
                (!NCEccentricCDetail::Same(position, runtime.StartPulse()[axis]) ||
                    point_.runtime.velocityPulse[axis] != 0.0)) return false;
        }
        return true;
    }
    BASE79E_TX_NOINLINE bool Stage(MotionEccentricCOperation operation,
        const MotionEccentricCContext& context, MotionEccentricCTicket& ticket) noexcept
    {
        if (!ValidPoint()) { InvalidatePending(); return false; }
        context_ = context; operation_ = operation; pending_ = true; ++serial_;
        ticket.transaction_ = this; ticket.serial_ = serial_;
        return true;
    }
    BASE79E_TX_NOINLINE bool PrepareOperation(MotionEccentricCOperation operation,
        const MotionEccentricCContext& context, MotionEccentricCTicket& ticket) noexcept
    {
        if (committing_) return false;
        ticket = MotionEccentricCTicket{};
        InvalidatePending();
        if (!IsValid() || serial_ == (std::numeric_limits<std::uint64_t>::max)()) return false;
        candidate_ = committed_;
        const bool prepared = operation == MotionEccentricCOperation::ADVANCE ? candidate_.Advance(context, point_) :
            operation == MotionEccentricCOperation::STOP ? candidate_.RequestStop(context, point_) :
            operation == MotionEccentricCOperation::RESUME ? candidate_.Resume(context, point_) : false;
        if (!prepared) return false;
        return Stage(operation, context, ticket);
    }

    MotionEccentricCExecutor committed_{}, candidate_{};
    MotionCommand command_{};
    MotionEccentricCContext context_{};
    NCEccentricCProfilePoint point_{};
    std::uint64_t serial_ = 0ULL, committedSerial_ = 0ULL;
    MotionEccentricCOperation operation_ = MotionEccentricCOperation::NONE;
    bool pending_ = false, committing_ = false;
};

static_assert(sizeof(MotionEccentricCTransaction) <= 12288U,
    "Transaction must remain bounded caller-owned storage, never an embedded Motion group member.");

#undef BASE79E_TX_NOINLINE
