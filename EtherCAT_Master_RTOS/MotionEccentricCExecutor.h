#pragma once

#include "MotionExecutionContract.h"
#include "NCEccentricCProfile.h"

// BASE79: RT-consumer facts, not a caller-created Motion permission. The sole
// production caller must obtain these from the existing owner/epoch/safety
// publication and recheck before committing all axis outputs. In particular,
// safetyAuthorized is the result of IsSafetyControlledStopAuthorized for the
// complete generated group; it cannot grant ownership by itself.
struct MotionEccentricCContext
{
    MotionExecutionIdentity execution{};
    MotionOwnerLease sourceOwner{};
    MotionExecutionEpoch currentEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    MotionOwnerLease currentOwner{};
    std::uint64_t safetyTicket = 0ULL;
    // A verified replacement request names the exact previous stop token.
    // Tickets/epochs wrap, so numeric greater-than is not an ordering proof.
    MotionExecutionEpoch previousSafetyEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    MotionOwnerLease previousSafetyOwner{};
    std::uint64_t previousSafetyTicket = 0ULL;
    bool safetyRequestAuthorized = false; // exact published request, before epoch application
    bool safetyAuthorized = false; // exact applied epoch/owner/ticket, required for axis samples
    bool fault = false;
};

// One owned copy, no external plan pointer, no mutable position/velocity, and
// no legacy FIR/history. Candidate operations commit the cursor and point only
// after the complete bounded sample and exact authority tuple pass. A failed
// operation clears its output and preserves the last committed cursor.
class MotionEccentricCExecutor
{
public:
    void Clear() noexcept { *this = MotionEccentricCExecutor{}; }
    bool IsValid() const noexcept { return cursor_.IsValid(); }
    bool IsSafetyStop() const noexcept { return safetyStop_; }
    bool IsStopping() const noexcept
    { return State() == NCEccentricCProfileState::CONTROLLED_STOP_ACTIVE; }
    NCEccentricCProfileState State() const noexcept { return cursor_.State(); }
    const NCEccentricCRuntimeValue& Runtime() const noexcept { return cursor_.Runtime(); }
    const MotionExecutionIdentity& Identity() const noexcept { return identity_; }
    const MotionOwnerLease& Owner() const noexcept { return owner_; }
    const NCEccentricCProfilePoint& Current() const noexcept { return point_; }

    bool Begin(const NCEccentricCProfileValue& plan, const MotionExecutionIdentity& identity,
        const MotionOwnerLease& owner, const MotionEccentricCContext& context,
        NCEccentricCProfilePoint& output) noexcept
    {
        output.Clear();
        if (IsValid() || !plan.IsValid() || !identity.IsAssigned() ||
            identity.source != MotionCommandSource::NC_MEMORY || identity.sourceBlockId < 0 ||
            !owner.IsValid() || owner.owner != MotionOwner::AUTO) return false;
        MotionEccentricCExecutor candidate{};
        candidate.identity_ = identity; candidate.owner_ = owner;
        if (!candidate.NormalContext(context) ||
            !BeginNCEccentricCProfile(plan, candidate.cursor_, candidate.point_)) return false;
        *this = candidate; output = point_; return true;
    }

    bool Advance(const MotionEccentricCContext& context, NCEccentricCProfilePoint& output) noexcept
    {
        output.Clear();
        if (!IsValid() || !(safetyStop_ ? SafetyContext(context, true) : NormalContext(context)))
            return false;
        MotionEccentricCExecutor candidate = *this;
        if (!AdvanceNCEccentricCProfile(candidate.cursor_, candidate.point_)) return false;
        *this = candidate; output = point_; return true;
    }

    bool RequestStop(const MotionEccentricCContext& context, NCEccentricCProfilePoint& output) noexcept
    {
        output.Clear();
        if (!IsValid()) return false;
        const bool safety = context.safetyRequestAuthorized;
        if (safety ? !SafetyRequestContext(context) : (safetyStop_ || !NormalContext(context)))
            return false;
        MotionEccentricCExecutor candidate = *this;
        // A safety takeover may race the authored endpoint's drain. Bind the
        // same zero-velocity endpoint without manufacturing a second movement.
        if (State() == NCEccentricCProfileState::AUTHORED_COMPLETE)
        {
            if (!safety || !point_.valid || point_.velocityPulsePerSec != 0.0 ||
                point_.scalarPulse != Runtime().AuthoredScalarPulse()) return false;
        }
        else if (!RequestNCEccentricCProfileStop(candidate.cursor_, candidate.point_)) return false;
        if (safety)
        {
            candidate.safetyStop_ = true;
            candidate.safetyEpoch_ = context.currentEpoch;
            candidate.safetyOwner_ = context.currentOwner;
            candidate.safetyTicket_ = context.safetyTicket;
        }
        *this = candidate; output = point_; return true;
    }

    bool Resume(const MotionEccentricCContext& context, NCEccentricCProfilePoint& output) noexcept
    {
        output.Clear();
        if (!IsValid() || safetyStop_ || !NormalContext(context)) return false;
        MotionEccentricCExecutor candidate = *this;
        if (!ResumeNCEccentricCProfile(candidate.cursor_, candidate.point_)) return false;
        *this = candidate; output = point_; return true;
    }

private:
    bool IdentityContext(const MotionEccentricCContext& c) const noexcept
    {
        return !c.fault && c.execution.epoch == identity_.epoch &&
            c.execution.segmentId == identity_.segmentId &&
            c.execution.sourceBlockId == identity_.sourceBlockId &&
            c.execution.source == identity_.source && c.sourceOwner.Matches(owner_);
    }
    bool NormalContext(const MotionEccentricCContext& c) const noexcept
    {
        return IdentityContext(c) && !c.safetyAuthorized && !c.safetyRequestAuthorized && c.safetyTicket == 0ULL &&
            c.previousSafetyEpoch == MOTION_EXECUTION_EPOCH_INVALID &&
            c.previousSafetyOwner.owner == MotionOwner::NONE &&
            c.previousSafetyOwner.generation == MOTION_OWNER_GENERATION_INVALID &&
            c.previousSafetyTicket == 0ULL &&
            c.currentEpoch == identity_.epoch && c.currentOwner.Matches(owner_);
    }
    bool SafetyRequestContext(const MotionEccentricCContext& c) const noexcept
    {
        if (!SafetyContext(c, false, false)) return false;
        if (!safetyStop_ || (c.currentEpoch == safetyEpoch_ &&
            c.currentOwner.Matches(safetyOwner_) && c.safetyTicket == safetyTicket_)) return true;
        // The RT consumer captures this predecessor before replacing its
        // published stop token. The complete old tuple prevents a previously
        // captured request from rolling a newer executor binding backward.
        return c.previousSafetyEpoch == safetyEpoch_ &&
            c.previousSafetyOwner.Matches(safetyOwner_) && c.previousSafetyTicket == safetyTicket_;
    }
    bool SafetyContext(const MotionEccentricCContext& c, bool frozen, bool applied = true) const noexcept
    {
        return IdentityContext(c) && (applied ? c.safetyAuthorized : c.safetyRequestAuthorized) &&
            c.safetyTicket != 0ULL &&
            c.currentEpoch != MOTION_EXECUTION_EPOCH_INVALID && c.currentOwner.IsValid() &&
            c.currentOwner.owner == MotionOwner::SAFETY && (!frozen ||
                (c.currentEpoch == safetyEpoch_ && c.currentOwner.Matches(safetyOwner_) &&
                    c.safetyTicket == safetyTicket_));
    }
    NCEccentricCProfileCursor cursor_{};
    NCEccentricCProfilePoint point_{};
    MotionExecutionIdentity identity_{};
    MotionOwnerLease owner_{}, safetyOwner_{};
    MotionExecutionEpoch safetyEpoch_ = MOTION_EXECUTION_EPOCH_INVALID;
    std::uint64_t safetyTicket_ = 0ULL;
    bool safetyStop_ = false;
};

static_assert(std::is_trivially_copyable<MotionEccentricCExecutor>::value &&
    sizeof(MotionEccentricCExecutor) <= 4608U,
    "BASE79 executor must remain an owned bounded value.");
