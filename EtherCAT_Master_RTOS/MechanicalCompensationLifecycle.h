#pragma once
// PBC-3B: bounded, platform-independent lifecycle / proposal transaction.
// This class does NOT read a drive, acquire Motion ownership, change HOME,
// commit AxisContext scalars, or prove delivery. MotionIntegrationReleased is
// still false. The caller must supply an independently verified authority and
// commit the returned reference plan at its exact stopped-coordinate seam.
#include "MechanicalCompensationCoordinates.h"
#include <cstdint>

namespace pbc
{
enum class LifecyclePhase : unsigned
{
    Unconfigured, NeedsReference, Ready, Held, HomeFrozen, OutputUncertain
};
enum class CycleMode : unsigned { Motion, Hold, ControlledStop, HomeFrozen };
enum class ReferenceAction : unsigned { Initial, ResetRetain, ServoRetain, HomeClear };
enum class ReferenceCommitResult : unsigned { NotApplied, Applied, Uncertain };
struct CycleIdentity
{
    std::uint64_t tick = 0ULL;
    std::uint64_t epoch = 0ULL;
    std::uint64_t ownerGeneration = 0ULL;
    std::uint64_t referenceGeneration = 0ULL;
    bool operator==(const CycleIdentity& b) const noexcept
    {
        return tick == b.tick && epoch == b.epoch &&
            ownerGeneration == b.ownerGeneration && referenceGeneration == b.referenceGeneration;
    }
};
struct CycleAuthority
{
    CycleIdentity identity{};
    CycleMode mode = CycleMode::Motion;
    bool sourceCurrent = false;
    bool rawServoReady = false;
    bool homed = false;
};
struct ReferenceAuthority
{
    std::uint64_t tick = 0ULL;
    std::uint64_t epoch = 0ULL;
    std::uint64_t ownerGeneration = 0ULL;
    std::uint64_t requestSequence = 0ULL;
    // Strictly increasing for this AxisLifecycle lifetime, across owner/epoch
    // changes. Consumed requests cannot be replayed at a newer tick. No wrap.
    bool stopped = false;
    bool queuesDrained = false;
    bool coordinateReservation = false;
    bool homeOwner = false;
};
struct ReferencePlan
{
    double nominalCommandPulse = 0.0;
    double rawFeedbackPulse = 0.0;
    double offsetPulse = 0.0;
    double coordinateShiftPulse = 0.0;
    ReferenceAction action = ReferenceAction::Initial;
    std::uint64_t ticket = 0ULL;
};

class AxisLifecycle
{
public:
    bool Configure(const Config& c, const std::vector<double>& positive,
        const std::vector<double>& negative, Diagnostic& d)
    {
        if (IsConfigured()) { d.error = Error::ConfigurationSealed; return false; }
        if (!model_.Configure(c, positive, negative, d)) return false;
        phase_ = LifecyclePhase::NeedsReference;
        return true;
    }
    bool IsConfigured() const noexcept { return model_.IsConfigured(); }
    bool IsEnabled() const noexcept { return model_.IsEnabled(); }
    LifecyclePhase Phase() const noexcept { return phase_; }
    const State& Runtime() const noexcept { return model_.Runtime(); }
    const CoordinateFrame& CommittedFrame() const noexcept { return committed_; }
    std::uint64_t ReferenceGeneration() const noexcept { return referenceGeneration_; }
    bool HasPending() const noexcept { return pending_.kind != Kind::None; }

    // HOME changes the machine coordinate origin; it must not release the
    // previous following error as a new motor command. Retain the prior servo
    // target in the shifted raw frame while clearing compensation history.
    // Only this model's committed offset is authoritative. An enabled model
    // with uncertain output cannot reconstruct that target; it needs the
    // separate physical HomeClear recovery established by its caller.
    bool PrepareHomeReferencePreservingCommand(double rawFeedbackPulse,
        double nominalCommandPulse, double coordinateShiftPulse,
        const ReferenceAuthority& authority, ReferencePlan& plan,
        Diagnostic& d) noexcept
    {
        d = Diagnostic{};
        if (!IsConfigured()) return Fail(d, Error::NotConfigured);
        if (!std::isfinite(nominalCommandPulse) ||
            (IsEnabled() && phase_ == LifecyclePhase::OutputUncertain))
            return Fail(d, Error::LifecycleRejected);
        const double oldOffsetPulse = IsEnabled() ?
            (model_.m_state.backlash + model_.m_state.pitch) *
                model_.Configuration().pulsePerUnit : 0.0;
        // Keep the historical OFF arithmetic, including its rounding order.
        const double nominal = IsEnabled() ?
            nominalCommandPulse + oldOffsetPulse + coordinateShiftPulse :
            nominalCommandPulse + coordinateShiftPulse;
        Input input{}; input.nominalPulse = nominal;
        Output output{}; output.servoPulse = nominal;
        CoordinateFrame frame{};
        if (!std::isfinite(oldOffsetPulse) || !std::isfinite(nominal) ||
            !BuildCoordinateFrame(model_.Configuration(), input, output, frame))
            return Fail(d, Error::RuntimeNumber);
        ReferencePlan candidate{};
        if (!PrepareReference(ReferenceAction::HomeClear, rawFeedbackPulse,
            coordinateShiftPulse, authority, candidate, d)) return false;
        pending_.state.lastNominalPulse = nominal;
        pending_.frame = frame;
        candidate.nominalCommandPulse = nominal;
        // HOME's actual feedback follows its existing scalar shift arithmetic.
        // RESET/Servo identity paths retain raw signed-zero bits instead.
        candidate.rawFeedbackPulse = rawFeedbackPulse + coordinateShiftPulse;
        plan = candidate;
        return true;
    }

    // Offset-only inverse, never a raw-feedback overwrite. RESET/Servo cancel
    // the obsolete ramp but retain the applied offset. HOME is a real stopped
    // coordinate establishment: shift raw feedback, clear calibration history.
    // An uncertain NIC outcome cannot be recovered by guessing the old offset.
    bool PrepareReference(ReferenceAction action, double rawFeedbackPulse,
        double coordinateShiftPulse, const ReferenceAuthority& authority,
        ReferencePlan& plan, Diagnostic& d) noexcept
    {
        d = Diagnostic{};
        if (!IsConfigured()) return Fail(d, Error::NotConfigured);
        if (HasPending() || !ValidReferenceAuthority(authority) ||
            authority.tick <= lastDecisionTick_ ||
            authority.requestSequence <= lastReferenceRequestSequence_ ||
            !std::isfinite(rawFeedbackPulse) || !std::isfinite(coordinateShiftPulse) ||
            (action != ReferenceAction::Initial && action != ReferenceAction::ResetRetain &&
                action != ReferenceAction::ServoRetain && action != ReferenceAction::HomeClear) ||
            (action != ReferenceAction::HomeClear && coordinateShiftPulse != 0.0) ||
            (action == ReferenceAction::Initial && referenceGeneration_ != 0ULL) ||
            (action == ReferenceAction::HomeClear && !authority.homeOwner) ||
            (action != ReferenceAction::Initial && action != ReferenceAction::HomeClear &&
                referenceGeneration_ == 0ULL) ||
            (phase_ == LifecyclePhase::OutputUncertain && action != ReferenceAction::HomeClear) ||
            referenceGeneration_ == (std::numeric_limits<std::uint64_t>::max)())
            return Fail(d, Error::LifecycleRejected);
        State state = model_.m_state;
        if (action == ReferenceAction::Initial || action == ReferenceAction::HomeClear)
            state = State{};
        const double offset = state.backlash + state.pitch;
        const bool disabledIdentity = !IsEnabled() && coordinateShiftPulse == 0.0;
        const double raw = disabledIdentity ? rawFeedbackPulse :
            rawFeedbackPulse + coordinateShiftPulse;
        const double nominal = !IsEnabled() ? raw :
            raw - offset * model_.Configuration().pulsePerUnit;
        const double reconstructed = !IsEnabled() ? nominal :
            nominal + offset * model_.Configuration().pulsePerUnit;
        if (!std::isfinite(raw) || !std::isfinite(nominal) ||
            !std::isfinite(reconstructed) || std::abs(reconstructed - raw) > 0.25)
            return Fail(d, Error::RuntimeNumber);
        // A reference transaction cancels the old motion; HOLD does not.
        state.targetBacklash = state.backlash;
        state.targetPitch = state.pitch;
        state.lastNominalPulse = nominal;
        state.lastNominalValid = false;
        state.armedByMotion = false;
        state.settled = true;
        Input input{}; input.nominalPulse = nominal;
        Output output{};
        output.servoPulse = reconstructed;
        output.offsetUnit = offset;
        CoordinateFrame frame{};
        if (!BuildCoordinateFrame(model_.Configuration(), input, output, frame))
            return Fail(d, Error::RuntimeNumber);
        std::uint64_t ticket = 0ULL;
        if (!NextTicket(ticket)) return Fail(d, Error::LifecycleRejected);
        Pending next{};
        next.kind = Kind::Reference;
        next.ticket = ticket;
        next.state = state;
        next.frame = frame;
        next.referenceAuthority = authority;
        next.action = action;
        pending_ = next;
        plan.nominalCommandPulse = nominal;
        plan.rawFeedbackPulse = raw;
        plan.offsetPulse = frame.offsetPulse;
        plan.coordinateShiftPulse = coordinateShiftPulse;
        plan.action = action;
        plan.ticket = ticket;
        return true;
    }

    // Call ONLY at the exact reserved coordinate commit seam. NotApplied must
    // mean NONE of the axis scalars changed. Partial/unknown external commits
    // poison the coordinate state instead of assuming successful rollback.
    bool FinishReference(std::uint64_t ticket, const ReferenceAuthority& current,
        ReferenceCommitResult result, Diagnostic& d) noexcept
    {
        d = Diagnostic{};
        if (pending_.kind != Kind::Reference || ticket != pending_.ticket)
        {
            // A stale callback cannot establish which coordinates were changed.
            // Any reported external application is uncertain even if another
            // proposal is pending. Only a definitely-unapplied stale result is
            // harmless to the currently committed coordinates.
            return result == ReferenceCommitResult::NotApplied ?
                Fail(d, Error::LifecycleRejected) : PoisonOutput(d);
        }
        const ReferenceAuthority& original = pending_.referenceAuthority;
        const bool currentAuthority = ValidReferenceAuthority(current) &&
            current.tick == original.tick && current.epoch == original.epoch &&
            current.ownerGeneration == original.ownerGeneration &&
            current.requestSequence == original.requestSequence && current.homeOwner == original.homeOwner;
        lastDecisionTick_ = original.tick;
        lastReferenceRequestSequence_ = original.requestSequence;
        if (!currentAuthority && result == ReferenceCommitResult::NotApplied)
        { pending_ = Pending{}; return Fail(d, Error::LifecycleRejected); }
        if (result == ReferenceCommitResult::NotApplied)
        { pending_ = Pending{}; return true; }
        if (!currentAuthority || result != ReferenceCommitResult::Applied)
            return PoisonOutput(d);
        model_.m_state = pending_.state;
        committed_ = pending_.frame;
        ++referenceGeneration_;
        phase_ = LifecyclePhase::Ready;
        pending_ = Pending{};
        return true;
    }

    // Preview consumes no model state. A target calculated under an old owner,
    // reference, tick or epoch is not a transferable send authorization.
    bool PrepareCycle(const Input& input, const CycleAuthority& authority,
        CoordinateFrame& frame, std::uint64_t& ticket, Diagnostic& d) noexcept
    {
        d = Diagnostic{};
        if (!IsConfigured()) return Fail(d, Error::NotConfigured);
        if (HasPending() || referenceGeneration_ == 0ULL ||
            phase_ == LifecyclePhase::OutputUncertain || !authority.sourceCurrent ||
            !authority.rawServoReady || !ValidIdentity(authority.identity) ||
            authority.identity.referenceGeneration != referenceGeneration_ ||
            authority.identity.tick <= lastDecisionTick_ ||
            (authority.mode != CycleMode::Motion && authority.mode != CycleMode::Hold &&
                authority.mode != CycleMode::ControlledStop && authority.mode != CycleMode::HomeFrozen) ||
            (authority.mode == CycleMode::Motion && !authority.homed) ||
            (authority.mode == CycleMode::Hold &&
                (input.nominalPulse != committed_.nominalCommandPulse || input.nominalVelocityPPS != 0.0)))
            return Fail(d, Error::LifecycleRejected);
        Input gated = input;
        gated.servoOn = true;
        gated.homed = authority.homed;
        gated.mayAdvance = authority.mode == CycleMode::Motion;
        State state{}; Output output{}; CoordinateFrame candidate{};
        if (!model_.EvaluateStep(gated, state, output, d)) return false;
        if (!BuildCoordinateFrame(model_.Configuration(), gated, output, candidate))
            return Fail(d, Error::RuntimeNumber);
        std::uint64_t nextTicket = 0ULL;
        if (!NextTicket(nextTicket)) return Fail(d, Error::LifecycleRejected);
        Pending next{};
        next.kind = Kind::Cycle;
        next.ticket = nextTicket;
        next.identity = authority.identity;
        next.mode = authority.mode;
        next.state = state;
        next.frame = candidate;
        pending_ = next;
        frame = candidate;
        ticket = nextTicket;
        return true;
    }

    // The transport owner calls this only AFTER final image/authority checks.
    // The compared frame is the exact candidate sent to the servo controller;
    // this function itself neither copies a PDO packet nor acquires a lease.
    bool SealCycle(std::uint64_t ticket, const CycleIdentity& identity,
        const CoordinateFrame& actualCandidate, Diagnostic& d) noexcept
    {
        d = Diagnostic{};
        if (pending_.kind != Kind::Cycle || ticket != pending_.ticket ||
            !(identity == pending_.identity) || pending_.sealed ||
            !SameFrame(actualCandidate, pending_.frame))
            return Fail(d, Error::LifecycleRejected);
        pending_.sealed = true;
        return true;
    }

    // Explicitly discard the candidate without making a claim about a NIC
    // attempt. Used when its output was not adopted (including a verified
    // zero fence). An engine may also use this for an uncertain OFF identity:
    // transport uncertainty is still reported, but its offset is provably zero.
    // Never use it to recover an enabled model's uncertain physical output.
    bool DiscardCycleProposal(std::uint64_t ticket, const CycleIdentity& identity,
        Diagnostic& d) noexcept
    {
        d = Diagnostic{};
        if (pending_.kind != Kind::Cycle || ticket != pending_.ticket ||
            !(identity == pending_.identity))
            return Fail(d, Error::LifecycleRejected);
        lastDecisionTick_ = identity.tick;
        pending_ = Pending{};
        return true;
    }

    // NIC API acceptance is a SOFTWARE handoff, NOT drive reception/consumption.
    // Definitely not attempted -> discard. Attempted but failed/unknown ->
    // poison active coordinates; never assume rollback proves the physical state.
    bool FinishSend(std::uint64_t ticket, const CycleIdentity& identity,
        bool attempted, bool apiAccepted, Diagnostic& d) noexcept
    {
        d = Diagnostic{};
        if (pending_.kind != Kind::Cycle || ticket != pending_.ticket ||
            !(identity == pending_.identity))
        {
            // An attempted stale/mismatched send may have reached the wire.
            // Rejecting its identity alone cannot prove the old offset remains
            // valid. Fail closed, including when another proposal is pending.
            return (attempted || apiAccepted) ? PoisonOutput(d) :
                Fail(d, Error::LifecycleRejected);
        }
        lastDecisionTick_ = identity.tick;
        if (!attempted)
        {
            // Acceptance without an attempt is a contradictory transport
            // outcome, not evidence that no output was issued.
            if (apiAccepted) return PoisonOutput(d);
            pending_ = Pending{};
            return true;
        }
        if (!pending_.sealed || !apiAccepted)
            return PoisonOutput(d);
        model_.m_state = pending_.state;
        committed_ = pending_.frame;
        phase_ = pending_.mode == CycleMode::Motion ? LifecyclePhase::Ready :
            pending_.mode == CycleMode::HomeFrozen ? LifecyclePhase::HomeFrozen : LifecyclePhase::Held;
        pending_ = Pending{};
        return true;
    }

private:
    enum class Kind : unsigned { None, Reference, Cycle };
    struct Pending
    {
        Kind kind = Kind::None;
        std::uint64_t ticket = 0ULL;
        State state{};
        CoordinateFrame frame{};
        CycleIdentity identity{};
        ReferenceAuthority referenceAuthority{};
        CycleMode mode = CycleMode::Motion;
        ReferenceAction action = ReferenceAction::Initial;
        bool sealed = false;
    };
    AxisModel model_{};
    CoordinateFrame committed_{};
    Pending pending_{};
    LifecyclePhase phase_ = LifecyclePhase::Unconfigured;
    std::uint64_t nextTicket_ = 0ULL;
    std::uint64_t referenceGeneration_ = 0ULL;
    std::uint64_t lastDecisionTick_ = 0ULL;
    std::uint64_t lastReferenceRequestSequence_ = 0ULL;
    static bool Fail(Diagnostic& d, Error e) noexcept { d.error = e; return false; }
    bool PoisonOutput(Diagnostic& d) noexcept
    {
        // Consume only our own proposal identity. An unrelated callback's
        // arbitrary future tick/sequence must not prevent a later HOME recovery.
        if (pending_.kind == Kind::Reference)
        {
            lastDecisionTick_ = (std::max)(lastDecisionTick_, pending_.referenceAuthority.tick);
            lastReferenceRequestSequence_ = (std::max)(lastReferenceRequestSequence_,
                pending_.referenceAuthority.requestSequence);
        }
        else if (pending_.kind == Kind::Cycle)
            lastDecisionTick_ = (std::max)(lastDecisionTick_, pending_.identity.tick);
        phase_ = LifecyclePhase::OutputUncertain;
        committed_.valid = false;
        pending_ = Pending{};
        return Fail(d, Error::OutputUncertain);
    }
    bool NextTicket(std::uint64_t& ticket) noexcept
    {
        if (nextTicket_ == (std::numeric_limits<std::uint64_t>::max)()) return false;
        ticket = ++nextTicket_; return true;
    }
    static bool ValidIdentity(const CycleIdentity& i) noexcept
    { return i.tick != 0ULL && i.epoch != 0ULL && i.ownerGeneration != 0ULL && i.referenceGeneration != 0ULL; }
    static bool ValidReferenceAuthority(const ReferenceAuthority& a) noexcept
    {
        return a.tick != 0ULL && a.epoch != 0ULL && a.ownerGeneration != 0ULL &&
            a.requestSequence != 0ULL && a.stopped && a.queuesDrained && a.coordinateReservation;
    }
    static bool SameFrame(const CoordinateFrame& a, const CoordinateFrame& b) noexcept
    {
        return IsCoordinateFrameValid(a) &&
            a.nominalCommandPulse == b.nominalCommandPulse && a.nominalVelocityPPS == b.nominalVelocityPPS &&
            a.servoCommandPulse == b.servoCommandPulse && a.servoVelocityPPS == b.servoVelocityPPS &&
            a.pulsePerUnit == b.pulsePerUnit && a.offsetUnit == b.offsetUnit && a.offsetPulse == b.offsetPulse &&
            a.offsetVelocityPPS == b.offsetVelocityPPS && a.remainingUnit == b.remainingUnit &&
            a.enabled == b.enabled && a.valid == b.valid && a.settled == b.settled;
    }
};

// Boot only: local synthetic data, no live axis / PDO / HOME mutation.
inline bool CheckLifecycleContract(unsigned& passed)
{
    passed = 0U;
    const auto check = [&passed](bool ok) { if (ok) ++passed; return ok; };
    AxisLifecycle life;
    Config c{}; c.pitch = true; c.pulsePerUnit = 1000.0; c.pitchSpeed = 0.5;
    Diagnostic d{};
    const std::vector<double> table{ 0.020, 0.020 };
    if (!check(life.Configure(c, table, table, d))) return false;
    ReferenceAuthority ref{};
    ref.tick = 1; ref.epoch = 1; ref.ownerGeneration = 1; ref.requestSequence = 1;
    ref.stopped = ref.queuesDrained = ref.coordinateReservation = true;
    ReferencePlan plan{};
    if (!check(life.PrepareReference(ReferenceAction::Initial, 10000.0, 0.0, ref, plan, d))) return false;
    if (!check(life.FinishReference(plan.ticket, ref, ReferenceCommitResult::Applied, d))) return false;
    CycleAuthority auth{};
    auth.identity = { 2, 1, 1, life.ReferenceGeneration() };
    auth.sourceCurrent = auth.rawServoReady = auth.homed = true;
    Input in{}; in.nominalPulse = 10001.0; in.nominalVelocityPPS = 10.0;
    in.maxVelocityPPS = 1000.0;
    CoordinateFrame frame{}; std::uint64_t ticket = 0;
    if (!check(life.PrepareCycle(in, auth, frame, ticket, d))) return false;
    if (!check(life.Runtime().pitch == 0.0 && frame.offsetUnit > 0.0)) return false;
    CycleIdentity stale = auth.identity; ++stale.epoch;
    if (!check(!life.SealCycle(ticket, stale, frame, d))) return false;
    if (!check(life.FinishSend(ticket, auth.identity, false, false, d) && life.Runtime().pitch == 0.0)) return false;
    ++auth.identity.tick;
    if (!check(life.PrepareCycle(in, auth, frame, ticket, d))) return false;
    if (!check(life.SealCycle(ticket, auth.identity, frame, d))) return false;
    if (!check(life.FinishSend(ticket, auth.identity, true, true, d))) return false;
    const double offset = frame.offsetUnit;
    if (!check(life.Runtime().pitch == offset && life.CommittedFrame().remainingUnit > 0.0)) return false;
    ++auth.identity.tick; auth.mode = CycleMode::Hold; in.nominalVelocityPPS = 0.0;
    if (!check(life.PrepareCycle(in, auth, frame, ticket, d) && frame.offsetUnit == offset &&
        frame.offsetVelocityPPS == 0.0 && frame.remainingUnit > 0.0 && !frame.settled)) return false;
    if (!check(life.SealCycle(ticket, auth.identity, frame, d) &&
        life.FinishSend(ticket, auth.identity, true, true, d))) return false;
    ref.tick = 5; ref.requestSequence = 2;
    if (!check(life.PrepareReference(ReferenceAction::ResetRetain, 10001.0 + offset * 1000.0,
        0.0, ref, plan, d) && plan.nominalCommandPulse == 10001.0)) return false;
    if (!check(life.FinishReference(plan.ticket, ref, ReferenceCommitResult::Applied, d) && life.Runtime().pitch == offset &&
        !life.Runtime().armedByMotion && life.CommittedFrame().remainingUnit == 0.0)) return false;
    ref.tick = 6; ref.requestSequence = 3;
    if (!check(life.PrepareReference(ReferenceAction::ServoRetain, 10001.0 + offset * 1000.0,
        0.0, ref, plan, d) && plan.nominalCommandPulse == 10001.0)) return false;
    if (!check(life.FinishReference(plan.ticket, ref, ReferenceCommitResult::Applied, d))) return false;
    auth.identity = { 7, 2, 2, life.ReferenceGeneration() }; auth.mode = CycleMode::Motion;
    if (!check(life.PrepareCycle(in, auth, frame, ticket, d) && frame.offsetUnit == offset)) return false;
    if (!check(life.SealCycle(ticket, auth.identity, frame, d))) return false;
    if (!check(!life.FinishSend(ticket, auth.identity, true, false, d) &&
        life.Phase() == LifecyclePhase::OutputUncertain && !life.CommittedFrame().valid)) return false;
    ref.tick = 8; ref.requestSequence = 4;
    if (!check(!life.PrepareReference(ReferenceAction::ResetRetain, 10001.0, 0.0, ref, plan, d))) return false;
    ref.homeOwner = true;
    if (!check(life.PrepareReference(ReferenceAction::HomeClear, 10001.0, -10000.0, ref, plan, d) &&
        plan.nominalCommandPulse == 1.0 && plan.offsetPulse == 0.0)) return false;
    if (!check(life.FinishReference(plan.ticket, ref, ReferenceCommitResult::Applied, d) && life.Runtime().pitch == 0.0)) return false;
    ref.tick = 9; // The completed HOME request cannot shift coordinates twice.
    if (!check(!life.PrepareReference(ReferenceAction::HomeClear, 1.0, -10000.0, ref, plan, d) &&
        life.CommittedFrame().nominalCommandPulse == 1.0)) return false;
    auth.identity = { 9, 2, 2, life.ReferenceGeneration() }; in.nominalPulse = 1.0;
    if (!check(life.PrepareCycle(in, auth, frame, ticket, d))) return false;
    if (!check(!life.FinishSend(ticket, auth.identity, false, true, d) &&
        life.Phase() == LifecyclePhase::OutputUncertain && !life.CommittedFrame().valid)) return false;
    if (!check(!life.Configure(c, table, table, d))) return false;
    return true;
}

// PBC-3D boot-only check of HOME error preservation and rejected partial
// commits. Synthetic enabled data does not release the live runtime gate.
inline bool CheckXReferenceContract(unsigned& passed)
{
    passed = 0U;
    const auto check = [&passed](bool ok) { if (ok) ++passed; return ok; };
    AxisLifecycle life;
    Config c{}; c.pitch = true; c.pulsePerUnit = 1024.0; c.pitchSpeed = 1.0;
    Diagnostic d{};
    const std::vector<double> table{ 0.03125, 0.03125 };
    if (!check(life.Configure(c, table, table, d))) return false;
    ReferenceAuthority ref{};
    ref.tick = ref.epoch = ref.ownerGeneration = ref.requestSequence = 1ULL;
    ref.stopped = ref.queuesDrained = ref.coordinateReservation = true;
    ReferencePlan plan{};
    if (!check(life.PrepareReference(ReferenceAction::Initial, 10000.0, 0.0, ref, plan, d))) return false;
    if (!check(life.FinishReference(plan.ticket, ref, ReferenceCommitResult::Applied, d))) return false;
    CycleAuthority auth{}; auth.identity = { 2ULL, 1ULL, 1ULL, life.ReferenceGeneration() };
    auth.homed = auth.sourceCurrent = auth.rawServoReady = true;
    Input in{}; in.nominalPulse = 10004.0; in.nominalVelocityPPS = 1.0;
    in.maxVelocityPPS = 2000.0; in.dt = 0.0078125;
    CoordinateFrame frame{}; std::uint64_t ticket = 0ULL;
    if (!check(life.PrepareCycle(in, auth, frame, ticket, d))) return false;
    if (!check(frame.offsetPulse == 8.0 && frame.remainingUnit > 0.0)) return false;
    if (!check(life.SealCycle(ticket, auth.identity, frame, d))) return false;
    if (!check(life.FinishSend(ticket, auth.identity, true, true, d))) return false;
    ref.tick = 3ULL; ref.requestSequence = 2ULL; ref.homeOwner = true;
    if (!check(life.PrepareHomeReferencePreservingCommand(10010.0, 10004.0, -9990.0, ref, plan, d))) return false;
    if (!check(plan.rawFeedbackPulse == 20.0 && plan.nominalCommandPulse == 22.0 && plan.offsetPulse == 0.0)) return false;
    if (!check(life.CommittedFrame().offsetPulse == 8.0)) return false;
    if (!check(life.FinishReference(plan.ticket, ref, ReferenceCommitResult::NotApplied, d))) return false;
    if (!check(life.CommittedFrame().offsetPulse == 8.0 && life.ReferenceGeneration() == 1ULL)) return false;
    ref.tick = 4ULL; ref.requestSequence = 3ULL;
    if (!check(life.PrepareHomeReferencePreservingCommand(10010.0, 10004.0, -9990.0, ref, plan, d))) return false;
    if (!check(life.FinishReference(plan.ticket, ref, ReferenceCommitResult::Applied, d))) return false;
    if (!check(life.CommittedFrame().servoCommandPulse - plan.rawFeedbackPulse == 2.0 &&
        life.CommittedFrame().offsetPulse == 0.0 && life.CommittedFrame().settled &&
        !life.Runtime().armedByMotion && life.ReferenceGeneration() == 2ULL)) return false;
    ref.tick = 5ULL; // Completed HOME may not be replayed.
    if (!check(!life.PrepareHomeReferencePreservingCommand(20.0, 22.0, -1.0, ref, plan, d))) return false;
    ref.requestSequence = 4ULL;
    if (!check(life.PrepareReference(ReferenceAction::ResetRetain, 20.0, 0.0, ref, plan, d))) return false;
    if (!check(!life.FinishReference(plan.ticket, ref, ReferenceCommitResult::Uncertain, d))) return false;
    ref.tick = 6ULL; ref.requestSequence = 5ULL;
    if (!check(!life.PrepareHomeReferencePreservingCommand(20.0, 22.0, -1.0, ref, plan, d))) return false;
    if (!check(life.Phase() == LifecyclePhase::OutputUncertain && !life.HasPending())) return false;
    // The dedicated raw-reference physical recovery remains a separate API.
    if (!check(life.PrepareReference(ReferenceAction::HomeClear, 20.0, -19.0, ref, plan, d))) return false;
    if (!check(life.FinishReference(plan.ticket, ref, ReferenceCommitResult::Applied, d))) return false;
    if (!check(life.CommittedFrame().nominalCommandPulse == 1.0)) return false;
    AxisLifecycle off; c.pitch = false;
    if (!check(off.Configure(c, {}, {}, d))) return false;
    ref.tick = ref.requestSequence = 1ULL;
    if (!check(off.PrepareHomeReferencePreservingCommand(4.0, 7.0, -2.0, ref, plan, d))) return false;
    if (!check(plan.rawFeedbackPulse == 2.0 && plan.nominalCommandPulse == 5.0)) return false;
    if (!check(off.FinishReference(plan.ticket, ref, ReferenceCommitResult::Applied, d))) return false;
    if (!check(!off.CommittedFrame().enabled && off.CommittedFrame().offsetPulse == 0.0)) return false;
    return true;
}
}
