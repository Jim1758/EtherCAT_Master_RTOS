#pragma once
// PBC-3C. A bounded software handoff transaction, NOT a drive acknowledgement
// and NOT a HOME/reference or AxisLifecycle commit. The transport must supply
// the final checked identity while its real owner/epoch reservations are held.
// Runtime integration in this release admits disabled compensation only.
// PBC-3G distinguishes immutable failure history from an unresolved active
// output. There is intentionally no recovery/clear API: this release has no
// independently verified physical-HOME receipt that could authorize one.
#include "MechanicalCompensationCoordinates.h"
#include <atomic>
#include <cstdint>

namespace pbc
{
struct SendIdentity
{
    std::uint64_t frame = 0ULL;
    std::uint64_t tick = 0ULL;
    std::uint64_t ownerState = 0ULL;
    std::uint64_t execution = 0ULL;
    std::uint64_t safety = 0ULL;
    std::uint64_t alarmSafety = 0ULL;
    std::uint64_t admissionGeneration = 0ULL;
    std::uint32_t alarmUpdate = 0U;
    std::uint32_t mode = 0U;
    bool operator==(const SendIdentity& b) const noexcept
    {
        return frame == b.frame && tick == b.tick && ownerState == b.ownerState &&
            execution == b.execution && safety == b.safety && alarmSafety == b.alarmSafety &&
            admissionGeneration == b.admissionGeneration && alarmUpdate == b.alarmUpdate && mode == b.mode;
    }
};
enum class SendResult : unsigned
{
    Bypassed = 0U, HandoffAccepted, FenceAccepted, Discarded, OutputUncertain
};

class SendContract
{
public:
    // Transactions/history have one RT writer. Only the unresolved latch query
    // is published atomically for concurrent command-admission readers.
    bool Prepare(const SendIdentity& id, const CoordinateFrame& frame,
        std::int32_t proposedVelocity, bool sourceReady, std::uint64_t& ticket) noexcept
    {
        ticket = 0ULL;
        if (pending_ || !sourceReady || !IsCoordinateFrameValid(frame) ||
            id.frame == 0ULL || id.tick == 0ULL || id.ownerState == 0ULL || id.execution == 0ULL ||
            id.frame <= lastFrame_ || id.tick <= lastTick_ ||
            // 2=NORMAL, 3=CONTROLLED_STOP, 4=IDLE_HOLD. ZERO_ONLY is not a candidate.
            (id.mode < 2U || id.mode > 4U) || (frame.enabled && HasUnresolvedActiveOutput()) ||
            sequence_ == (std::numeric_limits<std::uint64_t>::max)()) return false;
        pending_ = true; decision_ = Decision::Unsealed;
        identity_ = id; frame_ = frame; proposed_ = proposedVelocity;
        ticket = ++sequence_;
        return true;
    }

    // A verified zero scrub is a fence, never a successful candidate handoff.
    // Only the first final-image decision may seal the local pending ticket.
    bool Seal(std::uint64_t ticket, const SendIdentity& current,
        const CoordinateFrame& actualFrame, std::int32_t finalVelocity,
        bool imagePresent, bool reservationsCurrent, bool scrubbed) noexcept
    {
        if (!pending_ || ticket != sequence_ || decision_ != Decision::Unsealed) return false;
        decision_ = Decision::Rejected;
        if (!(current == identity_) || !imagePresent) return false;
        if (scrubbed)
        {
            if (finalVelocity != 0) return false;
            decision_ = Decision::Fence;
            return true;
        }
        if (!reservationsCurrent || finalVelocity != proposed_ || !SameFrame(frame_, actualFrame)) return false;
        decision_ = Decision::Sealed;
        return true;
    }

    SendResult Finish(std::uint64_t ticket, const SendIdentity& current,
        bool attempted, bool apiAccepted) noexcept
    {
        const bool matched = pending_ && ticket == sequence_ && current == identity_;
        // Only an exact canonical OFF image that reached the final seal can
        // prove that compensation state was unaffected by a NIC API failure.
        // A wrong identity, missing/failed seal or contradictory callback does
        // not establish what image was attempted, even if an old frame is OFF.
        const bool knownOffImage = attempted && matched &&
            (decision_ == Decision::Sealed || decision_ == Decision::Fence) &&
            HasDisabledCoordinateIdentity(frame_.enabled, frame_.offsetUnit, frame_);
        SendResult result = SendResult::Discarded;
        if ((!attempted && apiAccepted) || (attempted && (!apiAccepted || !matched)))
            result = SendResult::OutputUncertain;
        else if (attempted)
            result = decision_ == Decision::Sealed ? SendResult::HandoffAccepted :
                decision_ == Decision::Fence ? SendResult::FenceAccepted : SendResult::OutputUncertain;
        // Consume only our own pending identity. A stale external callback must
        // not advance the local high-water marks to an arbitrary future value.
        if (matched || result == SendResult::OutputUncertain)
        {
            if (pending_) { lastFrame_ = identity_.frame; lastTick_ = identity_.tick; }
            pending_ = false; decision_ = Decision::Unsealed;
        }
        if (result == SendResult::OutputUncertain)
        {
            everUncertain_ = true;
            if (uncertainIncidentGeneration_ != (std::numeric_limits<std::uint64_t>::max)())
                ++uncertainIncidentGeneration_;
            if (!knownOffImage) unresolvedActive_.store(true, std::memory_order_release);
        }
        return result;
    }
    bool HasPending() const noexcept { return pending_; }
    // History never disappears after subsequent successful or zero-only sends.
    bool SawUncertainOutput() const noexcept { return everUncertain_; }
    // Active/unknown output remains quarantined. Ordinary RESET, Servo, HOME,
    // a new reference number or later NIC success cannot clear this latch.
    bool HasUnresolvedActiveOutput() const noexcept
    { return unresolvedActive_.load(std::memory_order_acquire); }
    std::uint64_t UncertainIncidentGeneration() const noexcept
    { return uncertainIncidentGeneration_; }
    // One-way containment for an Engine-side unknown outcome even when the
    // NIC API accepted a packet (for example a lost final model receipt).
    // This is software-state uncertainty, not an invented NIC API failure.
    // Repeated observations of the same quarantined state are idempotent.
    void QuarantineActiveOutput() noexcept
    {
        everUncertain_ = true;
        // A proposal prepared before the incident cannot escape through an
        // already-issued ticket or seal. Retain it only for truthful retirement.
        if (pending_ && frame_.enabled) decision_ = Decision::Rejected;
        if (!unresolvedActive_.load(std::memory_order_relaxed))
        {
            if (uncertainIncidentGeneration_ != (std::numeric_limits<std::uint64_t>::max)())
                ++uncertainIncidentGeneration_;
            unresolvedActive_.store(true, std::memory_order_release);
        }
    }
private:
    enum class Decision : unsigned { Unsealed, Sealed, Fence, Rejected };
    SendIdentity identity_{};
    CoordinateFrame frame_{};
    std::uint64_t sequence_ = 0ULL, lastFrame_ = 0ULL, lastTick_ = 0ULL;
    std::int32_t proposed_ = 0;
    Decision decision_ = Decision::Unsealed;
    bool pending_ = false, everUncertain_ = false;
    std::atomic<bool> unresolvedActive_{ false };
    std::uint64_t uncertainIncidentGeneration_ = 0ULL;
    static bool SameFrame(const CoordinateFrame& a, const CoordinateFrame& b) noexcept
    {
        return IsCoordinateFrameValid(a) && IsCoordinateFrameValid(b) &&
            a.nominalCommandPulse == b.nominalCommandPulse && a.nominalVelocityPPS == b.nominalVelocityPPS &&
            a.servoCommandPulse == b.servoCommandPulse && a.servoVelocityPPS == b.servoVelocityPPS &&
            a.pulsePerUnit == b.pulsePerUnit && a.offsetUnit == b.offsetUnit &&
            a.offsetPulse == b.offsetPulse && a.offsetVelocityPPS == b.offsetVelocityPPS &&
            a.remainingUnit == b.remainingUnit && a.enabled == b.enabled && a.settled == b.settled;
    }
};

inline bool CheckSendContract(unsigned& checks) noexcept
{
    checks = 0U; bool pass = true;
    const auto check = [&checks, &pass](bool value) noexcept { ++checks; pass = pass && value; };
    CoordinateFrame f{};
    f.nominalCommandPulse = f.servoCommandPulse = 100.0;
    f.nominalVelocityPPS = f.servoVelocityPPS = 25.0;
    f.pulsePerUnit = 1000.0; f.valid = f.settled = true;
    SendIdentity id{}; id.frame = id.tick = id.ownerState = id.execution = 1ULL; id.mode = 2U;
    SendContract c; std::uint64_t t = 0ULL, ignored = 0ULL;
    check(c.Prepare(id, f, 30, true, t));
    check(!c.Prepare(id, f, 30, true, ignored));
    check(c.Seal(t, id, f, 30, true, true, false));
    check(!c.Seal(t, id, f, 30, true, true, false));
    check(c.Finish(t, id, true, true) == SendResult::HandoffAccepted);
    check(!c.HasPending() && !c.SawUncertainOutput());
    check(!c.Prepare(id, f, 30, true, t));
    ++id.frame; ++id.tick;
    check(c.Prepare(id, f, 30, true, t));
    check(c.Seal(t, id, f, 0, true, false, true));
    check(c.Finish(t, id, true, true) == SendResult::FenceAccepted);
    ++id.frame; ++id.tick;
    check(c.Prepare(id, f, 30, true, t));
    check(c.Finish(t, id, false, false) == SendResult::Discarded);
    ++id.frame; ++id.tick;
    check(c.Prepare(id, f, 30, true, t));
    check(c.Seal(t, id, f, 30, true, true, false));
    check(c.Finish(t, id, true, false) == SendResult::OutputUncertain);
    check(c.SawUncertainOutput());
    check(!c.HasUnresolvedActiveOutput() && c.UncertainIncidentGeneration() == 1ULL);
    ++id.frame; ++id.tick;
    check(c.Prepare(id, f, 30, true, t)); // zero-offset observation can continue
    SendIdentity changed = id; ++changed.execution;
    check(!c.Seal(t, changed, f, 30, true, true, false));
    check(c.Finish(t, id, true, true) == SendResult::OutputUncertain);
    check(c.HasUnresolvedActiveOutput() && c.UncertainIncidentGeneration() == 2ULL);
    ++id.frame; ++id.tick;
    check(c.Prepare(id, f, 30, true, t));
    check(!c.Seal(t, id, f, 31, true, true, false));
    check(c.Finish(t, id, false, false) == SendResult::Discarded);
    ++id.frame; ++id.tick;
    check(c.Prepare(id, f, 30, true, t));
    check(!c.Seal(t, id, f, 30, true, false, false));
    check(c.Finish(t, id, true, true) == SendResult::OutputUncertain);
    ++id.frame; ++id.tick;
    check(c.Prepare(id, f, 30, true, t));
    check(c.Finish(t + 1ULL, id, true, true) == SendResult::OutputUncertain);
    check(!c.HasPending());
    ++id.frame; ++id.tick;
    check(c.Prepare(id, f, 30, true, t));
    check(c.Finish(t, id, false, true) == SendResult::OutputUncertain);
    ++id.frame; ++id.tick;
    CoordinateFrame enabled = f;
    enabled.enabled = true; enabled.offsetUnit = 0.01; enabled.offsetPulse = 10.0;
    enabled.servoCommandPulse = 110.0;
    check(!c.Prepare(id, enabled, 30, true, t));
    check(!c.Prepare(id, f, 30, false, t));
    check(c.Prepare(id, f, 30, true, t));
    check(c.Seal(t, id, f, 30, true, true, false));
    check(c.Finish(t, id, true, true) == SendResult::HandoffAccepted);
    check(c.SawUncertainOutput()); // never healed by a subsequent NIC success
    check(c.HasUnresolvedActiveOutput());
    const std::uint64_t incidents = c.UncertainIncidentGeneration();
    c.QuarantineActiveOutput();
    check(c.HasUnresolvedActiveOutput() && c.UncertainIncidentGeneration() == incidents);
    SendContract knownOff;
    check(knownOff.Prepare(id, f, 30, true, t));
    check(knownOff.Seal(t, id, f, 30, true, true, false));
    check(knownOff.Finish(t, id, true, false) == SendResult::OutputUncertain);
    ++id.frame; ++id.tick;
    check(knownOff.Prepare(id, enabled, 30, true, t));
    check(knownOff.Finish(t, id, false, false) == SendResult::Discarded);
    check(knownOff.SawUncertainOutput() && !knownOff.HasUnresolvedActiveOutput());
    knownOff.QuarantineActiveOutput();
    check(knownOff.HasUnresolvedActiveOutput() && knownOff.UncertainIncidentGeneration() == 2ULL);
    ++id.frame; ++id.tick;
    check(!knownOff.Prepare(id, enabled, 30, true, t));
    return pass;
}
} // namespace pbc
