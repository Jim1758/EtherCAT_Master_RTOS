#pragma once

#include <cmath>
#include <cstdint>

// EDM27 pure single-Z intent contract. There is no mailbox, Motion API, PID,
// clock read, allocation or output here. All bounds/evidence are caller-owned
// fixtures until independently certified against the actual machine.
// A rejected Stop intent never gates the machine's existing emergency stop.
// Reset erases sequence history. A caller must rearm with a new Scope.session
// (or a new full generation); repeating the old scope cannot fence old packets.
namespace EDM27
{
    enum class ZCommand : std::uint8_t { Velocity, Stop, Return };
    enum class AdmissionState : std::uint8_t
    { Idle, Ready, Admitted, Acknowledged, Stopped, Returned, Cancelled, Fault };
    enum class AdmissionReason : std::uint8_t
    {
        None, InvalidConfig, InvalidLifecycle, ScopeMismatch, AxisMismatch,
        Sequence, Clock, InvalidFeedback, StaleFeedback, InvalidProposal,
        NativeRange, VelocityLimit, PositionLimit, StopEnvelope,
        AckMismatch, DuplicateAck, AwaitAck, NotStandstill, AwaitDwell,
        NotReturned, ServiceGap, InFlight, AppliedMismatch
    };
    struct Scope
    {
        std::uint64_t session = 1U, ownerLease = 1U, executionEpoch = 1U;
        std::uint64_t runGeneration = 1U, dispatchGeneration = 1U;
        std::uint64_t axisMapGeneration = 1U;
        std::uint64_t configGeneration = 1U;
    };
    inline bool SameScope(const Scope& a, const Scope& b) noexcept
    {
        return a.session == b.session && a.ownerLease == b.ownerLease &&
            a.executionEpoch == b.executionEpoch && a.runGeneration == b.runGeneration &&
            a.dispatchGeneration == b.dispatchGeneration && a.axisMapGeneration == b.axisMapGeneration &&
            a.configGeneration == b.configGeneration;
    }
    inline bool ValidScope(const Scope& s) noexcept
    {
        return s.session && s.ownerLease && s.executionEpoch && s.runGeneration &&
            s.dispatchGeneration && s.axisMapGeneration && s.configGeneration;
    }
    struct ZConfig
    {
        std::uint32_t axisIndex = 2U;
        int machiningAdvanceZSign = -1, hardwareSign = 1;
        double lowerMm = -2.0, upperMm = 2.0, nativePulsesPerMm = 1000.0;
        double maxVelocityMmS = 1.0, actualVelocityUpperBoundMmS = 1.2;
        double decelerationMmS2 = 10.0, positionReserveMm = 0.01;
        double feedbackLatencyMs = 10.0, consumerReactionUs = 250.0;
        double filterTailMs = 30.0, sCurveAllowanceMs = 20.0;
        double positionToleranceMm = 0.001, standstillToleranceMmS = 0.0001;
        std::uint32_t maximumFeedbackAgeMs = 10U, standstillDwellMs = 20U;
        // Local assumption validation only. This never certifies a real axis.
        bool assumptionsConfirmed = false;
    };
    struct ZFeedback
    {
        Scope scope{};
        std::uint32_t axisIndex = 2U;
        std::uint64_t sequence = 0U, sampledAtMs = 0U;
        std::uint64_t appliedProposalSequence = 0U, appliedAtMs = 0U;
        ZCommand appliedCommand = ZCommand::Stop;
        double positionMm = 0.0, velocityMmS = 0.0;
        double commandVelocityMmS = 0.0, actualVelocityMmS = 0.0, pdoVelocityMmS = 0.0;
        bool valid = false, axisExists = false, appliedIdentityValid = false;
    };
    struct ZProposal
    {
        Scope scope{};
        std::uint64_t sequence = 0U, createdAtMs = 0U;
        ZCommand command = ZCommand::Stop;
        double signedVelocityMmS = 0.0, targetMm = 0.0;
    };
    struct AdmissionSnapshot
    {
        AdmissionState state = AdmissionState::Idle;
        AdmissionReason reason = AdmissionReason::None;
        Scope scope{};
        ZCommand command = ZCommand::Stop;
        std::uint64_t proposalSequence = 0U, acknowledgedSequence = 0U, feedbackSequence = 0U;
        std::uint64_t acknowledgedAtMs = 0U;
        double stopReserveMm = 0.0, requestedVelocityMmS = 0.0, targetMm = 0.0;
        double nativeVelocityPulsesS = 0.0;
        bool acknowledged = false, standstillProven = false, returnProven = false;
        constexpr bool PhysicalMotionEnabled() const noexcept { return false; }
        constexpr bool PhysicalDischargeEnabled() const noexcept { return false; }
    };
    class ZAdmissionGuard
    {
    public:
        const AdmissionSnapshot& Snapshot() const noexcept { return snapshot_; }
        void Reset() noexcept { *this = ZAdmissionGuard{}; }
        void Revoke() noexcept
        {
            snapshot_.state = AdmissionState::Cancelled;
            snapshot_.requestedVelocityMmS = snapshot_.nativeVelocityPulsesS = 0.0;
            ClearEvidence();
        }
        bool Start(const ZConfig& config, const Scope& scope) noexcept
        {
            if (snapshot_.state != AdmissionState::Idle) return Fault(AdmissionReason::InvalidLifecycle);
            if (!Validate(config) || !ValidScope(scope)) return Fault(AdmissionReason::InvalidConfig);
            config_ = config; snapshot_.scope = scope;
            const double cap = config.actualVelocityUpperBoundMmS;
            // Conservative constant upper-bound travel during feedback,
            // consumer, filter and configured S-curve delays; then guaranteed
            // deceleration. No drive following bound is inferred from E8.
            snapshot_.stopReserveMm = config.positionReserveMm + cap *
                ((config.feedbackLatencyMs + config.filterTailMs + config.sCurveAllowanceMs) / 1000.0 +
                 config.consumerReactionUs / 1000000.0) + cap * cap / (2.0 * config.decelerationMmS2);
            if (!Finite(snapshot_.stopReserveMm) || snapshot_.stopReserveMm >= config.upperMm - config.lowerMm)
                return Fault(AdmissionReason::InvalidConfig);
            snapshot_.state = AdmissionState::Ready; return true;
        }
        bool Propose(const ZProposal& proposal, const ZFeedback& feedback, std::uint64_t nowMs) noexcept
        {
            if (!Usable()) return false;
            if (!OwnerClock(nowMs)) return false;
            if (!SameScope(proposal.scope, snapshot_.scope)) return Fault(AdmissionReason::ScopeMismatch);
            if (proposal.sequence == 0U || proposal.sequence <= lastProposalSequence_)
                return Reject(AdmissionReason::Sequence);
            if (proposal.createdAtMs > nowMs || nowMs < lastOwnerMs_)
                return Fault(AdmissionReason::Clock);
            lastOwnerMs_ = nowMs;
            lastProposalSequence_ = proposal.sequence;
            if (nowMs - proposal.createdAtMs > config_.maximumFeedbackAgeMs)
                return Reject(AdmissionReason::StaleFeedback);
            if (pendingTerminal_ && proposal.command != ZCommand::Stop)
                return Reject(AdmissionReason::InFlight);
            if (static_cast<unsigned>(proposal.command) > static_cast<unsigned>(ZCommand::Return) ||
                !Finite(proposal.signedVelocityMmS) || !Finite(proposal.targetMm) ||
                (proposal.command != ZCommand::Velocity && proposal.signedVelocityMmS != 0.0))
                return Reject(AdmissionReason::InvalidProposal);
            if (!ReadFeedback(feedback, nowMs, true)) return false;
            if (std::fabs(proposal.signedVelocityMmS) > config_.maxVelocityMmS)
                return Reject(AdmissionReason::VelocityLimit);
            const double native = proposal.signedVelocityMmS * config_.nativePulsesPerMm * config_.hardwareSign;
            const double targetNative = proposal.targetMm * config_.nativePulsesPerMm;
            if (!Native(native) || !Native(targetNative)) return Reject(AdmissionReason::NativeRange);
            if (!InBox(proposal.targetMm)) return Reject(AdmissionReason::PositionLimit);
            // Check every observed/current channel and prior admitted intent:
            // reversal does not erase momentum in the old direction.
            const double velocities[6] = { feedback.velocityMmS, feedback.commandVelocityMmS,
                feedback.actualVelocityMmS, feedback.pdoVelocityMmS,
                lastAdmittedVelocity_, proposal.signedVelocityMmS };
            for (unsigned i = 0U; i < 6U; ++i)
                if (!Envelope(feedback.positionMm, velocities[i]) || !Envelope(proposal.targetMm, velocities[i]))
                    return Reject(AdmissionReason::StopEnvelope);
            if (proposal.command == ZCommand::Return &&
                (!Envelope(feedback.positionMm, proposal.targetMm-feedback.positionMm) ||
                 !Envelope(proposal.targetMm, proposal.targetMm-feedback.positionMm)))
                return Reject(AdmissionReason::StopEnvelope);
            snapshot_.state = AdmissionState::Admitted; snapshot_.reason = AdmissionReason::None;
            snapshot_.command = proposal.command; snapshot_.proposalSequence = proposal.sequence;
            snapshot_.requestedVelocityMmS = proposal.signedVelocityMmS;
            snapshot_.nativeVelocityPulsesS = native; snapshot_.targetMm = proposal.targetMm;
            snapshot_.acknowledged = snapshot_.standstillProven = snapshot_.returnProven = false;
            snapshot_.acknowledgedSequence = 0U; admittedFeedbackMs_ = feedback.sampledAtMs;
            admittedFeedbackSequence_ = feedback.sequence; admittedAtMs_ = nowMs;
            lastAdmittedVelocity_ = proposal.signedVelocityMmS; haveDwell_ = false;
            pendingTerminal_ = proposal.command != ZCommand::Velocity; return true;
        }
        // The timestamp is the exact application/ACK event in this synchronous
        // fixture contract. It is not a delayed mailbox delivery timestamp.
        bool Acknowledge(const Scope& scope, std::uint64_t sequence, ZCommand command, std::uint64_t nowMs) noexcept
        {
            if (!Usable()) return false;
            if (!OwnerClock(nowMs)) return false;
            if (!SameScope(scope, snapshot_.scope)) return Fault(AdmissionReason::ScopeMismatch);
            if (nowMs < admittedAtMs_ || nowMs < lastOwnerMs_) return Fault(AdmissionReason::Clock);
            lastOwnerMs_ = nowMs;
            if (nowMs-admittedAtMs_ > config_.maximumFeedbackAgeMs) return Reject(AdmissionReason::StaleFeedback);
            if (sequence != snapshot_.proposalSequence || command != snapshot_.command)
                return Reject(AdmissionReason::AckMismatch);
            if (snapshot_.state == AdmissionState::Acknowledged || snapshot_.acknowledged)
                return Reject(AdmissionReason::DuplicateAck);
            if (snapshot_.state != AdmissionState::Admitted) return Reject(AdmissionReason::AckMismatch);
            snapshot_.acknowledged = true; snapshot_.acknowledgedSequence = sequence;
            snapshot_.acknowledgedAtMs = nowMs;
            snapshot_.state = AdmissionState::Acknowledged; snapshot_.reason = AdmissionReason::None;
            // A setter/mailbox ACK says nothing about velocity or return.
            return true;
        }
        bool Prove(const ZFeedback& feedback, std::uint64_t nowMs) noexcept
        {
            if (!Usable()) return false;
            if (!OwnerClock(nowMs)) return false;
            snapshot_.standstillProven = snapshot_.returnProven = false;
            if (!snapshot_.acknowledged) { snapshot_.reason = AdmissionReason::AwaitAck; return false; }
            snapshot_.state = AdmissionState::Acknowledged;
            if (snapshot_.command == ZCommand::Velocity)
            { snapshot_.reason = AdmissionReason::InvalidProposal; haveDwell_ = false; return false; }
            if (!ReadFeedback(feedback, nowMs, true)) { haveDwell_ = false; return false; }
            if (feedback.sequence <= admittedFeedbackSequence_ || feedback.sampledAtMs <= admittedFeedbackMs_ ||
                nowMs <= admittedAtMs_)
            { haveDwell_ = false; snapshot_.reason = AdmissionReason::Sequence; return false; }
            if (!feedback.appliedIdentityValid || feedback.appliedProposalSequence != snapshot_.proposalSequence ||
                feedback.appliedCommand != snapshot_.command || feedback.appliedAtMs != snapshot_.acknowledgedAtMs ||
                feedback.appliedAtMs > feedback.sampledAtMs || feedback.sampledAtMs <= snapshot_.acknowledgedAtMs)
            { haveDwell_ = false; snapshot_.reason = AdmissionReason::AppliedMismatch; return false; }
            if (std::fabs(feedback.velocityMmS) > config_.standstillToleranceMmS ||
                std::fabs(feedback.commandVelocityMmS) > config_.standstillToleranceMmS ||
                std::fabs(feedback.actualVelocityMmS) > config_.standstillToleranceMmS ||
                std::fabs(feedback.pdoVelocityMmS) > config_.standstillToleranceMmS)
            { haveDwell_ = false; snapshot_.reason = AdmissionReason::NotStandstill; return false; }
            if (snapshot_.command == ZCommand::Return &&
                std::fabs(feedback.positionMm - snapshot_.targetMm) > config_.positionToleranceMm)
            { haveDwell_ = false; snapshot_.reason = AdmissionReason::NotReturned; return false; }
            if (!haveDwell_ || std::fabs(feedback.positionMm-dwellPositionMm_) > config_.positionToleranceMm)
            { haveDwell_ = true; dwellSinceMs_ = feedback.sampledAtMs; dwellPositionMm_ = feedback.positionMm; }
            if (feedback.sampledAtMs - dwellSinceMs_ < config_.standstillDwellMs)
            { snapshot_.reason = AdmissionReason::AwaitDwell; return false; }
            snapshot_.standstillProven = true; snapshot_.state = AdmissionState::Stopped;
            if (snapshot_.command == ZCommand::Return)
            {
                snapshot_.returnProven = true; snapshot_.state = AdmissionState::Returned;
            }
            pendingTerminal_ = false; snapshot_.requestedVelocityMmS = snapshot_.nativeVelocityPulsesS = 0.0;
            snapshot_.reason = AdmissionReason::None; return true;
        }
    private:
        ZConfig config_{}; AdmissionSnapshot snapshot_{};
        std::uint64_t lastProposalSequence_ = 0U, lastFeedbackSequence_ = 0U, lastFeedbackMs_ = 0U;
        std::uint64_t lastOwnerMs_ = 0U, admittedFeedbackSequence_ = 0U, admittedFeedbackMs_ = 0U;
        std::uint64_t admittedAtMs_ = 0U, dwellSinceMs_ = 0U;
        double lastAdmittedVelocity_ = 0.0, dwellPositionMm_ = 0.0;
        bool haveFeedback_ = false, haveDwell_ = false, pendingTerminal_ = false;
        static bool Finite(double x) noexcept { return std::isfinite(x); }
        static bool Native(double x) noexcept { return Finite(x) && std::fabs(x) <= 2147483647.0; }
        static bool Validate(const ZConfig& c) noexcept
        {
            return c.assumptionsConfirmed && c.axisIndex == 2U &&
                (c.machiningAdvanceZSign == 1 || c.machiningAdvanceZSign == -1) &&
                (c.hardwareSign == 1 || c.hardwareSign == -1) &&
                Finite(c.lowerMm) && Finite(c.upperMm) && c.lowerMm < c.upperMm &&
                c.upperMm - c.lowerMm <= 1000.0 &&
                Finite(c.nativePulsesPerMm) && c.nativePulsesPerMm > 0.0 && c.nativePulsesPerMm <= 1000000000.0 &&
                Native(c.lowerMm * c.nativePulsesPerMm) && Native(c.upperMm * c.nativePulsesPerMm) &&
                Finite(c.maxVelocityMmS) && c.maxVelocityMmS > 0.0 && c.maxVelocityMmS <= 1000.0 &&
                Finite(c.actualVelocityUpperBoundMmS) && c.actualVelocityUpperBoundMmS >= c.maxVelocityMmS &&
                c.actualVelocityUpperBoundMmS <= 1000.0 && Native(c.actualVelocityUpperBoundMmS * c.nativePulsesPerMm) &&
                Finite(c.decelerationMmS2) && c.decelerationMmS2 > 0.0 && c.decelerationMmS2 <= 10000.0 &&
                Finite(c.positionReserveMm) && c.positionReserveMm >= 0.0 &&
                Finite(c.feedbackLatencyMs) && c.feedbackLatencyMs >= 10.0 && c.feedbackLatencyMs <= 100.0 &&
                Finite(c.consumerReactionUs) && c.consumerReactionUs >= 250.0 && c.consumerReactionUs <= 10000.0 &&
                Finite(c.filterTailMs) && c.filterTailMs >= 0.0 && c.filterTailMs <= 1000.0 &&
                Finite(c.sCurveAllowanceMs) && c.sCurveAllowanceMs >= 0.0 && c.sCurveAllowanceMs <= 1000.0 &&
                Finite(c.positionToleranceMm) && c.positionToleranceMm > 0.0 && c.positionToleranceMm <= 0.01 &&
                c.positionToleranceMm < (c.upperMm-c.lowerMm)/100.0 &&
                Finite(c.standstillToleranceMmS) && c.standstillToleranceMmS >= 0.0 &&
                c.standstillToleranceMmS <= 0.001 && c.standstillToleranceMmS < c.maxVelocityMmS/100.0 &&
                c.maximumFeedbackAgeMs > 0U && c.maximumFeedbackAgeMs <= 100U &&
                static_cast<double>(c.maximumFeedbackAgeMs) <= c.feedbackLatencyMs &&
                c.standstillDwellMs >= 20U && c.standstillDwellMs <= 1000U;
        }
        bool Usable() const noexcept
        { return snapshot_.state != AdmissionState::Idle && snapshot_.state != AdmissionState::Cancelled && snapshot_.state != AdmissionState::Fault; }
        bool OwnerClock(std::uint64_t nowMs) noexcept
        { if(nowMs<lastOwnerMs_) return Fault(AdmissionReason::Clock);lastOwnerMs_=nowMs;return true; }
        bool Fault(AdmissionReason reason) noexcept
        { snapshot_.state = AdmissionState::Fault; snapshot_.reason = reason; ClearEvidence(); snapshot_.requestedVelocityMmS = snapshot_.nativeVelocityPulsesS = 0.0; return false; }
        void ClearEvidence() noexcept
        { snapshot_.acknowledged = snapshot_.standstillProven = snapshot_.returnProven = false; haveDwell_ = false; }
        bool Reject(AdmissionReason reason) noexcept
        {
            snapshot_.state = AdmissionState::Ready; snapshot_.reason = reason;
            ClearEvidence();
            snapshot_.requestedVelocityMmS = snapshot_.nativeVelocityPulsesS = 0.0; return false;
        }
        bool InBox(double position) const noexcept
        { return position >= config_.lowerMm && position <= config_.upperMm; }
        bool Envelope(double position, double velocity) const noexcept
        {
            return velocity > 0.0 ? position + snapshot_.stopReserveMm <= config_.upperMm :
                (velocity < 0.0 ? position - snapshot_.stopReserveMm >= config_.lowerMm : InBox(position));
        }
        bool ReadFeedback(const ZFeedback& f, std::uint64_t nowMs, bool fresh) noexcept
        {
            if (!SameScope(f.scope, snapshot_.scope)) return Fault(AdmissionReason::ScopeMismatch);
            if (f.axisIndex != config_.axisIndex) return Fault(AdmissionReason::AxisMismatch);
            if (nowMs < lastOwnerMs_ || f.sampledAtMs > nowMs || (haveFeedback_ && f.sampledAtMs < lastFeedbackMs_))
                return Fault(AdmissionReason::Clock);
            if (haveFeedback_ && haveDwell_ && f.sampledAtMs - lastFeedbackMs_ > config_.maximumFeedbackAgeMs)
                haveDwell_ = false;
            lastOwnerMs_ = nowMs;
            if (f.sequence == 0U || (fresh && haveFeedback_ && f.sequence <= lastFeedbackSequence_))
                return Reject(AdmissionReason::Sequence);
            lastFeedbackSequence_ = f.sequence; lastFeedbackMs_ = f.sampledAtMs;
            haveFeedback_ = true; snapshot_.feedbackSequence = f.sequence;
            if (!f.valid || !f.axisExists || !Finite(f.positionMm) || !Finite(f.velocityMmS) ||
                !Finite(f.commandVelocityMmS) || !Finite(f.actualVelocityMmS) || !Finite(f.pdoVelocityMmS))
                return Reject(AdmissionReason::InvalidFeedback);
            if (nowMs - f.sampledAtMs > config_.maximumFeedbackAgeMs) return Reject(AdmissionReason::StaleFeedback);
            if (!InBox(f.positionMm)) return Reject(AdmissionReason::PositionLimit);
            const double cap = config_.actualVelocityUpperBoundMmS;
            if (std::fabs(f.velocityMmS) > cap || std::fabs(f.commandVelocityMmS) > cap ||
                std::fabs(f.actualVelocityMmS) > cap || std::fabs(f.pdoVelocityMmS) > cap)
                return Reject(AdmissionReason::VelocityLimit);
            if (!Native(f.positionMm * config_.nativePulsesPerMm)) return Reject(AdmissionReason::NativeRange);
            return true;
        }
    };
}
