#pragma once

#include "EDMProcessCoordinator.h"
#include "EDMMotionAdmission.h"
#include <limits>

// P12 isolated SIM replay. Tick is one continuous process timeline. Step runs
// bounded contract fixtures; it never fast-forwards or replaces that timeline.
// Feedback in this header is explicitly synthetic, not an executed trajectory.
namespace EDMMotionReplaySimulationTest
{
    constexpr std::uint32_t CaseCount = 24U, CaseMask = 0x00ffffffU;
    constexpr std::uint32_t MaximumModelCallsPerCase = 32U, CasePeriodMs = 400U;
    constexpr std::uint64_t MaximumReplayServiceGapMs = 25U;
    constexpr std::uint64_t MinimumWindowMs = 10000U, MaximumWindowMs = 12000U;
    inline const char* CaseName(std::uint32_t index) noexcept
    {
        static const char* const names[CaseCount] = {
            "NORMAL_POSITIVE_REPLAY", "NEGATIVE_CURVE_REPLAY", "FIXED_SHORT_RETREAT_REPLAY",
            "FULL_EXIT_RETURN_BLOCK_REPLAY", "FLUSH_DEEP_REPLAY", "HOLD_RESET_REPLAY",
            "B0_B1_SIGNED_SPEED_MAPPING", "VALID_NATIVE_Z_INTENT", "POSITIVE_STOP_ENVELOPE",
            "NEGATIVE_STOP_ENVELOPE", "REVERSAL_PRIOR_MOMENTUM", "SPEED_NATIVE_BOUNDS",
            "INVALID_FROZEN_CONFIG", "ALL_SCOPE_FENCES", "AXIS_MAP_IDENTITY",
            "PROPOSAL_FEEDBACK_SEQUENCE", "CLOCK_STALE_FEEDBACK", "NONFINITE_MISSING_FEEDBACK",
            "ACK_IS_NOT_STANDSTILL", "ACK_SEQUENCE_KIND_FENCE", "CONTINUOUS_STOP_DWELL",
            "RETURN_POSITION_RECEIPT", "REVOKE_RESET_REARM", "RECEIPT_REPLAY_FENCE"
        };
        return index < CaseCount ? names[index] : "INVALID_CASE";
    }
    inline double MachineZVelocity(const EDM25::ProcessSnapshot& process, const EDM27::ZConfig& config) noexcept
    {
        if (process.direction == EDM20::FlushDirection::MachinePositiveZ ||
            process.direction == EDM20::FlushDirection::MachineNegativeZ)
            return process.speedMmMin / 60.0;
        return process.speedMmMin * config.machiningAdvanceZSign / 60.0;
    }
    struct Snapshot
    {
        EDM27::Scope scope{};
        std::uint32_t state = 0U, reason = 0U, action = 0U, direction = 0U, shortState = 0U;
        std::uint32_t admissionState = 0U, admissionReason = 0U, caseMask = 0U;
        std::uint32_t modelCalls = 0U, replayModelCalls = 0U;
        std::uint64_t completedCycles = 0U, workElapsedMs = 0U, replayElapsedMs = 0U;
        std::uint64_t proposalSequence = 0U, acknowledgedSequence = 0U, feedbackSequence = 0U, receiptSequence = 0U;
        double machiningOffsetMm = 0.0, flushOffsetMm = 0.0, speedMmMin = 0.0;
        double requestedVelocityMmS = 0.0, targetMm = 0.0, stopReserveMm = 0.0;
        bool deepCycle = false, completionReady = false, acknowledged = false;
        bool standstillProven = false, returnProven = false, receiptVerified = false;
        constexpr bool PhysicalMotionEnabled() const noexcept { return false; }
        constexpr bool PhysicalDischargeEnabled() const noexcept { return false; }
    };
    struct CaseResult { bool passed = false; std::uint32_t checks = 0U; Snapshot actual{}; };
    class Scenario
    {
    public:
        void Reset() noexcept { Reset(EDM27::Scope{}); }
        void Reset(const EDM27::Scope& scope) noexcept
        {
            model_.Reset(); guard_.Reset(); actual_ = Snapshot{}; actual_.scope = scope;
            config_ = Preset(); zConfig_ = EDM27::ZConfig{}; zConfig_.assumptionsConfirmed = true;
            input_ = EDM25::ProcessInput{};
            input_.sourceGeneration = input_.configRevision = input_.recipeGeneration = 1U;
            input_.interlockReady = input_.processAllowed = input_.machiningAllowed = input_.returnAllowed = true;
            input_.sample.valid = true; input_.sample.voltage = 60.0;
            nextCase_ = seen_ = 0U; sequence_ = lastMs_ = proposal_ = feedback_ = 0U;
            returnLowSince_ = 0U; resetDone_ = revoked_ = haveTick_ = sealed_ = receiptUsed_ = false;
            wasFlushing_ = false; flushOriginMm_ = 0.0;
            if (!model_.Start(config_, 0U) || !guard_.Start(zConfig_, scope)) revoked_ = true;
            Capture();
        }
        void Revoke() noexcept
        {
            if (!revoked_)
            {
                sealed_ = actual_.completionReady; sealedAtMs_ = lastMs_;
                sealedScope_ = actual_.scope; sealedProposal_ = actual_.proposalSequence;
                sealedAck_ = actual_.acknowledgedSequence; sealedFeedback_ = actual_.feedbackSequence;
                sealedMask_ = actual_.caseMask; sealedCycles_ = actual_.completedCycles;
            }
            revoked_ = true; model_.Revoke(); guard_.Revoke(); Capture();
            actual_.speedMmMin = actual_.requestedVelocityMmS = 0.0;
        }
        const Snapshot& Current() const noexcept { return actual_; }
        const Snapshot& Tick(std::uint64_t elapsedMs) noexcept
        {
            if (revoked_) return actual_;
            if ((haveTick_ && elapsedMs < lastMs_) || elapsedMs > MaximumWindowMs ||
                (!haveTick_ && elapsedMs > MaximumReplayServiceGapMs) ||
                (haveTick_ && elapsedMs - lastMs_ > MaximumReplayServiceGapMs))
            { Revoke(); actual_.completionReady = false;
              actual_.reason=static_cast<std::uint32_t>(elapsedMs<lastMs_?EDM25::ProcessReason::ClockRollback:EDM25::ProcessReason::ServiceGap);
              return actual_; }
            const bool newTime = !haveTick_ || elapsedMs > lastMs_;
            input_.nowMs = input_.sample.nowMs = elapsedMs; input_.sample.sequence = ++sequence_;
            input_.command = EDM25::ProcessCommand::Tick;
            input_.sample.voltage = elapsedMs >= 450U && elapsedMs < 650U ? 40.0 :
                (elapsedMs >= 650U && elapsedMs < 1100U ? 20.0 : 60.0);
            // A real return-leg short episode is placed by state, not guessed
            // from callback frequency. Every observed sample goes to the sole detector.
            if (!returnLowSince_ && elapsedMs >= 1200U && elapsedMs < 1900U &&
                model_.CycleSnapshot().state == EDM23::AutomaticFlushCycleState::Flushing &&
                model_.CycleSnapshot().flushExecution.leg == EDM20::FlushLeg::Return)
                returnLowSince_ = elapsedMs;
            if (returnLowSince_ && elapsedMs >= returnLowSince_ && elapsedMs < returnLowSince_ + 120U)
                input_.sample.voltage = 20.0;
            if (elapsedMs >= 2000U && elapsedMs < 2200U) input_.command = EDM25::ProcessCommand::Hold;
            else if (model_.Snapshot().state == EDM25::ProcessState::Hold) input_.command = EDM25::ProcessCommand::Resume;
            if (!resetDone_ && elapsedMs >= 2300U)
            {
                model_.Revoke(); model_.Reset(); (void)model_.Start(config_, elapsedMs);
                resetDone_ = true; seen_ |= ResetSeen; wasFlushing_ = false;
            }
            if (newTime || input_.command != EDM25::ProcessCommand::Tick) (void)model_.Step(input_);
            else (void)model_.ObserveSafety(input_);
            ++actual_.replayModelCalls; lastMs_ = elapsedMs; haveTick_ = true;
            const auto& process = model_.Snapshot();
            if (process.action == EDM25::ProcessAction::ServoAdvance && process.speedMmMin > 0.0) seen_ |= Positive;
            if (process.action == EDM25::ProcessAction::ServoRetreat && process.speedMmMin < 0.0) seen_ |= Negative;
            if (process.action == EDM25::ProcessAction::ShortRetreat && process.speedMmMin == -0.5) seen_ |= Short;
            if (process.shortState == EDMGapServo::ShortState::Exiting) seen_ |= Exit;
            if ((seen_ & Exit) && process.shortState == EDMGapServo::ShortState::Clear) seen_ |= Cleared;
            if (process.action == EDM25::ProcessAction::ReturnBlocked) seen_ |= BlockedReturn;
            if (process.action == EDM25::ProcessAction::FlushRetreat) seen_ |= Flush;
            if (process.deepCycle) seen_ |= Deep;
            if (process.state == EDM25::ProcessState::Hold) seen_ |= Held;
            const bool flushing = model_.CycleSnapshot().state == EDM23::AutomaticFlushCycleState::Flushing ||
                model_.CycleSnapshot().state == EDM23::AutomaticFlushCycleState::ReturnBlocked;
            if (flushing && !wasFlushing_) flushOriginMm_ = process.machiningOffsetMm * zConfig_.machiningAdvanceZSign;
            wasFlushing_ = flushing;
            if (newTime)
            {
                EDM27::ZFeedback f = Feedback(actual_.scope, ++feedback_, elapsedMs);
                EDM27::ZProposal p{}; p.scope = actual_.scope; p.sequence = ++proposal_; p.createdAtMs = elapsedMs;
                p.signedVelocityMmS = MachineZVelocity(process, zConfig_);
                // A zero selected velocity is still an intent endpoint. It
                // is not a stop/return command with an inferred completion.
                p.command = EDM27::ZCommand::Velocity;
                // This preset is B0 only: capture its machining origin and
                // map the separate flush length along its retreat sign.
                // B1 position replay is not implemented by this fixture.
                p.targetMm = flushing ? flushOriginMm_ + process.flushOffsetMm * -zConfig_.machiningAdvanceZSign :
                    process.machiningOffsetMm * zConfig_.machiningAdvanceZSign;
                if (!guard_.Propose(p, f, elapsedMs) || !guard_.Acknowledge(p.scope, p.sequence, p.command,elapsedMs))
                { Revoke(); actual_.completionReady = false; return actual_; }
            }
            Capture();
            if (!newTime) actual_.requestedVelocityMmS = 0.0;
            actual_.completionReady = !revoked_ && actual_.caseMask == CaseMask && elapsedMs >= MinimumWindowMs &&
                process.state == EDM25::ProcessState::Running && process.shortState == EDMGapServo::ShortState::Clear &&
                process.completedCycles >= 3U && model_.CycleSnapshot().state == EDM23::AutomaticFlushCycleState::Machining &&
                process.workElapsedMs == 0U && std::fabs(process.flushOffsetMm) <= 1e-12;
            return actual_;
        }
        CaseResult Step(std::uint32_t index) noexcept
        {
            CaseResult result{}; checks_ = calls_ = 0U; passed_ = true;
            if (revoked_ || index != nextCase_ || index >= CaseCount)
            { Revoke(); result.actual = actual_; return result; }
            ++nextCase_;
            EDM27::ZAdmissionGuard g; EDM27::ZConfig c = zConfig_; EDM27::Scope s = actual_.scope;
            EDM27::ZFeedback f = Feedback(s, 1U, 100U);
            EDM27::ZProposal p{}; p.scope = s; p.sequence = 1U; p.createdAtMs = 100U;
            p.command = EDM27::ZCommand::Velocity; p.signedVelocityMmS = 0.1;
            switch (index)
            {
            case 0U: Require((seen_ & Positive) != 0U); break;
            case 1U: Require((seen_ & Negative) != 0U); break;
            case 2U: Require((seen_ & Short) != 0U); break;
            case 3U: Require((seen_ & (Exit | Cleared | BlockedReturn)) == (Exit | Cleared | BlockedReturn)); break;
            case 4U: Require((seen_ & Flush) != 0U); break;
            case 5U: Require((seen_ & (Held | ResetSeen)) == (Held | ResetSeen)); break;
            case 6U:
            {
                EDM25::ProcessSnapshot ps{}; ps.speedMmMin = 60.0;
                Near(MachineZVelocity(ps, c), -1.0); ps.speedMmMin = -60.0;
                Near(MachineZVelocity(ps, c), 1.0); ps.direction = EDM20::FlushDirection::MachinePositiveZ;
                Near(MachineZVelocity(ps, c), -1.0); ps.speedMmMin=60.0;
                Near(MachineZVelocity(ps,c),1.0);ps.direction=EDM20::FlushDirection::MachineNegativeZ;
                Near(MachineZVelocity(ps,c),1.0);ps.speedMmMin=-60.0;
                Near(MachineZVelocity(ps, c), -1.0); break;
            }
            case 7U:
                Begin(g,c,s); Require(Propose(g,p,f)); Near(g.Snapshot().nativeVelocityPulsesS,100.0);
                Near(g.Snapshot().stopReserveMm,0.1543); Require(!g.Snapshot().PhysicalMotionEnabled()); break;
            case 8U:
                Begin(g,c,s); f.positionMm = p.targetMm = c.upperMm - 0.1;
                Require(!Propose(g,p,f)); Reason(g,EDM27::AdmissionReason::StopEnvelope); break;
            case 9U:
                Begin(g,c,s); f.positionMm = p.targetMm = c.lowerMm + 0.1; p.signedVelocityMmS = -0.1;
                Require(!Propose(g,p,f)); Reason(g,EDM27::AdmissionReason::StopEnvelope); break;
            case 10U:
                Begin(g,c,s); Require(Propose(g,p,f)); f.sequence = 2U; f.sampledAtMs = p.createdAtMs = 101U;
                p.sequence = 2U; p.signedVelocityMmS = -0.1; f.positionMm = p.targetMm = c.upperMm - 0.1;
                Require(!Propose(g,p,f)); Reason(g,EDM27::AdmissionReason::StopEnvelope); break;
            case 11U:
                Begin(g,c,s); p.signedVelocityMmS = 1.1; Require(!Propose(g,p,f)); Reason(g,EDM27::AdmissionReason::VelocityLimit);
                g.Reset(); c.nativePulsesPerMm = 1000000000.0; c.upperMm = 3.0;
                Count(); Require(!g.Start(c,s)); break;
            case 12U:
                c.decelerationMmS2 = std::numeric_limits<double>::quiet_NaN(); Count(); Require(!g.Start(c,s));
                g.Reset(); c = zConfig_; c.assumptionsConfirmed = false; Count(); Require(!g.Start(c,s)); break;
            case 13U:
                for (unsigned i = 0U; i < 7U; ++i)
                {
                    g.Reset(); Begin(g,c,s); p.scope = s;
                    ChangeScope(p.scope,i); Require(!Propose(g,p,f)); Reason(g,EDM27::AdmissionReason::ScopeMismatch);
                } break;
            case 14U:
                Begin(g,c,s); f.axisIndex = 0U; Require(!Propose(g,p,f)); Reason(g,EDM27::AdmissionReason::AxisMismatch); break;
            case 15U:
                Begin(g,c,s); Require(Propose(g,p,f)); Require(!Propose(g,p,f)); Reason(g,EDM27::AdmissionReason::Sequence);
                p.sequence = 2U; Require(!Propose(g,p,f)); Reason(g,EDM27::AdmissionReason::Sequence); break;
            case 16U:
                Begin(g,c,s); f.sampledAtMs = 89U; Require(!Propose(g,p,f)); Reason(g,EDM27::AdmissionReason::StaleFeedback);
                g.Reset(); Begin(g,c,s); f.sampledAtMs = 101U; Require(!Propose(g,p,f)); Reason(g,EDM27::AdmissionReason::Clock); break;
            case 17U:
                Begin(g,c,s); f.pdoVelocityMmS = std::numeric_limits<double>::infinity();
                Require(!Propose(g,p,f)); Reason(g,EDM27::AdmissionReason::InvalidFeedback);
                g.Reset(); Begin(g,c,s); f = Feedback(s,1U,100U); f.axisExists = false;
                Require(!Propose(g,p,f)); Reason(g,EDM27::AdmissionReason::InvalidFeedback); break;
            case 18U:
                Begin(g,c,s); p.command = EDM27::ZCommand::Stop; p.signedVelocityMmS = 0.0;
                Require(Propose(g,p,f)); Count(); Require(g.Acknowledge(s,1U,p.command,100U)); Require(!g.Snapshot().standstillProven);
                f.sequence = 2U; f.sampledAtMs = 101U; f.actualVelocityMmS = 0.1;
                Associate(f,g);
                Count(); Require(!g.Prove(f,101U)); Reason(g,EDM27::AdmissionReason::NotStandstill); break;
            case 19U:
                Begin(g,c,s); Require(Propose(g,p,f)); Count(); Require(!g.Acknowledge(s,2U,p.command,100U));
                g.Reset();Begin(g,c,s);Require(Propose(g,p,f));
                Count(); Require(!g.Acknowledge(s,1U,EDM27::ZCommand::Stop,100U));
                g.Reset();Begin(g,c,s);Require(Propose(g,p,f));
                Count(); Require(g.Acknowledge(s,1U,EDM27::ZCommand::Velocity,100U));
                Count(); Require(!g.Acknowledge(s,1U,p.command,100U)); Reason(g,EDM27::AdmissionReason::DuplicateAck); break;
            case 20U:
                Begin(g,c,s); StopProposal(p); Require(Propose(g,p,f)); Count(); Require(g.Acknowledge(s,1U,p.command,100U));
                Dwell(g,f); Require(g.Snapshot().standstillProven); Require(!g.Snapshot().returnProven); break;
            case 21U:
                Begin(g,c,s); StopProposal(p); p.command = EDM27::ZCommand::Return; p.targetMm = 0.1;
                Require(Propose(g,p,f)); Count(); Require(g.Acknowledge(s,1U,p.command,100U)); Dwell(g,f,false);
                Reason(g,EDM27::AdmissionReason::NotReturned); Require(!g.Snapshot().returnProven);
                f.sequence = 5U; f.sampledAtMs = 131U; f.positionMm = 0.1;
                Count(); Require(!g.Prove(f,131U)); f.sequence=6U;f.sampledAtMs=141U;
                Count(); Require(!g.Prove(f,141U)); f.sequence=7U;f.sampledAtMs=151U;
                Count(); Require(g.Prove(f,151U)); Require(g.Snapshot().returnProven); break;
            case 22U:
                Begin(g,c,s); Require(Propose(g,p,f)); g.Revoke(); Count(); Require(!g.Acknowledge(s,1U,p.command,100U));
                Count(); Require(!g.Start(c,s)); g.Reset(); ++s.session; Begin(g,c,s);
                Require(!Propose(g,p,f)); Reason(g,EDM27::AdmissionReason::ScopeMismatch);
                g.Reset();Begin(g,c,s);p.scope=f.scope=s;Require(Propose(g,p,f));break;
            case 23U:
                Require((seen_ & Deep) != 0U);
                Begin(g,c,s); StopProposal(p); p.command = EDM27::ZCommand::Return;
                Require(Propose(g,p,f)); Count(); Require(g.Acknowledge(s,1U,p.command,100U)); Dwell(g,f);
                Count(); Require(!g.Prove(f,121U)); Require(!g.Snapshot().returnProven); break;
            default: Require(false); break;
            }
            Require(calls_ <= MaximumModelCallsPerCase); actual_.modelCalls = calls_;
            if (passed_) actual_.caseMask |= (1U << index);
            result.passed = passed_; result.checks = checks_; result.actual = actual_; return result;
        }
        bool VerifyReceipt(std::uint64_t elapsedMs) noexcept { return VerifyReceipt(elapsedMs,actual_.scope); }
        bool VerifyReceipt(std::uint64_t elapsedMs, const EDM27::Scope& scope) noexcept
        {
            if (!revoked_ || !sealed_ || receiptUsed_ || !EDM27::SameScope(scope,sealedScope_) ||
                !EDM27::SameScope(actual_.scope,sealedScope_) || actual_.caseMask!=sealedMask_ || sealedMask_!=CaseMask ||
                actual_.proposalSequence!=sealedProposal_ || actual_.acknowledgedSequence!=sealedAck_ ||
                actual_.feedbackSequence!=sealedFeedback_ || actual_.replayElapsedMs!=sealedAtMs_ ||
                actual_.completedCycles!=sealedCycles_ || actual_.workElapsedMs!=0U || actual_.flushOffsetMm!=0.0 ||
                elapsedMs <= sealedAtMs_ || elapsedMs - sealedAtMs_ > 250U ||
                sealedProposal_ == 0U || sealedAck_ != sealedProposal_ || sealedFeedback_!=feedback_)
                return false;
            // Completion receipt is for the isolated test contract, not a
            // machine stop/return receipt. The process is already revoked.
            receiptUsed_ = true; actual_.receiptVerified = true; actual_.receiptSequence = ++feedback_;
            return true;
        }
    private:
        enum Seen : std::uint32_t { Positive=1U, Negative=2U, Short=4U, Exit=8U, Cleared=16U,
            BlockedReturn=32U, Flush=64U, Deep=128U, Held=256U, ResetSeen=512U };
        EDM25::ProcessCoordinator model_{}; EDM27::ZAdmissionGuard guard_{};
        EDM25::ProcessConfig config_{}; EDM27::ZConfig zConfig_{}; EDM25::ProcessInput input_{};
        Snapshot actual_{};
        std::uint32_t nextCase_ = 0U, seen_ = 0U, checks_ = 0U, calls_ = 0U;
        std::uint64_t sequence_ = 0U, lastMs_ = 0U, proposal_ = 0U, feedback_ = 0U;
        std::uint64_t returnLowSince_ = 0U, sealedAtMs_ = 0U;
        EDM27::Scope sealedScope_{};
        std::uint64_t sealedProposal_=0U,sealedAck_=0U,sealedFeedback_=0U,sealedCycles_=0U;
        std::uint32_t sealedMask_=0U;
        double flushOriginMm_ = 0.0;
        bool resetDone_ = false, revoked_ = false, haveTick_ = false, sealed_ = false, receiptUsed_ = false;
        bool wasFlushing_ = false, passed_ = true;
        static EDM25::ProcessConfig Preset() noexcept
        {
            EDM25::ProcessConfig c{}; c.servo.shortMachiningV = 30.0;
            c.servo.shortEnterMs = 20U; c.servo.shortExitMs = 50U; c.servo.shortHysteresisV = 2.0;
            c.flush.workTimeMs = 300U; c.flush.baseJumpHeightMm = 0.010;
            c.flush.deepFlushCycleInterval = 3U; c.flush.deepFlushHeightMultiplier = 2.0;
            c.flush.configRevision = c.flush.recipeGeneration = 1U;
            c.flush.flushRequest.jumpHeightConfirmed = true;
            c.flush.flushRequest.frame.machiningDirectionConfirmed = true;
            c.flush.flushRequest.frame.machineZDirectionConfirmed = true;
            c.flush.flushProfile.mainRetractMmPerMin = c.flush.flushProfile.mainReturnMmPerMin = 60.0;
            c.flush.flushProfile.finalApproachDistanceMm = 0.0; return c;
        }
        static EDM27::ZFeedback Feedback(const EDM27::Scope& s,std::uint64_t sequence,std::uint64_t now) noexcept
        { EDM27::ZFeedback f{}; f.scope=s; f.sequence=sequence; f.sampledAtMs=now; f.valid=f.axisExists=true; return f; }
        static void ChangeScope(EDM27::Scope& s,unsigned i) noexcept
        {
            switch(i) { case 0U: ++s.session; break; case 1U: ++s.ownerLease; break;
            case 2U: ++s.executionEpoch; break; case 3U: ++s.runGeneration; break;
            case 4U: ++s.dispatchGeneration; break; case 5U: ++s.axisMapGeneration; break;
            default: ++s.configGeneration; break; }
        }
        void Require(bool v) noexcept { ++checks_; passed_ = passed_ && v; }
        void Count() noexcept { ++calls_; }
        void Near(double a,double b) noexcept { Require(std::fabs(a-b)<=1e-10); }
        void Begin(EDM27::ZAdmissionGuard& g,const EDM27::ZConfig& c,const EDM27::Scope& s) noexcept
        { Count(); Require(g.Start(c,s)); }
        bool Propose(EDM27::ZAdmissionGuard& g,const EDM27::ZProposal& p,const EDM27::ZFeedback& f) noexcept
        { Count(); return g.Propose(p,f,p.createdAtMs); }
        void Reason(const EDM27::ZAdmissionGuard& g,EDM27::AdmissionReason reason) noexcept
        { Require(g.Snapshot().reason==reason); }
        static void StopProposal(EDM27::ZProposal& p) noexcept
        { p.command=EDM27::ZCommand::Stop; p.signedVelocityMmS=0.0; }
        void Dwell(EDM27::ZAdmissionGuard& g,EDM27::ZFeedback& f,bool success=true) noexcept
        {
            Associate(f,g);
            for(unsigned i=0U;i<3U;++i)
            { f.sequence=2U+i; f.sampledAtMs=101U+i*10U; Count(); Require(g.Prove(f,f.sampledAtMs)==(i==2U&&success)); }
        }
        static void Associate(EDM27::ZFeedback& f,const EDM27::ZAdmissionGuard& g) noexcept
        { f.appliedIdentityValid=true;f.appliedProposalSequence=g.Snapshot().proposalSequence;
          f.appliedCommand=g.Snapshot().command;f.appliedAtMs=g.Snapshot().acknowledgedAtMs; }
        void Capture() noexcept
        {
            const auto& p=model_.Snapshot(); const auto& a=guard_.Snapshot();
            actual_.state=static_cast<std::uint32_t>(p.state); actual_.reason=static_cast<std::uint32_t>(p.reason);
            actual_.action=static_cast<std::uint32_t>(p.action); actual_.direction=static_cast<std::uint32_t>(p.direction);
            actual_.shortState=static_cast<std::uint32_t>(p.shortState); actual_.deepCycle=p.deepCycle;
            actual_.completedCycles=p.completedCycles; actual_.workElapsedMs=p.workElapsedMs;
            actual_.machiningOffsetMm=p.machiningOffsetMm; actual_.flushOffsetMm=p.flushOffsetMm;
            actual_.speedMmMin=p.speedMmMin; actual_.replayElapsedMs=lastMs_;
            actual_.admissionState=static_cast<std::uint32_t>(a.state); actual_.admissionReason=static_cast<std::uint32_t>(a.reason);
            actual_.proposalSequence=a.proposalSequence; actual_.acknowledgedSequence=a.acknowledgedSequence;
            actual_.feedbackSequence=a.feedbackSequence; actual_.stopReserveMm=a.stopReserveMm;
            actual_.requestedVelocityMmS=a.requestedVelocityMmS; actual_.targetMm=a.targetMm;
            actual_.acknowledged=a.acknowledged; actual_.standstillProven=a.standstillProven; actual_.returnProven=a.returnProven;
        }
    };
}
