#pragma once

#include "EDMFeedRetreatSimulation.h"
#include <cstdint>
#include <limits>

// Unlike EDM01's independent state checks, these 24 steps retain one continuous
// Monitor -> process permission -> coordinator -> virtual consumer session.
// Virtual milliseconds are independent of the NC scheduler. The only resets
// inside a run are the explicit RESET test (step 23, zero-based index 22)
// and its subsequent rearm (step 24, zero-based index 23).
namespace EDMFeedRetreatSimulationTest
{
    constexpr std::uint32_t CaseCount = 24U;
    inline const char* CaseName(std::uint32_t index) noexcept
    {
        static const char* const names[CaseCount] = {
            "START_STOP_REQUIRED", "STOP_ACK_NO_NORMAL", "FRESH_NORMAL_FEED", "FEED_PROGRESS",
            "PENDING_LOW_STOP", "LOW_WITHOUT_STOP_ACK", "STOP_ACK_RETREAT", "RETREAT_PROGRESS",
            "EARLY_NORMAL_NO_PERMIT", "RETREAT_DONE_WAIT_NORMAL", "FRESH_NORMAL_RETURN", "RETURN_PROGRESS",
            "RETURN_LOW_STOP", "RETURN_LOW_NO_STOP_ACK", "RETURN_STOP_ACK", "PENDING_NORMAL_NO_RETURN",
            "FRESH_NORMAL_RESUME_RETURN", "RETURN_DONE_NO_PERMIT", "FRESH_NORMAL_FEED_AGAIN", "HOLD_NO_AUTORESTART",
            "EXPLICIT_REARM", "STOP_NO_AUTORESTART", "RESET_NO_AUTORESTART", "OLD_TOKEN_FAULT_LATCH"
        };
        return index < CaseCount ? names[index] : "INVALID_CASE";
    }
    struct CaseResult
    {
        bool passed = false;
        EDMFeedRetreatSimulation::Phase expectedPhase = EDMFeedRetreatSimulation::Phase::Idle;
        EDMFeedRetreatSimulation::Snapshot actual{};
    };
    class Scenario
    {
    public:
        void Reset() noexcept
        {
            m_next = 0U;
            m_now = 1000ULL;
            m_sequence = 0ULL;
            m_ok = true;
            m_cancelled = false;
            if (m_session == (std::numeric_limits<std::uint64_t>::max)())
            {
                m_ok = false;
                m_cancelled = true;
                m_model.Revoke();
                return;
            }
            EDMFeedRetreatSimulation::Scope scope{};
            scope.session = ++m_session;
            scope.owner = 1ULL;
            scope.epoch = m_session;
            m_model.Reset(scope); // Preserves lifetime command-token serial.
            m_consumer.Reset(scope);
            Require(m_monitor.Configure(EDMGap::Config{}, EDMGap::Source::SIMULATED));
        }
        void Revoke() noexcept { m_cancelled = true; m_model.Revoke(); }
        const EDMFeedRetreatSimulation::Snapshot& Current() const noexcept { return m_model.Current(); }
        CaseResult Step(std::uint32_t index) noexcept
        {
            using namespace EDMFeedRetreatSimulation;
            using Command = EDMProcessSimulation::Command;
            CaseResult result{};
            if (m_cancelled || index != m_next || index >= CaseCount)
            {
                Revoke();
                result.actual = Current();
                return result;
            }
            ++m_next;
            Phase expected = Phase::Idle;
            switch (index)
            {
            case 0U:
                Apply(Command::Start);
                expected = Phase::WaitStop;
                Require(Current().action == Action::Stop);
                break;
            case 1U:
                Acknowledge(0ULL);
                expected = Phase::WaitStop;
                break;
            case 2U:
                Emit(50000, 10ULL);
                Require(Current().phase == Phase::WaitStop);
                Emit(50000, 30ULL);
                expected = Phase::Feeding;
                break;
            case 3U:
                Advance(50000);
                expected = Phase::Feeding;
                Require(Current().virtualPosition == Coordinator::InitialPosition + Consumer::FeedStep);
                break;
            case 4U:
                Emit(20000, 10ULL);
                expected = Phase::WaitStop;
                Require(Current().action == Action::Stop && !Current().retreatRequested);
                break;
            case 5U:
                Emit(20000, 30ULL);
                expected = Phase::WaitStop;
                Require(Current().action == Action::Stop && !Current().retreatRequested);
                break;
            case 6U:
                Acknowledge(0ULL);
                expected = Phase::Retreating;
                m_returnTarget = Current().virtualPosition;
                m_retreatTarget = Current().targetPosition;
                Require(m_returnTarget - m_retreatTarget == Coordinator::RetreatDistance);
                break;
            case 7U:
                Advance(20000);
                expected = Phase::Retreating;
                Require(Current().virtualPosition == m_returnTarget - Consumer::TravelStep);
                break;
            case 8U:
                Emit(50000, 10ULL);
                Emit(50000, 30ULL);
                expected = Phase::Retreating;
                Require(Current().processState == EDMProcessSimulation::State::Active);
                Require(!Current().simulatedPermit); // EDM01 alone is already permitting.
                break;
            case 9U:
                Advance(50000);
                Advance(50000);
                Advance(50000);
                expected = Phase::WaitNormal;
                Require(Current().virtualPosition == m_retreatTarget);
                Apply(); // Reusing completion-time NORMAL cannot start return.
                break;
            case 10U:
                Emit(50000, 10ULL);
                expected = Phase::Returning;
                Require(Current().targetPosition == m_returnTarget);
                break;
            case 11U:
                Advance(50000);
                expected = Phase::Returning;
                break;
            case 12U:
                Emit(20000, 10ULL);
                expected = Phase::WaitStop;
                Require(Current().action == Action::Stop && !Current().retreatRequested);
                break;
            case 13U:
                Emit(20000, 30ULL);
                expected = Phase::WaitStop;
                Require(Current().action == Action::Stop && !Current().retreatRequested);
                break;
            case 14U:
                Acknowledge(0ULL);
                expected = Phase::ReturnStopped;
                Require(Current().action == Action::Stop && !Current().retreatRequested);
                break;
            case 15U:
                Emit(50000, 10ULL);
                expected = Phase::ReturnStopped;
                break;
            case 16U:
                Emit(50000, 30ULL);
                expected = Phase::Returning;
                Require(Current().targetPosition == m_returnTarget);
                break;
            case 17U:
                Advance(50000);
                Advance(50000);
                Advance(50000);
                expected = Phase::WaitGap;
                Require(Current().virtualPosition == m_returnTarget);
                Apply(); // Return completion does not itself authorize pulses/feed.
                break;
            case 18U:
                Emit(50000, 10ULL);
                expected = Phase::Feeding;
                break;
            case 19U:
                Apply(Command::Hold);
                Check(Phase::Hold);
                Acknowledge(0ULL);
                Emit(50000, 10ULL);
                expected = Phase::Hold;
                break;
            case 20U:
                Apply(Command::Start);
                Require(Current().phase == Phase::WaitStop);
                Acknowledge(0ULL);
                Require(Current().phase == Phase::WaitStop);
                Emit(50000, 10ULL);
                expected = Phase::Feeding;
                break;
            case 21U:
                Apply(Command::Stop);
                Acknowledge(0ULL);
                Emit(50000, 10ULL);
                expected = Phase::Idle;
                break;
            case 22U:
                m_oldToken = Current().commandToken;
                Apply(Command::Reset);
                m_consumer.Reset(Current().scope);
                Emit(50000, 10ULL);
                expected = Phase::Idle;
                break;
            case 23U:
            {
                Apply(Command::Start);
                Require(Current().phase == Phase::WaitStop && Current().commandToken > m_oldToken);
                Feedback stale = m_consumer.Consume(Current(), m_now);
                stale.commandToken = m_oldToken;
                m_model.Step(Command::None, m_monitor.Current(), m_now, &stale);
                Require(Current().reason == Reason::Token);
                Emit(50000, 10ULL);
                Apply(Command::Start); // START/valid input do not clear the latch.
                m_model.Revoke();
                expected = Phase::Fault;
                break;
            }
            }
            Check(expected);
            result.expectedPhase = expected;
            result.actual = Current();
            result.passed = m_ok;
            return result;
        }

    private:
        void Require(bool condition) noexcept { m_ok = m_ok && condition; }
        void Invariant() noexcept
        {
            using namespace EDMFeedRetreatSimulation;
            const Snapshot& s = Current();
            Require(!s.PhysicalDischargeEnabled());
            Require(s.simulatedPermit == (s.phase == Phase::Feeding));
            Require(s.retreatRequested == (s.phase == Phase::Retreating));
            Require(s.virtualPosition >= Coordinator::MinimumPosition && s.virtualPosition <= Coordinator::MaximumPosition);
            if (s.phase != Phase::Feeding && s.phase != Phase::Retreating && s.phase != Phase::Returning)
                Require(s.simulatedVelocity == 0);
        }
        void Check(EDMFeedRetreatSimulation::Phase phase) noexcept
        { Require(Current().phase == phase); Invariant(); }
        void Apply(EDMProcessSimulation::Command command = EDMProcessSimulation::Command::None) noexcept
        { m_model.Step(command, m_monitor.Current(), m_now); Invariant(); }
        void Publish(std::int32_t voltage, std::uint64_t delta) noexcept
        {
            m_now += delta;
            EDMGap::Sample sample{};
            sample.voltageMv = voltage;
            sample.sequence = ++m_sequence;
            sample.sampledAtMs = m_now;
            sample.source = EDMGap::Source::SIMULATED;
            sample.valid = true;
            m_monitor.Publish(sample, m_now);
        }
        void Emit(std::int32_t voltage, std::uint64_t delta) noexcept
        { Publish(voltage, delta); Apply(); }
        void Acknowledge(std::uint64_t delta) noexcept
        {
            m_now += delta;
            m_monitor.Poll(m_now);
            const EDMFeedRetreatSimulation::Feedback feedback = m_consumer.Consume(Current(), m_now);
            m_model.Step(EDMProcessSimulation::Command::None, m_monitor.Current(), m_now, &feedback);
            Invariant();
        }
        void Advance(std::int32_t voltage) noexcept
        {
            Publish(voltage, 10ULL);
            const EDMFeedRetreatSimulation::Feedback feedback = m_consumer.Consume(Current(), m_now);
            m_model.Step(EDMProcessSimulation::Command::None, m_monitor.Current(), m_now, &feedback);
            Invariant();
        }
        EDMGap::Monitor m_monitor{};
        EDMFeedRetreatSimulation::Coordinator m_model{};
        EDMFeedRetreatSimulation::Consumer m_consumer{};
        std::uint64_t m_session = 0ULL, m_now = 1000ULL, m_sequence = 0ULL, m_oldToken = 0ULL;
        std::uint32_t m_next = 0U;
        std::int32_t m_returnTarget = 0, m_retreatTarget = 0;
        bool m_ok = true, m_cancelled = true;
    };
    static_assert(sizeof(Scenario) <= 768U, "EDM02 scenario fixed storage budget");
}
