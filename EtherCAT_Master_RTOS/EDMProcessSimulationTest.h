#pragma once

#include "EDMProcessSimulation.h"
#include <cstdint>

// EDM01 deterministic scenario, usable by both a host test and G180 P2.
// Each case owns a fresh virtual clock/source session and a bounded number of
// calls. The NC caller spaces Step() calls using its real clock; no sleep, I/O,
// Motion, PLC access, heap allocation, or hardware output occurs in this type.
namespace EDMProcessSimulationTest
{
    constexpr std::uint32_t CaseCount = 20U;

    inline const char* CaseName(std::uint32_t index) noexcept
    {
        static const char* const names[CaseCount] = {
            "RESET_IDLE", "START_NEEDS_FRESH", "NORMAL_ACTIVE", "PENDING_LOW_INHIBIT",
            "QUALIFIED_LOW_RETREAT", "NORMAL_RECOVERY", "HIGH_INHIBIT", "HOLD_EXPLICIT_REARM",
            "STOP_NO_AUTORESTART", "INTERLOCK_FAULT_LATCH", "INVALID_FAULT_LATCH", "STALE_BOUNDARY",
            "PHYSICAL_SOURCE_REJECT", "CONFIGURATION_REJECT", "CLOCK_REGRESSION", "FUTURE_SAMPLE_REJECT",
            "SEQUENCE_REGRESSION", "DUPLICATE_MUTATION", "RESET_NEEDS_START", "REVOKE_ALWAYS_OFF"
        };
        return index < CaseCount ? names[index] : "INVALID_CASE";
    }

    struct CaseResult
    {
        bool passed = false;
        EDMProcessSimulation::State expectedState = EDMProcessSimulation::State::Idle;
        EDMProcessSimulation::Snapshot actual{};
    };

    class Scenario
    {
    public:
        void Reset() noexcept
        {
            m_next = 0U;
            Prepare();
        }

        void Revoke() noexcept { m_model.Revoke(); }
        const EDMProcessSimulation::Snapshot& Current() const noexcept { return m_model.Current(); }

        CaseResult Step(std::uint32_t index) noexcept
        {
            using namespace EDMProcessSimulation;
            CaseResult result{};
            if (index != m_next || index >= CaseCount)
            {
                Revoke();
                result.actual = Current();
                return result;
            }
            ++m_next;
            Prepare();
            State expected = State::Idle;
            switch (index)
            {
            case 0U:
                Apply(Command::Reset);
                Check(State::Idle, false, false);
                Poll(10ULL);
                break;
            case 1U:
                Publish(50000, 0ULL);
                Apply(Command::Start);
                Check(State::WaitGap, false, false);
                Poll(30ULL); // Poll/duplicate cannot complete Monitor dwell.
                expected = State::WaitGap;
                break;
            case 2U:
                Activate();
                expected = State::Active;
                break;
            case 3U:
                Activate();
                Emit(20000, 10ULL);
                Require(Current().reason == Reason::PendingLow);
                expected = State::WaitGap;
                break;
            case 4U:
                Activate();
                Low();
                expected = State::Low;
                break;
            case 5U:
                Activate();
                Low();
                Emit(50000, 10ULL);
                Check(State::Low, false, true); // Recovery dwell has not elapsed.
                Poll(20ULL);
                Check(State::Low, false, true);
                Emit(50000, 10ULL);
                expected = State::Active;
                break;
            case 6U:
                Activate();
                Emit(90000, 10ULL);
                Check(State::WaitGap, false, false); // Pending HIGH is inhibited too.
                Emit(90000, 30ULL);
                Require(Current().reason == Reason::HighInhibit);
                expected = State::WaitGap;
                break;
            case 7U:
                Activate();
                Apply(Command::Hold);
                Check(State::Hold, false, false);
                Emit(50000, 10ULL);
                Check(State::Hold, false, false); // Fresh NORMAL cannot restart HOLD.
                Apply(Command::Start);
                Check(State::WaitGap, false, false);
                Poll(10ULL);
                Check(State::WaitGap, false, false); // Duplicate cannot release rearm.
                Emit(50000, 10ULL);
                expected = State::Active;
                break;
            case 8U:
                Activate();
                Apply(Command::Stop);
                Check(State::Idle, false, false);
                Emit(50000, 10ULL);
                expected = State::Idle;
                break;
            case 9U:
                Activate();
                Apply(Command::None, false);
                Require(Current().reason == Reason::Interlock);
                CheckFaultLatch();
                expected = State::Fault;
                break;
            case 10U:
                Activate();
                Publish(50000, 10ULL, false);
                Apply();
                Require(Current().reason == Reason::Quality);
                CheckFaultLatch();
                expected = State::Fault;
                break;
            case 11U:
                Activate();
                Poll(100ULL);
                Check(State::Active, true, false); // Exactly 100 ms remains fresh.
                Poll(1ULL);
                Require(Current().reason == Reason::Stale);
                expected = State::Fault;
                break;
            case 12U:
                Activate();
                Publish(50000, 10ULL, true, EDMGap::Source::PHYSICAL);
                Apply();
                Require(Current().reason == Reason::Source);
                expected = State::Fault;
                break;
            case 13U:
            {
                Activate();
                EDMGap::Config invalid{};
                invalid.lowExitMv = invalid.lowEnterMv;
                Require(!m_monitor.Configure(invalid, EDMGap::Source::SIMULATED));
                Apply();
                Require(Current().reason == Reason::Configuration);
                expected = State::Fault;
                break;
            }
            case 14U:
                Activate();
                --m_now;
                Apply();
                Require(Current().reason == Reason::Clock);
                expected = State::Fault;
                break;
            case 15U:
            {
                Activate();
                EDMGap::Snapshot future = m_monitor.Current();
                ++future.sequence;
                future.sampledAtMs = future.observedAtMs = m_now + 1ULL;
                m_model.Step(Command::None, future, m_now);
                CheckInvariant();
                Require(Current().reason == Reason::Clock);
                expected = State::Fault;
                break;
            }
            case 16U:
            {
                Activate();
                EDMGap::Snapshot replay = m_monitor.Current();
                --replay.sequence;
                m_model.Step(Command::None, replay, m_now);
                CheckInvariant();
                Require(Current().reason == Reason::Sequence);
                expected = State::Fault;
                break;
            }
            case 17U:
            {
                Activate();
                EDMGap::Snapshot mutation = m_monitor.Current();
                ++mutation.voltageMv;
                m_model.Step(Command::None, mutation, m_now);
                CheckInvariant();
                Require(Current().reason == Reason::Sequence);
                expected = State::Fault;
                break;
            }
            case 18U:
                Activate();
                Apply(Command::None, false);
                Check(State::Fault, false, false);
                Apply(Command::Reset);
                Emit(50000, 10ULL);
                Check(State::Idle, false, false);
                Apply(Command::Start);
                Check(State::WaitGap, false, false);
                Emit(50000, 10ULL);
                expected = State::Active;
                break;
            case 19U:
                Activate();
                Revoke();
                Check(State::Hold, false, false);
                Emit(50000, 10ULL);
                Check(State::Hold, false, false);
                Apply(Command::Start);
                Emit(50000, 10ULL);
                Check(State::Active, true, false);
                Apply(Command::None, false);
                Revoke();
                Check(State::Fault, false, false); // Revoke cannot erase a fault.
                expected = State::Fault;
                break;
            }
            const bool permit = expected == State::Active;
            const bool retreat = expected == State::Low;
            Check(expected, permit, retreat);
            result.expectedState = expected;
            result.actual = Current();
            result.passed = m_ok;
            return result;
        }

    private:
        void Prepare() noexcept
        {
            m_model.Reset();
            m_ok = m_monitor.Configure(EDMGap::Config{}, EDMGap::Source::SIMULATED);
            m_now = 1000ULL;
            m_sequence = 0ULL;
        }

        void Require(bool condition) noexcept { m_ok = m_ok && condition; }

        void CheckInvariant() noexcept
        {
            using namespace EDMProcessSimulation;
            const Snapshot& s = Current();
            Require(!s.PhysicalDischargeEnabled());
            Require(s.simulatedPermit == (s.state == State::Active));
            Require(s.retreatRequested == (s.state == State::Low));
            Require(s.feedIntent == (s.simulatedPermit ? FeedIntent::Feed :
                s.retreatRequested ? FeedIntent::Retreat : FeedIntent::Hold));
            Require(!(s.simulatedPermit && s.retreatRequested));
        }

        void Check(EDMProcessSimulation::State state, bool permit, bool retreat) noexcept
        {
            CheckInvariant();
            Require(Current().state == state && Current().simulatedPermit == permit &&
                Current().retreatRequested == retreat);
        }

        void Apply(EDMProcessSimulation::Command command = EDMProcessSimulation::Command::None,
            bool interlock = true) noexcept
        {
            m_model.Step(command, m_monitor.Current(), m_now, interlock);
            CheckInvariant();
        }

        void Publish(std::int32_t mv, std::uint64_t advanceMs, bool valid = true,
            EDMGap::Source source = EDMGap::Source::SIMULATED) noexcept
        {
            m_now += advanceMs;
            EDMGap::Sample sample{};
            sample.source = source;
            sample.sequence = ++m_sequence;
            sample.sampledAtMs = m_now;
            sample.voltageMv = mv;
            sample.valid = valid;
            m_monitor.Publish(sample, m_now);
        }

        void Emit(std::int32_t mv, std::uint64_t advanceMs) noexcept
        {
            Publish(mv, advanceMs);
            Apply();
        }

        void Poll(std::uint64_t advanceMs) noexcept
        {
            m_now += advanceMs;
            m_monitor.Poll(m_now);
            Apply();
        }

        void Activate() noexcept
        {
            Publish(50000, 0ULL);
            Apply(EDMProcessSimulation::Command::Start);
            Check(EDMProcessSimulation::State::WaitGap, false, false);
            Emit(50000, 30ULL);
            Check(EDMProcessSimulation::State::Active, true, false);
        }

        void Low() noexcept
        {
            Emit(20000, 10ULL);
            Check(EDMProcessSimulation::State::WaitGap, false, false);
            Emit(20000, 30ULL);
            Check(EDMProcessSimulation::State::Low, false, true);
        }

        void CheckFaultLatch() noexcept
        {
            using namespace EDMProcessSimulation;
            Check(State::Fault, false, false);
            Emit(50000, 10ULL);
            Check(State::Fault, false, false);
            Apply(Command::Start);
            Check(State::Fault, false, false);
            Apply(Command::Hold);
            Check(State::Fault, false, false);
            Apply(Command::Stop);
            Check(State::Fault, false, false);
        }

        EDMProcessSimulation::Controller m_model{};
        EDMGap::Monitor m_monitor{};
        std::uint64_t m_now = 1000ULL, m_sequence = 0ULL;
        std::uint32_t m_next = 0U;
        bool m_ok = true;
    };

    static_assert(sizeof(Scenario) <= 512U, "EDM simulation scenario storage budget");
}
