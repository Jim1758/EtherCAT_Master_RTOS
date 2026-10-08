#pragma once

#include "EDMAutomaticFlushCycle.h"
#include <cmath>
#include <cstdint>

// EDM23: deterministic fixtures for G180 P8. These values are independent of
// the installed COND and are never written into GAP, recipe, tuning or Motion.
namespace EDMAutomaticFlushSimulationTest
{
    constexpr std::uint32_t CaseCount = 22U;
    constexpr std::uint32_t MaximumModelCallsPerCase = 256U;

    inline const char* CaseName(std::uint32_t index) noexcept
    {
        static const char* const names[CaseCount] = {
            "WORK_TIME_GATE", "BASE_HEIGHT_PLAN", "SPEED_OVERRIDE", "DEEP_NTH_CYCLE",
            "SHORT_FREEZES_WORK", "WORK_GATE_NO_CATCHUP", "SHORT_BLOCKS_RETURN",
            "RETURN_GATE_NO_CATCHUP", "HOLD_WORK_EXPLICIT_RESUME", "HOLD_FLUSH_EXPLICIT_RESUME",
            "ZERO_HEIGHT_LOGICAL", "ZERO_WORK_REJECT", "ZERO_SPEED_PAUSED", "REVISION_FAULT_LATCH",
            "RECIPE_GENERATION_FAULT", "CLOCK_ROLLBACK", "ACTIVE_SERVICE_GAP", "INTERLOCK_FAULT",
            "REVOKE_REARM_EXPLICIT", "UNCONFIRMED_FRAME_REJECT", "UNSUPPORTED_MODE_REJECT", "INVALID_COMMAND"
        };
        return index < CaseCount ? names[index] : "INVALID_CASE";
    }

    struct Snapshot
    {
        std::uint32_t state = 0U, reason = 0U, segment = 0U, modelCalls = 0U;
        std::uint64_t completedCycles = 0ULL, workElapsedMs = 0ULL;
        double heightMm = 0.0, offsetMm = 0.0, speedMmMin = 0.0;
        bool deepCycle = false;
        constexpr bool PhysicalMotionEnabled() const noexcept { return false; }
        constexpr bool PhysicalDischargeEnabled() const noexcept { return false; }
    };

    struct CaseResult { bool passed = false; std::uint32_t checks = 0U; Snapshot actual{}; };

    class Scenario
    {
    public:
        void Reset() noexcept { m_next = 0U; m_revoked = false; Prepare(); }
        void Revoke() noexcept { m_revoked = true; m_cycle.Revoke(); Capture(); m_actual.speedMmMin = 0.0; }
        const Snapshot& Current() const noexcept { return m_actual; }

        CaseResult Step(std::uint32_t index) noexcept
        {
            using namespace EDM23;
            CaseResult result{};
            if (m_revoked || index != m_next || index >= CaseCount) { Revoke(); result.actual = Current(); return result; }
            ++m_next;
            Prepare();
            switch (index)
            {
            case 0U:
                Begin(); m_input.machiningAllowed = false; Tick(50ULL); Work(0ULL);
                m_input.machiningAllowed = true; Tick(50ULL); Work(0ULL);
                Tick(50ULL); Work(50ULL); Tick(50ULL); State(AutomaticFlushCycleState::Flushing);
                Work(100ULL); Near(m_cycle.Snapshot().virtualOffsetMm, 0.0); break;
            case 1U:
                Begin(); Trigger(); Near(m_cycle.Snapshot().effectiveHeightMm, 0.001);
                Require(!m_cycle.Snapshot().deepCycle); DrainFlush();
                Require(m_cycle.Snapshot().completedCycles == 1ULL); Work(0ULL);
                Near(m_cycle.Snapshot().virtualOffsetMm, 0.0); break;
            case 2U:
                m_config.jumpSpeedOverridePercent = 50.0; Begin(); Trigger(); Tick(1ULL);
                Near(m_cycle.Snapshot().requestedSpeedMmPerMin, 30.0);
                Near(m_cycle.Snapshot().virtualOffsetMm, 0.0005); DrainFlush(); break;
            case 3U:
                m_config.deepFlushCycleInterval = 3U; m_config.deepFlushHeightMultiplier = 3.0;
                Begin();
                for (std::uint32_t round = 0U; round < 3U; ++round)
                {
                    Trigger(); Require(m_cycle.Snapshot().deepCycle == (round == 2U));
                    Near(m_cycle.Snapshot().effectiveHeightMm, round == 2U ? 0.003 : 0.001);
                    DrainFlush(); Require(m_cycle.Snapshot().completedCycles == round + 1ULL);
                }
                break;
            case 4U:
                Begin(); Tick(40ULL); Work(40ULL); m_input.shortActive = true;
                Tick(100ULL); Work(40ULL); m_input.shortActive = false;
                Tick(100ULL); Work(40ULL); Tick(60ULL); State(AutomaticFlushCycleState::Flushing); break;
            case 5U:
                Begin(); Tick(40ULL); Work(40ULL); m_input.machiningAllowed = false;
                Tick(10000ULL); Work(40ULL); m_input.machiningAllowed = true;
                Tick(10000ULL); Work(40ULL); Tick(60ULL); State(AutomaticFlushCycleState::Flushing); break;
            case 6U:
                Begin(); Trigger(); ToReturn(); m_input.shortActive = true; Tick(1ULL);
                State(AutomaticFlushCycleState::ReturnBlocked); Near(m_cycle.Snapshot().requestedSpeedMmPerMin, 0.0);
                Near(m_cycle.Snapshot().virtualOffsetMm, 0.001); m_input.shortActive = false;
                Tick(10000ULL); Near(m_cycle.Snapshot().virtualOffsetMm, 0.001); DrainFlush(); break;
            case 7U:
                Begin(); Trigger(); ToReturn(); m_input.returnAllowed = false; Tick(1ULL);
                State(AutomaticFlushCycleState::ReturnBlocked); constOffset();
                Tick(10000ULL); constOffset(); m_input.returnAllowed = true; Tick(10000ULL); constOffset();
                DrainFlush(); break;
            case 8U:
                Begin(); Tick(40ULL); m_input.command = AutomaticFlushCycleCommand::Hold; Tick(1ULL);
                State(AutomaticFlushCycleState::Hold); Work(40ULL); m_input.command = AutomaticFlushCycleCommand::Tick;
                Tick(10000ULL); State(AutomaticFlushCycleState::Hold); Work(40ULL);
                m_input.command = AutomaticFlushCycleCommand::Resume; Tick(10000ULL); Work(40ULL);
                m_input.command = AutomaticFlushCycleCommand::Tick; Tick(60ULL); Work(40ULL); Tick(60ULL);
                State(AutomaticFlushCycleState::Flushing); break;
            case 9U:
                m_config.baseJumpHeightMm = 0.005; Begin(); Trigger(); Tick(1ULL);
                Near(m_cycle.Snapshot().virtualOffsetMm, 0.001);
                m_input.command = AutomaticFlushCycleCommand::Hold; Tick(1ULL);
                State(AutomaticFlushCycleState::Hold); Near(m_cycle.Snapshot().virtualOffsetMm, 0.001);
                m_input.command = AutomaticFlushCycleCommand::Tick; Tick(10000ULL);
                Near(m_cycle.Snapshot().virtualOffsetMm, 0.001);
                m_input.command = AutomaticFlushCycleCommand::Resume; Tick(10000ULL);
                Near(m_cycle.Snapshot().virtualOffsetMm, 0.001);
                m_input.command = AutomaticFlushCycleCommand::Tick; Tick(1ULL);
                Near(m_cycle.Snapshot().virtualOffsetMm, 0.002); DrainFlush(); break;
            case 10U:
                m_config.baseJumpHeightMm = 0.0; Begin(); Tick(100ULL);
                State(AutomaticFlushCycleState::Machining); Require(m_cycle.Snapshot().completedCycles == 1ULL);
                Work(0ULL); Near(m_cycle.Snapshot().virtualOffsetMm, 0.0);
                Run(); Tick(100ULL); Require(m_cycle.Snapshot().completedCycles == 2ULL); break;
            case 11U:
                m_config.workTimeMs = 0ULL; Require(!Start()); State(AutomaticFlushCycleState::Fault);
                Error(AutomaticFlushCycleError::InvalidConfig); break;
            case 12U:
                m_config.jumpSpeedOverridePercent = 0.0; Begin(); Tick(100ULL);
                State(AutomaticFlushCycleState::Machining); Work(100ULL);
                Tick(100ULL); Work(100ULL); Require(m_cycle.Snapshot().completedCycles == 0ULL);
                Near(m_cycle.Snapshot().requestedSpeedMmPerMin, 0.0); Near(m_cycle.Snapshot().virtualOffsetMm, 0.0); break;
            case 13U:
                Begin(); ++m_input.configRevision; Tick(1ULL); State(AutomaticFlushCycleState::Fault);
                Error(AutomaticFlushCycleError::ConfigRevisionChanged); --m_input.configRevision;
                Tick(1ULL); State(AutomaticFlushCycleState::Fault); Near(m_cycle.Snapshot().requestedSpeedMmPerMin, 0.0); break;
            case 14U:
                Begin(); ++m_input.recipeGeneration; Tick(1ULL); State(AutomaticFlushCycleState::Fault);
                Error(AutomaticFlushCycleError::RecipeGenerationChanged); break;
            case 15U:
                Begin(); Tick(20ULL); --m_now; Run(); State(AutomaticFlushCycleState::Fault);
                Error(AutomaticFlushCycleError::ClockRollback); break;
            case 16U:
                Begin(); Tick(251ULL); State(AutomaticFlushCycleState::Fault);
                Error(AutomaticFlushCycleError::ServiceGap); break;
            case 17U:
                Begin(); m_input.interlockReady = false; Tick(1ULL); State(AutomaticFlushCycleState::Fault);
                Error(AutomaticFlushCycleError::InterlockLost); m_input.interlockReady = true;
                Tick(1ULL); State(AutomaticFlushCycleState::Fault); break;
            case 18U:
                Begin(); Trigger(); m_cycle.Revoke(); CountCall(); State(AutomaticFlushCycleState::Cancelled);
                Tick(1ULL); State(AutomaticFlushCycleState::Cancelled);
                Require(!Start()); m_cycle.Reset(); CountCall(); Require(Start());
                State(AutomaticFlushCycleState::Machining); Near(m_cycle.Snapshot().virtualOffsetMm, 0.0); Work(0ULL); break;
            case 19U:
                m_config.flushRequest.frame.machiningDirectionConfirmed = false;
                Require(!Start()); State(AutomaticFlushCycleState::Fault); Error(AutomaticFlushCycleError::InvalidPlan); break;
            case 20U:
                m_config.flushRequest.mode = EDM20::FlushMode::B2;
                Require(!Start()); State(AutomaticFlushCycleState::Fault); Error(AutomaticFlushCycleError::InvalidConfig); break;
            case 21U:
                Begin(); m_input.command = static_cast<AutomaticFlushCycleCommand>(255U); Tick(1ULL);
                State(AutomaticFlushCycleState::Fault); Error(AutomaticFlushCycleError::InvalidCommand); break;
            default: Require(false); break;
            }
            Capture();
            Require(m_actual.modelCalls <= MaximumModelCallsPerCase);
            Require(std::isfinite(m_actual.offsetMm) && std::isfinite(m_actual.speedMmMin) && std::isfinite(m_actual.heightMm));
            Require(!m_actual.PhysicalMotionEnabled() && !m_actual.PhysicalDischargeEnabled());
            result.passed = m_passed; result.checks = m_checks; result.actual = m_actual;
            return result;
        }

    private:
        EDM23::AutomaticFlushCycle m_cycle{};
        EDM23::AutomaticFlushCycleConfig m_config{};
        EDM23::AutomaticFlushCycleInput m_input{};
        Snapshot m_actual{};
        std::uint64_t m_now = 0ULL;
        std::uint32_t m_next = 0U, m_checks = 0U, m_calls = 0U;
        bool m_revoked = false, m_passed = true;

        void Prepare() noexcept
        {
            m_cycle.Reset(); m_config = EDM23::AutomaticFlushCycleConfig{};
            m_config.workTimeMs = 100ULL; m_config.baseJumpHeightMm = 0.001;
            m_config.jumpSpeedOverridePercent = 100.0; m_config.deepFlushCycleInterval = 0U;
            m_config.deepFlushHeightMultiplier = 1.0; m_config.configRevision = 1ULL; m_config.recipeGeneration = 1ULL;
            m_config.flushProfile.mainRetractMmPerMin = 60.0; m_config.flushProfile.mainReturnMmPerMin = 60.0;
            m_config.flushProfile.finalApproachDistanceMm = 0.0;
            m_config.flushRequest.mode = EDM20::FlushMode::B0;
            m_config.flushRequest.jumpHeightConfirmed = true;
            m_config.flushRequest.frame.machiningDirectionConfirmed = true;
            m_config.flushRequest.frame.machineZDirectionConfirmed = true;
            m_input = EDM23::AutomaticFlushCycleInput{};
            m_input.configRevision = 1ULL; m_input.recipeGeneration = 1ULL;
            m_input.interlockReady = true; m_input.machiningAllowed = true; m_input.returnAllowed = true;
            m_now = 0ULL; m_checks = 0U; m_calls = 0U; m_passed = true; m_actual = Snapshot{};
        }
        void Require(bool value) noexcept { ++m_checks; m_passed = m_passed && value; }
        void Near(double actual, double expected) noexcept { Require(std::fabs(actual - expected) <= 1e-10); }
        void CountCall() noexcept { ++m_calls; Require(m_calls <= MaximumModelCallsPerCase); }
        bool Start() noexcept { CountCall(); return m_cycle.Start(m_config, m_now); }
        void Begin() noexcept { Require(Start()); State(EDM23::AutomaticFlushCycleState::Machining); Run(); }
        void Run() noexcept { CountCall(); m_input.nowMs = m_now; (void)m_cycle.Step(m_input); }
        void Tick(std::uint64_t deltaMs) noexcept { m_now += deltaMs; Run(); }
        void State(EDM23::AutomaticFlushCycleState state) noexcept { Require(m_cycle.Snapshot().state == state); }
        void Error(EDM23::AutomaticFlushCycleError error) noexcept { Require(m_cycle.Snapshot().error == error); }
        void Work(std::uint64_t elapsedMs) noexcept { Require(m_cycle.Snapshot().workElapsedMs == elapsedMs); }
        void Trigger() noexcept { Run(); Tick(100ULL); State(EDM23::AutomaticFlushCycleState::Flushing); }
        void constOffset() noexcept { Near(m_cycle.Snapshot().virtualOffsetMm, 0.001); Near(m_cycle.Snapshot().requestedSpeedMmPerMin, 0.0); }
        void ToReturn() noexcept
        {
            for (std::uint32_t i = 0U; i < 16U; ++i)
            {
                if (m_cycle.Snapshot().flushExecution.leg == EDM20::FlushLeg::Return) return;
                Tick(1ULL);
                if (m_cycle.Snapshot().state != EDM23::AutomaticFlushCycleState::Flushing) break;
            }
            Require(m_cycle.Snapshot().flushExecution.leg == EDM20::FlushLeg::Return);
        }
        void DrainFlush() noexcept
        {
            for (std::uint32_t i = 0U; i < 32U && m_cycle.Snapshot().state == EDM23::AutomaticFlushCycleState::Flushing; ++i) Tick(1ULL);
            State(EDM23::AutomaticFlushCycleState::Machining);
            Near(m_cycle.Snapshot().requestedSpeedMmPerMin, 0.0);
            Near(m_cycle.Snapshot().virtualOffsetMm, 0.0);
        }
        void Capture() noexcept
        {
            const auto& snapshot = m_cycle.Snapshot();
            m_actual.state = static_cast<std::uint32_t>(snapshot.state); m_actual.reason = static_cast<std::uint32_t>(snapshot.error);
            m_actual.segment = snapshot.flushExecution.segmentIndex; m_actual.modelCalls = m_calls;
            m_actual.completedCycles = snapshot.completedCycles; m_actual.workElapsedMs = snapshot.workElapsedMs;
            m_actual.heightMm = snapshot.effectiveHeightMm; m_actual.offsetMm = snapshot.virtualOffsetMm;
            m_actual.speedMmMin = snapshot.requestedSpeedMmPerMin; m_actual.deepCycle = snapshot.deepCycle;
        }
    };
}
