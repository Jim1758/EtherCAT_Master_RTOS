#pragma once

#include "EDMFlushExecution.h"
#include "EDMGapServo.h"
#include <cmath>
#include <cstdint>

// EDM22: bounded synthetic fixtures for G180 P7 and the identical HOST test.
// This type neither reads nor writes the live GAP, COND, tuning, axes or output.
// A fixture explicitly confirms mm and logical directions; it does not confirm
// the user's E6/E11 units or grant authority to a real machine.
namespace EDMShortFlushSimulationTest
{
    constexpr std::uint32_t CaseCount = 20U;
    constexpr std::uint32_t MaximumModelCallsPerCase = 256U;

    inline const char* CaseName(std::uint32_t index) noexcept
    {
        static const char* const names[CaseCount] = {
            "SHORT_ENTRY_DWELL", "SHORT_EXIT_HYSTERESIS", "ZERO_THRESHOLD",
            "SHORT_BAD_STREAM", "ZERO_RETREAT_NEGATIVE_ONLY", "FIXED_RETREAT_LIMIT",
            "B0_COMPLETE_APPROACH", "B1_SLOW_DIRECTION", "ZERO_SMALL_JUMP",
            "HOLD_EXPLICIT_RESUME", "RESET_REVOKE_LATCH", "RETURN_GATE_NO_CATCHUP",
            "SHORT_GATED_RETURN", "CLOCK_ROLLBACK", "RUNNING_SERVICE_GAP",
            "REVISION_FAULT_LATCH", "INTERLOCK_FAULT_LATCH", "UNSUPPORTED_B234",
            "UNCONFIRMED_PLAN_REJECT", "MALFORMED_PLAN_REJECT"
        };
        return index < CaseCount ? names[index] : "INVALID_CASE";
    }

    struct Snapshot
    {
        std::uint32_t state = 0U, reason = 0U, segment = 0U;
        std::uint32_t shortState = 0U, modelCalls = 0U;
        double offsetMm = 0.0, speedMmMin = 0.0;
        bool shortActive = false, feedInhibited = true;
        constexpr bool PhysicalMotionEnabled() const noexcept { return false; }
        constexpr bool PhysicalDischargeEnabled() const noexcept { return false; }
    };

    struct CaseResult
    {
        bool passed = false;
        std::uint32_t checks = 0U;
        Snapshot actual{};
    };

    class Scenario
    {
    public:
        void Reset() noexcept
        {
            m_next = 0U;
            m_revoked = false;
            Prepare();
        }

        void Revoke() noexcept
        {
            m_revoked = true;
            m_executor.Revoke();
            Capture();
            m_actual.speedMmMin = 0.0;
        }

        const Snapshot& Current() const noexcept { return m_actual; }

        CaseResult Step(std::uint32_t index) noexcept
        {
            using namespace EDMGapServo;
            using namespace EDM20;
            using namespace EDM22;
            CaseResult result{};
            if (m_revoked || index != m_next || index >= CaseCount)
            {
                Revoke();
                result.actual = Current();
                return result;
            }
            ++m_next;
            Prepare();
            switch (index)
            {
            case 0U:
                Short(9.0, 0ULL);
                ShortIs(ShortState::Entering, true, false);
                Short(9.0, 1ULL);
                ShortIs(ShortState::Entering, true, false);
                Short(9.0, 1ULL);
                ShortIs(ShortState::Active, true, true);
                break;
            case 1U:
                ActivateShort();
                Short(11.0, 1ULL); // Equality must not release hysteresis.
                ShortIs(ShortState::Active, true, true);
                Short(11.1, 1ULL);
                ShortIs(ShortState::Exiting, true, true);
                Short(11.1, 4ULL);
                ShortIs(ShortState::Exiting, true, true);
                Short(11.1, 1ULL);
                ShortIs(ShortState::Clear, false, false);
                break;
            case 2U:
                Short(0.1, 0ULL, true);
                ShortIs(ShortState::Clear, false, false);
                Short(0.0, 1ULL, true);
                ShortIs(ShortState::Entering, true, false);
                Short(0.0, 2ULL, true);
                ShortIs(ShortState::Active, true, true);
                Require(m_short.thresholdV == 0.0 && m_short.enabled);
                m_servo.shortMachiningEnabled = false;
                Short(0.0, 1ULL, true);
                ShortIs(ShortState::Clear, false, false);
                Require(!m_short.enabled);
                break;
            case 3U:
            {
                ActivateShort();
                Sample replay{};
                replay.valid = true; replay.voltage = 9.0;
                replay.sequence = m_sequence; replay.nowMs = m_now;
                Detector(replay, false);
                Require(!m_short.valid && m_short.error == Error::Sequence);
                Require(m_short.feedInhibited && !m_short.shortActive);
                Short(9.0, 0ULL);
                ShortIs(ShortState::Entering, true, false);
                replay.sequence = ++m_sequence; replay.nowMs = m_now - 1ULL;
                Detector(replay, false);
                Require(!m_short.valid && m_short.error == Error::Clock);
                Require(m_short.feedInhibited && !m_short.shortActive);
                break;
            }
            case 4U:
            {
                m_servo.shortRetreatMmPerMin = 0.0;
                const RetreatResult negative = Retreat(45.0);
                Require(negative.valid && negative.usingCurve);
                Near(negative.signedSpeedMmPerMin, -1.25);
                const RetreatResult positive = Retreat(55.0);
                Require(positive.valid && positive.usingCurve);
                Near(positive.signedSpeedMmPerMin, 0.0);
                const RetreatResult zero = Retreat(50.0);
                Require(zero.valid && zero.usingCurve);
                Near(zero.signedSpeedMmPerMin, 0.0);
                break;
            }
            case 5U:
            {
                m_servo.shortRetreatMmPerMin = 50.0;
                const RetreatResult fixed = Retreat(55.0);
                Require(fixed.valid && !fixed.usingCurve && fixed.saturated);
                Near(fixed.signedSpeedMmPerMin, -5.0);
                m_servo.shortRetreatMmPerMin = 0.5;
                const RetreatResult normal = Retreat(55.0);
                Require(normal.valid && !normal.saturated);
                Near(normal.signedSpeedMmPerMin, -0.5);
                break;
            }
            case 6U:
                StartPlan(FlushMode::B0, 0.1);
                Require(m_plan.plan.count == 3U);
                Require(m_plan.plan.segments[0].direction == FlushDirection::OppositeMachining);
                Require(m_plan.plan.segments[1].direction == FlushDirection::AlongMachining);
                RunComplete();
                Require(m_sawBoundary && m_sawApproach && !m_sawSlow);
                Near(m_executor.Snapshot().retreatTravelledMm, 0.1);
                Near(m_executor.Snapshot().returnTravelledMm, 0.1);
                break;
            case 7U:
                m_flush.initialSlowDistanceMm = 0.02;
                StartPlan(FlushMode::B1, 0.1);
                Require(m_plan.plan.count == 4U);
                Require(m_plan.plan.segments[0].kind == FlushSegmentKind::InitialSlowRetreat);
                Require(m_plan.plan.segments[0].direction == FlushDirection::MachinePositiveZ);
                Require(m_plan.plan.segments[2].direction == FlushDirection::MachineNegativeZ);
                RunComplete();
                Require(m_sawBoundary && m_sawApproach && m_sawSlow);
                break;
            case 8U:
                StartPlan(FlushMode::B0, 0.0);
                Require(m_plan.plan.count == 0U);
                Require(m_executor.Snapshot().state == FlushExecutionState::Complete);
                Near(m_executor.Snapshot().offsetMm, 0.0);
                ResetExecutor();
                m_flush.initialSlowDistanceMm = 0.02;
                StartPlan(FlushMode::B1, 0.005);
                Require(m_plan.plan.count == 2U);
                Near(m_plan.plan.segments[0].distanceMm, 0.005);
                Near(m_plan.plan.segments[1].distanceMm, 0.005);
                Require(m_plan.plan.segments[1].kind == FlushSegmentKind::FinalApproach);
                RunComplete();
                break;
            case 9U:
            {
                StartPlan(FlushMode::B0, 0.1);
                Tick(100ULL);
                const double saved = m_executor.Snapshot().offsetMm;
                const std::size_t segment = m_executor.Snapshot().segmentIndex;
                Tick(100ULL, true, FlushExecutionCommand::Hold);
                Require(m_executor.Snapshot().state == FlushExecutionState::Hold);
                Near(m_executor.Snapshot().offsetMm, saved);
                Tick(10000ULL);
                Require(m_executor.Snapshot().state == FlushExecutionState::Hold);
                Near(m_executor.Snapshot().requestedSpeedMmPerMin, 0.0);
                Near(m_executor.Snapshot().offsetMm, saved);
                Tick(10000ULL, true, FlushExecutionCommand::Resume);
                Require(m_executor.Snapshot().state == FlushExecutionState::Running);
                Near(m_executor.Snapshot().offsetMm, saved);
                Require(m_executor.Snapshot().segmentIndex == segment);
                Tick(100ULL);
                Require(m_executor.Snapshot().offsetMm > saved);
                RunComplete();
                break;
            }
            case 10U:
                StartPlan(FlushMode::B0, 0.1);
                Tick(100ULL);
                ResetExecutor();
                Tick(100ULL);
                Require(m_executor.Snapshot().state == FlushExecutionState::Idle);
                Near(m_executor.Snapshot().offsetMm, 0.0);
                StartPlan(FlushMode::B0, 0.1);
                Tick(100ULL);
                CountCall(); m_executor.Revoke(); Capture();
                Require(m_executor.Snapshot().state == FlushExecutionState::Cancelled);
                Tick(100ULL);
                Require(m_executor.Snapshot().state == FlushExecutionState::Cancelled);
                CountCall();
                Require(!m_executor.Start(m_plan.plan, Revision, m_now, true, true));
                Capture();
                Require(m_executor.Snapshot().state != FlushExecutionState::Running);
                Near(m_executor.Snapshot().requestedSpeedMmPerMin, 0.0);
                break;
            case 11U:
            {
                StartPlan(FlushMode::B0, 0.1, false);
                ReachReturnBlocked();
                const double saved = m_executor.Snapshot().offsetMm;
                Tick(10000ULL, false);
                Require(m_executor.Snapshot().state == FlushExecutionState::ReturnBlocked);
                Near(m_executor.Snapshot().offsetMm, saved);
                Tick(10000ULL, true);
                Require(m_executor.Snapshot().state == FlushExecutionState::Running);
                Near(m_executor.Snapshot().offsetMm, saved);
                Tick(100ULL, true);
                Near(m_executor.Snapshot().offsetMm, saved - 0.01);
                RunComplete();
                break;
            }
            case 12U:
            {
                ActivateShort();
                StartPlan(FlushMode::B0, 0.1, !m_short.feedInhibited);
                ReachReturnBlocked();
                const double saved = m_executor.Snapshot().offsetMm;
                Short(11.1, 0ULL);
                ShortIs(ShortState::Exiting, true, true);
                Tick(0ULL, !m_short.feedInhibited);
                Require(m_executor.Snapshot().state == FlushExecutionState::ReturnBlocked);
                Short(11.1, 5ULL);
                ShortIs(ShortState::Clear, false, false);
                Tick(0ULL, !m_short.feedInhibited);
                Require(m_executor.Snapshot().state == FlushExecutionState::Running);
                Near(m_executor.Snapshot().offsetMm, saved);
                Tick(100ULL);
                Near(m_executor.Snapshot().offsetMm, saved - 0.01);
                RunComplete();
                break;
            }
            case 13U:
                StartPlan(FlushMode::B0, 0.1);
                Tick(100ULL);
                --m_now;
                Tick(0ULL);
                FaultIs(FlushExecutionError::ClockRollback);
                break;
            case 14U:
                StartPlan(FlushMode::B0, 0.1);
                Tick(251ULL);
                FaultIs(FlushExecutionError::ServiceGap);
                break;
            case 15U:
                StartPlan(FlushMode::B0, 0.1);
                Tick(100ULL, true, FlushExecutionCommand::Tick, Revision + 1ULL);
                FaultIs(FlushExecutionError::ConfigRevisionChanged);
                Tick(100ULL);
                Require(m_executor.Snapshot().state == FlushExecutionState::Fault);
                Near(m_executor.Snapshot().requestedSpeedMmPerMin, 0.0);
                break;
            case 16U:
                StartPlan(FlushMode::B0, 0.1);
                Tick(100ULL, true, FlushExecutionCommand::Tick, Revision, false);
                FaultIs(FlushExecutionError::InterlockLost);
                Tick(100ULL);
                Require(m_executor.Snapshot().state == FlushExecutionState::Fault);
                Near(m_executor.Snapshot().requestedSpeedMmPerMin, 0.0);
                break;
            case 17U:
                Build(FlushMode::B2, 0.1);
                Require(m_plan.status == FlushPlanStatus::NeedsPathGeometry && m_plan.plan.count == 0U);
                Build(FlushMode::B3, 0.1);
                Require(m_plan.status == FlushPlanStatus::NeedsOrbitGeometry && m_plan.plan.count == 0U);
                Build(FlushMode::B4, 0.1);
                Require(m_plan.status == FlushPlanStatus::NeedsOrbitGeometry && m_plan.plan.count == 0U);
                break;
            case 18U:
                Build(FlushMode::B0, 0.1, false, true);
                Require(m_plan.status == FlushPlanStatus::UnconfirmedJumpUnits);
                Build(FlushMode::B0, 0.1, true, false);
                Require(m_plan.status == FlushPlanStatus::NeedsAxisFrame);
                m_flush.pathDistanceMm = 0.001;
                Build(FlushMode::B0, 0.1);
                Require(m_plan.status == FlushPlanStatus::NeedsPathGeometry);
                break;
            case 19U:
                Build(FlushMode::B0, 0.1);
                m_plan.plan.totalReturnMm = 0.2;
                CountCall();
                Require(!m_executor.Start(m_plan.plan, Revision, m_now, true, true));
                Capture();
                FaultIs(FlushExecutionError::InvalidPlan);
                Tick(100ULL);
                Require(m_executor.Snapshot().state == FlushExecutionState::Fault);
                break;
            }
            Capture();
            Require(m_calls <= MaximumModelCallsPerCase);
            result.actual = Current();
            result.checks = m_checks;
            result.passed = m_ok;
            return result;
        }

    private:
        static constexpr std::uint64_t Revision = 7ULL;

        void Prepare() noexcept
        {
            m_ok = true; m_checks = 0U; m_calls = 0U;
            m_now = 1000ULL; m_sequence = 0ULL;
            m_sawBoundary = false; m_sawApproach = false; m_sawSlow = false;
            m_servo = EDMGapServo::Profile{};
            m_flush = EDM20::FlushProfile{};
            m_flush.mainRetractMmPerMin = 6.0;
            m_flush.mainReturnMmPerMin = 6.0;
            m_flush.finalApproachMmPerMin = 3.0;
            m_flush.initialSlowMmPerMin = 3.0;
            m_flush.finalApproachDistanceMm = 0.02;
            m_short = EDMGapServo::ShortResult{};
            m_actual = Snapshot{};
            CountCall(); m_detector.Reset();
            ResetExecutor();
        }

        void Require(bool value) noexcept { ++m_checks; m_ok = value && m_ok; }
        void Near(double actual, double expected) noexcept
        { Require(std::isfinite(actual) && std::fabs(actual - expected) <= 1.0e-10); }
        void CountCall() noexcept { ++m_calls; Require(m_calls <= MaximumModelCallsPerCase); }

        void Capture() noexcept
        {
            const EDM22::FlushExecutionSnapshot& s = m_executor.Snapshot();
            m_actual.state = static_cast<std::uint32_t>(s.state);
            m_actual.reason = static_cast<std::uint32_t>(s.error);
            m_actual.segment = static_cast<std::uint32_t>(s.segmentIndex);
            m_actual.offsetMm = s.offsetMm;
            m_actual.speedMmMin = s.requestedSpeedMmPerMin;
            m_actual.shortState = static_cast<std::uint32_t>(m_short.state);
            m_actual.shortActive = m_short.shortActive;
            m_actual.feedInhibited = m_short.feedInhibited;
            m_actual.modelCalls = m_calls;
            Require(!s.PhysicalMotionEnabled() && !s.PhysicalDischargeEnabled());
            Require(!m_actual.PhysicalMotionEnabled() && !m_actual.PhysicalDischargeEnabled());
            Require(std::isfinite(s.offsetMm) && std::isfinite(s.requestedSpeedMmPerMin));
            Require(s.offsetMm >= -1.0e-10 && s.offsetMm <= 0.1000000001);
            if (s.state != EDM22::FlushExecutionState::Running)
                Near(s.requestedSpeedMmPerMin, 0.0);
        }

        void Detector(const EDMGapServo::Sample& sample, bool machining) noexcept
        {
            CountCall(); m_short = m_detector.Step(m_servo, sample, machining);
            Capture();
        }
        void Short(double voltage, std::uint64_t elapsed, bool machining = false) noexcept
        {
            m_now += elapsed;
            EDMGapServo::Sample sample{};
            sample.valid = true; sample.voltage = voltage;
            sample.sequence = ++m_sequence; sample.nowMs = m_now;
            Detector(sample, machining);
        }
        void ShortIs(EDMGapServo::ShortState state, bool inhibited, bool active) noexcept
        {
            Require(m_short.valid && m_short.state == state);
            Require(m_short.feedInhibited == inhibited && m_short.shortActive == active);
        }
        void ActivateShort() noexcept
        {
            Short(9.0, 0ULL);
            ShortIs(EDMGapServo::ShortState::Entering, true, false);
            Short(9.0, 2ULL);
            ShortIs(EDMGapServo::ShortState::Active, true, true);
        }
        EDMGapServo::RetreatResult Retreat(double voltage) noexcept
        {
            CountCall();
            const EDMGapServo::Result curve = EDMGapServo::Evaluate(m_servo, voltage, 50.0, 100.0);
            Require(curve.valid);
            CountCall();
            return EDMGapServo::EvaluateShortRetreat(m_servo, curve);
        }

        void Build(EDM20::FlushMode mode, double height,
            bool confirmedUnits = true, bool confirmedFrame = true) noexcept
        {
            EDM20::FlushRequest request{};
            request.mode = mode; request.jumpHeightMm = height;
            request.jumpHeightConfirmed = confirmedUnits;
            request.frame.machiningDirectionConfirmed = confirmedFrame;
            request.frame.machineZDirectionConfirmed = confirmedFrame;
            CountCall(); m_plan = EDM20::BuildFlushPlan(m_flush, request);
            Require(!m_plan.plan.PhysicalMotionEnabled());
        }
        void StartPlan(EDM20::FlushMode mode, double height, bool allowed = true) noexcept
        {
            Build(mode, height);
            Require(m_plan.status == EDM20::FlushPlanStatus::ReadyLogicalOnly);
            CountCall();
            Require(m_executor.Start(m_plan.plan, Revision, m_now, allowed, true));
            Capture();
            ObserveSegment();
        }
        void ResetExecutor() noexcept
        { CountCall(); m_executor.Reset(); Capture(); }

        void Tick(std::uint64_t elapsed, bool allowed = true,
            EDM22::FlushExecutionCommand command = EDM22::FlushExecutionCommand::Tick,
            std::uint64_t revision = Revision, bool interlock = true) noexcept
        {
            m_now += elapsed;
            EDM22::FlushExecutionInput input{};
            input.nowMs = m_now; input.configRevision = revision;
            input.returnAllowed = allowed; input.command = command;
            input.interlockReady = interlock;
            CountCall(); m_executor.Step(input); Capture(); ObserveSegment();
        }
        void ObserveSegment() noexcept
        {
            const EDM22::FlushExecutionSnapshot& s = m_executor.Snapshot();
            if (s.legBoundary)
            {
                m_sawBoundary = true;
                Near(s.requestedSpeedMmPerMin, 0.0);
            }
            if (s.state == EDM22::FlushExecutionState::Running)
            {
                if (s.kind == EDM20::FlushSegmentKind::InitialSlowRetreat) m_sawSlow = true;
                if (s.kind == EDM20::FlushSegmentKind::FinalApproach)
                {
                    m_sawApproach = true;
                    if (!s.legBoundary) Near(std::fabs(s.requestedSpeedMmPerMin), 3.0);
                }
            }
        }
        void RunComplete() noexcept
        {
            for (unsigned i = 0U; i < 96U &&
                m_executor.Snapshot().state == EDM22::FlushExecutionState::Running; ++i)
                Tick(100ULL);
            Require(m_executor.Snapshot().state == EDM22::FlushExecutionState::Complete);
            Near(m_executor.Snapshot().offsetMm, 0.0);
            Near(m_executor.Snapshot().requestedSpeedMmPerMin, 0.0);
        }
        void ReachReturnBlocked() noexcept
        {
            for (unsigned i = 0U; i < 96U &&
                m_executor.Snapshot().state == EDM22::FlushExecutionState::Running; ++i)
                Tick(100ULL, false);
            Require(m_executor.Snapshot().state == EDM22::FlushExecutionState::ReturnBlocked);
            Near(m_executor.Snapshot().offsetMm, 0.1);
            Near(m_executor.Snapshot().requestedSpeedMmPerMin, 0.0);
        }
        void FaultIs(EDM22::FlushExecutionError error) noexcept
        {
            Require(m_executor.Snapshot().state == EDM22::FlushExecutionState::Fault);
            Require(m_executor.Snapshot().error == error);
            Near(m_executor.Snapshot().requestedSpeedMmPerMin, 0.0);
        }

        EDM22::FlushExecutor m_executor{};
        EDMGapServo::ShortDetector m_detector{};
        EDMGapServo::Profile m_servo{};
        EDM20::FlushProfile m_flush{};
        EDM20::FlushPlanResult m_plan{};
        EDMGapServo::ShortResult m_short{};
        Snapshot m_actual{};
        std::uint64_t m_now = 1000ULL, m_sequence = 0ULL;
        std::uint32_t m_next = 0U, m_checks = 0U, m_calls = 0U;
        bool m_ok = true, m_revoked = true;
        bool m_sawBoundary = false, m_sawApproach = false, m_sawSlow = false;
    };

    static_assert(sizeof(Scenario) <= 2048U, "EDM22 fixed scenario storage budget");
}
