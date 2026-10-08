#pragma once

#include "EDMProcessSimulation.h"
#include <cstdint>
#include <limits>

// EDM02: a same-thread, SIMULATED-ONLY command/proof handshake. Virtual units
// have no axis, pulse, speed, or machining interpretation. In particular the
// instantaneous Stop proof below is NOT a physical deceleration/settle proof.
// No Motion/PLC/I/O dependency or physical-discharge capability exists here.
namespace EDMFeedRetreatSimulation
{
    enum class Phase : std::uint8_t
    { Idle, WaitGap, Feeding, WaitStop, Retreating, WaitNormal, Returning, ReturnStopped, Hold, Fault };
    enum class Action : std::uint8_t { None, Stop, Feed, Retreat, Return };
    enum class Ack : std::uint8_t { Progress, Stopped, Complete };
    enum class Reason : std::uint8_t
    {
        AwaitStart, FreshNormal, Feeding, StopRequired, Retreating, WaitingNormal,
        Returning, ReturnInterrupted, OperatorHold, Stopped, Revoked, GapFault,
        Scope, Token, Publication, FeedbackClock, FeedbackStale, FeedbackShape,
        Position, Timeout, SequenceExhausted
    };
    struct Scope
    {
        std::uint64_t session = 0ULL, owner = 0ULL, epoch = 0ULL;
    };
    inline bool SameScope(const Scope& a, const Scope& b) noexcept
    { return a.session == b.session && a.owner == b.owner && a.epoch == b.epoch; }
    struct Feedback
    {
        Scope scope{};
        std::uint64_t commandToken = 0ULL, publicationSequence = 0ULL, observedAtMs = 0ULL;
        Action action = Action::None;
        Ack ack = Ack::Progress;
        std::int32_t virtualPosition = 0;
        bool valid = false;
    };
    struct Snapshot
    {
        Scope scope{};
        Phase phase = Phase::Idle;
        Action action = Action::None;
        Reason reason = Reason::AwaitStart;
        EDMProcessSimulation::State processState = EDMProcessSimulation::State::Idle;
        EDMProcessSimulation::Reason processReason = EDMProcessSimulation::Reason::AwaitStart;
        std::int32_t virtualPosition = 500, targetPosition = 500, simulatedVelocity = 0;
        std::uint64_t commandToken = 0ULL, observedAtMs = 0ULL;
        bool simulatedPermit = false, retreatRequested = false;
        constexpr bool PhysicalDischargeEnabled() const noexcept { return false; }
    };
    inline const char* PhaseName(Phase value) noexcept
    {
        switch (value)
        {
        case Phase::Idle: return "IDLE"; case Phase::WaitGap: return "WAIT_GAP";
        case Phase::Feeding: return "FEEDING"; case Phase::WaitStop: return "WAIT_STOP";
        case Phase::Retreating: return "RETREATING"; case Phase::WaitNormal: return "WAIT_NORMAL";
        case Phase::Returning: return "RETURNING"; case Phase::ReturnStopped: return "RETURN_STOPPED";
        case Phase::Hold: return "HOLD"; case Phase::Fault: return "FAULT";
        }
        return "UNKNOWN";
    }
    inline const char* ActionName(Action value) noexcept
    {
        switch (value)
        {
        case Action::None: return "NONE"; case Action::Stop: return "STOP";
        case Action::Feed: return "FEED"; case Action::Retreat: return "RETREAT";
        case Action::Return: return "RETURN";
        }
        return "UNKNOWN";
    }
    inline const char* ReasonName(Reason value) noexcept
    {
        switch (value)
        {
        case Reason::AwaitStart: return "AWAIT_START"; case Reason::FreshNormal: return "FRESH_NORMAL";
        case Reason::Feeding: return "FEEDING"; case Reason::StopRequired: return "STOP_REQUIRED";
        case Reason::Retreating: return "RETREATING"; case Reason::WaitingNormal: return "WAIT_NORMAL";
        case Reason::Returning: return "RETURNING"; case Reason::ReturnInterrupted: return "RETURN_INTERRUPTED";
        case Reason::OperatorHold: return "OPERATOR_HOLD"; case Reason::Stopped: return "STOPPED";
        case Reason::Revoked: return "REVOKED"; case Reason::GapFault: return "GAP_FAULT";
        case Reason::Scope: return "SCOPE"; case Reason::Token: return "TOKEN";
        case Reason::Publication: return "PUBLICATION"; case Reason::FeedbackClock: return "FEEDBACK_CLOCK";
        case Reason::FeedbackStale: return "FEEDBACK_STALE"; case Reason::FeedbackShape: return "FEEDBACK_SHAPE";
        case Reason::Position: return "POSITION"; case Reason::Timeout: return "TIMEOUT";
        case Reason::SequenceExhausted: return "SEQUENCE_EXHAUSTED";
        }
        return "UNKNOWN";
    }

    class Coordinator
    {
    public:
        static constexpr std::int32_t MinimumPosition = 0, MaximumPosition = 1000;
        static constexpr std::int32_t InitialPosition = 500, RetreatDistance = 100;
        static constexpr std::uint64_t MaximumFeedbackAgeMs = 100ULL;

        // Reset changes the proof scope, disarms, and clears the virtual travel.
        // Command tokens never rewind, even if a caller reuses a scope by mistake.
        void Reset(const Scope& scope) noexcept
        {
            m_process.Reset();
            m_current = Snapshot{};
            m_current.scope = scope;
            m_running = m_cycle = m_returnInterrupted = m_stopped = false;
            m_lastPublication = m_lastProofAt = m_freshSequence = m_freshTime = 0ULL;
            m_heldPosition = m_retreatPosition = InitialPosition;
        }
        const Snapshot& Current() const noexcept { return m_current; }
        void Revoke() noexcept
        {
            m_process.Revoke();
            m_running = false;
            if (m_current.phase == Phase::Fault) return;
            m_current.processState = m_process.Current().state;
            m_current.processReason = m_process.Current().reason;
            Issue(Action::Stop, Phase::Hold, Reason::Revoked, m_current.virtualPosition);
        }

        // feedback is optional, one-shot, scope/token/time fenced, and consumed
        // before any new command is issued. Repeating even an identical proof is
        // a protocol fault. All failures latch until an explicit Reset.
        const Snapshot& Step(EDMProcessSimulation::Command command,
            const EDMGap::Snapshot& gap, std::uint64_t nowMs,
            const Feedback* feedback = nullptr, bool interlockReady = true) noexcept
        {
            using Command = EDMProcessSimulation::Command;
            if (command == Command::Reset)
            {
                const Scope scope = m_current.scope;
                Reset(scope);
                return m_current;
            }
            if (m_current.phase == Phase::Fault) return m_current;
            const EDMProcessSimulation::Snapshot& process = m_process.Step(command, gap, nowMs, interlockReady);
            m_current.observedAtMs = nowMs;
            m_current.processState = process.state;
            m_current.processReason = process.reason;
            m_current.simulatedPermit = false;
            if (process.state == EDMProcessSimulation::State::Fault) return Fail(Reason::GapFault);
            if (command == Command::Hold || command == Command::Stop)
            {
                m_running = false;
                m_cycle = m_returnInterrupted = false;
                Issue(Action::Stop, command == Command::Hold ? Phase::Hold : Phase::Idle,
                    command == Command::Hold ? Reason::OperatorHold : Reason::Stopped, m_current.virtualPosition);
                return m_current;
            }
            if (command == Command::Start)
            {
                if (!ValidScope()) return Fail(Reason::Scope);
                // START cannot silently rebase a live excursion. A held/stopped
                // simulation starts a new bounded cycle from its last proof.
                if (m_running) return Fail(Reason::FeedbackShape);
                m_running = true;
                m_cycle = m_returnInterrupted = false;
                Fence(gap, nowMs);
                Issue(Action::Stop, Phase::WaitStop, Reason::StopRequired, m_current.virtualPosition);
                // An old proof accompanying START may not acknowledge this token.
                if (feedback != nullptr) return Fail(Reason::Token);
                return m_current;
            }
            // A newly timestamped proof cannot erase an already exceeded
            // command/progress deadline.
            if (m_running && !m_stopped && nowMs - m_lastProofAt > MaximumFeedbackAgeMs)
                return Fail(Reason::Timeout);
            bool complete = false;
            if (feedback != nullptr && !Accept(*feedback, nowMs, complete)) return m_current;
            if (!m_running) return m_current;

            const bool normal = process.simulatedPermit;
            const bool low = process.retreatRequested;
            const bool pendingLow = gap.pendingBand == EDMGap::Band::LOW;
            const bool inhibited = !normal;
            switch (m_current.phase)
            {
            case Phase::WaitStop:
                if (!m_stopped) break;
                if (m_returnInterrupted)
                {
                    Fence(gap, nowMs);
                    m_current.phase = Phase::ReturnStopped;
                    m_current.reason = Reason::ReturnInterrupted;
                }
                else if (low)
                {
                    m_heldPosition = m_current.virtualPosition;
                    if (m_heldPosition - MinimumPosition < RetreatDistance) return Fail(Reason::Position);
                    m_retreatPosition = m_heldPosition - RetreatDistance;
                    m_cycle = true;
                    Issue(Action::Retreat, Phase::Retreating, Reason::Retreating, m_retreatPosition);
                }
                else if (normal && FreshNormal(gap))
                    Issue(Action::Feed, Phase::Feeding, Reason::Feeding, MaximumPosition);
                break;
            case Phase::Feeding:
                if (inhibited)
                {
                    Fence(gap, nowMs);
                    Issue(Action::Stop, Phase::WaitStop, Reason::StopRequired, m_current.virtualPosition);
                }
                break;
            case Phase::Retreating:
                if (complete)
                {
                    m_stopped = true;
                    Fence(gap, nowMs);
                    m_current.phase = Phase::WaitNormal;
                    m_current.action = Action::None;
                    m_current.reason = Reason::WaitingNormal;
                    m_current.simulatedVelocity = 0;
                    m_current.retreatRequested = false;
                }
                break;
            case Phase::WaitNormal:
                if (normal && FreshNormal(gap))
                    Issue(Action::Return, Phase::Returning, Reason::Returning, m_heldPosition);
                break;
            case Phase::Returning:
                // LOW wins over a simultaneous endpoint completion. The old
                // return fence survives the stop and must be completed again.
                if (inhibited || pendingLow)
                {
                    m_returnInterrupted = true;
                    Fence(gap, nowMs);
                    Issue(Action::Stop, Phase::WaitStop, Reason::ReturnInterrupted, m_current.virtualPosition);
                }
                else if (complete)
                {
                    m_stopped = true;
                    m_cycle = m_returnInterrupted = false;
                    Fence(gap, nowMs);
                    m_current.phase = Phase::WaitGap;
                    m_current.action = Action::None;
                    m_current.reason = Reason::FreshNormal;
                    m_current.simulatedVelocity = 0;
                }
                break;
            case Phase::ReturnStopped:
                if (normal && FreshNormal(gap))
                    Issue(Action::Return, Phase::Returning, Reason::Returning, m_heldPosition);
                break;
            case Phase::WaitGap:
                if (normal && FreshNormal(gap))
                    Issue(Action::Feed, Phase::Feeding, Reason::Feeding, MaximumPosition);
                break;
            default: break;
            }
            m_current.simulatedPermit = m_current.phase == Phase::Feeding && normal && !m_cycle;
            return m_current;
        }

    private:
        bool ValidScope() const noexcept
        { return m_current.scope.session != 0ULL && m_current.scope.owner != 0ULL && m_current.scope.epoch != 0ULL; }
        void Fence(const EDMGap::Snapshot& gap, std::uint64_t nowMs) noexcept
        { m_freshSequence = gap.sequence; m_freshTime = nowMs; }
        bool FreshNormal(const EDMGap::Snapshot& gap) const noexcept
        { return gap.sequence > m_freshSequence && gap.sampledAtMs > m_freshTime; }
        void Issue(Action action, Phase phase, Reason reason, std::int32_t target) noexcept
        {
            if (m_nextToken == (std::numeric_limits<std::uint64_t>::max)())
            {
                m_running = false;
                m_current.phase = Phase::Fault;
                m_current.action = Action::Stop;
                m_current.reason = Reason::SequenceExhausted;
                m_current.simulatedPermit = m_current.retreatRequested = false;
                m_current.simulatedVelocity = 0;
                return;
            }
            m_current.commandToken = ++m_nextToken;
            m_current.action = action;
            m_current.phase = phase;
            m_current.reason = reason;
            m_current.targetPosition = target;
            m_current.simulatedVelocity = action == Action::Retreat ? -1 :
                (action == Action::Feed || action == Action::Return ? 1 : 0);
            m_current.retreatRequested = action == Action::Retreat;
            m_current.simulatedPermit = false;
            m_stopped = false;
            m_lastProofAt = m_current.observedAtMs;
        }
        const Snapshot& Fail(Reason reason) noexcept
        {
            m_process.Revoke();
            m_running = false;
            Issue(Action::Stop, Phase::Fault, reason, m_current.virtualPosition);
            return m_current;
        }
        bool Accept(const Feedback& feedback, std::uint64_t nowMs, bool& complete) noexcept
        {
            if (!SameScope(feedback.scope, m_current.scope)) { Fail(Reason::Scope); return false; }
            if (feedback.commandToken == 0ULL || feedback.commandToken != m_current.commandToken)
            { Fail(Reason::Token); return false; }
            if (feedback.publicationSequence == 0ULL || feedback.publicationSequence <= m_lastPublication)
            { Fail(Reason::Publication); return false; }
            if (feedback.observedAtMs > nowMs || feedback.observedAtMs < m_lastProofAt)
            { Fail(Reason::FeedbackClock); return false; }
            if (nowMs - feedback.observedAtMs > MaximumFeedbackAgeMs)
            { Fail(Reason::FeedbackStale); return false; }
            if (!feedback.valid || feedback.action != m_current.action || m_current.action == Action::None || m_stopped)
            { Fail(Reason::FeedbackShape); return false; }
            const std::int32_t position = feedback.virtualPosition;
            if (position < MinimumPosition || position > MaximumPosition)
            { Fail(Reason::Position); return false; }
            switch (m_current.action)
            {
            case Action::Stop:
                if (feedback.ack != Ack::Stopped || position != m_current.virtualPosition)
                { Fail(Reason::FeedbackShape); return false; }
                m_stopped = true;
                break;
            case Action::Feed:
                if (feedback.ack != Ack::Progress || position < m_current.virtualPosition)
                { Fail(Reason::FeedbackShape); return false; }
                break;
            case Action::Retreat:
            case Action::Return:
                if (feedback.ack != Ack::Progress && feedback.ack != Ack::Complete)
                { Fail(Reason::FeedbackShape); return false; }
                if ((m_current.action == Action::Retreat &&
                    (position > m_current.virtualPosition || position < m_current.targetPosition)) ||
                    (m_current.action == Action::Return &&
                    (position < m_current.virtualPosition || position > m_current.targetPosition)) ||
                    (feedback.ack == Ack::Complete && position != m_current.targetPosition))
                { Fail(Reason::Position); return false; }
                complete = feedback.ack == Ack::Complete;
                break;
            default: Fail(Reason::FeedbackShape); return false;
            }
            m_current.virtualPosition = position;
            m_lastPublication = feedback.publicationSequence;
            m_lastProofAt = feedback.observedAtMs;
            return true;
        }
        EDMProcessSimulation::Controller m_process{};
        Snapshot m_current{};
        std::uint64_t m_nextToken = 0ULL, m_lastPublication = 0ULL, m_lastProofAt = 0ULL;
        std::uint64_t m_freshSequence = 0ULL, m_freshTime = 0ULL;
        std::int32_t m_heldPosition = InitialPosition, m_retreatPosition = InitialPosition;
        bool m_running = false, m_cycle = false, m_returnInterrupted = false, m_stopped = false;
    };

    // A deterministic zero-inertia consumer used only by the P3/host scenario.
    // Feedback position is a bounded virtual counter, never an axis coordinate.
    class Consumer
    {
    public:
        static constexpr std::int32_t FeedStep = 10, TravelStep = 25;
        void Reset(const Scope& scope = Scope{}) noexcept
        {
            m_position = Coordinator::InitialPosition;
            m_publication = m_lastToken = m_lastNow = 0ULL;
            m_scope = scope;
            m_haveScope = scope.session != 0ULL && scope.owner != 0ULL && scope.epoch != 0ULL;
            m_lastAction = Action::None;
            m_lastTarget = Coordinator::InitialPosition;
            m_complete = false;
        }
        Feedback Consume(const Snapshot& command, std::uint64_t nowMs) noexcept
        {
            Feedback result{};
            result.scope = command.scope;
            result.commandToken = command.commandToken;
            result.observedAtMs = nowMs;
            result.action = command.action;
            result.virtualPosition = m_position;
            // Reject without moving, advancing proof publication, or changing
            // consumer identity. A stale/replayed snapshot cannot move twice.
            if (command.scope.session == 0ULL || command.scope.owner == 0ULL || command.scope.epoch == 0ULL ||
                (m_haveScope && !SameScope(command.scope, m_scope)) ||
                command.commandToken == 0ULL || command.commandToken < m_lastToken ||
                command.observedAtMs > nowMs || nowMs - command.observedAtMs > Coordinator::MaximumFeedbackAgeMs ||
                (m_lastToken != 0ULL && nowMs < m_lastNow) ||
                command.virtualPosition != m_position ||
                command.virtualPosition < Coordinator::MinimumPosition || command.virtualPosition > Coordinator::MaximumPosition ||
                command.targetPosition < Coordinator::MinimumPosition || command.targetPosition > Coordinator::MaximumPosition ||
                m_publication == (std::numeric_limits<std::uint64_t>::max)()) return result;
            if (command.commandToken == m_lastToken &&
                (nowMs <= m_lastNow || m_complete || command.action != m_lastAction || command.targetPosition != m_lastTarget))
                return result;
            bool valid = false;
            switch (command.action)
            {
            case Action::Stop:
                valid = !command.simulatedPermit && !command.retreatRequested && command.simulatedVelocity == 0 &&
                    command.targetPosition == m_position &&
                    (command.phase == Phase::WaitStop || command.phase == Phase::ReturnStopped ||
                    command.phase == Phase::Hold || command.phase == Phase::Idle || command.phase == Phase::Fault);
                break;
            case Action::Feed:
                valid = command.phase == Phase::Feeding && command.simulatedPermit && !command.retreatRequested &&
                    command.simulatedVelocity == 1 && command.targetPosition == Coordinator::MaximumPosition;
                break;
            case Action::Retreat:
                valid = command.phase == Phase::Retreating && !command.simulatedPermit && command.retreatRequested &&
                    command.simulatedVelocity == -1 && command.targetPosition <= m_position;
                break;
            case Action::Return:
                valid = command.phase == Phase::Returning && !command.simulatedPermit && !command.retreatRequested &&
                    command.simulatedVelocity == 1 && command.targetPosition >= m_position;
                break;
            default: break;
            }
            if (!valid) return result;
            result.valid = true;
            result.publicationSequence = ++m_publication;
            result.ack = Ack::Progress;
            if (command.action == Action::Stop) result.ack = Ack::Stopped;
            else if (command.action == Action::Feed)
            {
                m_position += FeedStep;
                if (m_position > Coordinator::MaximumPosition) m_position = Coordinator::MaximumPosition;
            }
            else if (command.action == Action::Retreat)
            {
                m_position -= TravelStep;
                if (m_position <= command.targetPosition)
                { m_position = command.targetPosition; result.ack = Ack::Complete; }
            }
            else if (command.action == Action::Return)
            {
                m_position += TravelStep;
                if (m_position >= command.targetPosition)
                { m_position = command.targetPosition; result.ack = Ack::Complete; }
            }
            result.virtualPosition = m_position;
            m_scope = command.scope;
            m_haveScope = true;
            m_lastToken = command.commandToken;
            m_lastNow = nowMs;
            m_lastAction = command.action;
            m_lastTarget = command.targetPosition;
            m_complete = result.ack == Ack::Stopped || result.ack == Ack::Complete;
            return result;
        }
    private:
        Scope m_scope{};
        std::int32_t m_position = Coordinator::InitialPosition, m_lastTarget = Coordinator::InitialPosition;
        std::uint64_t m_publication = 0ULL, m_lastToken = 0ULL, m_lastNow = 0ULL;
        Action m_lastAction = Action::None;
        bool m_haveScope = false, m_complete = false;
    };
    static_assert(sizeof(Coordinator) <= 384U, "EDM02 fixed storage budget");
}
