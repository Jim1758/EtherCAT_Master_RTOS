#pragma once

#include "EDMGapSignal.h"
#include <cstdint>

// EDM01: NC-thread-owned process decisions for an explicitly simulated source.
// This type has no output, PLC, Motion, allocation, or physical-source interface.
// A simulated permit is a test result; it must never be wired to a pulse output.
namespace EDMProcessSimulation
{
    enum class Command : std::uint8_t { None, Start, Hold, Stop, Reset };
    enum class State : std::uint8_t { Idle, WaitGap, Active, Low, Hold, Fault };
    enum class FeedIntent : std::uint8_t { Hold, Feed, Retreat };
    enum class Reason : std::uint8_t
    {
        AwaitStart, FreshSampleRequired, Normal, PendingLow, GapLow,
        HighInhibit, UnqualifiedBand, OperatorHold, Stopped, Revoked,
        Interlock, Configuration, Source, Quality, Stale, Clock, Sequence
    };

    struct Snapshot
    {
        State state = State::Idle;
        Reason reason = Reason::AwaitStart;
        FeedIntent feedIntent = FeedIntent::Hold;
        EDMGap::Quality gapQuality = EDMGap::Quality::NO_SAMPLE;
        EDMGap::Band gapBand = EDMGap::Band::UNKNOWN;
        std::uint64_t sampleSequence = 0ULL;
        std::uint64_t sampledAtMs = 0ULL;
        std::uint64_t observedAtMs = 0ULL;
        bool simulatedPermit = false;
        bool retreatRequested = false;

        // There is deliberately no mutable physical-output flag or setter.
        constexpr bool PhysicalDischargeEnabled() const noexcept { return false; }
    };

    inline const char* StateName(State value) noexcept
    {
        switch (value)
        {
        case State::Idle: return "IDLE";
        case State::WaitGap: return "WAIT_GAP";
        case State::Active: return "ACTIVE";
        case State::Low: return "LOW";
        case State::Hold: return "HOLD";
        case State::Fault: return "FAULT";
        }
        return "UNKNOWN";
    }

    inline const char* ReasonName(Reason value) noexcept
    {
        switch (value)
        {
        case Reason::AwaitStart: return "AWAIT_START";
        case Reason::FreshSampleRequired: return "FRESH_SAMPLE_REQUIRED";
        case Reason::Normal: return "NORMAL";
        case Reason::PendingLow: return "PENDING_LOW";
        case Reason::GapLow: return "GAP_LOW";
        case Reason::HighInhibit: return "HIGH_INHIBIT";
        case Reason::UnqualifiedBand: return "UNQUALIFIED_BAND";
        case Reason::OperatorHold: return "OPERATOR_HOLD";
        case Reason::Stopped: return "STOPPED";
        case Reason::Revoked: return "REVOKED";
        case Reason::Interlock: return "INTERLOCK";
        case Reason::Configuration: return "CONFIGURATION";
        case Reason::Source: return "SOURCE";
        case Reason::Quality: return "QUALITY";
        case Reason::Stale: return "STALE";
        case Reason::Clock: return "CLOCK";
        case Reason::Sequence: return "SEQUENCE";
        }
        return "UNKNOWN";
    }

    inline const char* FeedIntentName(FeedIntent value) noexcept
    {
        switch (value)
        {
        case FeedIntent::Hold: return "HOLD";
        case FeedIntent::Feed: return "FEED";
        case FeedIntent::Retreat: return "RETREAT";
        }
        return "UNKNOWN";
    }

    class Controller
    {
    public:
        static constexpr std::uint64_t MaximumSampleAgeMs = 100ULL;

        // A reset starts a new clock/sequence session but never arms processing.
        void Reset() noexcept
        {
            m_current = Snapshot{};
            m_lastInput = EDMGap::Snapshot{};
            m_lastNowMs = m_armTimeMs = m_armSequence = 0ULL;
            m_haveClock = m_haveInput = m_armed = m_admitted = false;
        }

        // Actual NC HOLD/RESET/alarm/completion can revoke synchronously without
        // inventing a GAP sample or advancing the scenario's synthetic clock.
        void Revoke() noexcept
        {
            m_armed = m_admitted = false;
            if (m_current.state != State::Fault) SetOff(State::Hold, Reason::Revoked);
        }

        const Snapshot& Current() const noexcept { return m_current; }

        // Start fences the observed sequence and time. Admission requires a
        // strictly newer sample acquired strictly after this Start and stable
        // NORMAL. Duplicate samples may sustain an already admitted session
        // only within MaximumSampleAgeMs; they cannot arm or complete recovery.
        const Snapshot& Step(Command command, const EDMGap::Snapshot& gap,
            std::uint64_t nowMs, bool interlockReady = true) noexcept
        {
            if (command == Command::Reset)
            {
                Reset();
                return m_current;
            }
            if (m_current.state == State::Fault) return m_current;
            if (m_haveClock && nowMs < m_lastNowMs) return Fault(Reason::Clock);
            m_haveClock = true;
            m_lastNowMs = nowMs;
            m_current.observedAtMs = nowMs;
            if (command == Command::Stop)
            {
                m_armed = m_admitted = false;
                return SetOff(State::Idle, Reason::Stopped);
            }
            if (command == Command::Hold)
            {
                m_armed = m_admitted = false;
                return SetOff(State::Hold, Reason::OperatorHold);
            }
            if (command == Command::Start)
            {
                m_armed = true;
                m_admitted = false;
                m_armSequence = gap.sequence;
                m_armTimeMs = nowMs;
                SetOff(State::WaitGap, Reason::FreshSampleRequired);
            }
            if (!m_armed) return m_current;
            if (!interlockReady) return Fault(Reason::Interlock);
            m_current.gapQuality = gap.quality;
            m_current.gapBand = gap.band;
            m_current.sampleSequence = gap.sequence;
            m_current.sampledAtMs = gap.sampledAtMs;
            if (!gap.configured || gap.quality == EDMGap::Quality::CONFIG_ERROR)
                return Fault(Reason::Configuration);
            if (gap.quality == EDMGap::Quality::NO_SAMPLE && !m_admitted &&
                (gap.source == EDMGap::Source::NONE || gap.source == EDMGap::Source::SIMULATED))
                return SetOff(State::WaitGap, Reason::FreshSampleRequired);
            if (gap.source != EDMGap::Source::SIMULATED ||
                gap.quality == EDMGap::Quality::SOURCE_MISMATCH) return Fault(Reason::Source);
            if (gap.quality == EDMGap::Quality::CLOCK_ERROR) return Fault(Reason::Clock);
            if (gap.quality == EDMGap::Quality::SEQUENCE_ERROR) return Fault(Reason::Sequence);
            if (gap.quality == EDMGap::Quality::STALE) return Fault(Reason::Stale);
            if (gap.quality != EDMGap::Quality::VALID) return Fault(Reason::Quality);
            if (gap.sampledAtMs > nowMs || gap.observedAtMs > nowMs ||
                gap.observedAtMs < gap.sampledAtMs ||
                (m_haveInput && gap.sampledAtMs < m_lastInput.sampledAtMs)) return Fault(Reason::Clock);
            if (nowMs - gap.sampledAtMs > MaximumSampleAgeMs) return Fault(Reason::Stale);
            if (gap.sequence == 0ULL || (m_haveInput && gap.sequence < m_lastInput.sequence) ||
                (m_haveInput && gap.sequence == m_lastInput.sequence && !SameInput(gap, m_lastInput)))
                return Fault(Reason::Sequence);
            m_lastInput = gap;
            m_haveInput = true;
            if (!m_admitted && (gap.sequence <= m_armSequence || gap.sampledAtMs <= m_armTimeMs))
                return SetOff(State::WaitGap, Reason::FreshSampleRequired);
            // A pending LOW withdraws permission before Monitor's dwell expires.
            if (gap.pendingBand == EDMGap::Band::LOW)
                return SetOff(State::WaitGap, Reason::PendingLow);
            // HIGH is deliberately not interpreted as a machining recipe here.
            if (gap.band == EDMGap::Band::HIGH || gap.pendingBand == EDMGap::Band::HIGH)
                return SetOff(State::WaitGap, Reason::HighInhibit);
            if (gap.band == EDMGap::Band::NORMAL && gap.pendingBand == EDMGap::Band::UNKNOWN)
            {
                m_admitted = true;
                m_current.state = State::Active;
                m_current.reason = Reason::Normal;
                m_current.feedIntent = FeedIntent::Feed;
                m_current.simulatedPermit = true;
                m_current.retreatRequested = false;
                return m_current;
            }
            if (m_admitted && gap.band == EDMGap::Band::LOW)
            {
                SetOff(State::Low, Reason::GapLow);
                m_current.feedIntent = FeedIntent::Retreat;
                m_current.retreatRequested = true;
                return m_current;
            }
            return SetOff(State::WaitGap, Reason::UnqualifiedBand);
        }

    private:
        const Snapshot& SetOff(State state, Reason reason) noexcept
        {
            m_current.state = state;
            m_current.reason = reason;
            m_current.feedIntent = FeedIntent::Hold;
            m_current.simulatedPermit = false;
            m_current.retreatRequested = false;
            return m_current;
        }

        const Snapshot& Fault(Reason reason) noexcept
        {
            m_armed = m_admitted = false;
            return SetOff(State::Fault, reason);
        }

        static bool SameInput(const EDMGap::Snapshot& a, const EDMGap::Snapshot& b) noexcept
        {
            // observedAtMs is the observer clock, never acquisition freshness.
            return a.sequence == b.sequence && a.sampledAtMs == b.sampledAtMs &&
                a.voltageMv == b.voltageMv && a.source == b.source &&
                a.quality == b.quality && a.band == b.band && a.pendingBand == b.pendingBand &&
                a.pendingSinceMs == b.pendingSinceMs && a.configured == b.configured;
        }

        Snapshot m_current{};
        EDMGap::Snapshot m_lastInput{};
        std::uint64_t m_lastNowMs = 0ULL, m_armTimeMs = 0ULL, m_armSequence = 0ULL;
        bool m_haveClock = false, m_haveInput = false, m_armed = false, m_admitted = false;
    };

    static_assert(sizeof(Controller) <= 256U, "EDM simulation controller storage budget");
}
