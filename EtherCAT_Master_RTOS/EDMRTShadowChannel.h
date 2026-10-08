#pragma once

#include "EDMExecutionHandoff.h"
#include "MotionCommandRing.h"
#include <atomic>
#include <cstring>

// EDM48: sole NC producer -> sole runtime consumer, SHADOW VALUES ONLY.
// No axis, planner, PID, PDO, NIC or discharge interface exists here.
// Latest eligible command semantics: a newer submitted command fences older
// queued/output commands. Queue acceptance is never runtime application.
// Cancel/neutralize are independent of ring capacity and clock validity.
namespace EDM48
{
    enum class State : std::uint8_t { Idle, AwaitStart, Running, Blocked, Draining, Completed, Cancelled, Fault };
    enum class Reason : std::uint8_t
    {
        None, AwaitStart, InvalidStart, InvalidPacket, InvalidLifecycle, ScopeMismatch,
        RuntimeClock, RuntimeSource, AuthorityLost, Stale, Cancelled, Neutralized,
        Superseded, CounterOverflow, ConsumerRejected, AwaitLaterTick
    };
    struct Facts
    {
        std::uint64_t tick = 0U, nowUs = 0U, executionEpoch = 0U, ownerLease = 0U;
        bool clockValid = false, pdoValid = false, contiguous = false, authorityReady = false;
    };
    struct Feedback
    {
        EDM46::Scope scope{};
        State state = State::Idle;
        Reason reason = Reason::None;
        std::uint64_t publication = 0U, rtTick = 0U, nowUs = 0U;
        std::uint64_t latestSession = 0U, cancelThroughSession = 0U, neutralizeThroughCommand = 0U;
        EDM47::Receipt lastApplied{}, lastCompleted{};
        std::uint64_t appliedTick = 0U, completedTick = 0U;
        // Channel-lifetime counters, including discarded old-session work.
        std::uint64_t acceptedCount = 0U, appliedCount = 0U, supersededCount = 0U, lastCommand = 0U;
        std::array<double, EDM46::AxisSlots> axisSpeedMmMin{};
        bool startAcknowledged = false, pendingDrain = false;
        constexpr bool PhysicalMotionEnabled() const noexcept { return false; }
        constexpr bool PhysicalDischargeEnabled() const noexcept { return false; }
        constexpr bool PhysicalStopProven() const noexcept { return false; }
        constexpr bool DriveAcknowledged() const noexcept { return false; }
    };
    inline bool AllSpeedsZero(const Feedback& value) noexcept { return EDM47::AllSpeedsZero(value); }

    class Channel
    {
    public:
        Channel() noexcept = default;
        Channel(const Channel&) = delete;
        Channel& operator=(const Channel&) = delete;
        // NC owner only. Shared queues/banks/floors are never reset in use.
        bool ControlStart(const EDM46::Config& config, const EDM46::Scope& scope,
            const EDM46::IntentSnapshot& initial, std::uint64_t nowMs) noexcept
        {
            if (feedbackInvalid_.load(std::memory_order_acquire)) return false;
            const auto latest = latestSession_.load(std::memory_order_relaxed);
            if (scope.session <= latest || scope.session <= cancelThroughSession_.load(std::memory_order_acquire) ||
                !EDM47::ValidInitial(config, scope, initial, nowMs)) return false;
            if (latest && cancelThroughSession_.load(std::memory_order_acquire) < latest)
            {
                Feedback previous{};
                if (!ControlRead(previous) || previous.scope.session != latest || previous.state != State::Completed)
                    return false;
            }
            controlConfig_ = config; controlScope_ = scope; controlSource_ = initial.sourceIdentity;
            // Publish the new identity before the queue item: old-session
            // output becomes ineligible even if the bounded enqueue fails.
            latestSession_.store(scope.session, std::memory_order_release);
            Request request{}; request.kind = RequestKind::Start;
            request.config = config; request.initial = initial;
            if (requests_.ProducerTryPush(request)) return true;
            ControlCancel(scope.session); return false;
        }
        bool ControlSubmit(const EDM47::Packet& packet) noexcept
        {
            if (feedbackInvalid_.load(std::memory_order_acquire)) return false;
            const auto session = latestSession_.load(std::memory_order_acquire);
            // Detached old-session work cannot cancel the current session.
            if (!session || packet.scope.session != session) return false;
            if (!EDM46::SameScope(packet.scope, controlScope_) ||
                session <= cancelThroughSession_.load(std::memory_order_acquire) ||
                packet.commandSequence <= lastControlCommand_ ||
                packet.commandSequence <= neutralizeThroughCommand_.load(std::memory_order_acquire) ||
                !EDM47::ValidPacket(packet, controlConfig_, controlScope_, controlSource_, packet.issuedAtMs))
            { ControlCancel(session); return false; }
            lastControlCommand_ = packet.commandSequence;
            PublishMaximum(neutralizeThroughCommand_, packet.commandSequence - 1U);
            Request request{}; request.kind = RequestKind::Packet; request.packet = packet;
            if (requests_.ProducerTryPush(request)) return true;
            // Failed submission cannot leave the preceding vector eligible.
            PublishMaximum(neutralizeThroughCommand_, packet.commandSequence);
            ControlCancel(session); return false;
        }
        void ControlNeutralize(std::uint64_t session, std::uint64_t commandFloor) noexcept
        {
            if (session && session == latestSession_.load(std::memory_order_acquire))
                PublishMaximum(neutralizeThroughCommand_, commandFloor);
        }
        void ControlCancel(std::uint64_t session) noexcept
        {
            if (session) PublishMaximum(cancelThroughSession_, session);
        }
        // One bounded coherent read, at most four attempts. Every word is
        // atomic: the sequence check does not hide a plain-memory data race.
        // A late publication with obsolete producer fences is not returned.
        bool ControlRead(Feedback& output) const noexcept
        {
            output = Feedback{};
            if (feedbackInvalid_.load(std::memory_order_acquire)) return false;
            for (unsigned attempt = 0U; attempt < 4U; ++attempt)
            {
                const Fences before = ReadFences();
                const auto publication = published_.load(std::memory_order_acquire);
                if (!publication) return false;
                const Bank& bank = banks_[static_cast<std::size_t>(publication & 1U)];
                const auto expected = publication * 2U;
                if (bank.sequence.load(std::memory_order_acquire) != expected) continue;
                std::array<std::uint64_t, WordCount> words{};
                for (std::size_t word = 0U; word < WordCount; ++word)
                    words[word] = bank.words[word].load(std::memory_order_acquire);
                if (bank.sequence.load(std::memory_order_acquire) != expected ||
                    published_.load(std::memory_order_acquire) != publication) continue;
                Feedback candidate{}; std::memcpy(static_cast<void*>(&candidate), words.data(), sizeof(candidate));
                const Fences after = ReadFences();
                if (feedbackInvalid_.load(std::memory_order_acquire) ||
                    !SameFences(before, after) || candidate.latestSession != after.latest ||
                    candidate.cancelThroughSession != after.cancelled ||
                    candidate.neutralizeThroughCommand != after.neutralized) continue;
                output = candidate; return true;
            }
            return false;
        }
        // Runtime owner only. At most four bounded request pops each pass.
        // Call on invalid PDO cycles too; cancellation never needs valid PDO.
        void RuntimeCycle(const Facts& facts) noexcept
        {
            if (feedbackInvalid_.load(std::memory_order_acquire)) { Trip(Reason::CounterOverflow); return; }
            f_.rtTick = facts.tick; f_.nowUs = facts.nowUs;
            const bool clock = facts.clockValid && facts.tick && facts.nowUs &&
                (!haveRuntimeClock_ || (facts.tick > previousTick_ && facts.nowUs > previousUs_));
            if (facts.tick > previousTick_) previousTick_ = facts.tick;
            if (facts.nowUs > previousUs_) previousUs_ = facts.nowUs;
            haveRuntimeClock_ = true;
            ObserveFences(ReadFences());
            if (Active() && !RuntimeReady(facts, clock))
                Trip(!clock ? Reason::RuntimeClock : (!facts.pdoValid || !facts.contiguous ?
                    Reason::RuntimeSource : Reason::AuthorityLost));
            if (f_.state == State::Running &&
                !EDM47::ValidPacket(f_.lastApplied.packet, runtimeConfig_, f_.scope, runtimeSource_, facts.nowUs / 1000U))
                Trip(Reason::Stale);
            Request request{};
            for (unsigned count = 0U; count < 4U && requests_.ConsumerTryPop(request); ++count)
            {
                ObserveFences(ReadFences());
                if (request.kind == RequestKind::Start) StartRuntime(request, facts, clock);
                else ApplyRuntime(request.packet, facts, clock);
                // A concurrent NC cancellation can retire the just-consumed
                // shadow value before this pass's feedback is published.
                ObserveFences(ReadFences());
            }
            if (f_.state == State::Draining && facts.tick > f_.appliedTick &&
                facts.nowUs / 1000U > f_.lastApplied.atMs && RuntimeReady(facts, clock))
            {
                EDM47::Receipt completed{};
                if (!consumer_.CompleteDrain(facts.nowUs / 1000U, completed)) Trip(Reason::ConsumerRejected);
                else
                {
                    f_.lastCompleted = completed; f_.completedTick = facts.tick;
                    f_.pendingDrain = false; f_.state = State::Completed; f_.reason = Reason::None;
                }
            }
            ObserveFences(ReadFences());
            f_.axisSpeedMmMin = consumer_.Snapshot().axisSpeedMmMin;
            Publish();
        }
    private:
        enum class RequestKind : std::uint8_t { Start, Packet };
        struct Request
        {
            RequestKind kind = RequestKind::Start;
            EDM46::Config config{};
            EDM46::IntentSnapshot initial{};
            EDM47::Packet packet{};
        };
        struct Fences { std::uint64_t latest = 0U, cancelled = 0U, neutralized = 0U; };
        static constexpr std::size_t WordCount = (sizeof(Feedback) + sizeof(std::uint64_t) - 1U) / sizeof(std::uint64_t);
        struct Bank
        {
            std::atomic<std::uint64_t> sequence{0U};
            std::array<std::atomic<std::uint64_t>, WordCount> words{};
            Bank() noexcept { for (auto& word : words) word.store(0U, std::memory_order_relaxed); }
        };
        FixedCapacitySpscRing<Request, 8U> requests_{};
        std::atomic<std::uint64_t> latestSession_{0U}, cancelThroughSession_{0U}, neutralizeThroughCommand_{0U};
        std::array<Bank, 2U> banks_{};
        std::atomic<std::uint64_t> published_{0U};
        std::atomic<std::uint64_t> feedbackInvalid_{0U};
        // NC-only fields.
        EDM46::Config controlConfig_{};
        EDM46::Scope controlScope_{};
        EDM26::LiveSourceIdentity controlSource_{};
        std::uint64_t lastControlCommand_ = 0U;
        // Runtime-only fields.
        EDM47::ShadowConsumer consumer_{};
        EDM46::Config runtimeConfig_{};
        EDM26::LiveSourceIdentity runtimeSource_{};
        Feedback f_{};
        std::uint64_t nextPublication_ = 0U, previousTick_ = 0U, previousUs_ = 0U;
        bool haveRuntimeClock_ = false;
        static constexpr std::uint64_t Maximum() noexcept { return (std::numeric_limits<std::uint64_t>::max)(); }
        // Exactly one NC writer owns these floors; no retry/CAS loop needed.
        static void PublishMaximum(std::atomic<std::uint64_t>& target, std::uint64_t value) noexcept
        { if (value > target.load(std::memory_order_relaxed)) target.store(value, std::memory_order_release); }
        Fences ReadFences() const noexcept
        {
            Fences value{}; value.latest = latestSession_.load(std::memory_order_acquire);
            value.cancelled = cancelThroughSession_.load(std::memory_order_acquire);
            value.neutralized = neutralizeThroughCommand_.load(std::memory_order_acquire); return value;
        }
        static bool SameFences(const Fences& a, const Fences& b) noexcept
        { return a.latest == b.latest && a.cancelled == b.cancelled && a.neutralized == b.neutralized; }
        bool Active() const noexcept
        { return f_.state == State::Running || f_.state == State::Blocked || f_.state == State::Draining || f_.state == State::Completed; }
        bool RuntimeReady(const Facts& facts, bool clock) const noexcept
        {
            return clock && facts.pdoValid && facts.contiguous && facts.authorityReady &&
                facts.executionEpoch == f_.scope.executionEpoch && facts.ownerLease == f_.scope.ownerLease;
        }
        void ObserveFences(const Fences& fences) noexcept
        {
            f_.latestSession = fences.latest; f_.cancelThroughSession = fences.cancelled;
            f_.neutralizeThroughCommand = fences.neutralized;
            if (fences.latest && fences.latest <= fences.cancelled)
            {
                consumer_.Cancel(); f_.state = State::Cancelled; f_.reason = Reason::Cancelled;
                f_.startAcknowledged = f_.pendingDrain = false;
            }
            else if (f_.scope.session != fences.latest)
            {
                consumer_.Cancel(); f_.state = fences.latest ? State::AwaitStart : State::Idle;
                f_.reason = fences.latest ? Reason::AwaitStart : Reason::None;
                f_.startAcknowledged = f_.pendingDrain = false;
            }
            else if (f_.lastCommand && f_.lastCommand <= fences.neutralized &&
                (f_.state == State::Running || f_.state == State::Draining || f_.state == State::Completed))
            {
                if (f_.state != State::Running) Trip(Reason::Neutralized);
                else
                {
                    consumer_.Neutralize(fences.neutralized);
                    f_.state = State::Blocked; f_.reason = Reason::Neutralized;
                }
            }
            f_.axisSpeedMmMin = consumer_.Snapshot().axisSpeedMmMin;
        }
        void Superseded() noexcept
        {
            if (f_.supersededCount == Maximum()) { Trip(Reason::CounterOverflow); return; }
            ++f_.supersededCount;
        }
        void Trip(Reason reason) noexcept
        { consumer_.Cancel(); f_.state = State::Fault; f_.reason = reason; f_.pendingDrain = false; f_.axisSpeedMmMin.fill(0.0); }
        void StartRuntime(const Request& request, const Facts& facts, bool clock) noexcept
        {
            const auto& initial = request.initial;
            if (!initial.scope.session || initial.scope.session != f_.latestSession ||
                initial.scope.session <= f_.cancelThroughSession) { Superseded(); return; }
            if (f_.startAcknowledged || f_.scope.session == initial.scope.session)
            { Trip(Reason::InvalidLifecycle); return; }
            consumer_.Cancel();
            if (!consumer_.Reset()) { Trip(Reason::InvalidLifecycle); return; }
            f_.scope = initial.scope; runtimeConfig_ = request.config; runtimeSource_ = initial.sourceIdentity;
            f_.lastApplied = f_.lastCompleted = EDM47::Receipt{};
            f_.lastCommand = f_.appliedTick = f_.completedTick = 0U;
            const auto nowMs = facts.nowUs / 1000U;
            if (!RuntimeReady(facts, clock))
            { Trip(!clock ? Reason::RuntimeClock : (!facts.pdoValid || !facts.contiguous ? Reason::RuntimeSource : Reason::AuthorityLost)); return; }
            // NC and RT use the same performance-counter epoch. The frozen
            // initial timestamp is checked for age before starting the model.
            if (initial.observedAtMs > nowMs || nowMs - initial.observedAtMs > initial.sourceIdentity.maxAgeMs ||
                !EDM47::ValidInitial(request.config, initial.scope, initial, initial.observedAtMs) ||
                !consumer_.Start(request.config, initial.scope, initial, initial.observedAtMs))
            { Trip(Reason::InvalidStart); return; }
            f_.startAcknowledged = true; f_.pendingDrain = false;
            f_.state = State::Blocked; f_.reason = Reason::None;
        }
        void ApplyRuntime(const EDM47::Packet& packet, const Facts& facts, bool clock) noexcept
        {
            if (!packet.scope.session || packet.scope.session != f_.latestSession ||
                packet.scope.session <= f_.cancelThroughSession || packet.commandSequence <= f_.neutralizeThroughCommand)
            { Superseded(); return; }
            if (!f_.startAcknowledged || (f_.state != State::Running && f_.state != State::Blocked))
            { Trip(Reason::InvalidLifecycle); return; }
            if (!RuntimeReady(facts, clock))
            { Trip(!clock ? Reason::RuntimeClock : (!facts.pdoValid || !facts.contiguous ? Reason::RuntimeSource : Reason::AuthorityLost)); return; }
            if (!EDM46::SameScope(packet.scope, f_.scope)) { Trip(Reason::ScopeMismatch); return; }
            if (!EDM47::ValidPacket(packet, runtimeConfig_, f_.scope, runtimeSource_, facts.nowUs / 1000U))
            { Trip(Reason::InvalidPacket); return; }
            if (f_.acceptedCount == Maximum() || f_.appliedCount == Maximum()) { Trip(Reason::CounterOverflow); return; }
            // The model clock is producer issue order. A newer NC packet can
            // legitimately be issued before a delayed previous RT application.
            // Freshness/current facts were checked with actual RT time above;
            // the public Applied receipt is stamped with that RT observation.
            EDM47::Receipt accepted{packet, EDM47::ReceiptStage::Accepted, 0U, packet.issuedAtMs};
            EDM47::Receipt applied{};
            if (!consumer_.Apply(packet, accepted, packet.issuedAtMs, applied)) { Trip(Reason::ConsumerRejected); return; }
            applied.atMs = facts.nowUs / 1000U;
            ++f_.acceptedCount; ++f_.appliedCount; f_.lastApplied = applied;
            f_.lastCommand = packet.commandSequence; f_.appliedTick = facts.tick;
            f_.pendingDrain = packet.kind == EDM47::CommandKind::Drain;
            f_.state = f_.pendingDrain ? State::Draining : State::Running; f_.reason = Reason::None;
        }
        void Publish() noexcept
        {
            if (nextPublication_ >= Maximum() / 2U)
            {
                Trip(Reason::CounterOverflow);
                feedbackInvalid_.store(1U, std::memory_order_release); return;
            }
            const auto publication = ++nextPublication_; f_.publication = publication;
            std::array<std::uint64_t, WordCount> words{}; std::memcpy(words.data(), &f_, sizeof(f_));
            Bank& bank = banks_[static_cast<std::size_t>(publication & 1U)];
            bank.sequence.store(publication * 2U + 1U, std::memory_order_release);
            for (std::size_t word = 0U; word < WordCount; ++word) bank.words[word].store(words[word], std::memory_order_release);
            bank.sequence.store(publication * 2U, std::memory_order_release);
            published_.store(publication, std::memory_order_release);
        }
    };
    static_assert(std::is_trivially_copyable<Feedback>::value, "EDM48 feedback uses bounded atomic word copies.");
    static_assert(ATOMIC_LLONG_LOCK_FREE == 2, "EDM48 requires lock-free 64-bit atomic transport words.");
    static_assert(sizeof(Channel) <= 16384U, "EDM48 shadow transport storage must remain bounded.");
    inline const char* StateName(State state) noexcept
    {
        switch (state)
        {
        case State::Idle: return "IDLE"; case State::AwaitStart: return "AWAIT_START";
        case State::Running: return "RUNNING"; case State::Blocked: return "BLOCKED";
        case State::Draining: return "DRAINING"; case State::Completed: return "COMPLETED";
        case State::Cancelled: return "CANCELLED"; case State::Fault: return "FAULT";
        } return "UNKNOWN";
    }
    inline const char* ReasonName(Reason reason) noexcept
    {
        switch (reason)
        {
#define EDM48_REASON(name, label) case Reason::name: return label
        EDM48_REASON(None, "NONE"); EDM48_REASON(AwaitStart, "AWAIT_START");
        EDM48_REASON(InvalidStart, "INVALID_START"); EDM48_REASON(InvalidPacket, "INVALID_PACKET");
        EDM48_REASON(InvalidLifecycle, "INVALID_LIFECYCLE"); EDM48_REASON(ScopeMismatch, "SCOPE_MISMATCH");
        EDM48_REASON(RuntimeClock, "RUNTIME_CLOCK"); EDM48_REASON(RuntimeSource, "RUNTIME_SOURCE");
        EDM48_REASON(AuthorityLost, "AUTHORITY_LOST"); EDM48_REASON(Stale, "STALE");
        EDM48_REASON(Cancelled, "CANCELLED"); EDM48_REASON(Neutralized, "NEUTRALIZED");
        EDM48_REASON(Superseded, "SUPERSEDED"); EDM48_REASON(CounterOverflow, "COUNTER_OVERFLOW");
        EDM48_REASON(ConsumerRejected, "CONSUMER_REJECTED"); EDM48_REASON(AwaitLaterTick, "AWAIT_LATER_TICK");
#undef EDM48_REASON
        } return "UNKNOWN";
    }
}
