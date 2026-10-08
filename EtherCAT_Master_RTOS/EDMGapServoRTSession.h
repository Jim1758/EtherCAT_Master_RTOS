#pragma once

#include "EDMGapServoRuntime.h"
#include "EDMExecutionFeedbackChannel.h"
#include "MotionCommandRing.h"
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>

// EDM55: one NC owner admits/cancels a finite SIMULATION session; the actual
// runtime callback owns acquisition and the continuous servo calculation.
// Speeds below are advisory values only. There is no axis/planner/PDO/output
// interface, and a later zero receipt is NOT physical stopping evidence.
namespace EDM55RT
{
    enum class Profile : std::uint8_t { Normal = 33U, Stale = 34U };
    enum class State : std::uint8_t
    { Idle, AwaitingStart, Running, Draining, Completed, Cancelled, Fault };
    enum class Reason : std::uint8_t
    {
        None, AwaitStart, InvalidStart, Cancelled, Superseded, RuntimeClock,
        RuntimeSource, AuthorityLost, Heartbeat, SourceStale, Kernel,
        Coverage, CounterOverflow
    };
    struct Scope
    {
        std::uint64_t session = 0U, run = 0U, cache = 0U, dispatch = 0U;
        std::uint64_t executionEpoch = 0U, ownerLease = 0U;
    };
    inline bool ValidScope(const Scope& s) noexcept
    { return s.session && s.run && s.cache && s.dispatch && s.executionEpoch && s.ownerLease; }
    inline bool SameScope(const Scope& a, const Scope& b) noexcept
    {
        return a.session == b.session && a.run == b.run && a.cache == b.cache &&
            a.dispatch == b.dispatch && a.executionEpoch == b.executionEpoch && a.ownerLease == b.ownerLease;
    }
    struct Facts
    {
        std::uint64_t tick = 0U, nowUs = 0U, executionEpoch = 0U, ownerLease = 0U;
        bool clockValid = false, pdoValid = false, contiguous = false, authorityReady = false;
    };
    enum Coverage : std::uint64_t
    {
        Fresh = 1ULL << 0U, Filter = 1ULL << 1U, PositiveTarget = 1ULL << 2U,
        NegativeTarget = 1ULL << 3U, PositiveSpeed = 1ULL << 4U, NegativeSpeed = 1ULL << 5U,
        Ramp = 1ULL << 6U, Deceleration = 1ULL << 7U, ReversalZero = 1ULL << 8U,
        ShortActive = 1ULL << 9U, ShortRecovered = 1ULL << 10U,
        NoiseDeadband = 1ULL << 11U, FinalZero = 1ULL << 12U,
        RawShortStop = 1ULL << 13U, SpeedCap = 1ULL << 14U
    };
    constexpr std::uint64_t RequiredCoverage = (1ULL << 15U) - 1ULL;
    constexpr std::uint64_t DurationUs = 12000000ULL;
    constexpr std::uint64_t StaleInjectionUs = 9000000ULL;
    constexpr std::uint64_t HeartbeatMaximumAgeUs = 100000ULL;
    struct Feedback
    {
        Scope scope{};
        Profile profile = Profile::Normal;
        State state = State::Idle;
        Reason reason = Reason::None;
        EDM55::Fault kernelFault = EDM55::Fault::None;
        std::uint64_t publication = 0U, rtTick = 0U, nowUs = 0U;
        std::uint64_t latestSession = 0U, cancelThroughSession = 0U;
        std::uint64_t startedTick = 0U, startedUs = 0U, terminalTick = 0U, zeroReceiptTick = 0U;
        std::uint64_t sourceSequence = 0U, sourceCapturedAtUs = 0U;
        std::uint64_t runtimeSteps = 0U, freshSamples = 0U, stageMask = 0U, coverageMask = 0U;
        std::uint64_t minimumStepUs = 0U, maximumStepUs = 0U;
        std::uint32_t stage = 0U;
        double rawVoltageV = 0.0, filteredVoltageV = 0.0, targetMmMin = 0.0, speedMmMin = 0.0;
        bool startAcknowledged = false, expectedFault = false, zeroReceipt = false;
        bool feedInhibited = true, shortActive = false, stopRequired = false;
        constexpr bool PhysicalMotionEnabled() const noexcept { return false; }
        constexpr bool PhysicalDischargeEnabled() const noexcept { return false; }
        constexpr bool PhysicalStopProven() const noexcept { return false; }
    };

    class Session
    {
    public:
        Session() noexcept = default;
        Session(const Session&) = delete;
        Session& operator=(const Session&) = delete;
        bool ControlStart(const Scope& scope, Profile profile, std::uint64_t nowUs) noexcept
        {
            const auto latest = latest_.load(std::memory_order_acquire);
            if (!ValidScope(scope) || !nowUs || (profile != Profile::Normal && profile != Profile::Stale) ||
                scope.session <= latest || scope.session <= cancelled_.load(std::memory_order_acquire)) return false;
            if (latest)
            {
                Feedback previous{};
                if (!ControlRead(previous) || !previous.zeroReceipt ||
                    (previous.state != State::Completed && previous.state != State::Fault &&
                    previous.state != State::Cancelled)) return false;
                if (previous.scope.session != latest &&
                    !(previous.state == State::Cancelled && !previous.startAcknowledged &&
                      latest <= retired_.load(std::memory_order_acquire))) return false;
            }
            latest_.store(scope.session, std::memory_order_release);
            if (!ControlTouch(scope.session, nowUs)) { ControlCancel(scope.session); return false; }
            Request request{}; request.scope = scope; request.profile = profile; request.issuedUs = nowUs;
            if (requests_.ProducerTryPush(request)) return true;
            ControlCancel(scope.session); return false;
        }
        void ControlCancel(std::uint64_t session) noexcept
        {
            // One NC writer; detached earlier cancellation cannot retire a
            // later session. The bounded queue is never needed for revoke.
            if (session > cancelled_.load(std::memory_order_relaxed))
                cancelled_.store(session, std::memory_order_release);
        }
        void ControlRetire(std::uint64_t session) noexcept
        {
            if (!session || session != latest_.load(std::memory_order_acquire)) return;
            // Even an early retirement first cancels and lets RT publish its
            // later zero receipt; it can never freeze a nonzero running value.
            ControlCancel(session);
            if (session > retired_.load(std::memory_order_relaxed))
                retired_.store(session, std::memory_order_release);
        }
        bool ControlTouch(std::uint64_t session, std::uint64_t nowUs) noexcept
        {
            if (!session || !nowUs || session != latest_.load(std::memory_order_acquire) ||
                session <= cancelled_.load(std::memory_order_acquire) || nowUs < lastControlHeartbeatUs_) return false;
            Heartbeat heartbeat{}; heartbeat.session = session; heartbeat.atUs = nowUs;
            if (!heartbeats_.Publish(heartbeat)) { ControlCancel(session); return false; }
            lastControlHeartbeatUs_ = nowUs; return true;
        }
        bool ControlRead(Feedback& output) const noexcept
        {
            output = Feedback{};
            const auto latest = latest_.load(std::memory_order_acquire);
            const auto cancelled = cancelled_.load(std::memory_order_acquire);
            Feedback candidate{};
            const auto stamp = feedback_.Read(candidate);
            if (!stamp.Ready() || latest != latest_.load(std::memory_order_acquire) ||
                cancelled != cancelled_.load(std::memory_order_acquire) ||
                candidate.latestSession != latest || candidate.cancelThroughSession != cancelled) return false;
            candidate.publication = stamp.publication; output = candidate; return true;
        }
        void RuntimeCycle(const Facts& facts) noexcept
        {
            if (!latest_.load(std::memory_order_acquire) && f_.state == State::Idle) return;
            if (f_.latestSession && f_.latestSession == latest_.load(std::memory_order_acquire) &&
                f_.latestSession <= retired_.load(std::memory_order_acquire) &&
                f_.cancelThroughSession == cancelled_.load(std::memory_order_acquire) &&
                f_.state == State::Cancelled && f_.zeroReceipt) return;
            f_.rtTick = facts.tick; f_.nowUs = facts.nowUs;
            const bool clock = facts.clockValid && facts.tick && facts.nowUs &&
                (!haveClock_ || (facts.tick > previousTick_ && facts.nowUs > previousUs_));
            const auto delta = clock && haveClock_ ? facts.nowUs - previousUs_ : 0U;
            if (facts.tick > previousTick_) previousTick_ = facts.tick;
            if (facts.nowUs > previousUs_) previousUs_ = facts.nowUs;
            haveClock_ = true;
            ObserveFences();
            Heartbeat heartbeat{};
            if (heartbeats_.Read(heartbeat).Ready() && heartbeat.session == f_.scope.session &&
                heartbeat.atUs >= lastRuntimeHeartbeatUs_ && heartbeat.atUs <= facts.nowUs)
                lastRuntimeHeartbeatUs_ = heartbeat.atUs;
            Request request{};
            for (unsigned i = 0U; i < 4U && requests_.ConsumerTryPop(request); ++i)
            {
                ObserveFences();
                if (request.scope.session == f_.latestSession)
                {
                    pendingStart_ = request; havePendingStart_ = true;
                    if (request.scope.session <= f_.cancelThroughSession)
                    {
                        f_.scope = request.scope; f_.profile = request.profile;
                        f_.startAcknowledged = false;
                        havePendingStart_ = false;
                    }
                }
            }
            ObserveFences();
            if (havePendingStart_ && pendingStart_.scope.session == f_.latestSession &&
                pendingStart_.scope.session > f_.cancelThroughSession)
            {
                // NC may publish after this callback captured its clock. Wait
                // for the first callback at/after that genuine issue time.
                if (!clock || pendingStart_.issuedUs <= facts.nowUs ||
                    pendingStart_.issuedUs - facts.nowUs > HeartbeatMaximumAgeUs)
                {
                    StartRuntime(pendingStart_, facts, clock); havePendingStart_ = false;
                }
            }
            if (f_.state == State::Running)
            {
                if (!clock) Trip(Reason::RuntimeClock);
                else if (!facts.pdoValid || !facts.contiguous) Trip(Reason::RuntimeSource);
                else if (!Authority(facts)) Trip(Reason::AuthorityLost);
                else if (!HeartbeatReady(facts.nowUs)) Trip(Reason::Heartbeat);
                else Step(facts, delta);
            }
            else if (f_.state == State::Draining || f_.state == State::Completed)
            {
                if (!clock) Trip(Reason::RuntimeClock);
                else if (!facts.pdoValid || !facts.contiguous) Trip(Reason::RuntimeSource);
                else if (!Authority(facts)) Trip(Reason::AuthorityLost);
                else if (!HeartbeatReady(facts.nowUs)) Trip(Reason::Heartbeat);
                else if (f_.state == State::Draining && facts.tick > f_.terminalTick)
                {
                    Neutralize(); f_.zeroReceipt = true; f_.zeroReceiptTick = facts.tick;
                    f_.state = State::Completed; f_.reason = Reason::None;
                }
            }
            else if (f_.state == State::Fault || f_.state == State::Cancelled)
            {
                if (f_.expectedFault && (!clock || !facts.pdoValid || !facts.contiguous ||
                    !Authority(facts) || !HeartbeatReady(facts.nowUs)))
                {
                    // Keep the original sticky fault, but an independent
                    // runtime failure permanently invalidates test acceptance.
                    f_.expectedFault = false; f_.zeroReceipt = false; f_.zeroReceiptTick = 0U;
                }
                if (f_.terminalTick && facts.tick > f_.terminalTick && clock && facts.pdoValid && facts.contiguous)
                {
                    // This proves only that a later runtime observation retained
                    // the zero advisory output, never a drive/physical stop.
                    Neutralize();
                    if (!f_.zeroReceipt) { f_.zeroReceipt = true; f_.zeroReceiptTick = facts.tick; }
                }
                else if (!f_.terminalTick && clock && facts.pdoValid && facts.contiguous)
                {
                    // A cancellation first observed with tick zero has no
                    // usable time floor. Establish one, then await a later tick.
                    f_.terminalTick = facts.tick;
                }
            }
            ObserveFences();
            if (!feedback_.Publish(f_)) Trip(Reason::CounterOverflow);
        }
    private:
        struct Request { Scope scope{}; Profile profile = Profile::Normal; std::uint64_t issuedUs = 0U; };
        struct Heartbeat { std::uint64_t session = 0U, atUs = 0U; };
        FixedCapacitySpscRing<Request, 8U> requests_{};
        EDM53::Channel<Heartbeat> heartbeats_{};
        EDM53::Channel<Feedback> feedback_{};
        std::atomic<std::uint64_t> latest_{0U}, cancelled_{0U}, retired_{0U};
        std::uint64_t lastControlHeartbeatUs_ = 0U; // NC only.
        EDM55::Runtime core_{};
        EDM55::Sample sample_{};
        Request pendingStart_{};
        Feedback f_{};
        std::uint64_t previousTick_ = 0U, previousUs_ = 0U, lastRuntimeHeartbeatUs_ = 0U;
        bool haveClock_ = false, sawShort_ = false, havePendingStart_ = false;
        void Neutralize() noexcept
        { f_.targetMmMin = f_.speedMmMin = 0.0; f_.feedInhibited = true; }
        void Trip(Reason reason) noexcept
        {
            if (f_.state != State::Fault)
            {
                f_.state = State::Fault; f_.reason = reason; f_.terminalTick = f_.rtTick;
                f_.zeroReceipt = false; f_.zeroReceiptTick = 0U;
            }
            core_.Cancel(); Neutralize();
        }
        bool Authority(const Facts& facts) const noexcept
        {
            return facts.authorityReady && facts.executionEpoch == f_.scope.executionEpoch &&
                facts.ownerLease == f_.scope.ownerLease;
        }
        bool HeartbeatReady(std::uint64_t nowUs) const noexcept
        {
            return lastRuntimeHeartbeatUs_ && lastRuntimeHeartbeatUs_ <= nowUs &&
                nowUs - lastRuntimeHeartbeatUs_ <= HeartbeatMaximumAgeUs;
        }
        void ObserveFences() noexcept
        {
            const auto latest = latest_.load(std::memory_order_acquire);
            const auto cancelled = cancelled_.load(std::memory_order_acquire);
            const bool changed = latest != f_.latestSession || cancelled != f_.cancelThroughSession;
            f_.latestSession = latest; f_.cancelThroughSession = cancelled;
            if (f_.latestSession && f_.latestSession <= f_.cancelThroughSession)
            {
                if (havePendingStart_ && pendingStart_.scope.session == f_.latestSession)
                {
                    f_.scope = pendingStart_.scope; f_.profile = pendingStart_.profile;
                    f_.startAcknowledged = false; havePendingStart_ = false;
                }
                // A failed enqueue/heartbeat can reserve an identity without
                // ever admitting its scope. Retire that transport attempt,
                // but never manufacture a Start ACK or a scoped completion.
                if (f_.scope.session != f_.latestSession) f_.startAcknowledged = false;
                if (f_.state != State::Cancelled || changed)
                {
                    core_.Cancel(); Neutralize(); f_.state = State::Cancelled; f_.reason = Reason::Cancelled;
                    f_.terminalTick = f_.rtTick; f_.zeroReceipt = false; f_.zeroReceiptTick = 0U;
                    f_.expectedFault = false;
                }
            }
            else if (f_.scope.session != f_.latestSession)
            {
                core_.Cancel(); Neutralize(); f_.state = State::AwaitingStart; f_.reason = Reason::AwaitStart;
                f_.startAcknowledged = f_.zeroReceipt = f_.expectedFault = false;
                f_.zeroReceiptTick = 0U;
            }
        }
        void StartRuntime(const Request& request, const Facts& facts, bool clock) noexcept
        {
            core_.Cancel(); sample_ = EDM55::Sample{};
            const auto latest = f_.latestSession, cancelled = f_.cancelThroughSession;
            f_ = Feedback{}; f_.scope = request.scope; f_.profile = request.profile;
            f_.latestSession = latest; f_.cancelThroughSession = cancelled;
            f_.rtTick = facts.tick; f_.nowUs = facts.nowUs; f_.state = State::AwaitingStart;
            lastRuntimeHeartbeatUs_ = request.issuedUs; sawShort_ = false;
            Heartbeat heartbeat{};
            if (heartbeats_.Read(heartbeat).Ready() && heartbeat.session == request.scope.session &&
                heartbeat.atUs <= facts.nowUs && heartbeat.atUs >= request.issuedUs)
                lastRuntimeHeartbeatUs_ = heartbeat.atUs;
            if (!ValidScope(request.scope) || !request.issuedUs || request.issuedUs > facts.nowUs ||
                facts.nowUs - request.issuedUs > HeartbeatMaximumAgeUs)
            { Trip(Reason::InvalidStart); return; }
            if (!clock) { Trip(Reason::RuntimeClock); return; }
            if (!facts.pdoValid || !facts.contiguous) { Trip(Reason::RuntimeSource); return; }
            if (!Authority(facts)) { Trip(Reason::AuthorityLost); return; }
            if (!HeartbeatReady(facts.nowUs)) { Trip(Reason::Heartbeat); return; }
            EDM55::Config config{};
            config.referenceV = 60.0; config.curve.deadbandV = 0.5;
            config.curve.maxFeedMmPerMin = config.curve.maxRetreatMmPerMin = 30.0;
            config.curve.shortRetreatMmPerMin = 30.0;
            config.filterTauUs = 4000U; config.maxAgeUs = 5000U; config.maxTickGapUs = 2000U;
            config.shortThresholdV = 25.0; config.shortHysteresisV = 5.0;
            config.shortEnterUs = 2000U; config.shortExitUs = 5000U;
            config.accelerationMmPerMinPerSecond = config.decelerationMmPerMinPerSecond = 60.0;
            config.reversalDwellUs = 2000U;
            EDM55::Identity identity{}; identity.source = EDM55::SourceKind::Simulation;
            identity.sourceId = 55U; identity.sourceGeneration = identity.calibrationRevision = 1U;
            identity.session = request.scope.session;
            if (!core_.Configure(config) || !core_.Start(identity)) { Trip(Reason::InvalidStart); return; }
            sample_.identity = identity;
            f_.state = State::Running; f_.reason = Reason::None; f_.startAcknowledged = true;
            f_.startedTick = facts.tick; f_.startedUs = facts.nowUs;
        }
        static unsigned Stage(std::uint64_t elapsed) noexcept
        {
            if (elapsed < 500000U) return 0U;
            if (elapsed < 2000000U) return 1U;
            if (elapsed < 3500000U) return 2U;
            if (elapsed < 5000000U) return 3U;
            if (elapsed < 6500000U) return 4U;
            if (elapsed < 8000000U) return 5U;
            if (elapsed < 9500000U) return 6U;
            if (elapsed < 11000000U) return 7U;
            return 8U;
        }
        void Step(const Facts& facts, std::uint64_t delta) noexcept
        {
            if (facts.nowUs < f_.startedUs || f_.runtimeSteps >= 200000U)
            { Trip(Reason::RuntimeClock); return; }
            const auto elapsed = facts.nowUs - f_.startedUs;
            f_.stage = Stage(elapsed); f_.stageMask |= 1ULL << f_.stage;
            const bool injectStale = f_.profile == Profile::Stale && elapsed >= StaleInjectionUs;
            if (!injectStale && (!sample_.sequence || facts.nowUs - sample_.capturedAtUs >= 1000U))
            {
                if (sample_.sequence == (std::numeric_limits<std::uint64_t>::max)())
                { Trip(Reason::CounterOverflow); return; }
                sample_.valid = true; ++sample_.sequence; sample_.capturedAtUs = facts.nowUs;
                sample_.voltageV = f_.stage == 1U || f_.stage == 3U || f_.stage == 6U ||
                    (f_.stage == 5U && elapsed < 6750000U) ? 80.0 :
                    f_.stage == 2U ? 40.0 : f_.stage == 5U ? 20.0 :
                    f_.stage == 4U || f_.stage == 7U ? (sample_.sequence & 1U ? 60.2 : 59.8) : 60.0;
            }
            const double previousSpeed = f_.speedMmMin;
            const auto output = core_.Step(sample_, facts.nowUs, facts.tick);
            ++f_.runtimeSteps;
            if (delta && f_.runtimeSteps > 1U)
            {
                if (!f_.minimumStepUs || delta < f_.minimumStepUs) f_.minimumStepUs = delta;
                if (delta > f_.maximumStepUs) f_.maximumStepUs = delta;
            }
            f_.sourceSequence = output.sequence; f_.sourceCapturedAtUs = output.capturedAtUs;
            f_.rawVoltageV = output.rawVoltageV; f_.filteredVoltageV = output.filteredVoltageV;
            f_.targetMmMin = output.targetMmPerMin; f_.speedMmMin = output.speedMmPerMin;
            f_.feedInhibited = output.feedInhibited; f_.shortActive = output.shortActive;
            f_.stopRequired = output.stopRequired; f_.kernelFault = output.fault;
            if (output.fault != EDM55::Fault::None || !output.valid || !output.running)
            {
                const bool stale = output.fault == EDM55::Fault::Stale;
                Trip(stale ? Reason::SourceStale : Reason::Kernel);
                f_.expectedFault = stale && injectStale; return;
            }
            if (output.fresh) { ++f_.freshSamples; f_.coverageMask |= Fresh; }
            if (std::fabs(output.rawVoltageV - output.filteredVoltageV) > 0.01) f_.coverageMask |= Filter;
            if (output.targetMmPerMin > 0.0) f_.coverageMask |= PositiveTarget;
            if (output.targetMmPerMin < 0.0) f_.coverageMask |= NegativeTarget;
            if (output.speedMmPerMin > 0.0) f_.coverageMask |= PositiveSpeed;
            if (output.speedMmPerMin < 0.0) f_.coverageMask |= NegativeSpeed;
            if (output.speedMmPerMin != 0.0 && std::fabs(output.speedMmPerMin) < std::fabs(output.targetMmPerMin))
                f_.coverageMask |= Ramp;
            if (previousSpeed != 0.0 && std::fabs(output.speedMmPerMin) < std::fabs(previousSpeed))
                f_.coverageMask |= Deceleration;
            if (previousSpeed != 0.0 && output.speedMmPerMin == 0.0 &&
                (f_.stage == 2U || f_.stage == 3U || f_.stage == 6U)) f_.coverageMask |= ReversalZero;
            if (output.shortActive) { sawShort_ = true; f_.coverageMask |= ShortActive; }
            if (output.stopRequired && output.rawVoltageV <= 25.0 &&
                output.filteredVoltageV > 60.0 && output.speedMmPerMin == 0.0)
                f_.coverageMask |= RawShortStop;
            if (std::fabs(output.targetMmPerMin) == 30.0 && std::fabs(output.speedMmPerMin) == 30.0)
                f_.coverageMask |= SpeedCap;
            if (sawShort_ && !output.shortActive && !output.feedInhibited && f_.stage == 6U)
                f_.coverageMask |= ShortRecovered;
            if ((f_.stage == 4U && elapsed >= 6000000U) || (f_.stage == 7U && elapsed >= 10000000U))
                if (output.targetMmPerMin == 0.0 && output.speedMmPerMin == 0.0 &&
                    std::fabs(output.filteredVoltageV - 60.0) <= 0.5) f_.coverageMask |= NoiseDeadband;
            if (f_.stage == 8U && elapsed >= 11500000U && output.targetMmPerMin == 0.0 &&
                output.speedMmPerMin == 0.0 && !output.feedInhibited) f_.coverageMask |= FinalZero;
            if (elapsed >= DurationUs)
            {
                if (f_.profile != Profile::Normal || f_.coverageMask != RequiredCoverage || f_.stageMask != 0x1ffU)
                { Trip(Reason::Coverage); return; }
                core_.Cancel(); Neutralize(); f_.state = State::Draining;
                f_.terminalTick = facts.tick; f_.zeroReceipt = false; f_.zeroReceiptTick = 0U;
            }
        }
    };
    static_assert(sizeof(Feedback) <= 2048U, "EDM55 feedback is fixed coherent value storage.");
    inline const char* StateName(State value) noexcept
    {
        switch (value)
        {
        case State::Idle: return "IDLE"; case State::AwaitingStart: return "AWAITING_START";
        case State::Running: return "RUNNING"; case State::Draining: return "DRAINING";
        case State::Completed: return "COMPLETED"; case State::Cancelled: return "CANCELLED";
        case State::Fault: return "FAULT";
        } return "UNKNOWN";
    }
    inline const char* ReasonName(Reason value) noexcept
    {
        switch (value)
        {
        case Reason::None: return "NONE"; case Reason::AwaitStart: return "AWAIT_START";
        case Reason::InvalidStart: return "INVALID_START"; case Reason::Cancelled: return "CANCELLED";
        case Reason::Superseded: return "SUPERSEDED"; case Reason::RuntimeClock: return "RUNTIME_CLOCK";
        case Reason::RuntimeSource: return "RUNTIME_SOURCE"; case Reason::AuthorityLost: return "AUTHORITY_LOST";
        case Reason::Heartbeat: return "HEARTBEAT"; case Reason::SourceStale: return "SOURCE_STALE";
        case Reason::Kernel: return "KERNEL"; case Reason::Coverage: return "COVERAGE";
        case Reason::CounterOverflow: return "COUNTER_OVERFLOW";
        } return "UNKNOWN";
    }
}
