#pragma once
// PBC-3D bounded command ingress exclusion and immutable reference evidence.
#include "MechanicalCompensationLifecycle.h"
#include "MotionCommandRing.h"
#include <atomic>
#include <limits>

class MotionPbcIngressGate
{
public:
    static constexpr std::uint32_t Reserved = 0x80000000U;
    bool TryEnterProducer() noexcept
    {
        std::uint32_t before = state_.load(std::memory_order_acquire);
        if ((before & Reserved) != 0U || before == Reserved - 1U) return false;
        return state_.compare_exchange_strong(before, before + 1U,
            std::memory_order_acq_rel, std::memory_order_acquire);
    }
    void LeaveProducer() noexcept { state_.fetch_sub(1U, std::memory_order_release); }
    bool TryReserve() noexcept
    {
        std::uint32_t empty = 0U;
        return state_.compare_exchange_strong(empty, Reserved,
            std::memory_order_acq_rel, std::memory_order_acquire);
    }
    bool IsReserved() const noexcept { return state_.load(std::memory_order_acquire) == Reserved; }
    void Release() noexcept { state_.store(0U, std::memory_order_release); }
    class Producer
    {
    public:
        explicit Producer(MotionPbcIngressGate& gate) noexcept : gate_(gate), entered_(gate.TryEnterProducer()) {}
        ~Producer() noexcept { if (entered_) gate_.LeaveProducer(); }
        bool Entered() const noexcept { return entered_; }
        Producer(const Producer&) = delete;
        Producer& operator=(const Producer&) = delete;
    private:
        MotionPbcIngressGate& gate_; bool entered_;
    };
private:
    std::atomic<std::uint32_t> state_{0U};
};
enum class MotionPbcReferenceAction : unsigned { Reset, Home, StartupSnap, FaultSnap, ServoSnap, LimitSnap };
enum class MotionPbcReferenceResult : unsigned { Applied, Rejected, Uncertain, NumericSnap, Deferred };
struct MotionPbcReferenceEvent
{
    std::uint64_t sequence = 0, tick = 0, epoch = 0, ownerGeneration = 0;
    std::uint64_t request = 0, ticket = 0, referenceGeneration = 0;
    std::uint64_t applied = 0, rejected = 0, uncertain = 0, snaps = 0, deferred = 0;
    double rawPulse = 0, nominalPulse = 0, offsetPulse = 0, pulsePerUnit = 0;
    MotionPbcReferenceAction action = MotionPbcReferenceAction::Reset;
    MotionPbcReferenceResult result = MotionPbcReferenceResult::Rejected;
    pbc::LifecyclePhase phase = pbc::LifecyclePhase::Unconfigured;
    pbc::Error error = pbc::Error::None;
    pbc::ReferenceAction modelAction = pbc::ReferenceAction::Initial;
    bool homed = false, stopped = false, queuesDrained = false, reserved = false;
};
class MotionPbcReferenceAudit
{
public:
    static constexpr std::size_t DrainBudget = 4U;
    void Publish(MotionPbcReferenceEvent event) noexcept
    {
        event.sequence = ++sequence_;
        if (event.result == MotionPbcReferenceResult::Applied) ++applied_;
        else if (event.result == MotionPbcReferenceResult::Rejected) ++rejected_;
        else if (event.result == MotionPbcReferenceResult::Uncertain) ++uncertain_;
        else if (event.result == MotionPbcReferenceResult::Deferred) ++deferred_;
        else ++snaps_;
        event.applied = applied_; event.rejected = rejected_;
        event.uncertain = uncertain_; event.snaps = snaps_; event.deferred = deferred_;
        // A supervisory HOME retry may recur every 10 ms. Keep cumulative
        // counts exact, but publish repeated same-authority deferrals at most
        // once per 500 ms (2000 runtime ticks at the fixed 250 us cycle).
        if (event.result == MotionPbcReferenceResult::Deferred)
        {
            if (hasDeferred_ && event.epoch == deferredEpoch_ &&
                event.ownerGeneration == deferredOwner_ && event.tick >= deferredTick_ &&
                event.tick - deferredTick_ < 2000ULL) return;
            hasDeferred_ = true; deferredTick_ = event.tick;
            deferredEpoch_ = event.epoch; deferredOwner_ = event.ownerGeneration;
        }
        if (!events_.ProducerTryPush(event))
        {
            const std::uint32_t n = dropped_.load(std::memory_order_relaxed);
            if (n != (std::numeric_limits<std::uint32_t>::max)()) dropped_.store(n + 1U, std::memory_order_release);
        }
    }
    bool TryPop(MotionPbcReferenceEvent& event) noexcept { return events_.ConsumerTryPop(event); }
    std::uint32_t Dropped() const noexcept { return dropped_.load(std::memory_order_acquire); }
private:
    FixedCapacitySpscRing<MotionPbcReferenceEvent, 32U> events_{};
    std::atomic<std::uint32_t> dropped_{0U};
    bool hasDeferred_ = false;
    std::uint64_t deferredTick_ = 0, deferredEpoch_ = 0, deferredOwner_ = 0;
    std::uint64_t sequence_ = 0, applied_ = 0, rejected_ = 0, uncertain_ = 0, snaps_ = 0, deferred_ = 0;
};
