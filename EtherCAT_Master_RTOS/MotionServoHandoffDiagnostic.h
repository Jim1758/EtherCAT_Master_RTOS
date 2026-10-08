#pragma once
// PBC-3A. Observation of the software NIC handoff only. No control authority,
// PDO writes, format/I/O, allocation, or blocking in the producer. The sample
// retains the originating Motion proof; copied/scrubbed output and SendPacket's
// return value are deliberately distinct from drive reception / WKC / motion.
#include "MotionCommandRing.h"
#include <limits>

struct MotionServoHandoffSample
{
    std::uint64_t sequence = 0ULL;
    std::uint64_t frameGeneration = 0ULL;
    std::uint64_t sourceTick = 0ULL;
    std::uint32_t epoch = 0U;
    std::uint32_t ownerGeneration = 0U;
    std::uint32_t owner = 0U;
    std::uint32_t slotMask = 0U;
    std::uint32_t observedMask = 0U;
    std::uint32_t diagnosticMask = 0U;
    std::uint32_t sourceInputMask = 0U;
    std::uint32_t reverseMask = 0U;
    std::array<std::int32_t, 8> proposedVelocity{};
    std::array<std::int32_t, 8> copiedVelocity{};
    std::array<std::uint16_t, 8> sourceStatusWord{};
    std::array<std::int8_t, 8> sourceMode{};
    bool idleScope = false;
    bool scrubbed = false;
    bool finalChecked = false;
    bool attempted = false;
    bool apiAccepted = false;
};
struct MotionServoHandoffEvent
{
    std::uint64_t sequence = 0ULL;
    std::uint64_t frameGeneration = 0ULL;
    std::uint64_t sourceTick = 0ULL;
    std::uint64_t elapsedUs = 0ULL;
    std::uint32_t epoch = 0U;
    std::uint32_t ownerGeneration = 0U;
    std::uint32_t owner = 0U;
    std::uint32_t axis = 0U;
    std::uint32_t kind = 0U; // 1=SAMPLE, 2=END_SCOPE, 3=END_IDENTITY
    std::uint32_t samples = 0U;
    std::uint32_t attempted = 0U;
    std::uint32_t apiAccepted = 0U;
    std::uint32_t apiFailed = 0U;
    std::uint32_t notAttempted = 0U;
    std::uint32_t scrubbed = 0U;
    std::uint32_t changed = 0U;
    std::uint32_t unchecked = 0U;
    std::uint32_t missingImage = 0U;
    std::uint32_t sourceNotReady = 0U;
    std::uint32_t missingInput = 0U;
    std::uint32_t sourceTickGaps = 0U;
    std::int32_t proposedVelocity = 0;
    std::int32_t copiedVelocity = 0;
    std::int32_t minAcceptedVelocity = 0;
    std::int32_t maxAcceptedVelocity = 0;
    std::uint16_t sourceStatusWord = 0U;
    std::int8_t sourceMode = 0;
    bool reversed = false;
    bool acceptedRangeValid = false;
};
static_assert(std::is_trivially_copyable<MotionServoHandoffSample>::value &&
    std::is_trivially_copyable<MotionServoHandoffEvent>::value,
    "Handoff diagnostics must be fixed POD values.");

class MotionServoHandoffMonitor
{
public:
    static constexpr std::size_t Capacity = 32U;
    static constexpr std::size_t DrainBudget = 4U;
    void Observe(const MotionServoHandoffSample& sample) noexcept
    {
        for (std::size_t axis = 0U; axis < states_.size(); ++axis)
        {
            State& s = states_[axis];
            const std::uint32_t bit = 1U << static_cast<unsigned>(axis);
            const bool eligible = sample.idleScope && sample.ownerGeneration != 0U &&
                sample.epoch != 0U && sample.sourceTick != 0ULL &&
                (sample.diagnosticMask & bit) != 0U;
            if (!eligible) { Close(axis, 2U); continue; }
            if (s.active && (s.event.epoch != sample.epoch ||
                s.event.ownerGeneration != sample.ownerGeneration || s.event.owner != sample.owner))
                Close(axis, 3U);
            if (!s.active)
            {
                s = State{}; s.active = true; s.firstTick = sample.sourceTick;
                s.event.axis = static_cast<std::uint32_t>(axis);
            }
            MotionServoHandoffEvent& e = s.event;
            if (e.samples != 0U && (e.sourceTick == (std::numeric_limits<std::uint64_t>::max)() ||
                sample.sourceTick != e.sourceTick + 1ULL)) Increment(e.sourceTickGaps);
            e.sequence = sample.sequence;
            e.frameGeneration = sample.frameGeneration;
            e.sourceTick = sample.sourceTick;
            e.epoch = sample.epoch; e.ownerGeneration = sample.ownerGeneration; e.owner = sample.owner;
            const std::uint64_t ticks = sample.sourceTick >= s.firstTick ? sample.sourceTick - s.firstTick : 0ULL;
            e.elapsedUs = ticks <= (std::numeric_limits<std::uint64_t>::max)() / 250ULL ? ticks * 250ULL :
                (std::numeric_limits<std::uint64_t>::max)();
            e.proposedVelocity = sample.proposedVelocity[axis];
            e.copiedVelocity = sample.copiedVelocity[axis];
            e.sourceStatusWord = sample.sourceStatusWord[axis];
            e.sourceMode = sample.sourceMode[axis];
            e.reversed = (sample.reverseMask & bit) != 0U;
            Increment(e.samples);
            if (sample.attempted) Increment(e.attempted); else Increment(e.notAttempted);
            if (sample.attempted && sample.apiAccepted) Increment(e.apiAccepted);
            if (sample.attempted && !sample.apiAccepted) Increment(e.apiFailed);
            if (sample.scrubbed) Increment(e.scrubbed);
            if (e.proposedVelocity != e.copiedVelocity) Increment(e.changed);
            if (!sample.finalChecked) Increment(e.unchecked);
            const bool imagePresent = (sample.observedMask & bit) != 0U;
            if (!imagePresent) Increment(e.missingImage);
            if ((sample.sourceInputMask & bit) == 0U) Increment(e.missingInput);
            else if ((e.sourceStatusWord & 0x006FU) != 0x0027U || e.sourceMode != 9)
                Increment(e.sourceNotReady);
            if (sample.attempted && sample.apiAccepted && imagePresent)
            {
                if (!e.acceptedRangeValid)
                { e.minAcceptedVelocity = e.maxAcceptedVelocity = e.copiedVelocity; e.acceptedRangeValid = true; }
                else
                {
                    if (e.copiedVelocity < e.minAcceptedVelocity) e.minAcceptedVelocity = e.copiedVelocity;
                    if (e.copiedVelocity > e.maxAcceptedVelocity) e.maxAcceptedVelocity = e.copiedVelocity;
                }
            }
            const std::uint64_t thresholds[3] = { 500000ULL, 2000000ULL, 5000000ULL };
            if (s.nextSample < 3U && e.elapsedUs >= thresholds[s.nextSample])
            { ++s.nextSample; Emit(s, 1U); s.reported = true; }
        }
    }
    bool TryPop(MotionServoHandoffEvent& event) noexcept { return events_.ConsumerTryPop(event); }
    std::uint32_t Dropped() const noexcept { return dropped_.load(std::memory_order_acquire); }
private:
    struct State
    {
        MotionServoHandoffEvent event{};
        std::uint64_t firstTick = 0ULL;
        unsigned nextSample = 0U;
        bool active = false;
        bool reported = false;
    };
    std::array<State, 8> states_{};
    FixedCapacitySpscRing<MotionServoHandoffEvent, Capacity> events_{};
    std::atomic<std::uint32_t> dropped_{ 0U };
    static void Increment(std::uint32_t& n) noexcept
    { if (n != (std::numeric_limits<std::uint32_t>::max)()) ++n; }
    void Emit(State& state, std::uint32_t kind) noexcept
    {
        state.event.kind = kind;
        if (!events_.ProducerTryPush(state.event))
        {
            std::uint32_t n = dropped_.load(std::memory_order_relaxed);
            Increment(n); dropped_.store(n, std::memory_order_release);
        }
    }
    void Close(std::size_t axis, std::uint32_t kind) noexcept
    {
        State& s = states_[axis];
        if (s.active && (s.reported || s.event.apiFailed != 0U || s.event.scrubbed != 0U ||
            s.event.changed != 0U || s.event.notAttempted != 0U || s.event.unchecked != 0U)) Emit(s, kind);
        s.active = false;
    }
};
