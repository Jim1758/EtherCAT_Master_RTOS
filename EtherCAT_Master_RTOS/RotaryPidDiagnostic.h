#pragma once

#include "MotionCommandRing.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

// ROTPID_DIAG2: observation only. No controller, alarm, in-position, owner,
// epoch, PDO, parameter or NC-completion writes are possible through this API.
// Sole producer: 250-us Motion owner. Sole consumer: HMI 1000-ms task.
struct RotaryPidDiagnosticSample
{
    std::uint64_t tick = 0ULL;
    std::uint64_t segment = 0ULL;
    double commandPulse = 0.0;
    double actualPulse = 0.0;
    double windowPulse = 0.0;
    double unitsPerPulse = 0.0;
    double kp = 0.0;
    double ki = 0.0;
    double kvff = 0.0;
    double integral = 0.0;
    double commandVelocityPps = 0.0;
    double logicalVelocityPps = 0.0;
    double capVelocityPps = 0.0; // IDLE_HOLD cap; 0 = not observed in tail scope
    std::uint32_t scope = 0U; // 0 MOTION_TAIL; 1 IDLE_HOLD
    std::uint32_t epoch = 0U;
    std::uint32_t generation = 0U;
    std::uint32_t groupMask = 0U;
    std::int32_t axis = -1;
    std::int32_t imageVelocityPps = 0;
    std::uint32_t owner = 0U;
    std::uint32_t state = 0U;
    std::uint32_t flags = 0U; // bit0 image authorized; bit1 reversed; bit2 group
};

struct RotaryPidDiagnosticEvent
{
    RotaryPidDiagnosticSample sample{};
    double minErrorPulse = 0.0;
    double maxErrorPulse = 0.0;
    double averageActualVelocityPps = 0.0;
    std::uint64_t sequence = 0ULL;
    std::uint32_t elapsedUs = 0U;
    std::uint32_t samples = 0U;
    std::uint32_t outsideSamples = 0U;
    std::int32_t minImageVelocityPps = 0;
    std::int32_t maxImageVelocityPps = 0;
    std::uint32_t saturatedSamples = 0U;
    std::uint32_t unauthorizedSamples = 0U;
    std::uint32_t errorSignChanges = 0U;
    std::uint32_t imageSignChanges = 0U;
    // 0 WAIT; 1 END_SCOPE; 2 END_IDENTITY; 3 END_INPUT_GAP; 4 SAMPLE.
    // SAMPLE is an idle observation, NOT an NC wait or completion indication.
    // END never means completed, in position, or hardware PASS.
    std::uint32_t kind = 0U;
};
static_assert(std::is_trivially_copyable<RotaryPidDiagnosticEvent>::value,
    "Rotary diagnostics must be a fixed POD payload.");

class RotaryPidDiagnosticMonitor
{
public:
    static constexpr std::size_t AxisCount = 8U;
    static constexpr std::size_t Capacity = 16U;
    static constexpr std::size_t DrainBudget = 4U;
    static constexpr std::uint32_t CycleUs = 250U;

    void Observe(const RotaryPidDiagnosticSample& sample) noexcept
    {
        if (sample.axis < 0 || sample.axis >= static_cast<std::int32_t>(AxisCount)) return;
        State& state = states_[static_cast<std::size_t>(sample.axis)];
        if (!Valid(sample))
        {
            Close(static_cast<std::size_t>(sample.axis), 3U);
            return;
        }
        if (state.active && (sample.tick != state.last.tick + 1ULL))
            Close(static_cast<std::size_t>(sample.axis), 3U);
        if (state.active && !SameIdentity(state.last, sample))
            Close(static_cast<std::size_t>(sample.axis), 2U);

        const double error = sample.commandPulse - sample.actualPulse;
        if (!state.active)
        {
            state = State{};
            state.active = true;
            state.firstActualPulse = sample.actualPulse;
            state.minError = state.maxError = error;
            state.minImage = state.maxImage = sample.imageVelocityPps;
        }
        state.last = sample;
        if (state.samples < (std::numeric_limits<std::uint32_t>::max)()) ++state.samples;
        if (std::abs(error) > sample.windowPulse &&
            state.outside < (std::numeric_limits<std::uint32_t>::max)()) ++state.outside;
        state.minError = (std::min)(state.minError, error);
        state.maxError = (std::max)(state.maxError, error);

        state.minImage = (std::min)(state.minImage, sample.imageVelocityPps);
        state.maxImage = (std::max)(state.maxImage, sample.imageVelocityPps);
        if ((sample.flags & 1U) == 0U) Increment(state.unauthorized);
        // This counts the P-only IDLE_HOLD controller's calculated saturation.
        // It does not infer drive reception or saturation of an unobserved loop.
        if (sample.scope == 1U && sample.capVelocityPps > 0.0 && sample.kp > 0.0 &&
            std::abs(error) >= sample.capVelocityPps / sample.kp)
            Increment(state.saturated);
        CountSignChange(error > 0.0 ? 1 : (error < 0.0 ? -1 : 0),
            state.lastErrorSign, state.errorChanges);
        CountSignChange(sample.imageVelocityPps > 0 ? 1 :
            (sample.imageVelocityPps < 0 ? -1 : 0),
            state.lastImageSign, state.imageChanges);

        // Three snapshots per episode only: 0.5, 2 and 5 seconds.
        // No continuing log stream if a machine remains stuck indefinitely.
        const bool due = state.samples == 2001U || state.samples == 8001U ||
            state.samples == 20001U;
        if (due && (state.outside != 0U || sample.scope == 1U))
        {
            Emit(state, sample.scope == 1U ? 4U : 0U);
            state.reported = true;
        }
    }

    void Close(std::size_t axis, std::uint32_t kind = 1U) noexcept
    {
        if (axis >= AxisCount) return;
        State& state = states_[axis];
        if (state.active && state.reported) Emit(state, kind);
        state.active = false;
        state.reported = false;
    }

    bool TryPop(RotaryPidDiagnosticEvent& event) noexcept
    {
        return events_.ConsumerTryPop(event);
    }
    std::uint32_t Dropped() const noexcept
    {
        return dropped_.load(std::memory_order_acquire);
    }

private:
    struct State
    {
        RotaryPidDiagnosticSample last{};
        double firstActualPulse = 0.0;
        double minError = 0.0;
        double maxError = 0.0;
        std::uint32_t samples = 0U;
        std::uint32_t outside = 0U;
        std::int32_t minImage = 0;
        std::int32_t maxImage = 0;
        std::int32_t lastErrorSign = 0;
        std::int32_t lastImageSign = 0;
        std::uint32_t saturated = 0U;
        std::uint32_t unauthorized = 0U;
        std::uint32_t errorChanges = 0U;
        std::uint32_t imageChanges = 0U;
        bool active = false;
        bool reported = false;
    };
    std::array<State, AxisCount> states_{};
    FixedCapacitySpscRing<RotaryPidDiagnosticEvent, Capacity> events_{};
    RotaryPidDiagnosticEvent producerEvent_{};
    std::atomic<std::uint32_t> dropped_{ 0U };
    std::uint64_t sequence_ = 0ULL;

    static void Increment(std::uint32_t& value) noexcept
    {
        if (value != (std::numeric_limits<std::uint32_t>::max)()) ++value;
    }
    static void CountSignChange(std::int32_t sign, std::int32_t& previous,
        std::uint32_t& count) noexcept
    {
        if (sign == 0) return;
        if (previous != 0 && previous != sign) Increment(count);
        previous = sign;
    }
    static bool Valid(const RotaryPidDiagnosticSample& s) noexcept
    {
        return s.epoch != 0U && s.generation != 0U &&
            std::isfinite(s.commandPulse) && std::isfinite(s.actualPulse) &&
            std::isfinite(s.commandPulse - s.actualPulse) &&
            std::isfinite(s.windowPulse) && s.windowPulse > 0.0 &&
            std::isfinite(s.unitsPerPulse) && s.unitsPerPulse > 0.0 &&
            std::isfinite(s.kp) && std::isfinite(s.ki) && std::isfinite(s.kvff) &&
            std::isfinite(s.integral) && std::isfinite(s.commandVelocityPps) &&
            std::isfinite(s.logicalVelocityPps) &&
            s.scope <= 1U && std::isfinite(s.capVelocityPps) && s.capVelocityPps >= 0.0 &&
            (s.scope == 0U || (s.capVelocityPps > 0.0 && s.kp > 0.0 &&
                s.ki == 0.0 && s.kvff == 0.0 && s.integral == 0.0));
    }
    static bool SameIdentity(const RotaryPidDiagnosticSample& a,
        const RotaryPidDiagnosticSample& b) noexcept
    {
        return a.epoch == b.epoch && a.generation == b.generation &&
            a.owner == b.owner && a.segment == b.segment && a.groupMask == b.groupMask &&
            a.commandPulse == b.commandPulse && a.unitsPerPulse == b.unitsPerPulse &&
            a.windowPulse == b.windowPulse && a.kp == b.kp && a.ki == b.ki &&
            a.kvff == b.kvff && a.scope == b.scope &&
            a.capVelocityPps == b.capVelocityPps;
    }
    void Emit(const State& state, std::uint32_t kind) noexcept
    {
        RotaryPidDiagnosticEvent& e = producerEvent_;
        e.sample = state.last;
        e.minErrorPulse = state.minError;
        e.maxErrorPulse = state.maxError;
        const std::uint64_t elapsedUs = static_cast<std::uint64_t>(state.samples - 1U) * CycleUs;
        e.elapsedUs = static_cast<std::uint32_t>((std::min)(elapsedUs,
            static_cast<std::uint64_t>((std::numeric_limits<std::uint32_t>::max)())));
        e.averageActualVelocityPps = elapsedUs != 0ULL ?
            (state.last.actualPulse - state.firstActualPulse) /
                (static_cast<double>(elapsedUs) / 1000000.0) : 0.0;
        e.samples = state.samples;
        e.outsideSamples = state.outside;
        e.kind = kind;
        e.minImageVelocityPps = state.minImage;
        e.maxImageVelocityPps = state.maxImage;
        e.saturatedSamples = state.saturated;
        e.unauthorizedSamples = state.unauthorized;
        e.errorSignChanges = state.errorChanges;
        e.imageSignChanges = state.imageChanges;
        if (sequence_ != (std::numeric_limits<std::uint64_t>::max)()) ++sequence_;
        e.sequence = sequence_;
        if (!events_.ProducerTryPush(e))
        {
            const std::uint32_t n = dropped_.load(std::memory_order_relaxed);
            if (n != (std::numeric_limits<std::uint32_t>::max)())
                dropped_.store(n + 1U, std::memory_order_release);
        }
    }
};
static_assert(sizeof(RotaryPidDiagnosticMonitor) + sizeof(RotaryPidDiagnosticSample) +
    sizeof(RotaryPidDiagnosticEvent) <= 8192U,
    "ROTPID_DIAG2 transport, producer and consumer workspace must fit 8 KiB.");
