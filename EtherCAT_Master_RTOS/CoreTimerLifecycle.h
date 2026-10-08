#pragma once

#include <atomic>
#include <cstdint>
#include <type_traits>

// Admission and stop share one atomic word. A separate "stopped" flag plus
// active counter would allow shutdown to observe zero between a callback's
// flag check and counter increment.
//
// One gate belongs to one serialized timer handler. TryEnter deliberately makes
// only one CAS attempt: contention safely skips a cycle instead of spinning in
// the real-time handler. There is no reopen operation during a gate's lifetime.
//
// IMPORTANT: ActiveCount()==0 only proves that admitted work has drained.
// The owner must ALSO stop/delete the RTX timer before destroying the gate or
// callback context; rejected or not-yet-delivered callbacks can still exist.
class CoreTimerCallbackGate
{
public:
    CoreTimerCallbackGate() noexcept = default;

    CoreTimerCallbackGate(const CoreTimerCallbackGate&) = delete;
    CoreTimerCallbackGate& operator=(const CoreTimerCallbackGate&) = delete;

    void RequestStop() noexcept
    {
        m_state.fetch_or(kStopped, std::memory_order_acq_rel);
    }

    bool StopRequested() const noexcept
    {
        return (m_state.load(std::memory_order_acquire) & kStopped) != 0U;
    }

    std::uint32_t ActiveCount() const noexcept
    {
        return m_state.load(std::memory_order_acquire) & kCountMask;
    }

    bool TryEnter() noexcept
    {
        std::uint32_t observed = m_state.load(std::memory_order_acquire);
        if ((observed & kStopped) != 0U ||
            (observed & kCountMask) == kCountMask)
        {
            return false;
        }

        return m_state.compare_exchange_strong(
            observed,
            observed + 1U,
            std::memory_order_acq_rel,
            std::memory_order_acquire);
    }

    // Exactly one Leave is required for each successful TryEnter. Prefer the
    // scope below so every early return releases the admitted callback.
    void Leave() noexcept
    {
        m_state.fetch_sub(1U, std::memory_order_release);
    }

private:
    static_assert(std::is_same<std::uint32_t, unsigned int>::value,
        "Timer callback admission requires uint32_t to be unsigned int.");
    static_assert(ATOMIC_INT_LOCK_FREE == 2,
        "Timer callback admission requires lock-free 32-bit atomics.");

    static constexpr std::uint32_t kStopped = 0x80000000U;
    static constexpr std::uint32_t kCountMask = 0x7FFFFFFFU;

    std::atomic<std::uint32_t> m_state{0U};
};

class CoreTimerCallbackScope
{
public:
    explicit CoreTimerCallbackScope(CoreTimerCallbackGate& gate) noexcept
        : m_gate(gate), m_entered(gate.TryEnter())
    {
    }

    ~CoreTimerCallbackScope() noexcept
    {
        if (m_entered)
        {
            m_gate.Leave();
        }
    }

    CoreTimerCallbackScope(const CoreTimerCallbackScope&) = delete;
    CoreTimerCallbackScope& operator=(const CoreTimerCallbackScope&) = delete;

    explicit operator bool() const noexcept
    {
        return m_entered;
    }

private:
    CoreTimerCallbackGate& m_gate;
    const bool m_entered;
};
