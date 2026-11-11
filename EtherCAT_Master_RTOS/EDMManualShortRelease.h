#pragma once
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>

// HMI recovery only. These objects never command an axis or write the PLC C20 level.
namespace EDMManualShortRelease {
enum class Action { None, Arm, Cancel };

// Numeric mailbox: zero cancels; an arm carries (uint32 PLC_RunCount) + 1 as a double.
// Both SetMemory and the PLC->NC scan execute on the supervisory thread.
class Requests {
public:
    void Write(double value) noexcept {
        if (!(value >= 1.0 && value <= 4294967296.0) || std::floor(value) != value) {
            m_pending = 0;
            return;
        }
        m_pending = static_cast<std::uint64_t>(value);
    }

    Action Poll(std::uint32_t scan, std::uint64_t now, std::uint64_t frequency, bool permitted) noexcept {
        const auto command = m_pending;
        m_pending = Empty;
        const auto halfSecond = frequency / 2U;
        const bool interrupted = !m_started || !halfSecond || now <= m_previousTime ||
            now - m_previousTime > halfSecond || scan - m_lastScan > MaxSampleAge;
        if (interrupted || scan != m_lastScan) m_scanChangedAt = now;
        m_started = true;
        m_lastScan = scan;
        m_previousTime = now;
        if (!permitted || interrupted || now - m_scanChangedAt > halfSecond || command == 0) {
            m_floor = scan;
            return Action::Cancel;
        }
        // Retain a recent floor across uint32 wrap and consume each token at most once.
        if (scan - m_floor > MaxSampleAge + 1U) m_floor = scan - MaxSampleAge - 1U;
        if (command == Empty) return Action::None;
        const auto token = static_cast<std::uint32_t>(command - 1U);
        if (scan - token >= scan - m_floor) return Action::None;
        m_floor = token;
        return Action::Arm;
    }

private:
    static constexpr std::uint64_t Empty = (std::numeric_limits<std::uint64_t>::max)();
    static constexpr std::uint32_t MaxSampleAge = 500U;  // 500 x 1 ms PLC scans.
    std::uint64_t m_pending = Empty, m_previousTime = 0, m_scanChangedAt = 0;
    std::uint32_t m_floor = 0, m_lastScan = 0;
    bool m_started = false;
};

// The RT motion loop owns acknowledgement and expiry. The original physical C20 stays separate.
class Window {
public:
    bool Arm(std::uint64_t now, std::uint64_t frequency) noexcept {
        if (!now || !frequency || frequency > ((std::numeric_limits<std::uint64_t>::max)() - now) / 5U)
            return false;
        if (m_acknowledged.load(std::memory_order_acquire)) return false;
        std::uint64_t expected = 0;
        return m_deadline.compare_exchange_strong(expected, now + frequency * 5U);
    }
    void Cancel() noexcept { m_deadline.store(0, std::memory_order_release); }
    bool Active() const noexcept {
        const auto active = m_acknowledged.load(std::memory_order_acquire);
        return active && active == m_deadline.load(std::memory_order_acquire);
    }

    // Returns true once when an acknowledged recovery ends, even if contact is already clear.
    bool UpdateRT(std::uint64_t now, bool validContact) noexcept {
        auto deadline = m_deadline.load(std::memory_order_acquire);
        const auto previous = m_acknowledged.load(std::memory_order_relaxed);
        if (!deadline && !previous) return false;
        if (deadline && (!validContact || !now || now >= deadline)) {
            m_deadline.compare_exchange_strong(deadline, 0);
            deadline = 0;
        }
        if (previous != deadline) m_acknowledged.store(deadline, std::memory_order_release);
        return previous != 0 && previous != deadline;
    }
private:
    std::atomic<std::uint64_t> m_deadline{0}, m_acknowledged{0};
};
}
