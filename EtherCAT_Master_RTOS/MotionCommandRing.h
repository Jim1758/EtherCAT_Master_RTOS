#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

// ============================================================================
// Fixed-Capacity SPSC Motion Command Channel
//
// Producer side:
//     NC / MDI planner publishes future MotionCommand objects.
//
// Consumer side:
//     250 us Motion Runtime reads and removes commands.
//
// The ingress ring is a strict Single-Producer / Single-Consumer queue.
// No allocation, lock, mutex, deque growth or blocking occurs after startup.
//
// B2 path reverse needs one extra operation that a normal FIFO does not have:
// the RT consumer must put the segment it just left back in front of the future
// path.  That operation is isolated in a fixed-capacity replay deque which is
// owned only by the RT consumer.  The producer never touches replay storage.
// ============================================================================

// Stage NC-0.1C transport configuration.
//
// These constants live with the transport primitive so MotionFeedbackRing.h
// can independently validate its worst-case stale-command burst capacity.
constexpr std::size_t MOTION_COMMAND_INGRESS_CAPACITY = 256U;
constexpr std::size_t MOTION_COMMAND_REPLAY_CAPACITY = 1024U;
constexpr std::size_t MOTION_COMMAND_PREREAD_HIGH_WATERMARK = 100U;

// A RESET / GOTO can invalidate many queued commands at once.  The 250 us
// consumer discards only a bounded number per pass so an abnormal queue burst
// cannot monopolize one real-time cycle.
constexpr std::size_t
MOTION_COMMAND_STALE_DISCARD_LIMIT_PER_RUNTIME_PASS = 32U;

static_assert(
    MOTION_COMMAND_STALE_DISCARD_LIMIT_PER_RUNTIME_PASS > 0U &&
    MOTION_COMMAND_STALE_DISCARD_LIMIT_PER_RUNTIME_PASS <=
    MOTION_COMMAND_INGRESS_CAPACITY +
    MOTION_COMMAND_REPLAY_CAPACITY,
    "Stale-command discard budget must be positive and transport-bounded.");

template <typename T, std::size_t Capacity>
class FixedCapacitySpscRing
{
    static_assert(Capacity > 0U, "SPSC ring capacity must be greater than zero.");
    static_assert(
        std::is_trivially_copyable<T>::value,
        "SPSC ring payload must remain trivially copyable.");

public:
    FixedCapacitySpscRing() noexcept = default;

    FixedCapacitySpscRing(const FixedCapacitySpscRing&) = delete;
    FixedCapacitySpscRing& operator=(const FixedCapacitySpscRing&) = delete;

    static constexpr std::size_t capacity() noexcept
    {
        return Capacity;
    }

    // Producer thread only.
    bool ProducerTryPush(const T& value) noexcept
    {
        const std::uint64_t writeSequence =
            m_writeSequence.load(std::memory_order_relaxed);

        const std::uint64_t readSequence =
            m_readSequence.load(std::memory_order_acquire);

        if ((writeSequence - readSequence) >=
            static_cast<std::uint64_t>(Capacity))
        {
            return false;
        }

        m_storage[static_cast<std::size_t>(
            writeSequence % static_cast<std::uint64_t>(Capacity))] = value;

        m_writeSequence.store(
            writeSequence + 1ULL,
            std::memory_order_release);

        return true;
    }

    // Producer-only capacity proof. With exactly one producer, the consumer
    // can only make additional space until that producer's next publication.
    // Unlike size(), this reads the exact producer write position.
    bool ProducerHasCapacity(std::size_t count) const noexcept
    {
        if (count > Capacity) return false;
        const std::uint64_t writeSequence =
            m_writeSequence.load(std::memory_order_relaxed);
        const std::uint64_t readSequence =
            m_readSequence.load(std::memory_order_acquire);
        const std::uint64_t occupied = writeSequence - readSequence;
        return occupied <= static_cast<std::uint64_t>(Capacity) &&
            static_cast<std::uint64_t>(count) <=
                static_cast<std::uint64_t>(Capacity) - occupied;
    }

    // BASE42: both slots become visible at one release publication. A failed
    // capacity check copies neither command; the RT consumer cannot observe
    // the first leg alone, including when the pair straddles the ring end.
    bool ProducerTryPushPair(const T& first, const T& second) noexcept
    {
        const std::uint64_t writeSequence =
            m_writeSequence.load(std::memory_order_relaxed);
        const std::uint64_t readSequence =
            m_readSequence.load(std::memory_order_acquire);
        const std::uint64_t occupied = writeSequence - readSequence;
        if (Capacity < 2U || occupied > static_cast<std::uint64_t>(Capacity) ||
            static_cast<std::uint64_t>(Capacity) - occupied < 2ULL)
            return false;
        m_storage[static_cast<std::size_t>(
            writeSequence % static_cast<std::uint64_t>(Capacity))] = first;
        m_storage[static_cast<std::size_t>(
            (writeSequence + 1ULL) % static_cast<std::uint64_t>(Capacity))] = second;
        m_writeSequence.store(writeSequence + 2ULL, std::memory_order_release);
        return true;
    }

    // Consumer thread only.
    bool ConsumerTryPeek(T& outValue) const noexcept
    {
        return ConsumerTryPeekAt(0U, outValue);
    }

    // Consumer thread only.  A stable write snapshot is taken first; commands
    // published after that snapshot are intentionally not part of this read.
    bool ConsumerTryPeekAt(
        std::size_t offset,
        T& outValue) const noexcept
    {
        const std::uint64_t readSequence =
            m_readSequence.load(std::memory_order_relaxed);

        const std::uint64_t writeSequence =
            m_writeSequence.load(std::memory_order_acquire);

        const std::uint64_t available =
            writeSequence - readSequence;

        if (static_cast<std::uint64_t>(offset) >= available)
        {
            return false;
        }

        const std::uint64_t sequence =
            readSequence + static_cast<std::uint64_t>(offset);

        outValue =
            m_storage[static_cast<std::size_t>(
                sequence % static_cast<std::uint64_t>(Capacity))];

        return true;
    }

    // Consumer thread only.
    bool ConsumerTryPop(T& outValue) noexcept
    {
        const std::uint64_t readSequence =
            m_readSequence.load(std::memory_order_relaxed);

        const std::uint64_t writeSequence =
            m_writeSequence.load(std::memory_order_acquire);

        if (readSequence == writeSequence)
        {
            return false;
        }

        outValue =
            m_storage[static_cast<std::size_t>(
                readSequence % static_cast<std::uint64_t>(Capacity))];

        m_readSequence.store(
            readSequence + 1ULL,
            std::memory_order_release);

        return true;
    }

    // Consumer thread only.  A producer command that is being copied but has
    // not yet been release-published is not discarded; it becomes visible on
    // the following cycle and is then handled by the Epoch filter.
    void ConsumerDiscardAll() noexcept
    {
        const std::uint64_t writeSequence =
            m_writeSequence.load(std::memory_order_acquire);

        m_readSequence.store(
            writeSequence,
            std::memory_order_release);
    }

    std::size_t size() const noexcept
    {
        // 先讀 Consumer Sequence，再讀 Producer Sequence。
        // 若 Consumer 在兩次 Load 之間前進，這個 Snapshot 最多只會
        // 暫時高估 Queue 深度；反過來讀可能取得舊 write + 新 read，
        // 造成 unsigned underflow 或錯誤的 Queue Full 診斷。
        const std::uint64_t readSequence =
            m_readSequence.load(std::memory_order_acquire);

        const std::uint64_t writeSequence =
            m_writeSequence.load(std::memory_order_acquire);

        if (writeSequence <= readSequence)
        {
            return 0U;
        }

        const std::uint64_t count =
            writeSequence - readSequence;

        return static_cast<std::size_t>(
            (count > static_cast<std::uint64_t>(Capacity))
            ? static_cast<std::uint64_t>(Capacity)
            : count);
    }

    bool empty() const noexcept
    {
        return size() == 0U;
    }

    bool full() const noexcept
    {
        return size() >= Capacity;
    }

private:
    std::array<T, Capacity> m_storage{};

    // Producer owns writeSequence; consumer only observes it.
    alignas(64) std::atomic<std::uint64_t> m_writeSequence{ 0ULL };

    // Consumer owns readSequence; producer only observes it.
    alignas(64) std::atomic<std::uint64_t> m_readSequence{ 0ULL };
};


template <typename T, std::size_t Capacity>
class FixedCapacityConsumerReplayDeque
{
    static_assert(Capacity > 0U, "Replay capacity must be greater than zero.");
    static_assert(
        std::is_trivially_copyable<T>::value,
        "Replay payload must remain trivially copyable.");

public:
    FixedCapacityConsumerReplayDeque() noexcept = default;

    FixedCapacityConsumerReplayDeque(
        const FixedCapacityConsumerReplayDeque&) = delete;

    FixedCapacityConsumerReplayDeque& operator=(
        const FixedCapacityConsumerReplayDeque&) = delete;

    static constexpr std::size_t capacity() noexcept
    {
        return Capacity;
    }

    // RT consumer only.
    bool TryPushFront(const T& value) noexcept
    {
        if (m_count >= Capacity)
        {
            return false;
        }

        m_front =
            (m_front == 0U)
            ? (Capacity - 1U)
            : (m_front - 1U);

        m_storage[m_front] = value;
        ++m_count;

        return true;
    }

    // RT consumer only.
    bool TryPeekFront(T& outValue) const noexcept
    {
        return TryPeekAt(0U, outValue);
    }

    // RT consumer only.
    bool TryPeekAt(
        std::size_t offset,
        T& outValue) const noexcept
    {
        if (offset >= m_count)
        {
            return false;
        }

        outValue =
            m_storage[(m_front + offset) % Capacity];

        return true;
    }

    // RT consumer only.
    bool TryPopFront(T& outValue) noexcept
    {
        if (m_count == 0U)
        {
            return false;
        }

        outValue = m_storage[m_front];
        m_front = (m_front + 1U) % Capacity;
        --m_count;

        return true;
    }

    // RT consumer only.
    void Clear() noexcept
    {
        m_front = 0U;
        m_count = 0U;
    }

    std::size_t size() const noexcept
    {
        return m_count;
    }

    bool empty() const noexcept
    {
        return m_count == 0U;
    }

private:
    std::array<T, Capacity> m_storage{};
    std::size_t m_front = 0U;
    std::size_t m_count = 0U;
};


template <
    typename T,
    std::size_t IngressCapacity,
    std::size_t ReplayCapacity>
    class FixedCapacitySpscCommandChannel
{
public:
    FixedCapacitySpscCommandChannel() noexcept = default;

    FixedCapacitySpscCommandChannel(
        const FixedCapacitySpscCommandChannel&) = delete;

    FixedCapacitySpscCommandChannel& operator=(
        const FixedCapacitySpscCommandChannel&) = delete;

    static constexpr std::size_t ingress_capacity() noexcept
    {
        return IngressCapacity;
    }

    static constexpr std::size_t replay_capacity() noexcept
    {
        return ReplayCapacity;
    }

    // NC / MDI producer only.
    bool ProducerTryPush(const T& value) noexcept
    {
        return m_ingress.ProducerTryPush(value);
    }

    // NC / MDI producer only; preserve the ingress pair linearization point.
    bool ProducerHasCapacity(std::size_t count) const noexcept
    {
        return m_ingress.ProducerHasCapacity(count);
    }

    bool ProducerTryPushPair(const T& first, const T& second) noexcept
    {
        return m_ingress.ProducerTryPushPair(first, second);
    }

    // 250 us Motion consumer only.  Replay commands always have priority,
    // because they represent segments restored by B2 reverse traversal.
    bool ConsumerTryPeek(T& outValue) const noexcept
    {
        if (m_replay.TryPeekFront(outValue))
        {
            return true;
        }

        return m_ingress.ConsumerTryPeek(outValue);
    }

    // 250 us Motion consumer only.
    bool ConsumerTryPeekAt(
        std::size_t offset,
        T& outValue) const noexcept
    {
        const std::size_t replayCount =
            m_replay.size();

        if (offset < replayCount)
        {
            return m_replay.TryPeekAt(offset, outValue);
        }

        return m_ingress.ConsumerTryPeekAt(
            offset - replayCount,
            outValue);
    }

    // 250 us Motion consumer only.
    bool ConsumerTryPop(T& outValue) noexcept
    {
        if (m_replay.TryPopFront(outValue))
        {
            PublishReplaySize();
            return true;
        }

        return m_ingress.ConsumerTryPop(outValue);
    }

    // 250 us Motion consumer only.  Used by B2 reverse crossing.
    bool ConsumerTryPushFront(const T& value) noexcept
    {
        if (!m_replay.TryPushFront(value))
        {
            return false;
        }

        PublishReplaySize();
        return true;
    }

    // 250 us Motion consumer only.
    void ConsumerDiscardAll() noexcept
    {
        m_replay.Clear();
        PublishReplaySize();
        m_ingress.ConsumerDiscardAll();
    }

    std::size_t ingress_size() const noexcept
    {
        return m_ingress.size();
    }

    std::size_t replay_size() const noexcept
    {
        return m_replayPublishedSize.load(
            std::memory_order_acquire);
    }

    // Cross-thread diagnostic / pre-read throttle.  It is intentionally an
    // approximate snapshot and never grants permission to access queue slots.
    std::size_t size() const noexcept
    {
        return ingress_size() + replay_size();
    }

    bool empty() const noexcept
    {
        return size() == 0U;
    }

    bool producer_full() const noexcept
    {
        return m_ingress.full();
    }

private:
    void PublishReplaySize() noexcept
    {
        m_replayPublishedSize.store(
            m_replay.size(),
            std::memory_order_release);
    }

    FixedCapacitySpscRing<T, IngressCapacity> m_ingress{};
    FixedCapacityConsumerReplayDeque<T, ReplayCapacity> m_replay{};

    // Replay storage is RT-owned, but NC may read total queue depth.
    alignas(64) std::atomic<std::size_t> m_replayPublishedSize{ 0U };
};
