#pragma once
#include "NCTranslationSnapshot.h"
#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>

// Single NC writer; RT readers never read a concurrently mutable plain POD.
// All word accesses participate in one SC order. Two generation reads fence
// one bounded exact comparison without retries, hashing, locks or allocation.
class MotionNCTranslationPublication
{
public:
    static constexpr unsigned WordCount = sizeof(NCTranslationSnapshot) / sizeof(std::uint64_t);
    MotionNCTranslationPublication() noexcept
    {
        for (auto& word : m_words) word.store(0ULL, std::memory_order_relaxed);
    }
    bool Publish(const NCTranslationSnapshot& snapshot) noexcept
    {
        if (!IsNCTranslationSnapshotValid(snapshot)) return false;
        if (m_generation.load(std::memory_order_seq_cst) != 0ULL)
            return Matches(snapshot); // Idempotent publication never replaces a live run.
        if (snapshot.generation <= m_lastPublishedGeneration) return false;
        const unsigned char* bytes = reinterpret_cast<const unsigned char*>(&snapshot);
        for (unsigned i = 0U; i < WordCount; ++i)
        {
            std::uint64_t bits = 0ULL;
            std::memcpy(&bits, bytes + i * sizeof(bits), sizeof(bits));
            m_words[i].store(bits, std::memory_order_seq_cst);
        }
        m_lastPublishedGeneration = snapshot.generation;
        m_generation.store(snapshot.generation, std::memory_order_seq_cst);
        return true;
    }
    void Retire() noexcept { m_generation.store(0ULL, std::memory_order_seq_cst); }
    std::uint64_t Generation() const noexcept { return m_generation.load(std::memory_order_seq_cst); }
    bool Matches(const NCTranslationSnapshot& snapshot) const noexcept
    {
        const std::uint64_t generation = m_generation.load(std::memory_order_seq_cst);
        if (generation == 0ULL || generation != snapshot.generation ||
            !IsNCTranslationSnapshotValid(snapshot)) return false;
        const unsigned char* bytes = reinterpret_cast<const unsigned char*>(&snapshot);
        for (unsigned i = 0U; i < WordCount; ++i)
        {
            std::uint64_t bits = 0ULL;
            std::memcpy(&bits, bytes + i * sizeof(bits), sizeof(bits));
            if (m_words[i].load(std::memory_order_seq_cst) != bits) return false;
        }
        return m_generation.load(std::memory_order_seq_cst) == generation;
    }
private:
    std::array<std::atomic<std::uint64_t>, WordCount> m_words{};
    std::atomic<std::uint64_t> m_generation{0ULL};
    std::uint64_t m_lastPublishedGeneration = 0ULL; // NC writer only, never reset/reused.
};
static_assert(sizeof(NCTranslationSnapshot) % sizeof(std::uint64_t) == 0U,
    "Translation publication requires complete atomic words.");
#if defined(_WIN64) || defined(__x86_64__) || defined(__aarch64__)
static_assert((std::is_same<std::uint64_t, unsigned long>::value && ATOMIC_LONG_LOCK_FREE == 2) ||
    (std::is_same<std::uint64_t, unsigned long long>::value && ATOMIC_LLONG_LOCK_FREE == 2),
    "The fixed translation publication requires native lock-free 64-bit atomics.");
#endif
