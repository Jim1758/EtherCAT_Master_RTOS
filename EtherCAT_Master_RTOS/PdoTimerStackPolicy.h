#pragma once
#include <cstdint>

// EDM308 FIX5: the PDO timer has its own allocation, independent of the
// executable's main-thread stack. Check once during warm-up before OP/I/O.
namespace PdoTimerStack
{
constexpr unsigned long RequiredBytes = 131072UL;
constexpr unsigned long RequiredWarmupFreeBytes = 65536UL;
constexpr long WarmupPending = 0L;
constexpr long WarmupReady = 1L;
constexpr long WarmupRejected = -1L;

enum class Result : std::uint8_t
{ QueryFailed, InvalidCounters, StackSize, Headroom, Bounds, Ready };

inline const char* Name(Result result) noexcept
{
    switch (result)
    {
    case Result::QueryFailed: return "QUERY_FAILED";
    case Result::InvalidCounters: return "INVALID_COUNTERS";
    case Result::StackSize: return "STACK_SIZE";
    case Result::Headroom: return "HEADROOM";
    case Result::Bounds: return "BOUNDS";
    case Result::Ready: return "OK";
    }
    return "UNKNOWN";
}

inline bool ContainsTop(std::uint64_t base, std::uint64_t limit,
    std::uint64_t top) noexcept
{
    if (!base || !limit || base == limit || !top) return false;
    const auto low = base < limit ? base : limit;
    const auto high = base < limit ? limit : base;
    return top >= low && top <= high;
}

inline Result Evaluate(bool queried, std::uint64_t size,
    std::uint64_t available, std::uint64_t base, std::uint64_t limit,
    std::uint64_t top) noexcept
{
    if (!queried) return Result::QueryFailed;
    if (!size || available > size) return Result::InvalidCounters;
    if (size < RequiredBytes) return Result::StackSize;
    if (available < RequiredWarmupFreeBytes) return Result::Headroom;
    if (!ContainsTop(base, limit, top)) return Result::Bounds;
    return Result::Ready;
}
}
