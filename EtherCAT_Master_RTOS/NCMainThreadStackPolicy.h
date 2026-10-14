#pragma once
#include <cstdint>

// EDM85 FIX1: the RTSS main/NC thread has a fixed, committed stack.
// This startup-only policy does not change timer stacks, priorities or motion.
namespace NCMainStack
{
constexpr std::uint64_t RequiredBytes = 131072ULL;
constexpr std::uint64_t RequiredStartupFreeBytes = 65536ULL;
enum class Result : std::uint8_t
{ Ready, QueryFailed, InvalidCounters, StackSize, StartupHeadroom, StackBounds };
inline const char* Name(Result result) noexcept
{
    switch (result)
    {
    case Result::Ready: return "OK";
    case Result::QueryFailed: return "QUERY_FAILED";
    case Result::InvalidCounters: return "INVALID_COUNTERS";
    case Result::StackSize: return "STACK_SIZE";
    case Result::StartupHeadroom: return "STARTUP_HEADROOM";
    case Result::StackBounds: return "STACK_BOUNDS";
    }
    return "UNKNOWN";
}
inline bool ContainsTop(std::uint64_t base, std::uint64_t limit,
    std::uint64_t top) noexcept
{
    if (!base || !limit || base == limit || !top) return false;
    const auto low = base < limit ? base : limit;
    const auto high = base < limit ? limit : base;
    // Normalize the API's two boundary addresses; do not assume their names
    // denote a particular growth direction or an inclusive allocation span.
    return top >= low && top <= high;
}
inline Result Evaluate(bool queried, std::uint64_t size, std::uint64_t available,
    std::uint64_t base, std::uint64_t limit, std::uint64_t top) noexcept
{
    if (!queried) return Result::QueryFailed;
    if (!size || available > size) return Result::InvalidCounters;
    if (size < RequiredBytes) return Result::StackSize;
    if (available < RequiredStartupFreeBytes) return Result::StartupHeadroom;
    if (!ContainsTop(base, limit, top)) return Result::StackBounds;
    return Result::Ready;
}
}
