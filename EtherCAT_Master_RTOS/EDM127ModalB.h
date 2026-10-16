#pragma once
#include "EDMSixAxisModeFlush.h"
#include <limits>

// NC-owned modal value. It cannot authorize motion, replay a block or publish PDOs.
namespace EDM127
{
struct Selection
{
    EDM116::Mode mode = EDM116::Mode::Invalid;
    int pc = -1;
    std::uint64_t dispatch = 0ULL, revision = 0ULL;
};
inline bool Valid(const Selection& value) noexcept
{ return EDM116::ValidMode(value.mode) && value.pc >= 0 && value.dispatch && value.revision; }
inline Selection Initial(int armPC, std::uint64_t armDispatch) noexcept
{
    Selection value{};
    if (armPC >= 0 && armDispatch)
    { value.mode = EDM116::Mode::B0; value.pc = armPC; value.dispatch = armDispatch; value.revision = 1ULL; }
    return value;
}
inline bool Select(const Selection& previous, double word, int pc, std::uint64_t dispatch,
    Selection& output) noexcept
{
    EDM116::Mode mode{};
    if (!Valid(previous) || !EDM116::ParseMode(word, mode) || pc <= previous.pc ||
        dispatch <= previous.dispatch || previous.revision == (std::numeric_limits<std::uint64_t>::max)()) return false;
    Selection candidate{}; candidate.mode = mode; candidate.pc = pc;
    candidate.dispatch = dispatch; candidate.revision = previous.revision + 1ULL;
    output = candidate; return true;
}
inline bool Matches(const Selection& value, const EDM116::Policy& policy) noexcept
{
    return Valid(value) && value.mode == policy.mode && value.pc == policy.modeOriginPC &&
        value.dispatch == policy.modeOriginDispatch && value.revision == policy.modeOriginRevision;
}
}
