#pragma once
#include <cstdint>

// EDM340: value-only machining clocks. NC parses minutes to exact 60-us
// quanta; the RT parent owns elapsed time. This grants no motion authority.
namespace NCMachiningTime
{
constexpr std::uint64_t QuantumUs=60ULL;
constexpr std::uint64_t MaxDurationUs=86400000000ULL;
enum class TimerState : std::uint8_t
{ Disabled=0U,Waiting=1U,Running=2U,Paused=3U,Expired=4U,Completed=5U };
enum class TimePhase : std::uint8_t
{ Disabled=0U,Machining=1U,Additional=2U,Finishing=3U,Verification=4U,
  TimedOut=5U,Completed=6U,Fault=7U,TimedFinishing=8U };
enum class PauseReason : std::uint8_t
{ None=0U,Hold=1U,Flush=2U,Gap=3U,Returning=4U,Fault=5U,
  Qualification=6U,Waiting=7U };
struct Config
{
    std::uint64_t totalLimitUs=0ULL,additionalUs=0ULL;
};
inline bool ValidDuration(std::uint64_t value) noexcept
{return value<=MaxDurationUs&&value%QuantumUs==0ULL;}
inline bool ValidConfig(const Config& value) noexcept
{return ValidDuration(value.totalLimitUs)&&ValidDuration(value.additionalUs);}
inline bool SameConfig(const Config& a,const Config& b) noexcept
{return a.totalLimitUs==b.totalLimitUs&&a.additionalUs==b.additionalUs;}
inline bool Enabled(const Config& value) noexcept
{return value.totalLimitUs!=0ULL||value.additionalUs!=0ULL;}
}
