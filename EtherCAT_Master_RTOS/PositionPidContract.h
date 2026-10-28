#pragma once
#include <cstdint>
#include <cstddef>
#include <cmath>

// TUNE15: independent v1 ABI. No legacy shared-memory layout changes.
// G00 = 0, IDLE = 1. Only these four gains may change; protection limits,
// encoder scales, FLUSH/EDM gains and coordinates are outside this interface.
constexpr std::uint32_t POSITION_PID_MAGIC = 0x44495050U;
constexpr std::uint32_t POSITION_PID_SIZE = 832U;
constexpr std::uint32_t POSITION_PID_AXIS_COUNT = 6U;
constexpr std::uint32_t POSITION_PID_PUBLISHED = 1U;
constexpr std::uint32_t POSITION_PID_SHUTTING_DOWN = 2U;
enum class PositionPidMailboxState : std::uint32_t
{ Idle=0, Writing=1, Ready=2, Processing=3, Complete=4 };
enum class PositionPidStatus : std::uint32_t
{ None=0, Success=1, Busy=2, Stale=3, NotAllowed=4, InvalidCommand=5, DomainRejected=6, Unavailable=8 };
enum class PositionPidReason : std::uint32_t
{ Ready=0, RuntimeUnavailable=1, NcBusy=2, MotionBusy=3, AxisFault=4,
  NotSettled=5, Safety=6, TuneBusy=7, InvalidAxis=8, InvalidGain=9,
  Conflict=10, Expired=11, HoldTransition=12 };
#pragma pack(push, 1)
struct PositionPidHeader
{
    std::uint32_t Magic;
    std::uint16_t VersionMajor, VersionMinor;
    std::uint32_t StructSize, Sequence, Heartbeat, Flags;
    std::uint64_t RuntimeEpoch;
    std::uint8_t Reserved[32];
};
struct PositionPidMailbox { std::uint32_t State, RequestSequence; std::uint8_t Reserved[56]; };
struct PositionPidContext
{
    std::uint64_t Generation, RuntimeCycle;
    std::uint32_t AxisCount, ActiveAxisMask, CanApplyMask, BlockReason;
    std::uint8_t Reserved[32];
};
struct PositionPidGains { double Kp, Ki, Kd, Kvff; };
struct PositionPidValues { PositionPidGains Axis[POSITION_PID_AXIS_COUNT][2]; };
struct PositionPidRequest
{
    std::uint64_t SessionId, RequestId, RuntimeEpoch, ExpectedGeneration;
    std::uint32_t Axis, Bank;
    PositionPidGains Values;
    std::uint8_t Reserved[56];
};
struct PositionPidAck
{
    std::uint64_t SessionId, RequestId, RuntimeEpoch, Generation;
    std::uint32_t Status, Reason, Axis, Bank;
    PositionPidGains Applied;
    char Detail[48];
};
struct PositionPidData
{
    PositionPidHeader Header;
    PositionPidMailbox Mailbox;
    PositionPidContext Context;
    PositionPidValues Values;
    PositionPidRequest Request;
    PositionPidAck Ack;
};
#pragma pack(pop)
static_assert(sizeof(PositionPidData)==POSITION_PID_SIZE,"Position PID v1 ABI size");
static_assert(sizeof(PositionPidContext)==64U && sizeof(PositionPidRequest)==128U && sizeof(PositionPidAck)==128U,"Position PID record size");
static_assert(offsetof(PositionPidData,Context)==128U && offsetof(PositionPidData,Values)==192U,"Position PID observation offsets");
static_assert(offsetof(PositionPidData,Request)==576U && offsetof(PositionPidData,Ack)==704U,"Position PID command offsets");
static_assert(offsetof(PositionPidRequest,Values)==40U && offsetof(PositionPidAck,Applied)==48U,"Position PID gains offsets");
namespace PositionPid
{
// Numerical envelope only, not a promise of stability or tuned performance.
// The recommendation algorithm keeps its tighter configured tuning bounds.
inline bool Valid(const PositionPidGains& g) noexcept
{
    return std::isfinite(g.Kp) && g.Kp>0.0 && g.Kp<=10000.0 &&
        std::isfinite(g.Ki) && g.Ki>=0.0 && g.Ki<=10000.0 &&
        std::isfinite(g.Kd) && g.Kd>=0.0 && g.Kd<=1000.0 &&
        std::isfinite(g.Kvff) && g.Kvff>=0.0 && g.Kvff<=10.0;
}
inline bool Same(const PositionPidGains& a,const PositionPidGains& b) noexcept
{ return a.Kp==b.Kp && a.Ki==b.Ki && a.Kd==b.Kd && a.Kvff==b.Kvff; }
inline bool Canonical(const PositionPidRequest& r) noexcept
{
    if(!r.SessionId || !r.RequestId || !r.RuntimeEpoch || !r.ExpectedGeneration ||
        r.Axis>=POSITION_PID_AXIS_COUNT || r.Bank>1U || !Valid(r.Values)) return false;
    for(const auto b:r.Reserved) if(b!=0U)return false;
    return true;
}
}
