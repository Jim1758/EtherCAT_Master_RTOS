#pragma once
#include <cstdint>
#include <cstddef>

// Independent additive ABI. All distances mm; speeds mm/min; cutting scale
// mm/(V*min). The NC owner alone writes Context, Values and Ack. Windows SDK
// writers serialize with Global\\OSCARMAX_EDM_PROCESS_TUNING_WRITER.
constexpr std::uint32_t EDM_PROCESS_TUNING_MAGIC = 0x544D4445U;
constexpr std::uint32_t EDM_PROCESS_TUNING_SIZE = 2432U;
constexpr std::uint32_t EDM_PROCESS_TUNING_PUBLISHED = 1U;
constexpr std::uint32_t EDM_PROCESS_TUNING_SHUTTING_DOWN = 2U;
constexpr std::uint32_t EDM_PROCESS_TUNING_READY = 1U;
constexpr std::uint32_t EDM_PROCESS_TUNING_CAN_APPLY = 2U;
constexpr std::uint32_t EDM_PROCESS_TUNING_PROCESS = 1U;
constexpr std::uint32_t EDM_PROCESS_TUNING_DISCHARGE = 2U;
constexpr std::uint32_t EDM_PROCESS_TUNING_FLUSH = 4U;
enum class EDMProcessTuningMailboxState : std::uint32_t
{ Idle = 0U, Writing = 1U, Ready = 2U, Processing = 3U, Complete = 4U };
enum class EDMProcessTuningStatus : std::uint32_t
{ None = 0U, Success = 1U, Busy = 2U, Stale = 3U, NotAllowed = 4U,
  InvalidCommand = 5U, DomainRejected = 6U, Unavailable = 8U };
#pragma pack(push, 1)
struct EDMProcessTuningHeader
{
    std::uint32_t Magic;
    std::uint16_t VersionMajor, VersionMinor;
    std::uint32_t StructSize, Sequence, Heartbeat, Flags;
    std::uint64_t RuntimeEpoch;
    std::uint8_t Reserved[32];
};
struct EDMProcessTuningMailbox
{
    std::uint32_t State, RequestSequence;
    std::uint8_t Reserved[56];
};
struct EDMProcessTuningContext
{
    std::uint64_t Generation;
    std::uint32_t ProfileRevision, MachineProfileId, SchemaVersion, Flags;
    std::uint32_t ActiveAxisMask, PhysicalApplied; // PhysicalApplied is always 0.
    std::uint8_t Reserved[32];
};
struct EDMProcessTuningParameters
{
    std::uint32_t SchemaVersion, Revision, MachineProfileId, Mode;
    // bit0 limits confirmed; bit1 idle short enabled; bit2 machining short enabled
    std::uint32_t Flags, ShortEnterMs, ShortExitMs, Reserved32;
    double PositiveGain[3], NegativeGain[3], PositiveBreakV[2], NegativeBreakV[2];
    double CuttingScaleMmPerVoltMin, MaxFeedMmPerMin, MaxRetreatMmPerMin, DeadbandV;
    double ShortIdleV, ShortMachiningV, ShortHysteresisV, ShortRetreatMmPerMin;
    double FinalApproachMmPerMin, CenterRetractMmPerMin, MainRetractMmPerMin;
    double CenterReturnMmPerMin, MainReturnMmPerMin, PathRetractMmPerMin;
    double InitialSlowMmPerMin, ManualReturnMmPerMin;
    double FinalApproachDistanceMm, PathDistanceMm, InitialSlowDistanceMm;
    std::uint8_t Reserved[120];
};
struct EDMProcessTuningAxisGain
{
    std::uint32_t InheritCnc, Reserved;
    double Kp, Ki, Kd, Kvff;
};
struct EDMProcessTuningValues
{
    EDMProcessTuningParameters Process;
    EDMProcessTuningAxisGain Discharge[8], Flush[8];
};
struct EDMProcessTuningRequest
{
    std::uint64_t SessionId, RequestId, RuntimeEpoch, ExpectedGeneration;
    std::uint32_t SelectionMask;
    std::uint8_t Reserved[28];
    EDMProcessTuningValues Values; // Unselected blocks MUST be all zero.
};
struct EDMProcessTuningAck
{
    std::uint64_t SessionId, RequestId, RuntimeEpoch, Generation;
    std::uint32_t Status, DomainError, SelectionMask, Reserved;
    char Detail[80];
};
struct EDMProcessTuningData
{
    EDMProcessTuningHeader Header;
    EDMProcessTuningMailbox Mailbox;
    EDMProcessTuningContext Context;
    EDMProcessTuningValues Values;
    EDMProcessTuningRequest Request;
    EDMProcessTuningAck Ack;
};
#pragma pack(pop)
static_assert(sizeof(EDMProcessTuningHeader) == 64U, "EDM tuning header ABI");
static_assert(sizeof(EDMProcessTuningContext) == 64U, "EDM tuning context ABI");
static_assert(sizeof(EDMProcessTuningParameters) == 384U, "EDM tuning parameters ABI");
static_assert(sizeof(EDMProcessTuningAxisGain) == 40U, "EDM tuning axis gain ABI");
static_assert(sizeof(EDMProcessTuningValues) == 1024U, "EDM tuning values ABI");
static_assert(sizeof(EDMProcessTuningRequest) == 1088U, "EDM tuning request ABI");
static_assert(sizeof(EDMProcessTuningAck) == 128U, "EDM tuning ack ABI");
static_assert(sizeof(EDMProcessTuningData) == EDM_PROCESS_TUNING_SIZE, "EDM tuning map ABI");
static_assert(offsetof(EDMProcessTuningData, Context) == 128U, "EDM tuning context offset");
static_assert(offsetof(EDMProcessTuningData, Values) == 192U, "EDM tuning values offset");
static_assert(offsetof(EDMProcessTuningData, Request) == 1216U, "EDM tuning request offset");
static_assert(offsetof(EDMProcessTuningData, Ack) == 2304U, "EDM tuning ack offset");
