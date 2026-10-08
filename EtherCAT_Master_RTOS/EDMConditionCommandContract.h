#pragma once
#include <cstdint>
#include <cstddef>

// EDM19 additive mailbox. Windows serializes all writers with the named mutex
// Global\OSCARMAX_EDM_CONDITION_WRITER. The NC owner alone applies commands/acks.
// Header.Sequence protects Context and Ack only. RequestSequence protects the
// Windows-owned Request. State is an aligned interlocked ownership word.
constexpr std::uint32_t EDM_CONDITION_MAGIC = 0x434D4445U;
constexpr std::uint32_t EDM_CONDITION_SIZE = 512U;
constexpr std::uint32_t EDM_CONDITION_PUBLISHED = 1U;
constexpr std::uint32_t EDM_CONDITION_WORKER_AVAILABLE = 2U;
constexpr std::uint32_t EDM_CONDITION_SHUTTING_DOWN = 4U;
enum class EDMConditionMailboxState : std::uint32_t
{ Idle = 0U, Writing = 1U, Ready = 2U, Processing = 3U, Complete = 4U };
enum class EDMConditionAckStatus : std::uint32_t
{ None = 0U, Success = 1U, Busy = 2U, Stale = 3U, NotAllowed = 4U,
  InvalidCommand = 5U, DomainRejected = 6U, LoadFailure = 7U, Unavailable = 8U };

#pragma pack(push, 1)
struct EDMConditionHeader
{
    std::uint32_t Magic;
    std::uint16_t VersionMajor, VersionMinor;
    std::uint32_t StructSize, Sequence, Heartbeat, Flags;
    std::uint64_t RuntimeEpoch;
    std::uint8_t Reserved[32];
};
struct EDMConditionMailbox
{
    std::uint32_t State, RequestSequence;
    std::uint8_t Reserved[56];
};
struct EDMConditionContext
{
    std::uint64_t Generation, ControlGeneration;
    std::uint32_t ProfileId, DefinitionRevision, CatalogRevision, TableRevision;
    std::uint32_t SchemaVersion, TableId;
    std::uint16_t ECode, Flags; // bit0 edit permitted, bit1 reload permitted
    std::uint8_t Reserved[20];
};
struct EDMConditionRequest
{
    std::uint64_t SessionId, RequestId, RuntimeEpoch;
    EDMConditionContext Expected;
    std::uint32_t Operation, TableId;
    std::uint16_t ECode, FieldId, StageId, Reserved16; // SetActual: StageId=0 keeps anchor; >0 selects anchor atomically
    std::int32_t Step;
    std::uint32_t Reserved32;
    std::int64_t ActualValue; // exact millionths; never binary float
    std::uint64_t Reserved64;
};
struct EDMConditionAck
{
    std::uint64_t SessionId, RequestId, RuntimeEpoch;
    EDMConditionContext After;
    std::uint32_t Status, DomainError, ErrorLine, Reserved;
    char Detail[88];
};
struct EDMConditionData
{
    EDMConditionHeader Header;
    EDMConditionMailbox Mailbox;
    EDMConditionContext Context;
    EDMConditionRequest Request;
    EDMConditionAck Ack;
};
#pragma pack(pop)
static_assert(sizeof(EDMConditionHeader) == 64U, "EDM19 header ABI");
static_assert(sizeof(EDMConditionContext) == 64U, "EDM19 context ABI");
static_assert(sizeof(EDMConditionRequest) == 128U, "EDM19 request ABI");
static_assert(sizeof(EDMConditionAck) == 192U, "EDM19 ack ABI");
static_assert(sizeof(EDMConditionData) == EDM_CONDITION_SIZE, "EDM19 mapping ABI");
static_assert(offsetof(EDMConditionData, Mailbox) == 64U, "EDM19 state alignment");
static_assert(offsetof(EDMConditionData, Context) == 128U, "EDM19 context offset");
static_assert(offsetof(EDMConditionData, Request) == 192U, "EDM19 request offset");
static_assert(offsetof(EDMConditionData, Ack) == 320U, "EDM19 ack offset");
static_assert(offsetof(EDMConditionRequest, ActualValue) == 112U, "EDM19 exact value offset");
