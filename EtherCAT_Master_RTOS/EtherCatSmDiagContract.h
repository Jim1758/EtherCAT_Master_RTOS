#pragma once

#include <stdint.h>

#pragma pack(push, 1)

// ============================================================================
// Stage 12E.4B - OSCARMAX EtherCAT SyncManager Live Diagnostic Contract
// ============================================================================
//
// Shared Memory name (created by SHMManager in Stage 12E.4C):
//     OSCARMAX_ECAT_SM_DIAG
//
// Purpose:
//     Publish the live ESC SyncManager register image captured by the
//     Priority-64 owner-safe diagnostic sweep to the Windows ENI Tool.
//
// Data path:
//     ESC registers
//       -> Priority-64 EtherCatMaster_DC_Runtime.cpp RT shadow
//       -> Priority-50 System_EDM_SINKER_MODE.cpp publisher
//       -> OSCARMAX_ECAT_SM_DIAG
//       -> Windows ENI Tool (read-only)
//
// Design rules:
//   1. This is a NEW diagnostic Shared Memory channel. It does NOT change
//      EDM_SINKER_MODE, OSCARMAX_ECAT_DIAG or OSCARMAX_ECAT_SERVICE ABI.
//   2. Windows is read-only. No SyncManager write/control path exists here.
//   3. Fixed-width scalar fields only: no bool, pointer or STL type.
//   4. Pack=1 for deterministic C++ / C# layout.
//   5. One global Sequence protects the whole snapshot:
//        writer: Sequence++ -> odd, write, barrier, Sequence++ -> even
//        reader: accept only same even Sequence before/after copy.
//   6. Entries are fixed [slave][SM] slots:
//        FlatIndex = SlavePosition * 16 + SmIndex
//      so Windows does not need a variable-length parser.
//   7. Present means the Runtime/ENI configuration contains that SM.
//      Valid means a live 8-byte ESC register read has succeeded.
//   8. RegisterAddress is the physical ESC address:
//        0x0800 + SmIndex * 8
//
// Live 8-byte SyncManager register layout:
//     +0..1 Start Address
//     +2..3 Length
//     +4    Control
//     +5    Status
//     +6    Activate
//     +7    PDI Control
//
// Source fingerprint:
//     OSCARMAX_ECAT_SM_DIAG_CONTRACT_12E4B_20260826
// ============================================================================

static constexpr uint32_t SHM_ECAT_SM_DIAG_MAGIC =
0x4D534345u; // little-endian bytes "ECSM"

static constexpr uint16_t SHM_ECAT_SM_DIAG_VERSION_MAJOR = 1u;
static constexpr uint16_t SHM_ECAT_SM_DIAG_VERSION_MINOR = 0u;

static constexpr uint32_t SHM_ECAT_SM_DIAG_MAX_SLAVES = 128u;
static constexpr uint32_t SHM_ECAT_SM_DIAG_MAX_SYNC_MANAGERS = 16u;

static constexpr uint32_t SHM_ECAT_SM_DIAG_ENTRY_COUNT =
SHM_ECAT_SM_DIAG_MAX_SLAVES *
SHM_ECAT_SM_DIAG_MAX_SYNC_MANAGERS;

// 64-byte global header.
struct SHM_ECAT_SmDiagHeader
{
    uint32_t Magic;               // SHM_ECAT_SM_DIAG_MAGIC
    uint16_t VersionMajor;        // ABI major
    uint16_t VersionMinor;        // ABI minor
    uint32_t StructSize;          // sizeof(SHM_ECAT_SmDiagData)

    uint32_t Sequence;            // seqlock: even = stable
    uint32_t Heartbeat;           // Priority-50 publish heartbeat
    uint64_t PublishCount;        // completed P50 publishes

    uint16_t SlaveCount;          // current Runtime slave count
    uint16_t MaxSyncManagers;     // always 16 for ABI v1.0

    // Reserved for future global observability.
    // Stage 12E.4B does not assign semantics yet.
    uint32_t Reserved32[8];
};

// One fixed 48-byte live SM slot.
struct SHM_ECAT_SmDiagEntry
{
    uint16_t SlavePosition;       // zero-based topology position
    uint8_t  SmIndex;             // physical ESC SM number: 0..15
    uint8_t  Present;             // configured Runtime/ENI SM exists

    uint8_t  Valid;               // live ESC read is valid
    uint8_t  ControlByte;         // ESC SM +4
    uint8_t  StatusByte;          // ESC SM +5
    uint8_t  ActivateByte;        // ESC SM +6
    uint8_t  PdiControlByte;      // ESC SM +7

    uint8_t  Reserved8[3];

    uint16_t RegisterAddress;     // 0x0800 + SmIndex*8
    uint16_t StartAddress;        // ESC SM +0..1
    uint16_t Length;              // ESC SM +2..3
    uint16_t Reserved16;

    int32_t  LastTransportResult; // >0 WKC, 0 WKC0, <0 bounded transport fault

    uint64_t LastUpdateTick;      // Priority-64 PDO tick of last probe
    uint64_t ProbeSuccess;        // per-SM successful probes
    uint64_t ProbeFailure;        // per-SM failed probes
};

// Entire read-only Windows diagnostic mapping.
struct SHM_ECAT_SmDiagData
{
    SHM_ECAT_SmDiagHeader Header;

    SHM_ECAT_SmDiagEntry
        Entries[SHM_ECAT_SM_DIAG_ENTRY_COUNT];
};

#if !defined(__INTELLISENSE__)
static_assert(
    sizeof(SHM_ECAT_SmDiagHeader) == 64,
    "SHM_ECAT_SmDiagHeader ABI changed; update ENI Tool C# contract.");

static_assert(
    sizeof(SHM_ECAT_SmDiagEntry) == 48,
    "SHM_ECAT_SmDiagEntry ABI changed; update ENI Tool C# contract.");

static_assert(
    sizeof(SHM_ECAT_SmDiagData) == 98368,
    "SHM_ECAT_SmDiagData ABI changed; update ENI Tool C# contract.");
#endif

#pragma pack(pop)
