#pragma once

#include <stdint.h>

#pragma pack(push, 1)

// ============================================================================
// Stage 12E.5B - OSCARMAX EtherCAT FMMU Live Diagnostic Contract
// ============================================================================
//
// Shared Memory name (created by SHMManager in Stage 12E.5C):
//     OSCARMAX_ECAT_FMMU_DIAG
//
// Purpose:
//     Publish the live ESC FMMU register image captured by the Priority-64
//     owner-safe diagnostic sweep to the Windows ENI Tool.
//
// Data path:
//     ESC registers
//       -> Priority-64 EtherCatMaster_DC_Runtime.cpp RT shadow
//       -> Priority-50 System_EDM_SINKER_MODE.cpp publisher
//       -> OSCARMAX_ECAT_FMMU_DIAG
//       -> Windows ENI Tool (read-only)
//
// Design rules:
//   1. NEW diagnostic Shared Memory channel. Existing EDM_SINKER_MODE,
//      OSCARMAX_ECAT_DIAG, OSCARMAX_ECAT_SERVICE and OSCARMAX_ECAT_SM_DIAG
//      ABIs remain unchanged.
//   2. Windows is read-only. No FMMU write/control path exists here.
//   3. Fixed-width scalar fields only: no bool, pointer or STL type.
//   4. Pack=1 for deterministic C++ / C# layout.
//   5. One global Sequence protects the whole snapshot:
//        writer: Sequence++ -> odd, write, barrier, Sequence++ -> even
//        reader: accept only same even Sequence before/after copy.
//   6. Entries are fixed [slave][FMMU] slots:
//        FlatIndex = SlavePosition * 16 + FmmuIndex
//      so Windows does not need a variable-length parser.
//   7. Present means Runtime/ENI contains that FMMU.
//      Valid means a live 16-byte ESC register read has succeeded.
//   8. RegisterAddress is the physical ESC address:
//        0x0600 + FmmuIndex * 16
//
// Live 16-byte FMMU register layout:
//     +0..3  Logical Start Address
//     +4..5  Logical Length
//     +6     Logical Start Bit
//     +7     Logical End Bit
//     +8..9  Physical Start Address
//     +10    Physical Start Bit
//     +11    Type
//     +12    Activate
//     +13..15 Reserved
//
// Source fingerprint:
//     OSCARMAX_ECAT_FMMU_DIAG_CONTRACT_12E5B_20260826
// ============================================================================

static constexpr uint32_t SHM_ECAT_FMMU_DIAG_MAGIC =
0x4D464345u; // little-endian bytes "ECFM"

static constexpr uint16_t SHM_ECAT_FMMU_DIAG_VERSION_MAJOR = 1u;
static constexpr uint16_t SHM_ECAT_FMMU_DIAG_VERSION_MINOR = 0u;

static constexpr uint32_t SHM_ECAT_FMMU_DIAG_MAX_SLAVES = 128u;
static constexpr uint32_t SHM_ECAT_FMMU_DIAG_MAX_FMMUS = 16u;

static constexpr uint32_t SHM_ECAT_FMMU_DIAG_ENTRY_COUNT =
SHM_ECAT_FMMU_DIAG_MAX_SLAVES *
SHM_ECAT_FMMU_DIAG_MAX_FMMUS;

// 64-byte global header.
struct SHM_ECAT_FmmuDiagHeader
{
    uint32_t Magic;               // SHM_ECAT_FMMU_DIAG_MAGIC
    uint16_t VersionMajor;        // ABI major
    uint16_t VersionMinor;        // ABI minor
    uint32_t StructSize;          // sizeof(SHM_ECAT_FmmuDiagData)

    uint32_t Sequence;            // seqlock: even = stable
    uint32_t Heartbeat;           // Priority-50 publish heartbeat
    uint64_t PublishCount;        // completed P50 publishes

    uint16_t SlaveCount;          // current Runtime slave count
    uint16_t MaxFmmus;            // always 16 for ABI v1.0

    // Reserved for future global observability.
    uint32_t Reserved32[8];
};

// One fixed 56-byte live FMMU slot.
struct SHM_ECAT_FmmuDiagEntry
{
    uint16_t SlavePosition;       // zero-based topology position
    uint8_t  FmmuIndex;           // physical ESC FMMU number: 0..15
    uint8_t  Present;             // configured Runtime/ENI FMMU exists

    uint8_t  Valid;               // live ESC read is valid
    uint8_t  LogicalStartBit;     // ESC FMMU +6
    uint8_t  LogicalEndBit;       // ESC FMMU +7
    uint8_t  PhysicalStartBit;    // ESC FMMU +10

    uint8_t  Type;                // ESC FMMU +11
    uint8_t  Activate;            // ESC FMMU +12
    uint8_t  Reserved8[2];

    uint16_t RegisterAddress;     // 0x0600 + FmmuIndex*16
    uint16_t PhysicalStartAddress;// ESC FMMU +8..9

    uint32_t LogicalStartAddress; // ESC FMMU +0..3
    uint16_t LogicalLength;       // ESC FMMU +4..5
    uint16_t Reserved16;

    int32_t  LastTransportResult; // >0 WKC, 0 WKC0, <0 bounded transport fault
    uint32_t Reserved32;

    uint64_t LastUpdateTick;      // Priority-64 PDO tick of last probe
    uint64_t ProbeSuccess;        // per-FMMU successful probes
    uint64_t ProbeFailure;        // per-FMMU failed probes
};

// Entire read-only Windows diagnostic mapping.
struct SHM_ECAT_FmmuDiagData
{
    SHM_ECAT_FmmuDiagHeader Header;

    SHM_ECAT_FmmuDiagEntry
        Entries[SHM_ECAT_FMMU_DIAG_ENTRY_COUNT];
};

#if !defined(__INTELLISENSE__)
static_assert(
    sizeof(SHM_ECAT_FmmuDiagHeader) == 64,
    "SHM_ECAT_FmmuDiagHeader ABI changed; update ENI Tool C# contract.");

static_assert(
    sizeof(SHM_ECAT_FmmuDiagEntry) == 56,
    "SHM_ECAT_FmmuDiagEntry ABI changed; update ENI Tool C# contract.");

static_assert(
    sizeof(SHM_ECAT_FmmuDiagData) == 114752,
    "SHM_ECAT_FmmuDiagData ABI changed; update ENI Tool C# contract.");
#endif

#pragma pack(pop)
