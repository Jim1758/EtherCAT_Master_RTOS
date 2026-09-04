#pragma once

#include <stdint.h>

#pragma pack(push, 1)

// ============================================================================
// DC-RX.4B - OSCARMAX EtherCAT RX Forensics Shared Memory Contract
// ============================================================================
//
// Shared Memory:
//     OSCARMAX_ECAT_RX_FORENSICS
//
// Producer:
//     Existing Priority-50 supervisory loop. The publisher only copies the
//     already-existing DC-RX.4A owner-safe incident ring and ESC per-port
//     snapshots. It performs no EtherCAT transaction and changes no runtime
//     control state.
//
// Consumer:
//     EtherCAT_ENI_Tool (read-only).
//
// Snapshot protocol:
//     Header.Sequence odd  = writer active
//     Header.Sequence even = stable snapshot
//
// Layout goals:
//     Header       64 bytes
//     Summary     192 bytes
//     Incident    256 bytes x 128
//     ESC Port    256 bytes x 128
//     Total     65792 bytes
//
// Source fingerprint:
//     OSCARMAX_ECAT_RX_FORENSICS_SHM_4B_20260904
// ============================================================================

static constexpr uint32_t SHM_ECAT_RX_FORENSICS_MAGIC = 0x46585245u; // "ERXF"
static constexpr uint16_t SHM_ECAT_RX_FORENSICS_VERSION_MAJOR = 1u;
static constexpr uint16_t SHM_ECAT_RX_FORENSICS_VERSION_MINOR = 0u;
static constexpr uint32_t SHM_ECAT_RX_FORENSICS_INCIDENT_CAPACITY = 128u;
static constexpr uint32_t SHM_ECAT_RX_FORENSICS_SLAVE_CAPACITY = 128u;
static constexpr uint32_t SHM_ECAT_RX_FORENSICS_PORT_COUNT = 4u;

// Header.Flags
static constexpr uint32_t SHM_ECAT_RX_FORENSICS_FLAG_PUBLISHER_ACTIVE = 0x00000001u;
static constexpr uint32_t SHM_ECAT_RX_FORENSICS_FLAG_INCIDENTS_COHERENT = 0x00000002u;
static constexpr uint32_t SHM_ECAT_RX_FORENSICS_FLAG_PORTS_COHERENT = 0x00000004u;
static constexpr uint32_t SHM_ECAT_RX_FORENSICS_FLAG_SUMMARY_COHERENT = 0x00000008u;

// Summary.Flags
static constexpr uint32_t SHM_ECAT_RX_FORENSICS_SUMMARY_PROCESS_DATA_VALID = 0x00000001u;
static constexpr uint32_t SHM_ECAT_RX_FORENSICS_SUMMARY_DC_TRANSPORT_VALID = 0x00000002u;
static constexpr uint32_t SHM_ECAT_RX_FORENSICS_SUMMARY_RX_HEALTHY = 0x00000004u;
static constexpr uint32_t SHM_ECAT_RX_FORENSICS_SUMMARY_DC_HEALTHY = 0x00000008u;

struct SHM_ECAT_RxForensicsHeader
{
    uint32_t Magic;
    uint16_t VersionMajor;
    uint16_t VersionMinor;
    uint32_t StructSize;

    uint32_t Sequence;
    uint32_t Heartbeat;
    uint64_t PublishCount;

    uint32_t IncidentCapacity;
    uint32_t IncidentCount;
    uint32_t SlaveCapacity;
    uint32_t SlaveCount;

    uint64_t LatestIncidentId;
    uint64_t FirstIncidentId;

    uint32_t Flags;
};

struct SHM_ECAT_RxForensicsSummary
{
    uint64_t CaptureQpc;
    uint64_t QpcFrequency;
    uint64_t TotalHardTimeout;
    uint64_t TotalSoftLateAccepted;
    uint64_t SchedulerRecoveryTotal;
    uint64_t SchedulerSkippedCyclesTotal;
    uint64_t EscPortChangeSerial;

    int64_t PhaseErrorNs;
    int64_t AppliedPpb;
    int64_t RecommendedPpb;

    uint32_t RxRecentTimeout;
    uint32_t RxConsecutiveTimeout;
    uint32_t RxRecoveryWindow;
    uint32_t RxSkipWindow;
    uint32_t RxSoftLateWindow;
    uint32_t QpcFailWindow;

    uint32_t NalFrame;
    uint32_t NalNoData;
    uint32_t NalApiError;
    uint32_t NalInvalidLength;
    uint32_t NalUnavailable;
    uint32_t NalLastError;

    uint32_t CallOver50us;
    uint32_t CallOver100us;
    uint32_t CallOver150us;
    uint32_t CallOver200us;

    uint32_t TimeoutNoFrame;
    uint32_t TimeoutNalStall;
    uint32_t TimeoutLateFrame;

    uint32_t ResyncEvents;
    uint32_t ResyncFrames;
    uint32_t ResyncLimitHit;

    int32_t LrwWkc;
    int32_t DcWkc;
    int32_t ExpectedLrwWkc;

    uint32_t HoldMask;
    uint32_t TripMask;
    uint32_t Flags;
};

struct SHM_ECAT_RxForensicsIncident
{
    uint64_t IncidentId;
    uint64_t RelatedIncidentId;
    uint64_t BurstId;
    uint64_t PdoTick;
    uint64_t Qpc;
    uint64_t QpcFrequency;
    uint64_t TotalHardTimeout;
    uint64_t RxElapsedNs;
    uint64_t ReceiveCallNs;
    uint64_t ReceiveCallMaxNs;
    uint64_t SchedulerRecoveryTotal;
    uint64_t EscPortChangeSerial;
    uint64_t NalCallSequence;

    int64_t PhaseErrorNs;
    int64_t AppliedPpb;
    int64_t RecommendedPpb;

    uint32_t Kind;
    uint32_t Reason;
    uint32_t Classification;
    uint32_t ConsecutiveTimeout;
    uint32_t AttemptCount;
    uint32_t CoarseWaitCount;
    uint32_t EventSignaledCount;
    uint32_t EventTimeoutCount;
    uint32_t EventFailedCount;
    uint32_t EventFallbackSleepCount;
    uint32_t LastEventResult;
    uint32_t LastEventError;
    uint32_t NalOutcome;
    uint32_t NalCallSucceeded;
    uint32_t NalError;
    uint32_t NalLength;

    int32_t RxLength;
    int32_t LrwWkc;
    int32_t DcWkc;
    int32_t ExpectedLrwWkc;

    uint32_t ResyncAttempted;
    uint32_t ResyncDrainCount;
    uint32_t ResyncDrainLimitHit;
    uint32_t HoldMask;
    uint32_t TripMask;
    uint32_t ProcessDataValid;
    uint32_t DcTransportValid;

    uint8_t Reserved[20];
};

struct SHM_ECAT_RxForensicsEscPort
{
    uint32_t Valid;
    uint32_t SlavePosition;
    uint32_t ConfiguredAddress;
    uint32_t BaselineValid;

    uint64_t SampleTick;
    uint64_t InitialBaselineTick;
    uint64_t RxBaselineTick;
    uint64_t EcatProcessingUnitBaselineTick;
    uint64_t PdiBaselineTick;
    uint64_t LostLinkBaselineTick;
    uint64_t LastChangeTick;
    uint64_t SampleCount;
    uint64_t RxGroupResetCount;
    uint64_t EcatProcessingUnitResetCount;
    uint64_t PdiResetCount;
    uint64_t LostLinkGroupResetCount;
    uint64_t ChangeSerial;

    uint8_t InvalidFrame[SHM_ECAT_RX_FORENSICS_PORT_COUNT];
    uint8_t PhysicalRxError[SHM_ECAT_RX_FORENSICS_PORT_COUNT];
    uint8_t ForwardedRxError[SHM_ECAT_RX_FORENSICS_PORT_COUNT];
    uint8_t LostLink[SHM_ECAT_RX_FORENSICS_PORT_COUNT];
    uint8_t EcatProcessingUnitError;
    uint8_t PdiError;
    uint8_t Reserved0[2];

    uint8_t BaselineInvalidFrame[SHM_ECAT_RX_FORENSICS_PORT_COUNT];
    uint8_t BaselinePhysicalRxError[SHM_ECAT_RX_FORENSICS_PORT_COUNT];
    uint8_t BaselineForwardedRxError[SHM_ECAT_RX_FORENSICS_PORT_COUNT];
    uint8_t BaselineLostLink[SHM_ECAT_RX_FORENSICS_PORT_COUNT];
    uint8_t BaselineEcatProcessingUnitError;
    uint8_t BaselinePdiError;
    uint8_t Reserved1[2];

    uint32_t DeltaInvalidFrame[SHM_ECAT_RX_FORENSICS_PORT_COUNT];
    uint32_t DeltaPhysicalRxError[SHM_ECAT_RX_FORENSICS_PORT_COUNT];
    uint32_t DeltaForwardedRxError[SHM_ECAT_RX_FORENSICS_PORT_COUNT];
    uint32_t DeltaLostLink[SHM_ECAT_RX_FORENSICS_PORT_COUNT];
    uint32_t DeltaEcatProcessingUnitError;
    uint32_t DeltaPdiError;

    uint32_t SaturatedInvalidFrameMask;
    uint32_t SaturatedPhysicalRxErrorMask;
    uint32_t SaturatedForwardedRxErrorMask;
    uint32_t SaturatedLostLinkMask;
    uint32_t SaturatedMiscMask;

    uint8_t Reserved2[4];
};

struct SHM_ECAT_RxForensicsData
{
    SHM_ECAT_RxForensicsHeader Header;
    SHM_ECAT_RxForensicsSummary Summary;
    SHM_ECAT_RxForensicsIncident Incidents[SHM_ECAT_RX_FORENSICS_INCIDENT_CAPACITY];
    SHM_ECAT_RxForensicsEscPort Slaves[SHM_ECAT_RX_FORENSICS_SLAVE_CAPACITY];
};

#if !defined(__INTELLISENSE__)
static_assert(sizeof(SHM_ECAT_RxForensicsHeader) == 64,
    "SHM_ECAT_RxForensicsHeader ABI mismatch.");
static_assert(sizeof(SHM_ECAT_RxForensicsSummary) == 192,
    "SHM_ECAT_RxForensicsSummary ABI mismatch.");
static_assert(sizeof(SHM_ECAT_RxForensicsIncident) == 256,
    "SHM_ECAT_RxForensicsIncident ABI mismatch.");
static_assert(sizeof(SHM_ECAT_RxForensicsEscPort) == 256,
    "SHM_ECAT_RxForensicsEscPort ABI mismatch.");
static_assert(sizeof(SHM_ECAT_RxForensicsData) == 65792,
    "SHM_ECAT_RxForensicsData ABI mismatch.");
#endif

#pragma pack(pop)
