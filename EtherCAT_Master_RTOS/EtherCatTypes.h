#pragma once
#include <stdint.h>
#include <cstddef>
#include <vector>
#include <string>

// ============================================================================
//  1. EtherCAT 基礎定義 (命令與標頭)
// ============================================================================
#define EC_CMD_APRD 0x01
#define EC_CMD_APWR 0x02
#define EC_CMD_FPRD 0x04
#define EC_CMD_FPWR 0x05
#define EC_CMD_BRD  0x07
#define EC_CMD_BWR  0x08
#define EC_CMD_LRD  0x0A
#define EC_CMD_LWR  0x0B
#define EC_CMD_LRW  0x0C

#define REG_STATION_ADDR 0x0010
#define REG_AL_CONTROL   0x0120
#define REG_AL_STATUS    0x0130

#pragma pack(push, 1)
typedef struct
{
    uint16_t Length;
    uint16_t Type;
} EtherCatHeader;

typedef struct
{
    uint8_t  Cmd;
    uint8_t  Index;
    uint16_t Address;
    uint16_t Register;
    uint16_t Length;
    uint16_t Irq;
} EtherCatDatagramHeader;
#pragma pack(pop)

// ============================================================================
//  2. CiA402 伺服驅動器資料結構 (對應 RxPDO 和 TxPDO)
//  [新增] 這是為了 Delta A3-E 定義的硬體映射結構
// ============================================================================
#pragma pack(push, 1)

// 對應 RxPDO (1601h): 總長度 9 Bytes
struct ServoOutput
{

    uint16_t ControlWord;      // [0-1] 控制字 (6040h) - 16 bit
    int32_t  TargetVelocity;   // [2-5] 目標速度 (60FFh) - 32 bit
    uint16_t TouchProbeFunc;   // [6-7] 探針功能 (60B8h) - 16 bit
    int8_t   ModesOfOperation; // [8]   操作模式 (6060h) - 8 bit (1 Byte)

};

// 對應 TxPDO (1A01h): 總長度 23 Bytes
struct ServoInput
{

    uint16_t StatusWord;              // [0-1]   狀態字 (6041h) - 16 bit
    int32_t  ActualPosition;          // [2-5]   實際位置 (6064h) - 32 bit
    int32_t  ActualVelocity;          // [6-9]   實際速度 (606Ch) - 32 bit
    int16_t  ActualTorque;            // [10-11] 實際扭力 (6077h) - 16 bit
    uint16_t TouchProbeStatus;        // [12-13] 探針狀態 (60B9h) - 16 bit
    int32_t  TouchProbePos1;          // [14-17] 探針抓取位置 (60BAh) - 32 bit
    int8_t   ModesOfOperationDisplay; // [18]    目前模式顯示 (6061h) - 8 bit (1 Byte)
    uint32_t Object_2510;             // [19-22] 廠商自定義參數 (2510h) - 32 bit


};
#pragma pack(pop)

// ============================================================
// Servo PDO Compile-Time Safety Guard
//
// 目前正式 A3E Runtime Profile：
//
// RxPDO = 72 bits  = 9 Bytes
// TxPDO = 184 bits = 23 Bytes
//
// 若未來有人修改 ServoOutput / ServoInput 欄位、型別或 Packing，
// 造成 PDO Layout 改變，直接在編譯階段停止，而不是等到實機 WKC
// 或 Servo 資料錯位才發現。
// ============================================================

static_assert(
    sizeof(ServoOutput) == 9,
    "ServoOutput PDO layout mismatch. Expected 9 bytes.");

static_assert(
    sizeof(ServoInput) == 23,
    "ServoInput PDO layout mismatch. Expected 23 bytes.");


// ============================================================
// Stage 11D.1 - ServoPDO_9_23 exact field ABI guards
//
// Size-only checks are not enough for a structured Generic
// adapter.  Every field byte offset must remain exact.
// ============================================================

static_assert(
    offsetof(ServoOutput, ControlWord) == 0,
    "ServoOutput.ControlWord offset mismatch.");

static_assert(
    offsetof(ServoOutput, TargetVelocity) == 2,
    "ServoOutput.TargetVelocity offset mismatch.");

static_assert(
    offsetof(ServoOutput, TouchProbeFunc) == 6,
    "ServoOutput.TouchProbeFunc offset mismatch.");

static_assert(
    offsetof(ServoOutput, ModesOfOperation) == 8,
    "ServoOutput.ModesOfOperation offset mismatch.");


static_assert(
    offsetof(ServoInput, StatusWord) == 0,
    "ServoInput.StatusWord offset mismatch.");

static_assert(
    offsetof(ServoInput, ActualPosition) == 2,
    "ServoInput.ActualPosition offset mismatch.");

static_assert(
    offsetof(ServoInput, ActualVelocity) == 6,
    "ServoInput.ActualVelocity offset mismatch.");

static_assert(
    offsetof(ServoInput, ActualTorque) == 10,
    "ServoInput.ActualTorque offset mismatch.");

static_assert(
    offsetof(ServoInput, TouchProbeStatus) == 12,
    "ServoInput.TouchProbeStatus offset mismatch.");

static_assert(
    offsetof(ServoInput, TouchProbePos1) == 14,
    "ServoInput.TouchProbePos1 offset mismatch.");

static_assert(
    offsetof(ServoInput, ModesOfOperationDisplay) == 18,
    "ServoInput.ModesOfOperationDisplay offset mismatch.");

static_assert(
    offsetof(ServoInput, Object_2510) == 19,
    "ServoInput.Object_2510 offset mismatch.");



// ============================================================================
//  3. 模組物件定義 (伺服、IO、類比)
// ============================================================================

// 伺服物件
struct ENI_ServoDrive
{
    int slaveIndex;
    uint32_t vendorId;
    uint32_t productCode;

    ServoOutput* pOutput = nullptr;
    ServoInput* pInput = nullptr;
};

// [保留] 數位模組 (Generic IO)
struct ENI_GenericIO
{
    int slaveIndex;
    uint32_t vendorId;
    uint32_t productCode;
    uint16_t inputAddr;
    std::vector<uint8_t> inBuffer;
    uint16_t outputAddr;
    std::vector<uint8_t> outBuffer;

    // PDO 資料指標
    void* pOutputLoc = nullptr; // 指向 Output (燈號)
    void* pInputLoc = nullptr;  // 指向 Input (按鈕)
};

// [保留] 類比模組 (Analog / Special)
struct ENI_AnalogModule
{
    int slaveIndex;
    uint32_t vendorId;
    uint32_t productCode;
    int channelCount;
    uint16_t configAddr;

    // 解析後的數值
    std::vector<int16_t> channelValues;

    // PDO 資料指標
    void* pInputLoc = nullptr;
};

// ============================================================================
//  4. ENI XML 解析結構
// ============================================================================

// ============================================================================
// Stage 5A - Runtime XML V2 Metadata
//
// Configurator 現在已經輸出的 Profile / PDO Summary / DC / Watchdog
// 在本階段正式保存到 EtherCatSlave。
// Stage 5A 只 Parse / Store / Diagnostic，不取代既有硬體設定流程。
// ============================================================================

struct EtherCatRuntimeProfileConfig
{
    bool present = false;

    char name[128] = {};
    char pdoMappingMode[32] = {};

    uint16_t rxPdoAssignIndex = 0;
    uint16_t txPdoAssignIndex = 0;

    uint8_t outputSmControl = 0;
    uint8_t inputSmControl = 0;

    bool hasPdoMappingMode = false;
    bool hasRxPdoAssignIndex = false;
    bool hasTxPdoAssignIndex = false;
    bool hasOutputSmControl = false;
    bool hasInputSmControl = false;

    std::vector<uint16_t> rxPdoIndexes;
    std::vector<uint16_t> txPdoIndexes;
};


struct EtherCatRuntimeDcConfig
{
    bool present = false;

    char mode[32] = {};

    uint32_t cycleTimeNs = 0;
    int64_t shiftTimeNs = 0;
    bool referenceClock = false;

    bool hasCycleTimeNs = false;
    bool hasShiftTimeNs = false;
    bool hasReferenceClock = false;
};


struct EtherCatRuntimeWatchdogConfig
{
    bool present = false;

    uint32_t processDataTimeoutMs = 0;

    bool hasProcessDataTimeoutMs = false;
};


// ============================================================================
// Stage 5B - Executable Runtime Schema
//
// Complete SyncManager / PDO / InitCommand data is stored here.
// Stage 5B still does NOT apply this data to EtherCAT hardware.
// ============================================================================

struct EtherCatRuntimeSyncManagerConfig
{
    int index = -1;

    char name[64] = {};

    uint16_t startAddress = 0;
    uint16_t length = 0;

    uint16_t minimumSize = 0;
    uint16_t maximumSize = 0;

    uint8_t controlByte = 0;

    bool enabled = false;
    bool opOnly = false;
};


struct EtherCatRuntimePdoEntryConfig
{
    uint16_t index = 0;
    uint8_t subIndex = 0;
    uint8_t bitLength = 0;
    uint32_t mappingValue = 0;

    char name[96] = {};
    char dataType[32] = {};
};


struct EtherCatRuntimePdoConfig
{
    uint16_t index = 0;

    char name[96] = {};

    int syncManagerIndex = -1;

    uint16_t bitSize = 0;
    uint16_t byteSize = 0;

    std::vector<EtherCatRuntimePdoEntryConfig> entries;
};


struct EtherCatRuntimeInitCommandConfig
{
    char source[32] = {};
    bool apply = false;
    bool hasApply = false;

    char transition[16] = {};

    uint16_t index = 0;
    uint8_t subIndex = 0;

    std::vector<uint8_t> data;

    char comment[128] = {};
};


// ============================================================================
// Stage 6A - Runtime Transport Schema
//
// Parse / audit only.
// No EtherCAT hardware write is performed by these data structures.
// ============================================================================

struct EtherCatRuntimeMailboxDirectionConfig
{
    bool present = false;

    int smIndex = -1;

    uint16_t startAddress = 0;
    uint16_t length = 0;

    uint8_t controlByte = 0;
    bool enabled = false;
};


struct EtherCatRuntimeMailboxConfig
{
    bool present = false;

    EtherCatRuntimeMailboxDirectionConfig out;
    EtherCatRuntimeMailboxDirectionConfig in;
};


struct EtherCatRuntimeFmmuConfig
{
    int index = -1;

    char direction[16] = {};

    uint32_t logicalStartAddress = 0;
    uint16_t logicalLength = 0;

    uint8_t logicalStartBit = 0;
    uint8_t logicalEndBit = 0;

    uint16_t physicalStartAddress = 0;
    uint8_t physicalStartBit = 0;

    uint8_t type = 0;
    bool enabled = false;
};


// ============================================================================
// Stage 7A - Runtime Process Image Binding Schema
//
// Describes the application-side binding currently produced by BuildIoMap().
//
// Stage 7A is parse / shadow-audit only.
// ============================================================================

struct EtherCatRuntimeProcessImageBindingConfig
{
    bool present = false;

    char kind[32] = {};
    char abi[32] = {};

    int32_t outputOffset = -1;
    uint32_t outputBytes = 0;

    int32_t inputOffset = -1;
    uint32_t inputBytes = 0;

    uint16_t elementBytes = 0;
    uint16_t channelCount = 0;
};


// ============================================================================
// Stage 11A - Composite Multi-Binding Runtime Schema
//
// ProcessImageBinding remains the current compatibility envelope.
//
// ApplicationBindings adds a future-capable 0..N list of application
// functions inside one EtherCAT Slave.
//
// Bit-level offsets are used so a future custom EDM card may contain:
// - DigitalInput / DigitalOutput
// - AnalogInput / AnalogOutput
// - PositionFeedback
// - Status / Command
//
// Stage 11A is parse + shadow audit only.
// ============================================================================

// ============================================================================
// Stage 11C.2 - Generic Composite Application Descriptor
//
// Shadow application view over m_IoMap.
// Supports byte-aligned and bit-oriented fields.
// ============================================================================

struct EtherCatCompositeApplicationDescriptor
{
    int32_t slaveIndex = -1;
    int32_t bindingIndex = -1;
    int32_t sortOrder = 0;

    char id[64] = {};
    char name[64] = {};

    char kind[32] = {};
    char legacyKind[32] = {};
    char abi[32] = {};
    char interfaceType[32] = {};
    char dataType[32] = {};
    char sampleMode[32] = {};
    char unit[32] = {};
    char axisRef[16] = {};

    int32_t outputBitOffset = -1;
    uint32_t outputBitLength = 0;

    int32_t inputBitOffset = -1;
    uint32_t inputBitLength = 0;

    uint16_t elementBits = 0;
    uint16_t channelCount = 0;

    int32_t outputByteOffset = -1;
    uint8_t outputBitShift = 0;
    uint32_t outputByteSpan = 0;
    bool outputByteAligned = true;
    uint8_t* pOutputByteBase = nullptr;

    int32_t inputByteOffset = -1;
    uint8_t inputBitShift = 0;
    uint32_t inputByteSpan = 0;
    bool inputByteAligned = true;
    uint8_t* pInputByteBase = nullptr;
};


// ============================================================================
// Stage 11D.1 - Structured ServoDrive Field Descriptor
//
// Child field view under one parent ServoDrive Composite descriptor.
//
// Example parent:
//
//     ServoDrive / Primary / ServoPDO_9_23
//
// Child fields:
//
//     ControlWord
//     TargetVelocity
//     TouchProbeFunction
//     ModesOfOperation
//     StatusWord
//     ActualPosition
//     ActualVelocity
//     ActualTorque
//     TouchProbeStatus
//     TouchProbePosition
//     ModesOfOperationDisplay
//     Object2510
//
// Shadow only in Stage11D.1.
// ============================================================================

struct EtherCatStructuredServoFieldDescriptor
{
    int32_t slaveIndex = -1;
    int32_t parentBindingIndex = -1;

    char parentBindingId[64] = {};
    char parentAbi[32] = {};

    char fieldId[64] = {};
    char semanticKind[32] = {};
    char direction[16] = {};
    char interfaceType[32] = {};
    char dataType[32] = {};
    char unit[32] = {};
    char axisRef[16] = {};

    uint32_t relativeBitOffset = 0;
    int32_t absoluteBitOffset = -1;
    uint32_t bitLength = 0;

    int32_t byteOffset = -1;
    uint32_t byteSpan = 0;
    bool byteAligned = true;

    uint8_t* pByteBase = nullptr;
};



struct EtherCatRuntimeApplicationBindingConfig
{
    char id[64] = {};
    char name[64] = {};

    int32_t sortOrder = 0;

    char kind[32] = {};
    char legacyKind[32] = {};
    char abi[32] = {};
    char interfaceType[32] = {};
    char dataType[32] = {};
    char sampleMode[32] = {};
    char unit[32] = {};
    char axisRef[16] = {};

    // Stage11C.1 persisted-project relative position.
    int32_t outputRelativeBitOffset = -1;

    // Absolute Process Image position.
    int32_t outputBitOffset = -1;
    uint32_t outputBitLength = 0;

    // Stage11C.1 persisted-project relative position.
    int32_t inputRelativeBitOffset = -1;

    // Absolute Process Image position.
    int32_t inputBitOffset = -1;
    uint32_t inputBitLength = 0;

    uint16_t elementBits = 0;
    uint16_t channelCount = 0;
};


// [保留] 結構名稱必須是 EtherCatSlave
struct EtherCatSlave
{
    char name[128];
    char type[128];

    uint32_t vendorId;
    uint32_t productCode;

    // Runtime Config Identity
    //
    // Revision / ConfiguredAddress 由 EtherCAT_Runtime.xml 載入，
    // 用於啟動階段 Topology / Identity Verification。
    //
    // hasRevision / hasConfiguredAddress 用來保持舊 Runtime XML 相容：
    // 舊檔案若沒有這些欄位，就只跳過該項驗證。
    uint32_t revision = 0;
    uint16_t configuredAddress = 0;
    bool hasRevision = false;
    bool hasConfiguredAddress = false;

    uint16_t configAddrIn;
    uint16_t configAddrOut;
    uint32_t bitSize;

    // Input 資訊
    uint16_t config_addr_in;   // 輸入位址 (PhysAddr)
    uint32_t inputBitLength;   // 輸入長度 (BitSize)

    // Output 資訊
    uint16_t config_addr_out;  // 輸出位址 (PhysAddr)
    uint32_t outputBitLength;  // 輸出長度 (BitSize)

    // Stage 5A Runtime Metadata
    EtherCatRuntimeProfileConfig runtimeProfile;
    EtherCatRuntimeDcConfig runtimeDc;
    EtherCatRuntimeWatchdogConfig runtimeWatchdog;

    // Stage 5B complete executable schema.
    std::vector<EtherCatRuntimeSyncManagerConfig> runtimeSyncManagers;
    std::vector<EtherCatRuntimePdoConfig> runtimeRxPdos;
    std::vector<EtherCatRuntimePdoConfig> runtimeTxPdos;
    std::vector<EtherCatRuntimeInitCommandConfig> runtimeInitCommands;

    // Stage 6A / 6C Runtime Transport Schema.
    EtherCatRuntimeMailboxConfig runtimeMailbox;

    // true:
    //     Runtime XML contains an explicit <Fmmus> section.
    //
    // false:
    //     legacy Runtime XML; Stage 6C may use old C++ fallback.
    bool runtimeFmmuSchemaPresent = false;

    std::vector<EtherCatRuntimeFmmuConfig> runtimeFmmus;


    // Stage 7A application-side Process Image binding.
    EtherCatRuntimeProcessImageBindingConfig runtimeProcessImageBinding;


    // Stage 11A composite application binding schema.
    bool runtimeApplicationBindingsSchemaPresent =
        false;

    char runtimeApplicationBindingsMode[32] =
    {
        0
    };

    std::vector<EtherCatRuntimeApplicationBindingConfig>
        runtimeApplicationBindings;


    // ================================================================
    // Stage 11C.1 persisted Composite Runtime Shadow Schema
    //
    // Separate from active Stage11A <ApplicationBindings>.
    // Parse/audit only; no Runtime cutover.
    // ================================================================

    bool runtimeCompositeBindingsShadowSchemaPresent =
        false;

    char runtimeCompositeBindingsShadowMode[32] =
    {
        0
    };

    int32_t runtimeCompositeBindingsShadowOutputBaseBit =
        -1;

    int32_t runtimeCompositeBindingsShadowInputBaseBit =
        -1;

    std::vector<EtherCatRuntimeApplicationBindingConfig>
        runtimeCompositeBindingsShadow;


    struct InitCmd
    {
        uint16_t index;
        std::vector<uint8_t> data;
    };
    std::vector<InitCmd> initCmds;
};