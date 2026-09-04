#pragma once
#include "EtherCatTypes.h"
#include "EtherCatEni.h"
#include "NicDriver.h"
#include <vector>
#include "PlcCore.h"       // 引用 PLC 核心
#include "MotionCore.h"       // 引用 MotionCore 核心
#include "NCManager.h"
#include "PLCManager.h" // 🌟 1. 記得 include PLCManager 標頭檔
#include <atomic>
#pragma pack(push, 1)

// 標準 Mailbox 標頭 (6 Bytes)
struct MailboxHeader {
    uint16_t length;      // 資料長度 (Data Length)
    uint16_t address;     // 來源/目的位址 (Station Address)
    uint8_t  priority : 2;// 優先級 (0: Lowest)
    uint8_t  type : 4;    // 協議類型 (3 = CoE - CANopen over EtherCAT)
    uint8_t  cnt : 3;     // 計數器 (Counter, 1-7)
    uint8_t  reserved : 1;// 保留位
};

// CoE 標頭 (2 Bytes)
struct CoEHeader {
    uint16_t number : 9;  // PDO Number or Reserved
    uint16_t reserved : 3;// 保留
    uint16_t service : 4; // 服務類型 (2 = SDO Request, 3 = SDO Response, 8 = SDO Info)
};

// SDO 標頭 (4 Bytes) - 用於參數讀寫
struct SDOHeader {
    uint8_t  size_ind : 2; // 資料大小指示 (0: Set in size, 1: No size specified)
    uint8_t  exp_od : 1;   // 加速傳輸 (1 = Expedited Transfer, 資料在 Data 欄位中)
    uint8_t  size : 2;     // 資料大小 (僅在 Expedited 模式下有效, 表示未使用的 Bytes 數)
    uint8_t  command : 3;  // 指令 (1 = Download Request/Write, 2 = Upload Request/Read)
    uint16_t index;        // 物件字典索引 (Object Index, e.g., 0x6060)
    uint8_t  subIndex;     // 子索引 (Sub-Index)
    uint32_t data;         // 資料本體 (4 bytes, 用於 Expedited 傳輸)
};

struct EtherCatFrameHeader {
    uint16_t Length : 11;
    uint16_t Reserved : 1;
    uint16_t Type : 4;
};

struct EtherCatPduHeader {
    uint8_t  Command;
    uint8_t  Index;
    uint16_t Address;
    uint16_t Register;
    uint16_t Length : 11;
    uint16_t Reserved : 3;
    uint16_t Next : 1;
    uint16_t Irq : 1;
};


struct SlaveInfo {
    uint16_t configAddr;    // 我們分配給它的地址 (0x1001...)
    uint16_t APRDAPWR_Addr;    // APRDAPWR 使用的adp
    uint16_t mbxOutAddr;    // SM0 Addr
    uint16_t mbxOutLength;  // SM0 Len
    uint16_t mbxInAddr;     // SM1 Addr
    uint16_t mbxInLength;   // SM1 Len
    uint32_t Vendor_ID;
    uint32_t Product_Code;
    uint32_t Revision_No;   // SII Identity Revision (Word Address 0x000C)

};


// 定義指令類型
enum class EcatCmdType : int
{
    CMD_NONE = 0,
    CMD_SET_STATE,      // 切換狀態 (例如 OP, INIT)
    CMD_SDO_WRITE,      // SDO 寫入
    CMD_SDO_READ        // SDO 讀取
};

// 定義指令狀態
enum class EcatCmdStatus : int
{
    // --- 閒置狀態 ---
    // 代表目前指令槽位是空的，UI 執行緒可以安全地填寫新的指令資料。
    ECAT_STATUS_IDLE = 0,

    // --- 待處理/執行中 ---
    // UI 填寫完資料後將狀態改為 PENDING。
    // RT 執行緒在每個週期的空檔 (如 Tick 2) 會檢查此旗標，若看到 PENDING 則開始執行硬體通訊。
    ECAT_STATUS_PENDING = 1,

    // --- 執行完成 ---
    // RT 執行緒完成通訊並回填 WKC/結果後，會將狀態設為 DONE。
    // UI 執行緒看到 DONE 後會讀取結果，並將狀態重設回 IDLE。
    ECAT_STATUS_DONE = 2,

    // --- 錯誤狀態 ---
    // 代表指令在執行過程中發生異常 (例如：通訊超時、從站拒絕要求、或是 WKC 為 0)。
    ECAT_STATUS_ERROR = -1
};

struct AsyncCommandSlot
{
    // --- 控制旗標 ---
    volatile int status = (int)EcatCmdStatus::ECAT_STATUS_IDLE; // [關鍵] volatile 防止編譯器優化順序
    int type;            // 指令類型 (對應 EcatCmdType)

    // --- SDO / 命令參數 ---
    uint16_t slaveAddr;  // 目標站號 (0:廣播, 1~N:指定站)
    uint16_t index;      // SDO Index (例如 0x6040, 0x6060)
    uint8_t subIndex;    // SDO SubIndex (例如 0x00)

    uint32_t dataValue;  // 數值本體 (最大支援 4 bytes)
    int dataSize;        // [新增] 資料長度 (1, 2, 或 4 bytes) <--- 建議加上這個

    // --- 輸出結果 ---
    int resultWKC;       // RT 回傳的 WKC (用來判斷成功/失敗)
};



// 全域或類別成員變數，用來存所有從站資訊
SlaveInfo m_slaveInfo[128];
int Motor_Start_Index = -1;
#pragma pack(pop)

constexpr int DC_AUTO_MAX_SERVO_COUNT =
8;

struct DCAutoPropagationEntry
{
    int slaveIndex = -1;
    uint32_t delayNs = 0;
};

struct DCAutoPropagationTable
{
    int count = 0;
    int referenceSlaveIndex = -1;
    DCAutoPropagationEntry entries[DC_AUTO_MAX_SERVO_COUNT];
};

// ============================================================================
// DC-RX.3D - exact software TX/RX timing handoff
//
// ecx_LRW_FRMW() fills this POD only after a fully correlated LRW+FRMW frame
// has passed the hard RX deadline and datagram identity checks.  The caller may
// then use the narrower SendPacket()/matching ReceivePacket() software window
// instead of the wider whole-function call window when pairing QPC with the
// returned DC reference time.
//
// source:
//   0 = none / unavailable
//   1 = exact software TX-call midpoint -> matching RX-return timing
// ============================================================================
enum class EtherCatDcCycleTimingSource : uint32_t
{
    None = 0,
    ExactSoftwareTxRx = 1
};

struct EtherCatDcCycleTiming
{
    uint64_t txBeforeSendQpc = 0;
    uint64_t txAfterSendQpc = 0;
    uint64_t rxAfterMatchQpc = 0;
    uint64_t txMidpointQpc = 0;
    uint64_t sampleMidpointQpc = 0;
    uint64_t softwareRoundTripCounts = 0;
    uint32_t valid = 0;
    uint32_t source = 0;
};

static_assert(
    sizeof(EtherCatDcCycleTiming) == 56,
    "EtherCatDcCycleTiming ABI changed unexpectedly.");

// ============================================================================
// EtherCAT 主站類別 (EtherCatMaster)
// 負責底層封包收發、狀態機管理與從站配置
// ============================================================================

// 🌟 加上這兩行「前置宣告 (Forward Declaration)」
// 這樣所有 #include "EtherCatMaster.h" 的 .cpp 檔案，都會知道有這兩個函數存在！
void RTAPI GlobalTimerHandler_PDO(void* nContext);
void RTAPI GlobalTimerHandler_PLC(void* nContext);
class CoordinateManager;


// ============================================================================
// EtherCAT Master Startup State
//
// 由 EtherCatMaster 本身擁有。
// EtherCAT_Master_RTOS.cpp 不再保存 Startup global state。
// ============================================================================

enum class EtherCatStartupStage : LONG
{
    Idle = 0,

    OpenNic = 10,
    LoadRuntimeConfig = 20,
    AttachMaster = 30,
    ScanSlaves = 40,
    BuildProcessImage = 50,
    InitializeSlaves = 60,

    Ready = 100,

    Fault = 900
};


enum class EtherCatStartupResult : LONG
{
    Idle = 0,
    Running = 1,
    Pass = 2,
    Fail = -1
};


enum EtherCatStartupErrorCode : LONG
{
    EcatStartupOk = 0,

    EcatStartupErrorNicOpen = -101,
    EcatStartupErrorRuntimeConfig = -102,
    EcatStartupErrorScanSlaves = -103,
    EcatStartupErrorBuildProcessImage = -104,
    EcatStartupErrorInitializeSlaves = -105
};


struct EtherCatStartupSnapshot
{
    EtherCatStartupStage stage =
        EtherCatStartupStage::Idle;

    EtherCatStartupResult result =
        EtherCatStartupResult::Idle;

    EtherCatStartupStage failedStage =
        EtherCatStartupStage::Idle;

    LONG errorCode =
        EcatStartupOk;
};


// ============================================================================
// EtherCAT Master Runtime Lifecycle State
//
// Startup READY 之後進入 Runtime Lifecycle：
//
// PREPARE
// START_PDO_RUNTIME
// START_PLC_RUNTIME
// ENTER_OP
// LINK_MOTION
// INIT_SHARED_MEMORY
// RUNNING
// STOPPING
// STOPPED
// FAULT
// ============================================================================

enum class EtherCatRuntimeStage : LONG
{
    Idle = 0,

    Prepare = 10,
    StartPdoRuntime = 20,
    StartPlcRuntime = 30,
    EnterOp = 40,
    LinkMotion = 50,
    InitSharedMemory = 60,

    Running = 100,

    Stopping = 200,
    Stopped = 210,

    Fault = 900
};


enum class EtherCatRuntimeResult : LONG
{
    Idle = 0,
    Running = 1,
    Pass = 2,
    Fail = -1
};


enum EtherCatRuntimeErrorCode : LONG
{
    EcatRuntimeOk = 0,

    EcatRuntimeErrorPdoStart = -201,
    EcatRuntimeErrorPlcStart = -202,
    EcatRuntimeErrorEnterOp = -203,
    EcatRuntimeErrorServoAxisCount = -204,
    EcatRuntimeErrorSharedMemory = -205
};


struct EtherCatRuntimeSnapshot
{
    EtherCatRuntimeStage stage =
        EtherCatRuntimeStage::Idle;

    EtherCatRuntimeResult result =
        EtherCatRuntimeResult::Idle;

    EtherCatRuntimeStage failedStage =
        EtherCatRuntimeStage::Idle;

    LONG errorCode =
        EcatRuntimeOk;
};


class EtherCatMaster
{
public:
    EtherCatMaster();
    ~EtherCatMaster();


    // 🌟 2. 新增綁定 API
    void LinkCoordinateManager(CoordinateManager* pCoord);

    // 🌟 3. 新增指標變數 (預設設為 nullptr 防呆)
    CoordinateManager* pCoordMgr = nullptr;
    // 🌟 2. 新增 PLCManager 成員變數（名稱必須是 m_plcManager）
    PLCManager m_plcManager;

    //EtherCAT  Function
    void AttachNic(CNicDriver* pNic);// 綁定網卡驅動程式
    void AttachEni(EtherCatEni* pEni); // 綁定 ENI 設定檔解析器

    // =========================================================
    // Master Startup
    //
    // Startup 流程正式歸屬 EtherCatMaster。
    //
    // EtherCAT_Master_RTOS.cpp 只需要：
    //
    // Master.InitializeMaster(
    //     MyNic,
    //     MyEni,
    //     runtimeConfigPath);
    // =========================================================
    bool InitializeMaster(
        CNicDriver& nic,
        EtherCatEni& eni,
        const char* runtimeConfigPath);

    EtherCatStartupSnapshot GetStartupSnapshot() const;

    const char* GetStartupStageName(
        EtherCatStartupStage stage) const;


    // =========================================================
    // Runtime Lifecycle State
    // =========================================================

    EtherCatRuntimeSnapshot GetRuntimeSnapshot() const;

    const char* GetRuntimeStageName(
        EtherCatRuntimeStage stage) const;

    int ecx_BRD(uint16_t ADP, uint16_t ADO, uint16_t length, int timeout);//廣播讀取
    int ecx_BWR(uint16_t ADP, uint16_t ADO, uint16_t length, const void* data, int timeout);//廣播寫入
    int ecx_APRD(uint16_t ADP, uint16_t ADO, uint16_t length, void* data, int timeout);//自動增量物理讀取
    int ecx_APWR(uint16_t ADP, uint16_t ADO, uint16_t length, const void* data, int timeout);//自動增量物理寫入
    int ecx_LRW(uint32_t LogAddr, uint16_t length, void* data, int timeout);//邏輯讀寫
    int ecx_LRW_FRMW(
        uint32_t LogAddr,
        uint16_t length,
        void* data,
        uint16_t dcSlaveAddr,
        uint64_t* dcReferenceTime,
        int* dcWkc,
        int timeout,
        EtherCatDcCycleTiming* dcCycleTiming = nullptr);
    int ecx_FPWR(uint16_t slaveAddr, uint16_t regAddr, void* data, int len, int timeout);//指定位址物理寫入
    int ecx_FPRD(uint16_t slaveAddr, uint16_t regAddr, void* data, int len, int timeout);//指定位址物理讀取
    int ecx_SDOwrite(int slave_pos, uint16_t index, uint8_t subindex, int CA, int size, void* data, int timeout);//服務資料物件寫入 (SDO Write / 寫入物件字典)
    int ecx_SDOread(int slave_pos, uint16_t index, uint8_t subindex, int CA, int* size, void* data, int timeout);//服務資料物件讀取 (SDO Read / 讀取物件字典)
    int SendAndReceiveRegister(uint8_t cmd, uint16_t slaveAddr, uint16_t regAddr, void* data, int len, int timeout);//暫存器底層收發核心

    bool PDO_SendCommandAndWait(EcatCmdType type, uint16_t slave, uint16_t index, uint8_t sub, uint32_t value, int len, int timeoutMs);//非同步指令發送與同步等待 (執行緒安全指令)


    int ScanSlaves();//掃描所有從站

    // =========================================================
    // Stage 1 - Topology / Identity Verification
    //
    // 啟動前唯讀驗證：
    //
    // 1. Runtime Config 從站數量
    // 2. 實體 EtherCAT 從站數量（BRD WKC）
    // 3. 實體站序
    // 4. Vendor ID
    // 5. Product Code
    // 6. 目前 ScanSlaves() 配置的 Station Address 0x1001+
    //
    // 任一項不符即回傳 false，
    // Initialize_Slaves() 不再繼續寫入 Watchdog / SM / PDO / DC。
    //
    // Stage 1B：
    // Revision 與 Runtime XML ConfiguredAddress 已納入正式比對。
    // =========================================================
    bool VerifyTopologyAgainstRuntimeConfig();

    // =========================================================
    // Stage 2 - PRE-OP Configuration Readback Verification
    //
    // 在 PRE-OP 完成既有 SM / PDO 設定後，以唯讀方式讀回：
    //
    // 1. SyncManager Physical Start Address
    // 2. SyncManager Length
    // 3. SyncManager Control Byte
    // 4. SyncManager Enable
    // 5. A3E RxPDO Assignment / Mapping
    // 6. A3E TxPDO Assignment / Mapping
    //
    // 任一項不符即回傳 false，
    // Initialize_Slaves() 不允許繼續進入 SAFE-OP。
    //
    // 本函式不修改 SM / PDO / DC，只做 Readback + Compare。
    // =========================================================
    bool VerifyPreOpSmAndPdoConfiguration();


    // =========================================================
    // Stage 5C - Runtime Configuration Equivalence Audit
    //
    // Runtime XML executable schema
    //      vs
    // current verified C++ hard-code baseline.
    //
    // Audit only:
    // - no EtherCAT hardware writes
    // - no AL State change
    // - current Startup is not blocked
    // =========================================================
    bool AuditRuntimeConfigurationEquivalence();


    // =========================================================
    // Stage 6A - Runtime Transport Shadow Audit
    //
    // Read/compare only:
    // - Runtime Mailbox vs software mailbox + ESC SM readback
    // - Runtime FMMU vs ESC FMMU readback
    //
    // Stage 6A never writes transport hardware and never blocks
    // Startup. Cutover is deferred to later Stage 6 steps.
    // =========================================================
    bool AuditRuntimeTransportConfiguration();


    // =========================================================
    // Stage 7A - Process Image Binding Shadow Audit
    //
    // Runtime XML
    //      vs
    // existing BuildIoMap() result
    //
    // Checks:
    // - Binding kind
    // - m_IoList / m_AdList / m_ServoList classification
    // - list order
    // - exact m_IoMap pointer offsets
    // - Generic IO buffer sizes
    // - Analog INT16 channel layout
    // - Servo current ABI
    //
    // Shadow only:
    // BuildIoMap() is NOT modified and Startup is NOT blocked.
    // =========================================================
    bool AuditRuntimeProcessImageBindings();


    // =========================================================
    // Stage 11A - Composite Multi-Binding Shadow Audit
    //
    // Verifies new <ApplicationBindings> against the current
    // proven simple <ProcessImageBinding> envelope.
    //
    // Read-only:
    // - no BuildIoMap mutation
    // - no EtherCAT hardware read/write
    // - no startup gate change
    // =========================================================
    bool AuditRuntimeCompositeApplicationBindings();


    // =========================================================
    // Stage 11C.1 - Persisted Composite Runtime Shadow Audit
    //
    // Audits <CompositeApplicationBindingsShadow>:
    // - project relative bit position
    // - absolute Process Image conversion
    // - per-Slave active binding envelope
    // - overlap / range
    //
    // No hardware access and no Runtime cutover.
    // =========================================================
    bool AuditRuntimeCompositeProjectShadowBindings();


    // =========================================================
    // Stage 11C.2 - Composite Runtime Adapter Shadow Build
    //
    // Generic application descriptors over m_IoMap.
    // Legacy Stage7B adapters remain authoritative.
    // =========================================================
    bool BuildRuntimeCompositeApplicationDescriptorsShadow();
    bool AuditRuntimeCompositeApplicationDescriptorsShadow();


    // =========================================================
    // Stage 11D.1 - Structured ServoDrive Adapter Shadow
    //
    // Expands each approved ServoDrive parent descriptor into
    // field-level Generic descriptors while preserving the exact
    // ServoPDO_9_23 ABI.
    //
    // Shadow only:
    // - no Motion consumer cutover
    // - no Servo command cutover
    // - no Process Image write
    // =========================================================

    bool BuildStructuredServoDriveFieldDescriptorsShadow();

    bool AuditStructuredServoDriveFieldDescriptorsShadow();

    const EtherCatStructuredServoFieldDescriptor*
        FindStructuredServoDriveFieldShadow(
            int slaveIndex,
            const char* fieldId) const;


    // =========================================================
    // Stage 11D.2 - Structured ServoDrive Live Read SHADOW
    //
    // Reads only the 8 INPUT fields per ServoDrive through the
    // Stage11D.1 structured descriptors and compares them with
    // the current Motion-facing ENI_ServoDrive::pInput view.
    //
    // No Motion / Servo command cutover.
    // =========================================================

    bool PrepareStructuredServoDriveLiveReadShadow();

    void SampleStructuredServoDriveLiveReadShadow();

    void PrintStructuredServoDriveLiveReadShadow() const;

    bool ReadStructuredServoInputFieldBitsShadow(
        int slaveIndex,
        const char* fieldId,
        uint64_t& value) const;


    // =========================================================
    // Stage 11D.3 - Motion Servo Input Consumer Bridge SHADOW
    //
    // Current Motion identity:
    //
    //     Motion slot + AxisContext.axisIndex
    //         -> ServoDrive slave
    //         -> structured Servo input fields
    //
    // Axis NAME is intentionally outside this EtherCAT mapping.
    // NC owns axis-name / parameter interpretation.
    //
    // Shadow only:
    // - Motion consumer remains ENI_ServoDrive
    // - Servo command remains ENI_ServoDrive
    // =========================================================

    bool PrepareMotionServoInputConsumerBridgeShadow();

    void SampleMotionServoInputConsumerBridgeShadow();

    void PrintMotionServoInputConsumerBridgeShadow() const;


    // =========================================================
    // Stage 11D.4 Rev2 - AxisIndex Semantic Identity Gate
    //
    // EtherCAT / Motion identity is AxisContext.axisIndex.
    //
    // This gate verifies:
    //
    //     Motion slot
    //         -> AxisIndex
    //         -> Servo slave
    //         -> structured Servo field owner
    //
    // It does NOT use:
    // - AXIS_CFG.ini
    // - X/Y/Z names
    // - Runtime AxisRef
    //
    // NC remains the only owner of axis-name interpretation.
    // =========================================================

    bool AuditMotionServoAxisIndexIdentityGate();

    void PrintMotionServoAxisIndexIdentityGate() const;


    // =========================================================
    // Stage 11D.5 - AxisIndex Semantic Motion Input Preflight
    //
    // Typed read API keyed ONLY by AxisContext.axisIndex.
    //
    // This is the candidate semantic read contract for Motion.
    //
    // Actual MotionCore consumers remain legacy in Stage11D.5.
    // =========================================================

    bool PrepareMotionServoAxisIndexInputPreflightShadow();

    void SampleMotionServoAxisIndexInputPreflightShadow();

    void PrintMotionServoAxisIndexInputPreflightShadow() const;


    bool ReadMotionServoInputByAxisIndexShadow(
        int axisIndex,
        uint16_t& statusWord,
        int32_t& actualPosition,
        int8_t& modesOfOperationDisplay,
        uint16_t& touchProbeStatus,
        int32_t& touchProbePosition) const;


    // =========================================================
    // Stage 11D.6 - Motion Input Consumer Seam Preflight
    // =========================================================

    bool PrepareMotionServoInputConsumerSeamPreflightShadow();

    void ObserveMotionServoInputConsumerSeamShadow(
        int axisIndex,
        uint16_t statusWord,
        int32_t actualPosition,
        int8_t modesOfOperationDisplay,
        uint16_t touchProbeStatus,
        int32_t touchProbePosition);

    void PrintMotionServoInputConsumerSeamPreflightShadow() const;


    // =========================================================
    // Stage 11D.7 - Controlled AxisIndex Semantic Core Motion
    // Input Cutover
    //
    // Current-boot behavior:
    //
    // 1. Start on LEGACY snapshot.
    // 2. Wait until Stage11D.6 runtime qualification is clean.
    // 3. Enable AxisIndex semantic snapshot for:
    //        UpdateMotion
    //        UpdateServoState
    // 4. Any semantic read failure latches rollback to LEGACY
    //    for the rest of this boot.
    //
    // Servo command/output remains LEGACY.
    // =========================================================

    bool PrepareControlledMotionServoInputCutover();

    void UpdateControlledMotionServoInputCutoverGate();

    bool TryReadControlledMotionServoInputByAxisIndex(
        int axisIndex,
        MotionServoInputSnapshot& snapshot);

    void PrintControlledMotionServoInputCutover() const;


    // =========================================================
    // Stage 11D.8 - Semantic Motion Input Compatibility
    // Retirement
    //
    // The 250us Motion thread publishes the exact snapshot it
    // consumes.  Cross-thread compatibility readers consume that
    // stable published snapshot after Stage11D.7 is qualified.
    //
    // Compatibility consumers:
    // - GetDriveTouchProbeData
    // - ExportDebugInfo
    //
    // Legacy pInput remains warmup / emergency fallback only.
    // =========================================================

    bool PrepareMotionServoInputCompatibilityRetirement();


    void PublishMotionServoInputConsumerSnapshot(
        int axisIndex,
        const MotionServoInputSnapshot& snapshot);


    bool TryReadMotionServoPublishedInputCompatibility(
        int axisIndex,
        MotionServoInputSnapshot& snapshot);


    void PrintMotionServoInputCompatibilityRetirement() const;


    // =========================================================
    // Stage 11D.9 - Legacy Servo Input Normal-Path Retirement
    //
    // After Stage11D.8 full qualification:
    //
    // Normal 250us Motion input:
    //     AXIS_INDEX_SEMANTIC only
    //
    // Legacy ENI_ServoDrive::pInput:
    //     emergency fallback only
    //
    // Historical D3 / D5 / D6 runtime comparison windows are
    // frozen after retirement.
    // =========================================================

    bool PrepareLegacyMotionServoInputNormalPathRetirement();

    void UpdateLegacyMotionServoInputNormalPathRetirementGate();

    bool IsLegacyMotionServoInputNormalPathRetired() const;


    bool TryReadRetiredMotionServoInputByAxisIndex(
        int axisIndex,
        MotionServoInputSnapshot& snapshot);


    void ReportLegacyMotionServoInputEmergencyFallback();

    void PrintLegacyMotionServoInputNormalPathRetirement() const;


    // =========================================================
    // Stage 11D.10 - Servo Input Release / Diagnostic Retirement
    //
    // Final Servo INPUT release gate.
    //
    // After D9 + D2 qualification:
    // - request D2 1ms sampler stop
    // - wait until no D2 sample is active
    // - freeze D2 counters
    // - seal semantic Servo INPUT mainline
    //
    // Legacy pInput remains emergency fallback code only.
    // Servo OUTPUT remains a separate future track.
    // =========================================================

    bool PrepareServoInputReleaseGate();

    void UpdateServoInputReleaseGate();

    bool IsServoInputReleaseComplete() const;

    void PrintServoInputReleaseGate() const;


    // =========================================================
    // Stage 11E.1 - Servo Output Command Ownership SHADOW
    //
    // Independent Servo OUTPUT track.
    //
    // Current authoritative command path remains:
    //
    //     ENI_ServoDrive::pOutput
    //
    // This stage only prepares and observes:
    //
    //     AxisIndex
    //         -> structured Servo output descriptors
    //
    // No semantic output write / cutover.
    // =========================================================

    bool PrepareMotionServoOutputCommandOwnershipShadow();


    bool ReadMotionServoOutputByAxisIndexShadow(
        int axisIndex,
        uint16_t& controlWord,
        int32_t& targetVelocity,
        uint16_t& touchProbeFunction,
        int8_t& modesOfOperation) const;


    void ObserveMotionServoOutputCommandOwnershipShadow(
        int axisIndex,
        uint16_t controlWord,
        int32_t targetVelocity,
        uint16_t touchProbeFunction,
        int8_t modesOfOperation);


    void PrintMotionServoOutputCommandOwnershipShadow() const;


    // =========================================================
    // Stage 11E.2 - Servo Output Semantic Scratch Command Bridge
    //
    // Prerequisite:
    //     Stage11E.1 runtime ownership shadow qualified.
    //
    // Candidate write route:
    //
    //     AxisIndex
    //         -> structured Servo OUTPUT descriptors
    //         -> LOCAL ServoOutput scratch image
    //
    // The live ENI_ServoDrive::pOutput / Process Image is never
    // modified by Stage11E.2.
    // =========================================================

    bool PrepareMotionServoOutputSemanticScratchBridgeShadow();

    void PrintMotionServoOutputSemanticScratchBridgeShadow() const;


    // =========================================================
    // Stage 11E.3 - Servo Output Live-Write Target / Rollback
    // Preflight SHADOW
    //
    // Proves the exact future physical target:
    //
    //     AxisIndex
    //         -> structured Servo OUTPUT descriptors
    //         -> same ServoOutput bytes inside m_IoMap
    //
    // All candidate writes are redirected to a LOCAL shadow
    // window. The live Process Image is never modified.
    // =========================================================

    bool PrepareMotionServoOutputLiveWritePreflightShadow();

    void PrintMotionServoOutputLiveWritePreflightShadow() const;


    // =========================================================
    // Stage 11E.4 - Servo Output Command Seam SHADOW
    //
    // MotionCore calls this immediately AFTER each write through
    // the centralized LEGACY command seam.
    //
    // Active producer remains:
    //
    //     MotionCore seam -> same ENI ServoOutput / m_IoMap
    //
    // Structured output remains observation-only.
    // =========================================================

    bool PrepareMotionServoOutputCommandSeamShadow();


    void ObserveMotionServoOutputCommandSeamWriteShadow(
        int axisIndex,
        MotionServoOutputCommandField field,
        int64_t value);


    void PrintMotionServoOutputCommandSeamShadow() const;


    // =========================================================
    // Stage 11E.5 - Controlled Structured Servo Output Producer
    // Cutover
    //
    // true:
    //     structured AxisIndex field write + readback succeeded.
    //
    // false:
    //     MotionCore writes the same command through legacy
    //     pOutput fallback.
    //
    // Any structured fault is boot-latched to legacy.
    // =========================================================

    bool PrepareMotionServoOutputStructuredProducerCutover();

    bool TryWriteMotionServoOutputCommandStructured(
        int axisIndex,
        MotionServoOutputCommandField field,
        int64_t value,
        ServoOutput* legacyOutput);

    void PrintMotionServoOutputStructuredProducerCutover() const;


    // =========================================================
    // Stage 11E.6 - Servo Output Compatibility Retirement Shadow
    // =========================================================

    bool PrepareMotionServoOutputCompatibilityRetirementShadow();


    void ObserveMotionServoOutputFinalCommandRuntime(
        int axisIndex,
        uint16_t controlWord,
        int32_t targetVelocity,
        uint16_t touchProbeFunction,
        int8_t modesOfOperation);


    void PrintMotionServoOutputCompatibilityRetirementShadow() const;


    // =========================================================
    // Stage 11E.7 - Servo Generic I/O Release Gate
    // =========================================================

    bool PrepareServoGenericIoReleaseGate();

    void UpdateServoGenericIoReleaseGate();

    bool IsServoGenericIoReleaseComplete() const;

    void PrintServoGenericIoReleaseGate() const;


    // =========================================================
    // Stage 11C.3 - Generic Adapter Access API Shadow
    //
    // READ:
    // - Find by Slave + Binding Id
    // - Find by Kind + AxisRef + occurrence
    // - read <= 64-bit Input / Output fields
    // - typed Int16 / Int32 / Int64 Input reads
    //
    // WRITE:
    // - output write is SCRATCH BUFFER ONLY in this stage
    // - live m_IoMap is never modified by the shadow write API
    //
    // These APIs operate on the generic Stage11C.2 descriptor
    // layer and do not inspect ProductCode.
    // =========================================================

    const EtherCatCompositeApplicationDescriptor*
        FindCompositeBindingShadow(
            int slaveIndex,
            const char* bindingId) const;

    const EtherCatCompositeApplicationDescriptor*
        FindCompositeBindingByKindAxisShadow(
            const char* kind,
            const char* axisRef,
            int occurrence = 0) const;

    bool ReadCompositeInputBitsShadow(
        int slaveIndex,
        const char* bindingId,
        uint64_t& value) const;

    bool ReadCompositeOutputBitsShadow(
        int slaveIndex,
        const char* bindingId,
        uint64_t& value) const;

    bool ReadCompositeInputInt16Shadow(
        int slaveIndex,
        const char* bindingId,
        int16_t& value) const;

    bool ReadCompositeInputInt32Shadow(
        int slaveIndex,
        const char* bindingId,
        int32_t& value) const;

    bool ReadCompositeInputInt64Shadow(
        int slaveIndex,
        const char* bindingId,
        int64_t& value) const;

    bool WriteCompositeOutputBitsToScratchShadow(
        int slaveIndex,
        const char* bindingId,
        uint64_t value,
        uint8_t* scratchIoMap,
        uint32_t scratchIoMapBytes) const;

    bool AuditRuntimeCompositeAccessApiShadow();


    // =========================================================
    // Stage 11C.4 - Generic LIVE Read Route
    //
    // First controlled cutover:
    // - approved scalar INPUT functions use the generic descriptor
    //   layer as the authoritative read route.
    //
    // NOT cut over:
    // - live outputs
    // - PLC legacy read/write mapping
    // - Servo/Motion structured PDO access
    //
    // Current approved examples:
    // DigitalInput, AnalogInput, PositionFeedback,
    // LatchCapture, Status, Timestamp.
    // =========================================================

    bool BuildRuntimeCompositeLiveReadRoute();

    const EtherCatCompositeApplicationDescriptor*
        FindCompositeLiveReadBinding(
            int slaveIndex,
            const char* bindingId) const;

    const EtherCatCompositeApplicationDescriptor*
        FindCompositeLiveReadByKindAxis(
            const char* kind,
            const char* axisRef,
            int occurrence = 0) const;

    bool ReadCompositeLiveInputBits(
        int slaveIndex,
        const char* bindingId,
        uint64_t& value) const;

    bool ReadCompositeLiveInputInt16(
        int slaveIndex,
        const char* bindingId,
        int16_t& value) const;

    bool ReadCompositeLiveInputInt32(
        int slaveIndex,
        const char* bindingId,
        int32_t& value) const;

    bool ReadCompositeLiveInputInt64(
        int slaveIndex,
        const char* bindingId,
        int64_t& value) const;

    bool AuditRuntimeCompositeLiveReadRoute();


    // =========================================================
    // Stage 11C.5 - Generic Read Consumer Bridge SHADOW
    //
    // Consumer identity:
    //
    //     Kind + AxisRef + occurrence
    //
    // Examples:
    //
    //     DigitalInput / "" / 0
    //     AnalogInput  / "" / 2
    //     PositionFeedback / X / 0
    //     LatchCapture     / X / 0
    //
    // The bridge consumes Stage11C.4 GENERIC_LIVE_READ.
    //
    // Existing PLC / EDM / Motion consumers are NOT switched yet.
    // =========================================================

    const EtherCatCompositeApplicationDescriptor*
        ResolveCompositeReadConsumerShadow(
            const char* kind,
            const char* axisRef,
            int occurrence = 0) const;

    bool ReadCompositeConsumerBitsShadow(
        const char* kind,
        const char* axisRef,
        int occurrence,
        uint64_t& value) const;

    bool ReadCompositeConsumerInt16Shadow(
        const char* kind,
        const char* axisRef,
        int occurrence,
        int16_t& value) const;

    bool ReadCompositeConsumerInt32Shadow(
        const char* kind,
        const char* axisRef,
        int occurrence,
        int32_t& value) const;

    bool ReadCompositeConsumerInt64Shadow(
        const char* kind,
        const char* axisRef,
        int occurrence,
        int64_t& value) const;

    bool AuditRuntimeCompositeReadConsumerBridgeShadow();


    // =========================================================
    // Stage 11C.6 - EDM Read Consumer API LIVE Cutover
    //
    // Promotes the semantic consumer API to ACTIVE:
    //
    //     Kind + AxisRef + occurrence
    //
    // The underlying source remains Stage11C.4 GENERIC_LIVE_READ.
    //
    // Existing System_EDM_SINKER_MODE.cpp call sites are not
    // migrated in this stage; they remain legacy until the next
    // controlled call-site migration stage.
    //
    // NO live output write.
    // NO Servo/Motion cutover.
    // =========================================================

    bool BuildRuntimeCompositeReadConsumerLiveRoute();

    const EtherCatCompositeApplicationDescriptor*
        ResolveCompositeReadConsumer(
            const char* kind,
            const char* axisRef,
            int occurrence = 0) const;

    bool ReadCompositeConsumerBits(
        const char* kind,
        const char* axisRef,
        int occurrence,
        uint64_t& value) const;

    bool ReadCompositeConsumerInt16(
        const char* kind,
        const char* axisRef,
        int occurrence,
        int16_t& value) const;

    bool ReadCompositeConsumerInt32(
        const char* kind,
        const char* axisRef,
        int occurrence,
        int32_t& value) const;

    bool ReadCompositeConsumerInt64(
        const char* kind,
        const char* axisRef,
        int occurrence,
        int64_t& value) const;

    bool AuditRuntimeCompositeReadConsumerLiveRoute();


    // =========================================================
    // Stage 8A - Generic Runtime PRE-OP Verification Shadow Audit
    //
    // Runtime XML becomes the EXPECTED source for readback only:
    //
    // - SyncManagers
    // - Configurable PDO Assignment
    // - Configurable PDO Mapping
    //
    // Stage 8A does NOT replace the existing Stage-2 ProductCode
    // verifier yet. The existing verifier remains the real safety gate.
    // =========================================================
    bool AuditRuntimePreOpConfigurationGeneric();


    // =========================================================
    // Stage 9B - Generic Runtime Configuration Contract Gate
    //
    // Pure schema / cross-section consistency verification.
    //
    // No EtherCAT hardware read/write.
    //
    // With complete modern Runtime schema, Startup uses this
    // result to open the Runtime hardware cutover gate without
    // VendorId/ProductCode hard-code baselines.
    //
    // Old XML may still use Stage5C as compatibility fallback.
    // =========================================================
    bool AuditRuntimeConfigurationContractGeneric();


    // =========================================================
    // Stage 7B - Runtime Process Image Binding Cutover
    //
    // Called only when EVERY Runtime slave has an explicit
    // <ProcessImageBinding> section.
    //
    // Builds:
    // - m_IoList
    // - m_AdList
    // - m_ServoList
    // - m_IoMap pointers
    //
    // Classification and offsets come from Runtime XML.
    //
    // No EtherCAT hardware register is written here.
    // =========================================================
    int BuildRuntimeProcessImageBindings();


    // =========================================================
    // Stage 6C - Runtime FMMU Cutover
    //
    // Preflight:
    // Runtime FMMU plan must match the exact Process Image layout
    // produced by BuildIoMap():
    //
    // Slave order
    //   Output
    //   Input
    //
    // Write:
    // Runtime XML -> ESC FMMU
    //
    // Verify:
    // every write is immediately APRD read back.
    //
    // Legacy XML with no <Fmmus> may still use Config_Slave_FMMU().
    // =========================================================
    bool PreflightRuntimeFmmuConfiguration();

    bool ShouldUseRuntimeFmmuConfiguration(
        int slaveIdx) const;

    bool ConfigureRuntimeFmmus(
        int slaveIdx);


    // =========================================================
    // Stage 5D.1 - Fixed IO SyncManager Data-Driven Cutover
    //
    // First real Runtime XML hardware cutover.
    //
    // Current scope:
    // - R1-EC6002
    // - R1-EC7062
    // - R1-EC8124D0
    //
    // Safety:
    // - Runtime Equivalence Audit must be EQUIVALENT
    // - otherwise these functions return false and the old C++
    //   hard-code path remains active
    // =========================================================
    bool ShouldUseRuntimeSyncManagerConfiguration(
        int slaveIdx) const;

    bool ConfigureRuntimeSyncManagers(
        int slaveIdx);


    // =========================================================
    // Stage 5D - Complete Runtime Data-Driven Cutover
    //
    // Gate:
    // Runtime Equivalence Audit must be fully EQUIVALENT.
    //
    // INIT:
    // - Runtime SyncManagers
    // - Runtime DC / Sync0 ESC registers
    //
    // PRE-OP:
    // - Runtime Configurable PDO mapping
    // - Runtime DC 1C32 / 1C33
    // - Runtime Watchdog
    // - Runtime PRE_OP InitCommands
    //
    // SAFE-OP:
    // - Runtime SAFE_OP InitCommands
    //
    // Existing hard-code methods remain as automatic fallback.
    // =========================================================
    bool ShouldUseRuntimeConfiguration(
        int slaveIdx) const;

    bool ConfigureRuntimeInitStage(
        int slaveIdx,
        uint64_t unifiedStartTime);

    bool ConfigureRuntimePreOpStage(
        int slaveIdx);

    bool ConfigureRuntimeSafeOpStage(
        int slaveIdx);

    bool InitSlaveMailboxInfo(int slave_idx);//Stage 6B: Runtime Mailbox first, legacy fallback
    uint16_t ReadSII_Word16(int slave_idx, uint16_t word_addr);//讀取從站EEPROM 功能
    uint32_t ReadSII_Uint32(int slave_idx, uint16_t word_addr);
    // 流程: PRE-OP -> Config -> SAFE-OP -> OP
    int Initialize_Slaves();//初始化所有從站 INIT>>PRE-OP>>SAFE-OP>>OP
    void MeasureDCPortTimestamps();// Distributed Clock 分散式時鐘 診斷DC使用

    // 舊版固定 S4/S5/S6 介面暫時保留但不再由初始化流程呼叫。
    bool MeasureDCPropagationDelay(uint32_t& delaySlave4, uint32_t& delaySlave5, uint32_t& delaySlave6);
    bool ConfigureDCPropagationDelay(uint32_t delaySlave4, uint32_t delaySlave5, uint32_t delaySlave6);

    // AUTO：依 m_ServoList 實體順序量測並設定最多 8 軸 propagation delay。
    bool MeasureDCPropagationDelayAuto(
        DCAutoPropagationTable& delayTable);

    bool ConfigureDCPropagationDelayAuto(
        const DCAutoPropagationTable& delayTable);

    void UpdateDCMasterClockEstimator(
        uint64_t masterBeforeNs,
        uint64_t masterAfterNs,
        uint64_t dcReferenceNs,
        int dcWkc);

    void UpdateDCPdoPhaseController(
        uint64_t pdoStartMasterNs);


    void ConfigureSlaveGeneric_INIT(int slaveIdx, uint64_t unifiedStartTime);//從站配置INIT
    void ConfigureSlaveGeneric_PRE_OP(int slaveIdx);//從站配置PRE_OP
    void ConfigureSlaveGeneric_SAFE_OP(int slaveIdx);//從站配置SAFE_OP
    uint64_t GetCurrentMasterTimeNs();// 取得 RTX64 系統時間 (單位: 奈秒)
    void  Printf_Slaves_State();//印出從站狀態
    void  Printf_AL_Status_Code();//印出從站狀態0x134 code

    // IO  Function
    bool Get_I(int moduleIdx, int bitIdx);
    bool Get_O(int moduleIdx, int bitIdx);
    void Set_O(int moduleIdx, int bitIdx, bool val);

    std::vector<EtherCatSlave> m_slaves;
    std::vector<ENI_GenericIO> m_IoList;       // 只存 6002, 7062 (數位)
    std::vector<ENI_AnalogModule> m_AdList;    // 只存 8124 (類比)
    std::vector<ENI_ServoDrive> m_ServoList;

    // Stage 11C.2 generic descriptor SHADOW layer.
    // It points into m_IoMap but is not consumed by PLC/Motion.
    std::vector<EtherCatCompositeApplicationDescriptor>
        m_CompositeApplicationDescriptorsShadow;


    // Stage 11D.1 structured ServoDrive field SHADOW layer.
    //
    // Parent ownership remains the Stage11C.2 ServoDrive
    // Composite descriptor.  These are field-level child views.
    std::vector<EtherCatStructuredServoFieldDescriptor>
        m_StructuredServoDriveFieldsShadow;


    // =========================================================
    // Stage 11D.2 - sustained structured Servo INPUT SHADOW.
    //
    // Sampled from PlcCore's existing 1ms input bridge.
    // Diagnostics are read from the 1000ms supervisory path.
    // =========================================================

    bool m_StructuredServoLiveReadShadowPrepared =
        false;

    int m_StructuredServoLiveReadInputFieldCount =
        0;

    std::atomic<uint64_t> m_StructuredServoLiveReadCycles
    {
        0ULL
    };

    std::atomic<uint64_t> m_StructuredServoLiveReadExpectedChecks
    {
        0ULL
    };

    std::atomic<uint64_t> m_StructuredServoLiveReadStableChecks
    {
        0ULL
    };

    std::atomic<uint64_t> m_StructuredServoLiveReadMatches
    {
        0ULL
    };

    std::atomic<uint64_t> m_StructuredServoLiveReadUnstableSkips
    {
        0ULL
    };

    std::atomic<uint64_t> m_StructuredServoLiveReadMismatches
    {
        0ULL
    };

    std::atomic<uint64_t> m_StructuredServoLiveReadFailures
    {
        0ULL
    };


    // Stage 11D.10 D2 sampler retirement handshake.
    std::atomic<bool>
        m_ServoInputReleaseD2StopRequested
    {
        false
    };

    std::atomic<uint32_t>
        m_ServoInputReleaseD2SamplerActive
    {
        0U
    };


    // =========================================================
    // Stage 11D.3 - Motion Servo Input Consumer Bridge SHADOW
    // =========================================================

    struct MotionServoInputBridgeShadowMap
    {
        int motionSlot = -1;
        int axisIndex = -1;
        int servoSlaveIndex = -1;

        const EtherCatStructuredServoFieldDescriptor*
            statusWord = nullptr;

        const EtherCatStructuredServoFieldDescriptor*
            actualPosition = nullptr;

        const EtherCatStructuredServoFieldDescriptor*
            modesOfOperationDisplay = nullptr;

        const EtherCatStructuredServoFieldDescriptor*
            touchProbeStatus = nullptr;

        const EtherCatStructuredServoFieldDescriptor*
            touchProbePosition = nullptr;
    };


    std::vector<MotionServoInputBridgeShadowMap>
        m_MotionServoInputBridgeShadowMaps;


    bool m_MotionServoInputBridgeShadowPrepared =
        false;

    int m_MotionServoInputBridgeFieldCount =
        0;


    std::atomic<uint64_t> m_MotionServoInputBridgeCycles
    {
        0ULL
    };

    std::atomic<uint64_t> m_MotionServoInputBridgeChecks
    {
        0ULL
    };

    std::atomic<uint64_t> m_MotionServoInputBridgeMatches
    {
        0ULL
    };

    std::atomic<uint64_t> m_MotionServoInputBridgeMismatches
    {
        0ULL
    };

    std::atomic<uint64_t> m_MotionServoInputBridgeFailures
    {
        0ULL
    };


    // =========================================================
    // Stage 11D.4 Rev2 - AxisIndex Semantic Identity Gate
    // =========================================================

    struct MotionServoAxisIndexIdentityAuditEntry
    {
        int motionSlot = -1;
        int axisIndex = -1;
        int servoSlaveIndex = -1;

        bool slotValid = false;
        bool axisIndexValid = false;
        bool contextMatch = false;
        bool servoMatch = false;
        bool structuredFieldOwnerMatch = false;
    };


    std::vector<MotionServoAxisIndexIdentityAuditEntry>
        m_MotionServoAxisIndexIdentityAuditEntries;


    bool m_MotionServoAxisIndexIdentityAuditRan =
        false;

    bool m_MotionServoAxisIndexIdentityGatePassed =
        false;

    int m_MotionServoAxisIndexIdentityValidAxes =
        0;

    int m_MotionServoAxisIndexIdentityContextMatches =
        0;

    int m_MotionServoAxisIndexIdentityServoMatches =
        0;

    int m_MotionServoAxisIndexIdentityFieldOwnerMatches =
        0;

    int m_MotionServoAxisIndexIdentityDuplicates =
        0;

    int m_MotionServoAxisIndexIdentityErrors =
        0;


    // =========================================================
    // Stage 11D.5 - AxisIndex Semantic Motion Input Preflight
    //
    // Fixed array deliberately indexed by AxisIndex.
    //
    // This makes the semantic route independent of physical
    // Servo list order during runtime lookup.
    // =========================================================

    struct MotionServoAxisIndexInputRouteShadow
    {
        bool valid = false;

        int axisIndex = -1;
        int motionSlot = -1;
        int servoSlaveIndex = -1;

        const EtherCatStructuredServoFieldDescriptor*
            statusWord = nullptr;

        const EtherCatStructuredServoFieldDescriptor*
            actualPosition = nullptr;

        const EtherCatStructuredServoFieldDescriptor*
            modesOfOperationDisplay = nullptr;

        const EtherCatStructuredServoFieldDescriptor*
            touchProbeStatus = nullptr;

        const EtherCatStructuredServoFieldDescriptor*
            touchProbePosition = nullptr;
    };


    MotionServoAxisIndexInputRouteShadow
        m_MotionServoAxisIndexInputRoutesShadow[MAX_AXES] = {};


    bool m_MotionServoAxisIndexInputPreflightPrepared =
        false;

    int m_MotionServoAxisIndexInputRouteCount =
        0;

    int m_MotionServoAxisIndexInputFieldCount =
        0;


    std::atomic<uint64_t>
        m_MotionServoAxisIndexInputPreflightCycles
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoAxisIndexInputPreflightChecks
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoAxisIndexInputPreflightMatches
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoAxisIndexInputPreflightMismatches
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoAxisIndexInputPreflightFailures
    {
        0ULL
    };


    // =========================================================
    // Stage 11D.6 - Motion Input Consumer Seam Preflight
    // =========================================================

    bool m_MotionServoInputConsumerSeamPreflightPrepared =
        false;

    std::atomic<uint64_t>
        m_MotionServoInputConsumerSeamAxisSamples
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoInputConsumerSeamChecks
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoInputConsumerSeamMatches
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoInputConsumerSeamMismatches
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoInputConsumerSeamReadFailures
    {
        0ULL
    };


    // =========================================================
    // Stage 11D.7 - Controlled AxisIndex Semantic Core Motion
    // Input Cutover
    // =========================================================

    bool m_ControlledMotionServoInputCutoverPrepared =
        false;

    std::atomic<bool>
        m_ControlledMotionServoInputCutoverEnabled
    {
        false
    };

    std::atomic<bool>
        m_ControlledMotionServoInputCutoverFaultLatched
    {
        false
    };

    std::atomic<uint64_t>
        m_ControlledMotionServoInputCutoverTransitions
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_ControlledMotionServoInputLegacyAxisSamples
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_ControlledMotionServoInputSemanticAxisSamples
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_ControlledMotionServoInputFallbackAxisSamples
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_ControlledMotionServoInputRouteFaults
    {
        0ULL
    };


    // =========================================================
    // Stage 11D.8 - Published active Motion input snapshot.
    //
    // All payload fields are atomic.
    // sequence provides a coherent multi-field snapshot.
    // =========================================================

    struct PublishedMotionServoInputAtomic
    {
        std::atomic<uint32_t>
            sequence
        {
            0U
        };

        std::atomic<uint16_t>
            statusWord
        {
            0U
        };

        std::atomic<int32_t>
            actualPosition
        {
            0
        };

        std::atomic<int8_t>
            modesOfOperationDisplay
        {
            0
        };

        std::atomic<uint16_t>
            touchProbeStatus
        {
            0U
        };

        std::atomic<int32_t>
            touchProbePosition
        {
            0
        };

        std::atomic<uint64_t>
            publishCount
        {
            0ULL
        };
    };


    PublishedMotionServoInputAtomic
        m_PublishedMotionServoInput[MAX_AXES];


    bool m_MotionServoInputCompatibilityRetirementPrepared =
        false;


    std::atomic<uint64_t>
        m_MotionServoInputCompatibilityPublishedAxisSamples
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoInputCompatibilitySemanticReads
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoInputCompatibilityWarmupLegacyReads
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoInputCompatibilityFallbackReads
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoInputCompatibilityReadFailures
    {
        0ULL
    };


    // =========================================================
    // Stage 11D.9 - Legacy Servo Input Normal-Path Retirement
    // =========================================================

    bool m_LegacyMotionServoInputNormalPathRetirementPrepared =
        false;


    std::atomic<bool>
        m_LegacyMotionServoInputNormalPathRetired
    {
        false
    };


    std::atomic<bool>
        m_LegacyMotionServoInputNormalPathRetirementFaultLatched
    {
        false
    };


    std::atomic<uint64_t>
        m_LegacyMotionServoInputNormalPathRetirementTransitions
    {
        0ULL
    };


    std::atomic<uint64_t>
        m_LegacyMotionServoInputRetiredSemanticAxisSamples
    {
        0ULL
    };


    std::atomic<uint64_t>
        m_LegacyMotionServoInputEmergencyFallbackReads
    {
        0ULL
    };


    uint64_t
        m_LegacyMotionServoInputFrozenD3Cycles =
        0ULL;

    uint64_t
        m_LegacyMotionServoInputFrozenD5Cycles =
        0ULL;

    uint64_t
        m_LegacyMotionServoInputFrozenD6AxisSamples =
        0ULL;


    // =========================================================
    // Stage 11D.10 - Servo Input Release
    // =========================================================

    bool m_ServoInputReleaseGatePrepared =
        false;


    std::atomic<bool>
        m_ServoInputReleaseComplete
    {
        false
    };


    std::atomic<uint64_t>
        m_ServoInputReleaseTransitions
    {
        0ULL
    };


    uint64_t
        m_ServoInputReleaseFrozenD2Cycles =
        0ULL;

    uint64_t
        m_ServoInputReleaseFrozenD2ExpectedChecks =
        0ULL;


    // =========================================================
    // Stage 11E.1 - Servo Output Command Ownership SHADOW
    //
    // Fixed route table indexed by AxisIndex.
    // =========================================================

    struct MotionServoAxisIndexOutputRouteShadow
    {
        bool valid = false;

        int axisIndex = -1;
        int motionSlot = -1;
        int servoSlaveIndex = -1;

        const EtherCatStructuredServoFieldDescriptor*
            controlWord = nullptr;

        const EtherCatStructuredServoFieldDescriptor*
            targetVelocity = nullptr;

        const EtherCatStructuredServoFieldDescriptor*
            touchProbeFunction = nullptr;

        const EtherCatStructuredServoFieldDescriptor*
            modesOfOperation = nullptr;
    };


    MotionServoAxisIndexOutputRouteShadow
        m_MotionServoAxisIndexOutputRoutesShadow[MAX_AXES] = {};


    bool m_MotionServoOutputCommandOwnershipShadowPrepared =
        false;

    int m_MotionServoOutputCommandOwnershipRouteCount =
        0;

    int m_MotionServoOutputCommandOwnershipFieldCount =
        0;


    std::atomic<uint64_t>
        m_MotionServoOutputCommandOwnershipAxisSamples
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputCommandOwnershipChecks
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputCommandOwnershipMatches
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputCommandOwnershipMismatches
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputCommandOwnershipReadFailures
    {
        0ULL
    };


    // =========================================================
    // Stage 11E.2 - Servo Output Semantic Scratch Command Bridge
    // =========================================================

    bool m_MotionServoOutputSemanticScratchBridgePrepared =
        false;

    int m_MotionServoOutputSemanticScratchBridgeRouteCount =
        0;

    int m_MotionServoOutputSemanticScratchBridgeFieldCount =
        0;


    std::atomic<uint64_t>
        m_MotionServoOutputSemanticScratchAxisSamples
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputSemanticScratchFieldChecks
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputSemanticScratchFieldMatches
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputSemanticScratchFieldMismatches
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputSemanticScratchWriteFailures
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputSemanticScratchImageChecks
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputSemanticScratchImageMatches
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputSemanticScratchImageFailures
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputSemanticScratchLivePreserveChecks
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputSemanticScratchLivePreserveMatches
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputSemanticScratchLiveMutationFailures
    {
        0ULL
    };


    void ObserveMotionServoOutputSemanticScratchBridgeShadow(
        int axisIndex,
        uint16_t controlWord,
        int32_t targetVelocity,
        uint16_t touchProbeFunction,
        int8_t modesOfOperation);


    // =========================================================
    // Stage 11E.3 - Servo Output Live-Write Target / Rollback
    // Preflight SHADOW
    // =========================================================

    struct MotionServoOutputLiveWritePreflightRoute
    {
        bool valid = false;

        int axisIndex = -1;
        int motionSlot = -1;
        int servoSlaveIndex = -1;

        uint32_t outputOffsetBytes = 0U;
        uint32_t outputSpanBytes = 0U;
    };


    MotionServoOutputLiveWritePreflightRoute
        m_MotionServoOutputLiveWritePreflightRoutes[MAX_AXES] = {};


    bool m_MotionServoOutputLiveWritePreflightPrepared =
        false;

    int m_MotionServoOutputLiveWritePreflightRouteCount =
        0;

    int m_MotionServoOutputLiveWritePreflightFieldCount =
        0;


    std::atomic<uint64_t>
        m_MotionServoOutputLiveWritePreflightAxisSamples
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputLiveWritePreflightTargetChecks
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputLiveWritePreflightTargetMatches
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputLiveWritePreflightTargetFailures
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputLiveWritePreflightImageChecks
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputLiveWritePreflightImageMatches
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputLiveWritePreflightImageFailures
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputLiveWritePreflightRollbackChecks
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputLiveWritePreflightRollbackMatches
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputLiveWritePreflightRollbackFailures
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputLiveWritePreflightLivePreserveChecks
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputLiveWritePreflightLivePreserveMatches
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputLiveWritePreflightLiveMutationFailures
    {
        0ULL
    };


    void ObserveMotionServoOutputLiveWritePreflightShadow(
        int axisIndex,
        uint16_t controlWord,
        int32_t targetVelocity,
        uint16_t touchProbeFunction,
        int8_t modesOfOperation);


    // =========================================================
    // Stage 11E.4 - Servo Output Command Seam SHADOW
    // =========================================================

    struct MotionServoOutputCommandSeamAxisState
    {
        bool valid = false;

        int axisIndex = -1;
        int motionSlot = -1;
        int servoSlaveIndex = -1;

        uint16_t controlWord = 0U;
        int32_t targetVelocity = 0;
        uint16_t touchProbeFunction = 0U;
        int8_t modesOfOperation = 0;
    };


    MotionServoOutputCommandSeamAxisState
        m_MotionServoOutputCommandSeamAxisStates[MAX_AXES] = {};


    bool m_MotionServoOutputCommandSeamPrepared =
        false;

    int m_MotionServoOutputCommandSeamRouteCount =
        0;

    int m_MotionServoOutputCommandSeamFieldCount =
        0;


    std::atomic<bool>
        m_MotionServoOutputCommandSeamRuntimeStarted
    {
        false
    };


    std::atomic<uint64_t>
        m_MotionServoOutputCommandSeamCycleSamples
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputCommandSeamCycleChecks
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputCommandSeamCycleMatches
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputCommandSeamCycleMismatches
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputCommandSeamCycleFailures
    {
        0ULL
    };


    std::atomic<uint64_t>
        m_MotionServoOutputCommandSeamWriteChecks
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputCommandSeamWriteMatches
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputCommandSeamWriteFailures
    {
        0ULL
    };


    std::atomic<uint64_t>
        m_MotionServoOutputCommandSeamControlWordWrites
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputCommandSeamTargetVelocityWrites
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputCommandSeamTouchProbeWrites
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputCommandSeamModesWrites
    {
        0ULL
    };


    void ObserveMotionServoOutputCommandSeamCycleShadow(
        int axisIndex,
        uint16_t controlWord,
        int32_t targetVelocity,
        uint16_t touchProbeFunction,
        int8_t modesOfOperation);


    // =========================================================
    // Stage 11E.5 - Controlled Structured Servo Output Producer
    // =========================================================

    bool m_MotionServoOutputStructuredProducerPrepared = false;

    std::atomic<bool> m_MotionServoOutputStructuredProducerEnabled{ false };
    std::atomic<bool> m_MotionServoOutputStructuredProducerFault{ false };

    std::atomic<uint64_t> m_MotionServoOutputStructuredProducerTransitions{ 0ULL };

    std::atomic<uint64_t> m_MotionServoOutputStructuredProducerWarmupFallbacks{ 0ULL };
    std::atomic<uint64_t> m_MotionServoOutputStructuredProducerFaultFallbacks{ 0ULL };
    std::atomic<uint64_t> m_MotionServoOutputStructuredProducerRouteFaults{ 0ULL };

    std::atomic<uint64_t> m_MotionServoOutputStructuredProducerWriteAttempts{ 0ULL };
    std::atomic<uint64_t> m_MotionServoOutputStructuredProducerWrites{ 0ULL };
    std::atomic<uint64_t> m_MotionServoOutputStructuredProducerWriteFailures{ 0ULL };

    std::atomic<uint64_t> m_MotionServoOutputStructuredProducerReadbackChecks{ 0ULL };
    std::atomic<uint64_t> m_MotionServoOutputStructuredProducerReadbackMatches{ 0ULL };
    std::atomic<uint64_t> m_MotionServoOutputStructuredProducerReadbackFailures{ 0ULL };

    std::atomic<uint64_t> m_MotionServoOutputStructuredProducerControlWordWrites{ 0ULL };
    std::atomic<uint64_t> m_MotionServoOutputStructuredProducerTargetVelocityWrites{ 0ULL };
    std::atomic<uint64_t> m_MotionServoOutputStructuredProducerTouchProbeWrites{ 0ULL };
    std::atomic<uint64_t> m_MotionServoOutputStructuredProducerModesWrites{ 0ULL };

    std::atomic<uint64_t> m_MotionServoOutputStructuredProducerAxisSamples{ 0ULL };
    std::atomic<uint64_t> m_MotionServoOutputStructuredProducerCycleChecks{ 0ULL };
    std::atomic<uint64_t> m_MotionServoOutputStructuredProducerCycleMatches{ 0ULL };
    std::atomic<uint64_t> m_MotionServoOutputStructuredProducerCycleMismatches{ 0ULL };
    std::atomic<uint64_t> m_MotionServoOutputStructuredProducerCycleFailures{ 0ULL };

    std::atomic<uint64_t> m_MotionServoOutputStructuredProducerEmergencyRestores{ 0ULL };


    bool IsMotionServoOutputCommandSeamQualifiedForCutover() const;

    void ObserveMotionServoOutputStructuredProducerCycle(
        int axisIndex,
        uint16_t controlWord,
        int32_t targetVelocity,
        uint16_t touchProbeFunction,
        int8_t modesOfOperation);


    // =========================================================
    // Stage 11E.6 - Servo Output Compatibility Retirement Shadow
    // =========================================================

    bool m_MotionServoOutputCompatibilityRetirementPrepared =
        false;

    std::atomic<bool>
        m_MotionServoOutputCompatibilityRetirementActive
    {
        false
    };

    std::atomic<uint64_t>
        m_MotionServoOutputCompatibilityRetirementTransitions
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputCompatibilityRetirementAxisSamples
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputCompatibilityRetirementChecks
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputCompatibilityRetirementMatches
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputCompatibilityRetirementMismatches
    {
        0ULL
    };

    std::atomic<uint64_t>
        m_MotionServoOutputCompatibilityRetirementFailures
    {
        0ULL
    };


    uint64_t m_MotionServoOutputRetirementFrozenE1AxisSamples = 0ULL;
    uint64_t m_MotionServoOutputRetirementFrozenE2AxisSamples = 0ULL;
    uint64_t m_MotionServoOutputRetirementFrozenE3AxisSamples = 0ULL;
    uint64_t m_MotionServoOutputRetirementFrozenE4AxisSamples = 0ULL;
    uint64_t m_MotionServoOutputRetirementFrozenE4WriteChecks = 0ULL;
    uint64_t m_MotionServoOutputRetirementFrozenE5CycleSamples = 0ULL;

    uint64_t m_MotionServoOutputRetirementBaselineWarmupFallbacks = 0ULL;
    uint64_t m_MotionServoOutputRetirementBaselineFaultFallbacks = 0ULL;
    uint64_t m_MotionServoOutputRetirementBaselineRouteFaults = 0ULL;
    uint64_t m_MotionServoOutputRetirementBaselineEmergencyRestores = 0ULL;
    uint64_t m_MotionServoOutputRetirementBaselineStructuredWrites = 0ULL;
    uint64_t m_MotionServoOutputRetirementBaselineReadbackMatches = 0ULL;


    bool IsMotionServoOutputStructuredProducerQualifiedForRetirement() const;

    void ActivateMotionServoOutputCompatibilityRetirement();

    void ObserveMotionServoOutputCompatibilityRetirementCycle(
        int axisIndex,
        uint16_t controlWord,
        int32_t targetVelocity,
        uint16_t touchProbeFunction,
        int8_t modesOfOperation);


    // =========================================================
    // Stage 11E.7 - Servo Generic I/O Release Gate
    // =========================================================

    bool m_ServoGenericIoReleaseGatePrepared =
        false;

    std::atomic<bool>
        m_ServoGenericIoReleaseComplete
    {
        false
    };

    std::atomic<uint64_t>
        m_ServoGenericIoReleaseTransitions
    {
        0ULL
    };


    // Stage 11C.4 active generic LIVE Read route.
    //
    // Store descriptor indices rather than pointers so the route remains
    // stable even if the descriptor vector is relocated.
    std::vector<uint32_t>
        m_CompositeLiveReadDescriptorIndices;

    bool m_CompositeLiveReadRouteEnabled =
        false;


    // Stage 11C.6 active semantic consumer API route.
    bool m_CompositeReadConsumerLiveRouteEnabled =
        false;

    // System Function
    int BuildIoMap();   // 自動掃描並建立清單
    void Config_Slave_FMMU(int slaveIdx);//設定 FMMU
    int WriteFmmuRegister(int slaveIdx, int fmmuIdx, uint32_t logAddr, uint16_t len, uint16_t physAddr, uint8_t type, int timeout);//寫入單一 FMMU 設定



     //範例區塊-----------------------------------------------------------------------------------
    void RunRealTimeCycle_EXAMPLE_MODE();//主要程式迴圈執行_測試模式

    // System par
    CNicDriver* m_pNic;         // 網卡指標
    EtherCatEni* m_pEni;        // ENI 設定指標
    uint8_t m_idx;              // 全域封包索引 (Packet Index)
    uint8_t m_mboxCnt;          // Mailbox 計數器
    uint8_t m_txBuffer[1514];
    uint8_t m_rxBuffer[1514];
    PlcCore    m_Plc;

    char m_IoMap[4096];
    int m_IoMapSize = 0;


    int EXPECTED_WKC_PDO = 0;
    int wkc_error_count_PDO = 0;
    int timeout_count_PDO = 0;
    unsigned long long tickCount_PDO = 0;
    int wkc_PDO = 0;

    unsigned long long tickCount_RunRealTimeCycle = 0;
    unsigned long long tickCount_PLC = 0;

    uint64_t DC_reference_time = 0;
    int wk_read = 0;
    int wk_write = 0;




    AsyncCommandSlot m_asyncCmd;


    //Motion----------------------------------------------
    MotionCore m_Motion;
    std::vector<AxisContext> m_Axes;

    int test_timer = 0;
    int test_dir = 1; // 1: 正向, -1: 反向

    int debug_EDM = 1;



    int timer_10ms = 0;
    int timer_100ms = 0;
    int timer_500ms = 0;
    int timer_1000ms = 0;
    int timer_5000ms = 0;
    int timer_10000ms = 0;
    int Debug_test_timer = 0;

    int timer_10ms_Count = 0;
    int timer_100ms_Count = 0;
    int timer_500ms_Count = 0;
    int timer_1000ms_Count = 0;
    int timer_5000ms_Count = 0;
    int timer_10000ms_Count = 0;
    int Debug_test_timer_Count = 0;




    //主系統區塊-----------------------------------------------------------------------------------
    int TotalSlave_WKC_Count = 0;//從站統計WKC 分數 判斷是否失聯
    void Get_TotalSlave_WKC_Count();//取得從站WKC 分數

    int RunRealTimeCycle_EDM_SINKER_MODE();//主要程式迴圈執行 EDM 雕磨模式

    int StartDcPdoRuntime();//啟動PDO作業 DC同步
    void PrintDcRuntimeDiagnostics();//DC診斷訊息

    int StartPLCRuntime();//啟動PLC作業

    NCManager* m_NC = nullptr;

    // =========================================================
// Master <-> DC Estimator State
//
// Offset definition:
//
// CLOCK_2 - DC Reference
// =========================================================

    bool m_dcEstimatorValid = false;
    int64_t m_dcEstimatorOffsetNs = 0;
    int64_t m_dcEstimatorDriftPpb = 0;
    uint64_t m_dcEstimatorMasterTimeNs = 0;
    uint64_t m_dcEstimatorSequence = 0;

    bool m_dcHalDitherEnabled = false;
    uint32_t m_dcHalBaseCounts = 0;
    uint32_t m_dcHalAlternateCounts = 0;
    uint64_t m_dcHalDitherAccumulator = 0;
    uint64_t m_dcHalDitherStep = 0;
    uint64_t m_dcHalDitherThreshold = 1000000000ULL;
    uint64_t m_dcHalBaseCycleCount = 0;

    uint64_t m_dcHalAlternateCycleCount = 0;

    bool m_dcHalFrequencyCommandValid = false;
    int64_t m_dcHalFrequencyCommandPpb = 0;
    int64_t m_dcHalCalibrationSumPpb = 0;

    uint32_t m_dcHalCalibrationSamples = 0;
    uint32_t m_dcHalCalibrationRequiredSamples = 2;

    int64_t m_dcHalTrimSumPpb = 0;

    int64_t m_dcHalTrimMinPpb = 0;

    int64_t m_dcHalTrimMaxPpb = 0;

    uint32_t m_dcHalTrimSamples = 0;

    uint32_t m_dcHalTrimRequiredSamples = 8;

    int64_t m_dcHalTrimMaxStepPpb = 500;

    uint32_t m_dcHalTrimHoldoffRemaining = 0;

    uint32_t m_dcHalTrimHoldoffWindows = 2;

    uint64_t m_dcHalTrimUpdateCount = 0;


    // =========================================================
    // EtherCAT Master Startup State
    //
    // Priority 50 Startup thread writes.
    // HMI / Shared Memory 可透過 GetStartupSnapshot() 讀取。
    // =========================================================

    volatile LONG m_startupStage =
        (LONG)EtherCatStartupStage::Idle;

    volatile LONG m_startupResult =
        (LONG)EtherCatStartupResult::Idle;

    volatile LONG m_startupFailedStage =
        (LONG)EtherCatStartupStage::Idle;

    volatile LONG m_startupErrorCode =
        EcatStartupOk;


    void BeginStartupStage(
        EtherCatStartupStage stage);

    void PassStartupStage(
        EtherCatStartupStage stage);

    void FailStartupStage(
        EtherCatStartupStage failedStage,
        LONG errorCode,
        const char* reason);

    void MarkStartupReady();


    // =========================================================
    // EtherCAT Master Runtime Lifecycle State
    //
    // Priority 50 Main Thread writes.
    // HMI / Shared Memory 後續可由 GetRuntimeSnapshot() 讀取。
    // =========================================================

    volatile LONG m_runtimeStage =
        (LONG)EtherCatRuntimeStage::Idle;

    volatile LONG m_runtimeResult =
        (LONG)EtherCatRuntimeResult::Idle;

    volatile LONG m_runtimeFailedStage =
        (LONG)EtherCatRuntimeStage::Idle;

    volatile LONG m_runtimeErrorCode =
        EcatRuntimeOk;


    void ResetRuntimeState();

    void BeginRuntimeStage(
        EtherCatRuntimeStage stage);

    void PassRuntimeStage(
        EtherCatRuntimeStage stage);

    void FailRuntimeStage(
        EtherCatRuntimeStage failedStage,
        LONG errorCode,
        const char* reason);

    void MarkRuntimeRunning();

    void MarkRuntimeStopping();

    void MarkRuntimeStopped();


    // =========================================================
    // Runtime Hardware Cutover Gate
    //
    // Modern XML:
    //     Stage 9B Generic Runtime Contract opens this gate.
    //
    // Legacy XML:
    //     Stage5C ProductCode equivalence may open it.
    //
    // Default false = fail-safe / legacy hard-code fallback.
    // =========================================================
    bool m_runtimeEquivalenceVerified =
        false;
};

