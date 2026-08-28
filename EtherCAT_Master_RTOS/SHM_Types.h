#pragma once
#include <stdint.h>
#pragma pack(push, 1)


// API狀態區塊-----------------------------------------------------------
struct SHM_API_Status
{
    uint32_t SHM_API_RunCount;//API執行迴圈數

    int32_t reserved[16];// 預留擴充空間
};

// NC狀態區塊-----------------------------------------------------------
struct SHM_NC_Status
{
    uint32_t SHM_NC_RunCount;//NC執行迴圈數
    int32_t SHM_m_mode;//OP模式 0>>MEMORY  1>>MDI  2>>MANUAL 3>>EDIT
    int32_t SHM_NC_State;//NC狀態
    int32_t SHM_EDM_State;//EDM設備狀態


     // 🌟 1. 主程式追蹤
    char     mainProgName[64];  // 當前主程式名稱
    int32_t  mainCurrentLine;   // 主程式行號

    // 🌟 2. 副程式 (Macro) 追蹤
    char     macroProgName[64]; // 當前副程式名稱
    int32_t  macroCurrentLine;  // 副程式行號


    bool m_isSingleBlockEnabled;   // 單步執行
    bool m_isOptionalStopEnabled;  // 選擇性暫停 (M01)
    bool m_isBlockSkipEnabled;     // 選擇性跳躍 (/)

    // 🌟 新增：人機 UI 專用的模態狀態 (Modal Status)
    int currentWCS_GCode;  // 目前的座標系 (54~959)
    int isAbsoluteMode = true;    // 1 = G90 (絕對), 0 = G91 (增量)
    // 🌟 新增：給 HMI 畫面綁定刀具狀態用的變數
    int currentToolLengthMode; // 目前的刀長補正模式 (49=G49, 43=G43, 44=G44)
    int currentHCode;          // 目前的 H 碼 (0~100)

    // 🌟 新增：刀徑狀態變數
    int currentToolRadiusMode; // 40=G40(取消), 41=G41(左), 42=G42(右)
    int currentDCode;          // 目前的 D 碼 (0~100)

    // 🌟 新增：G68 旋轉狀態 (1 = 開啟, 0 = 關閉)
    int currentG68State;
    double currentG68Angle;          // 🌟 新增：目前的旋轉角度 (給 HMI 顯示用)


    int currentG168State;

    int currentWCode;            // 🌟 新增：目前的 W 碼 (給 HMI 顯示用，例如 W1, W2)
    // 🌟 新增：G51 縮放狀態與倍率
    int currentG51State;  // 1 = 開啟縮放, 0 = 關閉縮放
    double currentScaleRatio; // 目前的縮放倍率 (給 HMI 顯示用，例如 0.5)


    // 🌟 新增：鏡像狀態遮罩 (Bitmask: 第 0 bit 代表 X 軸，第 1 bit 代表 Y 軸...)
    uint8_t currentMirrorMask;

    // 🌟 新增：G16 極座標狀態 (1 = 開啟, 0 = 關閉)
    int  currentG16State;


    // 🌟 新增：G162 偏心補償狀態 (1=開啟, 0=關閉)
    int currentG162State;

    // 🌟 新增：目前插補平面 (17=G17, 18=G18, 19=G19)
    int currentPlaneMode;


    int currentG20State;

    bool  m_programmableTravelLimitEnabled;//G22 G23

    int currentTCode;//當下刀號
    int currentWorkpieceNum;//當下工件號

    // 🌟 新增：即時座標廣播區 (8軸)
    double actualMCS[8]; // 機械座標 (Machine Coordinate System)
    double actualWCS[8]; // 絕對座標/工件座標 (Work Coordinate System)
    double DistanceToGo[8];//剩餘座標

    int32_t manualFrameEnabled;
    // Rotation around Machine Z
    double manualFrameYawDeg;
    // Rotation around Machine Y
    double manualFramePitchDeg;
    // Rotation around Machine X
    double manualFrameRollDeg;



    int32_t reserved[23];// 預留擴充空間
};


//警報狀態區塊 ------------------------------------------------
struct SHM_Alarm_Status
{
    uint32_t alarmUpdateCount;     // 警報更新計數器 (HMI 監控此數值，改變代表有新警報或警報消除)
    int32_t  activeAlarmCount;     // 目前發生中的警報數量
    int32_t  activeAlarms[64];     // 發生中的警報代碼陣列 (最多支援同時 64 個警報)

    // ==========================================
    // 🌟 [請補上這行] 存軸編號的陣列，長度要跟 activeAlarms 一樣！
    // ==========================================
    int32_t activeAlarmAxes[64];

    int32_t  reserved[16];         // 預留擴充空間
};

// 坐標系參數
struct SHM_Coord_Table
{
    double extOffset[8];            // EXT (1行 x 8軸)
    double wcsTable[60][8];         // G54~G959 (60行 x 8軸)
    double toolOffset[100][8];      // 刀具補償 (100行 x 8軸)
    double workOffset[100][8];      // 工件補償 (100行 x 8軸)
};

//軸狀態區
struct SHM_AxisDebugInfo {
    double CmdPos;     // 大腦命令位置 (Pulse)
    double ActPos;     // 實際編碼器位置 (Pulse)
    double LagError;   // 跟隨誤差 (Cmd - Act)
    double CmdVel;     // 當前命令速度 (PPS)
    double ActVel;       // 🌟 [新增] 實際編碼器回授速度 (PPS)
    double MaxLagLimit;  // 🌟 [新增] 目前設定的容許最大誤差 (Pulse)

    // 🌟 [新增] 轉速與單位速度
    double ActualRPM;    // 實際馬達轉速 (RPM)
    double ActualMpm;    // 實際速度 (m/min)


    int    State;      // 運動狀態 (IDLE=0, MOVING=1...)
    bool    IsServoOn;   // 是否激磁
    bool    IsFault;     // 是否報警
    uint8_t IsLagAlarm;  // 追隨誤差過大
    uint8_t IsHomed;     // 0=尚未尋原點，1=已完成 Machine Home

    // ========================================================
    // Drive Touch Probe Raw Monitor
    //
    // 由 MotionCore::ExportDebugInfo() 填入：
    //
    // TouchProbeFunction = RxPDO 0x60B8
    // TouchProbeStatus   = TxPDO 0x60B9
    // TouchProbePosition = TxPDO 0x60BA
    // ========================================================

    uint16_t TouchProbeFunction;
    uint16_t TouchProbeStatus;
    int32_t  TouchProbePosition;
};

static_assert(
    sizeof(SHM_AxisDebugInfo) == 80,
    "SHM_AxisDebugInfo must stay 80 bytes (Pack=1).");

//NC命令區塊-----------------------------------------------------------

struct SHM_NC_Command
{
    bool Close_System;//關閉核心系統命令

    bool cycleStart;//執行NC
    bool feedHold;//暫停NC
    bool reset;//重置NC

    // 程式載入控制
    bool loadProgramReq;//載入NC檔案命令
    char loadprogramName[256];// 載入NC檔案名稱


    // OP 模式切換保留在這裡
    bool reqChangeMode;
    int32_t targetMode; // 0: MEMORY, 1: MDI, 2: MANUAL, 3: EDIT



    bool Set_isSingleBlockEnabled_ON;   // 單步執行
    bool Set_isSingleBlockEnabled_OFF;   // 單步執行
    bool Set_isOptionalStopEnabled_ON;  // 選擇性暫停 (M01)
    bool Set_isOptionalStopEnabled_OFF;  // 選擇性暫停 (M01)
    bool Set_isBlockSkipEnabled_ON;     // 選擇性跳躍 (/)
    bool Set_isBlockSkipEnabled_OFF;     // 選擇性跳躍 (/)

    int32_t reserved[16];// 預留擴充空間
};

// 🌟 2. 新增：專門用來傳遞 MDI/MANUAL 程式碼的獨立區塊
struct SHM_String_Command {
    bool reqLoadCode;       // 請求載入程式碼旗標
    char codeContent[2048]; // 2KB 字串緩衝區
};



//寫入Macro變數專用的指令區塊
struct SHM_Var_Write_Command
{
    bool writeReq;            // 寫入請求旗標
    char prefix;              // 變數種類 ('#', '@', '$')
    int index;                // 變數編號
    double writeValue;        // 要寫入的值
    bool refresh_local;//更新區域變數
    bool refresh_global;//更新全域變數
    bool refresh_system;//更新系統變數
};

//巨集變數狀態全廣播區塊 (大約 18.4 KB)
struct SHM_Macro_Status {
    int currentCallDepth;                   // 目前所在的副程式層數 (0~7)
    double localVars[8][101];               // 8層，每層 #1~#100 (index 0 不用)
    double globalVars[1001];             // @1~@1000 (嚴格限制)
    double sysVars[1001];                   // $1~$1000
};

struct SHM_Coord_Command {
    // 切換座標系命令
    bool reqSwitchWCS;   // 觸發旗標 (true=執行, false=待機)
    int targetWCS_GCode; // 目標 G 碼 (如 54, 154)

    // 參數修改命令
    bool reqWriteOffset; // 觸發旗標 (true=執行, false=待機)
    char offsetType;     // 0=EXT, 1=WCS, 2=Tool, 3=Work (C++ 建議用 char 或 uint8_t 對應 byte)
    int rowIndex;        // 陣列行號 (例如 G54 就是 0)
    int axisIndex;       // 軸號 (0=X, 1=Y, 2=Z...)
    double writeValue;   // 欲寫入的數值

    // 儲存檔案命令
    bool reqSave;        // 觸發旗標 (true=執行, false=待機)
    char saveType;       // 0=全部, 1=Status, 2=EXT, 3=WCS, 4=Tool, 5=Work

    bool reqSetManualFrame;

    bool manualFrameEnabled;

    double manualFrameYawDeg;
    double manualFramePitchDeg;
    double manualFrameRollDeg;
};

// PLC狀態區塊 (全廣播，供 HMI 讀取顯示) --------------------------------
struct SHM_PLC_Status
{
    uint32_t SHM_PLC_RunCount;//PLC執行迴圈數
    // 標準點位與暫存器
    uint8_t I[1000];
    uint8_t O[1000];
    uint8_t A[4000];
    uint8_t S[4000];
    uint8_t C[4000];
    int32_t R[1000];
    int32_t DR[1000];

    // 🌟 Timer (T) 專屬區塊：拆分當前值與目標值，讓人機可以顯示進度 (例如 5/10)
    int32_t T_acc[1000];    // 目前累積時間 (ms)
    int32_t T_preset[1000]; // 目標設定時間 (ms)
    uint8_t T_done[1000];   // 是否已經到達 (1=ON, 0=OFF)
    int32_t T_base[1000];   // 🌟 新增這行：把時基傳給 HMI

     // =========================================================
    // Counter (CNT0 ~ CNT999) 狀態廣播
    // =========================================================
    int32_t CNT_acc[1000];       // 目前計數值
    int32_t CNT_preset[1000];    // 目標計數值
    uint8_t CNT_done[1000];      // 1 = Done, 0 = Not Done
};

// PLC命令區塊 (供 HMI 寫入/設定點位) -----------------------------------
struct SHM_PLC_Command
{
    // 方式 A：透過區域代號與索引寫入 (例如寫入 M_O[5])
    bool writeReq;          // 寫入請求旗標 (HMI 設 true，C++ 執行完設 false)
    bool ReloadLogicProgram;//重置PLC邏輯檔案
    char regionPrefix[4];   // 變數前綴 (填入 "I", "O", "A", "S", "R", "DR")
    int index;              // 陣列索引 (如 5)
    double writeValue;      // 欲寫入的值 (如果是 BOOL，請傳 1.0 或 0.0)

    // 方式 B：透過字串名稱寫入 (如果 HMI 想要直接用變數名稱寫入 VAR)
    bool writeByNameReq;    // 名稱寫入請求旗標
    char varName[64];       // 變數名稱字串 (例如 "TARGET_SPEED")
};

// PLC Runtime Diagnostics (V7.5.3, read-only broadcast) -----------------------
struct SHM_PLC_Diagnostics
{
    uint8_t RuntimeFault;              // 0 = healthy, 1 = sticky runtime fault
    uint8_t LastFaultCode;             // PLCRuntimeFaultCode
    uint8_t LastOpcode;                // faulting VM opcode
    uint8_t LastOperandRegion;         // PLC operand region, 0xFF = N/A

    int32_t LastTaskIndex;             // runtime sorted task index
    int32_t LastInstructionIndex;      // instruction index inside task
    int32_t LastOperandAddress;        // address/index related to last fault

    uint32_t LastFaultRunCount;         // PLC_RunCount when last fault occurred
    uint32_t TotalFaultCount;

    uint32_t InvalidOperandCount;
    uint32_t InvalidTimerIndexCount;
    uint32_t InvalidCounterIndexCount;
    uint32_t UnknownOpcodeCount;
    uint32_t RuntimeStateMismatchCount;
    uint32_t InvalidFlowRowCount;

    // V7.6.0 - actual accepted logic.bin identity.
    uint32_t LoadedLogicCrc32;
    uint32_t LoadedLogicSize;
    uint32_t LogicLoadGeneration;
    uint32_t LastLogicLoadResult; // 0=None, 1=Success, 2=Failed

    // V7.6.4.1 Runtime Scan Health (read-only)
    uint32_t ScanLastCycleUs;
    uint32_t ScanWorstCycleUs;
    uint32_t ScanCycleBudgetUs;
    uint32_t ScanCycleOverrunCount;

    uint32_t ScanLastTaskUs;
    uint32_t ScanWorstTaskUs;
    uint32_t ScanLastTaskBudgetUs;
    int32_t ScanLastTaskIndex;
    int32_t ScanWorstTaskIndex;
    uint32_t ScanTaskOverrunCount;

    uint32_t ScanMeasuredCycleCount;
    uint32_t ScanLastOverrunRunCount;
    int32_t ScanLastOverrunTaskIndex;
};
static_assert(sizeof(SHM_PLC_Diagnostics) == 116, "SHM_PLC_Diagnostics must stay 64 bytes (Pack=1).");

//總記憶體區塊-------------------------------------------------------
struct SHM_Data
{
    SHM_API_Status API_Status;//API狀態區塊
    SHM_NC_Status  NC_Status;//NC狀態區塊
    SHM_Alarm_Status Alarm_Status;//警報狀態區塊
    SHM_NC_Command NC_Command;// 命令區塊
    SHM_Var_Write_Command varCmd;  // 新增：寫入變數指令
    SHM_Macro_Status macroStatus;  // 新增：變數全廣播區
    SHM_Coord_Command Coord_Command;//
    SHM_Coord_Table Coord_Table;  // 🌟 新增這行：將表格加入總結構！

    SHM_String_Command String_Command; // 加入這個新區塊
    SHM_AxisDebugInfo axisDebug[8]; // 🌟 給 HMI 看的 8 軸除錯資訊

    // 🌟 補上這兩行：PLC 專用讀寫區
    SHM_PLC_Status PLC_Status;
    SHM_PLC_Command PLC_Command;

    SHM_PLC_Diagnostics PLC_Diagnostics;
};


// ============================================================================
// Stage 12A.1 - OSCARMAX EtherCAT Online Diagnosis Shared Memory Contract
// ============================================================================
//
// 目的：
//     提供 ENI Tool / Commissioning Tool 一條「獨立於 EDM HMI」的唯讀診斷通道。
//
// Shared Memory 名稱（Stage 12A.2 建立）：
//     OSCARMAX_ECAT_DIAG
//
// 設計原則：
//   1. 不加入 SHM_Data，避免改變既有 EDM_SINKER_MODE Shared Memory ABI。
//   2. 這個區塊只做 Master -> Windows 診斷廣播；Stage 12A 不接受控制命令。
//   3. 全部使用固定寬度型別，不使用 bool / pointer / STL，方便 C# 精確對齊。
//   4. 使用 Sequence 做簡單 seqlock：
//        Producer 寫入前 Sequence++（變奇數）
//        Producer 完成後 Sequence++（變偶數）
//      Reader 只有在前後 Sequence 相同且為偶數時才採用 snapshot。
//   5. ProcessImage 為 Master 當下完整 Logical Process Image snapshot，
//      ENI Tool 依 Runtime XML / Composite Binding 自己解碼 Semantic Value。
//
// 注意：
//     這裡只定義 ABI；Stage 12A.1 不建立 Shared Memory、不改 Runtime 行為。
// ============================================================================

static constexpr uint32_t SHM_ECAT_DIAG_MAGIC = 0x54414345u; // little-endian "ECAT"
static constexpr uint16_t SHM_ECAT_DIAG_VERSION_MAJOR = 1;
static constexpr uint16_t SHM_ECAT_DIAG_VERSION_MINOR = 0;
static constexpr uint32_t SHM_ECAT_DIAG_MAX_SLAVES = 128;
static constexpr uint32_t SHM_ECAT_DIAG_PROCESS_IMAGE_CAPACITY = 4096;

// Master / Runtime 即時健康狀態。
struct SHM_ECAT_DiagMasterStatus
{
    uint32_t Magic;               // SHM_ECAT_DIAG_MAGIC
    uint16_t VersionMajor;        // ABI major
    uint16_t VersionMinor;        // ABI minor
    uint32_t StructSize;          // sizeof(SHM_ECAT_DiagData)

    uint32_t Sequence;            // seqlock sequence，偶數=穩定 snapshot
    uint32_t Heartbeat;           // 每次 publish +1
    uint64_t PublishCount;        // 64-bit 累積 publish 次數

    uint32_t CycleTimeNs;         // EtherCAT cycle，例如 250000 ns
    uint32_t ProcessImageBytes;   // 當下有效 Process Image bytes
    uint32_t ProcessImageCapacity;// 固定 = 4096

    uint16_t SlaveCount;          // 實際在線 / Runtime slave count
    uint16_t MasterState;         // EtherCAT AL state，例如 0x0008 = OP
    uint16_t RuntimeStage;        // EtherCatRuntimeStage（原始 enum 數值）
    uint16_t RuntimeErrorCode;    // Runtime lifecycle error code

    int32_t ExpectedWkc;          // Runtime XML / topology 計算值
    int32_t ActualLrwWkc;         // 最近一次 LRW WKC
    int32_t DcWkc;                // 最近一次 DC datagram WKC

    uint32_t RxRecentTimeout;     // Recent timeout window
    uint32_t RxTotalHardTimeout;  // 累積 hard timeout
    uint32_t RxConsecutiveTimeout;// current consecutive timeout
    uint32_t RxRecoveryCount;     // recovery after timeout
    uint32_t RxSkipCount;         // skipped cycle / receive
    uint32_t RxSoftLateCount;     // soft late count
    uint32_t QpcFailCount;        // QPC / timer failure count

    int64_t DcPhaseErrorNs;       // 最近一次 phase error
    int32_t DcDriftState;         // DC drift controller state
    int32_t DcRealFfState;        // Real FF state
    int32_t DcPhasePState;        // Phase-P state

    uint16_t DcReferenceSlave;    // 0-based slave index，0xFFFF = none
    uint16_t DcReferenceConfigAddr;

    uint8_t MasterReady;          // startup READY
    uint8_t RuntimeRunning;       // runtime lifecycle RUNNING
    uint8_t ProcessImageValid;    // Process Image snapshot valid
    uint8_t LrwHealthy;           // ActualLrwWkc == ExpectedWkc
    uint8_t RxHealthy;            // RX gate healthy
    uint8_t DcHealthy;            // DC gate healthy
    uint8_t DcPhaseGood;          // phase good
    uint8_t DcGateOpen;           // DC gate open
    uint8_t DcOffsetSaturated;    // offset saturation
    uint8_t DcTripMask;           // existing DC trip mask
    uint8_t UnifiedRcReady;       // Stage 11F.1 Ready
    uint8_t UnifiedRcReleased;    // Stage 11F.1 Released

    uint8_t Reserved8[20];
    uint32_t Reserved32[16];
};

// 每一顆 EtherCAT Slave 的 Online snapshot。
struct SHM_ECAT_DiagSlaveStatus
{
    uint16_t Position;            // 0-based topology position
    uint16_t ConfiguredAddress;   // station / configured address

    uint32_t VendorId;
    uint32_t ProductCode;
    uint32_t Revision;

    uint16_t AlState;             // INIT/PREOP/SAFEOP/OP
    uint16_t AlStatusCode;        // ESC AL Status Code，0 = no error

    int32_t OutputOffset;         // Logical Process Image byte offset，-1 = none
    uint16_t OutputBytes;
    int32_t InputOffset;          // Logical Process Image byte offset，-1 = none
    uint16_t InputBytes;

    uint16_t SmCount;
    uint16_t FmmuCount;

    uint8_t HasMailbox;
    uint8_t DcEnabled;
    uint8_t IsDcReference;
    uint8_t LinkUp;

    uint32_t RxErrorCount;        // Stage 12E 可填 ESC error counters
    uint32_t LostLinkCount;       // Stage 12E 可填 ESC lost-link counters

    // Runtime XML / topology 顯示名稱。
    // UTF-8 / ASCII；未使用區域必須為 0。
    char Name[64];

    uint8_t Reserved[16];
};

// 獨立 Shared Memory 的完整資料區。
struct SHM_ECAT_DiagData
{
    SHM_ECAT_DiagMasterStatus Master;

    SHM_ECAT_DiagSlaveStatus Slaves[SHM_ECAT_DIAG_MAX_SLAVES];

    // 完整 Logical Process Image。
    // ENI Tool 不依賴 C++ 特定 Device struct，直接依 Runtime XML Descriptor 解碼。
    uint8_t ProcessImage[SHM_ECAT_DIAG_PROCESS_IMAGE_CAPACITY];
};

// ABI 安全檢查。
//
// Visual Studio IntelliSense 有時會在 #pragma pack(push, 1) 的大型 ABI
// 結構上錯誤計算 sizeof，產生 E1574 假錯誤。
// __INTELLISENSE__ 只在 IntelliSense parser 中定義；真正 MSVC Build
// 仍會執行以下 static_assert，因此 ABI 保護完全保留。
#if !defined(__INTELLISENSE__)
static_assert(
    sizeof(SHM_ECAT_DiagMasterStatus) == 208,
    "SHM_ECAT_DiagMasterStatus ABI changed; update ENI Tool C# contract.");

static_assert(
    sizeof(SHM_ECAT_DiagSlaveStatus) == 128,
    "SHM_ECAT_DiagSlaveStatus ABI changed; update ENI Tool C# contract.");

static_assert(
    sizeof(SHM_ECAT_DiagData) == 20688,
    "SHM_ECAT_DiagData ABI changed; update ENI Tool C# contract.");
#endif

static_assert(
    SHM_ECAT_DIAG_PROCESS_IMAGE_CAPACITY >= 1514,
    "Diagnostic Process Image capacity must cover at least one full EtherCAT frame budget.");


// ============================================================================
// Stage 12F.3B1 - OSCARMAX EtherCAT Service Shared Memory Contract
// ============================================================================
//
// Shared Memory name (created in Stage 12F.3B2):
//     OSCARMAX_ECAT_SERVICE
//
// Purpose:
//     Windows ENI Tool <-> RTX64 Master request / response channel for
//     non-cyclic EtherCAT engineering services such as CoE / SDO.
//
// Design rules:
//   1. This contract is NOT part of SHM_Data and therefore does not change the
//      existing EDM_SINKER_MODE ABI.
//   2. It is also separate from OSCARMAX_ECAT_DIAG, which remains read-only.
//   3. Windows never sends EtherCAT frames directly. The RTX64 service bridge
//      validates the request and forwards it to the existing PDO-owner async
//      command slot.
//   4. One outstanding command at a time. This is intentional for engineering
//      / commissioning use and keeps mailbox ownership deterministic.
//   5. Request publication protocol:
//        - Client writes all request fields first.
//        - Client publishes RequestId LAST.
//        - Server processes a RequestId different from LastProcessedRequestId.
//      Response publication protocol:
//        - Server writes all response fields first.
//        - Server publishes ResponseId LAST.
//        - Client accepts a response only when ResponseId == RequestId.
//   6. No bool / pointer / STL types. Pack=1 and fixed-width integers only.
//   7. Stage 12F.3B starts with SDO READ. SDO WRITE is reserved in this same
//      ABI so Safe Write can be added later without another shared-memory ABI
//      change.
//
// IMPORTANT:
//     SlavePosition is ZERO-BASED and matches EtherCatMaster::ecx_SDOread /
//     ecx_SDOwrite slave_pos semantics (S4 in an 8-slave topology => 4).
// ============================================================================

static constexpr uint32_t SHM_ECAT_SERVICE_MAGIC = 0x53414345u; // "ECAS"
static constexpr uint16_t SHM_ECAT_SERVICE_VERSION_MAJOR = 1;
static constexpr uint16_t SHM_ECAT_SERVICE_VERSION_MINOR = 1;

// Operation values shared with the C# client.
static constexpr uint32_t SHM_ECAT_SERVICE_OP_NONE = 0u;
static constexpr uint32_t SHM_ECAT_SERVICE_OP_SDO_READ = 1u;
static constexpr uint32_t SHM_ECAT_SERVICE_OP_SDO_WRITE = 2u;

// Stage 12F.3D.1 - Safe SDO Write authorization contract.
//
// SDO WRITE is accepted by the RTX64 bridge only when BOTH conditions are met:
//   1) Flags contains SHM_ECAT_SERVICE_FLAG_EXPERT_WRITE
//   2) RequestReserved[0] equals SHM_ECAT_SERVICE_WRITE_CONFIRM_MAGIC
//
// This is intentionally separate from the Windows UI unlock timer.  The UI
// still performs the human-facing Expert Write unlock / confirmation, while
// this token prevents an accidental or stale request from becoming a mailbox
// write merely because Operation happens to contain SDO_WRITE.
//
// The ABI size does not change.  Existing v1.0 reserved fields are promoted
// to defined v1.1 semantics.
static constexpr uint32_t SHM_ECAT_SERVICE_FLAG_EXPERT_WRITE = 0x00000001u;
static constexpr uint32_t SHM_ECAT_SERVICE_ALLOWED_FLAGS =
SHM_ECAT_SERVICE_FLAG_EXPERT_WRITE;

// ASCII-ish guard token: "SDOW" in little-endian memory representation.
static constexpr uint32_t SHM_ECAT_SERVICE_WRITE_CONFIRM_MAGIC = 0x574F4453u;

// ResultCode values. WKC is reported separately.
static constexpr int32_t SHM_ECAT_SERVICE_RESULT_NONE = 0;
static constexpr int32_t SHM_ECAT_SERVICE_RESULT_SUCCESS = 1;
static constexpr int32_t SHM_ECAT_SERVICE_RESULT_INVALID_REQUEST = -1;
static constexpr int32_t SHM_ECAT_SERVICE_RESULT_BUSY = -2;
static constexpr int32_t SHM_ECAT_SERVICE_RESULT_TIMEOUT = -3;
static constexpr int32_t SHM_ECAT_SERVICE_RESULT_WKC_ERROR = -4;
static constexpr int32_t SHM_ECAT_SERVICE_RESULT_UNSUPPORTED = -5;
static constexpr int32_t SHM_ECAT_SERVICE_RESULT_NO_MAILBOX = -6;
static constexpr int32_t SHM_ECAT_SERVICE_RESULT_RUNTIME_NOT_READY = -7;

// Service-server state exposed to ENI Tool.
static constexpr uint32_t SHM_ECAT_SERVICE_SERVER_OFFLINE = 0u;
static constexpr uint32_t SHM_ECAT_SERVICE_SERVER_READY = 1u;
static constexpr uint32_t SHM_ECAT_SERVICE_SERVER_PROCESSING = 2u;
static constexpr uint32_t SHM_ECAT_SERVICE_SERVER_FAULT = 3u;

struct SHM_ECAT_ServiceData
{
    // ---------------------------------------------------------------------
    // Server-owned header / statistics (48 B)
    // ---------------------------------------------------------------------
    uint32_t Magic;                  // SHM_ECAT_SERVICE_MAGIC
    uint16_t VersionMajor;
    uint16_t VersionMinor;
    uint32_t StructSize;             // sizeof(SHM_ECAT_ServiceData)

    uint32_t ServerHeartbeat;        // supervisory service loop heartbeat
    uint32_t ServerState;            // SHM_ECAT_SERVICE_SERVER_*
    uint32_t LastProcessedRequestId; // last request consumed by RTX64

    uint32_t SdoReadCount;           // completed SDO read requests
    uint32_t SdoWriteCount;          // future Safe Write requests
    uint32_t ErrorCount;             // rejected / failed service requests

    uint32_t HeaderReserved[3];

    // ---------------------------------------------------------------------
    // Client-owned request (48 B)
    // ---------------------------------------------------------------------
    // RequestId MUST be published last by the Windows client.
    uint32_t RequestId;
    uint32_t Operation;              // SHM_ECAT_SERVICE_OP_*

    uint16_t SlavePosition;          // zero-based topology position
    uint16_t Index;                  // CoE object index, e.g. 0x6041
    uint8_t  SubIndex;               // CoE object sub-index
    uint8_t  CompleteAccess;         // 0 = normal; non-zero reserved/future
    uint8_t  DataSize;               // requested scalar size: 1 / 2 / 4 B
    uint8_t  RequestReserved8;

    uint32_t DataValue;              // write payload; ignored by SDO READ
    uint32_t TimeoutMs;              // requested service timeout (server clamps)
    uint32_t Flags;                  // SHM_ECAT_SERVICE_FLAG_*
    uint32_t RequestReserved[5];     // [0] = WRITE_CONFIRM_MAGIC for SDO WRITE

    // ---------------------------------------------------------------------
    // Server-owned response (32 B)
    // ---------------------------------------------------------------------
    // ResponseId MUST be published last by the RTX64 server.
    uint32_t ResponseId;
    int32_t  ResultCode;             // SHM_ECAT_SERVICE_RESULT_*
    int32_t  ResultWkc;              // mailbox operation WKC / result
    uint32_t ResponseDataValue;      // little-endian scalar, up to 4 B
    uint32_t ResponseDataSize;       // actual returned data bytes: 1 / 2 / 4
    uint32_t AbortCode;              // 0 until lower SDO layer exposes abort code
    uint32_t DurationUs;             // end-to-end RTX64 service duration
    uint32_t ResponseReserved;
};

#if !defined(__INTELLISENSE__)
static_assert(
    sizeof(SHM_ECAT_ServiceData) == 128,
    "SHM_ECAT_ServiceData ABI changed; update ENI Tool C# service contract.");
#endif


#pragma pack(pop)