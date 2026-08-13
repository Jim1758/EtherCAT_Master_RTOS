#pragma once
#include <vector>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <algorithm> // 包含演算法標頭檔

// V7.5.7.1:
// Windows.h defines min/max macros unless NOMINMAX is set.
// Those macros break std::min/std::max and std::numeric_limits<T>::min/max.
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h> // 🌟 引入 Windows 標頭檔以支援 CRITICAL_SECTION
// ==========================================
// Define PLC Capacity
// ==========================================
constexpr int MAX_PLC_I = 1000;
constexpr int MAX_PLC_O = 1000;
constexpr int MAX_PLC_A = 4000;
constexpr int MAX_PLC_S = 4000;
constexpr int MAX_PLC_C = 4000;
constexpr int MAX_PLC_T = 1000;
constexpr int MAX_PLC_CNT = 1000; // V7.4.6 dedicated Counter Device: CNT0..CNT999
constexpr int MAX_PLC_FLOW_ROWS = 4096; // runtime-only Ladder flow rows
constexpr int MAX_PLC_TASKS = 256;        // V7.5.7 load-time safety bound
constexpr int MAX_PLC_INSTRUCTIONS_PER_TASK = 100000; // V7.5.7 corrupt-file guard
constexpr int MAX_PLC_VAR_ALLOCATIONS = 100000;       // V7.5.7 corrupt-file guard
constexpr int MAX_PLC_R = 1000;
constexpr int MAX_PLC_DR = 1000;
constexpr int MAX_PLC_F = 1000;  // REAL (Float)
constexpr int MAX_PLC_L = 1000;  // LREAL (Double)

// ==========================================
// Timer Structure
// ==========================================
struct PLCTimer {
    bool enable;      // Timer engine active / timing
    bool done;        // DN / Q
    int32_t preset;   // Target time (ms)
    int32_t acc;      // Accumulated time (ms)
    int32_t timeBase; // Timer 時基 (1, 10, 100, 1000 ms)

    // V7.4.5 runtime-only timer state. SHM / logic.bin layout is unchanged.
    uint8_t mode;     // 0=TON(existing TMR), 1=TOF, 2=TP, 3=RTO
    bool input;       // latest ACC input for this timer instruction
    bool prevInput;   // previous scan input, used by TP/TOF edge detection
};

// ==========================================
// Counter Structure - V7.4.6
// ==========================================
struct PLCCounter {
    int32_t preset;   // Counter preset / threshold
    int32_t acc;      // Current accumulated count
    bool done;        // Upper done / legacy CTU-CTD done bit
    bool zero;        // V7.5 runtime-only lower done (QD foundation for future CTUD)
    bool initialized; // Runtime-only initialization state
    uint8_t mode;     // 0=unused/reset, 1=CTU, 2=CTD, 3=CTUD
};

// ==========================================
// 🌟 補上這一行：前置宣告 SHM_PLC_Status
// ==========================================
struct SHM_PLC_Status;
// ==========================================
// Custom Variable (VAR) Node
// ==========================================
struct PLCCustomVar {
    std::string varName; // 🌟 可以在這裡多存名字方便 Debug
    uint8_t dataType; // 1=BOOL, 2=INT, 3=DINT, 4=REAL, 5=LREAL
    union {
        bool    bVal;
        int32_t iVal;
        float   fVal;
        double  dVal;
        uint8_t raw[8];
    } value;
};

// ==========================================
// Virtual Machine: Binary Protocol (Strict 1-Byte Alignment)
// ==========================================
#pragma pack(push, 1)

// Operand Structure (10 Bytes)
struct PLCOperand {
    uint8_t region;   // 0=I,1=O,2=C,3=S,4=A,5=T,6=R,7=DR,8=#,9=F,10=L,11=VAR,12=CNT
    uint8_t dataType; // 1=BOOL, 2=INT, 3=DINT, 4=REAL, 5=LREAL
    union {
        int32_t address;
        int32_t intValue;
        float   realValue;
        double  lrealValue;
        uint8_t raw[8];
    } payload;
};

// Instruction Structure (21 Bytes)
struct PLCInstruction {
    uint8_t opCode;   // e.g., 1=LD, 20=ADD, 98..103=V7.6.1 System Contacts
    PLCOperand op1;   // Variable 1
    PLCOperand op2;   // Variable 2
};

#pragma pack(pop)

// V7.5.0 Binary ABI guards.
// Any accidental compiler packing/layout change must fail at compile time
// instead of silently corrupting logic.bin.
static_assert(sizeof(PLCOperand) == 10, "PLCOperand binary layout changed; expected 10 bytes.");
static_assert(sizeof(PLCInstruction) == 21, "PLCInstruction binary layout changed; expected 21 bytes.");

// ==========================================
// RTOS Task Scheduling Structure
// ==========================================
struct PLCTask {
    uint8_t type;         // 1=Cyclic, 2=Event, 3=Background, 4=Init
    uint8_t priority;     // 0~31 (0 is highest)
    int32_t cycleTimeMs;
    int32_t currentTimerMs; // Scheduler Timer
    std::vector<PLCInstruction> instructions;

    // Runtime-only per-instruction previous-scan state.
    // Used by OUT_UP / OUT_DOWN, CTU / CTD, and R_TRIG / F_TRIG.
    // MUST be allocated by Load/Reload before the real-time scan begins.
    // Never resize/assign this vector from ExecuteTask().
    // 不寫入 logic.bin，因此不改變既有 Binary Protocol。
    std::vector<uint8_t> edgeMemory;

    // True Ladder Logic Core: runtime-only power state for Ladder rows.
    // Preallocated during Load/Reload. No heap allocation is permitted in ExecuteTask().
    // Not serialized into logic.bin; hidden FLOW opcodes initialize/update it every scan.
    std::vector<uint8_t> flowRows;

    // V7.6.1 System Device: TRUE for exactly the first execution of THIS task
    // after a successful Load / Reload. Runtime-only; not serialized.
    bool firstScanPending = true;
};


// ==========================================
// V7.5.2 Runtime Diagnostics (runtime-only)
// ==========================================
enum class PLCRuntimeFaultCode : uint8_t {
    None = 0,
    InvalidOperand = 1,
    InvalidTimerIndex = 2,
    InvalidCounterIndex = 3,
    UnknownOpcode = 4,
    RuntimeStateMismatch = 5,
    InvalidFlowRow = 6,

    // V7.5.5 ~ V7.5.7 extended fault codes.
    // SHM diagnostic layout is unchanged: LastFaultCode is still one byte.
    ArithmeticDomain = 7,
    IntegerOverflow = 8,
    InvalidBitIndex = 9,
    InvalidTaskConfig = 10
};

struct PLCRuntimeDiagnostics {
    bool faultActive = false;
    uint8_t lastFaultCode = 0;
    uint8_t lastOpcode = 0;
    uint8_t lastOperandRegion = 0xFF;

    int32_t lastTaskIndex = -1;
    int32_t lastInstructionIndex = -1;
    int32_t lastOperandAddress = 0;

    uint32_t lastFaultRunCount = 0;
    uint32_t totalFaultCount = 0;
    uint32_t invalidOperandCount = 0;
    uint32_t invalidTimerIndexCount = 0;
    uint32_t invalidCounterIndexCount = 0;
    uint32_t unknownOpcodeCount = 0;
    uint32_t runtimeStateMismatchCount = 0;
    uint32_t invalidFlowRowCount = 0;
};



// ==========================================
// V7.6.3 Runtime Scan Health (runtime-only)
// ==========================================
struct PLCScanHealth {
    uint32_t lastCycleUs = 0;
    uint32_t worstCycleUs = 0;
    uint32_t cycleBudgetUs = 0;
    uint32_t cycleOverrunCount = 0;

    uint32_t lastTaskUs = 0;
    uint32_t worstTaskUs = 0;
    uint32_t lastTaskBudgetUs = 0;
    int32_t lastTaskIndex = -1;
    int32_t worstTaskIndex = -1;
    uint32_t taskOverrunCount = 0;

    uint32_t measuredCycleCount = 0;
    uint32_t lastOverrunRunCount = 0;
    int32_t lastOverrunTaskIndex = -1;
};

// ==========================================
// V7.6.0 Loaded Logic Verification
// ==========================================
enum class PLCLogicLoadResult : uint32_t {
    None = 0,
    Success = 1,
    Failed = 2
};

struct PLCLogicVerificationState {
    uint32_t loadedLogicCrc32 = 0;
    uint32_t loadedLogicSize = 0;
    uint32_t logicLoadGeneration = 0;
    uint32_t lastLogicLoadResult = static_cast<uint32_t>(PLCLogicLoadResult::None);
};

// ==========================================
// PLC Manager Core
// ==========================================
class PLCManager
{
public:
    PLCManager();
    ~PLCManager();

    uint32_t PLC_RunCount = 0;
    int Init_flag = 0;
    int Close_flag = 0;

    void Init();
    void Close_PLC();
    void RunCycle(int delta_ms);
    bool LoadLogicProgram(const std::string& filepath);

    // --- 1. 標準硬體點位 Direct API (高效能，供底層 RTOS 迴圈使用) ---
    void    SetBit_I(int index, bool value);
    bool    GetBit_I(int index) const;

    void    SetBit_O(int index, bool value);
    bool    GetBit_O(int index) const;

    int32_t GetReg_R(int index) const;
    void    SetReg_R(int index, int32_t value);

    int32_t GetReg_DR(int index) const;
    void    SetReg_DR(int index, int32_t value);

    // --- 2. 🌟 全方位萬用 Get / Set API (支援透過「名稱字串」或「地址」存取所有變數) ---
    // 支援名稱字串存取 (例如: GetVar("SPEED"), SetVar("MY_FLAG", true))
    double  GetVar(const std::string& name) const;
    void    SetVar(const std::string& name, double value);

    // 支援區域與地址存取 (例如: GetMemory("R", 10), SetMemory("O", 1, true))
    double  GetMemory(const std::string& regionPrefix, int index) const;
    void    SetMemory(const std::string& regionPrefix, int index, double value);
    // 🌟 補上這一行宣告
    int32_t GetStableHashCpp(const std::string& str) const;

    // --- 🌟 A 點與 S 點專屬高速度 API ---
    void    Set_A(int index, bool value);
    bool    Get_A(int index) const;

    // C Point
    void    Set_C(int index, bool value);
    bool    Get_C(int index) const;

    void    Set_S(int index, bool value);
    bool    Get_S(int index) const;

    void ExportPLCStatus(SHM_PLC_Status* pStatus) const;
    // 🌟 新增：DR 斷電保持暫存器的二進位讀取與儲存 API
    bool SaveDRValues(const std::string& filepath);
    bool LoadDRValues(const std::string& filepath);

    // 🌟 新增：動態重載 PLC 邏輯程式的 API
    bool ReloadLogicProgram();

    // V7.5.2 Runtime Diagnostics API.
    PLCRuntimeDiagnostics GetRuntimeDiagnostics() const;
    void ClearRuntimeDiagnostics();

    // V7.6.0 Loaded Logic Verification.
    // Read-only snapshot for HMI / Studio comparison.
    PLCLogicVerificationState GetLogicVerificationState() const;

    // V7.6.3 Runtime Scan Health.
    // Read-only; no SHM dependency inside PLCManager.
    PLCScanHealth GetScanHealth() const;

private:

    // 🌟 移除 std::mutex，改用 RTX64 支援的臨界區段
    mutable CRITICAL_SECTION m_logicCS;

    // Memory Maps
    uint8_t m_I[MAX_PLC_I];
    uint8_t m_O[MAX_PLC_O];
    uint8_t m_A[MAX_PLC_A];
    uint8_t m_S[MAX_PLC_S];
    uint8_t m_C[MAX_PLC_C];

    int32_t m_R[MAX_PLC_R];
    int32_t m_DR[MAX_PLC_DR];
    float   m_F[MAX_PLC_F];
    double  m_L[MAX_PLC_L];

    PLCTimer   m_T[MAX_PLC_T];
    PLCCounter m_CNT[MAX_PLC_CNT];

    // Static Allocation Table for VARs
    std::unordered_map<int32_t, PLCCustomVar> m_customVars;

    // Task List
    std::vector<PLCTask> m_tasks;

    // Internal VM Memory Access API
    double  GetOperandAsDouble(const PLCOperand& op);
    int32_t GetOperandAsInt(const PLCOperand& op);
    bool    GetOperandAsBool(const PLCOperand& op);

    void    SetOperandFromDouble(const PLCOperand& op, double val);
    void    SetOperandFromInt(const PLCOperand& op, int32_t val);
    void    SetOperandFromBool(const PLCOperand& op, bool val);

    // Runtime safety guards. Normal valid PLC programs are unaffected.
    bool    IsValidOperandAddress(const PLCOperand& op) const;
    bool    IsValidTimerIndex(int index) const;
    bool    IsValidCounterIndex(int index) const;

    // V7.5.0 Stateful Runtime Foundation.
    // All vector allocation/preparation happens outside the 1ms ExecuteTask path.
    void    PrepareTaskRuntimeState(PLCTask& task);
    bool    IsTaskRuntimeStateReady(const PLCTask& task) const;

    void    ResetTimerRuntimeState(PLCTimer& timer);
    void    ResetCounterRuntimeState(PLCCounter& counter);
    void    ResetAllStatefulDevices();

    // V7.5.4 ~ V7.5.7 data/runtime safety helpers.
    bool    IsSupportedOpcode(uint8_t opcode) const;
    bool    IsNumericWritableOperand(const PLCOperand& op) const;
    bool    IsIntegerWritableOperand(const PLCOperand& op) const;
    bool    IsNumericReadableOperand(const PLCOperand& op) const;
    bool    IsIntegerReadableOperand(const PLCOperand& op) const;
    int32_t ClampDoubleToInt32(double value);
    float   ClampDoubleToFloat(double value);
    int32_t NormalizePositivePreset(int32_t value);
    int32_t BuildTimerPresetMs(int32_t presetValue, int32_t timeBase);
    bool    ParseLogicProgramFile(
        const std::string& filepath,
        std::vector<PLCTask>& tasksOut,
        std::unordered_map<int32_t, PLCCustomVar>& varsOut,
        uint32_t& crc32Out,
        uint32_t& fileSizeOut);

    // V7.6.0 - metadata of the ACTUAL logic.bin that was successfully accepted
    // and swapped into the runtime. Failed reloads never replace these values.
    uint32_t m_loadedLogicCrc32 = 0;
    uint32_t m_loadedLogicSize = 0;
    uint32_t m_logicLoadGeneration = 0;
    uint32_t m_lastLogicLoadResult =
        static_cast<uint32_t>(PLCLogicLoadResult::None);

    void    ResetLogicVerificationUnsafe();
    void    MarkLogicLoadFailed();
    void    CommitLoadedLogicVerificationUnsafe(
        uint32_t crc32,
        uint32_t fileSize);

    PLCRuntimeDiagnostics m_runtimeDiagnostics;

    PLCScanHealth m_scanHealth;

    // V7.6.4.1 - RTX64-safe high-resolution timing.
    int64_t m_scanCounterFrequency = 0;
    bool ReadScanCounter(int64_t& valueOut) const;
    uint32_t ScanCounterDeltaToUs(
        int64_t startCounter,
        int64_t endCounter) const;

    void ResetScanHealthUnsafe();
    void UpdateTaskScanHealthUnsafe(
        int32_t taskIndex,
        uint32_t elapsedUs,
        uint32_t budgetUs);
    void UpdateCycleScanHealthUnsafe(
        uint32_t elapsedUs,
        uint32_t budgetUs);

    int32_t m_activeTaskIndex = -1;
    int32_t m_activeInstructionIndex = -1;
    uint8_t m_activeOpcode = 0;

    void    ResetRuntimeDiagnosticsUnsafe();
    void    RecordRuntimeFault(
        PLCRuntimeFaultCode code,
        int32_t operandRegion = -1,
        int32_t operandAddress = 0);
    void    IncrementRuntimeFaultCounter(uint32_t& counter);

    void    ExecuteTask(PLCTask& task, int32_t taskIndex);
};

// 🌟 新增這行：宣告一個全域指標，讓所有人都能認識它
extern PLCManager* g_PLC;