#pragma once
#include <vector>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <algorithm> // 包含演算法標頭檔
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
    bool done;        // Counter done bit (instruction-mode dependent)
    bool initialized; // Runtime-only initialization state
    uint8_t mode;     // 0=unused/reset, 1=CTU, 2=CTD
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
    uint8_t opCode;   // e.g., 1=LD, 20=ADD
    PLCOperand op1;   // Variable 1
    PLCOperand op2;   // Variable 2
};

#pragma pack(pop)

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
    // Used by OUT_UP / OUT_DOWN, CTU / CTD, and V7.4.9 R_TRIG / F_TRIG.
    // 不寫入 logic.bin，因此不改變既有 Binary Protocol。
    std::vector<uint8_t> edgeMemory;
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

    void    Set_S(int index, bool value);
    bool    Get_S(int index) const;

    void ExportPLCStatus(SHM_PLC_Status* pStatus) const;
    // 🌟 新增：DR 斷電保持暫存器的二進位讀取與儲存 API
    bool SaveDRValues(const std::string& filepath);
    bool LoadDRValues(const std::string& filepath);

    // 🌟 新增：動態重載 PLC 邏輯程式的 API
    bool ReloadLogicProgram();
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

    void    ExecuteTask(PLCTask& task);
};

// 🌟 新增這行：宣告一個全域指標，讓所有人都能認識它
extern PLCManager* g_PLC;