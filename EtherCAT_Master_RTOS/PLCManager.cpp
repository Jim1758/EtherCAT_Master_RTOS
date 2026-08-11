#include "PLCManager.h"
#include <fstream>
#include <iostream>
#include <cstring>
#include <cmath>
#include <limits>
#include <algorithm> // 🌟 加入此列修正 std::sort 找不到的錯誤
#include "GlobalConfig.h" // 如果你有用到 DEBUG_PRINT 等功能
#include "SHM_Types.h"


// ============================================================================
// 🌟 專為 RTX64 設計的「自動解鎖」防呆機制 (RAII)
// ============================================================================
class AutoLockCS {
public:
    AutoLockCS(CRITICAL_SECTION* cs) : m_cs(cs) {
        EnterCriticalSection(m_cs); // 建構時自動上鎖
    }
    ~AutoLockCS() {
        LeaveCriticalSection(m_cs); // 離開範圍時 (包含 return) 自動解鎖
    }
private:
    CRITICAL_SECTION* m_cs;
};

// 🌟 定義全域指標 (預設為空)
PLCManager* g_PLC = nullptr;

PLCManager::PLCManager()
{
    // 🌟 1. 初始化臨界區段鎖 (RTX64 支援)
    InitializeCriticalSection(&m_logicCS);
    Init();
}

PLCManager::~PLCManager()
{
    // 🌟 2. 程式關閉時銷毀鎖
    DeleteCriticalSection(&m_logicCS);
}



// ============================================================================
// V7.6.0 Loaded Logic Verification
// ============================================================================

void PLCManager::ResetLogicVerificationUnsafe()
{
    m_loadedLogicCrc32 = 0;
    m_loadedLogicSize = 0;
    m_logicLoadGeneration = 0;
    m_lastLogicLoadResult =
        static_cast<uint32_t>(PLCLogicLoadResult::None);
}

void PLCManager::MarkLogicLoadFailed()
{
    AutoLockCS lock(&m_logicCS);

    // Preserve the metadata of the logic that is still ACTUALLY running.
    // Only the result of the most recent load/reload attempt changes.
    m_lastLogicLoadResult =
        static_cast<uint32_t>(PLCLogicLoadResult::Failed);
}

void PLCManager::CommitLoadedLogicVerificationUnsafe(
    uint32_t crc32,
    uint32_t fileSize)
{
    m_loadedLogicCrc32 = crc32;
    m_loadedLogicSize = fileSize;

    // Saturate instead of wrapping after an extremely long controller lifetime.
    if (m_logicLoadGeneration < 0xFFFFFFFFu)
        ++m_logicLoadGeneration;

    m_lastLogicLoadResult =
        static_cast<uint32_t>(PLCLogicLoadResult::Success);
}

PLCLogicVerificationState PLCManager::GetLogicVerificationState() const
{
    AutoLockCS lock(&m_logicCS);

    PLCLogicVerificationState state{};
    state.loadedLogicCrc32 = m_loadedLogicCrc32;
    state.loadedLogicSize = m_loadedLogicSize;
    state.logicLoadGeneration = m_logicLoadGeneration;
    state.lastLogicLoadResult = m_lastLogicLoadResult;
    return state;
}


// ============================================================================
// V7.6.3 Runtime Scan Health
// ============================================================================


bool PLCManager::ReadScanCounter(int64_t& valueOut) const
{
    LARGE_INTEGER counter{};

    if (!QueryPerformanceCounter(&counter))
        return false;

    valueOut = static_cast<int64_t>(counter.QuadPart);
    return true;
}

uint32_t PLCManager::ScanCounterDeltaToUs(
    int64_t startCounter,
    int64_t endCounter) const
{
    if (m_scanCounterFrequency <= 0 ||
        startCounter < 0 ||
        endCounter < startCounter)
        return 0;

    const uint64_t ticks =
        static_cast<uint64_t>(endCounter - startCounter);

    const uint64_t frequency =
        static_cast<uint64_t>(m_scanCounterFrequency);

    const uint64_t wholeSeconds =
        ticks / frequency;

    const uint64_t remainder =
        ticks % frequency;

    if (wholeSeconds > 4294ull)
        return 0xFFFFFFFFu;

    uint64_t microseconds =
        wholeSeconds * 1000000ull;

    microseconds +=
        (remainder * 1000000ull) / frequency;

    return microseconds > 0xFFFFFFFFull
        ? 0xFFFFFFFFu
        : static_cast<uint32_t>(microseconds);
}

void PLCManager::ResetScanHealthUnsafe()
{
    m_scanHealth = PLCScanHealth{};
}

void PLCManager::UpdateTaskScanHealthUnsafe(
    int32_t taskIndex,
    uint32_t elapsedUs,
    uint32_t budgetUs)
{
    m_scanHealth.lastTaskUs = elapsedUs;
    m_scanHealth.lastTaskBudgetUs = budgetUs;
    m_scanHealth.lastTaskIndex = taskIndex;

    if (elapsedUs > m_scanHealth.worstTaskUs)
    {
        m_scanHealth.worstTaskUs = elapsedUs;
        m_scanHealth.worstTaskIndex = taskIndex;
    }

    if (budgetUs > 0 && elapsedUs > budgetUs)
    {
        if (m_scanHealth.taskOverrunCount < 0xFFFFFFFFu)
            ++m_scanHealth.taskOverrunCount;

        m_scanHealth.lastOverrunRunCount = PLC_RunCount;
        m_scanHealth.lastOverrunTaskIndex = taskIndex;
    }
}

void PLCManager::UpdateCycleScanHealthUnsafe(
    uint32_t elapsedUs,
    uint32_t budgetUs)
{
    m_scanHealth.lastCycleUs = elapsedUs;
    m_scanHealth.cycleBudgetUs = budgetUs;

    if (elapsedUs > m_scanHealth.worstCycleUs)
        m_scanHealth.worstCycleUs = elapsedUs;

    if (m_scanHealth.measuredCycleCount < 0xFFFFFFFFu)
        ++m_scanHealth.measuredCycleCount;

    if (budgetUs > 0 && elapsedUs > budgetUs)
    {
        if (m_scanHealth.cycleOverrunCount < 0xFFFFFFFFu)
            ++m_scanHealth.cycleOverrunCount;

        // If a task already overran in this same RunCycle, keep the specific
        // task index instead of overwriting it with the cycle-level marker.
        if (m_scanHealth.lastOverrunRunCount != PLC_RunCount)
        {
            m_scanHealth.lastOverrunRunCount = PLC_RunCount;
            m_scanHealth.lastOverrunTaskIndex = -1;
        }
    }
}

PLCScanHealth PLCManager::GetScanHealth() const
{
    AutoLockCS lock(&m_logicCS);
    return m_scanHealth;
}

// ============================================================================
// V7.5.2 Runtime Diagnostics & Safety Guard
// ============================================================================

void PLCManager::IncrementRuntimeFaultCounter(uint32_t& counter)
{
    if (counter < 0xFFFFFFFFu)
        ++counter;
}

void PLCManager::ResetRuntimeDiagnosticsUnsafe()
{
    m_runtimeDiagnostics = PLCRuntimeDiagnostics{};
    m_activeTaskIndex = -1;
    m_activeInstructionIndex = -1;
    m_activeOpcode = 0;
}

void PLCManager::RecordRuntimeFault(
    PLCRuntimeFaultCode code,
    int32_t operandRegion,
    int32_t operandAddress)
{
    const bool duplicateSameInstruction =
        m_runtimeDiagnostics.faultActive &&
        m_runtimeDiagnostics.lastFaultRunCount == PLC_RunCount &&
        m_runtimeDiagnostics.lastTaskIndex == m_activeTaskIndex &&
        m_runtimeDiagnostics.lastInstructionIndex == m_activeInstructionIndex &&
        m_runtimeDiagnostics.lastFaultCode == static_cast<uint8_t>(code);

    m_runtimeDiagnostics.faultActive = true;
    m_runtimeDiagnostics.lastFaultCode = static_cast<uint8_t>(code);
    m_runtimeDiagnostics.lastOpcode = m_activeOpcode;
    m_runtimeDiagnostics.lastTaskIndex = m_activeTaskIndex;
    m_runtimeDiagnostics.lastInstructionIndex = m_activeInstructionIndex;
    m_runtimeDiagnostics.lastFaultRunCount = PLC_RunCount;
    m_runtimeDiagnostics.lastOperandRegion =
        (operandRegion >= 0 && operandRegion <= 255)
        ? static_cast<uint8_t>(operandRegion)
        : static_cast<uint8_t>(0xFF);
    m_runtimeDiagnostics.lastOperandAddress = operandAddress;

    if (duplicateSameInstruction)
        return;

    IncrementRuntimeFaultCounter(m_runtimeDiagnostics.totalFaultCount);

    switch (code) {
    case PLCRuntimeFaultCode::InvalidOperand:
        IncrementRuntimeFaultCounter(m_runtimeDiagnostics.invalidOperandCount); break;
    case PLCRuntimeFaultCode::InvalidTimerIndex:
        IncrementRuntimeFaultCounter(m_runtimeDiagnostics.invalidTimerIndexCount); break;
    case PLCRuntimeFaultCode::InvalidCounterIndex:
        IncrementRuntimeFaultCounter(m_runtimeDiagnostics.invalidCounterIndexCount); break;
    case PLCRuntimeFaultCode::UnknownOpcode:
        IncrementRuntimeFaultCounter(m_runtimeDiagnostics.unknownOpcodeCount); break;
    case PLCRuntimeFaultCode::RuntimeStateMismatch:
        IncrementRuntimeFaultCounter(m_runtimeDiagnostics.runtimeStateMismatchCount); break;
    case PLCRuntimeFaultCode::InvalidFlowRow:
        IncrementRuntimeFaultCounter(m_runtimeDiagnostics.invalidFlowRowCount); break;

        // Extended V7.5.5+ faults intentionally reuse TotalFaultCount only.
        // This keeps the V7.5.3 SHM_PLC_Diagnostics binary layout unchanged.
    case PLCRuntimeFaultCode::ArithmeticDomain:
    case PLCRuntimeFaultCode::IntegerOverflow:
    case PLCRuntimeFaultCode::InvalidBitIndex:
    case PLCRuntimeFaultCode::InvalidTaskConfig:
        break;

    case PLCRuntimeFaultCode::None:
    default:
        break;
    }
}

PLCRuntimeDiagnostics PLCManager::GetRuntimeDiagnostics() const
{
    AutoLockCS lock(&m_logicCS);
    return m_runtimeDiagnostics;
}

void PLCManager::ClearRuntimeDiagnostics()
{
    AutoLockCS lock(&m_logicCS);
    ResetRuntimeDiagnosticsUnsafe();
}


// ============================================================================
// V7.5.4 ~ V7.5.7 Data / Instruction / Loader Safety Helpers
// ============================================================================

bool PLCManager::IsSupportedOpcode(uint8_t opcode) const
{
    switch (opcode)
    {
    case 1: case 2: case 3: case 4: case 5: case 6:
    case 7: case 8: case 9: case 10: case 11: case 12:
    case 13: case 14: case 15: case 16: case 17:
    case 20: case 21: case 22: case 23: case 24: case 25: case 26: case 27:
    case 30: case 31: case 32: case 33: case 34: case 35:
    case 40: case 41: case 42:
    case 50: case 51: case 52:
    case 60: case 61: case 62: case 63:
    case 70: case 71:
    case 72: case 73: case 74: case 75: case 76: case 77: case 78: case 79:
    case 80: case 81:
    case 83: case 84:
        // V7.5.4 Advanced PLC Instruction Pack
    case 85: case 86: case 87: case 88: case 89: case 90:
    case 91: case 92: case 93: case 94: case 95: case 96: case 97:
        // V7.6.1 PLC System Contacts
    case 98: case 99: case 100: case 101: case 102: case 103:
        // V7.6.2 Process Math
    case 104: case 105: case 106: case 107:
    case 108: case 109: case 110: case 111: case 112:
        return true;
    default:
        return false;
    }
}

bool PLCManager::IsNumericWritableOperand(const PLCOperand& op) const
{
    if (!IsValidOperandAddress(op)) return false;

    switch (op.region)
    {
    case 6:  // R
    case 7:  // DR
    case 9:  // F
    case 10: // L
        return true;

    case 11: // VAR
    {
        auto it = m_customVars.find(op.payload.address);
        if (it == m_customVars.end()) return false;
        return it->second.dataType >= 2 && it->second.dataType <= 5;
    }

    default:
        return false;
    }
}

bool PLCManager::IsIntegerWritableOperand(const PLCOperand& op) const
{
    if (!IsValidOperandAddress(op)) return false;

    if (op.region == 6 || op.region == 7)
        return true;

    if (op.region == 11)
    {
        auto it = m_customVars.find(op.payload.address);
        if (it == m_customVars.end()) return false;
        return it->second.dataType == 2 || it->second.dataType == 3;
    }

    return false;
}

bool PLCManager::IsNumericReadableOperand(const PLCOperand& op) const
{
    if (op.region == 8) // constant
        return op.dataType >= 2 && op.dataType <= 5;

    if (!IsValidOperandAddress(op))
        return false;

    if (op.region == 6 || op.region == 7 || op.region == 9 || op.region == 10)
        return true;

    if (op.region == 11)
    {
        auto it = m_customVars.find(op.payload.address);
        if (it == m_customVars.end()) return false;
        return it->second.dataType >= 2 && it->second.dataType <= 5;
    }

    return false;
}

bool PLCManager::IsIntegerReadableOperand(const PLCOperand& op) const
{
    if (op.region == 8)
        return op.dataType == 2 || op.dataType == 3;

    if (!IsValidOperandAddress(op))
        return false;

    if (op.region == 6 || op.region == 7)
        return true;

    if (op.region == 11)
    {
        auto it = m_customVars.find(op.payload.address);
        if (it == m_customVars.end()) return false;
        return it->second.dataType == 2 || it->second.dataType == 3;
    }

    return false;
}

int32_t PLCManager::ClampDoubleToInt32(double value)
{
    if (!std::isfinite(value))
    {
        RecordRuntimeFault(PLCRuntimeFaultCode::ArithmeticDomain);
        return 0;
    }

    const double hi = static_cast<double>((std::numeric_limits<int32_t>::max)());
    const double lo = static_cast<double>((std::numeric_limits<int32_t>::min)());

    if (value > hi)
    {
        RecordRuntimeFault(PLCRuntimeFaultCode::IntegerOverflow);
        return (std::numeric_limits<int32_t>::max)();
    }

    if (value < lo)
    {
        RecordRuntimeFault(PLCRuntimeFaultCode::IntegerOverflow);
        return (std::numeric_limits<int32_t>::min)();
    }

    return static_cast<int32_t>(value);
}

float PLCManager::ClampDoubleToFloat(double value)
{
    if (!std::isfinite(value))
    {
        RecordRuntimeFault(PLCRuntimeFaultCode::ArithmeticDomain);
        return 0.0f;
    }

    const double hi = static_cast<double>((std::numeric_limits<float>::max)());

    if (value > hi)
    {
        RecordRuntimeFault(PLCRuntimeFaultCode::IntegerOverflow);
        return (std::numeric_limits<float>::max)();
    }

    if (value < -hi)
    {
        RecordRuntimeFault(PLCRuntimeFaultCode::IntegerOverflow);
        return -(std::numeric_limits<float>::max)();
    }

    return static_cast<float>(value);
}

int32_t PLCManager::NormalizePositivePreset(int32_t value)
{
    if (value > 0)
        return value;

    RecordRuntimeFault(PLCRuntimeFaultCode::ArithmeticDomain);
    return 1;
}

int32_t PLCManager::BuildTimerPresetMs(int32_t presetValue, int32_t timeBase)
{
    const int32_t safePreset = NormalizePositivePreset(presetValue);
    const int32_t safeBase = timeBase > 0 ? timeBase : 1;

    const int64_t value =
        static_cast<int64_t>(safePreset) *
        static_cast<int64_t>(safeBase);

    if (value > (std::numeric_limits<int32_t>::max)())
    {
        RecordRuntimeFault(PLCRuntimeFaultCode::IntegerOverflow);
        return (std::numeric_limits<int32_t>::max)();
    }

    return static_cast<int32_t>(value);
}

bool PLCManager::ParseLogicProgramFile(
    const std::string& filepath,
    std::vector<PLCTask>& tasksOut,
    std::unordered_map<int32_t, PLCCustomVar>& varsOut,
    uint32_t& crc32Out,
    uint32_t& fileSizeOut)
{
    tasksOut.clear();
    varsOut.clear();
    crc32Out = 0;
    fileSizeOut = 0;

    std::ifstream file(filepath, std::ios::binary);
    if (!file.is_open())
        return false;

    uint32_t crcState = 0xFFFFFFFFu;
    uint32_t byteCount = 0;

    auto consumeBytesForCrc = [&](const void* buffer, std::streamsize bytes) -> bool
    {
        if (buffer == nullptr || bytes < 0)
            return false;

        const uint64_t bytes64 = static_cast<uint64_t>(bytes);
        if (bytes64 > static_cast<uint64_t>(0xFFFFFFFFu - byteCount))
            return false;

        const uint8_t* ptr = reinterpret_cast<const uint8_t*>(buffer);

        for (std::streamsize i = 0; i < bytes; ++i)
        {
            crcState ^= ptr[i];

            for (int bit = 0; bit < 8; ++bit)
            {
                crcState =
                    (crcState >> 1) ^
                    (0xEDB88320u & (0u - (crcState & 1u)));
            }
        }

        byteCount += static_cast<uint32_t>(bytes);
        return true;
    };

    auto readExact = [&](void* buffer, std::streamsize bytes) -> bool
    {
        if (bytes < 0)
            return false;

        file.read(reinterpret_cast<char*>(buffer), bytes);

        if (file.gcount() != bytes)
            return false;

        return consumeBytesForCrc(buffer, bytes);
    };

    char header[8] = {};
    if (!readExact(header, 8))
        return false;

    if (std::memcmp(header, "RTOS_PLC", 8) != 0)
        return false;

    bool sawEndTag = false;
    bool sawAllocationTable = false;
    bool sawTask = false;

    while (true)
    {
        uint8_t tag = 0;
        if (!readExact(&tag, 1))
            return false;

        if (tag == 255)
        {
            sawEndTag = true;
            break;
        }

        if (tag == 253)
        {
            // V7.6.4 final format: exactly one allocation table and it must
            // appear before every Task block.
            if (sawAllocationTable || sawTask)
                return false;

            sawAllocationTable = true;

            int32_t varCount = 0;
            if (!readExact(&varCount, 4))
                return false;

            if (varCount < 0 || varCount > MAX_PLC_VAR_ALLOCATIONS)
                return false;

            for (int32_t i = 0; i < varCount; ++i)
            {
                int32_t hashId = 0;
                uint8_t dataType = 0;

                if (!readExact(&hashId, 4) ||
                    !readExact(&dataType, 1))
                    return false;

                if (dataType < 1 || dataType > 5)
                    return false;

                if (varsOut.find(hashId) != varsOut.end())
                    return false;

                PLCCustomVar newVar{};
                newVar.dataType = dataType;
                std::memset(newVar.value.raw, 0, sizeof(newVar.value.raw));
                varsOut.emplace(hashId, newVar);
            }

            continue;
        }

        if (tag != 254)
            return false;

        if (!sawAllocationTable)
            return false;

        sawTask = true;

        if (tasksOut.size() >= static_cast<size_t>(MAX_PLC_TASKS))
            return false;

        PLCTask newTask{};

        if (!readExact(&newTask.type, 1) ||
            !readExact(&newTask.priority, 1) ||
            !readExact(&newTask.cycleTimeMs, 4))
            return false;

        if (newTask.type < 1 || newTask.type > 4 ||
            newTask.priority > 31 ||
            (newTask.type == 1 && newTask.cycleTimeMs < 1))
            return false;

        newTask.currentTimerMs = newTask.cycleTimeMs;

        while (true)
        {
            const int next = file.peek();
            if (next == EOF)
                return false;

            if (next == 253 || next == 254 || next == 255)
                break;

            if (newTask.instructions.size() >=
                static_cast<size_t>(MAX_PLC_INSTRUCTIONS_PER_TASK))
                return false;

            PLCInstruction inst{};
            if (!readExact(&inst, sizeof(PLCInstruction)))
                return false;

            if (!IsSupportedOpcode(inst.opCode))
                return false;

            newTask.instructions.push_back(inst);
        }

        PrepareTaskRuntimeState(newTask);
        tasksOut.push_back(std::move(newTask));
    }

    if (!sawEndTag || !sawAllocationTable || !sawTask)
        return false;

    // V7.6.4 final binary contract:
    // tag 255 is followed by exactly 5 zero bytes:
    //     byte 0 + int32 0
    // No extra bytes are accepted.
    uint8_t trailer[5] = {};
    if (!readExact(trailer, 5))
        return false;

    for (uint8_t value : trailer)
    {
        if (value != 0)
            return false;
    }

    if (file.peek() != EOF)
        return false;

    std::sort(
        tasksOut.begin(),
        tasksOut.end(),
        [](const PLCTask& a, const PLCTask& b)
        {
            return a.priority < b.priority;
        });

    crc32Out = ~crcState;
    fileSizeOut = byteCount;
    return true;
}

// ============================================================================
// V7.5.0 Stateful Runtime Foundation
// ============================================================================

void PLCManager::PrepareTaskRuntimeState(PLCTask& task)
{
    // IMPORTANT:
    // This function is called only from LoadLogicProgram / ReloadLogicProgram.
    // Heap allocation here is intentionally outside the 1ms ExecuteTask path.
    task.edgeMemory.assign(task.instructions.size(), 0);
    task.flowRows.assign(MAX_PLC_FLOW_ROWS, 0);
    task.firstScanPending = true;
}

bool PLCManager::IsTaskRuntimeStateReady(const PLCTask& task) const
{
    return
        task.edgeMemory.size() == task.instructions.size() &&
        task.flowRows.size() == static_cast<size_t>(MAX_PLC_FLOW_ROWS);
}

void PLCManager::ResetTimerRuntimeState(PLCTimer& timer)
{
    timer.enable = false;
    timer.done = false;
    timer.preset = 0;
    timer.acc = 0;
    timer.timeBase = 0;
    timer.mode = 0;
    timer.input = false;
    timer.prevInput = false;
}

void PLCManager::ResetCounterRuntimeState(PLCCounter& counter)
{
    counter.preset = 0;
    counter.acc = 0;
    counter.done = false;
    counter.zero = true;
    counter.initialized = false;
    counter.mode = 0;
}

void PLCManager::ResetAllStatefulDevices()
{
    for (int i = 0; i < MAX_PLC_T; ++i)
        ResetTimerRuntimeState(m_T[i]);

    for (int i = 0; i < MAX_PLC_CNT; ++i)
        ResetCounterRuntimeState(m_CNT[i]);
}


void PLCManager::Init()
{
    std::memset(m_I, 0, sizeof(m_I));
    std::memset(m_O, 0, sizeof(m_O));
    std::memset(m_A, 0, sizeof(m_A));
    std::memset(m_S, 0, sizeof(m_S));
    std::memset(m_C, 0, sizeof(m_C));
    std::memset(m_R, 0, sizeof(m_R));
    std::memset(m_DR, 0, sizeof(m_DR));
    std::memset(m_F, 0, sizeof(m_F));
    std::memset(m_L, 0, sizeof(m_L));

    // Stateful devices are reset through one shared implementation so Init and
    // Hot Reload cannot silently diverge.
    ResetAllStatefulDevices();
    ResetRuntimeDiagnosticsUnsafe();
    ResetScanHealthUnsafe();
    ResetLogicVerificationUnsafe();

    LARGE_INTEGER scanFrequency{};
    if (QueryPerformanceFrequency(&scanFrequency) &&
        scanFrequency.QuadPart > 0)
    {
        m_scanCounterFrequency =
            static_cast<int64_t>(scanFrequency.QuadPart);
    }
    else
    {
        // Measurement unavailable must never stop PLC execution.
        m_scanCounterFrequency = 0;
    }

    LoadDRValues(GlobalConfig::GetInstance().PLC_Dir + "PLC_DR.txt");
    Init_flag = 1;
}
void PLCManager::Close_PLC()
{
    Close_flag = 1;
    SaveDRValues(GlobalConfig::GetInstance().PLC_Dir + "PLC_DR.txt");
    std::memset(m_I, 0, sizeof(m_I));
    std::memset(m_O, 0, sizeof(m_O));
    std::memset(m_A, 0, sizeof(m_A));
    std::memset(m_S, 0, sizeof(m_S));
    std::memset(m_C, 0, sizeof(m_C));
    std::memset(m_R, 0, sizeof(m_R));
    std::memset(m_DR, 0, sizeof(m_DR));
    std::memset(m_F, 0, sizeof(m_F));
    std::memset(m_L, 0, sizeof(m_L));
    std::memset(m_CNT, 0, sizeof(m_CNT));


}

// =========================================================
// 1. Load Logic Program (Binary from C#)
// =========================================================
bool PLCManager::LoadLogicProgram(const std::string& filepath)
{
    std::vector<PLCTask> tempTasks;
    std::unordered_map<int32_t, PLCCustomVar> tempVars;
    uint32_t parsedCrc32 = 0;
    uint32_t parsedFileSize = 0;

    if (!ParseLogicProgramFile(
        filepath,
        tempTasks,
        tempVars,
        parsedCrc32,
        parsedFileSize))
    {
        MarkLogicLoadFailed();

        std::cerr << "[PLC] Invalid or incomplete PLC logic file: "
            << filepath << std::endl;
        return false;
    }

    // Atomic ownership handoff. Existing memory points are not changed.
    {
        AutoLockCS lock(&m_logicCS);
        m_tasks.swap(tempTasks);
        m_customVars.swap(tempVars);
        ResetAllStatefulDevices();
        ResetRuntimeDiagnosticsUnsafe();
        ResetScanHealthUnsafe();

        // Commit verification metadata ONLY after the new logic has been
        // accepted and swapped into the runtime.
        CommitLoadedLogicVerificationUnsafe(
            parsedCrc32,
            parsedFileSize);
    }

    std::cout << "[PLC] Loaded "
        << m_tasks.size()
        << " PLC task(s). CRC32="
        << std::hex << std::uppercase
        << m_loadedLogicCrc32
        << std::dec
        << ", Size="
        << m_loadedLogicSize
        << " bytes, Generation="
        << m_logicLoadGeneration
        << std::endl;

    return true;
}

// =========================================================
// 🌟 DR (Double Register) 斷電保持暫存器儲存與讀取 API
// =========================================================

/// <summary>
/// 將當前所有的 DR 暫存器 (1000 個 32-bit 數據) 直接傾印存入硬碟檔案
/// </summary>
/// <param name="filepath">要儲存的完整檔案路徑 (例如 "D:\\EtherCAT_Master_Data\\PLC\\DR_Data.dat")</param>
bool PLCManager::SaveDRValues(const std::string& filepath)
{
    // 以二進位寫入模式開啟檔案
    std::ofstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        //std::cerr << "[PLC Error] 無法建立或開啟 DR 儲存檔: " << filepath << std::endl;
        return false;
    }

    // 將整個 m_DR 陣列 (1000 * 4 bytes = 4000 bytes) 極速寫入硬碟
    file.write(reinterpret_cast<const char*>(m_DR), sizeof(m_DR));
    file.close();

    //std::cout << "[PLC] 成功將 DR 暫存器數值保存至: " << filepath << std::endl;
    return true;
}

/// <summary>
/// 從硬碟檔案載入歷史的 DR 暫存器數值並還原到 m_DR 陣列中
/// </summary>
/// <param name="filepath">要載入的完整檔案路徑</param>
bool PLCManager::LoadDRValues(const std::string& filepath)
{
    // 以二進位讀取模式開啟檔案
    std::ifstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        //std::cerr << "[PLC Warning] 找不到 DR 歷史數據檔 (可能是第一次啟動): " << filepath << std::endl;
        return false;
    }

    // 直接讀取至 m_DR 記憶體中
    file.read(reinterpret_cast<char*>(m_DR), sizeof(m_DR));
    file.close();

    //std::cout << "[PLC] 成功載入 DR 暫存器歷史數據: " << filepath << std::endl;
    return true;
}

// =========================================================
// 2. High-Speed Virtual Machine Operand Parser
// =========================================================
bool PLCManager::IsValidTimerIndex(int index) const {
    return index >= 0 && index < MAX_PLC_T;
}

bool PLCManager::IsValidCounterIndex(int index) const {
    return index >= 0 && index < MAX_PLC_CNT;
}

bool PLCManager::IsValidOperandAddress(const PLCOperand& op) const {
    if (op.region == 8) return true; // #Constant has no memory address.
    if (op.region == 11) return true; // VAR uses stable hash as payload.address.

    const int addr = op.payload.address;
    switch (op.region) {
    case 0: return addr >= 0 && addr < MAX_PLC_I;
    case 1: return addr >= 0 && addr < MAX_PLC_O;
    case 2: return addr >= 0 && addr < MAX_PLC_C;
    case 3: return addr >= 0 && addr < MAX_PLC_S;
    case 4: return addr >= 0 && addr < MAX_PLC_A;
    case 5: return addr >= 0 && addr < MAX_PLC_T;
    case 6: return addr >= 0 && addr < MAX_PLC_R;
    case 7: return addr >= 0 && addr < MAX_PLC_DR;
    case 9: return addr >= 0 && addr < MAX_PLC_F;
    case 10: return addr >= 0 && addr < MAX_PLC_L;
    case 12: return addr >= 0 && addr < MAX_PLC_CNT;
    default: return false;
    }
}

double PLCManager::GetOperandAsDouble(const PLCOperand& op) {
    if (op.region == 8) { // Constant (#)
        if (op.dataType == 4) return op.payload.realValue;
        if (op.dataType == 5) return op.payload.lrealValue;
        return op.payload.intValue;
    }

    if (op.region == 11) { // Custom VAR
        auto it = m_customVars.find(op.payload.address);
        if (it == m_customVars.end()) {
            RecordRuntimeFault(PLCRuntimeFaultCode::InvalidOperand, op.region, op.payload.address);
            return 0.0;
        }
        if (op.dataType == 4) return it->second.value.fVal;
        if (op.dataType == 5) return it->second.value.dVal;
        return it->second.value.iVal;
    }

    if (!IsValidOperandAddress(op)) {
        RecordRuntimeFault(PLCRuntimeFaultCode::InvalidOperand, op.region, op.payload.address);
        return 0.0;
    }

    const int addr = op.payload.address;
    switch (op.region) {
    case 0: return m_I[addr];
    case 1: return m_O[addr];
    case 2: return m_C[addr];
    case 3: return m_S[addr]; // V7.4.3: restore S as readable operand.
    case 4: return m_A[addr]; // V7.4.3: restore A as readable operand.
    case 5: return m_T[addr].done ? 1.0 : 0.0;
    case 6: return m_R[addr];
    case 7: return m_DR[addr];
    case 9: return m_F[addr];
    case 10: return m_L[addr];
    case 12: return m_CNT[addr].acc;
    default: return 0.0;
    }
}

int32_t PLCManager::GetOperandAsInt(const PLCOperand& op) {
    return ClampDoubleToInt32(GetOperandAsDouble(op));
}

bool PLCManager::GetOperandAsBool(const PLCOperand& op) {
    if (op.region == 5) {
        const int index = op.payload.address;
        if (!IsValidTimerIndex(index)) {
            RecordRuntimeFault(PLCRuntimeFaultCode::InvalidTimerIndex, op.region, index);
            return false;
        }
        return m_T[index].done;
    }
    if (op.region == 12) {
        const int index = op.payload.address;
        if (!IsValidCounterIndex(index)) {
            RecordRuntimeFault(PLCRuntimeFaultCode::InvalidCounterIndex, op.region, index);
            return false;
        }
        return m_CNT[index].done;
    }
    return GetOperandAsInt(op) != 0;
}

void PLCManager::SetOperandFromDouble(const PLCOperand& op, double val) {
    if (op.region == 8) {
        RecordRuntimeFault(PLCRuntimeFaultCode::InvalidOperand, op.region, op.payload.intValue);
        return;
    }

    if (op.region == 11) { // Custom VAR
        auto it = m_customVars.find(op.payload.address);
        if (it == m_customVars.end()) {
            RecordRuntimeFault(PLCRuntimeFaultCode::InvalidOperand, op.region, op.payload.address);
            return;
        }
        if (op.dataType == 4)
            it->second.value.fVal = ClampDoubleToFloat(val);
        else if (op.dataType == 5)
        {
            if (!std::isfinite(val)) {
                RecordRuntimeFault(PLCRuntimeFaultCode::ArithmeticDomain, op.region, op.payload.address);
                it->second.value.dVal = 0.0;
            }
            else {
                it->second.value.dVal = val;
            }
        }
        else
            it->second.value.iVal = ClampDoubleToInt32(val);
        return;
    }

    if (!IsValidOperandAddress(op)) {
        RecordRuntimeFault(PLCRuntimeFaultCode::InvalidOperand, op.region, op.payload.address);
        return;
    }

    const int addr = op.payload.address;
    switch (op.region) {
    case 6: m_R[addr] = ClampDoubleToInt32(val); break;
    case 7: m_DR[addr] = ClampDoubleToInt32(val); break;
    case 9: m_F[addr] = ClampDoubleToFloat(val); break;
    case 10:
        if (!std::isfinite(val)) {
            RecordRuntimeFault(PLCRuntimeFaultCode::ArithmeticDomain, op.region, addr);
            m_L[addr] = 0.0;
        }
        else {
            m_L[addr] = val;
        }
        break;
    case 1: m_O[addr] = (val != 0); break;
    case 2: m_C[addr] = (val != 0); break;
    case 3: m_S[addr] = (val != 0); break;
    case 4: m_A[addr] = (val != 0); break;
    case 0:
    case 5:
    case 12:
    default:
        RecordRuntimeFault(PLCRuntimeFaultCode::InvalidOperand, op.region, addr);
        break;
    }
}

void PLCManager::SetOperandFromInt(const PLCOperand& op, int32_t val) {
    SetOperandFromDouble(op, static_cast<double>(val));
}

void PLCManager::SetOperandFromBool(const PLCOperand& op, bool val) {
    SetOperandFromDouble(op, val ? 1.0 : 0.0);
}

// =========================================================
// 3. Main Run Cycle (Timers & Tasks Scheduler)
// =========================================================
void PLCManager::RunCycle(int delta_ms)
{
    if (delta_ms <= 0) {
        RecordRuntimeFault(PLCRuntimeFaultCode::InvalidTaskConfig);
        return;
    }

    if (Init_flag != 1)
    {
        return;
    }
    if (Close_flag == 1)
    {
        return;
    }

    int64_t cycleStartCounter = 0;
    const bool cycleTimingReady =
        ReadScanCounter(cycleStartCounter);

    PLC_RunCount += 1;

    // 🌟 使用我們自己寫的 AutoLockCS，傳入 m_logicCS 的記憶體位址
    AutoLockCS lock(&m_logicCS);

    // Process all Timers. RunCycle is treated as a 1ms base call by system contract.
    // Existing TON behavior is preserved; V7.4.5 adds TOF / TP / RTO modes.
    for (int i = 0; i < MAX_PLC_T; i++) {
        PLCTimer& timer = m_T[i];

        switch (timer.mode) {
        case 1: // TOF - done stays ON during OFF-delay timing.
            if (timer.input) {
                timer.enable = false;
                timer.acc = 0;
                timer.done = true;
            }
            else if (timer.enable && timer.done) {
                const int64_t nextAcc =
                    static_cast<int64_t>(timer.acc) +
                    static_cast<int64_t>(delta_ms);
                timer.acc = nextAcc >= timer.preset
                    ? timer.preset
                    : static_cast<int32_t>(nextAcc);
                if (timer.acc >= timer.preset) {
                    timer.done = false;
                    timer.enable = false;
                }
            }
            break;

        case 2: // TP - fixed pulse after rising edge, independent of later input state.
            if (timer.enable && timer.done) {
                const int64_t nextAcc =
                    static_cast<int64_t>(timer.acc) +
                    static_cast<int64_t>(delta_ms);
                timer.acc = nextAcc >= timer.preset
                    ? timer.preset
                    : static_cast<int32_t>(nextAcc);
                if (timer.acc >= timer.preset) {
                    timer.done = false;
                    timer.enable = false;
                }
            }
            break;

        case 3: // RTO - retain acc/done while input is OFF; TMR_RST clears it.
            if (timer.enable && !timer.done) {
                const int64_t nextAcc =
                    static_cast<int64_t>(timer.acc) +
                    static_cast<int64_t>(delta_ms);
                timer.acc = nextAcc >= timer.preset
                    ? timer.preset
                    : static_cast<int32_t>(nextAcc);
                if (timer.acc >= timer.preset) {
                    timer.done = true;
                    timer.enable = false;
                }
            }
            break;

        case 0:
        default: // Existing TON / TMR behavior.
            if (timer.enable) {
                if (!timer.done) {
                    const int64_t nextAcc =
                        static_cast<int64_t>(timer.acc) +
                        static_cast<int64_t>(delta_ms);
                    timer.acc = nextAcc >= timer.preset
                        ? timer.preset
                        : static_cast<int32_t>(nextAcc);
                    if (timer.acc >= timer.preset) {
                        timer.done = true;
                    }
                }
            }
            else {
                timer.acc = 0;
                timer.done = false;
            }
            break;
        }
    }

    // RTOS Task Scheduler based on Cycle Time
    for (size_t taskIndex = 0; taskIndex < m_tasks.size(); ++taskIndex) {
        PLCTask& task = m_tasks[taskIndex];
        if (task.type == 1) {
            if (task.cycleTimeMs < 1) {
                m_activeTaskIndex = static_cast<int32_t>(taskIndex);
                RecordRuntimeFault(PLCRuntimeFaultCode::InvalidTaskConfig);
                m_activeTaskIndex = -1;
                continue;
            }

            const int64_t nextTaskTimer =
                static_cast<int64_t>(task.currentTimerMs) +
                static_cast<int64_t>(delta_ms);

            task.currentTimerMs =
                nextTaskTimer > (std::numeric_limits<int32_t>::max)()
                ? task.cycleTimeMs
                : static_cast<int32_t>(nextTaskTimer);

            if (task.currentTimerMs >= task.cycleTimeMs) {
                task.currentTimerMs -= task.cycleTimeMs;

                int64_t taskStartCounter = 0;
                const bool taskTimingReady =
                    ReadScanCounter(taskStartCounter);

                ExecuteTask(task, static_cast<int32_t>(taskIndex));

                int64_t taskEndCounter = 0;

                if (taskTimingReady &&
                    ReadScanCounter(taskEndCounter))
                {
                    const uint32_t taskUs =
                        ScanCounterDeltaToUs(
                            taskStartCounter,
                            taskEndCounter);

                    const uint64_t rawBudgetUs =
                        static_cast<uint64_t>(task.cycleTimeMs) * 1000ull;

                    const uint32_t taskBudgetUs =
                        rawBudgetUs > 0xFFFFFFFFull
                        ? 0xFFFFFFFFu
                        : static_cast<uint32_t>(rawBudgetUs);

                    UpdateTaskScanHealthUnsafe(
                        static_cast<int32_t>(taskIndex),
                        taskUs,
                        taskBudgetUs);
                }
            }
        }
    }

    int64_t cycleEndCounter = 0;

    if (cycleTimingReady &&
        ReadScanCounter(cycleEndCounter))
    {
        const uint32_t cycleUs =
            ScanCounterDeltaToUs(
                cycleStartCounter,
                cycleEndCounter);

        const uint64_t rawCycleBudgetUs =
            static_cast<uint64_t>(delta_ms) * 1000ull;

        const uint32_t cycleBudgetUs =
            rawCycleBudgetUs > 0xFFFFFFFFull
            ? 0xFFFFFFFFu
            : static_cast<uint32_t>(rawCycleBudgetUs);

        UpdateCycleScanHealthUnsafe(
            cycleUs,
            cycleBudgetUs);
    }
}

// =========================================================
// 4. Virtual Machine: Instruction Decoding & Execution
// =========================================================
void PLCManager::ExecuteTask(PLCTask& task, int32_t taskIndex)
{
    bool ACC = false;

    m_activeTaskIndex = taskIndex;
    m_activeInstructionIndex = -1;
    m_activeOpcode = 0;

    if (!IsTaskRuntimeStateReady(task)) {
        RecordRuntimeFault(PLCRuntimeFaultCode::RuntimeStateMismatch);
        m_activeTaskIndex = -1;
        return;
    }

    auto ensureFlowRow = [&](int row) -> bool {
        if (row < 0 || row >= MAX_PLC_FLOW_ROWS) {
            RecordRuntimeFault(PLCRuntimeFaultCode::InvalidFlowRow, -1, row);
            return false;
        }
        return true;
    };

    // First Scan is task-local so slower cyclic tasks cannot miss the pulse.
    // Every instruction in this first task execution sees the same TRUE state.
    const bool taskFirstScan = task.firstScanPending;

    // V7.6.2 SCALE staging: scan-local only, no heap, no FB instance.
    bool scaleSequenceValid = false;
    bool scaleClampInput = false;
    double scaleInputValue = 0.0;
    double scaleInputLow = 0.0;
    double scaleInputHigh = 0.0;
    double scaleOutputLow = 0.0;

    for (size_t instIndex = 0; instIndex < task.instructions.size(); ++instIndex)
    {
        const auto& inst = task.instructions[instIndex];
        m_activeInstructionIndex = static_cast<int32_t>(instIndex);
        m_activeOpcode = inst.opCode;

        switch (inst.opCode)
        {
        case 1:  // LD
            ACC = GetOperandAsBool(inst.op1);
            break;

        case 2:  // LDI
            ACC = !GetOperandAsBool(inst.op1);
            break;

        case 3:  // LDT
        {
            const int tIdx = inst.op1.payload.address;
            if (!IsValidTimerIndex(tIdx)) {
                RecordRuntimeFault(PLCRuntimeFaultCode::InvalidTimerIndex, inst.op1.region, tIdx);
                ACC = false;
                break;
            }
            ACC = m_T[tIdx].done;
            break;
        }

        case 4:  // LDIT
        {
            const int tIdx = inst.op1.payload.address;
            if (!IsValidTimerIndex(tIdx)) {
                RecordRuntimeFault(PLCRuntimeFaultCode::InvalidTimerIndex, inst.op1.region, tIdx);
                ACC = false;
                break;
            }
            ACC = !m_T[tIdx].done;
            break;
        }

        case 5:  // OR
            ACC = ACC || GetOperandAsBool(inst.op1);
            break;

        case 6:  // ORI
            ACC = ACC || !GetOperandAsBool(inst.op1);
            break;

        case 7:  // OUT
            SetOperandFromBool(inst.op1, ACC);
            break;

        case 8:  // OUT_NOT
            SetOperandFromBool(inst.op1, !ACC);
            break;

        case 9:  // OUT_UP - one PLC-task-scan pulse on ACC rising edge
        {
            const bool previous = task.edgeMemory[instIndex] != 0;
            const bool pulse = ACC && !previous;
            SetOperandFromBool(inst.op1, pulse);
            task.edgeMemory[instIndex] = ACC ? 1 : 0;
            break;
        }

        case 10: // OUT_DOWN - one PLC-task-scan pulse on ACC falling edge
        {
            const bool previous = task.edgeMemory[instIndex] != 0;
            const bool pulse = !ACC && previous;
            SetOperandFromBool(inst.op1, pulse);
            task.edgeMemory[instIndex] = ACC ? 1 : 0;
            break;
        }

        case 11: // OUT_L (SET)
            if (ACC) SetOperandFromBool(inst.op1, true);
            break;

        case 12: // OUT_UL (RST)
            if (ACC) SetOperandFromBool(inst.op1, false);
            break;

        case 17: // TMR_1MS (1ms Base) - V7.4.3
        case 13: // TMR_10  (10ms Base)
        case 14: // TMR_100 (100ms Base)
        case 15: // TMR_1S  (1000ms Base)
        {
            const int tIdx = inst.op1.payload.address;
            if (!IsValidTimerIndex(tIdx)) {
                RecordRuntimeFault(PLCRuntimeFaultCode::InvalidTimerIndex, inst.op1.region, tIdx);
                break;
            }

            if (ACC) {
                const int base =
                    (inst.opCode == 17) ? 1 :
                    (inst.opCode == 13) ? 10 :
                    (inst.opCode == 14) ? 100 : 1000;

                const int32_t presetValue = GetOperandAsInt(inst.op2);
                m_T[tIdx].mode = 0;
                m_T[tIdx].input = true;
                m_T[tIdx].prevInput = true;
                m_T[tIdx].enable = true;
                m_T[tIdx].timeBase = base;
                m_T[tIdx].preset = BuildTimerPresetMs(presetValue, base);
            }
            else {
                m_T[tIdx].mode = 0;
                m_T[tIdx].input = false;
                m_T[tIdx].prevInput = false;
                m_T[tIdx].enable = false;
            }
            break;
        }

        case 50: // TOF_1MS - OFF Delay Timer, 1ms base
        {
            const int tIdx = inst.op1.payload.address;
            if (!IsValidTimerIndex(tIdx)) {
                RecordRuntimeFault(PLCRuntimeFaultCode::InvalidTimerIndex, inst.op1.region, tIdx);
                break;
            }

            PLCTimer& timer = m_T[tIdx];
            const bool previousInput = timer.prevInput;
            const int32_t presetValue = GetOperandAsInt(inst.op2);

            timer.mode = 1;
            timer.timeBase = 1;
            timer.preset = NormalizePositivePreset(presetValue);
            timer.input = ACC;

            if (ACC) {
                timer.enable = false;
                timer.acc = 0;
                timer.done = true;
            }
            else if (previousInput) {
                // Falling edge: start OFF-delay and keep Q/DN ON until preset expires.
                timer.enable = true;
                timer.acc = 0;
                timer.done = true;
            }

            timer.prevInput = ACC;
            break;
        }

        case 51: // TP_1MS - Pulse Timer, 1ms base
        {
            const int tIdx = inst.op1.payload.address;
            if (!IsValidTimerIndex(tIdx)) {
                RecordRuntimeFault(PLCRuntimeFaultCode::InvalidTimerIndex, inst.op1.region, tIdx);
                break;
            }

            PLCTimer& timer = m_T[tIdx];
            const bool previousInput = timer.prevInput;
            const int32_t presetValue = GetOperandAsInt(inst.op2);

            timer.mode = 2;
            timer.timeBase = 1;
            timer.preset = NormalizePositivePreset(presetValue);
            timer.input = ACC;

            if (ACC && !previousInput && !timer.enable) {
                timer.enable = true;
                timer.acc = 0;
                timer.done = true;
            }

            timer.prevInput = ACC;
            break;
        }

        case 52: // RTO_1MS - Retentive ON Delay, 1ms base
        {
            const int tIdx = inst.op1.payload.address;
            if (!IsValidTimerIndex(tIdx)) {
                RecordRuntimeFault(PLCRuntimeFaultCode::InvalidTimerIndex, inst.op1.region, tIdx);
                break;
            }

            PLCTimer& timer = m_T[tIdx];
            const int32_t presetValue = GetOperandAsInt(inst.op2);

            timer.mode = 3;
            timer.timeBase = 1;
            timer.preset = NormalizePositivePreset(presetValue);
            timer.input = ACC;
            timer.enable = ACC && !timer.done;
            timer.prevInput = ACC;
            break;
        }

        case 16: // TMR_RST
        {
            const int tIdx = inst.op1.payload.address;
            if (!IsValidTimerIndex(tIdx)) {
                RecordRuntimeFault(PLCRuntimeFaultCode::InvalidTimerIndex, inst.op1.region, tIdx);
                break;
            }
            if (ACC) ResetTimerRuntimeState(m_T[tIdx]);
            break;
        }

        case 60: // CTU - Counter Up, count on ACC rising edge
        {
            const int cIdx = inst.op1.payload.address;
            if (!IsValidCounterIndex(cIdx)) {
                RecordRuntimeFault(PLCRuntimeFaultCode::InvalidCounterIndex, inst.op1.region, cIdx);
                break;
            }

            PLCCounter& counter = m_CNT[cIdx];
            const int32_t presetValue = GetOperandAsInt(inst.op2);
            const int32_t preset = NormalizePositivePreset(presetValue);
            const bool previous = task.edgeMemory[instIndex] != 0;
            const bool rising = ACC && !previous;

            if (!counter.initialized) {
                counter.acc = 0;
                counter.initialized = true;
            }

            counter.mode = 1;
            counter.preset = preset;

            if (rising && counter.acc < INT32_MAX) {
                ++counter.acc;
            }

            counter.done = counter.acc >= counter.preset;
            counter.zero = counter.acc <= 0;
            task.edgeMemory[instIndex] = ACC ? 1 : 0;
            break;
        }

        case 61: // CTD - Counter Down, count on ACC rising edge
        {
            const int cIdx = inst.op1.payload.address;
            if (!IsValidCounterIndex(cIdx)) {
                RecordRuntimeFault(PLCRuntimeFaultCode::InvalidCounterIndex, inst.op1.region, cIdx);
                break;
            }

            PLCCounter& counter = m_CNT[cIdx];
            const int32_t presetValue = GetOperandAsInt(inst.op2);
            const int32_t preset = NormalizePositivePreset(presetValue);
            const bool previous = task.edgeMemory[instIndex] != 0;
            const bool rising = ACC && !previous;

            // Standalone CTD starts from Preset after Init/CNT_RST.
            // Repeated scans do not reload the counter.
            if (!counter.initialized) {
                counter.acc = preset;
                counter.initialized = true;
            }

            counter.mode = 2;
            counter.preset = preset;

            if (rising && counter.acc > 0) {
                --counter.acc;
            }

            counter.done = counter.acc <= 0;
            counter.zero = counter.acc <= 0;
            task.edgeMemory[instIndex] = ACC ? 1 : 0;
            break;
        }


        case 62: // CTUD_CU - visible CTUD phase 1: CU=current Ladder ACC, op1=CNT, op2=PV
        {
            const int cIdx = inst.op1.payload.address;
            if (!IsValidCounterIndex(cIdx)) {
                RecordRuntimeFault(PLCRuntimeFaultCode::InvalidCounterIndex, inst.op1.region, cIdx);
                break;
            }

            PLCCounter& counter = m_CNT[cIdx];
            const int32_t presetValue = GetOperandAsInt(inst.op2);
            const int32_t preset = NormalizePositivePreset(presetValue);
            const bool previousCU = task.edgeMemory[instIndex] != 0;
            const bool risingCU = ACC && !previousCU;

            if (!counter.initialized) {
                counter.acc = 0;
                counter.initialized = true;
            }

            counter.mode = 3;
            counter.preset = preset;
            if (risingCU && counter.acc < INT32_MAX) ++counter.acc;

            counter.done = counter.acc >= counter.preset; // QU
            counter.zero = counter.acc <= 0;              // QD
            task.edgeMemory[instIndex] = ACC ? 1 : 0;
            break;
        }

        case 80: // CTUD_CD - compiler-internal phase 2: op1=CNT, op2=CD BOOL
        {
            const int cIdx = inst.op1.payload.address;
            if (!IsValidCounterIndex(cIdx)) {
                RecordRuntimeFault(PLCRuntimeFaultCode::InvalidCounterIndex, inst.op1.region, cIdx);
                break;
            }

            PLCCounter& counter = m_CNT[cIdx];
            const bool cd = GetOperandAsBool(inst.op2);
            const bool previousCD = task.edgeMemory[instIndex] != 0;
            const bool risingCD = cd && !previousCD;

            if (!counter.initialized) {
                counter.acc = 0;
                counter.initialized = true;
            }

            counter.mode = 3;
            if (risingCD && counter.acc > INT32_MIN) --counter.acc;

            counter.done = counter.acc >= counter.preset; // QU
            counter.zero = counter.acc <= 0;              // QD
            task.edgeMemory[instIndex] = cd ? 1 : 0;
            break;
        }

        case 81: // CTUD_RST - compiler-internal phase 3: op1=CNT, op2=RST BOOL
        {
            const int cIdx = inst.op1.payload.address;
            if (!IsValidCounterIndex(cIdx)) {
                RecordRuntimeFault(PLCRuntimeFaultCode::InvalidCounterIndex, inst.op1.region, cIdx);
                break;
            }

            PLCCounter& counter = m_CNT[cIdx];
            if (GetOperandAsBool(inst.op2)) {
                // Reset is emitted last, therefore it has highest priority.
                counter.acc = 0;
                counter.done = false;
                counter.zero = true;
                counter.initialized = true;
                counter.mode = 3;
            }
            break;
        }

        case 83: // LDCNT_QD - QD = CV <= 0
        {
            const int cIdx = inst.op1.payload.address;
            if (!IsValidCounterIndex(cIdx)) {
                RecordRuntimeFault(PLCRuntimeFaultCode::InvalidCounterIndex, inst.op1.region, cIdx);
                ACC = false;
                break;
            }
            ACC = m_CNT[cIdx].zero;
            break;
        }

        case 84: // LDICNT_QD - inverted QD
        {
            const int cIdx = inst.op1.payload.address;
            if (!IsValidCounterIndex(cIdx)) {
                RecordRuntimeFault(PLCRuntimeFaultCode::InvalidCounterIndex, inst.op1.region, cIdx);
                ACC = false;
                break;
            }
            ACC = !m_CNT[cIdx].zero;
            break;
        }

        case 63: // CNT_RST - Counter Reset
        {
            const int cIdx = inst.op1.payload.address;
            if (!IsValidCounterIndex(cIdx)) {
                RecordRuntimeFault(PLCRuntimeFaultCode::InvalidCounterIndex, inst.op1.region, cIdx);
                break;
            }
            if (ACC) ResetCounterRuntimeState(m_CNT[cIdx]);
            break;
        }

        case 70: // R_TRIG - transform ACC rising edge into a one-task-scan pulse
        {
            const bool previous = task.edgeMemory[instIndex] != 0;
            const bool current = ACC;
            ACC = current && !previous;
            task.edgeMemory[instIndex] = current ? 1 : 0;
            break;
        }

        case 71: // F_TRIG - transform ACC falling edge into a one-task-scan pulse
        {
            const bool previous = task.edgeMemory[instIndex] != 0;
            const bool current = ACC;
            ACC = !current && previous;
            task.edgeMemory[instIndex] = current ? 1 : 0;
            break;
        }


        // =========================================================
        // V7.6.1 PLC System Contacts
        //
        // These are zero-operand condition instructions.
        // They do NOT add a memory region and do NOT modify PLCInstruction.
        //
        // Clock contract:
        // PLC_RunCount advances once per RunCycle().
        // The controller integration calls RunCycle(1) every 1ms.
        // 10ms clock  = 5ms ON / 5ms OFF
        // 100ms clock = 50ms ON / 50ms OFF
        // 1s clock    = 500ms ON / 500ms OFF
        // =========================================================
        case 98: // SYS_ALWAYS_ON
            ACC = true;
            break;

        case 99: // SYS_ALWAYS_OFF
            ACC = false;
            break;

        case 100: // SYS_FIRST_SCAN - first execution of each task after Load/Reload
            ACC = taskFirstScan;
            break;

        case 101: // SYS_CLK_10MS - 10ms full period, 50% duty
            ACC = (PLC_RunCount % 10u) < 5u;
            break;

        case 102: // SYS_CLK_100MS - 100ms full period, 50% duty
            ACC = (PLC_RunCount % 100u) < 50u;
            break;

        case 103: // SYS_CLK_1S - 1000ms full period, 50% duty
            ACC = (PLC_RunCount % 1000u) < 500u;
            break;


            // =========================================================
            // V7.6.2 Process Math
            //
            // SCALE Target InLow InHigh OutLow OutHigh
            // -> 104 Target/InLow
            // -> 105 InHigh/OutLow
            // -> 106 Target/OutHigh
            //
            // SCALE_LIMIT shares 105/106 but starts with 107.
            // Target is written only in COMMIT so invalid parameters cannot
            // leave a partially transformed value.
            // =========================================================
        case 104: // SCALE_BEGIN
        case 107: // SCALE_LIMIT_BEGIN
        {
            scaleSequenceValid = false;
            scaleClampInput = (inst.opCode == 107);

            if (!ACC)
                break;

            if (!IsNumericWritableOperand(inst.op1) ||
                !IsNumericReadableOperand(inst.op2))
            {
                const PLCOperand& bad =
                    !IsNumericWritableOperand(inst.op1) ? inst.op1 : inst.op2;
                RecordRuntimeFault(
                    PLCRuntimeFaultCode::InvalidOperand,
                    bad.region,
                    bad.payload.address);
                break;
            }

            const double source = GetOperandAsDouble(inst.op1);
            const double inputLow = GetOperandAsDouble(inst.op2);

            if (!std::isfinite(source) || !std::isfinite(inputLow))
            {
                RecordRuntimeFault(
                    PLCRuntimeFaultCode::ArithmeticDomain,
                    inst.op1.region,
                    inst.op1.payload.address);
                break;
            }

            scaleInputValue = source;
            scaleInputLow = inputLow;
            scaleSequenceValid = true;
            break;
        }

        case 105: // SCALE_RANGE: InHigh, OutLow
        {
            if (!ACC || !scaleSequenceValid)
                break;

            if (!IsNumericReadableOperand(inst.op1) ||
                !IsNumericReadableOperand(inst.op2))
            {
                const PLCOperand& bad =
                    !IsNumericReadableOperand(inst.op1) ? inst.op1 : inst.op2;
                RecordRuntimeFault(
                    PLCRuntimeFaultCode::InvalidOperand,
                    bad.region,
                    bad.payload.address);
                scaleSequenceValid = false;
                break;
            }

            const double inputHigh = GetOperandAsDouble(inst.op1);
            const double outputLow = GetOperandAsDouble(inst.op2);

            if (!std::isfinite(inputHigh) ||
                !std::isfinite(outputLow) ||
                inputHigh <= scaleInputLow)
            {
                RecordRuntimeFault(
                    PLCRuntimeFaultCode::ArithmeticDomain,
                    inst.op1.region,
                    inst.op1.payload.address);
                scaleSequenceValid = false;
                break;
            }

            scaleInputHigh = inputHigh;
            scaleOutputLow = outputLow;
            break;
        }

        case 106: // SCALE_COMMIT: Target, OutHigh
        {
            if (!ACC || !scaleSequenceValid)
            {
                scaleSequenceValid = false;
                break;
            }

            if (!IsNumericWritableOperand(inst.op1) ||
                !IsNumericReadableOperand(inst.op2))
            {
                const PLCOperand& bad =
                    !IsNumericWritableOperand(inst.op1) ? inst.op1 : inst.op2;
                RecordRuntimeFault(
                    PLCRuntimeFaultCode::InvalidOperand,
                    bad.region,
                    bad.payload.address);
                scaleSequenceValid = false;
                break;
            }

            const double outputHigh = GetOperandAsDouble(inst.op2);
            if (!std::isfinite(outputHigh))
            {
                RecordRuntimeFault(
                    PLCRuntimeFaultCode::ArithmeticDomain,
                    inst.op2.region,
                    inst.op2.payload.address);
                scaleSequenceValid = false;
                break;
            }

            double source = scaleInputValue;
            if (scaleClampInput)
            {
                source = (std::max)(
                    scaleInputLow,
                    (std::min)(source, scaleInputHigh));
            }

            const double inputSpan = scaleInputHigh - scaleInputLow;
            const double outputSpan = outputHigh - scaleOutputLow;
            const double normalized =
                (source - scaleInputLow) / inputSpan;
            const double result =
                scaleOutputLow + normalized * outputSpan;

            if (!std::isfinite(result))
            {
                RecordRuntimeFault(
                    PLCRuntimeFaultCode::ArithmeticDomain,
                    inst.op1.region,
                    inst.op1.payload.address);
                scaleSequenceValid = false;
                break;
            }

            SetOperandFromDouble(inst.op1, result);
            scaleSequenceValid = false;
            break;
        }

        case 108: // SQRT
            if (ACC)
            {
                if (!IsNumericWritableOperand(inst.op1))
                {
                    RecordRuntimeFault(
                        PLCRuntimeFaultCode::InvalidOperand,
                        inst.op1.region,
                        inst.op1.payload.address);
                    break;
                }

                const double value = GetOperandAsDouble(inst.op1);
                if (!std::isfinite(value) || value < 0.0)
                {
                    RecordRuntimeFault(
                        PLCRuntimeFaultCode::ArithmeticDomain,
                        inst.op1.region,
                        inst.op1.payload.address);
                    break;
                }

                SetOperandFromDouble(inst.op1, std::sqrt(value));
            }
            break;

        case 109: // ROUND - halfway away from zero
            if (ACC)
            {
                if (!IsNumericWritableOperand(inst.op1))
                {
                    RecordRuntimeFault(
                        PLCRuntimeFaultCode::InvalidOperand,
                        inst.op1.region,
                        inst.op1.payload.address);
                    break;
                }

                const double value = GetOperandAsDouble(inst.op1);
                if (!std::isfinite(value))
                {
                    RecordRuntimeFault(
                        PLCRuntimeFaultCode::ArithmeticDomain,
                        inst.op1.region,
                        inst.op1.payload.address);
                    break;
                }

                SetOperandFromDouble(inst.op1, std::round(value));
            }
            break;

        case 110: // TRUNC
            if (ACC)
            {
                if (!IsNumericWritableOperand(inst.op1))
                {
                    RecordRuntimeFault(
                        PLCRuntimeFaultCode::InvalidOperand,
                        inst.op1.region,
                        inst.op1.payload.address);
                    break;
                }

                const double value = GetOperandAsDouble(inst.op1);
                if (!std::isfinite(value))
                {
                    RecordRuntimeFault(
                        PLCRuntimeFaultCode::ArithmeticDomain,
                        inst.op1.region,
                        inst.op1.payload.address);
                    break;
                }

                SetOperandFromDouble(inst.op1, std::trunc(value));
            }
            break;

        case 111: // FLOOR
            if (ACC)
            {
                if (!IsNumericWritableOperand(inst.op1))
                {
                    RecordRuntimeFault(
                        PLCRuntimeFaultCode::InvalidOperand,
                        inst.op1.region,
                        inst.op1.payload.address);
                    break;
                }

                const double value = GetOperandAsDouble(inst.op1);
                if (!std::isfinite(value))
                {
                    RecordRuntimeFault(
                        PLCRuntimeFaultCode::ArithmeticDomain,
                        inst.op1.region,
                        inst.op1.payload.address);
                    break;
                }

                SetOperandFromDouble(inst.op1, std::floor(value));
            }
            break;

        case 112: // CEIL
            if (ACC)
            {
                if (!IsNumericWritableOperand(inst.op1))
                {
                    RecordRuntimeFault(
                        PLCRuntimeFaultCode::InvalidOperand,
                        inst.op1.region,
                        inst.op1.payload.address);
                    break;
                }

                const double value = GetOperandAsDouble(inst.op1);
                if (!std::isfinite(value))
                {
                    RecordRuntimeFault(
                        PLCRuntimeFaultCode::ArithmeticDomain,
                        inst.op1.region,
                        inst.op1.payload.address);
                    break;
                }

                SetOperandFromDouble(inst.op1, std::ceil(value));
            }
            break;

            // =========================================================
            // V7.4.9.1 True Ladder Logic Core - internal FLOW opcodes
            // 72..79 are compiler-internal; they are not user PLC instructions.
            // =========================================================
        case 72: // FLOW_ROW_TRUE  op1=#row
        {
            const int row = GetOperandAsInt(inst.op1);
            if (ensureFlowRow(row)) task.flowRows[row] = 1;
            break;
        }

        case 73: // FLOW_ROW_FALSE op1=#row
        {
            const int row = GetOperandAsInt(inst.op1);
            if (ensureFlowRow(row)) task.flowRows[row] = 0;
            break;
        }

        case 74: // FLOW_AND op1=BOOL operand, op2=#row
        {
            const int row = GetOperandAsInt(inst.op2);
            if (ensureFlowRow(row))
                task.flowRows[row] = (task.flowRows[row] != 0 && GetOperandAsBool(inst.op1)) ? 1 : 0;
            break;
        }

        case 75: // FLOW_AND_NOT op1=BOOL operand, op2=#row
        {
            const int row = GetOperandAsInt(inst.op2);
            if (ensureFlowRow(row))
                task.flowRows[row] = (task.flowRows[row] != 0 && !GetOperandAsBool(inst.op1)) ? 1 : 0;
            break;
        }

        case 76: // FLOW_LOAD_ROW op1=#row
        {
            const int row = GetOperandAsInt(inst.op1);
            ACC = ensureFlowRow(row) ? (task.flowRows[row] != 0) : false;
            break;
        }

        case 77: // FLOW_OR_ROW op1=#row
        {
            const int row = GetOperandAsInt(inst.op1);
            if (ensureFlowRow(row)) ACC = ACC || (task.flowRows[row] != 0);
            break;
        }

        case 78: // FLOW_STORE_ROW op1=#row
        {
            const int row = GetOperandAsInt(inst.op1);
            if (ensureFlowRow(row)) task.flowRows[row] = ACC ? 1 : 0;
            break;
        }

        case 79: // FLOW_AND_ACC op1=#row: gate current ACC by incoming row power
        {
            const int row = GetOperandAsInt(inst.op1);
            ACC = ensureFlowRow(row) ? (ACC && task.flowRows[row] != 0) : false;
            break;
        }

        case 20: // ADD
        case 21: // SUB
        case 22: // MUL
            if (ACC) {
                if (!IsNumericWritableOperand(inst.op1) ||
                    !IsNumericReadableOperand(inst.op2)) {
                    const PLCOperand& bad =
                        !IsNumericWritableOperand(inst.op1) ? inst.op1 : inst.op2;
                    RecordRuntimeFault(PLCRuntimeFaultCode::InvalidOperand, bad.region, bad.payload.address);
                    break;
                }

                const double left = GetOperandAsDouble(inst.op1);
                const double right = GetOperandAsDouble(inst.op2);
                const double result =
                    inst.opCode == 20 ? (left + right) :
                    inst.opCode == 21 ? (left - right) :
                    (left * right);

                SetOperandFromDouble(inst.op1, result);
            }
            break;

        case 23: // DIV
            if (ACC) {
                if (!IsNumericWritableOperand(inst.op1) ||
                    !IsNumericReadableOperand(inst.op2)) {
                    const PLCOperand& bad =
                        !IsNumericWritableOperand(inst.op1) ? inst.op1 : inst.op2;
                    RecordRuntimeFault(PLCRuntimeFaultCode::InvalidOperand, bad.region, bad.payload.address);
                    break;
                }

                const double divisor = GetOperandAsDouble(inst.op2);
                if (divisor != 0.0) {
                    SetOperandFromDouble(inst.op1,
                        GetOperandAsDouble(inst.op1) / divisor);
                }
                else {
                    RecordRuntimeFault(
                        PLCRuntimeFaultCode::ArithmeticDomain,
                        inst.op2.region,
                        inst.op2.payload.address);
                }
            }
            break;

        case 24: // MOV
            if (ACC) {
                if (!IsNumericWritableOperand(inst.op1) ||
                    !IsNumericReadableOperand(inst.op2)) {
                    const PLCOperand& bad =
                        !IsNumericWritableOperand(inst.op1) ? inst.op1 : inst.op2;
                    RecordRuntimeFault(PLCRuntimeFaultCode::InvalidOperand, bad.region, bad.payload.address);
                    break;
                }
                SetOperandFromDouble(inst.op1, GetOperandAsDouble(inst.op2));
            }
            break;

        case 25: // MOD
            if (ACC) {
                if (!IsIntegerWritableOperand(inst.op1) ||
                    !IsIntegerReadableOperand(inst.op2)) {
                    const PLCOperand& bad =
                        !IsIntegerWritableOperand(inst.op1) ? inst.op1 : inst.op2;
                    RecordRuntimeFault(PLCRuntimeFaultCode::InvalidOperand, bad.region, bad.payload.address);
                    break;
                }

                const int32_t divisor = GetOperandAsInt(inst.op2);
                if (divisor != 0) {
                    SetOperandFromInt(inst.op1, GetOperandAsInt(inst.op1) % divisor);
                }
                else {
                    RecordRuntimeFault(
                        PLCRuntimeFaultCode::ArithmeticDomain,
                        inst.op2.region,
                        inst.op2.payload.address);
                }
            }
            break;

        case 26: // ABS
            if (ACC) {
                if (!IsNumericWritableOperand(inst.op1)) {
                    RecordRuntimeFault(PLCRuntimeFaultCode::InvalidOperand, inst.op1.region, inst.op1.payload.address);
                    break;
                }
                const double value = GetOperandAsDouble(inst.op1);
                SetOperandFromDouble(inst.op1, std::fabs(value));
            }
            break;

        case 27: // NEG
            if (ACC) {
                if (!IsNumericWritableOperand(inst.op1)) {
                    RecordRuntimeFault(PLCRuntimeFaultCode::InvalidOperand, inst.op1.region, inst.op1.payload.address);
                    break;
                }
                SetOperandFromDouble(inst.op1, -GetOperandAsDouble(inst.op1));
            }
            break;

        case 30: // CMP >
            ACC = (GetOperandAsDouble(inst.op1) > GetOperandAsDouble(inst.op2));
            break;

        case 31: // CMP <
            ACC = (GetOperandAsDouble(inst.op1) < GetOperandAsDouble(inst.op2));
            break;

        case 32: // CMP =
            ACC = (GetOperandAsDouble(inst.op1) == GetOperandAsDouble(inst.op2));
            break;

        case 33: // CMP >=
            ACC = (GetOperandAsDouble(inst.op1) >= GetOperandAsDouble(inst.op2));
            break;

        case 34: // CMP <=
            ACC = (GetOperandAsDouble(inst.op1) <= GetOperandAsDouble(inst.op2));
            break;

        case 35: // CMP !=
            ACC = (GetOperandAsDouble(inst.op1) != GetOperandAsDouble(inst.op2));
            break;

        case 40: // AND
        case 41: // LOGIC_OR
        case 42: // XOR
            if (ACC) {
                if (!IsIntegerWritableOperand(inst.op1) ||
                    !IsIntegerReadableOperand(inst.op2)) {
                    const PLCOperand& bad =
                        !IsIntegerWritableOperand(inst.op1) ? inst.op1 : inst.op2;
                    RecordRuntimeFault(PLCRuntimeFaultCode::InvalidOperand, bad.region, bad.payload.address);
                    break;
                }

                const int32_t left = GetOperandAsInt(inst.op1);
                const int32_t right = GetOperandAsInt(inst.op2);
                const int32_t result =
                    inst.opCode == 40 ? (left & right) :
                    inst.opCode == 41 ? (left | right) :
                    (left ^ right);

                SetOperandFromInt(inst.op1, result);
            }
            break;


            // =========================================================
            // V7.5.4 Advanced PLC Instruction Pack
            // =========================================================

        case 85: // MIN destination, source
            if (ACC) {
                if (!IsNumericWritableOperand(inst.op1) ||
                    !IsNumericReadableOperand(inst.op2)) {
                    const PLCOperand& bad =
                        !IsNumericWritableOperand(inst.op1) ? inst.op1 : inst.op2;
                    RecordRuntimeFault(PLCRuntimeFaultCode::InvalidOperand, bad.region, bad.payload.address);
                    break;
                }

                const double current = GetOperandAsDouble(inst.op1);
                const double candidate = GetOperandAsDouble(inst.op2);
                SetOperandFromDouble(inst.op1, (std::min)(current, candidate));
            }
            break;

        case 86: // MAX destination, source
            if (ACC) {
                if (!IsNumericWritableOperand(inst.op1) ||
                    !IsNumericReadableOperand(inst.op2)) {
                    const PLCOperand& bad =
                        !IsNumericWritableOperand(inst.op1) ? inst.op1 : inst.op2;
                    RecordRuntimeFault(PLCRuntimeFaultCode::InvalidOperand, bad.region, bad.payload.address);
                    break;
                }

                const double current = GetOperandAsDouble(inst.op1);
                const double candidate = GetOperandAsDouble(inst.op2);
                SetOperandFromDouble(inst.op1, (std::max)(current, candidate));
            }
            break;

        case 87: // LIMIT_LOW destination, lower bound
            if (ACC) {
                if (!IsNumericWritableOperand(inst.op1) ||
                    !IsNumericReadableOperand(inst.op2)) {
                    const PLCOperand& bad =
                        !IsNumericWritableOperand(inst.op1) ? inst.op1 : inst.op2;
                    RecordRuntimeFault(PLCRuntimeFaultCode::InvalidOperand, bad.region, bad.payload.address);
                    break;
                }

                const double current = GetOperandAsDouble(inst.op1);
                const double lower = GetOperandAsDouble(inst.op2);
                if (current < lower)
                    SetOperandFromDouble(inst.op1, lower);
            }
            break;

        case 88: // LIMIT_HIGH destination, upper bound (compiler internal)
            if (ACC) {
                if (!IsNumericWritableOperand(inst.op1) ||
                    !IsNumericReadableOperand(inst.op2)) {
                    const PLCOperand& bad =
                        !IsNumericWritableOperand(inst.op1) ? inst.op1 : inst.op2;
                    RecordRuntimeFault(PLCRuntimeFaultCode::InvalidOperand, bad.region, bad.payload.address);
                    break;
                }

                const double current = GetOperandAsDouble(inst.op1);
                const double upper = GetOperandAsDouble(inst.op2);
                if (current > upper)
                    SetOperandFromDouble(inst.op1, upper);
            }
            break;

        case 89: // INC destination
            if (ACC) {
                if (!IsNumericWritableOperand(inst.op1)) {
                    RecordRuntimeFault(PLCRuntimeFaultCode::InvalidOperand, inst.op1.region, inst.op1.payload.address);
                    break;
                }

                SetOperandFromDouble(
                    inst.op1,
                    GetOperandAsDouble(inst.op1) + 1.0);
            }
            break;

        case 90: // DEC destination
            if (ACC) {
                if (!IsNumericWritableOperand(inst.op1)) {
                    RecordRuntimeFault(PLCRuntimeFaultCode::InvalidOperand, inst.op1.region, inst.op1.payload.address);
                    break;
                }

                SetOperandFromDouble(
                    inst.op1,
                    GetOperandAsDouble(inst.op1) - 1.0);
            }
            break;

        case 91: // SHL destination, bit count (logical left shift)
        case 92: // SHR destination, bit count (logical right shift)
        {
            if (!ACC) break;

            if (!IsIntegerWritableOperand(inst.op1) ||
                !IsIntegerReadableOperand(inst.op2)) {
                const PLCOperand& bad =
                    !IsIntegerWritableOperand(inst.op1) ? inst.op1 : inst.op2;
                RecordRuntimeFault(PLCRuntimeFaultCode::InvalidOperand, bad.region, bad.payload.address);
                break;
            }

            const int32_t bitCount = GetOperandAsInt(inst.op2);
            if (bitCount < 0 || bitCount > 31) {
                RecordRuntimeFault(PLCRuntimeFaultCode::InvalidBitIndex, inst.op2.region, bitCount);
                break;
            }

            const uint32_t value =
                static_cast<uint32_t>(GetOperandAsInt(inst.op1));

            const uint32_t result =
                inst.opCode == 91
                ? (value << static_cast<uint32_t>(bitCount))
                : (value >> static_cast<uint32_t>(bitCount));

            SetOperandFromInt(inst.op1, static_cast<int32_t>(result));
            break;
        }

        case 93: // BIT_TEST source, bit index -> ACC
        {
            if (!IsIntegerReadableOperand(inst.op1) ||
                !IsIntegerReadableOperand(inst.op2)) {
                const PLCOperand& bad =
                    !IsIntegerReadableOperand(inst.op1) ? inst.op1 : inst.op2;
                RecordRuntimeFault(PLCRuntimeFaultCode::InvalidOperand, bad.region, bad.payload.address);
                ACC = false;
                break;
            }

            const int32_t bitIndex = GetOperandAsInt(inst.op2);
            if (bitIndex < 0 || bitIndex > 31) {
                RecordRuntimeFault(PLCRuntimeFaultCode::InvalidBitIndex, inst.op2.region, bitIndex);
                ACC = false;
                break;
            }

            const uint32_t value =
                static_cast<uint32_t>(GetOperandAsInt(inst.op1));
            ACC = ((value >> static_cast<uint32_t>(bitIndex)) & 0x1u) != 0;
            break;
        }

        case 94: // BIT_SET destination, bit index
        case 95: // BIT_RESET destination, bit index
        {
            if (!ACC) break;

            if (!IsIntegerWritableOperand(inst.op1) ||
                !IsIntegerReadableOperand(inst.op2)) {
                const PLCOperand& bad =
                    !IsIntegerWritableOperand(inst.op1) ? inst.op1 : inst.op2;
                RecordRuntimeFault(PLCRuntimeFaultCode::InvalidOperand, bad.region, bad.payload.address);
                break;
            }

            const int32_t bitIndex = GetOperandAsInt(inst.op2);
            if (bitIndex < 0 || bitIndex > 31) {
                RecordRuntimeFault(PLCRuntimeFaultCode::InvalidBitIndex, inst.op2.region, bitIndex);
                break;
            }

            uint32_t value =
                static_cast<uint32_t>(GetOperandAsInt(inst.op1));
            const uint32_t mask =
                (1u << static_cast<uint32_t>(bitIndex));

            if (inst.opCode == 94)
                value |= mask;
            else
                value &= ~mask;

            SetOperandFromInt(inst.op1, static_cast<int32_t>(value));
            break;
        }

        case 96: // RANGE_LOW value, lower -> ACC = value >= lower
            if (!IsNumericReadableOperand(inst.op1) ||
                !IsNumericReadableOperand(inst.op2)) {
                const PLCOperand& bad =
                    !IsNumericReadableOperand(inst.op1) ? inst.op1 : inst.op2;
                RecordRuntimeFault(PLCRuntimeFaultCode::InvalidOperand, bad.region, bad.payload.address);
                ACC = false;
                break;
            }
            ACC = GetOperandAsDouble(inst.op1) >= GetOperandAsDouble(inst.op2);
            break;

        case 97: // RANGE_HIGH value, upper -> ACC = ACC && value <= upper
            if (!IsNumericReadableOperand(inst.op1) ||
                !IsNumericReadableOperand(inst.op2)) {
                const PLCOperand& bad =
                    !IsNumericReadableOperand(inst.op1) ? inst.op1 : inst.op2;
                RecordRuntimeFault(PLCRuntimeFaultCode::InvalidOperand, bad.region, bad.payload.address);
                ACC = false;
                break;
            }
            ACC = ACC &&
                (GetOperandAsDouble(inst.op1) <= GetOperandAsDouble(inst.op2));
            break;

        default:
            RecordRuntimeFault(PLCRuntimeFaultCode::UnknownOpcode);
            break;
        }
    }

    // Only a successfully executed task consumes its First Scan state.
    task.firstScanPending = false;

    m_activeInstructionIndex = -1;
    m_activeOpcode = 0;
    m_activeTaskIndex = -1;
}


// =========================================================
// Original Direct Hardware API
// =========================================================
void PLCManager::SetBit_I(int index, bool value) { if (index >= 0 && index < MAX_PLC_I) m_I[index] = value ? 1 : 0; }
bool PLCManager::GetBit_I(int index) const { if (index >= 0 && index < MAX_PLC_I) return m_I[index] != 0; return false; }

void PLCManager::SetBit_O(int index, bool value) { if (index >= 0 && index < MAX_PLC_O) m_O[index] = value ? 1 : 0; }
bool PLCManager::GetBit_O(int index) const { if (index >= 0 && index < MAX_PLC_O) return m_O[index] != 0; return false; }

int32_t PLCManager::GetReg_R(int index) const { if (index >= 0 && index < MAX_PLC_R) return m_R[index]; return 0; }
void PLCManager::SetReg_R(int index, int32_t value) { if (index >= 0 && index < MAX_PLC_R) m_R[index] = value; }

int32_t PLCManager::GetReg_DR(int index) const { if (index >= 0 && index < MAX_PLC_DR) return m_DR[index]; return 0; }
void PLCManager::SetReg_DR(int index, int32_t value) { if (index >= 0 && index < MAX_PLC_DR) m_DR[index] = value; }

// =========================================================
// 🌟 A 點與 S 點專屬高速讀寫 API 實作
// =========================================================
void PLCManager::Set_A(int index, bool value) {
    if (index >= 0 && index < MAX_PLC_A) {
        m_A[index] = value ? 1 : 0;
    }
}

bool PLCManager::Get_A(int index) const {
    if (index >= 0 && index < MAX_PLC_A) {
        return m_A[index] != 0;
    }
    return false;
}

void PLCManager::Set_S(int index, bool value) {
    if (index >= 0 && index < MAX_PLC_S) {
        m_S[index] = value ? 1 : 0;
    }
}

bool PLCManager::Get_S(int index) const {
    if (index >= 0 && index < MAX_PLC_S) {
        return m_S[index] != 0;
    }
    return false;
}

double PLCManager::GetMemory(const std::string& prefix, int index) const {
    if (prefix == "I" && index >= 0 && index < MAX_PLC_I) return m_I[index];
    if (prefix == "O" && index >= 0 && index < MAX_PLC_O) return m_O[index];
    if (prefix == "A" && index >= 0 && index < MAX_PLC_A) return m_A[index]; // 🌟 A 點
    if (prefix == "S" && index >= 0 && index < MAX_PLC_S) return m_S[index]; // 🌟 S 點
    if (prefix == "C" && index >= 0 && index < MAX_PLC_C) return m_C[index];
    if (prefix == "R" && index >= 0 && index < MAX_PLC_R) return m_R[index];
    if (prefix == "DR" && index >= 0 && index < MAX_PLC_DR) return m_DR[index];
    if (prefix == "F" && index >= 0 && index < MAX_PLC_F) return m_F[index];
    if (prefix == "L" && index >= 0 && index < MAX_PLC_L) return m_L[index];
    if (prefix == "T" && index >= 0 && index < MAX_PLC_T) return m_T[index].done ? 1.0 : 0.0;
    if (prefix == "CNT" && index >= 0 && index < MAX_PLC_CNT) return static_cast<double>(m_CNT[index].acc);
    return 0.0;
}

void PLCManager::SetMemory(const std::string& prefix, int index, double value) {
    if (prefix == "I" && index >= 0 && index < MAX_PLC_I) m_I[index] = (value != 0);
    else if (prefix == "O" && index >= 0 && index < MAX_PLC_O) m_O[index] = (value != 0);
    else if (prefix == "A" && index >= 0 && index < MAX_PLC_A) m_A[index] = (value != 0); // 🌟 A 點
    else if (prefix == "S" && index >= 0 && index < MAX_PLC_S) m_S[index] = (value != 0); // 🌟 S 點
    else if (prefix == "C" && index >= 0 && index < MAX_PLC_C) m_C[index] = (value != 0);
    else if (prefix == "R" && index >= 0 && index < MAX_PLC_R) m_R[index] = ClampDoubleToInt32(value);
    else if (prefix == "DR" && index >= 0 && index < MAX_PLC_DR) m_DR[index] = ClampDoubleToInt32(value);
    else if (prefix == "F" && index >= 0 && index < MAX_PLC_F) m_F[index] = ClampDoubleToFloat(value);
    else if (prefix == "L" && index >= 0 && index < MAX_PLC_L) {
        if (!std::isfinite(value)) {
            RecordRuntimeFault(PLCRuntimeFaultCode::ArithmeticDomain);
            m_L[index] = 0.0;
        }
        else {
            m_L[index] = value;
        }
    }
}

// =========================================================
// 🌟 補上 GetVar 與 SetVar 的字串查詢實作
// =========================================================
double PLCManager::GetVar(const std::string& name) const {
    if (name.empty()) return 0.0;

    // 1. 嘗試以自定義變數 (VAR) 名稱查詢
    int32_t hashId = GetStableHashCpp(name);
    // 🌟 字串查詢時也換成這把鎖
    AutoLockCS lock(&m_logicCS);
    auto it = m_customVars.find(hashId);
    if (it != m_customVars.end()) {
        const auto& var = it->second;
        if (var.dataType == 4) return var.value.fVal;
        if (var.dataType == 5) return var.value.dVal;
        return var.value.iVal;
    }

    // 2. 嘗試解析標準硬體點位格式 (例如 "R10", "I2", "DR50")
    std::string prefix = "";
    int index = 0;

    if (name.rfind("CNT", 0) == 0) {
        prefix = "CNT";
        index = std::stoi(name.substr(3));
    }
    else if (name.rfind("DR", 0) == 0) {
        prefix = "DR";
        index = std::stoi(name.substr(2));
    }
    else {
        prefix = name.substr(0, 1);
        index = std::stoi(name.substr(1));
    }

    return GetMemory(prefix, index);
}

void PLCManager::SetVar(const std::string& name, double value) {
    if (name.empty()) return;

    // 1. 嘗試以自定義變數 (VAR) 名稱寫入
    int32_t hashId = GetStableHashCpp(name);
    AutoLockCS lock(&m_logicCS);
    auto it = m_customVars.find(hashId);
    if (it != m_customVars.end()) {
        auto& var = it->second;
        if (var.dataType == 4)
            var.value.fVal = ClampDoubleToFloat(value);
        else if (var.dataType == 5)
        {
            if (!std::isfinite(value)) {
                RecordRuntimeFault(PLCRuntimeFaultCode::ArithmeticDomain);
                var.value.dVal = 0.0;
            }
            else {
                var.value.dVal = value;
            }
        }
        else
            var.value.iVal = ClampDoubleToInt32(value);
        return;
    }

    // 2. 嘗試解析標準硬體點位格式寫入
    std::string prefix = "";
    int index = 0;

    if (name.rfind("CNT", 0) == 0) {
        prefix = "CNT";
        index = std::stoi(name.substr(3));
    }
    else if (name.rfind("DR", 0) == 0) {
        prefix = "DR";
        index = std::stoi(name.substr(2));
    }
    else {
        prefix = name.substr(0, 1);
        index = std::stoi(name.substr(1));
    }

    SetMemory(prefix, index, value);
}

int32_t PLCManager::GetStableHashCpp(const std::string& str) const {
    int32_t hash = 23;
    for (char c : str) {
        char upperC = static_cast<char>(toupper(c));
        hash = hash * 31 + upperC;
    }
    return hash;
}

// =========================================================
// 🌟 匯出全域狀態給 HMI (極速記憶體拷貝)
// =========================================================
void PLCManager::ExportPLCStatus(SHM_PLC_Status* pStatus) const
{
    if (!pStatus) return;

    // 直接拷貝標準陣列
    std::memcpy(pStatus->I, m_I, sizeof(m_I));
    std::memcpy(pStatus->O, m_O, sizeof(m_O));
    std::memcpy(pStatus->A, m_A, sizeof(m_A));
    std::memcpy(pStatus->S, m_S, sizeof(m_S));
    std::memcpy(pStatus->C, m_C, sizeof(m_C));
    std::memcpy(pStatus->R, m_R, sizeof(m_R));
    std::memcpy(pStatus->DR, m_DR, sizeof(m_DR));

    // Timer 因為是 Struct，我們把它拆解放入對應的陣列給人機
    for (int i = 0; i < MAX_PLC_T; i++) {
        pStatus->T_acc[i] = m_T[i].acc;
        pStatus->T_preset[i] = m_T[i].preset;
        pStatus->T_done[i] = m_T[i].done ? 1 : 0;
        pStatus->T_base[i] = m_T[i].timeBase;
    }

    // V7.5.0: restore the Counter Online export chain that V7.4.8 Studio/API expects.
    // SHM layout itself is NOT changed here.
    for (int i = 0; i < MAX_PLC_CNT; i++) {
        pStatus->CNT_acc[i] = m_CNT[i].acc;
        pStatus->CNT_preset[i] = m_CNT[i].preset;
        pStatus->CNT_done[i] = m_CNT[i].done ? 1 : 0;
    }
}

// ============================================================================
// 🌟 動態重載 PLC 邏輯程式 (Hot-Reload) - 完整實作版
// ============================================================================
bool PLCManager::ReloadLogicProgram()
{
    const std::string filepath =
        GlobalConfig::GetInstance().PLC_Dir + "logic.bin";

    std::vector<PLCTask> tempTasks;
    std::unordered_map<int32_t, PLCCustomVar> tempVars;
    uint32_t parsedCrc32 = 0;
    uint32_t parsedFileSize = 0;

    // Slow file I/O and all vector allocations happen outside the RT lock.
    if (!ParseLogicProgramFile(
        filepath,
        tempTasks,
        tempVars,
        parsedCrc32,
        parsedFileSize))
    {
        // IMPORTANT:
        // Keep the CRC/Size/Generation of the program that is STILL RUNNING.
        MarkLogicLoadFailed();
        return false;
    }

    {
        AutoLockCS lock(&m_logicCS);

        // O(1)-style ownership swap; old containers are released after lock exit.
        m_tasks.swap(tempTasks);
        m_customVars.swap(tempVars);

        // A new program must not inherit half-completed stateful devices.
        ResetAllStatefulDevices();
        ResetRuntimeDiagnosticsUnsafe();
        ResetScanHealthUnsafe();

        // Verification metadata becomes visible in the SAME critical section
        // as the runtime program ownership swap.
        CommitLoadedLogicVerificationUnsafe(
            parsedCrc32,
            parsedFileSize);

        // I/O and R/DR memory intentionally keep their current values.
    }

    std::cout << "[PLC] Reloaded validated logic.bin. CRC32="
        << std::hex << std::uppercase
        << m_loadedLogicCrc32
        << std::dec
        << ", Size="
        << m_loadedLogicSize
        << " bytes, Generation="
        << m_logicLoadGeneration
        << std::endl;
    return true;
}

