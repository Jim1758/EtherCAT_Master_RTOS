#include "PLCManager.h"
#include <fstream>
#include <iostream>
#include <cstring>
#include <cmath>
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

    for (int i = 0; i < MAX_PLC_T; i++) {
        m_T[i].enable = false;
        m_T[i].done = false;
        m_T[i].preset = 0;
        m_T[i].acc = 0;
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

  
}

// =========================================================
// 1. Load Logic Program (Binary from C#)
// =========================================================
bool PLCManager::LoadLogicProgram(const std::string& filepath)
{
    std::ifstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "[PLC] Failed to open logic file: " << filepath << std::endl;
        return false;
    }

    char header[9] = { 0 };
    file.read(header, 8);
    if (std::string(header) != "RTOS_PLC") {
        std::cerr << "[PLC] Invalid PLC logic file format!" << std::endl;
        return false;
    }

    m_tasks.clear();
    m_customVars.clear();

    while (file.good() && !file.eof()) {
        uint8_t tag;
        file.read(reinterpret_cast<char*>(&tag), 1);
        if (file.eof()) break;

        // Parse Tag 253: Static Allocation Table for VAR
        if (tag == 253) {
            int32_t varCount;
            file.read(reinterpret_cast<char*>(&varCount), 4);

            for (int i = 0; i < varCount; i++) {
                int32_t hashId; uint8_t dataType;
                file.read(reinterpret_cast<char*>(&hashId), 4);
                file.read(reinterpret_cast<char*>(&dataType), 1);

                PLCCustomVar newVar;
                newVar.dataType = dataType;
                std::memset(newVar.value.raw, 0, 8);
                m_customVars[hashId] = newVar;
            }
            std::cout << "[PLC] Statically allocated " << varCount << " custom variable spaces.\n";
        }
        // Parse Tag 254: Task and Instructions
        else if (tag == 254) {
            PLCTask newTask;
            file.read(reinterpret_cast<char*>(&newTask.type), 1);
            file.read(reinterpret_cast<char*>(&newTask.priority), 1);
            file.read(reinterpret_cast<char*>(&newTask.cycleTimeMs), 4);

            newTask.currentTimerMs = newTask.cycleTimeMs; // Initialize timer

            while (true) {
                uint8_t nextByte = file.peek();
                if (nextByte == 253 || nextByte == 254 || nextByte == 255 || file.eof()) {
                    break;
                }
                PLCInstruction inst;
                file.read(reinterpret_cast<char*>(&inst), sizeof(PLCInstruction));
                newTask.instructions.push_back(inst);
            }
            m_tasks.push_back(newTask);
            std::cout << "[PLC] Successfully loaded Task, containing " << newTask.instructions.size() << " instructions.\n";
        }
        // Parse Tag 255: EOF
        else if (tag == 255) {
            break;
        }
    }

    file.close();

    // 🌟 依照 Task Priority 進行排序 (Priority 數字越小越優先執行)
    std::sort(m_tasks.begin(), m_tasks.end(), [](const PLCTask& a, const PLCTask& b) {
        return a.priority < b.priority;
        });

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
double PLCManager::GetOperandAsDouble(const PLCOperand& op) {
    if (op.region == 8) { // Constant (#)
        if (op.dataType == 4) return op.payload.realValue;
        if (op.dataType == 5) return op.payload.lrealValue;
        return op.payload.intValue;
    }
    if (op.region == 11) { // Custom VAR
        if (op.dataType == 4) return m_customVars[op.payload.address].value.fVal;
        if (op.dataType == 5) return m_customVars[op.payload.address].value.dVal;
        return m_customVars[op.payload.address].value.iVal;
    }

    int addr = op.payload.address;
    switch (op.region) {
    case 6: return m_R[addr];
    case 7: return m_DR[addr];
    case 9: return m_F[addr];
    case 10: return m_L[addr];
    case 0: return m_I[addr];
    case 1: return m_O[addr];
    case 2: return m_C[addr];
    default: return 0.0;
    }
}

int32_t PLCManager::GetOperandAsInt(const PLCOperand& op) {
    return static_cast<int32_t>(GetOperandAsDouble(op));
}

bool PLCManager::GetOperandAsBool(const PLCOperand& op) {
    if (op.region == 5) return m_T[op.payload.address].done; // T Timer Done
    return GetOperandAsInt(op) != 0;
}

void PLCManager::SetOperandFromDouble(const PLCOperand& op, double val) {
    if (op.region == 8) return; // Cannot write to constant

    if (op.region == 11) { // Custom VAR
        if (op.dataType == 4) m_customVars[op.payload.address].value.fVal = static_cast<float>(val);
        else if (op.dataType == 5) m_customVars[op.payload.address].value.dVal = val;
        else m_customVars[op.payload.address].value.iVal = static_cast<int32_t>(val);
        return;
    }

    int addr = op.payload.address;
    switch (op.region) {
    case 6: m_R[addr] = static_cast<int32_t>(val); break;
    case 7: m_DR[addr] = static_cast<int32_t>(val); break;
    case 9: m_F[addr] = static_cast<float>(val); break;
    case 10: m_L[addr] = val; break;
    case 1: m_O[addr] = (val != 0); break;
    case 2: m_C[addr] = (val != 0); break;
    case 3: m_S[addr] = (val != 0); break;
    case 4: m_A[addr] = (val != 0); break;
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
    if (Init_flag != 1)
    {
        return;
    }
    if (Close_flag == 1)
    {
        return;
    }
    PLC_RunCount += 1;

    // 🌟 使用我們自己寫的 AutoLockCS，傳入 m_logicCS 的記憶體位址
    AutoLockCS lock(&m_logicCS);

    // Process all Timers
    for (int i = 0; i < MAX_PLC_T; i++) {
        if (m_T[i].enable) {
            if (!m_T[i].done) {
                m_T[i].acc += delta_ms;
                if (m_T[i].acc >= m_T[i].preset) {
                    m_T[i].acc = m_T[i].preset;
                    m_T[i].done = true;
                }
            }
        }
        else {
            m_T[i].acc = 0;
            m_T[i].done = false;
        }
    }

    // RTOS Task Scheduler based on Cycle Time
    for (auto& task : m_tasks) {
        if (task.type == 1) { // Cyclic
            task.currentTimerMs += delta_ms;
            if (task.currentTimerMs >= task.cycleTimeMs) {
                task.currentTimerMs -= task.cycleTimeMs;
                ExecuteTask(task);
            }
        }
    }
}

// =========================================================
// 4. Virtual Machine: Instruction Decoding & Execution
// =========================================================
void PLCManager::ExecuteTask(PLCTask& task)
{
    bool ACC = false; // Accumulator

    for (const auto& inst : task.instructions)
    {
        switch (inst.opCode)
        {
        case 1:  // LD
            ACC = GetOperandAsBool(inst.op1); break;
        case 2:  // LDI
            ACC = !GetOperandAsBool(inst.op1); break;
        case 3:  // LDT 
            ACC = m_T[inst.op1.payload.address].done; break;
        case 4:  // LDIT 
            ACC = !m_T[inst.op1.payload.address].done; break;
        case 5:  // OR
            ACC = ACC || GetOperandAsBool(inst.op1); break;
        case 6:  // ORI
            ACC = ACC || !GetOperandAsBool(inst.op1); break;

        case 7:  // OUT
            SetOperandFromBool(inst.op1, ACC); break;
        case 8:  // OUT_NOT
            SetOperandFromBool(inst.op1, !ACC); break;
        case 11: // OUT_L (SET)
            if (ACC) SetOperandFromBool(inst.op1, true); break;
        case 12: // OUT_UL (RST)
            if (ACC) SetOperandFromBool(inst.op1, false); break;

        case 13: // TMR_10 (10ms Base)
        case 14: // TMR_100 (100ms Base)
        case 15: // TMR_1S (1s Base)
            if (ACC) {
                int tIdx = inst.op1.payload.address;
                int base = (inst.opCode == 13) ? 10 : (inst.opCode == 14) ? 100 : 1000;
                m_T[tIdx].enable = true;
                m_T[tIdx].timeBase = base; // 🌟 補上這行：記下它的時基
                m_T[tIdx].preset = GetOperandAsInt(inst.op2) * base;
            }
            else {
                m_T[inst.op1.payload.address].enable = false;
            }
            break;

        case 16: // TMR_RST 
            if (ACC) {
                m_T[inst.op1.payload.address].enable = false;
                m_T[inst.op1.payload.address].acc = 0;
                m_T[inst.op1.payload.address].done = false;
            }
            break;

        case 20: // ADD
            if (ACC) SetOperandFromDouble(inst.op1, GetOperandAsDouble(inst.op1) + GetOperandAsDouble(inst.op2)); break;
        case 21: // SUB
            if (ACC) SetOperandFromDouble(inst.op1, GetOperandAsDouble(inst.op1) - GetOperandAsDouble(inst.op2)); break;
        case 22: // MUL
            if (ACC) SetOperandFromDouble(inst.op1, GetOperandAsDouble(inst.op1) * GetOperandAsDouble(inst.op2)); break;
        case 23: // DIV
            if (ACC) {
                double div = GetOperandAsDouble(inst.op2);
                if (div != 0.0) SetOperandFromDouble(inst.op1, GetOperandAsDouble(inst.op1) / div);
            }
            break;
        case 24: // MOV 
            if (ACC) SetOperandFromDouble(inst.op1, GetOperandAsDouble(inst.op2)); break;

        case 30: // CMP >
            ACC = (GetOperandAsDouble(inst.op1) > GetOperandAsDouble(inst.op2)); break;
        case 31: // CMP <
            ACC = (GetOperandAsDouble(inst.op1) < GetOperandAsDouble(inst.op2)); break;
        case 32: // CMP =
            ACC = (GetOperandAsDouble(inst.op1) == GetOperandAsDouble(inst.op2)); break;

        case 40: // AND
            if (ACC) SetOperandFromInt(inst.op1, GetOperandAsInt(inst.op1) & GetOperandAsInt(inst.op2)); break;
        case 41: // LOGIC_OR
            if (ACC) SetOperandFromInt(inst.op1, GetOperandAsInt(inst.op1) | GetOperandAsInt(inst.op2)); break;
        case 42: // XOR
            if (ACC) SetOperandFromInt(inst.op1, GetOperandAsInt(inst.op1) ^ GetOperandAsInt(inst.op2)); break;

        default:
            break;
        }
    }
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
    return 0.0;
}

void PLCManager::SetMemory(const std::string& prefix, int index, double value) {
    if (prefix == "I" && index >= 0 && index < MAX_PLC_I) m_I[index] = (value != 0);
    else if (prefix == "O" && index >= 0 && index < MAX_PLC_O) m_O[index] = (value != 0);
    else if (prefix == "A" && index >= 0 && index < MAX_PLC_A) m_A[index] = (value != 0); // 🌟 A 點
    else if (prefix == "S" && index >= 0 && index < MAX_PLC_S) m_S[index] = (value != 0); // 🌟 S 點
    else if (prefix == "C" && index >= 0 && index < MAX_PLC_C) m_C[index] = (value != 0);
    else if (prefix == "R" && index >= 0 && index < MAX_PLC_R) m_R[index] = static_cast<int32_t>(value);
    else if (prefix == "DR" && index >= 0 && index < MAX_PLC_DR) m_DR[index] = static_cast<int32_t>(value);
    else if (prefix == "F" && index >= 0 && index < MAX_PLC_F) m_F[index] = static_cast<float>(value);
    else if (prefix == "L" && index >= 0 && index < MAX_PLC_L) m_L[index] = value;
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

    if (name.rfind("DR", 0) == 0) {
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
        if (var.dataType == 4) var.value.fVal = static_cast<float>(value);
        else if (var.dataType == 5) var.value.dVal = value;
        else var.value.iVal = static_cast<int32_t>(value);
        return;
    }

    // 2. 嘗試解析標準硬體點位格式寫入
    std::string prefix = "";
    int index = 0;

    if (name.rfind("DR", 0) == 0) {
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
        pStatus->T_base[i] = m_T[i].timeBase; // 🌟 補上這行
    }
}

// ============================================================================
// 🌟 動態重載 PLC 邏輯程式 (Hot-Reload) - 完整實作版
// ============================================================================
bool PLCManager::ReloadLogicProgram()
{
    const std::string& filepath = GlobalConfig::GetInstance().PLC_Dir + "logic.bin";
 

    // 1. 【非即時端安全區】宣告「暫時的」容器，避免在讀檔時污染運行中的記憶體
    std::vector<PLCTask> tempTasks;
    std::unordered_map<int32_t, PLCCustomVar> tempVars;

    // 開始慢慢讀取與解析檔案 (這段可能耗時幾毫秒，但不影響即時運算)
    std::ifstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        //std::cerr << "[PLC Error] 無法開啟要重載的邏輯檔: " << filepath << std::endl;
        return false;
    }

    char header[9] = { 0 };
    file.read(header, 8);
    if (std::string(header) != "RTOS_PLC") {
        //std::cerr << "[PLC Error] 無效的 PLC 邏輯檔格式!" << std::endl;
        return false;
    }

    while (file.good() && !file.eof()) {
        uint8_t tag;
        file.read(reinterpret_cast<char*>(&tag), 1);
        if (file.eof()) break;

        if (tag == 253) {
            int32_t varCount;
            file.read(reinterpret_cast<char*>(&varCount), 4);
            for (int i = 0; i < varCount; i++) {
                int32_t hashId; uint8_t dataType;
                file.read(reinterpret_cast<char*>(&hashId), 4);
                file.read(reinterpret_cast<char*>(&dataType), 1);

                PLCCustomVar newVar;
                newVar.dataType = dataType;
                std::memset(newVar.value.raw, 0, 8);
                tempVars[hashId] = newVar; // 寫入暫存容器
            }
        }
        else if (tag == 254) {
            PLCTask newTask;
            file.read(reinterpret_cast<char*>(&newTask.type), 1);
            file.read(reinterpret_cast<char*>(&newTask.priority), 1);
            file.read(reinterpret_cast<char*>(&newTask.cycleTimeMs), 4);
            newTask.currentTimerMs = newTask.cycleTimeMs;

            while (true) {
                uint8_t nextByte = file.peek();
                if (nextByte == 253 || nextByte == 254 || nextByte == 255 || file.eof()) {
                    break;
                }
                PLCInstruction inst;
                file.read(reinterpret_cast<char*>(&inst), sizeof(PLCInstruction));
                newTask.instructions.push_back(inst);
            }
            tempTasks.push_back(newTask); // 寫入暫存容器
        }
        else if (tag == 255) {
            break;
        }
    }
    file.close();

    // 依照 Task Priority 進行排序
    std::sort(tempTasks.begin(), tempTasks.end(), [](const PLCTask& a, const PLCTask& b) {
        return a.priority < b.priority;
        });

    // =========================================================
    // 2. 【即時安全交接區】檔案解析完畢，瞬間加鎖並替換記憶體！
    // =========================================================
    {
        // 🌟 瞬間取得鎖 (等待 RunCycle 結束)
        AutoLockCS lock(&m_logicCS);

        // 使用 std::move 瞬間轉移記憶體所有權 (耗時不到 1 微秒)
        m_tasks = std::move(tempTasks);
        m_customVars = std::move(tempVars);

        // 🌟 防呆：重載邏輯時，建議把所有 Timer 歸零！
        // 避免舊程式計時到一半的 Timer，在換了新程式後突然錯誤觸發
        for (int i = 0; i < MAX_PLC_T; i++) {
            m_T[i].enable = false;
            m_T[i].done = false;
            m_T[i].acc = 0;
        }

        // 注意：這裡不清除 I, O, R 等點位，因為機台還在運轉，保持現有的物理狀態最安全
    }

    std::cout << "[PLC] 成功動態重載邏輯檔案 (Hot-Reload): " << filepath << std::endl;
    return true;
}