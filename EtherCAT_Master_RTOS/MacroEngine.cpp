#include "MacroEngine.h"

MacroEngine::MacroEngine()
{
    // 初始化區域變數堆疊 (預設 1 層主程式)
    m_localStack.push_back(std::vector<double>(MAX_LOCAL_VARS + 1, MACRO_NULL));

    // 初始化全域與系統變數
    m_globalVars.resize(MAX_GLOBAL_VARS + 1, MACRO_NULL);
    m_sysVars.resize(MAX_SYS_VARS + 1, 0.0); // 系統變數預設為 0
}

// 進入下一層巨集 (G65)
bool MacroEngine::PushCallStack()
{
    if (m_callDepth >= MAX_MACRO_DEPTH) {
        // 發生警報：巨集呼叫過深！
        // DEBUG_PRINT("ALARM: Macro Call Stack Overflow (Max 8 levels)!\n");
        return false;
    }

    // 增加一層新的區域變數 (全部初始化為 MACRO_NULL)
    m_localStack.push_back(std::vector<double>(MAX_LOCAL_VARS + 1, MACRO_NULL));
    m_callDepth++;
    return true;
}

// 返回上一層 (M99)
bool MacroEngine::PopCallStack()
{
    if (m_callDepth <= 0) {
        // 發生警報：主程式不能再 Pop 了 (通常是寫錯 M99)
        return false;
    }

    m_localStack.pop_back(); // 銷毀這一層的區域變數
    m_callDepth--;
    return true;
}

// 讀取變數
double MacroEngine::GetVar(char prefix, int index)
{
    if (prefix == '#')
    {
        // Fanuc 規範：#1~#100 是區域，#501~ 是全域
        if (index >= 1 && index <= MAX_LOCAL_VARS) {
            return m_localStack[m_callDepth][index];
        }
        else if (index >= 501 && index <= 501 + MAX_GLOBAL_VARS) {
            return m_globalVars[index - 500]; // 映射到 1~500
        }
    }
    else if (prefix == '@')
    {
        if (index >= 1 && index <= MAX_GLOBAL_VARS) return m_globalVars[index];
    }
    else if (prefix == '$')
    {
        if (index >= 1 && index <= MAX_SYS_VARS) return m_sysVars[index];
    }

    return 0.0; // 越界或無效變數預設回傳 0
}

// 寫入變數
void MacroEngine::SetVar(char prefix, int index, double value)
{
    if (prefix == '#')
    {
        if (index >= 1 && index <= MAX_LOCAL_VARS) {
            m_localStack[m_callDepth][index] = value;
        }
        else if (index >= 501 && index <= 501 + MAX_GLOBAL_VARS) {
            m_globalVars[index - 500] = value;
        }
    }
    else if (prefix == '@')
    {
        if (index >= 1 && index <= MAX_GLOBAL_VARS) m_globalVars[index] = value;
    }
    else if (prefix == '$')
    {
        // 系統變數通常是唯讀的，但如果你允許 $ 變數可寫入，就放行：
        if (index >= 1 && index <= MAX_SYS_VARS) m_sysVars[index] = value;
    }
}