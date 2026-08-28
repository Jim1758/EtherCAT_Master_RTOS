#include "MacroEngine.h"

MacroEngine::MacroEngine()
{
    // 初始化全域與系統變數 (通常只在開機時執行一次)
    m_globalVars.assign(MAX_GLOBAL_VARS + 1, 0.0);
    m_sysVars.assign(MAX_SYS_VARS + 1, 0.0); // 系統變數預設為 0

    // 初始化區域變數與堆疊
    Reset();
    InitializeSystemDefaults();//初始化系統 $變數



}

void MacroEngine::InitializeSystemDefaults()//初始化系統 $變數
{
    //初始化G碼群駔--------------
    SetVar('$', 1, 0);//設定群組1變數
    SetVar('$', 2, 17);//設定群組2變數
    SetVar('$', 3, 90);//設定群組3變數
    SetVar('$', 4, 23);//設定群組4變數
    SetVar('$', 6, 21);//設定群組6變數
    SetVar('$', 7, 40);//設定群組8變數
    SetVar('$', 8, 49);//設定群組8變數
    SetVar('$', 16, 69);//設定群組16變數
    SetVar('$', 11, 50.0);//設定群組11變數
    SetVar('$', 17, 16.0);//設定群組17變數
}

// 🌟 新增：系統重置 (給 NCManager::Reset 呼叫)
void MacroEngine::Reset()
{
    m_localStack.clear();
    // 重新推入第 0 層 (主程式專用)，並全部初始化為 MACRO_NULL
    m_localStack.push_back(std::vector<double>(MAX_LOCAL_VARS + 1, MACRO_NULL));
    m_callDepth = 0;

    // 備註：全域變數 (@1~@1000) 與系統變數 ($1~$1000)
    // 按下 NC Reset 時不清除；# 只保留 #1~#100 的 Local 語意。
}

// 進入下一層巨集 (G65 / M98)
bool MacroEngine::PushCallStack()
{
    if (m_callDepth >= MAX_MACRO_DEPTH) {
        // 發生警報：巨集呼叫過深！(交給外部觸發 Alarm)
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

    m_localStack.pop_back(); // 銷毀這一層的區域變數 (#1~#33 將隨之消滅)
    m_callDepth--;
    return true;
}

// 讀取變數
double MacroEngine::GetVar(char prefix, int index)
{
    if (!MacroVariableRules::IsValidIndex(prefix, index))
    {
        return MACRO_NULL;
    }

    if (prefix == '#')
    {
        return m_localStack[m_callDepth][index];
    }
    if (prefix == '@')
    {
        return m_globalVars[index];
    }
    if (prefix == '$')
    {
        return m_sysVars[index];
    }

    return MACRO_NULL;
}

// 寫入變數
void MacroEngine::SetVar(char prefix, int index, double value)
{
    if (!MacroVariableRules::IsValidIndex(prefix, index))
    {
        return;
    }

    if (prefix == '#')
    {
        m_localStack[m_callDepth][index] = value;
        return;
    }
    if (prefix == '@')
    {
        m_globalVars[index] = value;
        return;
    }
    if (prefix == '$')
    {
        m_sysVars[index] = value;
    }
}


// ==========================================
// 🌟 實作：取得底層記憶體陣列指標供 HMI_Bridge 複製
// ==========================================
const double* MacroEngine::GetGlobalVarsArray() const
{
    // vector::data() 會回傳內部儲存資料的連續記憶體首位址
    return m_globalVars.data();
}

const double* MacroEngine::GetSysVarsArray() const
{
    return m_sysVars.data();
}

const double* MacroEngine::GetLocalVarsArray(int depth) const
{
    // 檢查範圍，保護系統不會因為陣列越界而崩潰 (BSOD)
    if (depth >= 0 && depth < (int)m_localStack.size()) {
        return m_localStack[depth].data();
    }

    // 如果外部傳來不存在的層數 (例如這台機台目前只跑到第 3 層，但 HMI 想抓第 5 層)
    // 我們不能回傳 nullptr 讓 memcpy 當掉。
    // 我們可以安全地回傳第 0 層 (或自己建一個充滿 MACRO_NULL 的靜態空陣列)，
    // 這裡簡單回傳主程式的記憶體，讓 HMI_Bridge 可以安全地 memcpy
    return m_localStack[0].data();
}
