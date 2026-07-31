#pragma once
#include <vector>
#include <string>
#include <map>
#include <iostream>



// 預設最大呼叫層數
constexpr int MAX_MACRO_DEPTH = 8;
constexpr int MAX_LOCAL_VARS = 100;   // 區域變數 #1~#100
constexpr int MAX_GLOBAL_VARS = 1000;  // 全域變數 @1~@500 (或 #501~#1000)
constexpr int MAX_SYS_VARS = 1000;    // 系統變數 $1~$1000

class MacroEngine {
public:
    MacroEngine();
    const double MACRO_NULL = 999999999.0;
    // ==========================================
    // 1. 變數存取 API (支援 #, @, $)
    // ==========================================
    double GetVar(char prefix, int index);
    void SetVar(char prefix, int index, double value);

    // ==========================================
    // 2. 巨集呼叫堆疊管理 (G65 / M98)
    // ==========================================
    bool PushCallStack(); // 進入下一層 (例如進入 G65 P1000)
    bool PopCallStack();  // 退出當前層 (M99 返回)
    int GetCurrentDepth() const { return m_callDepth; }

    // ==========================================
    // 3. 系統變數綁定 (供 NCManager 更新用)
    // ==========================================
    // 例如：NCManager 每一圈把目前的機械座標寫入 $100~$107
    void SetSysVar(int index, double value) {
        if (index >= 1 && index <= MAX_SYS_VARS) m_sysVars[index] = value;
    }
    void Reset(); // 新增：系統重置時，清空堆疊與區域變數

    // ==========================================
    // 🌟 新增：提供給 HMI_Bridge 進行全廣播複製的底層記憶體指標
    // ==========================================
    const double* GetGlobalVarsArray() const;
    const double* GetSysVarsArray() const;
    const double* GetLocalVarsArray(int depth) const;


    void InitializeSystemDefaults();//初始化系統 $變數
private:
    // --- 記憶體區塊 ---

    // 區域變數堆疊：二維陣列。第一維是層級(0~8)，第二維是變數(#1~#100)
    std::vector<std::vector<double>> m_localStack;
    int m_callDepth = 0; // 目前深度 (0 = 主程式)

    // 全域變數 (@)
    std::vector<double> m_globalVars;

    // 系統變數 ($)
    std::vector<double> m_sysVars;

    // 定義「空值」(Null/Vacant)。
    // 根據 CNC 規範，未初始化的變數是「空值」，它在加法中視為 0，但在某些運算中有特殊意義。
    // 我們可以用極大值來代表 NULL

};