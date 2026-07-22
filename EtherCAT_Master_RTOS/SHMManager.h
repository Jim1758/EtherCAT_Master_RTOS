#pragma once
#include "SHM_Types.h"
#include <string>

// 🌟 引入 RTX64 專用的標頭檔
#include <windows.h>
#include <rtapi.h> 

class SHMManager {
public:
    static SHMManager& GetInstance() {
        static SHMManager instance;
        return instance;
    }

    // 🌟 修正：RTX64 端「不需要」加 Global\，直接用原本的名字！
    bool Initialize(const std::string& shmName = "EDM_CNC_SHM");

    // 釋放記憶體
    void Shutdown();

    // 取得資料指標 (外部程式透過這個指標讀寫數據)
    SHM_Data* GetData() { return m_pData; }

private:
    SHMManager() = default;
    ~SHMManager() { Shutdown(); }

    // 禁止拷貝
    SHMManager(const SHMManager&) = delete;
    SHMManager& operator=(const SHMManager&) = delete;

    SHM_Data* m_pData = nullptr;

    // 🌟 RTX64 專用的 Handle (用來記錄作業系統配給我們的記憶體)
    HANDLE m_hShm = NULL;
};