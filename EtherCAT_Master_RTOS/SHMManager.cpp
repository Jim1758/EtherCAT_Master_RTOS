#include "SHMManager.h"
#include "GlobalConfig.h" // 為了使用 DEBUG_PRINT
#include <cstring>        // for memset

bool SHMManager::Initialize(const std::string& shmName) {
    if (m_pData != nullptr) return true; // 已經初始化過了

    DEBUG_PRINT("[SHM] Initializing Shared Memory: %s\n", shmName.c_str());
    DEBUG_PRINT("[SHM] SHM_Data Size: %zu Bytes\n", sizeof(SHM_Data));

    // ========================================================
    // 🌟 RTX64 正確的呼叫方式
    // ========================================================

    // 1. 將 std::string 轉成寬字元 wstring，因為 RTX 預設使用 Unicode API
    std::wstring wName(shmName.begin(), shmName.end());

    // 2. 準備一個 void 指標來接收系統配給我們的記憶體位址
    void* pLocation = nullptr;

    // 3. 建立並映射共享記憶體
    m_hShm = RtCreateSharedMemory(
        PAGE_READWRITE,
        0,
        sizeof(SHM_Data),
        wName.c_str(),
        &pLocation // RTX64 會直接把記憶體指標塞進這裡！
    );

    if (m_hShm == NULL) {
        DEBUG_PRINT("[SHM] Failed to create shared memory!\n");
        return false;
    }

    // 4. 將拿到的指標轉型為我們的通訊結構體指標
    m_pData = (SHM_Data*)pLocation;

    // 5. 初始化清零，確保沒有殘留的垃圾數據
    std::memset(m_pData, 0, sizeof(SHM_Data));

    DEBUG_PRINT("[SHM] RTX64 Shared Memory Initialized Successfully!\n");
    return true;
}

void SHMManager::Shutdown() {
    // 釋放記憶體
    if (m_hShm != NULL) {
        // RTX64 只需要關閉 Handle，作業系統就會自動回收該塊記憶體
        RtCloseHandle(m_hShm);
        m_hShm = NULL;
        m_pData = nullptr;
        DEBUG_PRINT("[SHM] Shared Memory Closed.\n");
    }
}