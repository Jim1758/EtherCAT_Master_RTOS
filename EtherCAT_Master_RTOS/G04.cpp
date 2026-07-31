#include "GMCodeHandlers.h"
#include "NCManager.h"
#include "GlobalConfig.h"
#include <windows.h>
#include <rtapi.h> // 🌟 RTX64 專用 API
namespace GCodeHandlers 
{
    // 🌟 換成 RTX64 專用的 64 位元大整數，用來記錄硬體 Ticks
    static LARGE_INTEGER s_g04_startTime;
    static LARGE_INTEGER s_perfFreq = { 0 }; // 系統計時頻率
    static double s_g04_targetMs = 0.0;

    // ==========================================
    // 🌟 新增：專屬於 G04 的重置邏輯
    // ==========================================
    void Reset_G04(NCManager* nc)
    {
        if (nc != nullptr) {
            nc->SetG04TimeMs(0.0); // 歸零
            //DEBUG_PRINT("[NC] G04 Timer Reset\n");
        }
    }
    // 🌟 專屬的檢查邏輯
    static bool CheckG04Done(NCManager* nc)
    {
        double timeMs = nc->GetG04TimeMs();

        timeMs -= 10.0;
        nc->SetG04TimeMs(timeMs);

        if (timeMs > 0.0) {
            return false; // 還沒結束
        }

        // 🌟 G04 結束！取得當下的硬體 Ticks
        
        LARGE_INTEGER endTime;
        RtQueryPerformanceCounter(&endTime);

        // 🌟 算出實際經過的毫秒數：(結束 Ticks - 開始 Ticks) * 1000.0 / 頻率
        double elapsedMs = (double)(endTime.QuadPart - s_g04_startTime.QuadPart) * 1000.0 / (double)s_perfFreq.QuadPart;

        double targetMs = s_g04_targetMs;
        double errorMs = elapsedMs - targetMs;

        // ==========================================
        // 🛠️ RTX64 安全寫法：拆解浮點數為整數與小數 (精準到小數後三位)
        // ==========================================
        
        int i_target = (int)targetMs;
        int f_target = (int)((targetMs - i_target) * 1000);
        f_target = f_target < 0 ? -f_target : f_target;

        int i_elapsed = (int)elapsedMs;
        int f_elapsed = (int)((elapsedMs - i_elapsed) * 1000);
        f_elapsed = f_elapsed < 0 ? -f_elapsed : f_elapsed;

        int i_error = (int)errorMs;
        int f_error = (int)((errorMs - i_error) * 1000);
        f_error = f_error < 0 ? -f_error : f_error;

        const char* sign = (errorMs < 0 && i_error == 0) ? "-" : "";

        DEBUG_PRINT("========================================\n");
        DEBUG_PRINT(" [G04 Precision Report] \n");
        DEBUG_PRINT(" Target Dwell   : %d.%03d ms\n", i_target, f_target);
        DEBUG_PRINT(" Actual Elapsed : %d.%03d ms\n", i_elapsed, f_elapsed);
        DEBUG_PRINT(" System Jitter  : %s%d.%03d ms\n", sign, i_error, f_error);
        DEBUG_PRINT("========================================\n");
        
        return true;
    }

    WaitConditionFunc Handle_G04(const NCBlock& block, NCManager* nc)
    {
        double seconds = 0.0;
        if (block.has('X')) seconds = block.val('X');
        else if (block.has('P')) seconds = block.val('P') / 1000.0;

        if (seconds <= 0.0) return nullptr;

        // 🌟 第一次執行時，取得 RTX64 系統的硬體計時頻率
        if (s_perfFreq.QuadPart == 0) {
            RtQueryPerformanceFrequency(&s_perfFreq);
            // 註：如果編譯器找不到 Rt 開頭的，請改用標準的 QueryPerformanceFrequency
        }

        double totalMs = seconds * 1000.0;
        nc->SetG04TimeMs(totalMs);

        s_g04_targetMs = totalMs;

        // 🌟 記錄開始的硬體 Ticks
        RtQueryPerformanceCounter(&s_g04_startTime);
        // 註：同上，若報錯可改為 QueryPerformanceCounter

        //DEBUG_PRINT("[NC] G04 Dwell Started: %.3f sec\n", seconds);

        return CheckG04Done;
    }
} // end namespace