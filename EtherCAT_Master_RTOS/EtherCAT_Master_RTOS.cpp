#include "EtherCAT_Master_RTOS.h"
#include <windows.h> 
#include <tchar.h>
#include <rtapi.h>
#include <string> // 確保引入 string
#include <fstream>   // 🌟 處理檔案讀取
#include <algorithm> // 🌟 處理字串轉換 (轉大寫防呆)
#include "NicDriver.h"
#include "EtherCatEni.h"
#include "EtherCatMaster.h"
#include "ConfigReader.h"
#include "GlobalConfig.h"
#include "SHMManager.h"
#include <RtssApi.h>
#include "NCMainThreadStackPolicy.h"
#include <atomic>
#include <cstdint>

// RTX64 Boilerplate (標準 RTX64 樣板)
#ifndef NUM_REGISTERS
#define NUM_REGISTERS 10 
#endif
PCHAR vMemAddr[NUM_REGISTERS];
BOOLEAN RTAPI DeviceIST(PVOID pContext) { return TRUE; }

//const std::string BASE_DATA_DIR = "D:\\EtherCAT_Master_Data\\";//系統主要資料夾位置
CNicDriver MyNic;
EtherCatEni MyEni;
EtherCatMaster Master;

bool InitEtherCATMaster();//初始化 EtherCAT 主站


// EDM85 FIX1: validate the actual main-thread allocation before runtime
// initialization can start the NIC, Motion/PDO or PLC timer threads.
// This is an entry-time observation, not a whole-program stack high-water proof.
static bool ValidateNCMainThreadStackBeforeInitialization() noexcept
{
    const DWORD incomingError = GetLastError();
    DWORD bytes = 0U, available = 0U;
    ULONG_PTR base = 0U, limit = 0U, top = 0U;
    const bool queried = RtGetThreadStack(GetCurrentThread(), &bytes, &available,
        &base, &limit, &top);
    const DWORD error = queried ? 0U : GetLastError();
    const auto result = NCMainStack::Evaluate(queried, bytes, available, base, limit, top);
    RtPrintf("[EDM110] event=MAIN_STACK build=EDM110_20261014 result=%s reason=%s threadId=%lu requiredBytes=%llu requiredStartupFreeBytes=%llu stackBytes=%lu availableBytes=%lu base=%llu limit=%llu top=%llu topInRange=%u error=%lu phase=BEFORE_RUNTIME_INIT\n",
        result == NCMainStack::Result::Ready ? "PASS" : "BLOCKED", NCMainStack::Name(result),
        static_cast<unsigned long>(GetCurrentThreadId()),
        static_cast<unsigned long long>(NCMainStack::RequiredBytes),
        static_cast<unsigned long long>(NCMainStack::RequiredStartupFreeBytes),
        static_cast<unsigned long>(bytes), static_cast<unsigned long>(available),
        static_cast<unsigned long long>(base), static_cast<unsigned long long>(limit),
        static_cast<unsigned long long>(top), NCMainStack::ContainsTop(base, limit, top) ? 1U : 0U,
        static_cast<unsigned long>(error));
    SetLastError(incomingError);
    return result == NCMainStack::Result::Ready;
}

// EDM173 FIX12: last-chance evidence only. Preserve the previous filter and
// normal RTSS freeze/unload behavior. The observer itself requests no continuation;
// any previous filter keeps its established return value.
// This is best effort: debugger interception, disabled RTSS SEH or an invalid
// fault stack can prevent this observer from running or printing.
static LPTOP_LEVEL_EXCEPTION_FILTER g_edm173PreviousExceptionFilter = nullptr;
static std::atomic_flag g_edm173ExceptionLogging = ATOMIC_FLAG_INIT;

static LONG WINAPI ObserveEDM173UnhandledException(EXCEPTION_POINTERS* info) noexcept
{
    const DWORD incomingError = GetLastError();
    if (g_edm173ExceptionLogging.test_and_set(std::memory_order_acquire))
    {
        SetLastError(incomingError);
        return EXCEPTION_CONTINUE_SEARCH; // Do not re-enter a faulting logger or previous filter.
    }
    {
        const EXCEPTION_RECORD* record = info != nullptr ? info->ExceptionRecord : nullptr;
        const CONTEXT* context = info != nullptr ? info->ContextRecord : nullptr;
        const unsigned long code = record != nullptr ? static_cast<unsigned long>(record->ExceptionCode) : 0UL;
        const unsigned long parameters = record != nullptr ? static_cast<unsigned long>(record->NumberParameters) : 0UL;
        const unsigned long long address = record != nullptr ? static_cast<unsigned long long>(
            reinterpret_cast<std::uintptr_t>(record->ExceptionAddress)) : 0ULL;
        const unsigned long long p0 = record != nullptr && parameters > 0UL ?
            static_cast<unsigned long long>(record->ExceptionInformation[0U]) : 0ULL;
        const unsigned long long p1 = record != nullptr && parameters > 1UL ?
            static_cast<unsigned long long>(record->ExceptionInformation[1U]) : 0ULL;
        unsigned long long ip = 0ULL, sp = 0ULL, ax = 0ULL, cx = 0ULL, dx = 0ULL, r8 = 0ULL, r9 = 0ULL;
#if defined(_M_X64) || defined(__x86_64__)
        if (context != nullptr)
        {
            ip = static_cast<unsigned long long>(context->Rip); sp = static_cast<unsigned long long>(context->Rsp);
            ax = static_cast<unsigned long long>(context->Rax); cx = static_cast<unsigned long long>(context->Rcx);
            dx = static_cast<unsigned long long>(context->Rdx); r8 = static_cast<unsigned long long>(context->R8);
            r9 = static_cast<unsigned long long>(context->R9);
        }
#else
        (void)context;
#endif
        RtPrintf("[EDM173_FIX12] event=UNHANDLED_EXCEPTION threadId=%lu code=0x%08lX exceptionAddress=0x%llX parameters=%lu parameter0=0x%llX parameter1=0x%llX ip=0x%llX sp=0x%llX ax=0x%llX cx=0x%llX dx=0x%llX r8=0x%llX r9=0x%llX action=NORMAL_EXCEPTION_HANDLING\n",
            static_cast<unsigned long>(GetCurrentThreadId()), code, address, parameters, p0, p1, ip, sp, ax, cx, dx, r8, r9);
    }
    SetLastError(incomingError);
    const auto previous = g_edm173PreviousExceptionFilter;
    const LONG result = previous != nullptr && previous != ObserveEDM173UnhandledException ?
        previous(info) : EXCEPTION_CONTINUE_SEARCH;
    g_edm173ExceptionLogging.clear(std::memory_order_release);
    return result;
}

static void InstallEDM173ExceptionObserverBeforeRuntime() noexcept
{
    const DWORD incomingError = GetLastError();
    g_edm173PreviousExceptionFilter = SetUnhandledExceptionFilter(ObserveEDM173UnhandledException);
    RtPrintf("[EDM173_FIX12] event=EXCEPTION_OBSERVER_INSTALL attempted=1 threadId=%lu previousFilter=%u action=NORMAL_EXCEPTION_HANDLING\n",
        static_cast<unsigned long>(GetCurrentThreadId()), g_edm173PreviousExceptionFilter != nullptr ? 1U : 0U);
    SetLastError(incomingError);
}

int _tmain(int argc, _TCHAR* argv[])//
{
    if (!ValidateNCMainThreadStackBeforeInitialization()) return -1;
    InstallEDM173ExceptionObserverBeforeRuntime();

    // ==============================================================
    // 🌟 1. 設定 CPU 核心親和性 (自動偵測安全版)
    // ==============================================================
    DWORD_PTR currentProcessMask = 0;
    DWORD_PTR availableSystemMask = 0; // 這個變數會裝載這台機台所有 RTSS 核心的遮罩
    HANDLE hCurrentProcess = GetCurrentProcess();

    // 步驟 A: 取得當前 RTSS 系統開放給您的可用核心遮罩
    if (!RtGetProcessAffinityMask(hCurrentProcess, &currentProcessMask, &availableSystemMask))
    {
        DEBUG_PRINT("[Error] RtGetProcessAffinityMask Failed! Error: %lu\n", GetLastError());
        return -1;
    }

    // 假設機台 RTSS 分配在 Core 2 與 Core 3，availableSystemMask 就會是 0x0C (二進位 1100)
    DEBUG_PRINT("[System] RTSS Available Core Mask is: 0x%llX\n", availableSystemMask);

    // 步驟 B: 從可用的核心中，抓出最低位的第一顆核心
    // (這是一招位元運算魔法: X & ~(X-1) 能精準提取最低位的 1)
    // 如果 availableSystemMask 是 0x0C (1100)，算出來的 targetMask 就會是 0x04 (0100)，也就是 Core 2
    DWORD_PTR targetMask = availableSystemMask & ~(availableSystemMask - 1);

    // 步驟 C: 將程式綁定到這顆核心
    if (!RtSetProcessAffinityMask(hCurrentProcess, targetMask))
    {
        DEBUG_PRINT("[Error] Failed to set CPU Affinity to Mask 0x%llX. Error: %lu\n", targetMask, GetLastError());
        return -1;
    }

    DEBUG_PRINT("[System] Successfully bound process to RTSS Core Mask: 0x%llX\n", targetMask);
    // ==============================================================


    std::string configPath = GlobalConfig::GetInstance().BaseDataDir + "SystemConfig.txt";
    GlobalConfig::GetInstance().LoadFromFile(configPath);//讀取系統檔案

    if (GlobalConfig::GetInstance().systemMode == SystemMode::UNKNOWN_MODE)
    {
        DEBUG_PRINT("Error System_Mode !\n");//錯誤系統模式
        return -1;
    }



    // Main Real-Time Loop Priority

    HANDLE hMainThread = GetCurrentThread();

    if (!RtSetThreadPriority(hMainThread, 50))
    {
        DEBUG_PRINT("[RTSS] Set Main Thread Priority Error >> %d\n", GetLastError());
        return -1;
    }

    DEBUG_PRINT("[RTSS] Main Thread Priority = %d\n", RtGetThreadPriority(hMainThread));


    switch (GlobalConfig::GetInstance().systemMode)
    {
    case SystemMode::EDM_SINKER_MODE://EDM 雕磨模式


        if (GlobalConfig::GetInstance().InitSystemParameters(Master) == false)
        {
            DEBUG_PRINT("InitSystemParameters Error !\n");
            goto Exit;
        }

        if (InitEtherCATMaster() == false)//初始化 主站
        {
            DEBUG_PRINT("InitEtherCATMaster Error !\n");
            goto Exit;
        }

        //Master.RunRealTimeCycle_EXAMPLE_MODE();//主要程式迴圈執行_測試模式
        DEBUG_PRINT("EDM_SINKER_MODE Start !\n");
        if (Master.RunRealTimeCycle_EDM_SINKER_MODE() == -1)
        {
            DEBUG_PRINT("RunRealTimeCycle_EDM Error !\n");
            goto Exit;
        }



        break;
    case SystemMode::EXAMPLE_MODE://範例模式 debug使用
        DEBUG_PRINT("EXAMPLE_MODE Start !\n");

        if (InitEtherCATMaster() == false)//初始化 主站
        {
            DEBUG_PRINT("InitEtherCATMaster Error !\n");
            goto Exit;
        }


        Master.RunRealTimeCycle_EXAMPLE_MODE();//主要程式迴圈執行_測試模式
        break;
    case SystemMode::UNKNOWN_MODE://未知模式
        DEBUG_PRINT("Error System_Mode>>UNKNOWN_MODE !\n");//錯誤系統模式
        goto Exit;

    }



Exit:
    Master.StopCyclicRuntime(); // Includes partial PDO/PLC startup failures.
    SHMManager::GetInstance().Shutdown(); // Idempotent; waits for condition-worker shared work to finish.
    DEBUG_PRINT("[CORE-CLOSE1] NIC_CLOSE_BEGIN\n");
    MyNic.Close();
    DEBUG_PRINT("[CORE-CLOSE1] COMPLETE\n");
    DEBUG_PRINT("EtherCAT_Master Close !\n");
    return 0;
}


// ============================================================================
// InitEtherCATMaster
//
// RTOS Entry 只負責組出 Runtime Config 路徑。
// 真正 Master Startup 流程已搬到 EtherCatMaster_Startup.cpp。
// ============================================================================

bool InitEtherCATMaster()//初始化 EtherCAT 主站
{
    DEBUG_PRINT("InitEtherCATMaster !\n");


    const std::string eniPath =
        GlobalConfig::GetInstance().BaseDataDir +
        "ENI\\EtherCAT_Runtime.xml";


    return
        Master.InitializeMaster(
            MyNic,
            MyEni,
            eniPath.c_str());
}
