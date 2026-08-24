#include "EtherCatMaster.h"

#include <windows.h>
#include <rtapi.h>

#include "GlobalConfig.h"


// ============================================================================
// EtherCatMaster Runtime Lifecycle State
//
// 本檔案只負責 Runtime lifecycle state / result / error snapshot。
//
// 不包含：
//
// - 250 us PDO callback
// - DC control
// - PLC callback
// - NC task
//
// 所有狀態更新由 Priority 50 主控流程呼叫。
// ============================================================================


// ============================================================================
// GetRuntimeStageName
// ============================================================================

const char* EtherCatMaster::GetRuntimeStageName(
    EtherCatRuntimeStage stage) const
{
    switch (stage)
    {
    case EtherCatRuntimeStage::Idle:
        return "IDLE";

    case EtherCatRuntimeStage::Prepare:
        return "PREPARE";

    case EtherCatRuntimeStage::StartPdoRuntime:
        return "START_PDO_RUNTIME";

    case EtherCatRuntimeStage::StartPlcRuntime:
        return "START_PLC_RUNTIME";

    case EtherCatRuntimeStage::EnterOp:
        return "ENTER_OP";

    case EtherCatRuntimeStage::LinkMotion:
        return "LINK_MOTION";

    case EtherCatRuntimeStage::InitSharedMemory:
        return "INIT_SHARED_MEMORY";

    case EtherCatRuntimeStage::Running:
        return "RUNNING";

    case EtherCatRuntimeStage::Stopping:
        return "STOPPING";

    case EtherCatRuntimeStage::Stopped:
        return "STOPPED";

    case EtherCatRuntimeStage::Fault:
        return "FAULT";

    default:
        return "UNKNOWN";
    }
}


// ============================================================================
// ResetRuntimeState
// ============================================================================

void EtherCatMaster::ResetRuntimeState()
{
    InterlockedExchange(
        &m_runtimeStage,
        (LONG)EtherCatRuntimeStage::Idle);


    InterlockedExchange(
        &m_runtimeResult,
        (LONG)EtherCatRuntimeResult::Idle);


    InterlockedExchange(
        &m_runtimeFailedStage,
        (LONG)EtherCatRuntimeStage::Idle);


    InterlockedExchange(
        &m_runtimeErrorCode,
        EcatRuntimeOk);
}


// ============================================================================
// BeginRuntimeStage
// ============================================================================

void EtherCatMaster::BeginRuntimeStage(
    EtherCatRuntimeStage stage)
{
    InterlockedExchange(
        &m_runtimeStage,
        (LONG)stage);


    InterlockedExchange(
        &m_runtimeResult,
        (LONG)EtherCatRuntimeResult::Running);


    DEBUG_PRINT(
        "[MASTER-RUNTIME] Stage:%s | Result:BEGIN\n",
        GetRuntimeStageName(
            stage));
}


// ============================================================================
// PassRuntimeStage
// ============================================================================

void EtherCatMaster::PassRuntimeStage(
    EtherCatRuntimeStage stage)
{
    InterlockedExchange(
        &m_runtimeStage,
        (LONG)stage);


    InterlockedExchange(
        &m_runtimeResult,
        (LONG)EtherCatRuntimeResult::Pass);


    DEBUG_PRINT(
        "[MASTER-RUNTIME] Stage:%s | Result:PASS\n",
        GetRuntimeStageName(
            stage));
}


// ============================================================================
// FailRuntimeStage
// ============================================================================

void EtherCatMaster::FailRuntimeStage(
    EtherCatRuntimeStage failedStage,
    LONG errorCode,
    const char* reason)
{
    InterlockedExchange(
        &m_runtimeFailedStage,
        (LONG)failedStage);


    InterlockedExchange(
        &m_runtimeErrorCode,
        errorCode);


    InterlockedExchange(
        &m_runtimeStage,
        (LONG)EtherCatRuntimeStage::Fault);


    InterlockedExchange(
        &m_runtimeResult,
        (LONG)EtherCatRuntimeResult::Fail);


    DEBUG_PRINT(
        "[MASTER-RUNTIME] Stage:%s | "
        "ErrorCode:%ld | Reason:%s | Result:FAIL\n",

        GetRuntimeStageName(
            failedStage),

        errorCode,

        reason != nullptr
        ? reason
        : "UNKNOWN");


    DEBUG_PRINT(
        "[MASTER-RUNTIME-RESULT] "
        "Stage:FAULT | FailedStage:%s | "
        "ErrorCode:%ld | Result:FAIL\n",

        GetRuntimeStageName(
            failedStage),

        errorCode);
}


// ============================================================================
// MarkRuntimeRunning
// ============================================================================

void EtherCatMaster::MarkRuntimeRunning()
{
    InterlockedExchange(
        &m_runtimeFailedStage,
        (LONG)EtherCatRuntimeStage::Idle);


    InterlockedExchange(
        &m_runtimeErrorCode,
        EcatRuntimeOk);


    InterlockedExchange(
        &m_runtimeStage,
        (LONG)EtherCatRuntimeStage::Running);


    InterlockedExchange(
        &m_runtimeResult,
        (LONG)EtherCatRuntimeResult::Pass);


    DEBUG_PRINT(
        "[MASTER-RUNTIME] Stage:RUNNING | Result:PASS\n");


    DEBUG_PRINT(
        "[MASTER-RUNTIME-RESULT] "
        "Stage:RUNNING | FailedStage:NONE | "
        "ErrorCode:0 | Result:PASS\n");
}


// ============================================================================
// MarkRuntimeStopping
// ============================================================================

void EtherCatMaster::MarkRuntimeStopping()
{
    InterlockedExchange(
        &m_runtimeStage,
        (LONG)EtherCatRuntimeStage::Stopping);


    InterlockedExchange(
        &m_runtimeResult,
        (LONG)EtherCatRuntimeResult::Running);


    DEBUG_PRINT(
        "[MASTER-RUNTIME] Stage:STOPPING | Result:BEGIN\n");
}


// ============================================================================
// MarkRuntimeStopped
// ============================================================================

void EtherCatMaster::MarkRuntimeStopped()
{
    InterlockedExchange(
        &m_runtimeStage,
        (LONG)EtherCatRuntimeStage::Stopped);


    InterlockedExchange(
        &m_runtimeResult,
        (LONG)EtherCatRuntimeResult::Pass);


    DEBUG_PRINT(
        "[MASTER-RUNTIME] Stage:STOPPED | Result:PASS\n");


    DEBUG_PRINT(
        "[MASTER-RUNTIME-RESULT] "
        "Stage:STOPPED | FailedStage:NONE | "
        "ErrorCode:0 | Result:PASS\n");
}


// ============================================================================
// GetRuntimeSnapshot
// ============================================================================

EtherCatRuntimeSnapshot
EtherCatMaster::GetRuntimeSnapshot() const
{
    EtherCatRuntimeSnapshot snapshot;


    snapshot.stage =
        (EtherCatRuntimeStage)
        InterlockedCompareExchange(
            const_cast<volatile LONG*>(
                &m_runtimeStage),
            0,
            0);


    snapshot.result =
        (EtherCatRuntimeResult)
        InterlockedCompareExchange(
            const_cast<volatile LONG*>(
                &m_runtimeResult),
            0,
            0);


    snapshot.failedStage =
        (EtherCatRuntimeStage)
        InterlockedCompareExchange(
            const_cast<volatile LONG*>(
                &m_runtimeFailedStage),
            0,
            0);


    snapshot.errorCode =
        InterlockedCompareExchange(
            const_cast<volatile LONG*>(
                &m_runtimeErrorCode),
            0,
            0);


    return
        snapshot;
}
