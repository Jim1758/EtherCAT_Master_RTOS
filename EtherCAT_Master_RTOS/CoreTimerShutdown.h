#pragma once
#include "CoreTimerLifecycle.h"
#include <windows.h>
#include <rtapi.h>

// CORE_CLOSE1: one serialized startup/shutdown owner (the main thread).
// Allocate each context once before RtCreateTimer, and retain its small storage
// until process exit. A rejected/pending handler can then read the closed gate
// without dereferencing an already destroyed EtherCatMaster. Owner is accessed
// only under an admitted callback lease; shutdown detaches it after drainage.
struct CoreTimerCallbackContext
{
    explicit CoreTimerCallbackContext(void* owner) noexcept : Owner(owner) {}
    CoreTimerCallbackGate Gate;
    void* Owner;
};

namespace CoreTimerShutdown
{
    inline void Cancel(HANDLE handle, const char* name, const char* phase)
    {
        if (handle != NULL && !RtCancelTimer(handle, NULL))
        {
            const DWORD error = GetLastError();
            RtPrintf("[CORE-CLOSE1] timer=%s phase=%s CANCEL_FAILED error=%lu\n",
                name, phase, static_cast<unsigned long>(error));
        }
    }

    // Main/startup thread only. Never call from a timer handler. Failure retains
    // resources and retries: a deadline must NOT turn unknown lifetime into a
    // successful close. The 1-second messages identify a stalled callback/API.
    inline void DrainAndDelete(HANDLE& handle, CoreTimerCallbackContext& context,
        const char* name)
    {
        context.Gate.RequestStop();
        Cancel(handle, name, "INITIAL");
        unsigned int waits = 0U;
        while (context.Gate.ActiveCount() != 0U)
        {
            if (waits++ % 1000U == 0U)
                RtPrintf("[CORE-CLOSE1] timer=%s WAIT_ACTIVE active=%lu resources=RETAINED\n",
                    name, static_cast<unsigned long>(context.Gate.ActiveCount()));
            RtSleep(1U);
        }

        // The first cancel can race with a last self-rearm. No admitted handler
        // remains now; cancel AGAIN before deleting. Delete itself cancels too.
        unsigned int attempts = 0U;
        while (handle != NULL)
        {
            Cancel(handle, name, "FINAL");
            if (RtDeleteTimer(handle))
            {
                handle = NULL; // Clear ownership ONLY after successful delete.
                RtPrintf("[CORE-CLOSE1] timer=%s DRAINED_DELETED active=0\n", name);
                break;
            }
            const DWORD error = GetLastError();
            if (attempts++ % 10U == 0U)
                RtPrintf("[CORE-CLOSE1] timer=%s DELETE_FAILED error=%lu resources=RETAINED\n",
                    name, static_cast<unsigned long>(error));
            RtSleep(100U);
        }
        context.Owner = nullptr;
    }
}
