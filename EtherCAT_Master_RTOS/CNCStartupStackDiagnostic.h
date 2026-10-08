#pragma once

#include <windows.h>
#include <rtapi.h>
#include <RtssApi.h>

// BASE79-DIAG1: observation only, on the existing startup / NC LOAD caller.
// Never call from a cyclic Motion/PDO callback. No allocation, shared state,
// thread creation, stack-size change, or motion/admission decision is made.
// availableBytes is a current-thread sample, not a peak-use measurement.
#if defined(_MSC_VER)
__declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
__attribute__((noinline))
#endif
inline void PrintCNCStartupStackCheckpoint(const char* phase, int axis = -1) noexcept
{
    const DWORD incomingError = GetLastError();
    DWORD stackBytes = 0U;
    DWORD availableBytes = 0U;
    const bool queried = RtGetThreadStack(GetCurrentThread(), &stackBytes,
        &availableBytes, nullptr, nullptr, nullptr);
    const DWORD queryError = queried ? 0U : GetLastError();
    const bool valid = queried && stackBytes != 0U && availableBytes <= stackBytes;
    if (!valid)
    {
        // Do not present a failed or inconsistent observation as usable space.
        stackBytes = 0U;
        availableBytes = 0U;
    }
    RtPrintf("[BASE79-DIAG1][STACK] phase=%.32s axis=%d query=%s stackBytes=%lu availableBytes=%lu error=%lu\n",
        phase != nullptr ? phase : "UNKNOWN", axis,
        valid ? "OK" : (queried ? "INVALID" : "FAILED"),
        static_cast<unsigned long>(stackBytes), static_cast<unsigned long>(availableBytes),
        static_cast<unsigned long>(queryError));
    SetLastError(incomingError);
}
