#pragma once

#include <windows.h>
#include <rtapi.h>
#include <RtssApi.h>

// EDM173 FIX12: bounded, NC-caller-only G76 admission observations.
// Do not call from the cyclic Motion/PDO callback or from a per-child loop.
// No allocation, shared diagnostic state, admission decision or exception handling.
// Available bytes describe this call only, not the runtime stack high-water mark.
#if defined(_MSC_VER)
__declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
__attribute__((noinline))
#endif
inline void PrintNCG76StartCheckpoint(const char* stage, int pc) noexcept
{
    const DWORD incomingError = GetLastError();
    DWORD stackBytes = 0U;
    DWORD availableBytes = 0U;
    const DWORD threadId = GetCurrentThreadId();
    const bool queried = RtGetThreadStack(GetCurrentThread(), &stackBytes,
        &availableBytes, nullptr, nullptr, nullptr);
    const DWORD queryError = queried ? 0U : GetLastError();
    const bool valid = queried && stackBytes != 0U && availableBytes <= stackBytes;
    if (!valid) { stackBytes = 0U; availableBytes = 0U; }
    RtPrintf("[EDM173_FIX12] event=G76_START_STAGE stage=%.32s pc=%d threadId=%lu stackQuery=%s stackBytes=%lu availableBytes=%lu queryError=%lu observation=CURRENT_NOT_PEAK\n",
        stage != nullptr ? stage : "UNKNOWN", pc, static_cast<unsigned long>(threadId),
        valid ? "OK" : (queried ? "INVALID" : "FAILED"),
        static_cast<unsigned long>(stackBytes), static_cast<unsigned long>(availableBytes),
        static_cast<unsigned long>(queryError));
    SetLastError(incomingError);
}
