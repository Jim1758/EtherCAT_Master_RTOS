#pragma once

#include <windows.h>
#include <rtapi.h>
#include <RtssApi.h>
#include <cstddef>

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

// EDM293 FIX1: one bounded observation at a P50 preparation/admission
// milestone. The caller supplies its own compile-time sizes, so the record
// does not depend on MotionCore/NCManager definitions or grant any authority.
// NC callers may emit a small fixed set of milestones. The RT caller emits
// this only once per admission, never once per servo tick. This is a current
// stack sample; neither the local marker nor these counters prove peak usage.
#if defined(_MSC_VER)
__declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
__attribute__((noinline))
#endif
inline void PrintG76SphereMemoryDiagnostic(const char* stage, int pc,
    std::size_t sizeSnapshot, std::size_t sizeFrame, std::size_t sizeGeometry,
    std::size_t sizeRequest, std::size_t sizeFeedback, std::size_t sizeOwner) noexcept
{
    const DWORD incomingError = GetLastError();
    unsigned char marker = 0U;
    DWORD stackBytes = 0U, availableBytes = 0U;
    ULONG_PTR base = 0U, limit = 0U, top = 0U;
    const DWORD threadId = GetCurrentThreadId();
    const bool queried = RtGetThreadStack(GetCurrentThread(), &stackBytes,
        &availableBytes, &base, &limit, &top);
    const DWORD queryError = queried ? 0U : GetLastError();
    const bool valid = queried && stackBytes != 0U && availableBytes <= stackBytes;
    if (!valid)
    {
        stackBytes = 0U; availableBytes = 0U;
        base = 0U; limit = 0U; top = 0U;
    }
    const ULONG_PTR low = base < limit ? base : limit;
    const ULONG_PTR high = base < limit ? limit : base;
    const bool topInRange = valid && base != 0U && limit != 0U && base != limit &&
        top != 0U && top >= low && top <= high;
    RtPrintf("[P50] build=EDM294 event=MEMORY_STAGE stage=%.32s pc=%d threadId=%lu stackQuery=%s stackBytes=%lu availableBytes=%lu queryError=%lu base=%llu limit=%llu top=%llu topInRange=%u marker=%llu sizeSnapshot=%llu sizeFrame=%llu sizeGeometry=%llu sizeRequest=%llu sizeFeedback=%llu sizeOwner=%llu observation=CURRENT_NOT_PEAK discharge=0\n",
        stage != nullptr ? stage : "UNKNOWN", pc, static_cast<unsigned long>(threadId),
        valid ? "OK" : (queried ? "INVALID" : "FAILED"),
        static_cast<unsigned long>(stackBytes), static_cast<unsigned long>(availableBytes),
        static_cast<unsigned long>(queryError), static_cast<unsigned long long>(base),
        static_cast<unsigned long long>(limit), static_cast<unsigned long long>(top),
        topInRange ? 1U : 0U,
        static_cast<unsigned long long>(reinterpret_cast<ULONG_PTR>(&marker)),
        static_cast<unsigned long long>(sizeSnapshot), static_cast<unsigned long long>(sizeFrame),
        static_cast<unsigned long long>(sizeGeometry), static_cast<unsigned long long>(sizeRequest),
        static_cast<unsigned long long>(sizeFeedback), static_cast<unsigned long long>(sizeOwner));
    SetLastError(incomingError);
}
