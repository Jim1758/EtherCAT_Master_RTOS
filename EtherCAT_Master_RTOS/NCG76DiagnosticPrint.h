#pragma once

#include <windows.h>
#include <rtapi.h>
#include <cstdarg>
#include <cstdio>
#include <cstring>

// EDM173 FIX13: RTSS RtPrintf does not support floating-point conversions.
// Format G76 NC/startup diagnostics through the CRT, then send one bounded
// string to RtPrintf. Keep this helper out of the cyclic Motion/PDO callback.
#if defined(_MSC_VER)
__declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
__attribute__((noinline, format(printf, 1, 2)))
#endif
inline void PrintNCG76DiagnosticLine(const char* format, ...) noexcept
{
    if (format == nullptr)
    {
        RtPrintf("%s", "[EDM173_FIX13] event=G76_LOG_FORMAT_FAILURE discharge=0\n");
        return;
    }

    char line[512];
    va_list args;
    va_start(args, format);
    const int length = std::vsnprintf(line, sizeof(line), format, args);
    va_end(args);
    if (length < 0)
    {
        RtPrintf("%s", "[EDM173_FIX13] event=G76_LOG_FORMAT_FAILURE discharge=0\n");
        return;
    }
    if (static_cast<std::size_t>(length) >= sizeof(line))
    {
        const char suffix[] = " [TRUNCATED]\n";
        std::memcpy(line + sizeof(line) - sizeof(suffix), suffix, sizeof(suffix));
    }
    RtPrintf("%s", line);
}
