#pragma once

#include <cstdint>

// =============================================================================
// NC-0.2K.7.8.1 - PDO Safety Stop Consecutive-Invalid Debounce
//
// This policy changes only the irreversible containment boundary.  Runtime
// cycle 1 remains fail-closed: invalid inputs are not adopted, interpolation
// and Motion updates are skipped, and NC settle/output proof is revoked.  Once
// that invalid result is known, following callbacks also hold new PLC output
// flushing.  EmergencyStopAllAxes() and AL1003 are applied only if the invalid
// episode reaches this bounded threshold.
// =============================================================================

namespace EtherCatPdoSafetyStopDebounce
{
    static constexpr std::uint32_t ConsecutiveInvalidCycles = 8U;

    constexpr bool IsContainmentRequired(
        std::uint32_t consecutiveInvalidCycles) noexcept
    {
        return consecutiveInvalidCycles >= ConsecutiveInvalidCycles;
    }

    static_assert(
        ConsecutiveInvalidCycles > 1U,
        "PDO debounce must retain a transient recovery window.");
    static_assert(
        !IsContainmentRequired(ConsecutiveInvalidCycles - 1U),
        "PDO containment must remain closed before the threshold.");
    static_assert(
        IsContainmentRequired(ConsecutiveInvalidCycles),
        "PDO containment must apply exactly at the threshold.");
}
