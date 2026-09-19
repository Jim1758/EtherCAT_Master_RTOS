#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

// Fixed native-axis identity. This is dimensional provenance, not permission
// to plan or execute a particular axis, transform, or eccentric trajectory.
// Axis types: 0 linear, 1 rotary, 2 rotary continuous.
// Native units: 0 absent, 1 millimetres, 2 degrees.
struct NCAxisIdentitySnapshot
{
    std::uint8_t exists[8] = {};
    std::uint8_t axisType[8] = {};
    std::uint8_t nativeUnit[8] = {};
    std::uint8_t physicalIndexPlusOne[8] = {};
    char address[8] = {};
    std::uint8_t electrodeAxisPlusOne = 0U; // 0 unbound, or physical axes 4..8.
    std::uint8_t systemMode = 0U; // GlobalConfig modes: 0 unknown, 1 EDM, 2 example.
    std::uint8_t eccentricEnabled = 0U; // Captured G162 switch; never grants motion.
    std::uint8_t bound = 0U;
    std::uint8_t reserved[4] = {};
};

inline bool IsNCAxisIdentitySnapshotEmpty(const NCAxisIdentitySnapshot& s) noexcept
{
    const NCAxisIdentitySnapshot empty{};
    return std::memcmp(&s, &empty, sizeof(s)) == 0;
}

inline bool SameNCAxisIdentitySnapshot(const NCAxisIdentitySnapshot& a,
    const NCAxisIdentitySnapshot& b) noexcept
{
    return std::memcmp(&a, &b, sizeof(a)) == 0;
}

inline bool IsNCAxisIdentitySnapshotValid(const NCAxisIdentitySnapshot& s) noexcept
{
    if (s.bound != 1U || s.systemMode > 2U || s.eccentricEnabled > 1U)
        return false;
    for (unsigned i = 0U; i < 4U; ++i)
        if (s.reserved[i] != 0U) return false;
    bool hasPresentAxis = false;
    for (unsigned axis = 0U; axis < 8U; ++axis)
    {
        const char letter = s.address[axis];
        if (letter != 0 && (letter < 'A' || letter > 'Z')) return false;
        if (letter != 0)
            for (unsigned previous = 0U; previous < axis; ++previous)
                if (s.address[previous] == letter) return false;
        if (s.exists[axis] == 0U)
        {
            if (s.axisType[axis] != 0U || s.nativeUnit[axis] != 0U ||
                s.physicalIndexPlusOne[axis] != 0U) return false;
        }
        else
        {
            if (s.exists[axis] != 1U || letter == 0 || s.axisType[axis] > 2U ||
                s.physicalIndexPlusOne[axis] != axis + 1U ||
                s.nativeUnit[axis] != (s.axisType[axis] == 0U ? 1U : 2U))
                return false;
            hasPresentAxis = true;
        }
    }
    if (!hasPresentAxis) return false;
    if (s.electrodeAxisPlusOne != 0U)
    {
        if (s.electrodeAxisPlusOne < 4U || s.electrodeAxisPlusOne > 8U)
            return false;
        const unsigned electrode = s.electrodeAxisPlusOne - 1U;
        if (s.exists[electrode] != 1U || s.nativeUnit[electrode] != 2U)
            return false;
    }
    return true;
}

// The container supplies size() and operator[] with isExist, axisIndex and
// axisType fields. No Motion or platform headers are needed by this contract.
// Capture occurs on the configuration-owning thread before publication.
// Absent dummy metadata is intentionally canonicalized, never interpreted.
template <class AxisContainer>
inline bool TryCaptureNCAxisIdentitySnapshot(const AxisContainer& axes,
    const char* addresses8, int electrodeAxisIndex, int systemMode,
    bool eccentricEnabled, NCAxisIdentitySnapshot& output) noexcept
{
    if (!addresses8 || axes.size() == 0U || axes.size() > 8U ||
        systemMode < 0 || systemMode > 2 ||
        (electrodeAxisIndex != -1 &&
            (electrodeAxisIndex < 3 || electrodeAxisIndex > 7))) return false;
    NCAxisIdentitySnapshot candidate{};
    candidate.electrodeAxisPlusOne = static_cast<std::uint8_t>(electrodeAxisIndex + 1);
    candidate.systemMode = static_cast<std::uint8_t>(systemMode);
    candidate.eccentricEnabled = eccentricEnabled ? 1U : 0U;
    candidate.bound = 1U;
    for (unsigned axis = 0U; axis < 8U; ++axis)
    {
        candidate.address[axis] = addresses8[axis] == ' ' ? 0 : addresses8[axis];
        if (axis >= axes.size() || !axes[axis].isExist) continue;
        const auto& native = axes[axis];
        // Widen before narrowing: negative or >255 enum values must not alias
        // a valid one-byte type. Slot comparison also precedes serialization.
        const std::uintmax_t type = static_cast<std::uintmax_t>(native.axisType);
        if (type > 2U || native.axisIndex != static_cast<decltype(native.axisIndex)>(axis))
            return false;
        candidate.exists[axis] = 1U;
        candidate.axisType[axis] = static_cast<std::uint8_t>(type);
        candidate.nativeUnit[axis] = type == 0U ? 1U : 2U;
        candidate.physicalIndexPlusOne[axis] = static_cast<std::uint8_t>(axis + 1U);
    }
    if (!IsNCAxisIdentitySnapshotValid(candidate)) return false;
    output = candidate;
    return true;
}

// Linear native values are mm; rotary values always remain native degrees.
// Never wrap a continuous angle or apply inch conversion to a rotary axis.
// Any invalid input or nonfinite result leaves output bit-for-bit unchanged.
inline bool TryNCAxisNativeToProgram(const NCAxisIdentitySnapshot& s,
    unsigned axis, int unitsMode, double value, double& output) noexcept
{
    if (!IsNCAxisIdentitySnapshotValid(s) || axis >= 8U || s.exists[axis] != 1U ||
        (unitsMode != 20 && unitsMode != 21) || !std::isfinite(value)) return false;
    const double candidate = s.nativeUnit[axis] == 1U && unitsMode == 20 ?
        value / 25.4 : value;
    if (!std::isfinite(candidate)) return false;
    output = candidate;
    return true;
}

inline bool TryNCAxisProgramToNative(const NCAxisIdentitySnapshot& s,
    unsigned axis, int unitsMode, double value, double& output) noexcept
{
    if (!IsNCAxisIdentitySnapshotValid(s) || axis >= 8U || s.exists[axis] != 1U ||
        (unitsMode != 20 && unitsMode != 21) || !std::isfinite(value)) return false;
    const double candidate = s.nativeUnit[axis] == 1U && unitsMode == 20 ?
        value * 25.4 : value;
    if (!std::isfinite(candidate)) return false;
    output = candidate;
    return true;
}

static_assert(sizeof(NCAxisIdentitySnapshot) == 48U &&
    alignof(NCAxisIdentitySnapshot) == 1U &&
    std::is_trivially_copyable<NCAxisIdentitySnapshot>::value &&
    std::is_standard_layout<NCAxisIdentitySnapshot>::value,
    "Axis identity must be a bounded, padding-free transport value.");
static_assert(offsetof(NCAxisIdentitySnapshot, exists) == 0U &&
    offsetof(NCAxisIdentitySnapshot, axisType) == 8U &&
    offsetof(NCAxisIdentitySnapshot, nativeUnit) == 16U &&
    offsetof(NCAxisIdentitySnapshot, physicalIndexPlusOne) == 24U &&
    offsetof(NCAxisIdentitySnapshot, address) == 32U &&
    offsetof(NCAxisIdentitySnapshot, electrodeAxisPlusOne) == 40U &&
    offsetof(NCAxisIdentitySnapshot, systemMode) == 41U &&
    offsetof(NCAxisIdentitySnapshot, eccentricEnabled) == 42U &&
    offsetof(NCAxisIdentitySnapshot, bound) == 43U &&
    offsetof(NCAxisIdentitySnapshot, reserved) == 44U,
    "Exact identity comparison requires explicit byte offsets.");
