#pragma once

#include <cstdint>

// BASE-PLANE-1: right-handed circular coordinates, viewed from the positive
// normal axis. G18 is (Z,X), NOT (X,Z); this preserves G02/G03 orientation.
// These are audited XYZ physical slots only. NC must separately prove the
// configured X/Y/Z names and linear/native identity before admitting motion.
struct NCArcPlaneAxes
{
    unsigned u = 0U;
    unsigned v = 1U;
    unsigned normal = 2U;
    char uAddress = 'X';
    char vAddress = 'Y';
    char uCenter = 'I';
    char vCenter = 'J';
    std::uint32_t mask = 3U;
};

inline bool TryGetNCArcPlaneAxes(int plane, NCArcPlaneAxes& axes) noexcept
{
    axes = NCArcPlaneAxes{};
    switch (plane)
    {
    case 17: return true;
    case 18:
        axes.u = 2U; axes.v = 0U; axes.normal = 1U;
        axes.uAddress = 'Z'; axes.vAddress = 'X';
        axes.uCenter = 'K'; axes.vCenter = 'I'; axes.mask = 5U;
        return true;
    case 19:
        axes.u = 1U; axes.v = 2U; axes.normal = 0U;
        axes.uAddress = 'Y'; axes.vAddress = 'Z';
        axes.uCenter = 'J'; axes.vCenter = 'K'; axes.mask = 6U;
        return true;
    default: return false;
    }
}

// BASE-PLANE-2: a straight line keeps native XYZ address/slot order in
// every plane. Plane selection affects circles, never swaps X/Y/Z endpoints.
// This bounded mapping is shared by the new producer and consumer gates.
inline bool IsNCNativeXYZLinearMapping(int count, const int* axes) noexcept
{
    if (axes == nullptr || count < 1 || count > 3) return false;
    int previous = -1;
    for (int slot = 0; slot < count; ++slot)
    {
        if (axes[slot] < 0 || axes[slot] > 2 || axes[slot] <= previous) return false;
        previous = axes[slot];
    }
    return true;
}

// BASE-PLANE-8: planar cutter LINES keep ascending physical XYZ slots.
// G18 therefore sends X/Z (0/2) to the line interpolator, while its pure
// cutter geometry uses right-handed Z/X (2/0). Never conflate these maps.
inline bool IsNCPlaneLinearPairMapping(int planeCode, int count, const int* axes) noexcept
{
    NCArcPlaneAxes plane{};
    return count == 2 && TryGetNCArcPlaneAxes(planeCode, plane) &&
        IsNCNativeXYZLinearMapping(count, axes) &&
        ((1U << static_cast<unsigned>(axes[0])) | (1U << static_cast<unsigned>(axes[1]))) == plane.mask;
}

inline bool IsNCArcPlaneCode(int plane) noexcept
{
    return plane == 17 || plane == 18 || plane == 19;
}

// BASE-PLANE-5: polar endpoint notation uses the same canonical (u,v)
// basis as circles: radius=u (native length), angle=v (degrees).
// G18 is Z radius / X angle, NOT the unaudited legacy X/Z convention.
// This only classifies authored endpoint words, never IJK or table fields.
inline bool IsNCPolarAngleAxis(bool polar, int plane, unsigned axis) noexcept
{
    NCArcPlaneAxes axes{};
    return polar && TryGetNCArcPlaneAxes(plane, axes) && axis == axes.v;
}
