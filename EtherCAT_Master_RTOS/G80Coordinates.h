#pragma once

#include "NCTranslationSnapshot.h"
#include <array>
#include <cstdint>

// NC-thread geometry only. These values grant no Motion admission or ownership.
namespace G80
{
using Point6 = std::array<double, 6U>; // Fixed physical slots: X Y Z C U V.

enum class CoordinateError : std::uint32_t
{
    None = 0U,
    InvalidFrame,
    AxisIdentity,
    NonfiniteInput,
    ZeroDirection,
    AbsentAxis,
    EccentricRole,
    NumericRange,
    InvalidOriginAction,
    InvalidOriginMask,
    CounterExhausted,
    OriginRoundoff,
    UnsupportedCutter
};

// Direction words are Cartesian/native components, independent of G90/G91,
// G16 and G20. Output preserves ratios: it is NOT a mixed mm/degree unit vector.
// P0 resolves compensation once at startMCS, including the instantaneous
// eccentric-H derivative, and then freezes that straight native direction.
// P1 returns the native components unchanged. G40 is required: this snapshot
// does not contain the active G41/G42 contour tangent/offset provenance.
// All failures preserve output.
bool ResolveDirection(const NCTranslationSnapshot& frame, const Point6& startMCS,
    const Point6& authoredDirection, bool mechanical, Point6& output,
    CoordinateError& error) noexcept;

// XYZ values are native mm, C/U/V are unwrapped native degrees. Eccentric H is
// evaluated at this point's physical role angle, never its WCS angle.
bool MachineToWork(const NCTranslationSnapshot& frame, const Point6& machine,
    Point6& work, CoordinateError& error) noexcept;

// EDM277: views of ONE supplied native point in ONE immutable G80 frame.
// For averaged contacts, the caller supplies mean MCS and these are transforms
// of that mean; nonlinear eccentric H means this need not equal mean(WCS).
// A separate call using the mean commanded point at the same contact latches
// produces the commanded/"ideal" comparison. Actual cannot infer commanded.
struct CoordinateViews
{
    Point6 machine{};
    Point6 work{};
    // Re-project work with G68 and WORK rotations cancelled. Retains signed
    // H (including its contact-angle eccentric orientation), EXT/WCS/WORK
    // translations, scaling and mirrors. This is a comparison point, not a
    // different reading of actual machine position or a command to move there.
    Point6 frameRotationRemovedMachine{};
    // Re-project work with G43/G44 signed H removed; preserves all other
    // transforms. This is the existing H model, not general multi-axis TCP.
    Point6 toolOffsetRemovedMachine{};
    // Re-project work with both frame rotations and signed H removed.
    Point6 frameRotationAndToolOffsetRemovedMachine{};
};

// Let W = inverse(F, machine). Each comparison is F_without_named_parts(W),
// not inverse(F_without_named_parts, machine). Rotation centres and transform
// order remain those of F; only the named transforms are cancelled. All views
// use native mm and unwrapped degrees, even under G20/G16. The active frame is
// never mutated. Failure preserves all output fields bit-for-bit.
bool BuildCoordinateViews(const NCTranslationSnapshot& frame, const Point6& machine,
    CoordinateViews& output, CoordinateError& error) noexcept;

// action: 0 -> zero, 1 -> L, 2 -> prior mean WCS / 2. L always means native
// mm/degrees, including G20; no inch conversion belongs in this G80 contract.
// authoredMask contains only explicitly authored NONZERO direction axes.
// Each selected axis receives the same scalar L for A1. Generated compensation
// axes retain their prior work value. No output changes on failure.
bool BuildOriginSnapshot(const NCTranslationSnapshot& previous,
    const Point6& meanMCS, std::uint32_t authoredMask, int action, double lValue,
    NCTranslationSnapshot& candidate, Point6& meanWCS, Point6& desiredWCS,
    CoordinateError& error) noexcept;

// Narrow same-bank origin descriptor transition. Motion still must separately
// prove exact stop/drain, epoch, current owner lease and current source.
bool IsOriginSnapshotTransition(const NCTranslationSnapshot& previous,
    const NCTranslationSnapshot& candidate) noexcept;
}
