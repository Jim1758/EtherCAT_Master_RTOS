#pragma once

#include <array>
#include <cstdint>
#include <cstddef>
#include <limits>
#include <type_traits>
#include "NCArcPlane.h"

// BASE-PLANE-1: finite G17/G18/G19 circular geometry, equal positive
// pulse/mm on the selected plane, F in (0,100] mm/min, relative IJK center.
// All storage is caller-owned. Input/output must not overlap or change while
// borrowed. No allocation, RTOS call, I/O or floating-point environment change.
// Every out-of-plane slot retains its original binary64 representation.
// A full circle is explicit; coincident endpoints never imply one.
#if defined(__FAST_MATH__) || defined(_M_FP_FAST)
#error BY_feed_arc_requires_precise_floating_point_semantics
#endif
#if defined(__FINITE_MATH_ONLY__) && (__FINITE_MATH_ONLY__ > 0)
#error BY_feed_arc_requires_nonfinite_checks
#endif

enum class NCPathCoreFeedArcCode : std::uint8_t
{
    NOT_BUILT = 0U,
    BUILT_ARC = 1U,
    INVALID_DIRECTION = 2U,
    INVALID_FEED = 3U,
    NONFINITE_COORDINATE = 4U,
    OUTSIDE_AXIS_CHANGED = 5U,
    INVALID_AXIS_CONFIGURATION = 6U,
    UNEQUAL_AXIS_SCALE = 7U,
    NONFINITE_GEOMETRY = 8U,
    INVALID_RADIUS = 9U,
    RADIUS_MISMATCH = 10U,
    SPACE_MISMATCH = 11U,
    INVALID_FULL_CIRCLE = 12U,
    DEGENERATE_ARC = 13U,
    PRECISION_BUDGET = 14U,
    INVALID_GROUP_VELOCITY = 15U,
    AXIS_VELOCITY_LIMIT = 16U,
    INVALID_PLANE = 17U
};

struct NCPathCoreFeedArcInput
{
    std::array<double, 8> startMCS{};
    std::array<double, 8> endMCS{};
    std::array<double, 8> startPulse{};
    std::array<double, 8> endPulse{};
    std::array<double, 2> centerOffsetMM{};
    std::array<double, 2> pulsePerMM{};
    std::array<double, 2> maxVelocityPPS{};
    double feedMMMin = 0.0;
    int direction = 0; // -1 CW, +1 CCW
    bool fullCircle = false;
    std::uint8_t plane = 17U; // Canonical order from NCArcPlaneAxes; existing padding.
};

struct NCPathCoreFeedArcV2
{
    std::array<double, 8> startMCS{};
    std::array<double, 8> endMCS{};
    std::array<double, 8> startPulse{};
    std::array<double, 8> endPulse{};
    std::array<double, 2> centerMCS{};
    std::array<double, 2> centerPulse{};
    std::array<double, 2> boundsMinMCS{};
    std::array<double, 2> boundsMaxMCS{};
    double radiusMM = 0.0;
    double radiusPulse = 0.0;
    double startAngle = 0.0;
    double sweepRadians = 0.0;
    double lengthMM = 0.0;
    double lengthPulse = 0.0;
    double feedMMMin = 0.0;
    double velocityPPS = 0.0;
    int direction = 0;
    std::uint32_t axisMask = 0U;
    bool fullCircle = false;
    bool valid = false;
    std::uint8_t plane = 17U; // Existing tail padding, not inferred from endpoint words.
    // Upward-rounded frozen-source allowance in native mm. Uses tail padding;
    // carries no execution authority and never changes canonical geometry.
    float sourceRoundoffMM = 0.0f;

    void Clear() noexcept;
};

// Shared by the builder and the actual Motion consumer. Radius is fixed to
// the sampled start radius; endpoint disagreement is roundoff-only (64 eps
// times the coordinate/radius scale plus an optional frozen-source allowance).
// This never resolves a spiral. The caller derives any extra allowance from
// its validated source; builder/consumer enforce a combined 1e-7 mm ceiling.
struct NCPathCoreArcPulseGeometry
{
    double radius = 0.0;
    double startAngle = 0.0;
    double sweepRadians = 0.0;
    double lengthPulse = 0.0;
};

bool ResolveNCPathCorePlanarCirclePulse(double sx, double sy,
    double ex, double ey, double cx, double cy, double expectedRadius,
    int direction, bool fullCircle,
    NCPathCoreArcPulseGeometry& output, double additionalTolerance = 0.0) noexcept;

// Every rejection clears output. Only BUILT_ARC permits its use. Bounds cover
// every cardinal extremum on the directed sweep, for whole-arc limit checks.
NCPathCoreFeedArcCode BuildNCPathCoreFeedArc(
    const NCPathCoreFeedArcInput& input,
    NCPathCoreFeedArcV2& output, float sourceRoundoffMM = 0.0f) noexcept;

// MCS coordinate at one common u in [0,1]. Endpoints are copied bit-exactly;
// interior in-plane samples use the fixed circle. Invalid arguments clear output.
bool EvaluateNCPathCoreFeedArcAxis(const NCPathCoreFeedArcV2& arc,
    std::uint32_t axisIndex, double unitParameter,
    double& outputMCS) noexcept;

static_assert(std::numeric_limits<double>::is_iec559 && sizeof(double) == 8U,
    "BY feed arc requires IEEE binary64 double.");
static_assert(sizeof(NCPathCoreFeedArcInput) == 320U,
    "BY caller-owned arc input budget changed.");
static_assert(offsetof(NCPathCoreFeedArcInput, plane) == 317U &&
    offsetof(NCPathCoreFeedArcV2, plane) == 394U,
    "Plane identity must use existing padding, preserving all old fields.");
static_assert(sizeof(float) == 4U && std::numeric_limits<float>::is_iec559 &&
    offsetof(NCPathCoreFeedArcV2, sourceRoundoffMM) == 396U,
    "Source roundoff must occupy the existing arc tail padding.");
static_assert(sizeof(NCPathCoreFeedArcV2) == 400U,
    "BY immutable arc value budget changed.");
static_assert(std::is_standard_layout<NCPathCoreFeedArcV2>::value&&
    std::is_trivially_copyable<NCPathCoreFeedArcV2>::value,
    "BY feed arc must remain a simple copyable value.");
