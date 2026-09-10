#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <type_traits>

// NC-0.2L.2BX / V2-01: finite XYZ G01 geometry and feed conversion.
// This value describes a common-parameter line in MCS millimetres and in
// logical pulse coordinates. It is separate from retained G00 endpoint chords.
// Input/output are caller-owned, must not overlap, and must remain unchanged
// while borrowed. A successful output is an immutable transaction value, not
// a live owner/epoch proof. There is no allocation, queue, I/O, or RTOS call.
//
// Only axisMask 1..7 is supported. All eight coordinate slots must be finite;
// unselected endpoints must retain exactly the same binary64 representation.
// Selected axes require a finite positive maxVelocityPPS even for a point.
// The caller supplies consistent positive pulse-per-mm conversions; the
// builder rejects zero/nonzero or direction disagreement between the spaces.
// No tolerance converts a small nonzero line to a point. A point has zero
// velocityPPS and requires the producer's explicit terminal-point handling.
//
// F is explicit mm/min, 0 < F <= 100. For a non-point line, pulse group speed
// is F * lengthPulse / (60 * lengthMM). Its projection on every selected axis
// must fit that axis's configured maximum; there is no hidden speed clamp.
// Formula evaluation scales exponents to avoid avoidable intermediate
// overflow/underflow. Unrepresentable differences, lengths, or speeds reject.
// Strict/precise IEEE binary64 arithmetic and the normal FP environment are
// required. No code in this unit changes the floating-point environment.
#if defined(__FAST_MATH__) || defined(_M_FP_FAST)
#error BX_feed_line_requires_precise_floating_point_semantics
#endif
#if defined(__FINITE_MATH_ONLY__) && (__FINITE_MATH_ONLY__ > 0)
#error BX_feed_line_requires_nonfinite_checks
#endif

enum class NCPathCoreFeedLineCode : std::uint8_t
{
    NOT_BUILT = 0U,
    BUILT_LINE = 1U,
    BUILT_POINT = 2U,
    INVALID_AXIS_MASK = 3U,
    INVALID_FEED = 4U,
    NONFINITE_COORDINATE = 5U,
    OUTSIDE_AXIS_CHANGED = 6U,
    INVALID_AXIS_CONFIGURATION = 7U,
    NONFINITE_GEOMETRY = 8U,
    ZERO_LENGTH_MISMATCH = 9U,
    DIRECTION_MISMATCH = 10U,
    INVALID_GROUP_VELOCITY = 11U,
    AXIS_VELOCITY_LIMIT = 12U
};

struct NCPathCoreFeedLineInput
{
    std::array<double, 8> startMCS{};
    std::array<double, 8> endMCS{};
    std::array<double, 8> startPulse{};
    std::array<double, 8> endPulse{};
    std::array<double, 8> maxVelocityPPS{};
    std::uint32_t axisMask = 0U;
    double feedMMMin = 0.0;
};

struct NCPathCoreFeedLineV2
{
    std::array<double, 8> startMCS{};
    std::array<double, 8> endMCS{};
    std::array<double, 8> startPulse{};
    std::array<double, 8> endPulse{};
    double lengthMM = 0.0;
    double lengthPulse = 0.0;
    double feedMMMin = 0.0;
    double velocityPPS = 0.0;
    std::uint32_t axisMask = 0U;
    bool point = false;
    bool valid = false;

    void Clear() noexcept;
};

// Every rejection clears all output fields. Only BUILT_LINE / BUILT_POINT
// permit use of the output. No aggregate temporary is created by this call.
NCPathCoreFeedLineCode BuildNCPathCoreFeedLine(
    const NCPathCoreFeedLineInput& input,
    NCPathCoreFeedLineV2& output) noexcept;

// Returns MCS millimetres for one slot at the common line parameter u.
// u must be finite and in [0,1]; invalid arguments clear output to +0.
// At u=0/1 the original endpoint representation is copied exactly. Interior
// results are bounded rounded values; neither bitwise reversal nor executed
// servo position is claimed. The built value must be immutable during use.
bool EvaluateNCPathCoreFeedLineAxis(
    const NCPathCoreFeedLineV2& line,
    std::uint32_t axisIndex,
    double unitParameter,
    double& outputMCS) noexcept;

static_assert(std::numeric_limits<double>::is_iec559 && sizeof(double) == 8U,
    "BX feed line requires IEEE binary64 double.");
static_assert(sizeof(NCPathCoreFeedLineInput) == 336U,
    "BX caller-owned feed-line input budget changed.");
static_assert(sizeof(NCPathCoreFeedLineV2) == 296U,
    "BX immutable feed-line value budget changed.");
static_assert(std::is_standard_layout<NCPathCoreFeedLineV2>::value&&
    std::is_trivially_copyable<NCPathCoreFeedLineV2>::value,
    "BX feed-line value must remain a simple copyable value.");
