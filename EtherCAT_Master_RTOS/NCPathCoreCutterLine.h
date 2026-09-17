#pragma once

#include <cmath>
#include <limits>
#include <type_traits>

// Pure, bounded G17 line-cutter construction in native MCS millimetres.
// The caller owns source/epoch proof, contour-state commit and travel checks.
// signedRadius is the physical radius with its final native handedness:
// positive is left, negative is right. No coordinate/unit/scale transform,
// table access, allocation, queue operation or servo-position read occurs here.
// All four point arrays must be finite, including an unused nominalNext.
// Entry is an uncompensated lead-in and requires one outgoing contour line.
// Ordinary lines intersect their incoming/outgoing offset lines; the last
// line before a separately validated G40 lead-out ends at its normal offset.
#if defined(__FAST_MATH__) || defined(_M_FP_FAST)
#error Cutter_line_requires_precise_floating_point_semantics
#endif
#if defined(__FINITE_MATH_ONLY__) && (__FINITE_MATH_ONLY__ > 0)
#error Cutter_line_requires_nonfinite_checks
#endif

enum class NCPathCoreCutterLineCode : unsigned char
{
    NOT_BUILT = 0U,
    BUILT = 1U,
    NONFINITE_INPUT = 2U,
    INVALID_RADIUS = 3U,
    MISSING_ENTRY_SUCCESSOR = 4U,
    ZERO_NOMINAL_LINE = 5U,
    NONFINITE_GEOMETRY = 6U,
    PRECISION_BUDGET = 7U,
    ENTRY_BASELINE_MISMATCH = 8U,
    ENTRY_TOO_SHORT = 9U,
    OFFSET_BASELINE_MISMATCH = 10U,
    REVERSAL = 11U,
    MITER_LIMIT = 12U,
    NEXT_LINE_CONSUMED = 13U,
    TRIMMED_LINE_REVERSED = 14U
};

struct NCPathCoreCutterLineInput
{
    double nominalStart[2] = {};
    double nominalEnd[2] = {};
    double nominalNext[2] = {};
    double actualStart[2] = {};
    double signedRadius = 0.0;
    bool entry = false;
    bool hasNext = false;
};

struct NCPathCoreCutterLineOutput
{
    double endpoint[2] = {};
    NCPathCoreCutterLineCode reason = NCPathCoreCutterLineCode::NOT_BUILT;
    bool valid = false;
};

// Every failure clears the endpoint and validity; reason is always returned
// and stored. Inputs/output must not overlap. At most two line directions are
// examined. The arithmetic allowance is 64 eps times native coordinate/length
// magnitude, never a tolerance for a genuinely reversed or consumed segment.
inline NCPathCoreCutterLineCode BuildNCPathCoreCutterLine(
    const NCPathCoreCutterLineInput& input,
    NCPathCoreCutterLineOutput& output) noexcept
{
    output = NCPathCoreCutterLineOutput{};
    const auto fail = [&output](NCPathCoreCutterLineCode reason) noexcept
    {
        output.reason = reason;
        return reason;
    };
    double magnitude = 0.0;
    const double* points[4] = {input.nominalStart, input.nominalEnd,
        input.nominalNext, input.actualStart};
    for (unsigned point = 0U; point < 4U; ++point)
        for (unsigned axis = 0U; axis < 2U; ++axis)
        {
            const double value = points[point][axis];
            if (!std::isfinite(value))
                return fail(NCPathCoreCutterLineCode::NONFINITE_INPUT);
            if ((point != 2U || input.hasNext) && std::fabs(value) > magnitude)
                magnitude = std::fabs(value);
        }
    if (!std::isfinite(input.signedRadius))
        return fail(NCPathCoreCutterLineCode::NONFINITE_INPUT);
    const double radius = std::fabs(input.signedRadius);
    if (radius == 0.0) return fail(NCPathCoreCutterLineCode::INVALID_RADIUS);
    if (radius > magnitude) magnitude = radius;
    if (input.entry && !input.hasNext)
        return fail(NCPathCoreCutterLineCode::MISSING_ENTRY_SUCCESSOR);

    const double dx = input.nominalEnd[0] - input.nominalStart[0];
    const double dy = input.nominalEnd[1] - input.nominalStart[1];
    const double length = std::hypot(dx, dy);
    if (!std::isfinite(length))
        return fail(NCPathCoreCutterLineCode::NONFINITE_GEOMETRY);
    if (length == 0.0) return fail(NCPathCoreCutterLineCode::ZERO_NOMINAL_LINE);
    if (length > magnitude) magnitude = length;
    const double ux = dx / length, uy = dy / length;
    double vx = ux, vy = uy, nextLength = 0.0;
    if (input.hasNext)
    {
        const double nx = input.nominalNext[0] - input.nominalEnd[0];
        const double ny = input.nominalNext[1] - input.nominalEnd[1];
        nextLength = std::hypot(nx, ny);
        if (!std::isfinite(nextLength))
            return fail(NCPathCoreCutterLineCode::NONFINITE_GEOMETRY);
        if (nextLength == 0.0)
            return fail(NCPathCoreCutterLineCode::ZERO_NOMINAL_LINE);
        if (nextLength > magnitude) magnitude = nextLength;
        vx = nx / nextLength;
        vy = ny / nextLength;
    }
    const double budget = 64.0 * std::numeric_limits<double>::epsilon() * magnitude;
    if (!std::isfinite(budget) || radius <= budget || length <= budget ||
        (input.hasNext && nextLength <= budget))
        return fail(NCPathCoreCutterLineCode::PRECISION_BUDGET);

    const double startDX = input.actualStart[0] - input.nominalStart[0];
    const double startDY = input.actualStart[1] - input.nominalStart[1];
    if (!std::isfinite(startDX) || !std::isfinite(startDY))
        return fail(NCPathCoreCutterLineCode::NONFINITE_GEOMETRY);
    if (input.entry)
    {
        if (std::fabs(startDX) > budget || std::fabs(startDY) > budget)
            return fail(NCPathCoreCutterLineCode::ENTRY_BASELINE_MISMATCH);
        if (length <= radius || length - radius <= budget)
            return fail(NCPathCoreCutterLineCode::ENTRY_TOO_SHORT);
    }
    else
    {
        const double residual = -uy * startDX + ux * startDY - input.signedRadius;
        if (!std::isfinite(residual))
            return fail(NCPathCoreCutterLineCode::NONFINITE_GEOMETRY);
        if (std::fabs(residual) > budget)
            return fail(NCPathCoreCutterLineCode::OFFSET_BASELINE_MISMATCH);
    }

    // Endpoint displacement expressed per unit signed radius. Computing the
    // miter ratio first avoids overflow in an otherwise bounded radius product.
    double denominator = 2.0;
    if (input.hasNext)
    {
        denominator = 1.0 + ux * vx + uy * vy;
        if (!std::isfinite(denominator) ||
            denominator <= 64.0 * std::numeric_limits<double>::epsilon())
            return fail(NCPathCoreCutterLineCode::REVERSAL);
    }
    double normalX = -uy, normalY = ux;
    if (input.entry)
    {
        normalX = -vy;
        normalY = vx;
    }
    else if (input.hasNext)
    {
        normalX = (-uy - vy) / denominator;
        normalY = (ux + vx) / denominator;
        const double miterRatio = std::hypot(normalX, normalY);
        if (!std::isfinite(miterRatio))
            return fail(NCPathCoreCutterLineCode::NONFINITE_GEOMETRY);
        if (miterRatio > 4.0) return fail(NCPathCoreCutterLineCode::MITER_LIMIT);
    }
    const double shiftX = input.signedRadius * normalX;
    const double shiftY = input.signedRadius * normalY;
    const double endX = input.nominalEnd[0] + shiftX;
    const double endY = input.nominalEnd[1] + shiftY;
    if (!std::isfinite(shiftX) || !std::isfinite(shiftY) ||
        !std::isfinite(endX) || !std::isfinite(endY))
        return fail(NCPathCoreCutterLineCode::NONFINITE_GEOMETRY);

    // A positive outgoing trim must leave room for the next authored segment;
    // an exterior miter extends the line and does not consume its forward span.
    if (!input.entry && input.hasNext)
    {
        const double nextTrim = shiftX * vx + shiftY * vy;
        if (!std::isfinite(nextTrim))
            return fail(NCPathCoreCutterLineCode::NONFINITE_GEOMETRY);
        if (nextTrim > 0.0 && nextLength - nextTrim <= budget)
            return fail(NCPathCoreCutterLineCode::NEXT_LINE_CONSUMED);
    }
    const double actualDX = endX - input.actualStart[0];
    const double actualDY = endY - input.actualStart[1];
    const double forward = actualDX * ux + actualDY * uy;
    if (!std::isfinite(actualDX) || !std::isfinite(actualDY) || !std::isfinite(forward))
        return fail(NCPathCoreCutterLineCode::NONFINITE_GEOMETRY);
    if (forward <= budget)
        return fail(NCPathCoreCutterLineCode::TRIMMED_LINE_REVERSED);
    output.endpoint[0] = endX;
    output.endpoint[1] = endY;
    output.reason = NCPathCoreCutterLineCode::BUILT;
    output.valid = true;
    return output.reason;
}

static_assert(std::numeric_limits<double>::is_iec559 && sizeof(double) == 8U,
    "Cutter line geometry requires IEEE binary64 arithmetic.");
static_assert(std::is_standard_layout<NCPathCoreCutterLineInput>::value &&
    std::is_trivially_copyable<NCPathCoreCutterLineInput>::value &&
    std::is_standard_layout<NCPathCoreCutterLineOutput>::value &&
    std::is_trivially_copyable<NCPathCoreCutterLineOutput>::value,
    "Cutter line inputs and output must remain bounded copyable values.");
