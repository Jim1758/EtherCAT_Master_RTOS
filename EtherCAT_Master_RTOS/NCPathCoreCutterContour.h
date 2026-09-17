#pragma once

#include "NCPathCoreCutterLine.h"

// Pure bounded XY cutter geometry in native MCS millimetres. The physical
// signed radius already includes the transformed frame's handedness. The
// caller owns frame/source proof, modal restrictions, commits and travel
// checks. No allocation, machine access or coordinate conversion occurs here.
// Entry and a separate G40 lead-out are straight lines. Arcs are planar,
// non-full circles; centre coordinates are absolute native MCS coordinates.
// The closest bounded intersection is selected before forward-span checks:
// a consumed nearest junction never silently selects the other circle root.
enum class NCPathCoreCutterPrimitiveKind : unsigned char { LINE = 0U, ARC = 1U };

enum class NCPathCoreCutterContourCode : unsigned char
{
    NOT_BUILT = 0U, BUILT = 1U, NONFINITE_INPUT = 2U, INVALID_RADIUS = 3U,
    MISSING_ENTRY_SUCCESSOR = 4U, ZERO_NOMINAL_LINE = 5U,
    NONFINITE_GEOMETRY = 6U, PRECISION_BUDGET = 7U,
    ENTRY_BASELINE_MISMATCH = 8U, ENTRY_TOO_SHORT = 9U,
    OFFSET_BASELINE_MISMATCH = 10U, REVERSAL = 11U, MITER_LIMIT = 12U,
    NEXT_LINE_CONSUMED = 13U, TRIMMED_LINE_REVERSED = 14U,
    INVALID_PRIMITIVE = 15U, ARC_ENTRY = 16U, ARC_RADIUS_MISMATCH = 17U,
    FULL_CIRCLE = 18U, ARC_RADIUS_COLLAPSE = 19U,
    JUNCTION_MISMATCH = 20U, NO_INTERSECTION = 21U,
    AMBIGUOUS_INTERSECTION = 22U, NEXT_ARC_CONSUMED = 23U,
    TRIMMED_ARC_REVERSED = 24U
};

struct NCPathCoreCutterPrimitive
{
    NCPathCoreCutterPrimitiveKind kind = NCPathCoreCutterPrimitiveKind::LINE;
    double start[2] = {};
    double end[2] = {};
    double centre[2] = {};
    bool clockwise = false;
};
struct NCPathCoreCutterContourInput
{
    NCPathCoreCutterPrimitive current;
    NCPathCoreCutterPrimitive next;
    double actualStart[2] = {};
    double signedRadius = 0.0;
    bool entry = false;
    bool hasNext = false;
};
struct NCPathCoreCutterContourOutput
{
    double endpoint[2] = {};
    double centre[2] = {};
    double radius = 0.0;
    double sweepRadians = 0.0;
    NCPathCoreCutterContourCode reason = NCPathCoreCutterContourCode::NOT_BUILT;
    bool valid = false;
};

namespace NCPathCoreCutterContourDetail
{
    using Code = NCPathCoreCutterContourCode;
    constexpr double Pi = 3.1415926535897932384626433832795;
    constexpr double Tau = 2.0 * Pi;
    struct Curve
    {
        bool arc = false;
        double ux = 0.0, uy = 0.0;
        double endTX = 0.0, endTY = 0.0;
        double radius = 0.0, sweep = 0.0, length = 0.0;
        double offsetStart[2] = {}, offsetEnd[2] = {};
    };
    inline double Distance(const double* a, const double* b) noexcept
    { return std::hypot(a[0] - b[0], a[1] - b[1]); }
    inline double Angle(const double* from, const double* to,
        const double* centre, bool clockwise) noexcept
    {
        const double fx = from[0] - centre[0], fy = from[1] - centre[1];
        const double tx = to[0] - centre[0], ty = to[1] - centre[1];
        const double fl = std::hypot(fx, fy), tl = std::hypot(tx, ty);
        const double ux = fx / fl, uy = fy / fl, vx = tx / tl, vy = ty / tl;
        const double angle = std::atan2(ux * vy - uy * vx, ux * vx + uy * vy);
        return clockwise ? -angle : angle;
    }
    inline Code Prepare(const NCPathCoreCutterPrimitive& p, double signedRadius,
        double budget, Curve& out) noexcept
    {
        out.arc = p.kind == NCPathCoreCutterPrimitiveKind::ARC;
        if (!out.arc)
        {
            const double dx = p.end[0] - p.start[0], dy = p.end[1] - p.start[1];
            out.length = std::hypot(dx, dy);
            if (!std::isfinite(out.length)) return Code::NONFINITE_GEOMETRY;
            if (out.length == 0.0) return Code::ZERO_NOMINAL_LINE;
            if (out.length <= budget) return Code::PRECISION_BUDGET;
            out.ux = dx / out.length; out.uy = dy / out.length;
            out.endTX = out.ux; out.endTY = out.uy;
            out.offsetStart[0] = p.start[0] - signedRadius * out.uy;
            out.offsetStart[1] = p.start[1] + signedRadius * out.ux;
            out.offsetEnd[0] = p.end[0] - signedRadius * out.uy;
            out.offsetEnd[1] = p.end[1] + signedRadius * out.ux;
        }
        else
        {
            const double r0 = Distance(p.start, p.centre), r1 = Distance(p.end, p.centre);
            if (!std::isfinite(r0) || !std::isfinite(r1)) return Code::NONFINITE_GEOMETRY;
            if (r0 <= budget || r1 <= budget) return Code::PRECISION_BUDGET;
            if (std::fabs(r0 - r1) > budget) return Code::ARC_RADIUS_MISMATCH;
            if (Distance(p.start, p.end) <= budget) return Code::FULL_CIRCLE;
            const double direction = p.clockwise ? -1.0 : 1.0;
            out.radius = r0 - direction * signedRadius;
            if (!std::isfinite(out.radius)) return Code::NONFINITE_GEOMETRY;
            if (out.radius <= budget) return Code::ARC_RADIUS_COLLAPSE;
            out.sweep = Angle(p.start, p.end, p.centre, p.clockwise);
            if (out.sweep < 0.0) out.sweep += Tau;
            if (!std::isfinite(out.sweep)) return Code::NONFINITE_GEOMETRY;
            if (out.sweep <= budget / r0 || Tau - out.sweep <= budget / r0)
                return Code::FULL_CIRCLE;
            const double sx = (p.start[0] - p.centre[0]) / r0;
            const double sy = (p.start[1] - p.centre[1]) / r0;
            const double ex = (p.end[0] - p.centre[0]) / r1;
            const double ey = (p.end[1] - p.centre[1]) / r1;
            out.ux = -direction * sy; out.uy = direction * sx;
            out.endTX = -direction * ey; out.endTY = direction * ex;
            out.offsetStart[0] = p.centre[0] + out.radius * sx;
            out.offsetStart[1] = p.centre[1] + out.radius * sy;
            out.offsetEnd[0] = p.centre[0] + out.radius * ex;
            out.offsetEnd[1] = p.centre[1] + out.radius * ey;
        }
        for (unsigned a = 0U; a < 2U; ++a)
            if (!std::isfinite(out.offsetStart[a]) || !std::isfinite(out.offsetEnd[a]))
                return Code::NONFINITE_GEOMETRY;
        return Code::BUILT;
    }
    // The centre-to-line foot avoids subtracting squared world coordinates.
    inline Code LineCircle(const Curve& line, const NCPathCoreCutterPrimitive& circle,
        const Curve& arc, double budget, double (&points)[2][2], unsigned& count) noexcept
    {
        const double dx = circle.centre[0] - line.offsetStart[0];
        const double dy = circle.centre[1] - line.offsetStart[1];
        const double along = dx * line.ux + dy * line.uy;
        const double normal = -dx * line.uy + dy * line.ux;
        const double gap = arc.radius - std::fabs(normal);
        if (!std::isfinite(along) || !std::isfinite(normal) || !std::isfinite(gap))
            return Code::NONFINITE_GEOMETRY;
        if (gap < -budget) return Code::NO_INTERSECTION;
        const double radicand = (gap > 0.0 ? gap : 0.0) * (arc.radius + std::fabs(normal));
        if (!std::isfinite(radicand)) return Code::NONFINITE_GEOMETRY;
        const double half = std::sqrt(radicand);
        const double fx = line.offsetStart[0] + along * line.ux;
        const double fy = line.offsetStart[1] + along * line.uy;
        count = half <= budget ? 1U : 2U;
        points[0][0] = fx + half * line.ux; points[0][1] = fy + half * line.uy;
        points[1][0] = fx - half * line.ux; points[1][1] = fy - half * line.uy;
        return Code::BUILT;
    }
    inline Code CircleCircle(const NCPathCoreCutterPrimitive& first, const Curve& a,
        const NCPathCoreCutterPrimitive& second, const Curve& b, double budget,
        double (&points)[2][2], unsigned& count) noexcept
    {
        const double dx = second.centre[0] - first.centre[0];
        const double dy = second.centre[1] - first.centre[1];
        const double distance = std::hypot(dx, dy);
        if (!std::isfinite(distance)) return Code::NONFINITE_GEOMETRY;
        if (distance <= budget)
        {
            if (std::fabs(a.radius - b.radius) > budget) return Code::NO_INTERSECTION;
            if (first.clockwise != second.clockwise) return Code::REVERSAL;
            // Consecutive arcs of the same nominal circle share its unique
            // radial offset at this authored junction, not an arbitrary root.
            if (Distance(a.offsetEnd, b.offsetStart) > budget) return Code::AMBIGUOUS_INTERSECTION;
            points[0][0] = a.offsetEnd[0]; points[0][1] = a.offsetEnd[1]; count = 1U;
            return Code::BUILT;
        }
        const double sum = a.radius + b.radius, difference = a.radius - b.radius;
        if (!std::isfinite(sum)) return Code::NONFINITE_GEOMETRY;
        if (distance > sum + budget || distance < std::fabs(difference) - budget)
            return Code::NO_INTERSECTION;
        const double along = 0.5 * (distance + (difference / distance) * sum);
        if (!std::isfinite(along)) return Code::NONFINITE_GEOMETRY;
        const double gap = a.radius - std::fabs(along);
        if (gap < -budget) return Code::NO_INTERSECTION;
        const double radicand = (gap > 0.0 ? gap : 0.0) * (a.radius + std::fabs(along));
        if (!std::isfinite(radicand)) return Code::NONFINITE_GEOMETRY;
        const double half = std::sqrt(radicand), ux = dx / distance, uy = dy / distance;
        const double fx = first.centre[0] + along * ux, fy = first.centre[1] + along * uy;
        count = half <= budget ? 1U : 2U;
        points[0][0] = fx - half * uy; points[0][1] = fy + half * ux;
        points[1][0] = fx + half * uy; points[1][1] = fy - half * ux;
        return Code::BUILT;
    }
}

inline NCPathCoreCutterContourCode BuildNCPathCoreCutterContour(
    const NCPathCoreCutterContourInput& input, NCPathCoreCutterContourOutput& output) noexcept
{
    using namespace NCPathCoreCutterContourDetail;
    output = NCPathCoreCutterContourOutput{};
    const auto fail = [&output](Code code) noexcept { output.reason = code; return code; };
    double magnitude = 0.0;
    const NCPathCoreCutterPrimitive* primitives[2] = { &input.current, &input.next };
    for (unsigned i = 0U; i < 2U; ++i)
    {
        const NCPathCoreCutterPrimitive& p = *primitives[i];
        if (p.kind != NCPathCoreCutterPrimitiveKind::LINE && p.kind != NCPathCoreCutterPrimitiveKind::ARC)
            return fail(Code::INVALID_PRIMITIVE);
        const double* points[3] = { p.start, p.end, p.centre };
        for (unsigned j = 0U; j < 3U; ++j)
            for (unsigned axis = 0U; axis < 2U; ++axis)
            {
                const double v = points[j][axis];
                if (!std::isfinite(v)) return fail(Code::NONFINITE_INPUT);
                if ((i == 0U || input.hasNext) && (j != 2U || p.kind == NCPathCoreCutterPrimitiveKind::ARC) && std::fabs(v) > magnitude)
                    magnitude = std::fabs(v);
            }
    }
    for (unsigned axis = 0U; axis < 2U; ++axis)
    {
        if (!std::isfinite(input.actualStart[axis])) return fail(Code::NONFINITE_INPUT);
        if (std::fabs(input.actualStart[axis]) > magnitude) magnitude = std::fabs(input.actualStart[axis]);
    }
    if (!std::isfinite(input.signedRadius)) return fail(Code::NONFINITE_INPUT);
    const double radius = std::fabs(input.signedRadius);
    if (radius == 0.0) return fail(Code::INVALID_RADIUS);
    if (input.entry && !input.hasNext) return fail(Code::MISSING_ENTRY_SUCCESSOR);
    if (input.entry && input.current.kind != NCPathCoreCutterPrimitiveKind::LINE) return fail(Code::ARC_ENTRY);
    if (radius > magnitude) magnitude = radius;
    const double budget = 64.0 * std::numeric_limits<double>::epsilon() * magnitude;
    if (!std::isfinite(budget) || radius <= budget) return fail(Code::PRECISION_BUDGET);
    if (input.hasNext && Distance(input.current.end, input.next.start) > budget)
        return fail(Code::JUNCTION_MISMATCH);

    if (input.current.kind == NCPathCoreCutterPrimitiveKind::LINE &&
        (!input.hasNext || input.next.kind == NCPathCoreCutterPrimitiveKind::LINE))
    {
        NCPathCoreCutterLineInput legacy;
        legacy.signedRadius = input.signedRadius; legacy.entry = input.entry; legacy.hasNext = input.hasNext;
        for (unsigned axis = 0U; axis < 2U; ++axis)
        {
            legacy.nominalStart[axis] = input.current.start[axis]; legacy.nominalEnd[axis] = input.current.end[axis];
            legacy.nominalNext[axis] = input.next.end[axis]; legacy.actualStart[axis] = input.actualStart[axis];
        }
        NCPathCoreCutterLineOutput built;
        const Code code = static_cast<Code>(BuildNCPathCoreCutterLine(legacy, built));
        if (!built.valid) return fail(code);
        output.endpoint[0] = built.endpoint[0]; output.endpoint[1] = built.endpoint[1];
        output.valid = true; output.reason = Code::BUILT; return output.reason;
    }

    Curve current, next;
    Code code = Prepare(input.current, input.signedRadius, budget, current);
    if (code != Code::BUILT) return fail(code);
    if (input.hasNext)
    {
        code = Prepare(input.next, input.signedRadius, budget, next);
        if (code != Code::BUILT) return fail(code);
    }
    if (input.entry)
    {
        if (Distance(input.actualStart, input.current.start) > budget) return fail(Code::ENTRY_BASELINE_MISMATCH);
        if (current.length - radius <= budget) return fail(Code::ENTRY_TOO_SHORT);
    }
    else if (current.arc)
    {
        if (std::fabs(Distance(input.actualStart, input.current.centre) - current.radius) > budget)
            return fail(Code::OFFSET_BASELINE_MISMATCH);
    }
    else
    {
        const double dx = input.actualStart[0] - input.current.start[0];
        const double dy = input.actualStart[1] - input.current.start[1];
        const double residual = -current.uy * dx + current.ux * dy - input.signedRadius;
        if (!std::isfinite(residual)) return fail(Code::NONFINITE_GEOMETRY);
        if (std::fabs(residual) > budget) return fail(Code::OFFSET_BASELINE_MISMATCH);
    }
    double candidates[2][2] = {}; unsigned count = 1U;
    if (input.entry)
    {
        candidates[0][0] = next.offsetStart[0]; candidates[0][1] = next.offsetStart[1];
    }
    else if (!input.hasNext)
    {
        candidates[0][0] = current.offsetEnd[0]; candidates[0][1] = current.offsetEnd[1];
    }
    else
    {
        if (1.0 + current.endTX * next.ux + current.endTY * next.uy <= 64.0 * std::numeric_limits<double>::epsilon())
            return fail(Code::REVERSAL);
        const double tangentCross = current.endTX * next.uy - current.endTY * next.ux;
        const double tangentDot = current.endTX * next.ux + current.endTY * next.uy;
        // Tangents come from differences of absolute MCS coordinates. Their
        // normalization can lose more than 64 eps in angle at a large origin.
        // Check that angular residual in the existing native length budget;
        // the larger nominal span makes this bound conservative for both
        // primitives. Still require forward direction and the two offset
        // points to coincide within that same budget. Distinct intersections,
        // reversal and all subsequent trimming/native guards stay unchanged.
        const double currentSpan = current.arc ?
            Distance(input.current.end, input.current.centre) : current.length;
        const double nextSpan = next.arc ?
            Distance(input.next.start, input.next.centre) : next.length;
        const double tangentSpan = currentSpan > nextSpan ? currentSpan : nextSpan;
        const bool tangentWithinLengthBudget =
            std::fabs(tangentCross) * tangentSpan <= budget;
        if (tangentDot > 0.0 &&
            (std::fabs(tangentCross) <= 64.0 * std::numeric_limits<double>::epsilon() ||
                tangentWithinLengthBudget) &&
            Distance(current.offsetEnd, next.offsetStart) <= budget)
        {
            // A proven nominal tangent has one radial/normal offset point.
            // For a mixed junction use the line's normal-offset spelling on
            // both sides. A circle's radial calculation can differ by one ULP
            // while producing the same pulse. Reusing that spelling for an
            // arc -> horizontal/vertical line would invent a nonzero MCS delta
            // with zero pulse delta. Canonicalize here, before either segment
            // is submitted; keep the native zero/direction guards unchanged.
            // Avoid taking sqrt of round-off near a double intersection root.
            const double* tangentPoint = current.arc && !next.arc ?
                next.offsetStart : current.offsetEnd;
            candidates[0][0] = tangentPoint[0]; candidates[0][1] = tangentPoint[1];
            count = 1U; code = Code::BUILT;
        }
        else if (!current.arc) code = LineCircle(current, input.next, next, budget, candidates, count);
        else if (!next.arc) code = LineCircle(next, input.current, current, budget, candidates, count);
        else code = CircleCircle(input.current, current, input.next, next, budget, candidates, count);
        if (code != Code::BUILT) return fail(code);
    }
    double distances[2] = {};
    for (unsigned i = 0U; i < count; ++i)
    {
        distances[i] = Distance(candidates[i], input.current.end);
        if (!std::isfinite(candidates[i][0]) || !std::isfinite(candidates[i][1]) || !std::isfinite(distances[i]))
            return fail(Code::NONFINITE_GEOMETRY);
    }
    if (count == 2U && std::fabs(distances[0] - distances[1]) <= budget)
        return fail(Code::AMBIGUOUS_INTERSECTION);
    const unsigned selected = count == 2U && distances[1] < distances[0] ? 1U : 0U;
    const double* endpoint = candidates[selected];
    if (distances[selected] / radius > 4.0) return fail(Code::MITER_LIMIT);
    if (input.hasNext && !input.entry)
    {
        if (next.arc)
        {
            const double trim = Angle(input.next.start, endpoint, input.next.centre, input.next.clockwise);
            const double allowance = budget / next.radius;
            if (!std::isfinite(trim)) return fail(Code::NONFINITE_GEOMETRY);
            if (std::fabs(trim) >= Pi - allowance || next.sweep - trim <= allowance || next.sweep - trim >= Tau - allowance)
                return fail(Code::NEXT_ARC_CONSUMED);
        }
        else
        {
            const double trim = (endpoint[0] - input.next.start[0]) * next.ux + (endpoint[1] - input.next.start[1]) * next.uy;
            if (!std::isfinite(trim)) return fail(Code::NONFINITE_GEOMETRY);
            if (trim > 0.0 && next.length - trim <= budget) return fail(Code::NEXT_LINE_CONSUMED);
        }
    }
    double sweep = 0.0;
    if (current.arc)
    {
        const double startTrim = Angle(input.current.start, input.actualStart, input.current.centre, input.current.clockwise);
        const double endExtension = Angle(input.current.end, endpoint, input.current.centre, input.current.clockwise);
        const double allowance = budget / current.radius;
        sweep = current.sweep + endExtension - startTrim;
        if (!std::isfinite(startTrim) || !std::isfinite(endExtension) || !std::isfinite(sweep))
            return fail(Code::NONFINITE_GEOMETRY);
        if (std::fabs(startTrim) >= Pi - allowance || std::fabs(endExtension) >= Pi - allowance ||
            sweep <= allowance || sweep >= Tau - allowance)
            return fail(Code::TRIMMED_ARC_REVERSED);
    }
    else
    {
        const double forward = (endpoint[0] - input.actualStart[0]) * current.ux + (endpoint[1] - input.actualStart[1]) * current.uy;
        if (!std::isfinite(forward)) return fail(Code::NONFINITE_GEOMETRY);
        if (forward <= budget) return fail(Code::TRIMMED_LINE_REVERSED);
    }
    output.endpoint[0] = endpoint[0]; output.endpoint[1] = endpoint[1];
    if (current.arc)
    {
        output.centre[0] = input.current.centre[0]; output.centre[1] = input.current.centre[1];
        output.radius = current.radius; output.sweepRadians = sweep;
    }
    output.reason = Code::BUILT; output.valid = true; return output.reason;
}

static_assert(std::is_standard_layout<NCPathCoreCutterContourInput>::value &&
    std::is_trivially_copyable<NCPathCoreCutterContourInput>::value &&
    std::is_standard_layout<NCPathCoreCutterContourOutput>::value &&
    std::is_trivially_copyable<NCPathCoreCutterContourOutput>::value,
    "Cutter contour inputs and output must remain bounded copyable values.");
