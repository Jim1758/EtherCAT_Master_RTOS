#pragma once

#include "NCPathCoreCommandedChordEval.h"
#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>

// NC-0.2L.2AX / Current Commanded Chord Axis-Coordinate Location Contract.
// Invert ONE changing axis of the mathematical commanded endpoint chord:
// u=(query-a)/(b-a), 0<=u<=1, not actual position feedback or G00 progress.
// AW supplies the complementary u -> axis-coordinate evaluation. No metric
// mixes mm and degrees; query uses exactly this slot's native MCS unit.
// A single-axis query does NOT prove any other measured axis lies on the chord.
// No projection, tolerance snapping, rotary modulo/shortest path, distance,
// queue, planner, history or Motion/B2 consumer is introduced.
//
// All eight D slots and the original AU identity/run fences are revalidated.
// First/boundary D count=1 is sufficient; E/F or an invented predecessor is not.
// A constant queried axis gives no unique u even when other axes move. A point
// chord also has no unique u. Both return explicit rejection with empty output;
// do not select u=0 merely to make the read succeed. Out-of-range is distinct.
//
// Successful START/END require numeric equality to the ORIGINAL endpoint, never
// merely a rounded u of 0/1. An interior query whose parameter rounds to 0/1 is
// rejected as PARAMETER_RESOLUTION_LOSS. Interior results are approximate, not
// the exact inverse of AW's rounded function. reconstructedCoordinateMCS is AW's
// actual re-evaluation at the returned u; it need not equal queryCoordinateMCS.
// No correctly-rounded, relative-error, bitwise roundtrip or position tolerance
// guarantee is made. Endpoints copy their representations; +0/-0 compare equal.
//
// SAME NC thread, trusted live owner, no reentry/source mutation for the entire
// call. Caller output must be disjoint from owner/source/NCManager. Values are
// copies, not tickets or proof. The local publication/chain IDs do not encode
// owner identity, full-cycle ABA, authentication or a RESET/STOP lifecycle epoch.
// Calling AW afterwards (including another axis) requires the SAME unchanged
// owner and matching IDs; do not combine results from changing sources.
// Uses AW's precise FP contract: round-to-nearest, gradual underflow, masked FP
// exceptions. No target FP mode is changed/detected. AW rejects known fast modes.

#if defined(_MSC_VER)
#define NC_PATH_CORE_AX_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_CORE_AX_NOINLINE __attribute__((noinline))
#else
#define NC_PATH_CORE_AX_NOINLINE
#endif

enum class NCPathCoreCommandedChordLocateCode : std::uint8_t
{
    NOT_LOCATED = 0U,
    INVALID_AXIS = 1U,
    INVALID_COORDINATE = 2U,
    NO_CURRENT_GEOMETRY = 3U,
    SOURCE_REJECTED = 4U,
    INVALID_GEOMETRY = 5U,
    OUTSIDE_AXIS_RANGE = 6U,
    NON_UNIQUE_CONSTANT_AXIS = 7U,
    NON_UNIQUE_POINT_CHORD = 8U,
    PARAMETER_RESOLUTION_LOSS = 9U,
    NONFINITE_ARITHMETIC = 10U,
    LOCATED_START = 11U,
    LOCATED_INTERIOR = 12U,
    LOCATED_END = 13U
};

enum class NCPathCoreCommandedChordLocationKind : std::uint8_t
{
    NONE = 0U,
    START = 1U,
    INTERIOR = 2U,
    END = 3U
};

// Caller-owned scalar value; no owner/slot pointer, self-proof or retained state.
struct NCPathCoreCommandedChordLocationV1
{
    std::uint64_t geometryPublicationSequence = 0ULL;
    std::uint64_t acceptedInputChainGeneration = 0ULL;
    double queryCoordinateMCS = 0.0;
    double unitParameter = 0.0;
    double reconstructedCoordinateMCS = 0.0;
    std::uint32_t axisIndex = 0U;
    std::uint16_t schemaVersion = 0U;
    NCPathCoreAcceptedInputPairRelation sourcePairRelation =
        NCPathCoreAcceptedInputPairRelation::NONE;
    NCPathCoreCommandedChordLocationKind location =
        NCPathCoreCommandedChordLocationKind::NONE;

    void Clear() noexcept
    {
        geometryPublicationSequence = 0ULL;
        acceptedInputChainGeneration = 0ULL;
        queryCoordinateMCS = 0.0;
        unitParameter = 0.0;
        reconstructedCoordinateMCS = 0.0;
        axisIndex = 0U;
        schemaVersion = 0U;
        sourcePairRelation = NCPathCoreAcceptedInputPairRelation::NONE;
        location = NCPathCoreCommandedChordLocationKind::NONE;
    }
};

namespace NCPathCoreCommandedChordLocateDetail
{
    using Geometry = NCPathCoreOrdinaryG00CommittedEndpointPairV1;
    using Code = NCPathCoreCommandedChordLocateCode;
    using Location = NCPathCoreCommandedChordLocationKind;

    // Precondition: finite unequal a,b; x strictly between them. Use half-scale
    // ONLY for large opposite-sign endpoints, where an unscaled subtraction
    // could overflow. Avoid unconditional halving, which loses subnormal gaps.
    // Compute from the nearer endpoint, limiting cancellation near either end.
    NC_PATH_CORE_AX_NOINLINE inline double InteriorParameter(
        const double a, const double b, const double x) noexcept
    {
        const double halfMax = (std::numeric_limits<double>::max)() * 0.5;
        const bool opposite = (a < 0.0 && b > 0.0) || (a > 0.0 && b < 0.0);
        const bool scale = opposite &&
            (std::fabs(a) > halfMax || std::fabs(b) > halfMax);
        const double sa = scale ? a * 0.5 : a;
        const double sb = scale ? b * 0.5 : b;
        const double sx = scale ? x * 0.5 : x;
        const double span = std::fabs(sb - sa);
        const double fromStart = std::fabs(sx - sa);
        const double fromEnd = std::fabs(sb - sx);
        return fromStart <= fromEnd ? fromStart / span : 1.0 - fromEnd / span;
    }

    // Borrowed helper alone does not establish newest; public owner entry uses
    // offset zero. No search, repair, fallback, source copy or source mutation.
    NC_PATH_CORE_AX_NOINLINE inline Code LocateBorrowed(
        const Geometry* const g, const std::uint32_t axisIndex,
        const double query, NCPathCoreCommandedChordLocationV1& output) noexcept
    {
        output.Clear();
        if (axisIndex >= NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY)
            return Code::INVALID_AXIS;
        if (!std::isfinite(query)) return Code::INVALID_COORDINATE;
        if (g == nullptr) return Code::NO_CURRENT_GEOMETRY;
        if (g->publicationSequence == 0ULL ||
            g->schemaVersion != NC_PATH_CORE_COMMITTED_GEOMETRY_SCHEMA_V1 ||
            g->kind != NCPathCoreCommittedGeometryKind::ORDINARY_G00_EXACT_STOP_ENDPOINT_PAIR ||
            g->frame != NCPathCoreCommittedGeometryFrame::MCS_AXIS_NATIVE ||
            g->commandPathPolicy != NCPathCoreCommittedGeometryCommandPathPolicy::EXACT_STOP ||
            g->extent != NCPathCoreCommittedGeometryExtent::COMMANDED_ENDPOINT_PAIR_ONLY)
            return Code::INVALID_GEOMETRY;
        if (g->validity == NCPathCoreCommittedGeometryValidity::INVALID_FENCE ||
            g->validity == NCPathCoreCommittedGeometryValidity::INVALID_COORDINATES)
            return Code::SOURCE_REJECTED;
        if (!NCPathCoreCommandedSegmentCheckDetail::IsGeometryValid(*g))
            return Code::INVALID_GEOMETRY;

        const double a = g->startMCS[axisIndex];
        const double b = g->endMCS[axisIndex];
        const double lo = a < b ? a : b;
        const double hi = a < b ? b : a;
        if (query < lo || query > hi) return Code::OUTSIDE_AXIS_RANGE;
        if (a == b)
            return NCPathCoreCommandedChordDetail::IsPointChord(*g)
            ? Code::NON_UNIQUE_POINT_CHORD : Code::NON_UNIQUE_CONSTANT_AXIS;

        const bool atStart = query == a;
        const bool atEnd = query == b;
        const double u = atStart ? 0.0 :
            (atEnd ? 1.0 : InteriorParameter(a, b, query));
        if (!std::isfinite(u)) return Code::NONFINITE_ARITHMETIC;
        if (!atStart && !atEnd && (u <= 0.0 || u >= 1.0))
            return Code::PARAMETER_RESOLUTION_LOSS;
        const double reconstructed =
            NCPathCoreCommandedChordDetail::InterpolateFinite(a, b, u);
        if (!std::isfinite(reconstructed)) return Code::NONFINITE_ARITHMETIC;

        output.geometryPublicationSequence = g->publicationSequence;
        output.acceptedInputChainGeneration = g->acceptedInputChainGeneration;
        output.queryCoordinateMCS = query;
        output.unitParameter = u;
        output.reconstructedCoordinateMCS = reconstructed;
        output.axisIndex = axisIndex;
        output.schemaVersion = 1U;
        output.sourcePairRelation = g->acceptedInputPairRelation;
        output.location = atStart ? Location::START :
            (atEnd ? Location::END : Location::INTERIOR);
        return atStart ? Code::LOCATED_START :
            (atEnd ? Code::LOCATED_END : Code::LOCATED_INTERIOR);
    }
}

NC_PATH_CORE_AX_NOINLINE inline NCPathCoreCommandedChordLocateCode
LocateCurrentCommandedChordAxisSameThread(
    const NCPathCoreCommittedGeometryShadow& geometryOwner,
    const std::uint32_t axisIndex, const double queryCoordinateMCS,
    NCPathCoreCommandedChordLocationV1& output) noexcept
{
    return NCPathCoreCommandedChordLocateDetail::LocateBorrowed(
        geometryOwner.GetNewestObservationSameThread(0U),
        axisIndex, queryCoordinateMCS, output);
}

#undef NC_PATH_CORE_AX_NOINLINE
static_assert(sizeof(NCPathCoreCommandedChordLocateCode) == 1U &&
    sizeof(NCPathCoreCommandedChordLocationKind) == 1U, "AX codes are one byte.");
static_assert(sizeof(NCPathCoreCommandedChordLocationV1) == 48U &&
    alignof(NCPathCoreCommandedChordLocationV1) == 8U, "AX scalar output ABI changed.");
static_assert(std::is_standard_layout<NCPathCoreCommandedChordLocationV1>::value&&
    std::is_trivially_copyable<NCPathCoreCommandedChordLocationV1>::value,
    "AX output must remain a plain value, not an owner.");
