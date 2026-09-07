#pragma once

#include "NCPathCoreCommandedSegmentCheck.h"
#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>

// NC-0.2L.2AW / Current Commanded Endpoint Chord Evaluation Contract.
// Mathematical chord C(u)=(1-u)*D.startMCS+u*D.endMCS, 0<=u<=1.
// This is a NEW explicit geometric interpretation of D's commanded endpoints,
// NOT a claim that ordinary G00 used linear interpolation or travelled C(u).
// A valid first D or CHAIN_BOUNDARY D is sufficient: no invented previous D,
// no E/F admission, no continuity/coverage prerequisite or missing-head repair.
// No length/norm is calculated: D slots mix axis-native mm/degrees; rotary
// unwrapping, physical metric, feed/timing and executed progress are undefined.
// Evaluate one requested slot; all eight source slots are validated first.
// Unmasked slots remain carried-forward coordinates. An all-slot numerically
// equal endpoint pair is an admissible POINT_CHORD, never divide by its length.
//
// SAME NC thread, live trusted owner, no reentry or source modification for the
// entire call. output must not overlap owner/source storage. Values are copies
// at call time, not current proof, tickets, history or control permission.
// Publication/chain identify only that source owner/lifetime; no owner identity,
// ABA, authentication, RESET/STOP epoch or cross-thread synchronization exists.
// When evaluating several slots, the caller must keep the owner unchanged and
// verify matching publication/chain; never combine results from changing owners.
// A result is usable ONLY for EVALUATED_LINE_CHORD or EVALUATED_POINT_CHORD.
// Failure clears all output fields; no clamping of an invalid input parameter,
// fallback to an older source, hidden recapture, queue or Motion consumer.
//
// Arithmetic requires precise floating semantics and the normal floating-point
// environment (round-to-nearest, gradual underflow, masked FP exceptions).
// This file never changes that environment. Fast/finite-only compilation is
// rejected when detectable. Endpoint u=0/1 copies preserve signed-zero bits;
// interior equal endpoints use the start representation. Interior results are
// rounded and bounded, not promised correctly rounded or bitwise reversible.
#if defined(__FAST_MATH__) || defined(_M_FP_FAST)
#error AW_requires_precise_floating_point_semantics_not_fast_math
#endif
#if defined(__FINITE_MATH_ONLY__) && (__FINITE_MATH_ONLY__ > 0)
#error AW_requires_NaN_and_infinity_checks
#endif

#if defined(_MSC_VER)
#define NC_PATH_CORE_AW_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_CORE_AW_NOINLINE __attribute__((noinline))
#else
#define NC_PATH_CORE_AW_NOINLINE
#endif

enum class NCPathCoreCommandedChordCode : std::uint8_t
{
    NOT_EVALUATED = 0U,
    INVALID_AXIS = 1U,
    INVALID_PARAMETER = 2U,
    NO_CURRENT_GEOMETRY = 3U,
    SOURCE_REJECTED = 4U,
    INVALID_GEOMETRY = 5U,
    NONFINITE_RESULT = 6U,
    EVALUATED_LINE_CHORD = 7U,
    EVALUATED_POINT_CHORD = 8U
};

enum class NCPathCoreCommandedChordKind : std::uint8_t
{
    NONE = 0U,
    LINE_CHORD = 1U,
    POINT_CHORD = 2U
};

// Caller-owned scalar output, not another NCManager record/member/history.
// No endpoint array, owner pointer or self-proof is embedded in this value.
struct NCPathCoreCommandedChordAxisValueV1
{
    std::uint64_t geometryPublicationSequence = 0ULL;
    std::uint64_t acceptedInputChainGeneration = 0ULL;
    double unitParameter = 0.0;
    double coordinateMCS = 0.0;
    std::uint32_t axisIndex = 0U;
    std::uint16_t schemaVersion = 0U;
    NCPathCoreAcceptedInputPairRelation sourcePairRelation =
        NCPathCoreAcceptedInputPairRelation::NONE;
    NCPathCoreCommandedChordKind kind = NCPathCoreCommandedChordKind::NONE;

    void Clear() noexcept
    {
        geometryPublicationSequence = 0ULL;
        acceptedInputChainGeneration = 0ULL;
        unitParameter = 0.0;
        coordinateMCS = 0.0;
        axisIndex = 0U;
        schemaVersion = 0U;
        sourcePairRelation = NCPathCoreAcceptedInputPairRelation::NONE;
        kind = NCPathCoreCommandedChordKind::NONE;
    }
};

namespace NCPathCoreCommandedChordDetail
{
    using Geometry = NCPathCoreOrdinaryG00CommittedEndpointPairV1;
    using Code = NCPathCoreCommandedChordCode;

    // Endpoint arithmetic only. Same-sign subtraction cannot overflow;
    // opposite-sign weighted terms avoid forming b-a across +/-DBL_MAX.
    // Clamp only computed roundoff, NEVER an invalid u or a nonfinite result.
    NC_PATH_CORE_AW_NOINLINE inline double InterpolateFinite(
        const double a, const double b, const double u) noexcept
    {
        if (u == 0.0) return a;
        if (u == 1.0) return b;
        if (a == b) return a;
        const bool straddlesZero =
            (a <= 0.0 && b >= 0.0) || (a >= 0.0 && b <= 0.0);
        const double x = straddlesZero
            ? (1.0 - u) * a + u * b
            : a + u * (b - a);
        if (!std::isfinite(x)) return x;
        const double lo = a < b ? a : b;
        const double hi = a < b ? b : a;
        return x < lo ? lo : (x > hi ? hi : x);
    }

    NC_PATH_CORE_AW_NOINLINE inline bool IsPointChord(const Geometry& g) noexcept
    {
        for (std::size_t axis = 0U;
            axis < NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY; ++axis)
            if (g.startMCS[axis] != g.endMCS[axis]) return false;
        return true;
    }

    // Borrowed-record helper does not establish "newest". The owner entry
    // below always selects offset zero; it never searches for an old success.
    NC_PATH_CORE_AW_NOINLINE inline Code EvaluateBorrowed(
        const Geometry* const g, const std::uint32_t axisIndex,
        const double u, NCPathCoreCommandedChordAxisValueV1& output) noexcept
    {
        output.Clear();
        if (axisIndex >= NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY)
            return Code::INVALID_AXIS;
        if (!std::isfinite(u) || u < 0.0 || u > 1.0)
            return Code::INVALID_PARAMETER;
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
        // Reuse AU's original D identity, native-coordinate and run checks.
        // FIRST_INPUT/CHAIN_BOUNDARY count=1 is valid here without an E/F pair.
        if (!NCPathCoreCommandedSegmentCheckDetail::IsGeometryValid(*g))
            return Code::INVALID_GEOMETRY;
        const double coordinate = InterpolateFinite(
            g->startMCS[axisIndex], g->endMCS[axisIndex], u);
        if (!std::isfinite(coordinate)) return Code::NONFINITE_RESULT;
        const bool isPoint = IsPointChord(*g);
        output.geometryPublicationSequence = g->publicationSequence;
        output.acceptedInputChainGeneration = g->acceptedInputChainGeneration;
        output.unitParameter = u;
        output.coordinateMCS = coordinate;
        output.axisIndex = axisIndex;
        output.schemaVersion = 1U;
        output.sourcePairRelation = g->acceptedInputPairRelation;
        output.kind = isPoint ? NCPathCoreCommandedChordKind::POINT_CHORD
            : NCPathCoreCommandedChordKind::LINE_CHORD;
        return isPoint ? Code::EVALUATED_POINT_CHORD : Code::EVALUATED_LINE_CHORD;
    }
}

NC_PATH_CORE_AW_NOINLINE inline NCPathCoreCommandedChordCode
EvaluateCurrentCommandedChordAxisSameThread(
    const NCPathCoreCommittedGeometryShadow& geometryOwner,
    const std::uint32_t axisIndex, const double unitParameter,
    NCPathCoreCommandedChordAxisValueV1& output) noexcept
{
    return NCPathCoreCommandedChordDetail::EvaluateBorrowed(
        geometryOwner.GetNewestObservationSameThread(0U),
        axisIndex, unitParameter, output);
}

#undef NC_PATH_CORE_AW_NOINLINE
static_assert(std::numeric_limits<double>::is_iec559 && sizeof(double) == 8U,
    "AW requires IEEE binary64 double.");
static_assert(sizeof(NCPathCoreCommandedChordCode) == 1U, "AW status is one byte.");
static_assert(sizeof(NCPathCoreCommandedChordKind) == 1U, "AW kind is one byte.");
static_assert(sizeof(NCPathCoreCommandedChordAxisValueV1) == 40U &&
    alignof(NCPathCoreCommandedChordAxisValueV1) == 8U,
    "AW caller-owned scalar value ABI changed.");
static_assert(std::is_standard_layout<NCPathCoreCommandedChordAxisValueV1>::value&&
    std::is_trivially_copyable<NCPathCoreCommandedChordAxisValueV1>::value,
    "AW scalar output must remain a simple value, not an owner.");
