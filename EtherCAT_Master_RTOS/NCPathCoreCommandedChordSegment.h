#pragma once

#include "NCPathCoreCommandedChordLocate.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

// NC-0.2L.2AY / Commanded Chord Segment V1 Value and Local-Identity Contract.
// One explicit, fixed-size MATHEMATICAL commanded endpoint chord. V1 capture
// admits the same ordinary no-P G00 exact-stop D sources as AW/AX, including
// FIRST_INPUT and CHAIN_BOUNDARY. It does not require E/F or invent a previous
// segment. CONTIGUOUS_PAIR/run describe accepted-input provenance ONLY, not
// a proven geometric seam, a stored head or retained-history coverage.
//
// Capture writes to disjoint caller-provided HEAP-OWNED workspace, on the SAME
// NC thread with a trusted live D owner and no reentry/source mutation for the
// entire call. It selects ONLY newest D. Failure clears every semantic field.
// No allocation, new NCManager member, observer, control hook or caller is added.
// Do not put SegmentV1 temporaries on the NC stack or return them by value.
//
// After capture, value queries use ONLY the explicit segment, not its old D
// slot or a current owner. A copied value remains mathematical data when D
// rotates, becomes invalid or ceases to exist; it is NOT then current proof.
// The caller must keep the value unchanged during a call and across related
// queries. All outputs must be disjoint from the input/owner. No synchronization
// or cross-thread publication is supplied. All functions require live objects.
//
// A local identity is (D publication, accepted-input chain), meaningful ONLY
// under an externally known same-owner/same-lifetime scope and before full-ID
// reuse. No owner pointer, store epoch, ordering, full-cycle ABA protection,
// authentication or RESET/STOP permission is encoded. Comparing keys is NOT
// comparing endpoint payloads or establishing currentness. A structurally valid
// caller-edited value can retain the same key; validation is NOT source proof.
// motionExecutionEpoch is deliberately NOT relabelled as a store lifetime.
//
// Eight slots retain their native MCS units (mm OR degrees, not a mixed norm).
// axisMask means submitted axes, not actual motion; unmasked slots must be
// numerically constant. POINT/LINE is checked from ALL eight endpoint pairs.
// No distance, rotary unwrap, timing, G00 trajectory, Motion acceptance/completion,
// cursor, multi-segment storage, Path/Motion Queue or B2 is represented.
//
// Pure queries reuse AW/AX's ORIGINAL arithmetic helpers and output/status
// types. Their finite/rounding/non-unique/resolution-loss rules are unchanged.
// For value queries, INVALID_GEOMETRY means an invalid explicit segment;
// NO_CURRENT_GEOMETRY/SOURCE_REJECTED are not produced by those pure queries.
// Precise FP, round-to-nearest, gradual underflow, masked FP exceptions required
// exactly as in AW/AX. This header does not modify/detect the target FP state.

#if defined(_MSC_VER)
#define NC_PATH_CORE_AY_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_CORE_AY_NOINLINE __attribute__((noinline))
#else
#define NC_PATH_CORE_AY_NOINLINE
#endif

constexpr std::uint16_t NC_PATH_CORE_COMMANDED_CHORD_SEGMENT_SCHEMA_V1 = 1U;
constexpr std::size_t NC_PATH_CORE_COMMANDED_CHORD_SEGMENT_AXIS_CAPACITY = 8U;

struct NCPathCoreCommandedChordLocalIdentityV1
{
    std::uint64_t geometryPublicationSequence = 0ULL;
    std::uint64_t acceptedInputChainGeneration = 0ULL;

    void Clear() noexcept
    {
        geometryPublicationSequence = 0ULL;
        acceptedInputChainGeneration = 0ULL;
    }
};

enum class NCPathCoreCommandedChordSegmentExtent : std::uint8_t
{
    NONE = 0U,
    COMMANDED_ENDPOINT_CHORD_ONLY = 1U
};

enum class NCPathCoreCommandedChordSegmentCaptureCode : std::uint8_t
{
    NOT_CAPTURED = 0U,
    NO_CURRENT_GEOMETRY = 1U,
    SOURCE_REJECTED = 2U,
    INVALID_GEOMETRY = 3U,
    CAPTURED_LINE_CHORD = 4U,
    CAPTURED_POINT_CHORD = 5U
};

// 160-byte VALUE, not another diagnostic record/ring/owner. No source pointer,
// predecessor, synthetic ordinal, Motion ticket, or event list is embedded.
// All-zero/default is invalid. Padding is not a serialized/wire representation.
struct NCPathCoreCommandedChordSegmentV1
{
    NCPathCoreCommandedChordLocalIdentityV1 localIdentity{};
    std::array<double, NC_PATH_CORE_COMMANDED_CHORD_SEGMENT_AXIS_CAPACITY> startMCS{};
    std::array<double, NC_PATH_CORE_COMMANDED_CHORD_SEGMENT_AXIS_CAPACITY> endMCS{};
    std::uint32_t axisMask = 0U;
    std::uint32_t acceptedInputRunLength = 0U;
    std::uint16_t schemaVersion = 0U;
    NCPathCoreAcceptedInputPairRelation sourcePairRelation =
        NCPathCoreAcceptedInputPairRelation::NONE;
    NCPathCoreCommandedChordKind kind = NCPathCoreCommandedChordKind::NONE;
    NCPathCoreCommittedGeometryFrame frame = NCPathCoreCommittedGeometryFrame::NONE;
    NCPathCoreCommandedChordSegmentExtent extent =
        NCPathCoreCommandedChordSegmentExtent::NONE;
    std::uint8_t reserved[2U]{};

    void Clear() noexcept
    {
        localIdentity.Clear();
        for (std::size_t axis = 0U;
            axis < NC_PATH_CORE_COMMANDED_CHORD_SEGMENT_AXIS_CAPACITY; ++axis)
        {
            startMCS[axis] = 0.0;
            endMCS[axis] = 0.0;
        }
        axisMask = 0U;
        acceptedInputRunLength = 0U;
        schemaVersion = 0U;
        sourcePairRelation = NCPathCoreAcceptedInputPairRelation::NONE;
        kind = NCPathCoreCommandedChordKind::NONE;
        frame = NCPathCoreCommittedGeometryFrame::NONE;
        extent = NCPathCoreCommandedChordSegmentExtent::NONE;
        reserved[0U] = 0U;
        reserved[1U] = 0U;
    }
};

// PRECONDITION: the caller already knows same source owner, same lifetime and
// no complete identity reuse. This function cannot establish those facts.
// Zero keys never match. There is intentionally NO sequence-order comparison.
inline bool HasSameCommandedChordLocalIdentitySameOwnerLifetime(
    const NCPathCoreCommandedChordLocalIdentityV1& left,
    const NCPathCoreCommandedChordLocalIdentityV1& right) noexcept
{
    return left.geometryPublicationSequence != 0ULL &&
        left.acceptedInputChainGeneration != 0ULL &&
        left.geometryPublicationSequence == right.geometryPublicationSequence &&
        left.acceptedInputChainGeneration == right.acceptedInputChainGeneration;
}

NC_PATH_CORE_AY_NOINLINE inline bool IsValidCommandedChordSegmentValue(
    const NCPathCoreCommandedChordSegmentV1& segment) noexcept
{
    using Pair = NCPathCoreAcceptedInputPairRelation;
    using Kind = NCPathCoreCommandedChordKind;
    if (segment.schemaVersion != NC_PATH_CORE_COMMANDED_CHORD_SEGMENT_SCHEMA_V1 ||
        segment.localIdentity.geometryPublicationSequence == 0ULL ||
        segment.localIdentity.acceptedInputChainGeneration == 0ULL ||
        segment.frame != NCPathCoreCommittedGeometryFrame::MCS_AXIS_NATIVE ||
        segment.extent != NCPathCoreCommandedChordSegmentExtent::COMMANDED_ENDPOINT_CHORD_ONLY ||
        segment.axisMask == 0U ||
        (segment.axisMask & ~NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_MASK) != 0U ||
        segment.reserved[0U] != 0U || segment.reserved[1U] != 0U)
        return false;
    const bool runValid =
        ((segment.sourcePairRelation == Pair::FIRST_INPUT ||
            segment.sourcePairRelation == Pair::CHAIN_BOUNDARY) &&
            segment.acceptedInputRunLength == 1U) ||
        (segment.sourcePairRelation == Pair::CONTIGUOUS_PAIR &&
            segment.acceptedInputRunLength >= 2U);
    if (!runValid) return false;
    bool isPoint = true;
    for (std::size_t axis = 0U;
        axis < NC_PATH_CORE_COMMANDED_CHORD_SEGMENT_AXIS_CAPACITY; ++axis)
    {
        const double a = segment.startMCS[axis];
        const double b = segment.endMCS[axis];
        if (!std::isfinite(a) || !std::isfinite(b)) return false;
        if (a != b)
        {
            const std::uint32_t bit = static_cast<std::uint32_t>(1U << axis);
            if ((segment.axisMask & bit) == 0U) return false;
            isPoint = false;
        }
    }
    return segment.kind == (isPoint ? Kind::POINT_CHORD : Kind::LINE_CHORD);
}

namespace NCPathCoreCommandedChordSegmentDetail
{
    using Geometry = NCPathCoreOrdinaryG00CommittedEndpointPairV1;
    using CaptureCode = NCPathCoreCommandedChordSegmentCaptureCode;

    // Borrowed helper is testable separately but does NOT establish newest.
    // Public capture below obtains offset zero from its trusted live owner.
    NC_PATH_CORE_AY_NOINLINE inline CaptureCode CaptureBorrowed(
        const Geometry* const geometry,
        NCPathCoreCommandedChordSegmentV1& output) noexcept
    {
        output.Clear();
        if (geometry == nullptr) return CaptureCode::NO_CURRENT_GEOMETRY;
        if (geometry->publicationSequence == 0ULL ||
            geometry->schemaVersion != NC_PATH_CORE_COMMITTED_GEOMETRY_SCHEMA_V1 ||
            geometry->kind != NCPathCoreCommittedGeometryKind::ORDINARY_G00_EXACT_STOP_ENDPOINT_PAIR ||
            geometry->frame != NCPathCoreCommittedGeometryFrame::MCS_AXIS_NATIVE ||
            geometry->commandPathPolicy != NCPathCoreCommittedGeometryCommandPathPolicy::EXACT_STOP ||
            geometry->extent != NCPathCoreCommittedGeometryExtent::COMMANDED_ENDPOINT_PAIR_ONLY)
            return CaptureCode::INVALID_GEOMETRY;
        if (geometry->validity == NCPathCoreCommittedGeometryValidity::INVALID_FENCE ||
            geometry->validity == NCPathCoreCommittedGeometryValidity::INVALID_COORDINATES)
            return CaptureCode::SOURCE_REJECTED;
        if (!NCPathCoreCommandedSegmentCheckDetail::IsGeometryValid(*geometry))
            return CaptureCode::INVALID_GEOMETRY;

        // Copy directly into caller workspace. No D or segment-sized temporary.
        output.localIdentity.geometryPublicationSequence = geometry->publicationSequence;
        output.localIdentity.acceptedInputChainGeneration = geometry->acceptedInputChainGeneration;
        for (std::size_t axis = 0U;
            axis < NC_PATH_CORE_COMMANDED_CHORD_SEGMENT_AXIS_CAPACITY; ++axis)
        {
            output.startMCS[axis] = geometry->startMCS[axis];
            output.endMCS[axis] = geometry->endMCS[axis];
        }
        output.axisMask = geometry->axisMask;
        output.acceptedInputRunLength = geometry->acceptedInputRunLength;
        output.sourcePairRelation = geometry->acceptedInputPairRelation;
        const bool isPoint = NCPathCoreCommandedChordDetail::IsPointChord(*geometry);
        output.kind = isPoint ? NCPathCoreCommandedChordKind::POINT_CHORD
            : NCPathCoreCommandedChordKind::LINE_CHORD;
        output.frame = NCPathCoreCommittedGeometryFrame::MCS_AXIS_NATIVE;
        output.extent = NCPathCoreCommandedChordSegmentExtent::COMMANDED_ENDPOINT_CHORD_ONLY;
        output.schemaVersion = NC_PATH_CORE_COMMANDED_CHORD_SEGMENT_SCHEMA_V1;
        return isPoint ? CaptureCode::CAPTURED_POINT_CHORD : CaptureCode::CAPTURED_LINE_CHORD;
    }
}

NC_PATH_CORE_AY_NOINLINE inline NCPathCoreCommandedChordSegmentCaptureCode
CaptureCurrentCommandedChordSegmentSameThread(
    const NCPathCoreCommittedGeometryShadow& geometryOwner,
    NCPathCoreCommandedChordSegmentV1& output) noexcept
{
    return NCPathCoreCommandedChordSegmentDetail::CaptureBorrowed(
        geometryOwner.GetNewestObservationSameThread(0U), output);
}

// PURE value query: independent of current D and its lifetime. Caller must
// keep this explicit segment unchanged across a locate/evaluate sequence.
NC_PATH_CORE_AY_NOINLINE inline NCPathCoreCommandedChordCode
EvaluateCommandedChordSegmentAxis(
    const NCPathCoreCommandedChordSegmentV1& segment,
    const std::uint32_t axisIndex, const double unitParameter,
    NCPathCoreCommandedChordAxisValueV1& output) noexcept
{
    using Code = NCPathCoreCommandedChordCode;
    output.Clear();
    if (axisIndex >= NC_PATH_CORE_COMMANDED_CHORD_SEGMENT_AXIS_CAPACITY)
        return Code::INVALID_AXIS;
    if (!std::isfinite(unitParameter) || unitParameter < 0.0 || unitParameter > 1.0)
        return Code::INVALID_PARAMETER;
    if (!IsValidCommandedChordSegmentValue(segment)) return Code::INVALID_GEOMETRY;
    const double coordinate = NCPathCoreCommandedChordDetail::InterpolateFinite(
        segment.startMCS[axisIndex], segment.endMCS[axisIndex], unitParameter);
    if (!std::isfinite(coordinate)) return Code::NONFINITE_RESULT;
    output.geometryPublicationSequence = segment.localIdentity.geometryPublicationSequence;
    output.acceptedInputChainGeneration = segment.localIdentity.acceptedInputChainGeneration;
    output.unitParameter = unitParameter;
    output.coordinateMCS = coordinate;
    output.axisIndex = axisIndex;
    output.schemaVersion = 1U; // Original AW output schema, not a new wire ABI.
    output.sourcePairRelation = segment.sourcePairRelation;
    output.kind = segment.kind;
    return segment.kind == NCPathCoreCommandedChordKind::POINT_CHORD
        ? Code::EVALUATED_POINT_CHORD : Code::EVALUATED_LINE_CHORD;
}

NC_PATH_CORE_AY_NOINLINE inline NCPathCoreCommandedChordLocateCode
LocateCommandedChordSegmentAxis(
    const NCPathCoreCommandedChordSegmentV1& segment,
    const std::uint32_t axisIndex, const double queryCoordinateMCS,
    NCPathCoreCommandedChordLocationV1& output) noexcept
{
    using Code = NCPathCoreCommandedChordLocateCode;
    using Location = NCPathCoreCommandedChordLocationKind;
    output.Clear();
    if (axisIndex >= NC_PATH_CORE_COMMANDED_CHORD_SEGMENT_AXIS_CAPACITY)
        return Code::INVALID_AXIS;
    if (!std::isfinite(queryCoordinateMCS)) return Code::INVALID_COORDINATE;
    if (!IsValidCommandedChordSegmentValue(segment)) return Code::INVALID_GEOMETRY;

    const double a = segment.startMCS[axisIndex];
    const double b = segment.endMCS[axisIndex];
    const double lo = a < b ? a : b;
    const double hi = a < b ? b : a;
    if (queryCoordinateMCS < lo || queryCoordinateMCS > hi) return Code::OUTSIDE_AXIS_RANGE;
    if (a == b)
        return segment.kind == NCPathCoreCommandedChordKind::POINT_CHORD
        ? Code::NON_UNIQUE_POINT_CHORD : Code::NON_UNIQUE_CONSTANT_AXIS;

    const bool atStart = queryCoordinateMCS == a;
    const bool atEnd = queryCoordinateMCS == b;
    const double u = atStart ? 0.0 : (atEnd ? 1.0 :
        NCPathCoreCommandedChordLocateDetail::InteriorParameter(a, b, queryCoordinateMCS));
    if (!std::isfinite(u)) return Code::NONFINITE_ARITHMETIC;
    if (!atStart && !atEnd && (u <= 0.0 || u >= 1.0))
        return Code::PARAMETER_RESOLUTION_LOSS;
    const double reconstructed = NCPathCoreCommandedChordDetail::InterpolateFinite(a, b, u);
    if (!std::isfinite(reconstructed)) return Code::NONFINITE_ARITHMETIC;

    output.geometryPublicationSequence = segment.localIdentity.geometryPublicationSequence;
    output.acceptedInputChainGeneration = segment.localIdentity.acceptedInputChainGeneration;
    output.queryCoordinateMCS = queryCoordinateMCS;
    output.unitParameter = u;
    output.reconstructedCoordinateMCS = reconstructed;
    output.axisIndex = axisIndex;
    output.schemaVersion = 1U; // Original AX output schema.
    output.sourcePairRelation = segment.sourcePairRelation;
    output.location = atStart ? Location::START : (atEnd ? Location::END : Location::INTERIOR);
    return atStart ? Code::LOCATED_START : (atEnd ? Code::LOCATED_END : Code::LOCATED_INTERIOR);
}

#undef NC_PATH_CORE_AY_NOINLINE
static_assert(NC_PATH_CORE_COMMANDED_CHORD_SEGMENT_AXIS_CAPACITY ==
    NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY &&
    NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_MASK == 0xFFU,
    "AY V1 has exactly eight axis-native slots.");
static_assert(sizeof(NCPathCoreCommandedChordLocalIdentityV1) == 16U &&
    alignof(NCPathCoreCommandedChordLocalIdentityV1) == 8U,
    "AY local identity layout changed; it is NOT an owner/store handle.");
static_assert(sizeof(NCPathCoreCommandedChordSegmentV1) == 160U &&
    alignof(NCPathCoreCommandedChordSegmentV1) == 8U,
    "AY segment is exactly one 160-byte caller-owned value.");
static_assert(std::is_standard_layout<NCPathCoreCommandedChordSegmentV1>::value&&
    std::is_trivially_copyable<NCPathCoreCommandedChordSegmentV1>::value&&
    std::is_trivially_destructible<NCPathCoreCommandedChordSegmentV1>::value&&
    std::is_nothrow_default_constructible<NCPathCoreCommandedChordSegmentV1>::value,
    "AY segment must remain a bounded non-owning plain value.");
static_assert(offsetof(NCPathCoreCommandedChordSegmentV1, localIdentity) == 0U &&
    offsetof(NCPathCoreCommandedChordSegmentV1, startMCS) == 16U &&
    offsetof(NCPathCoreCommandedChordSegmentV1, endMCS) == 80U &&
    offsetof(NCPathCoreCommandedChordSegmentV1, axisMask) == 144U &&
    offsetof(NCPathCoreCommandedChordSegmentV1, acceptedInputRunLength) == 148U &&
    offsetof(NCPathCoreCommandedChordSegmentV1, schemaVersion) == 152U &&
    offsetof(NCPathCoreCommandedChordSegmentV1, sourcePairRelation) == 154U &&
    offsetof(NCPathCoreCommandedChordSegmentV1, kind) == 155U &&
    offsetof(NCPathCoreCommandedChordSegmentV1, frame) == 156U &&
    offsetof(NCPathCoreCommandedChordSegmentV1, extent) == 157U &&
    offsetof(NCPathCoreCommandedChordSegmentV1, reserved) == 158U,
    "AY V1 member offsets changed.");
static_assert(sizeof(NCPathCoreCommandedChordSegmentCaptureCode) == 1U &&
    sizeof(NCPathCoreCommandedChordSegmentExtent) == 1U,
    "AY status/extent are one byte.");
