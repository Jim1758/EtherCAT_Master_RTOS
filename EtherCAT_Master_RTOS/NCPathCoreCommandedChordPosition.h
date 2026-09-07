#pragma once

#include "NCPathCoreCommandedChordCursor.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

// NC-0.2L.2BB / Retained Commanded Chord Position V1 and Revalidated Geometry
// Query Contract. A position is an AZ/BA retained segment binding plus an
// explicit local u in [0,1]. It is NOT Motion/feedback/executed progress, a
// distance, a trajectory parameter shared across segments, a queue or B2.
//
// Three standalone operations: bind an explicit u, locate u on ONE selected
// segment's changing axis, and sample all eight native-axis coordinates.
// (ordinal n,u=1) and (n+1,u=0) remain DISTINCT positions even at equal seams.
// No seam normalization, search, rotary unwrap, mixed-unit norm or cross-segment
// parameter movement is supplied. Axis mask is submitted axes, not actual motion.
// A single-axis location does not prove a measured multi-axis position lies here.
//
// ALL calls first revalidate via BA ReadCurrent, before scalar/math checks.
// A closed/faulted/stale binding never returns a scalar or geometry success.
// All failures clear EVERY output, INCLUDING the segment workspace. On success
// the workspace contains the selected captured mathematical segment, not a
// permanent source proof. Source rejection takes precedence over bad scalars.
//
// Explicit u is legal for POINT chords; this is the caller's parameter choice.
// Locate on a constant axis/POINT must return NON_UNIQUE and clear the position;
// it never invents u. Locate preserves AY/AX's approximate reconstructed value,
// endpoint equality and PARAMETER_RESOLUTION_LOSS rules. No exact inverse claim.
// Sampling reuses the original AW InterpolateFinite after BA's validation;
// endpoints and equal-coordinate signed-zero behavior remain unchanged.
// Finite u=-0 is allowed and preserved; invalid u is never clamped.
// AW's precise FP environment/preconditions and fast-math guards are inherited.
//
// Preconditions inherited from AZ/BA/AY:
// * Same owning thread, live Store; no reentry or mutation for the WHOLE call.
// * Store, Segment workspace and 120-byte Sample outputs are preallocated on
//   the heap outside realtime Observe/PDO paths. Operations allocate nothing.
// * ALL inputs/outputs/workspace/store are disjoint. No in-place binding,
//   sample.position alias, or Locate output/detail/workspace overlap is allowed.
// * ownerTag namespace/non-reuse and trusted source owner/lifetime are external
//   obligations. Mutable valid bindings/parameters can be copied or replaced;
//   neither Position nor Sample authenticates their origin or grants control.
//
// No production owner/caller/lifecycle hook or NCManager member is added.
// One BA read per call (at most 3 AZ const calls). Bind: at most 2 fixed
// eight-axis validations; Locate: at most 3; Sample: 2 plus 8 interpolations.
// No history scan, Segment/Store/Sample stack temporary, allocation, observer,
// thread, timer, mutex, wait, sleep, log, Alarm/Gate or Motion interaction.

enum class NCPathCoreCommandedChordPositionCode : std::uint8_t
{
    NONE = 0U,
    POSITION_BOUND = 1U,
    LOCATED_START = 2U,
    LOCATED_INTERIOR = 3U,
    LOCATED_END = 4U,
    EVALUATED_LINE_CHORD = 5U,
    EVALUATED_POINT_CHORD = 6U,
    STORE_CLOSED = 7U,
    STORE_FAULTED = 8U,
    CURSOR_REJECTED = 9U,
    RETAINED_VALUE_INVALID = 10U,
    STORE_REJECTED = 11U,
    INVALID_PARAMETER = 12U,
    INVALID_AXIS = 13U,
    INVALID_COORDINATE = 14U,
    OUTSIDE_AXIS_RANGE = 15U,
    NON_UNIQUE_CONSTANT_AXIS = 16U,
    NON_UNIQUE_POINT_CHORD = 17U,
    PARAMETER_RESOLUTION_LOSS = 18U,
    NONFINITE_ARITHMETIC = 19U,
    INVALID_GEOMETRY = 20U,
    GEOMETRY_REJECTED = 21U
};

// Process-local value, not a wire/SHM ABI. Default/cleared cursor is invalid.
struct NCPathCoreCommandedChordPositionV1
{
    NCPathCoreCommandedChordCursorV1 cursor{};
    double unitParameter = 0.0;

    void Clear() noexcept
    {
        cursor.Clear();
        unitParameter = 0.0;
    }
};

// Caller-owned heap output, never returned by value or added to NCManager.
// All eight native MCS slots are sampled, including unmasked constant axes.
struct NCPathCoreCommandedChordPositionSampleV1
{
    NCPathCoreCommandedChordPositionV1 position{};
    std::array<double, NC_PATH_CORE_COMMANDED_CHORD_SEGMENT_AXIS_CAPACITY> coordinateMCS{};
    std::uint32_t axisMask = 0U;
    std::uint16_t schemaVersion = 0U;
    NCPathCoreCommandedChordKind kind = NCPathCoreCommandedChordKind::NONE;
    NCPathCoreCommittedGeometryFrame frame = NCPathCoreCommittedGeometryFrame::NONE;

    void Clear() noexcept
    {
        position.Clear();
        for (std::size_t axis = 0U; axis < coordinateMCS.size(); ++axis)
            coordinateMCS[axis] = 0.0;
        axisMask = 0U;
        schemaVersion = 0U;
        kind = NCPathCoreCommandedChordKind::NONE;
        frame = NCPathCoreCommittedGeometryFrame::NONE;
    }
};

NCPathCoreCommandedChordPositionCode BindCommandedChordPosition(
    const NCPathCoreCommandedChordStoreV1& store,
    const NCPathCoreCommandedChordCursorV1& cursor, double unitParameter,
    NCPathCoreCommandedChordSegmentV1& workspace,
    NCPathCoreCommandedChordPositionV1& output) noexcept;

NCPathCoreCommandedChordPositionCode LocateCommandedChordPositionAxis(
    const NCPathCoreCommandedChordStoreV1& store,
    const NCPathCoreCommandedChordCursorV1& cursor, std::uint32_t axisIndex,
    double queryCoordinateMCS, NCPathCoreCommandedChordSegmentV1& workspace,
    NCPathCoreCommandedChordPositionV1& output,
    NCPathCoreCommandedChordLocationV1& detail) noexcept;

NCPathCoreCommandedChordPositionCode EvaluateCommandedChordPosition(
    const NCPathCoreCommandedChordStoreV1& store,
    const NCPathCoreCommandedChordPositionV1& position,
    NCPathCoreCommandedChordSegmentV1& workspace,
    NCPathCoreCommandedChordPositionSampleV1& output) noexcept;

static_assert(sizeof(NCPathCoreCommandedChordPositionCode) == 1U,
    "BB position result must remain one byte.");
static_assert(sizeof(NCPathCoreCommandedChordPositionV1) == 48U &&
    alignof(NCPathCoreCommandedChordPositionV1) == 8U &&
    offsetof(NCPathCoreCommandedChordPositionV1, cursor) == 0U &&
    offsetof(NCPathCoreCommandedChordPositionV1, unitParameter) == 40U,
    "BB position is one 40-byte cursor plus one double, not a wire ABI.");
static_assert(sizeof(NCPathCoreCommandedChordPositionSampleV1) == 120U &&
    alignof(NCPathCoreCommandedChordPositionSampleV1) == 8U &&
    offsetof(NCPathCoreCommandedChordPositionSampleV1, position) == 0U &&
    offsetof(NCPathCoreCommandedChordPositionSampleV1, coordinateMCS) == 48U &&
    offsetof(NCPathCoreCommandedChordPositionSampleV1, axisMask) == 112U &&
    offsetof(NCPathCoreCommandedChordPositionSampleV1, schemaVersion) == 116U &&
    offsetof(NCPathCoreCommandedChordPositionSampleV1, kind) == 118U &&
    offsetof(NCPathCoreCommandedChordPositionSampleV1, frame) == 119U,
    "BB sample is 120 bytes: position + eight coordinates + metadata.");
static_assert(std::is_standard_layout<NCPathCoreCommandedChordPositionV1>::value&&
    std::is_trivially_copyable<NCPathCoreCommandedChordPositionV1>::value&&
    std::is_trivially_destructible<NCPathCoreCommandedChordPositionV1>::value&&
    std::is_nothrow_default_constructible<NCPathCoreCommandedChordPositionV1>::value&&
    std::is_same<decltype(NCPathCoreCommandedChordPositionV1::cursor),
    NCPathCoreCommandedChordCursorV1>::value,
    "BB position must remain a plain bounded value.");
static_assert(std::is_standard_layout<NCPathCoreCommandedChordPositionSampleV1>::value&&
    std::is_trivially_copyable<NCPathCoreCommandedChordPositionSampleV1>::value&&
    std::is_trivially_destructible<NCPathCoreCommandedChordPositionSampleV1>::value&&
    std::is_nothrow_default_constructible<NCPathCoreCommandedChordPositionSampleV1>::value,
    "BB sample must remain a plain bounded caller-owned output.");
