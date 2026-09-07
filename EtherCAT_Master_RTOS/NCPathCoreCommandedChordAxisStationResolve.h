#pragma once

#include "NCPathCoreCommandedChordAxisStation.h"

// NC-0.2L.2BJ / Explicit Station Candidate Resolution and Eight-Axis Evaluation.
// Resolve an EXPLICITLY selected 1-based SNAPSHOT traversal piece from a fresh
// BI rounded native-axis prefix query. This index is not an original ordinal.
// Both APIs recompute the WHOLE BI/BF query against the CURRENT captured value
// of snapshot; no earlier BI result is accepted. An index from an earlier query
// is not a ticket proving the same capture still exists. Keep the intended
// snapshot immutable across related operations. Provenance labels are not
// authentication, live Store bindings, executed progress or control permission.
//
// Resolve...Candidate has no u argument. It accepts only BI's three LOCATED
// kinds, preserving the candidateParameter bits. PARAMETER_INTERVAL requires
// the other API even when its clip has zero span; no arbitrary u is invented.
// Resolve...Interval accepts only PARAMETER_INTERVAL and requires caller u in
// [0,1] and in that entry's CLOSED original clip. +/-0 is legal and preserved.
// Located entries reject this API BEFORE validating u; it cannot override a
// located candidate. Neither API bypasses NO_PREFIX_MATCH or numeric loss.
// Distinct pieces at a seam remain distinct explicit choices; no best match,
// deduplication or seam normalization is supplied.
//
// Order: clear output/workspace -> FULL BI query (including all BF source and
// numeric checks before station) -> piece index -> selected classification ->
// API kind -> explicit interval u, if needed -> ONE BD EvaluatePiece call.
// All query errors precede selection/u errors. No-match and parameter loss
// precede API-kind/u errors. An interval u must be finite in [0,1] before its
// closed-clip check. Unexpected query/entry/evaluation states are rejected.
// Every failure clears ALL output and workspace fields. Success also clears
// workspace. Snapshot is unchanged on every exit. Schema/resolution kind and
// other metadata commit only after evaluation, not as thread publication.
//
// The output retains the FULL original BI candidate, including its kind and
// candidateParameter=+0 for an interval. The explicit interval choice lives in
// sample.position.unitParameter; it does not rewrite the candidate. Sample
// keeps POINT/LINE kind and all eight native MCS coordinates, including
// unmasked constant axes, from BD/AW evaluation of the ORIGINAL AY endpoints.
// No synthetic clipped endpoints or extra source parameter remapping is used.
// This evaluates a rounded prefix candidate, not a certified physical inverse:
// re-evaluated coordinates or a rebuilt BF subrange need not reproduce station
// exactly. No error bound, exact roundtrip, reversal/repartition invariance,
// mixed mm/degree norm, rotary unwrap, measured progress, Motion/Path Queue,
// planner or B2 is supplied. Explicit plateau u is the caller's choice.
//
// Result contains independent values and survives source Clear/recapture/
// destruction; it does not become a live source binding. Detached operations
// remain legal after the original Store appends/faults/invalidates/reopens/dies.
// Snapshot, Result and Workspace must be preallocated on the HEAP outside
// Observe/PDO, disjoint and alive for the WHOLE call, with one owning thread,
// no reentry or mutation. Illegal overlapping object lifetimes are not checked.
// No local bulk objects, allocation, runtime member/caller, Store admission,
// owner allocator, lifecycle, observer, thread, timer, mutex, wait, sleep, log,
// hook, PDO/Alarm/Gate or Motion changes are introduced.
//
// One BI query: N<=32 BF ReadPiece/eight-axis validations and N classifications;
// then one BD EvaluatePiece: one selected eight-axis validation and eight
// original AW interpolations. Success therefore uses <=33 validations,
// <=32 classifications and eight interpolations. No second BI query, extra
// ReadPiece or Segment workspace is needed. Fixed 32-entry result/profile
// clears may repeat. These work counts are not WCET or an RTX64 full call-chain
// stack bound. Inherited precise FP, round-to-nearest, gradual underflow and
// masked-exception preconditions apply. FP control state is unchanged;
// ordinary arithmetic may set sticky flags. No new contraction constraint.

enum class NCPathCoreCommandedChordAxisStationResolveCode : std::uint8_t
{
    NONE = 0U,
    RESOLVED_CANDIDATE = 1U,
    RESOLVED_INTERVAL = 2U,
    NOT_CAPTURED = 3U,
    SNAPSHOT_REJECTED = 4U,
    INVALID_AXIS = 5U,
    NONFINITE_VARIATION = 6U,
    VARIATION_RESOLUTION_LOSS = 7U,
    NONFINITE_TOTAL = 8U,
    PREFIX_RESOLUTION_LOSS = 9U,
    INVALID_STATION = 10U,
    OUTSIDE_PROFILE = 11U,
    PIECE_INDEX_OUTSIDE = 12U,
    NO_PREFIX_MATCH = 13U,
    PARAMETER_RESOLUTION_LOSS = 14U,
    INTERVAL_PARAMETER_REQUIRED = 15U,
    NOT_PARAMETER_INTERVAL = 16U,
    INVALID_PARAMETER = 17U,
    OUTSIDE_PIECE_RANGE = 18U,
    NONFINITE_ARITHMETIC = 19U,
    GEOMETRY_REJECTED = 20U
};

enum class NCPathCoreCommandedChordAxisStationResolutionKind : std::uint8_t
{
    NONE = 0U,
    LOCATED_CANDIDATE = 1U,
    EXPLICIT_INTERVAL_PARAMETER = 2U
};

// Process-local caller-owned output, not a wire/SHM ABI or runtime member.
struct NCPathCoreCommandedChordAxisStationResolveResultV1 final
{
    NCPathCoreCommandedChordAxisStationResolveResultV1() noexcept = default;
    NCPathCoreCommandedChordAxisStationResolveResultV1(const NCPathCoreCommandedChordAxisStationResolveResultV1&) = delete;
    NCPathCoreCommandedChordAxisStationResolveResultV1& operator=(const NCPathCoreCommandedChordAxisStationResolveResultV1&) = delete;
    NCPathCoreCommandedChordAxisStationResolveResultV1(NCPathCoreCommandedChordAxisStationResolveResultV1&&) = delete;
    NCPathCoreCommandedChordAxisStationResolveResultV1& operator=(NCPathCoreCommandedChordAxisStationResolveResultV1&&) = delete;
    ~NCPathCoreCommandedChordAxisStationResolveResultV1() = default;

    NCPathCoreCommandedChordAxisStationCandidateV1 candidate{};
    NCPathCoreCommandedChordPositionSampleV1 sample{};
    double station = 0.0;
    double totalVariation = 0.0;
    std::uint32_t axisIndex = 0U;
    std::uint32_t pieceIndex = 0U;
    std::uint16_t schemaVersion = 0U;
    NCPathCoreCommandedChordSubpathDirection direction = NCPathCoreCommandedChordSubpathDirection::NONE;
    NCPathCoreCommandedChordAxisStationResolutionKind resolutionKind = NCPathCoreCommandedChordAxisStationResolutionKind::NONE;
    std::uint8_t reserved[4U]{};

    void Clear() noexcept
    {
        candidate.Clear();
        sample.Clear();
        station = 0.0;
        totalVariation = 0.0;
        axisIndex = 0U;
        pieceIndex = 0U;
        schemaVersion = 0U;
        direction = NCPathCoreCommandedChordSubpathDirection::NONE;
        resolutionKind = NCPathCoreCommandedChordAxisStationResolutionKind::NONE;
        for (auto& byte : reserved) byte = 0U;
    }
};

struct NCPathCoreCommandedChordAxisStationResolveWorkspaceV1 final
{
    NCPathCoreCommandedChordAxisStationResolveWorkspaceV1() noexcept = default;
    NCPathCoreCommandedChordAxisStationResolveWorkspaceV1(const NCPathCoreCommandedChordAxisStationResolveWorkspaceV1&) = delete;
    NCPathCoreCommandedChordAxisStationResolveWorkspaceV1& operator=(const NCPathCoreCommandedChordAxisStationResolveWorkspaceV1&) = delete;
    NCPathCoreCommandedChordAxisStationResolveWorkspaceV1(NCPathCoreCommandedChordAxisStationResolveWorkspaceV1&&) = delete;
    NCPathCoreCommandedChordAxisStationResolveWorkspaceV1& operator=(NCPathCoreCommandedChordAxisStationResolveWorkspaceV1&&) = delete;
    ~NCPathCoreCommandedChordAxisStationResolveWorkspaceV1() = default;

    NCPathCoreCommandedChordAxisStationResultV1 candidates{};
    NCPathCoreCommandedChordAxisStationWorkspaceV1 query{};

    void Clear() noexcept { candidates.Clear(); query.Clear(); }
};

NCPathCoreCommandedChordAxisStationResolveCode ResolveCommandedChordSnapshotAxisStationCandidate(
    const NCPathCoreCommandedChordSnapshotV1& snapshot, std::uint32_t axisIndex,
    double station, std::uint32_t pieceIndex,
    NCPathCoreCommandedChordAxisStationResolveWorkspaceV1& workspace,
    NCPathCoreCommandedChordAxisStationResolveResultV1& output) noexcept;

NCPathCoreCommandedChordAxisStationResolveCode ResolveCommandedChordSnapshotAxisStationInterval(
    const NCPathCoreCommandedChordSnapshotV1& snapshot, std::uint32_t axisIndex,
    double station, std::uint32_t pieceIndex, double sourceU,
    NCPathCoreCommandedChordAxisStationResolveWorkspaceV1& workspace,
    NCPathCoreCommandedChordAxisStationResolveResultV1& output) noexcept;

static_assert(sizeof(NCPathCoreCommandedChordAxisStationResolveCode) == 1U &&
    sizeof(NCPathCoreCommandedChordAxisStationResolutionKind) == 1U,
    "BJ status and resolution enums remain one byte.");
static_assert(sizeof(NCPathCoreCommandedChordAxisStationResolveResultV1) == 248U &&
    alignof(NCPathCoreCommandedChordAxisStationResolveResultV1) == 8U &&
    offsetof(NCPathCoreCommandedChordAxisStationResolveResultV1, candidate) == 0U &&
    offsetof(NCPathCoreCommandedChordAxisStationResolveResultV1, sample) == 96U &&
    offsetof(NCPathCoreCommandedChordAxisStationResolveResultV1, station) == 216U &&
    offsetof(NCPathCoreCommandedChordAxisStationResolveResultV1, totalVariation) == 224U &&
    offsetof(NCPathCoreCommandedChordAxisStationResolveResultV1, axisIndex) == 232U &&
    offsetof(NCPathCoreCommandedChordAxisStationResolveResultV1, pieceIndex) == 236U &&
    offsetof(NCPathCoreCommandedChordAxisStationResolveResultV1, schemaVersion) == 240U &&
    offsetof(NCPathCoreCommandedChordAxisStationResolveResultV1, direction) == 242U &&
    offsetof(NCPathCoreCommandedChordAxisStationResolveResultV1, resolutionKind) == 243U &&
    offsetof(NCPathCoreCommandedChordAxisStationResolveResultV1, reserved) == 244U,
    "BJ heap result is Candidate96 + Sample120 + metadata32.");
static_assert(sizeof(NCPathCoreCommandedChordAxisStationResolveWorkspaceV1) == 6168U &&
    alignof(NCPathCoreCommandedChordAxisStationResolveWorkspaceV1) == 8U &&
    offsetof(NCPathCoreCommandedChordAxisStationResolveWorkspaceV1, candidates) == 0U &&
    offsetof(NCPathCoreCommandedChordAxisStationResolveWorkspaceV1, query) == 3104U,
    "BJ heap workspace is BI Result3104 + Workspace3064.");
static_assert(std::is_standard_layout<NCPathCoreCommandedChordAxisStationResolveResultV1>::value&&
    std::is_trivially_destructible<NCPathCoreCommandedChordAxisStationResolveResultV1>::value&&
    std::is_nothrow_default_constructible<NCPathCoreCommandedChordAxisStationResolveResultV1>::value &&
    !std::is_copy_constructible<NCPathCoreCommandedChordAxisStationResolveResultV1>::value &&
    !std::is_copy_assignable<NCPathCoreCommandedChordAxisStationResolveResultV1>::value &&
    !std::is_move_constructible<NCPathCoreCommandedChordAxisStationResolveResultV1>::value &&
    !std::is_move_assignable<NCPathCoreCommandedChordAxisStationResolveResultV1>::value&&
    std::is_standard_layout<NCPathCoreCommandedChordAxisStationResolveWorkspaceV1>::value&&
    std::is_trivially_destructible<NCPathCoreCommandedChordAxisStationResolveWorkspaceV1>::value&&
    std::is_nothrow_default_constructible<NCPathCoreCommandedChordAxisStationResolveWorkspaceV1>::value &&
    !std::is_copy_constructible<NCPathCoreCommandedChordAxisStationResolveWorkspaceV1>::value &&
    !std::is_copy_assignable<NCPathCoreCommandedChordAxisStationResolveWorkspaceV1>::value &&
    !std::is_move_constructible<NCPathCoreCommandedChordAxisStationResolveWorkspaceV1>::value &&
    !std::is_move_assignable<NCPathCoreCommandedChordAxisStationResolveWorkspaceV1>::value,
    "BJ result/workspace must stay heap-owned without implicit bulk copies.");
