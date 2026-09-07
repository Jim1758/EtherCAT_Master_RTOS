#pragma once

#include "NCPathCoreCommandedChordAxisVariation.h"

// NC-0.2L.2BI / Rounded Native-Axis Variation Prefix Candidate Location Contract.
// Query ONE station in BF's ROUNDED prefix-knot coordinate for ONE native MCS
// axis. This is not exact geometric distance or an inverse of original
// |b-a| * clip span: prefixTo-prefixFrom can differ from saved variation.
// No mixed mm/degree norm, rotary unwrap, executed G00 travel, Motion progress,
// history, planning, Path/Motion Queue, B2 or control permission is supplied.
//
// First compute the FULL BF profile, preserving its validation order and
// numeric refusals, BEFORE validating station. A finite station >=0 is legal
// when <=totalVariation. Signed zero is legal and preserved; no clamping.
// Every BF piece remains in traversal order, including POINT, constant axes,
// zero clips and both original identities at a seam. No best match or dedup.
// Each copied entry receives one classification:
// * NO_PREFIX_MATCH: station is outside the closed saved prefix interval.
// * PARAMETER_INTERVAL: station matches a zero-width prefix plateau. The
//   WHOLE saved directed clip is the answer, even if its span is zero.
//   No arbitrary u is selected; both clip endpoint bits remain preserved.
// * LOCATED_CLIP_FROM / LOCATED_CLIP_TO: exact equality with a distinct
//   prefix endpoint returns that clip endpoint with its original bits.
// * LOCATED_INTERIOR: strict prefix interior uses
//   t=(station-prefixFrom)/(prefixTo-prefixFrom), then
//   candidate=clipFrom+(clipTo-clipFrom)*t. Both finite t in (0,1) and a finite
//   candidate STRICTLY between the numerical clip endpoints are required.
// * PARAMETER_RESOLUTION_LOSS: an interior candidate fails these conditions.
//   This remains a QUERIED classification, not absence of a true solution.
// ONLY the three LOCATED kinds have a meaningful candidateParameter.
// All other kinds keep candidateParameter=+0. No fallback, nextafter, nearest
// endpoint, clamp, AW/AX evaluation or reconstructed-coordinate test is used.
// Interior candidates are rounded affine-prefix candidates, not certified
// exact inverses or error bounds. The expression order is specified, but no
// new FP-contraction requirement or cross-target bit identity is promised.
// BF's rounding, traversal/repartition/reversal non-invariance and numeric
// refusals remain. Round-to-nearest, gradual underflow, masked FP exceptions
// and inherited precise-FP preconditions apply; no FP control state is changed.
// Ordinary arithmetic may set sticky FP exception flags, as in BF.
//
// Source Snapshot stays unchanged on every exit. Captured cursor fields are
// provenance LABELS, not authenticated live bindings. Detached mathematics
// remains legal after the original Store faults/invalidates/reopens/dies.
// Any query-level failure clears ALL output and workspace fields. Success
// also clears workspace; inactive entries stay clear. Global schema/count
// commit only after all classifications, not as cross-thread publication.
//
// Heap-allocate Snapshot, Result and Workspace outside Observe/PDO. All must
// be disjoint and alive for the WHOLE call, with one owning thread and no
// reentry/mutation. Illegal overlapping object lifetimes/aliasing are not
// detected. No local bulk objects, allocation, runtime member/caller, live
// Store/lifecycle, observer, thread, timer, mutex, wait, sleep, log or hook.
// One BF query: successful N<=32 uses N ReadPiece calls and N fixed eight-axis
// validations, followed by <=32 candidate transforms. Clear scans 32 result
// and 32 profile slots at entry/completion/failure and can repeat. These are
// bounded work counts, not WCET or a complete RTX64 call-chain stack bound.

enum class NCPathCoreCommandedChordAxisStationCode : std::uint8_t
{
    NONE = 0U,
    QUERIED = 1U,
    NOT_CAPTURED = 2U,
    SNAPSHOT_REJECTED = 3U,
    INVALID_AXIS = 4U,
    NONFINITE_VARIATION = 5U,
    VARIATION_RESOLUTION_LOSS = 6U,
    NONFINITE_TOTAL = 7U,
    PREFIX_RESOLUTION_LOSS = 8U,
    INVALID_STATION = 9U,
    OUTSIDE_PROFILE = 10U
};

enum class NCPathCoreCommandedChordAxisStationCandidateKind : std::uint8_t
{
    NONE = 0U,
    NO_PREFIX_MATCH = 1U,
    PARAMETER_INTERVAL = 2U,
    LOCATED_CLIP_FROM = 3U,
    LOCATED_INTERIOR = 4U,
    LOCATED_CLIP_TO = 5U,
    PARAMETER_RESOLUTION_LOSS = 6U
};

struct NCPathCoreCommandedChordAxisStationCandidateV1
{
    NCPathCoreCommandedChordCursorV1 cursor{};
    double clipFrom = 0.0;
    double clipTo = 0.0;
    double variation = 0.0;
    double prefixFrom = 0.0;
    double prefixTo = 0.0;
    double candidateParameter = 0.0;
    NCPathCoreCommandedChordAxisStationCandidateKind kind = NCPathCoreCommandedChordAxisStationCandidateKind::NONE;
    NCPathCoreCommandedChordAxisVariationKind variationKind = NCPathCoreCommandedChordAxisVariationKind::NONE;
    NCPathCoreCommandedChordKind sourceKind = NCPathCoreCommandedChordKind::NONE;
    std::uint8_t reserved[5U]{};

    void Clear() noexcept
    {
        cursor.Clear();
        clipFrom = 0.0;
        clipTo = 0.0;
        variation = 0.0;
        prefixFrom = 0.0;
        prefixTo = 0.0;
        candidateParameter = 0.0;
        kind = NCPathCoreCommandedChordAxisStationCandidateKind::NONE;
        variationKind = NCPathCoreCommandedChordAxisVariationKind::NONE;
        sourceKind = NCPathCoreCommandedChordKind::NONE;
        for (auto& byte : reserved) byte = 0U;
    }
};

// Process-local caller-owned output, not a wire/SHM ABI or runtime member.
struct NCPathCoreCommandedChordAxisStationResultV1 final
{
    NCPathCoreCommandedChordAxisStationResultV1() noexcept = default;
    NCPathCoreCommandedChordAxisStationResultV1(const NCPathCoreCommandedChordAxisStationResultV1&) = delete;
    NCPathCoreCommandedChordAxisStationResultV1& operator=(const NCPathCoreCommandedChordAxisStationResultV1&) = delete;
    NCPathCoreCommandedChordAxisStationResultV1(NCPathCoreCommandedChordAxisStationResultV1&&) = delete;
    NCPathCoreCommandedChordAxisStationResultV1& operator=(NCPathCoreCommandedChordAxisStationResultV1&&) = delete;
    ~NCPathCoreCommandedChordAxisStationResultV1() = default;

    std::array<NCPathCoreCommandedChordAxisStationCandidateV1, NCPathCoreCommandedChordSnapshotV1::Capacity> entries{};
    double station = 0.0;
    double totalVariation = 0.0;
    std::uint32_t axisIndex = 0U;
    std::uint32_t pieceCount = 0U;
    std::uint16_t schemaVersion = 0U;
    NCPathCoreCommandedChordSubpathDirection direction = NCPathCoreCommandedChordSubpathDirection::NONE;
    std::uint8_t reserved[5U]{};

    void Clear() noexcept
    {
        for (auto& entry : entries) entry.Clear();
        station = 0.0;
        totalVariation = 0.0;
        axisIndex = 0U;
        pieceCount = 0U;
        schemaVersion = 0U;
        direction = NCPathCoreCommandedChordSubpathDirection::NONE;
        for (auto& byte : reserved) byte = 0U;
    }
};

struct NCPathCoreCommandedChordAxisStationWorkspaceV1 final
{
    NCPathCoreCommandedChordAxisStationWorkspaceV1() noexcept = default;
    NCPathCoreCommandedChordAxisStationWorkspaceV1(const NCPathCoreCommandedChordAxisStationWorkspaceV1&) = delete;
    NCPathCoreCommandedChordAxisStationWorkspaceV1& operator=(const NCPathCoreCommandedChordAxisStationWorkspaceV1&) = delete;
    NCPathCoreCommandedChordAxisStationWorkspaceV1(NCPathCoreCommandedChordAxisStationWorkspaceV1&&) = delete;
    NCPathCoreCommandedChordAxisStationWorkspaceV1& operator=(NCPathCoreCommandedChordAxisStationWorkspaceV1&&) = delete;
    ~NCPathCoreCommandedChordAxisStationWorkspaceV1() = default;

    NCPathCoreCommandedChordAxisVariationResultV1 profile{};
    NCPathCoreCommandedChordAxisVariationWorkspaceV1 measure{};

    void Clear() noexcept { profile.Clear(); measure.Clear(); }
};

NCPathCoreCommandedChordAxisStationCode QueryCommandedChordSnapshotAxisStationCandidates(
    const NCPathCoreCommandedChordSnapshotV1& snapshot, std::uint32_t axisIndex,
    double station, NCPathCoreCommandedChordAxisStationWorkspaceV1& workspace,
    NCPathCoreCommandedChordAxisStationResultV1& output) noexcept;

static_assert(sizeof(NCPathCoreCommandedChordAxisStationCode) == 1U &&
    sizeof(NCPathCoreCommandedChordAxisStationCandidateKind) == 1U,
    "BI status and classification enums remain one byte.");
static_assert(sizeof(NCPathCoreCommandedChordAxisStationCandidateV1) == 96U &&
    alignof(NCPathCoreCommandedChordAxisStationCandidateV1) == 8U &&
    offsetof(NCPathCoreCommandedChordAxisStationCandidateV1, cursor) == 0U &&
    offsetof(NCPathCoreCommandedChordAxisStationCandidateV1, clipFrom) == 40U &&
    offsetof(NCPathCoreCommandedChordAxisStationCandidateV1, clipTo) == 48U &&
    offsetof(NCPathCoreCommandedChordAxisStationCandidateV1, variation) == 56U &&
    offsetof(NCPathCoreCommandedChordAxisStationCandidateV1, prefixFrom) == 64U &&
    offsetof(NCPathCoreCommandedChordAxisStationCandidateV1, prefixTo) == 72U &&
    offsetof(NCPathCoreCommandedChordAxisStationCandidateV1, candidateParameter) == 80U &&
    offsetof(NCPathCoreCommandedChordAxisStationCandidateV1, kind) == 88U &&
    offsetof(NCPathCoreCommandedChordAxisStationCandidateV1, variationKind) == 89U &&
    offsetof(NCPathCoreCommandedChordAxisStationCandidateV1, sourceKind) == 90U &&
    offsetof(NCPathCoreCommandedChordAxisStationCandidateV1, reserved) == 91U,
    "BI candidate is one cursor, six doubles and eight classification bytes.");
static_assert(sizeof(NCPathCoreCommandedChordAxisStationResultV1) == 3104U &&
    alignof(NCPathCoreCommandedChordAxisStationResultV1) == 8U &&
    offsetof(NCPathCoreCommandedChordAxisStationResultV1, entries) == 0U &&
    offsetof(NCPathCoreCommandedChordAxisStationResultV1, station) == 3072U &&
    offsetof(NCPathCoreCommandedChordAxisStationResultV1, totalVariation) == 3080U &&
    offsetof(NCPathCoreCommandedChordAxisStationResultV1, axisIndex) == 3088U &&
    offsetof(NCPathCoreCommandedChordAxisStationResultV1, pieceCount) == 3092U &&
    offsetof(NCPathCoreCommandedChordAxisStationResultV1, schemaVersion) == 3096U &&
    offsetof(NCPathCoreCommandedChordAxisStationResultV1, direction) == 3098U &&
    offsetof(NCPathCoreCommandedChordAxisStationResultV1, reserved) == 3099U,
    "BI heap result budget is 32 * 96 + 32 bytes.");
static_assert(sizeof(NCPathCoreCommandedChordAxisStationWorkspaceV1) == 3064U &&
    alignof(NCPathCoreCommandedChordAxisStationWorkspaceV1) == 8U &&
    offsetof(NCPathCoreCommandedChordAxisStationWorkspaceV1, profile) == 0U &&
    offsetof(NCPathCoreCommandedChordAxisStationWorkspaceV1, measure) == 2840U,
    "BI heap workspace is BF Result2840 + Workspace224.");
static_assert(std::is_standard_layout<NCPathCoreCommandedChordAxisStationCandidateV1>::value&&
    std::is_trivially_copyable<NCPathCoreCommandedChordAxisStationCandidateV1>::value&&
    std::is_trivially_destructible<NCPathCoreCommandedChordAxisStationCandidateV1>::value,
    "BI entries remain plain bounded mathematical values.");
static_assert(std::is_standard_layout<NCPathCoreCommandedChordAxisStationResultV1>::value&&
    std::is_trivially_destructible<NCPathCoreCommandedChordAxisStationResultV1>::value&&
    std::is_nothrow_default_constructible<NCPathCoreCommandedChordAxisStationResultV1>::value &&
    !std::is_copy_constructible<NCPathCoreCommandedChordAxisStationResultV1>::value &&
    !std::is_copy_assignable<NCPathCoreCommandedChordAxisStationResultV1>::value &&
    !std::is_move_constructible<NCPathCoreCommandedChordAxisStationResultV1>::value &&
    !std::is_move_assignable<NCPathCoreCommandedChordAxisStationResultV1>::value&&
    std::is_standard_layout<NCPathCoreCommandedChordAxisStationWorkspaceV1>::value&&
    std::is_trivially_destructible<NCPathCoreCommandedChordAxisStationWorkspaceV1>::value&&
    std::is_nothrow_default_constructible<NCPathCoreCommandedChordAxisStationWorkspaceV1>::value &&
    !std::is_copy_constructible<NCPathCoreCommandedChordAxisStationWorkspaceV1>::value &&
    !std::is_copy_assignable<NCPathCoreCommandedChordAxisStationWorkspaceV1>::value &&
    !std::is_move_constructible<NCPathCoreCommandedChordAxisStationWorkspaceV1>::value &&
    !std::is_move_assignable<NCPathCoreCommandedChordAxisStationWorkspaceV1>::value,
    "BI result/workspace must stay heap-owned without implicit bulk copies.");
