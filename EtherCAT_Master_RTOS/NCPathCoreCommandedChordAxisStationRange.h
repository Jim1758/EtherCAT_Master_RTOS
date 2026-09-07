#pragma once

#include "NCPathCoreCommandedChordAxisStationResolve.h"

// NC-0.2L.2BK / Explicit Station Endpoint Range Capture.
// Resolve TWO explicit endpoints on the SAME CURRENT immutable source snapshot
// using BJ, then capture their range using BG. Each pieceIndex is a 1-based
// SOURCE snapshot traversal index, not an original ordinal or destination index.
// Both stations use the same axis and the source's rounded BF/BI prefix profile.
// Candidate endpoints have no u argument; Interval endpoints require original
// source u for PARAMETER_INTERVAL, even for a zero-span clip. Neither mode can
// override located candidates, no-match or numeric loss. No implicit selection,
// seam deduplication, parameter clamping or station/index ordering is supplied.
//
// Order: source==destination FIRST -> clear destination/output/workspace ->
// complete first BJ resolution -> complete last BJ resolution -> one BG range
// capture -> output schema commit. An alias returns ALIASED_SNAPSHOT and leaves
// ALL objects bytewise unchanged, before scalar checks or Clear. Otherwise every
// failure clears the WHOLE destination/output/workspace; success also clears
// workspace. The entire first endpoint precedes any last-endpoint query, with
// each endpoint retaining BJ's full source/numeric/station/selection/mode/u
// precedence. Equal endpoints still require two complete BJ calls. Nested
// failure codes are retained exactly; unrelated status fields remain NONE.
// Schema commit is not thread publication. Source is unchanged on every exit.
//
// BG determines direction from ORIGINAL endpoint ordinals/u, including reversal
// and same-piece stationary ranges. Distinct original seam/POINT/zero-clip
// identities survive. Destination holds full original AY segments and clips;
// no synthetic endpoints are created. Output first/last retain full BJ values,
// all eight native MCS coordinates and original SOURCE indices. An interval's
// candidateParameter stays BI's +0; explicit u stays in sample.position.
// Signed zero follows BJ/BG without normalization.
//
// Completed destination and outputs are independent values and survive source
// Clear/recapture/destruction and original Store changes/faults/destruction.
// Each new call uses the CURRENT capture, so an index from an earlier capture
// is not a stale ticket or proof of unchanged geometry. No source pointer is
// retained. Provenance is not authentication, a live binding or permission.
// Stations are rounded native single-axis prefix coordinates, not a certified
// physical inverse, mixed mm/degree length or measured progress. Destination
// variation need not equal abs(lastStation-firstStation); exact roundtrip,
// reversal/repartition invariance and physical error bounds are not promised.
// Explicit plateau u is caller choice. No actual G00 path/history, Motion
// acceptance/completion/feedback, planner, Path Queue or B2 semantics are added.
//
// Source, destination, Result and the EXISTING BJ Workspace must be disjoint,
// alive, heap-preallocated outside Observe/PDO, with one owning thread and no
// mutation/reentry for the WHOLE call. The checked source==destination case is
// the only supported alias exception; illegal overlapping lifetimes are not
// checked. No allocation, bulk local/by-value copy, new runtime member/caller,
// Store admission, owner allocator, lifecycle, observer, thread, timer, mutex,
// wait, sleep, high-frequency log, hook, PDO/Alarm/Gate or Motion change.
//
// Success: two BJ calls plus one BG CaptureRange. For N<=32 source pieces and
// M<=32 destination pieces: 2N+2+M selected eight-axis validations (<=98), 2N BI
// classifications (<=64), 16 AW interpolations and M original Segment copies
// (<=32). Fixed-capacity Clears may repeat. These are not WCET or an RTX64 full
// call-chain stack bound. Inherited precise FP, nearest rounding, gradual
// underflow and masked exceptions apply. FP control state is unchanged;
// arithmetic may set sticky flags. No floating-point repair or new contraction
// guarantee is introduced.

enum class NCPathCoreCommandedChordAxisStationRangeCode : std::uint8_t
{
    NONE = 0U,
    CAPTURED = 1U,
    ALIASED_SNAPSHOT = 2U,
    FIRST_ENDPOINT_REJECTED = 3U,
    LAST_ENDPOINT_REJECTED = 4U,
    RANGE_REJECTED = 5U
};

// Small copyable return value. This is process-local, not a wire/SHM ABI.
struct NCPathCoreCommandedChordAxisStationRangeStatusV1 final
{
    NCPathCoreCommandedChordAxisStationRangeStatusV1() noexcept = default;

    NCPathCoreCommandedChordAxisStationRangeCode code = NCPathCoreCommandedChordAxisStationRangeCode::NONE;
    NCPathCoreCommandedChordAxisStationResolveCode resolveCode = NCPathCoreCommandedChordAxisStationResolveCode::NONE;
    NCPathCoreCommandedChordSnapshotCode snapshotCode = NCPathCoreCommandedChordSnapshotCode::NONE;
    std::uint8_t reserved = 0U;
};

// Caller-owned heap output. Reuses BJ Workspace6168 without another wrapper.
struct NCPathCoreCommandedChordAxisStationRangeResultV1 final
{
    NCPathCoreCommandedChordAxisStationRangeResultV1() noexcept = default;
    NCPathCoreCommandedChordAxisStationRangeResultV1(const NCPathCoreCommandedChordAxisStationRangeResultV1&) = delete;
    NCPathCoreCommandedChordAxisStationRangeResultV1& operator=(const NCPathCoreCommandedChordAxisStationRangeResultV1&) = delete;
    NCPathCoreCommandedChordAxisStationRangeResultV1(NCPathCoreCommandedChordAxisStationRangeResultV1&&) = delete;
    NCPathCoreCommandedChordAxisStationRangeResultV1& operator=(NCPathCoreCommandedChordAxisStationRangeResultV1&&) = delete;
    ~NCPathCoreCommandedChordAxisStationRangeResultV1() = default;

    NCPathCoreCommandedChordAxisStationResolveResultV1 first{};
    NCPathCoreCommandedChordAxisStationResolveResultV1 last{};
    std::uint16_t schemaVersion = 0U;
    std::uint8_t reserved[6U]{};

    void Clear() noexcept
    {
        first.Clear();
        last.Clear();
        schemaVersion = 0U;
        for (auto& byte : reserved) byte = 0U;
    }
};

NCPathCoreCommandedChordAxisStationRangeStatusV1 CaptureCommandedChordSnapshotAxisStationCandidateRange(
    const NCPathCoreCommandedChordSnapshotV1& source, std::uint32_t axisIndex,
    double firstStation, std::uint32_t firstPieceIndex,
    double lastStation, std::uint32_t lastPieceIndex,
    NCPathCoreCommandedChordAxisStationResolveWorkspaceV1& workspace,
    NCPathCoreCommandedChordSnapshotV1& destination,
    NCPathCoreCommandedChordAxisStationRangeResultV1& output) noexcept;

NCPathCoreCommandedChordAxisStationRangeStatusV1 CaptureCommandedChordSnapshotAxisStationCandidateIntervalRange(
    const NCPathCoreCommandedChordSnapshotV1& source, std::uint32_t axisIndex,
    double firstStation, std::uint32_t firstPieceIndex,
    double lastStation, std::uint32_t lastPieceIndex, double lastSourceU,
    NCPathCoreCommandedChordAxisStationResolveWorkspaceV1& workspace,
    NCPathCoreCommandedChordSnapshotV1& destination,
    NCPathCoreCommandedChordAxisStationRangeResultV1& output) noexcept;

NCPathCoreCommandedChordAxisStationRangeStatusV1 CaptureCommandedChordSnapshotAxisStationIntervalCandidateRange(
    const NCPathCoreCommandedChordSnapshotV1& source, std::uint32_t axisIndex,
    double firstStation, std::uint32_t firstPieceIndex, double firstSourceU,
    double lastStation, std::uint32_t lastPieceIndex,
    NCPathCoreCommandedChordAxisStationResolveWorkspaceV1& workspace,
    NCPathCoreCommandedChordSnapshotV1& destination,
    NCPathCoreCommandedChordAxisStationRangeResultV1& output) noexcept;

NCPathCoreCommandedChordAxisStationRangeStatusV1 CaptureCommandedChordSnapshotAxisStationIntervalRange(
    const NCPathCoreCommandedChordSnapshotV1& source, std::uint32_t axisIndex,
    double firstStation, std::uint32_t firstPieceIndex, double firstSourceU,
    double lastStation, std::uint32_t lastPieceIndex, double lastSourceU,
    NCPathCoreCommandedChordAxisStationResolveWorkspaceV1& workspace,
    NCPathCoreCommandedChordSnapshotV1& destination,
    NCPathCoreCommandedChordAxisStationRangeResultV1& output) noexcept;

static_assert(sizeof(NCPathCoreCommandedChordAxisStationRangeCode) == 1U &&
    sizeof(NCPathCoreCommandedChordAxisStationRangeStatusV1) == 4U &&
    alignof(NCPathCoreCommandedChordAxisStationRangeStatusV1) == 1U &&
    offsetof(NCPathCoreCommandedChordAxisStationRangeStatusV1, code) == 0U &&
    offsetof(NCPathCoreCommandedChordAxisStationRangeStatusV1, resolveCode) == 1U &&
    offsetof(NCPathCoreCommandedChordAxisStationRangeStatusV1, snapshotCode) == 2U &&
    offsetof(NCPathCoreCommandedChordAxisStationRangeStatusV1, reserved) == 3U &&
    std::is_standard_layout<NCPathCoreCommandedChordAxisStationRangeStatusV1>::value &&
    std::is_trivially_copyable<NCPathCoreCommandedChordAxisStationRangeStatusV1>::value &&
    std::is_trivially_destructible<NCPathCoreCommandedChordAxisStationRangeStatusV1>::value &&
    std::is_nothrow_default_constructible<NCPathCoreCommandedChordAxisStationRangeStatusV1>::value,
    "BK returns only a four-byte copyable status value.");
static_assert(sizeof(NCPathCoreCommandedChordAxisStationRangeResultV1) == 504U &&
    alignof(NCPathCoreCommandedChordAxisStationRangeResultV1) == 8U &&
    offsetof(NCPathCoreCommandedChordAxisStationRangeResultV1, first) == 0U &&
    offsetof(NCPathCoreCommandedChordAxisStationRangeResultV1, last) == 248U &&
    offsetof(NCPathCoreCommandedChordAxisStationRangeResultV1, schemaVersion) == 496U &&
    offsetof(NCPathCoreCommandedChordAxisStationRangeResultV1, reserved) == 498U &&
    std::is_standard_layout<NCPathCoreCommandedChordAxisStationRangeResultV1>::value &&
    std::is_trivially_destructible<NCPathCoreCommandedChordAxisStationRangeResultV1>::value &&
    std::is_nothrow_default_constructible<NCPathCoreCommandedChordAxisStationRangeResultV1>::value &&
    !std::is_copy_constructible<NCPathCoreCommandedChordAxisStationRangeResultV1>::value &&
    !std::is_copy_assignable<NCPathCoreCommandedChordAxisStationRangeResultV1>::value &&
    !std::is_move_constructible<NCPathCoreCommandedChordAxisStationRangeResultV1>::value &&
    !std::is_move_assignable<NCPathCoreCommandedChordAxisStationRangeResultV1>::value,
    "BK heap result is two BJ Result248 values plus metadata8; no bulk copies.");
