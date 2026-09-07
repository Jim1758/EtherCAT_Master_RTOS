#pragma once

#include "NCPathCoreCommandedChordSubpath.h"

// NC-0.2L.2BD / Fixed-Capacity Directed Commanded Subpath Snapshot Contract.
// Capture the FULL original AY segments of one BC directed subpath into a
// standalone, heap-owned snapshot. Clip bounds remain source-segment u values;
// no clipped endpoints are relabelled as a new AY segment with the old identity.
//
// Capture revalidates BOTH BC endpoint sources before either scalar, then reads
// every selected segment from the SAME immutable Store. It clears old contents
// first and commits count/schema only after all reads succeed. Failure clears
// the entire snapshot AND caller Segment workspace; no partial snapshot stays
// usable. Success leaves workspace holding the original end segment.
//
// Pure ReadPiece/EvaluatePiece NEVER consult the Store. They remain legal after
// the source Store is appended, invalidated, faulted, reopened or destroyed.
// Returned cursor/Position bindings are CAPTURED PROVENANCE LABELS, not current
// Store revalidation, execution proof, permission to move or a source lease.
// A caller must use BA/BB separately if it needs a current Store query.
//
// ReadPiece uses 1-based indices and returns the full original AY Segment,
// its BC clip bounds and this snapshot's fixed count/direction. EvaluatePiece
// accepts sourceU in that piece's CLOSED clip interval. It is NOT a new local
// 0..1 parameter of the clipped piece. No parameter remapping/extra rounding,
// distance, mixed-unit norm, rotary unwrap, planner, Motion/Path Queue or B2.
// POINT and zero-parameter pieces are retained. Explicit POINT u is legal.
// Same-segment direction, +/-0 and distinct seam identities inherit BC.
// Evaluation reuses original AW interpolation on the ORIGINAL eight endpoints.
//
// Query order: captured state/metadata -> index -> selected AY value -> scalar
// (Evaluate only). Every query failure clears ALL its outputs, but leaves the
// snapshot unchanged. No query silently refreshes or invalidates its snapshot.
// Only active slots [0,count) are values; unused slots are cleared by Capture, CaptureRange,
// CaptureJoined and Clear, not exposed or treated as authentication/serialized padding.
// Private storage is written only by Capture/CaptureRange/CaptureJoined/Clear. Query validation of the
// selected value does not promise a complete arbitrary-memory-damage audit.
//
// Allocate Snapshot, Store, Segment workspaces and Sample outputs on the HEAP
// before use, outside Observe/PDO. No stack snapshot or by-value bulk copy.
// Copy/move are deleted to prevent accidental 5-KiB implicit copies. This is
// captured mathematical storage, not another observer or runtime owner member.
// ALL objects passed as inputs/outputs/this/Store must be disjoint and alive.
// One owning thread, no reentry or mutation for the WHOLE operation; no cross-
// thread publication is supplied by the final schema/count writes. Trusted
// source owner/lifetime, ownerTag non-reuse and AW precise-FP remain external.
//
// Capture N<=32: <=6+2N AZ const calls (<=70), <=4+2N fixed eight-axis
// validations (<=68), one bounded N-slot copy loop. Clear visits all 32 slots;
// failure can clear them again. Read/Evaluate validate ONE selected AY value;
// Evaluate adds eight original AW interpolations. These are work counts, not
// WCET or a full target call-chain stack bound. No allocation/thread/timer/
// mutex/wait/sleep/log, production caller/member, lifecycle or control hooks.

enum class NCPathCoreCommandedChordSnapshotCode : std::uint8_t
{
    NONE = 0U,
    CAPTURED = 1U,
    PIECE_READ = 2U,
    EVALUATED_LINE_CHORD = 3U,
    EVALUATED_POINT_CHORD = 4U,
    STORE_CLOSED = 5U,
    STORE_FAULTED = 6U,
    CURSOR_REJECTED = 7U,
    RETAINED_VALUE_INVALID = 8U,
    STORE_REJECTED = 9U,
    INVALID_PARAMETER = 10U,
    NOT_CAPTURED = 11U,
    PIECE_INDEX_OUTSIDE = 12U,
    INVALID_SNAPSHOT = 13U,
    OUTSIDE_PIECE_RANGE = 14U,
    NONFINITE_ARITHMETIC = 15U,
    ALIASED_SNAPSHOT = 16U,
    SOURCE_SCOPE_MISMATCH = 17U,
    RANGE_NOT_CONTIGUOUS = 18U,
    RANGE_DIRECTION_CHANGE = 19U,
    SEGMENT_VALUE_CONFLICT = 20U
};

// NC-0.2L.2BG / Detached Snapshot Range Derivation and Directed Reversal.
// CaptureRange derives a new snapshot from two positions WITHIN a captured
// source snapshot, without a live Store. Indices are one-based SOURCE snapshot
// piece indices; parameters remain ORIGINAL source u, not clip-local t. Both
// endpoints must lie within their saved closed clips. Either index/u order is
// legal. Child direction compares ORIGINAL ordinals, then u on the same piece;
// it is not inferred from parent indices. Never extends beyond the parent.
// Original eight-axis endpoints, identity, POINT/LINE and all intervening
// pieces are retained. No interpolation, synthetic endpoints, renumbering of
// source ordinals, merged seams or skipped zero clips. Caller +/-0 u is kept.
// Nested crops/reversals remain usable after parent Clear/recapture/destruction.
//
// Alias exception: this == &source returns ALIASED_SNAPSHOT FIRST and leaves
// that sole object byte-unchanged, including when it is invalid. Every OTHER
// call clears destination first; source is always unchanged. Validate first,
// then distinct last source piece BEFORE either scalar (same index only once).
// Both u must be finite in [0,1] before either closed-clip check. Propagate BD
// source rejection; invalid u returns INVALID_PARAMETER, valid u outside its
// saved clip returns OUTSIDE_PIECE_RANGE. Interior value failure clears the
// whole destination, including already copied slots. Success returns CAPTURED;
// schema/count commit follows all copies, not cross-thread synchronization.
//
// Successful N<=32 pieces: N selected-source validations (fixed eight axes),
// N direct segment copies, no live Store/BC/AW/AX calls or extra workspace.
// Endpoints are not revalidated in the copy loop. Clear visits all 32 slots;
// late failure may clear again. No local bulk snapshot/allocation is needed.
// All existing heap ownership, whole-call same-thread/disjoint/live-object,
// trusted provenance and FP preconditions apply; self-alias is the one explicit
// detected exception. Not a full private-corruption audit or current binding
// proof. No production/caller/lifecycle/Motion/Queue/B2 integration is added.
// Work counts are not WCET or a complete RTX64 call-chain stack bound.

// NC-0.2L.2BH / Compatible Directed Snapshot Range Stitching Contract.
// CaptureJoined(first,second) joins CONTIGUOUS SAME-SCOPE directed ranges.
// All three objects must be distinct. ANY pair alias is detected FIRST and
// returns ALIASED_SNAPSHOT with ALL objects unchanged, including destination
// when only first==second. All other calls clear destination before validation;
// every disjoint failure leaves it fully cleared, both sources unchanged.
//
// Validate first's head/tail, then second's head/tail (one read per same-source
// endpoint index), before scope/join checks. Same owner/lifetime/chain required
// or SOURCE_SCOPE_MISMATCH. This compares provenance labels, not authentication.
// Topology must join at equal original ordinal and numerically equal source u,
// or at adjacent ordinals with forward 1->0 / reverse 0->1 parameter seam.
// Otherwise RANGE_NOT_CONTIGUOUS. Coordinate equality on POINT/constant pieces
// does not excuse a parameter gap. Direction derives from OUTER original
// ordinal/u endpoints; both non-STATIONARY source directions and any distinct-
// ordinal join step must agree, else RANGE_DIRECTION_CHANGE. No U-turn/revisit.
//
// At a same-ordinal join, original AY semantic fields and all endpoint double
// BITS must match (not padding). Merge that one duplicate original piece; its
// internal cut marker is not retained, even for zero/stationary clips. Distinct
// original zero clips/POINT/seam identities remain. Outer source u signed zeros
// are preserved. At adjacent ordinals, check original ascending next nonzero
// publication (MAX->1 allowed), run/CONTIGUOUS and eight numeric endpoint joins
// (+/-0 allowed). Any joint value/continuity conflict is SEGMENT_VALUE_CONFLICT.
// Never invent endpoint geometry, source ordinals, source owner or new motion.
//
// Guard combined count <=32 and equal to the outer ordinal extent; then copy
// directly to destination, validating each interior once. Reuse endpoint
// validations; a late source rejection clears all partial output. Commit only
// after all copies; CAPTURED is mathematical success, not thread publication.
// Successful A+B<=33 selected AY eight-axis validations; <=32 segment copies.
// Clear visits 32 slots and may repeat. No local bulk snapshot/workspace,
// allocation, live Store/BC/AW/AX query, observer or runtime owner is added.
//
// Source/destination heap ownership, same-thread whole-call no mutation/reentry,
// trusted provenance and inherited FP preconditions apply. Independent results
// remain usable after parents/Store Clear/recapture/fault/destruction. This is
// a compatible range join, not general concatenation, execution history,
// current binding, control permission, Motion/Path Queue/planner or B2. No
// production caller/member, lifecycle/PDO/Alarm/Gate/Motion change. No arbitrary
// private-corruption audit, WCET or full target call-chain stack guarantee.

class NCPathCoreCommandedChordSnapshotV1 final
{
public:
    static constexpr std::uint32_t Capacity = NCPathCoreCommandedChordStoreV1::Capacity;

    NCPathCoreCommandedChordSnapshotV1() noexcept = default;
    NCPathCoreCommandedChordSnapshotV1(const NCPathCoreCommandedChordSnapshotV1&) = delete;
    NCPathCoreCommandedChordSnapshotV1& operator=(const NCPathCoreCommandedChordSnapshotV1&) = delete;
    NCPathCoreCommandedChordSnapshotV1(NCPathCoreCommandedChordSnapshotV1&&) = delete;
    NCPathCoreCommandedChordSnapshotV1& operator=(NCPathCoreCommandedChordSnapshotV1&&) = delete;
    ~NCPathCoreCommandedChordSnapshotV1() = default;

    void Clear() noexcept;
    NCPathCoreCommandedChordSnapshotCode Capture(
        const NCPathCoreCommandedChordStoreV1& store,
        const NCPathCoreCommandedChordSubpathV1& subpath,
        NCPathCoreCommandedChordSegmentV1& workspace) noexcept;
    NCPathCoreCommandedChordSnapshotCode CaptureRange(
        const NCPathCoreCommandedChordSnapshotV1& source,
        std::uint32_t firstPieceIndex, double firstSourceU,
        std::uint32_t lastPieceIndex, double lastSourceU) noexcept;
    NCPathCoreCommandedChordSnapshotCode CaptureJoined(
        const NCPathCoreCommandedChordSnapshotV1& first,
        const NCPathCoreCommandedChordSnapshotV1& second) noexcept;
    NCPathCoreCommandedChordSnapshotCode ReadPiece(
        std::uint32_t pieceIndex, NCPathCoreCommandedChordSegmentV1& segment,
        NCPathCoreCommandedChordSubpathPieceV1& piece,
        NCPathCoreCommandedChordSubpathInfoV1& info) const noexcept;
    NCPathCoreCommandedChordSnapshotCode EvaluatePiece(
        std::uint32_t pieceIndex, double sourceU,
        NCPathCoreCommandedChordPositionSampleV1& output) const noexcept;

private:
    NCPathCoreCommandedChordSnapshotCode ValidateSelected(std::uint32_t pieceIndex) const noexcept;
    std::uint32_t SourceOrdinal(std::uint32_t pieceIndex) const noexcept;
    double FromParameter(std::uint32_t pieceIndex) const noexcept;
    double ToParameter(std::uint32_t pieceIndex) const noexcept;
    void WriteCursor(std::uint32_t pieceIndex,
        NCPathCoreCommandedChordCursorV1& output) const noexcept;

    NCPathCoreCommandedChordSubpathV1 m_source{};
    std::array<NCPathCoreCommandedChordSegmentV1, Capacity> m_segments{};
    std::uint32_t m_pieceCount = 0U;
    std::uint16_t m_schemaVersion = 0U;
    NCPathCoreCommandedChordSubpathDirection m_direction =
        NCPathCoreCommandedChordSubpathDirection::NONE;
    std::uint8_t m_reserved = 0U;
};

static_assert(NCPathCoreCommandedChordSnapshotV1::Capacity == 32U &&
    sizeof(NCPathCoreCommandedChordSnapshotCode) == 1U,
    "BD is a fixed 32-piece snapshot with one-byte results.");
static_assert(sizeof(NCPathCoreCommandedChordSnapshotV1) == 5224U &&
    alignof(NCPathCoreCommandedChordSnapshotV1) == 8U,
    "BD heap snapshot is 96 + 32 * 160 + 8 = 5224 bytes, not a wire ABI.");
static_assert(std::is_standard_layout<NCPathCoreCommandedChordSnapshotV1>::value&&
    std::is_trivially_destructible<NCPathCoreCommandedChordSnapshotV1>::value&&
    std::is_nothrow_default_constructible<NCPathCoreCommandedChordSnapshotV1>::value &&
    !std::is_copy_constructible<NCPathCoreCommandedChordSnapshotV1>::value &&
    !std::is_copy_assignable<NCPathCoreCommandedChordSnapshotV1>::value &&
    !std::is_move_constructible<NCPathCoreCommandedChordSnapshotV1>::value &&
    !std::is_move_assignable<NCPathCoreCommandedChordSnapshotV1>::value,
    "BD snapshot must stay heap-owned with no implicit bulk copies.");
