#pragma once
#include "NCTranslationSnapshot.h"

#include "NCProgramCache.h"
#include "NCGCodeSemantics.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

// =============================================================================
// Stage NC-0.2K.1 - Prepared Block Queue Shadow
//
// This observer is intentionally disconnected from MotionCore, the lifecycle
// ledger and every G/M handler.  It copies immutable, load-time parsed blocks
// into a fixed-capacity planning window and observes the existing Dispatch /
// Commit evidence.  It never advances the NC PC, starts a timer, changes a
// modal state or submits a motion command.
// =============================================================================

constexpr std::size_t NC_PREPARED_BLOCK_QUEUE_CAPACITY = 32U;
constexpr std::size_t NC_PREPARED_BLOCK_PLAN_BUDGET_PER_TASK = 8U;
constexpr std::size_t NC_PREPARED_BLOCK_MAX_SOURCE_BYTES = 512U;
constexpr std::size_t NC_PREPARED_BLOCK_SOURCE_BYTE_BUDGET_PER_TASK = 2048U;

using NCPreparedQueueSession = std::uint64_t;
using NCPreparedEntrySequence = std::uint64_t;

constexpr NCPreparedQueueSession NC_PREPARED_QUEUE_SESSION_INVALID = 0ULL;
constexpr NCPreparedEntrySequence NC_PREPARED_ENTRY_SEQUENCE_INVALID = 0ULL;

enum class NCPreparedBlockClass : std::uint8_t
{
    NONE = 0,
    EMPTY,
    BLOCK_SKIP,
    PURE_MODAL_COPY,
    MOTION_SHADOW,
    PROGRAM_CONTROL,
    AUXILIARY,
    INVALID
};

enum class NCPreparedBarrierKind : std::uint8_t
{
    NONE = 0,
    BLOCK_SKIP,
    SINGLE_BLOCK,
    MODAL_SNAPSHOT,
    POSITION_DERIVED,
    TABLE_WRITE,
    TIME_STOP,
    PROGRAM_FLOW,
    PROGRAM_END,
    AUX_IO,
    IMPLICIT_MOTION,
    DYNAMIC_EXPRESSION,
    PARSE_ERROR,
    SEMANTIC_ERROR,
    NATURAL_EOF
};

enum class NCPreparedQueueStopReason : std::uint8_t
{
    NONE = 0,
    INACTIVE,
    COMMITTED_BASELINE,
    BARRIER,
    CAPACITY,
    EOF_REACHED,
    FAIL_CLOSED
};

enum class NCPreparedInvalidationReason : std::uint8_t
{
    NONE = 0,
    NOT_RUNNING,
    ALARM,
    RESET,
    PROGRAM_END,
    SOURCE_REPLACED,
    EXECUTION_EPOCH_CHANGED,
    OWNER_CHANGED,
    MODE_CHANGED,
    FRAME_CHANGED,
    PANEL_SWITCH_CHANGED,
    CURSOR_DISCONTINUITY,
    IDENTITY_INVALID
};

enum NCPreparedBarrierFlag : std::uint32_t
{
    NC_PREPARED_BARRIER_FLAG_NONE = 0U,
    NC_PREPARED_BARRIER_FLAG_BLOCK_SKIP = 1U << 0U,
    NC_PREPARED_BARRIER_FLAG_SINGLE_BLOCK = 1U << 1U,
    NC_PREPARED_BARRIER_FLAG_MODAL = 1U << 2U,
    NC_PREPARED_BARRIER_FLAG_POSITION = 1U << 3U,
    NC_PREPARED_BARRIER_FLAG_TABLE_WRITE = 1U << 4U,
    NC_PREPARED_BARRIER_FLAG_TIME_STOP = 1U << 5U,
    NC_PREPARED_BARRIER_FLAG_FLOW = 1U << 6U,
    NC_PREPARED_BARRIER_FLAG_END = 1U << 7U,
    NC_PREPARED_BARRIER_FLAG_AUX = 1U << 8U,
    NC_PREPARED_BARRIER_FLAG_IMPLICIT = 1U << 9U,
    NC_PREPARED_BARRIER_FLAG_DYNAMIC = 1U << 10U,
    NC_PREPARED_BARRIER_FLAG_PARSE = 1U << 11U,
    NC_PREPARED_BARRIER_FLAG_SEMANTIC = 1U << 12U
};

struct NCPreparedPanelSwitchImage
{
    bool blockSkipEnabled = false;
    bool singleBlockEnabled = false;
    bool optionalStopEnabled = false;
};

struct NCPreparedSourceIdentity
{
    NCProgramScope scope = NCProgramScope::NONE;
    NCProgramCacheGeneration cacheGeneration =
        NC_PROGRAM_CACHE_GENERATION_INVALID;
    NCProgramFrameId frameId = NC_PROGRAM_FRAME_ID_INVALID;

    std::uint64_t executionEpoch = 0ULL;
    std::uint64_t programFlowGeneration = 0ULL;
    std::uint8_t owner = 0U;
    std::uint64_t ownerGeneration = 0ULL;

    NCPreparedPanelSwitchImage panel{};

    bool IsValid() const noexcept;
};

struct NCPreparedModalSnapshot
{
    int distanceMode = 90;
    int unitsMode = 21;
    int planeMode = 17;
    int workCoordinateCode = 54;
    NCTranslationSnapshot translation{};
    int storedStrokeMode = 23;

    int toolLengthMode = 49;
    int hCode = 0;
    int toolRadiusMode = 40;
    int dCode = 0;
    int toolCode = 0;

    bool g68Active = false;
    double g68Angle = 0.0;
    bool g168Active = false;
    int workpieceCode = 0;
    bool scalingActive = false;
    double scalingFactor = 1.0;
    std::uint8_t mirrorMask = 0U;
    bool polarActive = false;
    bool cAxisOffsetRotationEnabled = true;
    bool modalMacroActive = false;

    double g00OverrideRatio = 1.0;
    double commandedMCS[8] = { 0.0 };

    // Scalar modal fields may remain exact while a prepared motion's future
    // commanded endpoint is deliberately not calculated in K.1.
    bool commandedMCSValid = true;
    bool imageValid = true;
};

struct NCPreparedBlockClassification
{
    NCPreparedBlockClass blockClass = NCPreparedBlockClass::NONE;
    NCPreparedBarrierKind barrierKind = NCPreparedBarrierKind::NONE;
    std::uint32_t barrierFlags = NC_PREPARED_BARRIER_FLAG_NONE;

    int primaryGCode = -1;
    int firstMCode = -1;

    bool planningStopsHere = false;
    bool legacyDrainRequired = false;
    bool literalResolved = false;
    bool modalAfterValid = false;
    bool runtimeGotoControl = false;
};

struct NCPreparedBlockEntrySnapshot
{
    NCPreparedQueueSession session = NC_PREPARED_QUEUE_SESSION_INVALID;
    NCPreparedEntrySequence entrySequence =
        NC_PREPARED_ENTRY_SEQUENCE_INVALID;
    NCPreparedSourceIdentity source{};

    int sourcePC = -1;
    int sourceLineNumber = 0;

    NCBlock preparedBlock{};
    NCPreparedBlockClassification classification{};
    NCPreparedModalSnapshot modalBefore{};
    NCPreparedModalSnapshot modalAfter{};

    std::uint64_t dispatchId = 0ULL;
    NCProgramCommitSequence commitSequence =
        NC_PROGRAM_COMMIT_SEQUENCE_INVALID;

    bool dispatchObserved = false;
    bool commitObserved = false;
};

struct NCPreparedRuntimeProof
{
    bool hasDispatch = false;
    std::uint64_t dispatchId = 0ULL;
    NCProgramCommitSnapshot dispatchTarget{};
    NCProgramCommitSnapshot commitTarget{};
};

struct NCPreparedBlockQueueSnapshot
{
    std::uint64_t publicationSequence = 0ULL;
    NCPreparedQueueSession session = NC_PREPARED_QUEUE_SESSION_INVALID;
    NCPreparedSourceIdentity source{};

    int runtimeCurrentPC = -1;
    int nextPlanPC = -1;
    int tailPC = -1;
    int dispatchPC = -1;
    int commitPC = -1;
    int committedBaselinePC = -1;
    int barrierPC = -1;
    int barrierLineNumber = 0;

    std::uint32_t depth = 0U;
    std::uint32_t capacity =
        static_cast<std::uint32_t>(NC_PREPARED_BLOCK_QUEUE_CAPACITY);

    NCPreparedQueueStopReason stopReason =
        NCPreparedQueueStopReason::INACTIVE;
    NCPreparedBarrierKind barrierKind = NCPreparedBarrierKind::NONE;
    NCPreparedInvalidationReason lastInvalidationReason =
        NCPreparedInvalidationReason::NONE;

    NCPreparedModalSnapshot seedModal{};
    NCPreparedModalSnapshot tailModal{};

    bool active = false;
    bool valid = true;
    bool cursorOrderValid = true;
    bool accountingValid = true;
    bool barrierLatched = false;
    bool eofLatched = false;
    bool capacityLatched = false;
    bool committedBaselinePlanningBlocked = false;
    bool shadowOnly = true;
};

struct NCPreparedBlockQueueCounters
{
    std::uint64_t evaluations = 0ULL;
    std::uint64_t publications = 0ULL;
    std::uint64_t sessions = 0ULL;
    std::uint64_t prepared = 0ULL;
    std::uint64_t dispatchMatched = 0ULL;
    std::uint64_t commitMatched = 0ULL;
    std::uint64_t retired = 0ULL;
    std::uint64_t invalidatedEntries = 0ULL;

    std::uint64_t invalidations = 0ULL;
    std::uint64_t alarmInvalidations = 0ULL;
    std::uint64_t resetInvalidations = 0ULL;
    std::uint64_t sourceInvalidations = 0ULL;
    std::uint64_t epochInvalidations = 0ULL;
    std::uint64_t correlatedEpochAdvances = 0ULL;
    std::uint64_t ownerInvalidations = 0ULL;
    std::uint64_t frameInvalidations = 0ULL;
    std::uint64_t panelInvalidations = 0ULL;

    std::uint64_t barrierStops = 0ULL;
    std::uint64_t eofStops = 0ULL;
    std::uint64_t capacityStops = 0ULL;
    std::uint64_t overwritePrevented = 0ULL;

    std::uint64_t cursorRegressions = 0ULL;
    std::uint64_t planDiscontinuities = 0ULL;
    std::uint64_t expectedFlowCutovers = 0ULL;
    std::uint64_t dispatchMismatches = 0ULL;
    std::uint64_t commitMismatches = 0ULL;
    std::uint64_t staleRuntimeProofs = 0ULL;
    std::uint64_t identityFailures = 0ULL;

    // K.1 is observation-only.  This counter has no mutating API and must
    // remain zero in every Build and Runtime acceptance trace.
    std::uint64_t cutoverAttempts = 0ULL;
};

class NCPreparedBlockQueueShadow
{
public:
    NCPreparedBlockQueueShadow() noexcept = default;

    void ObserveInactive(
        NCPreparedInvalidationReason reason) noexcept;

    void ObserveFinalProofAndInvalidate(
        const NCPreparedRuntimeProof& proof,
        NCPreparedInvalidationReason reason) noexcept;

    bool BeginObservation(
        const NCPreparedSourceIdentity& source,
        int runtimeCurrentPC,
        int committedPC,
        const NCPreparedModalSnapshot& liveModal,
        const NCPreparedRuntimeProof& proof) noexcept;

    bool CanPrepare() const noexcept;
    int GetNextPlanPC() const noexcept;
    NCPreparedModalSnapshot GetNextModalBefore() const noexcept;

    bool PrepareParsedLine(
        int sourcePC,
        int sourceLineNumber,
        const NCParsedBlock& parsedBlock) noexcept;

    void ObserveNaturalEOF(int sourcePC) noexcept;
    void FinishObservation() noexcept;

    NCPreparedBlockQueueSnapshot GetSnapshot() const noexcept
    {
        return m_snapshot;
    }

    NCPreparedBlockQueueCounters GetCounters() const noexcept
    {
        return m_counters;
    }

    bool TryGetEntry(
        std::size_t logicalOffset,
        NCPreparedBlockEntrySnapshot& entry) const noexcept;

    static bool TryBuildLiteralBlock(
        const NCParsedBlock& parsedBlock,
        NCBlock& block) noexcept;

    static NCPreparedBlockClassification ClassifyLiteralBlock(
        const NCParsedBlock& parsedBlock,
        const NCBlock& block,
        const NCPreparedPanelSwitchImage& panel) noexcept;

    static NCPreparedModalSnapshot ReduceModalSnapshot(
        const NCPreparedModalSnapshot& before,
        const NCBlock& block,
        bool& afterValid) noexcept;

private:
    static bool SourceIdentityEqual(
        const NCPreparedSourceIdentity& left,
        const NCPreparedSourceIdentity& right) noexcept;
    static bool ProgramTargetMatchesSource(
        const NCProgramCommitSnapshot& target,
        const NCPreparedSourceIdentity& source) noexcept;
    static bool ProgramTargetMatchesEntry(
        const NCProgramCommitSnapshot& target,
        const NCPreparedBlockEntrySnapshot& entry) noexcept;

    static NCPreparedInvalidationReason ClassifyIdentityChange(
        const NCPreparedSourceIdentity& previous,
        const NCPreparedSourceIdentity& current) noexcept;

    static NCPreparedQueueSession AllocateNonZero(
        NCPreparedQueueSession& next) noexcept;
    static NCPreparedEntrySequence AllocateNonZeroEntry(
        NCPreparedEntrySequence& next) noexcept;

    void BeginNewSession(
        const NCPreparedSourceIdentity& source,
        int runtimeCurrentPC,
        int committedPC,
        const NCPreparedModalSnapshot& liveModal,
        const NCPreparedRuntimeProof& baselineProof,
        bool permitCommittedBaseline,
        bool blockPlanningAtCommittedBaseline) noexcept;
    void InvalidateActive(
        NCPreparedInvalidationReason reason) noexcept;
    void RecordInvalidationReason(
        NCPreparedInvalidationReason reason) noexcept;
    void ObserveRuntimeProof(
        const NCPreparedRuntimeProof& proof) noexcept;
    void RetireProvenHeadBeforeCutover() noexcept;
    void RetireBefore(int runtimeCurrentPC) noexcept;
    void PopHeadAsRetired() noexcept;
    void RefreshLatchesFromTail() noexcept;
    void RefreshSnapshot() noexcept;

    std::size_t PhysicalIndex(std::size_t logicalOffset) const noexcept;

    std::array<NCPreparedBlockEntrySnapshot,
        NC_PREPARED_BLOCK_QUEUE_CAPACITY> m_entries{};
    std::size_t m_head = 0U;
    std::size_t m_depth = 0U;

    NCPreparedSourceIdentity m_source{};
    NCPreparedModalSnapshot m_seedModal{};
    int m_runtimeCurrentPC = -1;
    int m_committedPC = -1;
    int m_nextPlanPC = -1;
    int m_dispatchPC = -1;
    int m_commitPC = -1;
    int m_committedBaselinePC = -1;
    std::uint64_t m_lastDispatchProofId = 0ULL;
    NCProgramCommitSequence m_lastCommitProofSequence =
        NC_PROGRAM_COMMIT_SEQUENCE_INVALID;
    std::uint64_t m_epochAuthorizationDispatchId = 0ULL;
    std::uint64_t m_epochAuthorizationFrom = 0ULL;

    NCPreparedQueueSession m_session =
        NC_PREPARED_QUEUE_SESSION_INVALID;
    NCPreparedQueueSession m_nextSession = 1ULL;
    NCPreparedEntrySequence m_nextEntrySequence = 1ULL;
    std::uint64_t m_nextPublicationSequence = 1ULL;

    bool m_active = false;
    bool m_valid = true;
    bool m_barrierLatched = false;
    bool m_eofLatched = false;
    bool m_capacityLatched = false;
    bool m_committedBaselinePlanningBlocked = false;
    NCPreparedInvalidationReason m_lastInvalidationReason =
        NCPreparedInvalidationReason::NONE;

    NCPreparedBlockQueueSnapshot m_snapshot{};
    NCPreparedBlockQueueCounters m_counters{};
};

const char* NCPreparedBlockClassToDiagnosticName(
    NCPreparedBlockClass value) noexcept;
const char* NCPreparedBarrierKindToDiagnosticName(
    NCPreparedBarrierKind value) noexcept;
const char* NCPreparedQueueStopReasonToDiagnosticName(
    NCPreparedQueueStopReason value) noexcept;
const char* NCPreparedInvalidationReasonToDiagnosticName(
    NCPreparedInvalidationReason value) noexcept;

static_assert(
    std::is_trivially_copyable<NCPreparedSourceIdentity>::value,
    "NCPreparedSourceIdentity must remain trivially copyable.");
static_assert(
    std::is_trivially_copyable<NCPreparedModalSnapshot>::value,
    "NCPreparedModalSnapshot must remain trivially copyable.");
static_assert(
    std::is_trivially_copyable<NCPreparedBlockEntrySnapshot>::value,
    "NCPreparedBlockEntrySnapshot must remain trivially copyable.");
static_assert(
    std::is_trivially_copyable<NCPreparedBlockQueueSnapshot>::value,
    "NCPreparedBlockQueueSnapshot must remain trivially copyable.");
static_assert(
    std::is_trivially_copyable<NCPreparedBlockQueueCounters>::value,
    "NCPreparedBlockQueueCounters must remain trivially copyable.");
static_assert(
    sizeof(NCPreparedBlockEntrySnapshot) <= 1680U,
    "A Prepared Entry, including two fixed cutter coordinate proofs, must remain bounded.");
static_assert(
    sizeof(NCPreparedBlockQueueShadow) <= 56320U,
    "The fixed Shadow Queue, including 67 fixed coordinate proofs, must remain at most 55 KiB.");
