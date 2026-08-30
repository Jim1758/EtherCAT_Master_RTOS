#pragma once

#include "NCPreparedBlockQueueShadow.h"

#include <cstdint>
#include <type_traits>

// =============================================================================
// Stage NC-0.2K.2 - Prepared Head Exact Equivalence Shadow
//
// This observer qualifies one exact Prepared Queue head against the unchanged
// legacy Runtime resolve / Dispatch / Program Commit path.  It never supplies
// an NCBlock to Runtime, never calls a handler and never advances a PC.
// =============================================================================

enum class NCPreparedHeadEquivalenceState : std::uint8_t
{
    IDLE = 0,
    INELIGIBLE,
    PENDING,
    MATCHED,
    MISMATCHED,
    INVALIDATED
};

enum class NCPreparedHeadEquivalenceInvalidation : std::uint8_t
{
    NONE = 0,
    QUEUE_INACTIVE,
    ALARM,
    RESET,
    PROGRAM_END,
    SOURCE_CHANGED,
    EPOCH_CHANGED,
    OWNER_CHANGED,
    FRAME_CHANGED,
    PANEL_CHANGED,
    TOKEN_REPLACED,
    RUNTIME_FAILURE
};

enum NCPreparedHeadEquivalenceMismatchFlag : std::uint32_t
{
    NC_PREPARED_EQUIVALENCE_MISMATCH_NONE = 0U,
    NC_PREPARED_EQUIVALENCE_MISMATCH_QUEUE = 1U << 0U,
    NC_PREPARED_EQUIVALENCE_MISMATCH_SOURCE = 1U << 1U,
    NC_PREPARED_EQUIVALENCE_MISMATCH_PC = 1U << 2U,
    NC_PREPARED_EQUIVALENCE_MISMATCH_LINE = 1U << 3U,
    NC_PREPARED_EQUIVALENCE_MISMATCH_BLOCK = 1U << 4U,
    NC_PREPARED_EQUIVALENCE_MISMATCH_PLAN = 1U << 5U,
    NC_PREPARED_EQUIVALENCE_MISMATCH_CLASSIFICATION = 1U << 6U,
    NC_PREPARED_EQUIVALENCE_MISMATCH_DRAIN = 1U << 7U,
    NC_PREPARED_EQUIVALENCE_MISMATCH_MODAL_BEFORE = 1U << 8U,
    NC_PREPARED_EQUIVALENCE_MISMATCH_MODAL_AFTER = 1U << 9U,
    NC_PREPARED_EQUIVALENCE_MISMATCH_LIFECYCLE = 1U << 10U,
    NC_PREPARED_EQUIVALENCE_MISMATCH_STALE = 1U << 11U,
    NC_PREPARED_EQUIVALENCE_MISMATCH_RESOLVE = 1U << 12U,
    NC_PREPARED_EQUIVALENCE_MISMATCH_UPSTREAM = 1U << 13U,
    NC_PREPARED_EQUIVALENCE_MISMATCH_LEDGER = 1U << 14U,
    NC_PREPARED_EQUIVALENCE_MISMATCH_BIND_IDENTITY = 1U << 15U
};

struct NCPreparedHeadEquivalenceSnapshot
{
    std::uint64_t publicationSequence = 0ULL;
    NCPreparedHeadEquivalenceState state =
        NCPreparedHeadEquivalenceState::IDLE;
    NCPreparedHeadEquivalenceInvalidation lastInvalidation =
        NCPreparedHeadEquivalenceInvalidation::NONE;

    NCPreparedQueueSession session = NC_PREPARED_QUEUE_SESSION_INVALID;
    NCPreparedEntrySequence entrySequence =
        NC_PREPARED_ENTRY_SEQUENCE_INVALID;
    NCProgramScope scope = NCProgramScope::NONE;
    NCProgramCacheGeneration cacheGeneration =
        NC_PROGRAM_CACHE_GENERATION_INVALID;
    NCProgramFrameId frameId = NC_PROGRAM_FRAME_ID_INVALID;
    std::uint64_t executionEpoch = 0ULL;
    std::uint64_t commitExecutionEpoch = 0ULL;
    std::uint64_t programFlowGeneration = 0ULL;
    std::uint8_t owner = 0U;
    std::uint8_t panelMask = 0U;
    std::uint64_t ownerGeneration = 0ULL;
    int sourcePC = -1;
    int sourceLineNumber = 0;

    NCPreparedBlockClass blockClass = NCPreparedBlockClass::NONE;
    NCPreparedBarrierKind barrierKind = NCPreparedBarrierKind::NONE;
    std::uint32_t mismatchFlags =
        NC_PREPARED_EQUIVALENCE_MISMATCH_NONE;

    std::uint64_t dispatchId = 0ULL;
    NCProgramCommitSequence commitSequence =
        NC_PROGRAM_COMMIT_SEQUENCE_INVALID;

    std::uint32_t preparedDepth = 0U;
    std::uint32_t sessionPeakDepth = 0U;
    std::uint32_t lifetimePeakDepth = 0U;
    NCPreparedQueueSession qualifiedSession =
        NC_PREPARED_QUEUE_SESSION_INVALID;

    bool candidate = false;
    bool pending = false;
    bool resolved = false;
    bool dispatchBound = false;
    bool commitBound = false;
    bool blockMatch = false;
    bool planMatch = false;
    bool classificationMatch = false;
    bool drainMatch = false;
    bool modalBeforeMatch = false;
    bool modalAfterMatch = false;
    bool ledgerDispatchMatch = false;
    bool ledgerCommitMatch = false;
    bool runtimeWaitCallbackActive = false;
    bool upstreamDispatchCommitMatch = false;
    bool retirementMatch = false;
    bool upstreamProofMatch = false;
    bool lifecycleMatch = false;
    bool matched = false;
    bool accountingValid = true;
    bool readinessQualified = false;

    bool shadowOnly = true;
    bool runtimeInfluence = false;
    bool cutoverApplied = false;
};

struct NCPreparedHeadEquivalenceCounters
{
    std::uint64_t observations = 0ULL;
    std::uint64_t publications = 0ULL;
    std::uint64_t candidates = 0ULL;
    std::uint64_t matched = 0ULL;
    std::uint64_t mismatched = 0ULL;
    std::uint64_t invalidated = 0ULL;
    std::uint64_t ineligible = 0ULL;
    std::uint64_t replays = 0ULL;
    std::uint64_t queueBypasses = 0ULL;
    std::uint64_t lifetimePeakDepth = 0ULL;
    std::uint64_t multiBlockObservations = 0ULL;

    std::uint64_t queueMismatches = 0ULL;
    std::uint64_t sourceMismatches = 0ULL;
    std::uint64_t pcMismatches = 0ULL;
    std::uint64_t lineMismatches = 0ULL;
    std::uint64_t blockMismatches = 0ULL;
    std::uint64_t planMismatches = 0ULL;
    std::uint64_t classificationMismatches = 0ULL;
    std::uint64_t drainMismatches = 0ULL;
    std::uint64_t modalBeforeMismatches = 0ULL;
    std::uint64_t modalAfterMismatches = 0ULL;
    std::uint64_t lifecycleMismatches = 0ULL;
    std::uint64_t staleTokens = 0ULL;
    std::uint64_t resolveMismatches = 0ULL;
    std::uint64_t upstreamMismatches = 0ULL;
    std::uint64_t ledgerMismatches = 0ULL;
    std::uint64_t bindIdentityMismatches = 0ULL;
    std::uint64_t runtimeFailures = 0ULL;
    std::uint64_t sessionTransitions = 0ULL;
    std::uint64_t qualifiedSessions = 0ULL;

    std::uint64_t wouldUse = 0ULL;
    std::uint64_t useAttempts = 0ULL;
    std::uint64_t cutoverAttempts = 0ULL;
    std::uint64_t runtimeInfluence = 0ULL;
};

class NCPreparedHeadEquivalenceShadow
{
public:
    NCPreparedHeadEquivalenceShadow() noexcept = default;

    bool ObserveResolvedHead(
        const NCPreparedBlockQueueSnapshot& queue,
        const NCPreparedBlockQueueCounters& queueCounters,
        bool hasHead,
        const NCPreparedBlockEntrySnapshot& head,
        const NCPreparedSourceIdentity& runtimeSource,
        int sourcePC,
        int sourceLineNumber,
        const NCParsedBlock& parsedBlock,
        const NCBlock& legacyBlock,
        const NCPreparedModalSnapshot& liveModalBefore,
        bool legacyDrainRequired) noexcept;

    void ObserveResolveFailure(
        const NCPreparedBlockQueueSnapshot& queue,
        const NCPreparedBlockQueueCounters& queueCounters,
        bool hasHead,
        const NCPreparedBlockEntrySnapshot& head,
        const NCPreparedSourceIdentity& runtimeSource,
        int sourcePC,
        int sourceLineNumber) noexcept;

    bool BindDispatch(
        std::uint64_t dispatchId,
        const NCProgramCommitSnapshot& dispatchTarget,
        const NCPreparedSourceIdentity& liveSource,
        bool ledgerFound,
        const NCProgramCommitSnapshot& ledgerDispatchTarget,
        int ledgerSourceLineNumber) noexcept;

    void ObserveCommit(
        std::uint64_t dispatchId,
        const NCProgramCommitSnapshot& commitTarget,
        const NCPreparedModalSnapshot& liveModalAfter,
        const NCPreparedSourceIdentity& liveSource,
        bool runtimeWaitCallbackActive,
        bool commitSucceeded,
        bool ledgerFound,
        bool ledgerProgramCommitted,
        const NCProgramCommitSnapshot& ledgerDispatchTarget,
        const NCProgramCommitSnapshot& ledgerCommitTarget,
        int ledgerSourceLineNumber) noexcept;

    void ObserveUpstreamProof(
        const NCPreparedBlockQueueSnapshot& queue,
        const NCPreparedBlockQueueCounters& queueCounters,
        const NCPreparedRuntimeProof& proof) noexcept;

    void ObserveRuntimeFailure() noexcept;

    void ObserveQueueInactive(
        NCPreparedInvalidationReason reason) noexcept;

    NCPreparedHeadEquivalenceSnapshot GetSnapshot() const noexcept
    {
        return m_snapshot;
    }

    NCPreparedHeadEquivalenceCounters GetCounters() const noexcept
    {
        return m_counters;
    }

    // K.3 calls this once after exact Ledger Dispatch binding and before any
    // handler side effect.  It repeats the semantic and execution-plan check
    // against the exact head captured by ObserveResolvedHead.  No observer
    // state or counter is changed.
    bool RevalidateBoundPreparedValue(
        const NCPreparedBlockEntrySnapshot& head,
        const NCBlock& legacyBlock) const noexcept;

private:
    static bool SourceIdentityExact(
        const NCPreparedSourceIdentity& left,
        const NCPreparedSourceIdentity& right) noexcept;
    static bool ProgramTargetMatchesEntry(
        const NCProgramCommitSnapshot& target,
        const NCPreparedBlockEntrySnapshot& entry) noexcept;
    static bool SemanticBlockEqual(
        const NCBlock& left,
        const NCBlock& right) noexcept;
    static bool StorageBlockEqual(
        const NCBlock& left,
        const NCBlock& right) noexcept;
    static bool ExecutionPlanEqual(
        const NCBlock& left,
        const NCBlock& right) noexcept;
    static bool ClassificationEqual(
        const NCPreparedBlockClassification& left,
        const NCPreparedBlockClassification& right) noexcept;
    static bool ModalEqual(
        const NCPreparedModalSnapshot& prepared,
        const NCPreparedModalSnapshot& live,
        bool compareG00Override) noexcept;
    static bool CandidateClassEligible(
        const NCPreparedBlockEntrySnapshot& entry,
        const NCPreparedSourceIdentity& runtimeSource,
        const NCPreparedBlockQueueSnapshot& queue) noexcept;

    bool SameToken(
        const NCPreparedBlockEntrySnapshot& entry) const noexcept;
    static bool UpstreamCountersHealthy(
        const NCPreparedBlockQueueCounters& counters) noexcept;
    static bool SourceIdentityStableForCommit(
        const NCPreparedSourceIdentity& prepared,
        const NCPreparedSourceIdentity& live,
        const NCPreparedBlockEntrySnapshot& entry) noexcept;
    bool PrepareCandidateToken(
        const NCPreparedBlockQueueSnapshot& queue,
        const NCPreparedBlockQueueCounters& queueCounters,
        bool hasHead,
        const NCPreparedBlockEntrySnapshot& head,
        const NCPreparedSourceIdentity& runtimeSource,
        int sourcePC,
        int sourceLineNumber) noexcept;
    void BeginSession(
        NCPreparedQueueSession session,
        NCPreparedInvalidationReason transitionReason) noexcept;
    void BeginToken(
        const NCPreparedBlockEntrySnapshot& entry,
        std::uint32_t preparedDepth) noexcept;
    void RecordMismatch(std::uint32_t flags) noexcept;
    void InvalidatePending(
        NCPreparedHeadEquivalenceInvalidation reason,
        bool stale) noexcept;
    void RefreshPublication() noexcept;

    NCPreparedBlockEntrySnapshot m_entry{};
    NCPreparedHeadEquivalenceSnapshot m_snapshot{};
    NCPreparedHeadEquivalenceCounters m_counters{};
    std::uint64_t m_nextPublicationSequence = 1ULL;
    NCPreparedQueueSession m_observedSession =
        NC_PREPARED_QUEUE_SESSION_INVALID;
    std::uint64_t m_baselineDispatchMatched = 0ULL;
    std::uint64_t m_baselineCommitMatched = 0ULL;
    std::uint64_t m_baselineRetired = 0ULL;
    std::uint64_t m_sessionCandidates = 0ULL;
    std::uint64_t m_sessionMatched = 0ULL;
    std::uint64_t m_sessionMismatched = 0ULL;
    std::uint64_t m_sessionStale = 0ULL;
    std::uint64_t m_sessionRuntimeFailures = 0ULL;
    std::uint32_t m_sessionPeakDepth = 0U;
    bool m_queueActive = false;
    bool m_upstreamHealthy = true;
    bool m_sessionQualificationLatched = false;
    bool m_sessionUpstreamFailureLatched = false;
    bool m_upstreamDispatchCommitObserved = false;
};

const char* NCPreparedHeadEquivalenceStateToDiagnosticName(
    NCPreparedHeadEquivalenceState value) noexcept;
const char* NCPreparedHeadEquivalenceInvalidationToDiagnosticName(
    NCPreparedHeadEquivalenceInvalidation value) noexcept;

static_assert(
    std::is_trivially_copyable<NCPreparedHeadEquivalenceSnapshot>::value,
    "K.2 equivalence publication must remain a fixed value snapshot.");
static_assert(
    std::is_trivially_copyable<NCPreparedHeadEquivalenceCounters>::value,
    "K.2 equivalence counters must remain a fixed value snapshot.");
static_assert(
    sizeof(NCPreparedHeadEquivalenceShadow) <= 4096U,
    "The K.2 exact-head observer must remain a compact fixed object.");
