#pragma once

#include "NCPreparedHeadCutoverGate.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <type_traits>

// =============================================================================
// Stage NC-0.2K.4 - Prepared Head Pre-Resolve Admission Shadow
//
// This observer answers one deliberately narrow question before the unchanged
// NCExpressionResolver::ResolveBlock call:
//
//   "Could this exact, already-qualified Prepared head be admitted by a later
//    controlled resolver-bypass stage?"
//
// K.4 never supplies an NCBlock, skips a resolver, opens a Lifecycle block,
// calls a handler, advances PC or touches MotionCore.  A positive pre-resolve
// candidate must still pass the complete legacy Resolve -> K.2 exact compare ->
// Ledger Dispatch bind -> K.3.1 APPLIED path in the same producer-thread call.
// Only that post-resolve proof records LEGACY_CONFIRMED.  K.4 deliberately
// does not move the existing Motion drain reads ahead of ResolveBlock; an
// ordinary G00 is first a STRUCTURAL_CANDIDATE and the unchanged later drain
// point either records WAIT_LEGACY_DRAIN or continues to final confirmation.
// =============================================================================

enum class NCPreparedPreResolveAdmissionDecision : std::uint8_t
{
    IDLE = 0,
    REVOKED,
    NO_HEAD,
    QUEUE_INVALID,
    UPSTREAM_UNHEALTHY,
    SESSION_UNQUALIFIED,
    TOKEN_MISMATCH,
    SOURCE_MISMATCH,
    PC_LINE_MISMATCH,
    MODAL_MISMATCH,
    CLASS_INELIGIBLE,
    WAIT_LEGACY_DRAIN,
    CANDIDATE,
    LEGACY_CONFIRMED,
    LEGACY_RESOLVE_FAILED,
    POST_RESOLVE_MISMATCH
};

enum class NCPreparedPreResolveAdmissionRevocation : std::uint8_t
{
    NONE = 0,
    QUEUE_INACTIVE,
    ALARM,
    RESET,
    PROGRAM_END,
    SOURCE_CHANGED,
    UPSTREAM_FAILURE
};

struct NCPreparedPreResolveAdmissionSnapshot
{
    std::uint64_t publicationSequence = 0ULL;
    NCPreparedPreResolveAdmissionDecision decision =
        NCPreparedPreResolveAdmissionDecision::IDLE;
    NCPreparedPreResolveAdmissionRevocation lastRevocation =
        NCPreparedPreResolveAdmissionRevocation::NONE;

    NCPreparedQueueSession session = NC_PREPARED_QUEUE_SESSION_INVALID;
    NCPreparedEntrySequence entrySequence =
        NC_PREPARED_ENTRY_SEQUENCE_INVALID;
    NCProgramScope scope = NCProgramScope::NONE;
    NCProgramCacheGeneration cacheGeneration =
        NC_PROGRAM_CACHE_GENERATION_INVALID;
    NCProgramFrameId frameId = NC_PROGRAM_FRAME_ID_INVALID;
    std::uint64_t executionEpoch = 0ULL;
    std::uint64_t programFlowGeneration = 0ULL;
    std::uint8_t owner = 0U;
    std::uint8_t panelMask = 0U;
    std::uint64_t ownerGeneration = 0ULL;
    int sourcePC = -1;
    int sourceLineNumber = 0;
    std::uint64_t dispatchId = 0ULL;
    NCPreparedBlockClass blockClass = NCPreparedBlockClass::NONE;

    bool hasHead = false;
    bool queueExact = false;
    bool upstreamHealthy = false;
    bool sessionQualified = false;
    bool tokenExact = false;
    bool sourceExact = false;
    bool pcLineExact = false;
    bool modalExact = false;
    bool classEligible = false;
    bool legacyDrainRequired = false;
    bool legacyDrainSatisfied = false;
    bool candidate = false;
    bool legacyResolveObserved = false;
    bool equivalenceObserved = false;
    bool cutoverObserved = false;
    bool confirmed = false;
    bool permanentLockout = false;

    // These three invariants are the acceptance boundary for K.4.  They are
    // constants by design; a later stage requires a separate controlled gate.
    bool shadowOnly = true;
    bool runtimeInfluence = false;
    bool resolverBypassed = false;
    bool accountingValid = true;
};

struct NCPreparedPreResolveAdmissionCounters
{
    std::uint64_t evaluations = 0ULL;
    std::uint64_t publications = 0ULL;
    std::uint64_t candidates = 0ULL;
    std::uint64_t confirmed = 0ULL;
    std::uint64_t rejections = 0ULL;
    std::uint64_t drainWaitSamples = 0ULL;

    std::uint64_t noHead = 0ULL;
    std::uint64_t queueRejected = 0ULL;
    std::uint64_t upstreamRejected = 0ULL;
    std::uint64_t sessionRejected = 0ULL;
    std::uint64_t tokenRejected = 0ULL;
    std::uint64_t sourceRejected = 0ULL;
    std::uint64_t pcLineRejected = 0ULL;
    std::uint64_t modalRejected = 0ULL;
    std::uint64_t classRejected = 0ULL;

    std::uint64_t legacyResolveFailures = 0ULL;
    std::uint64_t postResolveMismatches = 0ULL;
    std::uint64_t candidateInvalidations = 0ULL;
    std::uint64_t confirmedRuntimeFailures = 0ULL;

    std::uint64_t revocations = 0ULL;
    std::uint64_t queueRevocations = 0ULL;
    std::uint64_t alarmRevocations = 0ULL;
    std::uint64_t resetRevocations = 0ULL;
    std::uint64_t programEndRevocations = 0ULL;
    std::uint64_t sourceRevocations = 0ULL;
    std::uint64_t upstreamRevocations = 0ULL;

    // No public mutating API exists for these counters in K.4.
    std::uint64_t runtimeInfluence = 0ULL;
    std::uint64_t resolverBypasses = 0ULL;
};

class NCPreparedHeadPreResolveAdmissionShadow
{
public:
    NCPreparedHeadPreResolveAdmissionShadow() noexcept = default;

    // Returns true only for a diagnostic candidate.  The caller must still
    // execute the complete legacy resolver and all existing K.2/K.3 gates.
    bool ObserveBeforeResolve(
        const NCPreparedHeadCutoverContext& context,
        const NCPreparedHeadEquivalenceCounters& equivalenceCounters,
        const NCPreparedHeadCutoverSnapshot& cutoverSnapshot,
        const NCPreparedHeadCutoverCounters& cutoverCounters) noexcept
    {
        ObserveActiveQueueSession(context.queue);
        if (m_candidatePending)
        {
            // A candidate must close in the same producer-thread invocation.
            // Re-entry with a still-open token is evidence loss, never a new
            // permission to bypass the legacy resolver.
            ClosePendingAsPostResolveMismatch();
        }

        ++m_counters.evaluations;
        BeginEvaluation(context);

        if (!context.hasHead)
        {
            ++m_counters.noHead;
            Reject(NCPreparedPreResolveAdmissionDecision::NO_HEAD);
            return false;
        }

        m_lastObservedSession = context.queue.session;

        m_snapshot.queueExact = QueueExact(
            context.queue,
            context.queueCounters,
            context.hasHead);
        if (!m_snapshot.queueExact)
        {
            ++m_counters.queueRejected;
            Reject(NCPreparedPreResolveAdmissionDecision::QUEUE_INVALID);
            return false;
        }

        m_snapshot.upstreamHealthy = UpstreamHealthy(
            context.queueCounters,
            equivalenceCounters,
            cutoverSnapshot,
            cutoverCounters) &&
            !m_permanentLockout;
        if (!m_snapshot.upstreamHealthy)
        {
            ++m_counters.upstreamRejected;
            Reject(
                NCPreparedPreResolveAdmissionDecision::UPSTREAM_UNHEALTHY);
            return false;
        }

        m_snapshot.sessionQualified =
            cutoverSnapshot.qualifiedSession == context.queue.session &&
            cutoverSnapshot.qualificationArmed &&
            !cutoverSnapshot.permanentLockout;
        if (!m_snapshot.sessionQualified)
        {
            ++m_counters.sessionRejected;
            Reject(
                NCPreparedPreResolveAdmissionDecision::SESSION_UNQUALIFIED);
            return false;
        }

        m_snapshot.tokenExact = TokenExact(
            context.queue,
            context.head);
        if (!m_snapshot.tokenExact)
        {
            ++m_counters.tokenRejected;
            Reject(NCPreparedPreResolveAdmissionDecision::TOKEN_MISMATCH);
            return false;
        }

        m_snapshot.sourceExact =
            SourceIdentityExact(context.queue.source, context.head.source) &&
            SourceIdentityExact(
                context.head.source,
                context.runtimeSource);
        if (!m_snapshot.sourceExact)
        {
            ++m_counters.sourceRejected;
            Reject(NCPreparedPreResolveAdmissionDecision::SOURCE_MISMATCH);
            return false;
        }

        m_snapshot.pcLineExact =
            context.queue.runtimeCurrentPC == context.sourcePC &&
            context.head.sourcePC == context.sourcePC &&
            context.head.sourceLineNumber == context.sourceLineNumber;
        if (!m_snapshot.pcLineExact)
        {
            ++m_counters.pcLineRejected;
            Reject(NCPreparedPreResolveAdmissionDecision::PC_LINE_MISMATCH);
            return false;
        }

        m_snapshot.modalExact = ModalExact(
            context.head.modalBefore,
            context.runtimeModalBefore,
            false);
        if (!m_snapshot.modalExact)
        {
            ++m_counters.modalRejected;
            Reject(NCPreparedPreResolveAdmissionDecision::MODAL_MISMATCH);
            return false;
        }

        m_snapshot.classEligible = ClassEligibleBeforeResolve(context);
        if (!m_snapshot.classEligible)
        {
            ++m_counters.classRejected;
            Reject(NCPreparedPreResolveAdmissionDecision::CLASS_INELIGIBLE);
            return false;
        }

        m_candidatePending = true;
        m_pendingSession = context.head.session;
        m_pendingEntrySequence = context.head.entrySequence;
        m_pendingSource = context.head.source;
        m_pendingPC = context.sourcePC;
        m_pendingLine = context.sourceLineNumber;
        m_pendingLegacyDrainRequired = context.legacyDrainRequired;

        ++m_counters.candidates;
        m_snapshot.candidate = true;
        Publish(NCPreparedPreResolveAdmissionDecision::CANDIDATE);
        return true;
    }

    // Called only from the unchanged post-resolve legacy drain branch.  A
    // successful K.2 current-head comparison is required even for a wait
    // sample; otherwise the structural prediction closes as a mismatch.
    void ObserveLegacyDrainWait(
        const NCPreparedHeadCutoverContext& context,
        const NCPreparedHeadEquivalenceSnapshot& equivalence,
        bool equivalencePending) noexcept
    {
        if (!m_candidatePending)
        {
            return;
        }

        m_snapshot.legacyResolveObserved = true;
        m_snapshot.legacyDrainRequired = context.legacyDrainRequired;
        m_snapshot.legacyDrainSatisfied = context.legacyDrainSatisfied;

        const bool contextExact =
            context.hasHead &&
            context.head.session == m_pendingSession &&
            context.head.entrySequence == m_pendingEntrySequence &&
            context.sourcePC == m_pendingPC &&
            context.sourceLineNumber == m_pendingLine &&
            SourceIdentityExact(context.head.source, m_pendingSource) &&
            SourceIdentityExact(context.runtimeSource, m_pendingSource) &&
            context.legacyDrainRequired ==
            m_pendingLegacyDrainRequired &&
            context.legacyDrainRequired &&
            !context.legacyDrainSatisfied;

        m_snapshot.equivalenceObserved =
            contextExact &&
            equivalencePending &&
            equivalence.state == NCPreparedHeadEquivalenceState::PENDING &&
            equivalence.session == m_pendingSession &&
            equivalence.entrySequence == m_pendingEntrySequence &&
            EquivalenceIdentityExact(
                equivalence,
                context.head,
                m_pendingSource,
                m_pendingPC,
                m_pendingLine) &&
            equivalence.candidate &&
            equivalence.pending &&
            equivalence.resolved &&
            !equivalence.dispatchBound &&
            !equivalence.commitBound &&
            equivalence.mismatchFlags ==
            NC_PREPARED_EQUIVALENCE_MISMATCH_NONE &&
            equivalence.blockMatch &&
            equivalence.planMatch &&
            equivalence.classificationMatch &&
            equivalence.drainMatch &&
            equivalence.modalBeforeMatch &&
            equivalence.accountingValid &&
            equivalence.shadowOnly &&
            !equivalence.runtimeInfluence &&
            !equivalence.cutoverApplied;

        m_candidatePending = false;
        if (m_snapshot.equivalenceObserved)
        {
            ++m_counters.drainWaitSamples;
            Publish(
                NCPreparedPreResolveAdmissionDecision::WAIT_LEGACY_DRAIN);
            return;
        }

        ++m_counters.postResolveMismatches;
        m_permanentLockout = true;
        m_snapshot.permanentLockout = true;
        Publish(
            NCPreparedPreResolveAdmissionDecision::POST_RESOLVE_MISMATCH);
    }

    void ObserveResolvedOutcome(
        const NCPreparedHeadCutoverContext& context,
        const NCPreparedHeadEquivalenceSnapshot& equivalence,
        const NCPreparedHeadCutoverSnapshot& cutover,
        std::uint64_t dispatchId,
        bool equivalencePending,
        bool dispatchBound,
        bool lastMileValueExact,
        bool cutoverApplied) noexcept
    {
        if (!m_candidatePending)
        {
            return;
        }

        m_snapshot.legacyResolveObserved = true;
        m_snapshot.dispatchId = dispatchId;
        m_snapshot.legacyDrainRequired = context.legacyDrainRequired;
        m_snapshot.legacyDrainSatisfied = context.legacyDrainSatisfied;

        const bool contextExact =
            context.hasHead &&
            context.head.session == m_pendingSession &&
            context.head.entrySequence == m_pendingEntrySequence &&
            context.sourcePC == m_pendingPC &&
            context.sourceLineNumber == m_pendingLine &&
            SourceIdentityExact(context.head.source, m_pendingSource) &&
            SourceIdentityExact(context.runtimeSource, m_pendingSource) &&
            context.legacyDrainRequired ==
            m_pendingLegacyDrainRequired &&
            (!context.legacyDrainRequired ||
                context.legacyDrainSatisfied);

        m_snapshot.equivalenceObserved =
            contextExact &&
            equivalencePending &&
            dispatchBound &&
            lastMileValueExact &&
            dispatchId != 0ULL &&
            equivalence.state == NCPreparedHeadEquivalenceState::PENDING &&
            equivalence.session == m_pendingSession &&
            equivalence.entrySequence == m_pendingEntrySequence &&
            equivalence.dispatchId == dispatchId &&
            EquivalenceIdentityExact(
                equivalence,
                context.head,
                m_pendingSource,
                m_pendingPC,
                m_pendingLine) &&
            equivalence.candidate &&
            equivalence.pending &&
            equivalence.resolved &&
            equivalence.dispatchBound &&
            !equivalence.commitBound &&
            equivalence.mismatchFlags ==
            NC_PREPARED_EQUIVALENCE_MISMATCH_NONE &&
            equivalence.blockMatch &&
            equivalence.planMatch &&
            equivalence.classificationMatch &&
            equivalence.drainMatch &&
            equivalence.modalBeforeMatch &&
            equivalence.ledgerDispatchMatch &&
            equivalence.accountingValid &&
            equivalence.shadowOnly &&
            !equivalence.runtimeInfluence &&
            !equivalence.cutoverApplied;

        m_snapshot.cutoverObserved =
            contextExact &&
            cutoverApplied &&
            cutover.decision == NCPreparedHeadCutoverDecision::APPLIED &&
            cutover.session == m_pendingSession &&
            cutover.entrySequence == m_pendingEntrySequence &&
            cutover.dispatchId == dispatchId &&
            CutoverIdentityExact(
                cutover,
                context.head,
                m_pendingSource,
                m_pendingPC,
                m_pendingLine) &&
            cutover.applied &&
            !cutover.legacyRetained &&
            cutover.queueExact &&
            cutover.tokenExact &&
            cutover.sourceExact &&
            cutover.pcLineExact &&
            cutover.classEligible &&
            cutover.equivalenceExact &&
            cutover.valueRevalidated &&
            cutover.preparedValueSelected &&
            cutover.runtimeInfluence &&
            cutover.cutoverApplied &&
            cutover.accountingValid &&
            !cutover.permanentLockout;

        m_candidatePending = false;
        if (m_snapshot.equivalenceObserved &&
            m_snapshot.cutoverObserved)
        {
            ++m_counters.confirmed;
            m_lastConfirmedDispatchId = dispatchId;
            m_snapshot.confirmed = true;
            Publish(
                NCPreparedPreResolveAdmissionDecision::LEGACY_CONFIRMED);
            return;
        }

        ++m_counters.postResolveMismatches;
        m_permanentLockout = true;
        m_snapshot.permanentLockout = true;
        m_snapshot.confirmed = false;
        Publish(
            NCPreparedPreResolveAdmissionDecision::POST_RESOLVE_MISMATCH);
    }

    void ObserveResolveFailure(
        const NCPreparedSourceIdentity& runtimeSource,
        int sourcePC,
        int sourceLineNumber) noexcept
    {
        if (!m_candidatePending)
        {
            return;
        }

        m_snapshot.legacyResolveObserved = true;
        m_snapshot.sourceExact =
            SourceIdentityExact(runtimeSource, m_pendingSource);
        m_snapshot.pcLineExact =
            sourcePC == m_pendingPC &&
            sourceLineNumber == m_pendingLine;
        m_candidatePending = false;
        ++m_counters.legacyResolveFailures;
        m_permanentLockout = true;
        m_snapshot.permanentLockout = true;
        Publish(
            NCPreparedPreResolveAdmissionDecision::LEGACY_RESOLVE_FAILED);
    }

    // Called from the existing same-Dispatch Runtime failure branch after
    // K.2 and K.3 have closed their own state.  A previously confirmed K.4
    // prediction remains a valid historical confirmation; the later handler
    // failure is a separate upstream revocation, never a post-resolve mismatch.
    void ObserveConfirmedRuntimeFailure(std::uint64_t dispatchId) noexcept
    {
        if (dispatchId == 0ULL ||
            dispatchId != m_lastConfirmedDispatchId ||
            dispatchId == m_lastRuntimeFailureDispatchId)
        {
            return;
        }

        m_lastRuntimeFailureDispatchId = dispatchId;
        ++m_counters.confirmedRuntimeFailures;
        ++m_counters.revocations;
        ++m_counters.upstreamRevocations;
        m_snapshot.lastRevocation =
            NCPreparedPreResolveAdmissionRevocation::UPSTREAM_FAILURE;
        m_permanentLockout = true;
        m_snapshot.permanentLockout = true;
        m_snapshot.candidate = false;
        // Keep confirmed=true: the later ExecuteBlock failure does not rewrite
        // the already-closed resolver/K.2/K.3 oracle result.
        Publish(NCPreparedPreResolveAdmissionDecision::REVOKED);
        m_lastObservedSession = NC_PREPARED_QUEUE_SESSION_INVALID;
    }

    // Observed from K.1/K.2 upstream proof so a source generation change is
    // seen even when the new head is Assignment/GOTO/Block Skip and therefore
    // never enters the ordinary ResolveBlock branch.
    void ObserveActiveQueueSession(
        const NCPreparedBlockQueueSnapshot& queue) noexcept
    {
        if (!queue.active ||
            !queue.valid ||
            queue.session == NC_PREPARED_QUEUE_SESSION_INVALID)
        {
            return;
        }

        if (m_lastObservedSession != NC_PREPARED_QUEUE_SESSION_INVALID &&
            queue.session != m_lastObservedSession)
        {
            if (m_candidatePending)
            {
                m_candidatePending = false;
                ++m_counters.candidateInvalidations;
            }
            ++m_counters.revocations;
            ++m_counters.sourceRevocations;
            m_snapshot.lastRevocation =
                NCPreparedPreResolveAdmissionRevocation::SOURCE_CHANGED;
            m_snapshot.candidate = false;
            m_snapshot.confirmed = false;
            Publish(NCPreparedPreResolveAdmissionDecision::REVOKED);
        }

        m_lastObservedSession = queue.session;
    }

    void ObserveQueueInactive(
        NCPreparedInvalidationReason reason) noexcept
    {
        if (m_lastObservedSession == NC_PREPARED_QUEUE_SESSION_INVALID &&
            !m_candidatePending)
        {
            return;
        }

        if (m_candidatePending)
        {
            m_candidatePending = false;
            ++m_counters.candidateInvalidations;
        }

        const NCPreparedPreResolveAdmissionRevocation revocation =
            MapRevocation(reason);
        ++m_counters.revocations;
        RecordRevocation(revocation);
        m_snapshot.lastRevocation = revocation;
        m_snapshot.candidate = false;
        m_snapshot.confirmed = false;
        Publish(NCPreparedPreResolveAdmissionDecision::REVOKED);
        m_lastObservedSession = NC_PREPARED_QUEUE_SESSION_INVALID;
    }

    NCPreparedPreResolveAdmissionSnapshot GetSnapshot() const noexcept
    {
        return m_snapshot;
    }

    NCPreparedPreResolveAdmissionCounters GetCounters() const noexcept
    {
        return m_counters;
    }

private:
    static std::uint8_t PanelMask(
        const NCPreparedPanelSwitchImage& panel) noexcept
    {
        return static_cast<std::uint8_t>(
            (panel.blockSkipEnabled ? 1U : 0U) |
            (panel.singleBlockEnabled ? 2U : 0U) |
            (panel.optionalStopEnabled ? 4U : 0U));
    }

    static bool SourceIdentityExact(
        const NCPreparedSourceIdentity& left,
        const NCPreparedSourceIdentity& right) noexcept
    {
        return
            left.scope == right.scope &&
            left.cacheGeneration == right.cacheGeneration &&
            left.frameId == right.frameId &&
            left.executionEpoch == right.executionEpoch &&
            left.programFlowGeneration == right.programFlowGeneration &&
            left.owner == right.owner &&
            left.ownerGeneration == right.ownerGeneration &&
            left.panel.blockSkipEnabled == right.panel.blockSkipEnabled &&
            left.panel.singleBlockEnabled ==
            right.panel.singleBlockEnabled &&
            left.panel.optionalStopEnabled ==
            right.panel.optionalStopEnabled;
    }

    static bool QueueExact(
        const NCPreparedBlockQueueSnapshot& queue,
        const NCPreparedBlockQueueCounters& counters,
        bool hasHead) noexcept
    {
        return
            queue.active &&
            queue.valid &&
            queue.cursorOrderValid &&
            queue.accountingValid &&
            queue.shadowOnly &&
            !queue.committedBaselinePlanningBlocked &&
            queue.session != NC_PREPARED_QUEUE_SESSION_INVALID &&
            queue.capacity == NC_PREPARED_BLOCK_QUEUE_CAPACITY &&
            queue.depth > 0U &&
            queue.depth <= queue.capacity &&
            hasHead &&
            counters.sessions == queue.session &&
            counters.commitMatched <= counters.dispatchMatched &&
            counters.dispatchMatched <= counters.prepared &&
            counters.prepared ==
            counters.retired +
            counters.invalidatedEntries +
            static_cast<std::uint64_t>(queue.depth);
    }

    static bool UpstreamHealthy(
        const NCPreparedBlockQueueCounters& queueCounters,
        const NCPreparedHeadEquivalenceCounters& equivalenceCounters,
        const NCPreparedHeadCutoverSnapshot& cutoverSnapshot,
        const NCPreparedHeadCutoverCounters& cutoverCounters) noexcept
    {
        return
            queueCounters.cursorRegressions == 0ULL &&
            queueCounters.planDiscontinuities == 0ULL &&
            queueCounters.dispatchMismatches == 0ULL &&
            queueCounters.commitMismatches == 0ULL &&
            queueCounters.staleRuntimeProofs == 0ULL &&
            queueCounters.identityFailures == 0ULL &&
            queueCounters.cutoverAttempts == 0ULL &&
            equivalenceCounters.mismatched == 0ULL &&
            equivalenceCounters.staleTokens == 0ULL &&
            equivalenceCounters.resolveMismatches == 0ULL &&
            equivalenceCounters.upstreamMismatches == 0ULL &&
            equivalenceCounters.runtimeFailures == 0ULL &&
            equivalenceCounters.useAttempts == 0ULL &&
            equivalenceCounters.cutoverAttempts == 0ULL &&
            equivalenceCounters.runtimeInfluence == 0ULL &&
            cutoverCounters.runtimeFailures == 0ULL &&
            cutoverCounters.noHead == 0ULL &&
            cutoverCounters.queueRejected == 0ULL &&
            cutoverCounters.upstreamRejected == 0ULL &&
            cutoverCounters.quarantined == 0ULL &&
            cutoverCounters.duplicateRejected == 0ULL &&
            cutoverCounters.tokenRejected == 0ULL &&
            cutoverCounters.sourceRejected == 0ULL &&
            cutoverCounters.pcLineRejected == 0ULL &&
            cutoverCounters.equivalenceRejected == 0ULL &&
            cutoverSnapshot.enabled &&
            cutoverSnapshot.accountingValid &&
            !cutoverSnapshot.permanentLockout;
    }

    static bool TokenExact(
        const NCPreparedBlockQueueSnapshot& queue,
        const NCPreparedBlockEntrySnapshot& head) noexcept
    {
        return
            head.session != NC_PREPARED_QUEUE_SESSION_INVALID &&
            head.entrySequence != NC_PREPARED_ENTRY_SEQUENCE_INVALID &&
            queue.session == head.session &&
            !head.dispatchObserved &&
            !head.commitObserved;
    }

    static bool EquivalenceIdentityExact(
        const NCPreparedHeadEquivalenceSnapshot& equivalence,
        const NCPreparedBlockEntrySnapshot& head,
        const NCPreparedSourceIdentity& source,
        int sourcePC,
        int sourceLineNumber) noexcept
    {
        return
            equivalence.scope == source.scope &&
            equivalence.cacheGeneration == source.cacheGeneration &&
            equivalence.frameId == source.frameId &&
            equivalence.executionEpoch == source.executionEpoch &&
            equivalence.programFlowGeneration ==
            source.programFlowGeneration &&
            equivalence.owner == source.owner &&
            equivalence.ownerGeneration == source.ownerGeneration &&
            equivalence.panelMask == PanelMask(source.panel) &&
            equivalence.sourcePC == sourcePC &&
            equivalence.sourceLineNumber == sourceLineNumber &&
            equivalence.blockClass == head.classification.blockClass &&
            equivalence.barrierKind == head.classification.barrierKind;
    }

    static bool CutoverIdentityExact(
        const NCPreparedHeadCutoverSnapshot& cutover,
        const NCPreparedBlockEntrySnapshot& head,
        const NCPreparedSourceIdentity& source,
        int sourcePC,
        int sourceLineNumber) noexcept
    {
        return
            cutover.enabled &&
            cutover.scope == source.scope &&
            cutover.cacheGeneration == source.cacheGeneration &&
            cutover.frameId == source.frameId &&
            cutover.executionEpoch == source.executionEpoch &&
            cutover.programFlowGeneration ==
            source.programFlowGeneration &&
            cutover.owner == source.owner &&
            cutover.ownerGeneration == source.ownerGeneration &&
            cutover.panelMask == PanelMask(source.panel) &&
            cutover.sourcePC == sourcePC &&
            cutover.sourceLineNumber == sourceLineNumber &&
            cutover.blockClass == head.classification.blockClass &&
            cutover.qualifiedSession == head.session &&
            cutover.sessionQualified &&
            cutover.qualificationArmed;
    }

    static bool HasAnyAxisAddress(const NCBlock& block) noexcept
    {
        return
            block.has('X') || block.has('Y') || block.has('Z') ||
            block.has('A') || block.has('B') || block.has('C') ||
            block.has('U') || block.has('V') || block.has('W');
    }

    static bool DoubleBitsEqual(double left, double right) noexcept
    {
        std::uint64_t leftBits = 0ULL;
        std::uint64_t rightBits = 0ULL;
        static_assert(
            sizeof(leftBits) == sizeof(left),
            "K.4 assumes an IEEE-754-sized double value.");
        std::memcpy(&leftBits, &left, sizeof(leftBits));
        std::memcpy(&rightBits, &right, sizeof(rightBits));
        return leftBits == rightBits;
    }

    static bool ModalExact(
        const NCPreparedModalSnapshot& prepared,
        const NCPreparedModalSnapshot& live,
        bool compareG00Override) noexcept
    {
        if (!prepared.imageValid ||
            !live.imageValid ||
            prepared.distanceMode != live.distanceMode ||
            prepared.unitsMode != live.unitsMode ||
            prepared.planeMode != live.planeMode ||
            prepared.workCoordinateCode != live.workCoordinateCode ||
            prepared.storedStrokeMode != live.storedStrokeMode ||
            prepared.toolLengthMode != live.toolLengthMode ||
            prepared.hCode != live.hCode ||
            prepared.toolRadiusMode != live.toolRadiusMode ||
            prepared.dCode != live.dCode ||
            prepared.toolCode != live.toolCode ||
            prepared.g68Active != live.g68Active ||
            !DoubleBitsEqual(prepared.g68Angle, live.g68Angle) ||
            prepared.g168Active != live.g168Active ||
            prepared.workpieceCode != live.workpieceCode ||
            prepared.scalingActive != live.scalingActive ||
            !DoubleBitsEqual(
                prepared.scalingFactor,
                live.scalingFactor) ||
            prepared.mirrorMask != live.mirrorMask ||
            prepared.polarActive != live.polarActive ||
            prepared.cAxisOffsetRotationEnabled !=
            live.cAxisOffsetRotationEnabled ||
            prepared.modalMacroActive != live.modalMacroActive ||
            (compareG00Override &&
                !DoubleBitsEqual(
                    prepared.g00OverrideRatio,
                    live.g00OverrideRatio)))
        {
            return false;
        }

        if (prepared.commandedMCSValid)
        {
            if (!live.commandedMCSValid)
            {
                return false;
            }
            for (int axis = 0; axis < 8; ++axis)
            {
                if (!DoubleBitsEqual(
                    prepared.commandedMCS[axis],
                    live.commandedMCS[axis]))
                {
                    return false;
                }
            }
        }
        return true;
    }

    static bool BlockStorageValid(const NCBlock& block) noexcept
    {
        if (block.gCount <= 0 ||
            block.gCount > NC_MAX_G_CODES_PER_BLOCK ||
            !block.hasG ||
            block.mCount < 0 ||
            block.mCount > NC_MAX_M_CODES_PER_BLOCK)
        {
            return false;
        }

        for (int index = 0; index < 26; ++index)
        {
            if (block.hasParam[index] &&
                !std::isfinite(block.param[index]))
            {
                return false;
            }
        }
        return true;
    }

    static bool ContainsStoredGCode(
        const NCBlock& block,
        int requiredCode) noexcept
    {
        if (block.gCount < 0 ||
            block.gCount > NC_MAX_G_CODES_PER_BLOCK)
        {
            return false;
        }

        if (block.hasG && block.gCode == requiredCode)
        {
            return true;
        }

        for (int index = 0; index < block.gCount; ++index)
        {
            if (block.gCodes[index] == requiredCode)
            {
                return true;
            }
        }
        return false;
    }

    static bool ClassEligibleBeforeResolve(
        const NCPreparedHeadCutoverContext& context) noexcept
    {
        const NCPreparedBlockEntrySnapshot& head = context.head;
        const NCPreparedSourceIdentity& source = context.runtimeSource;

        if (source.scope != NCProgramScope::MEMORY ||
            source.frameId != NC_PROGRAM_FRAME_ID_INVALID ||
            source.panel.blockSkipEnabled ||
            source.panel.singleBlockEnabled ||
            source.panel.optionalStopEnabled ||
            !context.capturedBeforeResolve ||
            !context.runtimeModalBeforeValid ||
            head.classification.legacyDrainRequired !=
            context.legacyDrainRequired ||
            !head.classification.literalResolved ||
            !head.classification.modalAfterValid ||
            head.classification.planningStopsHere ||
            head.classification.barrierFlags !=
            NC_PREPARED_BARRIER_FLAG_NONE ||
            head.modalBefore.modalMacroActive ||
            head.preparedBlock.isEmpty ||
            head.preparedBlock.isGoto ||
            head.preparedBlock.mCount != 0 ||
            head.preparedBlock.has('T') ||
            !BlockStorageValid(head.preparedBlock))
        {
            return false;
        }

        if (head.classification.blockClass ==
            NCPreparedBlockClass::PURE_MODAL_COPY)
        {
            return !context.legacyDrainRequired;
        }

        if (head.classification.blockClass !=
            NCPreparedBlockClass::MOTION_SHADOW ||
            head.classification.primaryGCode != 0 ||
            !ContainsStoredGCode(head.preparedBlock, 0))
        {
            return false;
        }

        if (head.preparedBlock.has('P'))
        {
            return
                head.preparedBlock.val('P') == 1.0 &&
                !context.legacyDrainRequired;
        }

        return
            HasAnyAxisAddress(head.preparedBlock) &&
            context.legacyDrainRequired;
    }

    static NCPreparedPreResolveAdmissionRevocation MapRevocation(
        NCPreparedInvalidationReason reason) noexcept
    {
        switch (reason)
        {
        case NCPreparedInvalidationReason::ALARM:
            return NCPreparedPreResolveAdmissionRevocation::ALARM;
        case NCPreparedInvalidationReason::RESET:
            return NCPreparedPreResolveAdmissionRevocation::RESET;
        case NCPreparedInvalidationReason::PROGRAM_END:
            return NCPreparedPreResolveAdmissionRevocation::PROGRAM_END;
        case NCPreparedInvalidationReason::SOURCE_REPLACED:
        case NCPreparedInvalidationReason::MODE_CHANGED:
        case NCPreparedInvalidationReason::CURSOR_DISCONTINUITY:
        case NCPreparedInvalidationReason::EXECUTION_EPOCH_CHANGED:
        case NCPreparedInvalidationReason::OWNER_CHANGED:
        case NCPreparedInvalidationReason::FRAME_CHANGED:
        case NCPreparedInvalidationReason::PANEL_SWITCH_CHANGED:
        case NCPreparedInvalidationReason::IDENTITY_INVALID:
            return NCPreparedPreResolveAdmissionRevocation::SOURCE_CHANGED;
        case NCPreparedInvalidationReason::NOT_RUNNING:
        case NCPreparedInvalidationReason::NONE:
        default:
            return NCPreparedPreResolveAdmissionRevocation::QUEUE_INACTIVE;
        }
    }

    void BeginEvaluation(
        const NCPreparedHeadCutoverContext& context) noexcept
    {
        m_snapshot = NCPreparedPreResolveAdmissionSnapshot{};
        m_snapshot.permanentLockout = m_permanentLockout;
        m_snapshot.hasHead = context.hasHead;
        m_snapshot.legacyDrainRequired = context.legacyDrainRequired;
        m_snapshot.legacyDrainSatisfied = context.legacyDrainSatisfied;
        if (!context.hasHead)
        {
            return;
        }

        const NCPreparedBlockEntrySnapshot& head = context.head;
        m_snapshot.session = head.session;
        m_snapshot.entrySequence = head.entrySequence;
        m_snapshot.scope = head.source.scope;
        m_snapshot.cacheGeneration = head.source.cacheGeneration;
        m_snapshot.frameId = head.source.frameId;
        m_snapshot.executionEpoch = head.source.executionEpoch;
        m_snapshot.programFlowGeneration =
            head.source.programFlowGeneration;
        m_snapshot.owner = head.source.owner;
        m_snapshot.panelMask = PanelMask(head.source.panel);
        m_snapshot.ownerGeneration = head.source.ownerGeneration;
        m_snapshot.sourcePC = head.sourcePC;
        m_snapshot.sourceLineNumber = head.sourceLineNumber;
        m_snapshot.blockClass = head.classification.blockClass;
    }

    void Reject(NCPreparedPreResolveAdmissionDecision decision) noexcept
    {
        ++m_counters.rejections;
        Publish(decision);
    }

    void ClosePendingAsPostResolveMismatch() noexcept
    {
        m_candidatePending = false;
        ++m_counters.postResolveMismatches;
        m_permanentLockout = true;
        m_snapshot.permanentLockout = true;
        m_snapshot.confirmed = false;
        Publish(
            NCPreparedPreResolveAdmissionDecision::POST_RESOLVE_MISMATCH);
    }

    void RecordRevocation(
        NCPreparedPreResolveAdmissionRevocation revocation) noexcept
    {
        switch (revocation)
        {
        case NCPreparedPreResolveAdmissionRevocation::ALARM:
            ++m_counters.alarmRevocations;
            break;
        case NCPreparedPreResolveAdmissionRevocation::RESET:
            ++m_counters.resetRevocations;
            break;
        case NCPreparedPreResolveAdmissionRevocation::PROGRAM_END:
            ++m_counters.programEndRevocations;
            break;
        case NCPreparedPreResolveAdmissionRevocation::SOURCE_CHANGED:
            ++m_counters.sourceRevocations;
            break;
        case NCPreparedPreResolveAdmissionRevocation::UPSTREAM_FAILURE:
            ++m_counters.upstreamRevocations;
            break;
        case NCPreparedPreResolveAdmissionRevocation::QUEUE_INACTIVE:
            ++m_counters.queueRevocations;
            break;
        case NCPreparedPreResolveAdmissionRevocation::NONE:
        default:
            break;
        }
    }

    bool AccountingValid() const noexcept
    {
        const std::uint64_t evaluatedOutcomes =
            m_counters.candidates +
            m_counters.rejections;
        const std::uint64_t candidateOutcomes =
            m_counters.confirmed +
            m_counters.drainWaitSamples +
            m_counters.legacyResolveFailures +
            m_counters.postResolveMismatches +
            m_counters.candidateInvalidations;
        const std::uint64_t rejectionReasons =
            m_counters.noHead +
            m_counters.queueRejected +
            m_counters.upstreamRejected +
            m_counters.sessionRejected +
            m_counters.tokenRejected +
            m_counters.sourceRejected +
            m_counters.pcLineRejected +
            m_counters.modalRejected +
            m_counters.classRejected;
        const std::uint64_t revocationReasons =
            m_counters.queueRevocations +
            m_counters.alarmRevocations +
            m_counters.resetRevocations +
            m_counters.programEndRevocations +
            m_counters.sourceRevocations +
            m_counters.upstreamRevocations;
        const std::uint64_t expectedPublications =
            m_counters.evaluations +
            m_counters.confirmed +
            m_counters.drainWaitSamples +
            m_counters.legacyResolveFailures +
            m_counters.postResolveMismatches +
            m_counters.revocations;
        return
            m_counters.evaluations == evaluatedOutcomes &&
            m_counters.rejections == rejectionReasons &&
            m_counters.candidates ==
            candidateOutcomes + (m_candidatePending ? 1ULL : 0ULL) &&
            m_counters.revocations == revocationReasons &&
            m_counters.publications == expectedPublications &&
            m_counters.runtimeInfluence == 0ULL &&
            m_counters.resolverBypasses == 0ULL;
    }

    void Publish(
        NCPreparedPreResolveAdmissionDecision decision) noexcept
    {
        m_snapshot.publicationSequence = m_nextPublicationSequence++;
        if (m_snapshot.publicationSequence == 0ULL)
        {
            m_snapshot.publicationSequence = m_nextPublicationSequence++;
        }
        m_snapshot.decision = decision;
        m_snapshot.shadowOnly = true;
        m_snapshot.runtimeInfluence = false;
        m_snapshot.resolverBypassed = false;
        ++m_counters.publications;
        m_snapshot.accountingValid = AccountingValid();
    }

    NCPreparedPreResolveAdmissionSnapshot m_snapshot{};
    NCPreparedPreResolveAdmissionCounters m_counters{};
    std::uint64_t m_nextPublicationSequence = 1ULL;
    NCPreparedQueueSession m_lastObservedSession =
        NC_PREPARED_QUEUE_SESSION_INVALID;

    bool m_candidatePending = false;
    NCPreparedQueueSession m_pendingSession =
        NC_PREPARED_QUEUE_SESSION_INVALID;
    NCPreparedEntrySequence m_pendingEntrySequence =
        NC_PREPARED_ENTRY_SEQUENCE_INVALID;
    NCPreparedSourceIdentity m_pendingSource{};
    int m_pendingPC = -1;
    int m_pendingLine = 0;
    bool m_pendingLegacyDrainRequired = false;
    std::uint64_t m_lastConfirmedDispatchId = 0ULL;
    std::uint64_t m_lastRuntimeFailureDispatchId = 0ULL;
    bool m_permanentLockout = false;
};

static_assert(
    std::is_trivially_copyable<
    NCPreparedPreResolveAdmissionSnapshot>::value,
    "K.4 snapshots must remain bounded POD diagnostics.");

static_assert(
    std::is_trivially_copyable<
    NCPreparedPreResolveAdmissionCounters>::value,
    "K.4 counters must remain bounded POD diagnostics.");

inline const char* NCPreparedPreResolveAdmissionDecisionToDiagnosticName(
    NCPreparedPreResolveAdmissionDecision decision) noexcept
{
    switch (decision)
    {
    case NCPreparedPreResolveAdmissionDecision::REVOKED: return "REVOKED";
    case NCPreparedPreResolveAdmissionDecision::NO_HEAD: return "NO_HEAD";
    case NCPreparedPreResolveAdmissionDecision::QUEUE_INVALID: return "QUEUE_INVALID";
    case NCPreparedPreResolveAdmissionDecision::UPSTREAM_UNHEALTHY: return "UPSTREAM_UNHEALTHY";
    case NCPreparedPreResolveAdmissionDecision::SESSION_UNQUALIFIED: return "SESSION_UNQUALIFIED";
    case NCPreparedPreResolveAdmissionDecision::TOKEN_MISMATCH: return "TOKEN_MISMATCH";
    case NCPreparedPreResolveAdmissionDecision::SOURCE_MISMATCH: return "SOURCE_MISMATCH";
    case NCPreparedPreResolveAdmissionDecision::PC_LINE_MISMATCH: return "PC_LINE_MISMATCH";
    case NCPreparedPreResolveAdmissionDecision::MODAL_MISMATCH: return "MODAL_MISMATCH";
    case NCPreparedPreResolveAdmissionDecision::CLASS_INELIGIBLE: return "CLASS_INELIGIBLE";
    case NCPreparedPreResolveAdmissionDecision::WAIT_LEGACY_DRAIN: return "WAIT_DRAIN";
    case NCPreparedPreResolveAdmissionDecision::CANDIDATE: return "STRUCTURAL_CANDIDATE";
    case NCPreparedPreResolveAdmissionDecision::LEGACY_CONFIRMED: return "LEGACY_CONFIRMED";
    case NCPreparedPreResolveAdmissionDecision::LEGACY_RESOLVE_FAILED: return "RESOLVE_FAILED";
    case NCPreparedPreResolveAdmissionDecision::POST_RESOLVE_MISMATCH: return "POST_MISMATCH";
    case NCPreparedPreResolveAdmissionDecision::IDLE:
    default: return "IDLE";
    }
}

inline const char* NCPreparedPreResolveAdmissionRevocationToDiagnosticName(
    NCPreparedPreResolveAdmissionRevocation revocation) noexcept
{
    switch (revocation)
    {
    case NCPreparedPreResolveAdmissionRevocation::QUEUE_INACTIVE: return "QUEUE_INACTIVE";
    case NCPreparedPreResolveAdmissionRevocation::ALARM: return "ALARM";
    case NCPreparedPreResolveAdmissionRevocation::RESET: return "RESET";
    case NCPreparedPreResolveAdmissionRevocation::PROGRAM_END: return "PROGRAM_END";
    case NCPreparedPreResolveAdmissionRevocation::SOURCE_CHANGED: return "SOURCE_CHANGED";
    case NCPreparedPreResolveAdmissionRevocation::UPSTREAM_FAILURE: return "UPSTREAM_FAILURE";
    case NCPreparedPreResolveAdmissionRevocation::NONE:
    default: return "NONE";
    }
}
