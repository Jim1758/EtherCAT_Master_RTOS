#pragma once

#include "NCPreparedHeadEquivalenceShadow.h"

#include <cstdint>
#include <type_traits>

// =============================================================================
// Stage NC-0.2K.3 - Prepared Head Exact-Value Controlled Cutover
//
// The unchanged Runtime parser / expression resolver remains mandatory.  This
// gate may only replace that already-resolved value with the exact Prepared
// head that K.2 compared in the same NC producer-thread call.  It does not
// dispatch, commit, advance PC, call a handler or access MotionCore.
//
// Any missing or stale proof keeps legacyBlock selected.  The gate is a
// bounded value object and can be disabled at Runtime for immediate rollback.
// =============================================================================

enum class NCPreparedHeadCutoverDecision : std::uint8_t
{
    IDLE = 0,
    ARMED,
    REVOKED,
    DISABLED,
    NO_HEAD,
    QUEUE_INVALID,
    UPSTREAM_UNHEALTHY,
    SESSION_UNQUALIFIED,
    SESSION_QUARANTINED,
    DUPLICATE_TOKEN,
    TOKEN_MISMATCH,
    SOURCE_MISMATCH,
    PC_LINE_MISMATCH,
    CLASS_INELIGIBLE,
    EQUIVALENCE_NOT_READY,
    APPLIED,
    RUNTIME_FAILURE
};

enum class NCPreparedHeadCutoverRevocation : std::uint8_t
{
    NONE = 0,
    DISABLED,
    QUEUE_INACTIVE,
    QUEUE_INVALID,
    ALARM,
    RESET,
    PROGRAM_END,
    SOURCE_CHANGED,
    UPSTREAM_FAILURE,
    RUNTIME_FAILURE
};

struct NCPreparedHeadCutoverSnapshot
{
    std::uint64_t publicationSequence = 0ULL;
    NCPreparedHeadCutoverDecision decision =
        NCPreparedHeadCutoverDecision::IDLE;
    NCPreparedHeadCutoverRevocation lastRevocation =
        NCPreparedHeadCutoverRevocation::NONE;

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
    NCPreparedQueueSession qualifiedSession =
        NC_PREPARED_QUEUE_SESSION_INVALID;

    bool enabled = true;
    bool attempted = false;
    bool applied = false;
    bool legacyRetained = true;
    bool queueExact = false;
    bool tokenExact = false;
    bool sourceExact = false;
    bool pcLineExact = false;
    bool classEligible = false;
    bool equivalenceExact = false;
    bool valueRevalidated = false;
    bool preparedValueSelected = false;
    bool runtimeInfluence = false;
    bool cutoverApplied = false;
    bool sessionQuarantined = false;
    bool sessionQualified = false;
    bool qualificationArmed = false;
    bool permanentLockout = false;
    bool accountingValid = true;
};

struct NCPreparedHeadCutoverCounters
{
    std::uint64_t evaluations = 0ULL;
    std::uint64_t publications = 0ULL;
    std::uint64_t attempts = 0ULL;
    std::uint64_t applied = 0ULL;
    std::uint64_t legacyFallbacks = 0ULL;

    std::uint64_t disabled = 0ULL;
    std::uint64_t noHead = 0ULL;
    std::uint64_t queueRejected = 0ULL;
    std::uint64_t upstreamRejected = 0ULL;
    std::uint64_t unqualified = 0ULL;
    std::uint64_t quarantined = 0ULL;
    std::uint64_t duplicateRejected = 0ULL;
    std::uint64_t tokenRejected = 0ULL;
    std::uint64_t sourceRejected = 0ULL;
    std::uint64_t pcLineRejected = 0ULL;
    std::uint64_t classRejected = 0ULL;
    std::uint64_t equivalenceRejected = 0ULL;
    std::uint64_t runtimeFailures = 0ULL;
    std::uint64_t arms = 0ULL;
    std::uint64_t revocations = 0ULL;
    std::uint64_t disabledRevocations = 0ULL;
    std::uint64_t queueRevocations = 0ULL;
    std::uint64_t alarmRevocations = 0ULL;
    std::uint64_t resetRevocations = 0ULL;
    std::uint64_t programEndRevocations = 0ULL;
    std::uint64_t sourceRevocations = 0ULL;
    std::uint64_t upstreamRevocations = 0ULL;
};

// One immutable producer-thread capture is shared by the K.2 comparison and
// the K.3 last-mile admission check.  The queue head is never fetched twice.
struct NCPreparedHeadCutoverContext
{
    NCPreparedBlockQueueSnapshot queue{};
    NCPreparedBlockQueueCounters queueCounters{};
    NCPreparedBlockEntrySnapshot head{};
    NCPreparedSourceIdentity runtimeSource{};
    int sourcePC = -1;
    int sourceLineNumber = 0;
    bool legacyDrainRequired = false;
    bool hasHead = false;
};

class NCPreparedHeadCutoverGate
{
public:
    NCPreparedHeadCutoverGate() noexcept = default;

    void SetEnabled(bool enabled) noexcept
    {
        if (m_enabled == enabled)
        {
            return;
        }

        m_enabled = enabled;
        if (!enabled)
        {
            Revoke(
                NCPreparedHeadCutoverRevocation::DISABLED,
                NCPreparedHeadCutoverDecision::DISABLED);
            return;
        }

        m_qualifiedSession = NC_PREPARED_QUEUE_SESSION_INVALID;
        m_quarantinedSession = NC_PREPARED_QUEUE_SESSION_INVALID;
        m_lastObservedSession = NC_PREPARED_QUEUE_SESSION_INVALID;
        m_snapshot = NCPreparedHeadCutoverSnapshot{};
        Publish(NCPreparedHeadCutoverDecision::IDLE, false);
    }

    bool IsEnabled() const noexcept
    {
        return m_enabled;
    }

    // selectedBlock is always initialized from legacyBlock first.  A true
    // return is the only path that copies head.preparedBlock over it.
    bool SelectExactPreparedValue(
        const NCPreparedHeadCutoverContext& context,
        const NCPreparedHeadEquivalenceSnapshot& equivalence,
        const NCPreparedHeadEquivalenceCounters& equivalenceCounters,
        const NCPreparedSourceIdentity& liveSource,
        std::uint64_t dispatchId,
        bool lastMileValueExact,
        const NCBlock& legacyBlock,
        NCBlock& selectedBlock) noexcept
    {
        selectedBlock = legacyBlock;
        if (context.hasHead)
        {
            ObserveSession(context.queue.session);
        }
        ++m_counters.evaluations;
        BeginEvaluation(context.head, context.hasHead);

        if (!m_enabled)
        {
            ++m_counters.disabled;
            Publish(NCPreparedHeadCutoverDecision::DISABLED, false);
            return false;
        }

        ++m_counters.attempts;
        m_snapshot.attempted = true;

        if (!context.hasHead)
        {
            ++m_counters.noHead;
            Reject(NCPreparedHeadCutoverDecision::NO_HEAD);
            return false;
        }

        m_snapshot.queueExact = QueueExact(
            context.queue,
            context.queueCounters,
            context.hasHead);
        if (!m_snapshot.queueExact)
        {
            ++m_counters.queueRejected;
            Reject(NCPreparedHeadCutoverDecision::QUEUE_INVALID);
            return false;
        }

        if (m_quarantinedSession != NC_PREPARED_QUEUE_SESSION_INVALID &&
            context.queue.session == m_quarantinedSession)
        {
            ++m_counters.quarantined;
            m_snapshot.sessionQuarantined = true;
            Reject(NCPreparedHeadCutoverDecision::SESSION_QUARANTINED);
            return false;
        }

        if (m_permanentLockout)
        {
            ++m_counters.upstreamRejected;
            Reject(NCPreparedHeadCutoverDecision::UPSTREAM_UNHEALTHY);
            return false;
        }

        if (!UpstreamHealthy(
            context.queueCounters,
            equivalenceCounters))
        {
            ++m_counters.upstreamRejected;
            Reject(NCPreparedHeadCutoverDecision::UPSTREAM_UNHEALTHY);
            return false;
        }

        m_snapshot.tokenExact = TokenExact(
            context.queue,
            context.head,
            equivalence);
        if (!m_snapshot.tokenExact)
        {
            ++m_counters.tokenRejected;
            Reject(NCPreparedHeadCutoverDecision::TOKEN_MISMATCH);
            return false;
        }

        m_snapshot.sourceExact =
            SourceIdentityExact(
                context.queue.source,
                context.head.source) &&
            SourceIdentityExact(
                context.head.source,
                context.runtimeSource) &&
            SourceIdentityExact(context.runtimeSource, liveSource) &&
            EquivalenceSourceExact(equivalence, liveSource);
        if (!m_snapshot.sourceExact)
        {
            ++m_counters.sourceRejected;
            Reject(NCPreparedHeadCutoverDecision::SOURCE_MISMATCH);
            return false;
        }

        m_snapshot.pcLineExact =
            context.queue.runtimeCurrentPC == context.sourcePC &&
            context.head.sourcePC == context.sourcePC &&
            context.head.sourceLineNumber ==
            context.sourceLineNumber &&
            equivalence.sourcePC == context.sourcePC &&
            equivalence.sourceLineNumber == context.sourceLineNumber;
        if (!m_snapshot.pcLineExact)
        {
            ++m_counters.pcLineRejected;
            Reject(NCPreparedHeadCutoverDecision::PC_LINE_MISMATCH);
            return false;
        }

        m_snapshot.classEligible =
            ClassEligible(
                context.head,
                liveSource,
                context.legacyDrainRequired);
        if (!m_snapshot.classEligible)
        {
            ++m_counters.classRejected;
            Reject(NCPreparedHeadCutoverDecision::CLASS_INELIGIBLE);
            return false;
        }

        m_snapshot.valueRevalidated = lastMileValueExact;
        m_snapshot.equivalenceExact =
            lastMileValueExact &&
            EquivalenceReady(
                equivalence,
                context.head,
                dispatchId);
        if (!m_snapshot.equivalenceExact)
        {
            ++m_counters.equivalenceRejected;
            Reject(NCPreparedHeadCutoverDecision::EQUIVALENCE_NOT_READY);
            return false;
        }

        m_snapshot.sessionQualified =
            m_qualifiedSession == context.queue.session;
        m_snapshot.qualifiedSession = m_qualifiedSession;
        if (!m_snapshot.sessionQualified)
        {
            ++m_counters.unqualified;
            Reject(NCPreparedHeadCutoverDecision::SESSION_UNQUALIFIED);
            return false;
        }

        if (m_lastAppliedSession == context.head.session &&
            m_lastAppliedEntry == context.head.entrySequence)
        {
            ++m_counters.duplicateRejected;
            Reject(NCPreparedHeadCutoverDecision::DUPLICATE_TOKEN);
            return false;
        }

        selectedBlock = context.head.preparedBlock;
        m_lastAppliedSession = context.head.session;
        m_lastAppliedEntry = context.head.entrySequence;
        m_snapshot.dispatchId = dispatchId;
        ++m_counters.applied;
        m_snapshot.applied = true;
        m_snapshot.legacyRetained = false;
        m_snapshot.preparedValueSelected = true;
        m_snapshot.runtimeInfluence = true;
        m_snapshot.cutoverApplied = true;
        Publish(NCPreparedHeadCutoverDecision::APPLIED, true);
        return true;
    }

    // K.3 never treats a pending K.2 result as historical qualification.
    // Only K.2's completed, session-level readiness publication can arm later
    // entries, and that publication must still belong to the active K.1
    // queue.  Structural K.1/K.2 failure is a process-lifetime lockout.
    void ObserveEquivalenceState(
        const NCPreparedBlockQueueSnapshot& queue,
        const NCPreparedBlockQueueCounters& queueCounters,
        const NCPreparedHeadEquivalenceSnapshot& equivalence,
        const NCPreparedHeadEquivalenceCounters& counters) noexcept
    {
        if (!m_enabled)
        {
            return;
        }

        if (!queue.active)
        {
            Revoke(NCPreparedHeadCutoverRevocation::QUEUE_INACTIVE);
            return;
        }

        if (!QueueSessionHealthy(queue, queueCounters))
        {
            m_permanentLockout = true;
            Revoke(NCPreparedHeadCutoverRevocation::QUEUE_INVALID);
            return;
        }

        ObserveSession(queue.session);

        if (!UpstreamHealthy(queueCounters, counters) ||
            equivalence.mismatchFlags !=
            NC_PREPARED_EQUIVALENCE_MISMATCH_NONE ||
            equivalence.state ==
            NCPreparedHeadEquivalenceState::MISMATCHED)
        {
            m_permanentLockout = true;
            Revoke(NCPreparedHeadCutoverRevocation::UPSTREAM_FAILURE);
            return;
        }

        const bool exactReadiness =
            equivalence.session == queue.session &&
            equivalence.session != NC_PREPARED_QUEUE_SESSION_INVALID &&
            equivalence.publicationSequence != 0ULL &&
            equivalence.candidate &&
            equivalence.scope == NCProgramScope::MEMORY &&
            equivalence.frameId == NC_PROGRAM_FRAME_ID_INVALID &&
            equivalence.qualifiedSession == queue.session &&
            equivalence.readinessQualified &&
            equivalence.blockClass != NCPreparedBlockClass::EMPTY &&
            equivalence.state == NCPreparedHeadEquivalenceState::MATCHED &&
            equivalence.matched &&
            !equivalence.pending &&
            equivalence.resolved &&
            equivalence.dispatchBound &&
            equivalence.commitBound &&
            equivalence.ledgerDispatchMatch &&
            equivalence.ledgerCommitMatch &&
            equivalence.upstreamDispatchCommitMatch &&
            equivalence.retirementMatch &&
            equivalence.upstreamProofMatch &&
            equivalence.lifecycleMatch &&
            equivalence.accountingValid &&
            equivalence.shadowOnly &&
            !equivalence.runtimeInfluence &&
            !equivalence.cutoverApplied;
        if (!exactReadiness || m_permanentLockout)
        {
            return;
        }

        ArmSession(queue.session);
    }

    void ObserveQueueInactive(
        NCPreparedInvalidationReason reason) noexcept
    {
        Revoke(MapRevocation(reason));
        m_lastObservedSession = NC_PREPARED_QUEUE_SESSION_INVALID;
    }

    // A dispatch failure after APPLIED quarantines that exact Prepared Queue
    // session.  The legacy alarm path still owns the failure and PC behavior.
    void ObserveRuntimeFailure(std::uint64_t dispatchId) noexcept
    {
        if (!m_snapshot.applied ||
            m_snapshot.dispatchId != dispatchId ||
            dispatchId == 0ULL ||
            m_snapshot.session == NC_PREPARED_QUEUE_SESSION_INVALID ||
            m_snapshot.decision ==
            NCPreparedHeadCutoverDecision::RUNTIME_FAILURE)
        {
            return;
        }

        m_quarantinedSession = m_snapshot.session;
        m_permanentLockout = true;
        ++m_counters.runtimeFailures;
        m_snapshot.sessionQuarantined = true;
        Revoke(
            NCPreparedHeadCutoverRevocation::RUNTIME_FAILURE,
            NCPreparedHeadCutoverDecision::RUNTIME_FAILURE);
    }

    NCPreparedHeadCutoverSnapshot GetSnapshot() const noexcept
    {
        return m_snapshot;
    }

    NCPreparedHeadCutoverCounters GetCounters() const noexcept
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

    static bool QueueSessionHealthy(
        const NCPreparedBlockQueueSnapshot& queue,
        const NCPreparedBlockQueueCounters& counters) noexcept
    {
        return
            queue.active &&
            queue.valid &&
            queue.cursorOrderValid &&
            queue.accountingValid &&
            queue.shadowOnly &&
            queue.session != NC_PREPARED_QUEUE_SESSION_INVALID &&
            queue.capacity == NC_PREPARED_BLOCK_QUEUE_CAPACITY &&
            queue.depth <= queue.capacity &&
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
        const NCPreparedHeadEquivalenceCounters& equivalenceCounters) noexcept
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
            equivalenceCounters.runtimeInfluence == 0ULL;
    }

    static bool TokenExact(
        const NCPreparedBlockQueueSnapshot& queue,
        const NCPreparedBlockEntrySnapshot& head,
        const NCPreparedHeadEquivalenceSnapshot& equivalence) noexcept
    {
        return
            head.session != NC_PREPARED_QUEUE_SESSION_INVALID &&
            head.entrySequence != NC_PREPARED_ENTRY_SEQUENCE_INVALID &&
            queue.session == head.session &&
            head.session == equivalence.session &&
            head.entrySequence == equivalence.entrySequence &&
            !head.dispatchObserved &&
            !head.commitObserved;
    }

    static bool EquivalenceSourceExact(
        const NCPreparedHeadEquivalenceSnapshot& equivalence,
        const NCPreparedSourceIdentity& source) noexcept
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
            equivalence.panelMask == PanelMask(source.panel);
    }

    static bool ClassEligible(
        const NCPreparedBlockEntrySnapshot& head,
        const NCPreparedSourceIdentity& runtimeSource,
        bool legacyDrainRequired) noexcept
    {
        if (runtimeSource.scope != NCProgramScope::MEMORY ||
            runtimeSource.frameId != NC_PROGRAM_FRAME_ID_INVALID ||
            runtimeSource.panel.blockSkipEnabled ||
            runtimeSource.panel.singleBlockEnabled ||
            runtimeSource.panel.optionalStopEnabled ||
            legacyDrainRequired ||
            !head.classification.literalResolved ||
            !head.classification.modalAfterValid ||
            head.classification.planningStopsHere ||
            head.classification.legacyDrainRequired ||
            head.classification.barrierFlags !=
            NC_PREPARED_BARRIER_FLAG_NONE ||
            head.modalBefore.modalMacroActive ||
            head.preparedBlock.isEmpty ||
            head.preparedBlock.isGoto ||
            head.preparedBlock.mCount != 0 ||
            head.preparedBlock.has('T'))
        {
            return false;
        }

        if (head.classification.blockClass ==
            NCPreparedBlockClass::PURE_MODAL_COPY)
        {
            return true;
        }

        return
            head.classification.blockClass ==
            NCPreparedBlockClass::MOTION_SHADOW &&
            head.classification.primaryGCode == 0 &&
            head.preparedBlock.has('P') &&
            head.preparedBlock.val('P') == 1.0;
    }

    static bool EquivalenceReady(
        const NCPreparedHeadEquivalenceSnapshot& equivalence,
        const NCPreparedBlockEntrySnapshot& head,
        std::uint64_t dispatchId) noexcept
    {
        return
            equivalence.publicationSequence != 0ULL &&
            equivalence.state == NCPreparedHeadEquivalenceState::PENDING &&
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
            equivalence.dispatchId == dispatchId &&
            dispatchId != 0ULL &&
            !equivalence.matched &&
            equivalence.accountingValid &&
            equivalence.shadowOnly &&
            !equivalence.runtimeInfluence &&
            !equivalence.cutoverApplied &&
            equivalence.blockClass == head.classification.blockClass &&
            equivalence.barrierKind == head.classification.barrierKind;
    }

    void BeginEvaluation(
        const NCPreparedBlockEntrySnapshot& head,
        bool hasHead) noexcept
    {
        m_snapshot = NCPreparedHeadCutoverSnapshot{};
        m_snapshot.enabled = m_enabled;
        m_snapshot.qualifiedSession = m_qualifiedSession;
        m_snapshot.qualificationArmed =
            m_qualifiedSession != NC_PREPARED_QUEUE_SESSION_INVALID;
        m_snapshot.permanentLockout = m_permanentLockout;
        if (!hasHead)
        {
            return;
        }

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
        m_snapshot.sessionQualified =
            head.session == m_qualifiedSession;
    }

    void ObserveSession(NCPreparedQueueSession session) noexcept
    {
        if (session == NC_PREPARED_QUEUE_SESSION_INVALID)
        {
            return;
        }

        if (m_lastObservedSession != session)
        {
            if (m_lastObservedSession !=
                NC_PREPARED_QUEUE_SESSION_INVALID ||
                m_qualifiedSession !=
                NC_PREPARED_QUEUE_SESSION_INVALID)
            {
                Revoke(
                    NCPreparedHeadCutoverRevocation::SOURCE_CHANGED);
            }
            m_lastObservedSession = session;
            if (m_quarantinedSession != session)
            {
                m_quarantinedSession =
                    NC_PREPARED_QUEUE_SESSION_INVALID;
            }
        }
    }

    static NCPreparedHeadCutoverRevocation MapRevocation(
        NCPreparedInvalidationReason reason) noexcept
    {
        switch (reason)
        {
        case NCPreparedInvalidationReason::ALARM:
            return NCPreparedHeadCutoverRevocation::ALARM;
        case NCPreparedInvalidationReason::RESET:
            return NCPreparedHeadCutoverRevocation::RESET;
        case NCPreparedInvalidationReason::PROGRAM_END:
            return NCPreparedHeadCutoverRevocation::PROGRAM_END;
        case NCPreparedInvalidationReason::SOURCE_REPLACED:
        case NCPreparedInvalidationReason::EXECUTION_EPOCH_CHANGED:
        case NCPreparedInvalidationReason::OWNER_CHANGED:
        case NCPreparedInvalidationReason::MODE_CHANGED:
        case NCPreparedInvalidationReason::FRAME_CHANGED:
        case NCPreparedInvalidationReason::PANEL_SWITCH_CHANGED:
        case NCPreparedInvalidationReason::CURSOR_DISCONTINUITY:
        case NCPreparedInvalidationReason::IDENTITY_INVALID:
            return NCPreparedHeadCutoverRevocation::SOURCE_CHANGED;
        case NCPreparedInvalidationReason::NOT_RUNNING:
        case NCPreparedInvalidationReason::NONE:
        default:
            return NCPreparedHeadCutoverRevocation::QUEUE_INACTIVE;
        }
    }

    void ArmSession(NCPreparedQueueSession session) noexcept
    {
        if (session == NC_PREPARED_QUEUE_SESSION_INVALID ||
            !m_enabled ||
            m_permanentLockout ||
            m_qualifiedSession == session)
        {
            return;
        }

        m_qualifiedSession = session;
        ++m_counters.arms;
        m_snapshot.session = session;
        m_snapshot.qualifiedSession = session;
        m_snapshot.sessionQualified = true;
        m_snapshot.qualificationArmed = true;
        m_snapshot.attempted = false;
        Publish(NCPreparedHeadCutoverDecision::ARMED, false);
    }

    void Revoke(
        NCPreparedHeadCutoverRevocation reason,
        NCPreparedHeadCutoverDecision decision =
        NCPreparedHeadCutoverDecision::REVOKED) noexcept
    {
        const bool hasPublishedState =
            m_snapshot.publicationSequence != 0ULL &&
            m_snapshot.decision != NCPreparedHeadCutoverDecision::IDLE &&
            m_snapshot.decision != NCPreparedHeadCutoverDecision::REVOKED &&
            m_snapshot.decision != NCPreparedHeadCutoverDecision::DISABLED &&
            m_snapshot.decision !=
            NCPreparedHeadCutoverDecision::RUNTIME_FAILURE;
        const bool hadQualification =
            m_qualifiedSession != NC_PREPARED_QUEUE_SESSION_INVALID;
        if (!hasPublishedState && !hadQualification &&
            m_snapshot.decision == decision &&
            m_snapshot.lastRevocation == reason)
        {
            return;
        }

        m_qualifiedSession = NC_PREPARED_QUEUE_SESSION_INVALID;
        m_snapshot.qualifiedSession = NC_PREPARED_QUEUE_SESSION_INVALID;
        m_snapshot.sessionQualified = false;
        m_snapshot.qualificationArmed = false;
        m_snapshot.permanentLockout = m_permanentLockout;
        m_snapshot.lastRevocation = reason;
        m_snapshot.attempted = false;

        if (hasPublishedState || hadQualification)
        {
            ++m_counters.revocations;
            switch (reason)
            {
            case NCPreparedHeadCutoverRevocation::DISABLED:
                ++m_counters.disabledRevocations;
                break;
            case NCPreparedHeadCutoverRevocation::ALARM:
                ++m_counters.alarmRevocations;
                break;
            case NCPreparedHeadCutoverRevocation::RESET:
                ++m_counters.resetRevocations;
                break;
            case NCPreparedHeadCutoverRevocation::PROGRAM_END:
                ++m_counters.programEndRevocations;
                break;
            case NCPreparedHeadCutoverRevocation::SOURCE_CHANGED:
                ++m_counters.sourceRevocations;
                break;
            case NCPreparedHeadCutoverRevocation::UPSTREAM_FAILURE:
            case NCPreparedHeadCutoverRevocation::RUNTIME_FAILURE:
                ++m_counters.upstreamRevocations;
                break;
            case NCPreparedHeadCutoverRevocation::QUEUE_INACTIVE:
            case NCPreparedHeadCutoverRevocation::QUEUE_INVALID:
                ++m_counters.queueRevocations;
                break;
            case NCPreparedHeadCutoverRevocation::NONE:
            default:
                break;
            }
        }

        Publish(decision, false);
    }

    void Reject(NCPreparedHeadCutoverDecision decision) noexcept
    {
        ++m_counters.legacyFallbacks;
        Publish(decision, false);
    }

    void Publish(
        NCPreparedHeadCutoverDecision decision,
        bool preserveApplied) noexcept
    {
        m_snapshot.decision = decision;
        m_snapshot.enabled = m_enabled;
        m_snapshot.qualifiedSession = m_qualifiedSession;
        m_snapshot.sessionQualified =
            m_snapshot.session != NC_PREPARED_QUEUE_SESSION_INVALID &&
            m_snapshot.session == m_qualifiedSession;
        m_snapshot.qualificationArmed =
            m_qualifiedSession != NC_PREPARED_QUEUE_SESSION_INVALID;
        m_snapshot.permanentLockout = m_permanentLockout;
        if (!preserveApplied)
        {
            m_snapshot.applied = false;
            m_snapshot.legacyRetained = true;
            m_snapshot.preparedValueSelected = false;
            m_snapshot.runtimeInfluence = false;
            m_snapshot.cutoverApplied = false;
        }
        m_snapshot.accountingValid =
            m_counters.evaluations ==
            m_counters.disabled + m_counters.attempts &&
            m_counters.attempts ==
            m_counters.applied + m_counters.legacyFallbacks;
        m_snapshot.publicationSequence = m_nextPublicationSequence++;
        if (m_nextPublicationSequence == 0ULL)
        {
            m_nextPublicationSequence = 1ULL;
        }
        ++m_counters.publications;
    }

    bool m_enabled = true;
    NCPreparedQueueSession m_lastObservedSession =
        NC_PREPARED_QUEUE_SESSION_INVALID;
    NCPreparedQueueSession m_quarantinedSession =
        NC_PREPARED_QUEUE_SESSION_INVALID;
    NCPreparedQueueSession m_qualifiedSession =
        NC_PREPARED_QUEUE_SESSION_INVALID;
    NCPreparedQueueSession m_lastAppliedSession =
        NC_PREPARED_QUEUE_SESSION_INVALID;
    NCPreparedEntrySequence m_lastAppliedEntry =
        NC_PREPARED_ENTRY_SEQUENCE_INVALID;
    bool m_permanentLockout = false;
    std::uint64_t m_nextPublicationSequence = 1ULL;
    NCPreparedHeadCutoverSnapshot m_snapshot{};
    NCPreparedHeadCutoverCounters m_counters{};
};

static_assert(
    std::is_trivially_copyable<NCPreparedHeadCutoverSnapshot>::value,
    "K.3 cutover publication must remain a fixed value snapshot.");
static_assert(
    std::is_trivially_copyable<NCPreparedHeadCutoverCounters>::value,
    "K.3 cutover counters must remain a fixed value snapshot.");
static_assert(
    std::is_trivially_copyable<NCPreparedHeadCutoverContext>::value,
    "K.3 admission context must remain a fixed value snapshot.");

inline const char* NCPreparedHeadCutoverDecisionToDiagnosticName(
    NCPreparedHeadCutoverDecision decision) noexcept
{
    switch (decision)
    {
    case NCPreparedHeadCutoverDecision::ARMED: return "ARMED";
    case NCPreparedHeadCutoverDecision::REVOKED: return "REVOKED";
    case NCPreparedHeadCutoverDecision::DISABLED: return "DISABLED";
    case NCPreparedHeadCutoverDecision::NO_HEAD: return "NO_HEAD";
    case NCPreparedHeadCutoverDecision::QUEUE_INVALID: return "QUEUE_INVALID";
    case NCPreparedHeadCutoverDecision::UPSTREAM_UNHEALTHY: return "UPSTREAM_BAD";
    case NCPreparedHeadCutoverDecision::SESSION_UNQUALIFIED: return "SESSION_WARMUP";
    case NCPreparedHeadCutoverDecision::SESSION_QUARANTINED: return "QUARANTINED";
    case NCPreparedHeadCutoverDecision::DUPLICATE_TOKEN: return "DUPLICATE";
    case NCPreparedHeadCutoverDecision::TOKEN_MISMATCH: return "TOKEN_MISMATCH";
    case NCPreparedHeadCutoverDecision::SOURCE_MISMATCH: return "SOURCE_MISMATCH";
    case NCPreparedHeadCutoverDecision::PC_LINE_MISMATCH: return "PC_LINE_MISMATCH";
    case NCPreparedHeadCutoverDecision::CLASS_INELIGIBLE: return "CLASS_INELIGIBLE";
    case NCPreparedHeadCutoverDecision::EQUIVALENCE_NOT_READY: return "EQ_NOT_READY";
    case NCPreparedHeadCutoverDecision::APPLIED: return "APPLIED";
    case NCPreparedHeadCutoverDecision::RUNTIME_FAILURE: return "RUNTIME_FAILURE";
    case NCPreparedHeadCutoverDecision::IDLE:
    default: return "IDLE";
    }
}

inline const char* NCPreparedHeadCutoverRevocationToDiagnosticName(
    NCPreparedHeadCutoverRevocation reason) noexcept
{
    switch (reason)
    {
    case NCPreparedHeadCutoverRevocation::DISABLED: return "DISABLED";
    case NCPreparedHeadCutoverRevocation::QUEUE_INACTIVE: return "QUEUE_INACTIVE";
    case NCPreparedHeadCutoverRevocation::QUEUE_INVALID: return "QUEUE_INVALID";
    case NCPreparedHeadCutoverRevocation::ALARM: return "ALARM";
    case NCPreparedHeadCutoverRevocation::RESET: return "RESET";
    case NCPreparedHeadCutoverRevocation::PROGRAM_END: return "PROGRAM_END";
    case NCPreparedHeadCutoverRevocation::SOURCE_CHANGED: return "SOURCE_CHANGED";
    case NCPreparedHeadCutoverRevocation::UPSTREAM_FAILURE: return "UPSTREAM_FAILURE";
    case NCPreparedHeadCutoverRevocation::RUNTIME_FAILURE: return "RUNTIME_FAILURE";
    case NCPreparedHeadCutoverRevocation::NONE:
    default: return "NONE";
    }
}
