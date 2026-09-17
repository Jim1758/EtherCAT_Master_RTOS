#pragma once

#include "NCOrdinaryG00BufferedExactStopAdmissionShadow.h"

#include <cstddef>
#include <cstdint>
#include <type_traits>

// =============================================================================
// Stage NC-0.2K.7.2 - Ordinary G00 Continuous Registry Health Fence
//
// The gate is intentionally narrow:
//   * MEMORY main-program, literal ordinary no-P G00 only.
//   * The same K.4.2 lane must already be qualified in this Queue session.
//   * The same session must first complete one legacy K.6.3-tracked G00.
//   * At most two K.6.3 entries may be active at once.
//   * Motion uses BUFFERED transport with command-local EXACT_STOP, the
//     existing execution Epoch, no per-block completion callback, and the
//     existing capture-bound transactional queue-tail endpoint.
//
// Path Core is not involved.  Every selection is fenced by the complete
// K.6.3 in-flight Registry integrity proof, including after the one-session
// legacy warmup has already succeeded.  Any mismatch after selection is
// fail-closed; ordinary candidates that are not yet qualified, or whose
// Registry health fence closes, retain the complete legacy
// drain/ABORTING/callback path.
// =============================================================================

constexpr std::uint32_t NC_ORDINARY_G00_READ_AHEAD_ACTIVE_LIMIT = 2U;

enum class NCOrdinaryG00ReadAheadSelectResult : std::uint8_t
{
    NOT_SELECTED = 0,
    WAIT_CAPACITY,
    SELECTED
};

enum class NCOrdinaryG00ReadAheadDecision : std::uint8_t
{
    IDLE = 0,
    DISABLED,
    INELIGIBLE,
    WAIT_LEGACY_WARMUP,
    WAIT_CAPACITY,
    SELECTED,
    DISPATCH_BOUND,
    MOTION_AUTHORIZED,
    COMMITTED,
    REVOKED,
    RUNTIME_FAILURE,
    PROOF_MISMATCH,
    REGISTRY_UNHEALTHY_FALLBACK
};

struct NCOrdinaryG00ReadAheadSnapshot
{
    std::uint64_t publicationSequence = 0ULL;
    NCOrdinaryG00ReadAheadDecision decision =
        NCOrdinaryG00ReadAheadDecision::IDLE;
    NCPreparedInvalidationReason lastInvalidation =
        NCPreparedInvalidationReason::NONE;

    NCPreparedQueueSession session = NC_PREPARED_QUEUE_SESSION_INVALID;
    NCPreparedQueueSession warmupSession =
        NC_PREPARED_QUEUE_SESSION_INVALID;
    NCPreparedEntrySequence entrySequence =
        NC_PREPARED_ENTRY_SEQUENCE_INVALID;
    NCProgramScope scope = NCProgramScope::NONE;
    NCProgramCacheGeneration cacheGeneration =
        NC_PROGRAM_CACHE_GENERATION_INVALID;
    NCProgramFrameId frameId = NC_PROGRAM_FRAME_ID_INVALID;
    std::uint64_t sourceExecutionEpoch = 0ULL;
    std::uint64_t programFlowGeneration = 0ULL;
    std::uint8_t owner = 0U;
    std::uint8_t panelMask = 0U;
    std::uint64_t ownerGeneration = 0ULL;
    int sourcePC = -1;
    int sourceLineNumber = 0;
    NCBlockDispatchId dispatchId = NC_BLOCK_DISPATCH_ID_INVALID;
    NCProgramCommitSequence commitSequence =
        NC_PROGRAM_COMMIT_SEQUENCE_INVALID;
    MotionExecutionIdentity identity{};
    MotionQueueTailTransactionSequence queueTailTransactionSequence =
        MOTION_QUEUE_TAIL_TRANSACTION_SEQUENCE_INVALID;

    std::uint32_t activeLimit =
        NC_ORDINARY_G00_READ_AHEAD_ACTIVE_LIMIT;
    std::uint32_t activeEntriesAtSelection = 0U;
    std::uint64_t queueDepthAtSelection = 0ULL;
    int chainSourcePC = -1;

    bool enabled = true;
    bool candidateExact = false;
    bool envelopeExact = false;
    bool registryReady = false;
    bool registryHealthy = false;
    bool registryHealthFenced = false;
    bool registryHealthLockout = false;
    bool legacyWarmupProven = false;
    bool contiguousChain = false;
    bool capacityAvailable = false;
    bool selected = false;
    bool dispatchBound = false;
    bool motionAuthorized = false;
    bool commitBound = false;
    bool registryBound = false;
    bool bufferedTransport = false;
    bool exactStop = false;
    bool stableExecutionEpoch = false;
    bool noPerBlockCallback = false;
    bool transactionalEndpoint = false;
    bool pending = false;
    bool permanentLockout = false;
    bool runtimeInfluence = false;
    bool resolverBypassed = false;
    bool motionWrite = false;
    bool accountingValid = true;
};

struct NCOrdinaryG00ReadAheadCounters
{
    std::uint64_t evaluations = 0ULL;
    std::uint64_t publications = 0ULL;
    std::uint64_t ineligible = 0ULL;
    std::uint64_t legacyWarmupWaits = 0ULL;
    std::uint64_t registryHealthFences = 0ULL;
    std::uint64_t registryHealthLockouts = 0ULL;
    std::uint64_t discontinuityFallbacks = 0ULL;
    std::uint64_t capacityWaits = 0ULL;
    std::uint64_t selected = 0ULL;
    std::uint64_t dispatchBound = 0ULL;
    std::uint64_t motionAuthorized = 0ULL;
    std::uint64_t committed = 0ULL;
    std::uint64_t registryBound = 0ULL;
    std::uint64_t revokedPending = 0ULL;
    std::uint64_t runtimeFailures = 0ULL;
    std::uint64_t proofMismatches = 0ULL;
    std::uint64_t sessionWarmups = 0ULL;
    std::uint64_t maxActiveObserved = 0ULL;
    std::uint64_t runtimeInfluence = 0ULL;
    std::uint64_t resolverBypasses = 0ULL;
    std::uint64_t motionWrites = 0ULL;
};

class NCOrdinaryG00ReadAheadCutoverGate
{
public:
    NCOrdinaryG00ReadAheadCutoverGate() noexcept = default;

    void SetEnabled(bool enabled) noexcept
    {
        if (m_enabled == enabled)
        {
            return;
        }
        m_enabled = enabled;
        if (!enabled && m_pending)
        {
            ++m_counters.revokedPending;
            ClearPending();
        }
        Publish(enabled
            ? NCOrdinaryG00ReadAheadDecision::IDLE
            : NCOrdinaryG00ReadAheadDecision::DISABLED);
    }

    bool IsEnabled() const noexcept
    {
        return m_enabled;
    }

    NCOrdinaryG00ReadAheadSelectResult TrySelect(
        const NCPreparedHeadCutoverContext& context,
        bool upstreamCandidateExact,
        const NCOrdinaryG00AdmissionSnapshot& admission,
        const NCOrdinaryG00AdmissionCounters& admissionCounters,
        const NCOrdinaryG00InflightRegistrySnapshot& registry,
        const NCOrdinaryG00InflightRegistryCounters& registryCounters,
        std::uint64_t queueDepth,
        NCBlock& selectedBlock) noexcept
    {
        selectedBlock = NCBlock{};
        ++m_counters.evaluations;
        ObserveActiveSession(context.queue.session);
        BeginEvaluation(context, queueDepth, registry.activeEntries);

        if (!m_enabled)
        {
            ++m_counters.ineligible;
            Publish(NCOrdinaryG00ReadAheadDecision::DISABLED);
            return NCOrdinaryG00ReadAheadSelectResult::NOT_SELECTED;
        }

        const bool candidateExact =
            upstreamCandidateExact &&
            !m_permanentLockout &&
            !m_pending &&
            ContextIdentityExact(context) &&
            SimpleG90Envelope(context);
        m_snapshot.candidateExact = upstreamCandidateExact;
        m_snapshot.envelopeExact = SimpleG90Envelope(context);
        if (!candidateExact)
        {
            ++m_counters.ineligible;
            Publish(NCOrdinaryG00ReadAheadDecision::INELIGIBLE);
            return NCOrdinaryG00ReadAheadSelectResult::NOT_SELECTED;
        }

        const bool registryIntegrityHealthy =
            RegistryIntegrityHealthy(registry, registryCounters);
        const bool registryHealthy =
            RegistryHealthy(
                context.queue.session,
                registry,
                registryCounters);
        m_snapshot.registryReady = registry.ready;
        m_snapshot.registryHealthy = registryHealthy;

        // Before the first legacy K.6.3 terminal, Registry ready=false is an
        // expected warmup state.  Every structural fault is still fenced
        // immediately.  After warmup, even loss of ready alone is a hard
        // admission fence: K.7 may not add Runtime work without its terminal
        // safety oracle.
        if (!registryIntegrityHealthy ||
            (m_warmupSession == context.queue.session &&
                !registryHealthy))
        {
            FenceRegistryHealth();
            return NCOrdinaryG00ReadAheadSelectResult::NOT_SELECTED;
        }

        const bool freshLegacyAnchorExact = LegacyWarmupExact(
            context,
            admission,
            admissionCounters,
            registry,
            registryCounters);
        if (m_warmupSession != context.queue.session)
        {
            if (!freshLegacyAnchorExact)
            {
                ++m_counters.legacyWarmupWaits;
                Publish(
                    NCOrdinaryG00ReadAheadDecision::WAIT_LEGACY_WARMUP);
                return NCOrdinaryG00ReadAheadSelectResult::NOT_SELECTED;
            }
            m_warmupSession = context.queue.session;
            m_chainSourcePC = admission.sourcePC;
            m_chainAnchorCommitSequence = admission.commitSequence;
            ++m_counters.sessionWarmups;
        }
        else if (freshLegacyAnchorExact &&
            admission.commitSequence != m_chainAnchorCommitSequence)
        {
            // A non-G00 or other fallback broke the controlled chain.  A new
            // independently completed legacy G00 may establish the next
            // conservative one-line anchor in the same Queue session.
            m_chainSourcePC = admission.sourcePC;
            m_chainAnchorCommitSequence = admission.commitSequence;
            ++m_counters.sessionWarmups;
        }

        m_snapshot.warmupSession = m_warmupSession;
        m_snapshot.legacyWarmupProven = true;
        m_snapshot.chainSourcePC = m_chainSourcePC;
        const bool contiguousChain =
            m_chainSourcePC >= 0 &&
            context.sourcePC == m_chainSourcePC + 1;
        m_snapshot.contiguousChain = contiguousChain;
        if (!contiguousChain)
        {
            ++m_counters.discontinuityFallbacks;
            ++m_counters.ineligible;
            Publish(NCOrdinaryG00ReadAheadDecision::INELIGIBLE);
            return NCOrdinaryG00ReadAheadSelectResult::NOT_SELECTED;
        }
        const bool capacityAvailable =
            registry.activeEntries <
            NC_ORDINARY_G00_READ_AHEAD_ACTIVE_LIMIT&&
            registry.activeEntries == registryCounters.activeEntries;
        m_snapshot.capacityAvailable = capacityAvailable;
        if (!capacityAvailable)
        {
            ++m_counters.capacityWaits;
            Publish(NCOrdinaryG00ReadAheadDecision::WAIT_CAPACITY);
            return NCOrdinaryG00ReadAheadSelectResult::WAIT_CAPACITY;
        }

        m_pendingEntry = context.head;
        m_pending = true;
        m_pendingDispatchBound = false;
        m_pendingMotionAuthorized = false;
        ++m_counters.selected;
        ++m_counters.runtimeInfluence;
        ++m_counters.resolverBypasses;
        m_snapshot.selected = true;
        m_snapshot.pending = true;
        m_snapshot.runtimeInfluence = true;
        m_snapshot.resolverBypassed = true;
        selectedBlock = context.head.preparedBlock;
        Publish(NCOrdinaryG00ReadAheadDecision::SELECTED);
        return NCOrdinaryG00ReadAheadSelectResult::SELECTED;
    }

    bool BindDispatch(
        const NCPreparedHeadCutoverContext& context,
        NCBlockDispatchId dispatchId,
        const NCProgramCommitSnapshot& dispatchTarget,
        const NCPreparedSourceIdentity& liveSource,
        bool ledgerFound,
        const NCProgramCommitSnapshot& ledgerDispatchTarget,
        int ledgerSourceLineNumber) noexcept
    {
        const bool exact =
            m_pending &&
            !m_pendingDispatchBound &&
            dispatchId != NC_BLOCK_DISPATCH_ID_INVALID &&
            PendingContextExact(context) &&
            SourceExact(m_pendingEntry.source, liveSource) &&
            ProgramTargetMatchesEntry(dispatchTarget, m_pendingEntry) &&
            ledgerFound &&
            ProgramTargetMatchesEntry(
                ledgerDispatchTarget,
                m_pendingEntry) &&
            ledgerSourceLineNumber == m_pendingEntry.sourceLineNumber;
        if (!exact)
        {
            FailProof(false);
            return false;
        }

        m_pendingDispatchId = dispatchId;
        m_pendingDispatchTarget = dispatchTarget;
        m_pendingDispatchBound = true;
        ++m_counters.dispatchBound;
        m_snapshot.dispatchId = dispatchId;
        m_snapshot.dispatchBound = true;
        Publish(NCOrdinaryG00ReadAheadDecision::DISPATCH_BOUND);
        return true;
    }

    bool ConsumeMotionAuthorization(
        NCBlockDispatchId dispatchId) noexcept
    {
        const bool exact =
            m_pending &&
            m_pendingDispatchBound &&
            !m_pendingMotionAuthorized &&
            dispatchId != NC_BLOCK_DISPATCH_ID_INVALID &&
            dispatchId == m_pendingDispatchId;
        if (!exact)
        {
            return false;
        }

        m_pendingMotionAuthorized = true;
        ++m_counters.motionAuthorized;
        m_snapshot.motionAuthorized = true;
        Publish(NCOrdinaryG00ReadAheadDecision::MOTION_AUTHORIZED);
        return true;
    }

    bool ObserveCommit(
        const NCPreparedHeadCutoverContext& context,
        const NCOrdinaryG00InflightRegistrationEvidence& evidence,
        const NCOrdinaryG00InflightRegistrationProof& registryProof) noexcept
    {
        const MotionExecutionIdentity& identity =
            evidence.submissionIdentity;
        const bool identityExact =
            identity.IsAssigned() &&
            static_cast<std::uint64_t>(identity.epoch) ==
            context.runtimeSource.executionEpoch &&
            identity.epoch == evidence.queueTailReceipt.identity.epoch &&
            identity.segmentId ==
            evidence.queueTailReceipt.identity.segmentId &&
            identity.sourceBlockId ==
            evidence.queueTailReceipt.identity.sourceBlockId &&
            identity.source == evidence.queueTailReceipt.identity.source;
        const bool proofExact =
            registryProof.registered &&
            registryProof.exact &&
            registryProof.active &&
            registryProof.bounded &&
            registryProof.readAheadCutover &&
            !registryProof.shadowOnly &&
            registryProof.runtimeInfluence &&
            !registryProof.motionWrite &&
            registryProof.accountingValid &&
            registryProof.session == context.head.session &&
            registryProof.entrySequence == context.head.entrySequence &&
            registryProof.dispatchId == evidence.dispatchId &&
            registryProof.commitSequence == evidence.commitSequence &&
            SameIdentity(registryProof.identity, identity);
        const bool exact =
            m_pending &&
            m_pendingDispatchBound &&
            m_pendingMotionAuthorized &&
            PendingContextExact(context) &&
            evidence.dispatchId == m_pendingDispatchId &&
            evidence.commitSequence !=
            NC_PROGRAM_COMMIT_SEQUENCE_INVALID &&
            ProgramTargetMatchesEntry(
                m_pendingDispatchTarget,
                m_pendingEntry) &&
            evidence.currentExecutionEpoch ==
            context.runtimeSource.executionEpoch &&
            evidence.segmentExecutionEpoch ==
            context.runtimeSource.executionEpoch &&
            evidence.submissionCount == 1U &&
            !evidence.captureOverflow &&
            evidence.producerAccepted &&
            evidence.immediateRejectNone &&
            evidence.commandPathMode ==
            MotionCommandPathMode::EXACT_STOP &&
            !evidence.waitCallbackActive &&
            evidence.commitSucceeded &&
            identityExact &&
            evidence.queueTailReceipt.IsCommitted() &&
            evidence.queueTailReceipt.captureBound &&
            evidence.queueTailReceipt.transactionSequence !=
            MOTION_QUEUE_TAIL_TRANSACTION_SEQUENCE_INVALID &&
            evidence.queueTailReceipt.axisMask != 0U &&
            evidence.ledgerFound &&
            evidence.ledgerProgramCommitted &&
            !evidence.ledgerCaptureOverflow &&
            evidence.ledgerDispatchId == evidence.dispatchId &&
            evidence.ledgerCommitSequence == evidence.commitSequence &&
            evidence.ledgerMotionSegmentCount == 1U &&
            evidence.ledgerProducerAccepted &&
            evidence.ledgerImmediateRejectReason ==
            MotionRejectReason::NONE &&
            SameIdentity(evidence.ledgerIdentity, identity) &&
            proofExact;
        if (!exact)
        {
            FailProof(false);
            return false;
        }

        ++m_counters.committed;
        ++m_counters.registryBound;
        m_snapshot.commitSequence = evidence.commitSequence;
        m_snapshot.identity = identity;
        m_snapshot.queueTailTransactionSequence =
            evidence.queueTailReceipt.transactionSequence;
        m_snapshot.commitBound = true;
        m_snapshot.registryBound = true;
        m_snapshot.bufferedTransport = true;
        m_snapshot.exactStop = true;
        m_snapshot.stableExecutionEpoch = true;
        m_snapshot.noPerBlockCallback = true;
        m_snapshot.transactionalEndpoint = true;
        m_snapshot.pending = false;
        m_chainSourcePC = context.sourcePC;
        ClearPending();
        Publish(NCOrdinaryG00ReadAheadDecision::COMMITTED);
        return true;
    }

    void ObserveRuntimeFailure(NCBlockDispatchId dispatchId) noexcept
    {
        if (!m_pending ||
            (dispatchId != NC_BLOCK_DISPATCH_ID_INVALID &&
                m_pendingDispatchBound &&
                dispatchId != m_pendingDispatchId))
        {
            return;
        }
        FailProof(true);
    }

    void ObserveQueueInactive(NCPreparedInvalidationReason reason) noexcept
    {
        if (m_inactivePublished && !m_pending &&
            m_activeSession == NC_PREPARED_QUEUE_SESSION_INVALID)
        {
            return;
        }
        if (m_pending)
        {
            ++m_counters.revokedPending;
        }
        ClearPending();
        m_activeSession = NC_PREPARED_QUEUE_SESSION_INVALID;
        m_warmupSession = NC_PREPARED_QUEUE_SESSION_INVALID;
        m_chainSourcePC = -1;
        m_chainAnchorCommitSequence =
            NC_PROGRAM_COMMIT_SEQUENCE_INVALID;
        m_snapshot.lastInvalidation = reason;
        m_snapshot.pending = false;
        m_inactivePublished = true;
        Publish(NCOrdinaryG00ReadAheadDecision::REVOKED);
    }

    NCOrdinaryG00ReadAheadSnapshot GetSnapshot() const noexcept
    {
        return m_snapshot;
    }

    NCOrdinaryG00ReadAheadCounters GetCounters() const noexcept
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

    static bool SourceExact(
        const NCPreparedSourceIdentity& lhs,
        const NCPreparedSourceIdentity& rhs) noexcept
    {
        return
            lhs.scope == rhs.scope &&
            lhs.cacheGeneration == rhs.cacheGeneration &&
            lhs.frameId == rhs.frameId &&
            lhs.executionEpoch == rhs.executionEpoch &&
            lhs.programFlowGeneration == rhs.programFlowGeneration &&
            lhs.owner == rhs.owner &&
            lhs.ownerGeneration == rhs.ownerGeneration &&
            lhs.panel.blockSkipEnabled == rhs.panel.blockSkipEnabled &&
            lhs.panel.singleBlockEnabled == rhs.panel.singleBlockEnabled &&
            lhs.panel.optionalStopEnabled == rhs.panel.optionalStopEnabled;
    }

    static bool SameIdentity(
        const MotionExecutionIdentity& lhs,
        const MotionExecutionIdentity& rhs) noexcept
    {
        return
            lhs.epoch == rhs.epoch &&
            lhs.segmentId == rhs.segmentId &&
            lhs.sourceBlockId == rhs.sourceBlockId &&
            lhs.source == rhs.source;
    }

    static bool ProgramTargetMatchesEntry(
        const NCProgramCommitSnapshot& target,
        const NCPreparedBlockEntrySnapshot& entry) noexcept
    {
        return
            target.scope == entry.source.scope &&
            target.cacheGeneration == entry.source.cacheGeneration &&
            target.frameId == entry.source.frameId &&
            target.sourcePC == entry.sourcePC;
    }

    static bool ContextIdentityExact(
        const NCPreparedHeadCutoverContext& context) noexcept
    {
        return
            context.hasHead &&
            context.capturedBeforeResolve &&
            context.runtimeModalBeforeValid &&
            context.queue.active &&
            context.queue.valid &&
            context.queue.accountingValid &&
            context.queue.session != NC_PREPARED_QUEUE_SESSION_INVALID &&
            context.queue.session == context.head.session &&
            context.queue.runtimeCurrentPC == context.sourcePC &&
            context.head.entrySequence !=
            NC_PREPARED_ENTRY_SEQUENCE_INVALID &&
            context.head.sourcePC == context.sourcePC &&
            context.head.sourceLineNumber == context.sourceLineNumber &&
            SourceExact(context.queue.source, context.runtimeSource) &&
            SourceExact(context.head.source, context.runtimeSource) &&
            context.runtimeSource.scope == NCProgramScope::MEMORY &&
            context.runtimeSource.frameId == NC_PROGRAM_FRAME_ID_INVALID &&
            context.runtimeSource.owner ==
            static_cast<std::uint8_t>(MotionOwner::AUTO) &&
            context.runtimeSource.ownerGeneration != 0ULL &&
            context.runtimeSource.executionEpoch != 0ULL &&
            context.runtimeSource.executionEpoch <=
            static_cast<std::uint64_t>(UINT32_MAX) &&
            !context.runtimeSource.panel.blockSkipEnabled &&
            !context.runtimeSource.panel.singleBlockEnabled &&
            !context.runtimeSource.panel.optionalStopEnabled &&
            context.legacyDrainRequired &&
            context.head.classification.legacyDrainRequired &&
            context.head.classification.blockClass ==
            NCPreparedBlockClass::MOTION_SHADOW &&
            context.head.classification.primaryGCode == 0 &&
            context.head.classification.literalResolved &&
            context.head.classification.modalAfterValid &&
            !context.head.classification.planningStopsHere &&
            context.head.classification.barrierKind ==
            NCPreparedBarrierKind::NONE &&
            context.head.classification.barrierFlags ==
            NC_PREPARED_BARRIER_FLAG_NONE;
    }

    static bool SimpleG90Envelope(
        const NCPreparedHeadCutoverContext& context) noexcept
    {
        const NCPreparedModalSnapshot& modal = context.head.modalAfter;
        return
            modal.imageValid &&
            modal.distanceMode == 90 &&
            modal.unitsMode == 21 &&
            modal.planeMode == 17 &&
            IsNCTranslationSourceAllowed(modal.workCoordinateCode, modal.translation) &&
            modal.storedStrokeMode == 22 &&
            IsNCTranslationToolModeAllowed(modal.toolLengthMode, modal.translation) &&
            modal.hCode == modal.translation.toolHCode &&
            modal.toolRadiusMode == 40 &&
            !modal.g68Active && !NCTranslationHasPlanarRotation(modal.translation) &&
            IsNCTranslationWorkModeAllowed(modal.g168Active, modal.workpieceCode, modal.translation) &&
            !modal.scalingActive &&
            modal.mirrorMask == 0U &&
            !modal.polarActive &&
            (modal.toolLengthMode == 49 && !modal.g168Active ? modal.cAxisOffsetRotationEnabled :
                !modal.cAxisOffsetRotationEnabled) &&
            !modal.modalMacroActive;
    }

    static bool RegistryIntegrityHealthy(
        const NCOrdinaryG00InflightRegistrySnapshot& registry,
        const NCOrdinaryG00InflightRegistryCounters& counters) noexcept
    {
        return
            registry.bounded &&
            !registry.permanentLockout &&
            !registry.motionWrite &&
            registry.accountingValid &&
            registry.capacity ==
            NC_ORDINARY_G00_INFLIGHT_REGISTRY_CAPACITY &&
            registry.activeEntries <= registry.capacity &&
            static_cast<std::uint64_t>(registry.activeEntries) ==
            counters.activeEntries &&
            counters.registrationRejected == 0ULL &&
            counters.invalidRegistration == 0ULL &&
            counters.duplicateIdentity == 0ULL &&
            counters.capacityOverflow == 0ULL &&
            counters.invalidFeedbackSequence == 0ULL &&
            counters.feedbackSequenceGaps == 0ULL &&
            counters.activeSequenceGapFailures == 0ULL &&
            counters.ledgerOrphans == 0ULL &&
            counters.identityConflicts == 0ULL &&
            counters.ownerMismatches == 0ULL &&
            counters.duplicateTerminal == 0ULL &&
            counters.terminalConflict == 0ULL &&
            counters.postTerminalFeedback == 0ULL &&
            counters.runtimeFailures == 0ULL &&
            counters.permanentFailures == 0ULL &&
            counters.motionWrites == 0ULL;
    }

    static bool RegistryHealthy(
        NCPreparedQueueSession expectedSession,
        const NCOrdinaryG00InflightRegistrySnapshot& registry,
        const NCOrdinaryG00InflightRegistryCounters& counters) noexcept
    {
        return
            expectedSession != NC_PREPARED_QUEUE_SESSION_INVALID &&
            registry.currentSession == expectedSession &&
            registry.ready &&
            RegistryIntegrityHealthy(registry, counters);
    }

    static bool LegacyWarmupExact(
        const NCPreparedHeadCutoverContext& context,
        const NCOrdinaryG00AdmissionSnapshot& admission,
        const NCOrdinaryG00AdmissionCounters& admissionCounters,
        const NCOrdinaryG00InflightRegistrySnapshot& registry,
        const NCOrdinaryG00InflightRegistryCounters& registryCounters)
        noexcept
    {
        const NCOrdinaryG00InflightEntrySnapshot& last = registry.lastEntry;
        return
            admission.session == context.head.session &&
            admission.entrySequence !=
            NC_PREPARED_ENTRY_SEQUENCE_INVALID &&
            admission.sourcePC >= 0 &&
            admission.dispatchId != NC_BLOCK_DISPATCH_ID_INVALID &&
            admission.commitSequence !=
            NC_PROGRAM_COMMIT_SEQUENCE_INVALID &&
            admission.legacyCompleted &&
            admission.legacyCallbackCompleted &&
            admission.legacyUpstreamProofVerified &&
            admission.commandPathModeProven &&
            admission.transactionalEndpointProven &&
            admission.inflightRegistryProven &&
            admission.blockerMask ==
            NC_ORDINARY_G00_BLOCKER_LEGACY_ABORTING &&
            !admission.pending &&
            !admission.permanentLockout &&
            admission.shadowOnly &&
            !admission.runtimeInfluence &&
            admission.accountingValid &&
            admissionCounters.legacyCompleted != 0ULL &&
            admissionCounters.mismatches == 0ULL &&
            admissionCounters.runtimeFailures == 0ULL &&
            RegistryHealthy(
                context.head.session,
                registry,
                registryCounters) &&
            last.session == context.head.session &&
            last.registrySequence ==
            admission.inflightRegistrySequence &&
            last.entrySequence == admission.entrySequence &&
            last.sourcePC == admission.sourcePC &&
            last.sourceLineNumber == admission.sourceLineNumber &&
            last.dispatchId == admission.dispatchId &&
            last.commitSequence == admission.commitSequence &&
            SameIdentity(last.identity, admission.legacyCommitIdentity) &&
            last.state == NCOrdinaryG00InflightState::K63_COMPLETED &&
            last.terminal &&
            !last.active &&
            last.ledgerAcceptedTerminal;
    }

    bool PendingContextExact(
        const NCPreparedHeadCutoverContext& context) const noexcept
    {
        return
            ContextIdentityExact(context) &&
            context.head.session == m_pendingEntry.session &&
            context.head.entrySequence == m_pendingEntry.entrySequence &&
            context.head.sourcePC == m_pendingEntry.sourcePC &&
            context.head.sourceLineNumber ==
            m_pendingEntry.sourceLineNumber &&
            SourceExact(context.head.source, m_pendingEntry.source);
    }

    void ObserveActiveSession(NCPreparedQueueSession session) noexcept
    {
        if (session == NC_PREPARED_QUEUE_SESSION_INVALID ||
            session == m_activeSession)
        {
            return;
        }
        if (m_pending)
        {
            ++m_counters.revokedPending;
            ClearPending();
        }
        m_activeSession = session;
        m_warmupSession = NC_PREPARED_QUEUE_SESSION_INVALID;
        m_chainSourcePC = -1;
        m_chainAnchorCommitSequence =
            NC_PROGRAM_COMMIT_SEQUENCE_INVALID;
        m_inactivePublished = false;
    }

    void BeginEvaluation(
        const NCPreparedHeadCutoverContext& context,
        std::uint64_t queueDepth,
        std::uint32_t activeEntries) noexcept
    {
        m_snapshot = NCOrdinaryG00ReadAheadSnapshot{};
        m_snapshot.enabled = m_enabled;
        m_snapshot.permanentLockout = m_permanentLockout;
        m_snapshot.registryHealthFenced = m_registryHealthFenced;
        m_snapshot.registryHealthLockout = m_registryHealthLockout;
        m_snapshot.session = context.queue.session;
        m_snapshot.warmupSession = m_warmupSession;
        m_snapshot.queueDepthAtSelection = queueDepth;
        m_snapshot.activeEntriesAtSelection = activeEntries;
        m_snapshot.chainSourcePC = m_chainSourcePC;
        if (activeEntries > m_counters.maxActiveObserved)
        {
            m_counters.maxActiveObserved = activeEntries;
        }
        if (!context.hasHead)
        {
            return;
        }
        m_snapshot.entrySequence = context.head.entrySequence;
        m_snapshot.scope = context.head.source.scope;
        m_snapshot.cacheGeneration = context.head.source.cacheGeneration;
        m_snapshot.frameId = context.head.source.frameId;
        m_snapshot.sourceExecutionEpoch =
            context.head.source.executionEpoch;
        m_snapshot.programFlowGeneration =
            context.head.source.programFlowGeneration;
        m_snapshot.owner = context.head.source.owner;
        m_snapshot.panelMask = PanelMask(context.head.source.panel);
        m_snapshot.ownerGeneration = context.head.source.ownerGeneration;
        m_snapshot.sourcePC = context.head.sourcePC;
        m_snapshot.sourceLineNumber = context.head.sourceLineNumber;
    }

    void FailProof(bool runtimeFailure) noexcept
    {
        if (runtimeFailure)
        {
            ++m_counters.runtimeFailures;
        }
        else
        {
            ++m_counters.proofMismatches;
        }
        m_permanentLockout = true;
        m_snapshot.permanentLockout = true;
        m_snapshot.pending = false;
        ClearPending();
        Publish(runtimeFailure
            ? NCOrdinaryG00ReadAheadDecision::RUNTIME_FAILURE
            : NCOrdinaryG00ReadAheadDecision::PROOF_MISMATCH);
    }

    void FenceRegistryHealth() noexcept
    {
        ++m_counters.registryHealthFences;
        if (!m_registryHealthLockout)
        {
            ++m_counters.registryHealthLockouts;
        }
        m_registryHealthFenced = true;
        m_registryHealthLockout = true;
        m_permanentLockout = true;
        m_snapshot.registryHealthFenced = true;
        m_snapshot.registryHealthLockout = true;
        m_snapshot.permanentLockout = true;
        Publish(
            NCOrdinaryG00ReadAheadDecision::
            REGISTRY_UNHEALTHY_FALLBACK);
    }

    void ClearPending() noexcept
    {
        m_pending = false;
        m_pendingDispatchBound = false;
        m_pendingMotionAuthorized = false;
        m_pendingEntry = NCPreparedBlockEntrySnapshot{};
        m_pendingDispatchId = NC_BLOCK_DISPATCH_ID_INVALID;
        m_pendingDispatchTarget = NCProgramCommitSnapshot{};
    }

    bool AccountingValid() const noexcept
    {
        const std::uint64_t terminalSelections =
            m_counters.committed +
            m_counters.revokedPending +
            m_counters.runtimeFailures +
            m_counters.proofMismatches +
            (m_pending ? 1ULL : 0ULL);
        return
            m_counters.selected == terminalSelections &&
            m_counters.dispatchBound <= m_counters.selected &&
            m_counters.motionAuthorized <= m_counters.dispatchBound &&
            m_counters.committed <= m_counters.motionAuthorized &&
            m_counters.registryBound == m_counters.committed &&
            m_counters.runtimeInfluence == m_counters.selected &&
            m_counters.resolverBypasses == m_counters.selected &&
            m_counters.motionWrites == 0ULL;
    }

    void Publish(NCOrdinaryG00ReadAheadDecision decision) noexcept
    {
        m_snapshot.publicationSequence = m_nextPublicationSequence++;
        if (m_snapshot.publicationSequence == 0ULL)
        {
            m_snapshot.publicationSequence = m_nextPublicationSequence++;
        }
        m_snapshot.decision = decision;
        m_snapshot.enabled = m_enabled;
        m_snapshot.warmupSession = m_warmupSession;
        m_snapshot.permanentLockout = m_permanentLockout;
        m_snapshot.registryHealthFenced = m_registryHealthFenced;
        m_snapshot.registryHealthLockout = m_registryHealthLockout;
        m_snapshot.pending = m_pending;
        m_snapshot.accountingValid = AccountingValid();
        ++m_counters.publications;
    }

    bool m_enabled = true;
    bool m_permanentLockout = false;
    bool m_registryHealthFenced = false;
    bool m_registryHealthLockout = false;
    bool m_pending = false;
    bool m_pendingDispatchBound = false;
    bool m_pendingMotionAuthorized = false;
    bool m_inactivePublished = false;
    NCPreparedQueueSession m_activeSession =
        NC_PREPARED_QUEUE_SESSION_INVALID;
    NCPreparedQueueSession m_warmupSession =
        NC_PREPARED_QUEUE_SESSION_INVALID;
    int m_chainSourcePC = -1;
    NCProgramCommitSequence m_chainAnchorCommitSequence =
        NC_PROGRAM_COMMIT_SEQUENCE_INVALID;
    NCPreparedBlockEntrySnapshot m_pendingEntry{};
    NCBlockDispatchId m_pendingDispatchId =
        NC_BLOCK_DISPATCH_ID_INVALID;
    NCProgramCommitSnapshot m_pendingDispatchTarget{};
    NCOrdinaryG00ReadAheadSnapshot m_snapshot{};
    NCOrdinaryG00ReadAheadCounters m_counters{};
    std::uint64_t m_nextPublicationSequence = 1ULL;
};

static_assert(
    std::is_trivially_copyable<NCOrdinaryG00ReadAheadSnapshot>::value,
    "NCOrdinaryG00ReadAheadSnapshot must remain trivially copyable.");
static_assert(
    std::is_trivially_copyable<NCOrdinaryG00ReadAheadCounters>::value,
    "NCOrdinaryG00ReadAheadCounters must remain trivially copyable.");

inline const char* NCOrdinaryG00ReadAheadDecisionToDiagnosticName(
    NCOrdinaryG00ReadAheadDecision decision) noexcept
{
    switch (decision)
    {
    case NCOrdinaryG00ReadAheadDecision::DISABLED: return "DISABLED";
    case NCOrdinaryG00ReadAheadDecision::INELIGIBLE: return "INELIGIBLE";
    case NCOrdinaryG00ReadAheadDecision::WAIT_LEGACY_WARMUP:
        return "WAIT_LEGACY_WARMUP";
    case NCOrdinaryG00ReadAheadDecision::WAIT_CAPACITY:
        return "WAIT_CAPACITY";
    case NCOrdinaryG00ReadAheadDecision::SELECTED: return "SELECTED";
    case NCOrdinaryG00ReadAheadDecision::DISPATCH_BOUND:
        return "DISPATCH_BOUND";
    case NCOrdinaryG00ReadAheadDecision::MOTION_AUTHORIZED:
        return "MOTION_AUTHORIZED";
    case NCOrdinaryG00ReadAheadDecision::COMMITTED: return "COMMITTED";
    case NCOrdinaryG00ReadAheadDecision::REVOKED: return "REVOKED";
    case NCOrdinaryG00ReadAheadDecision::RUNTIME_FAILURE:
        return "RUNTIME_FAILURE";
    case NCOrdinaryG00ReadAheadDecision::PROOF_MISMATCH:
        return "PROOF_MISMATCH";
    case NCOrdinaryG00ReadAheadDecision::REGISTRY_UNHEALTHY_FALLBACK:
        return "REGISTRY_UNHEALTHY_FALLBACK";
    case NCOrdinaryG00ReadAheadDecision::IDLE:
    default: return "IDLE";
    }
}
