#pragma once

#include "NCPreparedHeadPreResolveAdmissionShadow.h"
#include "NCBlockCompletionBoundary.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <type_traits>

// =============================================================================
// Stage NC-0.2K.4.2 - Prepared Head Controlled Resolver Bypass
//
// This gate owns the first deliberately narrow path that may skip
// NCExpressionResolver::ResolveBlock.  It is restricted to literal MEMORY
// heads in three independently-qualified lanes:
//
//   * PURE_MODAL_COPY containing only G17/G18/G19/G90/G91 and no addresses.
//   * One explicit G00 with bit-exact P1 and only XYZABCUVW/F/P addresses.
//   * One ordinary no-P G00 with at least one runtime-mapped, enabled
//     XYZABCUVW address and optional F.
//
// Each lane must first complete an exact legacy K.4 confirmation followed by
// the same Dispatch's Program Commit, K.2 MATCHED publication and K.1
// Dispatch/Commit/retirement proof.  A bypassed token never enters K.2/K.3/K.4
// as a synthetic legacy comparison.  Once selected, any later bind, handler,
// commit or proof failure is fail-closed; the same token never falls back to
// the resolver.
// =============================================================================

enum class NCPreparedResolverBypassLane : std::uint8_t
{
    NONE = 0,
    PURE_MODAL,
    G00_P1,
    G00_NO_P
};

enum class NCPreparedResolverBypassDecision : std::uint8_t
{
    IDLE = 0,
    REVOKED,
    DISABLED,
    BUSY,
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
    QUALIFICATION_PENDING,
    QUALIFIED,
    SELECTED,
    DISPATCH_BOUND,
    COMMIT_BOUND,
    WAIT_CALLBACK_COMPLETION,
    CALLBACK_COMPLETED,
    WAIT_UPSTREAM_PROOF,
    PROOF_VERIFIED,
    RUNTIME_FAILURE,
    PROOF_MISMATCH
};

enum class NCPreparedResolverBypassRevocation : std::uint8_t
{
    NONE = 0,
    DISABLED,
    QUEUE_INACTIVE,
    ALARM,
    RESET,
    PROGRAM_END,
    SOURCE_CHANGED,
    RUNTIME_FAILURE,
    PROOF_FAILURE
};

struct NCPreparedResolverBypassSnapshot
{
    std::uint64_t publicationSequence = 0ULL;
    NCPreparedResolverBypassDecision decision =
        NCPreparedResolverBypassDecision::IDLE;
    NCPreparedResolverBypassRevocation lastRevocation =
        NCPreparedResolverBypassRevocation::NONE;
    NCPreparedResolverBypassLane lane =
        NCPreparedResolverBypassLane::NONE;

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
    NCProgramCommitSequence commitSequence =
        NC_PROGRAM_COMMIT_SEQUENCE_INVALID;

    NCPreparedQueueSession pureModalQualifiedSession =
        NC_PREPARED_QUEUE_SESSION_INVALID;
    NCPreparedQueueSession g00P1QualifiedSession =
        NC_PREPARED_QUEUE_SESSION_INVALID;
    NCPreparedQueueSession g00NoPQualifiedSession =
        NC_PREPARED_QUEUE_SESSION_INVALID;

    std::uint64_t commitExecutionEpoch = 0ULL;

    bool enabled = true;
    bool attempted = false;
    bool selected = false;
    bool dispatchBound = false;
    bool commitBound = false;
    bool upstreamProofVerified = false;
    bool queueExact = false;
    bool upstreamHealthy = false;
    bool laneQualified = false;
    bool tokenExact = false;
    bool sourceExact = false;
    bool pcLineExact = false;
    bool modalExact = false;
    bool classEligible = false;
    bool literalRebuiltExact = false;
    bool preparedValueSelected = false;
    bool legacyDrainRequired = false;
    bool legacyDrainSatisfied = false;
    bool deferredForDrain = false;
    bool callbackRequired = false;
    bool callbackActiveAtCommit = false;
    bool waitPhaseObserved = false;
    bool callbackCompletionObserved = false;
    bool legacyResolverRetained = true;
    bool runtimeInfluence = false;
    bool resolverBypassed = false;
    bool pending = false;
    bool permanentLockout = false;
    bool accountingValid = true;
};

struct NCPreparedResolverBypassCounters
{
    std::uint64_t evaluations = 0ULL;
    std::uint64_t publications = 0ULL;
    std::uint64_t selected = 0ULL;
    std::uint64_t selectedPureModal = 0ULL;
    std::uint64_t selectedG00P1 = 0ULL;
    std::uint64_t selectedG00NoP = 0ULL;
    std::uint64_t legacyFallbacks = 0ULL;
    std::uint64_t dispatchBound = 0ULL;
    std::uint64_t commitBound = 0ULL;
    std::uint64_t proofVerified = 0ULL;
    std::uint64_t proofVerifiedPureModal = 0ULL;
    std::uint64_t proofVerifiedG00P1 = 0ULL;
    std::uint64_t proofVerifiedG00NoP = 0ULL;
    std::uint64_t proofWaitSamples = 0ULL;
    std::uint64_t drainWaitSamples = 0ULL;
    std::uint64_t callbackWaitSamples = 0ULL;
    std::uint64_t callbackCompletions = 0ULL;
    std::uint64_t ordinaryWaitPhases = 0ULL;

    std::uint64_t disabled = 0ULL;
    std::uint64_t busy = 0ULL;
    std::uint64_t noHead = 0ULL;
    std::uint64_t queueRejected = 0ULL;
    std::uint64_t upstreamRejected = 0ULL;
    std::uint64_t sessionRejected = 0ULL;
    std::uint64_t tokenRejected = 0ULL;
    std::uint64_t sourceRejected = 0ULL;
    std::uint64_t pcLineRejected = 0ULL;
    std::uint64_t modalRejected = 0ULL;
    std::uint64_t classRejected = 0ULL;

    std::uint64_t qualificationCandidates = 0ULL;
    std::uint64_t qualifiedPureModal = 0ULL;
    std::uint64_t qualifiedG00P1 = 0ULL;
    std::uint64_t qualifiedG00NoP = 0ULL;
    std::uint64_t qualificationFailures = 0ULL;
    std::uint64_t qualificationInvalidations = 0ULL;

    std::uint64_t runtimeFailures = 0ULL;
    std::uint64_t proofMismatches = 0ULL;
    std::uint64_t invalidatedPendingBypasses = 0ULL;

    std::uint64_t revocations = 0ULL;
    std::uint64_t disabledRevocations = 0ULL;
    std::uint64_t queueRevocations = 0ULL;
    std::uint64_t alarmRevocations = 0ULL;
    std::uint64_t resetRevocations = 0ULL;
    std::uint64_t programEndRevocations = 0ULL;
    std::uint64_t sourceRevocations = 0ULL;
    std::uint64_t runtimeRevocations = 0ULL;
    std::uint64_t proofRevocations = 0ULL;

    std::uint64_t preparedSelections = 0ULL;
    std::uint64_t runtimeInfluence = 0ULL;
    std::uint64_t resolverBypasses = 0ULL;
};

class NCPreparedHeadResolverBypassGate
{
public:
    NCPreparedHeadResolverBypassGate() noexcept = default;

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
                NCPreparedResolverBypassRevocation::DISABLED,
                NCPreparedResolverBypassDecision::DISABLED,
                m_pendingKind != PendingKind::NONE);
            return;
        }

        m_lastObservedSession = NC_PREPARED_QUEUE_SESSION_INVALID;
        m_pureModalQualifiedSession =
            NC_PREPARED_QUEUE_SESSION_INVALID;
        m_g00P1QualifiedSession =
            NC_PREPARED_QUEUE_SESSION_INVALID;
        m_g00NoPQualifiedSession =
            NC_PREPARED_QUEUE_SESSION_INVALID;
        ClearPending();
        m_snapshot = NCPreparedResolverBypassSnapshot{};
        Publish(NCPreparedResolverBypassDecision::IDLE);
    }

    bool IsEnabled() const noexcept
    {
        return m_enabled;
    }

    // A true result is the irreversible decision for this token to skip the
    // legacy resolver.  selectedBlock is then an exact storage copy of the
    // immutable K.1 head.  A false result leaves selectedBlock empty and the
    // caller executes the complete legacy path.
    bool TrySelectPreparedBlock(
        const NCPreparedHeadCutoverContext& context,
        const NCParsedBlock& parsedBlock,
        const NCPreparedHeadEquivalenceCounters& equivalenceCounters,
        const NCPreparedHeadCutoverSnapshot& cutoverSnapshot,
        const NCPreparedHeadCutoverCounters& cutoverCounters,
        const NCPreparedPreResolveAdmissionSnapshot& admissionSnapshot,
        const NCPreparedPreResolveAdmissionCounters& admissionCounters,
        NCBlock& selectedBlock,
        bool ordinaryConfiguredAxisPresent) noexcept
    {
        selectedBlock = NCBlock{};
        ObserveActiveSession(context.queue);

        ++m_counters.evaluations;
        BeginEvaluation(context);

        if (!m_enabled)
        {
            ++m_counters.disabled;
            Fallback(NCPreparedResolverBypassDecision::DISABLED);
            return false;
        }

        m_snapshot.attempted = true;
        if (m_pendingKind != PendingKind::NONE)
        {
            ++m_counters.busy;
            Fallback(NCPreparedResolverBypassDecision::BUSY);
            return false;
        }

        if (!context.hasHead)
        {
            ++m_counters.noHead;
            Fallback(NCPreparedResolverBypassDecision::NO_HEAD);
            return false;
        }

        m_snapshot.queueExact = QueueExact(
            context.queue,
            context.queueCounters,
            context.hasHead);
        if (!m_snapshot.queueExact)
        {
            ++m_counters.queueRejected;
            Fallback(NCPreparedResolverBypassDecision::QUEUE_INVALID);
            return false;
        }

        m_snapshot.upstreamHealthy = UpstreamHealthy(
            context.queueCounters,
            equivalenceCounters,
            cutoverSnapshot,
            cutoverCounters,
            admissionSnapshot,
            admissionCounters) &&
            !m_permanentLockout;
        if (!m_snapshot.upstreamHealthy)
        {
            ++m_counters.upstreamRejected;
            Fallback(
                NCPreparedResolverBypassDecision::UPSTREAM_UNHEALTHY);
            return false;
        }

        m_snapshot.tokenExact = TokenExact(context.queue, context.head);
        if (!m_snapshot.tokenExact)
        {
            ++m_counters.tokenRejected;
            Fallback(NCPreparedResolverBypassDecision::TOKEN_MISMATCH);
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
            Fallback(NCPreparedResolverBypassDecision::SOURCE_MISMATCH);
            return false;
        }

        m_snapshot.pcLineExact =
            context.queue.runtimeCurrentPC == context.sourcePC &&
            context.head.sourcePC == context.sourcePC &&
            context.head.sourceLineNumber == context.sourceLineNumber;
        if (!m_snapshot.pcLineExact)
        {
            ++m_counters.pcLineRejected;
            Fallback(NCPreparedResolverBypassDecision::PC_LINE_MISMATCH);
            return false;
        }

        m_snapshot.modalExact =
            context.capturedBeforeResolve &&
            context.runtimeModalBeforeValid &&
            ModalExact(
                context.head.modalBefore,
                context.runtimeModalBefore,
                true);
        if (!m_snapshot.modalExact)
        {
            ++m_counters.modalRejected;
            Fallback(NCPreparedResolverBypassDecision::MODAL_MISMATCH);
            return false;
        }

        NCBlock rebuiltBlock{};
        NCPreparedBlockClassification rebuiltClassification{};
        m_snapshot.literalRebuiltExact =
            NCPreparedBlockQueueShadow::TryBuildLiteralBlock(
                parsedBlock,
                rebuiltBlock) &&
            StorageBlockEqual(
                rebuiltBlock,
                context.head.preparedBlock);
        if (m_snapshot.literalRebuiltExact)
        {
            rebuiltClassification =
                NCPreparedBlockQueueShadow::ClassifyLiteralBlock(
                    parsedBlock,
                    rebuiltBlock,
                    context.runtimeSource.panel);
            m_snapshot.literalRebuiltExact =
                ClassificationEqual(
                    rebuiltClassification,
                    context.head.classification) &&
                ExecutionPlanValidAndEqual(
                    rebuiltBlock,
                    context.head.preparedBlock);
        }

        m_snapshot.lane = ClassifyEligibleLane(
            context,
            rebuiltBlock,
            rebuiltClassification,
            m_snapshot.literalRebuiltExact,
            ordinaryConfiguredAxisPresent);
        m_snapshot.classEligible =
            m_snapshot.lane != NCPreparedResolverBypassLane::NONE;
        if (!m_snapshot.classEligible)
        {
            ++m_counters.classRejected;
            Fallback(NCPreparedResolverBypassDecision::CLASS_INELIGIBLE);
            return false;
        }

        m_snapshot.laneQualified = LaneQualified(
            m_snapshot.lane,
            context.queue.session);
        if (!m_snapshot.laneQualified)
        {
            ++m_counters.sessionRejected;
            Fallback(
                NCPreparedResolverBypassDecision::SESSION_UNQUALIFIED);
            return false;
        }

        // An already-qualified ordinary G00 must not enter Resolver/K.2/K.3/
        // K.4 on a pre-dispatch drain-wait scan.  No pending bypass token is
        // created until the same exact head is observed drained.
        if (m_snapshot.lane ==
            NCPreparedResolverBypassLane::G00_NO_P &&
            !context.legacyDrainSatisfied)
        {
            ++m_counters.drainWaitSamples;
            m_snapshot.deferredForDrain = true;
            m_snapshot.legacyResolverRetained = true;
            Publish(
                NCPreparedResolverBypassDecision::WAIT_LEGACY_DRAIN);
            return false;
        }

        StartPending(
            PendingKind::BYPASS,
            context,
            m_snapshot.lane,
            0ULL);
        selectedBlock = context.head.preparedBlock;
        ++m_counters.selected;
        RecordSelectedLane(m_snapshot.lane);
        ++m_counters.preparedSelections;
        ++m_counters.runtimeInfluence;
        ++m_counters.resolverBypasses;
        m_snapshot.selected = true;
        m_snapshot.preparedValueSelected = true;
        m_snapshot.legacyResolverRetained = false;
        m_snapshot.runtimeInfluence = true;
        m_snapshot.resolverBypassed = true;
        m_snapshot.pending = true;
        Publish(NCPreparedResolverBypassDecision::SELECTED);
        return true;
    }

    // K.4's pre-handler confirmation only stages a qualification candidate.
    // The lane is armed later, after the same token reaches K.2 MATCHED and
    // the exact K.1 proof retires it.
    void ObserveLegacyConfirmation(
        const NCPreparedHeadCutoverContext& context,
        const NCPreparedHeadCutoverSnapshot& cutover,
        const NCPreparedPreResolveAdmissionSnapshot& admission,
        std::uint64_t dispatchId,
        bool ordinaryConfiguredAxisPresent) noexcept
    {
        if (!m_enabled || m_permanentLockout ||
            m_pendingKind != PendingKind::NONE ||
            !context.hasHead || dispatchId == 0ULL)
        {
            return;
        }

        NCBlock rebuiltBlock = context.head.preparedBlock;
        NCPreparedBlockClassification rebuiltClassification =
            context.head.classification;
        const NCPreparedResolverBypassLane lane = ClassifyEligibleLane(
            context,
            rebuiltBlock,
            rebuiltClassification,
            true,
            ordinaryConfiguredAxisPresent);
        if (lane == NCPreparedResolverBypassLane::NONE ||
            LaneQualified(lane, context.queue.session))
        {
            return;
        }

        const bool exactK4 =
            admission.decision ==
            NCPreparedPreResolveAdmissionDecision::LEGACY_CONFIRMED &&
            admission.confirmed &&
            admission.session == context.head.session &&
            admission.entrySequence == context.head.entrySequence &&
            admission.dispatchId == dispatchId &&
            admission.sourcePC == context.sourcePC &&
            admission.sourceLineNumber == context.sourceLineNumber &&
            admission.scope == context.head.source.scope &&
            admission.cacheGeneration ==
            context.head.source.cacheGeneration &&
            admission.frameId == context.head.source.frameId &&
            admission.executionEpoch == context.head.source.executionEpoch &&
            admission.programFlowGeneration ==
            context.head.source.programFlowGeneration &&
            admission.owner == context.head.source.owner &&
            admission.ownerGeneration ==
            context.head.source.ownerGeneration &&
            admission.panelMask == PanelMask(context.head.source.panel) &&
            admission.queueExact &&
            admission.upstreamHealthy &&
            admission.sessionQualified &&
            admission.tokenExact &&
            admission.sourceExact &&
            admission.pcLineExact &&
            admission.modalExact &&
            admission.classEligible &&
            admission.legacyResolveObserved &&
            admission.equivalenceObserved &&
            admission.cutoverObserved &&
            admission.shadowOnly &&
            !admission.runtimeInfluence &&
            !admission.resolverBypassed &&
            admission.accountingValid &&
            !admission.permanentLockout;

        const bool exactK3 =
            cutover.decision == NCPreparedHeadCutoverDecision::APPLIED &&
            cutover.session == context.head.session &&
            cutover.entrySequence == context.head.entrySequence &&
            cutover.dispatchId == dispatchId &&
            cutover.sourcePC == context.sourcePC &&
            cutover.sourceLineNumber == context.sourceLineNumber &&
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

        if (!exactK4 || !exactK3)
        {
            return;
        }

        BeginEvaluation(context);
        StartPending(
            PendingKind::QUALIFICATION,
            context,
            lane,
            dispatchId);
        m_pendingDispatchBound = true;
        ++m_counters.qualificationCandidates;
        m_snapshot.dispatchId = dispatchId;
        m_snapshot.dispatchBound = true;
        m_snapshot.pending = true;
        Publish(
            NCPreparedResolverBypassDecision::QUALIFICATION_PENDING);
    }

    bool BindDispatch(
        const NCPreparedHeadCutoverContext& context,
        std::uint64_t dispatchId,
        const NCProgramCommitSnapshot& dispatchTarget,
        const NCPreparedSourceIdentity& liveSource,
        bool ledgerFound,
        const NCProgramCommitSnapshot& ledgerDispatchTarget,
        int ledgerSourceLineNumber) noexcept
    {
        if (m_pendingKind != PendingKind::BYPASS ||
            dispatchId == 0ULL ||
            m_pendingDispatchBound)
        {
            return false;
        }

        const bool exact =
            PendingContextExact(context) &&
            SourceIdentityExact(m_pendingEntry.source, liveSource) &&
            ProgramTargetMatchesEntry(
                dispatchTarget,
                m_pendingEntry) &&
            ledgerFound &&
            ProgramTargetMatchesEntry(
                ledgerDispatchTarget,
                m_pendingEntry) &&
            ledgerSourceLineNumber == m_pendingEntry.sourceLineNumber;
        if (!exact)
        {
            FailProof();
            return false;
        }

        m_pendingDispatchId = dispatchId;
        m_pendingDispatchTarget = dispatchTarget;
        m_pendingDispatchBound = true;
        ++m_counters.dispatchBound;
        m_snapshot.dispatchId = dispatchId;
        m_snapshot.dispatchBound = true;
        Publish(NCPreparedResolverBypassDecision::DISPATCH_BOUND);
        return true;
    }

    // Returns false only when a currently pending bypass failed its exact
    // local commit proof.  Qualification failures lock the future gate but do
    // not replace the already-executed legacy behavior of that line.
    bool ObserveProgramCommit(
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
        int ledgerSourceLineNumber) noexcept
    {
        if (m_pendingKind == PendingKind::NONE ||
            dispatchId == 0ULL ||
            dispatchId != m_pendingDispatchId)
        {
            return true;
        }

        const bool ordinaryG00 =
            m_pendingLane == NCPreparedResolverBypassLane::G00_NO_P;
        const bool compareG00Override =
            (m_pendingLane == NCPreparedResolverBypassLane::G00_P1 ||
                ordinaryG00) &&
            m_pendingEntry.preparedBlock.has('F');
        m_snapshot.callbackRequired = ordinaryG00;
        m_snapshot.callbackActiveAtCommit =
            runtimeWaitCallbackActive;
        m_snapshot.commitExecutionEpoch = liveSource.executionEpoch;

        const bool exact =
            m_pendingDispatchBound &&
            !m_pendingCommitBound &&
            commitSucceeded &&
            commitTarget.sequence !=
            NC_PROGRAM_COMMIT_SEQUENCE_INVALID &&
            ProgramTargetMatchesEntry(commitTarget, m_pendingEntry) &&
            SourceIdentityMatchesCommitLane(
                m_pendingEntry.source,
                liveSource,
                m_pendingLane) &&
            ModalExact(
                m_pendingEntry.modalAfter,
                liveModalAfter,
                compareG00Override) &&
            (!ordinaryG00 || runtimeWaitCallbackActive) &&
            ledgerFound &&
            ledgerProgramCommitted &&
            ProgramTargetMatchesEntry(
                ledgerDispatchTarget,
                m_pendingEntry) &&
            ProgramTargetMatchesEntry(
                ledgerCommitTarget,
                m_pendingEntry) &&
            ledgerCommitTarget.sequence == commitTarget.sequence &&
            ledgerSourceLineNumber == m_pendingEntry.sourceLineNumber;

        if (!exact)
        {
            const bool wasBypass =
                m_pendingKind == PendingKind::BYPASS;
            if (wasBypass)
            {
                FailProof();
            }
            else
            {
                FailQualification();
            }
            return !wasBypass;
        }

        m_pendingCommitTarget = commitTarget;
        m_pendingCommitSource = liveSource;
        m_pendingCommitBound = true;
        m_pendingCallbackActiveAtCommit = runtimeWaitCallbackActive;
        m_snapshot.commitSequence = commitTarget.sequence;
        m_snapshot.commitBound = true;
        if (m_pendingKind == PendingKind::BYPASS)
        {
            ++m_counters.commitBound;
        }
        Publish(NCPreparedResolverBypassDecision::COMMIT_BOUND);
        return true;
    }

    // Called on every active completion-callback sample after the Stage F
    // dual-key boundary has been evaluated, but before the callback/binding is
    // cleared or Runtime PC is advanced.  Only ordinary no-P G00 consumes this
    // evidence; the two K.4.1 lanes remain unchanged.
    bool ObserveCompletionWaitSample(
        std::uint64_t dispatchId,
        const NCBlockCompletionBoundarySnapshot& boundary,
        bool legacyReady,
        bool effectiveReady) noexcept
    {
        if (m_pendingKind == PendingKind::NONE ||
            m_pendingLane != NCPreparedResolverBypassLane::G00_NO_P)
        {
            return true;
        }

        const bool identityExact =
            m_pendingDispatchBound &&
            m_pendingCommitBound &&
            m_pendingCallbackActiveAtCommit &&
            dispatchId != 0ULL &&
            dispatchId == m_pendingDispatchId &&
            boundary.IsValid() &&
            boundary.dispatchId == m_pendingDispatchId &&
            boundary.waitKind == NCBlockWaitKind::MOTION_HANDLER &&
            boundary.bound &&
            boundary.guardEligible &&
            boundary.guardApplied &&
            boundary.legacyReady == legacyReady &&
            boundary.effectiveReady == effectiveReady &&
            boundary.motionSegmentCount > 0U &&
            boundary.motionCompletedCount <=
            boundary.motionSegmentCount &&
            boundary.motionTerminalCount <=
            boundary.motionSegmentCount &&
            boundary.motionFailedCount == 0U &&
            !boundary.failClosed;

        bool sampleExact = identityExact;
        if (sampleExact && effectiveReady)
        {
            sampleExact =
                legacyReady &&
                boundary.comparison ==
                NCBlockCompletionComparison::AGREE_READY &&
                boundary.gateDecision ==
                NCBlockCompletionGateDecision::RELEASE_DUAL_KEY &&
                boundary.ledgerBoundary ==
                NCBlockMotionBoundaryState::SUCCEEDED &&
                boundary.lifecycleState ==
                NCBlockLifecycleState::MOTION_COMPLETED &&
                boundary.motionCompletedCount ==
                boundary.motionSegmentCount &&
                boundary.motionTerminalCount ==
                boundary.motionSegmentCount &&
                boundary.releaseObserved;
        }
        else if (sampleExact)
        {
            const bool bothWaiting =
                boundary.comparison ==
                NCBlockCompletionComparison::AGREE_WAITING &&
                boundary.gateDecision ==
                NCBlockCompletionGateDecision::WAIT_BOTH &&
                boundary.ledgerBoundary ==
                NCBlockMotionBoundaryState::PENDING;
            const bool legacyWaiting =
                boundary.comparison ==
                NCBlockCompletionComparison::LEDGER_READY_LEGACY_WAITING &&
                boundary.gateDecision ==
                NCBlockCompletionGateDecision::WAIT_LEGACY &&
                boundary.ledgerBoundary ==
                NCBlockMotionBoundaryState::SUCCEEDED;
            const bool ledgerWaiting =
                boundary.comparison ==
                NCBlockCompletionComparison::LEGACY_READY_LEDGER_PENDING &&
                boundary.gateDecision ==
                NCBlockCompletionGateDecision::BLOCK_LEDGER_PENDING &&
                boundary.ledgerBoundary ==
                NCBlockMotionBoundaryState::PENDING;
            sampleExact =
                bothWaiting || legacyWaiting || ledgerWaiting;
        }

        if (!sampleExact)
        {
            const bool wasBypass =
                m_pendingKind == PendingKind::BYPASS;
            if (wasBypass)
            {
                FailProof();
            }
            else
            {
                FailQualification();
            }
            return !wasBypass;
        }

        if (!effectiveReady)
        {
            ++m_counters.callbackWaitSamples;
            Publish(
                NCPreparedResolverBypassDecision::WAIT_CALLBACK_COMPLETION);
            return true;
        }

        if (!m_pendingCallbackCompletionObserved)
        {
            m_pendingCallbackCompletionObserved = true;
            ++m_counters.callbackCompletions;
        }
        m_snapshot.callbackCompletionObserved = true;
        Publish(NCPreparedResolverBypassDecision::CALLBACK_COMPLETED);
        return true;
    }

    // Called after K.1 has consumed the Runtime proof and after K.2/K.3 have
    // consumed K.1's updated state.  It closes either one bypass or one
    // legacy lane-qualification candidate.
    void ObserveUpstreamProof(
        const NCPreparedBlockQueueSnapshot& queue,
        const NCPreparedBlockQueueCounters& queueCounters,
        const NCPreparedRuntimeProof& proof,
        const NCPreparedHeadEquivalenceSnapshot& equivalence,
        const NCPreparedHeadEquivalenceCounters& equivalenceCounters) noexcept
    {
        if (m_pendingKind == PendingKind::NONE)
        {
            ObserveActiveSession(queue);
            return;
        }

        // Lifecycle invalidation is classified by ObserveQueueInactive(),
        // which is called immediately after this observer.  Do not turn an
        // Alarm/RESET/PROGRAM_END invalidation into a permanent proof fault.
        if (!queue.active || !queue.valid ||
            queue.session == NC_PREPARED_QUEUE_SESSION_INVALID)
        {
            return;
        }

        // A live, valid Queue that has already moved to another session is a
        // source change, not a same-token proof mismatch.
        if (queue.session != m_pendingEntry.session)
        {
            Revoke(
                NCPreparedResolverBypassRevocation::SOURCE_CHANGED,
                NCPreparedResolverBypassDecision::REVOKED,
                true);
            m_lastObservedSession = queue.session;
            return;
        }

        if (!m_pendingDispatchBound || !m_pendingCommitBound)
        {
            return;
        }

        const bool ordinaryG00 =
            m_pendingLane == NCPreparedResolverBypassLane::G00_NO_P;
        const bool baselinesIncrementable =
            m_baselineQueueCounters.dispatchMatched != UINT64_MAX &&
            m_baselineQueueCounters.commitMatched != UINT64_MAX &&
            m_baselineQueueCounters.retired != UINT64_MAX &&
            (!ordinaryG00 ||
                m_baselineQueueCounters.correlatedEpochAdvances !=
                UINT64_MAX);
        const bool countersNotAdvancedTooFar =
            baselinesIncrementable &&
            queueCounters.dispatchMatched <=
            m_baselineQueueCounters.dispatchMatched + 1ULL &&
            queueCounters.commitMatched <=
            m_baselineQueueCounters.commitMatched + 1ULL &&
            queueCounters.retired <=
            m_baselineQueueCounters.retired + 1ULL &&
            queueCounters.correlatedEpochAdvances <=
            m_baselineQueueCounters.correlatedEpochAdvances +
            (ordinaryG00 ? 1ULL : 0ULL);
        const bool failuresUnchanged = QueueFailuresExact(
            queueCounters,
            m_baselineQueueCounters) &&
            EquivalenceFailuresHealthy(equivalenceCounters);
        const bool exactProofTargets =
            proof.hasDispatch &&
            proof.dispatchId == m_pendingDispatchId &&
            ProgramTargetMatchesEntry(
                proof.dispatchTarget,
                m_pendingEntry) &&
            ProgramTargetMatchesEntry(
                proof.commitTarget,
                m_pendingEntry) &&
            proof.commitTarget.sequence ==
            m_pendingCommitTarget.sequence;

        if (!countersNotAdvancedTooFar ||
            !failuresUnchanged ||
            !exactProofTargets)
        {
            if (m_pendingKind == PendingKind::BYPASS)
            {
                FailProof();
            }
            else
            {
                FailQualification();
            }
            return;
        }

        const bool dispatchCommitMatched =
            queueCounters.dispatchMatched ==
            m_baselineQueueCounters.dispatchMatched + 1ULL &&
            queueCounters.commitMatched ==
            m_baselineQueueCounters.commitMatched + 1ULL;
        const bool retired =
            queueCounters.retired ==
            m_baselineQueueCounters.retired + 1ULL;

        const bool queueBaseExact =
            queue.active &&
            queue.valid &&
            queue.cursorOrderValid &&
            queue.accountingValid &&
            queue.shadowOnly &&
            queue.session == m_pendingEntry.session;

        if (ordinaryG00)
        {
            const bool correlatedEpochExact =
                queueCounters.correlatedEpochAdvances ==
                m_baselineQueueCounters.correlatedEpochAdvances + 1ULL;
            const bool queueCommitSourceExact =
                SourceIdentityExact(
                    queue.source,
                    m_pendingCommitSource);

            if (!dispatchCommitMatched)
            {
                if (retired)
                {
                    FailCurrentPendingProof();
                    return;
                }
                ++m_counters.proofWaitSamples;
                Publish(
                    NCPreparedResolverBypassDecision::WAIT_UPSTREAM_PROOF);
                return;
            }

            if (!retired)
            {
                const bool exactWaitPhase =
                    queueBaseExact &&
                    queueCommitSourceExact &&
                    correlatedEpochExact &&
                    m_pendingCallbackActiveAtCommit &&
                    queue.runtimeCurrentPC == m_pendingEntry.sourcePC &&
                    queueCounters.retired ==
                    m_baselineQueueCounters.retired;
                if (!exactWaitPhase)
                {
                    FailCurrentPendingProof();
                    return;
                }
                if (!m_pendingWaitPhaseObserved)
                {
                    m_pendingWaitPhaseObserved = true;
                    ++m_counters.ordinaryWaitPhases;
                }
                m_snapshot.waitPhaseObserved = true;
                ++m_counters.proofWaitSamples;
                Publish(
                    NCPreparedResolverBypassDecision::WAIT_UPSTREAM_PROOF);
                return;
            }

            const bool exactRetiredPhase =
                queueBaseExact &&
                queueCommitSourceExact &&
                correlatedEpochExact &&
                m_pendingWaitPhaseObserved &&
                m_pendingCallbackCompletionObserved &&
                queue.runtimeCurrentPC == m_pendingEntry.sourcePC + 1;
            if (!exactRetiredPhase)
            {
                FailCurrentPendingProof();
                return;
            }

            if (m_pendingKind == PendingKind::QUALIFICATION)
            {
                const bool exactK2 =
                    ExactQualificationProof(equivalence) &&
                    equivalence.executionEpoch ==
                    m_pendingEntry.source.executionEpoch &&
                    equivalence.commitExecutionEpoch ==
                    m_pendingCommitSource.executionEpoch &&
                    equivalence.runtimeWaitCallbackActive;
                if (!exactK2)
                {
                    FailQualification();
                    return;
                }
                QualifyPendingLane();
                return;
            }

            ++m_counters.proofVerified;
            RecordVerifiedLane(m_pendingLane);
            m_snapshot.upstreamProofVerified = true;
            m_snapshot.pending = false;
            ClearPending();
            Publish(NCPreparedResolverBypassDecision::PROOF_VERIFIED);
            return;
        }

        if (!dispatchCommitMatched || !retired)
        {
            ++m_counters.proofWaitSamples;
            Publish(
                NCPreparedResolverBypassDecision::WAIT_UPSTREAM_PROOF);
            return;
        }

        const bool queueSessionExact =
            queueBaseExact &&
            SourceIdentityExact(
                queue.source,
                m_pendingEntry.source) &&
            queue.runtimeCurrentPC == m_pendingEntry.sourcePC + 1;
        if (!queueSessionExact)
        {
            if (m_pendingKind == PendingKind::BYPASS)
            {
                FailProof();
            }
            else
            {
                FailQualification();
            }
            return;
        }

        if (m_pendingKind == PendingKind::QUALIFICATION)
        {
            const bool exactK2 = ExactQualificationProof(equivalence);
            if (!exactK2)
            {
                FailQualification();
                return;
            }

            QualifyPendingLane();
            return;
        }

        ++m_counters.proofVerified;
        RecordVerifiedLane(m_pendingLane);
        m_snapshot.upstreamProofVerified = true;
        m_snapshot.pending = false;
        ClearPending();
        Publish(NCPreparedResolverBypassDecision::PROOF_VERIFIED);
    }

    void ObserveRuntimeFailure(std::uint64_t dispatchId) noexcept
    {
        if (m_pendingKind == PendingKind::NONE ||
            dispatchId == 0ULL ||
            dispatchId != m_pendingDispatchId)
        {
            return;
        }

        if (m_pendingKind == PendingKind::BYPASS)
        {
            ++m_counters.runtimeFailures;
        }
        else
        {
            ++m_counters.qualificationFailures;
        }
        m_permanentLockout = true;
        Revoke(
            NCPreparedResolverBypassRevocation::RUNTIME_FAILURE,
            NCPreparedResolverBypassDecision::RUNTIME_FAILURE,
            false);
    }

    void ObserveSelectedInvariantFailure() noexcept
    {
        if (m_pendingKind == PendingKind::BYPASS)
        {
            FailProof();
        }
    }

    void ObserveActiveSession(
        const NCPreparedBlockQueueSnapshot& queue) noexcept
    {
        if (!queue.active || !queue.valid ||
            queue.session == NC_PREPARED_QUEUE_SESSION_INVALID)
        {
            return;
        }

        if (m_lastObservedSession !=
            NC_PREPARED_QUEUE_SESSION_INVALID &&
            queue.session != m_lastObservedSession)
        {
            const bool pendingLost =
                m_pendingKind != PendingKind::NONE;
            Revoke(
                NCPreparedResolverBypassRevocation::SOURCE_CHANGED,
                NCPreparedResolverBypassDecision::REVOKED,
                pendingLost);
        }
        m_lastObservedSession = queue.session;
    }

    void ObserveQueueInactive(
        NCPreparedInvalidationReason reason) noexcept
    {
        if (m_lastObservedSession ==
            NC_PREPARED_QUEUE_SESSION_INVALID &&
            m_pendingKind == PendingKind::NONE &&
            m_pureModalQualifiedSession ==
            NC_PREPARED_QUEUE_SESSION_INVALID &&
            m_g00P1QualifiedSession ==
            NC_PREPARED_QUEUE_SESSION_INVALID &&
            m_g00NoPQualifiedSession ==
            NC_PREPARED_QUEUE_SESSION_INVALID)
        {
            return;
        }

        const bool pendingLost = m_pendingKind != PendingKind::NONE;
        Revoke(
            MapRevocation(reason),
            NCPreparedResolverBypassDecision::REVOKED,
            pendingLost);
    }

    NCPreparedResolverBypassSnapshot GetSnapshot() const noexcept
    {
        return m_snapshot;
    }

    NCPreparedResolverBypassCounters GetCounters() const noexcept
    {
        return m_counters;
    }

private:
    enum class PendingKind : std::uint8_t
    {
        NONE = 0,
        QUALIFICATION,
        BYPASS
    };

    static bool DoubleBitsEqual(double left, double right) noexcept
    {
        std::uint64_t leftBits = 0ULL;
        std::uint64_t rightBits = 0ULL;
        static_assert(
            sizeof(leftBits) == sizeof(left),
            "K.4.1 assumes an IEEE-754-sized double value.");
        std::memcpy(&leftBits, &left, sizeof(leftBits));
        std::memcpy(&rightBits, &right, sizeof(rightBits));
        return leftBits == rightBits;
    }

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

    static bool SourceIdentityImmutableExact(
        const NCPreparedSourceIdentity& left,
        const NCPreparedSourceIdentity& right) noexcept
    {
        return
            left.scope == right.scope &&
            left.cacheGeneration == right.cacheGeneration &&
            left.frameId == right.frameId &&
            left.programFlowGeneration == right.programFlowGeneration &&
            left.owner == right.owner &&
            left.ownerGeneration == right.ownerGeneration &&
            left.panel.blockSkipEnabled == right.panel.blockSkipEnabled &&
            left.panel.singleBlockEnabled ==
            right.panel.singleBlockEnabled &&
            left.panel.optionalStopEnabled ==
            right.panel.optionalStopEnabled;
    }

    static bool SourceIdentityMatchesCommitLane(
        const NCPreparedSourceIdentity& prepared,
        const NCPreparedSourceIdentity& live,
        NCPreparedResolverBypassLane lane) noexcept
    {
        if (lane != NCPreparedResolverBypassLane::G00_NO_P)
        {
            return SourceIdentityExact(prepared, live);
        }
        return
            SourceIdentityImmutableExact(prepared, live) &&
            prepared.executionEpoch <
            static_cast<std::uint64_t>(UINT32_MAX) &&
            live.executionEpoch == prepared.executionEpoch + 1ULL;
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

    static bool StorageBlockEqual(
        const NCBlock& left,
        const NCBlock& right) noexcept
    {
        if (left.isEmpty != right.isEmpty ||
            left.isBlockSkip != right.isBlockSkip ||
            left.isGoto != right.isGoto ||
            left.gotoTarget != right.gotoTarget ||
            left.gCount != right.gCount ||
            left.hasG != right.hasG ||
            left.gCode != right.gCode ||
            left.mCount != right.mCount)
        {
            return false;
        }

        if (left.gCount < 0 ||
            left.gCount > NC_MAX_G_CODES_PER_BLOCK ||
            left.mCount < 0 ||
            left.mCount > NC_MAX_M_CODES_PER_BLOCK)
        {
            return false;
        }

        for (int index = 0;
            index < NC_MAX_G_CODES_PER_BLOCK;
            ++index)
        {
            if (left.gCodes[index] != right.gCodes[index])
            {
                return false;
            }
        }
        for (int index = 0;
            index < NC_MAX_M_CODES_PER_BLOCK;
            ++index)
        {
            if (left.mCode[index] != right.mCode[index])
            {
                return false;
            }
        }
        for (int index = 0; index < 26; ++index)
        {
            if (left.hasParam[index] != right.hasParam[index] ||
                !DoubleBitsEqual(left.param[index], right.param[index]))
            {
                return false;
            }
        }
        return true;
    }

    static bool ClassificationEqual(
        const NCPreparedBlockClassification& left,
        const NCPreparedBlockClassification& right) noexcept
    {
        return
            left.blockClass == right.blockClass &&
            left.barrierKind == right.barrierKind &&
            left.barrierFlags == right.barrierFlags &&
            left.primaryGCode == right.primaryGCode &&
            left.firstMCode == right.firstMCode &&
            left.planningStopsHere == right.planningStopsHere &&
            left.legacyDrainRequired == right.legacyDrainRequired &&
            left.literalResolved == right.literalResolved &&
            left.modalAfterValid == right.modalAfterValid &&
            left.runtimeGotoControl == right.runtimeGotoControl;
    }

    static bool ExecutionPlanValidAndEqual(
        const NCBlock& left,
        const NCBlock& right) noexcept
    {
        NCGCodeExecutionPlan leftPlan{};
        NCGCodeExecutionPlan rightPlan{};
        NCGCodePlanError leftError = NCGCodePlanError::NONE;
        NCGCodePlanError rightError = NCGCodePlanError::NONE;
        int leftFirst = -1;
        int leftSecond = -1;
        int rightFirst = -1;
        int rightSecond = -1;
        const bool leftValid = NCGCodeSemantics::BuildExecutionPlan(
            left,
            leftPlan,
            leftError,
            leftFirst,
            leftSecond);
        const bool rightValid = NCGCodeSemantics::BuildExecutionPlan(
            right,
            rightPlan,
            rightError,
            rightFirst,
            rightSecond);
        if (!leftValid || !rightValid ||
            leftError != NCGCodePlanError::NONE ||
            rightError != NCGCodePlanError::NONE ||
            leftFirst != rightFirst ||
            leftSecond != rightSecond ||
            leftPlan.count != rightPlan.count ||
            leftPlan.hasPrimaryAction != rightPlan.hasPrimaryAction ||
            leftPlan.primaryActionCode != rightPlan.primaryActionCode ||
            leftPlan.count < 0 ||
            leftPlan.count > NC_MAX_G_CODES_PER_BLOCK)
        {
            return false;
        }
        for (int index = 0; index < leftPlan.count; ++index)
        {
            if (leftPlan.orderedCodes[index] !=
                rightPlan.orderedCodes[index])
            {
                return false;
            }
        }
        return true;
    }

    static bool ModalExact(
        const NCPreparedModalSnapshot& prepared,
        const NCPreparedModalSnapshot& live,
        bool compareG00Override) noexcept
    {
        if (!prepared.imageValid || !live.imageValid ||
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

    static bool EquivalenceFailuresHealthy(
        const NCPreparedHeadEquivalenceCounters& counters) noexcept
    {
        return
            counters.mismatched == 0ULL &&
            counters.staleTokens == 0ULL &&
            counters.resolveMismatches == 0ULL &&
            counters.upstreamMismatches == 0ULL &&
            counters.runtimeFailures == 0ULL &&
            counters.useAttempts == 0ULL &&
            counters.cutoverAttempts == 0ULL &&
            counters.runtimeInfluence == 0ULL;
    }

    static bool UpstreamHealthy(
        const NCPreparedBlockQueueCounters& queueCounters,
        const NCPreparedHeadEquivalenceCounters& equivalenceCounters,
        const NCPreparedHeadCutoverSnapshot& cutoverSnapshot,
        const NCPreparedHeadCutoverCounters& cutoverCounters,
        const NCPreparedPreResolveAdmissionSnapshot& admissionSnapshot,
        const NCPreparedPreResolveAdmissionCounters& admissionCounters)
        noexcept
    {
        return
            queueCounters.cursorRegressions == 0ULL &&
            queueCounters.planDiscontinuities == 0ULL &&
            queueCounters.dispatchMismatches == 0ULL &&
            queueCounters.commitMismatches == 0ULL &&
            queueCounters.staleRuntimeProofs == 0ULL &&
            queueCounters.identityFailures == 0ULL &&
            queueCounters.cutoverAttempts == 0ULL &&
            EquivalenceFailuresHealthy(equivalenceCounters) &&
            cutoverSnapshot.enabled &&
            cutoverSnapshot.accountingValid &&
            !cutoverSnapshot.permanentLockout &&
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
            admissionSnapshot.shadowOnly &&
            !admissionSnapshot.runtimeInfluence &&
            !admissionSnapshot.resolverBypassed &&
            admissionSnapshot.accountingValid &&
            !admissionSnapshot.permanentLockout &&
            admissionCounters.legacyResolveFailures == 0ULL &&
            admissionCounters.postResolveMismatches == 0ULL &&
            admissionCounters.confirmedRuntimeFailures == 0ULL &&
            admissionCounters.runtimeInfluence == 0ULL &&
            admissionCounters.resolverBypasses == 0ULL;
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

    static bool ContainsStoredGCode(
        const NCBlock& block,
        int requiredCode) noexcept
    {
        if (block.gCount < 0 ||
            block.gCount > NC_MAX_G_CODES_PER_BLOCK)
        {
            return false;
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

    static bool IsAllowedPureModalCode(int code) noexcept
    {
        return code == 17 || code == 18 || code == 19 ||
            code == 90 || code == 91;
    }

    static bool IsAllowedP1Address(int index) noexcept
    {
        const char letter = static_cast<char>('A' + index);
        return
            letter == 'X' || letter == 'Y' || letter == 'Z' ||
            letter == 'A' || letter == 'B' || letter == 'C' ||
            letter == 'U' || letter == 'V' || letter == 'W' ||
            letter == 'F' || letter == 'P';
    }

    static bool IsAllowedNoPAddress(int index) noexcept
    {
        const char letter = static_cast<char>('A' + index);
        return
            letter == 'X' || letter == 'Y' || letter == 'Z' ||
            letter == 'A' || letter == 'B' || letter == 'C' ||
            letter == 'U' || letter == 'V' || letter == 'W' ||
            letter == 'F';
    }

    static bool HasAnyAxisAddress(const NCBlock& block) noexcept
    {
        return
            block.has('X') || block.has('Y') || block.has('Z') ||
            block.has('A') || block.has('B') || block.has('C') ||
            block.has('U') || block.has('V') || block.has('W');
    }

    static bool BlockStorageFinite(const NCBlock& block) noexcept
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
            if (!std::isfinite(block.param[index]))
            {
                return false;
            }
        }
        return true;
    }

    static NCPreparedResolverBypassLane ClassifyEligibleLane(
        const NCPreparedHeadCutoverContext& context,
        const NCBlock& rebuiltBlock,
        const NCPreparedBlockClassification& rebuiltClassification,
        bool rebuiltExact,
        bool ordinaryConfiguredAxisPresent) noexcept
    {
        const NCPreparedBlockEntrySnapshot& head = context.head;
        const NCPreparedSourceIdentity& source = context.runtimeSource;
        const NCBlock& block = head.preparedBlock;
        if (!rebuiltExact ||
            !StorageBlockEqual(block, rebuiltBlock) ||
            !ClassificationEqual(
                head.classification,
                rebuiltClassification) ||
            source.scope != NCProgramScope::MEMORY ||
            source.frameId != NC_PROGRAM_FRAME_ID_INVALID ||
            source.panel.blockSkipEnabled ||
            source.panel.singleBlockEnabled ||
            source.panel.optionalStopEnabled ||
            !context.capturedBeforeResolve ||
            !context.runtimeModalBeforeValid ||
            !head.classification.literalResolved ||
            !head.classification.modalAfterValid ||
            head.classification.planningStopsHere ||
            head.classification.barrierKind !=
            NCPreparedBarrierKind::NONE ||
            head.classification.barrierFlags !=
            NC_PREPARED_BARRIER_FLAG_NONE ||
            head.modalBefore.modalMacroActive ||
            block.isEmpty || block.isBlockSkip || block.isGoto ||
            block.mCount != 0 || block.has('T') ||
            !BlockStorageFinite(block))
        {
            return NCPreparedResolverBypassLane::NONE;
        }

        if (head.classification.blockClass ==
            NCPreparedBlockClass::PURE_MODAL_COPY)
        {
            if (context.legacyDrainRequired ||
                !context.legacyDrainSatisfied ||
                head.classification.legacyDrainRequired ||
                head.classification.primaryGCode != -1 ||
                block.gCount <= 0)
            {
                return NCPreparedResolverBypassLane::NONE;
            }
            for (int index = 0; index < block.gCount; ++index)
            {
                if (!IsAllowedPureModalCode(block.gCodes[index]))
                {
                    return NCPreparedResolverBypassLane::NONE;
                }
            }
            for (int index = 0; index < 26; ++index)
            {
                if (block.hasParam[index])
                {
                    return NCPreparedResolverBypassLane::NONE;
                }
            }
            return NCPreparedResolverBypassLane::PURE_MODAL;
        }

        if (head.classification.blockClass !=
            NCPreparedBlockClass::MOTION_SHADOW ||
            head.classification.primaryGCode != 0 ||
            block.gCount != 1 || block.gCodes[0] != 0 ||
            block.gCode != 0 ||
            !ContainsStoredGCode(block, 0))
        {
            return NCPreparedResolverBypassLane::NONE;
        }

        if (block.has('P'))
        {
            if (context.legacyDrainRequired ||
                !context.legacyDrainSatisfied ||
                head.classification.legacyDrainRequired ||
                !DoubleBitsEqual(block.val('P'), 1.0))
            {
                return NCPreparedResolverBypassLane::NONE;
            }
            for (int index = 0; index < 26; ++index)
            {
                if (block.hasParam[index] &&
                    !IsAllowedP1Address(index))
                {
                    return NCPreparedResolverBypassLane::NONE;
                }
            }
            if (block.has('F') && !HasAnyAxisAddress(block))
            {
                return NCPreparedResolverBypassLane::NONE;
            }
            return NCPreparedResolverBypassLane::G00_P1;
        }

        if (!context.legacyDrainRequired ||
            !head.classification.legacyDrainRequired ||
            source.executionEpoch >=
            static_cast<std::uint64_t>(UINT32_MAX) ||
            !ordinaryConfiguredAxisPresent ||
            !HasAnyAxisAddress(block))
        {
            return NCPreparedResolverBypassLane::NONE;
        }
        for (int index = 0; index < 26; ++index)
        {
            if (block.hasParam[index] && !IsAllowedNoPAddress(index))
            {
                return NCPreparedResolverBypassLane::NONE;
            }
        }
        return NCPreparedResolverBypassLane::G00_NO_P;
    }

    bool LaneQualified(
        NCPreparedResolverBypassLane lane,
        NCPreparedQueueSession session) const noexcept
    {
        if (lane == NCPreparedResolverBypassLane::PURE_MODAL)
        {
            return m_pureModalQualifiedSession == session;
        }
        if (lane == NCPreparedResolverBypassLane::G00_P1)
        {
            return m_g00P1QualifiedSession == session;
        }
        if (lane == NCPreparedResolverBypassLane::G00_NO_P)
        {
            return m_g00NoPQualifiedSession == session;
        }
        return false;
    }

    static bool QueueFailuresExact(
        const NCPreparedBlockQueueCounters& current,
        const NCPreparedBlockQueueCounters& baseline) noexcept
    {
        return
            current.cursorRegressions == baseline.cursorRegressions &&
            current.planDiscontinuities == baseline.planDiscontinuities &&
            current.dispatchMismatches == baseline.dispatchMismatches &&
            current.commitMismatches == baseline.commitMismatches &&
            current.staleRuntimeProofs == baseline.staleRuntimeProofs &&
            current.identityFailures == baseline.identityFailures &&
            current.invalidations == baseline.invalidations &&
            current.epochInvalidations == baseline.epochInvalidations &&
            current.sourceInvalidations == baseline.sourceInvalidations &&
            current.ownerInvalidations == baseline.ownerInvalidations &&
            current.frameInvalidations == baseline.frameInvalidations &&
            current.panelInvalidations == baseline.panelInvalidations &&
            current.cutoverAttempts == baseline.cutoverAttempts;
    }

    bool PendingContextExact(
        const NCPreparedHeadCutoverContext& context) const noexcept
    {
        const bool drainExact =
            m_pendingLane == NCPreparedResolverBypassLane::G00_NO_P
            ? (context.legacyDrainRequired &&
                context.legacyDrainSatisfied &&
                context.head.classification.legacyDrainRequired)
            : (!context.legacyDrainRequired &&
                context.legacyDrainSatisfied &&
                !context.head.classification.legacyDrainRequired);
        return
            context.hasHead &&
            drainExact &&
            context.head.session == m_pendingEntry.session &&
            context.head.entrySequence ==
            m_pendingEntry.entrySequence &&
            context.sourcePC == m_pendingEntry.sourcePC &&
            context.sourceLineNumber ==
            m_pendingEntry.sourceLineNumber &&
            SourceIdentityExact(
                context.head.source,
                m_pendingEntry.source) &&
            StorageBlockEqual(
                context.head.preparedBlock,
                m_pendingEntry.preparedBlock) &&
            ClassificationEqual(
                context.head.classification,
                m_pendingEntry.classification);
    }

    bool ExactQualificationProof(
        const NCPreparedHeadEquivalenceSnapshot& equivalence) const noexcept
    {
        return
            equivalence.state ==
            NCPreparedHeadEquivalenceState::MATCHED &&
            equivalence.session == m_pendingEntry.session &&
            equivalence.entrySequence == m_pendingEntry.entrySequence &&
            equivalence.dispatchId == m_pendingDispatchId &&
            equivalence.commitSequence ==
            m_pendingCommitTarget.sequence &&
            equivalence.sourcePC == m_pendingEntry.sourcePC &&
            equivalence.sourceLineNumber ==
            m_pendingEntry.sourceLineNumber &&
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
            equivalence.readinessQualified &&
            equivalence.accountingValid &&
            equivalence.shadowOnly &&
            !equivalence.runtimeInfluence &&
            !equivalence.cutoverApplied;
    }

    void RecordSelectedLane(
        NCPreparedResolverBypassLane lane) noexcept
    {
        if (lane == NCPreparedResolverBypassLane::PURE_MODAL)
        {
            ++m_counters.selectedPureModal;
        }
        else if (lane == NCPreparedResolverBypassLane::G00_P1)
        {
            ++m_counters.selectedG00P1;
        }
        else if (lane == NCPreparedResolverBypassLane::G00_NO_P)
        {
            ++m_counters.selectedG00NoP;
        }
    }

    void RecordVerifiedLane(
        NCPreparedResolverBypassLane lane) noexcept
    {
        if (lane == NCPreparedResolverBypassLane::PURE_MODAL)
        {
            ++m_counters.proofVerifiedPureModal;
        }
        else if (lane == NCPreparedResolverBypassLane::G00_P1)
        {
            ++m_counters.proofVerifiedG00P1;
        }
        else if (lane == NCPreparedResolverBypassLane::G00_NO_P)
        {
            ++m_counters.proofVerifiedG00NoP;
        }
    }

    void StartPending(
        PendingKind kind,
        const NCPreparedHeadCutoverContext& context,
        NCPreparedResolverBypassLane lane,
        std::uint64_t dispatchId) noexcept
    {
        m_pendingKind = kind;
        m_pendingEntry = context.head;
        m_pendingLane = lane;
        m_pendingDispatchId = dispatchId;
        m_pendingDispatchTarget = NCProgramCommitSnapshot{};
        m_pendingCommitTarget = NCProgramCommitSnapshot{};
        m_pendingCommitSource = NCPreparedSourceIdentity{};
        m_baselineQueueCounters = context.queueCounters;
        m_pendingDispatchBound = false;
        m_pendingCommitBound = false;
        m_pendingCallbackActiveAtCommit = false;
        m_pendingWaitPhaseObserved = false;
        m_pendingCallbackCompletionObserved = false;
        m_snapshot.lane = lane;
        m_snapshot.callbackRequired =
            lane == NCPreparedResolverBypassLane::G00_NO_P;
        m_snapshot.pending = true;
    }

    void ClearPending() noexcept
    {
        m_pendingKind = PendingKind::NONE;
        m_pendingEntry = NCPreparedBlockEntrySnapshot{};
        m_pendingLane = NCPreparedResolverBypassLane::NONE;
        m_pendingDispatchId = 0ULL;
        m_pendingDispatchTarget = NCProgramCommitSnapshot{};
        m_pendingCommitTarget = NCProgramCommitSnapshot{};
        m_pendingCommitSource = NCPreparedSourceIdentity{};
        m_baselineQueueCounters = NCPreparedBlockQueueCounters{};
        m_pendingDispatchBound = false;
        m_pendingCommitBound = false;
        m_pendingCallbackActiveAtCommit = false;
        m_pendingWaitPhaseObserved = false;
        m_pendingCallbackCompletionObserved = false;
    }

    void QualifyPendingLane() noexcept
    {
        if (m_pendingLane == NCPreparedResolverBypassLane::PURE_MODAL)
        {
            m_pureModalQualifiedSession = m_pendingEntry.session;
            ++m_counters.qualifiedPureModal;
        }
        else if (m_pendingLane == NCPreparedResolverBypassLane::G00_P1)
        {
            m_g00P1QualifiedSession = m_pendingEntry.session;
            ++m_counters.qualifiedG00P1;
        }
        else if (m_pendingLane == NCPreparedResolverBypassLane::G00_NO_P)
        {
            m_g00NoPQualifiedSession = m_pendingEntry.session;
            ++m_counters.qualifiedG00NoP;
        }
        m_snapshot.pureModalQualifiedSession =
            m_pureModalQualifiedSession;
        m_snapshot.g00P1QualifiedSession =
            m_g00P1QualifiedSession;
        m_snapshot.g00NoPQualifiedSession =
            m_g00NoPQualifiedSession;
        m_snapshot.laneQualified = true;
        m_snapshot.pending = false;
        ClearPending();
        Publish(NCPreparedResolverBypassDecision::QUALIFIED);
    }

    void FailCurrentPendingProof() noexcept
    {
        if (m_pendingKind == PendingKind::BYPASS)
        {
            FailProof();
        }
        else
        {
            FailQualification();
        }
    }

    void FailQualification() noexcept
    {
        ++m_counters.qualificationFailures;
        m_permanentLockout = true;
        Revoke(
            NCPreparedResolverBypassRevocation::PROOF_FAILURE,
            NCPreparedResolverBypassDecision::PROOF_MISMATCH,
            false);
    }

    void FailProof() noexcept
    {
        ++m_counters.proofMismatches;
        m_permanentLockout = true;
        Revoke(
            NCPreparedResolverBypassRevocation::PROOF_FAILURE,
            NCPreparedResolverBypassDecision::PROOF_MISMATCH,
            false);
    }

    static NCPreparedResolverBypassRevocation MapRevocation(
        NCPreparedInvalidationReason reason) noexcept
    {
        switch (reason)
        {
        case NCPreparedInvalidationReason::ALARM:
            return NCPreparedResolverBypassRevocation::ALARM;
        case NCPreparedInvalidationReason::RESET:
            return NCPreparedResolverBypassRevocation::RESET;
        case NCPreparedInvalidationReason::PROGRAM_END:
            return NCPreparedResolverBypassRevocation::PROGRAM_END;
        case NCPreparedInvalidationReason::SOURCE_REPLACED:
        case NCPreparedInvalidationReason::EXECUTION_EPOCH_CHANGED:
        case NCPreparedInvalidationReason::OWNER_CHANGED:
        case NCPreparedInvalidationReason::MODE_CHANGED:
        case NCPreparedInvalidationReason::FRAME_CHANGED:
        case NCPreparedInvalidationReason::PANEL_SWITCH_CHANGED:
        case NCPreparedInvalidationReason::CURSOR_DISCONTINUITY:
        case NCPreparedInvalidationReason::IDENTITY_INVALID:
            return NCPreparedResolverBypassRevocation::SOURCE_CHANGED;
        case NCPreparedInvalidationReason::NOT_RUNNING:
        case NCPreparedInvalidationReason::NONE:
        default:
            return NCPreparedResolverBypassRevocation::QUEUE_INACTIVE;
        }
    }

    void BeginEvaluation(
        const NCPreparedHeadCutoverContext& context) noexcept
    {
        m_snapshot = NCPreparedResolverBypassSnapshot{};
        m_snapshot.enabled = m_enabled;
        m_snapshot.permanentLockout = m_permanentLockout;
        m_snapshot.pureModalQualifiedSession =
            m_pureModalQualifiedSession;
        m_snapshot.g00P1QualifiedSession =
            m_g00P1QualifiedSession;
        m_snapshot.g00NoPQualifiedSession =
            m_g00NoPQualifiedSession;
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
        m_snapshot.legacyDrainRequired =
            context.legacyDrainRequired;
        m_snapshot.legacyDrainSatisfied =
            context.legacyDrainSatisfied;
    }

    void Fallback(
        NCPreparedResolverBypassDecision decision) noexcept
    {
        ++m_counters.legacyFallbacks;
        m_snapshot.legacyResolverRetained = true;
        Publish(decision);
    }

    void RecordRevocation(
        NCPreparedResolverBypassRevocation revocation) noexcept
    {
        switch (revocation)
        {
        case NCPreparedResolverBypassRevocation::DISABLED:
            ++m_counters.disabledRevocations;
            break;
        case NCPreparedResolverBypassRevocation::ALARM:
            ++m_counters.alarmRevocations;
            break;
        case NCPreparedResolverBypassRevocation::RESET:
            ++m_counters.resetRevocations;
            break;
        case NCPreparedResolverBypassRevocation::PROGRAM_END:
            ++m_counters.programEndRevocations;
            break;
        case NCPreparedResolverBypassRevocation::SOURCE_CHANGED:
            ++m_counters.sourceRevocations;
            break;
        case NCPreparedResolverBypassRevocation::RUNTIME_FAILURE:
            ++m_counters.runtimeRevocations;
            break;
        case NCPreparedResolverBypassRevocation::PROOF_FAILURE:
            ++m_counters.proofRevocations;
            break;
        case NCPreparedResolverBypassRevocation::QUEUE_INACTIVE:
            ++m_counters.queueRevocations;
            break;
        case NCPreparedResolverBypassRevocation::NONE:
        default:
            break;
        }
    }

    void Revoke(
        NCPreparedResolverBypassRevocation revocation,
        NCPreparedResolverBypassDecision decision,
        bool pendingLost) noexcept
    {
        if (pendingLost)
        {
            if (m_pendingKind == PendingKind::BYPASS)
            {
                ++m_counters.invalidatedPendingBypasses;
                m_permanentLockout = true;
            }
            else if (m_pendingKind == PendingKind::QUALIFICATION)
            {
                ++m_counters.qualificationInvalidations;
            }
        }

        ++m_counters.revocations;
        RecordRevocation(revocation);
        m_snapshot.lastRevocation = revocation;
        m_snapshot.permanentLockout = m_permanentLockout;
        m_snapshot.pending = false;
        m_snapshot.pureModalQualifiedSession =
            NC_PREPARED_QUEUE_SESSION_INVALID;
        m_snapshot.g00P1QualifiedSession =
            NC_PREPARED_QUEUE_SESSION_INVALID;
        m_snapshot.g00NoPQualifiedSession =
            NC_PREPARED_QUEUE_SESSION_INVALID;
        m_pureModalQualifiedSession =
            NC_PREPARED_QUEUE_SESSION_INVALID;
        m_g00P1QualifiedSession =
            NC_PREPARED_QUEUE_SESSION_INVALID;
        m_g00NoPQualifiedSession =
            NC_PREPARED_QUEUE_SESSION_INVALID;
        ClearPending();
        Publish(decision);
        m_lastObservedSession = NC_PREPARED_QUEUE_SESSION_INVALID;
    }

    bool AccountingValid() const noexcept
    {
        const std::uint64_t fallbackReasons =
            m_counters.disabled +
            m_counters.busy +
            m_counters.noHead +
            m_counters.queueRejected +
            m_counters.upstreamRejected +
            m_counters.sessionRejected +
            m_counters.tokenRejected +
            m_counters.sourceRejected +
            m_counters.pcLineRejected +
            m_counters.modalRejected +
            m_counters.classRejected;
        const std::uint64_t qualifiedTotal =
            m_counters.qualifiedPureModal +
            m_counters.qualifiedG00P1 +
            m_counters.qualifiedG00NoP;
        const std::uint64_t qualificationTerminal =
            qualifiedTotal +
            m_counters.qualificationFailures +
            m_counters.qualificationInvalidations +
            (m_pendingKind == PendingKind::QUALIFICATION ? 1ULL : 0ULL);
        const std::uint64_t bypassTerminal =
            m_counters.proofVerified +
            m_counters.runtimeFailures +
            m_counters.proofMismatches +
            m_counters.invalidatedPendingBypasses +
            (m_pendingKind == PendingKind::BYPASS ? 1ULL : 0ULL);
        const std::uint64_t revocationReasons =
            m_counters.disabledRevocations +
            m_counters.queueRevocations +
            m_counters.alarmRevocations +
            m_counters.resetRevocations +
            m_counters.programEndRevocations +
            m_counters.sourceRevocations +
            m_counters.runtimeRevocations +
            m_counters.proofRevocations;
        return
            m_counters.evaluations ==
            m_counters.selected +
            m_counters.legacyFallbacks +
            m_counters.drainWaitSamples &&
            m_counters.legacyFallbacks == fallbackReasons &&
            m_counters.qualificationCandidates == qualificationTerminal &&
            m_counters.selected == bypassTerminal &&
            m_counters.dispatchBound <= m_counters.selected &&
            m_counters.commitBound <= m_counters.dispatchBound &&
            m_counters.proofVerified <= m_counters.commitBound &&
            m_counters.selected ==
            m_counters.selectedPureModal +
            m_counters.selectedG00P1 +
            m_counters.selectedG00NoP &&
            m_counters.proofVerified ==
            m_counters.proofVerifiedPureModal +
            m_counters.proofVerifiedG00P1 +
            m_counters.proofVerifiedG00NoP &&
            m_counters.callbackCompletions <=
            m_counters.qualificationCandidates +
            m_counters.selectedG00NoP &&
            m_counters.preparedSelections == m_counters.selected &&
            m_counters.runtimeInfluence == m_counters.selected &&
            m_counters.resolverBypasses == m_counters.selected &&
            m_counters.revocations == revocationReasons;
    }

    void Publish(
        NCPreparedResolverBypassDecision decision) noexcept
    {
        m_snapshot.publicationSequence = m_nextPublicationSequence++;
        if (m_snapshot.publicationSequence == 0ULL)
        {
            m_snapshot.publicationSequence = m_nextPublicationSequence++;
        }
        m_snapshot.decision = decision;
        m_snapshot.enabled = m_enabled;
        m_snapshot.permanentLockout = m_permanentLockout;
        m_snapshot.pureModalQualifiedSession =
            m_pureModalQualifiedSession;
        m_snapshot.g00P1QualifiedSession =
            m_g00P1QualifiedSession;
        m_snapshot.g00NoPQualifiedSession =
            m_g00NoPQualifiedSession;
        if (m_pendingKind != PendingKind::NONE)
        {
            m_snapshot.waitPhaseObserved =
                m_pendingWaitPhaseObserved;
            m_snapshot.callbackCompletionObserved =
                m_pendingCallbackCompletionObserved;
        }
        ++m_counters.publications;
        m_snapshot.accountingValid = AccountingValid();
    }

    bool m_enabled = true;
    bool m_permanentLockout = false;
    NCPreparedResolverBypassSnapshot m_snapshot{};
    NCPreparedResolverBypassCounters m_counters{};
    std::uint64_t m_nextPublicationSequence = 1ULL;
    NCPreparedQueueSession m_lastObservedSession =
        NC_PREPARED_QUEUE_SESSION_INVALID;
    NCPreparedQueueSession m_pureModalQualifiedSession =
        NC_PREPARED_QUEUE_SESSION_INVALID;
    NCPreparedQueueSession m_g00P1QualifiedSession =
        NC_PREPARED_QUEUE_SESSION_INVALID;
    NCPreparedQueueSession m_g00NoPQualifiedSession =
        NC_PREPARED_QUEUE_SESSION_INVALID;

    PendingKind m_pendingKind = PendingKind::NONE;
    NCPreparedBlockEntrySnapshot m_pendingEntry{};
    NCPreparedResolverBypassLane m_pendingLane =
        NCPreparedResolverBypassLane::NONE;
    std::uint64_t m_pendingDispatchId = 0ULL;
    NCProgramCommitSnapshot m_pendingDispatchTarget{};
    NCProgramCommitSnapshot m_pendingCommitTarget{};
    NCPreparedSourceIdentity m_pendingCommitSource{};
    NCPreparedBlockQueueCounters m_baselineQueueCounters{};
    bool m_pendingDispatchBound = false;
    bool m_pendingCommitBound = false;
    bool m_pendingCallbackActiveAtCommit = false;
    bool m_pendingWaitPhaseObserved = false;
    bool m_pendingCallbackCompletionObserved = false;
};

static_assert(
    std::is_trivially_copyable<
    NCPreparedResolverBypassSnapshot>::value,
    "K.4.2 snapshots must remain bounded POD diagnostics.");

static_assert(
    std::is_trivially_copyable<
    NCPreparedResolverBypassCounters>::value,
    "K.4.2 counters must remain bounded POD diagnostics.");

inline const char* NCPreparedResolverBypassLaneToDiagnosticName(
    NCPreparedResolverBypassLane lane) noexcept
{
    switch (lane)
    {
    case NCPreparedResolverBypassLane::PURE_MODAL: return "PURE_MODAL";
    case NCPreparedResolverBypassLane::G00_P1: return "G00_P1";
    case NCPreparedResolverBypassLane::G00_NO_P: return "G00_NO_P";
    case NCPreparedResolverBypassLane::NONE:
    default: return "NONE";
    }
}

inline const char* NCPreparedResolverBypassDecisionToDiagnosticName(
    NCPreparedResolverBypassDecision decision) noexcept
{
    switch (decision)
    {
    case NCPreparedResolverBypassDecision::REVOKED: return "REVOKED";
    case NCPreparedResolverBypassDecision::DISABLED: return "DISABLED";
    case NCPreparedResolverBypassDecision::BUSY: return "BUSY";
    case NCPreparedResolverBypassDecision::NO_HEAD: return "NO_HEAD";
    case NCPreparedResolverBypassDecision::QUEUE_INVALID: return "QUEUE_INVALID";
    case NCPreparedResolverBypassDecision::UPSTREAM_UNHEALTHY: return "UPSTREAM_UNHEALTHY";
    case NCPreparedResolverBypassDecision::SESSION_UNQUALIFIED: return "SESSION_UNQUALIFIED";
    case NCPreparedResolverBypassDecision::TOKEN_MISMATCH: return "TOKEN_MISMATCH";
    case NCPreparedResolverBypassDecision::SOURCE_MISMATCH: return "SOURCE_MISMATCH";
    case NCPreparedResolverBypassDecision::PC_LINE_MISMATCH: return "PC_LINE_MISMATCH";
    case NCPreparedResolverBypassDecision::MODAL_MISMATCH: return "MODAL_MISMATCH";
    case NCPreparedResolverBypassDecision::CLASS_INELIGIBLE: return "CLASS_INELIGIBLE";
    case NCPreparedResolverBypassDecision::WAIT_LEGACY_DRAIN: return "WAIT_DRAIN";
    case NCPreparedResolverBypassDecision::QUALIFICATION_PENDING: return "QUAL_PENDING";
    case NCPreparedResolverBypassDecision::QUALIFIED: return "QUALIFIED";
    case NCPreparedResolverBypassDecision::SELECTED: return "SELECTED";
    case NCPreparedResolverBypassDecision::DISPATCH_BOUND: return "DISPATCH_BOUND";
    case NCPreparedResolverBypassDecision::COMMIT_BOUND: return "COMMIT_BOUND";
    case NCPreparedResolverBypassDecision::WAIT_CALLBACK_COMPLETION: return "WAIT_CALLBACK";
    case NCPreparedResolverBypassDecision::CALLBACK_COMPLETED: return "CALLBACK_DONE";
    case NCPreparedResolverBypassDecision::WAIT_UPSTREAM_PROOF: return "WAIT_PROOF";
    case NCPreparedResolverBypassDecision::PROOF_VERIFIED: return "PROOF_VERIFIED";
    case NCPreparedResolverBypassDecision::RUNTIME_FAILURE: return "RUNTIME_FAILURE";
    case NCPreparedResolverBypassDecision::PROOF_MISMATCH: return "PROOF_MISMATCH";
    case NCPreparedResolverBypassDecision::IDLE:
    default: return "IDLE";
    }
}

inline const char* NCPreparedResolverBypassRevocationToDiagnosticName(
    NCPreparedResolverBypassRevocation revocation) noexcept
{
    switch (revocation)
    {
    case NCPreparedResolverBypassRevocation::DISABLED: return "DISABLED";
    case NCPreparedResolverBypassRevocation::QUEUE_INACTIVE: return "QUEUE_INACTIVE";
    case NCPreparedResolverBypassRevocation::ALARM: return "ALARM";
    case NCPreparedResolverBypassRevocation::RESET: return "RESET";
    case NCPreparedResolverBypassRevocation::PROGRAM_END: return "PROGRAM_END";
    case NCPreparedResolverBypassRevocation::SOURCE_CHANGED: return "SOURCE_CHANGED";
    case NCPreparedResolverBypassRevocation::RUNTIME_FAILURE: return "RUNTIME_FAILURE";
    case NCPreparedResolverBypassRevocation::PROOF_FAILURE: return "PROOF_FAILURE";
    case NCPreparedResolverBypassRevocation::NONE:
    default: return "NONE";
    }
}
