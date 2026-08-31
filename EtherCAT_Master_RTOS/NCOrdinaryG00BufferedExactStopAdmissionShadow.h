#pragma once

#include "NCPreparedHeadResolverBypassGate.h"
#include "MotionCommandPathModeTransport.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

// =============================================================================
// Stage NC-0.2K.5 / K.6.1 - Ordinary G00 Buffered Exact-Stop Admission
//
// This observer describes the contract that a future ordinary no-P G00
// read-ahead cutover would have to satisfy.  It cannot select an NCBlock,
// change a barrier, publish an execution epoch, submit Motion work, change a
// callback, or advance a Program PC.
//
// The accepted K.4.2 Runtime path remains unchanged:
//   drain -> resolver bypass -> ABORTING / epoch+1 -> CheckMotionDone.
// K.5 correlates that real path with one immutable Prepared token and reports
// the blockers that must be removed before a later controlled cutover:
//   command-local EXACT_STOP, transactional queue-tail endpoint, and a bounded
//   multi-dispatch terminal registry.  K.6.1 closes only the first item from
//   immutable Motion submission evidence; admission remains shadow-only.
// =============================================================================

enum class NCOrdinaryG00AdmissionDecision : std::uint8_t
{
    IDLE = 0,
    WARMUP,
    PROJECTED_DRAINED,
    PROJECTED_BUSY,
    WAIT_BUSY,
    LEGACY_SELECTED,
    LEGACY_COMMITTED,
    WAIT_COMPLETION,
    CALLBACK_COMPLETED,
    WAIT_UPSTREAM_PROOF,
    LEGACY_COMPLETED,
    REJECTED,
    REVOKED,
    MISMATCHED
};

enum class NCOrdinaryG00AdmissionRevocation : std::uint8_t
{
    NONE = 0,
    QUEUE_INACTIVE,
    ALARM,
    RESET,
    PROGRAM_END,
    SOURCE_CHANGED,
    RUNTIME_FAILURE
};

enum NCOrdinaryG00AdmissionBlocker : std::uint32_t
{
    NC_ORDINARY_G00_BLOCKER_NONE = 0U,
    NC_ORDINARY_G00_BLOCKER_LEGACY_ABORTING = 1U << 0U,
    NC_ORDINARY_G00_BLOCKER_COMMAND_PATH_MODE = 1U << 1U,
    NC_ORDINARY_G00_BLOCKER_TRANSACTIONAL_ENDPOINT = 1U << 2U,
    NC_ORDINARY_G00_BLOCKER_INFLIGHT_REGISTRY = 1U << 3U
};

constexpr std::uint32_t NC_ORDINARY_G00_REQUIRED_BLOCKERS =
NC_ORDINARY_G00_BLOCKER_LEGACY_ABORTING |
NC_ORDINARY_G00_BLOCKER_COMMAND_PATH_MODE |
NC_ORDINARY_G00_BLOCKER_TRANSACTIONAL_ENDPOINT |
NC_ORDINARY_G00_BLOCKER_INFLIGHT_REGISTRY;

struct NCOrdinaryG00AdmissionSnapshot
{
    std::uint64_t publicationSequence = 0ULL;
    NCOrdinaryG00AdmissionDecision decision =
        NCOrdinaryG00AdmissionDecision::IDLE;
    NCOrdinaryG00AdmissionRevocation lastRevocation =
        NCOrdinaryG00AdmissionRevocation::NONE;

    NCPreparedQueueSession session = NC_PREPARED_QUEUE_SESSION_INVALID;
    NCPreparedEntrySequence entrySequence =
        NC_PREPARED_ENTRY_SEQUENCE_INVALID;
    NCProgramScope scope = NCProgramScope::NONE;
    NCProgramCacheGeneration cacheGeneration =
        NC_PROGRAM_CACHE_GENERATION_INVALID;
    NCProgramFrameId frameId = NC_PROGRAM_FRAME_ID_INVALID;
    std::uint64_t programFlowGeneration = 0ULL;
    std::uint8_t owner = 0U;
    std::uint8_t panelMask = 0U;
    std::uint64_t ownerGeneration = 0ULL;
    std::uint64_t sourceExecutionEpoch = 0ULL;
    std::uint64_t legacyCommitExecutionEpoch = 0ULL;
    int sourcePC = -1;
    int sourceLineNumber = 0;
    std::uint64_t dispatchId = 0ULL;
    NCProgramCommitSequence commitSequence =
        NC_PROGRAM_COMMIT_SEQUENCE_INVALID;

    std::uint64_t initialQueueDepth = 0ULL;
    std::uint64_t busySamples = 0ULL;
    std::uint32_t blockerMask = NC_ORDINARY_G00_BLOCKER_NONE;

    bool initialGroupStandstill = true;
    bool upstreamQualified = false;
    bool simpleG90Envelope = false;
    bool configuredAxisPresent = false;
    bool projected = false;
    bool initialDrained = false;
    bool initialBusy = false;
    bool legacySelected = false;
    bool legacyDispatchBound = false;
    bool legacyCommitBound = false;
    bool legacyCallbackObserved = false;
    bool legacyCallbackCompleted = false;
    bool legacyEpochAdvanced = false;
    bool commandPathModeProven = false;
    bool legacyUpstreamProofVerified = false;
    bool legacyCompleted = false;

    bool requiresBufferedTransport = true;
    bool requiresPerCommandExactStop = true;
    bool requiresStableExecutionEpoch = true;
    bool requiresNoPerBlockCallback = true;
    bool requiresTransactionalEndpoint = true;
    bool requiresInflightTerminalRegistry = true;

    bool shadowOnly = true;
    bool pending = false;
    bool permanentLockout = false;
    bool cutoverReady = false;
    bool runtimeInfluence = false;
    bool resolverBypassed = false;
    bool accountingValid = true;
};

struct NCOrdinaryG00AdmissionCounters
{
    std::uint64_t scans = 0ULL;
    std::uint64_t publications = 0ULL;
    std::uint64_t uniqueEvaluations = 0ULL;
    std::uint64_t warmup = 0ULL;
    std::uint64_t projected = 0ULL;
    std::uint64_t candidates = 0ULL;
    std::uint64_t initialDrained = 0ULL;
    std::uint64_t initialBusy = 0ULL;
    std::uint64_t busySamples = 0ULL;
    std::uint64_t legacySelections = 0ULL;
    std::uint64_t legacyDispatchBound = 0ULL;
    std::uint64_t legacyCommitBound = 0ULL;
    std::uint64_t completionWaitSamples = 0ULL;
    std::uint64_t callbackCompleted = 0ULL;
    std::uint64_t legacyCompleted = 0ULL;
    std::uint64_t rejected = 0ULL;

    std::uint64_t missingCommandPathMode = 0ULL;
    std::uint64_t missingTransactionalEndpoint = 0ULL;
    std::uint64_t missingInflightRegistry = 0ULL;

    std::uint64_t mismatches = 0ULL;
    std::uint64_t runtimeFailures = 0ULL;
    std::uint64_t invalidatedPending = 0ULL;
    std::uint64_t revocations = 0ULL;
    std::uint64_t queueRevocations = 0ULL;
    std::uint64_t alarmRevocations = 0ULL;
    std::uint64_t resetRevocations = 0ULL;
    std::uint64_t programEndRevocations = 0ULL;
    std::uint64_t sourceRevocations = 0ULL;

    // Observation-only invariants.  There is intentionally no mutating API
    // for these counters in this stage.
    std::uint64_t cutoverAttempts = 0ULL;
    std::uint64_t runtimeInfluence = 0ULL;
    std::uint64_t resolverBypasses = 0ULL;
};

struct NCOrdinaryG00LegacyCommitEvidence
{
    std::uint64_t dispatchId = 0ULL;
    NCProgramCommitSequence commitSequence =
        NC_PROGRAM_COMMIT_SEQUENCE_INVALID;
    std::uint64_t currentExecutionEpoch = 0ULL;
    std::uint64_t segmentExecutionEpoch = 0ULL;
    std::uint64_t segmentId = 0ULL;
    std::size_t submissionCount = 0U;
    MotionCommandPathMode commandPathMode =
        MotionCommandPathMode::UNSPECIFIED;
    bool captureOverflow = false;
    bool producerAccepted = false;
    bool immediateRejectNone = false;
    bool waitCallbackActive = false;
    bool commitSucceeded = false;
};

class NCOrdinaryG00BufferedExactStopAdmissionShadow
{
public:
    NCOrdinaryG00BufferedExactStopAdmissionShadow() noexcept = default;

    void ObserveResolverDecision(
        const NCPreparedHeadCutoverContext& context,
        const NCPreparedResolverBypassSnapshot& bypass,
        bool resolverBypassed,
        std::uint64_t queueDepth,
        bool groupStandstill,
        bool configuredAxisPresent) noexcept
    {
        if (bypass.lane != NCPreparedResolverBypassLane::G00_NO_P)
        {
            return;
        }

        ++m_counters.scans;
        ObserveActiveSession(context.queue.session);

        const TokenKey key = MakeKey(context);
        const bool exactEnvelope =
            BaseEvidenceExact(
                context,
                bypass,
                configuredAxisPresent) &&
            SimpleG90Envelope(context);

        if (m_pending && !KeyEqual(key, m_pendingKey))
        {
            FailMismatch();
            return;
        }

        const bool alreadySeen = HasSeen(key);
        if (!alreadySeen)
        {
            if (!RecordSeen(key))
            {
                return;
            }
            ++m_counters.uniqueEvaluations;
            ResetTokenSnapshot();
        }

        // One immutable token receives exactly one initial classification.
        // Re-observation is useful only while that same candidate is pending
        // (for example, repeated WAIT_LEGACY_DRAIN scans).
        if (alreadySeen && !m_pending)
        {
            return;
        }

        if (m_permanentLockout)
        {
            if (!alreadySeen)
            {
                ++m_counters.rejected;
            }
            CaptureIdentity(context);
            Publish(NCOrdinaryG00AdmissionDecision::REJECTED);
            return;
        }

        if (!exactEnvelope)
        {
            if (m_pending && KeyEqual(key, m_pendingKey))
            {
                FailMismatch();
                return;
            }
            if (!alreadySeen)
            {
                ++m_counters.rejected;
                CaptureIdentity(context);
                m_snapshot.simpleG90Envelope = false;
                Publish(NCOrdinaryG00AdmissionDecision::REJECTED);
            }
            return;
        }

        if (bypass.decision ==
            NCPreparedResolverBypassDecision::SESSION_UNQUALIFIED &&
            !bypass.laneQualified)
        {
            if (!alreadySeen)
            {
                ++m_counters.warmup;
                CaptureIdentity(context);
                m_snapshot.simpleG90Envelope = true;
                m_snapshot.configuredAxisPresent = true;
                m_snapshot.upstreamQualified = false;
                Publish(NCOrdinaryG00AdmissionDecision::WARMUP);
            }
            return;
        }

        const bool isBusyProjection =
            bypass.decision ==
            NCPreparedResolverBypassDecision::WAIT_LEGACY_DRAIN &&
            !resolverBypassed &&
            bypass.laneQualified &&
            bypass.deferredForDrain &&
            !context.legacyDrainSatisfied &&
            (queueDepth != 0ULL || !groupStandstill);
        const bool isDrainedSelection =
            bypass.decision ==
            NCPreparedResolverBypassDecision::SELECTED &&
            resolverBypassed &&
            bypass.selected &&
            bypass.resolverBypassed &&
            bypass.laneQualified &&
            context.legacyDrainSatisfied &&
            queueDepth == 0ULL &&
            groupStandstill;

        if (!isBusyProjection && !isDrainedSelection)
        {
            if (m_pending && KeyEqual(key, m_pendingKey))
            {
                FailMismatch();
                return;
            }
            if (!alreadySeen)
            {
                ++m_counters.rejected;
                CaptureIdentity(context);
                Publish(NCOrdinaryG00AdmissionDecision::REJECTED);
            }
            return;
        }

        if (!m_pending)
        {
            StartCandidate(
                context,
                key,
                queueDepth,
                groupStandstill,
                isBusyProjection);
        }

        if (isBusyProjection)
        {
            ++m_counters.busySamples;
            ++m_snapshot.busySamples;
            Publish(
                m_snapshot.busySamples == 1ULL
                ? NCOrdinaryG00AdmissionDecision::PROJECTED_BUSY
                : NCOrdinaryG00AdmissionDecision::WAIT_BUSY);
            return;
        }

        // The same busy token must remain byte/identity exact until the real
        // K.4.2 drain completes.  A direct drained candidate reaches here on
        // its first observation.
        if (!m_pending ||
            !KeyEqual(key, m_pendingKey) ||
            bypass.executionEpoch != m_snapshot.sourceExecutionEpoch)
        {
            FailMismatch();
            return;
        }

        m_snapshot.legacySelected = true;
        m_snapshot.resolverBypassed = false; // K.5 itself never bypasses.
        ++m_counters.legacySelections;
        Publish(NCOrdinaryG00AdmissionDecision::LEGACY_SELECTED);
    }

    void ObserveLegacyCommit(
        const NCPreparedHeadCutoverContext& context,
        const NCPreparedResolverBypassSnapshot& bypass,
        const NCOrdinaryG00LegacyCommitEvidence& evidence) noexcept
    {
        if (!m_pending)
        {
            return;
        }

        const TokenKey key = MakeKey(context);
        const bool epochCanAdvance =
            m_snapshot.sourceExecutionEpoch <
            static_cast<std::uint64_t>(UINT32_MAX);
        const std::uint64_t expectedEpoch =
            m_snapshot.sourceExecutionEpoch + 1ULL;
        const bool commandPathModeProven =
            evidence.submissionCount == 1U &&
            evidence.commandPathMode ==
            MotionCommandPathMode::EXACT_STOP;
        const bool exact =
            KeyEqual(key, m_pendingKey) &&
            m_snapshot.legacySelected &&
            BypassIdentityExact(bypass) &&
            bypass.dispatchId != 0ULL &&
            bypass.dispatchId == evidence.dispatchId &&
            bypass.selected &&
            bypass.dispatchBound &&
            bypass.commitBound &&
            bypass.callbackRequired &&
            bypass.callbackActiveAtCommit &&
            bypass.legacyDrainRequired &&
            bypass.legacyDrainSatisfied &&
            bypass.runtimeInfluence &&
            bypass.resolverBypassed &&
            evidence.commitSucceeded &&
            evidence.commitSequence !=
            NC_PROGRAM_COMMIT_SEQUENCE_INVALID &&
            evidence.commitSequence == bypass.commitSequence &&
            evidence.waitCallbackActive &&
            evidence.submissionCount == 1U &&
            commandPathModeProven &&
            !evidence.captureOverflow &&
            evidence.producerAccepted &&
            evidence.immediateRejectNone &&
            evidence.segmentId != 0ULL &&
            epochCanAdvance &&
            evidence.currentExecutionEpoch == expectedEpoch &&
            evidence.segmentExecutionEpoch == expectedEpoch &&
            bypass.commitExecutionEpoch == expectedEpoch;

        if (!exact)
        {
            if (!commandPathModeProven)
            {
                ++m_counters.missingCommandPathMode;
            }
            FailMismatch();
            return;
        }

        m_snapshot.dispatchId = evidence.dispatchId;
        m_snapshot.commitSequence = evidence.commitSequence;
        m_snapshot.legacyCommitExecutionEpoch =
            evidence.currentExecutionEpoch;
        m_snapshot.legacyDispatchBound = true;
        m_snapshot.legacyCommitBound = true;
        m_snapshot.legacyCallbackObserved = true;
        m_snapshot.legacyEpochAdvanced = true;
        m_snapshot.commandPathModeProven = true;
        m_snapshot.blockerMask &=
            ~static_cast<std::uint32_t>(
                NC_ORDINARY_G00_BLOCKER_COMMAND_PATH_MODE);
        ++m_counters.legacyDispatchBound;
        ++m_counters.legacyCommitBound;
        Publish(NCOrdinaryG00AdmissionDecision::LEGACY_COMMITTED);
    }

    void ObserveLegacyCompletionSample(
        std::uint64_t dispatchId,
        const NCPreparedResolverBypassSnapshot& bypass,
        bool legacyReady,
        bool effectiveReady) noexcept
    {
        if (!m_pending || !m_snapshot.legacyCommitBound)
        {
            return;
        }

        ++m_counters.completionWaitSamples;
        const bool identityExact =
            dispatchId != 0ULL &&
            dispatchId == m_snapshot.dispatchId &&
            CommittedBypassIdentityExact(bypass) &&
            bypass.callbackRequired &&
            bypass.callbackActiveAtCommit &&
            bypass.accountingValid;
        if (!identityExact)
        {
            FailMismatch();
            return;
        }

        if (!effectiveReady)
        {
            Publish(NCOrdinaryG00AdmissionDecision::WAIT_COMPLETION);
            return;
        }

        if (!legacyReady ||
            !bypass.callbackCompletionObserved ||
            bypass.decision !=
            NCPreparedResolverBypassDecision::CALLBACK_COMPLETED)
        {
            FailMismatch();
            return;
        }

        m_snapshot.legacyCompleted = false;
        m_snapshot.legacyCallbackCompleted = true;
        ++m_counters.callbackCompleted;
        Publish(NCOrdinaryG00AdmissionDecision::CALLBACK_COMPLETED);
    }

    void ObserveUpstreamProof(
        const NCPreparedResolverBypassSnapshot& bypass) noexcept
    {
        if (!m_pending || !m_snapshot.legacyCallbackCompleted)
        {
            return;
        }

        const bool identityExact =
            CommittedBypassIdentityExact(bypass) &&
            bypass.accountingValid;
        if (!identityExact)
        {
            FailMismatch();
            return;
        }

        if (bypass.decision ==
            NCPreparedResolverBypassDecision::WAIT_UPSTREAM_PROOF)
        {
            Publish(
                NCOrdinaryG00AdmissionDecision::WAIT_UPSTREAM_PROOF);
            return;
        }

        if (bypass.decision !=
            NCPreparedResolverBypassDecision::PROOF_VERIFIED ||
            !bypass.upstreamProofVerified ||
            !bypass.callbackCompletionObserved)
        {
            FailMismatch();
            return;
        }

        m_snapshot.legacyUpstreamProofVerified = true;
        m_snapshot.legacyCompleted = true;
        m_snapshot.pending = false;
        m_pending = false;
        ++m_counters.legacyCompleted;
        Publish(NCOrdinaryG00AdmissionDecision::LEGACY_COMPLETED);
    }

    void ObserveRuntimeFailure(std::uint64_t dispatchId) noexcept
    {
        if (!m_pending)
        {
            return;
        }
        if (dispatchId != 0ULL &&
            m_snapshot.dispatchId != 0ULL &&
            dispatchId != m_snapshot.dispatchId)
        {
            return;
        }
        ++m_counters.runtimeFailures;
        ++m_counters.mismatches;
        m_permanentLockout = true;
        m_snapshot.permanentLockout = true;
        m_snapshot.pending = false;
        m_pending = false;
        Publish(NCOrdinaryG00AdmissionDecision::MISMATCHED);
    }

    void ObserveQueueInactive(
        NCPreparedInvalidationReason reason) noexcept
    {
        if (m_currentSession == NC_PREPARED_QUEUE_SESSION_INVALID ||
            m_inactivePublished)
        {
            return;
        }

        if (m_pending)
        {
            ++m_counters.invalidatedPending;
            m_pending = false;
            m_snapshot.pending = false;
        }

        ++m_counters.revocations;
        m_snapshot.lastRevocation = MapRevocation(reason);
        switch (m_snapshot.lastRevocation)
        {
        case NCOrdinaryG00AdmissionRevocation::ALARM:
            ++m_counters.alarmRevocations;
            break;
        case NCOrdinaryG00AdmissionRevocation::RESET:
            ++m_counters.resetRevocations;
            break;
        case NCOrdinaryG00AdmissionRevocation::PROGRAM_END:
            ++m_counters.programEndRevocations;
            break;
        case NCOrdinaryG00AdmissionRevocation::SOURCE_CHANGED:
            ++m_counters.sourceRevocations;
            break;
        case NCOrdinaryG00AdmissionRevocation::QUEUE_INACTIVE:
        case NCOrdinaryG00AdmissionRevocation::RUNTIME_FAILURE:
        case NCOrdinaryG00AdmissionRevocation::NONE:
        default:
            ++m_counters.queueRevocations;
            break;
        }
        m_inactivePublished = true;
        Publish(NCOrdinaryG00AdmissionDecision::REVOKED);
    }

    NCOrdinaryG00AdmissionSnapshot GetSnapshot() const noexcept
    {
        return m_snapshot;
    }

    NCOrdinaryG00AdmissionCounters GetCounters() const noexcept
    {
        return m_counters;
    }

private:
    struct TokenKey
    {
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
    };

    static constexpr std::size_t SEEN_CAPACITY = 32U;

    static std::uint8_t PanelMask(
        const NCPreparedPanelSwitchImage& panel) noexcept
    {
        return static_cast<std::uint8_t>(
            (panel.blockSkipEnabled ? 1U : 0U) |
            (panel.singleBlockEnabled ? 2U : 0U) |
            (panel.optionalStopEnabled ? 4U : 0U));
    }

    static TokenKey MakeKey(
        const NCPreparedHeadCutoverContext& context) noexcept
    {
        TokenKey key{};
        key.session = context.head.session;
        key.entrySequence = context.head.entrySequence;
        key.scope = context.head.source.scope;
        key.cacheGeneration = context.head.source.cacheGeneration;
        key.frameId = context.head.source.frameId;
        key.executionEpoch = context.head.source.executionEpoch;
        key.programFlowGeneration =
            context.head.source.programFlowGeneration;
        key.owner = context.head.source.owner;
        key.panelMask = PanelMask(context.head.source.panel);
        key.ownerGeneration = context.head.source.ownerGeneration;
        key.sourcePC = context.head.sourcePC;
        key.sourceLineNumber = context.head.sourceLineNumber;
        return key;
    }

    static bool KeyEqual(
        const TokenKey& lhs,
        const TokenKey& rhs) noexcept
    {
        return
            lhs.session == rhs.session &&
            lhs.entrySequence == rhs.entrySequence &&
            lhs.scope == rhs.scope &&
            lhs.cacheGeneration == rhs.cacheGeneration &&
            lhs.frameId == rhs.frameId &&
            lhs.executionEpoch == rhs.executionEpoch &&
            lhs.programFlowGeneration == rhs.programFlowGeneration &&
            lhs.owner == rhs.owner &&
            lhs.panelMask == rhs.panelMask &&
            lhs.ownerGeneration == rhs.ownerGeneration &&
            lhs.sourcePC == rhs.sourcePC &&
            lhs.sourceLineNumber == rhs.sourceLineNumber;
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

    bool BypassIdentityExact(
        const NCPreparedResolverBypassSnapshot& bypass) const noexcept
    {
        return
            bypass.lane == NCPreparedResolverBypassLane::G00_NO_P &&
            bypass.session == m_snapshot.session &&
            bypass.entrySequence == m_snapshot.entrySequence &&
            bypass.scope == m_snapshot.scope &&
            bypass.cacheGeneration == m_snapshot.cacheGeneration &&
            bypass.frameId == m_snapshot.frameId &&
            bypass.executionEpoch == m_snapshot.sourceExecutionEpoch &&
            bypass.programFlowGeneration ==
            m_snapshot.programFlowGeneration &&
            bypass.owner == m_snapshot.owner &&
            bypass.panelMask == m_snapshot.panelMask &&
            bypass.ownerGeneration == m_snapshot.ownerGeneration &&
            bypass.sourcePC == m_snapshot.sourcePC &&
            bypass.sourceLineNumber == m_snapshot.sourceLineNumber &&
            bypass.g00NoPQualifiedSession == m_snapshot.session &&
            bypass.enabled &&
            bypass.laneQualified &&
            bypass.queueExact &&
            bypass.upstreamHealthy &&
            bypass.tokenExact &&
            bypass.sourceExact &&
            bypass.pcLineExact &&
            bypass.modalExact &&
            bypass.classEligible &&
            bypass.literalRebuiltExact &&
            bypass.accountingValid &&
            !bypass.permanentLockout;
    }

    bool CommittedBypassIdentityExact(
        const NCPreparedResolverBypassSnapshot& bypass) const noexcept
    {
        return
            BypassIdentityExact(bypass) &&
            bypass.dispatchId == m_snapshot.dispatchId &&
            bypass.commitSequence == m_snapshot.commitSequence &&
            bypass.commitExecutionEpoch ==
            m_snapshot.legacyCommitExecutionEpoch &&
            bypass.dispatchBound &&
            bypass.commitBound;
    }

    static bool BaseEvidenceExact(
        const NCPreparedHeadCutoverContext& context,
        const NCPreparedResolverBypassSnapshot& bypass,
        bool configuredAxisPresent) noexcept
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
            SourceExact(context.head.source, context.runtimeSource) &&
            context.runtimeSource.scope == NCProgramScope::MEMORY &&
            context.runtimeSource.frameId ==
            NC_PROGRAM_FRAME_ID_INVALID &&
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
            NC_PREPARED_BARRIER_FLAG_NONE &&
            configuredAxisPresent &&
            bypass.session == context.head.session &&
            bypass.entrySequence == context.head.entrySequence &&
            bypass.sourcePC == context.sourcePC &&
            bypass.sourceLineNumber == context.sourceLineNumber &&
            bypass.executionEpoch ==
            context.runtimeSource.executionEpoch &&
            bypass.scope == context.runtimeSource.scope &&
            bypass.cacheGeneration ==
            context.runtimeSource.cacheGeneration &&
            bypass.frameId == context.runtimeSource.frameId &&
            bypass.programFlowGeneration ==
            context.runtimeSource.programFlowGeneration &&
            bypass.owner == context.runtimeSource.owner &&
            bypass.ownerGeneration ==
            context.runtimeSource.ownerGeneration &&
            bypass.panelMask == PanelMask(context.runtimeSource.panel) &&
            bypass.g00NoPQualifiedSession ==
            (bypass.laneQualified
                ? context.head.session
                : NC_PREPARED_QUEUE_SESSION_INVALID) &&
            bypass.queueExact &&
            bypass.upstreamHealthy &&
            bypass.tokenExact &&
            bypass.sourceExact &&
            bypass.pcLineExact &&
            bypass.modalExact &&
            bypass.classEligible &&
            bypass.literalRebuiltExact &&
            bypass.legacyDrainRequired &&
            !bypass.permanentLockout &&
            bypass.accountingValid;
    }

    static bool SimpleG90Envelope(
        const NCPreparedHeadCutoverContext& context) noexcept
    {
        const NCPreparedModalSnapshot& modal =
            context.runtimeModalBefore;
        return
            modal.imageValid &&
            modal.distanceMode == 90 &&
            modal.unitsMode == 21 &&
            modal.planeMode == 17 &&
            modal.workCoordinateCode == 54 &&
            // The target's accepted K.4.2 motion trace runs with G22 stored
            // stroke checking enabled.  Do not project a future cutover while
            // that machine-level safety envelope is disabled by G23.
            modal.storedStrokeMode == 22 &&
            modal.toolLengthMode == 49 &&
            modal.toolRadiusMode == 40 &&
            !modal.g68Active &&
            !modal.g168Active &&
            !modal.scalingActive &&
            modal.mirrorMask == 0U &&
            !modal.polarActive &&
            modal.cAxisOffsetRotationEnabled &&
            !modal.modalMacroActive;
    }

    bool HasSeen(const TokenKey& key) const noexcept
    {
        for (std::size_t index = 0U; index < m_seenCount; ++index)
        {
            if (KeyEqual(m_seen[index], key))
            {
                return true;
            }
        }
        return false;
    }

    bool RecordSeen(const TokenKey& key) noexcept
    {
        if (m_seenCount < SEEN_CAPACITY)
        {
            m_seen[m_seenCount++] = key;
            return true;
        }
        // A full diagnostic registry cannot affect Runtime.  It closes only
        // this shadow's readiness and retains the accepted K.4.2 path.  The
        // unrecorded token must not be recounted on every later scan.
        if (!m_seenRegistryOverflowed)
        {
            m_seenRegistryOverflowed = true;
            FailMismatch();
        }
        return false;
    }

    void ObserveActiveSession(NCPreparedQueueSession session) noexcept
    {
        if (session == NC_PREPARED_QUEUE_SESSION_INVALID)
        {
            return;
        }
        if (m_currentSession == session)
        {
            m_inactivePublished = false;
            return;
        }
        if (m_currentSession != NC_PREPARED_QUEUE_SESSION_INVALID &&
            !m_inactivePublished)
        {
            ObserveQueueInactive(
                NCPreparedInvalidationReason::SOURCE_REPLACED);
        }
        m_currentSession = session;
        m_inactivePublished = false;
        m_pending = false;
        m_seenCount = 0U;
        m_seenRegistryOverflowed = false;
        m_snapshot = NCOrdinaryG00AdmissionSnapshot{};
        m_snapshot.session = session;
        RefreshAccounting();
    }

    void CaptureIdentity(
        const NCPreparedHeadCutoverContext& context) noexcept
    {
        m_snapshot.session = context.head.session;
        m_snapshot.entrySequence = context.head.entrySequence;
        m_snapshot.scope = context.head.source.scope;
        m_snapshot.cacheGeneration = context.head.source.cacheGeneration;
        m_snapshot.frameId = context.head.source.frameId;
        m_snapshot.programFlowGeneration =
            context.head.source.programFlowGeneration;
        m_snapshot.owner = context.head.source.owner;
        m_snapshot.panelMask = PanelMask(context.head.source.panel);
        m_snapshot.ownerGeneration = context.head.source.ownerGeneration;
        m_snapshot.sourceExecutionEpoch =
            context.head.source.executionEpoch;
        m_snapshot.sourcePC = context.head.sourcePC;
        m_snapshot.sourceLineNumber = context.head.sourceLineNumber;
    }

    void ResetTokenSnapshot() noexcept
    {
        const std::uint64_t publicationSequence =
            m_snapshot.publicationSequence;
        m_snapshot = NCOrdinaryG00AdmissionSnapshot{};
        m_snapshot.publicationSequence = publicationSequence;
        m_snapshot.session = m_currentSession;
        m_snapshot.permanentLockout = m_permanentLockout;
        RefreshAccounting();
    }

    void StartCandidate(
        const NCPreparedHeadCutoverContext& context,
        const TokenKey& key,
        std::uint64_t queueDepth,
        bool groupStandstill,
        bool initialBusy) noexcept
    {
        CaptureIdentity(context);
        m_pendingKey = key;
        m_pending = true;
        m_snapshot.pending = true;
        m_snapshot.upstreamQualified = true;
        m_snapshot.simpleG90Envelope = true;
        m_snapshot.configuredAxisPresent = true;
        m_snapshot.projected = true;
        m_snapshot.initialQueueDepth = queueDepth;
        m_snapshot.initialGroupStandstill = groupStandstill;
        m_snapshot.initialBusy = initialBusy;
        m_snapshot.initialDrained = !initialBusy;
        m_snapshot.blockerMask = NC_ORDINARY_G00_REQUIRED_BLOCKERS;
        ++m_counters.projected;
        ++m_counters.candidates;
        if (initialBusy)
        {
            ++m_counters.initialBusy;
        }
        else
        {
            ++m_counters.initialDrained;
        }
        ++m_counters.missingTransactionalEndpoint;
        ++m_counters.missingInflightRegistry;
    }

    void FailMismatch() noexcept
    {
        ++m_counters.mismatches;
        m_permanentLockout = true;
        m_snapshot.permanentLockout = true;
        m_snapshot.pending = false;
        m_pending = false;
        Publish(NCOrdinaryG00AdmissionDecision::MISMATCHED);
    }

    static NCOrdinaryG00AdmissionRevocation MapRevocation(
        NCPreparedInvalidationReason reason) noexcept
    {
        switch (reason)
        {
        case NCPreparedInvalidationReason::ALARM:
            return NCOrdinaryG00AdmissionRevocation::ALARM;
        case NCPreparedInvalidationReason::RESET:
            return NCOrdinaryG00AdmissionRevocation::RESET;
        case NCPreparedInvalidationReason::PROGRAM_END:
            return NCOrdinaryG00AdmissionRevocation::PROGRAM_END;
        case NCPreparedInvalidationReason::SOURCE_REPLACED:
        case NCPreparedInvalidationReason::EXECUTION_EPOCH_CHANGED:
        case NCPreparedInvalidationReason::OWNER_CHANGED:
        case NCPreparedInvalidationReason::MODE_CHANGED:
        case NCPreparedInvalidationReason::FRAME_CHANGED:
        case NCPreparedInvalidationReason::PANEL_SWITCH_CHANGED:
        case NCPreparedInvalidationReason::CURSOR_DISCONTINUITY:
        case NCPreparedInvalidationReason::IDENTITY_INVALID:
            return NCOrdinaryG00AdmissionRevocation::SOURCE_CHANGED;
        case NCPreparedInvalidationReason::NONE:
        case NCPreparedInvalidationReason::NOT_RUNNING:
        default:
            return NCOrdinaryG00AdmissionRevocation::QUEUE_INACTIVE;
        }
    }

    void RefreshAccounting() noexcept
    {
        m_snapshot.shadowOnly = true;
        m_snapshot.runtimeInfluence = false;
        m_snapshot.resolverBypassed = false;
        m_snapshot.cutoverReady = false;
        m_snapshot.permanentLockout = m_permanentLockout;
        m_snapshot.accountingValid =
            m_counters.uniqueEvaluations ==
            m_counters.warmup +
            m_counters.candidates +
            m_counters.rejected &&
            m_counters.candidates ==
            m_counters.initialDrained +
            m_counters.initialBusy &&
            m_counters.projected == m_counters.candidates &&
            m_counters.legacySelections >=
            m_counters.legacyDispatchBound &&
            m_counters.legacyDispatchBound ==
            m_counters.legacyCommitBound &&
            m_counters.legacyCommitBound >=
            m_counters.callbackCompleted &&
            m_counters.callbackCompleted >=
            m_counters.legacyCompleted &&
            m_counters.completionWaitSamples >=
            m_counters.callbackCompleted &&
            m_counters.runtimeFailures <= m_counters.mismatches &&
            m_counters.invalidatedPending <= m_counters.revocations &&
            m_counters.revocations ==
            m_counters.queueRevocations +
            m_counters.alarmRevocations +
            m_counters.resetRevocations +
            m_counters.programEndRevocations +
            m_counters.sourceRevocations &&
            m_counters.missingCommandPathMode <=
            m_counters.candidates &&
            m_counters.missingTransactionalEndpoint ==
            m_counters.candidates &&
            m_counters.missingInflightRegistry ==
            m_counters.candidates &&
            m_counters.cutoverAttempts == 0ULL &&
            m_counters.runtimeInfluence == 0ULL &&
            m_counters.resolverBypasses == 0ULL;
    }

    void Publish(NCOrdinaryG00AdmissionDecision decision) noexcept
    {
        m_snapshot.decision = decision;
        RefreshAccounting();
        ++m_snapshot.publicationSequence;
        ++m_counters.publications;
    }

    NCOrdinaryG00AdmissionSnapshot m_snapshot{};
    NCOrdinaryG00AdmissionCounters m_counters{};
    NCPreparedQueueSession m_currentSession =
        NC_PREPARED_QUEUE_SESSION_INVALID;
    TokenKey m_pendingKey{};
    std::array<TokenKey, SEEN_CAPACITY> m_seen{};
    std::size_t m_seenCount = 0U;
    bool m_seenRegistryOverflowed = false;
    bool m_pending = false;
    bool m_inactivePublished = false;
    bool m_permanentLockout = false;
};

static_assert(
    std::is_trivially_copyable<NCOrdinaryG00AdmissionSnapshot>::value,
    "K.5 admission snapshot must remain trivially copyable.");
static_assert(
    std::is_trivially_copyable<NCOrdinaryG00AdmissionCounters>::value,
    "K.5 admission counters must remain trivially copyable.");
static_assert(
    sizeof(NCOrdinaryG00BufferedExactStopAdmissionShadow) <= 4096U,
    "K.5 admission shadow must remain bounded.");

inline const char* NCOrdinaryG00AdmissionDecisionToDiagnosticName(
    NCOrdinaryG00AdmissionDecision decision) noexcept
{
    switch (decision)
    {
    case NCOrdinaryG00AdmissionDecision::WARMUP: return "WARMUP";
    case NCOrdinaryG00AdmissionDecision::PROJECTED_DRAINED: return "DRAINED";
    case NCOrdinaryG00AdmissionDecision::PROJECTED_BUSY: return "BUSY";
    case NCOrdinaryG00AdmissionDecision::WAIT_BUSY: return "WAIT_BUSY";
    case NCOrdinaryG00AdmissionDecision::LEGACY_SELECTED: return "SELECTED";
    case NCOrdinaryG00AdmissionDecision::LEGACY_COMMITTED: return "COMMITTED";
    case NCOrdinaryG00AdmissionDecision::WAIT_COMPLETION: return "WAIT_DONE";
    case NCOrdinaryG00AdmissionDecision::CALLBACK_COMPLETED: return "CALLBACK_DONE";
    case NCOrdinaryG00AdmissionDecision::WAIT_UPSTREAM_PROOF: return "WAIT_PROOF";
    case NCOrdinaryG00AdmissionDecision::LEGACY_COMPLETED: return "COMPLETED";
    case NCOrdinaryG00AdmissionDecision::REJECTED: return "REJECTED";
    case NCOrdinaryG00AdmissionDecision::REVOKED: return "REVOKED";
    case NCOrdinaryG00AdmissionDecision::MISMATCHED: return "MISMATCHED";
    case NCOrdinaryG00AdmissionDecision::IDLE:
    default: return "IDLE";
    }
}

inline const char* NCOrdinaryG00AdmissionRevocationToDiagnosticName(
    NCOrdinaryG00AdmissionRevocation revocation) noexcept
{
    switch (revocation)
    {
    case NCOrdinaryG00AdmissionRevocation::QUEUE_INACTIVE: return "QUEUE_INACTIVE";
    case NCOrdinaryG00AdmissionRevocation::ALARM: return "ALARM";
    case NCOrdinaryG00AdmissionRevocation::RESET: return "RESET";
    case NCOrdinaryG00AdmissionRevocation::PROGRAM_END: return "PROGRAM_END";
    case NCOrdinaryG00AdmissionRevocation::SOURCE_CHANGED: return "SOURCE_CHANGED";
    case NCOrdinaryG00AdmissionRevocation::RUNTIME_FAILURE: return "RUNTIME_FAILURE";
    case NCOrdinaryG00AdmissionRevocation::NONE:
    default: return "NONE";
    }
}
