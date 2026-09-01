#pragma once

#include "NCPreparedHeadResolverBypassGate.h"
#include "MotionCommandPathModeTransport.h"
#include "MotionQueueTailTransaction.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

// =============================================================================
// Stage NC-0.2K.6.3 - Ordinary G00 Bounded In-Flight Terminal Registry Shadow
//
// The accepted K.4.2/K.6.2 Runtime path remains the sole producer and Motion
// remains the sole consumer/writer.  This fixed-capacity NC-thread observer
// binds one exact ordinary G00 Program Commit to its already-accepted Motion
// segment and then retains that identity until exactly one Ledger-accepted
// terminal feedback event is observed.
//
// It never advances a PC, changes a callback/Epoch/owner, submits Motion work,
// writes a PDO, allocates memory, waits, or authorises a read-ahead cutover.
// =============================================================================

constexpr std::size_t NC_ORDINARY_G00_INFLIGHT_REGISTRY_CAPACITY = 32U;

enum class NCOrdinaryG00InflightState : std::uint8_t
{
    K63_EMPTY = 0,
    K63_REGISTERED,
    K63_ACCEPTED,
    K63_STARTED,
    K63_ACTIVE,
    K63_HELD,
    K63_REVOKED_PENDING,
    K63_COMPLETED,
    K63_REJECTED,
    K63_CANCELLED,
    K63_ABORTED,
    K63_FAULTED
};

enum class NCOrdinaryG00InflightRevocation : std::uint8_t
{
    K63_NONE = 0,
    K63_QUEUE_INACTIVE,
    K63_ALARM,
    K63_RESET,
    K63_PROGRAM_END,
    K63_SOURCE_CHANGED,
    K63_RUNTIME_FAILURE
};

struct NCOrdinaryG00InflightRegistrationEvidence
{
    std::uint64_t dispatchId = 0ULL;
    NCProgramCommitSequence commitSequence =
        NC_PROGRAM_COMMIT_SEQUENCE_INVALID;
    std::uint64_t currentExecutionEpoch = 0ULL;
    std::uint64_t segmentExecutionEpoch = 0ULL;
    std::uint64_t segmentId = 0ULL;
    std::size_t submissionCount = 0U;
    MotionExecutionIdentity submissionIdentity{};
    MotionCommandPathMode commandPathMode =
        MotionCommandPathMode::UNSPECIFIED;
    MotionQueueTailCommitReceipt queueTailReceipt{};

    // Independent NC-0.2D Ledger binding captured after Program Commit.
    MotionExecutionIdentity ledgerIdentity{};
    MotionRejectReason ledgerImmediateRejectReason =
        MotionRejectReason::NONE;
    std::uint64_t ledgerDispatchId = 0ULL;
    NCProgramCommitSequence ledgerCommitSequence =
        NC_PROGRAM_COMMIT_SEQUENCE_INVALID;
    NCProgramScope ledgerScope = NCProgramScope::NONE;
    NCProgramCacheGeneration ledgerCacheGeneration =
        NC_PROGRAM_CACHE_GENERATION_INVALID;
    NCProgramFrameId ledgerFrameId = NC_PROGRAM_FRAME_ID_INVALID;
    int ledgerSourcePC = -1;
    int ledgerSourceLineNumber = 0;
    std::uint16_t ledgerMotionSegmentCount = 0U;
    bool ledgerFound = false;
    bool ledgerProgramCommitted = false;
    bool ledgerProducerAccepted = false;
    bool ledgerCaptureOverflow = false;

    bool captureOverflow = false;
    bool producerAccepted = false;
    bool immediateRejectNone = false;
    bool waitCallbackActive = false;
    bool commitSucceeded = false;
};

struct NCOrdinaryG00InflightRegistrationProof
{
    std::uint64_t registrySequence = 0ULL;
    NCPreparedQueueSession session = NC_PREPARED_QUEUE_SESSION_INVALID;
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
    std::uint64_t dispatchId = 0ULL;
    NCProgramCommitSequence commitSequence =
        NC_PROGRAM_COMMIT_SEQUENCE_INVALID;
    MotionExecutionIdentity identity{};
    MotionOwnerLease ownerLease{};
    MotionQueueTailTransactionSequence queueTailTransactionSequence =
        MOTION_QUEUE_TAIL_TRANSACTION_SEQUENCE_INVALID;
    std::uint32_t queueTailAxisMask = 0U;
    std::uint64_t queueTailBeforeFingerprint =
        MOTION_QUEUE_TAIL_FINGERPRINT_SEED;
    std::uint64_t queueTailCommittedFingerprint =
        MOTION_QUEUE_TAIL_FINGERPRINT_SEED;

    bool registered = false;
    bool exact = false;
    bool active = false;
    bool bounded = true;
    bool shadowOnly = true;
    bool runtimeInfluence = false;
    bool motionWrite = false;
    bool accountingValid = true;
};

struct NCOrdinaryG00InflightEntrySnapshot
{
    std::uint64_t registrySequence = 0ULL;
    NCPreparedQueueSession session = NC_PREPARED_QUEUE_SESSION_INVALID;
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
    std::uint64_t dispatchId = 0ULL;
    NCProgramCommitSequence commitSequence =
        NC_PROGRAM_COMMIT_SEQUENCE_INVALID;
    MotionExecutionIdentity identity{};
    MotionOwnerLease ownerLease{};
    MotionQueueTailCommitReceipt queueTailReceipt{};
    MotionFeedbackSequence lastFeedbackSequence =
        MOTION_FEEDBACK_SEQUENCE_INVALID;
    MotionFeedbackType lastFeedbackType = MotionFeedbackType::NONE;
    NCOrdinaryG00InflightState state =
        NCOrdinaryG00InflightState::K63_EMPTY;
    NCOrdinaryG00InflightRevocation revocation =
        NCOrdinaryG00InflightRevocation::K63_NONE;

    bool occupied = false;
    bool active = false;
    bool terminal = false;
    bool revoked = false;
    bool ledgerAcceptedTerminal = false;
};

struct NCOrdinaryG00InflightRegistrySnapshot
{
    std::uint64_t publicationSequence = 0ULL;
    std::uint64_t lastRegistrySequence = 0ULL;
    NCPreparedQueueSession currentSession =
        NC_PREPARED_QUEUE_SESSION_INVALID;
    std::uint32_t capacity =
        static_cast<std::uint32_t>(
            NC_ORDINARY_G00_INFLIGHT_REGISTRY_CAPACITY);
    std::uint32_t occupiedEntries = 0U;
    std::uint32_t activeEntries = 0U;
    std::uint32_t revokedPendingEntries = 0U;
    std::uint32_t peakActiveEntries = 0U;

    NCOrdinaryG00InflightEntrySnapshot lastEntry{};
    MotionFeedbackSequence lastObservedFeedbackSequence =
        MOTION_FEEDBACK_SEQUENCE_INVALID;

    bool ready = false;
    bool permanentLockout = false;
    bool bounded = true;
    bool shadowOnly = true;
    bool runtimeInfluence = false;
    bool motionWrite = false;
    bool accountingValid = true;
};

struct NCOrdinaryG00InflightRegistryCounters
{
    std::uint64_t registrationAttempts = 0ULL;
    std::uint64_t registered = 0ULL;
    std::uint64_t registrationRejected = 0ULL;
    std::uint64_t invalidRegistration = 0ULL;
    std::uint64_t duplicateIdentity = 0ULL;
    std::uint64_t capacityOverflow = 0ULL;
    std::uint64_t slotReuses = 0ULL;

    std::uint64_t feedbackObserved = 0ULL;
    std::uint64_t feedbackMatched = 0ULL;
    std::uint64_t feedbackIgnored = 0ULL;
    std::uint64_t accepted = 0ULL;
    std::uint64_t started = 0ULL;
    std::uint64_t progress = 0ULL;
    std::uint64_t held = 0ULL;
    std::uint64_t resumed = 0ULL;
    std::uint64_t terminal = 0ULL;
    std::uint64_t completed = 0ULL;
    std::uint64_t rejected = 0ULL;
    std::uint64_t cancelled = 0ULL;
    std::uint64_t aborted = 0ULL;
    std::uint64_t faulted = 0ULL;

    std::uint64_t invalidFeedbackSequence = 0ULL;
    std::uint64_t feedbackSequenceGaps = 0ULL;
    std::uint64_t activeSequenceGapFailures = 0ULL;
    std::uint64_t ledgerOrphans = 0ULL;
    std::uint64_t identityConflicts = 0ULL;
    std::uint64_t ownerMismatches = 0ULL;
    std::uint64_t duplicateTerminal = 0ULL;
    std::uint64_t terminalConflict = 0ULL;
    std::uint64_t postTerminalFeedback = 0ULL;

    std::uint64_t transportHealthChecks = 0ULL;
    std::uint64_t feedbackOverflowObserved = 0ULL;
    std::uint64_t producerNoticeOverflowObserved = 0ULL;
    std::uint64_t ledgerActiveBlockOverwriteObserved = 0ULL;
    std::uint64_t ledgerActiveSegmentIndexOverwriteObserved = 0ULL;
    std::uint64_t ledgerDuplicateObserved = 0ULL;
    std::uint64_t ledgerConflictObserved = 0ULL;

    std::uint64_t revocationCalls = 0ULL;
    std::uint64_t entriesRevoked = 0ULL;
    std::uint64_t queueRevocations = 0ULL;
    std::uint64_t alarmRevocations = 0ULL;
    std::uint64_t resetRevocations = 0ULL;
    std::uint64_t programEndRevocations = 0ULL;
    std::uint64_t sourceRevocations = 0ULL;
    std::uint64_t runtimeRevocations = 0ULL;
    std::uint64_t revokedTerminals = 0ULL;
    std::uint64_t runtimeFailures = 0ULL;
    std::uint64_t permanentFailures = 0ULL;

    std::uint64_t activeEntries = 0ULL;
    std::uint64_t peakActiveEntries = 0ULL;

    // These remain structural zeros in K.6.3.
    std::uint64_t runtimeInfluence = 0ULL;
    std::uint64_t motionWrites = 0ULL;
};

class NCOrdinaryG00InflightTerminalRegistryShadow
{
public:
    NCOrdinaryG00InflightTerminalRegistryShadow() noexcept = default;

    bool TryRegister(
        const NCPreparedHeadCutoverContext& context,
        const NCPreparedResolverBypassSnapshot& bypass,
        const NCOrdinaryG00InflightRegistrationEvidence& evidence,
        NCOrdinaryG00InflightRegistrationProof& proof) noexcept
    {
        proof = NCOrdinaryG00InflightRegistrationProof{};
        ++m_counters.registrationAttempts;

        ObserveActiveSessionInternal(context.queue.session);

        if (m_permanentLockout ||
            !RegistrationEvidenceExact(context, bypass, evidence))
        {
            RejectRegistration(RegistrationFailure::K63_INVALID);
            return false;
        }

        for (const NCOrdinaryG00InflightEntrySnapshot& entry : m_entries)
        {
            if (!entry.occupied)
            {
                continue;
            }
            if (SameEpochSegment(entry.identity, evidence.submissionIdentity))
            {
                RejectRegistration(RegistrationFailure::K63_DUPLICATE);
                return false;
            }
        }

        std::size_t target = NC_ORDINARY_G00_INFLIGHT_REGISTRY_CAPACITY;
        std::uint64_t oldestTerminalSequence =
            (std::numeric_limits<std::uint64_t>::max)();
        for (std::size_t index = 0U; index < m_entries.size(); ++index)
        {
            const NCOrdinaryG00InflightEntrySnapshot& entry =
                m_entries[index];
            if (!entry.occupied)
            {
                target = index;
                break;
            }
            if (!entry.active && entry.terminal &&
                entry.registrySequence < oldestTerminalSequence)
            {
                target = index;
                oldestTerminalSequence = entry.registrySequence;
            }
        }

        if (target >= m_entries.size())
        {
            RejectRegistration(RegistrationFailure::K63_OVERFLOW);
            return false;
        }

        if (m_entries[target].occupied)
        {
            ++m_counters.slotReuses;
        }

        NCOrdinaryG00InflightEntrySnapshot entry{};
        entry.registrySequence = AllocateRegistrySequence();
        entry.session = context.head.session;
        entry.entrySequence = context.head.entrySequence;
        entry.scope = context.head.source.scope;
        entry.cacheGeneration = context.head.source.cacheGeneration;
        entry.frameId = context.head.source.frameId;
        entry.sourceExecutionEpoch = context.head.source.executionEpoch;
        entry.programFlowGeneration =
            context.head.source.programFlowGeneration;
        entry.owner = context.head.source.owner;
        entry.panelMask = PanelMask(context.head.source.panel);
        entry.ownerGeneration = context.head.source.ownerGeneration;
        entry.sourcePC = context.head.sourcePC;
        entry.sourceLineNumber = context.head.sourceLineNumber;
        entry.dispatchId = evidence.dispatchId;
        entry.commitSequence = evidence.commitSequence;
        entry.identity = evidence.submissionIdentity;
        entry.ownerLease = evidence.queueTailReceipt.ownerLease;
        entry.queueTailReceipt = evidence.queueTailReceipt;
        entry.state = NCOrdinaryG00InflightState::K63_REGISTERED;
        entry.occupied = true;
        entry.active = true;

        m_entries[target] = entry;
        ++m_counters.registered;
        ++m_counters.activeEntries;
        if (m_counters.activeEntries > m_counters.peakActiveEntries)
        {
            m_counters.peakActiveEntries = m_counters.activeEntries;
        }
        m_lastEntryIndex = target;
        Publish();
        FillProof(m_entries[target], proof);
        return true;
    }

    void ObserveMotionFeedback(
        const MotionFeedbackEvent& event,
        bool ledgerAccepted) noexcept
    {
        ++m_counters.feedbackObserved;

        const bool sequenceValid =
            ObserveFeedbackSequence(event.sequence);
        if (!sequenceValid && m_counters.activeEntries != 0ULL)
        {
            ++m_counters.activeSequenceGapFailures;
            FailPermanent();
            Publish();
            return;
        }

        if (!ledgerAccepted && m_counters.activeEntries != 0ULL)
        {
            ++m_counters.ledgerOrphans;
            FailPermanent();
            Publish();
            return;
        }

        std::size_t exactIndex = m_entries.size();
        bool pairCollision = false;
        for (std::size_t index = 0U; index < m_entries.size(); ++index)
        {
            const NCOrdinaryG00InflightEntrySnapshot& entry =
                m_entries[index];
            if (!entry.occupied ||
                !SameEpochSegment(entry.identity, event.identity))
            {
                continue;
            }
            if (IdentityExactlyMatches(entry.identity, event.identity))
            {
                exactIndex = index;
                break;
            }
            pairCollision = true;
        }

        if (exactIndex >= m_entries.size())
        {
            if (pairCollision)
            {
                ++m_counters.identityConflicts;
                FailPermanent();
            }
            else
            {
                ++m_counters.feedbackIgnored;
            }
            Publish();
            return;
        }

        NCOrdinaryG00InflightEntrySnapshot& entry = m_entries[exactIndex];
        if (!ledgerAccepted)
        {
            ++m_counters.ledgerOrphans;
            FailPermanent();
            Publish();
            return;
        }
        if (event.owner != entry.ownerLease.owner ||
            event.ownerGeneration != entry.ownerLease.generation)
        {
            ++m_counters.ownerMismatches;
            FailPermanent();
            Publish();
            return;
        }

        const bool terminalEvent = IsTerminal(event.type);
        if (entry.terminal)
        {
            if (terminalEvent)
            {
                ++m_counters.duplicateTerminal;
                if (event.type != entry.lastFeedbackType)
                {
                    ++m_counters.terminalConflict;
                }
            }
            else
            {
                ++m_counters.postTerminalFeedback;
            }
            FailPermanent();
            Publish();
            return;
        }

        entry.lastFeedbackSequence = event.sequence;
        entry.lastFeedbackType = event.type;
        ++m_counters.feedbackMatched;

        switch (event.type)
        {
        case MotionFeedbackType::ACCEPTED:
            ++m_counters.accepted;
            entry.state = entry.revoked
                ? NCOrdinaryG00InflightState::K63_REVOKED_PENDING
                : NCOrdinaryG00InflightState::K63_ACCEPTED;
            break;
        case MotionFeedbackType::STARTED:
            ++m_counters.started;
            entry.state = entry.revoked
                ? NCOrdinaryG00InflightState::K63_REVOKED_PENDING
                : NCOrdinaryG00InflightState::K63_STARTED;
            break;
        case MotionFeedbackType::PROGRESS:
            ++m_counters.progress;
            entry.state = entry.revoked
                ? NCOrdinaryG00InflightState::K63_REVOKED_PENDING
                : NCOrdinaryG00InflightState::K63_ACTIVE;
            break;
        case MotionFeedbackType::HELD:
            ++m_counters.held;
            entry.state = entry.revoked
                ? NCOrdinaryG00InflightState::K63_REVOKED_PENDING
                : NCOrdinaryG00InflightState::K63_HELD;
            break;
        case MotionFeedbackType::RESUMED:
            ++m_counters.resumed;
            entry.state = entry.revoked
                ? NCOrdinaryG00InflightState::K63_REVOKED_PENDING
                : NCOrdinaryG00InflightState::K63_ACTIVE;
            break;
        case MotionFeedbackType::COMPLETED:
        case MotionFeedbackType::REJECTED:
        case MotionFeedbackType::CANCELLED:
        case MotionFeedbackType::ABORTED:
        case MotionFeedbackType::FAULTED:
            CompleteTerminal(entry, event.type);
            break;
        case MotionFeedbackType::NONE:
        default:
            // NONE is not a lifecycle event and cannot provide proof.
            --m_counters.feedbackMatched;
            ++m_counters.identityConflicts;
            FailPermanent();
            break;
        }

        m_lastEntryIndex = exactIndex;
        Publish();
    }

    void ObserveTransportHealth(
        std::uint64_t feedbackOverflow,
        std::uint64_t producerNoticeOverflow,
        std::uint64_t ncSequenceGaps,
        std::uint64_t ledgerActiveBlockOverwrites,
        std::uint64_t ledgerActiveSegmentIndexOverwrites,
        std::uint64_t ledgerOrphans,
        std::uint64_t ledgerDuplicates,
        std::uint64_t ledgerConflicts) noexcept
    {
        ++m_counters.transportHealthChecks;
        const bool unhealthy =
            feedbackOverflow != 0ULL ||
            producerNoticeOverflow != 0ULL ||
            ncSequenceGaps != 0ULL ||
            ledgerActiveBlockOverwrites != 0ULL ||
            ledgerActiveSegmentIndexOverwrites != 0ULL ||
            ledgerOrphans != 0ULL ||
            ledgerDuplicates != 0ULL ||
            ledgerConflicts != 0ULL;
        if (!unhealthy)
        {
            RefreshSnapshot();
            return;
        }

        const std::uint64_t mergedSequenceGaps =
            (m_counters.feedbackSequenceGaps < ncSequenceGaps)
            ? ncSequenceGaps
            : m_counters.feedbackSequenceGaps;
        const std::uint64_t mergedLedgerOrphans =
            (m_counters.ledgerOrphans < ledgerOrphans)
            ? ledgerOrphans
            : m_counters.ledgerOrphans;
        const bool evidenceChanged =
            m_counters.feedbackOverflowObserved != feedbackOverflow ||
            m_counters.producerNoticeOverflowObserved !=
            producerNoticeOverflow ||
            m_counters.feedbackSequenceGaps != mergedSequenceGaps ||
            m_counters.ledgerActiveBlockOverwriteObserved !=
            ledgerActiveBlockOverwrites ||
            m_counters.ledgerActiveSegmentIndexOverwriteObserved !=
            ledgerActiveSegmentIndexOverwrites ||
            m_counters.ledgerOrphans != mergedLedgerOrphans ||
            m_counters.ledgerDuplicateObserved != ledgerDuplicates ||
            m_counters.ledgerConflictObserved != ledgerConflicts;
        if (!evidenceChanged && m_permanentLockout)
        {
            RefreshSnapshot();
            return;
        }

        m_counters.feedbackOverflowObserved = feedbackOverflow;
        m_counters.producerNoticeOverflowObserved =
            producerNoticeOverflow;
        m_counters.ledgerActiveBlockOverwriteObserved =
            ledgerActiveBlockOverwrites;
        m_counters.ledgerActiveSegmentIndexOverwriteObserved =
            ledgerActiveSegmentIndexOverwrites;
        m_counters.feedbackSequenceGaps = mergedSequenceGaps;
        m_counters.ledgerOrphans = mergedLedgerOrphans;
        m_counters.ledgerDuplicateObserved = ledgerDuplicates;
        m_counters.ledgerConflictObserved = ledgerConflicts;
        FailPermanent();
        Publish();
    }

    void ObserveActiveSession(NCPreparedQueueSession session) noexcept
    {
        if (ObserveActiveSessionInternal(session))
        {
            Publish();
        }
    }

    void ObserveLiveSource(
        const NCPreparedSourceIdentity& source) noexcept
    {
        bool changed = false;
        for (NCOrdinaryG00InflightEntrySnapshot& entry : m_entries)
        {
            if (!entry.occupied || !entry.active || entry.revoked)
            {
                continue;
            }
            // Multiple committed G00 entries may remain in flight while the
            // same program/source identity advances through later execution
            // epochs.  That monotonic advance is not a source replacement;
            // a backward/reset epoch or any cache/frame/flow/owner/panel
            // change is.
            const bool executionEpochCompatible =
                source.executionEpoch >= entry.sourceExecutionEpoch &&
                source.executionEpoch <=
                static_cast<std::uint64_t>(UINT32_MAX);
            const bool exact =
                source.scope == entry.scope &&
                source.cacheGeneration == entry.cacheGeneration &&
                source.frameId == entry.frameId &&
                executionEpochCompatible &&
                source.programFlowGeneration ==
                entry.programFlowGeneration &&
                source.owner == entry.owner &&
                source.ownerGeneration == entry.ownerGeneration &&
                PanelMask(source.panel) == entry.panelMask;
            if (exact)
            {
                continue;
            }
            entry.revoked = true;
            entry.revocation =
                NCOrdinaryG00InflightRevocation::K63_SOURCE_CHANGED;
            entry.state = NCOrdinaryG00InflightState::K63_REVOKED_PENDING;
            ++m_counters.entriesRevoked;
            ++m_counters.sourceRevocations;
            changed = true;
        }
        if (changed)
        {
            ++m_counters.revocationCalls;
            Publish();
        }
    }

    void ObserveQueueInactive(NCPreparedInvalidationReason reason) noexcept
    {
        const NCOrdinaryG00InflightRevocation revocation =
            MapRevocation(reason);
        if (RevokeEntries(revocation, 0ULL, false))
        {
            ++m_counters.revocationCalls;
            Publish();
        }
    }

    void ObserveRuntimeFailure(std::uint64_t dispatchId) noexcept
    {
        const bool globalFailure = dispatchId == 0ULL;
        bool activeMatch = false;
        for (const NCOrdinaryG00InflightEntrySnapshot& entry : m_entries)
        {
            activeMatch = activeMatch ||
                (entry.occupied && entry.active && !entry.revoked &&
                    (globalFailure || entry.dispatchId == dispatchId));
        }
        if (!activeMatch)
        {
            return;
        }
        const bool revoked = RevokeEntries(
            NCOrdinaryG00InflightRevocation::K63_RUNTIME_FAILURE,
            dispatchId,
            !globalFailure);
        ++m_counters.runtimeFailures;
        if (revoked)
        {
            ++m_counters.revocationCalls;
        }
        FailPermanent();
        Publish();
    }

    NCOrdinaryG00InflightRegistrySnapshot GetSnapshot() const noexcept
    {
        return m_snapshot;
    }

    NCOrdinaryG00InflightRegistryCounters GetCounters() const noexcept
    {
        return m_counters;
    }

    bool TryGetEntry(
        std::size_t slot,
        NCOrdinaryG00InflightEntrySnapshot& entry) const noexcept
    {
        if (slot >= m_entries.size() || !m_entries[slot].occupied)
        {
            entry = NCOrdinaryG00InflightEntrySnapshot{};
            return false;
        }
        entry = m_entries[slot];
        return true;
    }

private:
    enum class RegistrationFailure : std::uint8_t
    {
        K63_INVALID = 0,
        K63_DUPLICATE,
        K63_OVERFLOW
    };

    static std::uint8_t PanelMask(
        const NCPreparedPanelSwitchImage& panel) noexcept
    {
        return static_cast<std::uint8_t>(
            (panel.blockSkipEnabled ? 1U : 0U) |
            (panel.singleBlockEnabled ? 2U : 0U) |
            (panel.optionalStopEnabled ? 4U : 0U));
    }

    static bool IdentityExactlyMatches(
        const MotionExecutionIdentity& lhs,
        const MotionExecutionIdentity& rhs) noexcept
    {
        return
            lhs.epoch == rhs.epoch &&
            lhs.segmentId == rhs.segmentId &&
            lhs.sourceBlockId == rhs.sourceBlockId &&
            lhs.source == rhs.source;
    }

    static bool SameEpochSegment(
        const MotionExecutionIdentity& lhs,
        const MotionExecutionIdentity& rhs) noexcept
    {
        return
            lhs.epoch == rhs.epoch &&
            lhs.segmentId == rhs.segmentId;
    }

    static bool SourceExactlyMatches(
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

    static bool QueueTailExactlyMatches(
        const MotionQueueTailCommitReceipt& receipt,
        const MotionExecutionIdentity& identity,
        std::uint8_t owner,
        std::uint64_t ownerGeneration) noexcept
    {
        return
            receipt.IsCommitted() &&
            IdentityExactlyMatches(receipt.identity, identity) &&
            receipt.ownerLease.owner == MotionOwner::AUTO &&
            static_cast<std::uint8_t>(receipt.ownerLease.owner) == owner &&
            static_cast<std::uint64_t>(receipt.ownerLease.generation) ==
            ownerGeneration &&
            receipt.captureBound &&
            receipt.transactionSequence !=
            MOTION_QUEUE_TAIL_TRANSACTION_SEQUENCE_INVALID &&
            receipt.axisMask != 0U;
    }

    static bool RegistrationEvidenceExact(
        const NCPreparedHeadCutoverContext& context,
        const NCPreparedResolverBypassSnapshot& bypass,
        const NCOrdinaryG00InflightRegistrationEvidence& evidence) noexcept
    {
        if (context.runtimeSource.executionEpoch >=
            static_cast<std::uint64_t>(UINT32_MAX))
        {
            return false;
        }
        const std::uint64_t expectedExecutionEpoch =
            context.runtimeSource.executionEpoch + 1ULL;
        const MotionExecutionIdentity& identity =
            evidence.submissionIdentity;
        const bool identityExact =
            identity.IsAssigned() &&
            static_cast<std::uint64_t>(identity.epoch) ==
            expectedExecutionEpoch &&
            static_cast<std::uint64_t>(identity.epoch) ==
            evidence.segmentExecutionEpoch &&
            static_cast<std::uint64_t>(identity.segmentId) ==
            evidence.segmentId &&
            identity.sourceBlockId ==
            static_cast<MotionSourceBlockId>(context.sourcePC) &&
            identity.source == MotionCommandSource::NC_MEMORY;
        const bool ledgerExact =
            evidence.ledgerFound &&
            evidence.ledgerProgramCommitted &&
            !evidence.ledgerCaptureOverflow &&
            evidence.ledgerDispatchId == evidence.dispatchId &&
            evidence.ledgerCommitSequence == evidence.commitSequence &&
            evidence.ledgerScope == context.runtimeSource.scope &&
            evidence.ledgerCacheGeneration ==
            context.runtimeSource.cacheGeneration &&
            evidence.ledgerFrameId == context.runtimeSource.frameId &&
            evidence.ledgerSourcePC == context.sourcePC &&
            evidence.ledgerSourceLineNumber == context.sourceLineNumber &&
            evidence.ledgerMotionSegmentCount == 1U &&
            evidence.ledgerProducerAccepted &&
            evidence.ledgerImmediateRejectReason ==
            MotionRejectReason::NONE &&
            IdentityExactlyMatches(evidence.ledgerIdentity, identity);

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
            SourceExactlyMatches(context.queue.source, context.runtimeSource) &&
            context.head.entrySequence !=
            NC_PREPARED_ENTRY_SEQUENCE_INVALID &&
            context.head.sourcePC == context.sourcePC &&
            context.head.sourceLineNumber == context.sourceLineNumber &&
            SourceExactlyMatches(context.head.source, context.runtimeSource) &&
            context.runtimeSource.scope == NCProgramScope::MEMORY &&
            context.runtimeSource.frameId == NC_PROGRAM_FRAME_ID_INVALID &&
            context.runtimeSource.owner ==
            static_cast<std::uint8_t>(MotionOwner::AUTO) &&
            context.runtimeSource.ownerGeneration != 0ULL &&
            context.runtimeSource.ownerGeneration <=
            static_cast<std::uint64_t>(UINT32_MAX) &&
            !context.runtimeSource.panel.blockSkipEnabled &&
            !context.runtimeSource.panel.singleBlockEnabled &&
            !context.runtimeSource.panel.optionalStopEnabled &&
            context.legacyDrainRequired &&
            context.legacyDrainSatisfied &&
            context.head.classification.blockClass ==
            NCPreparedBlockClass::MOTION_SHADOW &&
            context.head.classification.primaryGCode == 0 &&
            context.head.classification.literalResolved &&
            context.head.classification.modalAfterValid &&
            context.head.classification.legacyDrainRequired &&
            !context.head.classification.planningStopsHere &&
            context.head.classification.barrierKind ==
            NCPreparedBarrierKind::NONE &&
            context.head.classification.barrierFlags ==
            NC_PREPARED_BARRIER_FLAG_NONE &&
            bypass.lane == NCPreparedResolverBypassLane::G00_NO_P &&
            bypass.session == context.head.session &&
            bypass.entrySequence == context.head.entrySequence &&
            bypass.scope == context.runtimeSource.scope &&
            bypass.cacheGeneration == context.runtimeSource.cacheGeneration &&
            bypass.frameId == context.runtimeSource.frameId &&
            bypass.executionEpoch == context.runtimeSource.executionEpoch &&
            bypass.programFlowGeneration ==
            context.runtimeSource.programFlowGeneration &&
            bypass.owner == context.runtimeSource.owner &&
            bypass.ownerGeneration == context.runtimeSource.ownerGeneration &&
            bypass.panelMask == PanelMask(context.runtimeSource.panel) &&
            bypass.sourcePC == context.sourcePC &&
            bypass.sourceLineNumber == context.sourceLineNumber &&
            bypass.g00NoPQualifiedSession == context.head.session &&
            bypass.enabled &&
            bypass.selected &&
            bypass.dispatchBound &&
            bypass.commitBound &&
            bypass.callbackRequired &&
            bypass.callbackActiveAtCommit &&
            bypass.legacyDrainRequired &&
            bypass.legacyDrainSatisfied &&
            bypass.runtimeInfluence &&
            bypass.resolverBypassed &&
            bypass.preparedValueSelected &&
            !bypass.legacyResolverRetained &&
            bypass.queueExact &&
            bypass.upstreamHealthy &&
            bypass.tokenExact &&
            bypass.sourceExact &&
            bypass.pcLineExact &&
            bypass.modalExact &&
            bypass.classEligible &&
            bypass.literalRebuiltExact &&
            bypass.accountingValid &&
            !bypass.permanentLockout &&
            evidence.dispatchId != 0ULL &&
            bypass.dispatchId == evidence.dispatchId &&
            evidence.commitSequence !=
            NC_PROGRAM_COMMIT_SEQUENCE_INVALID &&
            bypass.commitSequence == evidence.commitSequence &&
            evidence.currentExecutionEpoch == expectedExecutionEpoch &&
            bypass.commitExecutionEpoch == expectedExecutionEpoch &&
            evidence.submissionCount == 1U &&
            evidence.commandPathMode == MotionCommandPathMode::EXACT_STOP &&
            identityExact &&
            QueueTailExactlyMatches(
                evidence.queueTailReceipt,
                identity,
                context.runtimeSource.owner,
                context.runtimeSource.ownerGeneration) &&
            ledgerExact &&
            !evidence.captureOverflow &&
            evidence.producerAccepted &&
            evidence.immediateRejectNone &&
            evidence.waitCallbackActive &&
            evidence.commitSucceeded;
    }

    static bool IsTerminal(MotionFeedbackType type) noexcept
    {
        return
            type == MotionFeedbackType::COMPLETED ||
            type == MotionFeedbackType::REJECTED ||
            type == MotionFeedbackType::CANCELLED ||
            type == MotionFeedbackType::ABORTED ||
            type == MotionFeedbackType::FAULTED;
    }

    static NCOrdinaryG00InflightState TerminalState(
        MotionFeedbackType type) noexcept
    {
        switch (type)
        {
        case MotionFeedbackType::COMPLETED:
            return NCOrdinaryG00InflightState::K63_COMPLETED;
        case MotionFeedbackType::REJECTED:
            return NCOrdinaryG00InflightState::K63_REJECTED;
        case MotionFeedbackType::CANCELLED:
            return NCOrdinaryG00InflightState::K63_CANCELLED;
        case MotionFeedbackType::ABORTED:
            return NCOrdinaryG00InflightState::K63_ABORTED;
        case MotionFeedbackType::FAULTED:
        default:
            return NCOrdinaryG00InflightState::K63_FAULTED;
        }
    }

    static NCOrdinaryG00InflightRevocation MapRevocation(
        NCPreparedInvalidationReason reason) noexcept
    {
        switch (reason)
        {
        case NCPreparedInvalidationReason::ALARM:
            return NCOrdinaryG00InflightRevocation::K63_ALARM;
        case NCPreparedInvalidationReason::RESET:
            return NCOrdinaryG00InflightRevocation::K63_RESET;
        case NCPreparedInvalidationReason::PROGRAM_END:
            return NCOrdinaryG00InflightRevocation::K63_PROGRAM_END;
        case NCPreparedInvalidationReason::SOURCE_REPLACED:
        case NCPreparedInvalidationReason::EXECUTION_EPOCH_CHANGED:
        case NCPreparedInvalidationReason::OWNER_CHANGED:
        case NCPreparedInvalidationReason::MODE_CHANGED:
        case NCPreparedInvalidationReason::FRAME_CHANGED:
        case NCPreparedInvalidationReason::PANEL_SWITCH_CHANGED:
        case NCPreparedInvalidationReason::CURSOR_DISCONTINUITY:
        case NCPreparedInvalidationReason::IDENTITY_INVALID:
            return NCOrdinaryG00InflightRevocation::K63_SOURCE_CHANGED;
        case NCPreparedInvalidationReason::NONE:
        case NCPreparedInvalidationReason::NOT_RUNNING:
        default:
            return NCOrdinaryG00InflightRevocation::K63_QUEUE_INACTIVE;
        }
    }

    void FillProof(
        const NCOrdinaryG00InflightEntrySnapshot& entry,
        NCOrdinaryG00InflightRegistrationProof& proof) const noexcept
    {
        proof.registrySequence = entry.registrySequence;
        proof.session = entry.session;
        proof.entrySequence = entry.entrySequence;
        proof.scope = entry.scope;
        proof.cacheGeneration = entry.cacheGeneration;
        proof.frameId = entry.frameId;
        proof.sourceExecutionEpoch = entry.sourceExecutionEpoch;
        proof.programFlowGeneration = entry.programFlowGeneration;
        proof.owner = entry.owner;
        proof.panelMask = entry.panelMask;
        proof.ownerGeneration = entry.ownerGeneration;
        proof.sourcePC = entry.sourcePC;
        proof.sourceLineNumber = entry.sourceLineNumber;
        proof.dispatchId = entry.dispatchId;
        proof.commitSequence = entry.commitSequence;
        proof.identity = entry.identity;
        proof.ownerLease = entry.ownerLease;
        proof.queueTailTransactionSequence =
            entry.queueTailReceipt.transactionSequence;
        proof.queueTailAxisMask = entry.queueTailReceipt.axisMask;
        proof.queueTailBeforeFingerprint =
            entry.queueTailReceipt.beforeFingerprint;
        proof.queueTailCommittedFingerprint =
            entry.queueTailReceipt.committedFingerprint;
        proof.registered = true;
        proof.exact = true;
        proof.active = true;
        proof.accountingValid = m_snapshot.accountingValid;
    }

    void CompleteTerminal(
        NCOrdinaryG00InflightEntrySnapshot& entry,
        MotionFeedbackType type) noexcept
    {
        entry.state = TerminalState(type);
        entry.active = false;
        entry.terminal = true;
        entry.ledgerAcceptedTerminal = true;
        ++m_counters.terminal;
        if (m_counters.activeEntries != 0ULL)
        {
            --m_counters.activeEntries;
        }
        if (entry.revoked)
        {
            ++m_counters.revokedTerminals;
        }

        switch (type)
        {
        case MotionFeedbackType::COMPLETED: ++m_counters.completed; break;
        case MotionFeedbackType::REJECTED: ++m_counters.rejected; break;
        case MotionFeedbackType::CANCELLED: ++m_counters.cancelled; break;
        case MotionFeedbackType::ABORTED: ++m_counters.aborted; break;
        case MotionFeedbackType::FAULTED: ++m_counters.faulted; break;
        default: break;
        }
    }

    bool ObserveFeedbackSequence(MotionFeedbackSequence sequence) noexcept
    {
        if (sequence == MOTION_FEEDBACK_SEQUENCE_INVALID)
        {
            ++m_counters.invalidFeedbackSequence;
            return false;
        }
        bool valid = true;
        if (m_lastObservedFeedbackSequence !=
            MOTION_FEEDBACK_SEQUENCE_INVALID)
        {
            const MotionFeedbackSequence expected =
                m_lastObservedFeedbackSequence ==
                (std::numeric_limits<MotionFeedbackSequence>::max)()
                ? 1ULL
                : m_lastObservedFeedbackSequence + 1ULL;
            if (sequence != expected)
            {
                ++m_counters.feedbackSequenceGaps;
                valid = false;
            }
        }
        m_lastObservedFeedbackSequence = sequence;
        return valid;
    }

    bool ObserveActiveSessionInternal(
        NCPreparedQueueSession session) noexcept
    {
        if (session == NC_PREPARED_QUEUE_SESSION_INVALID ||
            session == m_currentSession)
        {
            return false;
        }
        const bool hadSession =
            m_currentSession != NC_PREPARED_QUEUE_SESSION_INVALID;
        m_currentSession = session;
        if (!hadSession)
        {
            return true;
        }
        const bool revoked = RevokeEntries(
            NCOrdinaryG00InflightRevocation::K63_SOURCE_CHANGED,
            0ULL,
            false,
            session);
        if (revoked)
        {
            ++m_counters.revocationCalls;
        }
        return true;
    }

    bool RevokeEntries(
        NCOrdinaryG00InflightRevocation revocation,
        std::uint64_t dispatchId,
        bool filterDispatch,
        NCPreparedQueueSession keepSession =
        NC_PREPARED_QUEUE_SESSION_INVALID) noexcept
    {
        bool changed = false;
        for (NCOrdinaryG00InflightEntrySnapshot& entry : m_entries)
        {
            if (!entry.occupied || !entry.active || entry.revoked ||
                (filterDispatch && entry.dispatchId != dispatchId) ||
                (keepSession != NC_PREPARED_QUEUE_SESSION_INVALID &&
                    entry.session == keepSession))
            {
                continue;
            }
            entry.revoked = true;
            entry.revocation = revocation;
            entry.state = NCOrdinaryG00InflightState::K63_REVOKED_PENDING;
            ++m_counters.entriesRevoked;
            IncrementRevocationCounter(revocation);
            changed = true;
        }
        return changed;
    }

    void IncrementRevocationCounter(
        NCOrdinaryG00InflightRevocation revocation) noexcept
    {
        switch (revocation)
        {
        case NCOrdinaryG00InflightRevocation::K63_ALARM:
            ++m_counters.alarmRevocations;
            break;
        case NCOrdinaryG00InflightRevocation::K63_RESET:
            ++m_counters.resetRevocations;
            break;
        case NCOrdinaryG00InflightRevocation::K63_PROGRAM_END:
            ++m_counters.programEndRevocations;
            break;
        case NCOrdinaryG00InflightRevocation::K63_SOURCE_CHANGED:
            ++m_counters.sourceRevocations;
            break;
        case NCOrdinaryG00InflightRevocation::K63_RUNTIME_FAILURE:
            ++m_counters.runtimeRevocations;
            break;
        case NCOrdinaryG00InflightRevocation::K63_QUEUE_INACTIVE:
        case NCOrdinaryG00InflightRevocation::K63_NONE:
        default:
            ++m_counters.queueRevocations;
            break;
        }
    }

    void RejectRegistration(RegistrationFailure failure) noexcept
    {
        ++m_counters.registrationRejected;
        switch (failure)
        {
        case RegistrationFailure::K63_DUPLICATE:
            ++m_counters.duplicateIdentity;
            break;
        case RegistrationFailure::K63_OVERFLOW:
            ++m_counters.capacityOverflow;
            break;
        case RegistrationFailure::K63_INVALID:
        default:
            ++m_counters.invalidRegistration;
            break;
        }
        FailPermanent();
        Publish();
    }

    void FailPermanent() noexcept
    {
        if (!m_permanentLockout)
        {
            ++m_counters.permanentFailures;
        }
        m_permanentLockout = true;
    }

    std::uint64_t AllocateRegistrySequence() noexcept
    {
        const std::uint64_t sequence = m_nextRegistrySequence;
        ++m_nextRegistrySequence;
        if (m_nextRegistrySequence == 0ULL)
        {
            m_nextRegistrySequence = 1ULL;
        }
        return sequence;
    }

    void RefreshSnapshot() noexcept
    {
        std::uint32_t occupied = 0U;
        std::uint32_t active = 0U;
        std::uint32_t revokedPending = 0U;
        for (const NCOrdinaryG00InflightEntrySnapshot& entry : m_entries)
        {
            if (!entry.occupied)
            {
                continue;
            }
            ++occupied;
            if (entry.active)
            {
                ++active;
                if (entry.revoked)
                {
                    ++revokedPending;
                }
            }
        }

        m_snapshot.currentSession = m_currentSession;
        m_snapshot.occupiedEntries = occupied;
        m_snapshot.activeEntries = active;
        m_snapshot.revokedPendingEntries = revokedPending;
        m_snapshot.peakActiveEntries = static_cast<std::uint32_t>(
            m_counters.peakActiveEntries);
        m_snapshot.lastObservedFeedbackSequence =
            m_lastObservedFeedbackSequence;
        m_snapshot.permanentLockout = m_permanentLockout;
        m_snapshot.bounded = true;
        m_snapshot.shadowOnly = true;
        m_snapshot.runtimeInfluence = false;
        m_snapshot.motionWrite = false;
        if (m_lastEntryIndex < m_entries.size())
        {
            m_snapshot.lastEntry = m_entries[m_lastEntryIndex];
            m_snapshot.lastRegistrySequence =
                m_entries[m_lastEntryIndex].registrySequence;
        }

        const std::uint64_t revocationSum =
            m_counters.queueRevocations +
            m_counters.alarmRevocations +
            m_counters.resetRevocations +
            m_counters.programEndRevocations +
            m_counters.sourceRevocations +
            m_counters.runtimeRevocations;
        const std::uint64_t matchedSum =
            m_counters.accepted +
            m_counters.started +
            m_counters.progress +
            m_counters.held +
            m_counters.resumed +
            m_counters.terminal;
        const std::uint64_t terminalSum =
            m_counters.completed +
            m_counters.rejected +
            m_counters.cancelled +
            m_counters.aborted +
            m_counters.faulted;
        m_snapshot.accountingValid =
            m_counters.registrationAttempts ==
            m_counters.registered +
            m_counters.registrationRejected &&
            m_counters.registrationRejected ==
            m_counters.invalidRegistration +
            m_counters.duplicateIdentity +
            m_counters.capacityOverflow &&
            m_counters.registered ==
            m_counters.terminal + m_counters.activeEntries &&
            m_counters.feedbackMatched == matchedSum &&
            m_counters.terminal == terminalSum &&
            m_counters.entriesRevoked == revocationSum &&
            m_counters.revokedTerminals <= m_counters.entriesRevoked &&
            m_counters.activeEntries == active &&
            revokedPending <= active &&
            m_counters.peakActiveEntries >= m_counters.activeEntries &&
            m_counters.peakActiveEntries <=
            NC_ORDINARY_G00_INFLIGHT_REGISTRY_CAPACITY &&
            m_counters.runtimeInfluence == 0ULL &&
            m_counters.motionWrites == 0ULL;
        m_snapshot.ready =
            m_counters.registered != 0ULL &&
            !m_permanentLockout &&
            m_snapshot.accountingValid;
    }

    void Publish() noexcept
    {
        RefreshSnapshot();
        ++m_snapshot.publicationSequence;
    }

    std::array<NCOrdinaryG00InflightEntrySnapshot,
        NC_ORDINARY_G00_INFLIGHT_REGISTRY_CAPACITY> m_entries{};
    NCOrdinaryG00InflightRegistrySnapshot m_snapshot{};
    NCOrdinaryG00InflightRegistryCounters m_counters{};
    NCPreparedQueueSession m_currentSession =
        NC_PREPARED_QUEUE_SESSION_INVALID;
    MotionFeedbackSequence m_lastObservedFeedbackSequence =
        MOTION_FEEDBACK_SEQUENCE_INVALID;
    std::uint64_t m_nextRegistrySequence = 1ULL;
    std::size_t m_lastEntryIndex =
        NC_ORDINARY_G00_INFLIGHT_REGISTRY_CAPACITY;
    bool m_permanentLockout = false;
};

static_assert(
    std::is_trivially_copyable<NCOrdinaryG00InflightRegistrationProof>::value,
    "K.6.3 registration proof must remain trivially copyable.");
static_assert(
    std::is_trivially_copyable<NCOrdinaryG00InflightEntrySnapshot>::value,
    "K.6.3 registry entry must remain trivially copyable.");
static_assert(
    std::is_trivially_copyable<NCOrdinaryG00InflightRegistrySnapshot>::value,
    "K.6.3 registry snapshot must remain trivially copyable.");
static_assert(
    sizeof(NCOrdinaryG00InflightTerminalRegistryShadow) <= 16384U,
    "K.6.3 registry must remain fixed and bounded.");

inline const char* NCOrdinaryG00InflightStateToDiagnosticName(
    NCOrdinaryG00InflightState state) noexcept
{
    switch (state)
    {
    case NCOrdinaryG00InflightState::K63_REGISTERED: return "REGISTERED";
    case NCOrdinaryG00InflightState::K63_ACCEPTED: return "ACCEPTED";
    case NCOrdinaryG00InflightState::K63_STARTED: return "STARTED";
    case NCOrdinaryG00InflightState::K63_ACTIVE: return "ACTIVE";
    case NCOrdinaryG00InflightState::K63_HELD: return "HELD";
    case NCOrdinaryG00InflightState::K63_REVOKED_PENDING: return "REVOKED_PENDING";
    case NCOrdinaryG00InflightState::K63_COMPLETED: return "COMPLETED";
    case NCOrdinaryG00InflightState::K63_REJECTED: return "REJECTED";
    case NCOrdinaryG00InflightState::K63_CANCELLED: return "CANCELLED";
    case NCOrdinaryG00InflightState::K63_ABORTED: return "ABORTED";
    case NCOrdinaryG00InflightState::K63_FAULTED: return "FAULTED";
    case NCOrdinaryG00InflightState::K63_EMPTY:
    default: return "EMPTY";
    }
}

inline const char* NCOrdinaryG00InflightRevocationToDiagnosticName(
    NCOrdinaryG00InflightRevocation revocation) noexcept
{
    switch (revocation)
    {
    case NCOrdinaryG00InflightRevocation::K63_QUEUE_INACTIVE: return "QUEUE_INACTIVE";
    case NCOrdinaryG00InflightRevocation::K63_ALARM: return "ALARM";
    case NCOrdinaryG00InflightRevocation::K63_RESET: return "RESET";
    case NCOrdinaryG00InflightRevocation::K63_PROGRAM_END: return "PROGRAM_END";
    case NCOrdinaryG00InflightRevocation::K63_SOURCE_CHANGED: return "SOURCE_CHANGED";
    case NCOrdinaryG00InflightRevocation::K63_RUNTIME_FAILURE: return "RUNTIME_FAILURE";
    case NCOrdinaryG00InflightRevocation::K63_NONE:
    default: return "NONE";
    }
}
