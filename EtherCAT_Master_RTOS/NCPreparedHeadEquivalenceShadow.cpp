#include "NCPreparedHeadEquivalenceShadow.h"

#include <cstring>

namespace
{
    bool BoolEqual(bool left, bool right) noexcept
    {
        return left == right;
    }

    bool DoubleBitsEqual(double left, double right) noexcept
    {
        std::uint64_t leftBits = 0ULL;
        std::uint64_t rightBits = 0ULL;
        static_assert(
            sizeof(leftBits) == sizeof(left),
            "K.2 assumes an IEEE-754-sized double value.");
        std::memcpy(&leftBits, &left, sizeof(leftBits));
        std::memcpy(&rightBits, &right, sizeof(rightBits));
        return leftBits == rightBits;
    }

    bool PanelEqual(
        const NCPreparedPanelSwitchImage& left,
        const NCPreparedPanelSwitchImage& right) noexcept
    {
        return
            left.blockSkipEnabled == right.blockSkipEnabled &&
            left.singleBlockEnabled == right.singleBlockEnabled &&
            left.optionalStopEnabled == right.optionalStopEnabled;
    }

    std::uint8_t PanelMask(
        const NCPreparedPanelSwitchImage& panel) noexcept
    {
        return static_cast<std::uint8_t>(
            (panel.blockSkipEnabled ? 1U : 0U) |
            (panel.singleBlockEnabled ? 2U : 0U) |
            (panel.optionalStopEnabled ? 4U : 0U));
    }

    bool IsOrdinaryAbortingG00(
        const NCPreparedBlockEntrySnapshot& entry) noexcept
    {
        return
            entry.classification.blockClass ==
            NCPreparedBlockClass::MOTION_SHADOW &&
            entry.classification.primaryGCode == 0 &&
            !(entry.preparedBlock.has('P') &&
                entry.preparedBlock.val('P') == 1.0);
    }

    NCPreparedHeadEquivalenceInvalidation MapInvalidation(
        NCPreparedInvalidationReason reason) noexcept
    {
        switch (reason)
        {
        case NCPreparedInvalidationReason::ALARM:
            return NCPreparedHeadEquivalenceInvalidation::ALARM;
        case NCPreparedInvalidationReason::RESET:
            return NCPreparedHeadEquivalenceInvalidation::RESET;
        case NCPreparedInvalidationReason::PROGRAM_END:
            return NCPreparedHeadEquivalenceInvalidation::PROGRAM_END;
        case NCPreparedInvalidationReason::EXECUTION_EPOCH_CHANGED:
            return NCPreparedHeadEquivalenceInvalidation::EPOCH_CHANGED;
        case NCPreparedInvalidationReason::OWNER_CHANGED:
            return NCPreparedHeadEquivalenceInvalidation::OWNER_CHANGED;
        case NCPreparedInvalidationReason::FRAME_CHANGED:
            return NCPreparedHeadEquivalenceInvalidation::FRAME_CHANGED;
        case NCPreparedInvalidationReason::PANEL_SWITCH_CHANGED:
            return NCPreparedHeadEquivalenceInvalidation::PANEL_CHANGED;
        case NCPreparedInvalidationReason::SOURCE_REPLACED:
        case NCPreparedInvalidationReason::MODE_CHANGED:
        case NCPreparedInvalidationReason::CURSOR_DISCONTINUITY:
        case NCPreparedInvalidationReason::IDENTITY_INVALID:
            return NCPreparedHeadEquivalenceInvalidation::SOURCE_CHANGED;
        case NCPreparedInvalidationReason::NOT_RUNNING:
        case NCPreparedInvalidationReason::NONE:
        default:
            return NCPreparedHeadEquivalenceInvalidation::QUEUE_INACTIVE;
        }
    }
}

bool NCPreparedHeadEquivalenceShadow::SourceIdentityExact(
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
        PanelEqual(left.panel, right.panel);
}

bool NCPreparedHeadEquivalenceShadow::ProgramTargetMatchesEntry(
    const NCProgramCommitSnapshot& target,
    const NCPreparedBlockEntrySnapshot& entry) noexcept
{
    return
        target.scope == entry.source.scope &&
        target.cacheGeneration == entry.source.cacheGeneration &&
        target.frameId == entry.source.frameId &&
        target.sourcePC == entry.sourcePC;
}

bool NCPreparedHeadEquivalenceShadow::SemanticBlockEqual(
    const NCBlock& left,
    const NCBlock& right) noexcept
{
    if (!BoolEqual(left.isEmpty, right.isEmpty) ||
        !BoolEqual(left.isBlockSkip, right.isBlockSkip) ||
        !BoolEqual(left.isGoto, right.isGoto) ||
        left.gotoTarget != right.gotoTarget ||
        left.gCount != right.gCount ||
        !BoolEqual(left.hasG, right.hasG) ||
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

    for (int index = 0; index < left.gCount; ++index)
    {
        if (left.gCodes[index] != right.gCodes[index])
        {
            return false;
        }
    }

    for (int index = 0; index < left.mCount; ++index)
    {
        if (left.mCode[index] != right.mCode[index])
        {
            return false;
        }
    }

    for (int index = 0; index < 26; ++index)
    {
        if (left.hasParam[index] != right.hasParam[index])
        {
            return false;
        }
        if (left.hasParam[index] &&
            !DoubleBitsEqual(left.param[index], right.param[index]))
        {
            return false;
        }
    }

    return true;
}

bool NCPreparedHeadEquivalenceShadow::StorageBlockEqual(
    const NCBlock& left,
    const NCBlock& right) noexcept
{
    if (!BoolEqual(left.isEmpty, right.isEmpty) ||
        !BoolEqual(left.isBlockSkip, right.isBlockSkip) ||
        !BoolEqual(left.isGoto, right.isGoto) ||
        left.gotoTarget != right.gotoTarget ||
        left.gCount != right.gCount ||
        !BoolEqual(left.hasG, right.hasG) ||
        left.gCode != right.gCode ||
        left.mCount != right.mCount)
    {
        return false;
    }

    for (int index = 0; index < NC_MAX_G_CODES_PER_BLOCK; ++index)
    {
        if (left.gCodes[index] != right.gCodes[index])
        {
            return false;
        }
    }

    for (int index = 0; index < NC_MAX_M_CODES_PER_BLOCK; ++index)
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

bool NCPreparedHeadEquivalenceShadow::ExecutionPlanEqual(
    const NCBlock& left,
    const NCBlock& right) noexcept
{
    NCGCodeExecutionPlan leftPlan{};
    NCGCodeExecutionPlan rightPlan{};
    NCGCodePlanError leftError = NCGCodePlanError::NONE;
    NCGCodePlanError rightError = NCGCodePlanError::NONE;
    int leftFirstConflict = -1;
    int leftSecondConflict = -1;
    int rightFirstConflict = -1;
    int rightSecondConflict = -1;

    const bool leftValid = NCGCodeSemantics::BuildExecutionPlan(
        left,
        leftPlan,
        leftError,
        leftFirstConflict,
        leftSecondConflict);
    const bool rightValid = NCGCodeSemantics::BuildExecutionPlan(
        right,
        rightPlan,
        rightError,
        rightFirstConflict,
        rightSecondConflict);

    if (leftValid != rightValid ||
        leftError != rightError ||
        leftFirstConflict != rightFirstConflict ||
        leftSecondConflict != rightSecondConflict ||
        leftPlan.count != rightPlan.count ||
        leftPlan.hasPrimaryAction != rightPlan.hasPrimaryAction ||
        leftPlan.primaryActionCode != rightPlan.primaryActionCode)
    {
        return false;
    }

    if (leftPlan.count < 0 ||
        leftPlan.count > NC_MAX_G_CODES_PER_BLOCK)
    {
        return false;
    }

    for (int index = 0; index < leftPlan.count; ++index)
    {
        if (leftPlan.orderedCodes[index] != rightPlan.orderedCodes[index])
        {
            return false;
        }
    }

    return true;
}

bool NCPreparedHeadEquivalenceShadow::ClassificationEqual(
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

bool NCPreparedHeadEquivalenceShadow::ModalEqual(
    const NCPreparedModalSnapshot& prepared,
    const NCPreparedModalSnapshot& live,
    bool compareG00Override) noexcept
{
    if (!prepared.imageValid || !live.imageValid ||
        prepared.distanceMode != live.distanceMode ||
        prepared.unitsMode != live.unitsMode ||
        prepared.planeMode != live.planeMode ||
        prepared.workCoordinateCode != live.workCoordinateCode ||
            !SameNCTranslationSnapshot(prepared.translation, live.translation) ||
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
        !DoubleBitsEqual(prepared.scalingFactor, live.scalingFactor) ||
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

    // G00 override is normally an operator-owned live input.  It is compared
    // only when this exact block carries F and therefore claims a new scalar
    // post-image.  Merely observing the live knob never freezes it.

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

bool NCPreparedHeadEquivalenceShadow::CandidateClassEligible(
    const NCPreparedBlockEntrySnapshot& entry,
    const NCPreparedSourceIdentity& runtimeSource,
    const NCPreparedBlockQueueSnapshot& queue) noexcept
{
    if (!queue.active ||
        !queue.valid ||
        !queue.cursorOrderValid ||
        !queue.accountingValid ||
        !queue.shadowOnly ||
        queue.committedBaselinePlanningBlocked ||
        runtimeSource.scope != NCProgramScope::MEMORY ||
        runtimeSource.frameId != NC_PROGRAM_FRAME_ID_INVALID ||
        runtimeSource.panel.blockSkipEnabled ||
        runtimeSource.panel.singleBlockEnabled ||
        runtimeSource.panel.optionalStopEnabled ||
        !entry.classification.literalResolved ||
        !entry.classification.modalAfterValid ||
        entry.classification.planningStopsHere ||
        entry.classification.barrierFlags !=
        NC_PREPARED_BARRIER_FLAG_NONE ||
        entry.modalBefore.modalMacroActive ||
        entry.preparedBlock.isGoto ||
        entry.preparedBlock.mCount != 0 ||
        entry.preparedBlock.has('T'))
    {
        return false;
    }

    switch (entry.classification.blockClass)
    {
    case NCPreparedBlockClass::EMPTY:
    case NCPreparedBlockClass::PURE_MODAL_COPY:
        return true;
    case NCPreparedBlockClass::MOTION_SHADOW:
        return entry.classification.primaryGCode == 0;
    case NCPreparedBlockClass::NONE:
    case NCPreparedBlockClass::BLOCK_SKIP:
    case NCPreparedBlockClass::PROGRAM_CONTROL:
    case NCPreparedBlockClass::AUXILIARY:
    case NCPreparedBlockClass::INVALID:
    default:
        return false;
    }
}

bool NCPreparedHeadEquivalenceShadow::UpstreamCountersHealthy(
    const NCPreparedBlockQueueCounters& counters) noexcept
{
    return
        counters.cursorRegressions == 0ULL &&
        counters.planDiscontinuities == 0ULL &&
        counters.dispatchMismatches == 0ULL &&
        counters.commitMismatches == 0ULL &&
        counters.staleRuntimeProofs == 0ULL &&
        counters.identityFailures == 0ULL &&
        counters.cutoverAttempts == 0ULL;
}

bool NCPreparedHeadEquivalenceShadow::SourceIdentityStableForCommit(
    const NCPreparedSourceIdentity& prepared,
    const NCPreparedSourceIdentity& live,
    const NCPreparedBlockEntrySnapshot& entry) noexcept
{
    const bool immutableIdentityMatch =
        prepared.scope == live.scope &&
        prepared.cacheGeneration == live.cacheGeneration &&
        prepared.frameId == live.frameId &&
        prepared.programFlowGeneration == live.programFlowGeneration &&
        prepared.owner == live.owner &&
        prepared.ownerGeneration == live.ownerGeneration &&
        PanelEqual(prepared.panel, live.panel);
    if (!immutableIdentityMatch)
    {
        return false;
    }

    if (prepared.executionEpoch == live.executionEpoch)
    {
        return true;
    }

    return
        IsOrdinaryAbortingG00(entry) &&
        prepared.executionEpoch != UINT64_MAX &&
        live.executionEpoch == prepared.executionEpoch + 1ULL;
}

bool NCPreparedHeadEquivalenceShadow::SameToken(
    const NCPreparedBlockEntrySnapshot& entry) const noexcept
{
    return
        m_snapshot.session == entry.session &&
        m_snapshot.entrySequence == entry.entrySequence &&
        m_snapshot.sourcePC == entry.sourcePC &&
        m_snapshot.sourceLineNumber == entry.sourceLineNumber;
}

bool NCPreparedHeadEquivalenceShadow::RevalidateBoundPreparedValue(
    const NCPreparedBlockEntrySnapshot& head,
    const NCBlock& legacyBlock) const noexcept
{
    return
        m_snapshot.state == NCPreparedHeadEquivalenceState::PENDING &&
        m_snapshot.candidate &&
        m_snapshot.pending &&
        m_snapshot.resolved &&
        m_snapshot.dispatchBound &&
        !m_snapshot.commitBound &&
        SameToken(head) &&
        SourceIdentityExact(m_entry.source, head.source) &&
        ClassificationEqual(
            m_entry.classification,
            head.classification) &&
        StorageBlockEqual(
            m_entry.preparedBlock,
            head.preparedBlock) &&
        StorageBlockEqual(head.preparedBlock, legacyBlock) &&
        ExecutionPlanEqual(head.preparedBlock, legacyBlock);
}

void NCPreparedHeadEquivalenceShadow::BeginSession(
    NCPreparedQueueSession session,
    NCPreparedInvalidationReason transitionReason) noexcept
{
    if (m_observedSession != NC_PREPARED_QUEUE_SESSION_INVALID &&
        m_observedSession != session)
    {
        ++m_counters.sessionTransitions;
    }

    m_observedSession = session;
    m_sessionCandidates = 0ULL;
    m_sessionMatched = 0ULL;
    m_sessionMismatched = 0ULL;
    m_sessionStale = 0ULL;
    m_sessionRuntimeFailures = 0ULL;
    m_sessionPeakDepth = 0U;
    m_queueActive = true;
    m_upstreamHealthy = true;
    m_sessionQualificationLatched = false;
    m_sessionUpstreamFailureLatched = false;

    if (transitionReason != NCPreparedInvalidationReason::NONE)
    {
        m_snapshot.lastInvalidation = MapInvalidation(transitionReason);
    }
}

void NCPreparedHeadEquivalenceShadow::BeginToken(
    const NCPreparedBlockEntrySnapshot& entry,
    std::uint32_t preparedDepth) noexcept
{
    const std::uint32_t lifetimePeak = static_cast<std::uint32_t>(
        m_counters.lifetimePeakDepth);
    const NCPreparedHeadEquivalenceInvalidation lastInvalidation =
        m_snapshot.lastInvalidation;
    m_entry = entry;
    m_upstreamDispatchCommitObserved = false;
    m_snapshot = NCPreparedHeadEquivalenceSnapshot{};
    m_snapshot.lastInvalidation = lastInvalidation;
    m_snapshot.state = NCPreparedHeadEquivalenceState::PENDING;
    m_snapshot.session = entry.session;
    m_snapshot.entrySequence = entry.entrySequence;
    m_snapshot.scope = entry.source.scope;
    m_snapshot.cacheGeneration = entry.source.cacheGeneration;
    m_snapshot.frameId = entry.source.frameId;
    m_snapshot.executionEpoch = entry.source.executionEpoch;
    m_snapshot.programFlowGeneration =
        entry.source.programFlowGeneration;
    m_snapshot.owner = entry.source.owner;
    m_snapshot.panelMask = PanelMask(entry.source.panel);
    m_snapshot.ownerGeneration = entry.source.ownerGeneration;
    m_snapshot.sourcePC = entry.sourcePC;
    m_snapshot.sourceLineNumber = entry.sourceLineNumber;
    m_snapshot.blockClass = entry.classification.blockClass;
    m_snapshot.barrierKind = entry.classification.barrierKind;
    m_snapshot.preparedDepth = preparedDepth;
    m_snapshot.sessionPeakDepth = m_sessionPeakDepth;
    m_snapshot.lifetimePeakDepth = lifetimePeak;
}

void NCPreparedHeadEquivalenceShadow::RecordMismatch(
    std::uint32_t flags) noexcept
{
    if (!m_snapshot.candidate || !m_snapshot.pending)
    {
        return;
    }

    m_snapshot.mismatchFlags |= flags;
    m_snapshot.state = NCPreparedHeadEquivalenceState::MISMATCHED;
    m_snapshot.pending = false;
    m_snapshot.matched = false;
    ++m_counters.mismatched;
    ++m_sessionMismatched;

    if ((flags & NC_PREPARED_EQUIVALENCE_MISMATCH_QUEUE) != 0U)
        ++m_counters.queueMismatches;
    if ((flags & NC_PREPARED_EQUIVALENCE_MISMATCH_SOURCE) != 0U)
        ++m_counters.sourceMismatches;
    if ((flags & NC_PREPARED_EQUIVALENCE_MISMATCH_PC) != 0U)
        ++m_counters.pcMismatches;
    if ((flags & NC_PREPARED_EQUIVALENCE_MISMATCH_LINE) != 0U)
        ++m_counters.lineMismatches;
    if ((flags & NC_PREPARED_EQUIVALENCE_MISMATCH_BLOCK) != 0U)
        ++m_counters.blockMismatches;
    if ((flags & NC_PREPARED_EQUIVALENCE_MISMATCH_PLAN) != 0U)
        ++m_counters.planMismatches;
    if ((flags & NC_PREPARED_EQUIVALENCE_MISMATCH_CLASSIFICATION) != 0U)
        ++m_counters.classificationMismatches;
    if ((flags & NC_PREPARED_EQUIVALENCE_MISMATCH_DRAIN) != 0U)
        ++m_counters.drainMismatches;
    if ((flags & NC_PREPARED_EQUIVALENCE_MISMATCH_MODAL_BEFORE) != 0U)
        ++m_counters.modalBeforeMismatches;
    if ((flags & NC_PREPARED_EQUIVALENCE_MISMATCH_MODAL_AFTER) != 0U)
        ++m_counters.modalAfterMismatches;
    if ((flags & NC_PREPARED_EQUIVALENCE_MISMATCH_LIFECYCLE) != 0U)
        ++m_counters.lifecycleMismatches;
    if ((flags & NC_PREPARED_EQUIVALENCE_MISMATCH_STALE) != 0U)
    {
        ++m_counters.staleTokens;
        ++m_sessionStale;
    }
    if ((flags & NC_PREPARED_EQUIVALENCE_MISMATCH_RESOLVE) != 0U)
        ++m_counters.resolveMismatches;
    if ((flags & NC_PREPARED_EQUIVALENCE_MISMATCH_UPSTREAM) != 0U)
        ++m_counters.upstreamMismatches;
    if ((flags & NC_PREPARED_EQUIVALENCE_MISMATCH_LEDGER) != 0U)
        ++m_counters.ledgerMismatches;
    if ((flags & NC_PREPARED_EQUIVALENCE_MISMATCH_BIND_IDENTITY) != 0U)
        ++m_counters.bindIdentityMismatches;
}

void NCPreparedHeadEquivalenceShadow::InvalidatePending(
    NCPreparedHeadEquivalenceInvalidation reason,
    bool stale) noexcept
{
    if (!m_snapshot.candidate || !m_snapshot.pending)
    {
        return;
    }

    m_snapshot.state = NCPreparedHeadEquivalenceState::INVALIDATED;
    m_snapshot.lastInvalidation = reason;
    m_snapshot.pending = false;
    m_snapshot.matched = false;
    ++m_counters.invalidated;
    if (stale)
    {
        m_snapshot.mismatchFlags |=
            NC_PREPARED_EQUIVALENCE_MISMATCH_STALE;
        ++m_counters.staleTokens;
        ++m_sessionStale;
    }
}

void NCPreparedHeadEquivalenceShadow::RefreshPublication() noexcept
{
    m_snapshot.pending =
        m_snapshot.state == NCPreparedHeadEquivalenceState::PENDING &&
        m_snapshot.candidate;
    m_snapshot.lifetimePeakDepth = static_cast<std::uint32_t>(
        m_counters.lifetimePeakDepth);
    m_snapshot.sessionPeakDepth = m_sessionPeakDepth;

    const std::uint64_t pending = m_snapshot.pending ? 1ULL : 0ULL;
    m_snapshot.accountingValid =
        m_counters.candidates ==
        m_counters.matched +
        m_counters.mismatched +
        m_counters.invalidated +
        pending &&
        m_counters.useAttempts == 0ULL &&
        m_counters.cutoverAttempts == 0ULL &&
        m_counters.runtimeInfluence == 0ULL;
    const bool currentSessionQualified =
        m_snapshot.accountingValid &&
        m_queueActive &&
        m_observedSession != NC_PREPARED_QUEUE_SESSION_INVALID &&
        m_snapshot.session == m_observedSession &&
        !m_snapshot.pending &&
        m_sessionMatched > 0ULL &&
        m_sessionMismatched == 0ULL &&
        m_sessionStale == 0ULL &&
        m_sessionRuntimeFailures == 0ULL &&
        m_counters.mismatched == 0ULL &&
        m_counters.staleTokens == 0ULL &&
        m_counters.upstreamMismatches == 0ULL &&
        m_counters.runtimeFailures == 0ULL &&
        m_sessionPeakDepth > 1U &&
        m_upstreamHealthy;
    m_snapshot.readinessQualified = currentSessionQualified;
    m_snapshot.qualifiedSession =
        currentSessionQualified
        ? m_observedSession
        : NC_PREPARED_QUEUE_SESSION_INVALID;
    if (currentSessionQualified && !m_sessionQualificationLatched)
    {
        ++m_counters.qualifiedSessions;
        m_sessionQualificationLatched = true;
    }
    m_snapshot.shadowOnly = true;
    m_snapshot.runtimeInfluence = false;
    m_snapshot.cutoverApplied = false;
    m_snapshot.publicationSequence = m_nextPublicationSequence++;
    if (m_nextPublicationSequence == 0ULL)
    {
        m_nextPublicationSequence = 1ULL;
    }
    ++m_counters.publications;
}

bool NCPreparedHeadEquivalenceShadow::PrepareCandidateToken(
    const NCPreparedBlockQueueSnapshot& queue,
    const NCPreparedBlockQueueCounters& queueCounters,
    bool hasHead,
    const NCPreparedBlockEntrySnapshot& head,
    const NCPreparedSourceIdentity& runtimeSource,
    int sourcePC,
    int sourceLineNumber) noexcept
{
    ++m_counters.observations;
    if (queue.depth > m_counters.lifetimePeakDepth)
    {
        m_counters.lifetimePeakDepth = queue.depth;
    }

    const bool changingSession =
        queue.active &&
        queue.session != NC_PREPARED_QUEUE_SESSION_INVALID &&
        queue.session != m_observedSession;
    if (changingSession)
    {
        const bool hadSession =
            m_observedSession != NC_PREPARED_QUEUE_SESSION_INVALID;
        NCPreparedInvalidationReason transitionReason =
            hadSession
            ? queue.lastInvalidationReason
            : NCPreparedInvalidationReason::NONE;
        if (m_snapshot.pending)
        {
            if (transitionReason == NCPreparedInvalidationReason::NONE)
            {
                InvalidatePending(
                    NCPreparedHeadEquivalenceInvalidation::TOKEN_REPLACED,
                    true);
            }
            else
            {
                InvalidatePending(
                    MapInvalidation(transitionReason),
                    false);
            }
        }
        BeginSession(queue.session, transitionReason);
    }

    if (queue.active && queue.session == m_observedSession)
    {
        m_queueActive = true;
        if (queue.depth > m_sessionPeakDepth)
        {
            m_sessionPeakDepth = queue.depth;
        }
    }

    const bool queueBaseValid =
        queue.active &&
        queue.valid &&
        queue.cursorOrderValid &&
        queue.accountingValid &&
        queue.shadowOnly &&
        queue.capacity == NC_PREPARED_BLOCK_QUEUE_CAPACITY &&
        queue.depth <= queue.capacity &&
        queueCounters.commitMatched <= queueCounters.dispatchMatched &&
        queueCounters.dispatchMatched <= queueCounters.prepared &&
        queueCounters.prepared ==
        queueCounters.retired +
        queueCounters.invalidatedEntries +
        static_cast<std::uint64_t>(queue.depth);
    const bool upstreamCountersValid =
        UpstreamCountersHealthy(queueCounters);

    if (!hasHead || !queueBaseValid)
    {
        const bool unexpectedQueueFailure =
            !queueBaseValid || (hasHead != (queue.depth > 0U));
        if (unexpectedQueueFailure)
        {
            m_upstreamHealthy = false;
            if (m_snapshot.pending)
            {
                RecordMismatch(
                    NC_PREPARED_EQUIVALENCE_MISMATCH_QUEUE |
                    NC_PREPARED_EQUIVALENCE_MISMATCH_UPSTREAM);
                m_sessionUpstreamFailureLatched = true;
            }
            else if (!m_sessionUpstreamFailureLatched)
            {
                ++m_counters.queueMismatches;
                ++m_counters.upstreamMismatches;
                m_sessionUpstreamFailureLatched = true;
            }
        }
        ++m_counters.queueBypasses;
        RefreshPublication();
        return false;
    }

    if (!SameToken(head))
    {
        if (m_snapshot.pending)
        {
            InvalidatePending(
                NCPreparedHeadEquivalenceInvalidation::TOKEN_REPLACED,
                true);
        }
        BeginToken(head, queue.depth);
        if (queue.depth > 1U)
        {
            ++m_counters.multiBlockObservations;
        }
    }
    else
    {
        ++m_counters.replays;
        if (m_snapshot.state != NCPreparedHeadEquivalenceState::PENDING)
        {
            RefreshPublication();
            return false;
        }
    }

    m_snapshot.lifetimePeakDepth = static_cast<std::uint32_t>(
        m_counters.lifetimePeakDepth);

    const bool queueHeadCoherent =
        queue.depth > 0U &&
        queue.session != NC_PREPARED_QUEUE_SESSION_INVALID &&
        head.session != NC_PREPARED_QUEUE_SESSION_INVALID &&
        head.entrySequence != NC_PREPARED_ENTRY_SEQUENCE_INVALID &&
        queue.session == head.session &&
        queue.session == m_observedSession &&
        queueCounters.sessions == queue.session &&
        !head.dispatchObserved &&
        !head.commitObserved &&
        SourceIdentityExact(queue.source, head.source);

    if (!queueHeadCoherent || !upstreamCountersValid)
    {
        if (!m_snapshot.candidate)
        {
            m_snapshot.candidate = true;
            m_snapshot.pending = true;
            ++m_counters.candidates;
            ++m_sessionCandidates;
        }
        m_upstreamHealthy = false;
        RecordMismatch(
            NC_PREPARED_EQUIVALENCE_MISMATCH_QUEUE |
            NC_PREPARED_EQUIVALENCE_MISMATCH_UPSTREAM);
        RefreshPublication();
        return false;
    }

    std::uint32_t identityFlags =
        NC_PREPARED_EQUIVALENCE_MISMATCH_NONE;
    if (!SourceIdentityExact(head.source, runtimeSource))
        identityFlags |= NC_PREPARED_EQUIVALENCE_MISMATCH_SOURCE;
    if (head.sourcePC != sourcePC ||
        queue.runtimeCurrentPC != sourcePC)
        identityFlags |= NC_PREPARED_EQUIVALENCE_MISMATCH_PC;
    if (head.sourceLineNumber != sourceLineNumber)
        identityFlags |= NC_PREPARED_EQUIVALENCE_MISMATCH_LINE;

    if (identityFlags != NC_PREPARED_EQUIVALENCE_MISMATCH_NONE)
    {
        if (!m_snapshot.candidate)
        {
            m_snapshot.candidate = true;
            m_snapshot.pending = true;
            ++m_counters.candidates;
            ++m_sessionCandidates;
        }
        RecordMismatch(identityFlags);
        RefreshPublication();
        return false;
    }

    if (!CandidateClassEligible(head, runtimeSource, queue))
    {
        m_snapshot.state = NCPreparedHeadEquivalenceState::INELIGIBLE;
        m_snapshot.candidate = false;
        m_snapshot.pending = false;
        ++m_counters.ineligible;
        RefreshPublication();
        return false;
    }

    if (!m_snapshot.candidate)
    {
        m_snapshot.candidate = true;
        m_snapshot.pending = true;
        ++m_counters.candidates;
        ++m_sessionCandidates;
        m_baselineDispatchMatched = queueCounters.dispatchMatched;
        m_baselineCommitMatched = queueCounters.commitMatched;
        m_baselineRetired = queueCounters.retired;
    }

    return true;
}

bool NCPreparedHeadEquivalenceShadow::ObserveResolvedHead(
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
    bool legacyDrainRequired) noexcept
{
    if (!PrepareCandidateToken(
        queue,
        queueCounters,
        hasHead,
        head,
        runtimeSource,
        sourcePC,
        sourceLineNumber))
    {
        return false;
    }

    const bool blockMatch = SemanticBlockEqual(
        head.preparedBlock,
        legacyBlock);
    const bool planMatch = ExecutionPlanEqual(
        head.preparedBlock,
        legacyBlock);
    const NCPreparedBlockClassification legacyClassification =
        NCPreparedBlockQueueShadow::ClassifyLiteralBlock(
            parsedBlock,
            legacyBlock,
            runtimeSource.panel);
    const bool classificationMatch = ClassificationEqual(
        head.classification,
        legacyClassification);
    const bool drainMatch =
        head.classification.legacyDrainRequired ==
        legacyDrainRequired;
    const bool modalBeforeMatch = ModalEqual(
        head.modalBefore,
        liveModalBefore,
        false);

    m_snapshot.blockMatch = blockMatch;
    m_snapshot.planMatch = planMatch;
    m_snapshot.classificationMatch = classificationMatch;
    m_snapshot.drainMatch = drainMatch;
    m_snapshot.modalBeforeMatch = modalBeforeMatch;
    m_snapshot.resolved =
        blockMatch &&
        planMatch &&
        classificationMatch &&
        drainMatch &&
        modalBeforeMatch;

    std::uint32_t semanticFlags =
        NC_PREPARED_EQUIVALENCE_MISMATCH_NONE;
    if (!blockMatch)
        semanticFlags |= NC_PREPARED_EQUIVALENCE_MISMATCH_BLOCK;
    if (!planMatch)
        semanticFlags |= NC_PREPARED_EQUIVALENCE_MISMATCH_PLAN;
    if (!classificationMatch)
        semanticFlags |=
        NC_PREPARED_EQUIVALENCE_MISMATCH_CLASSIFICATION;
    if (!drainMatch)
        semanticFlags |= NC_PREPARED_EQUIVALENCE_MISMATCH_DRAIN;
    if (!modalBeforeMatch)
        semanticFlags |=
        NC_PREPARED_EQUIVALENCE_MISMATCH_MODAL_BEFORE;

    if (semanticFlags != NC_PREPARED_EQUIVALENCE_MISMATCH_NONE)
    {
        RecordMismatch(semanticFlags);
        RefreshPublication();
        return false;
    }

    RefreshPublication();
    return true;
}

void NCPreparedHeadEquivalenceShadow::ObserveResolveFailure(
    const NCPreparedBlockQueueSnapshot& queue,
    const NCPreparedBlockQueueCounters& queueCounters,
    bool hasHead,
    const NCPreparedBlockEntrySnapshot& head,
    const NCPreparedSourceIdentity& runtimeSource,
    int sourcePC,
    int sourceLineNumber) noexcept
{
    if (!PrepareCandidateToken(
        queue,
        queueCounters,
        hasHead,
        head,
        runtimeSource,
        sourcePC,
        sourceLineNumber))
    {
        return;
    }

    RecordMismatch(NC_PREPARED_EQUIVALENCE_MISMATCH_RESOLVE);
    RefreshPublication();
}

bool NCPreparedHeadEquivalenceShadow::BindDispatch(
    std::uint64_t dispatchId,
    const NCProgramCommitSnapshot& dispatchTarget,
    const NCPreparedSourceIdentity& liveSource,
    bool ledgerFound,
    const NCProgramCommitSnapshot& ledgerDispatchTarget,
    int ledgerSourceLineNumber) noexcept
{
    if (!m_snapshot.candidate ||
        !m_snapshot.pending ||
        !m_snapshot.resolved)
    {
        return false;
    }

    if (m_snapshot.dispatchBound)
    {
        ++m_counters.replays;
        const bool replayTargetMatch =
            m_snapshot.dispatchId == dispatchId &&
            ProgramTargetMatchesEntry(dispatchTarget, m_entry);
        const bool replayIdentityMatch =
            SourceIdentityExact(m_entry.source, liveSource);
        const bool replayLedgerMatch =
            ledgerFound &&
            ProgramTargetMatchesEntry(ledgerDispatchTarget, m_entry) &&
            ledgerSourceLineNumber == m_entry.sourceLineNumber;
        if (replayTargetMatch &&
            replayIdentityMatch &&
            replayLedgerMatch)
        {
            RefreshPublication();
            return true;
        }

        std::uint32_t replayFlags =
            NC_PREPARED_EQUIVALENCE_MISMATCH_NONE;
        if (!replayTargetMatch)
            replayFlags |= NC_PREPARED_EQUIVALENCE_MISMATCH_LIFECYCLE;
        if (!replayIdentityMatch)
            replayFlags |=
            NC_PREPARED_EQUIVALENCE_MISMATCH_BIND_IDENTITY |
            NC_PREPARED_EQUIVALENCE_MISMATCH_STALE;
        if (!replayLedgerMatch)
            replayFlags |=
            NC_PREPARED_EQUIVALENCE_MISMATCH_LEDGER |
            NC_PREPARED_EQUIVALENCE_MISMATCH_LIFECYCLE;
        RecordMismatch(replayFlags);
        RefreshPublication();
        return false;
    }

    const bool liveIdentityMatch =
        SourceIdentityExact(m_entry.source, liveSource);
    const bool ledgerDispatchMatch =
        ledgerFound &&
        ProgramTargetMatchesEntry(ledgerDispatchTarget, m_entry) &&
        ledgerSourceLineNumber == m_entry.sourceLineNumber;
    m_snapshot.ledgerDispatchMatch = ledgerDispatchMatch;

    std::uint32_t flags = NC_PREPARED_EQUIVALENCE_MISMATCH_NONE;
    if (dispatchId == 0ULL ||
        !ProgramTargetMatchesEntry(dispatchTarget, m_entry))
    {
        flags |= NC_PREPARED_EQUIVALENCE_MISMATCH_LIFECYCLE;
    }
    if (!liveIdentityMatch)
    {
        flags |=
            NC_PREPARED_EQUIVALENCE_MISMATCH_BIND_IDENTITY |
            NC_PREPARED_EQUIVALENCE_MISMATCH_STALE;
    }
    if (!ledgerDispatchMatch)
    {
        flags |=
            NC_PREPARED_EQUIVALENCE_MISMATCH_LEDGER |
            NC_PREPARED_EQUIVALENCE_MISMATCH_LIFECYCLE;
    }

    if (flags != NC_PREPARED_EQUIVALENCE_MISMATCH_NONE)
    {
        RecordMismatch(flags);
        RefreshPublication();
        return false;
    }

    m_snapshot.dispatchId = dispatchId;
    m_snapshot.dispatchBound = true;
    RefreshPublication();
    return true;
}

void NCPreparedHeadEquivalenceShadow::ObserveCommit(
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
    if (!m_snapshot.candidate || !m_snapshot.pending)
    {
        return;
    }

    if (m_snapshot.commitBound)
    {
        ++m_counters.replays;
        const bool compareReplayG00Override =
            m_entry.classification.primaryGCode == 0 &&
            m_entry.preparedBlock.has('F');
        const bool replayLocalMatch =
            commitSucceeded &&
            dispatchId == m_snapshot.dispatchId &&
            commitTarget.sequence == m_snapshot.commitSequence &&
            ProgramTargetMatchesEntry(commitTarget, m_entry);
        const bool replayIdentityMatch =
            SourceIdentityStableForCommit(
                m_entry.source,
                liveSource,
                m_entry) &&
            liveSource.executionEpoch ==
            m_snapshot.commitExecutionEpoch;
        const bool replayLedgerMatch =
            ledgerFound &&
            ledgerProgramCommitted &&
            m_snapshot.ledgerDispatchMatch &&
            ProgramTargetMatchesEntry(
                ledgerDispatchTarget,
                m_entry) &&
            ProgramTargetMatchesEntry(
                ledgerCommitTarget,
                m_entry) &&
            ledgerCommitTarget.sequence ==
            m_snapshot.commitSequence &&
            ledgerSourceLineNumber == m_entry.sourceLineNumber;
        const bool replayModalAfterMatch =
            m_entry.classification.modalAfterValid &&
            ModalEqual(
                m_entry.modalAfter,
                liveModalAfter,
                compareReplayG00Override);
        const bool replayWaitMatch =
            runtimeWaitCallbackActive ==
            m_snapshot.runtimeWaitCallbackActive;

        if (replayLocalMatch &&
            replayIdentityMatch &&
            replayLedgerMatch &&
            replayModalAfterMatch &&
            replayWaitMatch)
        {
            RefreshPublication();
            return;
        }

        std::uint32_t replayFlags =
            NC_PREPARED_EQUIVALENCE_MISMATCH_NONE;
        if (!replayLocalMatch || !replayWaitMatch)
            replayFlags |= NC_PREPARED_EQUIVALENCE_MISMATCH_LIFECYCLE;
        if (!replayIdentityMatch)
            replayFlags |=
            NC_PREPARED_EQUIVALENCE_MISMATCH_BIND_IDENTITY |
            NC_PREPARED_EQUIVALENCE_MISMATCH_STALE;
        if (!replayLedgerMatch)
            replayFlags |=
            NC_PREPARED_EQUIVALENCE_MISMATCH_LEDGER |
            NC_PREPARED_EQUIVALENCE_MISMATCH_LIFECYCLE;
        if (!replayModalAfterMatch)
            replayFlags |=
            NC_PREPARED_EQUIVALENCE_MISMATCH_MODAL_AFTER;
        RecordMismatch(replayFlags);
        RefreshPublication();
        return;
    }

    const bool localCommitMatch =
        commitSucceeded &&
        m_snapshot.dispatchBound &&
        dispatchId != 0ULL &&
        dispatchId == m_snapshot.dispatchId &&
        commitTarget.sequence != NC_PROGRAM_COMMIT_SEQUENCE_INVALID &&
        ProgramTargetMatchesEntry(commitTarget, m_entry);
    const bool commitIdentityMatch =
        SourceIdentityStableForCommit(
            m_entry.source,
            liveSource,
            m_entry);
    const bool ledgerCommitMatch =
        ledgerFound &&
        ledgerProgramCommitted &&
        m_snapshot.ledgerDispatchMatch &&
        ProgramTargetMatchesEntry(ledgerDispatchTarget, m_entry) &&
        ProgramTargetMatchesEntry(ledgerCommitTarget, m_entry) &&
        ledgerCommitTarget.sequence == commitTarget.sequence &&
        ledgerSourceLineNumber == m_entry.sourceLineNumber;
    const bool compareG00Override =
        m_entry.classification.primaryGCode == 0 &&
        m_entry.preparedBlock.has('F');
    const bool modalAfterMatch =
        m_entry.classification.modalAfterValid &&
        ModalEqual(
            m_entry.modalAfter,
            liveModalAfter,
            compareG00Override);
    const bool runtimeWaitEvidenceMatch =
        !IsOrdinaryAbortingG00(m_entry) ||
        runtimeWaitCallbackActive;

    m_snapshot.ledgerCommitMatch = ledgerCommitMatch;
    m_snapshot.modalAfterMatch = modalAfterMatch;
    m_snapshot.runtimeWaitCallbackActive =
        runtimeWaitCallbackActive;
    m_snapshot.commitBound =
        localCommitMatch &&
        commitIdentityMatch &&
        ledgerCommitMatch &&
        runtimeWaitEvidenceMatch;
    m_snapshot.commitSequence =
        m_snapshot.commitBound
        ? commitTarget.sequence
        : NC_PROGRAM_COMMIT_SEQUENCE_INVALID;
    m_snapshot.commitExecutionEpoch =
        m_snapshot.commitBound
        ? liveSource.executionEpoch
        : 0ULL;

    std::uint32_t flags = NC_PREPARED_EQUIVALENCE_MISMATCH_NONE;
    if (!localCommitMatch)
        flags |= NC_PREPARED_EQUIVALENCE_MISMATCH_LIFECYCLE;
    if (!commitIdentityMatch)
        flags |=
        NC_PREPARED_EQUIVALENCE_MISMATCH_BIND_IDENTITY |
        NC_PREPARED_EQUIVALENCE_MISMATCH_STALE;
    if (!ledgerCommitMatch)
        flags |=
        NC_PREPARED_EQUIVALENCE_MISMATCH_LEDGER |
        NC_PREPARED_EQUIVALENCE_MISMATCH_LIFECYCLE;
    if (!runtimeWaitEvidenceMatch)
        flags |= NC_PREPARED_EQUIVALENCE_MISMATCH_LIFECYCLE;
    if (!modalAfterMatch)
        flags |= NC_PREPARED_EQUIVALENCE_MISMATCH_MODAL_AFTER;

    if (flags != NC_PREPARED_EQUIVALENCE_MISMATCH_NONE)
    {
        RecordMismatch(flags);
        RefreshPublication();
        return;
    }

    // The local Program and Ledger evidence is complete.  MATCHED is still
    // deferred until K.1 consumes this exact Dispatch/Commit proof from the
    // unchanged Runtime publication later in the same NC task.
    RefreshPublication();
}

void NCPreparedHeadEquivalenceShadow::ObserveUpstreamProof(
    const NCPreparedBlockQueueSnapshot& queue,
    const NCPreparedBlockQueueCounters& queueCounters,
    const NCPreparedRuntimeProof& proof) noexcept
{
    if (!queue.active)
    {
        ObserveQueueInactive(queue.lastInvalidationReason);
        return;
    }

    if (queue.depth > m_counters.lifetimePeakDepth)
    {
        m_counters.lifetimePeakDepth = queue.depth;
    }

    if (queue.session != NC_PREPARED_QUEUE_SESSION_INVALID &&
        queue.session != m_observedSession)
    {
        const bool hadSession =
            m_observedSession != NC_PREPARED_QUEUE_SESSION_INVALID;
        const NCPreparedInvalidationReason transitionReason =
            hadSession
            ? queue.lastInvalidationReason
            : NCPreparedInvalidationReason::NONE;
        if (m_snapshot.pending)
        {
            if (transitionReason == NCPreparedInvalidationReason::NONE)
            {
                InvalidatePending(
                    NCPreparedHeadEquivalenceInvalidation::TOKEN_REPLACED,
                    true);
            }
            else
            {
                InvalidatePending(
                    MapInvalidation(transitionReason),
                    false);
            }
        }
        BeginSession(queue.session, transitionReason);
    }

    m_queueActive = queue.session == m_observedSession;
    if (m_queueActive && queue.depth > m_sessionPeakDepth)
    {
        m_sessionPeakDepth = queue.depth;
    }

    const bool upstreamCountersValid =
        UpstreamCountersHealthy(queueCounters);
    const bool queueProofCoherent =
        queue.valid &&
        queue.cursorOrderValid &&
        queue.accountingValid &&
        queue.shadowOnly &&
        queue.capacity == NC_PREPARED_BLOCK_QUEUE_CAPACITY &&
        queue.depth <= queue.capacity &&
        queueCounters.commitMatched <= queueCounters.dispatchMatched &&
        queueCounters.dispatchMatched <= queueCounters.prepared &&
        queueCounters.prepared ==
        queueCounters.retired +
        queueCounters.invalidatedEntries +
        static_cast<std::uint64_t>(queue.depth);
    if (!queueProofCoherent || !upstreamCountersValid)
    {
        m_upstreamHealthy = false;
        if (m_snapshot.pending)
        {
            RecordMismatch(
                NC_PREPARED_EQUIVALENCE_MISMATCH_QUEUE |
                NC_PREPARED_EQUIVALENCE_MISMATCH_UPSTREAM);
            m_sessionUpstreamFailureLatched = true;
        }
        else if (!m_sessionUpstreamFailureLatched)
        {
            ++m_counters.queueMismatches;
            ++m_counters.upstreamMismatches;
            m_sessionUpstreamFailureLatched = true;
        }
    }

    if (!m_snapshot.candidate ||
        !m_snapshot.pending ||
        !m_snapshot.commitBound)
    {
        RefreshPublication();
        return;
    }

    const bool proofAvailable =
        proof.hasDispatch &&
        proof.dispatchId != 0ULL &&
        proof.commitTarget.sequence !=
        NC_PROGRAM_COMMIT_SEQUENCE_INVALID;
    if (!proofAvailable)
    {
        m_upstreamHealthy = false;
        RecordMismatch(
            NC_PREPARED_EQUIVALENCE_MISMATCH_UPSTREAM |
            NC_PREPARED_EQUIVALENCE_MISMATCH_LIFECYCLE);
        RefreshPublication();
        return;
    }

    const bool exactDispatchCommitAdvance =
        m_baselineDispatchMatched != UINT64_MAX &&
        m_baselineCommitMatched != UINT64_MAX &&
        m_baselineRetired != UINT64_MAX &&
        queueCounters.dispatchMatched ==
        m_baselineDispatchMatched + 1ULL &&
        queueCounters.commitMatched ==
        m_baselineCommitMatched + 1ULL;

    const bool exactProofIdentity =
        queueProofCoherent &&
        queue.session == m_entry.session &&
        SourceIdentityStableForCommit(
            m_entry.source,
            queue.source,
            m_entry) &&
        proof.dispatchId == m_snapshot.dispatchId &&
        ProgramTargetMatchesEntry(proof.dispatchTarget, m_entry) &&
        ProgramTargetMatchesEntry(proof.commitTarget, m_entry) &&
        proof.commitTarget.sequence == m_snapshot.commitSequence &&
        exactDispatchCommitAdvance &&
        upstreamCountersValid;
    const bool waitEligibleG00 =
        m_entry.classification.blockClass ==
        NCPreparedBlockClass::MOTION_SHADOW &&
        m_entry.classification.primaryGCode == 0;
    const bool hadExactWaitProof =
        m_upstreamDispatchCommitObserved;
    const bool exactWaitPhase =
        exactProofIdentity &&
        waitEligibleG00 &&
        m_snapshot.runtimeWaitCallbackActive &&
        queue.runtimeCurrentPC == m_entry.sourcePC &&
        queueCounters.retired == m_baselineRetired;
    const bool exactRetiredPhase =
        exactProofIdentity &&
        (!m_snapshot.runtimeWaitCallbackActive ||
            hadExactWaitProof) &&
        queue.runtimeCurrentPC > m_entry.sourcePC &&
        queue.runtimeCurrentPC - m_entry.sourcePC == 1 &&
        queueCounters.retired == m_baselineRetired + 1ULL;

    if (!exactWaitPhase && !exactRetiredPhase)
    {
        m_upstreamHealthy = false;
        RecordMismatch(
            NC_PREPARED_EQUIVALENCE_MISMATCH_UPSTREAM |
            NC_PREPARED_EQUIVALENCE_MISMATCH_LIFECYCLE);
        RefreshPublication();
        return;
    }

    m_upstreamDispatchCommitObserved = true;
    m_snapshot.upstreamDispatchCommitMatch = true;
    if (exactWaitPhase)
    {
        // Motion/Completion wait owns PC advance.  The exact K.1 proof has
        // been consumed, but the same Head must remain pending until the
        // unchanged wait callback advances PC and K.1 retires one entry.
        RefreshPublication();
        return;
    }

    m_snapshot.retirementMatch = exactRetiredPhase;
    m_snapshot.upstreamProofMatch = true;
    m_snapshot.lifecycleMatch = true;
    m_snapshot.state = NCPreparedHeadEquivalenceState::MATCHED;
    m_snapshot.pending = false;
    m_snapshot.matched = true;
    ++m_counters.matched;
    ++m_counters.wouldUse;
    ++m_sessionMatched;
    RefreshPublication();
}

void NCPreparedHeadEquivalenceShadow::ObserveRuntimeFailure() noexcept
{
    if (!m_snapshot.candidate || !m_snapshot.pending)
    {
        return;
    }
    ++m_counters.runtimeFailures;
    ++m_sessionRuntimeFailures;
    InvalidatePending(
        NCPreparedHeadEquivalenceInvalidation::RUNTIME_FAILURE,
        false);
    RefreshPublication();
}

void NCPreparedHeadEquivalenceShadow::ObserveQueueInactive(
    NCPreparedInvalidationReason reason) noexcept
{
    const NCPreparedHeadEquivalenceInvalidation mapped =
        MapInvalidation(reason);
    const bool activeEdge = m_queueActive;
    m_queueActive = false;
    m_upstreamHealthy = false;
    if (m_snapshot.pending)
    {
        InvalidatePending(mapped, false);
    }
    else if (activeEdge)
    {
        m_snapshot.lastInvalidation = mapped;
    }
    RefreshPublication();
}

const char* NCPreparedHeadEquivalenceStateToDiagnosticName(
    NCPreparedHeadEquivalenceState value) noexcept
{
    switch (value)
    {
    case NCPreparedHeadEquivalenceState::IDLE: return "IDLE";
    case NCPreparedHeadEquivalenceState::INELIGIBLE: return "INELIGIBLE";
    case NCPreparedHeadEquivalenceState::PENDING: return "PENDING";
    case NCPreparedHeadEquivalenceState::MATCHED: return "MATCHED";
    case NCPreparedHeadEquivalenceState::MISMATCHED: return "MISMATCHED";
    case NCPreparedHeadEquivalenceState::INVALIDATED: return "INVALIDATED";
    default: return "UNKNOWN";
    }
}

const char* NCPreparedHeadEquivalenceInvalidationToDiagnosticName(
    NCPreparedHeadEquivalenceInvalidation value) noexcept
{
    switch (value)
    {
    case NCPreparedHeadEquivalenceInvalidation::QUEUE_INACTIVE:
        return "INACTIVE";
    case NCPreparedHeadEquivalenceInvalidation::ALARM: return "ALARM";
    case NCPreparedHeadEquivalenceInvalidation::RESET: return "RESET";
    case NCPreparedHeadEquivalenceInvalidation::PROGRAM_END:
        return "PROGRAM_END";
    case NCPreparedHeadEquivalenceInvalidation::SOURCE_CHANGED:
        return "SOURCE";
    case NCPreparedHeadEquivalenceInvalidation::EPOCH_CHANGED:
        return "EPOCH";
    case NCPreparedHeadEquivalenceInvalidation::OWNER_CHANGED:
        return "OWNER";
    case NCPreparedHeadEquivalenceInvalidation::FRAME_CHANGED:
        return "FRAME";
    case NCPreparedHeadEquivalenceInvalidation::PANEL_CHANGED:
        return "PANEL";
    case NCPreparedHeadEquivalenceInvalidation::TOKEN_REPLACED:
        return "TOKEN";
    case NCPreparedHeadEquivalenceInvalidation::RUNTIME_FAILURE:
        return "RUNTIME";
    case NCPreparedHeadEquivalenceInvalidation::NONE:
    default:
        return "NONE";
    }
}
