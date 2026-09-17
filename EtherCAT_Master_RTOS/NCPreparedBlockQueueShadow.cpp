#include "NCPreparedBlockQueueShadow.h"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <limits>

namespace
{
    bool TryParseLiteralDouble(
        const std::string& expression,
        double& value) noexcept
    {
        value = 0.0;
        if (expression.empty())
        {
            return false;
        }

        errno = 0;
        char* end = nullptr;
        const char* begin = expression.c_str();
        const double parsed = std::strtod(begin, &end);

        if (begin == end || errno == ERANGE || !std::isfinite(parsed))
        {
            return false;
        }

        while (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n')
        {
            ++end;
        }

        if (*end != '\0')
        {
            return false;
        }

        value = parsed;
        return true;
    }

    bool TryParseLiteralInteger(
        const std::string& expression,
        int& value) noexcept
    {
        double parsed = 0.0;
        if (!TryParseLiteralDouble(expression, parsed))
        {
            return false;
        }

        const double rounded = std::round(parsed);
        if (std::fabs(parsed - rounded) > 1.0e-9 ||
            rounded < static_cast<double>((std::numeric_limits<int>::min)()) ||
            rounded > static_cast<double>((std::numeric_limits<int>::max)()))
        {
            return false;
        }

        value = static_cast<int>(rounded);
        return true;
    }

    int SafeGCodeCount(const NCBlock& block) noexcept
    {
        if (block.gCount <= 0)
        {
            return block.hasG ? 1 : 0;
        }

        return
            block.gCount < NC_MAX_G_CODES_PER_BLOCK
            ? block.gCount
            : NC_MAX_G_CODES_PER_BLOCK;
    }

    int StoredGCode(const NCBlock& block, int index) noexcept
    {
        return block.gCount <= 0 ? block.gCode : block.gCodes[index];
    }

    bool IsExtendedWorkCoordinateCode(int code) noexcept
    {
        if (code < 54 || code > 959)
        {
            return false;
        }

        const int suffix = code % 100;
        const int prefix = code / 100;
        return
            suffix >= 54 && suffix <= 59 &&
            prefix >= 0 && prefix <= 9;
    }

    int BarrierPriority(NCPreparedBarrierKind kind) noexcept
    {
        switch (kind)
        {
        case NCPreparedBarrierKind::PARSE_ERROR: return 130;
        case NCPreparedBarrierKind::SEMANTIC_ERROR: return 125;
        case NCPreparedBarrierKind::DYNAMIC_EXPRESSION: return 120;
        case NCPreparedBarrierKind::PROGRAM_END: return 110;
        case NCPreparedBarrierKind::PROGRAM_FLOW: return 100;
        case NCPreparedBarrierKind::TABLE_WRITE: return 90;
        case NCPreparedBarrierKind::POSITION_DERIVED: return 80;
        case NCPreparedBarrierKind::TIME_STOP: return 70;
        case NCPreparedBarrierKind::MODAL_SNAPSHOT: return 60;
        case NCPreparedBarrierKind::AUX_IO: return 50;
        case NCPreparedBarrierKind::IMPLICIT_MOTION: return 45;
        case NCPreparedBarrierKind::SINGLE_BLOCK: return 40;
        case NCPreparedBarrierKind::BLOCK_SKIP: return 35;
        case NCPreparedBarrierKind::NATURAL_EOF: return 30;
        case NCPreparedBarrierKind::NONE:
        default: return 0;
        }
    }

    void AddBarrier(
        NCPreparedBlockClassification& classification,
        NCPreparedBarrierKind kind,
        std::uint32_t flag) noexcept
    {
        classification.barrierFlags |= flag;
        classification.planningStopsHere = true;

        if (BarrierPriority(kind) >
            BarrierPriority(classification.barrierKind))
        {
            classification.barrierKind = kind;
        }
    }

    bool HasAnyAxisAddress(const NCBlock& block) noexcept
    {
        return
            block.has('X') || block.has('Y') || block.has('Z') ||
            block.has('A') || block.has('B') || block.has('C') ||
            block.has('U') || block.has('V') || block.has('W');
    }

    bool IsSingleBlockCandidate(const NCBlock& block) noexcept
    {
        if (block.gCount > 0 || block.hasG || block.mCount > 0)
        {
            return true;
        }

        for (int index = 0; index < 26; ++index)
        {
            if (!block.hasParam[index])
            {
                continue;
            }

            const char address = static_cast<char>('A' + index);
            if (address != 'N' && address != 'O')
            {
                return true;
            }
        }
        return false;
    }

    bool IsLegacySingleBlockDrainCandidate(
        const NCBlock& block) noexcept
    {
        return
            block.hasG || block.gCount > 0 || block.mCount > 0 ||
            block.has('X') || block.has('Y') || block.has('Z');
    }

    bool IsProgramFlowMCode(int code) noexcept
    {
        return
            code == 0 || code == 1 || code == 2 || code == 30 ||
            code == 98 || code == 99;
    }

    bool IsModalMacroMotionCandidate(const NCBlock& block) noexcept
    {
        if (!block.has('X') && !block.has('Y') && !block.has('Z'))
        {
            return false;
        }

        const int primary = NCGCodeSemantics::GetPrimaryActionCode(block);
        if (primary >= 0)
        {
            return NCGCodeSemantics::IsMotionAction(primary);
        }

        return !NCGCodeSemantics::BlockSuppressesImplicitMotion(block);
    }

    bool IsAbortingG00PreparedEntry(
        const NCPreparedBlockEntrySnapshot& entry) noexcept
    {
        if (entry.classification.blockClass !=
            NCPreparedBlockClass::MOTION_SHADOW ||
            entry.classification.primaryGCode != 0)
        {
            return false;
        }

        const NCBlock& block = entry.preparedBlock;
        return !(block.has('P') && block.val('P') == 1.0);
    }

    bool BarrierFlagsLeaveScalarModalUnproven(
        std::uint32_t flags) noexcept
    {
        constexpr std::uint32_t unprovenFlags =
            NC_PREPARED_BARRIER_FLAG_MODAL |
            NC_PREPARED_BARRIER_FLAG_POSITION |
            NC_PREPARED_BARRIER_FLAG_TABLE_WRITE |
            NC_PREPARED_BARRIER_FLAG_FLOW |
            NC_PREPARED_BARRIER_FLAG_AUX |
            NC_PREPARED_BARRIER_FLAG_IMPLICIT |
            NC_PREPARED_BARRIER_FLAG_DYNAMIC |
            NC_PREPARED_BARRIER_FLAG_PARSE |
            NC_PREPARED_BARRIER_FLAG_SEMANTIC;
        return (flags & unprovenFlags) != 0U;
    }

    bool PanelImageEqual(
        const NCPreparedPanelSwitchImage& left,
        const NCPreparedPanelSwitchImage& right) noexcept
    {
        return
            left.blockSkipEnabled == right.blockSkipEnabled &&
            left.singleBlockEnabled == right.singleBlockEnabled &&
            left.optionalStopEnabled == right.optionalStopEnabled;
    }

}

bool NCPreparedSourceIdentity::IsValid() const noexcept
{
    return
        scope != NCProgramScope::NONE &&
        cacheGeneration != NC_PROGRAM_CACHE_GENERATION_INVALID &&
        executionEpoch != 0ULL &&
        owner != 0U &&
        ownerGeneration != 0ULL &&
        (scope != NCProgramScope::MACRO ||
            frameId != NC_PROGRAM_FRAME_ID_INVALID);
}

bool NCPreparedBlockQueueShadow::SourceIdentityEqual(
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
        PanelImageEqual(left.panel, right.panel);
}

bool NCPreparedBlockQueueShadow::ProgramTargetMatchesSource(
    const NCProgramCommitSnapshot& target,
    const NCPreparedSourceIdentity& source) noexcept
{
    return
        target.scope == source.scope &&
        target.cacheGeneration == source.cacheGeneration &&
        target.frameId == source.frameId &&
        target.sourcePC >= 0;
}

bool NCPreparedBlockQueueShadow::ProgramTargetMatchesEntry(
    const NCProgramCommitSnapshot& target,
    const NCPreparedBlockEntrySnapshot& entry) noexcept
{
    return
        ProgramTargetMatchesSource(target, entry.source) &&
        target.sourcePC == entry.sourcePC;
}

NCPreparedInvalidationReason
NCPreparedBlockQueueShadow::ClassifyIdentityChange(
    const NCPreparedSourceIdentity& previous,
    const NCPreparedSourceIdentity& current) noexcept
{
    if (previous.scope != current.scope)
    {
        return NCPreparedInvalidationReason::MODE_CHANGED;
    }
    if (previous.cacheGeneration != current.cacheGeneration)
    {
        return NCPreparedInvalidationReason::SOURCE_REPLACED;
    }
    if (previous.frameId != current.frameId)
    {
        return NCPreparedInvalidationReason::FRAME_CHANGED;
    }
    if (previous.programFlowGeneration !=
        current.programFlowGeneration)
    {
        return NCPreparedInvalidationReason::CURSOR_DISCONTINUITY;
    }
    if (previous.owner != current.owner ||
        previous.ownerGeneration != current.ownerGeneration)
    {
        return NCPreparedInvalidationReason::OWNER_CHANGED;
    }
    if (!PanelImageEqual(previous.panel, current.panel))
    {
        return NCPreparedInvalidationReason::PANEL_SWITCH_CHANGED;
    }
    return NCPreparedInvalidationReason::SOURCE_REPLACED;
}

NCPreparedQueueSession NCPreparedBlockQueueShadow::AllocateNonZero(
    NCPreparedQueueSession& next) noexcept
{
    NCPreparedQueueSession value = next++;
    if (value == NC_PREPARED_QUEUE_SESSION_INVALID)
    {
        value = next++;
    }
    return value;
}

NCPreparedEntrySequence
NCPreparedBlockQueueShadow::AllocateNonZeroEntry(
    NCPreparedEntrySequence& next) noexcept
{
    NCPreparedEntrySequence value = next++;
    if (value == NC_PREPARED_ENTRY_SEQUENCE_INVALID)
    {
        value = next++;
    }
    return value;
}

void NCPreparedBlockQueueShadow::RecordInvalidationReason(
    NCPreparedInvalidationReason reason) noexcept
{
    ++m_counters.invalidations;
    switch (reason)
    {
    case NCPreparedInvalidationReason::ALARM:
        ++m_counters.alarmInvalidations;
        break;
    case NCPreparedInvalidationReason::RESET:
        ++m_counters.resetInvalidations;
        break;
    case NCPreparedInvalidationReason::SOURCE_REPLACED:
    case NCPreparedInvalidationReason::MODE_CHANGED:
        ++m_counters.sourceInvalidations;
        break;
    case NCPreparedInvalidationReason::EXECUTION_EPOCH_CHANGED:
        ++m_counters.epochInvalidations;
        break;
    case NCPreparedInvalidationReason::OWNER_CHANGED:
        ++m_counters.ownerInvalidations;
        break;
    case NCPreparedInvalidationReason::FRAME_CHANGED:
        ++m_counters.frameInvalidations;
        break;
    case NCPreparedInvalidationReason::PANEL_SWITCH_CHANGED:
        ++m_counters.panelInvalidations;
        break;
    case NCPreparedInvalidationReason::NONE:
    case NCPreparedInvalidationReason::NOT_RUNNING:
    case NCPreparedInvalidationReason::PROGRAM_END:
    case NCPreparedInvalidationReason::CURSOR_DISCONTINUITY:
    case NCPreparedInvalidationReason::IDENTITY_INVALID:
    default:
        break;
    }
}

void NCPreparedBlockQueueShadow::InvalidateActive(
    NCPreparedInvalidationReason reason) noexcept
{
    if (!m_active)
    {
        return;
    }

    RecordInvalidationReason(reason);
    m_counters.invalidatedEntries +=
        static_cast<std::uint64_t>(m_depth);

    m_entries = {};
    m_head = 0U;
    m_depth = 0U;
    m_active = false;
    m_valid = true;
    m_barrierLatched = false;
    m_eofLatched = false;
    m_capacityLatched = false;
    m_lastInvalidationReason = reason;
    m_nextPlanPC = -1;
    m_dispatchPC = -1;
    m_commitPC = -1;
    m_committedBaselinePC = -1;
    m_epochAuthorizationDispatchId = 0ULL;
    m_epochAuthorizationFrom = 0ULL;
    m_committedBaselinePlanningBlocked = false;
}

void NCPreparedBlockQueueShadow::ObserveInactive(
    NCPreparedInvalidationReason reason) noexcept
{
    ++m_counters.evaluations;
    InvalidateActive(reason);
    m_snapshot = NCPreparedBlockQueueSnapshot{};
    m_snapshot.publicationSequence = m_nextPublicationSequence++;
    if (m_snapshot.publicationSequence == 0ULL)
    {
        m_snapshot.publicationSequence = m_nextPublicationSequence++;
    }
    m_snapshot.stopReason = NCPreparedQueueStopReason::INACTIVE;
    m_snapshot.lastInvalidationReason = m_lastInvalidationReason;
    m_snapshot.accountingValid =
        m_counters.prepared ==
        m_counters.retired +
        m_counters.invalidatedEntries;
    ++m_counters.publications;
}

void NCPreparedBlockQueueShadow::ObserveFinalProofAndInvalidate(
    const NCPreparedRuntimeProof& proof,
    NCPreparedInvalidationReason reason) noexcept
{
    if (!m_active)
    {
        ObserveInactive(reason);
        return;
    }

    ++m_counters.evaluations;
    ObserveRuntimeProof(proof);
    RetireProvenHeadBeforeCutover();
    InvalidateActive(reason);

    RefreshSnapshot();
    m_snapshot.publicationSequence = m_nextPublicationSequence++;
    if (m_snapshot.publicationSequence == 0ULL)
    {
        m_snapshot.publicationSequence = m_nextPublicationSequence++;
    }
    ++m_counters.publications;
}

void NCPreparedBlockQueueShadow::BeginNewSession(
    const NCPreparedSourceIdentity& source,
    int runtimeCurrentPC,
    int committedPC,
    const NCPreparedModalSnapshot& liveModal,
    const NCPreparedRuntimeProof& baselineProof,
    bool permitCommittedBaseline,
    bool blockPlanningAtCommittedBaseline) noexcept
{
    m_entries = {};
    m_head = 0U;
    m_depth = 0U;
    m_source = source;
    m_seedModal = liveModal;
    m_runtimeCurrentPC = runtimeCurrentPC;
    m_committedPC = committedPC;
    const bool exactCommittedBaseline =
        permitCommittedBaseline &&
        committedPC == runtimeCurrentPC &&
        baselineProof.hasDispatch &&
        baselineProof.dispatchId != 0ULL &&
        baselineProof.commitTarget.sequence !=
        NC_PROGRAM_COMMIT_SEQUENCE_INVALID &&
        ProgramTargetMatchesSource(
            baselineProof.dispatchTarget,
            source) &&
        ProgramTargetMatchesSource(
            baselineProof.commitTarget,
            source) &&
        baselineProof.dispatchTarget.sourcePC == runtimeCurrentPC &&
        baselineProof.commitTarget.sourcePC == runtimeCurrentPC;
    m_committedBaselinePC =
        exactCommittedBaseline ? runtimeCurrentPC : -1;
    m_committedBaselinePlanningBlocked =
        exactCommittedBaseline && blockPlanningAtCommittedBaseline;
    m_nextPlanPC =
        exactCommittedBaseline ? runtimeCurrentPC + 1 : runtimeCurrentPC;
    m_dispatchPC = -1;
    m_commitPC = -1;
    m_lastDispatchProofId =
        baselineProof.hasDispatch ? baselineProof.dispatchId : 0ULL;
    m_lastCommitProofSequence = baselineProof.commitTarget.sequence;
    m_epochAuthorizationDispatchId = 0ULL;
    m_epochAuthorizationFrom = 0ULL;
    m_session = AllocateNonZero(m_nextSession);
    m_active = true;
    m_valid = true;
    m_barrierLatched = false;
    m_eofLatched = false;
    m_capacityLatched = false;
    ++m_counters.sessions;
}

bool NCPreparedBlockQueueShadow::BeginObservation(
    const NCPreparedSourceIdentity& source,
    int runtimeCurrentPC,
    int committedPC,
    const NCPreparedModalSnapshot& liveModal,
    const NCPreparedRuntimeProof& proof) noexcept
{
    ++m_counters.evaluations;
    bool freshDispatchProof = false;
    bool freshCommitProof = false;

    // A Runtime call may Dispatch/Commit the old head and then change PC,
    // Macro frame or Epoch in the same NC scan.  Always consume that proof
    // against the old session before evaluating the new observation image.
    if (m_active)
    {
        const std::uint64_t previousDispatchProofId =
            m_lastDispatchProofId;
        const NCProgramCommitSequence previousCommitProofSequence =
            m_lastCommitProofSequence;
        ObserveRuntimeProof(proof);
        freshDispatchProof =
            proof.hasDispatch &&
            proof.dispatchId != 0ULL &&
            proof.dispatchId != previousDispatchProofId;
        freshCommitProof =
            proof.commitTarget.sequence !=
            NC_PROGRAM_COMMIT_SEQUENCE_INVALID &&
            proof.commitTarget.sequence != previousCommitProofSequence;

        if (m_depth > 0U &&
            freshDispatchProof &&
            freshCommitProof)
        {
            const NCPreparedBlockEntrySnapshot& head = m_entries[m_head];
            if (IsAbortingG00PreparedEntry(head) &&
                head.dispatchObserved &&
                head.commitObserved &&
                head.dispatchId != 0ULL)
            {
                // One fresh, exact Dispatch/Commit pair grants one future
                // asynchronous RT Epoch correlation.  The authorization is
                // consumed on use or cleared when this Head leaves.
                m_epochAuthorizationDispatchId = head.dispatchId;
                m_epochAuthorizationFrom = m_source.executionEpoch;
            }
        }
    }

    if (!source.IsValid() || runtimeCurrentPC < 0)
    {
        if (m_active)
        {
            RetireProvenHeadBeforeCutover();
            InvalidateActive(NCPreparedInvalidationReason::IDENTITY_INVALID);
        }
        ++m_counters.identityFailures;
        RefreshSnapshot();
        return false;
    }

    const bool exactCommittedGotoHead =
        m_active &&
        m_depth > 0U &&
        m_entries[m_head].classification.runtimeGotoControl &&
        m_entries[m_head].dispatchObserved &&
        m_entries[m_head].commitObserved &&
        ProgramTargetMatchesEntry(
            proof.dispatchTarget,
            m_entries[m_head]) &&
        ProgramTargetMatchesEntry(
            proof.commitTarget,
            m_entries[m_head]);

    if (!m_active)
    {
        BeginNewSession(
            source,
            runtimeCurrentPC,
            committedPC,
            liveModal,
            proof,
            true,
            true);
    }
    else if (!SourceIdentityEqual(m_source, source))
    {
        const NCPreparedInvalidationReason reason =
            ClassifyIdentityChange(m_source, source);
        const bool committedHeadWasFence =
            m_depth > 0U &&
            m_entries[m_head].sourcePC == runtimeCurrentPC &&
            m_entries[m_head].dispatchObserved &&
            m_entries[m_head].commitObserved &&
            m_entries[m_head].classification.planningStopsHere;
        const bool outstandingCommittedFence =
            m_committedBaselinePlanningBlocked &&
            m_committedBaselinePC == runtimeCurrentPC;
        if (reason == NCPreparedInvalidationReason::CURSOR_DISCONTINUITY)
        {
            ++m_counters.expectedFlowCutovers;
        }
        RetireProvenHeadBeforeCutover();
        InvalidateActive(reason);
        BeginNewSession(
            source,
            runtimeCurrentPC,
            committedPC,
            liveModal,
            proof,
            reason == NCPreparedInvalidationReason::PANEL_SWITCH_CHANGED,
            reason == NCPreparedInvalidationReason::PANEL_SWITCH_CHANGED &&
            (committedHeadWasFence || outstandingCommittedFence));
    }
    else if (exactCommittedGotoHead &&
        (runtimeCurrentPC != m_runtimeCurrentPC ||
            source.executionEpoch == m_source.executionEpoch + 1ULL))
    {
        // PC movement proves a completed GOTO decision.  A same-PC decision
        // additionally requires its exact GOTO Epoch.  Both are expected
        // flow cutovers, not Plan discontinuities.
        ++m_counters.expectedFlowCutovers;
        RetireProvenHeadBeforeCutover();
        InvalidateActive(
            NCPreparedInvalidationReason::CURSOR_DISCONTINUITY);
        BeginNewSession(
            source,
            runtimeCurrentPC,
            committedPC,
            liveModal,
            proof,
            false,
            false);
    }
    else if (runtimeCurrentPC < m_runtimeCurrentPC)
    {
        ++m_counters.cursorRegressions;
        RetireProvenHeadBeforeCutover();
        InvalidateActive(
            NCPreparedInvalidationReason::CURSOR_DISCONTINUITY);
        BeginNewSession(
            source,
            runtimeCurrentPC,
            committedPC,
            liveModal,
            proof,
            false,
            false);
    }
    else if (m_runtimeCurrentPC >= 0 &&
        runtimeCurrentPC > m_runtimeCurrentPC + 1)
    {
        // One ProcessExecutionEngine invocation can advance at most one
        // source line.  A larger forward step is a GOTO / flow cutover and
        // therefore starts a fresh planning generation; no intervening
        // Prepared entry may survive it.
        ++m_counters.planDiscontinuities;
        RetireProvenHeadBeforeCutover();
        InvalidateActive(
            NCPreparedInvalidationReason::CURSOR_DISCONTINUITY);
        BeginNewSession(
            source,
            runtimeCurrentPC,
            committedPC,
            liveModal,
            proof,
            false,
            false);
    }
    else if (m_source.executionEpoch != source.executionEpoch)
    {
        // Ordinary ABORTING G00 intentionally advances Motion's execution
        // Epoch.  It is not a Program / Prepared-session cutover.  Accept the
        // transition only when the unchanged Lifecycle proves that the
        // logical Queue Head is the exact dispatch responsible for it.
        const bool proofTargetsHead =
            m_depth > 0U &&
            IsAbortingG00PreparedEntry(m_entries[m_head]) &&
            source.executionEpoch == m_source.executionEpoch + 1ULL &&
            m_epochAuthorizationDispatchId != 0ULL &&
            m_epochAuthorizationFrom == m_source.executionEpoch &&
            proof.hasDispatch &&
            proof.dispatchId != 0ULL &&
            m_entries[m_head].dispatchObserved &&
            m_entries[m_head].commitObserved &&
            m_entries[m_head].dispatchId == proof.dispatchId &&
            m_entries[m_head].dispatchId ==
            m_epochAuthorizationDispatchId &&
            ProgramTargetMatchesEntry(
                proof.dispatchTarget,
                m_entries[m_head]);

        if (proofTargetsHead && m_valid)
        {
            m_source.executionEpoch = source.executionEpoch;
            for (std::size_t offset = 0U; offset < m_depth; ++offset)
            {
                m_entries[PhysicalIndex(offset)].source.executionEpoch =
                    source.executionEpoch;
            }
            m_epochAuthorizationDispatchId = 0ULL;
            m_epochAuthorizationFrom = 0ULL;
            ++m_counters.correlatedEpochAdvances;
        }
        else
        {
            RetireProvenHeadBeforeCutover();
            InvalidateActive(
                NCPreparedInvalidationReason::EXECUTION_EPOCH_CHANGED);
            BeginNewSession(
                source,
                runtimeCurrentPC,
                committedPC,
                liveModal,
                proof,
                true,
                true);
        }
    }

    if (runtimeCurrentPC > m_runtimeCurrentPC)
    {
        if (m_committedBaselinePC >= 0 &&
            runtimeCurrentPC > m_committedBaselinePC)
        {
            m_committedBaselinePC = -1;
            m_committedBaselinePlanningBlocked = false;
        }
        RetireBefore(runtimeCurrentPC);
    }

    m_runtimeCurrentPC = runtimeCurrentPC;
    m_committedPC = committedPC;

    if (m_depth == 0U)
    {
        m_seedModal = liveModal;
        m_nextPlanPC =
            m_committedBaselinePC == runtimeCurrentPC
            ? runtimeCurrentPC + 1
            : runtimeCurrentPC;
        if (m_committedBaselinePC != runtimeCurrentPC)
        {
            m_committedBaselinePlanningBlocked = false;
        }
        m_barrierLatched = false;
        m_capacityLatched = false;
    }
    else
    {
        const NCPreparedBlockEntrySnapshot& head = m_entries[m_head];
        const bool exactOneAheadBaseline =
            m_committedBaselinePC == runtimeCurrentPC &&
            head.sourcePC == runtimeCurrentPC + 1;
        if (head.sourcePC > runtimeCurrentPC &&
            !exactOneAheadBaseline)
        {
            ++m_counters.planDiscontinuities;
            m_valid = false;
        }
    }

    RefreshLatchesFromTail();
    RefreshSnapshot();
    return m_active && m_valid;
}

std::size_t NCPreparedBlockQueueShadow::PhysicalIndex(
    std::size_t logicalOffset) const noexcept
{
    return (m_head + logicalOffset) % NC_PREPARED_BLOCK_QUEUE_CAPACITY;
}

bool NCPreparedBlockQueueShadow::TryGetEntry(
    std::size_t logicalOffset,
    NCPreparedBlockEntrySnapshot& entry) const noexcept
{
    if (logicalOffset >= m_depth)
    {
        entry = NCPreparedBlockEntrySnapshot{};
        return false;
    }

    entry = m_entries[PhysicalIndex(logicalOffset)];
    return true;
}

void NCPreparedBlockQueueShadow::ObserveRuntimeProof(
    const NCPreparedRuntimeProof& proof) noexcept
{
    if (proof.hasDispatch &&
        proof.dispatchId != 0ULL &&
        proof.dispatchId != m_lastDispatchProofId)
    {
        m_lastDispatchProofId = proof.dispatchId;

        if (!ProgramTargetMatchesSource(proof.dispatchTarget, m_source))
        {
            ++m_counters.staleRuntimeProofs;
        }
        else
        {
            if (m_depth == 0U ||
                !ProgramTargetMatchesEntry(
                    proof.dispatchTarget,
                    m_entries[m_head]))
            {
                ++m_counters.dispatchMismatches;
                m_valid = false;
            }
            else
            {
                NCPreparedBlockEntrySnapshot& entry = m_entries[m_head];
                if (!entry.dispatchObserved)
                {
                    entry.dispatchObserved = true;
                    entry.dispatchId = proof.dispatchId;
                    ++m_counters.dispatchMatched;
                }
                else if (entry.dispatchId != proof.dispatchId)
                {
                    ++m_counters.dispatchMismatches;
                    m_valid = false;
                }
                m_dispatchPC = entry.sourcePC;
            }
        }
    }

    const NCProgramCommitSequence commitSequence =
        proof.commitTarget.sequence;
    if (commitSequence != NC_PROGRAM_COMMIT_SEQUENCE_INVALID &&
        commitSequence != m_lastCommitProofSequence)
    {
        m_lastCommitProofSequence = commitSequence;

        if (!ProgramTargetMatchesSource(proof.commitTarget, m_source))
        {
            ++m_counters.staleRuntimeProofs;
        }
        else
        {
            if (m_depth == 0U ||
                !ProgramTargetMatchesEntry(
                    proof.commitTarget,
                    m_entries[m_head]))
            {
                ++m_counters.commitMismatches;
                m_valid = false;
            }
            else
            {
                NCPreparedBlockEntrySnapshot& entry = m_entries[m_head];
                if (!entry.dispatchObserved)
                {
                    ++m_counters.commitMismatches;
                    m_valid = false;
                }
                else if (!entry.commitObserved)
                {
                    entry.commitObserved = true;
                    entry.commitSequence = commitSequence;
                    ++m_counters.commitMatched;
                }
                else if (entry.commitSequence != commitSequence)
                {
                    ++m_counters.commitMismatches;
                    m_valid = false;
                }
                m_commitPC = entry.sourcePC;
            }
        }
    }
}

void NCPreparedBlockQueueShadow::PopHeadAsRetired() noexcept
{
    if (m_depth == 0U)
    {
        return;
    }

    if (m_entries[m_head].dispatchId ==
        m_epochAuthorizationDispatchId)
    {
        m_epochAuthorizationDispatchId = 0ULL;
        m_epochAuthorizationFrom = 0ULL;
    }

    m_entries[m_head] = NCPreparedBlockEntrySnapshot{};
    m_head = (m_head + 1U) % NC_PREPARED_BLOCK_QUEUE_CAPACITY;
    --m_depth;
    ++m_counters.retired;
}

void NCPreparedBlockQueueShadow::RetireProvenHeadBeforeCutover() noexcept
{
    if (m_depth == 0U)
    {
        return;
    }

    const NCPreparedBlockEntrySnapshot& head = m_entries[m_head];
    if (head.dispatchObserved && head.commitObserved)
    {
        PopHeadAsRetired();
    }
}

void NCPreparedBlockQueueShadow::RetireBefore(
    int runtimeCurrentPC) noexcept
{
    while (m_depth > 0U)
    {
        NCPreparedBlockEntrySnapshot& entry = m_entries[m_head];
        if (entry.sourcePC >= runtimeCurrentPC)
        {
            break;
        }

        if (!entry.dispatchObserved || !entry.commitObserved)
        {
            ++m_counters.commitMismatches;
            m_valid = false;
        }

        PopHeadAsRetired();
    }

    if (m_depth < NC_PREPARED_BLOCK_QUEUE_CAPACITY)
    {
        m_capacityLatched = false;
    }
}

bool NCPreparedBlockQueueShadow::CanPrepare() const noexcept
{
    return
        m_active &&
        m_valid &&
        !m_barrierLatched &&
        !m_eofLatched &&
        !m_committedBaselinePlanningBlocked &&
        m_depth < NC_PREPARED_BLOCK_QUEUE_CAPACITY;
}

int NCPreparedBlockQueueShadow::GetNextPlanPC() const noexcept
{
    return m_nextPlanPC;
}

NCPreparedModalSnapshot
NCPreparedBlockQueueShadow::GetNextModalBefore() const noexcept
{
    if (m_depth == 0U)
    {
        return m_seedModal;
    }

    return m_entries[PhysicalIndex(m_depth - 1U)].modalAfter;
}

bool NCPreparedBlockQueueShadow::TryBuildLiteralBlock(
    const NCParsedBlock& parsedBlock,
    NCBlock& block) noexcept
{
    block = NCBlock{};
    block.isEmpty = parsedBlock.isEmpty;
    block.isBlockSkip = parsedBlock.isBlockSkip;

    if (parsedBlock.error != NCParseError::NONE ||
        parsedBlock.controlType != NCParsedControlType::NONE ||
        parsedBlock.dependsOnMacroState ||
        parsedBlock.gCount < 0 ||
        parsedBlock.gCount > NC_MAX_G_CODES_PER_BLOCK ||
        parsedBlock.mCount < 0 ||
        parsedBlock.mCount > NC_MAX_M_CODES_PER_BLOCK)
    {
        return false;
    }

    block.gCount = parsedBlock.gCount;
    block.hasG = block.gCount > 0;
    for (int i = 0; i < block.gCount; ++i)
    {
        int code = -1;
        if (!TryParseLiteralInteger(
            parsedBlock.gExpressions[static_cast<std::size_t>(i)],
            code))
        {
            return false;
        }
        block.gCodes[i] = code;
        if (i == 0)
        {
            block.gCode = code;
        }
    }

    block.mCount = parsedBlock.mCount;
    for (int i = 0; i < block.mCount; ++i)
    {
        int code = -1;
        if (!TryParseLiteralInteger(
            parsedBlock.mExpressions[static_cast<std::size_t>(i)],
            code))
        {
            return false;
        }
        block.mCode[i] = code;
    }

    for (int i = 0; i < 26; ++i)
    {
        if (!parsedBlock.hasParam[static_cast<std::size_t>(i)])
        {
            continue;
        }

        double value = 0.0;
        if (!TryParseLiteralDouble(
            parsedBlock.paramExpressions[static_cast<std::size_t>(i)],
            value))
        {
            return false;
        }
        block.hasParam[i] = true;
        block.param[i] = value;
    }

    return true;
}

NCPreparedBlockClassification
NCPreparedBlockQueueShadow::ClassifyLiteralBlock(
    const NCParsedBlock& parsedBlock,
    const NCBlock& block,
    const NCPreparedPanelSwitchImage& panel) noexcept
{
    NCPreparedBlockClassification result{};
    result.literalResolved = true;
    result.modalAfterValid = true;

    if (parsedBlock.error != NCParseError::NONE)
    {
        result.blockClass = NCPreparedBlockClass::INVALID;
        AddBarrier(
            result,
            NCPreparedBarrierKind::PARSE_ERROR,
            NC_PREPARED_BARRIER_FLAG_PARSE);
        return result;
    }

    if (block.isBlockSkip && panel.blockSkipEnabled)
    {
        result.blockClass = NCPreparedBlockClass::BLOCK_SKIP;
        AddBarrier(
            result,
            NCPreparedBarrierKind::BLOCK_SKIP,
            NC_PREPARED_BARRIER_FLAG_BLOCK_SKIP);
        return result;
    }

    if (block.isEmpty && block.gCount == 0 && block.mCount == 0)
    {
        result.blockClass = NCPreparedBlockClass::EMPTY;
        return result;
    }

    // Preserve the existing Runtime's pre-dispatch drain diagnostic even if
    // BuildExecutionPlan later rejects this same atomic G/M block.
    const bool legacyMDrain =
        block.mCount > 0 && IsProgramFlowMCode(block.mCode[0]);
    result.legacyDrainRequired =
        NCGCodeSemantics::IsBlockBarrier(block) ||
        legacyMDrain ||
        (panel.singleBlockEnabled &&
            IsLegacySingleBlockDrainCandidate(block));

    if (panel.singleBlockEnabled && IsSingleBlockCandidate(block))
    {
        AddBarrier(
            result,
            NCPreparedBarrierKind::SINGLE_BLOCK,
            NC_PREPARED_BARRIER_FLAG_SINGLE_BLOCK);
    }

    NCGCodeExecutionPlan plan{};
    NCGCodePlanError planError = NCGCodePlanError::NONE;
    int firstConflict = -1;
    int secondConflict = -1;
    if (!NCGCodeSemantics::BuildExecutionPlan(
        block,
        plan,
        planError,
        firstConflict,
        secondConflict))
    {
        (void)planError;
        (void)firstConflict;
        (void)secondConflict;
        result.blockClass = NCPreparedBlockClass::INVALID;
        AddBarrier(
            result,
            NCPreparedBarrierKind::SEMANTIC_ERROR,
            NC_PREPARED_BARRIER_FLAG_SEMANTIC);
        return result;
    }

    result.primaryGCode = plan.primaryActionCode;
    result.firstMCode = block.mCount > 0 ? block.mCode[0] : -1;

    bool hasPureModal = false;
    bool hasExplicitMotion = false;
    const int gCount = SafeGCodeCount(block);
    for (int i = 0; i < gCount; ++i)
    {
        const int code = StoredGCode(block, i);

        if (code == 0)
        {
            hasExplicitMotion = true;
            continue;
        }

        if (code == 17 || code == 18 || code == 19 ||
            code == 90 || code == 91)
        {
            hasPureModal = true;
            AddBarrier(result, NCPreparedBarrierKind::MODAL_SNAPSHOT,
                NC_PREPARED_BARRIER_FLAG_MODAL);
            continue;
        }

        if (code == 20 || code == 21 || code == 22 || code == 23 ||
            code == 43 || code == 44 || code == 49 ||
            code == 40 || code == 41 || code == 42 ||
            code == 15 || code == 16 || code == 162 || code == 163 ||
            IsExtendedWorkCoordinateCode(code))
        {
            AddBarrier(
                result,
                NCPreparedBarrierKind::MODAL_SNAPSHOT,
                NC_PREPARED_BARRIER_FLAG_MODAL);
            continue;
        }

        if (code == 68 || code == 69 || code == 168 || code == 169 ||
            code == 50 || code == 51 || code == 150 || code == 151 ||
            code == 7 || code == 28 || code == 30 || code == 32 ||
            code == 53 || code == 81 || code == 161)
        {
            NCGCodeDescriptor descriptor{};
            if (NCGCodeSemantics::TryGetDescriptor(code, descriptor) &&
                descriptor.motionAction)
            {
                hasExplicitMotion = true;
            }
            AddBarrier(
                result,
                NCPreparedBarrierKind::POSITION_DERIVED,
                NC_PREPARED_BARRIER_FLAG_POSITION);
            continue;
        }

        if (code == 10 || code == 92 || code == 160)
        {
            AddBarrier(
                result,
                NCPreparedBarrierKind::TABLE_WRITE,
                NC_PREPARED_BARRIER_FLAG_TABLE_WRITE);
            continue;
        }

        if (code == 4 || code == 12)
        {
            AddBarrier(
                result,
                NCPreparedBarrierKind::TIME_STOP,
                NC_PREPARED_BARRIER_FLAG_TIME_STOP);
            continue;
        }

        if (code == 65 || code == 66 || code == 67)
        {
            AddBarrier(
                result,
                NCPreparedBarrierKind::PROGRAM_FLOW,
                NC_PREPARED_BARRIER_FLAG_FLOW);
            continue;
        }

        // A descriptor may be added later without a K.1 planning policy.
        // Fence it instead of speculatively crossing it.
        AddBarrier(
            result,
            NCPreparedBarrierKind::SEMANTIC_ERROR,
            NC_PREPARED_BARRIER_FLAG_SEMANTIC);
    }

    for (int i = 0; i < block.mCount; ++i)
    {
        const int code = block.mCode[i];
        if (code == 0 || code == 1)
        {
            AddBarrier(
                result,
                NCPreparedBarrierKind::TIME_STOP,
                NC_PREPARED_BARRIER_FLAG_TIME_STOP);
        }
        else if (code == 2 || code == 30)
        {
            AddBarrier(
                result,
                NCPreparedBarrierKind::PROGRAM_END,
                NC_PREPARED_BARRIER_FLAG_END);
        }
        else if (code == 98 || code == 99)
        {
            AddBarrier(
                result,
                NCPreparedBarrierKind::PROGRAM_FLOW,
                NC_PREPARED_BARRIER_FLAG_FLOW);
        }
        else
        {
            AddBarrier(
                result,
                NCPreparedBarrierKind::AUX_IO,
                NC_PREPARED_BARRIER_FLAG_AUX);
        }
    }

    if (!hasExplicitMotion && plan.primaryActionCode < 0 &&
        HasAnyAxisAddress(block))
    {
        AddBarrier(
            result,
            NCPreparedBarrierKind::IMPLICIT_MOTION,
            NC_PREPARED_BARRIER_FLAG_IMPLICIT);
    }

    // H is table-dependent even when no explicit G43/G44 shares the block.
    // Never pre-resolve a later motion across an unproven tool selection.
    if (block.has('H'))
    {
        AddBarrier(result, NCPreparedBarrierKind::MODAL_SNAPSHOT,
            NC_PREPARED_BARRIER_FLAG_MODAL);
    }

    if (block.has('T'))
    {
        AddBarrier(
            result,
            NCPreparedBarrierKind::AUX_IO,
            NC_PREPARED_BARRIER_FLAG_AUX);
    }

    if (hasExplicitMotion)
    {
        result.blockClass = NCPreparedBlockClass::MOTION_SHADOW;
    }
    else if (result.barrierFlags != NC_PREPARED_BARRIER_FLAG_NONE)
    {
        result.blockClass =
            result.barrierKind == NCPreparedBarrierKind::PROGRAM_FLOW
            ? NCPreparedBlockClass::PROGRAM_CONTROL
            : NCPreparedBlockClass::AUXILIARY;
    }
    else if (hasPureModal)
    {
        result.blockClass = NCPreparedBlockClass::PURE_MODAL_COPY;
    }
    else
    {
        result.blockClass = NCPreparedBlockClass::AUXILIARY;
    }

    return result;
}

NCPreparedModalSnapshot NCPreparedBlockQueueShadow::ReduceModalSnapshot(
    const NCPreparedModalSnapshot& before,
    const NCBlock& block,
    bool& afterValid) noexcept
{
    NCPreparedModalSnapshot after = before;
    afterValid = before.imageValid;
    if (block.has('H')) afterValid = false;

    const int count = SafeGCodeCount(block);
    for (int i = 0; i < count; ++i)
    {
        const int code = StoredGCode(block, i);
        // Coordinate settings depend on the actual table revision. Never
        // predict a new translation from only a G-code or a table index.
        if ((code >= 54 && code <= 959 && IsExtendedWorkCoordinateCode(code)) ||
            code == 10 || code == 92 || code == 160 || code == 168 || code == 169 ||
            code == 20 || code == 21 || code == 17 || code == 18 || code == 19 ||
            code == 22 || code == 23 || code == 90 || code == 91 ||
            code == 40 || code == 41 || code == 42 || code == 43 || code == 44 || code == 49 ||
            code == 50 || code == 51 || code == 68 || code == 69 ||
            code == 150 || code == 151 || code == 162 || code == 163 || code == 15 || code == 16)
        {
            afterValid = false;
            after.imageValid = false;
        }
        if (code == 90 || code == 91) after.distanceMode = code;
        else if (code == 20 || code == 21) after.unitsMode = code;
        else if (code == 17 || code == 18 || code == 19) after.planeMode = code;
        else if (code == 22 || code == 23) after.storedStrokeMode = code;
        else if (IsExtendedWorkCoordinateCode(code)) after.workCoordinateCode = code;
        else if (code == 43 || code == 44 || code == 49)
        {
            int mode = 49, hCode = 0;
            if (!TryDecodeNCToolLengthSelection(code, block.has('H'),
                block.has('H') ? block.val('H') : 0.0, mode, hCode))
            {
                afterValid = false;
            }
            else
            {
                // The scalar diagnostic is decoded safely; the surrounding
                // fence still prevents prediction of a live H-table snapshot.
                after.toolLengthMode = mode;
                after.hCode = hCode;
            }
        }
        else if (code == 40 || code == 41 || code == 42)
        {
            int mode = 40, dCode = 0;
            if (!TryDecodeNCToolRadiusSelection(code, block.has('D'),
                block.has('D') ? block.val('D') : 0.0, mode, dCode))
            {
                afterValid = false;
            }
            else
            {
                // Diagnostic-only scalar decode; the table descriptor remains fenced.
                after.toolRadiusMode = mode;
                after.dCode = dCode;
            }
        }
        else if (code == 68)
        {
            // Diagnostic tags only. The complete captured descriptor above
            // (including its rotation centre) stays unchanged and invalidated;
            // a reducer never manufactures authority for a new G68 source.
            if (block.has('X') && block.has('Y') && block.has('R') &&
                std::isfinite(block.val('X')) && std::isfinite(block.val('Y')) &&
                std::isfinite(block.val('R')) && std::abs(block.val('R')) <= 360.0)
            {
                after.g68Active = true;
                after.g68Angle = block.val('R');
            }
        }
        else if (code == 69)
        {
            after.g68Active = false;
            after.g68Angle = 0.0;
        }
        else if (code == 168 || code == 169)
        {
            int mode = 169, wCode = 0;
            if (!TryDecodeNCWorkSelection(code, block.has('W'),
                block.has('W') ? block.val('W') : 0.0, mode, wCode))
            {
                afterValid = false;
            }
            else
            {
                // Safely decode only the diagnostic tag. The barrier above
                // still refuses prediction of a live WORK table row.
                after.g168Active = mode == 168;
                after.workpieceCode = wCode;
            }
        }
        else if (code == 51)
        {
            after.scalingActive = true;
            if (block.has('P')) after.scalingFactor = block.val('P');
        }
        else if (code == 50)
        {
            after.scalingActive = false;
            after.scalingFactor = 1.0;
        }
        else if (code == 150 || code == 151)
        {
            // Runtime maps address letters through its configured axis-name
            // table.  K.1 deliberately does not copy that mutable table, so
            // the fixed conventional mask below is diagnostic only.
            afterValid = false;
            bool hasMirrorAxis = false;
            for (int axis = 0; axis < 8; ++axis)
            {
                static const char axisLetters[8] =
                { 'X', 'Y', 'Z', 'A', 'B', 'C', 'U', 'V' };
                if (!block.has(axisLetters[axis])) continue;
                hasMirrorAxis = true;
                const std::uint8_t bit =
                    static_cast<std::uint8_t>(1U << axis);
                if (code == 151) after.mirrorMask |= bit;
                else after.mirrorMask &= static_cast<std::uint8_t>(~bit);
            }
            if (code == 150 && !hasMirrorAxis)
            {
                after.mirrorMask = 0U;
            }
        }
        else if (code == 15) after.polarActive = false;
        else if (code == 16) after.polarActive = true;
        else if (code == 162) after.cAxisOffsetRotationEnabled = true;
        else if (code == 163) after.cAxisOffsetRotationEnabled = false;
        else if (code == 66) after.modalMacroActive = true;
        else if (code == 67) after.modalMacroActive = false;

        if (code == 0 && block.has('F'))
        {
            after.g00OverrideRatio = block.val('F') / 100.0;
        }
        if (code == 0)
        {
            // K.1 deliberately performs no geometry or coordinate transform.
            // Scalar modals remain useful, but the future commanded endpoint
            // cannot be claimed exact after a prepared motion.
            after.commandedMCSValid = false;
        }
    }

    if (block.has('T'))
    {
        after.toolCode = static_cast<int>(block.val('T'));
    }

    after.imageValid = afterValid;
    return after;
}

bool NCPreparedBlockQueueShadow::PrepareParsedLine(
    int sourcePC,
    int sourceLineNumber,
    const NCParsedBlock& parsedBlock) noexcept
{
    if (!CanPrepare())
    {
        if (m_depth >= NC_PREPARED_BLOCK_QUEUE_CAPACITY)
        {
            m_capacityLatched = true;
            ++m_counters.overwritePrevented;
        }
        RefreshSnapshot();
        return false;
    }

    if (sourcePC != m_nextPlanPC)
    {
        ++m_counters.planDiscontinuities;
        m_valid = false;
        RefreshSnapshot();
        return false;
    }

    NCPreparedBlockEntrySnapshot entry{};
    entry.session = m_session;
    entry.entrySequence = AllocateNonZeroEntry(m_nextEntrySequence);
    entry.source = m_source;
    entry.sourcePC = sourcePC;
    entry.sourceLineNumber = sourceLineNumber;
    entry.modalBefore = GetNextModalBefore();

    if (parsedBlock.error != NCParseError::NONE)
    {
        entry.classification.blockClass = NCPreparedBlockClass::INVALID;
        entry.classification.barrierKind = NCPreparedBarrierKind::PARSE_ERROR;
        entry.classification.barrierFlags = NC_PREPARED_BARRIER_FLAG_PARSE;
        entry.classification.planningStopsHere = true;
    }
    else if (parsedBlock.isBlockSkip &&
        m_source.panel.blockSkipEnabled)
    {
        // Runtime skips the entire line before Resolve/control evaluation.
        // Preserve that priority even for /G00 X#1 or /IF...GOTO.
        entry.classification.blockClass =
            NCPreparedBlockClass::BLOCK_SKIP;
        entry.classification.barrierKind =
            NCPreparedBarrierKind::BLOCK_SKIP;
        entry.classification.barrierFlags =
            NC_PREPARED_BARRIER_FLAG_BLOCK_SKIP;
        entry.classification.planningStopsHere = true;
        entry.classification.modalAfterValid =
            entry.modalBefore.imageValid;
    }
    else if (parsedBlock.controlType != NCParsedControlType::NONE)
    {
        entry.classification.blockClass =
            NCPreparedBlockClass::PROGRAM_CONTROL;
        entry.classification.barrierKind =
            NCPreparedBarrierKind::PROGRAM_FLOW;
        entry.classification.barrierFlags =
            NC_PREPARED_BARRIER_FLAG_FLOW;
        entry.classification.planningStopsHere = true;
        entry.classification.legacyDrainRequired = true;
        entry.classification.runtimeGotoControl =
            parsedBlock.controlType == NCParsedControlType::GOTO;
    }
    else if (parsedBlock.dependsOnMacroState)
    {
        entry.classification.blockClass =
            NCPreparedBlockClass::INVALID;
        entry.classification.barrierKind =
            NCPreparedBarrierKind::DYNAMIC_EXPRESSION;
        entry.classification.barrierFlags =
            NC_PREPARED_BARRIER_FLAG_DYNAMIC;
        entry.classification.planningStopsHere = true;
        entry.classification.legacyDrainRequired = true;
    }
    else if (!TryBuildLiteralBlock(parsedBlock, entry.preparedBlock))
    {
        entry.classification.blockClass = NCPreparedBlockClass::INVALID;
        entry.classification.barrierKind =
            NCPreparedBarrierKind::DYNAMIC_EXPRESSION;
        entry.classification.barrierFlags =
            NC_PREPARED_BARRIER_FLAG_DYNAMIC;
        entry.classification.planningStopsHere = true;
    }
    else
    {
        entry.classification = ClassifyLiteralBlock(
            parsedBlock,
            entry.preparedBlock,
            m_source.panel);

        if (m_source.scope != NCProgramScope::MACRO &&
            entry.modalBefore.modalMacroActive &&
            !NCGCodeSemantics::Contains(entry.preparedBlock, 66) &&
            !NCGCodeSemantics::Contains(entry.preparedBlock, 67) &&
            IsModalMacroMotionCandidate(entry.preparedBlock))
        {
            AddBarrier(
                entry.classification,
                NCPreparedBarrierKind::PROGRAM_FLOW,
                NC_PREPARED_BARRIER_FLAG_FLOW);
        }
    }

    bool modalAfterValid = entry.modalBefore.imageValid;
    if (entry.classification.blockClass ==
        NCPreparedBlockClass::BLOCK_SKIP)
    {
        entry.modalAfter = entry.modalBefore;
        modalAfterValid = entry.modalBefore.imageValid;
    }
    else if (entry.classification.literalResolved &&
        entry.classification.blockClass != NCPreparedBlockClass::INVALID)
    {
        entry.modalAfter = ReduceModalSnapshot(
            entry.modalBefore,
            entry.preparedBlock,
            modalAfterValid);

        if (BarrierFlagsLeaveScalarModalUnproven(
            entry.classification.barrierFlags))
        {
            modalAfterValid = false;
            entry.modalAfter.imageValid = false;
        }
    }
    else
    {
        entry.modalAfter = entry.modalBefore;
        modalAfterValid = false;
        entry.modalAfter.imageValid = false;
    }
    entry.classification.modalAfterValid = modalAfterValid;

    const std::size_t tail = PhysicalIndex(m_depth);
    m_entries[tail] = entry;
    ++m_depth;
    ++m_nextPlanPC;
    ++m_counters.prepared;

    if (entry.classification.planningStopsHere)
    {
        m_barrierLatched = true;
        ++m_counters.barrierStops;
    }

    if (m_depth == NC_PREPARED_BLOCK_QUEUE_CAPACITY)
    {
        m_capacityLatched = true;
        ++m_counters.capacityStops;
    }

    RefreshSnapshot();
    return true;
}

void NCPreparedBlockQueueShadow::ObserveNaturalEOF(
    int sourcePC) noexcept
{
    if (!m_active || !m_valid || m_barrierLatched || m_eofLatched)
    {
        return;
    }

    if (sourcePC != m_nextPlanPC)
    {
        ++m_counters.planDiscontinuities;
        m_valid = false;
        RefreshSnapshot();
        return;
    }

    m_eofLatched = true;
    ++m_counters.eofStops;
    RefreshSnapshot();
}

void NCPreparedBlockQueueShadow::RefreshLatchesFromTail() noexcept
{
    m_barrierLatched = false;
    if (m_depth > 0U)
    {
        const NCPreparedBlockEntrySnapshot& tail =
            m_entries[PhysicalIndex(m_depth - 1U)];
        m_barrierLatched = tail.classification.planningStopsHere;
    }
}

void NCPreparedBlockQueueShadow::RefreshSnapshot() noexcept
{
    NCPreparedBlockQueueSnapshot snapshot{};
    snapshot.publicationSequence = m_snapshot.publicationSequence;
    snapshot.session = m_session;
    snapshot.source = m_source;
    snapshot.runtimeCurrentPC = m_runtimeCurrentPC;
    snapshot.nextPlanPC = m_nextPlanPC;
    snapshot.dispatchPC = m_dispatchPC;
    snapshot.commitPC = m_commitPC;
    snapshot.committedBaselinePC = m_committedBaselinePC;
    snapshot.depth = static_cast<std::uint32_t>(m_depth);
    snapshot.active = m_active;
    snapshot.valid = m_valid;
    snapshot.barrierLatched = m_barrierLatched;
    snapshot.eofLatched = m_eofLatched;
    snapshot.capacityLatched = m_capacityLatched;
    snapshot.committedBaselinePlanningBlocked =
        m_committedBaselinePlanningBlocked;
    snapshot.lastInvalidationReason = m_lastInvalidationReason;
    snapshot.seedModal = m_seedModal;
    snapshot.tailModal = m_seedModal;

    if (m_depth > 0U)
    {
        const NCPreparedBlockEntrySnapshot& tail =
            m_entries[PhysicalIndex(m_depth - 1U)];
        snapshot.tailPC = tail.sourcePC;
        snapshot.tailModal = tail.modalAfter;

        if (m_barrierLatched)
        {
            snapshot.barrierPC = tail.sourcePC;
            snapshot.barrierLineNumber = tail.sourceLineNumber;
            snapshot.barrierKind = tail.classification.barrierKind;
        }
    }

    snapshot.cursorOrderValid =
        (m_commitPC < 0 || m_dispatchPC < 0 || m_commitPC <= m_dispatchPC) &&
        (m_dispatchPC < 0 || m_nextPlanPC < 0 || m_dispatchPC < m_nextPlanPC) &&
        (m_runtimeCurrentPC < 0 || m_nextPlanPC < 0 ||
            m_runtimeCurrentPC <= m_nextPlanPC);
    snapshot.accountingValid =
        m_counters.prepared ==
        m_counters.retired +
        m_counters.invalidatedEntries +
        static_cast<std::uint64_t>(m_depth);

    if (!m_active)
    {
        snapshot.stopReason = NCPreparedQueueStopReason::INACTIVE;
    }
    else if (!m_valid || !snapshot.cursorOrderValid ||
        !snapshot.accountingValid)
    {
        snapshot.stopReason = NCPreparedQueueStopReason::FAIL_CLOSED;
    }
    else if (m_barrierLatched)
    {
        snapshot.stopReason = NCPreparedQueueStopReason::BARRIER;
    }
    else if (m_committedBaselinePlanningBlocked)
    {
        snapshot.stopReason =
            NCPreparedQueueStopReason::COMMITTED_BASELINE;
    }
    else if (m_eofLatched)
    {
        snapshot.stopReason = NCPreparedQueueStopReason::EOF_REACHED;
        snapshot.barrierKind = NCPreparedBarrierKind::NATURAL_EOF;
        snapshot.barrierPC = m_nextPlanPC;
    }
    else if (m_capacityLatched)
    {
        snapshot.stopReason = NCPreparedQueueStopReason::CAPACITY;
    }
    else
    {
        snapshot.stopReason = NCPreparedQueueStopReason::NONE;
    }

    m_snapshot = snapshot;
}

void NCPreparedBlockQueueShadow::FinishObservation() noexcept
{
    RefreshSnapshot();
    m_snapshot.publicationSequence = m_nextPublicationSequence++;
    if (m_snapshot.publicationSequence == 0ULL)
    {
        m_snapshot.publicationSequence = m_nextPublicationSequence++;
    }
    ++m_counters.publications;
}

const char* NCPreparedBlockClassToDiagnosticName(
    NCPreparedBlockClass value) noexcept
{
    switch (value)
    {
    case NCPreparedBlockClass::EMPTY: return "EMPTY";
    case NCPreparedBlockClass::BLOCK_SKIP: return "BLOCK_SKIP";
    case NCPreparedBlockClass::PURE_MODAL_COPY: return "PURE_MODAL";
    case NCPreparedBlockClass::MOTION_SHADOW: return "MOTION";
    case NCPreparedBlockClass::PROGRAM_CONTROL: return "CONTROL";
    case NCPreparedBlockClass::AUXILIARY: return "AUX";
    case NCPreparedBlockClass::INVALID: return "INVALID";
    case NCPreparedBlockClass::NONE:
    default: return "NONE";
    }
}

const char* NCPreparedBarrierKindToDiagnosticName(
    NCPreparedBarrierKind value) noexcept
{
    switch (value)
    {
    case NCPreparedBarrierKind::BLOCK_SKIP: return "BLOCK_SKIP";
    case NCPreparedBarrierKind::SINGLE_BLOCK: return "SINGLE_BLOCK";
    case NCPreparedBarrierKind::MODAL_SNAPSHOT: return "MODAL";
    case NCPreparedBarrierKind::POSITION_DERIVED: return "POSITION";
    case NCPreparedBarrierKind::TABLE_WRITE: return "TABLE_WRITE";
    case NCPreparedBarrierKind::TIME_STOP: return "TIME_STOP";
    case NCPreparedBarrierKind::PROGRAM_FLOW: return "FLOW";
    case NCPreparedBarrierKind::PROGRAM_END: return "PROGRAM_END";
    case NCPreparedBarrierKind::AUX_IO: return "AUX_IO";
    case NCPreparedBarrierKind::IMPLICIT_MOTION: return "IMPLICIT";
    case NCPreparedBarrierKind::DYNAMIC_EXPRESSION: return "DYNAMIC";
    case NCPreparedBarrierKind::PARSE_ERROR: return "PARSE_ERROR";
    case NCPreparedBarrierKind::SEMANTIC_ERROR: return "SEMANTIC_ERROR";
    case NCPreparedBarrierKind::NATURAL_EOF: return "EOF";
    case NCPreparedBarrierKind::NONE:
    default: return "NONE";
    }
}

const char* NCPreparedQueueStopReasonToDiagnosticName(
    NCPreparedQueueStopReason value) noexcept
{
    switch (value)
    {
    case NCPreparedQueueStopReason::INACTIVE: return "INACTIVE";
    case NCPreparedQueueStopReason::COMMITTED_BASELINE: return "BASELINE";
    case NCPreparedQueueStopReason::BARRIER: return "BARRIER";
    case NCPreparedQueueStopReason::CAPACITY: return "CAPACITY";
    case NCPreparedQueueStopReason::EOF_REACHED: return "EOF";
    case NCPreparedQueueStopReason::FAIL_CLOSED: return "FAIL_CLOSED";
    case NCPreparedQueueStopReason::NONE:
    default: return "NONE";
    }
}

const char* NCPreparedInvalidationReasonToDiagnosticName(
    NCPreparedInvalidationReason value) noexcept
{
    switch (value)
    {
    case NCPreparedInvalidationReason::NOT_RUNNING: return "NOT_RUNNING";
    case NCPreparedInvalidationReason::ALARM: return "ALARM";
    case NCPreparedInvalidationReason::RESET: return "RESET";
    case NCPreparedInvalidationReason::PROGRAM_END: return "PROGRAM_END";
    case NCPreparedInvalidationReason::SOURCE_REPLACED: return "SOURCE";
    case NCPreparedInvalidationReason::EXECUTION_EPOCH_CHANGED: return "EPOCH";
    case NCPreparedInvalidationReason::OWNER_CHANGED: return "OWNER";
    case NCPreparedInvalidationReason::MODE_CHANGED: return "MODE";
    case NCPreparedInvalidationReason::FRAME_CHANGED: return "FRAME";
    case NCPreparedInvalidationReason::PANEL_SWITCH_CHANGED: return "PANEL";
    case NCPreparedInvalidationReason::CURSOR_DISCONTINUITY: return "CURSOR";
    case NCPreparedInvalidationReason::IDENTITY_INVALID: return "IDENTITY";
    case NCPreparedInvalidationReason::NONE:
    default: return "NONE";
    }
}
