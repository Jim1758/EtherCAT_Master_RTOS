#include "NCPathCoreCommittedRun.h"
#include <cmath>
#include <cstring>
#include <limits>

namespace
{
    bool FullIdentity(const MotionExecutionIdentity& a,
        const MotionExecutionIdentity& b) noexcept
    {
        return a.epoch == b.epoch && a.segmentId == b.segmentId &&
            a.sourceBlockId == b.sourceBlockId && a.source == b.source;
    }

    bool ValidReceipt(const MotionCommandedEndpointReceiptV1& receipt,
        const MotionOwnerLease& ownerLease) noexcept
    {
        const MotionQueueTailCommitReceipt& transaction = receipt.transaction;
        if (!receipt.valid || receipt.schemaVersion != 1U ||
            !transaction.IsCommitted() || !transaction.captureBound ||
            transaction.identity.source != MotionCommandSource::NC_MEMORY ||
            transaction.identity.sourceBlockId < 0 ||
            !transaction.ownerLease.Matches(ownerLease) ||
            receipt.validAxisMask == 0U ||
            (receipt.validAxisMask & ~0xffU) != 0U ||
            (transaction.axisMask & ~receipt.validAxisMask) != 0U ||
            (receipt.origin != MotionCommandedBaselineOrigin::BUFFERED_TAIL &&
                receipt.origin != MotionCommandedBaselineOrigin::ABORTING_BASELINE))
            return false;
        for (std::size_t axis = 0U; axis < 8U; ++axis)
        {
            if (!std::isfinite(receipt.startMCS[axis]) ||
                !std::isfinite(receipt.endMCS[axis])) return false;
        }
        return true;
    }

    bool SameSeam(const MotionCommandedEndpointReceiptV1& before,
        const MotionCommandedEndpointReceiptV1& after) noexcept
    {
        return before.validAxisMask == after.validAxisMask &&
            std::memcmp(before.endMCS.data(), after.startMCS.data(),
                sizeof(double) * 8U) == 0;
    }
}

void NCPathCoreCommittedScopeV1::Clear() noexcept
{
    ownerTag = 0ULL;
    runToken = 0ULL;
    cacheGeneration = 0ULL;
    programScope = 0U;
    operationMode = 0U;
    ownerLease = MotionOwnerLease{};
}

bool NCPathCoreCommittedScopeV1::IsValid() const noexcept
{
    return ownerTag != 0ULL && runToken != 0ULL && cacheGeneration != 0ULL &&
        programScope == 1U && operationMode == 0U &&
        ownerLease.owner == MotionOwner::AUTO && ownerLease.IsValid();
}

bool NCPathCoreCommittedScopeV1::Matches(
    const NCPathCoreCommittedScopeV1& other) const noexcept
{
    return ownerTag == other.ownerTag && runToken == other.runToken &&
        cacheGeneration == other.cacheGeneration &&
        programScope == other.programScope && operationMode == other.operationMode &&
        ownerLease.Matches(other.ownerLease);
}

void NCPathCoreCommittedRecordV1::Clear() noexcept
{
    receipt.Clear();
    dispatchId = 0ULL;
    commitSequence = 0ULL;
    lastSequence = 0ULL;
    errorCode = 0U;
    boundaryFlags = 0U;
    terminalType = MotionFeedbackType::NONE;
    lastType = MotionFeedbackType::NONE;
    rejectReason = MotionRejectReason::NONE;
    consumerAccepted = false;
    started = false;
    for (std::uint8_t& value : reserved) value = 0U;
}

void NCPathCoreCommittedSampleV1::Clear() noexcept
{
    positionMCS.fill(0.0);
    derivativeMCS.fill(0.0);
}

bool NCPathCoreCommittedRun::IsObserving() const noexcept
{
    return m_runOpen && m_observeOpen;
}

bool NCPathCoreCommittedRun::Arm(const NCPathCoreCommittedScopeV1& scope,
    MotionFeedbackSequence lastConsumedSequence) noexcept
{
    // In particular, a rejected Arm cannot erase an already published result.
    if (!scope.IsValid() || m_runOpen || scope.runToken <= m_scope.runToken ||
        (m_scope.ownerTag != 0ULL && scope.ownerTag != m_scope.ownerTag)) return false;
    for (NCPathCoreCommittedRecordV1& row : m_rows) row.Clear();
    m_scope = scope;
    m_lastSequence = lastConsumedSequence;
    m_count = 0U;
    m_epochChanges = 0U;
    m_discontinuities = 0U;
    m_holds = 0U;
    m_resumes = 0U;
    m_state = NCPathCoreCommittedRunState::TRACKING;
    m_fault = NCPathCoreCommittedRunFault::NONE;
    m_held = false;
    m_afterHold = false;
    m_runOpen = true;
    m_observeOpen = true;
    return true;
}

void NCPathCoreCommittedRun::Fail(NCPathCoreCommittedRunFault reason) noexcept
{
    if (!m_runOpen || reason == NCPathCoreCommittedRunFault::NONE) return;
    if (m_fault == NCPathCoreCommittedRunFault::NONE) m_fault = reason;
    m_state = NCPathCoreCommittedRunState::FAULTED;
    // Capacity only seals new geometry. Feedback for retained rows remains
    // useful diagnostic evidence; every other failure freezes it immediately.
    if (reason != NCPathCoreCommittedRunFault::CAPACITY) m_observeOpen = false;
}

bool NCPathCoreCommittedRun::CheckScope(
    const NCPathCoreCommittedScopeV1& scope) noexcept
{
    if (!m_runOpen) return false;
    if (!scope.IsValid() || !m_scope.Matches(scope))
    {
        Fail(NCPathCoreCommittedRunFault::SCOPE);
        return false;
    }
    return m_fault == NCPathCoreCommittedRunFault::NONE &&
        (m_state == NCPathCoreCommittedRunState::TRACKING ||
            m_state == NCPathCoreCommittedRunState::PREPARED);
}

bool NCPathCoreCommittedRun::Admit(const MotionCommandedEndpointReceiptV1& receipt,
    std::uint64_t dispatchId, std::uint64_t commitSequence) noexcept
{
    if (!m_runOpen || m_state != NCPathCoreCommittedRunState::TRACKING) return false;
    if (m_held)
    {
        Fail(NCPathCoreCommittedRunFault::TRANSITION);
        return false;
    }
    if (m_count == 32U)
    {
        Fail(NCPathCoreCommittedRunFault::CAPACITY);
        return false;
    }
    if (!ValidReceipt(receipt, m_scope.ownerLease) ||
        dispatchId == 0ULL || commitSequence == 0ULL)
    {
        Fail(NCPathCoreCommittedRunFault::RECEIPT);
        return false;
    }
    if (m_count != 0U)
    {
        const NCPathCoreCommittedRecordV1& previous = m_rows[m_count - 1U];
        if (dispatchId <= previous.dispatchId || commitSequence <= previous.commitSequence ||
            receipt.transaction.transactionSequence <=
            previous.receipt.transaction.transactionSequence)
        {
            Fail(NCPathCoreCommittedRunFault::SEQUENCE);
            return false;
        }
    }
    for (std::uint32_t index = 0U; index < m_count; ++index)
    {
        if (m_rows[index].receipt.transaction.identity.Matches(receipt.transaction.identity))
        {
            Fail(NCPathCoreCommittedRunFault::IDENTITY);
            return false;
        }
    }
    NCPathCoreCommittedRecordV1& row = m_rows[m_count];
    row.receipt = receipt;
    row.dispatchId = dispatchId;
    row.commitSequence = commitSequence;
    if (m_count == 0U) row.boundaryFlags |= NC_PATH_CORE_COMMITTED_FIRST;
    else
    {
        const NCPathCoreCommittedRecordV1& previous = m_rows[m_count - 1U];
        if (receipt.transaction.identity.epoch != previous.receipt.transaction.identity.epoch)
        {
            row.boundaryFlags |= NC_PATH_CORE_COMMITTED_EPOCH_CHANGED;
            ++m_epochChanges;
        }
        if (!SameSeam(previous.receipt, receipt))
        {
            row.boundaryFlags |= NC_PATH_CORE_COMMITTED_DISCONTINUITY;
            ++m_discontinuities;
        }
    }
    if (m_afterHold) row.boundaryFlags |= NC_PATH_CORE_COMMITTED_AFTER_HOLD;
    m_afterHold = false;
    ++m_count;
    return true;
}

void NCPathCoreCommittedRun::Observe(const MotionFeedbackEvent& event,
    bool ledgerAccepted) noexcept
{
    if (!IsObserving()) return;
    const MotionFeedbackSequence expected = m_lastSequence ==
        (std::numeric_limits<MotionFeedbackSequence>::max)() ? 1ULL : m_lastSequence + 1ULL;
    // As in the existing BO observer, a zero seed establishes its first observed
    // nonzero watermark. Every subsequent event must be exactly consecutive.
    if (event.sequence == 0ULL || (m_lastSequence != 0ULL && event.sequence != expected))
    {
        Fail(NCPathCoreCommittedRunFault::SEQUENCE);
        return;
    }
    m_lastSequence = event.sequence;
    for (std::uint32_t index = 0U; index < m_count; ++index)
    {
        NCPathCoreCommittedRecordV1& row = m_rows[index];
        const MotionQueueTailCommitReceipt& transaction = row.receipt.transaction;
        if (!transaction.identity.Matches(event.identity)) continue;
        if (!FullIdentity(transaction.identity, event.identity) ||
            transaction.ownerLease.owner != event.owner ||
            transaction.ownerLease.generation != event.ownerGeneration)
        {
            Fail(NCPathCoreCommittedRunFault::IDENTITY);
            return;
        }
        if (!ledgerAccepted)
        {
            Fail(NCPathCoreCommittedRunFault::LEDGER);
            return;
        }
        if (row.terminalType != MotionFeedbackType::NONE)
        {
            Fail(NCPathCoreCommittedRunFault::TRANSITION);
            return;
        }
        bool valid = false;
        switch (event.type)
        {
        case MotionFeedbackType::ACCEPTED:
            valid = row.lastType == MotionFeedbackType::NONE;
            if (valid) row.consumerAccepted = true;
            break;
        case MotionFeedbackType::STARTED:
            valid = row.consumerAccepted && !row.started;
            if (valid) row.started = true;
            break;
        case MotionFeedbackType::PROGRESS:
            valid = row.consumerAccepted && row.started;
            break;
        case MotionFeedbackType::HELD:
            valid = row.consumerAccepted && row.lastType != MotionFeedbackType::HELD;
            break;
        case MotionFeedbackType::RESUMED:
            valid = row.lastType == MotionFeedbackType::HELD;
            break;
        case MotionFeedbackType::COMPLETED:
            valid = row.consumerAccepted;
            if (valid) row.terminalType = event.type;
            break;
        case MotionFeedbackType::REJECTED:
        case MotionFeedbackType::CANCELLED:
        case MotionFeedbackType::ABORTED:
        case MotionFeedbackType::FAULTED:
            valid = true;
            row.terminalType = event.type;
            break;
        case MotionFeedbackType::NONE:
        default:
            break;
        }
        if (!valid)
        {
            Fail(NCPathCoreCommittedRunFault::TRANSITION);
            return;
        }
        row.lastSequence = event.sequence;
        row.lastType = event.type;
        row.errorCode = event.errorCode;
        row.rejectReason = event.rejectReason;
        return;
    }
}

void NCPathCoreCommittedRun::CheckTransport(bool healthy) noexcept
{
    // Prepared data is still a candidate: a fresh transport-health failure
    // before the caller's final publication gate must revoke that candidate.
    if (!healthy && m_runOpen) Fail(NCPathCoreCommittedRunFault::TRANSPORT);
}

void NCPathCoreCommittedRun::Hold() noexcept
{
    if (!m_runOpen || m_state != NCPathCoreCommittedRunState::TRACKING || m_held) return;
    if (m_holds == (std::numeric_limits<std::uint32_t>::max)())
    {
        Fail(NCPathCoreCommittedRunFault::SEQUENCE);
        return;
    }
    m_held = true;
    ++m_holds;
}

void NCPathCoreCommittedRun::Resume() noexcept
{
    if (!m_runOpen || m_state != NCPathCoreCommittedRunState::TRACKING || !m_held) return;
    if (m_resumes == (std::numeric_limits<std::uint32_t>::max)())
    {
        Fail(NCPathCoreCommittedRunFault::SEQUENCE);
        return;
    }
    m_held = false;
    m_afterHold = true;
    ++m_resumes;
}

void NCPathCoreCommittedRun::Close() noexcept
{
    m_runOpen = false;
    m_observeOpen = false;
    if (m_state != NCPathCoreCommittedRunState::IDLE &&
        m_state != NCPathCoreCommittedRunState::FAULTED)
        m_state = NCPathCoreCommittedRunState::CLOSED;
}

bool NCPathCoreCommittedRun::Prepare() noexcept
{
    if (!m_runOpen || m_state != NCPathCoreCommittedRunState::TRACKING ||
        m_fault != NCPathCoreCommittedRunFault::NONE || m_held || m_count == 0U)
        return false;
    for (std::uint32_t index = 0U; index < m_count; ++index)
    {
        if (!m_rows[index].consumerAccepted ||
            m_rows[index].terminalType != MotionFeedbackType::COMPLETED ||
            m_rows[index].errorCode != 0U ||
            m_rows[index].rejectReason != MotionRejectReason::NONE) return false;
    }
    m_state = NCPathCoreCommittedRunState::PREPARED;
    m_observeOpen = false;
    return true;
}

bool NCPathCoreCommittedRun::Publish() noexcept
{
    if (!m_runOpen || m_state != NCPathCoreCommittedRunState::PREPARED ||
        m_fault != NCPathCoreCommittedRunFault::NONE) return false;
    m_state = NCPathCoreCommittedRunState::PUBLISHED;
    m_runOpen = false;
    return true;
}

bool NCPathCoreCommittedRun::ReadLastCompleted(
    NCPathCoreCommittedRecordV1& output) const noexcept
{
    // The indexed helper also clears output when the run has no retained row.
    return ReadCompleted(m_count == 0U ? 32U : m_count - 1U, output);
}

bool NCPathCoreCommittedRun::ReadCompleted(std::uint32_t index,
    NCPathCoreCommittedRecordV1& output) const noexcept
{
    output.Clear();
    if (m_state != NCPathCoreCommittedRunState::TRACKING ||
        !m_runOpen || !m_observeOpen ||
        m_fault != NCPathCoreCommittedRunFault::NONE || m_held ||
        m_count == 0U || m_count > 32U || index >= m_count) return false;
    for (std::uint32_t rowIndex = 0U; rowIndex < m_count; ++rowIndex)
    {
        const NCPathCoreCommittedRecordV1& row = m_rows[rowIndex];
        if (!row.consumerAccepted ||
            row.terminalType != MotionFeedbackType::COMPLETED ||
            row.errorCode != 0U || row.rejectReason != MotionRejectReason::NONE)
            return false;
    }
    output = m_rows[index];
    return true;
}

bool NCPathCoreCommittedRun::Read(std::uint32_t index,
    NCPathCoreCommittedRecordV1& output) const noexcept
{
    output.Clear();
    if (m_state != NCPathCoreCommittedRunState::PUBLISHED ||
        m_fault != NCPathCoreCommittedRunFault::NONE || index >= m_count) return false;
    output = m_rows[index];
    return true;
}

bool NCPathCoreCommittedRun::Evaluate(std::uint32_t index, double u,
    NCPathCoreCommittedSampleV1& output) const noexcept
{
    output.Clear();
    if (m_state != NCPathCoreCommittedRunState::PUBLISHED ||
        m_fault != NCPathCoreCommittedRunFault::NONE || index >= m_count ||
        !std::isfinite(u) || u < 0.0 || u > 1.0) return false;
    const MotionCommandedEndpointReceiptV1& receipt = m_rows[index].receipt;
    for (std::size_t axis = 0U; axis < 8U; ++axis)
    {
        const double derivative = receipt.endMCS[axis] - receipt.startMCS[axis];
        const double position = u == 0.0 ? receipt.startMCS[axis] :
            (u == 1.0 ? receipt.endMCS[axis] : receipt.startMCS[axis] + u * derivative);
        if (!std::isfinite(derivative) || !std::isfinite(position))
        {
            output.Clear();
            return false;
        }
        output.positionMCS[axis] = position;
        output.derivativeMCS[axis] = derivative;
    }
    return true;
}

void NCPathCoreCommittedRun::Describe(NCPathCoreCommittedRunStatusV1& output) const noexcept
{
    output = NCPathCoreCommittedRunStatusV1{};
    output.ownerTag = m_scope.ownerTag;
    output.runToken = m_scope.runToken;
    output.lastSequence = m_lastSequence;
    output.count = m_count;
    output.epochChanges = m_epochChanges;
    output.discontinuities = m_discontinuities;
    output.holds = m_holds;
    output.resumes = m_resumes;
    output.state = m_state;
    output.fault = m_fault;
    output.held = m_held;
    for (std::uint32_t index = 0U; index < m_count; ++index)
    {
        const NCPathCoreCommittedRecordV1& row = m_rows[index];
        if (row.consumerAccepted) ++output.accepted;
        if (row.started) ++output.started;
        if (row.terminalType == MotionFeedbackType::COMPLETED) ++output.completed;
        else if (row.terminalType != MotionFeedbackType::NONE) ++output.failed;
        else ++output.pending;
    }
}
