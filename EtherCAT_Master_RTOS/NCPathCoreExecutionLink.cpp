#include "NCPathCoreExecutionLink.h"
#include <limits>

namespace
{
    bool SameHandle(const NCPathCoreCommandedChordStoreHandleV1& a,
        const NCPathCoreCommandedChordStoreHandleV1& b) noexcept
    {
        return a.ownerTag == b.ownerTag && a.lifetime == b.lifetime &&
            a.ordinal == b.ordinal && a.reserved == b.reserved &&
            a.localIdentity.geometryPublicationSequence ==
            b.localIdentity.geometryPublicationSequence &&
            a.localIdentity.acceptedInputChainGeneration ==
            b.localIdentity.acceptedInputChainGeneration;
    }

    bool FullIdentity(const MotionExecutionIdentity& a,
        const MotionExecutionIdentity& b) noexcept
    {
        return a.epoch == b.epoch && a.segmentId == b.segmentId &&
            a.sourceBlockId == b.sourceBlockId && a.source == b.source;
    }
}

void NCPathCoreExecutionRecordV1::Clear() noexcept
{
    handle.Clear();
    identity = MotionExecutionIdentity{};
    ownerLease = MotionOwnerLease{};
    lastSequence = 0ULL;
    errorCode = 0U;
    lastType = MotionFeedbackType::NONE;
    terminalType = MotionFeedbackType::NONE;
    rejectReason = MotionRejectReason::NONE;
    consumerAccepted = false;
    started = false;
    for (std::uint8_t& value : reserved) value = 0U;
}

bool NCPathCoreExecutionLink::IsObserving() const noexcept
{
    return m_runOpen && (m_state == NCPathCoreExecutionLinkState::TRACKING ||
        m_state == NCPathCoreExecutionLinkState::SEALED);
}

void NCPathCoreExecutionLink::Fail(NCPathCoreExecutionLinkFault reason) noexcept
{
    if (!m_runOpen || m_fault != NCPathCoreExecutionLinkFault::NONE) return;
    m_fault = reason;
    m_state = NCPathCoreExecutionLinkState::FAULTED;
}

bool NCPathCoreExecutionLink::Arm(std::uint64_t ownerTag,
    std::uint64_t runToken, MotionFeedbackSequence lastConsumedSequence) noexcept
{
    if (runToken == 0ULL || runToken <= m_runToken) return false;
    // Do not silently discard a still-open run, even for a newer token.
    if (m_runOpen)
    {
        Fail(NCPathCoreExecutionLinkFault::INVALID_SCOPE);
        return false;
    }
    for (NCPathCoreExecutionRecordV1& row : m_rows) row.Clear();
    m_runToken = runToken;
    m_lifetime = 0ULL;
    m_lastSequence = lastConsumedSequence;
    m_count = 0U;
    m_state = NCPathCoreExecutionLinkState::TRACKING;
    m_fault = NCPathCoreExecutionLinkFault::NONE;
    m_retentionReason = 0U;
    m_endReason = 0U;
    m_runOpen = true;
    if (ownerTag == 0ULL || (m_ownerTag != 0ULL && ownerTag != m_ownerTag))
    {
        Fail(NCPathCoreExecutionLinkFault::INVALID_SCOPE);
        return false;
    }
    m_ownerTag = ownerTag;
    return true;
}

bool NCPathCoreExecutionLink::Bind(
    const NCPathCoreCommandedChordStoreHandleV1& handle,
    const MotionExecutionIdentity& identity, const MotionOwnerLease& ownerLease) noexcept
{
    if (!m_runOpen || m_state != NCPathCoreExecutionLinkState::TRACKING) return false;
    if (handle.ownerTag != m_ownerTag || handle.lifetime == 0ULL ||
        handle.ordinal == 0U || handle.reserved != 0U ||
        handle.localIdentity.geometryPublicationSequence == 0ULL ||
        handle.localIdentity.acceptedInputChainGeneration == 0ULL ||
        !identity.IsAssigned() || identity.source != MotionCommandSource::NC_MEMORY ||
        identity.sourceBlockId < 0 || ownerLease.owner != MotionOwner::AUTO ||
        !ownerLease.IsValid())
    {
        Fail(NCPathCoreExecutionLinkFault::INVALID_BINDING);
        return false;
    }
    if ((m_count == 0U && handle.lifetime <= m_lastLifetime) ||
        (m_count != 0U && handle.lifetime != m_lifetime))
    {
        Fail(NCPathCoreExecutionLinkFault::LIFETIME);
        return false;
    }
    if (handle.ordinal <= m_count)
    {
        const NCPathCoreExecutionRecordV1& row = m_rows[handle.ordinal - 1U];
        if (SameHandle(row.handle, handle) && FullIdentity(row.identity, identity) &&
            row.ownerLease.Matches(ownerLease)) return true;
        Fail(NCPathCoreExecutionLinkFault::HANDLE_CONFLICT);
        return false;
    }
    if (m_count == 32U)
    {
        Fail(NCPathCoreExecutionLinkFault::CAPACITY);
        return false;
    }
    if (handle.ordinal != m_count + 1U || (m_count != 0U &&
        (identity.epoch != m_rows[0U].identity.epoch ||
            !ownerLease.Matches(m_rows[0U].ownerLease) ||
            handle.localIdentity.acceptedInputChainGeneration !=
            m_rows[0U].handle.localIdentity.acceptedInputChainGeneration)))
    {
        Fail(NCPathCoreExecutionLinkFault::INVALID_BINDING);
        return false;
    }
    for (std::uint32_t index = 0U; index < m_count; ++index)
    {
        if (m_rows[index].identity.Matches(identity) ||
            m_rows[index].handle.localIdentity.geometryPublicationSequence ==
            handle.localIdentity.geometryPublicationSequence)
        {
            Fail(NCPathCoreExecutionLinkFault::IDENTITY_CONFLICT);
            return false;
        }
    }
    NCPathCoreExecutionRecordV1& row = m_rows[m_count];
    row.handle = handle;
    row.identity = identity;
    row.ownerLease = ownerLease;
    if (m_count == 0U)
    {
        m_lifetime = handle.lifetime;
        m_lastLifetime = handle.lifetime;
    }
    ++m_count;
    return true;
}

void NCPathCoreExecutionLink::CheckTransport(bool healthy) noexcept
{
    if (!healthy && IsObserving()) Fail(NCPathCoreExecutionLinkFault::TRANSPORT);
}

void NCPathCoreExecutionLink::Observe(const MotionFeedbackEvent& event,
    bool ledgerAccepted) noexcept
{
    if (!IsObserving()) return;
    const MotionFeedbackSequence expected = m_lastSequence ==
        (std::numeric_limits<MotionFeedbackSequence>::max)() ? 1ULL : m_lastSequence + 1ULL;
    if (event.sequence == 0ULL || (m_lastSequence != 0ULL && event.sequence != expected))
    {
        Fail(NCPathCoreExecutionLinkFault::SEQUENCE);
        return;
    }
    m_lastSequence = event.sequence;
    for (std::uint32_t index = 0U; index < m_count; ++index)
    {
        NCPathCoreExecutionRecordV1& row = m_rows[index];
        if (!row.identity.Matches(event.identity)) continue;
        if (!FullIdentity(row.identity, event.identity) ||
            row.ownerLease.owner != event.owner ||
            row.ownerLease.generation != event.ownerGeneration)
        {
            Fail(NCPathCoreExecutionLinkFault::IDENTITY_CONFLICT);
            return;
        }
        if (!ledgerAccepted)
        {
            Fail(NCPathCoreExecutionLinkFault::LEDGER);
            return;
        }
        if (row.terminalType != MotionFeedbackType::NONE)
        {
            Fail(NCPathCoreExecutionLinkFault::TRANSITION);
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
            Fail(NCPathCoreExecutionLinkFault::TRANSITION);
            return;
        }
        row.lastSequence = event.sequence;
        row.lastType = event.type;
        row.errorCode = event.errorCode;
        row.rejectReason = event.rejectReason;
        return;
    }
}

void NCPathCoreExecutionLink::Seal(std::uint8_t retentionReason) noexcept
{
    if (!m_runOpen) return;
    if (m_retentionReason == 0U) m_retentionReason = retentionReason;
    if (m_state == NCPathCoreExecutionLinkState::TRACKING)
        m_state = NCPathCoreExecutionLinkState::SEALED;
}

bool NCPathCoreExecutionLink::Close(std::uint8_t endReason) noexcept
{
    if (!m_runOpen) return false;
    m_runOpen = false;
    m_endReason = endReason;
    if (m_state != NCPathCoreExecutionLinkState::FAULTED)
        m_state = NCPathCoreExecutionLinkState::CLOSED;
    return true;
}

bool NCPathCoreExecutionLink::Read(
    const NCPathCoreCommandedChordStoreHandleV1& handle,
    NCPathCoreExecutionRecordV1& output) const noexcept
{
    output.Clear();
    if (!IsObserving() || handle.ordinal == 0U || handle.ordinal > m_count) return false;
    const NCPathCoreExecutionRecordV1& row = m_rows[handle.ordinal - 1U];
    if (!SameHandle(row.handle, handle)) return false;
    output = row;
    return true;
}

void NCPathCoreExecutionLink::Describe(NCPathCoreExecutionLinkStatusV1& output) const noexcept
{
    output = NCPathCoreExecutionLinkStatusV1{};
    output.ownerTag = m_ownerTag;
    output.runToken = m_runToken;
    output.lifetime = m_lifetime;
    output.lastSequence = m_lastSequence;
    output.bound = m_count;
    output.state = m_state;
    output.fault = m_fault;
    output.retentionReason = m_retentionReason;
    output.endReason = m_endReason;
    for (std::uint32_t index = 0U; index < m_count; ++index)
    {
        const NCPathCoreExecutionRecordV1& row = m_rows[index];
        if (row.consumerAccepted) ++output.accepted;
        if (row.started) ++output.started;
        if (row.terminalType == MotionFeedbackType::COMPLETED) ++output.completed;
        else if (row.terminalType != MotionFeedbackType::NONE) ++output.failed;
        else ++output.pending;
    }
}
