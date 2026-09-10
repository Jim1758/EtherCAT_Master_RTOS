#include "NCPathCoreLiveRetention.h"
#include <limits>

namespace
{
    using State = NCPathCoreLiveRetentionState;
    using Reason = NCPathCoreLiveRetentionReason;
    using Code = NCPathCoreCommandedChordStoreCode;
    using Pair = NCPathCoreAcceptedInputPairRelation;

    void SaturatingIncrement(std::uint32_t& value) noexcept
    {
        if (value != (std::numeric_limits<std::uint32_t>::max)()) ++value;
    }
}

void NCPathCoreLiveRetentionScopeV1::Clear() noexcept
{
    runToken = 0ULL;
    cacheGeneration = 0ULL;
    programScope = 0U;
    operationMode = 0U;
    ownerLease.owner = MotionOwner::NONE;
    ownerLease.generation = MOTION_OWNER_GENERATION_INVALID;
}

bool NCPathCoreLiveRetentionScopeV1::IsValid() const noexcept
{
    if (runToken == 0ULL || cacheGeneration == 0ULL || !ownerLease.IsValid()) return false;
    switch (programScope)
    {
    case 1U: return operationMode == 0U && ownerLease.owner == MotionOwner::AUTO;
    case 2U: return operationMode == 1U && ownerLease.owner == MotionOwner::MDI;
    case 3U: return operationMode == 2U && ownerLease.owner == MotionOwner::MANUAL_AUTO;
    default: return false;
    }
}

bool NCPathCoreLiveRetentionScopeV1::Matches(
    const NCPathCoreLiveRetentionScopeV1& other) const noexcept
{
    return runToken == other.runToken && cacheGeneration == other.cacheGeneration &&
        programScope == other.programScope && operationMode == other.operationMode &&
        ownerLease.owner == other.ownerLease.owner &&
        ownerLease.generation == other.ownerLease.generation;
}

void NCPathCoreLiveRetentionStatusV1::Clear() noexcept
{
    ownerTag = 0ULL;
    runToken = 0ULL;
    lifetime = 0ULL;
    boundEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    storedCount = 0U;
    readableCount = 0U;
    admittedCount = 0U;
    replayCount = 0U;
    state = State::CLOSED;
    reason = Reason::NONE;
    lastStoreCode = Code::NONE;
    heldPrior = State::CLOSED;
}

std::uint64_t NCPathCoreLiveRetentionOwnerCounter::Allocate() noexcept
{
    if (m_lastIssued == (std::numeric_limits<std::uint64_t>::max)()) return 0ULL;
    ++m_lastIssued;
    return m_lastIssued;
}

NCPathCoreLiveRetention::NCPathCoreLiveRetention(
    const std::uint64_t ownerTag, const std::uint64_t initialLifetime) noexcept
    : m_store(ownerTag), m_lifetime(initialLifetime)
{
    if (ownerTag == 0ULL)
    {
        m_state = State::EXHAUSTED;
        m_reason = Reason::OWNER_EXHAUSTED;
        m_lastStoreCode = Code::INVALID_OWNER;
    }
}

bool NCPathCoreLiveRetention::HasTerminalReason() const noexcept
{
    return m_state == State::FAULTED || m_state == State::EXHAUSTED ||
        (m_state == State::CLOSED && m_reason != Reason::NONE);
}

void NCPathCoreLiveRetention::Arm(const NCPathCoreLiveRetentionScopeV1& scope) noexcept
{
    if (m_state == State::EXHAUSTED) return;
    if (scope.runToken != 0ULL && scope.runToken < m_scope.runToken) return;
    if (scope.runToken != 0ULL && scope.runToken == m_scope.runToken)
    {
        if (!scope.Matches(m_scope)) Reject(Reason::SCOPE_CHANGED);
        return;
    }
    if (!scope.IsValid())
    {
        Reject(Reason::INVALID_SCOPE);
        return;
    }
    m_store.Invalidate();
    m_scope = scope;
    m_boundEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    m_admittedCount = 0U;
    m_replayCount = 0U;
    m_state = State::ARMED;
    m_heldPrior = State::CLOSED;
    m_reason = Reason::START;
    m_lastStoreCode = Code::NONE;
}

bool NCPathCoreLiveRetention::CheckScope(
    const NCPathCoreLiveRetentionScopeV1& scope,
    const MotionExecutionEpoch currentEpoch) noexcept
{
    if (m_state != State::ARMED && m_state != State::OPEN && m_state != State::HELD) return false;
    if (!scope.IsValid())
    {
        Fence(Reason::INVALID_SCOPE);
        return false;
    }
    if (!scope.Matches(m_scope))
    {
        Fence(Reason::SCOPE_CHANGED);
        return false;
    }
    if (m_boundEpoch != MOTION_EXECUTION_EPOCH_INVALID && currentEpoch != m_boundEpoch)
    {
        Fence(Reason::EPOCH_CHANGED);
        return false;
    }
    return true;
}

Code NCPathCoreLiveRetention::Admit(
    const NCPathCoreCommandedChordSegmentV1& value,
    const NCPathCoreLiveRetentionScopeV1& scope,
    const MotionExecutionEpoch currentEpoch,
    NCPathCoreCommandedChordStoreHandleV1& output) noexcept
{
    output.Clear();
    if (!CheckScope(scope, currentEpoch)) return Code::NOT_OPEN;
    if (m_state == State::HELD)
    {
        Reject(Reason::ADMISSION_DURING_HOLD);
        return Code::NOT_OPEN;
    }
    const bool isHead = m_state == State::ARMED;
    if (isHead)
    {
        if (!IsValidCommandedChordSegmentValue(value))
        {
            Reject(Reason::ADMISSION_FAILED, Code::INVALID_VALUE);
            return Code::INVALID_VALUE;
        }
        if (value.acceptedInputRunLength != 1U ||
            (value.sourcePairRelation != Pair::FIRST_INPUT && value.sourcePairRelation != Pair::CHAIN_BOUNDARY))
        {
            Reject(Reason::ADMISSION_FAILED, Code::MISSING_HEAD);
            return Code::MISSING_HEAD;
        }
        if (currentEpoch == MOTION_EXECUTION_EPOCH_INVALID)
        {
            Reject(Reason::EPOCH_CHANGED);
            return Code::NOT_OPEN;
        }
        if (m_lifetime == (std::numeric_limits<std::uint64_t>::max)())
        {
            DisableExhausted(Reason::LIFETIME_EXHAUSTED);
            m_lastStoreCode = Code::LIFETIME_EXHAUSTED;
            return Code::LIFETIME_EXHAUSTED;
        }
        ++m_lifetime;
        const Code opened = m_store.BeginLifetime(m_lifetime);
        if (opened != Code::OPENED)
        {
            Reject(Reason::ADMISSION_FAILED, opened);
            return opened;
        }
    }
    const Code code = m_store.Append(value, output);
    m_lastStoreCode = code;
    if (code != Code::APPENDED && code != Code::ALREADY_RETAINED)
    {
        // AZ's first admission fault retains its bounded physical payload for
        // diagnostics, while every live read is refused by both layers.
        m_state = State::FAULTED;
        m_heldPrior = State::CLOSED;
        m_reason = Reason::ADMISSION_FAILED;
        return code;
    }
    if (isHead) m_boundEpoch = currentEpoch;
    m_state = State::OPEN;
    if (code == Code::APPENDED) SaturatingIncrement(m_admittedCount);
    else SaturatingIncrement(m_replayCount);
    return code;
}

void NCPathCoreLiveRetention::Hold(
    const NCPathCoreLiveRetentionScopeV1& scope,
    const MotionExecutionEpoch currentEpoch) noexcept
{
    if (!CheckScope(scope, currentEpoch)) return;
    if (m_state == State::ARMED || m_state == State::OPEN)
    {
        m_heldPrior = m_state;
        m_state = State::HELD;
        m_reason = Reason::HOLD;
    }
}

void NCPathCoreLiveRetention::Resume(
    const NCPathCoreLiveRetentionScopeV1& scope,
    const MotionExecutionEpoch currentEpoch, const bool successfulGate) noexcept
{
    if (!CheckScope(scope, currentEpoch)) return;
    if (!successfulGate || m_state != State::HELD) return;
    m_state = m_heldPrior;
    m_heldPrior = State::CLOSED;
    m_reason = Reason::RESUMED;
}

void NCPathCoreLiveRetention::Fence(const Reason reason) noexcept
{
    if (!HasTerminalReason()) m_reason = reason;
    m_store.Invalidate();
    m_boundEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    m_heldPrior = State::CLOSED;
    if (m_state != State::EXHAUSTED) m_state = State::CLOSED;
}

void NCPathCoreLiveRetention::Reject(const Reason reason, const Code code) noexcept
{
    if (m_state == State::FAULTED || m_state == State::EXHAUSTED) return;
    if (!HasTerminalReason())
    {
        m_reason = reason;
        m_lastStoreCode = code;
    }
    m_store.Invalidate();
    m_boundEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    m_heldPrior = State::CLOSED;
    m_state = State::FAULTED;
}

void NCPathCoreLiveRetention::DisableExhausted(const Reason reason) noexcept
{
    if (m_state == State::EXHAUSTED) return;
    m_store.Invalidate();
    m_boundEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    m_heldPrior = State::CLOSED;
    m_state = State::EXHAUSTED;
    m_reason = reason;
}

Code NCPathCoreLiveRetention::GetHandleAtOrdinal(
    const NCPathCoreLiveRetentionScopeV1& scope,
    const MotionExecutionEpoch currentEpoch, const std::uint32_t ordinal,
    NCPathCoreCommandedChordStoreHandleV1& output) noexcept
{
    output.Clear();
    if (!CheckScope(scope, currentEpoch) || m_state != State::OPEN) return Code::NOT_OPEN;
    return m_store.GetHandleAtOrdinal(ordinal, output);
}

Code NCPathCoreLiveRetention::Read(
    const NCPathCoreLiveRetentionScopeV1& scope,
    const MotionExecutionEpoch currentEpoch,
    const NCPathCoreCommandedChordStoreHandleV1& handle,
    NCPathCoreCommandedChordSegmentV1& output) noexcept
{
    output.Clear();
    if (!CheckScope(scope, currentEpoch) || m_state != State::OPEN) return Code::NOT_OPEN;
    return m_store.Read(handle, output);
}

NCPathCoreCommandedChordSnapshotCode NCPathCoreLiveRetention::CaptureSnapshot(
    const NCPathCoreLiveRetentionScopeV1& scope,
    const MotionExecutionEpoch currentEpoch,
    const NCPathCoreCommandedChordSubpathV1& subpath,
    NCPathCoreCommandedChordSegmentV1& workspace,
    NCPathCoreCommandedChordSnapshotV1& output) noexcept
{
    using SnapshotCode = NCPathCoreCommandedChordSnapshotCode;
    output.Clear();
    workspace.Clear();
    if (!CheckScope(scope, currentEpoch) || m_state != State::OPEN)
        return m_state == State::FAULTED ? SnapshotCode::STORE_FAULTED : SnapshotCode::STORE_CLOSED;
    return output.Capture(m_store, subpath, workspace);
}

void NCPathCoreLiveRetention::Describe(NCPathCoreLiveRetentionStatusV1& output) const noexcept
{
    NCPathCoreCommandedChordStoreInfoV1 info;
    m_store.Describe(info);
    output.Clear();
    output.ownerTag = info.ownerTag;
    output.runToken = m_scope.runToken;
    output.lifetime = m_lifetime;
    output.boundEpoch = m_boundEpoch;
    output.storedCount = info.storedCount;
    output.readableCount = m_state == State::OPEN ? info.readableCount : 0U;
    output.admittedCount = m_admittedCount;
    output.replayCount = m_replayCount;
    output.state = m_state;
    output.reason = m_reason;
    output.lastStoreCode = m_lastStoreCode;
    output.heldPrior = m_heldPrior;
}
