#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

// =============================================================
// NC-0.2L.2A / Path Core Accepted Input Contract Shadow
//
// An accepted K.7 read-ahead handoff is reduced to scalar identity fields
// only.  The fixed two-record ring is history for the same NC thread; it is
// not a queue, ownership claim, Runtime permit or Motion command source.
// HMI exposure remains limited to the one-byte compact state and four-byte
// change token below.  There are no counters, large snapshots, allocations
// or cross-thread references.
// =============================================================

constexpr std::size_t NC_PATH_CORE_INPUT_CONTRACT_CAPACITY = 2U;

// This is the latest publication kind, latched for HMI observation. It does
// not describe live readiness and must never be interpreted as a permit.
enum class NCPathCoreInputHandoffCompactState : std::uint8_t
{
    NOT_RUNNING = 0U,
    READ_AHEAD_INPUT_OBSERVED = 1U
};

struct NCPathCoreAcceptedInputRecord
{
    std::uint64_t preparedSession = 0ULL;
    std::uint64_t preparedEntrySequence = 0ULL;
    std::uint64_t dispatchId = 0ULL;
    std::uint64_t commitSequence = 0ULL;
    std::uint64_t motionExecutionEpoch = 0ULL;
    std::uint64_t motionSegmentId = 0ULL;
    std::int32_t sourcePC = -1;
    std::int32_t sourceLineNumber = 0;

    bool IsPopulated() const noexcept
    {
        return dispatchId != 0ULL;
    }
};

class NCPathCoreInputContractShadow final
{
public:
    NCPathCoreInputContractShadow() noexcept = default;

    void ObserveAcceptedReadAhead(
        std::uint64_t preparedSession,
        std::uint64_t preparedEntrySequence,
        std::uint64_t dispatchId,
        std::uint64_t commitSequence,
        std::uint64_t motionExecutionEpoch,
        std::uint64_t motionSegmentId,
        std::int32_t sourcePC,
        std::int32_t sourceLineNumber) noexcept
    {
        const std::size_t targetIndex =
            m_latestIndex == INVALID_INDEX
            ? 0U
            : (static_cast<std::size_t>(m_latestIndex) + 1U) %
            NC_PATH_CORE_INPUT_CONTRACT_CAPACITY;

        // Assign directly into the fixed member workspace.  Do not create a
        // second record on the RT thread stack.
        NCPathCoreAcceptedInputRecord& target = m_records[targetIndex];
        target.preparedSession = preparedSession;
        target.preparedEntrySequence = preparedEntrySequence;
        target.dispatchId = dispatchId;
        target.commitSequence = commitSequence;
        target.motionExecutionEpoch = motionExecutionEpoch;
        target.motionSegmentId = motionSegmentId;
        target.sourcePC = sourcePC;
        target.sourceLineNumber = sourceLineNumber;

        m_latestIndex = static_cast<std::uint8_t>(targetIndex);
        if (m_recordCount < NC_PATH_CORE_INPUT_CONTRACT_CAPACITY)
        {
            ++m_recordCount;
        }
    }

    const NCPathCoreAcceptedInputRecord* GetNewestSameThread(
        std::size_t historyOffset = 0U) const noexcept
    {
        if (m_latestIndex == INVALID_INDEX || historyOffset >= m_recordCount)
        {
            return nullptr;
        }

        const std::size_t index =
            (static_cast<std::size_t>(m_latestIndex) +
                NC_PATH_CORE_INPUT_CONTRACT_CAPACITY - historyOffset) %
            NC_PATH_CORE_INPUT_CONTRACT_CAPACITY;
        return &m_records[index];
    }

    std::size_t GetRecordCountSameThread() const noexcept
    {
        return m_recordCount;
    }

private:
    static constexpr std::uint8_t INVALID_INDEX = 0xFFU;

    std::array<
        NCPathCoreAcceptedInputRecord,
        NC_PATH_CORE_INPUT_CONTRACT_CAPACITY> m_records{};
    std::uint8_t m_latestIndex = INVALID_INDEX;
    std::uint8_t m_recordCount = 0U;
};

class NCPathCoreInputHandoffCompactShadow final
{
public:
    NCPathCoreInputHandoffCompactShadow() noexcept = default;

    void EnsureNotRunningPublished() noexcept
    {
        if (m_changeToken == 0U)
        {
            PublishState(NCPathCoreInputHandoffCompactState::NOT_RUNNING);
        }
    }

    void ObserveNotRunning() noexcept
    {
        PublishState(NCPathCoreInputHandoffCompactState::NOT_RUNNING);
    }

    void ObserveReadAheadInputEvent() noexcept
    {
        PublishEvent(
            NCPathCoreInputHandoffCompactState::
            READ_AHEAD_INPUT_OBSERVED);
    }

    NCPathCoreInputHandoffCompactState GetCompactState() const noexcept
    {
        return m_state;
    }

    std::uint32_t GetChangeToken() const noexcept
    {
        return m_changeToken;
    }

private:
    void AdvanceChangeToken() noexcept
    {
        ++m_changeToken;
        if (m_changeToken == 0U)
        {
            // Zero remains the process-start "not published yet" value.
            m_changeToken = 1U;
        }
    }

    void PublishState(NCPathCoreInputHandoffCompactState observedState) noexcept
    {
        const bool stateChanged = m_state != observedState;

        // Retain the already accepted B2 one-byte Runtime store.  Only the
        // small change token is conditional.
        m_state = observedState;

        if (m_changeToken == 0U || stateChanged)
        {
            AdvanceChangeToken();
        }
    }

    void PublishEvent(
        NCPathCoreInputHandoffCompactState observedEvent) noexcept
    {
        // Each accepted handoff is a distinct event even when its compact
        // event kind matches the preceding one.
        m_state = observedEvent;
        AdvanceChangeToken();
    }

    NCPathCoreInputHandoffCompactState m_state =
        NCPathCoreInputHandoffCompactState::NOT_RUNNING;
    std::uint32_t m_changeToken = 0U;
};

static_assert(
    sizeof(NCPathCoreInputHandoffCompactState) == 1U,
    "Path Core compact state must remain one byte.");
static_assert(
    std::is_standard_layout<NCPathCoreAcceptedInputRecord>::value,
    "Path Core accepted input record must remain standard-layout.");
static_assert(
    std::is_trivially_copyable<NCPathCoreAcceptedInputRecord>::value,
    "Path Core accepted input record must remain trivially copyable.");
static_assert(
    sizeof(NCPathCoreAcceptedInputRecord) <= 56U,
    "Path Core accepted input record exceeded its fixed size budget.");
static_assert(
    std::is_standard_layout<NCPathCoreInputContractShadow>::value,
    "Path Core input contract shadow must remain standard-layout.");
static_assert(
    std::is_trivially_copyable<NCPathCoreInputContractShadow>::value,
    "Path Core input contract shadow must remain trivially copyable.");
static_assert(
    sizeof(NCPathCoreInputContractShadow) <= 120U,
    "Path Core input contract shadow exceeded its fixed size budget.");
static_assert(
    sizeof(NCPathCoreInputHandoffCompactShadow) <= 8U,
    "Path Core compact state and token must remain eight bytes or less.");
static_assert(
    std::is_trivially_copyable<
    NCPathCoreInputHandoffCompactShadow>::value,
    "Path Core compact observer foundation must remain trivially copyable.");

