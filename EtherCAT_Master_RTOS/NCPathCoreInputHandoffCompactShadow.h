#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

// =============================================================
// NC-0.2L.2C / Path Core Accepted-Handoff Run-Length Shadow
//
// An accepted K.7 read-ahead handoff is reduced to scalar identity fields
// only.  The fixed two-record ring is history for the same NC thread; it is
// not a queue, ownership claim, Runtime permit or Motion command source.
// L.2B classifies only the relationship between the newest accepted input and
// its immediate predecessor.  A chain boundary is neutral history, never a
// fault, rejection or fallback request.
// L.2C reduces those pair relations to the number of consecutive accepted
// K.7 handoff observations since the latest boundary.  This is not path
// distance, Geometry segment count, Motion queue depth, execution progress or
// a B2 breadcrumb count, and it must never be interpreted as a permit.
// HMI exposure remains limited to the one-byte compact state and four-byte
// change token below.  There are no counters, large snapshots, allocations
// or cross-thread references.
// =============================================================

constexpr std::size_t NC_PATH_CORE_INPUT_CONTRACT_CAPACITY = 2U;

namespace NCPathCoreDetail
{
    constexpr std::uint32_t ACCEPTED_HANDOFF_RUN_LENGTH_MAX = 0xFFFFFFFFU;

    constexpr std::uint32_t SaturatingIncrementAcceptedHandoffRunLength(
        std::uint32_t current) noexcept
    {
        return current == ACCEPTED_HANDOFF_RUN_LENGTH_MAX
            ? ACCEPTED_HANDOFF_RUN_LENGTH_MAX
            : current + 1U;
    }
}

// This is the latest publication kind, latched for HMI observation. It does
// not describe live readiness and must never be interpreted as a permit.
enum class NCPathCoreInputHandoffCompactState : std::uint8_t
{
    NOT_RUNNING = 0U,
    READ_AHEAD_INPUT_OBSERVED = 1U
};

enum class NCPathCoreAcceptedInputPairRelation : std::uint8_t
{
    NONE = 0U,
    FIRST_INPUT = 1U,
    CONTIGUOUS_PAIR = 2U,
    CHAIN_BOUNDARY = 3U
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
        const NCPathCoreAcceptedInputRecord* const previous =
            GetNewestSameThread();
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

        m_pairRelation =
            previous == nullptr
            ? NCPathCoreAcceptedInputPairRelation::FIRST_INPUT
            : IsContiguousPairSameThread(*previous, target)
            ? NCPathCoreAcceptedInputPairRelation::CONTIGUOUS_PAIR
            : NCPathCoreAcceptedInputPairRelation::CHAIN_BOUNDARY;

        if (m_pairRelation ==
            NCPathCoreAcceptedInputPairRelation::CONTIGUOUS_PAIR)
        {
            m_latestAcceptedHandoffRunLength =
                NCPathCoreDetail::SaturatingIncrementAcceptedHandoffRunLength(
                    m_latestAcceptedHandoffRunLength);
        }
        else if (
            m_pairRelation ==
            NCPathCoreAcceptedInputPairRelation::FIRST_INPUT ||
            m_pairRelation ==
            NCPathCoreAcceptedInputPairRelation::CHAIN_BOUNDARY)
        {
            m_latestAcceptedHandoffRunLength = 1U;
        }
        else
        {
            m_latestAcceptedHandoffRunLength = 0U;
        }

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

    NCPathCoreAcceptedInputPairRelation
        GetPairRelationSameThread() const noexcept
    {
        return m_pairRelation;
    }

    std::uint32_t GetLatestAcceptedHandoffRunLengthSameThread() const noexcept
    {
        return m_latestAcceptedHandoffRunLength;
    }

private:
    static constexpr std::uint8_t INVALID_INDEX = 0xFFU;

    static bool IsStrictlyIncreasingNonZero(
        std::uint64_t previous,
        std::uint64_t current) noexcept
    {
        return previous != 0ULL &&
            current != 0ULL &&
            current > previous;
    }

    static bool IsMatchingNonZero(
        std::uint64_t previous,
        std::uint64_t current) noexcept
    {
        return previous != 0ULL &&
            current != 0ULL &&
            current == previous;
    }

    static bool IsContiguousPairSameThread(
        const NCPathCoreAcceptedInputRecord& previous,
        const NCPathCoreAcceptedInputRecord& current) noexcept
    {
        if (!previous.IsPopulated() || !current.IsPopulated())
        {
            return false;
        }

        // sourceLineNumber is diagnostic provenance and is intentionally not
        // required to be dense. Convert sourcePC to int64_t before adding one
        // so the comparison remains defined at the int32_t boundary.
        const bool sourcePcContiguous =
            previous.sourcePC >= 0 &&
            current.sourcePC >= 0 &&
            static_cast<std::int64_t>(current.sourcePC) ==
            static_cast<std::int64_t>(previous.sourcePC) + 1LL;

        // Zero is the upstream invalid sentinel for both the prepared queue
        // session and Motion execution epoch.  Equal zero anchors therefore
        // cannot establish a real accepted-input chain.
        return IsMatchingNonZero(
            previous.preparedSession,
            current.preparedSession) &&
            IsMatchingNonZero(
                previous.motionExecutionEpoch,
                current.motionExecutionEpoch) &&
            sourcePcContiguous &&
            IsStrictlyIncreasingNonZero(
                previous.preparedEntrySequence,
                current.preparedEntrySequence) &&
            IsStrictlyIncreasingNonZero(
                previous.dispatchId,
                current.dispatchId) &&
            IsStrictlyIncreasingNonZero(
                previous.commitSequence,
                current.commitSequence) &&
            IsStrictlyIncreasingNonZero(
                previous.motionSegmentId,
                current.motionSegmentId);
    }

    std::array<
        NCPathCoreAcceptedInputRecord,
        NC_PATH_CORE_INPUT_CONTRACT_CAPACITY> m_records{};
    std::uint8_t m_latestIndex = INVALID_INDEX;
    std::uint8_t m_recordCount = 0U;
    NCPathCoreAcceptedInputPairRelation m_pairRelation =
        NCPathCoreAcceptedInputPairRelation::NONE;
    std::uint32_t m_latestAcceptedHandoffRunLength = 0U;
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
    sizeof(NCPathCoreAcceptedInputPairRelation) == 1U,
    "Path Core pair relation must remain one byte.");
static_assert(
    NCPathCoreDetail::SaturatingIncrementAcceptedHandoffRunLength(0U) == 1U,
    "Path Core run length must advance from zero to one.");
static_assert(
    NCPathCoreDetail::SaturatingIncrementAcceptedHandoffRunLength(
        NCPathCoreDetail::ACCEPTED_HANDOFF_RUN_LENGTH_MAX) ==
    NCPathCoreDetail::ACCEPTED_HANDOFF_RUN_LENGTH_MAX,
    "Path Core run length must saturate instead of wrapping.");
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
    sizeof(NCPathCoreInputContractShadow) == 120U,
    "Path Core input contract shadow must remain exactly 120 bytes.");
static_assert(
    sizeof(NCPathCoreInputHandoffCompactShadow) <= 8U,
    "Path Core compact state and token must remain eight bytes or less.");
static_assert(
    std::is_trivially_copyable<
    NCPathCoreInputHandoffCompactShadow>::value,
    "Path Core compact observer foundation must remain trivially copyable.");

