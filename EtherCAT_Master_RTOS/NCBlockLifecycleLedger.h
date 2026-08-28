#pragma once

#include "NCProgramCache.h"
#include "MotionExecutionContract.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

// =============================================================================
// Stage NC-0.2D - Program Block Lifecycle Ledger
//
// 把同一個 NC Program Block 的兩條生命週期關聯起來：
//
//   Program：Pure Parse -> Runtime Resolve -> Semantic Commit
//   Motion ：Producer Submit -> ACCEPTED -> STARTED -> Terminal Feedback
//
// 本階段只建立觀察與診斷邊界，不改變既有 PC 推進、Wait Callback、
// Single Block、速度規劃或馬達運動結果。
//
// 一個 Block 可能產生多筆 Motion Segment（例如 G28 / G30），因此 Ledger
// 原生支援一對多關聯，不會只保存「最後一筆 Segment」。所有容器皆為固定
// 容量，NC Runtime 不會因 Ledger 產生動態記憶體配置。
// =============================================================================

using NCBlockDispatchId = std::uint64_t;

constexpr NCBlockDispatchId NC_BLOCK_DISPATCH_ID_INVALID = 0ULL;
constexpr std::size_t NC_BLOCK_LIFECYCLE_CAPACITY = 256U;
constexpr std::size_t NC_BLOCK_MAX_MOTION_SEGMENTS = 32U;
constexpr std::size_t NC_BLOCK_SEGMENT_INDEX_CAPACITY =
NC_BLOCK_LIFECYCLE_CAPACITY * NC_BLOCK_MAX_MOTION_SEGMENTS;

enum class NCBlockLifecycleState : std::uint8_t
{
    NONE = 0,
    DISPATCHED = 1,
    PROGRAM_COMMITTED = 2,
    PROGRAM_ONLY_COMPLETED = 3,
    MOTION_PENDING = 4,
    MOTION_ACCEPTED = 5,
    MOTION_ACTIVE = 6,
    MOTION_HELD = 7,
    MOTION_COMPLETED = 8,
    MOTION_REJECTED = 9,
    MOTION_CANCELLED = 10,
    MOTION_ABORTED = 11,
    MOTION_FAULTED = 12,
    NC_DISPATCH_FAILED = 13,
    TRACKING_OVERFLOW = 14
};


enum class NCBlockMotionBoundaryState : std::uint8_t
{
    NONE = 0,
    NOT_TRACKED = 1,
    PENDING = 2,
    SUCCEEDED = 3,
    FAILED = 4,
    TRACKING_OVERFLOW = 5
};

struct NCBlockMotionBoundarySnapshot
{
    NCBlockDispatchId dispatchId = NC_BLOCK_DISPATCH_ID_INVALID;
    NCBlockMotionBoundaryState state = NCBlockMotionBoundaryState::NONE;
    NCBlockLifecycleState lifecycleState = NCBlockLifecycleState::NONE;

    std::uint16_t motionSegmentCount = 0U;
    std::uint16_t motionCompletedCount = 0U;
    std::uint16_t motionTerminalCount = 0U;
    std::uint16_t motionFailedCount = 0U;

    bool programCommitted = false;
    bool lifecycleTerminal = false;
    bool lifecycleSuccess = false;

    bool IsValid() const noexcept
    {
        return
            dispatchId != NC_BLOCK_DISPATCH_ID_INVALID &&
            state != NCBlockMotionBoundaryState::NONE;
    }
};

static_assert(
    std::is_trivially_copyable<NCBlockMotionBoundarySnapshot>::value,
    "NCBlockMotionBoundarySnapshot must remain trivially copyable.");

struct NCBlockMotionSegmentSnapshot
{
    MotionExecutionIdentity identity{};
    MotionRejectReason immediateRejectReason = MotionRejectReason::NONE;
    MotionRejectReason lastRejectReason = MotionRejectReason::NONE;
    MotionFeedbackSequence lastFeedbackSequence =
        MOTION_FEEDBACK_SEQUENCE_INVALID;
    MotionFeedbackType lastFeedbackType = MotionFeedbackType::NONE;
    MotionOwner owner = MotionOwner::NONE;
    MotionOwnerGeneration ownerGeneration =
        MOTION_OWNER_GENERATION_INVALID;
    std::uint32_t errorCode = 0U;
    double progress = 0.0;
    std::uint32_t feedbackEventCount = 0U;

    bool producerAccepted = false;
    bool runtimeAccepted = false;
    bool started = false;
    bool held = false;
    bool terminal = false;
    bool completed = false;
    bool rejected = false;
    bool cancelled = false;
    bool aborted = false;
    bool faulted = false;

    bool IsBound() const noexcept
    {
        return identity.IsAssigned();
    }
};

struct NCBlockLifecycleSnapshot
{
    NCBlockDispatchId dispatchId = NC_BLOCK_DISPATCH_ID_INVALID;
    NCProgramCommitSnapshot programTarget{};
    NCProgramCommitSnapshot programCommit{};
    int sourceLineNumber = 0;

    std::array<NCBlockMotionSegmentSnapshot,
        NC_BLOCK_MAX_MOTION_SEGMENTS> motionSegments{};
    std::uint16_t motionSegmentCount = 0U;
    std::uint16_t motionAcceptedCount = 0U;
    std::uint16_t motionStartedCount = 0U;
    std::uint16_t motionCompletedCount = 0U;
    std::uint16_t motionTerminalCount = 0U;
    std::uint16_t motionFailedCount = 0U;

    NCBlockLifecycleState state = NCBlockLifecycleState::NONE;
    std::uint32_t ncErrorCode = 0U;

    bool programCommitted = false;
    bool ncDispatchFailed = false;
    bool motionCaptureOverflow = false;
    bool terminal = false;
    bool terminalSuccess = false;
    bool terminalFailure = false;

    bool IsValid() const noexcept
    {
        return
            dispatchId != NC_BLOCK_DISPATCH_ID_INVALID &&
            state != NCBlockLifecycleState::NONE;
    }

    bool HasMotion() const noexcept
    {
        return motionSegmentCount != 0U;
    }
};

struct NCBlockLifecycleCounters
{
    std::uint64_t dispatched = 0ULL;
    std::uint64_t programCommitted = 0ULL;
    std::uint64_t programOnlyCompleted = 0ULL;
    std::uint64_t motionBlocks = 0ULL;
    std::uint64_t motionSegmentsBound = 0ULL;
    std::uint64_t producerAccepted = 0ULL;
    std::uint64_t producerRejected = 0ULL;

    std::uint64_t feedbackAccepted = 0ULL;
    std::uint64_t feedbackStarted = 0ULL;
    std::uint64_t feedbackCompleted = 0ULL;
    std::uint64_t feedbackRejected = 0ULL;
    std::uint64_t feedbackCancelled = 0ULL;
    std::uint64_t feedbackAborted = 0ULL;
    std::uint64_t feedbackFaulted = 0ULL;

    std::uint64_t blockCompleted = 0ULL;
    std::uint64_t blockFailed = 0ULL;
    std::uint64_t ncDispatchFailed = 0ULL;
    std::uint64_t motionCaptureOverflow = 0ULL;
    std::uint64_t orphanFeedback = 0ULL;
    std::uint64_t duplicateTerminalFeedback = 0ULL;
    std::uint64_t terminalFeedbackConflict = 0ULL;
    std::uint64_t activeBlockOverwrite = 0ULL;
    std::uint64_t activeSegmentIndexOverwrite = 0ULL;
    std::uint32_t activeBlocks = 0U;
};

static_assert(
    std::is_trivially_copyable<NCBlockMotionSegmentSnapshot>::value,
    "NCBlockMotionSegmentSnapshot must remain trivially copyable.");

static_assert(
    std::is_trivially_copyable<NCBlockLifecycleSnapshot>::value,
    "NCBlockLifecycleSnapshot must remain trivially copyable.");

static_assert(
    std::is_trivially_copyable<NCBlockLifecycleCounters>::value,
    "NCBlockLifecycleCounters must remain trivially copyable.");

class NCBlockLifecycleLedger
{
public:
    NCBlockLifecycleLedger() noexcept = default;

    NCBlockDispatchId BeginBlock(
        const NCProgramCommitSnapshot& programTarget,
        int sourceLineNumber) noexcept;

    bool MarkProgramCommitted(
        NCBlockDispatchId dispatchId,
        const NCProgramCommitSnapshot& programCommit) noexcept;

    bool BindMotionSegment(
        NCBlockDispatchId dispatchId,
        const MotionExecutionIdentity& identity,
        bool producerAccepted,
        MotionRejectReason immediateRejectReason) noexcept;

    bool MarkMotionCaptureOverflow(
        NCBlockDispatchId dispatchId) noexcept;

    bool MarkNCDispatchFailed(
        NCBlockDispatchId dispatchId,
        std::uint32_t errorCode) noexcept;

    bool ApplyMotionFeedback(
        const MotionFeedbackEvent& event) noexcept;

    bool GetLastDispatchedSnapshot(
        NCBlockLifecycleSnapshot& snapshot) const noexcept;

    bool GetLastProgramCommittedSnapshot(
        NCBlockLifecycleSnapshot& snapshot) const noexcept;

    bool GetLastMotionCompletedSnapshot(
        NCBlockLifecycleSnapshot& snapshot) const noexcept;

    bool GetLastTerminalSnapshot(
        NCBlockLifecycleSnapshot& snapshot) const noexcept;

    bool TryGetSnapshot(
        NCBlockDispatchId dispatchId,
        NCBlockLifecycleSnapshot& snapshot) const noexcept;

    bool GetMotionBoundarySnapshot(
        NCBlockDispatchId dispatchId,
        NCBlockMotionBoundarySnapshot& snapshot) const noexcept;

    NCBlockLifecycleCounters GetCounters() const noexcept
    {
        return m_counters;
    }

private:
    struct SegmentIndexEntry
    {
        MotionExecutionIdentity identity{};
        NCBlockDispatchId dispatchId = NC_BLOCK_DISPATCH_ID_INVALID;
        std::uint16_t segmentIndex = 0U;
        bool valid = false;
    };

    static bool IsProgramTargetValid(
        const NCProgramCommitSnapshot& target) noexcept;

    static std::size_t BlockSlotIndex(
        NCBlockDispatchId dispatchId) noexcept;

    static std::size_t SegmentSlotIndex(
        MotionSegmentId segmentId) noexcept;

    NCBlockDispatchId AllocateDispatchId() noexcept;

    NCBlockLifecycleSnapshot* FindMutable(
        NCBlockDispatchId dispatchId) noexcept;

    const NCBlockLifecycleSnapshot* Find(
        NCBlockDispatchId dispatchId) const noexcept;

    void ClearSegmentBindings(
        const NCBlockLifecycleSnapshot& snapshot) noexcept;

    void IndexMotionSegment(
        NCBlockDispatchId dispatchId,
        std::uint16_t segmentIndex,
        const MotionExecutionIdentity& identity) noexcept;

    NCBlockMotionSegmentSnapshot* FindMotionSegment(
        const MotionExecutionIdentity& identity,
        NCBlockLifecycleSnapshot*& block) noexcept;

    void RefreshAggregate(
        NCBlockLifecycleSnapshot& snapshot) noexcept;

    void Finalize(
        NCBlockLifecycleSnapshot& snapshot,
        NCBlockLifecycleState terminalState,
        bool success) noexcept;

    void CopySnapshot(
        NCBlockDispatchId dispatchId,
        NCBlockLifecycleSnapshot& snapshot,
        bool& result) const noexcept;

    std::array<NCBlockLifecycleSnapshot,
        NC_BLOCK_LIFECYCLE_CAPACITY> m_blocks{};
    std::array<SegmentIndexEntry,
        NC_BLOCK_SEGMENT_INDEX_CAPACITY> m_segmentIndex{};

    NCBlockDispatchId m_nextDispatchId = 1ULL;
    NCBlockDispatchId m_lastDispatchedId = NC_BLOCK_DISPATCH_ID_INVALID;
    NCBlockDispatchId m_lastProgramCommittedId = NC_BLOCK_DISPATCH_ID_INVALID;
    NCBlockDispatchId m_lastMotionCompletedId = NC_BLOCK_DISPATCH_ID_INVALID;
    NCBlockDispatchId m_lastTerminalId = NC_BLOCK_DISPATCH_ID_INVALID;

    NCBlockLifecycleCounters m_counters{};
};
