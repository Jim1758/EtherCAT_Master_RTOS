#pragma once

#include <cstdint>
#include <type_traits>

// ============================================================================
// Motion Execution Contract
//
// NC / Manual / Home / EDM 與 Motion Runtime 之間共用的：
//
//     Execution Epoch
//     Segment Identity
//     Motion Owner
//     Motion Feedback Event
//
// 所有跨執行緒 Payload 都必須保持 trivially-copyable，
// 才能安全放入固定容量 Lock-Free Ring。
// ============================================================================

using MotionExecutionEpoch = std::uint32_t;
using MotionSegmentId = std::uint64_t;
using MotionSourceBlockId = std::int32_t;
using MotionFeedbackSequence = std::uint64_t;
using MotionOwnerGeneration = std::uint32_t;

constexpr MotionExecutionEpoch MOTION_EXECUTION_EPOCH_INVALID = 0U;
constexpr MotionSegmentId MOTION_SEGMENT_ID_INVALID = 0ULL;
constexpr MotionSourceBlockId MOTION_SOURCE_BLOCK_ID_INVALID = -1;
constexpr MotionFeedbackSequence MOTION_FEEDBACK_SEQUENCE_INVALID = 0ULL;
constexpr MotionOwnerGeneration MOTION_OWNER_GENERATION_INVALID = 0U;

// 命令由哪一個上層子系統產生。
enum class MotionCommandSource : std::uint8_t
{
    UNKNOWN = 0,
    NC_MEMORY = 1,
    NC_MDI = 2,
    NC_MANUAL_AUTO = 3,
    JOG = 4,
    MPG = 5,
    HOME = 6,
    EDM_PATH = 7,
    EDM_RETRACT = 8,
    RECOVERY = 9,
    SAFETY = 10
};

// 哪一個子系統目前擁有 Motion 控制權。
enum class MotionOwner : std::uint8_t
{
    NONE = 0,
    AUTO = 1,
    MDI = 2,
    MANUAL_AUTO = 3,
    JOG = 4,
    MPG = 5,
    HOME = 6,
    EDM_PATH = 7,
    EDM_RETRACT = 8,
    RECOVERY = 9,
    SAFETY = 10,
    IDLE_HOLD = 11 // Output-only; no command source maps to this owner.
};

// Stage NC-0.1E：Motion 控制權租約。
//
// owner：目前控制 Motion 的子系統。
// generation：每次真正交棒都遞增。舊執行緒持有的 Lease 即使晚到，
//             只要 Generation 不同，就不能釋放或繼續使用新的控制權。
struct MotionOwnerLease
{
    MotionOwner owner = MotionOwner::NONE;
    MotionOwnerGeneration generation = MOTION_OWNER_GENERATION_INVALID;

    bool IsValid() const noexcept
    {
        return
            owner != MotionOwner::NONE &&
            generation != MOTION_OWNER_GENERATION_INVALID;
    }

    bool Matches(const MotionOwnerLease& other) const noexcept
    {
        return
            owner == other.owner &&
            generation == other.generation;
    }
};

inline MotionOwner ResolveMotionOwnerForSource(
    MotionCommandSource source) noexcept
{
    switch (source)
    {
    case MotionCommandSource::NC_MEMORY:      return MotionOwner::AUTO;
    case MotionCommandSource::NC_MDI:         return MotionOwner::MDI;
    case MotionCommandSource::NC_MANUAL_AUTO: return MotionOwner::MANUAL_AUTO;
    case MotionCommandSource::JOG:            return MotionOwner::JOG;
    case MotionCommandSource::MPG:            return MotionOwner::MPG;
    case MotionCommandSource::HOME:           return MotionOwner::HOME;
    case MotionCommandSource::EDM_PATH:       return MotionOwner::EDM_PATH;
    case MotionCommandSource::EDM_RETRACT:    return MotionOwner::EDM_RETRACT;
    case MotionCommandSource::RECOVERY:       return MotionOwner::RECOVERY;
    case MotionCommandSource::SAFETY:         return MotionOwner::SAFETY;
    case MotionCommandSource::UNKNOWN:
    default:                                  return MotionOwner::NONE;
    }
}

inline MotionCommandSource ResolveMotionCommandSourceForOwner(
    MotionOwner owner) noexcept
{
    switch (owner)
    {
    case MotionOwner::AUTO:         return MotionCommandSource::NC_MEMORY;
    case MotionOwner::MDI:          return MotionCommandSource::NC_MDI;
    case MotionOwner::MANUAL_AUTO:  return MotionCommandSource::NC_MANUAL_AUTO;
    case MotionOwner::JOG:          return MotionCommandSource::JOG;
    case MotionOwner::MPG:          return MotionCommandSource::MPG;
    case MotionOwner::HOME:         return MotionCommandSource::HOME;
    case MotionOwner::EDM_PATH:     return MotionCommandSource::EDM_PATH;
    case MotionOwner::EDM_RETRACT:  return MotionCommandSource::EDM_RETRACT;
    case MotionOwner::RECOVERY:     return MotionCommandSource::RECOVERY;
    case MotionOwner::SAFETY:       return MotionCommandSource::SAFETY;
    case MotionOwner::NONE:
    default:                        return MotionCommandSource::UNKNOWN;
    }
}

// Motion Runtime 回報給上層的事件種類。
//
// Stage NC-0.1D 正式產生：
//
//     ACCEPTED
//     REJECTED
//     STARTED
//     COMPLETED
//     ABORTED
//     FAULTED
//
// PROGRESS / HELD / RESUMED / CANCELLED 保留給後續階段。
enum class MotionFeedbackType : std::uint8_t
{
    NONE = 0,
    ACCEPTED = 1,
    REJECTED = 2,
    STARTED = 3,
    PROGRESS = 4,
    HELD = 5,
    RESUMED = 6,
    COMPLETED = 7,
    CANCELLED = 8,
    FAULTED = 9,
    ABORTED = 10
};

// 命令被 Motion Runtime 拒絕時的原因。
enum class MotionRejectReason : std::uint8_t
{
    NONE = 0,
    STALE_EPOCH = 1,
    QUEUE_FULL = 2,
    OWNER_CONFLICT = 3,
    NOT_READY = 4,
    INVALID_AXIS = 5,
    INVALID_GEOMETRY = 6,
    SAFETY_INTERLOCK = 7,

    // 固定容量 Command / Replay / Feedback Transport 無法保證交付。
    TRANSPORT_OVERFLOW = 8
};

// 每一筆 Motion 命令都必須能被唯一追蹤。
//
// epoch：
//     Reset / GOTO / 重新規劃時遞增；舊 epoch 命令不得再執行。
//
// segmentId：
//     全系統單調遞增的路徑段識別碼。
//
// sourceBlockId：
//     對應 NC 原始 Block；手動命令可維持 INVALID。
struct MotionExecutionIdentity
{
    MotionExecutionEpoch epoch = MOTION_EXECUTION_EPOCH_INVALID;
    MotionSegmentId segmentId = MOTION_SEGMENT_ID_INVALID;
    MotionSourceBlockId sourceBlockId = MOTION_SOURCE_BLOCK_ID_INVALID;
    MotionCommandSource source = MotionCommandSource::UNKNOWN;

    bool IsAssigned() const noexcept
    {
        return
            epoch != MOTION_EXECUTION_EPOCH_INVALID &&
            segmentId != MOTION_SEGMENT_ID_INVALID;
    }

    bool Matches(
        const MotionExecutionIdentity& other) const noexcept
    {
        return
            epoch == other.epoch &&
            segmentId == other.segmentId;
    }
};

// 固定容量 Feedback Ring 使用的事件格式。
//
// sequence：
//     由 250 us Motion Runtime 單一 Producer 配發；0 保留為 INVALID。
//     NC 可用它檢查事件順序與是否發生遺失。
//
// owner / ownerGeneration：
//     來自命令實際持有的 MotionOwnerLease。
//     可用來辨識同一個 Owner 的不同控制世代。
//
// progress：
//     0.0 ~ 1.0。COMPLETED 固定為 1.0；ABORTED / FAULTED 保存當下進度。
struct MotionFeedbackEvent
{
    MotionFeedbackSequence sequence = MOTION_FEEDBACK_SEQUENCE_INVALID;
    MotionExecutionIdentity identity{};
    MotionFeedbackType type = MotionFeedbackType::NONE;
    MotionRejectReason rejectReason = MotionRejectReason::NONE;
    MotionOwner owner = MotionOwner::NONE;
    MotionOwnerGeneration ownerGeneration = MOTION_OWNER_GENERATION_INVALID;
    std::uint32_t errorCode = 0U;
    double progress = 0.0;
};

static_assert(
    std::is_standard_layout<MotionExecutionIdentity>::value,
    "MotionExecutionIdentity must remain standard-layout.");

static_assert(
    std::is_trivially_copyable<MotionExecutionIdentity>::value,
    "MotionExecutionIdentity must remain trivially copyable.");

static_assert(
    std::is_standard_layout<MotionOwnerLease>::value,
    "MotionOwnerLease must remain standard-layout.");

static_assert(
    std::is_trivially_copyable<MotionOwnerLease>::value,
    "MotionOwnerLease must remain trivially copyable.");

static_assert(
    std::is_standard_layout<MotionFeedbackEvent>::value,
    "MotionFeedbackEvent must remain standard-layout.");

static_assert(
    std::is_trivially_copyable<MotionFeedbackEvent>::value,
    "MotionFeedbackEvent must remain trivially copyable.");
