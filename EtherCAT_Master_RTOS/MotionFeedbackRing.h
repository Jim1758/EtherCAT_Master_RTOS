#pragma once

#include <cstddef>

#include "MotionExecutionContract.h"
#include "MotionCommandRing.h"

// ============================================================================
// Stage NC-0.1D - Fixed-Capacity Motion Feedback Channel
//
// Final Feedback Ring:
//
//     Producer = 250 us Motion Runtime only
//     Consumer = NC 10 ms Task only
//
// Producer Notice Ring:
//
//     Producer = NC / MDI Command Producer only
//     Consumer = 250 us Motion Runtime only
//
// Command Queue Full 發生在 NC Producer 執行緒。它不能直接寫入 Final
// Feedback Ring，否則 Final Ring 就會變成多 Producer。Notice Ring 先把拒絕
// 事件交給 Runtime，再由 Runtime 統一配發 sequence 並發布到 Final Ring。
//
// 兩個方向都不配置記憶體、不使用 Mutex、不阻塞等待。
// ============================================================================

constexpr std::size_t MOTION_FEEDBACK_EVENT_CAPACITY = 2048U;
constexpr std::size_t MOTION_FEEDBACK_PRODUCER_NOTICE_CAPACITY = 256U;
constexpr std::size_t
MOTION_FEEDBACK_PRODUCER_NOTICE_DRAIN_LIMIT_PER_RUNTIME_CYCLE = 32U;
constexpr std::size_t MOTION_FEEDBACK_NC_DRAIN_LIMIT_PER_TASK = 512U;

// Epoch 切換時，最壞情況會同時淘汰 Replay + Ingress 全部命令。
// Final Feedback Ring 必須能容納整批 STALE_EPOCH 回報，並保留終止事件餘裕。
static_assert(
    MOTION_FEEDBACK_EVENT_CAPACITY >=
    MOTION_COMMAND_INGRESS_CAPACITY +
    MOTION_COMMAND_REPLAY_CAPACITY +
    64U,
    "Feedback ring must cover the maximum stale-command burst.");

static_assert(
    MOTION_FEEDBACK_PRODUCER_NOTICE_CAPACITY >=
    MOTION_COMMAND_INGRESS_CAPACITY,
    "Producer notice ring must cover the command ingress capacity.");

static_assert(
    MOTION_FEEDBACK_PRODUCER_NOTICE_DRAIN_LIMIT_PER_RUNTIME_CYCLE > 0U &&
    MOTION_FEEDBACK_PRODUCER_NOTICE_DRAIN_LIMIT_PER_RUNTIME_CYCLE <=
    MOTION_FEEDBACK_PRODUCER_NOTICE_CAPACITY,
    "Runtime notice drain limit must be bounded by notice ring capacity.");

static_assert(
    MOTION_FEEDBACK_NC_DRAIN_LIMIT_PER_TASK > 0U &&
    MOTION_FEEDBACK_NC_DRAIN_LIMIT_PER_TASK <=
    MOTION_FEEDBACK_EVENT_CAPACITY,
    "NC feedback drain limit must be bounded by ring capacity.");

class MotionFeedbackChannel
{
public:
    MotionFeedbackChannel() noexcept = default;

    MotionFeedbackChannel(
        const MotionFeedbackChannel&) = delete;

    MotionFeedbackChannel& operator=(
        const MotionFeedbackChannel&) = delete;

    // NC / MDI Command Producer only.
    bool CommandProducerTryPublishNotice(
        const MotionFeedbackEvent& event) noexcept
    {
        return
            m_producerNotices.ProducerTryPush(event);
    }

    // 250 us Motion Runtime only.
    bool RuntimeTryConsumeProducerNotice(
        MotionFeedbackEvent& event) noexcept
    {
        return
            m_producerNotices.ConsumerTryPop(event);
    }

    // 250 us Motion Runtime only.
    bool RuntimeTryPublishFeedback(
        const MotionFeedbackEvent& event) noexcept
    {
        return
            m_feedbackEvents.ProducerTryPush(event);
    }

    // NC 10 ms Task only.
    bool NcTryConsumeFeedback(
        MotionFeedbackEvent& event) noexcept
    {
        return
            m_feedbackEvents.ConsumerTryPop(event);
    }

    std::size_t feedback_size() const noexcept
    {
        return
            m_feedbackEvents.size();
    }

    std::size_t producer_notice_size() const noexcept
    {
        return
            m_producerNotices.size();
    }

    static constexpr std::size_t feedback_capacity() noexcept
    {
        return
            MOTION_FEEDBACK_EVENT_CAPACITY;
    }

    static constexpr std::size_t producer_notice_capacity() noexcept
    {
        return
            MOTION_FEEDBACK_PRODUCER_NOTICE_CAPACITY;
    }

private:
    FixedCapacitySpscRing<
        MotionFeedbackEvent,
        MOTION_FEEDBACK_PRODUCER_NOTICE_CAPACITY> m_producerNotices{};

    FixedCapacitySpscRing<
        MotionFeedbackEvent,
        MOTION_FEEDBACK_EVENT_CAPACITY> m_feedbackEvents{};
};
