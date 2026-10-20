#pragma once

#include <cstdint>
#include <limits>
#include <type_traits>

// Observation only. All nanosecond stamps and raw QPC stamps come from the
// caller's existing captures. These records are not wire or drive receipts.
namespace EDM174Comm
{
    static constexpr std::uint32_t HistoryCapacity = 20U;

    enum SampleFlag : std::uint32_t
    {
        ProcessValid = 1U << 0,
        MatchedTimingValid = 1U << 1,
        Injected = 1U << 2,
        CallbackTimingMissing = 1U << 8,
        CallbackTimingBackwards = 1U << 9,
        CommTimingMissing = 1U << 10,
        CommTimingBackwards = 1U << 11,
        MotionTimingMissing = 1U << 12,
        MotionTimingBackwards = 1U << 13,
        CommOutsideCallback = 1U << 14,
        MotionOutsideCallback = 1U << 15,
        StageOrderBackwards = 1U << 16,
        PreviousCycleMissing = 1U << 17,
        StartTimingBackwards = 1U << 18,
        QpcTimingMissing = 1U << 19,
        QpcTimingBackwards = 1U << 20,
        QpcFrequencyMissing = 1U << 21,
        QpcFrequencyChanged = 1U << 22,
        PreviousCycleOverlap = 1U << 23
    };
    static constexpr std::uint32_t DerivedFlags = 0x00ffff00U;

    enum FaultReason : std::uint32_t
    {
        InputFault = 1U << 0,
        GuardFault = 1U << 1
    };
    enum MaximumValid : std::uint32_t
    {
        CallbackMaximum = 1U << 0,
        MotionMaximum = 1U << 1,
        CommMaximum = 1U << 2,
        StartGapMaximum = 1U << 3
    };

    struct MotionObservation
    {
        std::uint64_t session = 0U, inputFaultTick = 0U, guardFaultTick = 0U;
        std::uint32_t phase = 0U, child = 0U;
    };

    struct Cycle
    {
        std::uint64_t tick = 0U, startNs = 0U, commStartNs = 0U, commEndNs = 0U;
        std::uint64_t motionStartNs = 0U, motionEndNs = 0U, endNs = 0U;
        std::uint64_t txBeforeQpc = 0U, txAfterQpc = 0U, rxAfterQpc = 0U;
        // Always replaced by the collector with the preceding recorded cycle.
        std::uint64_t prevStartNs = 0U, prevEndNs = 0U;
        std::int32_t lrwWkc = 0, dcWkc = 0, expectedWkc = 0;
        std::uint32_t phaseBefore = 0U, phaseAfter = 0U;
        std::uint32_t childBefore = 0U, childAfter = 0U, flags = 0U;
    };
    static_assert(sizeof(Cycle) == 128U, "Communication trace cycle size changed.");

    struct Snapshot
    {
        std::uint64_t session = 0U, qpcFrequency = 0U, recorded = 0U;
        std::uint64_t firstTick = 0U, lastTick = 0U;
        std::uint64_t inputFaultTick = 0U, guardFaultTick = 0U;
        std::uint32_t reasonFlags = 0U, historyCount = 0U;
        std::uint32_t maxValid = 0U, invalidTimingCycles = 0U;
        // Chronological at freeze; never includes firstBad.
        Cycle history[HistoryCapacity]{};
        Cycle firstBad{};
        Cycle maxCallback{}, maxMotion{}, maxComm{}, maxStartGap{};
        Cycle maxStartGapPrevious{};
        std::uint64_t maxCallbackNs = 0U, maxMotionNs = 0U;
        std::uint64_t maxCommNs = 0U, maxStartGapNs = 0U;
    };
    static_assert(sizeof(Snapshot) <= 4096U,
        "Communication trace must fit the bounded feedback channel.");
    static_assert(std::is_trivially_copyable<Snapshot>::value,
        "Communication trace must remain a value publication.");

    // Zero means unavailable, not proof that an operation did not happen.
    // Equal nonzero stamps are valid at the source clock's resolution.
    inline bool SafeElapsed(std::uint64_t begin, std::uint64_t end,
        std::uint64_t& elapsed) noexcept
    {
        elapsed = 0U;
        if (begin == 0U || end == 0U || end < begin) return false;
        elapsed = end - begin;
        return true;
    }

    class Collector
    {
    public:
        // True only for the first fault that freezes this nonzero session.
        // Zero session is ignored so a fault-induced retirement cannot erase
        // an unpublished first fault. A new nonzero session starts cleanly.
        bool Record(std::uint64_t session, const Cycle& supplied,
            std::uint64_t inputFaultTick, std::uint64_t guardFaultTick,
            std::uint64_t qpcFrequency) noexcept
        {
            if (session == 0U) return false;
            if (snapshot_.session == session && frozen_) return false;
            Cycle cycle = supplied;
            if (snapshot_.session != session) Reset(session);
            cycle.flags &= ~DerivedFlags;
            cycle.prevStartNs = havePrevious_ ? previous_.startNs : 0U;
            cycle.prevEndNs = havePrevious_ ? previous_.endNs : 0U;
            if (snapshot_.qpcFrequency == 0U && qpcFrequency != 0U)
                snapshot_.qpcFrequency = qpcFrequency;
            Validate(cycle, qpcFrequency);
            if (snapshot_.recorded == 0U) snapshot_.firstTick = cycle.tick;
            Increment(snapshot_.recorded);
            snapshot_.lastTick = cycle.tick;
            if ((cycle.flags & (DerivedFlags & ~PreviousCycleMissing)) != 0U)
                Increment(snapshot_.invalidTimingCycles);
            UpdateMaxima(cycle);
            if (inputFaultTick != 0U || guardFaultTick != 0U)
            {
                snapshot_.inputFaultTick = inputFaultTick;
                snapshot_.guardFaultTick = guardFaultTick;
                snapshot_.reasonFlags = (inputFaultTick != 0U ? InputFault : 0U) |
                    (guardFaultTick != 0U ? GuardFault : 0U);
                snapshot_.firstBad = cycle;
                OrderHistory();
                frozen_ = true;
                return true;
            }
            snapshot_.history[nextHistory_] = cycle;
            nextHistory_ = (nextHistory_ + 1U) % HistoryCapacity;
            if (snapshot_.historyCount < HistoryCapacity) ++snapshot_.historyCount;
            previous_ = cycle;
            havePrevious_ = true;
            return false;
        }

        bool Pending() const noexcept { return frozen_ && !published_; }
        const EDM174Comm::Snapshot& Snapshot() const noexcept { return snapshot_; }
        void MarkPublished() noexcept { if (frozen_) published_ = true; }

    private:
        template<class Integer> static void Increment(Integer& value) noexcept
        { if (value != (std::numeric_limits<Integer>::max)()) ++value; }

        void Reset(std::uint64_t session) noexcept
        {
            // Only a session transition clears the complete fixed-size value.
            snapshot_ = EDM174Comm::Snapshot{};
            snapshot_.session = session;
            previous_ = Cycle{};
            nextHistory_ = 0U;
            havePrevious_ = frozen_ = published_ = false;
        }

        void Validate(Cycle& c, std::uint64_t qpcFrequency) const noexcept
        {
            if (c.startNs == 0U || c.endNs == 0U) c.flags |= CallbackTimingMissing;
            else if (c.endNs < c.startNs) c.flags |= CallbackTimingBackwards;
            if (c.commStartNs == 0U || c.commEndNs == 0U) c.flags |= CommTimingMissing;
            else if (c.commEndNs < c.commStartNs) c.flags |= CommTimingBackwards;
            if (c.motionStartNs == 0U || c.motionEndNs == 0U) c.flags |= MotionTimingMissing;
            else if (c.motionEndNs < c.motionStartNs) c.flags |= MotionTimingBackwards;
            if ((c.startNs != 0U && c.commStartNs != 0U && c.commStartNs < c.startNs) ||
                (c.endNs != 0U && c.commEndNs != 0U && c.commEndNs > c.endNs))
                c.flags |= CommOutsideCallback;
            if ((c.startNs != 0U && c.motionStartNs != 0U && c.motionStartNs < c.startNs) ||
                (c.endNs != 0U && c.motionEndNs != 0U && c.motionEndNs > c.endNs))
                c.flags |= MotionOutsideCallback;
            if (c.commEndNs != 0U && c.motionStartNs != 0U && c.motionStartNs < c.commEndNs)
                c.flags |= StageOrderBackwards;
            if (!havePrevious_) c.flags |= PreviousCycleMissing;
            else
            {
                if (c.startNs == 0U || previous_.startNs == 0U)
                    c.flags |= PreviousCycleMissing;
                else if (c.startNs < previous_.startNs) c.flags |= StartTimingBackwards;
                if (c.startNs != 0U && previous_.endNs != 0U && c.startNs < previous_.endNs)
                    c.flags |= PreviousCycleOverlap;
            }
            if (c.txBeforeQpc == 0U || c.txAfterQpc == 0U || c.rxAfterQpc == 0U)
                c.flags |= QpcTimingMissing;
            else if (c.txAfterQpc < c.txBeforeQpc || c.rxAfterQpc < c.txAfterQpc)
                c.flags |= QpcTimingBackwards;
            if (qpcFrequency == 0U) c.flags |= QpcFrequencyMissing;
            else if (snapshot_.qpcFrequency != qpcFrequency) c.flags |= QpcFrequencyChanged;
        }

        void Maximum(std::uint32_t bit, const Cycle& cycle, std::uint64_t elapsed,
            Cycle& retained, std::uint64_t& maximum) noexcept
        {
            if ((snapshot_.maxValid & bit) == 0U || elapsed > maximum)
            {
                retained = cycle;
                maximum = elapsed;
                snapshot_.maxValid |= bit;
            }
        }

        void UpdateMaxima(const Cycle& c) noexcept
        {
            std::uint64_t elapsed = 0U;
            if (SafeElapsed(c.startNs, c.endNs, elapsed))
                Maximum(CallbackMaximum, c, elapsed, snapshot_.maxCallback, snapshot_.maxCallbackNs);
            if ((c.flags & MotionOutsideCallback) == 0U && SafeElapsed(c.motionStartNs, c.motionEndNs, elapsed))
                Maximum(MotionMaximum, c, elapsed, snapshot_.maxMotion, snapshot_.maxMotionNs);
            if ((c.flags & CommOutsideCallback) == 0U && SafeElapsed(c.commStartNs, c.commEndNs, elapsed))
                Maximum(CommMaximum, c, elapsed, snapshot_.maxComm, snapshot_.maxCommNs);
            if (havePrevious_ && SafeElapsed(c.prevStartNs, c.startNs, elapsed) &&
                ((snapshot_.maxValid & StartGapMaximum) == 0U || elapsed > snapshot_.maxStartGapNs))
            {
                Maximum(StartGapMaximum, c, elapsed, snapshot_.maxStartGap, snapshot_.maxStartGapNs);
                snapshot_.maxStartGapPrevious = previous_;
            }
        }

        void Reverse(std::uint32_t first, std::uint32_t last) noexcept
        {
            while (first < last)
            {
                --last;
                if (first >= last) break;
                const Cycle temporary = snapshot_.history[first];
                snapshot_.history[first++] = snapshot_.history[last];
                snapshot_.history[last] = temporary;
            }
        }
        void OrderHistory() noexcept
        {
            if (snapshot_.historyCount != HistoryCapacity || nextHistory_ == 0U) return;
            Reverse(0U, nextHistory_);
            Reverse(nextHistory_, HistoryCapacity);
            Reverse(0U, HistoryCapacity);
        }

        EDM174Comm::Snapshot snapshot_{};
        Cycle previous_{};
        std::uint32_t nextHistory_ = 0U;
        bool havePrevious_ = false, frozen_ = false, published_ = false;
    };

    // Defined at the existing runtime-to-supervisory publication seam.
    bool Read(Snapshot& snapshot, std::uint64_t& publication) noexcept;
}
