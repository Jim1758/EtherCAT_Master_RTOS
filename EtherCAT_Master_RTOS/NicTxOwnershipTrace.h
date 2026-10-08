#pragma once
// EDM45 diagnostics only, one independent instance per allocated TX frame.
// Previous/history belongs to that slot, never the globally previous call.
// This diagnostic scope never grants ownership or serializes transmission.
#include "NicTxFailureTrace.h"

class NicTxOwnershipTraceState
{
public:
    class CallScope
    {
    public:
        explicit CallScope(NicTxOwnershipTraceState& state) noexcept : state_(state)
        {
            std::uint32_t expected = 0U;
            held_ = state_.active_.compare_exchange_strong(expected, 1U,
                std::memory_order_acquire, std::memory_order_relaxed);
            if (!held_)
            {
                state_.unguardedActive_.fetch_add(1ULL, std::memory_order_acq_rel);
                state_.gaps_.fetch_add(1ULL, std::memory_order_acq_rel);
                return;
            }
            generation_ = state_.gaps_.load(std::memory_order_acquire);
            tainted_ = state_.unguardedActive_.load(std::memory_order_acquire) != 0ULL;
            if (state_.knownGap_ != generation_)
            {
                state_.ClearHistory();
                state_.knownGap_ = generation_;
            }
        }
        ~CallScope() noexcept
        {
            if (!held_)
            {
                // An unrecorded invocation may happen anywhere in this losing
                // scope. Keep all later holders untrusted until it has exited.
                state_.gaps_.fetch_add(1ULL, std::memory_order_acq_rel);
                state_.unguardedActive_.fetch_sub(1ULL, std::memory_order_acq_rel);
                return;
            }
            (void)Trusted();
            state_.active_.store(0U, std::memory_order_release);
        }
        CallScope(const CallScope&) = delete;
        CallScope& operator=(const CallScope&) = delete;
    private:
        friend class NicTxOwnershipTraceState;
        bool Trusted() noexcept
        {
            if (!held_) return false;
            const std::uint64_t beforeGap = state_.gaps_.load(std::memory_order_acquire);
            if (state_.unguardedActive_.load(std::memory_order_acquire) != 0ULL)
                tainted_ = true;
            // A losing scope can enter AND exit between these reads. The
            // second generation load prevents mistaking that ABA for no gap.
            const std::uint64_t gap = state_.gaps_.load(std::memory_order_acquire);
            if (tainted_ || beforeGap != generation_ || gap != generation_)
            {
                state_.ClearHistory();
                state_.knownGap_ = gap;
                return false;
            }
            return true;
        }
        NicTxOwnershipTraceState& state_;
        std::uint64_t generation_ = 0ULL;
        bool held_ = false;
        bool tainted_ = false;
    };

    // Existing quiescent Open only; no unread ring event is reset here.
    void Reset(std::uint64_t frequency, bool frequencyValid) noexcept
    {
        ClearHistory();
        knownGap_ = gaps_.load(std::memory_order_acquire);
        frequency_ = frequency;
        frequencyValid_ = frequencyValid && frequency != 0ULL;
    }
    bool HasBusyEpisode(CallScope& scope) noexcept
    { return scope.Trusted() && busy_.count != 0ULL; }

    void ObserveFailure(CallScope& scope, NicTxFailureEvent& event,
        bool ownershipNotOwner) noexcept
    {
        if (!Capture(scope, event)) return;
        if (event.receipt.reason == NicTxReason::PreSubmitOwnershipRejected &&
            !ownershipNotOwner)
        {
            busy_ = NicTxBusyEpisode{};
            event.busy = busy_;
            return;
        }
        if (event.receipt.reason == NicTxReason::PreSubmitOwnershipRejected &&
            ownershipNotOwner)
        {
            if (busy_.count == 0ULL)
            {
                busy_.firstCall = event.receipt.callSequence;
                busy_.firstQpc = event.failureQpc;
                busy_.firstQpcValid = event.failureQpcValid;
            }
            busy_.lastCall = event.receipt.callSequence;
            busy_.lastQpc = event.failureQpc;
            busy_.lastQpcValid = event.failureQpcValid;
            ++busy_.count;
            event.busy = busy_;
        }
    }
    bool ObserveOwnershipReturn(CallScope& scope, NicTxFailureEvent& event) noexcept
    {
        if (!HasBusyEpisode(scope) || !Capture(scope, event)) return false;
        event.kind = NicTxEventKind::OwnershipReturned;
        event.busy = busy_;
        busy_ = NicTxBusyEpisode{};
        return true;
    }
    // A final generation check gives the diagnostic snapshot a bounded
    // linearization point. Later overlap invalidates retained history at End.
    void ValidateEvent(CallScope& scope, NicTxFailureEvent& event) noexcept
    {
        const bool trusted = scope.Trusted();
        if (!trusted)
        {
            event.previous = NicTxPreviousInvocation{};
            event.busy = NicTxBusyEpisode{};
            event.historyCoherent = 0U;
            event.previousAgeQpc = 0ULL;
            event.previousAgeQpcValid = 0U;
        }
        event.historyGaps = trusted ? scope.generation_ :
            gaps_.load(std::memory_order_acquire);
    }
    void RecordInvocation(CallScope& scope,
        const NicTxPreviousInvocation& invocation) noexcept
    {
        if (!scope.Trusted()) return;
        previous_ = invocation;
        previous_.valid = invocation.receipt.nalInvoked == 1U ? 1U : 0U;
    }
private:
    void ClearHistory() noexcept
    { previous_ = NicTxPreviousInvocation{}; busy_ = NicTxBusyEpisode{}; }
    bool Capture(CallScope& scope, NicTxFailureEvent& event) noexcept
    {
        event.qpcFrequency = frequency_;
        event.qpcFrequencyValid = frequencyValid_ ? 1U : 0U;
        event.historyGaps = gaps_.load(std::memory_order_acquire);
        if (!scope.Trusted()) return false;
        event.historyCoherent = 1U;
        event.previous = previous_;
        event.busy = busy_;
        if (previous_.valid != 0U && previous_.returnQpcValid != 0U &&
            event.failureQpcValid != 0U && frequencyValid_ &&
            previous_.returnQpc >= 0 && event.failureQpc >= previous_.returnQpc)
        {
            event.previousAgeQpc = static_cast<std::uint64_t>(
                event.failureQpc - previous_.returnQpc);
            event.previousAgeQpcValid = 1U;
        }
        return true;
    }
    std::atomic<std::uint32_t> active_{ 0U };
    std::atomic<std::uint64_t> gaps_{ 0ULL };
    std::atomic<std::uint64_t> unguardedActive_{ 0ULL };
    std::uint64_t knownGap_ = 0ULL;
    std::uint64_t frequency_ = 0ULL;
    bool frequencyValid_ = false;
    NicTxPreviousInvocation previous_{};
    NicTxBusyEpisode busy_{};
};
