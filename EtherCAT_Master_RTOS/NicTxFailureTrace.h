#pragma once
// EDM45 per-frame failure/ownership-return software handoff trace. This never grants motion or
// transport authority and is not evidence of a drive acknowledgement.
#include "NicTxReceipt.h"
#include <array>
#include <atomic>
#include <cstdint>
#include <type_traits>

enum class NicTxCaller : std::uint32_t
{
    Unspecified = 0U,
    ControlDatagram = 1U,
    RuntimeLRW = 2U,
    RuntimeLRW_FRMW = 3U,
    RuntimeSDO = 4U,
    OtherLRW = 5U,
    OtherLRW_FRMW = 6U,
    RuntimeEscDiag = 7U
};

// Value owned by this exact SendPacket caller. Packed authority is valid only
// while the caller still owns its existing send reservation. Source tick is
// the originating image proof, not an inference about present EDM test IDs.
struct NicTxCallContext
{
    NicTxCaller caller = NicTxCaller::Unspecified;
    std::uint32_t pdoTickValid = 0U;
    std::uint32_t sourceTickValid = 0U;
    std::uint32_t authorityValid = 0U;
    std::uint64_t pdoTick = 0ULL;
    std::uint64_t sourceTick = 0ULL;
    std::uint64_t packedOwnerState = 0ULL;
    std::uint64_t packedExecutionPublication = 0ULL;
    std::uint64_t edmFeedbackPublication = 0ULL;
    std::uint32_t edmGuardHeld = 0U;
};

enum class NicTxEventKind : std::uint32_t
{
    Failure = 1U,
    OwnershipReturned = 2U
};

// Software provenance only. An invocation timestamp is sampled AFTER NAL
// returns, and is neither transmit-completion time nor a drive acknowledgement.
struct NicTxPreviousInvocation
{
    NicTxReceipt receipt{};
    NicTxCallContext context{};
    std::uint64_t openGeneration = 0ULL;
    std::int64_t returnQpc = 0;
    std::uint32_t valid = 0U;
    std::uint32_t returnQpcValid = 0U;
    std::uint32_t datagramValid = 0U;
    std::uint32_t command = 0U;
    std::uint32_t index = 0U;
    std::uint32_t addressLow = 0U;
    std::uint32_t addressHigh = 0U;
};

struct NicTxBusyEpisode
{
    std::uint64_t firstCall = 0ULL;
    std::uint64_t lastCall = 0ULL;
    std::uint64_t count = 0ULL;
    std::int64_t firstQpc = 0;
    std::int64_t lastQpc = 0;
    std::uint32_t firstQpcValid = 0U;
    std::uint32_t lastQpcValid = 0U;
};

// status: 0 unavailable, 1 valid, 2 enumeration error, 3 bounded cap,
// 4 no MAC match, 5 duplicate MAC match, 6 priority outside RTSS range.
struct NicTxInterfaceObservation
{
    std::uint32_t status = 0U;
    std::uint32_t valid = 0U;
    std::uint32_t enumerationComplete = 0U;
    std::uint32_t matches = 0U;
    std::uint32_t enumerationError = 0U;
    std::uint32_t enumerated = 0U;
    std::uint32_t version = 0U;
    std::uint32_t intPriority = 0U;
    std::uint32_t intIdealProcessor = 0U;
    std::uint32_t txCompletePriority = 0U;
    std::uint32_t txCompleteIdealProcessor = 0U;
    std::uint32_t numTxBuffers = 0U;
};

struct NicTxFailureEvent
{
    NicTxReceipt receipt{};
    NicTxCallContext context{};
    std::uint64_t publicationSequence = 0ULL;
    std::uint64_t openGeneration = 0ULL;
    std::int64_t failureQpc = 0;
    std::uint32_t failureQpcValid = 0U;
    std::uint32_t datagramValid = 0U;
    std::uint32_t command = 0U;
    std::uint32_t index = 0U;
    std::uint32_t addressLow = 0U;
    std::uint32_t addressHigh = 0U;
    NicTxInterfaceObservation interfaceObservation{};
    NicTxEventKind kind = NicTxEventKind::Failure;
    NicTxPreviousInvocation previous{};
    NicTxBusyEpisode busy{};
    std::uint64_t historyGaps = 0ULL;
    std::uint64_t qpcFrequency = 0ULL;
    std::uint64_t previousAgeQpc = 0ULL;
    std::uint32_t historyCoherent = 0U;
    std::uint32_t qpcFrequencyValid = 0U;
    std::uint32_t previousAgeQpcValid = 0U;
    std::int32_t observedThreadPriority = 0;
    std::uint32_t observedThreadPriorityValid = 0U;
    // A blocking or reclaimed slot, not necessarily the current call's slot.
    // Previous invocation/history always belongs to this one observed slot.
    std::uint32_t observedFrameSlot = 0U;
    std::uint32_t poolSlotCount = 4U;
    std::uint32_t poolOwnedCount = 0U;
    std::uint32_t protocolFence = 0U;
    std::uint32_t producerOverlap = 0U;
    std::uint32_t slotState = 0U;
    std::uint32_t responseTrustTainted = 0U;
    std::uint32_t responseTrustValid = 0U;
    std::uint32_t slotReadOnlyEsc = 0U;
    std::uint32_t currentCyclicFrame = 0U;
};

static_assert(std::is_trivially_copyable<NicTxFailureEvent>::value &&
    sizeof(NicTxFailureEvent) <= 544U, "NIC failure trace must remain bounded POD.");
static_assert(ATOMIC_INT_LOCK_FREE == 2 && ATOMIC_LLONG_LOCK_FREE == 2,
    "NIC trace requires the existing x64 lock-free atomic platform.");

// Multi-producer, single-consumer, fixed storage. A producer chooses one slot
// and makes ONE claim attempt: a busy slot drops this diagnostic instead of
// waiting, scanning, overwriting unread evidence, or changing SendPacket.
class NicTxFailureTrace
{
public:
    static constexpr std::size_t Capacity = 16U;
    bool TryPublish(const NicTxFailureEvent& event) noexcept
    {
        const std::uint64_t ticket = attempts_.fetch_add(1ULL, std::memory_order_relaxed);
        Slot& slot = slots_[static_cast<std::size_t>(ticket % Capacity)];
        std::uint32_t expected = Empty;
        if (!slot.state.compare_exchange_strong(expected, Writing,
            std::memory_order_acquire, std::memory_order_relaxed))
        {
            dropped_.fetch_add(1ULL, std::memory_order_relaxed);
            return false;
        }
        slot.event = event;
        slot.event.publicationSequence = ticket + 1ULL;
        slot.state.store(Ready, std::memory_order_release);
        return true;
    }
    // Exactly one low-priority consumer owns nextSlot_. Empty/being-written
    // slots are skipped; each caller can inspect at most Capacity slots.
    bool TryPop(NicTxFailureEvent& event) noexcept
    {
        for (std::size_t inspected = 0U; inspected < Capacity; ++inspected)
        {
            Slot& slot = slots_[nextSlot_];
            nextSlot_ = (nextSlot_ + 1U) % Capacity;
            if (slot.state.load(std::memory_order_acquire) != Ready) continue;
            event = slot.event;
            slot.state.store(Empty, std::memory_order_release);
            return true;
        }
        return false;
    }
    std::uint64_t Attempts() const noexcept
    { return attempts_.load(std::memory_order_relaxed); }
    std::uint64_t Dropped() const noexcept
    { return dropped_.load(std::memory_order_relaxed); }
private:
    enum : std::uint32_t { Empty = 0U, Writing = 1U, Ready = 2U };
    struct Slot
    {
        std::atomic<std::uint32_t> state{ Empty };
        NicTxFailureEvent event{};
    };
    std::array<Slot, Capacity> slots_{};
    std::atomic<std::uint64_t> attempts_{ 0ULL }, dropped_{ 0ULL };
    std::size_t nextSlot_ = 0U;
};
