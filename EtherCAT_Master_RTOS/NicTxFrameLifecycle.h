#pragma once
// EDM45 bounded transport lifecycle. Protocol response evidence and NAL frame
// ownership are independent facts. Neither is a motion/drive acknowledgement.
#include "NicTxReceipt.h"
#include <array>
#include <atomic>
#include <cstddef>

class NicTxProducerScope
{
public:
    explicit NicTxProducerScope(std::atomic<std::uint32_t>& active) noexcept
        : active_(active)
    {
        std::uint32_t expected = 0U;
        held_ = active_.compare_exchange_strong(expected, 1U,
            std::memory_order_acquire, std::memory_order_relaxed);
    }
    ~NicTxProducerScope() noexcept
    { if (held_) active_.store(0U, std::memory_order_release); }
    bool Held() const noexcept { return held_; }
    NicTxProducerScope(const NicTxProducerScope&) = delete;
    NicTxProducerScope& operator=(const NicTxProducerScope&) = delete;
private:
    std::atomic<std::uint32_t>& active_;
    bool held_ = false;
};

class NicTxFrameLifecycle
{
public:
    static constexpr std::size_t SlotCount = 4U;
    static constexpr std::size_t NoSlot = SlotCount;
    enum class State : std::uint32_t
    {
        Available = 0U,
        AcceptedPendingResponse = 1U,
        AcceptedResponseObserved = 2U,
        Uncertain = 3U
    };
    void Reset(std::uint64_t openGeneration) noexcept
    {
        openGeneration_ = openGeneration;
        slots_ = std::array<Slot, SlotCount>{};
        responseTrustTainted_ = false;
    }
    void Invalidate() noexcept
    { openGeneration_ = 0ULL; slots_ = std::array<Slot, SlotCount>{}; }
    // Caller has just observed RtNalIsApplicationFrame == TRUE. This removes
    // any older transmit-order fence, including an uncertain invocation. It
    // never rewrites the original receipt or resolves higher-layer uncertainty.
    void ObserveApplicationOwnership(std::size_t index) noexcept
    {
        if (index >= SlotCount) return;
        const Slot& slot = slots_[index];
        if (slot.responseTrustRelevant && slot.state == State::AcceptedPendingResponse)
            responseTrustTainted_ = true;
        slots_[index] = Slot{};
    }
    void RecordInvocation(std::size_t index, const NicTxReceipt& receipt,
        bool responseWitnessAllowed = false, bool responseTrustRelevant = false,
        bool readOnlyEscProbe = false) noexcept
    {
        if (index >= SlotCount) return;
        Slot& slot = slots_[index];
        slot.receipt = receipt;
        slot.responseWitnessAllowed = responseWitnessAllowed;
        slot.responseTrustRelevant = responseTrustRelevant;
        slot.readOnlyEscProbe = readOnlyEscProbe && IsAccepted(receipt);
        slot.state = IsAccepted(receipt) ? State::AcceptedPendingResponse : State::Uncertain;
        if (responseTrustRelevant && slot.state == State::Uncertain)
            responseTrustTainted_ = true;
    }
    // This is a protocol-order observation supplied ONLY by the same current
    // transaction's validated RX path. It does not return application ownership,
    // permit writing this slot, imply WKC success, or authorize motion recovery.
    bool ObserveValidatedResponse(const NicTxReceipt& receipt) noexcept
    {
        if (!IsAccepted(receipt) || receipt.openGeneration != openGeneration_ ||
            receipt.frameSlot == 0U || receipt.frameSlot > SlotCount) return false;
        Slot& slot = slots_[receipt.frameSlot - 1U];
        if (!slot.responseWitnessAllowed ||
            (slot.state != State::AcceptedPendingResponse &&
            slot.state != State::AcceptedResponseObserved) ||
            !SameReceipt(slot.receipt, receipt)) return false;
        slot.state = State::AcceptedResponseObserved;
        return true;
    }
    // A spare frame is insufficient if an older transaction may still be sent.
    // Lowest-call fence makes provenance deterministic; no FIFO is assumed.
    std::size_t BlockingSlot(bool currentIsRuntimeCyclic = false) const noexcept
    {
        std::size_t selected = NoSlot;
        for (std::size_t i = 0U; i < SlotCount; ++i)
        {
            // The exact accepted ESC read cannot write an older process image.
            // Only a later validated cyclic-shaped submission may pass it.
            // This is independent of RX identity and never permits slot reuse.
            if (currentIsRuntimeCyclic && slots_[i].readOnlyEscProbe &&
                slots_[i].state == State::AcceptedPendingResponse) continue;
            if (slots_[i].state != State::AcceptedPendingResponse &&
                slots_[i].state != State::Uncertain &&
                !(responseTrustTainted_ &&
                    slots_[i].state == State::AcceptedResponseObserved)) continue;
            if (selected == NoSlot || slots_[i].receipt.callSequence <
                slots_[selected].receipt.callSequence) selected = i;
        }
        return selected;
    }
    State SlotState(std::size_t index) const noexcept
    { return index < SlotCount ? slots_[index].state : State::Available; }
    bool IsReadOnlyEscSlot(std::size_t index) const noexcept
    { return index < SlotCount && slots_[index].readOnlyEscProbe; }
    // Sticky until successful quiescent Open. A missing/ambiguous transaction
    // in the LRW wire domain disables response-only advancement; ownership
    // reuse remains. Startup OtherLRW variants share that same wire domain.
    bool ResponseTrustTainted() const noexcept { return responseTrustTainted_; }
    // Serialized request classification, not a caller-label assertion. Limits
    // exactly mirror the existing non-output ProcessRuntimeEscDiagProbe set.
    static bool IsReadOnlyEscProbe(const unsigned char* data, std::size_t length) noexcept
    {
        if (!EtherCatEnvelope(data, length) || data[16] != 0x04U ||
            Read16(data + 18U) == 0U) return false;
        const std::uint16_t bytes = Read16(data + 22U);
        const std::uint16_t address = Read16(data + 20U);
        if (bytes == 0U || length != 28U + bytes ||
            Read16(data + 24U) != 0U || Read16(data + length - 2U) != 0U)
            return false;
        return (address == 0x0130U && bytes == 6U) ||
            (address == 0x0110U && bytes == 2U) ||
            (address == 0x0300U && bytes == 20U) ||
            (address == 0x0400U && bytes == 67U) ||
            (address >= 0x0800U && address <= 0x0878U &&
                ((address - 0x0800U) % 8U) == 0U && bytes == 8U) ||
            (address >= 0x0600U && address <= 0x06F0U &&
                ((address - 0x0600U) % 16U) == 0U && bytes == 16U);
    }
    static bool IsCyclicFrame(const unsigned char* data, std::size_t length,
        bool withFrmw) noexcept
    {
        if (!EtherCatEnvelope(data, length) || data[16] != 0x0CU) return false;
        const std::uint16_t field = Read16(data + 22U);
        const std::size_t bytes = field & 0x07FFU;
        if (bytes == 0U || field != (bytes | (withFrmw ? 0x8000U : 0U)) ||
            length != 28U + bytes + (withFrmw ? 20U : 0U) ||
            Read16(data + 24U) != 0U || Read16(data + 26U + bytes) != 0U)
            return false;
        if (!withFrmw) return true;
        const unsigned char* frmw = data + 28U + bytes;
        return frmw[0] == 0x0EU && Read16(frmw + 2U) != 0U &&
            Read16(frmw + 4U) == 0x0910U && Read16(frmw + 6U) == 8U &&
            Read16(frmw + 8U) == 0U && Read16(frmw + 18U) == 0U;
    }
    static bool IsAccepted(const NicTxReceipt& receipt) noexcept
    {
        return receipt.callSequence != 0ULL && receipt.openGeneration != 0ULL &&
            receipt.frameSlot != 0U && receipt.frameSlot <= SlotCount &&
            receipt.reason == NicTxReason::Accepted && receipt.nalInvoked == 1U &&
            receipt.apiResult == 1 && receipt.submitted == 1U &&
            receipt.error == 0U && receipt.errorValid == 0U;
    }
private:
    struct Slot
    {
        State state = State::Available;
        NicTxReceipt receipt{};
        bool responseWitnessAllowed = false;
        bool responseTrustRelevant = false;
        bool readOnlyEscProbe = false;
    };
    static std::uint16_t Read16(const unsigned char* data) noexcept
    {
        return static_cast<std::uint16_t>(data[0]) |
            static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[1]) << 8U);
    }
    static bool EtherCatEnvelope(const unsigned char* data, std::size_t length) noexcept
    {
        return data != nullptr && length >= 28U && length <= 1514U &&
            data[12] == 0x88U && data[13] == 0xA4U &&
            Read16(data + 14U) == (0x1000U | (length - 16U));
    }
    static bool SameReceipt(const NicTxReceipt& a, const NicTxReceipt& b) noexcept
    {
        return a.callSequence == b.callSequence && a.length == b.length &&
            a.nalInvoked == b.nalInvoked && a.apiResult == b.apiResult &&
            a.submitted == b.submitted && a.error == b.error &&
            a.errorValid == b.errorValid && a.reason == b.reason &&
            a.openGeneration == b.openGeneration && a.frameSlot == b.frameSlot;
    }
    std::uint64_t openGeneration_ = 0ULL;
    bool responseTrustTainted_ = false;
    std::array<Slot, SlotCount> slots_{};
};
