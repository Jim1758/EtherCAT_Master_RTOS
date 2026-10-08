#pragma once
// Per-call software handoff evidence. This is not a drive acknowledgement.
// Only the exact SendPacket caller consumes its local receipt for safety.
#include <cstdint>

enum class NicTxReason : std::uint32_t
{
    Unreported = 0U,
    PreSubmitInvalidInput,
    PreSubmitOwnershipRejected,
    Accepted,
    NalCallFailed,
    SubmittedCountMismatch
};

struct NicTxReceipt
{
    std::uint64_t callSequence = 0ULL;
    std::uint32_t length = 0U;
    std::uint32_t nalInvoked = 0U;
    std::int32_t apiResult = 0;
    std::uint32_t submitted = 0U;
    std::uint32_t error = 0U;
    std::uint32_t errorValid = 0U;
    NicTxReason reason = NicTxReason::Unreported;
    // Transport-only identity for an exact validated-response observation.
    // Slots are 1..4; zero means no NAL frame was selected for this call.
    std::uint64_t openGeneration = 0ULL;
    std::uint32_t frameSlot = 0U;

    bool WasTransmissionAttempted() const noexcept
    {
        // Missing/default or contradictory evidence must remain conservative.
        // No inference is made from an error code or submitted==0 after NAL.
        const bool explicitlyBeforeSubmit =
            reason == NicTxReason::PreSubmitInvalidInput ||
            reason == NicTxReason::PreSubmitOwnershipRejected;
        const bool consistentBeforeSubmit =
            callSequence != 0ULL && nalInvoked == 0U && apiResult == 0 &&
            submitted == 0U && errorValid == 1U;
        return !(explicitlyBeforeSubmit && consistentBeforeSubmit);
    }
};

// Low-priority, single diagnostic consumer only. Cumulative/advisory values
// never replace a same-call receipt or authorize recovery of unknown output.
void PrintNicTransmitFailureDiagnostic() noexcept;
