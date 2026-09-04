#pragma once

#include <stdint.h>

// DC-DIAG.2 - executable-internal Priority-64 diagnostic handoff.
//
// Priority 64 publishes numeric values only. Priority 50 performs any text
// formatting after a coherent, bounded read. This contract does not alter
// Shared Memory or any public EtherCAT API.
enum class EtherCatP64DeferredDiagKind : uint32_t
{
    None = 0u,
    HalBurstStart = 1u,
    HalBurstGetCountsFailed = 2u,
    HalBurstAltCyclesInvalid = 3u,
    HalBurstPlan = 4u,
    HalBurstSetFailed = 5u,
    HalBurstBaseRestored = 6u,
    HalBurstBaseRestoreFailed = 7u,
    HalBurstSummary = 8u,
    HalBurstDisabledBaseRestored = 9u,
    DcSlaveSample = 10u,
    DcSlaveReadFailed = 11u,
    AsyncStateWithoutFullPdoWkc = 12u
};

struct EtherCatP64DeferredDiagSnapshot
{
    uint64_t EventSerial = 0ULL;
    uint64_t PdoTick = 0ULL;
    uint32_t Kind =
        static_cast<uint32_t>(EtherCatP64DeferredDiagKind::None);
    uint32_t Reserved = 0u;
    int64_t Value0 = 0LL;
    int64_t Value1 = 0LL;
    int64_t Value2 = 0LL;
    int64_t Value3 = 0LL;
    int64_t Value4 = 0LL;
    int64_t Value5 = 0LL;
    int64_t Value6 = 0LL;
    int64_t Value7 = 0LL;
};

bool TryReadEtherCatP64DeferredDiagnostic(
    EtherCatP64DeferredDiagSnapshot& snapshot) noexcept;
