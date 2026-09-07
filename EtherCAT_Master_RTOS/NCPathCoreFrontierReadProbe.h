#pragma once

#include "NCPathCoreFrontierReadContract.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

// NC-0.2L.2AI / Same-Thread Frontier Read Integration Shadow Probe.
//
// First production CALL SITE for the AG ticket and AH value-read contracts.
// This is NOT another proof/publication/history layer: exactly ONE last-sample
// workspace lives in the already heap-owned NCManager, with no generation,
// counter, ring, queue, new task, timer or control consumer. Sampling happens
// only at the end of the existing accepted-read-ahead shadow observation,
// after AF. It is NOT added to the PDO thread or to every Idle/ProcessTask scan.
//
// Capture AG into the member value.ticket, then use AH's supported in-place
// read. Only AF's existing 80 scalar bytes and AG's 32-byte identity are copied.
// Sources are const and all results are diagnostic-only. Rejection clears the
// entire value; a previous success cannot survive as a new successful sample.
// The member result records capture/read stages separately. A successful AG
// capture alone is NOT successful reading; check HasSuccessfulSampleShape().
//
// The getter returns a LAST-SAMPLED diagnostic, not live proof. That predicate
// is detached structural consistency only, NOT current-owner revalidation.
// Owner publications may advance after sampling; RESET/STOP/Idle that does not
// invoke this path does NOT clear the last sample or invalidate its ticket.
// No machine-lifecycle epoch, wall-clock freshness, pinning, cross-thread
// safety, owner-address-reuse/full-wrap ABA protection or authentication is
// added. A later same-thread user must revalidate through AH against live
// owners. Discard all tickets before owners are destroyed/replaced/restarted.
//
// Mandatory for the ENTIRE call: same thread as all six corresponding live
// owners, no callbacks/reentrancy/concurrent mutation. The diagnostic getter
// is same-thread only; it must NOT be polled by HMI/another thread. No owner
// view or ring-slot pointer is retained; AG's borrowed owner identity remains
// within the 112-byte value. The probe cannot be copied/moved independently.
//
// AVAILABLE/UNAVAILABLE means AA BOUNDARY PROOF availability, not a usable
// machine path. AD count remains its local proof-record count; coverage is
// eight U relation modes. No coordinate/endpoint/event arrays, history replay,
// one common Z/committed interval, Path Queue, Motion completion or B2 claim.
//
// Added storage=120 bytes/align8, in one NCManager member (not stack).
// Observe allocates nothing; no printf/log/output, thread, timer, mutex, wait,
// sleep, HMI/SHM/API/Alarm/Gate/PC/Motion/DC/PDO/NIC/EtherCAT changes. Two bounded
// source rebindings run here (AG Capture and AH Read); this adds NC-thread work.
// Host function-frame checks are NOT a target call-chain/WCET guarantee.

constexpr std::uint16_t NC_PATH_CORE_FRONTIER_READ_PROBE_SCHEMA_V1 = 1U;

enum class NCPathCoreFrontierReadProbeDisposition : std::uint8_t
{
    NOT_SAMPLED = 0U,
    CAPTURE_REJECTED = 1U,
    READ_REJECTED = 2U,
    CAPTURE_READ_STATUS_MISMATCH = 3U,
    SAMPLED_BOUNDARY_AVAILABLE = 4U,
    SAMPLED_BOUNDARY_UNAVAILABLE = 5U
};

struct NCPathCoreFrontierReadProbeRecordV1
{
    using Disposition = NCPathCoreFrontierReadProbeDisposition;
    using TicketStatus = NCPathCoreFrontierTicketStatus;
    NCPathCoreFrontierReadValueV1 value{};
    TicketStatus captureStatus = TicketStatus::EMPTY_TICKET;
    NCPathCoreFrontierReadResult readResult{};
    Disposition disposition = Disposition::NOT_SAMPLED;
    std::uint16_t schemaVersion = NC_PATH_CORE_FRONTIER_READ_PROBE_SCHEMA_V1;
    std::array<std::uint8_t, 2U> reserved{};

    // Detached LAST SAMPLE shape only; never permission/current proof.
    bool HasSuccessfulSampleShape() const noexcept
    {
        if (schemaVersion != NC_PATH_CORE_FRONTIER_READ_PROBE_SCHEMA_V1 ||
            reserved[0U] != 0U || reserved[1U] != 0U ||
            !readResult.IsBound() || captureStatus != readResult.ticketStatus ||
            !value.HasCapturedShape()) return false;
        if (disposition == Disposition::SAMPLED_BOUNDARY_AVAILABLE)
            return captureStatus == TicketStatus::BOUND_CURRENT_BOUNDARY_AVAILABLE &&
            value.frontier.disposition ==
            NCPathCoreProofFrontierDisposition::PROVEN_CURRENT_FRONTIER_BOUNDARY_AVAILABLE;
        return disposition == Disposition::SAMPLED_BOUNDARY_UNAVAILABLE &&
            captureStatus == TicketStatus::BOUND_CURRENT_BOUNDARY_UNAVAILABLE &&
            value.frontier.disposition ==
            NCPathCoreProofFrontierDisposition::PROVEN_CURRENT_FRONTIER_BOUNDARY_UNAVAILABLE;
    }
};

#if defined(_MSC_VER)
#define NC_PATH_CORE_FRONTIER_PROBE_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_CORE_FRONTIER_PROBE_NOINLINE __attribute__((noinline))
#else
#define NC_PATH_CORE_FRONTIER_PROBE_NOINLINE
#endif

class NCPathCoreFrontierReadProbe final
{
public:
    NCPathCoreFrontierReadProbe() noexcept = default;
    NCPathCoreFrontierReadProbe(const NCPathCoreFrontierReadProbe&) = delete;
    NCPathCoreFrontierReadProbe& operator=(const NCPathCoreFrontierReadProbe&) = delete;
    NCPathCoreFrontierReadProbe(NCPathCoreFrontierReadProbe&&) = delete;
    NCPathCoreFrontierReadProbe& operator=(NCPathCoreFrontierReadProbe&&) = delete;

    // Void deliberately: production caller cannot branch on a returned permit.
    NC_PATH_CORE_FRONTIER_PROBE_NOINLINE
        void SampleCurrentFrontierSameThread(const NCPathCoreFrontierOwnersSameThread& owners) noexcept
    {
        using Status = NCPathCoreFrontierTicketStatus;
        using Disposition = NCPathCoreFrontierReadProbeDisposition;
        // Fieldwise reset; no 120-byte temporary or predecessor snapshot.
        m_lastSample.value.Clear();
        m_lastSample.readResult.code = NCPathCoreFrontierReadCode::NOT_CHECKED;
        m_lastSample.readResult.ticketStatus = Status::EMPTY_TICKET;
        m_lastSample.disposition = Disposition::CAPTURE_REJECTED;
        m_lastSample.schemaVersion = NC_PATH_CORE_FRONTIER_READ_PROBE_SCHEMA_V1;
        m_lastSample.reserved[0U] = 0U;
        m_lastSample.reserved[1U] = 0U;
        m_lastSample.captureStatus = CaptureCurrentFrontierTicketSameThread(
            m_lastSample.value.ticket, owners);
        if (m_lastSample.captureStatus != Status::BOUND_CURRENT_BOUNDARY_AVAILABLE &&
            m_lastSample.captureStatus != Status::BOUND_CURRENT_BOUNDARY_UNAVAILABLE)
            return;

        m_lastSample.readResult = ReadCurrentFrontierValueSameThread(
            m_lastSample.value.ticket, owners, m_lastSample.value);
        if (!m_lastSample.readResult.IsBound())
        {
            m_lastSample.value.Clear();
            m_lastSample.disposition = Disposition::READ_REJECTED;
            return;
        }
        // Defensive consistency fence, not a concurrent mutation detector.
        if (m_lastSample.captureStatus != m_lastSample.readResult.ticketStatus)
        {
            m_lastSample.value.Clear();
            m_lastSample.disposition = Disposition::CAPTURE_READ_STATUS_MISMATCH;
            return;
        }
        m_lastSample.disposition =
            m_lastSample.captureStatus == Status::BOUND_CURRENT_BOUNDARY_AVAILABLE ?
            Disposition::SAMPLED_BOUNDARY_AVAILABLE : Disposition::SAMPLED_BOUNDARY_UNAVAILABLE;
    }

    // Borrowed reference to the same-thread probe workspace, NOT an AF ring slot.
    // A later sample overwrites it. No external polling/publication is added.
    const NCPathCoreFrontierReadProbeRecordV1& GetLastSampleSameThread() const noexcept
    {
        return m_lastSample;
    }

private:
    NCPathCoreFrontierReadProbeRecordV1 m_lastSample{};
};

#undef NC_PATH_CORE_FRONTIER_PROBE_NOINLINE

static_assert(sizeof(NCPathCoreFrontierReadProbeDisposition) == 1U, "AI disposition is one byte.");
static_assert(std::is_standard_layout<NCPathCoreFrontierReadProbeRecordV1>::value, "AI record layout.");
static_assert(std::is_trivially_copyable<NCPathCoreFrontierReadProbeRecordV1>::value, "AI diagnostic scalar copy.");
static_assert(sizeof(NCPathCoreFrontierReadProbeRecordV1) == 120U, "AI single sample=120 bytes.");
static_assert(alignof(NCPathCoreFrontierReadProbeRecordV1) == 8U, "AI x64 record alignment.");
static_assert(offsetof(NCPathCoreFrontierReadProbeRecordV1, captureStatus) == 112U, "AI capture offset.");
static_assert(offsetof(NCPathCoreFrontierReadProbeRecordV1, readResult) == 113U, "AI read result offset.");
static_assert(offsetof(NCPathCoreFrontierReadProbeRecordV1, disposition) == 115U, "AI disposition offset.");
static_assert(offsetof(NCPathCoreFrontierReadProbeRecordV1, schemaVersion) == 116U, "AI schema offset.");
static_assert(offsetof(NCPathCoreFrontierReadProbeRecordV1, reserved) == 118U, "AI reserved offset.");
static_assert(sizeof(NCPathCoreFrontierReadProbe) == 120U, "AI probe holds exactly one sample.");
static_assert(alignof(NCPathCoreFrontierReadProbe) == 8U, "AI probe x64 alignment.");
static_assert(std::is_nothrow_default_constructible<NCPathCoreFrontierReadProbe>::value, "AI no-throw construction.");
static_assert(!std::is_copy_constructible<NCPathCoreFrontierReadProbe>::value, "AI probe stays with its owner.");
