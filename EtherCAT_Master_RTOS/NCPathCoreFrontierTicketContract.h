#pragma once

#include "NCPathCoreProofFrontierShadow.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

// NC-0.2L.2AG / Same-Thread Current Frontier Ticket Revalidation Shadow Contract.
//
// AF's detached scalar proof and its supplied-record rebinding deliberately do
// not establish that it is still the newest publication in its owner. The two
// AF slots can also reuse an address. AG separates a captured identity from a
// use-site check against the NEWEST records obtained from the six live owners.
// It accepts no caller-supplied historical record bundle and retains no pointer
// into a ring slot. AF and every predecessor remain unchanged.
//
// A ticket stores only the AF owner identity and exact AF/AE publications. It
// is neither a proof payload nor a pinned lease. IsIssuedShape() is syntax,
// NOT proof. Capture validates the current owners before issuing; validation
// repeats AF's complete current-source rebinding after checking owner and
// publication identity. No historyOffset is accepted by either entry point.
// Both normal AA-boundary AVAILABLE and UNAVAILABLE can be rebound. Every
// capture failure clears the output ticket; invalid/stale/gap/diagnostic input
// never becomes normal AA-boundary unavailability.
//
// Preconditions: all six owners are the corresponding live, trusted producer
// objects in the SAME NC thread. No owner mutation/reentrant producer call may
// occur DURING capture or validation. A successful result describes only that
// call, not a future use after another producer update. The owner pointer is
// compared for identity only, never dereferenced through the ticket. Tickets
// must be discarded before owner destruction, copying/replacement or restart;
// reuse of an address after reconstruction is NOT detected. No cross-thread
// safety, persistence, serialization, full-sequence-wrap ABA protection,
// coherent-forgery authentication or wall-clock freshness is provided.
//
// This is publication freshness, NOT machine lifecycle freshness: a RESET,
// STOP or elapsed time that has not published new observer data does not by
// itself invalidate a ticket. No execution epoch/lifecycle guarantee is added.
// It proves neither a single Z/committed/coverage interval nor path availability,
// path capture/admission, queue ownership, Motion completion, B2 or readiness.
// Eight-U-relation coverage retains its existing meaning; no old payload or
// discarded event order is reconstructed.
//
// CONTRACT-ONLY stage: no observer, history, counter, NCManager member, Observe
// call or runtime consumer is added. NCManager includes this header for target
// parsing/ABI checks only. Ticket=32 bytes on x64; borrowed owner view=48 bytes.
// No allocation, geometry/source snapshot, coordinate/endpoint/event array,
// log, thread, timer, mutex, wait, sleep, HMI/SHM/API/PDO or control changes.

constexpr std::uint16_t NC_PATH_CORE_FRONTIER_TICKET_SCHEMA_V1 = 1U;

enum class NCPathCoreFrontierTicketStatus : std::uint8_t
{
    EMPTY_TICKET = 0U,
    BOUND_CURRENT_BOUNDARY_AVAILABLE = 1U,
    BOUND_CURRENT_BOUNDARY_UNAVAILABLE = 2U,
    INVALID_TICKET = 3U,
    OWNER_MISMATCH = 4U,
    NO_CURRENT_FRONTIER = 5U,
    INVALID_CURRENT_FRONTIER_RECORD = 6U,
    FRONTIER_REPLACED = 7U,
    FRONTIER_SOURCE_UNAVAILABLE = 8U,
    FRONTIER_AUDIT_NOT_COHERENT = 9U,
    FRONTIER_INVALID_SOURCE_AUDIT = 10U,
    FRONTIER_INVALID_PREVIOUS_RECORD = 11U,
    FRONTIER_STALE_SOURCE_AUDIT = 12U,
    FRONTIER_SOURCE_AUDIT_GAP = 13U,
    FRONTIER_INVALID_SOURCE_BINDING = 14U,
    FRONTIER_INVALID_PROJECTION = 15U,
    CURRENT_SOURCES_UNAVAILABLE = 16U,
    CURRENT_SOURCE_REBINDING_FAILED = 17U
};

#if defined(_MSC_VER)
#define NC_PATH_CORE_FRONTIER_TICKET_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_CORE_FRONTIER_TICKET_NOINLINE __attribute__((noinline))
#else
#define NC_PATH_CORE_FRONTIER_TICKET_NOINLINE
#endif

struct NCPathCoreFrontierTicketV1
{
    // Borrowed owner identity, not a ring-record address or an owning pointer.
    const NCPathCoreProofFrontierShadow* owner = nullptr;
    std::uint64_t frontierPublicationSequence = 0ULL;
    std::uint64_t sourceAuditPublicationSequence = 0ULL;
    std::uint16_t schemaVersion = 0U;
    std::array<std::uint8_t, 6U> reserved{};

    bool IsEmpty() const noexcept
    {
        for (std::uint8_t value : reserved) if (value != 0U) return false;
        return owner == nullptr && frontierPublicationSequence == 0ULL &&
            sourceAuditPublicationSequence == 0ULL && schemaVersion == 0U;
    }

    bool IsIssuedShape() const noexcept
    {
        for (std::uint8_t value : reserved) if (value != 0U) return false;
        return owner != nullptr && frontierPublicationSequence != 0ULL &&
            sourceAuditPublicationSequence != 0ULL &&
            schemaVersion == NC_PATH_CORE_FRONTIER_TICKET_SCHEMA_V1;
    }

    void Clear() noexcept
    {
        owner = nullptr;
        frontierPublicationSequence = 0ULL;
        sourceAuditPublicationSequence = 0ULL;
        schemaVersion = 0U;
        for (std::uint8_t& value : reserved) value = 0U;
    }
};

// Ephemeral same-thread view of owner objects, not of individual records.
// Do not store this view or transport it between threads/lifetimes.
struct NCPathCoreFrontierOwnersSameThread
{
    const NCPathCoreProofFrontierShadow& frontier;
    const NCPathCoreCurrentProofScopeShadow& audit;
    const NCPathCoreRunCoverageBoundaryAvailabilityPairRunShadow& run;
    const NCPathCoreRunCoverageBoundaryAvailabilityPairShadow& pair;
    const NCPathCoreRunCoverageBoundaryAvailabilityShadow& transition;
    const NCPathCoreRunCoverageBoundaryShadow& boundary;
};

namespace NCPathCoreFrontierTicketDetail
{
    using Status = NCPathCoreFrontierTicketStatus;
    using Record = NCPathCoreProofFrontierRecordV1;
    using Disposition = NCPathCoreProofFrontierDisposition;

    inline bool IsBoundStatus(Status status) noexcept
    {
        return status == Status::BOUND_CURRENT_BOUNDARY_AVAILABLE ||
            status == Status::BOUND_CURRENT_BOUNDARY_UNAVAILABLE;
    }

    // Only canonical diagnostics are mapped. Malformed diagnostics fail first.
    inline Status DiagnosticStatus(const Record& record) noexcept
    {
        if (!record.IsCanonicalDiagnostic()) return Status::INVALID_CURRENT_FRONTIER_RECORD;
        switch (record.disposition)
        {
        case Disposition::SOURCE_UNAVAILABLE: return Status::FRONTIER_SOURCE_UNAVAILABLE;
        case Disposition::SOURCE_AUDIT_NOT_COHERENT: return Status::FRONTIER_AUDIT_NOT_COHERENT;
        case Disposition::INVALID_SOURCE_AUDIT_RECORD: return Status::FRONTIER_INVALID_SOURCE_AUDIT;
        case Disposition::INVALID_PREVIOUS_FRONTIER_RECORD: return Status::FRONTIER_INVALID_PREVIOUS_RECORD;
        case Disposition::INVALID_STALE_SOURCE_AUDIT: return Status::FRONTIER_STALE_SOURCE_AUDIT;
        case Disposition::INVALID_SOURCE_AUDIT_SEQUENCE_GAP: return Status::FRONTIER_SOURCE_AUDIT_GAP;
        case Disposition::INVALID_CURRENT_SOURCE_BINDING: return Status::FRONTIER_INVALID_SOURCE_BINDING;
        case Disposition::INVALID_PROJECTION: return Status::FRONTIER_INVALID_PROJECTION;
        default: return Status::INVALID_CURRENT_FRONTIER_RECORD;
        }
    }

    // Internal implementation only. Public calls obtain frontier from its
    // live owner themselves, never from a caller-provided record pointer.
    NC_PATH_CORE_FRONTIER_TICKET_NOINLINE
        inline Status RebindNewestSources(const Record& frontier,
            const NCPathCoreFrontierOwnersSameThread& owners) noexcept
    {
        const auto* const audit = owners.audit.GetNewestObservationSameThread();
        const auto* const run = owners.run.GetNewestObservationSameThread();
        const auto* const pair = owners.pair.GetNewestObservationSameThread();
        const auto* const transition = owners.transition.GetNewestObservationSameThread();
        const auto* const boundary = owners.boundary.GetNewestObservationSameThread();
        if (audit == nullptr || run == nullptr || pair == nullptr || transition == nullptr || boundary == nullptr)
            return Status::CURRENT_SOURCES_UNAVAILABLE;
        if (!frontier.IsBoundToCurrentSourcesSameThread(audit, run, pair, transition, boundary))
            return Status::CURRENT_SOURCE_REBINDING_FAILED;
        return frontier.disposition == Disposition::PROVEN_CURRENT_FRONTIER_BOUNDARY_AVAILABLE ?
            Status::BOUND_CURRENT_BOUNDARY_AVAILABLE : Status::BOUND_CURRENT_BOUNDARY_UNAVAILABLE;
    }
}

// Does not allocate or mutate any owner. A rejected capture ALWAYS leaves a
// canonical empty output, including when the caller supplies an older ticket.
NC_PATH_CORE_FRONTIER_TICKET_NOINLINE
inline NCPathCoreFrontierTicketStatus CaptureCurrentFrontierTicketSameThread(
    NCPathCoreFrontierTicketV1& output, const NCPathCoreFrontierOwnersSameThread& owners) noexcept
{
    using namespace NCPathCoreFrontierTicketDetail;
    output.Clear();
    const Record* const latest = owners.frontier.GetNewestObservationSameThread();
    if (latest == nullptr) return Status::NO_CURRENT_FRONTIER;
    if (!latest->IsProvenCurrentProofFrontier()) return DiagnosticStatus(*latest);
    const Status status = RebindNewestSources(*latest, owners);
    if (!IsBoundStatus(status)) return status;
    output.owner = &owners.frontier;
    output.frontierPublicationSequence = latest->publicationSequence;
    output.sourceAuditPublicationSequence = latest->sourceAuditPublicationSequence;
    output.schemaVersion = NC_PATH_CORE_FRONTIER_TICKET_SCHEMA_V1;
    return status;
}

// Read-only check. This does not pin the publication, extend its lifetime,
// clear the caller's ticket or authorize a subsequent control operation.
NC_PATH_CORE_FRONTIER_TICKET_NOINLINE
inline NCPathCoreFrontierTicketStatus ValidateCurrentFrontierTicketSameThread(
    const NCPathCoreFrontierTicketV1& ticket, const NCPathCoreFrontierOwnersSameThread& owners) noexcept
{
    using namespace NCPathCoreFrontierTicketDetail;
    if (ticket.IsEmpty()) return Status::EMPTY_TICKET;
    if (!ticket.IsIssuedShape()) return Status::INVALID_TICKET;
    if (ticket.owner != &owners.frontier) return Status::OWNER_MISMATCH;
    const Record* const latest = owners.frontier.GetNewestObservationSameThread();
    if (latest == nullptr) return Status::NO_CURRENT_FRONTIER;
    if (!latest->IsProvenCurrentProofFrontier()) return DiagnosticStatus(*latest);
    if (ticket.frontierPublicationSequence != latest->publicationSequence ||
        ticket.sourceAuditPublicationSequence != latest->sourceAuditPublicationSequence)
        return Status::FRONTIER_REPLACED;
    return RebindNewestSources(*latest, owners);
}

#undef NC_PATH_CORE_FRONTIER_TICKET_NOINLINE

static_assert(sizeof(NCPathCoreFrontierTicketStatus) == 1U, "AG status is one byte.");
static_assert(sizeof(void*) == 8U, "AG same-thread ticket targets x64 only.");
static_assert(std::is_standard_layout<NCPathCoreFrontierTicketV1>::value, "AG ticket standard layout.");
static_assert(std::is_trivially_copyable<NCPathCoreFrontierTicketV1>::value, "AG ticket scalar copyability.");
static_assert(sizeof(NCPathCoreFrontierTicketV1) == 32U, "AG ticket exactly 32 bytes on x64.");
static_assert(alignof(NCPathCoreFrontierTicketV1) == 8U, "AG ticket alignment.");
static_assert(offsetof(NCPathCoreFrontierTicketV1, frontierPublicationSequence) == 8U, "AG AF publication offset.");
static_assert(offsetof(NCPathCoreFrontierTicketV1, sourceAuditPublicationSequence) == 16U, "AG AE publication offset.");
static_assert(offsetof(NCPathCoreFrontierTicketV1, schemaVersion) == 24U, "AG schema offset.");
static_assert(offsetof(NCPathCoreFrontierTicketV1, reserved) == 26U, "AG reserved offset.");
static_assert(sizeof(NCPathCoreFrontierOwnersSameThread) == 48U, "AG owner view is six x64 references.");
