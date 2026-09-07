#pragma once

#include "NCPathCoreFrontierTicketContract.h"

#include <cstddef>
#include <cstdint>
#include <type_traits>

// NC-0.2L.2AH / Same-Thread Ticket-Bound Frontier Value Read Shadow Contract.
//
// AG answers whether a ticket is still bound to the newest six live owners;
// it does not return the corresponding scalar value. AH joins that check and
// the value read in ONE non-reentrant same-thread call, without exposing an
// observer ring-slot pointer. It copies only AF's existing 80-byte scalar
// certificate, together with AG's 32-byte identity. No predecessor payload,
// geometry, endpoint, event list or omitted history is reconstructed.
//
// Read validates the requested ticket against CURRENT owners before copying.
// Every rejected read clears BOTH output parts, including when the ticket is
// output.ticket itself (supported in-place reread). The 32-byte input identity
// is saved before clearing; no AF/AE/AD/AC/AB/AA source snapshot is built on the
// stack. A read never silently recaptures a newer ticket after replacement.
// Validate rechecks an existing value's ticket against CURRENT owners and
// compares EVERY AF scalar field with the current AF record. Thus a detached
// value that still has valid shape, but different content, is not accepted.
// No memcmp/padding equality is used. Validation does not mutate its input.
//
// HasCapturedShape() is only a detached structural check, NOT current proof.
// A copied value remains readable after ring overwrite, but becomes historical;
// it does not pin a slot, extend validity or confer an execution permission.
// Successful results describe ONLY this call. A later use after publication
// advance must revalidate, and owner destruction/replacement/restart requires
// discarding the value and ticket. AG's same-thread/live-corresponding-owner/
// no-reentrancy preconditions and full-wrap ABA/owner-address-reuse/coherent-
// forgery limitations remain. This is NOT a cross-thread atomic snapshot or
// a thread-safe reader. RESET/STOP without a new publication does not by itself
// invalidate the value; no lifecycle epoch or wall-clock guarantee is added.
//
// AVAILABLE/UNAVAILABLE refer only to AA boundary-proof availability. A valid
// unavailable value is distinct from rejection. sourceConsecutivePairCount
// still belongs to AD's LOCAL proof run, not an AH count or a path length.
// Eight-U-relation coverage is not axes, geometry, Motion completion or B2.
// No common Z/committed/coverage interval, discarded reason/order, path queue
// admission, planner readiness or motion eligibility can be inferred.
//
// CONTRACT-ONLY: no observer, history, counter, NCManager member, production
// call or consumer. Existing Path Core headers and all .cpp files stay unchanged.
// No allocation, log, callback, thread, timer, mutex, wait, sleep, HMI/SHM/API,
// PDO/control integration. Value=112 bytes, result=2 bytes on the x64 target.

#if defined(_MSC_VER)
#define NC_PATH_CORE_FRONTIER_READ_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_CORE_FRONTIER_READ_NOINLINE __attribute__((noinline))
#else
#define NC_PATH_CORE_FRONTIER_READ_NOINLINE
#endif

enum class NCPathCoreFrontierReadCode : std::uint8_t
{
    NOT_CHECKED = 0U,
    EMPTY_VALUE = 1U,
    INVALID_VALUE = 2U,
    TICKET_REJECTED = 3U,
    VALUE_CONTENT_MISMATCH = 4U,
    BOUND_CURRENT_BOUNDARY_AVAILABLE = 5U,
    BOUND_CURRENT_BOUNDARY_UNAVAILABLE = 6U
};

struct NCPathCoreFrontierReadResult
{
    using Code = NCPathCoreFrontierReadCode;
    using TicketStatus = NCPathCoreFrontierTicketStatus;
    Code code = Code::NOT_CHECKED;
    // Meaningful only for TICKET_REJECTED, VALUE_CONTENT_MISMATCH and the two
    // bound codes. For local empty/invalid/not-checked it is EMPTY_TICKET and
    // AG was NOT called; do not reinterpret that placeholder as a diagnosis.
    // ticketStatus alone is NEVER success: VALUE_CONTENT_MISMATCH can carry a
    // bound AG status while AH rejects the value. Check result.IsBound().
    TicketStatus ticketStatus = TicketStatus::EMPTY_TICKET;

    bool IsBound() const noexcept
    {
        return (code == Code::BOUND_CURRENT_BOUNDARY_AVAILABLE &&
            ticketStatus == TicketStatus::BOUND_CURRENT_BOUNDARY_AVAILABLE) ||
            (code == Code::BOUND_CURRENT_BOUNDARY_UNAVAILABLE &&
                ticketStatus == TicketStatus::BOUND_CURRENT_BOUNDARY_UNAVAILABLE);
    }
};

namespace NCPathCoreFrontierReadDetail
{
    using Frontier = NCPathCoreProofFrontierRecordV1;
    using TicketStatus = NCPathCoreFrontierTicketStatus;
    using Code = NCPathCoreFrontierReadCode;
    using Result = NCPathCoreFrontierReadResult;

    inline bool IsEmptyFrontier(const Frontier& record) noexcept
    {
        for (std::uint8_t value : record.reserved) if (value != 0U) return false;
        return record.publicationSequence == 0ULL && record.sourceAuditPublicationSequence == 0ULL &&
            record.sourcePublicationSequence == 0ULL && record.sourceRunGeneration == 0ULL &&
            record.firstPairPublicationSequence == 0ULL && record.currentPairPublicationSequence == 0ULL &&
            record.currentTransitionPublicationSequence == 0ULL && record.currentBoundaryPublicationSequence == 0ULL &&
            record.sourceConsecutivePairCount == 0U && record.schemaVersion == 0U &&
            record.disposition == Frontier::Disposition::EMPTY && record.scope == Frontier::Scope::NONE &&
            record.currentBoundaryAvailability == Frontier::Availability::UNKNOWN && record.currentCoverageMask == 0U &&
            record.bindingMask == 0U && record.sourceAuditDisposition == 0U;
    }

    inline void ClearFrontier(Frontier& record) noexcept
    {
        record.publicationSequence = 0ULL;
        record.sourceAuditPublicationSequence = 0ULL;
        record.sourcePublicationSequence = 0ULL;
        record.sourceRunGeneration = 0ULL;
        record.firstPairPublicationSequence = 0ULL;
        record.currentPairPublicationSequence = 0ULL;
        record.currentTransitionPublicationSequence = 0ULL;
        record.currentBoundaryPublicationSequence = 0ULL;
        record.sourceConsecutivePairCount = 0U;
        record.schemaVersion = 0U;
        record.disposition = Frontier::Disposition::EMPTY;
        record.scope = Frontier::Scope::NONE;
        record.currentBoundaryAvailability = Frontier::Availability::UNKNOWN;
        record.currentCoverageMask = 0U;
        record.bindingMask = 0U;
        record.sourceAuditDisposition = 0U;
        for (std::uint8_t& value : record.reserved) value = 0U;
    }

    inline bool SameFrontierScalars(const Frontier& left, const Frontier& right) noexcept
    {
        for (std::size_t index = 0U; index < left.reserved.size(); ++index)
            if (left.reserved[index] != right.reserved[index]) return false;
        return left.publicationSequence == right.publicationSequence &&
            left.sourceAuditPublicationSequence == right.sourceAuditPublicationSequence &&
            left.sourcePublicationSequence == right.sourcePublicationSequence &&
            left.sourceRunGeneration == right.sourceRunGeneration &&
            left.firstPairPublicationSequence == right.firstPairPublicationSequence &&
            left.currentPairPublicationSequence == right.currentPairPublicationSequence &&
            left.currentTransitionPublicationSequence == right.currentTransitionPublicationSequence &&
            left.currentBoundaryPublicationSequence == right.currentBoundaryPublicationSequence &&
            left.sourceConsecutivePairCount == right.sourceConsecutivePairCount &&
            left.schemaVersion == right.schemaVersion && left.disposition == right.disposition && left.scope == right.scope &&
            left.currentBoundaryAvailability == right.currentBoundaryAvailability &&
            left.currentCoverageMask == right.currentCoverageMask && left.bindingMask == right.bindingMask &&
            left.sourceAuditDisposition == right.sourceAuditDisposition;
    }

    inline bool IsBoundTicketStatus(TicketStatus status) noexcept
    {
        return status == TicketStatus::BOUND_CURRENT_BOUNDARY_AVAILABLE ||
            status == TicketStatus::BOUND_CURRENT_BOUNDARY_UNAVAILABLE;
    }

    inline Result BoundResult(TicketStatus status) noexcept
    {
        if (status == TicketStatus::BOUND_CURRENT_BOUNDARY_AVAILABLE)
            return { Code::BOUND_CURRENT_BOUNDARY_AVAILABLE, status };
        if (status == TicketStatus::BOUND_CURRENT_BOUNDARY_UNAVAILABLE)
            return { Code::BOUND_CURRENT_BOUNDARY_UNAVAILABLE, status };
        return { Code::TICKET_REJECTED, status };
    }
}

struct NCPathCoreFrontierReadValueV1
{
    // The nested V1 ticket/frontier schemas are the version contract. This
    // envelope introduces no additional publication, generation or counter.
    NCPathCoreFrontierTicketV1 ticket{};
    NCPathCoreProofFrontierRecordV1 frontier{};

    bool IsEmpty() const noexcept
    {
        return ticket.IsEmpty() && NCPathCoreFrontierReadDetail::IsEmptyFrontier(frontier);
    }

    NC_PATH_CORE_FRONTIER_READ_NOINLINE
        bool HasCapturedShape() const noexcept
    {
        return ticket.IsIssuedShape() && frontier.IsProvenCurrentProofFrontier() &&
            ticket.frontierPublicationSequence == frontier.publicationSequence &&
            ticket.sourceAuditPublicationSequence == frontier.sourceAuditPublicationSequence;
    }

    void Clear() noexcept
    {
        ticket.Clear();
        NCPathCoreFrontierReadDetail::ClearFrontier(frontier);
    }
};

// Caller-owned output, disjoint from all owners. Aliasing requestedTicket with
// output.ticket is explicitly supported, for either a successful or failed
// reread. The only local payload copy is the 32-byte requested identity.
NC_PATH_CORE_FRONTIER_READ_NOINLINE
inline NCPathCoreFrontierReadResult ReadCurrentFrontierValueSameThread(
    const NCPathCoreFrontierTicketV1& requestedTicket,
    const NCPathCoreFrontierOwnersSameThread& owners, NCPathCoreFrontierReadValueV1& output) noexcept
{
    using namespace NCPathCoreFrontierReadDetail;
    const NCPathCoreFrontierTicketV1 requested = requestedTicket;
    output.Clear();
    const TicketStatus status = ValidateCurrentFrontierTicketSameThread(requested, owners);
    if (!IsBoundTicketStatus(status)) return { Code::TICKET_REJECTED, status };
    // Same-thread, no mutation/reentrancy for this ENTIRE call is mandatory.
    // The null/identity checks below do not make concurrent reads well-defined.
    const Frontier* const current = owners.frontier.GetNewestObservationSameThread();
    if (current == nullptr) return { Code::TICKET_REJECTED, TicketStatus::NO_CURRENT_FRONTIER };
    if (current->publicationSequence != requested.frontierPublicationSequence ||
        current->sourceAuditPublicationSequence != requested.sourceAuditPublicationSequence)
        return { Code::TICKET_REJECTED, TicketStatus::FRONTIER_REPLACED };
    output.ticket = requested;
    output.frontier = *current; // Only the existing 80-byte AF scalar certificate.
    return BoundResult(status);
}

// A value can remain structurally coherent after corruption or ring advance.
// Accept only exact scalar equality AND current six-owner binding. No source
// can be supplied from caller-retained historical record payloads.
NC_PATH_CORE_FRONTIER_READ_NOINLINE
inline NCPathCoreFrontierReadResult ValidateCurrentFrontierValueSameThread(
    const NCPathCoreFrontierReadValueV1& value, const NCPathCoreFrontierOwnersSameThread& owners) noexcept
{
    using namespace NCPathCoreFrontierReadDetail;
    if (value.IsEmpty()) return { Code::EMPTY_VALUE, TicketStatus::EMPTY_TICKET };
    if (!value.HasCapturedShape()) return { Code::INVALID_VALUE, TicketStatus::EMPTY_TICKET };
    const TicketStatus status = ValidateCurrentFrontierTicketSameThread(value.ticket, owners);
    if (!IsBoundTicketStatus(status)) return { Code::TICKET_REJECTED, status };
    const Frontier* const current = owners.frontier.GetNewestObservationSameThread();
    if (current == nullptr) return { Code::TICKET_REJECTED, TicketStatus::NO_CURRENT_FRONTIER };
    if (!SameFrontierScalars(value.frontier, *current)) return { Code::VALUE_CONTENT_MISMATCH, status };
    return BoundResult(status);
}

#undef NC_PATH_CORE_FRONTIER_READ_NOINLINE

static_assert(sizeof(NCPathCoreFrontierReadCode) == 1U, "AH read code is one byte.");
static_assert(std::is_standard_layout<NCPathCoreFrontierReadResult>::value, "AH result standard layout.");
static_assert(sizeof(NCPathCoreFrontierReadResult) == 2U, "AH code and AG diagnostic are two bytes.");
static_assert(std::is_standard_layout<NCPathCoreFrontierReadValueV1>::value, "AH value standard layout.");
static_assert(std::is_trivially_copyable<NCPathCoreFrontierReadValueV1>::value, "AH scalar value copyability.");
static_assert(sizeof(NCPathCoreFrontierReadValueV1) == 112U, "AH value is a 32-byte ticket plus 80-byte AF.");
static_assert(alignof(NCPathCoreFrontierReadValueV1) == 8U, "AH x64 value alignment.");
static_assert(offsetof(NCPathCoreFrontierReadValueV1, ticket) == 0U, "AH ticket offset.");
static_assert(offsetof(NCPathCoreFrontierReadValueV1, frontier) == 32U, "AH scalar certificate offset.");
