#pragma once

#include "NCPathCoreFrontierReadProbe.h"

#include <cstddef>
#include <cstdint>
#include <type_traits>

// NC-0.2L.2AJ / NCManager-Owned Last-Sample Revalidated Read Boundary.
//
// AI owns ONE last-sampled diagnostic; its getter/shape predicate intentionally
// does not prove that its saved value is still current. AJ supplies the small
// read-side result/validator used by NCManager's same-thread copy-out method.
// The manager, not its caller, chooses its six private corresponding owners
// and its AI probe. After this validator succeeds, the manager copies only the
// existing 112-byte value to caller-owned output; every rejection clears that
// output. No 120-byte source snapshot, ring-slot reference or owner view is
// returned. There is NO recapture/resample/fallback to a different publication.
//
// This is a read boundary, NOT another observer, proof payload, history, count,
// generation or lifecycle epoch. The result is ephemeral and not retained in
// NCManager. Sample-time capture/read diagnoses and use-time AH revalidation
// are separate. IsBound() requires ALL success fields; the original sample's
// successful readResult by itself is NOT present validity. A structurally
// coherent but modified value is still subject to full AH scalar equality.
// Canonical failed samples are classified, never promoted to availability.
//
// Mandatory for the ENTIRE manager call: same thread as the six live owners,
// no callbacks/reentrancy/concurrent mutation; output disjoint from NCManager
// and its owners. The caller must not poll this method from HMI, SHM/API or a
// different thread. This is an internal native C++ diagnostic entry point,
// NOT an exported API. No scheduler/thread-identity enforcement is added.
// The copied value denotes only this call; later use requires revalidation.
// RESET/STOP/Idle without new publication does not invalidate a prior sample.
// No pinning, queue ownership, owner-reconstruction/full-wrap ABA protection,
// wall-clock freshness, cross-thread safety or authentication is supplied.
//
// Available/unavailable still means AA boundary proof, not a usable path.
// AD count remains a local proof-record count; masks remain eight U relations.
// No coordinate, endpoint, event/segment list, traversal, omitted history,
// single committed/Z interval, Motion completion, readiness or B2 is inferred.
// No existing header, AI sample logic, Gate/PC/Motion, HMI/SHM/API, DC/PDO/NIC
// or 250 us configuration is changed. This stage adds no periodic/runtime
// call site: work occurs only if the new same-thread read method is invoked.

enum class NCPathCoreSampleReadCode : std::uint8_t
{
    NOT_CHECKED = 0U,
    NOT_SAMPLED = 1U,
    SAMPLE_CAPTURE_REJECTED = 2U,
    SAMPLE_READ_REJECTED = 3U,
    SAMPLE_STATUS_MISMATCH = 4U,
    INVALID_SAMPLE_RECORD = 5U,
    CURRENT_VALUE_REJECTED = 6U,
    BOUND_CURRENT_BOUNDARY_AVAILABLE = 7U,
    BOUND_CURRENT_BOUNDARY_UNAVAILABLE = 8U
};

struct NCPathCoreSampleReadResult
{
    using Code = NCPathCoreSampleReadCode;
    using SampleDisposition = NCPathCoreFrontierReadProbeDisposition;
    using TicketStatus = NCPathCoreFrontierTicketStatus;

    Code code = Code::NOT_CHECKED;
    // These are the LAST SAMPLE'S fields, not the outcome of a new capture.
    SampleDisposition sampleDisposition = SampleDisposition::NOT_SAMPLED;
    TicketStatus sampleCaptureStatus = TicketStatus::EMPTY_TICKET;
    NCPathCoreFrontierReadResult sampleReadResult{};
    // NOT_CHECKED means AH use-time validation was NOT called.
    NCPathCoreFrontierReadResult currentValidationResult{};
    std::uint8_t reserved = 0U;

    bool IsBound() const noexcept
    {
        if (reserved != 0U || !sampleReadResult.IsBound() ||
            !currentValidationResult.IsBound() ||
            sampleCaptureStatus != sampleReadResult.ticketStatus ||
            sampleCaptureStatus != currentValidationResult.ticketStatus) return false;
        return (code == Code::BOUND_CURRENT_BOUNDARY_AVAILABLE &&
            sampleDisposition == SampleDisposition::SAMPLED_BOUNDARY_AVAILABLE &&
            sampleCaptureStatus == TicketStatus::BOUND_CURRENT_BOUNDARY_AVAILABLE) ||
            (code == Code::BOUND_CURRENT_BOUNDARY_UNAVAILABLE &&
                sampleDisposition == SampleDisposition::SAMPLED_BOUNDARY_UNAVAILABLE &&
                sampleCaptureStatus == TicketStatus::BOUND_CURRENT_BOUNDARY_UNAVAILABLE);
    }
};

#if defined(_MSC_VER)
#define NC_PATH_CORE_SAMPLE_READ_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_CORE_SAMPLE_READ_NOINLINE __attribute__((noinline))
#else
#define NC_PATH_CORE_SAMPLE_READ_NOINLINE
#endif

namespace NCPathCoreSampleReadDetail
{
    using Status = NCPathCoreFrontierTicketStatus;
    using ReadCode = NCPathCoreFrontierReadCode;
    using ReadResult = NCPathCoreFrontierReadResult;

    inline bool IsNotChecked(const ReadResult& result) noexcept
    {
        return result.code == ReadCode::NOT_CHECKED && result.ticketStatus == Status::EMPTY_TICKET;
    }

    inline bool IsBoundStatus(Status status) noexcept
    {
        return status == Status::BOUND_CURRENT_BOUNDARY_AVAILABLE ||
            status == Status::BOUND_CURRENT_BOUNDARY_UNAVAILABLE;
    }

    // Exhaustive current AG V1 failures; unknown enum values fail closed.
    inline bool IsValidationFailure(Status status) noexcept
    {
        switch (status)
        {
        case Status::EMPTY_TICKET:
        case Status::INVALID_TICKET:
        case Status::OWNER_MISMATCH:
        case Status::NO_CURRENT_FRONTIER:
        case Status::INVALID_CURRENT_FRONTIER_RECORD:
        case Status::FRONTIER_REPLACED:
        case Status::FRONTIER_SOURCE_UNAVAILABLE:
        case Status::FRONTIER_AUDIT_NOT_COHERENT:
        case Status::FRONTIER_INVALID_SOURCE_AUDIT:
        case Status::FRONTIER_INVALID_PREVIOUS_RECORD:
        case Status::FRONTIER_STALE_SOURCE_AUDIT:
        case Status::FRONTIER_SOURCE_AUDIT_GAP:
        case Status::FRONTIER_INVALID_SOURCE_BINDING:
        case Status::FRONTIER_INVALID_PROJECTION:
        case Status::CURRENT_SOURCES_UNAVAILABLE:
        case Status::CURRENT_SOURCE_REBINDING_FAILED: return true;
        default: return false;
        }
    }

    inline bool IsCaptureFailure(Status status) noexcept
    {
        // Capture clears its ticket first and obtains identity from the owner;
        // it cannot fail on an input ticket/owner/publication mismatch.
        return IsValidationFailure(status) && status != Status::EMPTY_TICKET &&
            status != Status::INVALID_TICKET && status != Status::OWNER_MISMATCH &&
            status != Status::FRONTIER_REPLACED;
    }
}

// Read-only. This helper returns no borrowed value pointer. Its probe and
// owners MUST be corresponding trusted objects; the NCManager wrapper fixes
// that correspondence internally. It is not a provenance/authentication test
// for an arbitrary caller-fabricated probe with coherently forged contents.
NC_PATH_CORE_SAMPLE_READ_NOINLINE
inline NCPathCoreSampleReadResult ValidateLastFrontierSampleSameThread(
    const NCPathCoreFrontierReadProbe& probe,
    const NCPathCoreFrontierOwnersSameThread& owners) noexcept
{
    using namespace NCPathCoreSampleReadDetail;
    using Code = NCPathCoreSampleReadCode;
    using Disposition = NCPathCoreFrontierReadProbeDisposition;
    const auto& sample = probe.GetLastSampleSameThread();
    NCPathCoreSampleReadResult result{};
    result.code = Code::INVALID_SAMPLE_RECORD;
    result.sampleDisposition = sample.disposition;
    result.sampleCaptureStatus = sample.captureStatus;
    result.sampleReadResult = sample.readResult;
    if (sample.schemaVersion != NC_PATH_CORE_FRONTIER_READ_PROBE_SCHEMA_V1 ||
        sample.reserved[0U] != 0U || sample.reserved[1U] != 0U) return result;

    switch (sample.disposition)
    {
    case Disposition::NOT_SAMPLED:
        if (sample.value.IsEmpty() && sample.captureStatus == Status::EMPTY_TICKET &&
            IsNotChecked(sample.readResult)) result.code = Code::NOT_SAMPLED;
        return result;
    case Disposition::CAPTURE_REJECTED:
        if (sample.value.IsEmpty() && IsCaptureFailure(sample.captureStatus) &&
            IsNotChecked(sample.readResult)) result.code = Code::SAMPLE_CAPTURE_REJECTED;
        return result;
    case Disposition::READ_REJECTED:
        if (sample.value.IsEmpty() && IsBoundStatus(sample.captureStatus) &&
            sample.readResult.code == ReadCode::TICKET_REJECTED &&
            IsValidationFailure(sample.readResult.ticketStatus)) result.code = Code::SAMPLE_READ_REJECTED;
        return result;
    case Disposition::CAPTURE_READ_STATUS_MISMATCH:
        if (sample.value.IsEmpty() && IsBoundStatus(sample.captureStatus) &&
            sample.readResult.IsBound() && sample.captureStatus != sample.readResult.ticketStatus)
            result.code = Code::SAMPLE_STATUS_MISMATCH;
        return result;
    case Disposition::SAMPLED_BOUNDARY_AVAILABLE:
    case Disposition::SAMPLED_BOUNDARY_UNAVAILABLE:
        if (!sample.HasSuccessfulSampleShape()) return result;
        break;
    default: return result;
    }

    result.currentValidationResult = ValidateCurrentFrontierValueSameThread(sample.value, owners);
    if (!result.currentValidationResult.IsBound() ||
        result.currentValidationResult.ticketStatus != sample.captureStatus)
    {
        result.code = Code::CURRENT_VALUE_REJECTED;
        return result;
    }
    result.code = sample.captureStatus == Status::BOUND_CURRENT_BOUNDARY_AVAILABLE ?
        Code::BOUND_CURRENT_BOUNDARY_AVAILABLE : Code::BOUND_CURRENT_BOUNDARY_UNAVAILABLE;
    return result;
}

#undef NC_PATH_CORE_SAMPLE_READ_NOINLINE

static_assert(sizeof(NCPathCoreSampleReadCode) == 1U, "AJ code is one byte.");
static_assert(std::is_standard_layout<NCPathCoreSampleReadResult>::value, "AJ result layout.");
static_assert(std::is_trivially_copyable<NCPathCoreSampleReadResult>::value, "AJ result scalar copy.");
static_assert(sizeof(NCPathCoreSampleReadResult) == 8U, "AJ ephemeral result=8 bytes.");
static_assert(alignof(NCPathCoreSampleReadResult) == 1U, "AJ result byte alignment.");
static_assert(offsetof(NCPathCoreSampleReadResult, sampleReadResult) == 3U, "AJ sample diagnostic offset.");
static_assert(offsetof(NCPathCoreSampleReadResult, currentValidationResult) == 5U, "AJ current diagnostic offset.");
static_assert(offsetof(NCPathCoreSampleReadResult, reserved) == 7U, "AJ reserved offset.");
