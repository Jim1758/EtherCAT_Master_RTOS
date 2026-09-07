#pragma once

#include "NCPathCoreSampleReadContract.h"

// NC-0.2L.2AL / Last-Sample Readback Exact-Value Binding Guard.
//
// AK checked output shape and availability, but did not compare the returned
// value with the AI sample that AJ had read. Another self-consistent ticket /
// frontier with the same availability could therefore pass that local check.
// AL checks the AJ diagnostic envelope against the SAME retained AI sample,
// and requires exact ticket + AF scalar equality for every successful read.
// Padding is ignored; all reserved fields and owner identity are compared.
//
// This is an additional CONSUMER CONSISTENCY check, NOT a source proof. True
// also describes a consistent rejected read with EMPTY output. It is never a
// motion/readiness permit. AJ/AH still perform current-source revalidation;
// this function does not repeat it or authenticate whether AJ actually ran.
// A jointly forged sample/result/output cannot be detected by this contract.
// In INVALID_SAMPLE_RECORD, copied raw diagnostics may contain unknown enums;
// those remain failure provenance, not successful/canonical sample evidence.
//
// Same live manager/sample, same NC thread, and no mutation/reentrancy across
// the ENTIRE AJ call and this check are mandatory. The output remains disjoint
// from manager/owners. No retained pointer, new record/history/member/counter,
// allocation, source snapshot, lifecycle epoch, HMI/SHM/API or PDO work is added.
// RESET/STOP/Idle without another publication does not invalidate a sample.
// Availability is AA-boundary proof availability; count is AD-local and masks
// are eight U relations. No path/geometry/queue/Motion/B2 meaning is added.

#if defined(_MSC_VER)
#define NC_PATH_CORE_READBACK_CHECK_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_CORE_READBACK_CHECK_NOINLINE __attribute__((noinline))
#else
#define NC_PATH_CORE_READBACK_CHECK_NOINLINE
#endif

namespace NCPathCoreReadbackCheckDetail
{
    using Sample = NCPathCoreFrontierReadProbeRecordV1;
    using Code = NCPathCoreSampleReadCode;
    using Status = NCPathCoreFrontierTicketStatus;
    using ReadCode = NCPathCoreFrontierReadCode;

    inline bool SameReadResult(const NCPathCoreFrontierReadResult& left,
        const NCPathCoreFrontierReadResult& right) noexcept
    {
        return left.code == right.code && left.ticketStatus == right.ticketStatus;
    }

    inline bool SameTicket(const NCPathCoreFrontierTicketV1& left,
        const NCPathCoreFrontierTicketV1& right) noexcept
    {
        for (std::size_t index = 0U; index < left.reserved.size(); ++index)
            if (left.reserved[index] != right.reserved[index]) return false;
        return left.owner == right.owner &&
            left.frontierPublicationSequence == right.frontierPublicationSequence &&
            left.sourceAuditPublicationSequence == right.sourceAuditPublicationSequence &&
            left.schemaVersion == right.schemaVersion;
    }

    // Mirrors only AJ's LOCAL, pre-revalidation classification. NOT_CHECKED
    // here is an internal sentinel: valid success-shaped sample needs AH.
    // It never permits a returned AJ NOT_CHECKED result after an actual call.
    inline Code ClassifySampleBeforeCurrentValidation(const Sample& sample) noexcept
    {
        using namespace NCPathCoreSampleReadDetail;
        using Disposition = NCPathCoreFrontierReadProbeDisposition;
        if (sample.schemaVersion != NC_PATH_CORE_FRONTIER_READ_PROBE_SCHEMA_V1 ||
            sample.reserved[0U] != 0U || sample.reserved[1U] != 0U)
            return Code::INVALID_SAMPLE_RECORD;
        switch (sample.disposition)
        {
        case Disposition::NOT_SAMPLED:
            return sample.value.IsEmpty() && sample.captureStatus == Status::EMPTY_TICKET &&
                IsNotChecked(sample.readResult) ? Code::NOT_SAMPLED : Code::INVALID_SAMPLE_RECORD;
        case Disposition::CAPTURE_REJECTED:
            return sample.value.IsEmpty() && IsCaptureFailure(sample.captureStatus) &&
                IsNotChecked(sample.readResult) ? Code::SAMPLE_CAPTURE_REJECTED : Code::INVALID_SAMPLE_RECORD;
        case Disposition::READ_REJECTED:
            return sample.value.IsEmpty() && IsBoundStatus(sample.captureStatus) &&
                sample.readResult.code == ReadCode::TICKET_REJECTED &&
                IsValidationFailure(sample.readResult.ticketStatus) ?
                Code::SAMPLE_READ_REJECTED : Code::INVALID_SAMPLE_RECORD;
        case Disposition::CAPTURE_READ_STATUS_MISMATCH:
            return sample.value.IsEmpty() && IsBoundStatus(sample.captureStatus) &&
                sample.readResult.IsBound() && sample.captureStatus != sample.readResult.ticketStatus ?
                Code::SAMPLE_STATUS_MISMATCH : Code::INVALID_SAMPLE_RECORD;
        case Disposition::SAMPLED_BOUNDARY_AVAILABLE:
        case Disposition::SAMPLED_BOUNDARY_UNAVAILABLE:
            return sample.HasSuccessfulSampleShape() ? Code::NOT_CHECKED : Code::INVALID_SAMPLE_RECORD;
        default: return Code::INVALID_SAMPLE_RECORD;
        }
    }

    inline bool IsConsistentCurrentRejection(const NCPathCoreFrontierReadResult& current,
        Status capturedStatus) noexcept
    {
        using namespace NCPathCoreSampleReadDetail;
        if (current.code == ReadCode::TICKET_REJECTED)
            return IsValidationFailure(current.ticketStatus);
        if (current.code == ReadCode::VALUE_CONTENT_MISMATCH)
            return IsBoundStatus(current.ticketStatus);
        // AJ also defensively rejects an AH success with the other availability.
        // This is diagnostic consistency, not proof that such a call is possible
        // with unmodified implementations and the no-mutation precondition.
        return current.IsBound() && current.ticketStatus != capturedStatus;
    }
}

NC_PATH_CORE_READBACK_CHECK_NOINLINE
inline bool IsLastSampleReadbackConsistentSameThread(
    const NCPathCoreFrontierReadProbeRecordV1& sample,
    const NCPathCoreSampleReadResult& result,
    const NCPathCoreFrontierReadValueV1& output) noexcept
{
    using namespace NCPathCoreReadbackCheckDetail;
    if (result.reserved != 0U || result.sampleDisposition != sample.disposition ||
        result.sampleCaptureStatus != sample.captureStatus ||
        !SameReadResult(result.sampleReadResult, sample.readResult)) return false;

    const Code local = ClassifySampleBeforeCurrentValidation(sample);
    if (local != Code::NOT_CHECKED)
        return result.code == local &&
        NCPathCoreSampleReadDetail::IsNotChecked(result.currentValidationResult) && output.IsEmpty();

    if (!result.IsBound())
        return result.code == Code::CURRENT_VALUE_REJECTED && output.IsEmpty() &&
        IsConsistentCurrentRejection(result.currentValidationResult, sample.captureStatus);

    // AJ already checked current owners. Bind its complete copy-out to that
    // exact AI sample, rather than accepting merely the same availability.
    return SameTicket(output.ticket, sample.value.ticket) &&
        NCPathCoreFrontierReadDetail::SameFrontierScalars(output.frontier, sample.value.frontier);
}

#undef NC_PATH_CORE_READBACK_CHECK_NOINLINE
