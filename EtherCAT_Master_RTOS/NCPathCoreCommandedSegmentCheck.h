#pragma once

#include "NCPathCoreLinkedCommittedSegmentShadow.h"
#include <cstddef>
#include <cstdint>

// NC-0.2L.2AU / Current Commanded Segment Source Revalidation Contract.
// Check the newest F against newest E and the two retained D records, without
// publishing, retaining, copying or returning any geometry/owner/slot pointer.
// This is independent of the later eight-U-pattern/AA/AF coverage chain.
//
// The sources must be corresponding trusted owners on the SAME NC thread,
// alive and unmodified for this entire call (no reentry). The NCManager entry
// selects its own owners. A returned code is an instantaneous classification,
// NOT a ticket, lease, pinned value, cryptographic proof or owner-lifetime token.
// RESET/STOP without new observations does not by itself change this result.
// There is no fallback to an older formed F, no resampling and no repair.
//
// BOUND means only the current ordinary no-P G00 COMMANDED endpoint descriptor
// agrees with its currently retained sources. It does not recreate discarded
// K.7/producer/queue evidence, verify an opaque fingerprint, or describe the
// interpolation locus, executed/remaining path, Motion completion or B2.
// Coherently altered sources / same-valued foreign owners are not authenticated.
// A producer NOT_FORMED/NOT_PROVEN diagnostic is a rejection, not a proof that
// its entire neutral payload is canonical. Only BOUND is a successful result.

enum class NCPathCoreCommandedSegmentCheck : std::uint8_t
{
    NO_CURRENT_SEGMENT = 0U,
    SEGMENT_NOT_FORMED = 1U,
    INVALID_SEGMENT_RECORD = 2U,
    MISSING_LINK_SOURCE = 3U,
    LINK_NOT_PROVEN = 4U,
    INVALID_LINK_RECORD = 5U,
    MISSING_GEOMETRY_PAIR = 6U,
    INVALID_GEOMETRY_RECORD = 7U,
    NONCONTIGUOUS_GEOMETRY_PAIR = 8U,
    LINK_SOURCE_MISMATCH = 9U,
    LINK_SEAM_MISMATCH = 10U,
    SEGMENT_SOURCE_MISMATCH = 11U,
    SEGMENT_ENDPOINT_MISMATCH = 12U,
    BOUND_CURRENT_COMMANDED_ENDPOINT_SEGMENT = 13U
};

#if defined(_MSC_VER)
#define NC_PATH_CORE_AU_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_CORE_AU_NOINLINE __attribute__((noinline))
#else
#define NC_PATH_CORE_AU_NOINLINE
#endif

namespace NCPathCoreCommandedSegmentCheckDetail
{
    using Geometry = NCPathCoreOrdinaryG00CommittedEndpointPairV1;
    using Link = NCPathCoreCommittedGeometryLinkRecordV1;
    using Segment = NCPathCoreLinkedCommittedSegmentRecordV1;
    using Code = NCPathCoreCommandedSegmentCheck;
    using Pair = NCPathCoreAcceptedInputPairRelation;

    // Recheck D's finite/axis-native endpoint and identity fences. D's own
    // IsAcceptedCommandCommitted checks its schema/classification only.
    NC_PATH_CORE_AU_NOINLINE inline bool IsGeometryValid(const Geometry& g) noexcept
    {
        if (!g.IsAcceptedCommandCommitted() || g.publicationSequence == 0ULL ||
            g.acceptedInputChainGeneration == 0ULL || g.preparedSession == 0ULL ||
            g.preparedEntrySequence == 0ULL || g.dispatchId == 0ULL ||
            g.commitSequence == 0ULL || g.motionExecutionEpoch == 0ULL ||
            g.motionSegmentId == 0ULL || g.queueTailTransactionSequence == 0ULL ||
            g.sourcePC < 0 || g.axisMask == 0U ||
            (g.axisMask & ~NC_PATH_CORE_COMMITTED_GEOMETRY_LINK_COMPARISON_AXIS_MASK) != 0U)
            return false;
        const bool runValid =
            ((g.acceptedInputPairRelation == Pair::FIRST_INPUT ||
                g.acceptedInputPairRelation == Pair::CHAIN_BOUNDARY) &&
                g.acceptedInputRunLength == 1U) ||
            (g.acceptedInputPairRelation == Pair::CONTIGUOUS_PAIR &&
                g.acceptedInputRunLength >= 2U);
        if (!runValid) return false;
        for (std::size_t axis = 0U; axis < NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY; ++axis)
        {
            if (!std::isfinite(g.startMCS[axis]) || !std::isfinite(g.endMCS[axis]))
                return false;
            const std::uint32_t bit = static_cast<std::uint32_t>(1U << axis);
            if ((g.axisMask & bit) == 0U && g.startMCS[axis] != g.endMCS[axis])
                return false;
        }
        return true;
    }

    // Same direct-pair rules as E: no tolerance, modulo or new run policy.
    NC_PATH_CORE_AU_NOINLINE inline bool IsDirectPair(
        const Geometry& previous, const Geometry& current) noexcept
    {
        return current.acceptedInputPairRelation == Pair::CONTIGUOUS_PAIR &&
            current.publicationSequence ==
            NCPathCoreCommittedGeometryLinkDetail::NextNonZero(previous.publicationSequence) &&
            current.acceptedInputChainGeneration == previous.acceptedInputChainGeneration &&
            current.acceptedInputRunLength ==
            NCPathCoreDetail::SaturatingIncrementAcceptedHandoffRunLength(previous.acceptedInputRunLength) &&
            current.preparedSession == previous.preparedSession &&
            current.motionExecutionEpoch == previous.motionExecutionEpoch &&
            static_cast<std::int64_t>(current.sourcePC) ==
            static_cast<std::int64_t>(previous.sourcePC) + 1LL &&
            current.preparedEntrySequence > previous.preparedEntrySequence &&
            current.dispatchId > previous.dispatchId &&
            current.commitSequence > previous.commitSequence &&
            current.motionSegmentId > previous.motionSegmentId &&
            NCPathCoreCommittedGeometryLinkDetail::IsAdvancingNonZeroSerialSequence(
                previous.queueTailTransactionSequence, current.queueTailTransactionSequence);
    }

    NC_PATH_CORE_AU_NOINLINE inline bool IsLinkBound(
        const Link& link, const Geometry& previous, const Geometry& current) noexcept
    {
        return link.acceptedInputChainGeneration == current.acceptedInputChainGeneration &&
            link.previousGeometryPublicationSequence == previous.publicationSequence &&
            link.currentGeometryPublicationSequence == current.publicationSequence &&
            link.previousMotionSegmentId == previous.motionSegmentId &&
            link.currentMotionSegmentId == current.motionSegmentId &&
            link.previousQueueTailTransactionSequence == previous.queueTailTransactionSequence &&
            link.currentQueueTailTransactionSequence == current.queueTailTransactionSequence &&
            link.previousQueueTailCommittedFingerprint == previous.queueTailCommittedFingerprint &&
            link.currentQueueTailBeforeFingerprint == current.queueTailBeforeFingerprint &&
            link.previousAxisMask == previous.axisMask && link.currentAxisMask == current.axisMask &&
            link.acceptedInputPairRelation == current.acceptedInputPairRelation;
    }

    NC_PATH_CORE_AU_NOINLINE inline bool IsSeamBound(
        const Link& link, const Geometry& previous, const Geometry& current) noexcept
    {
        std::uint32_t representationMask = 0U;
        for (std::size_t axis = 0U; axis < NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY; ++axis)
        {
            if (previous.endMCS[axis] != current.startMCS[axis]) return false;
            if (NCPathCoreCommittedGeometryLinkDetail::DoubleObjectBits(previous.endMCS[axis]) !=
                NCPathCoreCommittedGeometryLinkDetail::DoubleObjectBits(current.startMCS[axis]))
                representationMask |= static_cast<std::uint32_t>(1U << axis);
        }
        // E admits equal numeric seams with different representations (+0/-0).
        // Preserve its reported distinction instead of rejecting that case.
        return representationMask == link.representationDifferenceAxisMask;
    }

    NC_PATH_CORE_AU_NOINLINE inline bool IsSegmentBound(
        const Segment& segment, const Link& link,
        const Geometry& previous, const Geometry& current) noexcept
    {
        return segment.linkPublicationSequence == link.publicationSequence &&
            segment.acceptedInputChainGeneration == current.acceptedInputChainGeneration &&
            segment.previousGeometryPublicationSequence == previous.publicationSequence &&
            segment.currentGeometryPublicationSequence == current.publicationSequence &&
            segment.previousMotionSegmentId == previous.motionSegmentId &&
            segment.currentMotionSegmentId == current.motionSegmentId &&
            segment.previousAxisMask == previous.axisMask && segment.currentAxisMask == current.axisMask &&
            segment.acceptedInputRunLength == current.acceptedInputRunLength &&
            segment.sourcePC == current.sourcePC && segment.sourceLineNumber == current.sourceLineNumber;
    }

    NC_PATH_CORE_AU_NOINLINE inline bool AreSegmentEndpointsBound(
        const Segment& segment, const Geometry& current) noexcept
    {
        for (std::size_t axis = 0U; axis < NC_PATH_CORE_COMMITTED_GEOMETRY_AXIS_CAPACITY; ++axis)
        {
            // Unlike the adjacent numeric seam, F copies the current D image:
            // bind the representations too. No tolerance or rotary transform.
            if (NCPathCoreCommittedGeometryLinkDetail::DoubleObjectBits(segment.startMCS[axis]) !=
                NCPathCoreCommittedGeometryLinkDetail::DoubleObjectBits(current.startMCS[axis]) ||
                NCPathCoreCommittedGeometryLinkDetail::DoubleObjectBits(segment.endMCS[axis]) !=
                NCPathCoreCommittedGeometryLinkDetail::DoubleObjectBits(current.endMCS[axis]))
                return false;
        }
        return true;
    }

    // Record pointers are borrowed only for this synchronous evaluation. This
    // helper alone cannot establish "newest"; use the owner-based entry below.
    NC_PATH_CORE_AU_NOINLINE inline Code CheckBorrowedRecords(
        const Segment* segment, const Link* link,
        const Geometry* previous, const Geometry* current) noexcept
    {
        if (segment == nullptr) return Code::NO_CURRENT_SEGMENT;
        if (segment->schemaVersion != NC_PATH_CORE_LINKED_COMMITTED_SEGMENT_SCHEMA_V1 ||
            segment->publicationSequence == 0ULL) return Code::INVALID_SEGMENT_RECORD;
        if (!segment->IsFormedLinkedCommandedEndpointSegment())
            return segment->disposition == NCPathCoreLinkedCommittedSegmentDisposition::NOT_FORMED_LINK_NOT_PROVEN
            ? Code::SEGMENT_NOT_FORMED : Code::INVALID_SEGMENT_RECORD;
        if (link == nullptr) return Code::MISSING_LINK_SOURCE;
        if (link->schemaVersion != NC_PATH_CORE_COMMITTED_GEOMETRY_LINK_SCHEMA_V1 ||
            link->publicationSequence == 0ULL || link->reserved[0U] != 0U || link->reserved[1U] != 0U)
            return Code::INVALID_LINK_RECORD;
        if (!link->IsProvenContiguousCommandedTailLink()) return Code::LINK_NOT_PROVEN;
        if (previous == nullptr || current == nullptr) return Code::MISSING_GEOMETRY_PAIR;
        if (!IsGeometryValid(*previous) || !IsGeometryValid(*current)) return Code::INVALID_GEOMETRY_RECORD;
        if (!IsDirectPair(*previous, *current)) return Code::NONCONTIGUOUS_GEOMETRY_PAIR;
        if (!IsLinkBound(*link, *previous, *current)) return Code::LINK_SOURCE_MISMATCH;
        if (!IsSeamBound(*link, *previous, *current)) return Code::LINK_SEAM_MISMATCH;
        if (!IsSegmentBound(*segment, *link, *previous, *current)) return Code::SEGMENT_SOURCE_MISMATCH;
        if (!AreSegmentEndpointsBound(*segment, *current)) return Code::SEGMENT_ENDPOINT_MISMATCH;
        return Code::BOUND_CURRENT_COMMANDED_ENDPOINT_SEGMENT;
    }
}

NC_PATH_CORE_AU_NOINLINE inline NCPathCoreCommandedSegmentCheck
CheckCurrentCommandedSegmentSameThread(
    const NCPathCoreLinkedCommittedSegmentShadow& segmentOwner,
    const NCPathCoreCommittedGeometryLinkShadow& linkOwner,
    const NCPathCoreCommittedGeometryShadow& geometryOwner) noexcept
{
    // Always select newest F/E and D offsets 1/0; never search old successes.
    return NCPathCoreCommandedSegmentCheckDetail::CheckBorrowedRecords(
        segmentOwner.GetNewestObservationSameThread(),
        linkOwner.GetNewestObservationSameThread(),
        geometryOwner.GetNewestObservationSameThread(1U),
        geometryOwner.GetNewestObservationSameThread(0U));
}

#undef NC_PATH_CORE_AU_NOINLINE
static_assert(sizeof(NCPathCoreCommandedSegmentCheck) == 1U, "AU result is one byte.");
static_assert(alignof(NCPathCoreCommandedSegmentCheck) == 1U, "AU result byte alignment.");
