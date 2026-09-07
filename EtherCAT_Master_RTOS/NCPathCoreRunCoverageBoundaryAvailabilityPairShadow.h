#pragma once

#include "NCPathCoreRunCoverageBoundaryAvailabilityShadow.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

// NC-0.2L.2AC / Run Pattern Coverage Transition Pair Continuity Run
// Observed-Head Coverage Boundary Availability Transition Pair Shadow.
//
// Describes one compatible pair of adjacent, individually proven AB
// availability relations. U means a canonical unavailable AA certificate;
// A means an available AA observed-head coverage-boundary certificate.
// The eight relations describe UAA, UAU, AAA, AAU, AUA, AUU, UUA and UUU.
// An unproven AB observation is never an ordinary unavailable AA state.
// No elapsed duration, frequency, physical event order, path availability,
// geometric loss or Motion completion is inferred from these certificates.
//
// Admission checks both AB self-proofs in place, direct AB publication,
// shared AA identity and availability, and the supplied current AA edge.
// Retention rechecks the same first-Y mask, all Z/source heads, both counts,
// eight direct Z/Y/X/W/V/U/T/S publications and shared masks/relations.
// A missing Z anchor resets only that lower anchor; it cannot prove a shared
// Z interval. U -> A requires observed START and A -> A requires direct EXT.
// A -> U cannot report HEAD_NOT_OBSERVED. Invalid, stale and gapped proof
// produce no pair and cannot masquerade as normal loss or unavailability.
//
// The retained current AB is bound exhaustively to the next supplied
// previous AB by semantic field equality, including its complete AA/Z
// payload. Padding is not compared. On first attachment, the shared AA
// cache supplied by previous AB can be checked for compatibility with the
// current AB edge; current AB has discarded its earlier full AA payload.
// Byte identity with that discarded AA, including its unavailable reason,
// is not proven. A coherent changed shared neutral reason can therefore be
// compatible on first attachment. Once retained, that reason is fully bound.
// The previous AB is discarded after admission except its identity/relation;
// record self-proof cannot repeat its omitted AA fences or authenticate it.
// Trusted same-thread producers are required. Coherent forged history,
// omitted earlier fields, full-wrap identity reuse and wall-clock freshness
// are not authenticated. This is one local pair descriptor, not an event
// list, retained total ordering, cumulative counter or stable milestone.
//
// Newest AC proof/canonical diagnostic is checked before older-slot overwrite.
// Rejection clears proof payload and retains only current AB publication as
// a diagnostic anchor; null clears it. A later complete adjacent pair may
// recertify without filling in missing transitions. The older slot is not
// a recurrence source. Exactly two 216-byte scalar records in a 448-byte
// heap-owned NCManager observer; direct source lvalue-to-slot copy, no source
// stack snapshot or Observe-time allocation. No coordinates, displacement,
// endpoint arrays, nodes, segment/event list or reverse traversal data.
// Shadow-only: no Path/Motion Queue, planner, STARTED/DONE proof, actual
// position history, B2 breadcrumb or execution. No control consumer, log,
// thread, timer, mutex, wait or sleep. Predecessors and control paths,
// including M00 FIX1, L.2I FIX1 and Priority-64 PDO, remain unchanged.

constexpr std::size_t NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_HISTORY_CAPACITY = 2U;
constexpr std::uint16_t NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_SCHEMA_V1 = 1U;

enum class NCPathCoreRunCoverageBoundaryAvailabilityPairDisposition : std::uint8_t
{
    EMPTY = 0U,
    NOT_APPLICABLE_CURRENT_TRANSITION_UNAVAILABLE = 1U,
    NOT_APPLICABLE_PREVIOUS_TRANSITION_UNAVAILABLE = 2U,
    SOURCE_REPORTED_CURRENT_TRANSITION_INVALID = 3U,
    SOURCE_REPORTED_PREVIOUS_TRANSITION_INVALID = 4U,
    INVALID_CURRENT_TRANSITION_RECORD = 5U,
    INVALID_PREVIOUS_TRANSITION_RECORD = 6U,
    INVALID_OBSERVER_SOURCE_ADVANCE = 7U,
    INVALID_PREVIOUS_PAIR_RECORD = 8U,
    INVALID_PREVIOUS_SOURCE_BINDING = 9U,
    INVALID_TRANSITION_ADVANCE = 10U,
    INVALID_SHARED_BOUNDARY_BINDING = 11U,
    INVALID_AVAILABILITY_STATE_BINDING = 12U,
    INVALID_BOUNDARY_ADVANCE = 13U,
    INVALID_SOURCE_RUN_ADVANCE = 14U,
    INVALID_SOURCE_RUN_BINDING = 15U,
    INVALID_SHARED_RUN_BINDING = 16U,
    INVALID_RUN_COUNT_OVERFLOW = 17U,
    INVALID_AVAILABILITY_TRANSITION = 18U,
    PROVEN_AVAILABILITY_TRANSITION_PAIR = 19U
};

enum class NCPathCoreRunCoverageBoundaryAvailabilityPairRelation : std::uint8_t
{
    NONE = 0U,
    GAIN_THEN_RETAIN = 1U,
    GAIN_THEN_LOSS = 2U,
    RETAIN_THEN_RETAIN = 3U,
    RETAIN_THEN_LOSS = 4U,
    LOSS_THEN_GAIN = 5U,
    LOSS_THEN_UNAVAILABLE = 6U,
    UNAVAILABLE_THEN_GAIN = 7U,
    UNAVAILABLE_THEN_UNAVAILABLE = 8U
};

#if defined(_MSC_VER)
#define NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_NOINLINE __declspec(noinline)
#elif defined(__GNUC__) || defined(__clang__)
#define NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_NOINLINE __attribute__((noinline))
#else
#define NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_NOINLINE
#endif

namespace NCPathCoreRunCoverageBoundaryAvailabilityPairDetail
{
    using TransitionRecord = NCPathCoreRunCoverageBoundaryAvailabilityRecordV1;
    using TransitionDisposition = TransitionRecord::Disposition;
    using TransitionRelation = TransitionRecord::Relation;
    using Availability = TransitionRecord::Availability;
    using UnavailableReason = TransitionRecord::UnavailableReason;
    using BoundaryRecord = TransitionRecord::BoundaryRecord;
    using BoundaryDisposition = BoundaryRecord::Disposition;
    using RunRecord = BoundaryRecord::SourceRecord;
    using Relation = NCPathCoreRunCoverageBoundaryAvailabilityPairRelation;
    using NCPathCoreRunCoverageBoundaryAvailabilityDetail::NextNonZeroSequence;

    inline Availability EndingAvailability(TransitionRelation relation) noexcept
    {
        switch (relation)
        {
        case TransitionRelation::BECAME_AVAILABLE_AT_OBSERVED_HEAD:
        case TransitionRelation::RETAINED_BY_DIRECT_BOUNDARY_EXTENSION: return Availability::AVAILABLE;
        case TransitionRelation::BECAME_UNAVAILABLE:
        case TransitionRelation::REMAINED_UNAVAILABLE: return Availability::UNAVAILABLE;
        default: return Availability::UNKNOWN;
        }
    }

    inline Relation ClassifyPair(TransitionRelation previous, TransitionRelation current) noexcept
    {
        switch (previous)
        {
        case TransitionRelation::BECAME_AVAILABLE_AT_OBSERVED_HEAD:
            if (current == TransitionRelation::RETAINED_BY_DIRECT_BOUNDARY_EXTENSION) return Relation::GAIN_THEN_RETAIN;
            if (current == TransitionRelation::BECAME_UNAVAILABLE) return Relation::GAIN_THEN_LOSS;
            break;
        case TransitionRelation::RETAINED_BY_DIRECT_BOUNDARY_EXTENSION:
            if (current == TransitionRelation::RETAINED_BY_DIRECT_BOUNDARY_EXTENSION) return Relation::RETAIN_THEN_RETAIN;
            if (current == TransitionRelation::BECAME_UNAVAILABLE) return Relation::RETAIN_THEN_LOSS;
            break;
        case TransitionRelation::BECAME_UNAVAILABLE:
            if (current == TransitionRelation::BECAME_AVAILABLE_AT_OBSERVED_HEAD) return Relation::LOSS_THEN_GAIN;
            if (current == TransitionRelation::REMAINED_UNAVAILABLE) return Relation::LOSS_THEN_UNAVAILABLE;
            break;
        case TransitionRelation::REMAINED_UNAVAILABLE:
            if (current == TransitionRelation::BECAME_AVAILABLE_AT_OBSERVED_HEAD) return Relation::UNAVAILABLE_THEN_GAIN;
            if (current == TransitionRelation::REMAINED_UNAVAILABLE) return Relation::UNAVAILABLE_THEN_UNAVAILABLE;
            break;
        default: break;
        }
        return Relation::NONE;
    }

    // Excludes only the AA publication diagnostic anchor.
    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_NOINLINE
        inline bool IsZeroBoundaryPayload(const BoundaryRecord& boundary) noexcept
    {
        return boundary.schemaVersion == 0U && boundary.disposition == BoundaryDisposition::EMPTY &&
            boundary.currentRun.publicationSequence == 0ULL &&
            NCPathCoreRunCoverageBoundaryAvailabilityDetail::IsZeroRunPayload(boundary.currentRun) &&
            boundary.firstYCoverageMask == 0U && boundary.gainedSinceFirstYMask == 0U &&
            boundary.fullCoverageRelation == BoundaryRecord::FullCoverageRelation::NONE &&
            boundary.reserved[0U] == 0U && boundary.reserved[1U] == 0U;
    }

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_NOINLINE
        inline bool IsCanonicalNonProvenTransition(const TransitionRecord& source) noexcept
    {
        const bool known = source.disposition >= TransitionDisposition::NOT_APPLICABLE_CURRENT_BOUNDARY_UNAVAILABLE &&
            source.disposition <= TransitionDisposition::INVALID_AVAILABILITY_TRANSITION;
        const bool identity = source.disposition == TransitionDisposition::NOT_APPLICABLE_CURRENT_BOUNDARY_UNAVAILABLE
            ? source.currentBoundary.publicationSequence == 0ULL
            : source.currentBoundary.publicationSequence != 0ULL ||
            source.disposition == TransitionDisposition::INVALID_CURRENT_BOUNDARY_RECORD ||
            source.disposition == TransitionDisposition::INVALID_OBSERVER_SOURCE_ADVANCE ||
            source.disposition == TransitionDisposition::INVALID_PREVIOUS_AVAILABILITY_RECORD;
        return known && identity && source.publicationSequence != 0ULL &&
            source.schemaVersion == NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_SCHEMA_V1 &&
            source.previousBoundaryPublicationSequence == 0ULL && source.relation == TransitionRelation::NONE &&
            source.previousAvailability == Availability::UNKNOWN && source.currentAvailability == Availability::UNKNOWN &&
            source.currentUnavailableReason == UnavailableReason::NONE && source.reserved == 0U &&
            IsZeroBoundaryPayload(source.currentBoundary);
    }

    inline bool IsNeutralTransitionDisposition(TransitionDisposition disposition) noexcept
    {
        return disposition == TransitionDisposition::NOT_APPLICABLE_CURRENT_BOUNDARY_UNAVAILABLE ||
            disposition == TransitionDisposition::NOT_APPLICABLE_PREVIOUS_BOUNDARY_UNAVAILABLE;
    }
}

struct NCPathCoreRunCoverageBoundaryAvailabilityPairRecordV1
{
    using Disposition = NCPathCoreRunCoverageBoundaryAvailabilityPairDisposition;
    using Relation = NCPathCoreRunCoverageBoundaryAvailabilityPairRelation;
    using TransitionRecord = NCPathCoreRunCoverageBoundaryAvailabilityPairDetail::TransitionRecord;
    using TransitionRelation = TransitionRecord::Relation;
    using BoundaryRecord = TransitionRecord::BoundaryRecord;
    using RunRecord = BoundaryRecord::SourceRecord;

    std::uint64_t publicationSequence = 0ULL;
    std::uint64_t previousTransitionPublicationSequence = 0ULL;
    TransitionRecord currentTransition{};
    std::uint16_t schemaVersion = 0U;
    Disposition disposition = Disposition::EMPTY;
    TransitionRelation previousRelation = TransitionRelation::NONE;
    Relation relation = Relation::NONE;
    std::array<std::uint8_t, 3U> reserved{};

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_NOINLINE
        bool IsProvenRunCoverageBoundaryAvailabilityPair() const noexcept
    {
        using namespace NCPathCoreRunCoverageBoundaryAvailabilityPairDetail;
        return publicationSequence != 0ULL && previousTransitionPublicationSequence != 0ULL &&
            currentTransition.publicationSequence == NextNonZeroSequence(previousTransitionPublicationSequence) &&
            schemaVersion == NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_SCHEMA_V1 &&
            disposition == Disposition::PROVEN_AVAILABILITY_TRANSITION_PAIR &&
            reserved[0U] == 0U && reserved[1U] == 0U && reserved[2U] == 0U &&
            currentTransition.IsProvenRunCoverageBoundaryAvailabilityTransition() &&
            EndingAvailability(previousRelation) == currentTransition.previousAvailability &&
            relation != Relation::NONE && relation == ClassifyPair(previousRelation, currentTransition.relation);
    }
};

class NCPathCoreRunCoverageBoundaryAvailabilityPairShadow final
{
public:
    using Record = NCPathCoreRunCoverageBoundaryAvailabilityPairRecordV1;
    using Disposition = Record::Disposition;
    using Relation = Record::Relation;
    using TransitionRecord = Record::TransitionRecord;
    using TransitionDisposition = TransitionRecord::Disposition;
    using TransitionRelation = TransitionRecord::Relation;
    using Availability = TransitionRecord::Availability;
    using UnavailableReason = TransitionRecord::UnavailableReason;
    using BoundaryRecord = Record::BoundaryRecord;
    using BoundaryDisposition = BoundaryRecord::Disposition;
    using RunRecord = Record::RunRecord;

    NCPathCoreRunCoverageBoundaryAvailabilityPairShadow() noexcept = default;

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_NOINLINE
        void ObserveLatestRunCoverageBoundaryAvailabilityPairSameThread(const TransitionRecord* previousTransition,
            const TransitionRecord* currentTransition) noexcept
    {
        const Record* const previous = GetNewestObservationSameThread();
        const bool previousProven = previous != nullptr && previous->IsProvenRunCoverageBoundaryAvailabilityPair();
        // Validate newest before clearing the independent older diagnostic slot.
        const bool previousValid = previous == nullptr || (previous->publicationSequence == m_publicationSequence &&
            (previousProven || IsCanonicalNonProvenPair(*previous)));
        const std::uint64_t anchor = previous == nullptr ? 0ULL : previous->currentTransition.publicationSequence;
        m_publicationSequence = NCPathCoreRunCoverageBoundaryAvailabilityPairDetail::NextNonZeroSequence(m_publicationSequence);
        const std::size_t index = m_latestIndex == INVALID_INDEX ? 0U :
            (static_cast<std::size_t>(m_latestIndex) + 1U) % HISTORY_CAPACITY;
        Record& target = m_records[index];
        ResetTarget(target, m_publicationSequence, currentTransition == nullptr ? 0ULL : currentTransition->publicationSequence);
        if (!previousValid) target.disposition = Disposition::INVALID_PREVIOUS_PAIR_RECORD;
        else if (currentTransition == nullptr) target.disposition = Disposition::NOT_APPLICABLE_CURRENT_TRANSITION_UNAVAILABLE;
        else if (anchor != 0ULL && currentTransition->publicationSequence !=
            NCPathCoreRunCoverageBoundaryAvailabilityPairDetail::NextNonZeroSequence(anchor))
            target.disposition = Disposition::INVALID_OBSERVER_SOURCE_ADVANCE;
        else EvaluatePair(previousTransition, *currentTransition, previous, previousProven, anchor, target);
        m_latestIndex = static_cast<std::uint8_t>(index);
        if (m_recordCount < HISTORY_CAPACITY) ++m_recordCount;
    }

    const Record* GetNewestObservationSameThread(std::size_t historyOffset = 0U) const noexcept
    {
        if (m_latestIndex == INVALID_INDEX || historyOffset >= m_recordCount) return nullptr;
        const std::size_t index = (static_cast<std::size_t>(m_latestIndex) + HISTORY_CAPACITY - historyOffset) % HISTORY_CAPACITY;
        return &m_records[index];
    }

    std::size_t GetRecordCountSameThread() const noexcept { return m_recordCount; }

private:
    static constexpr std::uint8_t INVALID_INDEX = 0xFFU;
    static constexpr std::size_t HISTORY_CAPACITY = NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_HISTORY_CAPACITY;

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_NOINLINE
        static bool IsCanonicalNonProvenPair(const Record& record) noexcept
    {
        using namespace NCPathCoreRunCoverageBoundaryAvailabilityPairDetail;
        const bool known = record.disposition >= Disposition::NOT_APPLICABLE_CURRENT_TRANSITION_UNAVAILABLE &&
            record.disposition <= Disposition::INVALID_AVAILABILITY_TRANSITION;
        // A missing pointer has zero identity; canonical unproven AB carries
        // a nonzero diagnostic identity under the same unavailable disposition.
        const bool identity = record.currentTransition.publicationSequence != 0ULL ||
            record.disposition == Disposition::NOT_APPLICABLE_CURRENT_TRANSITION_UNAVAILABLE ||
            record.disposition == Disposition::INVALID_CURRENT_TRANSITION_RECORD ||
            record.disposition == Disposition::INVALID_OBSERVER_SOURCE_ADVANCE ||
            record.disposition == Disposition::INVALID_PREVIOUS_PAIR_RECORD;
        const TransitionRecord& transition = record.currentTransition;
        return known && identity && record.publicationSequence != 0ULL &&
            record.schemaVersion == NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_SCHEMA_V1 &&
            record.previousTransitionPublicationSequence == 0ULL && record.previousRelation == TransitionRelation::NONE &&
            record.relation == Relation::NONE && record.reserved[0U] == 0U &&
            record.reserved[1U] == 0U && record.reserved[2U] == 0U &&
            transition.previousBoundaryPublicationSequence == 0ULL && transition.currentBoundary.publicationSequence == 0ULL &&
            transition.schemaVersion == 0U && transition.disposition == TransitionDisposition::EMPTY &&
            transition.relation == TransitionRelation::NONE && transition.previousAvailability == Availability::UNKNOWN &&
            transition.currentAvailability == Availability::UNKNOWN && transition.currentUnavailableReason == UnavailableReason::NONE &&
            transition.reserved == 0U && IsZeroBoundaryPayload(transition.currentBoundary);
    }

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_NOINLINE
        static bool PreviousSourceMatches(const TransitionRecord& source, const TransitionRecord& retained) noexcept
    {
        return source.publicationSequence == retained.publicationSequence &&
            source.previousBoundaryPublicationSequence == retained.previousBoundaryPublicationSequence &&
            source.schemaVersion == retained.schemaVersion && source.disposition == retained.disposition &&
            source.relation == retained.relation && source.previousAvailability == retained.previousAvailability &&
            source.currentAvailability == retained.currentAvailability &&
            source.currentUnavailableReason == retained.currentUnavailableReason && source.reserved == retained.reserved &&
            PreviousBoundaryMatches(source.currentBoundary, retained.currentBoundary);
    }

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_NOINLINE
        static bool PreviousBoundaryMatches(const BoundaryRecord& source, const BoundaryRecord& retained) noexcept
    {
        // Semantic equality is exhaustive; padding and object representations are not read.
        return source.publicationSequence == retained.publicationSequence &&
            source.schemaVersion == retained.schemaVersion && source.disposition == retained.disposition &&
            source.firstYCoverageMask == retained.firstYCoverageMask &&
            source.gainedSinceFirstYMask == retained.gainedSinceFirstYMask &&
            source.fullCoverageRelation == retained.fullCoverageRelation &&
            source.reserved[0U] == retained.reserved[0U] && source.reserved[1U] == retained.reserved[1U] &&
            source.currentRun.publicationSequence == retained.currentRun.publicationSequence &&
            source.currentRun.runGeneration == retained.currentRun.runGeneration &&
            source.currentRun.firstCertificatePublicationSequence == retained.currentRun.firstCertificatePublicationSequence &&
            source.currentRun.currentCertificatePublicationSequence == retained.currentRun.currentCertificatePublicationSequence &&
            source.currentRun.currentCoverageTransitionPublicationSequence == retained.currentRun.currentCoverageTransitionPublicationSequence &&
            source.currentRun.currentCoveragePublicationSequence == retained.currentRun.currentCoveragePublicationSequence &&
            source.currentRun.coverageGeneration == retained.currentRun.coverageGeneration &&
            source.currentRun.sourceRunPublicationSequence == retained.currentRun.sourceRunPublicationSequence &&
            source.currentRun.continuityRunGeneration == retained.currentRun.continuityRunGeneration &&
            source.currentRun.firstPairPublicationSequence == retained.currentRun.firstPairPublicationSequence &&
            source.currentRun.sourcePairPublicationSequence == retained.currentRun.sourcePairPublicationSequence &&
            source.currentRun.firstTransitionPublicationSequence == retained.currentRun.firstTransitionPublicationSequence &&
            source.currentRun.currentTransitionPublicationSequence == retained.currentRun.currentTransitionPublicationSequence &&
            source.currentRun.firstBoundaryPublicationSequence == retained.currentRun.firstBoundaryPublicationSequence &&
            source.currentRun.currentBoundaryPublicationSequence == retained.currentRun.currentBoundaryPublicationSequence &&
            source.currentRun.consecutivePairCount == retained.currentRun.consecutivePairCount &&
            source.currentRun.consecutiveCertificateCount == retained.currentRun.consecutiveCertificateCount &&
            source.currentRun.firstSourcePairCount == retained.currentRun.firstSourcePairCount &&
            source.currentRun.schemaVersion == retained.currentRun.schemaVersion &&
            source.currentRun.disposition == retained.currentRun.disposition &&
            source.currentRun.previousObservedPairRelationMask == retained.currentRun.previousObservedPairRelationMask &&
            source.currentRun.currentObservedPairRelationMask == retained.currentRun.currentObservedPairRelationMask &&
            source.currentRun.newlyObservedPairRelationMask == retained.currentRun.newlyObservedPairRelationMask &&
            source.currentRun.retainedPairRelationMask == retained.currentRun.retainedPairRelationMask &&
            source.currentRun.fullCoverageTransition == retained.currentRun.fullCoverageTransition &&
            source.currentRun.firstPairRelation == retained.currentRun.firstPairRelation &&
            source.currentRun.sourcePairRelation == retained.currentRun.sourcePairRelation &&
            source.currentRun.previousSourcePairRelation == retained.currentRun.previousSourcePairRelation &&
            source.currentRun.previousTransitionRelation == retained.currentRun.previousTransitionRelation &&
            source.currentRun.currentTransitionRelation == retained.currentRun.currentTransitionRelation &&
            source.currentRun.currentUnavailableReason == retained.currentRun.currentUnavailableReason &&
            source.currentRun.earlierObservedPairRelationMask == retained.currentRun.earlierObservedPairRelationMask &&
            source.currentRun.earlierSourcePairRelation == retained.currentRun.earlierSourcePairRelation &&
            source.currentRun.discoveryPairRelation == retained.currentRun.discoveryPairRelation &&
            source.currentRun.fullCoveragePairRelation == retained.currentRun.fullCoveragePairRelation &&
            source.currentRun.reserved[0U] == retained.currentRun.reserved[0U] &&
            source.currentRun.reserved[1U] == retained.currentRun.reserved[1U];
    }

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_NOINLINE
        static bool SameSourceRun(const RunRecord& previous, const RunRecord& current) noexcept
    {
        return current.runGeneration == previous.runGeneration &&
            current.firstCertificatePublicationSequence == previous.firstCertificatePublicationSequence &&
            current.firstSourcePairCount == previous.firstSourcePairCount &&
            current.coverageGeneration == previous.coverageGeneration &&
            current.continuityRunGeneration == previous.continuityRunGeneration &&
            current.firstPairPublicationSequence == previous.firstPairPublicationSequence &&
            current.firstTransitionPublicationSequence == previous.firstTransitionPublicationSequence &&
            current.firstBoundaryPublicationSequence == previous.firstBoundaryPublicationSequence &&
            current.firstPairRelation == previous.firstPairRelation;
    }

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_NOINLINE
        static bool DirectRunAdvance(const RunRecord& previous, const RunRecord& current) noexcept
    {
        using NCPathCoreRunCoverageBoundaryAvailabilityDetail::NextNonZeroSequence;
        return previous.consecutivePairCount != (std::numeric_limits<std::uint32_t>::max)() &&
            previous.consecutiveCertificateCount < (std::numeric_limits<std::uint32_t>::max)() - 3U &&
            current.consecutivePairCount == previous.consecutivePairCount + 1U &&
            current.consecutiveCertificateCount == previous.consecutiveCertificateCount + 1U &&
            current.publicationSequence == NextNonZeroSequence(previous.publicationSequence) &&
            current.currentCertificatePublicationSequence == NextNonZeroSequence(previous.currentCertificatePublicationSequence) &&
            current.currentCoverageTransitionPublicationSequence == NextNonZeroSequence(previous.currentCoverageTransitionPublicationSequence) &&
            current.currentCoveragePublicationSequence == NextNonZeroSequence(previous.currentCoveragePublicationSequence) &&
            current.sourceRunPublicationSequence == NextNonZeroSequence(previous.sourceRunPublicationSequence) &&
            current.sourcePairPublicationSequence == NextNonZeroSequence(previous.sourcePairPublicationSequence) &&
            current.currentTransitionPublicationSequence == NextNonZeroSequence(previous.currentTransitionPublicationSequence) &&
            current.currentBoundaryPublicationSequence == NextNonZeroSequence(previous.currentBoundaryPublicationSequence);
    }

    static bool SharedRunBinding(const RunRecord& previous, const RunRecord& current) noexcept
    {
        return current.earlierObservedPairRelationMask == previous.previousObservedPairRelationMask &&
            current.previousObservedPairRelationMask == previous.currentObservedPairRelationMask &&
            current.earlierSourcePairRelation == previous.previousSourcePairRelation &&
            current.previousSourcePairRelation == previous.sourcePairRelation &&
            current.previousTransitionRelation == previous.currentTransitionRelation;
    }

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_NOINLINE
        static void EvaluatePair(const TransitionRecord* previousTransition, const TransitionRecord& currentTransition,
            const Record* previous, bool previousProven, std::uint64_t anchor, Record& target) noexcept
    {
        using namespace NCPathCoreRunCoverageBoundaryAvailabilityPairDetail;
        if (!currentTransition.IsProvenRunCoverageBoundaryAvailabilityTransition())
        {
            target.disposition = !IsCanonicalNonProvenTransition(currentTransition) ?
                Disposition::INVALID_CURRENT_TRANSITION_RECORD :
                (IsNeutralTransitionDisposition(currentTransition.disposition) ?
                    Disposition::NOT_APPLICABLE_CURRENT_TRANSITION_UNAVAILABLE :
                    Disposition::SOURCE_REPORTED_CURRENT_TRANSITION_INVALID);
            return;
        }
        if (previousTransition == nullptr)
        {
            target.disposition = Disposition::NOT_APPLICABLE_PREVIOUS_TRANSITION_UNAVAILABLE;
            return;
        }
        if (!previousTransition->IsProvenRunCoverageBoundaryAvailabilityTransition())
        {
            target.disposition = !IsCanonicalNonProvenTransition(*previousTransition) ?
                Disposition::INVALID_PREVIOUS_TRANSITION_RECORD :
                (IsNeutralTransitionDisposition(previousTransition->disposition) ?
                    Disposition::NOT_APPLICABLE_PREVIOUS_TRANSITION_UNAVAILABLE :
                    Disposition::SOURCE_REPORTED_PREVIOUS_TRANSITION_INVALID);
            return;
        }
        if (currentTransition.publicationSequence != NextNonZeroSequence(previousTransition->publicationSequence))
        {
            target.disposition = Disposition::INVALID_TRANSITION_ADVANCE;
            return;
        }
        if ((anchor != 0ULL && previousTransition->publicationSequence != anchor) ||
            (previousProven && previous != nullptr && !PreviousSourceMatches(*previousTransition, previous->currentTransition)))
        {
            target.disposition = Disposition::INVALID_PREVIOUS_SOURCE_BINDING;
            return;
        }
        if (currentTransition.previousBoundaryPublicationSequence != previousTransition->currentBoundary.publicationSequence)
        {
            target.disposition = Disposition::INVALID_SHARED_BOUNDARY_BINDING;
            return;
        }
        if (currentTransition.previousAvailability != previousTransition->currentAvailability)
        {
            target.disposition = Disposition::INVALID_AVAILABILITY_STATE_BINDING;
            return;
        }
        if (!CheckBoundaryEdge(*previousTransition, currentTransition, target)) return;
        BuildPair(*previousTransition, currentTransition, target);
    }

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_NOINLINE
        static bool CheckBoundaryEdge(const TransitionRecord& previousTransition,
            const TransitionRecord& currentTransition, Record& target) noexcept
    {
        using NCPathCoreRunCoverageBoundaryAvailabilityPairDetail::NextNonZeroSequence;
        const BoundaryRecord& previousBoundary = previousTransition.currentBoundary;
        const BoundaryRecord& currentBoundary = currentTransition.currentBoundary;
        if (currentBoundary.publicationSequence != NextNonZeroSequence(previousBoundary.publicationSequence))
        {
            target.disposition = Disposition::INVALID_BOUNDARY_ADVANCE;
            return false;
        }
        const RunRecord& previousRun = previousBoundary.currentRun;
        const RunRecord& currentRun = currentBoundary.currentRun;
        if (previousRun.publicationSequence != 0ULL && currentRun.publicationSequence != 0ULL &&
            currentRun.publicationSequence != NextNonZeroSequence(previousRun.publicationSequence))
        {
            target.disposition = Disposition::INVALID_SOURCE_RUN_ADVANCE;
            return false;
        }
        // Both AB self-proofs have already validated these complete AA states,
        // including the exact current unavailable reason. Only the supplied
        // overlap is available; current AB's discarded previous AA is unknown.
        const bool previousAvailable = previousTransition.currentAvailability == Availability::AVAILABLE;
        const bool currentAvailable = currentTransition.currentAvailability == Availability::AVAILABLE;
        if (currentAvailable)
        {
            if ((previousAvailable && currentBoundary.disposition != BoundaryDisposition::PROVEN_OBSERVED_HEAD_BOUNDARY_EXTENSION) ||
                (!previousAvailable && currentBoundary.disposition != BoundaryDisposition::PROVEN_OBSERVED_HEAD_BOUNDARY_START))
            {
                target.disposition = Disposition::INVALID_AVAILABILITY_STATE_BINDING;
                return false;
            }
            if (previousAvailable)
            {
                if (currentBoundary.firstYCoverageMask != previousBoundary.firstYCoverageMask ||
                    !SameSourceRun(previousRun, currentRun))
                {
                    target.disposition = Disposition::INVALID_SOURCE_RUN_BINDING;
                    return false;
                }
                if (previousRun.consecutivePairCount == (std::numeric_limits<std::uint32_t>::max)() ||
                    previousRun.consecutiveCertificateCount >= (std::numeric_limits<std::uint32_t>::max)() - 3U)
                {
                    target.disposition = Disposition::INVALID_RUN_COUNT_OVERFLOW;
                    return false;
                }
                if (!DirectRunAdvance(previousRun, currentRun))
                {
                    target.disposition = Disposition::INVALID_SOURCE_RUN_ADVANCE;
                    return false;
                }
                if (!SharedRunBinding(previousRun, currentRun))
                {
                    target.disposition = Disposition::INVALID_SHARED_RUN_BINDING;
                    return false;
                }
            }
        }
        else if (previousAvailable && currentBoundary.disposition == BoundaryDisposition::NOT_APPLICABLE_HEAD_NOT_OBSERVED)
        {
            target.disposition = Disposition::INVALID_AVAILABILITY_STATE_BINDING;
            return false;
        }
        return true;
    }

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_NOINLINE
        static void BuildPair(const TransitionRecord& previousTransition, const TransitionRecord& currentTransition,
            Record& target) noexcept
    {
        target.previousTransitionPublicationSequence = previousTransition.publicationSequence;
        target.currentTransition = currentTransition; // Direct lvalue-to-slot copy; no AB/AA/Z temporary.
        target.previousRelation = previousTransition.relation;
        target.relation = NCPathCoreRunCoverageBoundaryAvailabilityPairDetail::ClassifyPair(
            previousTransition.relation, currentTransition.relation);
        target.disposition = Disposition::PROVEN_AVAILABILITY_TRANSITION_PAIR;
        if (!target.IsProvenRunCoverageBoundaryAvailabilityPair())
        {
            ClearPayload(target);
            target.disposition = Disposition::INVALID_AVAILABILITY_TRANSITION;
        }
    }

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_NOINLINE
        static void ClearPayload(Record& target) noexcept
    {
        // Preserve only currentTransition.publicationSequence as diagnostic identity.
        RunRecord& source = target.currentTransition.currentBoundary.currentRun;
        source.publicationSequence = 0ULL;
        source.runGeneration = 0ULL;
        source.firstCertificatePublicationSequence = 0ULL;
        source.currentCertificatePublicationSequence = 0ULL;
        source.currentCoverageTransitionPublicationSequence = 0ULL;
        source.currentCoveragePublicationSequence = 0ULL;
        source.coverageGeneration = 0ULL;
        source.sourceRunPublicationSequence = 0ULL;
        source.continuityRunGeneration = 0ULL;
        source.firstPairPublicationSequence = 0ULL;
        source.sourcePairPublicationSequence = 0ULL;
        source.firstTransitionPublicationSequence = 0ULL;
        source.currentTransitionPublicationSequence = 0ULL;
        source.firstBoundaryPublicationSequence = 0ULL;
        source.currentBoundaryPublicationSequence = 0ULL;
        source.consecutivePairCount = 0U;
        source.consecutiveCertificateCount = 0U;
        source.firstSourcePairCount = 0U;
        source.schemaVersion = 0U;
        source.disposition = RunRecord::Disposition::EMPTY;
        source.previousObservedPairRelationMask = 0U;
        source.currentObservedPairRelationMask = 0U;
        source.newlyObservedPairRelationMask = 0U;
        source.retainedPairRelationMask = 0U;
        source.fullCoverageTransition = RunRecord::FullCoverageTransition::NONE;
        source.firstPairRelation = RunRecord::PairRelation::NONE;
        source.sourcePairRelation = RunRecord::PairRelation::NONE;
        source.previousSourcePairRelation = RunRecord::PairRelation::NONE;
        source.previousTransitionRelation = RunRecord::TransitionRelation::NONE;
        source.currentTransitionRelation = RunRecord::TransitionRelation::NONE;
        source.currentUnavailableReason = RunRecord::UnavailableReason::NONE;
        source.earlierObservedPairRelationMask = 0U;
        source.earlierSourcePairRelation = RunRecord::PairRelation::NONE;
        source.discoveryPairRelation = RunRecord::DiscoveryPairRelation::NONE;
        source.fullCoveragePairRelation = RunRecord::FullCoveragePairRelation::NONE;
        for (std::uint8_t& value : source.reserved) value = 0U;
        target.currentTransition.currentBoundary.schemaVersion = 0U;
        target.currentTransition.currentBoundary.disposition = BoundaryDisposition::EMPTY;
        target.currentTransition.currentBoundary.firstYCoverageMask = 0U;
        target.currentTransition.currentBoundary.gainedSinceFirstYMask = 0U;
        target.currentTransition.currentBoundary.fullCoverageRelation = BoundaryRecord::FullCoverageRelation::NONE;
        for (std::uint8_t& value : target.currentTransition.currentBoundary.reserved) value = 0U;
        target.currentTransition.currentBoundary.publicationSequence = 0ULL;
        target.currentTransition.previousBoundaryPublicationSequence = 0ULL;
        target.currentTransition.schemaVersion = 0U;
        target.currentTransition.disposition = TransitionDisposition::EMPTY;
        target.currentTransition.relation = TransitionRelation::NONE;
        target.currentTransition.previousAvailability = Availability::UNKNOWN;
        target.currentTransition.currentAvailability = Availability::UNKNOWN;
        target.currentTransition.currentUnavailableReason = UnavailableReason::NONE;
        target.currentTransition.reserved = 0U;
        target.previousTransitionPublicationSequence = 0ULL;
        target.previousRelation = TransitionRelation::NONE;
        target.relation = Relation::NONE;
        for (std::uint8_t& value : target.reserved) value = 0U;
    }

    NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_NOINLINE
        static void ResetTarget(Record& target, std::uint64_t publication, std::uint64_t source) noexcept
    {
        target.publicationSequence = publication;
        target.currentTransition.publicationSequence = source;
        ClearPayload(target);
        target.schemaVersion = NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_SCHEMA_V1;
        target.disposition = Disposition::NOT_APPLICABLE_CURRENT_TRANSITION_UNAVAILABLE;
    }

    std::array<Record, HISTORY_CAPACITY> m_records{};
    std::uint64_t m_publicationSequence = 0ULL;
    std::uint8_t m_latestIndex = INVALID_INDEX;
    std::uint8_t m_recordCount = 0U;
    std::array<std::uint8_t, 6U> m_reserved{};
};

#undef NC_PATH_CORE_RUN_COVERAGE_BOUNDARY_AVAILABILITY_PAIR_NOINLINE

static_assert(sizeof(NCPathCoreRunCoverageBoundaryAvailabilityPairDisposition) == 1U, "AC disposition one byte.");
static_assert(sizeof(NCPathCoreRunCoverageBoundaryAvailabilityPairRelation) == 1U, "AC relation one byte.");
static_assert(std::is_standard_layout<NCPathCoreRunCoverageBoundaryAvailabilityPairRecordV1>::value, "AC record standard layout.");
static_assert(std::is_trivially_copyable<NCPathCoreRunCoverageBoundaryAvailabilityPairRecordV1>::value, "AC record scalar copyability.");
static_assert(sizeof(NCPathCoreRunCoverageBoundaryAvailabilityPairRecordV1) == 216U, "AC record exactly 216 bytes.");
static_assert(alignof(NCPathCoreRunCoverageBoundaryAvailabilityPairRecordV1) == 8U, "AC record alignment.");
static_assert(offsetof(NCPathCoreRunCoverageBoundaryAvailabilityPairRecordV1, previousTransitionPublicationSequence) == 8U, "AC previous AB identity offset.");
static_assert(offsetof(NCPathCoreRunCoverageBoundaryAvailabilityPairRecordV1, currentTransition) == 16U, "AC retained AB offset.");
static_assert(offsetof(NCPathCoreRunCoverageBoundaryAvailabilityPairRecordV1, schemaVersion) == 208U, "AC schema offset.");
static_assert(offsetof(NCPathCoreRunCoverageBoundaryAvailabilityPairRecordV1, disposition) == 210U, "AC disposition offset.");
static_assert(offsetof(NCPathCoreRunCoverageBoundaryAvailabilityPairRecordV1, previousRelation) == 211U, "AC previous relation offset.");
static_assert(offsetof(NCPathCoreRunCoverageBoundaryAvailabilityPairRecordV1, relation) == 212U, "AC relation offset.");
static_assert(offsetof(NCPathCoreRunCoverageBoundaryAvailabilityPairRecordV1, reserved) == 213U, "AC reserved offset.");
static_assert(std::is_standard_layout<NCPathCoreRunCoverageBoundaryAvailabilityPairShadow>::value, "AC observer standard layout.");
static_assert(std::is_trivially_copyable<NCPathCoreRunCoverageBoundaryAvailabilityPairShadow>::value, "AC observer scalar copyability.");
static_assert(sizeof(NCPathCoreRunCoverageBoundaryAvailabilityPairShadow) == 448U, "AC observer exactly 448 bytes.");
static_assert(alignof(NCPathCoreRunCoverageBoundaryAvailabilityPairShadow) == 8U, "AC observer alignment.");
